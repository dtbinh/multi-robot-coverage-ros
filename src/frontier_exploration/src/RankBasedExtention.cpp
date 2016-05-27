/*
    ** RankBasedExtention.cpp

    * This program creates a threshold based allocation algorithm for frontier allocation. 
    * The frontiers are allocated in such a way that all frontiers in a threshold are allocated fits
*/

#include<RankBasedExtention.h>
#include<fstream>
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>
using namespace std;

typedef std::multimap<double, unsigned int> FrontQueue;
typedef std::pair<double, unsigned int> Value;
	
RankBasedExtention::RankBasedExtention()
{
	ros::NodeHandle navigatorNode("~/");
	nxtPlan = navigatorNode.advertise<std_msgs::Int64>("nextplan", 1);
 	nxtFrontier = navigatorNode.advertise<std_msgs::Int64>("nextGoal", 1);
	currentGoal = navigatorNode.advertise<std_msgs::Int64>("currentGoal", 1);
 	repeat = navigatorNode.advertise<std_msgs::Bool>("boolean", 1);
        overlap =  navigatorNode.advertise<std_msgs::Int64>("overlap", 1);

 	//ros:: Subscriber mSubscriber = navigatorNode.subscribe("nextplan", 1, planCallback);
	ros::NodeHandle robot;
	robot.param("robot_id", robotID, 1);
	//robot.param("nextGoal", nxtGoal, 0);
	robot.param("goalRobot", goalRobot, 0);
	
	ind_flag = 0;
	front_count=0;
	robot_1_counter=0;
	robot_2_counter=0;
	robot_3_counter=0;
}

RankBasedExtention::~RankBasedExtention()
{

}

int RankBasedExtention::findExplorationTarget(GridMap* map, unsigned int start, unsigned int &goal)
{

	ofstream myfile("/home/raja/threshold.txt", ios_base::app);
	if(nxtmsg.data != 0 && test.data)
	{
		if(nxtmsg.data!=nextgoalmsg.data)
		{
			nextgoalmsg.data=nxtmsg.data;
			nxtFrontier.publish(nextgoalmsg);
	         	goal=nxtmsg.data;
			count.data = count.data + 20* (map->getResolution());
			overlap.publish(count);
			return EXPL_TARGET_SET; 
		}
	}	
	test.data=false;
	unsigned int size=map->getSize();
	plan = new double[size];
	for(unsigned int x=0; x<size; x++)
	{
		plan[x] = -1;
	}

	double thresholdDistance=10*(map->getResolution());
	double resolution = map->getResolution();
	unsigned int count=0, ind;
	double dist;
	frontierList.clear();
	frontierCells=0;
	FrontQueue list;
	FrontQueue::iterator next;
	Value startvalue(0.0,start);
	list.insert(startvalue);
	plan[start]=0;
	unsigned int offset[4];
	offset[0]=-1;
	offset[1]=+1;
	offset[2]=-map->getWidth();
	offset[3]=+map->getWidth();

	while(!list.empty())
	{
		count++;
		next=list.begin();
		dist = next->first;
		ind = next->second;
		list.erase(next);
			
		for(int i=0; i<4 ; i++)
		{
			unsigned int y=ind+offset[i];
			if(map->isFree(y) && plan[y] == -1)
			{
				if(map->isFrontier(y))
				{
					findCluster(map,y);						
				}
				else
				{
					list.insert(Value(dist+resolution, y));
					plan[y]=dist+resolution;
				}
			}
		}
	}

	ind_flag = ind;

	if(frontierList.size()==0)
	{
		if (count>30)
		{
			return EXPL_FINISHED;
		}
		else
		{
			return EXPL_FAILED;
		}
	}

	unsigned int bRank=999;
	unsigned int bFrontier=0;
	unsigned int goalFrontier=0;
	double bDistance=999;
	double currentDist=999;
	FrontQueue queue;
	unsigned int rank=0;
	double robotDistance=-1;
	std::set<unsigned int> otherRobots;
	PoseList p = robotList.getRobots();

	for(PoseList::iterator it = p.begin(); it!= p.end(); it++)
	{
		if((int) it->first == robotID) continue;
		unsigned int robot_x = (double)(it->second.x - map->getOriginX())/map->getResolution();
		unsigned int robot_y = (double)(it->second.y - map->getOriginY())/map->getResolution();
		unsigned int robot = 0;
		if(map->getIndex(robot_x, robot_y, robot))
		{
			otherRobots.insert(robot);
			ROS_DEBUG("Inserted robot at index %d", robot);
		}
		else
		{
			ROS_WARN("Robot %d cannot be placed in the map", it->first);
		}
	}

	for(unsigned int f=0; f<frontierList.size(); f++)
	{
		double* plan = new double[size];
		for(unsigned int x=0; x<size; x++)
		{
			plan[x] = -1;
		}	
		for(unsigned int cell=0; cell<frontierList[f].size(); cell++)
		{
			plan[frontierList[f][cell]]=0;
			queue.insert(Value(0.0,frontierList[f][cell]));		
		}
		while(!queue.empty())
		{
			FrontQueue::iterator next;
			next=queue.begin();
			dist = next->first;
			ind = next->second;
			queue.erase(next);
			if(ind==start)
			{
				robotDistance=dist;
				break;
			}
			if(otherRobots.find(ind)!=otherRobots.end()) rank++;

			for( unsigned int i=0; i<4; i++)
			{
				unsigned int y=ind+offset[i];	
				if( y>=0 && y<size && map->getData(y)==0 && plan[y]==-1)
				{
					plan[y]=dist+resolution;
					queue.insert(Value(dist+resolution, y));
				}
			}

		}
		ROS_DEBUG("Frontier %d is at distance %d and its rank is %.2f", f, rank, robotDistance);
		if(robotDistance>0 && (rank < bRank || rank == bRank && robotDistance < bDistance))
		{
			bRank=rank;
			bFrontier=f;
			bDistance=robotDistance;
		}
		else if(robotDistance>0 && ((rank > bRank && (robotDistance-bDistance)<= thresholdDistance && robotDistance < currentDist) || (rank == bRank && (robotDistance-bDistance) <= thresholdDistance&& robotDistance < currentDist)))
		{
			nxtGoal=frontierList.at(goalFrontier).at(0);
			goalFrontier=f;
			currentDist=robotDistance;
		}
	delete[] plan;	
	}

	if(bFrontier>=frontierList.size())
	{
		ROS_DEBUG("Best frontier could not be determined");
		return EXPL_FAILED;
	}
	goal=frontierList.at(bFrontier).at(0);
	myfile<<"Chosen Frontier : "<<goal<<"\n";
	
	if(robotID == 1)
	{
		robot_1_counter++;
	}
	else if(robotID == 2)
	{
		robot_2_counter++;
	}
	else if(robotID == 3)
	{
		robot_3_counter++;
	}

	front_count++;
	goalmsg.data=goal;
	currentGoal.publish(goalmsg);
	if(goalFrontier!=0)
	{
		
		nxtGoal=frontierList.at(goalFrontier).at(0);
		nxtmsg.data=nxtGoal;
		nxtPlan.publish(nxtmsg);
		goalRobot=robotID;
		test.data=true;
	}
	repeat.publish(test);

	myfile<<"Robot 1 Counter: "<<robot_1_counter<<"\n";
	myfile<<"Robot 2 Counter: "<<robot_2_counter<<"\n";
	myfile<<"Robot 3 Counter: "<<robot_3_counter<<"\n";

	return EXPL_TARGET_SET; 	
	myfile.close();
}

void RankBasedExtention::findCluster(GridMap* map, unsigned int startCell)
{
	FrontQueue queue;
	queue.insert(Value(0.0,startCell));
	Frontiers front;

	while(!queue.empty())
	{

		FrontQueue::iterator next;
		next=queue.begin();
		double dist = next->first;
		int ind = next->second;
		queue.erase(next);
		if(!map->isFrontier(ind)) continue;
		//ROS_INFO("Frontier is %d", ind);
		front.push_back(ind);
		frontierCells++;
		unsigned int offset[4];
		offset[0]=-1;
		offset[1]=+1;
		offset[2]=-map->getWidth();
		offset[3]=+map->getWidth();

		for(int i=0; i<4 ; i++)
		{
			unsigned int y=ind+offset[i];
			if(map->isFree(y) && plan[y] == -1)
			{
				plan[y]=dist+map->getResolution();
				queue.insert(Value(dist+ map->getResolution(),y));				
			}
		}
	}
	frontierList.push_back(front);
}

