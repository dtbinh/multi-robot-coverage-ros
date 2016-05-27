/*
    ** CaseBasedAllocation.cpp
    ** Sushil Parti

    * This program implements the Remember-All frontier allocation strategy for a three robot strategy
    * The frontiers are allocated based on a variant of MinPos algorithm - Rank Based Strategy.
    * The robots store the unallocated frontiers in individual lists with each robot.
    * In case the robots don't find any frontier in the current stage, then they choose frontiers from the unallocated lists
    * In case there are no more frontiers, then frontiers are chosen from the nearest robot based on MinPos.
    * The coordination strategy is that robots work independently until they required help of other robots.

*/

#include<fstream>
#include<CaseBasedAllocation.h>
using namespace std;

typedef std::multimap<double, unsigned int> FQueue;
typedef std::pair<double, unsigned int> FValue;

//Initializes the node and robot parameters
CaseBasedAllocation::CaseBasedAllocation()
{
	ros::NodeHandle robot;
	robot.param("robot_id", robotID, 1);
	
	//number of robots being used
	robotCount = 2;

	//number of robots that can be transferred from the unallocated_frontiers list to the identified_frontiers list
	unallocated_counter = 2;

	robot_flag = 100;
	robot_1_counter = 0;
	robot_2_counter = 0;
	dup_counter = 0;
	robot1_flag = 0;
	robot2_flag = 0;
}

CaseBasedAllocation::~CaseBasedAllocation()
{

}

int CaseBasedAllocation::findExplorationTarget(GridMap* map, unsigned int start, unsigned int &goal)
{	
	//write output to a file for metric purposes
	ofstream myfile("/home/raja/remember.txt", ios_base::app);
	ofstream testfile("/home/raja/test.txt", ios_base::app);

	//create a sample workspace for the wavefront algorithm	
	double distance=0.0;
	unsigned int count=0, index=0;
	unsigned int map_size = map->getSize();
	double map_resolution = map->getResolution();

	grid = new double[map_size];
	for(unsigned int x=0; x<map_size; x++)
	{
		grid[x] = -1;
	}
		
	//clear the robot's 'identified_frontiers' list
	frontierList.clear();
	frontierCells=0;
	
	/*the queue maintains a frontier and its Euclidean distance from the robot
	 * initialize the queue with the robot's start position 
	 */
	FQueue list;
	FQueue::iterator next;
	FValue startvalue(0.0,start);
	list.insert(startvalue);
	grid[start]=0;
	
	// 4- way expansion
	unsigned int offset[4];
	offset[0]=-1;			//left
	offset[1]=+1;			//right
	offset[2]=-map->getWidth();	//up
	offset[3]=+map->getWidth();	//down

	//create a wave and propogate from the robot's current position to the edge of explored space
	while(!list.empty())
	{
		count++;
		next=list.begin();
		distance = next->first;
		index = next->second;
		list.erase(next);
		
		for(int i=0; i<4 ; i++)
		{
			unsigned int cell = index + offset[i];
			//if the cell is free and has not been checked yet
			if(map->isFree(cell) && grid[cell] == -1)
			{
				if(map->isFrontier(cell))
				{
					findCluster(map,cell);
				}
				else
				{
					list.insert(FValue(distance+map_resolution, cell));
					grid[cell] = distance + map_resolution;
				}
			}
		}
	}

	//transfer fixed amount of frontiers from unallocated list to identified list
	if(robotID == 1) 
	{
		int size = robot_1_list.size()-1;
		if(size > 2) 
		{	
			for(int i = 0; i<unallocated_counter; i++) 
			{
				unsigned int frontier = robot_1_list[size-i];
				unsigned int frontier_size = frontierList.size()-1;
				frontierList[frontier_size].push_back(frontier);
			}
		}
	}	
	else if(robotID == 2) 
	{
		int size = robot_2_list.size()-1;
		if(size > 2)
		{
			for(int i = 0; i<unallocated_counter; i++) 
			{	
				unsigned int frontier = robot_2_list[size-i];
				unsigned int frontier_size = frontierList.size()-1;
				frontierList[frontier_size].push_back(frontier);
			}
		}
	}
	else
	{
		myfile<<"Mismatch in number of robots \n";
	}
	

	//Coordination Strategy, starts if robot find no new frontiers and robot's own storage is also empty
	if(frontierList.size() == 0)
	{
		myfile<<"ENTERING COORDINATION STRATEGY\n";	
		if (count > 30)
		{
			//no frontiers found for robot 1 in this turn and no frontiers in own storage
			if(robotID == 1 && (!robot_2_list.empty()))
			{
				unsigned int size = robot_2_list.size()-1;
				unsigned int frontier_size = frontierList.size();
				frontierList[frontier_size].push_back(robot_2_list[size]);
			}

			//no frontiers found for robot 2 in this turn and no frontiers in own storage
			if(robotID == 2 && (!robot_1_list.empty()))
			{
				unsigned int size = robot_1_list.size()-1;	
				unsigned int frontier_size = frontierList.size();
				frontierList[frontier_size].push_back(robot_1_list[size]);
			}

			//no frontier found, no frontier left in all the other robot's storage, then follow the robot
			else if((robot_1_list.empty()) && (robot_2_list.empty()))
			{
				if(robot_flag != robotID && robot_flag != 100)
				{
					myfile<<"Exploration Finished \n";
					return EXPL_FINISHED;
				}
						
				if(robotID == 1)
				{
					unsigned int size = exploredFrontiers_robot1.size();
					if(size > 2)
					{ 
						goal = exploredFrontiers_robot1[size-2];
					}
					else
				        { 
						goal = exploredFrontiers_robot1[size-1];
					}
					robot_flag = robotID;	
					return EXPL_TARGET_SET;
				}
				else if(robotID == 2)
				{		
					unsigned int size = exploredFrontiers_robot2.size();
					if(size > 2)
					{ 
						goal = exploredFrontiers_robot2[size-2];
					}
					else
				        { 
						goal = exploredFrontiers_robot2[size-1];
					}
					robot_flag = robotID;	
					return EXPL_TARGET_SET;
				}
				else
				{
					myfile<<"Mismatch in number of robots and code\n";
				}				
			}
			else
			{
				myfile<<"PASS \n";
				return EXPL_FINISHED;
			}					
		}
		else
		{
			myfile<<"FAIL \n";
			return EXPL_FAILED;
		}
	}

	//if there is one single item in list, directly assign it
	unsigned int sizeoflist = frontierList[frontierList.size()-1].size();
	if(sizeoflist == 0)
	{
		myfile<<"Single Frontier .. Assigning directly \n";
		goal = frontierList[0][1];

		if(robotID == 1)
		{
			robot_1_counter++;
			exploredFrontiers_robot1.push_back(goal);
		}
		else if(robotID == 2)
		{
			robot_2_counter++;
			exploredFrontiers_robot2.push_back(goal);
		}
		return EXPL_TARGET_SET;
	}

	//compare the 'identified_frontiers' list of the robot with its 'explored_frontiers' list
	removeDuplicates(myfile);

	//test--
	myfile<<"List of identified frontiers after check \n";
	for(int i=0; i<frontierList.size(); i++)
	{
		for(int j=0; j<frontierList[i].size(); j++)
		{
			myfile<<frontierList[i][j]<<"\t";
		}	
	}
	myfile<<"\n";

	//Use Rank Based technique to allocate frontier from robot's 'identified_frontiers'
	unsigned int bestRank=9999;
	unsigned int bestFrontier=0;
	unsigned int rank=0;
	double bestDistance=999;
	double robotDistance=-1;
	FQueue rankedList;
	std::set<unsigned int> otherRobots;
	PoseList p = rList.getRobots();

	for(PoseList::iterator it = p.begin(); it!= p.end(); it++)
	{
		if((int) it->first == robotID) continue;
		unsigned int robot_x = (double)(it->second.x - map->getOriginX())/map->getResolution();
		unsigned int robot_y = (double)(it->second.y - map->getOriginY())/map->getResolution();
		unsigned int robot = 0;

		if(map->getIndex(robot_x, robot_y, robot))
		{
			otherRobots.insert(robot);
			myfile<<"Inserted robot at index "<<robot<<"\n";
		}
		else
		{
			myfile<<"Robot "<< it->first <<"cannot be placed in the map \n";
			ROS_WARN("Robot %d cannot be placed in the map", it->first);
		}
	}

	for(unsigned int f=0; f<frontierList.size(); f++)
	{
		double* grid = new double[map_size];
		for(unsigned int x=0; x<map_size; x++)
		{
			grid[x] = -1;
		}	

		for(unsigned int unit=0; unit<frontierList[f].size(); unit++)
		{
			grid[frontierList[f][unit]]=0;
			rankedList.insert(FValue(0.0,frontierList[f][unit]));		
		}
		
		while(!rankedList.empty())
		{
			FQueue::iterator after;
			after = rankedList.begin();
			distance = after->first;
			index = after->second;
			rankedList.erase(after);
			
			if(index == start)
			{
				robotDistance = distance;
				break;
			}
			if(otherRobots.find(index) != otherRobots.end()) rank++;

			for( unsigned int i=0; i<4; i++)
			{
				unsigned int ss = index + offset[i];	
				if( ss >= 0 && ss <map_size && map->getData(ss)==0 && grid[ss]==-1)
				{
					grid[ss]=distance + map_resolution;
					rankedList.insert(FValue(distance+map_resolution, ss));
				}
			}
		}
		
		if(robotDistance > 0 && (rank < bestRank || rank == bestRank && robotDistance < bestDistance))
		{
			bestRank=rank;
			bestFrontier=f;
			bestDistance=robotDistance;
		}
		delete[] grid;	
	}	
	
	if(bestFrontier >= frontierList.size())
	{
		myfile<<"Best frontier could not be determined \n";
		return EXPL_FAILED;
	}

	//chosen frontier is set as the immediate goal
	goal=frontierList.at(bestFrontier).at(0);


	//transfer extra frontiers to robot's individual 'unallocated_frontiers' lists
	if(robotID == 1)
	{
		//store explored frontier in the robot's 'explored_frontiers' list
		exploredFrontiers_robot1.push_back(frontierList[bestFrontier][0]);
		myfile<<"Chosen frontier with robot 1 is: "<<goal<<"\n";
		testfile<<"Chosen frontier with robot 1 is: "<<goal<<"\n";		

		robot_1_counter++;

		//delete the chosen frontier from 'identified_frontiers' list
		frontierList[bestFrontier].erase(frontierList[bestFrontier].begin() + 0);

		//adding unallocated frontiers to 'unallocated_lists'
		for(unsigned int i=0; i<frontierList.size(); i++)
		{
			for(unsigned int j=0; j<frontierList[i].size(); j++)
			{
				robot_1_list.push_back(frontierList[i][j]);
			}
		}

	}
	else if(robotID == 2)
	{
		//store explored frontier in the robot's 'explored_frontiers' list
		exploredFrontiers_robot2.push_back(frontierList[bestFrontier][0]);
		myfile<<"Chosen frontier with robot 2 is: "<<goal<<"\n";
		testfile<<"Chosen frontier with robot 2 is: "<<goal<<"\n";

		robot_2_counter++;
		
		//adding unallocated frontiers to 'unallocated_lists'
		for(unsigned int i=0; i<frontierList.size(); i++)
		{
			for(unsigned int j=0; j<frontierList[i].size(); j++)
			{
				robot_2_list.push_back(frontierList[i][j]);
			}
		}
	}
	else
	{
		ROS_WARN("Mismatch in number of robots");
	}	
	
	myfile<<"Robot 1 Counter: "<<robot_1_counter<<"\n";
	myfile<<"Robot 2 Counter: "<<robot_2_counter<<"\n";
	myfile<<"THIS ITERATION ENDS\n\n";

	myfile.close();
	testfile.close();

	return EXPL_TARGET_SET;
}

//This method identifies a cluster of frontier cells
void CaseBasedAllocation::findCluster(GridMap* map, unsigned int startCell)
{
	//write output to a file for metric purposes
	ofstream newfile("/home/raja/remember_clusters.txt", ios_base::app);

	FQueue cluster_queue;
	cluster_queue.insert(FValue(0.0,startCell));
	FrontierCells fCluster;
	double distance = 0.0;
	int index = 0;

	//use wavefront propogation to find clusters of frontiers
	while(!cluster_queue.empty())
	{
		FQueue::iterator next;
		next=cluster_queue.begin();
		distance = next->first;
		index = next->second;
		cluster_queue.erase(next);

		if(!map->isFrontier(index)) continue;
		
		fCluster.push_back(index);
		frontierCells++;
		newfile<<"Found a frontier: "<<index<<"\n";
		
		//grid[index] = distance + map->getResolution();

		unsigned int offset[4];
		offset[0]=-1;
		offset[1]=+1;
		offset[2]=-map->getWidth();
		offset[3]=+map->getWidth();

		for(int in = 0; in<4 ; in++)
		{
			unsigned int val = index + offset[in];
			if(map->isFree(val) && grid[val] == -1)
			{
				grid[val]=distance+map->getResolution();
				cluster_queue.insert(FValue(distance+ map->getResolution(),val));	
			}
		}
	}	

	frontierList.push_back(fCluster);
	fCluster.clear();
	newfile<<"New Cluster \n";
	newfile.close();
}

//Inefficient
//Needs to be changed to use hashmaps -- future work
void CaseBasedAllocation::removeDuplicates(std::ofstream& myfile)
{
	myfile<<"-------------------------------------------------\n";
	myfile<<"DUPLICATION CHECK\n";
	myfile<<"-------------------------------------------------\n";

	if(robotID == 1) 
	{	
		for(int i=0; i<frontierList.size(); i++)
		{
			for(int j=0; j<frontierList[i].size(); j++)
			{
				for(int k=0; k<exploredFrontiers_robot1.size(); k++)
				{
					myfile<<"Frontier: "<<frontierList[i][j]<<" compared with " <<exploredFrontiers_robot1[k]<<"\t";
					if(frontierList[i][j] == exploredFrontiers_robot1[k])
					{
						//found a duplicate
						myfile<<"Found a duplicate: "<<frontierList[i][j]<<"\n";
						frontierList[i].erase(frontierList[i].begin()+j);
						j--;
						break;
						
					}
					else
					{
						myfile<<" Not a duplicate \n";
					}
				}
			}
		}
	}
	else if(robotID == 2)
	{
		for(int i=0; i<frontierList.size(); i++)
		{
			for(int j=0; j<frontierList[i].size(); j++)
			{
				for(int k=0; k<exploredFrontiers_robot2.size(); k++)
				{
			
					myfile<<"Frontier: "<<frontierList[i][j]<<" compared with " <<exploredFrontiers_robot2[k]<<"\t";
					if(frontierList[i][j] == exploredFrontiers_robot2[k])
					{
						//found a duplicate
						myfile<<"Found a duplicate: "<<frontierList[i][j]<<"\n";
						frontierList[i].erase(frontierList[i].begin()+j);
						j--;
						break;
					}
				}
			}
		}
	}
	//myfile<<"-------------------------------------------------\n";
}
