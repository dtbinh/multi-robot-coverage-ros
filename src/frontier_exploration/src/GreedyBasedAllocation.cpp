#include<GreedyBasedAllocation.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/GridCells.h>

typedef std::multimap<double, unsigned int> FrontQueue;
typedef std::pair<double, unsigned int> Value;

using namespace ros;

	GreedyBasedAllocation::GreedyBasedAllocation()
	{
		ros::NodeHandle robot;
		robot.param("robot_id", robotID, 1);
		waitForOtherRobots = false;
	}

GreedyBasedAllocation::~GreedyBasedAllocation()
{

}

int GreedyBasedAllocation::findExplorationTarget(GridMap* map, unsigned int start, unsigned int &goal)
{
unsigned int size=map->getSize();
double* plan = new double[size];
	for(unsigned int x=0; x<size; x++)
	{
		plan[x] = 0;
	}
double resolution = map->getResolution();
bool frontierFound = false;
bool frontierAvailable =  false;
unsigned int count=0, ind;
double dist;
FrontQueue list;
FrontQueue listOthers;
FrontQueue::iterator next;
Value startvalue(0.0,start);
list.insert(startvalue);
plan[start]=robotID;
PoseList p = robotsList.getRobots();
	for(PoseList::iterator it = p.begin(); it!= p.end(); it++)
	{
		if((int) it->first == robotID) continue;
		unsigned int robot_x = (double)(it->second.x - map->getOriginX())/map->getResolution();
		unsigned int robot_y = (double)(it->second.y - map->getOriginY())/map->getResolution();
		unsigned int robot = 0;
		if(map->getIndex(robot_x, robot_y, robot))
		{
			list.insert(Value(0.0, robot));
			plan[robot] = it->first;	
		}
		else
		{
			ROS_WARN("Robot %d cannot be placed in the map", it->first);
		}
	}

	while(!list.empty())
	{
		count++;
		next=list.begin();
		dist = next->first;
		ind = next->second;
		list.erase(next);
		int currentRobot = plan[ind];
		if(map->isFrontier(ind))
		{
			frontierAvailable=true;
			if(currentRobot == robotID)
			{
				frontierFound=true;
				goal=ind;
				break;
			}
		}
		else
		{
			std::vector<unsigned int> neighbors = map->getFreeNeighbors(ind);
			for(unsigned int i = 0; i < neighbors.size(); i++)
			{
				unsigned int x,y;
				if(!map->getCoordinates(x,y,neighbors[i])) continue;
				if(plan[neighbors[i]] == 0)
				{
					list.insert(Value(dist+resolution, neighbors[i]));
					plan[neighbors[i]] = currentRobot;
				}
				else if(plan[neighbors[i]] != robotID && currentRobot == robotID)
				{
					listOthers.insert(Value(dist+resolution, neighbors[i]));
				}
			}				
		}
	}

	if(!waitForOtherRobots)
	{
		while(!frontierFound && !listOthers.empty())
		{
			count++;
			next = listOthers.begin();
			double dist = next->first;
			unsigned int ind = next->second;
			listOthers.erase(next);
			
			if(map->isFrontier(ind))
			{
				frontierFound = true;
				goal = ind;
				break;
			}else
			{
				std::vector<unsigned int> neighbors = map->getFreeNeighbors(ind);
				for(unsigned int i = 0; i < neighbors.size(); i++)
				{
					if(map->isFree(neighbors[i]) && plan[neighbors[i]] != robotID)
					{
						unsigned int x,y;
						if(!map->getCoordinates(x,y,neighbors[i])) continue;
						listOthers.insert(Value(dist+resolution, neighbors[i]));
						plan[neighbors[i]] = robotID;
					}
				}
			}
		}
	}
delete[] plan;

	if(frontierFound)
	{
		return EXPL_TARGET_SET;
	}
	else if(frontierAvailable)
	{
		if(listOthers.size()>0)
		{
			return EXPL_WAITING;
		}
		else
		{
			ROS_WARN("Frontiers are available but are not reachable");
			return EXPL_FAILED;
		}
	}
	else
	{
		if(count>50)
			return EXPL_FAILED;
		else
			ROS_WARN("No Frontiers are available");
	}
}
