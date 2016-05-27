#ifndef RANKBASEDALLOCATION_H_
#define RANKBASEDALLOCATION_H_
#include<nav2d_navigator/ExplorationPlanner.h>

class RankBasedAllocation : public ExplorationPlanner
{
public:
RankBasedAllocation();
~RankBasedAllocation();

int findExplorationTarget(GridMap* map, unsigned int start, unsigned int &goal);

private: 
 	typedef std::vector<unsigned int> Frontiers;
        typedef std::vector<Frontiers> ListOfFrontiers;
        
        // Methods
        void findCluster(GridMap* map, unsigned int startCell);
        
        // ROS-Stuff
        //ros::Publisher mFrontierPublisher;
        
        // Components
        RobotList robotList;
        ListOfFrontiers frontierList;
        unsigned int frontierCells;
	unsigned int front_count;
        double* plan;
        // Parameters
        int robotID;
	int ind_flag;
	int robot_1_counter;
	int robot_2_counter;
	int robot_3_counter;
};
#endif





