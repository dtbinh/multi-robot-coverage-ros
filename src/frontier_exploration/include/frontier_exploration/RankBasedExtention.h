#ifndef RANKBASEDEXTENTION_H_
#define RANKBASEDEXTENTION_H_
#include<nav2d_navigator/ExplorationPlanner.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>
class RankBasedExtention : public ExplorationPlanner
{
public:
RankBasedExtention();
~RankBasedExtention();

int findExplorationTarget(GridMap* map, unsigned int start, unsigned int &goal);
void planCallback(const std_msgs::Int64::ConstPtr& msg);
private: 
 	typedef std::vector<unsigned int> Frontiers;
        typedef std::vector<Frontiers> ListOfFrontiers;
        
        // Methods
        void findCluster(GridMap* map, unsigned int startCell);
        
        // ROS-Stuff
        ros::Publisher nxtPlan;
	ros::Publisher nxtFrontier;
        ros::Publisher currentGoal;
	ros::Publisher repeat;
	ros::Publisher overlap;

        // Components
        RobotList robotList;
        ListOfFrontiers frontierList;
        unsigned int frontierCells;
	unsigned int front_count;
        double* plan;
        // Parameters
        int robotID;
        int nxtGoal;
	int goalRobot;
	bool nextG;
	std_msgs::Int64 nxtmsg;
      	std_msgs::Int64 nextgoalmsg;
	std_msgs::Int64 goalmsg;
	std_msgs::Int64 count;
	std_msgs::Bool test;
	int ind_flag;
	int robot_1_counter;
	int robot_2_counter;		
	int robot_3_counter;
};
#endif

