#ifndef CASEBASEDALLOCATION_H_
#define CASEBASEDALLOCATION_H_
#include<nav2d_navigator/ExplorationPlanner.h>

class CaseBasedAllocation : public ExplorationPlanner
{

	public:
		CaseBasedAllocation();
		~CaseBasedAllocation();

		int findExplorationTarget(GridMap* map, unsigned int start, unsigned int &goal);

	private: 
 		typedef std::vector<unsigned int> FrontierCells;
        	typedef std::vector<FrontierCells> ListOfFrontiers;
        
        	// Methods
        	void findCluster(GridMap* map, unsigned int startCell);
		void removeDuplicates(std::ofstream& myfile);
		void printAllTheLists(std::ofstream& myfile, int id);
        
		// Components
		RobotList rList;
		
		//'identified_frontiers' list
		ListOfFrontiers frontierList;

		//'explored_frontiers' list
		FrontierCells exploredFrontiers_robot1;
		FrontierCells exploredFrontiers_robot2;
		
		//'unallocated_frontiers' list
		FrontierCells robot_1_list;
		FrontierCells robot_2_list;


		double* grid;
		int robotID;
		int robot_1_counter;
		int robot_2_counter;
		int dup_counter;
		unsigned int robot_flag;
		unsigned int robotCount;
		unsigned int frontierCells;
		unsigned int unallocated_counter;
		int robot1_flag;
		int robot2_flag;

};
#endif
