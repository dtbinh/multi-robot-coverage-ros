#ifndef GREEDYBASEDALLOCATION_H_
#define GREEDYBASEDALLOCATION_H_
#include<nav2d_navigator/ExplorationPlanner.h>

class GreedyBasedAllocation : public ExplorationPlanner
{
public:
GreedyBasedAllocation();
~GreedyBasedAllocation();

int findExplorationTarget(GridMap* map, unsigned int start, unsigned int &goal);

private:
int robotID;
bool waitForOtherRobots;
RobotList robotsList;

};
#endif




