#ifndef NEARESTBASEDALLOCATION_H_
#define NEARESTBASEDALLOCATION_H_
#include<nav2d_navigator/ExplorationPlanner.h>

class NearestBasedAllocation : public ExplorationPlanner
{
public:
NearestBasedAllocation();
~NearestBasedAllocation();

int findExplorationTarget(GridMap* map, unsigned int start, unsigned int &goal);

};
#endif

















