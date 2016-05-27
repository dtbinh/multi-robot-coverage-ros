#include <pluginlib/class_list_macros.h>
#include <nav2d_navigator/ExplorationPlanner.h>

#include "NearestBasedAllocation.h"
#include "GreedyBasedAllocation.h"
#include "RankBasedAllocation.h"
#include "RankBasedExtention.h"
#include "CaseBasedAllocation.h"

PLUGINLIB_EXPORT_CLASS(NearestBasedAllocation, ExplorationPlanner)
PLUGINLIB_EXPORT_CLASS(GreedyBasedAllocation, ExplorationPlanner)
PLUGINLIB_EXPORT_CLASS(RankBasedAllocation, ExplorationPlanner)
PLUGINLIB_EXPORT_CLASS(RankBasedExtention, ExplorationPlanner)
PLUGINLIB_EXPORT_CLASS(CaseBasedAllocation, ExplorationPlanner)
