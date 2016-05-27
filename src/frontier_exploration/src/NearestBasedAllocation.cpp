#include<NearestBasedAllocation.h>
using namespace std;

typedef std::multimap<double, unsigned int> FrontQueue;
typedef std::pair<double, unsigned int> Value;

NearestBasedAllocation::NearestBasedAllocation()
{

}

NearestBasedAllocation::~NearestBasedAllocation()
{

}

int NearestBasedAllocation::findExplorationTarget(GridMap* map, unsigned int start, unsigned int &goal)
{
unsigned int size=map->getSize();
double* plan = new double[size];
for(unsigned int x=0; x<size; x++)
{
plan[x] = -1;
}

double resolution = map->getResolution();
bool frontierFound=false;
unsigned int count=0, ind;
double dist;
FrontQueue list;
FrontQueue::iterator next;
Value startvalue(0.0,start);
list.insert(startvalue);
plan[start]=0;

while(!list.empty())
{
count++;
next=list.begin();
dist = next->first;
ind = next->second;
list.erase(next);
if(map->isFrontier(ind))
{
frontierFound=true;
goal=ind;
break;
}

else
{
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
list.insert(Value(dist+resolution, y));
plan[y]=dist+resolution;
}
}
}
}
ROS_DEBUG("Checked %d cells",count);
delete[] plan;
if(frontierFound)
{
return EXPL_TARGET_SET;
}
else if (count>30)
{
return EXPL_FINISHED;
}
else
{
return EXPL_FAILED;
}
}

