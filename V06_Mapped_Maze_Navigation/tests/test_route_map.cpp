#include "../RouteMap.h"
#include <assert.h>
#include <stdio.h>
using namespace route;

void shortestShortcut() {
  Graph g;
  for(int i=0;i<5;++i) g.landmark({float(i*1000),0,0},2,i*1000,false);
  assert(g.connect(0,1,0,1,100));
  assert(g.connect(1,2,1,1,300));
  assert(g.connect(2,3,2,1,300));
  assert(g.connect(1,3,3,1,50));
  assert(g.connect(3,4,4,1,100));
  uint16_t p[MAX_NODES]; int n=g.shortest(0,4,p,MAX_NODES);
  assert(n==3 && p[0]==0 && p[1]==3 && p[2]==4);
  assert(g.shortest(4,0,p,MAX_NODES)==-1); // no invented reverse corridor
  assert(g.shortest(0,4,p,2)==-1);
}
void localization() {
  Graph g;
  assert(g.landmark({0,0,0},3,0,true)==0);
  assert(g.landmark({20,0,0},3,100,true)==1); // too soon to call a revisit
  assert(g.landmark({20,0,0},3,3000,true)==2); // two ambiguous landmarks
  g.clear(); g.landmark({0,0,0},3,0,true);
  assert(g.landmark({50,10,0.03f},3,3000,true)==0);
  assert(g.landmark({50,10,0.03f},7,6000,true)==1); // different walls
  assert(g.landmark({50,10,PI_F},3,9000,true)==2); // different non-centre heading
  assert(g.landmark({50,10,0},255,10000,false)==3); // explicit goal never merged
}
void loopHeadingBridge() {
  Graph g;
  int start=g.landmark({-1000,0,0},2,0,false);
  int a=g.landmark({0,0,0},0x8f,1000,true);
  int b=g.landmark({0,0,PI_F/2},0x8f,1520,true);
  assert(g.connect(start,a,0,1,1000));
  assert(g.connect(a,b,1,1,520)); // actual explored turn
  int back=g.landmark({15,0,PI_F},0x8f,10000,true);
  assert(g.connect(b,back,2,1,8000)); // long excursion
  int goal=g.landmark({-1000,10,PI_F},255,11000,false);
  assert(g.connect(back,goal,3,1,1000));
  uint16_t p[MAX_NODES]; int n=g.shortest(start,goal,p,MAX_NODES);
  assert(n>0);
  float cost=0; bool usedBridge=false;
  for(int i=0;i<n;++i) { cost+=g.edges[p[i]].cost; usedBridge|=g.edges[p[i]].turnQuarters!=0; }
  assert(usedBridge && cost<4000); // dead-end excursion actually removed
}
void odometryAndStorage() {
  Pose p={0,0,0}; integrate(p,800,800);
  assert(fabsf(p.x-800)<0.01f && fabsf(p.y)<0.01f);
  integrate(p,-520,520); assert(fabsf(p.yaw-PI_F/2)<0.001f);
  integrate(p,800,800); assert(fabsf(p.y-800)<0.01f);
  integrate(p,-800,-800); assert(fabsf(p.y)<0.01f);
  Command c={800,0,0,0,CRUISE,0,0}; assert(validCommand(c));
  uint32_t h=checksum(&c,sizeof(c)); c.left++;
  assert(h!=checksum(&c,sizeof(c)));
  c.kind=RAW; assert(!validCommand(c)); // nonzero distance with zero motor speed
  c={520,520,-150,150,RAW,0,0}; assert(validCommand(c));
  c={0,0,0,0,CHECK,7,0}; assert(validCommand(c));
  c.mask=255; assert(!validCommand(c));
}
void capacityAndDisconnected() {
  Graph g; uint16_t p[MAX_NODES];
  assert(g.shortest(0,0,p,MAX_NODES)==-1);
  for(int i=0;i<MAX_NODES;++i) assert(g.landmark({float(i*1000),0,0},2,0,false)==i);
  assert(g.landmark({0,0,0},2,0,false)==-1);
  assert(g.shortest(0,1,p,MAX_NODES)==-1);
  for(int i=0;i<MAX_EDGES;++i) assert(g.connect(0,1,0,1,10));
  assert(!g.connect(0,1,0,1,10));
}
int main() {
  shortestShortcut(); localization(); loopHeadingBridge(); odometryAndStorage(); capacityAndDisconnected();
  puts("PASS: shortest routes, heading bridges, ambiguous matches, odometry, storage validation, capacity.");
}
