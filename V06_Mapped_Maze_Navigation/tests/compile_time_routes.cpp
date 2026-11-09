#include "../RouteMap.h"
using namespace route;
// Executed by the compiler, so route-planner tests also work on managed Windows
// hosts that prohibit launching newly built test executables.
constexpr bool shortcut() {
  Graph g; g.nodeCount=5;
  g.connect(0,1,0,1,100); g.connect(1,2,1,1,300);
  g.connect(2,3,2,1,300); g.connect(1,3,3,1,50); g.connect(3,4,4,1,100);
  uint16_t p[MAX_NODES]={};
  return g.shortest(0,4,p,MAX_NODES)==3 && p[0]==0 && p[1]==3 && p[2]==4;
}
constexpr bool noInventedPassages() {
  Graph g; g.nodeCount=3; uint16_t p[MAX_NODES]={};
  g.connect(0,1,0,1,100);
  return g.shortest(1,0,p,MAX_NODES)==-1 && g.shortest(0,2,p,MAX_NODES)==-1 &&
    g.shortest(0,0,p,MAX_NODES)==0 && g.shortest(-1,0,p,MAX_NODES)==-1;
}
constexpr bool turnShortcut() {
  Graph g; g.nodeCount=4; uint16_t p[MAX_NODES]={};
  g.connect(0,1,0,1,1000); g.connect(1,2,1,1,8000); g.connect(2,3,2,1,1000);
  g.edges[g.edgeCount++]={1,2,0,0,1040,2};
  return g.shortest(0,3,p,MAX_NODES)==3 && p[1]==3;
}
constexpr bool capacities() {
  Graph g; g.nodeCount=3; uint16_t p[MAX_NODES]={};
  g.connect(0,1,0,1,1); g.connect(1,2,1,1,1);
  if(g.shortest(0,2,p,1)!=-1) return false;
  if(g.connect(0,1,MAX_COMMANDS,1,1)) return false;
  g.edgeCount=MAX_EDGES;
  return !g.connect(0,1,0,1,1);
}
static_assert(shortcut(), "Dijkstra must use the discovered shortcut");
static_assert(noInventedPassages(), "Do not invent reverse or unknown passages");
static_assert(turnShortcut(), "A mapped heading transition must remove the excursion");
static_assert(capacities(), "Planner must enforce storage/output limits");
constexpr bool matchesAndBridges() {
  Graph g;
  g.landmark({0,0,0},0x8f,0,true);
  g.landmark({0,0,PI_F/2},0x8f,520,true);
  int back=g.landmark({10,0,PI_F},0x8f,5000,true);
  if(back!=2 || g.edgeCount!=2 || g.edges[0].turnQuarters!=1) return false;
  Graph ambiguous;
  ambiguous.landmark({0,0,0},3,0,false);
  ambiguous.landmark({100,0,0},3,0,false);
  if(ambiguous.landmark({50,0,0},3,5000,true)!=2) return false;
  Graph exact;
  exact.landmark({0,0,0},3,0,false);
  if(exact.landmark({10,0,0},3,5000,true)!=0) return false;
  if(exact.landmark({10,0,PI_F},3,10000,true)!=1) return false;
  return exact.landmark({10,0,0},7,15000,true)==2;
}
static_assert(matchesAndBridges(), "Revisit matching must reject ambiguity and support observed headings");
static_assert(validCommand({800,0,0,0,CRUISE,0,0}), "Valid cruise");
static_assert(!validCommand({800,0,0,0,RAW,0,0}), "Reject movement with no motor drive");
static_assert(!validCommand({0,0,0,0,CHECK,255,0}), "Reject invalid sensor masks");
constexpr bool finishExactlyAtCheckpoint() {
  Graph g; g.nodeCount=3; uint16_t p[MAX_NODES]={};
  g.connect(0,1,0,1,100);
  if(!g.connect(1,2,1,0,0)) return false;
  return g.shortest(0,2,p,MAX_NODES)==2;
}
static_assert(finishExactlyAtCheckpoint(), "Finish at an existing motion boundary must be saveable");
