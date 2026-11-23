#include "planner.hpp"
#include "terrain.hpp"
#include "discovery.hpp"
#include "obstacles.hpp"
#include <iostream>
#include <random>
using namespace rover;
void check(bool ok,const char* message) { if(!ok) throw std::runtime_error(message); }
int main() {
  Grid unknown(7,3,1); std::vector<bool> seen(21,false);
  seen[7]=seen[8]=seen[9]=true;
  auto local=discoveredGrid(unknown,seen);
  check(plan(local,7,13,true).path.empty(),"Unknown terrain must not be planned through");
  std::vector<int> visits(21,0);
  check(chooseLocalTarget(local,seen,7,13,visits)==9,"Discovery selects local reachable frontier");
  for(int i=7;i<=13;++i) seen[i]=true;
  check(chooseLocalTarget(discoveredGrid(unknown,seen),seen,7,13,visits)==13,"Goal becomes available after discovery");
  Grid loopMap(5,3,1); std::vector<bool> loopSeen(15,true); std::vector<int> loopVisits(15,0);
  for(int i=0;i<5;++i) loopSeen[i]=false;
  auto loopKnown=discoveredGrid(loopMap,loopSeen);
  loopVisits[6]=loopVisits[8]=1;
  ExplorationMemory memory(15); memory.record(loopKnown,loopSeen,6); memory.record(loopKnown,loopSeen,8);
  int alternative=chooseLocalTarget(loopKnown,loopSeen,7,2,loopVisits,{},&memory);
  check(alternative==5 || alternative==9,"Completed pair must not be selected again");
  loopVisits[5]=loopVisits[9]=1;
  memory.record(loopKnown,loopSeen,5); memory.record(loopKnown,loopSeen,9);
  check(chooseLocalTarget(loopKnown,loopSeen,7,2,loopVisits,{},&memory)==-1,"Exhausted exploration must stop instead of cycling");
  check(chooseLocalTarget(loopKnown,loopSeen,7,2,loopVisits)!=-1,"Previously traversed frontiers must remain explorable");
  loopSeen[3]=true; loopKnown=discoveredGrid(loopMap,loopSeen);
  check(chooseLocalTarget(loopKnown,loopSeen,7,2,loopVisits,{},&memory)!=-1,"New local information reopens a previously scanned frontier");
  check(!plan(loopKnown,5,9,true).path.empty(),"Visited cells remain available for backtracking");
  Grid edgeObstacle(4,3,1);
  blockObstacleDisc(edgeObstacle,0,0,1.5,1,.2);
  check(!edgeObstacle.cells[5].blocked && !edgeObstacle.cells[6].blocked,"Obstacle between vertices must not erase free endpoints");
  auto around=plan(edgeObstacle,5,6,true);
  check(around.path.size()>2,"A path must not cut through a rock between free vertices");
  check(std::abs(around.cost-plan(edgeObstacle,5,6,false).cost)<1e-9,"Edge collision constraints preserve A*/Dijkstra agreement");
  Grid coarsePassage(9,5,.78125),physicalPassage(9,5,.78125);
  std::vector<bool> roundedHits(45,false); roundedHits[coarsePassage.id(4,1)]=roundedHits[coarsePassage.id(4,3)]=true;
  inflate(coarsePassage,roundedHits,1.15);
  check(plan(coarsePassage,coarsePassage.id(0,2),coarsePassage.id(8,2),true).path.empty(),"Old coarse inflation closes the passage");
  blockObstacleDisc(physicalPassage,0,0,4*.78125,.5*.78125,.65);
  blockObstacleDisc(physicalPassage,0,0,4*.78125,3.5*.78125,.65);
  auto passageRoute=plan(physicalPassage,physicalPassage.id(0,2),physicalPassage.id(8,2),true);
  check(std::abs(passageRoute.cost-6.25)<1e-9,"Physical clearance must retain the safe corridor");
  Grid plane(5,5,.1);
  for(int y=0;y<5;++y) for(int x=0;x<5;++x) plane.cells[plane.id(x,y)].elevation=.02*x+.01*y;
  deriveRoughness(plane);
  check(plane.cells[12].roughness<1e-12,"Planar slope is not roughness");
  plane.cells[12].elevation+=.1; deriveRoughness(plane);
  check(plane.cells[12].roughness>.9,"Local bump produces roughness");
  Grid g(5,5,1);
  check(std::abs(plan(g,0,24,true).cost-4*std::sqrt(2.0))<1e-9,"Diagonal distance");
  check(plan(g,0,0,true).path.size()==1,"Start equals goal");
  g.cells[1].blocked=true; g.cells[5].blocked=true;
  check(plan(g,0,24,true).path.empty(),"Corner cutting");
  check(plan(g,-1,24,true).path.empty(),"Invalid start");
  Grid cliff(2,1,1); cliff.cells[1].elevation=1;
  check(plan(cliff,0,1,true).path.empty(),"Slope limit");
  Grid rough(5,3,1); rough.cells[rough.id(2,1)].roughness=100;
  auto detour=plan(rough,rough.id(0,1),rough.id(4,1),true);
  check(std::find(detour.path.begin(),detour.path.end(),rough.id(2,1))==detour.path.end(),"Avoid expensive terrain");
  std::mt19937 rng(42);
  for(int trial=0;trial<100;++trial) {
    Grid random(20,20,0.2);
    for(auto& c:random.cells) { c.blocked=rng()%5==0; c.elevation=(rng()%20)*0.001; c.roughness=(rng()%100)*0.01; }
    random.cells.front().blocked=random.cells.back().blocked=false;
    auto d=plan(random,0,399,false),a=plan(random,0,399,true);
    check(d.path.empty()==a.path.empty(),"Reachability mismatch");
    if(!d.path.empty()) check(std::abs(d.cost-a.cost)<1e-8,"Optimal cost mismatch");
  }
  Grid inflated(5,5,1); std::vector<bool> mask(25,false); mask[12]=true;
  inflate(inflated,mask,1); inflate(inflated,mask,1);
  check(inflated.cells[7].blocked && !inflated.cells[2].blocked,"Inflation repeated growth");
  std::cout<<"[TEST] passed: graph search, 100 seeded comparisons, physical obstacle clearance, safe corridor, frontier revisits and loop prevention\n";
}
