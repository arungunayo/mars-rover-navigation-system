#include "planner.hpp"
#include <iostream>
#include <random>
using namespace rover;
void check(bool ok,const char* message) { if(!ok) throw std::runtime_error(message); }
int main() {
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
  std::cout<<"[TEST] passed: geometry, boundaries, slope, terrain costs, 100 seeded comparisons, inflation\n";
}
