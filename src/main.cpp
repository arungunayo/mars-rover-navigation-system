#include "planner.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
using namespace rover;
void report(const char* name,const Result& r) {
  std::cout<<"[PLAN] algorithm="<<name<<" status="<<(r.path.empty()?"no_path":"success")
    <<" cost="<<r.cost<<" expanded="<<r.expanded<<" time_ms="<<r.milliseconds<<'\n';
}
int main() {
  Grid map(60,40,0.2);
  for(int y=0;y<map.height;++y) for(int x=0;x<map.width;++x) {
    auto& c=map.cells[map.id(x,y)]; c.elevation=0.15*std::sin(x*0.12)*std::cos(y*0.1);
    c.roughness=(x>=20 && x<35 && y<25)?0.8:0.05;
  }
  std::vector<bool> occupied(map.cells.size(),false);
  for(int y=8;y<32;++y) occupied[map.id(40,y)]=true;
  inflate(map,occupied,0.3);
  int start=map.id(3,20),goal=map.id(55,20);
  std::cout<<std::fixed<<std::setprecision(4)<<"[MISSION] mode=synthetic_planner_demo slam=not_connected\n";
  auto d=plan(map,start,goal,false), a=plan(map,start,goal,true);
  report("Dijkstra",d); report("A*",a);
  if(a.path.empty() || std::abs(d.cost-a.cost)>1e-8) return 1;
  int current=a.path[a.path.size()/4], obstacle=a.path[a.path.size()/2];
  occupied[obstacle]=true; inflate(map,occupied,0.3);
  std::cout<<"[OBSTACLE] source=synthetic path_blocked=true\n";
  auto revised=plan(map,current,goal,true); report("A*_replan",revised);
  std::filesystem::create_directories("results");
  std::ofstream csv("results/planner_metrics.csv");
  csv<<"algorithm,cost,nodes_expanded,runtime_ms\n";
  for(auto entry: {std::make_pair("Dijkstra",d),std::make_pair("A*",a),std::make_pair("A*_replan",revised)})
    csv<<entry.first<<','<<entry.second.cost<<','<<entry.second.expanded<<','<<entry.second.milliseconds<<'\n';
  std::ofstream grid("results/terrain.csv"); grid<<"x,y,elevation,roughness,blocked\n";
  for(int y=0;y<map.height;++y) for(int x=0;x<map.width;++x) {
    auto c=map.cells[map.id(x,y)]; grid<<x<<','<<y<<','<<c.elevation<<','<<c.roughness<<','<<c.blocked<<'\n';
  }
  std::ofstream paths("results/paths.csv"); paths<<"phase,step,x,y\n";
  for(auto entry:{std::make_pair("initial",a),std::make_pair("replanned",revised)})
    for(size_t i=0;i<entry.second.path.size();++i) { int v=entry.second.path[i]; paths<<entry.first<<','<<i<<','<<v%map.width<<','<<v/map.width<<'\n'; }
  std::cout<<"[MISSION] status="<<(revised.path.empty()?"no_path":"replan_verified")<<" results=results/\n";
  return revised.path.empty()?1:0;
}
