#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <vector>

namespace rover {
struct Cell { double elevation=0, roughness=0; bool blocked=false; };
struct Grid {
  int width, height; double resolution;
  std::vector<Cell> cells;
  std::vector<unsigned short> blockedMoves;
  Grid(int w,int h,double r):width(w),height(h),resolution(r) {
    if(w<=0 || h<=0 || r<=0) throw std::invalid_argument("Invalid grid dimensions");
    cells.resize(w*h);
    blockedMoves.resize(w*h,0);
  }
  bool contains(int x,int y) const { return x>=0 && y>=0 && x<width && y<height; }
  int id(int x,int y) const { return y*width+x; }
};
struct Costs { double slopeWeight=4, roughnessWeight=2, maxSlope=0.6; };
struct Result {
  std::vector<int> path; double cost=std::numeric_limits<double>::infinity();
  int expanded=0; double milliseconds=0;
};
inline Result plan(const Grid& g,int start,int goal,bool astar,const Costs& weights={}) {
  if(weights.slopeWeight<0 || weights.roughnessWeight<0 || weights.maxSlope<0)
    throw std::invalid_argument("Costs must be nonnegative");
  for(const auto& c:g.cells) if(!std::isfinite(c.elevation) || !std::isfinite(c.roughness) || c.roughness<0)
    throw std::invalid_argument("Invalid terrain data");
  auto begin=std::chrono::steady_clock::now(); Result result;
  auto finish=[&]() { result.milliseconds=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count(); return result; };
  int n=static_cast<int>(g.cells.size());
  if(start<0 || goal<0 || start>=n || goal>=n || g.cells[start].blocked || g.cells[goal].blocked) return finish();
  auto heuristic=[&](int v) { return astar ? g.resolution*std::hypot(v%g.width-goal%g.width,v/g.width-goal/g.width):0.0; };
  struct Entry { double f,d; int v; bool operator<(const Entry& b) const { return f!=b.f ? f>b.f : v>b.v; } };
  std::priority_queue<Entry> open;
  std::vector<double> distance(n,std::numeric_limits<double>::infinity());
  std::vector<int> parent(n,-1); distance[start]=0; open.push({heuristic(start),0,start});
  while(!open.empty()) {
    auto current=open.top(); open.pop(); int u=current.v;
    if(current.d>distance[u]) continue;
    ++result.expanded;
    if(u==goal) {
      result.cost=distance[u];
      for(int v=goal;v!=-1;v=parent[v]) result.path.push_back(v);
      std::reverse(result.path.begin(),result.path.end()); return finish();
    }
    int x=u%g.width,y=u/g.width;
    for(int dy=-1;dy<=1;++dy) for(int dx=-1;dx<=1;++dx) {
      if((dx==0 && dy==0) || !g.contains(x+dx,y+dy)) continue;
      if(g.blockedMoves[u] & (1u<<((dy+1)*3+dx+1))) continue;
      int v=g.id(x+dx,y+dy); if(g.cells[v].blocked) continue;
      if(dx && dy && (g.cells[g.id(x+dx,y)].blocked || g.cells[g.id(x,y+dy)].blocked)) continue;
      double length=g.resolution*std::hypot(dx,dy);
      double slope=std::abs(g.cells[v].elevation-g.cells[u].elevation)/length;
      if(slope>weights.maxSlope) continue;
      double roughness=(g.cells[u].roughness+g.cells[v].roughness)/2;
      double candidate=distance[u]+length*(1+weights.slopeWeight*slope+weights.roughnessWeight*roughness);
      if(candidate<distance[v]) { distance[v]=candidate; parent[v]=u; open.push({candidate+heuristic(v),candidate,v}); }
    }
  }
  return finish();
}
// Occupancy inflation uses a separate source mask so repeated calls do not grow obstacles.
inline void inflate(Grid& g,const std::vector<bool>& occupied,double clearance) {
  if(occupied.size()!=g.cells.size() || clearance<0) throw std::invalid_argument("Invalid occupancy input");
  int radius=static_cast<int>(std::ceil(clearance/g.resolution));
  for(auto& c:g.cells) c.blocked=false;
  for(int y=0;y<g.height;++y) for(int x=0;x<g.width;++x) if(occupied[g.id(x,y)])
    for(int dy=-radius;dy<=radius;++dy) for(int dx=-radius;dx<=radius;++dx)
      if(g.contains(x+dx,y+dy) && std::hypot(dx,dy)*g.resolution<=clearance)
        g.cells[g.id(x+dx,y+dy)].blocked=true;
}
}
