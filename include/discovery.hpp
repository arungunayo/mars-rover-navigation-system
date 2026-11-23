#pragma once
#include "planner.hpp"
#include <cstdint>
namespace rover {
inline std::uint64_t observationSignature(const Grid& known,const std::vector<bool>& observed,int node) {
  std::uint64_t hash=1469598103934665603ull;
  int radius=static_cast<int>(std::ceil(2.6/known.resolution));
  for(int dy=-radius;dy<=radius;++dy) for(int dx=-radius;dx<=radius;++dx) {
    int x=node%known.width+dx,y=node/known.width+dy;
    unsigned value=3;
    if(known.contains(x,y)) { int v=known.id(x,y); value=!observed[v]?0:known.cells[v].blocked?2:1; value+=4*known.blockedMoves[v]; }
    hash^=value; hash*=1099511628211ull;
  }
  return hash;
}
struct ExplorationMemory {
  std::vector<std::uint64_t> signatures;
  std::vector<bool> scanned;
  explicit ExplorationMemory(size_t n):signatures(n,0),scanned(n,false) {}
  void record(const Grid& known,const std::vector<bool>& observed,int node) {
    scanned[node]=true; signatures[node]=observationSignature(known,observed,node);
  }
  bool exhausted(const Grid& known,const std::vector<bool>& observed,int node) const {
    return scanned[node] && signatures[node]==observationSignature(known,observed,node);
  }
};
// Truth is used by simulated sensing only. Unknown cells are forbidden to search.
inline Grid discoveredGrid(const Grid& terrain,const std::vector<bool>& seen) {
  if(seen.size()!=terrain.cells.size()) throw std::invalid_argument("Discovery mask mismatch");
  Grid known=terrain;
  for(size_t i=0;i<seen.size();++i) if(!seen[i]) { known.cells[i].blocked=true; known.cells[i].elevation=0; known.cells[i].roughness=0; }
  return known;
}
inline int chooseLocalTarget(const Grid& known,const std::vector<bool>& observed,int current,int goal,const std::vector<int>& visits,const Costs& costs={},const ExplorationMemory* memory=nullptr) {
  if(observed.size()!=known.cells.size() || visits.size()!=known.cells.size()) throw std::invalid_argument("Exploration state mismatch");
  if(!known.cells[goal].blocked && !plan(known,current,goal,true,costs).path.empty()) return goal;
  // Rank reachable frontiers by actual terrain-weighted route cost plus a lower
  // bound on remaining travel. Previous transit is never a reason to reject one.
  struct Candidate { double score; int information,visits,node; };
  std::vector<Candidate> candidates;
  for(int i=0;i<static_cast<int>(known.cells.size());++i) if(i!=current && !known.cells[i].blocked) {
    int unknownNeighbors=0,x=i%known.width,y=i/known.width;
    for(int dy=-1;dy<=1;++dy) for(int dx=-1;dx<=1;++dx)
      if((dx||dy) && known.contains(x+dx,y+dy) && !observed[known.id(x+dx,y+dy)]) ++unknownNeighbors;
    if(unknownNeighbors==0) continue;
    if(memory && memory->exhausted(known,observed,i)) continue;
    auto route=plan(known,current,i,true,costs); if(route.path.empty()) continue;
    double progress=known.resolution*std::hypot(i%known.width-goal%known.width,i/known.width-goal/known.width);
    candidates.push_back({route.cost+progress,unknownNeighbors,visits[i],i});
  }
  std::sort(candidates.begin(),candidates.end(),[](const Candidate& a,const Candidate& b) {
    if(a.score!=b.score) return a.score<b.score;
    if(a.information!=b.information) return a.information>b.information;
    if(a.visits!=b.visits) return a.visits<b.visits;
    return a.node<b.node;
  });
  if(!candidates.empty()) return candidates.front().node;
  return -1;
}
}
