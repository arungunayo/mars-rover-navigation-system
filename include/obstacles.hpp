#pragma once
#include "planner.hpp"
namespace rover {
struct ObstaclePoint { double x,y; };
inline double segmentDistanceSquared(double ax,double ay,double bx,double by,double px,double py) {
  double dx=bx-ax,dy=by-ay,length=dx*dx+dy*dy;
  double t=length>0?std::clamp(((px-ax)*dx+(py-ay)*dy)/length,0.0,1.0):0;
  double x=ax+t*dx-px,y=ay+t*dy-py; return x*x+y*y;
}
// Preserve physical clearance around measured surfaces, without rounding a hit
// to a vertex first. Block crossing edges too, even if both endpoints are free.
inline void blockObstacleDisc(Grid& grid,double originX,double originY,double px,double py,double clearance) {
  if(clearance<0) throw std::invalid_argument("Negative obstacle clearance");
  double reach=clearance+grid.resolution,r2=clearance*clearance;
  int left=std::max(0,static_cast<int>(std::floor((px-reach-originX)/grid.resolution)));
  int right=std::min(grid.width-1,static_cast<int>(std::ceil((px+reach-originX)/grid.resolution)));
  int bottom=std::max(0,static_cast<int>(std::floor((py-reach-originY)/grid.resolution)));
  int top=std::min(grid.height-1,static_cast<int>(std::ceil((py+reach-originY)/grid.resolution)));
  for(int y=bottom;y<=top;++y) for(int x=left;x<=right;++x) {
    int u=grid.id(x,y); double ax=originX+x*grid.resolution,ay=originY+y*grid.resolution;
    if(segmentDistanceSquared(ax,ay,ax,ay,px,py)<=r2) grid.cells[u].blocked=true;
    for(int dy=-1;dy<=1;++dy) for(int dx=-1;dx<=1;++dx) if((dx||dy) && grid.contains(x+dx,y+dy))
      if(segmentDistanceSquared(ax,ay,ax+dx*grid.resolution,ay+dy*grid.resolution,px,py)<=r2)
        grid.blockedMoves[u]|=static_cast<unsigned short>(1u<<((dy+1)*3+dx+1));
  }
}
}
