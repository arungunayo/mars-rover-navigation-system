#pragma once
#include "planner.hpp"
namespace rover {
inline void deriveRoughness(Grid& grid) {
  std::vector<double> values(grid.cells.size(),0);
  for(int y=1;y<grid.height-1;++y) for(int x=1;x<grid.width-1;++x) {
    double center=grid.cells[grid.id(x,y)].elevation;
    // Curvature residual: a planar slope has zero roughness.
    double mean=(grid.cells[grid.id(x-1,y)].elevation+grid.cells[grid.id(x+1,y)].elevation+
      grid.cells[grid.id(x,y-1)].elevation+grid.cells[grid.id(x,y+1)].elevation)/4;
    values[grid.id(x,y)]=std::abs(center-mean)/grid.resolution;
  }
  for(size_t i=0;i<values.size();++i) grid.cells[i].roughness=values[i];
}
}
