#pragma once
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace rover {
struct Pose2D { double x=0,y=0,yaw=0; };
struct SlamUpdate { Pose2D pose; int hits=0, occupied=0; bool matched=false; double score=0; };
// Local 2D occupancy-grid SLAM: odometry prediction, correlative scan matching,
// then inverse-sensor mapping. No loop closure or global relocalization.
class LocalSlam {
 public:
  static constexpr double pi=3.14159265358979323846;
  const int width=240,height=240;
  const double resolution=0.05,origin=-6;
  std::vector<float> odds;
  Pose2D pose;
  LocalSlam():odds(width*height,0) {}
  int cell(double x,double y) const {
    int col=static_cast<int>(std::floor((x-origin)/resolution));
    int row=static_cast<int>(std::floor((y-origin)/resolution));
    return col>=0 && row>=0 && col<width && row<height ? row*width+col:-1;
  }
  static double angle(int i,int count) { return pi-(i+0.5)*2*pi/count; }
  float evidence(double x,double y) const { int id=cell(x,y); return id<0?0:odds[id]; }
  SlamUpdate update(double travel,double turn,const std::vector<double>& ranges,double maxRange=6) {
    if(ranges.empty() || !std::isfinite(travel) || !std::isfinite(turn)) throw std::invalid_argument("Invalid SLAM input");
    Pose2D predicted{pose.x+travel*std::cos(pose.yaw+turn/2),pose.y+travel*std::sin(pose.yaw+turn/2),std::remainder(pose.yaw+turn,2*pi)};
    std::vector<Pose2D> points;
    for(int i=0;i<static_cast<int>(ranges.size());++i) if(std::isfinite(ranges[i]) && ranges[i]>=0.1 && ranges[i]<maxRange) {
      double a=angle(i,static_cast<int>(ranges.size())); points.push_back({ranges[i]*std::cos(a),ranges[i]*std::sin(a),0});
    }
    SlamUpdate result; result.hits=static_cast<int>(points.size());
    int occupied=0; for(float v:odds) occupied+=v>0.6f;
    pose=predicted;
    if(occupied>5 && points.size()>=8) {
      auto score=[&](const Pose2D& candidate) {
        double total=0,c=std::cos(candidate.yaw),s=std::sin(candidate.yaw);
        for(const auto& p:points) {
          double x=candidate.x+c*p.x-s*p.y,y=candidate.y+s*p.x+c*p.y;
          int id=cell(x,y); double best=0;
          if(id>=0) for(int dy=-2;dy<=2;++dy) for(int dx=-2;dx<=2;++dx) {
            int col=id%width+dx,row=id/width+dy;
            if(col>=0 && row>=0 && col<width && row<height && odds[row*width+col]>0.6f)
              best=std::max(best,std::exp(-0.5*(dx*dx+dy*dy)));
          }
          total+=best;
        }
        return total/points.size()-0.15*(std::pow((candidate.x-predicted.x)/0.1,2)+std::pow((candidate.y-predicted.y)/0.1,2)+std::pow((candidate.yaw-predicted.yaw)/0.1,2));
      };
      double best=score(predicted); result.score=best;
      for(int da=-2;da<=2;++da) for(int dy=-2;dy<=2;++dy) for(int dx=-2;dx<=2;++dx) {
        Pose2D candidate{predicted.x+dx*0.02,predicted.y+dy*0.02,predicted.yaw+da*0.01};
        double value=score(candidate);
        if(value>best+1e-9) { best=value; pose=candidate; }
      }
      result.matched=best>0.2; result.score=best;
      if(!result.matched) pose=predicted;
    }
    // Aggregate each cell once per scan: a hit wins over a crossing free ray.
    std::vector<unsigned char> updates(odds.size(),0);
    for(int i=0;i<static_cast<int>(ranges.size());++i) {
      double r=ranges[i]; if(std::isnan(r) || r<0.1) continue;
      bool hit=std::isfinite(r) && r<maxRange;
      r=hit?r:maxRange;
      double a=pose.yaw+angle(i,static_cast<int>(ranges.size())),c=std::cos(a),s=std::sin(a);
      for(double distance=0;distance<r-resolution/2;distance+=resolution/2) {
        int id=cell(pose.x+c*distance,pose.y+s*distance); if(id>=0 && updates[id]==0) updates[id]=1;
      }
      if(hit) { int id=cell(pose.x+c*r,pose.y+s*r); if(id>=0) updates[id]=2; }
    }
    for(size_t i=0;i<odds.size();++i) {
      if(updates[i]) odds[i]=std::clamp(odds[i]+(updates[i]==2?0.85f:-0.4f),-4.f,4.f);
    }
    result.pose=pose;
    for(float v:odds) result.occupied+=v>0.6f;
    return result;
  }
  void save(const char* filename) const {
    std::ofstream out(filename); out<<"x,y,log_odds,state\n";
    for(int y=0;y<height;++y) for(int x=0;x<width;++x) {
      float v=odds[y*width+x];
      out<<origin+(x+0.5)*resolution<<','<<origin+(y+0.5)*resolution<<','<<v<<','<<(v>0.6?"occupied":v<-.2?"free":"unknown")<<'\n';
    }
  }
};
}
