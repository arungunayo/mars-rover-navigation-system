#pragma once
#include "slam.hpp"
inline void testSlam() {
  rover::LocalSlam map;
  std::vector<double> scan(360,std::numeric_limits<double>::infinity());
  // Synthetic room with nonparallel walls; sensor ray intersections are analytic.
  auto room=[](double x,double y,double yaw) {
    std::vector<double> r(360,6);
    for(int i=0;i<360;++i) {
      double a=yaw+rover::LocalSlam::angle(i,360),c=std::cos(a),s=std::sin(a);
      double tx=(c>0?2-x:-2-x)/c,ty=(s>0?1.5-y:-1.5-y)/s;
      r[i]=std::min(tx,ty);
    }
    return r;
  };
  auto first=map.update(0,0,room(0,0,0));
  check(first.occupied>100,"SLAM map has walls");
  check(map.evidence(0,0)<0,"SLAM carves free space");
  auto corrected=map.update(.14,0,room(.10,0,0));
  check(corrected.matched,"SLAM matches room");
  check(std::abs(corrected.pose.x-.10)<.035,"SLAM reduces odometry error");
  check(std::hypot(corrected.pose.x-.1,corrected.pose.y)<.04,"SLAM translation accuracy");
  rover::LocalSlam empty; auto unknown=empty.update(0,0,scan);
  check(unknown.occupied==0,"No-return rays do not create obstacles");
  check(!unknown.matched,"Unobserved map has no match");
}
