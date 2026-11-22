#include "slam.hpp"
#include <filesystem>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
std::vector<std::string> fields(const std::string& line) {
  std::vector<std::string> v; std::stringstream s(line); std::string item;
  while(std::getline(s,item,',')) v.push_back(item); return v;
}
int main() {
  try {
    std::ifstream scanFile("results/webots_smoke/scans.csv"),stateFile("results/webots_smoke/state.csv");
    if(!scanFile || !stateFile) throw std::runtime_error("Run the Webots sensor world first");
    std::map<double,std::vector<double>> scans; std::string line;
    std::getline(scanFile,line);
    while(std::getline(scanFile,line)) { auto f=fields(line); if(f.size()!=3) continue;
      auto& scan=scans[std::stod(f[0])]; int beam=std::stoi(f[1]);
      if(beam<0 || beam>=360) throw std::runtime_error("Unexpected beam index");
      scan.resize(360,std::numeric_limits<double>::infinity()); scan[beam]=std::stod(f[2]);
    }
    std::filesystem::create_directories("results/slam_replay");
    std::ofstream output("results/slam_replay/trajectory.csv"); output<<"time,x,y,yaw,truth_x,truth_y,error_m,matched\n";
    rover::LocalSlam slam; double previous=0,originX=0,originY=0,lastError=0; int count=0;
    std::getline(stateFile,line);
    while(std::getline(stateFile,line)) { auto f=fields(line); if(f.size()!=7) continue;
      double t=std::stod(f[0]),odom=std::stod(f[2]); auto scan=scans.find(t); if(scan==scans.end()) continue;
      if(count==0) { originX=std::stod(f[3]); originY=std::stod(f[4]); }
      auto result=slam.update(odom-previous,0,scan->second); previous=odom; ++count;
      // Ground truth enters only this evaluation, never LocalSlam::update.
      double tx=std::stod(f[3])-originX,ty=std::stod(f[4])-originY;
      lastError=std::hypot(result.pose.x-tx,result.pose.y-ty);
      output<<t<<','<<result.pose.x<<','<<result.pose.y<<','<<result.pose.yaw<<','<<tx<<','<<ty<<','<<lastError<<','<<result.matched<<'\n';
      std::cout<<"[SLAM] t="<<t<<" x="<<result.pose.x<<" y="<<result.pose.y<<" yaw="<<result.pose.yaw<<" occupied="<<result.occupied<<" matched="<<result.matched<<" error_m="<<lastError<<'\n';
    }
    slam.save("results/slam_replay/map.csv");
    if(count<2) throw std::runtime_error("Insufficient synchronized frames");
    std::cout<<"[SLAM] replay_complete frames="<<count<<" final_error_m="<<lastError<<" loop_closure=false\n";
    return 0;
  } catch(const std::exception& e) { std::cerr<<"[ERROR] "<<e.what()<<'\n'; return 1; }
}
