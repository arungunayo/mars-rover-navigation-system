#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/lidar.h>
#include <webots/supervisor.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>

// Stage 2: sensor and straight-motion verification. No ground-truth feedback.
int main() {
  wb_robot_init();
  const int step=static_cast<int>(wb_robot_get_basic_time_step());
  const char* names[]={"FrontLeftWheel","MiddleLeftWheel","BackLeftWheel","FrontRightWheel","MiddleRightWheel","BackRightWheel"};
  WbDeviceTag motors[6],encoders[6];
  for(int i=0;i<6;++i) {
    motors[i]=wb_robot_get_device(names[i]);
    if(!motors[i]) { std::printf("[ERROR] missing_motor=%s\n",names[i]); wb_robot_cleanup(); return 1; }
    encoders[i]=wb_motor_get_position_sensor(motors[i]);
    if(!encoders[i]) { std::printf("[ERROR] missing_encoder=%s\n",names[i]); wb_robot_cleanup(); return 1; }
    wb_position_sensor_enable(encoders[i],step);
    wb_motor_set_position(motors[i],INFINITY); wb_motor_set_velocity(motors[i],0);
  }
  WbDeviceTag lidar=wb_robot_get_device("lidar");
  if(!lidar) { std::puts("[ERROR] missing_lidar"); wb_robot_cleanup(); return 1; }
  wb_lidar_enable(lidar,step);
  WbNodeRef self=wb_supervisor_node_get_self();
  std::filesystem::path output=std::filesystem::path("../../../results/webots_smoke");
  std::filesystem::create_directories(output);
  std::ofstream state(output/"state.csv"),scans(output/"scans.csv");
  state<<"time,encoder_mean_rad,odom_distance_m,truth_x,truth_y,min_range,command_rad_s\n";
  scans<<"time,beam,range_m\n";
  double baseline=0,originX=0,originY=0,initialEncoder=0,lastEncoder=0;
  int validScans=0; bool initialized=false; double nextLog=0;
  std::puts("[MISSION] stage=sensor_smoke slam=not_connected ground_truth=evaluation_only");
  std::fflush(stdout);
  while(wb_robot_step(step)!=-1) {
    double t=wb_robot_get_time(),angle=0;
    for(auto e:encoders) angle+=wb_position_sensor_get_value(e)/6;
    const double* truth=wb_supervisor_node_get_position(self);
    if(!initialized) { baseline=initialEncoder=angle; originX=truth[0]; originY=truth[1]; initialized=true; }
    lastEncoder=angle;
    const float* ranges=wb_lidar_get_range_image(lidar);
    double nearest=std::numeric_limits<double>::infinity();
    int beams=wb_lidar_get_horizontal_resolution(lidar);
    for(int i=0;i<beams;++i) if(std::isfinite(ranges[i]) && ranges[i]>0) nearest=std::min(nearest,double(ranges[i]));
    if(std::isfinite(nearest)) ++validScans;
    // Settle first, then low-speed straight drive, then stop and evaluate.
    double command=t>=2 && t<18 ? 0.6:0;
    for(auto m:motors) wb_motor_set_velocity(m,command);
    if(t>=nextLog) {
      nextLog=t+0.5;
      std::printf("[SENSOR] t=%.2f encoder_rad=%.3f odom_distance_m=%.3f lidar_min_m=%.3f command=%.1f\n",t,angle,(angle-baseline)*0.06,nearest,command);
      std::fflush(stdout);
      state<<t<<','<<angle<<','<<(angle-baseline)*0.06<<','<<truth[0]<<','<<truth[1]<<','<<nearest<<','<<command<<'\n';
      for(int i=0;i<beams;++i) scans<<t<<','<<i<<','<<ranges[i]<<'\n';
    }
    if(t>=20) {
      double displacement=std::hypot(truth[0]-originX,truth[1]-originY);
      bool pass=std::abs(lastEncoder-initialEncoder)>1 && displacement>0.1 && validScans>10;
      std::ofstream summary(output/"summary.txt");
      summary<<"status="<<(pass?"passed":"failed")<<"\ndisplacement_m="<<displacement
        <<"\nodom_distance_m="<<(angle-baseline)*0.06<<"\nvalid_scans="<<validScans
        <<"\nfinal_nearest_range_m="<<nearest<<'\n';
      summary.close();
      std::printf("[CHECK] status=%s displacement_m=%.3f valid_scans=%d\n",pass?"passed":"failed",displacement,validScans);
      std::fflush(stdout); state.close(); scans.close();
      wb_supervisor_simulation_quit(pass?0:1); break;
    }
  }
  wb_robot_cleanup(); return 0;
}
