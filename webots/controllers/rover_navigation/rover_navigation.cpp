#include "../../../include/planner.hpp"
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include <webots/lidar.h>
#include <webots/supervisor.h>
#include <cstdio>
#include <filesystem>
#include <fstream>
using namespace rover;
int main() {
  wb_robot_init(); int dt=static_cast<int>(wb_robot_get_basic_time_step());
  const char* wheels[]={"FrontLeftWheel","MiddleLeftWheel","BackLeftWheel","FrontRightWheel","MiddleRightWheel","BackRightWheel"};
  const char* arms[]={"FrontLeftArm","FrontRightArm","BackLeftArm","BackRightArm"};
  WbDeviceTag motors[6],steer[4];
  for(int i=0;i<6;++i) { motors[i]=wb_robot_get_device(wheels[i]); wb_motor_set_position(motors[i],INFINITY); wb_motor_set_velocity(motors[i],0); }
  for(int i=0;i<4;++i) { steer[i]=wb_robot_get_device(arms[i]); wb_motor_set_position(steer[i],0); }
  auto gps=wb_robot_get_device("gps"),imu=wb_robot_get_device("imu"),lidar=wb_robot_get_device("lidar");
  if(!gps || !imu || !lidar) { std::puts("[ERROR] missing_navigation_sensor"); wb_robot_cleanup(); return 1; }
  wb_gps_enable(gps,dt); wb_inertial_unit_enable(imu,dt); wb_lidar_enable(lidar,dt);
  Grid grid(101,101,.1); const double origin=-5,pi=3.141592653589793;
  std::vector<bool> occupied(grid.cells.size(),false);
  auto node=[&](double x,double y) { int col=static_cast<int>(std::round((x-origin)/grid.resolution)),row=static_cast<int>(std::round((y-origin)/grid.resolution)); return grid.contains(col,row)?grid.id(col,row):-1; };
  auto wx=[&](int v) { return origin+(v%grid.width)*grid.resolution; };
  auto wy=[&](int v) { return origin+(v/grid.width)*grid.resolution; };
  int goal=node(3.2,0),replans=0; Result route; size_t next=0;
  double logAt=0,senseAt=2,modeSince=0; int mode=0;
  std::filesystem::create_directories("../../../results/navigation");
  std::ofstream telemetry("../../../results/navigation/state.csv"); telemetry<<"time,x,y,yaw,node,waypoint,replans\n";
  std::puts("[MISSION] localization=GPS heading=IMU map=known_flat_grid slam=disabled");
  auto stop=[&]() { for(auto motor:motors) wb_motor_set_velocity(motor,0); };
  auto finish=[&](bool success,const char* reason,double x,double y) {
    stop(); std::ofstream summary("../../../results/navigation/summary.txt");
    summary<<"status="<<(success?"goal_reached":"failed")<<"\nreason="<<reason<<"\nx="<<x<<"\ny="<<y<<"\nreplans="<<replans<<'\n'; summary.close();
    telemetry.close(); std::printf("[MISSION] status=%s reason=%s\n",success?"goal_reached":"failed",reason); std::fflush(stdout);
    wb_supervisor_simulation_quit(success?0:1);
  };
  while(wb_robot_step(dt)!=-1) {
    double t=wb_robot_get_time(); const double* position=wb_gps_get_values(gps);
    double x=position[0],y=position[1],yaw=wb_inertial_unit_get_roll_pitch_yaw(imu)[2];
    if(t<2) continue;
    int current=node(x,y); if(current<0) { finish(false,"outside_map",x,y); break; }
    bool needPlan=route.path.empty();
    if(t>=senseAt) {
      senseAt=t+.5; const float* ranges=wb_lidar_get_range_image(lidar); int beams=wb_lidar_get_horizontal_resolution(lidar); bool changed=false;
      for(int i=0;i<beams;++i) if(std::isfinite(ranges[i]) && ranges[i]>.15 && ranges[i]<1.25) {
        double angle=yaw+pi-(i+.5)*2*pi/beams;
        int hit=node(x+ranges[i]*std::cos(angle),y+ranges[i]*std::sin(angle));
        if(hit>=0 && !occupied[hit]) { occupied[hit]=true; changed=true; }
      }
      if(changed) {
        inflate(grid,occupied,.55);
        for(size_t i=next;i<route.path.size();++i) if(grid.cells[route.path[i]].blocked) needPlan=true;
      }
    }
    if(needPlan) {
      stop();
      if(grid.cells[current].blocked) { finish(false,"clearance_violation",x,y); break; }
      auto dijkstra=plan(grid,current,goal,false); route=plan(grid,current,goal,true); next=route.path.size()>1?1:0;
      ++replans;
      std::printf("[PLAN] count=%d Dijkstra_cost=%.3f Astar_cost=%.3f Dijkstra_nodes=%d Astar_nodes=%d Astar_ms=%.3f\n",replans,dijkstra.cost,route.cost,dijkstra.expanded,route.expanded,route.milliseconds); std::fflush(stdout);
      if(route.path.empty()) { finish(false,"no_path",x,y); break; }
    }
    if(std::hypot(x-wx(goal),y-wy(goal))<.12) { finish(true,"within_goal_tolerance",x,y); break; }
    while(next+1<route.path.size() && std::hypot(x-wx(route.path[next]),y-wy(route.path[next]))<.075) ++next;
    double error=std::remainder(std::atan2(wy(route.path[next])-y,wx(route.path[next])-x)-yaw,2*pi);
    // Hysteresis: finish aligning before driving, avoid repeated steer/drive toggles.
    double threshold=mode==0?.35:.04;
    int requested=std::abs(error)>threshold ? (error>0?1:-1):0;
    if(requested!=mode) { mode=requested; modeSince=t; stop(); }
    if(mode) {
      const double angles[]={-.87,.87,.87,-.87};
      for(int i=0;i<4;++i) wb_motor_set_position(steer[i],angles[i]);
      // Sojourner's sample uses opposed wheel speeds for an in-place turn.
      for(int i=0;i<6;++i) wb_motor_set_velocity(motors[i],t-modeSince>.7 ? (i<3?mode:-mode)*.4:0);
    } else {
      for(auto arm:steer) wb_motor_set_position(arm,0);
      for(auto motor:motors) wb_motor_set_velocity(motor,t-modeSince>.7?.6:0);
    }
    if(t>=logAt) {
      logAt=t+1; std::printf("[MOVE] t=%.1f xy=(%.2f,%.2f) node=%d next=%zu/%zu heading_error=%.2f mode=%d\n",t,x,y,current,next,route.path.size(),error,mode); std::fflush(stdout);
      telemetry<<t<<','<<x<<','<<y<<','<<yaw<<','<<current<<','<<next<<','<<replans<<'\n';
    }
    if(t>600) { finish(false,"timeout",x,y); break; }
  }
  wb_robot_cleanup(); return 0;
}
