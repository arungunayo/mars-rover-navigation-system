#include "../../../include/planner.hpp"
#include "../../../include/terrain.hpp"
#include "../../../include/discovery.hpp"
#include "../../../include/obstacles.hpp"
#include <webots/display.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include <webots/lidar.h>
#include <webots/supervisor.h>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
using namespace rover;
int main() {
  wb_robot_init(); int dt=static_cast<int>(wb_robot_get_basic_time_step());
  const std::string configuration=wb_robot_get_custom_data();
  double slopeLimit=.3;
  auto slopeSetting=configuration.find("maxGrade=");
  if(slopeSetting!=std::string::npos) {
    std::istringstream value(configuration.substr(slopeSetting+9)); value>>slopeLimit;
    if(!value || !std::isfinite(slopeLimit) || slopeLimit<=0 || slopeLimit>.6) {
      std::puts("[ERROR] invalid_maxGrade_configuration"); wb_robot_cleanup(); return 1;
    }
  }
  wb_supervisor_set_label(0,"WHITE: grid | BLUE: discovered | ORANGE: sensing circle | CYAN: local route",.02,.02,.035,0xFFFFFF,0,"Arial");
  auto oldRays=wb_supervisor_node_get_from_def("SENSOR_RAYS"); if(oldRays) wb_supervisor_node_remove(oldRays);
  const char* wheels[]={"FrontLeftWheel","MiddleLeftWheel","BackLeftWheel","FrontRightWheel","MiddleRightWheel","BackRightWheel"};
  const char* arms[]={"FrontLeftArm","FrontRightArm","BackLeftArm","BackRightArm"};
  WbDeviceTag motors[6],steer[4];
  for(int i=0;i<6;++i) { motors[i]=wb_robot_get_device(wheels[i]); wb_motor_set_position(motors[i],INFINITY); wb_motor_set_velocity(motors[i],0); }
  for(int i=0;i<4;++i) { steer[i]=wb_robot_get_device(arms[i]); wb_motor_set_position(steer[i],0); }
  const double cruiseSpeed=std::min(6.0,wb_motor_get_max_velocity(motors[0]));
  const double turnSpeed=std::min(2.0,wb_motor_get_max_velocity(motors[0]));
  auto gps=wb_robot_get_device("gps"),imu=wb_robot_get_device("imu"),lidar=wb_robot_get_device("lidar");
  if(!gps || !imu || !lidar) { std::puts("[ERROR] missing_navigation_sensor"); wb_robot_cleanup(); return 1; }
  wb_gps_enable(gps,dt); wb_inertial_unit_enable(imu,dt);
  // Depth rendering every physics tick was slowing the whole simulation.
  wb_lidar_enable(lidar,4*dt);
  auto lidarNode=wb_supervisor_node_get_from_device(lidar);
  // Debug graphics are for the user, never physical sensor obstacles.
  // Limit visibility overrides to annotations; the LIDAR is itself a child
  // of the rover, so do not hide the entire rover subtree from its sensor.
  for(const char* name:{"GRID_OVERLAY","START","GOAL"}) {
    auto visual=wb_supervisor_node_get_from_def(name); if(visual) wb_supervisor_node_set_visibility(visual,lidarNode,false);
  }
  auto radar=wb_robot_get_device("lidar_view"),mapView=wb_robot_get_device("discovery_view");
  if(mapView) wb_display_set_font(mapView,"Arial",8,false);
  auto terrain=wb_supervisor_node_get_from_def("TERRAIN_HEIGHTS");
  int width=101,height=101; double spacing=.1,originX=-5,originY=-5,baseZ=0;
  if(terrain) {
    width=wb_supervisor_field_get_sf_int32(wb_supervisor_node_get_field(terrain,"xDimension"));
    height=wb_supervisor_field_get_sf_int32(wb_supervisor_node_get_field(terrain,"yDimension"));
    spacing=wb_supervisor_field_get_sf_float(wb_supervisor_node_get_field(terrain,"xSpacing"));
    double ySpacing=wb_supervisor_field_get_sf_float(wb_supervisor_node_get_field(terrain,"ySpacing"));
    auto ground=wb_supervisor_node_get_from_def("TERRAIN");
    if(!ground || std::abs(spacing-ySpacing)>1e-8) { std::puts("[ERROR] terrain_requires_square_spacing"); wb_robot_cleanup(); return 1; }
    const double* offset=wb_supervisor_field_get_sf_vec3f(wb_supervisor_node_get_field(ground,"translation"));
    originX=offset[0]; originY=offset[1]; baseZ=offset[2];
  }
  Grid grid(width,height,spacing); const double pi=3.141592653589793;
  if(terrain) {
    auto heights=wb_supervisor_node_get_field(terrain,"height");
    if(wb_supervisor_field_get_count(heights)!=width*height) { std::puts("[ERROR] incomplete_height_array"); wb_robot_cleanup(); return 1; }
    for(int i=0;i<width*height;++i) grid.cells[i].elevation=baseZ+wb_supervisor_field_get_mf_float(heights,i);
    deriveRoughness(grid);
  }
  std::vector<bool> occupied(grid.cells.size(),false);
  std::vector<bool> boundary(grid.cells.size(),false);
  std::vector<ObstaclePoint> obstacleHits;
  std::vector<bool> terrainUnsafe(grid.cells.size(),false);
  for(int row=1;row<height-1;++row) for(int col=1;col<width-1;++col) {
    double gx=(grid.cells[grid.id(col+1,row)].elevation-grid.cells[grid.id(col-1,row)].elevation)/(2*spacing);
    double gy=(grid.cells[grid.id(col,row+1)].elevation-grid.cells[grid.id(col,row-1)].elevation)/(2*spacing);
    terrainUnsafe[grid.id(col,row)]=std::hypot(gx,gy)>slopeLimit;
  }
  auto applyClearance=[&]() {
    inflate(grid,boundary,.65);
    std::fill(grid.blockedMoves.begin(),grid.blockedMoves.end(),static_cast<unsigned short>(0));
    for(size_t i=0;i<terrainUnsafe.size();++i) grid.cells[i].blocked=grid.cells[i].blocked || terrainUnsafe[i];
    for(const auto& hit:obstacleHits) blockObstacleDisc(grid,originX,originY,hit.x,hit.y,.65);
  };
  std::vector<bool> seen(grid.cells.size(),false);
  std::vector<int> visits(grid.cells.size(),0);
  ExplorationMemory exploration(grid.cells.size());
  Costs costs; costs.maxSlope=slopeLimit;
  auto node=[&](double x,double y) { int col=static_cast<int>(std::round((x-originX)/grid.resolution)),row=static_cast<int>(std::round((y-originY)/grid.resolution)); return grid.contains(col,row)?grid.id(col,row):-1; };
  auto wx=[&](int v) { return originX+(v%grid.width)*grid.resolution; };
  auto wy=[&](int v) { return originY+(v/grid.width)*grid.resolution; };
  auto goalMarker=wb_supervisor_node_get_from_def("GOAL");
  double goalX=3.2,goalY=0;
  if(goalMarker) { const double* p=wb_supervisor_field_get_sf_vec3f(wb_supervisor_node_get_field(goalMarker,"translation")); goalX=p[0]; goalY=p[1]; }
  int goal=node(goalX,goalY),localGoal=-1,replans=0; Result route; size_t next=0;
  if(goal<0) { std::puts("[ERROR] goal_outside_map"); wb_robot_cleanup(); return 1; }
  for(int row=0;row<height;++row) for(int col=0;col<width;++col)
    if(col*spacing<.5 || row*spacing<.5 || (width-1-col)*spacing<.5 || (height-1-row)*spacing<.5) occupied[grid.id(col,row)]=true;
  boundary=occupied;
  applyClearance();
  double logAt=0,senseAt=2,modeSince=0; int mode=0;
  constexpr double frontierTolerance=.25;
  int previousNode=-1;
  std::filesystem::path logRoot=configuration.find("validation")!=std::string::npos?"../../../results/explorer_validation":"../../../results/explorer";
  std::filesystem::create_directories(logRoot);
  { std::ofstream status(logRoot/"summary.txt"); status<<"status=running\n"; }
  std::ofstream telemetry(logRoot/"state.csv"); telemetry<<"time,x,y,yaw,node,waypoint,replans,z,ground_z\n";
  std::ofstream terrainCsv(logRoot/"terrain.csv"); terrainCsv<<"node,x,y,elevation,roughness\n";
  for(int i=0;i<width*height;++i) terrainCsv<<i<<','<<wx(i)<<','<<wy(i)<<','<<grid.cells[i].elevation<<','<<grid.cells[i].roughness<<'\n';
  terrainCsv.close();
  std::ofstream metrics(logRoot/"plans.csv"); metrics<<"plan,algorithm,cost,expanded,runtime_ms,target,goal_known\n";
  std::printf("[MISSION] localization=GPS heading=IMU map=%s nodes=%d goal=(%.2f,%.2f) max_slope_grade=%.2f\n",terrain?"ElevationGrid":"flat",width*height,goalX,goalY,slopeLimit);
  auto stop=[&]() { for(auto motor:motors) wb_motor_set_velocity(motor,0); };
  auto finish=[&](bool success,const char* reason,double x,double y) {
    stop(); std::ofstream summary(logRoot/"summary.txt");
    summary<<"status="<<(success?"goal_reached":"failed")<<"\nreason="<<reason<<"\nx="<<x<<"\ny="<<y<<"\nreplans="<<replans<<'\n'; summary.close();
    telemetry.close(); metrics.close(); std::printf("[MISSION] status=%s reason=%s\n",success?"goal_reached":"failed",reason); std::fflush(stdout);
    wb_supervisor_simulation_quit(success?0:1);
  };
  while(wb_robot_step(dt)!=-1) {
    double t=wb_robot_get_time(); const double* position=wb_gps_get_values(gps);
    const double* attitude=wb_inertial_unit_get_roll_pitch_yaw(imu);
    double x=position[0],y=position[1],yaw=attitude[2],roll=attitude[0],pitch=attitude[1];
    if(t<2) continue;
    int current=node(x,y); if(current<0) { finish(false,"outside_map",x,y); break; }
    if(current!=previousNode) { ++visits[current]; previousNode=current; }
    bool needPlan=route.path.empty() || (localGoal>=0 && localGoal!=goal && std::hypot(x-wx(localGoal),y-wy(localGoal))<frontierTolerance);
    if(needPlan) { stop(); senseAt=0; }
    if(t>=senseAt) {
      senseAt=t+.25; int beams=wb_lidar_get_horizontal_resolution(lidar); bool changed=false;
      const float* sensorRanges=wb_lidar_get_range_image(lidar);
      const std::vector<float> scan(sensorRanges,sensorRanges+beams);
      const float* ranges=scan.data();
      // Read the sensor's actual global transform. This also handles any
      // mounting/physics rotation independently of the rover heading controller.
      const double* rawSensorPosition=wb_supervisor_node_get_position(lidarNode);
      double sensorX=rawSensorPosition[0],sensorY=rawSensorPosition[1],sensorZ=rawSensorPosition[2];
      const double* rawSensorRotation=wb_supervisor_node_get_orientation(lidarNode);
      double rotation[9]; std::copy(rawSensorRotation,rawSensorRotation+9,rotation);
      double sensorYaw=std::atan2(rotation[3],rotation[0]);
      int discovered=0; const double sensingRadius=2.6;
      for(int v=0;v<width*height;++v) {
        double distance=std::hypot(wx(v)-x,wy(v)-y);
        if(distance>sensingRadius) continue;
        bool visible=true;
        // Known simulator heights provide local depth sensing, including terrain occlusion.
        for(double d=spacing/2;d<distance-spacing/2;d+=spacing/2) {
          int sample=node(x+(wx(v)-x)*d/distance,y+(wy(v)-y)*d/distance);
          double rayZ=position[2]+.8+(grid.cells[v].elevation-position[2]-.8)*d/distance;
          if(sample>=0 && grid.cells[sample].elevation>rayZ+.08) { visible=false; break; }
        }
        double angle=std::remainder(std::atan2(wy(v)-sensorY,wx(v)-sensorX)-sensorYaw,2*pi);
        int beam=std::clamp(static_cast<int>((pi-angle)*beams/(2*pi)),0,beams-1);
        if(std::isfinite(ranges[beam]) && ranges[beam]<distance-.4) visible=false;
        if(visible && !seen[v]) { seen[v]=true; ++discovered; }
      }
      seen[current]=true;
      if(localGoal>=0 && localGoal!=goal && !route.path.empty() && next>=route.path.size()-1 && std::hypot(x-wx(localGoal),y-wy(localGoal))<frontierTolerance) needPlan=true;
      if(discovered) std::printf("[DISCOVERY] t=%.1f new_nodes=%d goal_known=%s\n",t,discovered,seen[goal]?"true":"false");
      if(radar) {
        int w=wb_display_get_width(radar),h=wb_display_get_height(radar),cx=w/2,cy=h/2+2;
        double scale=std::min(w,h)*.1;
        wb_display_set_color(radar,0x071421); wb_display_fill_rectangle(radar,0,0,w,h);
        wb_display_set_font(radar,"Arial",10,false);
        wb_display_set_color(radar,0x286070);
        for(int metre=1;metre<=4;++metre) wb_display_draw_oval(radar,cx,cy,static_cast<int>(metre*scale),static_cast<int>(metre*scale));
        wb_display_set_color(radar,0xFF674D);
        for(int i=0;i<beams;++i) if(std::isfinite(ranges[i]) && ranges[i]>=.1 && ranges[i]<=4) {
          double a=pi-(i+.5)*2*pi/beams;
          int px=cx-static_cast<int>(ranges[i]*scale*std::sin(a)),py=cy-static_cast<int>(ranges[i]*scale*std::cos(a));
          wb_display_fill_oval(radar,px,py,2,2);
        }
        wb_display_set_color(radar,0xFFFFFF); wb_display_draw_text(radar,"LIVE LIDAR | FRONT UP",7,6);
        wb_display_draw_text(radar,"Returns = red | rings = 1 m",7,h-14);
        wb_display_set_color(radar,0x00FFFF); wb_display_fill_oval(radar,cx,cy,4,4);
      }
      if(mapView) {
        wb_display_set_color(mapView,0x071421); wb_display_fill_rectangle(mapView,0,0,320,320);
        for(int dy=-4;dy<=4;++dy) for(int dx=-4;dx<=4;++dx) {
          int col=current%width+dx,row=current/width+dy; if(!grid.contains(col,row)) continue;
          int v=grid.id(col,row),sx=(dx+4)*34,sy2=(4-dy)*32+25;
          wb_display_set_color(mapView,!seen[v]?0x202833:grid.cells[v].blocked?0xA03232:0x155D80);
          wb_display_fill_rectangle(mapView,sx,sy2,33,31);
          if(seen[v]) {
            double localSlope=0;
            for(int ny=-1;ny<=1;++ny) for(int nx=-1;nx<=1;++nx) if((nx||ny) && grid.contains(col+nx,row+ny)) {
              int neighbor=grid.id(col+nx,row+ny);
              if(seen[neighbor]) localSlope=std::max(localSlope,std::abs(grid.cells[v].elevation-grid.cells[neighbor].elevation)/(spacing*std::hypot(nx,ny)));
            }
            char label[32]; std::snprintf(label,sizeof(label),"%.1fm",grid.cells[v].elevation);
            wb_display_set_color(mapView,0xFFFFFF); wb_display_draw_text(mapView,label,sx+1,sy2+2);
            std::snprintf(label,sizeof(label),"w%.1f",1+4*localSlope+2*grid.cells[v].roughness); wb_display_draw_text(mapView,label,sx+1,sy2+14);
          }
          if(v==goal || v==current) { wb_display_set_color(mapView,v==goal?0xFF3333:0x00FF66); wb_display_fill_oval(mapView,sx+16,sy2+23,4,4); }
        }
        wb_display_set_color(mapView,0xFFFFFF); wb_display_draw_text(mapView,"DISCOVERED HEIGHT GRID | m",5,5);
      }
      auto replaceGeometry=[&](const char* name,const std::string& geometry) {
        auto old=wb_supervisor_node_get_from_def(name); if(old) wb_supervisor_node_remove(old);
        auto children=wb_supervisor_node_get_field(wb_supervisor_node_get_root(),"children");
        wb_supervisor_field_import_mf_node_from_string(children,-1,geometry.c_str());
        auto visual=wb_supervisor_node_get_from_def(name); if(visual) wb_supervisor_node_set_visibility(visual,lidarNode,false);
      };
      std::ostringstream footprint; footprint<<"DEF SENSOR_FOOTPRINT Shape { appearance Appearance { material Material { emissiveColor 1 .6 .1 } } geometry IndexedLineSet { coord Coordinate { point [ ";
      for(int i=0;i<=64;++i) { double a=i*2*pi/64,fx=x+sensingRadius*std::cos(a),fy=y+sensingRadius*std::sin(a); int v=node(fx,fy); footprint<<fx<<' '<<fy<<' '<<(v>=0?grid.cells[v].elevation+.1:position[2])<<", "; }
      footprint<<"] } coordIndex [ "; for(int i=0;i<=64;++i) footprint<<i<<' '; footprint<<"-1 ] } }";
      replaceGeometry("SENSOR_FOOTPRINT",footprint.str());
      std::ostringstream revealed; revealed<<"DEF DISCOVERED_NODES Shape { appearance Appearance { material Material { emissiveColor .1 .65 1 } } geometry PointSet { coord Coordinate { point [ ";
      for(int v=0;v<width*height;++v) if(seen[v]) revealed<<wx(v)<<' '<<wy(v)<<' '<<grid.cells[v].elevation+.1<<", ";
      revealed<<"] } } }"; replaceGeometry("DISCOVERED_NODES",revealed.str());
      for(int i=0;i<beams;++i) if(std::isfinite(ranges[i]) && ranges[i]>.4 && ranges[i]<2.6) {
        // Single-layer sensor: reconstruct endpoints directly from its range
        // image and documented beam angles, then apply the current rover pose.
        double beamAngle=pi-(i+.5)*2*pi/beams;
        double px=ranges[i]*std::cos(beamAngle),py=ranges[i]*std::sin(beamAngle);
        double hx=sensorX+rotation[0]*px+rotation[1]*py;
        double hy=sensorY+rotation[3]*px+rotation[4]*py;
        double hz=sensorZ+rotation[6]*px+rotation[7]*py;
        int hit=node(hx,hy);
        // The coarse original DEM has steep triangles: nearest-vertex height
        // alone misclassifies ordinary terrain returns as above-ground objects.
        double upperGround=hit>=0?grid.cells[hit].elevation:0;
        if(hit>=0) for(int oy=-1;oy<=1;++oy) for(int ox=-1;ox<=1;++ox) {
          int col=hit%width+ox,row=hit/width+oy;
          if(grid.contains(col,row)) upperGround=std::max(upperGround,grid.cells[grid.id(col,row)].elevation);
        }
        if(hit<0 || hz<upperGround+.18) continue;
        bool duplicate=false;
        for(const auto& previous:obstacleHits) if(std::hypot(previous.x-hx,previous.y-hy)<.12) { duplicate=true; break; }
        if(!duplicate) {
          obstacleHits.push_back({hx,hy}); occupied[hit]=true; changed=true;
          std::printf("[OBSTACLE] t=%.1f node=%d xy=(%.2f,%.2f)\n",t,hit,hx,hy);
        }
      }
      if(changed) {
        applyClearance();
        if(grid.cells[current].blocked) needPlan=true;
        for(size_t i=next;i<route.path.size();++i) if(grid.cells[route.path[i]].blocked) needPlan=true;
        for(size_t i=next?next-1:0;i+1<route.path.size();++i) {
          int a=route.path[i],b=route.path[i+1],dx=b%width-a%width,dy=b/width-a/width;
          if(grid.blockedMoves[a] & (1u<<((dy+1)*3+dx+1))) needPlan=true;
        }
      }
    }
    if(needPlan) {
      stop();
      if(grid.cells[current].blocked) {
        bool physicallyBlocked=boundary[current] || terrainUnsafe[current];
        for(const auto& hit:obstacleHits) if(std::hypot(hit.x-x,hit.y-y)<.42) physicallyBlocked=true;
        if(physicallyBlocked) { finish(false,"unsafe_current_cell",x,y); break; }
        // A newly observed rock can expand its conservative clearance mask
        // over the cell the rover already occupies. Keep the measured-free
        // start cell available so it can take a free adjacent escape edge.
        grid.cells[current].blocked=false;
        grid.blockedMoves[current]=0;
        for(int dy=-1;dy<=1;++dy) for(int dx=-1;dx<=1;++dx) if((dx||dy) && grid.contains(current%width+dx,current/width+dy)) {
          int v=grid.id(current%width+dx,current/width+dy);
          for(const auto& hit:obstacleHits) if(segmentDistanceSquared(x,y,wx(v),wy(v),hit.x,hit.y)<.45*.45)
            grid.blockedMoves[current]|=static_cast<unsigned short>(1u<<((dy+1)*3+dx+1));
        }
        std::printf("[RECOVER] current_cell_in_clearance_margin node=%d replanning_to_free_neighbor\n",current);
      }
      if(localGoal>=0 && std::hypot(x-wx(localGoal),y-wy(localGoal))<frontierTolerance) ++visits[localGoal];
      auto known=discoveredGrid(grid,seen);
      if(localGoal>=0 && localGoal!=goal && std::hypot(x-wx(localGoal),y-wy(localGoal))<frontierTolerance)
        exploration.record(known,seen,localGoal);
      localGoal=chooseLocalTarget(known,seen,current,goal,visits,costs,&exploration);
      if(localGoal<0) { finish(false,"exploration_exhausted_no_route",x,y); break; }
      auto dijkstra=plan(known,current,localGoal,false,costs); route=plan(known,current,localGoal,true,costs); next=route.path.size()>1?1:0;
      ++replans;
      metrics<<replans<<",Dijkstra,"<<dijkstra.cost<<','<<dijkstra.expanded<<','<<dijkstra.milliseconds<<','<<localGoal<<','<<seen[goal]<<'\n';
      metrics<<replans<<",A*,"<<route.cost<<','<<route.expanded<<','<<route.milliseconds<<','<<localGoal<<','<<seen[goal]<<'\n'; metrics.flush();
      std::printf("[PLAN] count=%d scope=%s target=%d Dijkstra_cost=%.3f Astar_cost=%.3f Dijkstra_nodes=%d Astar_nodes=%d Astar_ms=%.3f\n",replans,localGoal==goal?"visible_goal":"local_discovered_frontier",localGoal,dijkstra.cost,route.cost,dijkstra.expanded,route.expanded,route.milliseconds); std::fflush(stdout);
      if(route.path.empty()) { finish(false,"no_path",x,y); break; }
      if(terrain && route.path.size()>1) {
        auto old=wb_supervisor_node_get_from_def("PLANNED_PATH"); if(old) wb_supervisor_node_remove(old);
        std::ostringstream shape;
        shape<<"DEF PLANNED_PATH Shape { appearance Appearance { material Material { emissiveColor 0 1 1 } } geometry IndexedLineSet { coord Coordinate { point [ ";
        for(int v:route.path) shape<<wx(v)<<' '<<wy(v)<<' '<<grid.cells[v].elevation+.025<<", ";
        shape<<"] } coordIndex [ "; for(size_t i=0;i<route.path.size();++i) shape<<i<<' ';
        shape<<"-1 ] } }";
        auto children=wb_supervisor_node_get_field(wb_supervisor_node_get_root(),"children");
        wb_supervisor_field_import_mf_node_from_string(children,-1,shape.str().c_str());
        auto visual=wb_supervisor_node_get_from_def("PLANNED_PATH"); if(visual) wb_supervisor_node_set_visibility(visual,lidarNode,false);
      }
    }
    if(std::hypot(x-goalX,y-goalY)<.2) { finish(true,"within_goal_tolerance",x,y); break; }
    while(next+1<route.path.size() && std::hypot(x-wx(route.path[next]),y-wy(route.path[next]))<.2) ++next;
    double error=std::remainder(std::atan2(wy(route.path[next])-y,wx(route.path[next])-x)-yaw,2*pi);
    // Hysteresis: finish aligning before driving, avoid repeated steer/drive toggles.
    double threshold=mode==0?.35:.04;
    int requested=std::abs(error)>threshold ? (error>0?1:-1):0;
    if(requested!=mode) { mode=requested; modeSince=t; stop(); }
    if(mode) {
      const double angles[]={-.87,.87,.87,-.87};
      for(int i=0;i<4;++i) wb_motor_set_position(steer[i],angles[i]);
      // Sojourner's sample uses opposed wheel speeds for an in-place turn.
      double safeTurnSpeed=std::max(std::abs(pitch),std::abs(roll))>.15?std::min(turnSpeed,1.0):turnSpeed;
      for(int i=0;i<6;++i) wb_motor_set_velocity(motors[i],t-modeSince>.35 ? (i<3?mode:-mode)*safeTurnSpeed:0);
    } else {
      for(auto arm:steer) wb_motor_set_position(arm,0);
      double speed=cruiseSpeed;
      if(std::max(std::abs(pitch),std::abs(roll))>.22) speed=std::min(speed,2.0);
      if(std::abs(error)>.2) speed=std::min(speed,3.5);
      for(auto motor:motors) wb_motor_set_velocity(motor,t-modeSince>.35?speed:0);
    }
    if(t>=logAt) {
      logAt=t+1; std::printf("[MOVE] t=%.1f xy=(%.2f,%.2f) node=%d next=%zu/%zu heading_error=%.2f mode=%d\n",t,x,y,current,next,route.path.size(),error,mode); std::fflush(stdout);
      telemetry<<t<<','<<x<<','<<y<<','<<yaw<<','<<current<<','<<next<<','<<replans<<','<<position[2]<<','<<grid.cells[current].elevation<<'\n';
    }
    if(t>1200) { finish(false,"timeout",x,y); break; }
  }
  wb_robot_cleanup(); return 0;
}


