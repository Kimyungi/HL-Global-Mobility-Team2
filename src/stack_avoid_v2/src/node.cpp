// Wall planner ROS input adapter. Control mode is activated only by MGM feedback.
#include "stack_avoid_v2/core.hpp"
#include "stack_avoid_v2/return_tracker.hpp"
#include <rclcpp/rclcpp.hpp>
#include <fma_interfaces/msg/avoid_course.hpp>
#include <fma_interfaces/msg/avoid_plan.hpp>
#include <fma_interfaces/msg/gps_path.hpp>
#include <fma_interfaces/msg/gps_route.hpp>
#include <fma_interfaces/msg/vehicle_vector.hpp>
#include <fma_interfaces/msg/mgm_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <algorithm>
#include <cmath>
#include <map>
#include <memory>

namespace
{
using namespace avoid_v2;
using CourseMsg=fma_interfaces::msg::AvoidCourse;
using PlanMsg=fma_interfaces::msg::AvoidPlan;
using Scan=sensor_msgs::msg::LaserScan;
double seconds(const builtin_interfaces::msg::Time & t) {return t.sec+t.nanosec*1e-9;}
builtin_interfaces::msg::Time stamp(double time)
{return rclcpp::Time(int64_t(std::max(0.,time)*1e9));}
double steady()
{return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
fma_interfaces::msg::RefPoint point(const State & s,double wheelbase)
{
  fma_interfaces::msg::RefPoint p;p.x=s.x;p.y=s.y;p.yaw=s.yaw;
  p.curvature=std::tan(s.steer)/wheelbase;return p;
}
struct Sensor
{
  std::string id,frame;
  double x{},y{},yaw{},fov_min{},fov_max{},offset{},min_range{},max_range{};
  double last_stamp{},last_success{},received{};
};

class AvoidNode : public rclcpp::Node
{
public:
  AvoidNode():Node("avoid_v2_node")
  {
    auto param=[&](const char * name,double & value) {value=declare_parameter<double>(name,value);};
    param("resolution",cfg_.resolution);param("free_ttl",cfg_.free_ttl);
    param("width",cfg_.width);param("front",cfg_.front);param("rear",cfg_.rear);param("margin",cfg_.margin);
    param("wheelbase",cfg_.wheelbase);param("max_steer",cfg_.max_steer);
    param("steer_rate",cfg_.steer_rate);param("steer_tau",cfg_.steer_tau);
    param("cruise_speed",cfg_.cruise_speed);param("brake_decel",cfg_.brake_decel);
    cfg_.adaptive_speed=declare_parameter<bool>("adaptive_speed",cfg_.adaptive_speed);
    param("crawl_speed",cfg_.crawl_speed);param("acceleration",cfg_.acceleration);
    param("planning_decel",cfg_.planning_decel);
    param("brake_delay",cfg_.brake_delay);param("observation_gap",cfg_.observation_gap);
    param("processing_delay",cfg_.processing_delay);param("longitudinal_margin",cfg_.longitudinal_margin);
    param("lateral_accel",cfg_.lateral_accel);param("preview",cfg_.preview);
    param("horizon",cfg_.horizon);param("step",cfg_.step);param("sample_step",cfg_.sample_step);
    param("budget_ms",cfg_.budget_ms);cfg_.validate();
    frame_=declare_parameter<std::string>("course_frame","map");
    trigger_=declare_parameter<std::string>("trigger_sensor","a1");
    input_timeout_=declare_parameter<double>("input_timeout",.20);
    pose_timeout_=declare_parameter<double>("pose_timeout",.10);
    gps_timeout_=declare_parameter<double>("gps_timeout",.50);
    callback_budget_=declare_parameter<double>("callback_budget_ms",35.);
    if (frame_.empty()||!std::isfinite(input_timeout_)||input_timeout_<=0||input_timeout_>.3||
      !std::isfinite(pose_timeout_)||pose_timeout_<=0||pose_timeout_>.2||
      !std::isfinite(gps_timeout_)||gps_timeout_<=0||gps_timeout_>.5||
      !std::isfinite(callback_budget_)||callback_budget_<=0||callback_budget_>35.) {
      throw std::invalid_argument("invalid input/processing limits");
    }
    control_enabled_=declare_parameter<bool>("control_enabled",false);
    planner_=std::make_unique<Planner>(cfg_);grid_=std::make_unique<Grid>(cfg_);
    if(control_enabled_){
      mgm_sub_=create_subscription<fma_interfaces::msg::MgmState>("/adas/mgm_state",1,
        [this](fma_interfaces::msg::MgmState::ConstSharedPtr m){
          const double t=seconds(m->header.stamp);
          if(!fresh(t,.2)||t<=mgm_stamp_){return;}
          mgm_stamp_=t;mgm_received_=steady();
          const bool active=m->top==1&&m->avoidance==1&&m->mission!=m->MISSION_ACTIVE;
          if(active!=mgm_active_){
            mgm_active_=active;planner_->reset();++episode_;
            return_tracker_.reset();
            if(course_ready_){grid_->configure(planner_->course());}
            zone_enter_stamp_=now().seconds();
            for(auto & pair:sensors_){pair.second.last_stamp=pair.second.last_success=pair.second.received=0;}
            publish_blocked(active?"MGM activation requires new observations":"MGM released avoidance");
          }
        });
    }
    plan_pub_=create_publisher<PlanMsg>("/avoid_v2/plan",1);
    path_pub_=create_publisher<nav_msgs::msg::Path>("/avoid_v2/path",1);
    course_sub_=create_subscription<CourseMsg>("/avoid_v2/course",rclcpp::QoS(1).reliable().transient_local(),
      [this](CourseMsg::ConstSharedPtr m) {on_course(*m);});
    route_sub_=create_subscription<fma_interfaces::msg::GpsRoute>("/perception/gps_route",
      rclcpp::QoS(1).reliable().transient_local(),
      [this](fma_interfaces::msg::GpsRoute::ConstSharedPtr m){
        const bool changed=!gps_route_||gps_route_->header.frame_id!=m->header.frame_id||
          gps_route_->route_id!=m->route_id||gps_route_->route!=m->route||gps_route_->points!=m->points;
        gps_route_=m;
        if(control_enabled_&&!boundary_&&changed){course_ready_=false;reset();}
        configure_course();});
    vv_sub_=create_subscription<fma_interfaces::msg::VehicleVector>("/vehicle/vector",rclcpp::SensorDataQoS().keep_last(1),
      [this](fma_interfaces::msg::VehicleVector::ConstSharedPtr m) {
        const double t=seconds(m->header.stamp);
        State s{m->x,m->y,m->yaw,m->v,m->str};
        if (!fresh(t,pose_timeout_)||!finite(s)||std::fabs(s.steer)>cfg_.max_steer||s.speed<-3.||s.speed>3.) {
          vehicle_valid_=false;return;
        }
        if(latest_stamp_>0&&t>latest_stamp_){
          const double dt=t-latest_stamp_;
          if(std::hypot(s.x-latest_.x,s.y-latest_.y)>std::max(std::fabs(s.speed),std::fabs(latest_.speed))*dt+.10||
            std::fabs(wrap(s.yaw-latest_.yaw))>3.*dt+.08){
            alignment_fault_=true;vehicle_valid_=false;return;
          }
        }
        if (history_.push({t,s})) {latest_=s;latest_stamp_=t;vehicle_received_=steady();vehicle_valid_=true;}
      });
    gps_sub_=create_subscription<fma_interfaces::msg::GpsPath>("/perception/gps_path",1,
      [this](fma_interfaces::msg::GpsPath::ConstSharedPtr m) {on_gps(*m);});
    session_sub_=create_subscription<std_msgs::msg::Bool>("/operator/start_session",1,
      [this](std_msgs::msg::Bool::ConstSharedPtr m) {if(m->data){reset();}});
    const auto ids=declare_parameter<std::vector<std::string>>("sensor_ids",{"a1","b1","b2"});
    if(ids.empty()||ids.size()>4){throw std::invalid_argument("one to four calibrated sensors required");}
    for(const auto & id:ids){
      Sensor s;s.id=id;const auto prefix="sensors."+id+".";
      s.frame=declare_parameter<std::string>(prefix+"frame","lidar_"+id+"_link");
      const auto topic=declare_parameter<std::string>(prefix+"topic","/lidar/"+id+"/scan");
      s.x=declare_parameter<double>(prefix+"x",0);s.y=declare_parameter<double>(prefix+"y",0);
      s.yaw=declare_parameter<double>(prefix+"yaw_deg",0)*std::acos(-1.)/180.;
      s.fov_min=declare_parameter<double>(prefix+"fov_min_deg",0)*std::acos(-1.)/180.;
      s.fov_max=declare_parameter<double>(prefix+"fov_max_deg",0)*std::acos(-1.)/180.;
      s.offset=declare_parameter<double>(prefix+"range_offset_m",0);
      s.min_range=declare_parameter<double>(prefix+"min_range",.15);
      s.max_range=declare_parameter<double>(prefix+"max_range",12);
      for(double v:{s.x,s.y,s.yaw,s.fov_min,s.fov_max,s.offset,s.min_range,s.max_range}){
        if(!std::isfinite(v)){throw std::invalid_argument("nonfinite sensor calibration");}
      }
      if(s.fov_max<=s.fov_min||s.min_range<0||s.max_range<=s.min_range||s.max_range>30||sensors_.count(id)){
        throw std::invalid_argument("missing/invalid sensor calibration");
      }
      sensors_[id]=s;
      scan_subs_.push_back(create_subscription<Scan>(topic,rclcpp::SensorDataQoS().keep_last(1),
        [this,id](Scan::ConstSharedPtr m){on_scan(id,*m);}));
    }
    if(!sensors_.count(trigger_)){throw std::invalid_argument("trigger sensor missing");}
    timer_=create_wall_timer(std::chrono::milliseconds(20),[this](){
      if(control_enabled_&&!control_active()){
        planner_->set_zone(false,false);publish_blocked("MGM avoidance inactive or stale");return;
      }
      if(!planner_->perception_required()&&gps_valid_&&fresh(gps_stamp_,gps_timeout_)){
        if(!fresh(latest_stamp_,pose_timeout_)){publish_blocked("vehicle pose unavailable");}
        return;
      }
      if(last_plan_received_==0||steady()-last_plan_received_>input_timeout_){publish_blocked("scan/planner heartbeat missing");}
    });
    parameter_guard_=add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter> &){
      rcl_interfaces::msg::SetParametersResult result;
      result.successful=false;result.reason="restart planner node to change geometry/model parameters";return result;
    });
    RCLCPP_INFO(get_logger(),"Wall planner mode: %s",control_enabled_?"MGM provider":"shadow");
  }
private:
  bool control_active() const
  {return mgm_active_&&fresh(mgm_stamp_,.2)&&steady()-mgm_received_<=.2;}
  bool fresh(double t,double max_age) const
  {const double age=now().seconds()-t;return t>0&&age>=0&&age<=max_age;}
  State world(State s) const
  {
    const double c=std::cos(alignment_.yaw),sn=std::sin(alignment_.yaw);
    return {alignment_.x+c*s.x-sn*s.y,alignment_.y+sn*s.x+c*s.y,
      wrap(s.yaw+alignment_.yaw),s.speed,s.steer};
  }
  void reset()
  {
    ++episode_;planner_->reset();aligned_=false;alignment_fault_=false;gps_valid_=false;
    return_tracker_.reset();pending_trigger_=planned_trigger_=0;
    history_.clear();vehicle_valid_=false;latest_stamp_=0;gps_stamp_=0;
    for(auto & pair:sensors_){pair.second.last_stamp=pair.second.last_success=pair.second.received=0;}
    if(course_ready_){grid_->configure(planner_->course());}
    publish_blocked("new session requires fresh localization and scans");
  }
  void on_course(const CourseMsg & m)
  {boundary_=std::make_shared<CourseMsg>(m);configure_course();}
  void configure_course()
  {
    // Auto workspace is refreshed on route changes or near its end, not on
    // harmless repeats of the latched route message.
    if(control_enabled_&&!boundary_&&gps_route_){return;}
    // Reliable/transient publishers may repeat unchanged geometry with a fresh
    // header stamp. Only semantic changes invalidate the active observation map.
    if(course_ready_&&boundary_&&gps_route_&&applied_boundary_&&applied_route_&&
      boundary_->header.frame_id==applied_boundary_->header.frame_id&&
      boundary_->route_id==applied_boundary_->route_id&&boundary_->boundary==applied_boundary_->boundary&&
      boundary_->entry_station_m==applied_boundary_->entry_station_m&&
      boundary_->exit_station_m==applied_boundary_->exit_station_m&&
      gps_route_->header.frame_id==applied_route_->header.frame_id&&
      gps_route_->route_id==applied_route_->route_id&&gps_route_->route==applied_route_->route&&
      gps_route_->points==applied_route_->points){return;}
    course_ready_=false;
    if(!boundary_||!gps_route_){
      if(control_enabled_&&gps_route_&&!boundary_){return;}
      publish_blocked("GPS route and explicit boundary required");return;
    }
    const auto & m=*boundary_;
    try{
      if(m.header.frame_id!=frame_||gps_route_->header.frame_id!=frame_){throw std::invalid_argument("course frame mismatch");}
      if(m.route_id!=gps_route_->route_id){throw std::invalid_argument("GPS route and boundary identity mismatch");}
      if(gps_route_->points.size()>10000||m.boundary.size()>2000){throw std::invalid_argument("oversized course");}
      Course c;c.id=m.route_id;c.entry=m.entry_station_m;c.exit=m.exit_station_m;
      for(const auto & p:gps_route_->points){c.center.push_back({p.x,p.y});}
      for(const auto & p:m.boundary){c.boundary.push_back({p.x,p.y});}
      c.validate();
      // A course replacement invalidates localization and every occupancy observation.
      planner_->set_course(c);grid_->configure(c);course_ready_=true;reset();
      applied_boundary_=boundary_;applied_route_=gps_route_;
      RCLCPP_INFO(get_logger(),"course %s loaded, cells=%zu",c.id.c_str(),grid_->size());
    }catch(const std::exception & e){course_ready_=false;publish_blocked(e.what());}
  }
  void on_gps(const fma_interfaces::msg::GpsPath & m)
  {
    const double t=seconds(m.reference_stamp);
    if(control_enabled_&&!boundary_&&gps_route_&&fresh(t,gps_timeout_)&&m.position_valid&&
      m.fix_quality==4&&std::isfinite(m.position_x)&&std::isfinite(m.position_y)){
      if(!course_ready_){configure_workspace(m);}
      else if(aligned_&&control_active()){
        const auto pr=planner_->course().project({m.position_x,m.position_y},0,planner_->course().length());
        if(pr.station>planner_->course().length()-cfg_.horizon-2.){configure_workspace(m);}
      }
    }
    if(!course_ready_||!fresh(t,gps_timeout_)||m.fix_quality!=4||!m.position_valid||
      !m.vehicle_heading_valid||m.heading_source==m.HEADING_TANGENT||!m.zone_valid||m.points.empty()||
      !std::isfinite(m.position_x)||!std::isfinite(m.position_y)||!std::isfinite(m.vehicle_heading_rad)||
      m.route.enabled!=gps_route_->route.enabled||
      (m.route.enabled&&(m.route.route_id!=planner_->course().id||
        m.route.instance_id!=gps_route_->route.instance_id||
        m.route.acknowledged_request!=gps_route_->route.acknowledged_request||
        m.route.connecting!=gps_route_->route.connecting))){
      gps_valid_=false;planner_->set_zone(false,false);publish_blocked("GPS route/zone/localization unavailable");return;
    }
    if(t<=gps_stamp_){return;}
    State odom;
    if(!history_.at(t,odom)){gps_valid_=false;return;}
    if(aligned_){
      const auto predicted=world(odom);
      if(std::hypot(predicted.x-m.position_x,predicted.y-m.position_y)>.15||
        std::fabs(wrap(predicted.yaw-m.vehicle_heading_rad))>.08){
        alignment_fault_=true;gps_valid_=false;publish_blocked("GPS/odometry frame disagreement; new session required");return;
      }
    }else{
      alignment_.yaw=wrap(m.vehicle_heading_rad-odom.yaw);
      alignment_.x=m.position_x-std::cos(alignment_.yaw)*odom.x+std::sin(alignment_.yaw)*odom.y;
      alignment_.y=m.position_y-std::sin(alignment_.yaw)*odom.x-std::cos(alignment_.yaw)*odom.y;
      aligned_=true;
    }
    gps_stamp_=t;gps_received_=steady();gps_valid_=true;
    const bool was_active=planner_->episode_active();
    // MGM owns the zone latch and completion. A geographic exit cannot cancel its episode.
    planner_->set_zone(!control_enabled_||control_active(),control_enabled_?control_active():m.avoid_zone);
    if(!was_active&&planner_->perception_required()){
      grid_->configure(planner_->course());zone_enter_stamp_=t;
      for(auto & pair:sensors_){pair.second.last_stamp=pair.second.last_success=pair.second.received=0;}
    }
    if(!planner_->perception_required()){
      const auto p=planner_->plan(world(latest_),*grid_,now().seconds(),++generation_);
      emit(p,world(latest_),0);
    }
  }
  void on_scan(const std::string & id,const Scan & scan)
  {
    // Subscription stays alive, but recognition/occupancy work starts only in the zone.
    if((control_enabled_&&!control_active())||!planner_->perception_required()){return;}
    const double start=steady();auto & sensor=sensors_.at(id);const double t=seconds(scan.header.stamp);
    auto fail=[&](const char * why){sensor.last_success=0;publish_blocked(why);};
    if(!course_ready_||!aligned_||alignment_fault_||!gps_valid_||!vehicle_valid_||
      !fresh(gps_stamp_,gps_timeout_)||steady()-gps_received_>gps_timeout_||
      !fresh(latest_stamp_,pose_timeout_)||steady()-vehicle_received_>pose_timeout_){fail("course/localization unavailable");return;}
    if(t<=sensor.last_stamp||t<zone_enter_stamp_){return;}
    sensor.last_stamp=t;
    if(scan.header.frame_id!=sensor.frame||!fresh(t,input_timeout_)||scan.ranges.size()<2||scan.ranges.size()>10000||
      !std::isfinite(scan.angle_min)||!std::isfinite(scan.angle_increment)||scan.angle_increment<=0||
      !std::isfinite(scan.time_increment)||scan.time_increment<=0||
      !std::isfinite(scan.range_min)||!std::isfinite(scan.range_max)||scan.range_min<0||scan.range_max<=scan.range_min||
      scan.time_increment*(scan.ranges.size()-1)>.15){fail("invalid raw scan frame/timing/envelope");return;}
    std::vector<Ray> rays;rays.reserve(scan.ranges.size());
    for(size_t i=0;i<scan.ranges.size();++i){
      const double angle=scan.angle_min+i*scan.angle_increment;
      if(angle<sensor.fov_min||angle>sensor.fov_max){continue;}
      const double raw=scan.ranges[i],range=raw+sensor.offset;
      // NaN/Inf and truncated returns remain unknown; no virtual-origin clearing.
      if(!std::isfinite(raw)||raw<scan.range_min||raw>scan.range_max||range<sensor.min_range||range>sensor.max_range){continue;}
      const double beam_time=t+i*scan.time_increment;State odom;
      if(!history_.at(beam_time,odom)){fail("missing per-beam pose history");return;}
      const auto pose=world(odom);
      const Point origin{pose.x+std::cos(pose.yaw)*sensor.x-std::sin(pose.yaw)*sensor.y,
        pose.y+std::sin(pose.yaw)*sensor.x+std::cos(pose.yaw)*sensor.y};
      rays.push_back({origin,{origin.x+range*std::cos(pose.yaw+sensor.yaw+angle),
        origin.y+range*std::sin(pose.yaw+sensor.yaw+angle)},beam_time,true});
    }
    if(rays.empty()){fail("scan contains no usable observed rays");return;}
    grid_->observe(rays,++generation_);sensor.last_success=t;sensor.received=steady();
    if(id==trigger_){pending_trigger_=t;}
    // Three independent subscriptions can be dispatched in any order. Finish the
    // pending front-scan cycle when its required side observations arrive.
    if(pending_trigger_<=planned_trigger_){return;}
    for(const auto & item:sensors_){
      if(!fresh(item.second.last_success,input_timeout_)||steady()-item.second.received>input_timeout_){
        publish_blocked("required sensor missing/stale");return;
      }
    }
    planned_trigger_=pending_trigger_;
    const auto current=world(latest_);
    if(current.speed<0){publish_blocked("reverse recovery owns motion; forward planner waits");return;}
    auto plan=planner_->plan(current,*grid_,now().seconds(),generation_);
    if(control_enabled_){update_return(plan,current);}
    const double processing=(steady()-start)*1000.;
    if(processing>callback_budget_){
      plan.valid=plan.complete=false;plan.path.clear();plan.phase=Phase::HOLD;plan.deadline_hit=true;plan.reason="callback processing budget exceeded";
    }
    emit(plan,current,processing);
  }
  void publish_blocked(const std::string & reason)
  {
    Plan p;p.reason=reason;p.generation=generation_;emit(p,{},0);
  }
  void emit(const Plan & p,const State & pose,double processing)
  {
    PlanMsg msg;msg.header.stamp=now();msg.header.frame_id=frame_;
    msg.control_enabled=control_enabled_;
    if(gps_route_){msg.route=gps_route_->route;}
    msg.route_id=course_ready_?planner_->course().id:"";msg.episode_id=episode_;msg.plan_id=p.id;
    double oldest=now().seconds();
    for(const auto & item:sensors_){oldest=std::min(oldest,item.second.last_success);}
    const bool outside=p.gps_follow&&!p.perception_active&&gps_valid_&&vehicle_valid_;
    const double valid_until=outside?std::min(latest_stamp_+pose_timeout_,gps_stamp_+gps_timeout_):
      std::min({p.valid_until,oldest+input_timeout_,latest_stamp_+pose_timeout_,gps_stamp_+gps_timeout_});
    const bool valid=p.valid&&now().seconds()<valid_until&&(!control_enabled_||control_active());
    if(control_enabled_&&!valid){return_tracker_.invalidate();}
    msg.observation_generation=p.generation;msg.phase=uint8_t(valid||outside?p.phase:Phase::HOLD);
    msg.episode_active=planner_->episode_active();msg.perception_active=planner_->perception_required();
    msg.obstacle_detected=p.obstacle_detected;msg.maneuver_active=p.maneuver_active;
    msg.gps_follow=p.gps_follow&&(valid||outside);
    msg.plan_valid=valid;msg.complete=control_enabled_?(valid&&return_tracker_.done()):((valid||outside)&&p.complete);
    msg.deadline_hit=p.deadline_hit;msg.reason=p.reason;msg.processing_ms=processing;
    if(p.valid&&!valid){msg.reason="observation/pose lease expired before publication";}
    msg.planner_ms=p.compute_ms;msg.expanded_nodes=uint32_t(p.expanded);msg.station_m=p.station;
    msg.stop_distance_m=p.stop_distance;msg.min_clearance_m=p.min_clearance;
    msg.speed_limit_mps=valid?p.speed_limit:0;
    msg.observation_stamp=stamp(oldest);msg.pose_stamp=stamp(latest_stamp_);
    msg.valid_until=stamp(valid||outside?valid_until:0);
    auto & ref=msg.reference;ref.header.stamp=stamp(latest_stamp_);ref.header.frame_id="base_link";
    ref.reference_stamp=stamp(valid?std::min(oldest,latest_stamp_):0);
    // Membership/ownership is carried explicitly above, never disguised as obstacle detection.
    ref.scan_valid=valid;ref.avoidable=valid;ref.maneuver_done=msg.complete;
    ref.obstacle_detected=p.obstacle_detected;
    ref.ttc=1e9f;ref.v_suggest=valid?p.speed_limit:0;
    if(valid){
      State local=p.reference;const double dx=local.x-pose.x,dy=local.y-pose.y;
      local.x=std::cos(pose.yaw)*dx+std::sin(pose.yaw)*dy;
      local.y=-std::sin(pose.yaw)*dx+std::cos(pose.yaw)*dy;local.yaw=wrap(local.yaw-pose.yaw);
      ref.points.push_back(point(local,cfg_.wheelbase));
    }
    nav_msgs::msg::Path path;path.header=msg.header;
    for(const auto & s:p.path){
      if(!valid){break;}
      msg.path.push_back(point(s,cfg_.wheelbase));geometry_msgs::msg::PoseStamped ps;ps.header=path.header;
      msg.path_speed_mps.push_back(s.speed);
      ps.pose.position.x=s.x;ps.pose.position.y=s.y;
      ps.pose.orientation.z=std::sin(s.yaw/2);ps.pose.orientation.w=std::cos(s.yaw/2);path.poses.push_back(ps);
    }
    plan_pub_->publish(msg);path_pub_->publish(path);
    if(p.id||outside){last_plan_received_=steady();}
  }
  // Rectangle bounds computation/storage only; they never mark cells free or invent curbs.
  void configure_workspace(const fma_interfaces::msg::GpsPath & gps)
  {
    try{
      if(gps_route_->header.frame_id!=frame_||gps_route_->points.size()<2||gps_route_->points.size()>10000){
        throw std::invalid_argument("GPS route missing for observation workspace");
      }
      Course full;full.id=gps_route_->route_id;
      for(const auto & p:gps_route_->points){full.center.push_back({p.x,p.y});}
      // Validate all route coordinates and spacing before selecting a local window.
      double minx=full.center[0].x,maxx=minx,miny=full.center[0].y,maxy=miny;
      for(auto p:full.center){minx=std::min(minx,p.x);maxx=std::max(maxx,p.x);miny=std::min(miny,p.y);maxy=std::max(maxy,p.y);}
      full.boundary={{minx-12,miny-12},{maxx+12,miny-12},{maxx+12,maxy+12},{minx-12,maxy+12}};
      full.entry=0;full.exit=.01;full.validate();
      const auto at=full.project({gps.position_x,gps.position_y},0,full.length());
      Course c;c.id=full.id;
      const double low=std::max(0.,at.station-5),high=std::min(full.length(),at.station+30);
      c.center.push_back(full.at(low));
      for(size_t i=0;i<full.center.size();++i){if(full.stations[i]>low+1e-5&&full.stations[i]<high-1e-5){c.center.push_back(full.center[i]);}}
      c.center.push_back(full.at(high));
      minx=maxx=c.center[0].x;miny=maxy=c.center[0].y;
      for(auto p:c.center){minx=std::min(minx,p.x);maxx=std::max(maxx,p.x);miny=std::min(miny,p.y);maxy=std::max(maxy,p.y);}
      c.boundary={{minx-12,miny-12},{maxx+12,miny-12},{maxx+12,maxy+12},{minx-12,maxy+12}};
      c.entry=0;c.exit=.01;c.validate();
      planner_->set_course(c);grid_->configure(c);course_ready_=true;
      planner_->set_zone(control_active(),control_active());zone_enter_stamp_=now().seconds();
      for(auto & pair:sensors_){pair.second.last_stamp=pair.second.last_success=pair.second.received=0;}
      return_tracker_.invalidate();
    }catch(const std::exception & e){course_ready_=false;publish_blocked(e.what());}
  }
  void update_return(const Plan & p,const State & pose)
  {return_tracker_.update(p,pose,planner_->course(),*grid_,cfg_,planner_->stopping_distance(pose.speed));}
  bool control_enabled_{},mgm_active_{};
  ReturnTracker return_tracker_;
  double mgm_stamp_{},mgm_received_{};
  rclcpp::Subscription<fma_interfaces::msg::MgmState>::SharedPtr mgm_sub_;
  Config cfg_;std::unique_ptr<Planner> planner_;std::unique_ptr<Grid> grid_;PoseHistory history_;
  std::string frame_,trigger_;std::map<std::string,Sensor> sensors_;
  State alignment_,latest_;double latest_stamp_{},gps_stamp_{},vehicle_received_{},gps_received_{},last_plan_received_{};
  double input_timeout_{},pose_timeout_{},gps_timeout_{},callback_budget_{};
  double zone_enter_stamp_{},pending_trigger_{},planned_trigger_{};
  std::shared_ptr<CourseMsg> boundary_;
  fma_interfaces::msg::GpsRoute::ConstSharedPtr gps_route_;
  std::shared_ptr<CourseMsg> applied_boundary_;
  fma_interfaces::msg::GpsRoute::ConstSharedPtr applied_route_;
  bool course_ready_{},aligned_{},alignment_fault_{},gps_valid_{},vehicle_valid_{};
  uint64_t episode_{1},generation_{};
  rclcpp::Publisher<PlanMsg>::SharedPtr plan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<CourseMsg>::SharedPtr course_sub_;
  rclcpp::Subscription<fma_interfaces::msg::GpsRoute>::SharedPtr route_sub_;
  rclcpp::Subscription<fma_interfaces::msg::GpsPath>::SharedPtr gps_sub_;
  rclcpp::Subscription<fma_interfaces::msg::VehicleVector>::SharedPtr vv_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr session_sub_;
  std::vector<rclcpp::Subscription<Scan>::SharedPtr> scan_subs_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_guard_;
};
}
int main(int argc,char ** argv)
{
  rclcpp::init(argc,argv);
  try{rclcpp::spin(std::make_shared<AvoidNode>());}
  catch(const std::exception & e){std::fprintf(stderr,"avoid_v2: %s\n",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
