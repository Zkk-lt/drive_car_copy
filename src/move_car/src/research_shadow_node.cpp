// Passive external adapter reusing the existing research bridge. No command publisher.
#define RESEARCH_EXTERNAL_SHADOW_NODE
#include "control/research_ros_bridge.h"
#include "control/control_snapshot_continuity.h"
#include "control/causal_raw_snapshot.h"
#include "control/local_motion_transport.h"
#include <move_car/ControlResearchTelemetry.h>
#include <move_car/car_parameter.h>
#include <deque>
class ExternalShadow {
 struct Pose {double t,x,y,yaw;};
 struct Point {double s=0.,x=0.,y=0.,yaw=0.,curvature=0.;bool pose_yaw_valid=false;};
 ros::NodeHandle nh_,pnh_{"~"};research_control::ResearchRosBridge bridge_;
 ros::Subscriber wheel_,imu_,ground_,control_;ros::Publisher health_;ros::Timer timer_;
 move_car::ControlResearchTelemetry::ConstPtr pending_;ros::WallTime pending_since_;
 std::deque<Pose> poses_;std::string run_,world_,body_;bool pose_verified_=false;double last_=0.;
 std::string transport_mode_="verified_external_pose",local_imu_frame_;
 research_control::LocalMotionTransport<> local_motion_;
 research_control::ControlSnapshotContinuity continuity_;
 unsigned retained_without_prediction_=0,rejected_pending_=0,history_resets_=0;
 struct Wheel {double speed,steering;};
 research_control::CausalRawSnapshot<Wheel> wheel_raw_;
 research_control::CausalRawSnapshot<sensor_msgs::Imu::ConstPtr> imu_raw_;
 research_control::CausalRawSnapshot<nav_msgs::Odometry::ConstPtr> ground_raw_;
 void invalidateHistory(){
  bridge_.invalidateCommandHistory();wheel_raw_.resetFed();imu_raw_.resetFed();ground_raw_.resetFed();
 }
 void feedCausal(double stamp){
  wheel_raw_.feed(stamp,[&](const Wheel& value,double receipt){bridge_.wheel(value.speed,value.steering,receipt);});
  imu_raw_.feed(stamp,[&](const sensor_msgs::Imu::ConstPtr& value,double receipt){bridge_.imu(*value,receipt);});
  ground_raw_.feed(stamp,[&](const nav_msgs::Odometry::ConstPtr& value,double receipt){bridge_.ground(*value,receipt);});
 }
 bool pose(double t,Pose& out){
  for(size_t i=1;i<poses_.size();++i){const auto& a=poses_[i-1];const auto& b=poses_[i];
   if(a.t<=t&&t<=b.t&&b.t-a.t>0.&&b.t-a.t<=.1){const double f=(t-a.t)/(b.t-a.t);
    out={t,a.x+f*(b.x-a.x),a.y+f*(b.y-a.y),a.yaw+f*research_control::wrap(b.yaw-a.yaw)};return true;}}
  return false;
 }
 void wheel(const move_car::car_parameter::ConstPtr& m){
  const double receipt=ros::Time::now().toSec();wheel_raw_.push({m->back_wheel_speed,m->turn_angle},receipt);
  if(transport_mode_=="local_wheel_imu"){
   local_motion_.updateWheel(m->back_wheel_speed,0.,receipt);
   local_motion_.updateSteering(m->turn_angle,0.,receipt);
  }
 }
 void imu(const sensor_msgs::Imu::ConstPtr& m){
  const double receipt=ros::Time::now().toSec();imu_raw_.push(m,receipt);
  if(transport_mode_=="local_wheel_imu"&&m->header.frame_id==local_imu_frame_&&
      m->angular_velocity_covariance[0]>=0.)
   local_motion_.updateImu(m->angular_velocity.z,m->header.stamp.toSec(),receipt);
 }
 void ground(const nav_msgs::Odometry::ConstPtr& m){
  ground_raw_.push(m,ros::Time::now().toSec());const auto& q=m->pose.pose.orientation;
  const auto& p=m->pose.pose.position;const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w,t=m->header.stamp.toSec();
  if(!pose_verified_||m->header.frame_id!=world_||m->child_frame_id!=body_||!std::isfinite(norm)||std::fabs(norm-1.)>.02||
     !std::isfinite(p.x)||!std::isfinite(p.y)||t<=0.)return;
  if(!poses_.empty()&&t<=poses_.back().t){poses_.clear();return;}
  poses_.push_back({t,p.x,p.y,std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))});
  while(poses_.size()>300)poses_.pop_front();
 }
 void control(const move_car::ControlResearchTelemetry::ConstPtr& m){
  bool lost_pending=false;
  if(pending_){
   // Retain every received finite actual command in the new history, even if
   // its pose never arrived. No optimization is allowed for this old snapshot.
   // Successfully recording it is not a command-history loss. A rejected
   // stale/nonfinite/mismatched snapshot must still invalidate the history.
   const auto skipped=pending_;pending_.reset();lost_pending=!process(skipped,true);
   if(lost_pending)++rejected_pending_;else ++retained_without_prediction_;
  }
  const auto continuity=continuity_.observe(m->header.seq,m->header.stamp.toSec(),lost_pending);
  if(continuity.reset_history){invalidateHistory();++history_resets_;}
  pending_since_=ros::WallTime::now();pending_=m;
 }
 void scheduled(const ros::TimerEvent&){
  if(!pending_)return;
  if(transport_mode_=="verified_external_pose"&&pose_verified_&&
     (poses_.empty()||poses_.back().t<pending_->header.stamp.toSec())&&
     (ros::WallTime::now()-pending_since_).toSec()<.08)return;
  const auto m=pending_;pending_.reset();process(m);
 }
 bool process(const move_car::ControlResearchTelemetry::ConstPtr& m,bool forced_invalid=false){
  const double now=m->header.stamp.toSec(),age=ros::Time::now().toSec()-now;
  std_msgs::String health;
  if(m->run_id!=run_||now<=last_||age<-.02||age>.1){
   continuity_.invalidate(now,"rejected_stale_or_mismatched_control_snapshot");invalidateHistory();++history_resets_;
   health.data="rejected_stale_or_mismatched_control_snapshot";health_.publish(health);return false;}
  last_=now;bridge_.beginCycle();std::vector<Point> points;Pose old,current;
  double da=0.,tx=0.,ty=0.;bool transform_valid=false;
  research_control::LocalTransportResult local;
  if(transport_mode_=="local_wheel_imu"){
   local=local_motion_.relative(m->reference_stamp.toSec(),now);
   transform_valid=local.valid;da=local.yaw;tx=local.tx;ty=local.ty;
  }else if(pose(m->reference_stamp.toSec(),old)&&pose(now,current)){
   transform_valid=true;da=old.yaw-current.yaw;
   const double dx=old.x-current.x,dy=old.y-current.y;
   tx=std::cos(current.yaw)*dx+std::sin(current.yaw)*dy;
   ty=-std::sin(current.yaw)*dx+std::cos(current.yaw)*dy;
  }
  const bool geometry=transform_valid&&bridge_.fillGeometry(m->reference_stamp,points);
  if(geometry){
   for(auto& p:points){const double x=p.x,y=p.y;p.x=std::cos(da)*x-std::sin(da)*y+tx;p.y=std::sin(da)*x+std::cos(da)*y+ty;p.yaw=research_control::wrap(p.yaw+da);}
   bridge_.setGeometry(points);
  }
  research_control::Command actual;actual.speed=m->command_speed;actual.steering=m->command_steering;
  if(!std::isfinite(actual.speed)||!std::isfinite(actual.steering)){
   continuity_.invalidate(now,"nonfinite_actual_command");invalidateHistory();++history_resets_;
   health.data="nonfinite_actual_command";health_.publish(health);return false;}
  const bool history_valid=!forced_invalid&&continuity_.ready(now);
  // Select only raw records already received by this control snapshot. Newer
  // raw callbacks stay buffered; no timestamp is backdated to make it usable.
  feedCausal(now);
  bridge_.tick(actual,m->target_speed,now,m->tracking_valid&&geometry&&history_valid,m->safe_stop||m->collision_stop);
  if(forced_invalid&&continuity_.ready(now))health.data="prediction_skipped_pending_snapshot_actual_command_retained";
  else if(!history_valid)health.data=std::string("invalid_command_history:")+continuity_.reason()+
    ";remaining_contiguous_delay_window_s="+std::to_string(continuity_.remaining(now));
  else if(geometry)health.data="external_shadow_only_snapshot_age_s="+std::to_string(age);
  else if(transport_mode_=="local_wheel_imu")health.data=std::string("invalid_local_reference_or_metadata:")+local.reason;
  else health.data="invalid_reference_transport_requires_calibrated_body_center_pose";
  health.data+=";transport_mode="+transport_mode_;
  if(transport_mode_=="local_wheel_imu")health.data+=std::string(";local_motion_reason=")+local.reason+
    ";local_position_uncertainty_m="+std::to_string(local.position_uncertainty_m)+
    ";local_yaw_uncertainty_rad="+std::to_string(local.yaw_uncertainty_rad)+
    ";local_bicycle_fallback="+std::to_string(local.bicycle_fallback_used)+
    ";local_rejected_samples="+std::to_string(local.rejected_samples)+
    ";local_hardware_freshness_verified=0;local_independent_ground_speed=0";
  health.data+=";retained_without_prediction="+std::to_string(retained_without_prediction_)+
    ";rejected_pending="+std::to_string(rejected_pending_)+";history_resets="+std::to_string(history_resets_)+
    ";causal_wheel_receipt_s="+std::to_string(wheel_raw_.lastFedReceipt())+
    ";causal_imu_receipt_s="+std::to_string(imu_raw_.lastFedReceipt())+
    ";causal_ground_receipt_s="+std::to_string(ground_raw_.lastFedReceipt())+
    ";raw_overwritten="+std::to_string(wheel_raw_.overwritten()+imu_raw_.overwritten()+ground_raw_.overwritten())+
    ";raw_rejected="+std::to_string(wheel_raw_.rejected()+imu_raw_.rejected()+ground_raw_.rejected());
  health_.publish(health);return true;
 }
public:
 ExternalShadow(){
  nh_.param<std::string>("/control_research/run_id",run_,"");
  pnh_.param<std::string>("reference_transport",transport_mode_,"verified_external_pose");
  if(transport_mode_!="verified_external_pose"&&transport_mode_!="local_wheel_imu")
   throw std::runtime_error("unknown research reference transport");
  if(transport_mode_=="local_wheel_imu"){
   research_control::LocalMotionConfig c;
   ros::NodeHandle s(pnh_,"research_runtime/sensors"),v(pnh_,"research_runtime/values"),l(pnh_,"local_transport");
   // Reuse the SAME calibrated units/sign and FLU frame contract as observer.
   // No global pose/independence claim is added for wheel/gyro short motion.
   c.wheel_scale=s.param("wheel_scale",.001);c.wheel_sign=s.param("wheel_sign",1.);
   c.steering_scale=s.param("steering_scale",1.);c.steering_sign=s.param("steering_sign",1.);
   c.gyro_scale=s.param("gyro_scale",1.);c.gyro_sign=s.param("gyro_sign",1.);
   c.wheel_calibrated=s.param("wheel_calibrated",false);
   c.steering_calibrated=s.param("steering_calibrated",false);
   c.imu_frame_verified=s.param("imu_frame_verified",false);
   c.wheelbase=v.param("wheelbase",.60);
   pnh_.param<std::string>("research_runtime/frame_contract/imu_body_frame",local_imu_frame_,"");
   if(local_imu_frame_.empty())c.imu_frame_verified=false;
   // Explicitly reviewed short-window and uncertainty bounds, never a source
   // acquisition or real-vehicle-control certificate. Defaults stay bounded.
   c.max_transport_s=l.param("max_transport_s",c.max_transport_s);
   c.max_sample_age_s=l.param("max_sample_age_s",c.max_sample_age_s);
   c.allow_bicycle_fallback=l.param("allow_bicycle_fallback",false);
   c.speed_uncertainty_mps=l.param("speed_uncertainty_mps",c.speed_uncertainty_mps);
   c.gyro_uncertainty_radps=l.param("gyro_uncertainty_radps",c.gyro_uncertainty_radps);
   c.max_position_uncertainty_m=l.param("max_position_uncertainty_m",c.max_position_uncertainty_m);
   c.max_yaw_uncertainty_rad=l.param("max_yaw_uncertainty_rad",c.max_yaw_uncertainty_rad);
   if(!local_motion_.configure(c))throw std::runtime_error("invalid local motion transport contract");
  }
  pnh_.param("pose_reference_body_center_verified",pose_verified_,false);
  pnh_.param<std::string>("research_runtime/frame_contract/ground_world_frame",world_,"");
  pnh_.param<std::string>("research_runtime/frame_contract/ground_body_frame",body_,"");
  if(!bridge_.configure(nh_,pnh_,run_))throw std::runtime_error("external research configuration rejected");
  continuity_=research_control::ControlSnapshotContinuity(bridge_.maximumActuatorDelay());
  health_=nh_.advertise<std_msgs::String>("/control_research/shadow_health",2);
  wheel_=nh_.subscribe("/car_message",8,&ExternalShadow::wheel,this);
  imu_=nh_.subscribe("/imu",32,&ExternalShadow::imu,this);
  // Local mode does not even subscribe to NED; optional external mode retains
  // its prior compatibility path unchanged. Neither mode owns chassis output.
  if(transport_mode_=="verified_external_pose")ground_=nh_.subscribe("/NED_odometry",32,&ExternalShadow::ground,this);
  control_=nh_.subscribe("/control_research/controller",1,&ExternalShadow::control,this);
  timer_=nh_.createTimer(ros::Duration(.01),&ExternalShadow::scheduled,this);
 }
};
int main(int argc,char** argv){ros::init(argc,argv,"research_external_shadow");try{ExternalShadow s;ros::spin();}
 catch(const std::exception& e){ROS_ERROR("External shadow stopped: %s",e.what());return 1;}return 0;}
