#pragma once
#include "runtime_ros_config.h"
#include <move_car/AdaptiveControlTelemetry.h>
#include <move_car/ResearchValidationTelemetry.h>
#include <move_car/ReferenceInfo.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <deque>

namespace research_control {
// A side-effect-free adapter for control authority: it publishes diagnostics,
// never chassis commands. The existing control_pub remains the sole publisher.
class ResearchRosBridge {
public:
 bool configure(ros::NodeHandle& nh,ros::NodeHandle& pnh,const std::string& run_id) {
   enabled_=pnh.param("enable_adaptive_shadow",false);if(!enabled_)return false;
#ifndef RESEARCH_EXTERNAL_SHADOW_NODE
   // Research optimization must never execute in the S8 command callback.
   enabled_=false;
   ROS_ERROR("Inline NMPC shadow disabled: use research_shadow_node in an independent process");
   return false;
#endif
   try {
     if(run_id.empty())throw std::runtime_error("a shared nonempty run_id is required for shadow experiments");
     std::string mode,terrain;config_=loadRuntime(pnh,mode,terrain,hash_);run_id_=run_id;
     config_.solver.profile=pnh.param("research_detailed_profiling",false);
     pnh.param<std::string>("research_runtime/frame_contract/ground_twist_convention",ground_convention_,"unverified");
     pnh.param<std::string>("research_runtime/frame_contract/ground_world_frame",ground_world_frame_,"");
     pnh.param<std::string>("research_runtime/frame_contract/ground_body_frame",ground_body_frame_,"");
     pnh.param<std::string>("research_runtime/frame_contract/imu_body_frame",imu_body_frame_,"");
     if(!runtime_.configure(config_)||!runtime_.requestMode(mode)||
        !runtime_.requestTerrain(terrain,ros::Time::now().toSec()))throw std::runtime_error("runtime initialization rejected");
     pub_=nh.advertise<move_car::AdaptiveControlTelemetry>("/control_research/adaptive",2);
     validation_pub_=nh.advertise<move_car::ResearchValidationTelemetry>("/control_research/validation",2);
     reference_sub_=nh.subscribe("/control_research/reference",8,&ResearchRosBridge::reference,this);
     mode_sub_=nh.subscribe("/control_research/request_mode",1,&ResearchRosBridge::mode,this);
     terrain_sub_=nh.subscribe("/control_research/request_terrain",1,&ResearchRosBridge::terrain,this);
     spatial_.reserve(512);history_.reserve(512);
     ROS_WARN("Adaptive research SHADOW enabled; actual command owner remains legacy_slqr; snapshot=%s",hash_.c_str());
   }catch(const std::exception& e){enabled_=false;ROS_ERROR("Adaptive shadow disabled: %s",e.what());}
   return enabled_;
 }
 bool enabled()const{return enabled_;}
 // External telemetry can lose intermediate actual commands. Never use such
 // a sparse sequence as a complete delay/RLS history. The caller must hold
 // reference validity false until a continuous delay window has refilled.
 void invalidateCommandHistory(){runtime_.observer().reset();history_.clear();previous_acceleration_=0.;last_now_=0.;}
 double maximumActuatorDelay()const{return std::max(config_.solver.model.steering_delay,config_.solver.model.speed_delay);}
 void beginCycle(){if(enabled_){spatial_.clear();reference_reason_="no_accepted_reference";}}
 void wheel(double raw,double steering,double receipt) {
   if(!enabled_)return;
   runtime_.observer().updateWheel(raw,0.,receipt);runtime_.observer().updateSteering(steering,0.,receipt);
 }
 void imu(const sensor_msgs::Imu& m,double receipt) {
   if(!enabled_)return;const auto& q=m.orientation;
   const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
   const bool oriented=m.orientation_covariance[0]>=0.&&std::isfinite(norm)&&std::fabs(norm-1.)<.02;
   const double roll=std::atan2(2.*(q.w*q.x+q.y*q.z),1.-2.*(q.x*q.x+q.y*q.y));
   const double pitch=std::asin(clamp(2.*(q.w*q.y-q.z*q.x),-1.,1.));
   runtime_.observer().updateImu(m.angular_velocity.z,m.linear_acceleration.x,m.linear_acceleration.y,
      m.linear_acceleration.z,roll,pitch,m.header.stamp.toSec(),receipt,
      m.angular_velocity_covariance[0]>=0.,m.linear_acceleration_covariance[0]>=0.,oriented);
   imu_frame_status_=config_.observer.imu_frame_verified&&!imu_body_frame_.empty()&&m.header.frame_id==imu_body_frame_?
       "configured_flu_frame_contract":"imu_frame_contract_unverified_or_mismatch";
   if(imu_frame_status_!="configured_flu_frame_contract")runtime_.observer().invalidateImu(imu_frame_status_);
 }
 void ground(const nav_msgs::Odometry& m,double receipt) {
   if(!enabled_)return;const auto& q=m.pose.pose.orientation;
   const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
   const bool heading_valid=std::isfinite(norm)&&std::fabs(norm-1.)<.02;
   const double yaw=std::atan2(2.*(q.w*q.z+q.x*q.y),1.-2.*(q.y*q.y+q.z*q.z));
   runtime_.observer().updateGround(m.twist.twist.linear.x,m.twist.twist.linear.y,yaw,
       m.header.stamp.toSec(),receipt,heading_valid);
   // nav_msgs/Odometry convention normally puts twist in child frame. The
   // original driver instead writes world NED components with empty IDs. Never
   // infer axes from the message type or rotate them using an invented heading.
   ground_frame_status_=config_.observer.ground_frame_verified&&ground_convention_=="world_enu"&&
       !ground_world_frame_.empty()&&!ground_body_frame_.empty()&&m.header.frame_id==ground_world_frame_&&
       m.child_frame_id==ground_body_frame_?"configured_world_enu_body_flu_contract":"ground_twist_frame_contract_unverified_or_mismatch";
   if(ground_frame_status_!="configured_world_enu_body_flu_contract")runtime_.observer().invalidateGround(ground_frame_status_);
 }
 // Called before the EXISTING transportCenterPath(), reusing its SE(2) pose
 // attachment and exact frame transport for the analytic P1 geometry channel.
 template<class Points> bool fillGeometry(const ros::Time& stamp,Points& points) {
   if(!enabled_)return false;
   const move_car::ReferenceInfo* info=nullptr;
   for(auto i=references_.rbegin();i!=references_.rend();++i)
     if((*i)->geometry_stamp_ns==stamp.toNSec()){info=i->get();break;}
   if(!info){reference_reason_="reference_metadata_stamp_mismatch";return false;}
   const double age=(ros::Time::now()-stamp).toSec();
   if(info->schema_version!=1||!info->geometry_valid||info->header.stamp!=stamp||
      info->header.frame_id!="base_link"||info->run_id!=run_id_||
      info->tracking_point!="body_center"||info->samples.size()<2||info->samples.size()>4096||
      !std::isfinite(info->confidence)||info->confidence<=0.||age<-.02||age>1.2) {
     reference_reason_="reference_metadata_invalid_or_stale";return false;
   }
   points.clear();double last=-std::numeric_limits<double>::infinity();
   for(const auto& a:info->samples){
     if(!a.geometry_valid||!std::isfinite(a.s)||!std::isfinite(a.x)||!std::isfinite(a.y)||
        !std::isfinite(a.tangent_yaw)||!std::isfinite(a.curvature)||a.s<=last){
       reference_reason_="reference_sample_invalid";return false;}
     typename Points::value_type p;p.s=a.s;p.x=a.x;p.y=a.y;p.yaw=a.tangent_yaw;p.curvature=a.curvature;
     p.pose_yaw_valid=true;points.push_back(p);last=a.s;
   }
   reference_stamp_=stamp;source_braking_=info->braking_only;
   source_cap_=info->speed_cap_valid?info->speed_cap:config_.solver.limits.speed;
   if(!std::isfinite(source_cap_)||source_cap_<0.){reference_reason_="reference_speed_cap_invalid";return false;}
   reference_reason_="analytic_geometry_matched";return true;
 }
 template<class Points> void setGeometry(const Points& points) {
   if(!enabled_||points.size()<2)return;
   // Project the current body center onto the transported analytic polyline.
   std::size_t nearest=0;double fraction=0.,best=std::numeric_limits<double>::infinity();
   for(std::size_t i=0;i+1<points.size();++i){const auto& a=points[i];const auto& b=points[i+1];
     const double dx=b.x-a.x,dy=b.y-a.y,len=dx*dx+dy*dy;
     const double f=len>1e-12?clamp(-(a.x*dx+a.y*dy)/len,0.,1.):0.;
     const double d=std::hypot(a.x+f*dx,a.y+f*dy);
     if(d<best){best=d;nearest=i;fraction=f;}}
   const auto& a=points[nearest];const auto& b=points[nearest+1];ReferencePoint p;
   p.s=a.s+fraction*(b.s-a.s);p.x=a.x+fraction*(b.x-a.x);p.y=a.y+fraction*(b.y-a.y);
   p.yaw=wrap(a.yaw+fraction*wrap(b.yaw-a.yaw));p.curvature=a.curvature+fraction*(b.curvature-a.curvature);
   spatial_.push_back(p);
   for(std::size_t i=nearest+1;i<points.size();++i)if(points[i].s>spatial_.back().s+1e-9){
     const auto& q=points[i];p.s=q.s;p.x=q.x;p.y=q.y;p.yaw=q.yaw;p.curvature=q.curvature;spatial_.push_back(p);}
 }
 void tick(const Command& actual,double target_speed,double now,bool tracking_valid,bool safety_stop) {
   if(!enabled_)return;const auto start=std::chrono::steady_clock::now();
   const double dt=last_now_>0.?now-last_now_:1./30.;
   if(dt<=0.||dt>.25){runtime_.observer().reset();history_.clear();previous_=actual;previous_acceleration_=0.;}
   SolveRequest request;request.now=now;request.control_dt=clamp(dt,.001,.25);
   request.previous=previous_;request.baseline=actual;request.previous_command_acceleration=previous_acceleration_;
   request.state.x=-config_.solver.model.rear_to_center;
   // Do not silently reuse the legacy mixed speed as an independent measurement.
   request.state.speed=missing();request.state.steering=missing();
   request.speed_cap=std::min(std::max(0.,target_speed),source_cap_);
   request.history=history_;
   // Only schedule needs a starting speed here. Verified observer state replaces
   // the solver initial speed, or the solver explicitly rejects it as invalid.
   const auto& raw=runtime_.observer().rawWheel();
   const double schedule_speed=config_.observer.wheel_calibrated&&raw.time.accepted?
       std::max(0.,raw.value*config_.observer.wheel_scale*config_.observer.wheel_sign):previous_.speed;
   request.reference=FlatnessReferenceModel::buildTimeReference(spatial_,schedule_speed,
       request.speed_cap,reference_stamp_.toSec(),config_.solver,source_braking_);
   const auto result=runtime_.cycle(request,now,tracking_valid&&spatial_.size()>1,safety_stop);
   move_car::AdaptiveControlTelemetry t;t.header.stamp.fromSec(now);t.schema_version=1;t.run_id=run_id_;t.config_hash=hash_;
   t.shadow_only=true;t.requested_mode=result.requested_mode;t.selected_owner=result.owner;t.actual_owner="legacy_slqr";
   t.reason=result.reason;t.terrain=result.terrain;t.estimated_terrain=result.terrain_estimate.label;
   t.terrain_confidence=result.terrain_estimate.confidence;t.terrain_reason=result.terrain_estimate.reason;
   for(int i=0;i<4;++i)t.terrain_probabilities[i]=result.terrain_estimate.probabilities[i];
   const auto& o=result.observation;
   const Estimate* estimates[]={&o.speed,&o.slip,&o.yaw_rate,&o.yaw_reference,&o.yaw_residual,&o.steering_tau,
       &o.steering_bias,&o.sideslip,&o.roughness,&o.yaw_gain,&o.longitudinal_residual,&o.lateral_residual,&o.steering};
   const char* names[]={"speed","slip","yaw_rate","yaw_reference","yaw_residual","steering_tau",
       "steering_bias","sideslip","roughness","yaw_gain","longitudinal_residual","lateral_residual","steering"};
   for(int i=0;i<13;++i){move_car::EstimatedValue e;e.value=estimates[i]->value;e.valid=estimates[i]->valid;
     e.confidence=estimates[i]->confidence;if(std::isfinite(estimates[i]->stamp)&&estimates[i]->stamp>0.)e.stamp.fromSec(estimates[i]->stamp);
     e.reason=estimates[i]->reason;t.estimate_names.push_back(names[i]);t.estimates.push_back(e);}
   t.actual_speed=actual.speed;t.actual_steering=actual.steering;
   t.candidate_speed=result.command.speed;t.candidate_steering=result.command.steering;
   const auto& s=result.solve;t.feasible=s.feasible;t.timed_out=s.timed_out;t.warm_started=s.warm_started;
   t.iterations=s.iterations;t.evaluations=s.evaluations;t.decision_dimension=s.decision_dimension;
   t.objective=s.objective;t.initial_objective=s.initial_objective;t.max_violation=s.max_violation;t.solve_ms=s.solve_ms;
   t.effective_yaw_gain=result.effective.model.yaw_gain;t.effective_steering_tau=result.effective.model.steering_tau;
   t.effective_speed_cap=result.effective.limits.speed;t.effective_lateral_acceleration=result.effective.limits.lateral_acceleration;
   const auto& w=result.effective.weights;t.effective_weights[0]=w.lateral;t.effective_weights[1]=w.heading;
   t.effective_weights[2]=w.speed;t.effective_weights[3]=w.smoothness;
   for(int i=0;i<s.count;++i){const auto& z=s.predicted[i];
     for(double v:{z.x,z.y,z.yaw,z.speed,z.steering,z.acceleration})t.predicted_states.push_back(v);
     if(i+1<s.count){t.predicted_commands.push_back(s.controls[i].speed);t.predicted_commands.push_back(s.controls[i].steering);}}
   t.reference_stamp=reference_stamp_;t.reference_reason=spatial_.empty()?reference_reason_:request.reference.reason;
   t.shadow_cycle_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();pub_.publish(t);
   move_car::ResearchValidationTelemetry v;v.header=t.header;v.run_id=run_id_;v.config_hash=hash_;
   v.ground_twist_convention=ground_convention_;v.ground_frame_status=ground_frame_status_;v.imu_frame_status=imu_frame_status_;
   v.source_timestamps_verified=config_.observer.source_timestamps_verified;v.steering_delay_verified=config_.observer.steering_delay_verified;
   v.wheel_freshness_status="receipt_only_CAN_cache_republication_cannot_detect_stale_acquisition";
   v.source_time_status=v.source_timestamps_verified?"acquisition_time_verified_by_configuration":"driver_receipt_time_not_verified_acquisition";
   const auto& id=runtime_.observer().identificationDiagnostics();
   v.yaw_identification_samples=id.yaw_samples;v.yaw_identification_rejected=id.yaw_rejected;
   v.yaw_identification_information=id.yaw_information;v.yaw_regressor_variance=id.yaw_regressor_variance;
   v.yaw_innovation_rms=id.yaw_innovation_rms;v.yaw_observability_reason=o.yaw_gain.reason;
   v.steering_identification_samples=id.steering_samples;v.steering_identification_rejected=id.steering_rejected;
   v.steering_regressor_variance=id.steering_regressor_variance;v.steering_innovation_rms=id.steering_innovation_rms;
   v.steering_observability_reason=o.steering_tau.reason;
   v.budget_limited=s.budget_limited;v.early_stopped=s.early_stopped;v.warm_available=s.warm_available;
   v.model_adaptation_active=result.model_adaptation_active;v.weight_adaptation_active=result.weight_adaptation_active;
   v.adaptation_risk=result.adaptation_risk;v.adaptation_reason=result.adaptation_reason;
   v.evidence_names={"yaw_gain","steering_tau","yaw_residual","longitudinal_residual","lateral_residual"};
   for(const auto& e:result.adaptation){v.evidence_samples.push_back(e.samples);v.nominal_mse.push_back(e.nominal_mse);
     v.candidate_mse.push_back(e.candidate_mse);v.adaptation_active.push_back(e.active);v.adaptation_reasons.push_back(e.reason);}
   v.detailed_profiling_enabled=config_.solver.profile;v.initialization_ms=s.profile.initialization_ms;v.reference_ms=s.profile.reference_ms;
   v.prediction_ms=s.profile.prediction_ms;v.derivative_ms=s.profile.derivative_ms;v.constraint_cost_ms=s.profile.constraint_cost_ms;
   v.direction_ms=s.profile.direction_ms;v.candidate_selection_ms=s.profile.candidate_selection_ms;v.other_ms=s.profile.other_ms;
   validation_pub_.publish(v);
   // Actual command history, never the unapplied counterfactual NMPC command.
   runtime_.observer().recordCommand(actual,now);history_.push_back({now,actual});
   if(history_.size()>120)history_.erase(history_.begin());
   previous_acceleration_=(actual.speed-previous_.speed)/request.control_dt;previous_=actual;last_now_=now;
 }
private:
 void reference(const move_car::ReferenceInfo::ConstPtr& m){references_.push_back(m);while(references_.size()>8)references_.pop_front();}
 void mode(const std_msgs::String::ConstPtr& m){if(!runtime_.requestMode(m->data))ROS_WARN("Rejected research mode %s",m->data.c_str());}
 void terrain(const std_msgs::String::ConstPtr& m){if(!runtime_.requestTerrain(m->data,ros::Time::now().toSec()))ROS_WARN("Rejected research terrain %s",m->data.c_str());}
 bool enabled_=false,source_braking_=false;RuntimeConfig config_;AdaptiveRuntime runtime_;
 ros::Publisher pub_,validation_pub_;ros::Subscriber reference_sub_,mode_sub_,terrain_sub_;
 std::deque<move_car::ReferenceInfo::ConstPtr> references_;std::vector<ReferencePoint> spatial_;
 std::vector<CommandSample> history_;Command previous_;double previous_acceleration_=0.,last_now_=0.,source_cap_=.1;
 ros::Time reference_stamp_;std::string run_id_,hash_,reference_reason_="unobserved";
 std::string ground_convention_="unverified",ground_world_frame_,ground_body_frame_,imu_body_frame_;
 std::string ground_frame_status_="unobserved",imu_frame_status_="unobserved";
};
}
