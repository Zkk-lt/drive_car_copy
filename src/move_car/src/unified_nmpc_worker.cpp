#include "control/p6_observation_hold.h"
#include "control/p7_straight_tracking_reference.h"
#include "control/p7_turn_timing.h"
#include "control/qualified_steering_sample.h"
#include <ros/ros.h>
#include <move_car/P5ControlRequest.h>
#include <move_car/P5ControlCandidate.h>
#include <move_car/P5PublishedCommand.h>
#include <std_msgs/String.h>
#include "control/research_ros_bridge.h"
#include "control/p5_nominal_propagation.h"
#include "control/p6_ros_bridge.h"
#include <ros/callback_queue.h>
#include <move_car/car_parameter.h>
#include <sensor_msgs/Imu.h>
#include <mutex>
#include <memory>
#include <deque>
class UnifiedNMPCWorker {
 using Request=move_car::P5ControlRequest;
 ros::NodeHandle nh_,pnh_{"~"};ros::Publisher candidates_,diagnostic_;ros::Subscriber request_,published_;
 research_control::RuntimeConfig config_;research_control::AdaptiveRuntime runtime_;
 std::deque<move_car::P5PublishedCommand> history_;
 std::string hash_,run_id_,last_mode_,backend_;std::uint64_t last_cycle_=0;
 bool history_valid_=false;double recovery_from_=0.;
 unified_control::SupervisedGuard p7_guard_;
 bool p7_reference_revalidation_=false;
 bool p6_qualified_steering_=false;research_control::QualifiedSteeringSample p6_steering_sample_;
 bool p6_separated_speed_target_=false,p6_speed_preview_=false,p6_synchronized_preview_=false,p6_terminal_approach_=false;
 bool p6_actuator_aligned_preview_=false;
 bool p7_bounded_slqr_feedback_=false,p6_nominal_observation_hold_=false;
 bool p6_can_only_feedback_=false;
 bool p7_qualify_chassis_motion_=false;
 std::string p6_flat_strategy_diagnostic_;
 bool p6_shadow_only_strategy_=false,p7_candidate_control_trial_=false,p7_straight_tracking_reference_=false;
 bool p7_supported_course_=false;
 double p7_straight_yaw_damping_weight_=0.;
 double p7_turn_speed_reserve_=0.,p7_turn_release_weight_=0.;
 bool p6_enabled_=false,p6_updates_enabled_=false;research_control::P6LayeredConfig p6_config_;research_control::P6SceneManager p6_scenes_;research_control::P6Controller p6_controller_;
 ros::Publisher p6_candidates_;ros::Subscriber p6_request_;
 double p6_previous_total_ms_=0.,p6_previous_publish_ms_=0.;
 ros::CallbackQueue p6_observation_queue_;std::unique_ptr<ros::AsyncSpinner> p6_observation_spinner_;ros::Subscriber p6_wheel_sub_,p6_imu_sub_;
 std::mutex p6_observation_mutex_;move_car::car_parameter p6_live_wheel_;sensor_msgs::Imu p6_live_imu_;ros::Time p6_live_wheel_receipt_,p6_live_imu_receipt_;
 void p6Wheel(const move_car::car_parameter::ConstPtr&m){
  std::lock_guard<std::mutex>lock(p6_observation_mutex_);
  const auto receipt=ros::Time::now();
  const double steering=m->turn_angle*(p7_guard_.enabled?p7_guard_.steering_sign:1.);
  const double rate=config_.solver.limits.steering_rate;
  if(p7_qualify_chassis_motion_&&p6_steering_sample_.discontinuousZeroBatteryMotion(
     m->battery_level,steering,receipt.toSec(),rate))return;
  p6_live_wheel_=*m;p6_live_wheel_receipt_=receipt;
  if(p6_qualified_steering_)p6_steering_sample_.update(steering,receipt.toSec(),rate);
 }
 void p6Imu(const sensor_msgs::Imu::ConstPtr&m){std::lock_guard<std::mutex>lock(p6_observation_mutex_);p6_live_imu_=*m;p6_live_imu_receipt_=ros::Time::now();}
 bool p6FreshState(const move_car::P6ControlRequest&wrapper,const research_control::P6EffectiveParameters&effective,
   const research_control::SolveRequest&solve,research_control::State&state,ros::Time&cutoff,bool*unchanged=nullptr,std::string*why=nullptr){
  auto reject=[&](const char*reason){if(why)*why=reason;return false;};
  if(unchanged)*unchanged=false;
  // Explicit synthetic or operator-confirmed SI/frame engineering contract.
  // Receipt-domain short-time feedback is not physical sampling calibration,
  // independent ground velocity, or a long-term global pose estimate.
  if((!wrapper.synthetic&&!p7_guard_.configured())||!wrapper.wheel_speed.unit_verified||!wrapper.imu_frame_verified)return reject("feedback_mapping_contract_invalid");
  move_car::car_parameter wheel;sensor_msgs::Imu imu;ros::Time wt,it;research_control::QualifiedSteeringSample steering_sample;
  {std::unique_lock<std::mutex>lock(p6_observation_mutex_,std::try_to_lock);if(!lock.owns_lock())return reject("feedback_observation_lock_busy");wheel=p6_live_wheel_;imu=p6_live_imu_;wt=p6_live_wheel_receipt_;it=p6_live_imu_receipt_;steering_sample=p6_steering_sample_;}
  const auto&q=wrapper.base;cutoff=std::max(wt,it);const auto available=ros::Time::now();
  // Compare the shared IMU source identity and actual wheel/angle payload,
  // not receipt equality across subscribers. CAN sample identity stays unknown.
  const double cached_speed=wheel.back_wheel_speed*(p7_guard_.enabled?p7_guard_.wheel_scale:.001);
  const double cached_steering=p6_qualified_steering_?steering_sample.accepted:
    wheel.turn_angle*(p7_guard_.enabled?p7_guard_.steering_sign:1.);
  if(p6_qualified_steering_&&(!steering_sample.usable(available.toSec())||
      steering_sample.accepted_stamp>cutoff.toSec()||
      cutoff.toSec()-steering_sample.accepted_stamp>.06))
    return reject("feedback_qualified_steering_stale_or_skewed");
  if(unchanged&&imu.header.frame_id==wrapper.yaw_rate.frame_id&&finite(cached_speed)&&finite(cached_steering)&&
     std::abs(cached_speed-q.state[3])<=1e-12&&std::abs(cached_steering-q.state[4])<=1e-12&&
     research_control::p6NominalHoldTiming(q.wheel_receipt_stamp.toSec(),q.imu_receipt_stamp.toSec(),q.imu_source_stamp.toSec(),
       wt.toSec(),it.toSec(),imu.header.stamp.toSec(),q.header.stamp.toSec(),available.toSec(),outApply(q).toSec()))*unchanged=true;
  // Preserve every original gate; expose why no new causal state was usable.
  const char* timing=research_control::p6FeedbackTimingRejection(q.state_observation_stamp.toSec(),q.header.stamp.toSec(),
    q.imu_source_stamp.toSec(),wt.toSec(),it.toSec(),imu.header.stamp.toSec(),available.toSec(),outApply(q).toSec());
  const bool can_only=p6_can_only_feedback_&&p6_qualified_steering_&&timing&&
    imu.header.stamp==wrapper.yaw_rate.source_stamp&&
    wrapper.yaw_rate.source_stamp.toSec()==q.imu_source_stamp.toSec()&&imu.angular_velocity.z==q.yaw_rate&&
    research_control::p6CanOnlyTiming(q.state_observation_stamp.toSec(),q.header.stamp.toSec(),
      q.imu_receipt_stamp.toSec(),q.imu_source_stamp.toSec(),wt.toSec(),steering_sample.accepted_stamp,
      it.toSec(),imu.header.stamp.toSec(),available.toSec(),outApply(q).toSec());
  if(timing&&!can_only)return reject(timing);
  if(imu.header.frame_id!=wrapper.yaw_rate.frame_id)return reject("feedback_frame_mismatch");
  if(can_only)cutoff=wt;
  const double dt=(cutoff-q.state_observation_stamp).toSec();if(dt<=0.||dt>.075)return reject("feedback_observation_interval_invalid");
  const double v0=q.state[3],v1=wheel.back_wheel_speed*(p7_guard_.enabled?p7_guard_.wheel_scale:.001),r0=q.yaw_rate,r1=imu.angular_velocity.z;
  if(!finite(v1)||!finite(r1)||!finite(wheel.turn_angle))return reject("feedback_nonfinite_measurement");
  if(p7_guard_.enabled&&(v1<0.||std::abs(v1)>2.||std::abs(r1)>1.5))return reject("feedback_measurement_outside_contract");
  research_control::State measured{q.state[0],q.state[1],q.state[2],v1,
    cached_steering,0.};
  const auto& model=effective.solver.model;
  const auto& limits=effective.solver.limits;
  double position_uncertainty=q.position_uncertainty,yaw_uncertainty=q.yaw_uncertainty;
  if(can_only){
   // The same IMU sample cannot update pose twice. Propagate pose using
   // issued commands, then update only the two CAN channels at their receipt.
   const research_control::State frozen{q.state[0],q.state[1],q.state[2],v0,q.state[4],q.state[5]};
   const auto causal_end=std::upper_bound(solve.history.begin(),solve.history.end(),cutoff.toSec(),
     [](double t,const research_control::CommandSample& h){return t<h.stamp;});
   std::vector<research_control::CommandSample> causal;
   const auto* known=&solve.history;
   if(causal_end!=solve.history.end()){
    causal.assign(solve.history.begin(),causal_end);known=&causal;
   }
   const auto prediction=research_control::p5PropagateNominal(frozen,q.state_observation_stamp.toSec(),
     cutoff.toSec(),model,limits,*known,position_uncertainty,yaw_uncertainty,.04,.08,cutoff.toSec());
   if(!prediction.valid)return reject("feedback_can_only_prior_projection_invalid");
   measured=prediction.state;
   position_uncertainty=prediction.position_uncertainty+dt*std::abs(v1-measured.speed);
   const double nominal_yaw_rate=model.yaw_gain*measured.speed*std::tan(measured.steering)/model.wheelbase;
   const double updated_yaw_rate=model.yaw_gain*v1*std::tan(cached_steering)/model.wheelbase;
   if(!finite(updated_yaw_rate)||v1>limits.speed+1e-7||std::abs(cached_steering)>limits.steering+1e-7||
      std::abs(v1*(updated_yaw_rate+model.yaw_residual))>limits.lateral_acceleration+1e-7)
    return reject("feedback_can_only_state_outside_limits");
   yaw_uncertainty=prediction.yaw_uncertainty+dt*std::abs(updated_yaw_rate-nominal_yaw_rate);
   measured.speed=v1;measured.steering=cached_steering;
  }else{
   for(int j=0;j<4;++j){const double f=(j+.5)/4.,h=dt/4.,v=v0+f*(v1-v0),r=r0+f*(r1-r0),mid=measured.yaw+.5*h*r;
    measured.x+=h*v*std::cos(mid);measured.y+=h*v*std::sin(mid);measured.yaw=research_control::wrap(measured.yaw+h*r);}
  }
  const auto command=research_control::P6PolicyValidator::held(solve.history,cutoff.toSec()-model.speed_delay);
  measured.acceleration=(model.speed_gain*command.speed-measured.speed)/model.speed_tau+model.longitudinal_residual;
  // Measurement cutoff and command knowledge time are different. A command
  // actually published after this measurement but known now may participate
  // in the forward prediction; it must not be presented as an older sensor.
  const double command_knowledge=std::min(available.toSec(),outApply(q).toSec());
  const auto projected=research_control::p5PropagateNominal(measured,cutoff.toSec(),outApply(q).toSec(),model,limits,solve.history,position_uncertainty,yaw_uncertainty,.04,.08,command_knowledge);
  if(!projected.valid||projected.position_uncertainty>.03||projected.yaw_uncertainty>.1)return reject("feedback_projection_or_uncertainty_invalid");state=projected.state;
  if(why)*why=can_only?"feedback_can_receipt_only_nominal_pose":
    (cutoff<=q.header.stamp?"fresh_after_observation_before_request":"fresh_state_verified");return true;
 }
 static ros::Time outApply(const Request&q){return q.header.stamp+ros::Duration(q.control_dt);}
 static bool finite(double x){return std::isfinite(x);}
 void published(const move_car::P5PublishedCommand::ConstPtr&m){
  if(m->run_id!=run_id_||m->config_hash!=hash_||m->schema_version!=1)return;
  if(!m->publish_returned||!m->history_committed){history_valid_=false;recovery_from_=0.;return;}
  if(m->physical_execution_confirmed||m->header.stamp.isZero()||!finite(m->speed)||!finite(m->steering)||!finite(m->acceleration)){history_valid_=false;return;}
  if(!history_.empty()){
   const auto&b=history_.back();
   if(m->sequence==b.sequence){if(m->epoch!=b.epoch||m->speed!=b.speed||m->steering!=b.steering||m->header.stamp!=b.header.stamp)history_valid_=false;return;}
   if(m->epoch!=b.epoch||m->sequence!=b.sequence+1||m->header.stamp<=b.header.stamp||(m->header.stamp-b.header.stamp).toSec()>.12){history_valid_=false;recovery_from_=m->header.stamp.toSec();}
   if(m->sequence<=b.sequence)return;
  }else recovery_from_=m->header.stamp.toSec();
  runtime_.observer().recordCommand({m->speed,m->steering},m->header.stamp.toSec());
  history_.push_back(*m);while(history_.size()>256)history_.pop_front();
  if(!history_valid_){if(recovery_from_==0.)recovery_from_=m->header.stamp.toSec();if(m->header.stamp.toSec()-recovery_from_>=.75)history_valid_=true;}
 }
 bool stateFor(const Request&q,research_control::State&s,std::string&why){
  const double observed=q.state_observation_stamp.toSec(),snapshot=q.header.stamp.toSec();
  if(!q.nominal_state_contract||!q.state_valid||!q.yaw_rate_valid||!finite(q.state_confidence)||q.state_confidence<=0.||q.state_confidence>1.||observed<=0.||snapshot<observed||snapshot-observed>.025||q.reference_frame_stamp!=q.state_observation_stamp){why="invalid_nominal_local_state_contract";return false;}
  for(double v:q.state)if(!finite(v)){why="nonfinite_local_state";return false;}
  double oldest=observed,newest=0.;for(const auto&t:{q.wheel_receipt_stamp,q.steering_receipt_stamp,q.imu_receipt_stamp}){const double v=t.toSec();if(v<=0.||v>observed||observed-v>.12){why="stale_or_future_sensor_receipt";return false;}oldest=std::min(oldest,v);newest=std::max(newest,v);}
  if(newest-oldest>.06||q.imu_source_stamp.isZero()||q.imu_source_stamp>q.imu_receipt_stamp||(q.state_observation_stamp-q.imu_source_stamp).toSec()>.12){why="source_time_or_sensor_skew_invalid";return false;}
  s={q.state[0],q.state[1],q.state[2],q.state[3],q.state[4],q.state[5]};return true;
 }
 bool historyFor(const Request&q,std::vector<research_control::CommandSample>&out,std::string&why){
  if(!q.history_valid||!history_valid_||history_.empty()){why="actual_history_unavailable";return false;}
  const auto&last=history_.back();
  if(last.sequence!=q.previous_sequence||last.epoch!=q.epoch||last.header.stamp!=q.previous_publish_stamp||last.header.stamp>q.header.stamp||std::fabs(last.speed-q.previous_speed)>1e-9||std::fabs(last.steering-q.previous_steering)>1e-9||std::fabs(last.acceleration-q.previous_acceleration)>1e-9||q.state_history_sequence>=q.previous_sequence){why="actual_history_context_mismatch";return false;}
  for(const auto&h:history_){if(h.header.stamp>q.header.stamp){why="future_history";return false;}out.push_back({h.header.stamp.toSec(),{h.speed,h.steering}});}return true;
 }
 bool geometryFor(const Request&q,const research_control::State&s,std::vector<research_control::ReferencePoint>&out,std::string&why,const research_control::Model* model=nullptr,bool* derived_curvature=nullptr){
  if(!q.reference_valid||q.reference.size()<2||q.reference.size()>1024||q.reference_stamp.isZero()||q.reference_stamp>q.state_observation_stamp){why="invalid_reference";return false;}
  double last=-INFINITY;std::vector<research_control::ReferencePoint>p;p.reserve(q.reference.size());
  for(const auto&a:q.reference){if(!a.geometry_valid||!finite(a.s)||!finite(a.x)||!finite(a.y)||!finite(a.tangent_yaw)||!finite(a.curvature)||a.s<=last){why="invalid_reference_geometry";return false;}research_control::ReferencePoint b;b.s=a.s;b.x=a.x;b.y=a.y;b.yaw=a.tangent_yaw;b.curvature=a.curvature;p.push_back(b);last=a.s;}
  // Geometry-only P5 paths can carry zero-initialized curvature even on bends.
  // Derive it only in this research copy, before nearest-point cropping.
  const bool derived=research_control::FlatnessReferenceModel::recoverMissingSpatialCurvature(p);
  if(derived_curvature)*derived_curvature=derived;
  const auto& geometry_model=model?*model:config_.solver.model;
  return research_control::FlatnessReferenceModel::projectSpatialSuffix(p,s,geometry_model,out);
 }
 void request(const Request::ConstPtr&q){
  const auto start=std::chrono::steady_clock::now();move_car::P5ControlCandidate out;
  out.schema_version=1;out.run_id=q->run_id;out.config_hash=hash_;out.mode=q->mode;out.backend=backend_;out.epoch=q->epoch;out.cycle=q->cycle;out.reference_revision=q->reference_revision;out.config_revision=q->config_revision;out.previous_sequence=q->previous_sequence;out.snapshot_stamp=q->header.stamp;
  out.actual_previous_speed=q->previous_speed;out.actual_previous_steering=q->previous_steering;out.actual_previous_acceleration=q->previous_acceleration;
  auto finish=[&](const std::string&why){out.reason=why;out.header.stamp=ros::Time::now();out.header.frame_id=q->header.frame_id;out.worker_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();if(out.valid&&(out.worker_ms>=std::min(8.,q->solve_budget_ms)||out.header.stamp>out.valid_until)){out.valid=false;out.timed_out=true;out.reason="worker_end_to_end_deadline";}candidates_.publish(out);std_msgs::String d;d.data=out.reason;diagnostic_.publish(d);};
  const double age=(ros::Time::now()-q->header.stamp).toSec();
  if(q->schema_version!=1||q->run_id!=run_id_||q->config_hash!=hash_||q->epoch==0||q->cycle==0||q->reference_revision==0||q->config_revision==0||age<0.||age>.075||q->header.stamp.isZero()||!finite(q->control_dt)||q->control_dt<.001||q->control_dt>.05||!finite(q->speed_cap)||q->speed_cap<0.||!finite(q->solve_budget_ms)||q->solve_budget_ms<=0.){finish("invalid_stale_or_mismatched_request");return;}
  if(q->cycle<=last_cycle_){finish("duplicate_or_out_of_order_request");return;}last_cycle_=q->cycle;
  if(q->hard_stop){finish("hard_stop_no_candidate");return;}
  if(q->mode!="nmpc_shadow"&&q->mode!="nmpc_fixed"&&q->mode!="nmpc_adaptive"&&q->mode!="auto_fallback"){finish("mode_does_not_request_nmpc");return;}
  research_control::Limits external;external.speed=q->max_speed;external.steering=q->max_steering;external.steering_rate=q->max_steering_rate;external.acceleration=q->max_acceleration;external.deceleration=q->max_deceleration;external.jerk=q->max_jerk;external.lateral_acceleration=q->max_lateral_acceleration;
  if(!unified_control::validLimits(external)){finish("invalid_external_safety_limits");return;}
  auto bounded=config_.solver;bounded.limits=unified_control::intersectLimits(bounded.limits,external);bounded.budget_ms=std::min(bounded.budget_ms,q->solve_budget_ms);
  out.expected_apply_stamp=q->header.stamp+ros::Duration(q->control_dt);out.valid_until=out.expected_apply_stamp+ros::Duration(.04);
  std::string why;research_control::SolveRequest solve;
  if(!stateFor(*q,solve.state,why)){finish(why);return;}out.state_valid=true;
  if(!historyFor(*q,solve.history,why)){finish(why);return;}out.history_valid=true;
  const auto propagated=research_control::p5PropagateNominal(solve.state,q->state_observation_stamp.toSec(),out.expected_apply_stamp.toSec(),bounded.model,bounded.limits,solve.history,q->position_uncertainty,q->yaw_uncertainty,.04,.08,q->header.stamp.toSec());
  if(!propagated.valid){finish(propagated.reason);return;}out.position_uncertainty=propagated.position_uncertainty;out.yaw_uncertainty=propagated.yaw_uncertainty;
  if(out.position_uncertainty>.03||out.yaw_uncertainty>.10){finish("propagated_uncertainty_limit");return;}
  solve.state=propagated.state;out.propagated_speed=solve.state.speed;out.propagated_steering=solve.state.steering;out.propagated_yaw_rate=bounded.model.yaw_gain*solve.state.speed*std::tan(solve.state.steering)/bounded.model.wheelbase+bounded.model.yaw_residual;out.state_pair_validated=true;
  std::vector<research_control::ReferencePoint>spatial;if(!geometryFor(*q,solve.state,spatial,why)){finish(why);return;}out.reference_valid=true;
  solve.now=out.expected_apply_stamp.toSec();solve.control_dt=q->control_dt;solve.speed_cap=std::min(q->speed_cap,bounded.limits.speed);solve.previous={q->previous_speed,q->previous_steering};solve.previous_command_acceleration=q->previous_acceleration;solve.baseline={q->baseline_speed,q->baseline_steering};
  solve.reference=research_control::FlatnessReferenceModel::buildTimeReference(spatial,solve.state.speed,solve.speed_cap,solve.now,bounded,q->reference_braking);
  solve.force_timeout=q->force_timeout;solve.force_infeasible=q->force_infeasible;solve.force_numerical_failure=q->force_numerical_failure;
  const std::string mode=q->mode=="nmpc_adaptive"?"nmpc_full_adaptive":(q->mode=="auto_fallback"?"auto_fallback":"nmpc_fixed");if(mode!=last_mode_){runtime_.requestMode(mode);last_mode_=mode;}
  if(!runtime_.requestTerrain(q->terrain,solve.now)){finish("invalid_terrain_label");return;}
  // Raw observers are not upgraded by nominal state flags. No fictitious ground speed.
  auto& observer=runtime_.observer();
  if(q->wheel_calibrated&&config_.observer.wheel_calibrated)
   observer.updateWheel(q->state[3]/(config_.observer.wheel_scale*config_.observer.wheel_sign),0.,q->wheel_receipt_stamp.toSec());
  if(q->steering_calibrated&&config_.observer.steering_calibrated)
   observer.updateSteering(q->state[4]/(config_.observer.steering_scale*config_.observer.steering_sign),0.,q->steering_receipt_stamp.toSec());
  if(q->imu_frame_verified&&config_.observer.imu_frame_verified)
   observer.updateImu(q->yaw_rate/(config_.observer.gyro_scale*config_.observer.gyro_sign),research_control::missing(),research_control::missing(),research_control::missing(),research_control::missing(),research_control::missing(),q->imu_source_stamp.toSec(),q->imu_receipt_stamp.toSec(),true,false,false);
  const auto result=runtime_.cycle(solve,solve.now,solve.reference.valid,false,true,&external);
  const auto&s=result.solve;out.speed=s.command.speed;out.steering=s.command.steering;out.acceleration=s.command_acceleration;out.solve_ms=s.solve_ms;out.objective=s.objective;out.max_violation=s.max_violation;out.iterations=s.iterations;out.evaluations=s.evaluations;out.decision_dimension=s.decision_dimension;out.feasible=s.feasible;out.timed_out=s.timed_out;out.numerical_failure=q->force_numerical_failure||s.reason.find("numerical")!=std::string::npos;
  out.model_adaptation_active=result.model_adaptation_active;out.weight_adaptation_active=result.weight_adaptation_active;out.complete_substep_constraints=s.ok&&s.feasible&&!s.timed_out&&finite(s.max_violation)&&s.max_violation<=1e-7;out.valid=out.complete_substep_constraints&&result.nmpc_selected&&finite(out.speed)&&finite(out.steering)&&finite(out.acceleration);
  for(int i=0;i<s.count;++i){const auto&z=s.predicted[i];for(double v:{z.x,z.y,z.yaw,z.speed,z.steering,z.acceleration})out.predicted_states.push_back(v);if(i+1<s.count){out.predicted_commands.push_back(s.controls[i].speed);out.predicted_commands.push_back(s.controls[i].steering);}}
  finish(s.reason+";"+result.adaptation_reason+";state="+propagated.reason);
 }
 void p6Request(const move_car::P6ControlRequest::ConstPtr&wrapper){
  const auto start=std::chrono::steady_clock::now();const auto&q=wrapper->base;
  move_car::P6ControlCandidate result;auto&out=result.base;result.schema_version=2;result.session_id=wrapper->session_id;result.algorithm_mode=wrapper->algorithm_mode;result.computation_mode=wrapper->computation_mode;
  result.observation_steady_ns=wrapper->observation_steady_ns;result.request_steady_ns=wrapper->request_steady_ns;result.worker_start_steady_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(start.time_since_epoch()).count();
  out.schema_version=1;out.run_id=q.run_id;out.config_hash=hash_;out.mode=q.mode;out.backend=wrapper->computation_mode;
  out.epoch=q.epoch;out.cycle=q.cycle;out.reference_revision=q.reference_revision;out.config_revision=q.config_revision;out.previous_sequence=q.previous_sequence;
  out.snapshot_stamp=q.header.stamp;result.optimization_stamp=q.state_observation_stamp;
  out.actual_previous_speed=q.previous_speed;out.actual_previous_steering=q.previous_steering;out.actual_previous_acceleration=q.previous_acceleration;
  bool derived_curvature=false, actuator_aligned_schedule=false;
  auto finish=[&](const std::string&why){out.reason=why+std::string(";reference_curvature=")+(derived_curvature?"derived_local_geometry":"supplied_or_not_built");out.header.stamp=ros::Time::now();out.header.frame_id=q.header.frame_id;
   if(actuator_aligned_schedule)out.reason+=";reference_schedule=actuator_aligned";
   out.worker_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
   if(out.valid&&(out.worker_ms>=std::min(8.,q.solve_budget_ms)||out.header.stamp>out.valid_until)){
    out.valid=out.feasible=false;out.timed_out=true;out.reason="p6_complete_worker_deadline";
   }
   result.worker_finish_steady_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
   result.previous_worker_total_ms=p6_previous_total_ms_;result.previous_candidate_publish_ms=p6_previous_publish_ms_;
   const auto publication_start=std::chrono::steady_clock::now();p6_candidates_.publish(result);
   p6_previous_publish_ms_=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-publication_start).count();
   p6_previous_total_ms_=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
  };
  const double age=(ros::Time::now()-q.header.stamp).toSec();
  if(p6_shadow_only_strategy_&&wrapper->algorithm_mode!="nmpc_shadow"){
    finish("p6_localjet_strategy_requires_shadow");return;
  }
  if(p7_candidate_control_trial_&&(wrapper->algorithm_mode!="flat_nmpc_slqr"||wrapper->computation_mode!="flat_nmpc_slqr")){
    finish("p7_candidate_control_trial_mode_mismatch");return;
  }
  if((wrapper->schema_version!=2&&wrapper->schema_version!=3)||wrapper->observation_steady_ns==0||wrapper->request_steady_ns<wrapper->observation_steady_ns||wrapper->session_id.empty()||!research_control::p6KnownMode(wrapper->algorithm_mode)||!research_control::p6KnownMode(wrapper->computation_mode)||(wrapper->algorithm_mode!="nmpc_shadow"&&wrapper->computation_mode!=wrapper->algorithm_mode)||q.schema_version!=1||q.run_id!=run_id_||q.config_hash!=hash_||q.epoch==0||q.reference_revision==0||q.config_revision==0||q.cycle<=last_cycle_||age<0.||age>.075||!finite(q.control_dt)||q.control_dt<.001||q.control_dt>.05||!finite(q.solve_budget_ms)||q.solve_budget_ms<=0.){finish("p6_request_identity_or_time_invalid");return;}
  if(p7_guard_.enabled&&(wrapper->synthetic||q.terrain!="smooth"||!unified_control::p7FixedMode(wrapper->algorithm_mode)||
     !unified_control::p7FixedMode(wrapper->computation_mode)||q.force_timeout||q.force_infeasible||q.force_numerical_failure||
     !finite(q.speed_cap)||q.speed_cap<0.||q.speed_cap>p7_guard_.speed_cap||q.state[3]<0.||
     wrapper->yaw_rate.frame_id!=p7_guard_.imu_frame||!wrapper->imu_frame_verified)){
    finish("p7_real_nominal_request_contract_invalid");return;
  }
  if(wrapper->speed_target_separated&&(!p6_separated_speed_target_||wrapper->schema_version!=3||
     !finite(wrapper->target_speed)||wrapper->target_speed<0.||wrapper->target_speed>q.speed_cap||
     wrapper->speed_policy_reason!="performance_target_with_unchanged_safety_caps")){
    finish("p6_speed_target_contract_invalid");return;
  }
  last_cycle_=q.cycle;if(q.hard_stop){finish("p6_existing_safety_stop");return;}
  research_control::SolveRequest solve;std::string why;
  if(!stateFor(q,solve.state,why)){finish(why);return;}out.state_valid=true;
  if(!historyFor(q,solve.history,why)){finish(why);return;}out.history_valid=true;solve.published_history_contiguous=true;
  for(std::size_t i=1;i<history_.size();++i)if(history_[i].epoch!=history_[i-1].epoch||history_[i].sequence!=history_[i-1].sequence+1||
     (history_[i].header.stamp-history_[i-1].header.stamp).toSec()<=0.||(history_[i].header.stamp-history_[i-1].header.stamp).toSec()>.12)solve.published_history_contiguous=false;
  std::vector<research_control::p6::PublishedSample> causal_history;
  // The current publication may follow the frozen measurement. It is allowed
  // for future prediction, but not for a response fit at the earlier cutoff.
  for(const auto&h:history_)if(h.header.stamp<=q.state_observation_stamp){research_control::p6::PublishedSample p;p.sequence=h.sequence;p.stamp=h.header.stamp.toSec();p.command={h.speed,h.steering};p.published=h.publish_returned;p.committed=h.history_committed;causal_history.push_back(p);}
  auto nominal=p6_scenes_.update(p6_config_,q.terrain,q.state_observation_stamp.toSec());
  auto response_policy=research_control::p6ResponsePolicy(p6_config_);
  if(!p6_updates_enabled_){response_policy.steering.enabled=response_policy.yaw.enabled=response_policy.longitudinal.enabled=false;}
  auto parameters=p6_controller_.parameters(nominal,research_control::p6Sensors(*wrapper),causal_history,response_policy,wrapper->computation_mode);
  parameters.history_base_sequence=q.previous_sequence;
  research_control::Limits external;external.speed=q.max_speed;external.steering=q.max_steering;external.steering_rate=q.max_steering_rate;external.acceleration=q.max_acceleration;external.deceleration=q.max_deceleration;external.jerk=q.max_jerk;external.lateral_acceleration=q.max_lateral_acceleration;
  if(!unified_control::validLimits(external)||!finite(q.speed_cap)||q.speed_cap<0.){finish("p6_external_limits_invalid");return;}
  parameters.solver.limits=research_control::p6SafetyIntersection(parameters.solver.limits,external);
  parameters.speed_cap=std::min(parameters.speed_cap,q.speed_cap);
  parameters.solver.budget_ms=std::min(parameters.solver.budget_ms,q.solve_budget_ms);
  parameters.snapshot_hash=research_control::p6SnapshotIdentity(parameters);
  const auto effective=parameters; // Immutable BEFORE propagation/reference/solve/feedback.
  out.expected_apply_stamp=q.header.stamp+ros::Duration(q.control_dt);out.valid_until=out.expected_apply_stamp+ros::Duration(.04);
  const auto propagated=research_control::p5PropagateNominal(solve.state,q.state_observation_stamp.toSec(),out.expected_apply_stamp.toSec(),effective.solver.model,effective.solver.limits,solve.history,q.position_uncertainty,q.yaw_uncertainty,.04,.08,q.header.stamp.toSec());
  if(!effective.valid||!propagated.valid){finish(effective.valid?propagated.reason:effective.response.reason);return;}
  out.position_uncertainty=propagated.position_uncertainty;out.yaw_uncertainty=propagated.yaw_uncertainty;
  if(out.position_uncertainty>.03||out.yaw_uncertainty>.10){finish("p6_initialization_uncertainty_limit");return;}
  solve.state=propagated.state;out.propagated_speed=solve.state.speed;out.propagated_steering=solve.state.steering;
  out.propagated_yaw_rate=effective.solver.model.yaw_gain*solve.state.speed*std::tan(solve.state.steering)/effective.solver.model.wheelbase+effective.solver.model.yaw_residual;out.state_pair_validated=true;
  std::vector<research_control::ReferencePoint> spatial;if(!geometryFor(q,solve.state,spatial,why,&effective.solver.model,&derived_curvature)){finish(why);return;}
  solve.now=out.expected_apply_stamp.toSec();solve.control_dt=q.control_dt;solve.speed_cap=effective.speed_cap;solve.previous={q.previous_speed,q.previous_steering};solve.previous_command_acceleration=q.previous_acceleration;solve.baseline={q.baseline_speed,q.baseline_steering};
  solve.target_speed=wrapper->speed_target_separated?std::min(wrapper->target_speed,solve.speed_cap):solve.speed_cap;
  const bool speed_preview=p6_speed_preview_&&research_control::p6FlatMode(wrapper->computation_mode);
  solve.reference=research_control::FlatnessReferenceModel::buildTimeReference(spatial,solve.state.speed,
    solve.target_speed,solve.now,effective.solver,q.reference_braking,speed_preview,solve.state.acceleration,
    p6_synchronized_preview_,p6_terminal_approach_,p6_actuator_aligned_preview_?solve.state.yaw:research_control::missing());out.reference_valid=solve.reference.valid;
  actuator_aligned_schedule=solve.reference.valid&&std::isfinite(solve.reference.initial_body_yaw);
  const bool conditioning=p7_straight_tracking_reference_&&wrapper->computation_mode=="flat_nmpc_slqr";
  research_control::Reference unconditioned;
  if(conditioning&&p7_supported_course_)unconditioned=solve.reference;
  double straight_tracking_blend=conditioning?
      research_control::P7StraightTrackingReference::condition(spatial,solve.reference):0.;
  research_control::P7TurnTiming::Result turn_timing;
  if(wrapper->computation_mode=="flat_nmpc_slqr"&&straight_tracking_blend==0.)
    turn_timing=research_control::P7TurnTiming::apply(spatial,effective.solver,solve,
        p7_turn_speed_reserve_,p7_turn_release_weight_);
  research_control::P7StraightTrackingReference::Course course;
  if(conditioning&&p7_supported_course_&&straight_tracking_blend<.25&&
     !turn_timing.tightening&&!turn_timing.opening){
    const auto fitted=research_control::P7StraightTrackingReference::conditionSupportedCourse(
        spatial,unconditioned,straight_tracking_blend);
    if(fitted.blend>straight_tracking_blend){course=fitted;solve.reference=std::move(unconditioned);}
  }
  solve.straight_yaw_damping_weight=research_control::P7StraightTrackingReference::yawDampingWeight(
      course.blend>0.?course.straight_blend:straight_tracking_blend,p7_straight_yaw_damping_weight_);
  solve.supported_course_yaw_weight=research_control::P7StraightTrackingReference::courseYawWeight(
      course,p7_straight_yaw_damping_weight_);
  solve.force_timeout=q.force_timeout;solve.force_infeasible=q.force_infeasible;solve.force_numerical_failure=q.force_numerical_failure;
  const double setup_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
  // Reserve current-frame revalidation from the SAME end-to-end 8ms allowance.
  const double remaining_ms=effective.solver.budget_ms-setup_ms-.2-(p7_reference_revalidation_?.8:0.);
  if(remaining_ms<.1){out.timed_out=true;finish("p6_setup_exhausted_candidate_budget");return;}
  auto controlled=p6_controller_.solve(solve,effective,wrapper->computation_mode,nullptr,0.,nullptr,remaining_ms);
  if(controlled.solve.ok&&controlled.feedback.valid){
   research_control::State latest;ros::Time cutoff;bool unchanged=false;std::string observation_reason;
   if(p6FreshState(*wrapper,effective,solve,latest,cutoff,&unchanged,&observation_reason)){
    const auto deadline=start+std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double,std::milli>(effective.solver.budget_ms-.15));
    const auto objective_config=p6_controller_.objectiveConfig(effective.solver,wrapper->computation_mode);
    const auto validated=research_control::P6PolicyValidator::validate(solve,objective_config,controlled.solve,controlled.feedback,latest,solve.now,solve.history,deadline,true,p7_bounded_slqr_feedback_);
    controlled.solve=validated.solve;controlled.feedback_correction+=validated.feedback;controlled.feedback_ms+=validated.feedback_ms;
    controlled.feedback.reason+=";observation_gate="+observation_reason;
    if(validated.feedback_bounded)controlled.feedback.reason+=
      ";constrained_scale="+std::to_string(validated.feedback_scale)+
      (validated.feedback_nonzero?";constrained_nonzero":";constrained_zero");
    controlled.solve.solve_ms+=validated.feedback_ms;
    result.feedback_used_new_observation=true;result.validation_observation_cutoff=cutoff;
    out.propagated_speed=latest.speed;out.propagated_steering=latest.steering;out.propagated_yaw_rate=effective.solver.model.yaw_gain*latest.speed*std::tan(latest.steering)/effective.solver.model.wheelbase+effective.solver.model.yaw_residual;
   }
   else if(p6_nominal_observation_hold_&&unchanged){
    // The same verified IMU source and matching causal feedback remain available, so no new local error
    // can be inferred. Retain ONLY the already feasible nominal trajectory;
    // do not claim a fresh observation or manufacture a feedback correction.
    // P7 still revalidates the current state/reference before using a candidate.
    controlled.feedback.reason+=";matching_observation_nominal_retained;observation_gate="+observation_reason;
    result.feedback_used_new_observation=false;
   }
   else if(p7_guard_.enabled){
    controlled.solve.ok=controlled.solve.feasible=false;
    controlled.solve.reason="p7_slqr_requires_fresh_causal_observation";
    controlled.feedback.reason+=";observation_gate="+observation_reason;
   }
  }
  const auto&s=controlled.solve;
  research_control::p6Snapshot(result.parameters,effective);result.validation_stamp=out.expected_apply_stamp;result.validation_state_is_prediction=true;
  result.feedback_correction=controlled.feedback_correction;result.feedback_ms=controlled.feedback_ms;result.feedback_active=controlled.feedback.valid;
  result.feedback_reason=controlled.feedback.reason;result.feedback_count=controlled.feedback.count;result.feedback_dt=controlled.feedback.h;
  result.feedback_state_dimension=controlled.feedback.valid?3+controlled.feedback.queue_size:0;
  if(!controlled.feedback.valid)result.feedback_count=0;
  for(unsigned k=0;k<result.feedback_count;++k)for(unsigned j=0;j<result.feedback_state_dimension;++j)result.feedback_gains.push_back(controlled.feedback.gains[k](j));
  out.speed=s.command.speed;out.steering=s.command.steering;out.acceleration=s.command_acceleration;out.solve_ms=s.solve_ms;out.objective=s.objective;out.max_violation=s.max_violation;out.iterations=s.iterations;out.evaluations=s.evaluations;out.decision_dimension=s.decision_dimension;
  out.feasible=s.feasible;out.timed_out=s.timed_out;out.numerical_failure=q.force_numerical_failure||s.reason.find("numerical")!=std::string::npos;
  out.model_adaptation_active=false;for(const auto&change:effective.response.changes)out.model_adaptation_active|=std::isfinite(change.effective)&&std::abs(change.effective-change.nominal)>1e-10;
  out.weight_adaptation_active=false;out.complete_substep_constraints=s.ok&&s.feasible&&!s.timed_out&&s.max_violation<=1e-7;
  out.valid=out.complete_substep_constraints&&finite(out.speed)&&finite(out.steering)&&finite(out.acceleration);
  for(int i=0;i<s.count;++i){const auto&z=s.predicted[i];for(double v:{z.x,z.y,z.yaw,z.speed,z.steering,z.acceleration})out.predicted_states.push_back(v);if(i+1<s.count){out.predicted_commands.push_back(s.controls[i].speed);out.predicted_commands.push_back(s.controls[i].steering);}}
  finish(s.reason+";"+effective.response.reason+";slip="+effective.response.slip.reason+
    ";speed_target_separated="+(wrapper->speed_target_separated?std::string("1"):std::string("0"))+
    ";target_speed="+std::to_string(solve.target_speed)+";hard_speed_cap="+std::to_string(solve.speed_cap)+
    (controlled.near_knot_restored?";flat_chart=near_first_intervals":";flat_chart=original")+
    (controlled.feasibility_recovered?";flat_recovery=accepted":
     (controlled.feasibility_recovery_attempted?";flat_recovery=failed":";flat_recovery=not_used"))+
    (research_control::p6FlatMode(wrapper->computation_mode)?p6_flat_strategy_diagnostic_+
     ";flat_gn_chart=raw_cp_cached;flat_gn_directions="+std::to_string(controlled.tracking_directions)+
     ";flat_gn_accepted="+std::to_string(controlled.tracking_accepted)+
     ";flat_knot_s="+std::to_string(controlled.flat_knot)+
     ";flat_reference_fit_m="+std::to_string(controlled.flat_reference_fit)+
     ";flat_core_rejections="+std::to_string(controlled.flat_rejected_rollouts)+
     ";flat_first_rejected_step="+std::to_string(controlled.flat_first_rejected_step)+
     ";flat_first_rejection="+controlled.flat_first_rejection+
     ";flat_input_sqp_steps="+std::to_string(controlled.input_sqp_steps)+
     ";flat_input_merit_before="+std::to_string(controlled.input_merit_before)+
     ";flat_input_merit_after="+std::to_string(controlled.input_merit_after)+
     ";flat_coarse_poll_skipped="+(controlled.flat_coarse_poll_skipped?"1":"0")+
     ";flat_fine_poll_evaluations="+std::to_string(controlled.flat_fine_poll_evaluations)+
     ";flat_fine_poll_improvements="+std::to_string(controlled.flat_fine_poll_improvements)+
     ";flat_coarse_poll_sweeps="+std::to_string(controlled.flat_coarse_poll_sweeps)+
     ";flat_poll_stop_stage="+std::to_string(controlled.flat_poll_stop_stage)+
     ";policy_build_ms="+std::to_string(controlled.policy_build_ms)+
     ";nominal_validation_reused="+(controlled.nominal_validation_reused?"1":"0")+
     ";steering_rate_history_valid="+(controlled.steering_rate_history_valid?"1":"0")+
     ";steering_rate_continuity_weight="+std::to_string(controlled.steering_rate_continuity_weight)+
     ";straight_tracking_blend="+std::to_string(straight_tracking_blend)+
     ";supported_course_blend="+std::to_string(course.blend)+
     ";supported_course_curvature="+std::to_string(course.curvature)+
     ";supported_course_yaw_weight="+std::to_string(solve.supported_course_yaw_weight)+
     ";straight_yaw_damping_weight="+std::to_string(solve.straight_yaw_damping_weight)+
     ";turn_speed_reserve="+std::to_string(turn_timing.speed_reserve)+
     ";turn_release_weight="+std::to_string(turn_timing.release_weight)+
     ";steering_rate_prior="+std::to_string(controlled.steering_rate_prior)+
     ";steering_rate_continuity_cost="+std::to_string(controlled.steering_rate_continuity_cost)+
     ";flat_response_recovery_seed_attempted="+(controlled.response_recovery_seed_attempted?"1":"0")+
     ";flat_response_recovery_seed_used="+(controlled.response_recovery_seed_used?"1":"0")+
     ";flat_response_base_used="+(controlled.flat_response_base_used?"1":"0")+
     ";flat_response_base_cost="+std::to_string(controlled.flat_response_base_cost)+
     ";flat_response_warm_available="+(controlled.flat_response_warm_available?"1":"0")+
     ";flat_response_warm_accepted="+(controlled.flat_response_warm_accepted?"1":"0")+
     ";flat_response_warm_cost_before="+std::to_string(controlled.flat_response_warm_cost_before)+
     ";flat_response_warm_cost_after="+std::to_string(controlled.flat_response_warm_cost_after)+
     ";flat_warm_available="+(s.warm_available?"1":"0")+
     ";flat_warm_selected="+(s.warm_started?"1":"0"):std::string()));
 }
public:UnifiedNMPCWorker(){std::string mode,terrain;p6_enabled_=pnh_.param("p6_enabled",false);if(p6_enabled_){p6_config_=research_control::loadP6Runtime(pnh_,p6_scenes_);hash_=p6_config_.config_hash;config_.solver=p6_config_.profiles[0].solver;}else config_=research_control::loadRuntime(pnh_,mode,terrain,hash_);pnh_.param<std::string>("run_id",run_id_,"");if(run_id_.empty())throw std::runtime_error("run_id required");pnh_.param<std::string>("research_backend",backend_,"original12");if(backend_!="original12"&&backend_!="blocked8"&&backend_!="flat6"&&backend_!="front8"&&backend_!="curve8"&&backend_!="hold8"&&backend_!="flatseed8"&&backend_!="search8"&&backend_!="refine12")throw std::runtime_error("unknown research backend");config_.research_backend=backend_;
 bool adaptation=false;pnh_.param("enable_adaptation",adaptation,false);config_.model_adaptation_enabled&=adaptation;config_.weight_adaptation_enabled&=adaptation;config_.research_strict_adaptation=false;config_.research_validity_adaptation=false;
 p6_updates_enabled_=pnh_.param("enable_p6_adaptation",false);
 p7_guard_.enabled=pnh_.param("p7_supervised_test",false);
 p6_qualified_steering_=p6_enabled_&&pnh_.param("p6_qualified_steering",false);
 p6_separated_speed_target_=p6_enabled_&&pnh_.param("p6_separated_speed_target",false);
 p6_speed_preview_=p6_enabled_&&pnh_.param("p6_speed_preview",false);
 p6_synchronized_preview_=p6_enabled_&&pnh_.param("p6_synchronized_preview",false);
 p6_actuator_aligned_preview_=p6_enabled_&&pnh_.param("p6_actuator_aligned_preview",false);
 p6_terminal_approach_=p6_enabled_&&pnh_.param("p6_terminal_approach",false);
 if(p6_terminal_approach_&&!p6_speed_preview_)throw std::runtime_error("terminal approach requires speed preview");
 if(p6_synchronized_preview_&&!p6_speed_preview_)throw std::runtime_error("synchronized preview requires speed preview");
 if(p6_actuator_aligned_preview_&&!p6_synchronized_preview_)throw std::runtime_error("actuator-aligned preview requires synchronized preview");
 const double continuity_weight=pnh_.param("p6_steering_rate_continuity_weight",0.);
 if(!finite(continuity_weight)||continuity_weight<0.||continuity_weight>1.)throw std::runtime_error("invalid steering rate continuity weight");
 p6_controller_.setSteeringRateContinuityWeight(continuity_weight);
 p6_controller_.setTurnAwareRateContinuity(pnh_.param("p6_turn_aware_rate_continuity",false));
 const double yaw_rate_weight=pnh_.param("p6_yaw_rate_tracking_weight",0.);
 if(!finite(yaw_rate_weight)||yaw_rate_weight<0.||yaw_rate_weight>4.)throw std::runtime_error("invalid yaw-rate tracking weight");
 p6_controller_.setYawRateTrackingWeight(yaw_rate_weight);
  p7_reference_revalidation_=p7_guard_.enabled&&pnh_.param("p7_reference_revalidation",false);
  const bool near_knot=pnh_.param("p7_flat_near_knot_restoration",p7_reference_revalidation_);
  if(near_knot&&!p7_reference_revalidation_)throw std::runtime_error("near-knot restoration requires reference revalidation");
  p6_controller_.setNearKnotRestoration(near_knot);
  const bool flat_recovery=pnh_.param("p7_flat_feasibility_recovery",false);
  if(flat_recovery&&!p7_reference_revalidation_)
    throw std::runtime_error("flat feasibility recovery requires supervised current-reference revalidation");
  p6_controller_.setFeasibilityRecovery(flat_recovery);
  const bool accuracy=pnh_.param("p7_flat_accuracy_refinement",false);
  // New options also work through the original P5 isolated research transport.
  // This worker only publishes candidates; command authority remains in P5.
  // In supervised P7 only frozen p7_* options count; stale p6_* params cannot override them.
  const bool isolated_research=p6_enabled_&&!p7_guard_.enabled&&run_id_.compare(0,3,"p5_")==0;
  const bool conditioned_geometry=(isolated_research?pnh_.param("p6_flat_conditioned_geometry",pnh_.param("p7_flat_conditioned_geometry",false)):pnh_.param("p7_flat_conditioned_geometry",false));
  const bool baseline_seed=(isolated_research?pnh_.param("p6_flat_baseline_seed",pnh_.param("p7_flat_baseline_seed",false)):pnh_.param("p7_flat_baseline_seed",false));
  const bool delay_aligned=(isolated_research?pnh_.param("p6_flat_delay_aligned_reconstruction",pnh_.param("p7_flat_delay_aligned_reconstruction",false)):pnh_.param("p7_flat_delay_aligned_reconstruction",false));
  const bool independent_inputs=(isolated_research?pnh_.param("p6_flat_independent_input_encoding",pnh_.param("p7_flat_independent_input_encoding",false)):pnh_.param("p7_flat_independent_input_encoding",false));
  const bool command_projection=(isolated_research?pnh_.param("p6_flat_command_projection",pnh_.param("p7_flat_command_projection",false)):pnh_.param("p7_flat_command_projection",false));
  const bool causal_seed=(isolated_research?pnh_.param("p6_flat_causal_reference_seed",pnh_.param("p7_flat_causal_reference_seed",false)):pnh_.param("p7_flat_causal_reference_seed",false));
  const bool response_knots=(isolated_research?pnh_.param("p6_flat_response_knot_monitor",pnh_.param("p7_flat_response_knot_monitor",false)):pnh_.param("p7_flat_response_knot_monitor",false));
  const bool input_sqp=(isolated_research?pnh_.param("p6_flat_input_feasibility_sqp",pnh_.param("p7_flat_input_feasibility_sqp",false)):pnh_.param("p7_flat_input_feasibility_sqp",false));
  const bool response_base=(isolated_research?pnh_.param("p6_flat_response_base",pnh_.param("p7_flat_response_base",false)):pnh_.param("p7_flat_response_base",false));
  const bool near_support=(isolated_research?pnh_.param("p6_flat_response_near_support",false):pnh_.param("p7_flat_response_near_support",false));
  const bool response_warm=(isolated_research?pnh_.param("p6_flat_response_warm_start",false):pnh_.param("p7_flat_response_warm_start",false));
  if(response_warm&&!response_base)throw std::runtime_error("response warm start requires response base");
  const bool budgeted_poll=(isolated_research?pnh_.param("p6_flat_budgeted_poll",false):pnh_.param("p7_flat_budgeted_poll",false));
  if(budgeted_poll&&!response_base)throw std::runtime_error("budgeted flat poll requires response base");
  p6_nominal_observation_hold_=(isolated_research?pnh_.param("p6_nominal_observation_hold",false):pnh_.param("p7_nominal_observation_hold",false));
  if(near_support&&!response_base)throw std::runtime_error("near correction support requires response base");
  p7_candidate_control_trial_=pnh_.param("p7_candidate_control_trial",false);
  p7_straight_tracking_reference_=pnh_.param("p7_straight_tracking_reference",false);
  p7_supported_course_=pnh_.param("p7_supported_course",false);
  if(p7_supported_course_&&!p7_straight_tracking_reference_)
    throw std::runtime_error("supported course requires supervised straight tracking reference");
  p7_straight_yaw_damping_weight_=pnh_.param("p7_straight_yaw_damping_weight",0.);
  p7_turn_speed_reserve_=pnh_.param("p7_turn_speed_reserve",0.);
  p7_turn_release_weight_=pnh_.param("p7_turn_release_weight",0.);
  if(!finite(p7_turn_speed_reserve_)||p7_turn_speed_reserve_<0.||p7_turn_speed_reserve_>.05||
     !finite(p7_turn_release_weight_)||p7_turn_release_weight_<0.||p7_turn_release_weight_>4.||
     ((p7_turn_speed_reserve_>0.||p7_turn_release_weight_>0.)&&
      (!p7_candidate_control_trial_||!p7_reference_revalidation_||!p6_speed_preview_)))
    throw std::runtime_error("turn timing requires supervised speed preview with current-reference revalidation");
  if(!finite(p7_straight_yaw_damping_weight_)||p7_straight_yaw_damping_weight_<0.||
     p7_straight_yaw_damping_weight_>4.||(p7_straight_yaw_damping_weight_>0.&&!p7_straight_tracking_reference_))
    throw std::runtime_error("invalid supervised straight yaw damping weight");
  if(p7_straight_tracking_reference_&&(!p7_candidate_control_trial_||!p7_reference_revalidation_))
    throw std::runtime_error("straight tracking reference requires supervised current-reference revalidation");
  // Explicit operator-supervised experiment only, never an automatic upgrade
  // of an existing shadow run. Other research strategies retain their lock.
  if(p7_candidate_control_trial_&&(!p6_enabled_||!p7_guard_.enabled||!p7_reference_revalidation_||
      !response_base||!near_support||!p6_nominal_observation_hold_||causal_seed||response_knots||
      input_sqp||budgeted_poll||p6_updates_enabled_||adaptation))
    throw std::runtime_error("candidate control trial requires fixed smooth response strategy and supervised reference validation");
  p6_shadow_only_strategy_=(causal_seed||response_knots||input_sqp||budgeted_poll||
    ((response_base||response_warm||near_support||p6_nominal_observation_hold_)&&!p7_candidate_control_trial_))&&!isolated_research;
  p7_bounded_slqr_feedback_=pnh_.param("p7_bounded_slqr_feedback",false);
  p6_can_only_feedback_=pnh_.param("p6_can_only_feedback",false);
 p7_qualify_chassis_motion_=pnh_.param("p7_qualify_chassis_motion",false);
 if(p7_qualify_chassis_motion_&&(!p7_guard_.enabled||!p6_qualified_steering_))
   throw std::runtime_error("P7 chassis motion qualification requires supervised trial steering qualification");
  if(p6_can_only_feedback_&&(!p6_enabled_||!p6_qualified_steering_||!p7_bounded_slqr_feedback_||
     (!p7_reference_revalidation_&&!isolated_research)))
    throw std::runtime_error("CAN-only feedback requires qualified steering, bounded SLQR and reference revalidation");
  if((accuracy||p7_bounded_slqr_feedback_||conditioned_geometry||baseline_seed||delay_aligned||independent_inputs||command_projection||causal_seed||response_knots||input_sqp||response_base||response_warm||near_support||budgeted_poll||p6_nominal_observation_hold_)&&!p7_reference_revalidation_&&!isolated_research)
    throw std::runtime_error("P7 accuracy strategies require current-reference revalidation");
  if(accuracy&&!flat_recovery)throw std::runtime_error("P7 flat accuracy refinement requires feasibility recovery");
  p6_controller_.setAccuracyRefinement(accuracy);
  p6_controller_.setConditionedGeometry(conditioned_geometry);
  p6_controller_.setBaselineSeed(baseline_seed);
  p6_controller_.setDelayAlignedReconstruction(delay_aligned);
  p6_controller_.setIndependentInputEncoding(independent_inputs);
  p6_controller_.setCommandProjection(command_projection);
  p6_controller_.setCausalReferenceSeed(causal_seed);
  p6_controller_.setResponseKnotMonitor(response_knots);
  p6_controller_.setInputFeasibilitySQP(input_sqp);
  p6_controller_.setResponseBase(response_base);
  p6_controller_.setResponseWarmStart(response_warm);
  p6_controller_.setResponseNearSupport(near_support);
  p6_controller_.setBudgetedPoll(budgeted_poll);
  p6_flat_strategy_diagnostic_=std::string(";flat_conditioned_geometry=")+(conditioned_geometry?"1":"0")+
    ";flat_baseline_seed="+(baseline_seed?"1":"0")+";flat_delay_aligned_reconstruction="+(delay_aligned?"1":"0")+
    ";flat_independent_input_encoding="+(independent_inputs?"1":"0")+";flat_command_projection="+(command_projection?"1":"0")+
    ";flat_causal_reference_seed="+(causal_seed?"1":"0")+";flat_response_knot_monitor="+(response_knots?"1":"0")+
    ";flat_input_feasibility_sqp="+(input_sqp?"1":"0")+";flat_response_base="+(response_base?"1":"0")+
    ";flat_response_warm_start="+(response_warm?"1":"0")+";flat_budgeted_poll="+(budgeted_poll?"1":"0")+";flat_response_near_support="+(near_support?"1":"0")+";nominal_observation_hold="+(p6_nominal_observation_hold_?"1":"0")+
    ";candidate_control_trial="+(p7_candidate_control_trial_?"1":"0")+
    ";yaw_rate_tracking_weight="+std::to_string(yaw_rate_weight)+";speed_preview="+(p6_speed_preview_?"1":"0")+";synchronized_preview="+(p6_synchronized_preview_?"1":"0")+";terminal_approach="+(p6_terminal_approach_?"1":"0")+
    ";actuator_aligned_preview="+(p6_actuator_aligned_preview_?"1":"0")+";can_only_feedback="+(p6_can_only_feedback_?"1":"0");
  p6_controller_.setBoundedFeedback(p7_bounded_slqr_feedback_);
  p6_controller_.setCourseConsistentFeedback(pnh_.param("p6_course_consistent_feedback",false));
 if(p7_guard_.enabled){
  p7_guard_.mapping_confirmed=pnh_.param("p7_mapping_confirmed",false);p7_guard_.command_topic="/cmd_vel";
  p7_guard_.run_id=pnh_.param<std::string>("p7_run_id","");p7_guard_.terrain=pnh_.param<std::string>("p5_terrain","unknown");
  p7_guard_.operator_id=pnh_.param<std::string>("p7_operator_id","");p7_guard_.mapping_evidence=pnh_.param<std::string>("p7_mapping_evidence","");
  p7_guard_.imu_frame=pnh_.param<std::string>("p7_imu_frame","");p7_guard_.speed_cap=pnh_.param("p7_speed_cap",0.);
  p7_guard_.degraded_speed_limit=pnh_.param("degraded_speed_limit",.10);p7_guard_.wheel_scale=pnh_.param("wheel_speed_scale",.001);
  p7_guard_.steering_sign=pnh_.param("feedback_steer_sign",1.);p7_guard_.imu_yaw_sign=1.;
  p7_guard_.synthetic=pnh_.param("p6_synthetic_observation_contract",false);
  if(!p7_guard_.configured()||run_id_!=p7_guard_.run_id||!p6_enabled_||p6_updates_enabled_||adaptation||p6_scenes_.enabled)
   throw std::runtime_error("P7 worker requires real engineering mapping, fixed smooth model, and matching identity");
 }

 if(!runtime_.configure(config_)||(backend_=="original12"&&config_.solver.knots!=6))throw std::runtime_error("invalid worker config");
 std::string q,c,p,d;pnh_.param<std::string>("request_topic",q,"/control_research/p5/request");pnh_.param<std::string>("candidate_topic",c,"/control_research/p5/candidate");pnh_.param<std::string>("published_topic",p,"/control_research/p5/published");pnh_.param<std::string>("diagnostics_topic",d,"/control_research/p5/worker_diagnostics");
 for(const auto&t:{q,c,p,d})if(nh_.resolveName(t).compare(0,21,"/control_research/p5/")!=0)throw std::runtime_error("worker topic outside P5 namespace");
 candidates_=nh_.advertise<move_car::P5ControlCandidate>(c,1);diagnostic_=nh_.advertise<std_msgs::String>(d,4);
 // P6 research transport only: avoid small history/request packets waiting for
 // TCP delayed ACKs. Causal sequence matching and all deadlines stay unchanged.
 if(p6_enabled_)published_=nh_.subscribe(p,256,&UnifiedNMPCWorker::published,this,ros::TransportHints().tcpNoDelay());
 else published_=nh_.subscribe(p,256,&UnifiedNMPCWorker::published,this);
 if(!p6_enabled_)request_=nh_.subscribe(q,1,&UnifiedNMPCWorker::request,this);
 else{
  const std::string request=nh_.resolveName("/control_research/p5_test/p6/request"),candidate=nh_.resolveName("/control_research/p5_test/p6/candidate");
  for(const auto&topic:{request,candidate})if(topic.compare(0,29,"/control_research/p5_test/p6/")!=0)throw std::runtime_error("P6 worker remap outside isolated namespace");
  if(run_id_.compare(0,3,"p5_")!=0&&!p7_guard_.configured())throw std::runtime_error("P6 requires isolated p5_ or explicit supervised p7real_ run identity");
  p6_candidates_=nh_.advertise<move_car::P6ControlCandidate>(candidate,1);p6_request_=nh_.subscribe(request,1,&UnifiedNMPCWorker::p6Request,this,ros::TransportHints().tcpNoDelay());
  ros::NodeHandle observations(nh_);observations.setCallbackQueue(&p6_observation_queue_);
  p6_wheel_sub_=observations.subscribe("/car_message",1,&UnifiedNMPCWorker::p6Wheel,this,ros::TransportHints().tcpNoDelay());p6_imu_sub_=observations.subscribe("/imu",1,&UnifiedNMPCWorker::p6Imu,this,ros::TransportHints().tcpNoDelay());
  p6_observation_spinner_.reset(new ros::AsyncSpinner(1,&p6_observation_queue_));p6_observation_spinner_->start();
 }
 }
 ~UnifiedNMPCWorker(){if(p6_observation_spinner_)p6_observation_spinner_->stop();}
};
int main(int argc,char**argv){ros::init(argc,argv,"p5_nmpc_worker");try{UnifiedNMPCWorker worker;ros::spin();return 0;}catch(const std::exception&e){ROS_FATAL("P5 worker stopped: %s",e.what());return 1;}}
