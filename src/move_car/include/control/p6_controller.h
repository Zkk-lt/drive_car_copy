#pragma once
#include "control/p6_config.h"
#include "control/p6_flatness_nmpc.h"
#include "control/p6_slqr_feedback.h"
#include <sstream>
#include <iomanip>
namespace research_control {
inline bool p6KnownMode(const std::string&m){return m=="legacy_slqr"||m=="nmpc_shadow"||m=="nmpc_12_fixed"||m=="blocked8"||m=="flat_nmpc_8"||m=="flat_nmpc_8_uniform"||m=="flat_nmpc_residual"||m=="flat_nmpc_slqr"||m=="flat_nmpc_residual_slqr"||m=="research_auto_fallback"||m=="scene_fixed"||m=="yaw_compensation"||m=="steering_compensation"||m=="multi_residual_12";}
inline bool p6ResidualMode(const std::string&m){return m=="flat_nmpc_residual"||m=="flat_nmpc_residual_slqr"||m=="research_auto_fallback"||m=="yaw_compensation"||m=="steering_compensation"||m=="multi_residual_12";}
inline bool p6FeedbackMode(const std::string&m){return m=="flat_nmpc_slqr"||m=="flat_nmpc_residual_slqr"||m=="research_auto_fallback";}
inline bool p6FlatMode(const std::string&m){return m.compare(0,9,"flat_nmpc")==0||m=="research_auto_fallback";}
inline std::string p6SnapshotIdentity(const P6EffectiveParameters& p){
 std::ostringstream s;s<<std::setprecision(17)<<p.nominal_config_hash<<'|'<<p.scene<<'|'<<p.cycle<<'|'<<p.history_base_sequence<<'|'<<p.stamp;
 const auto&m=p.solver.model;const auto&w=p.solver.weights;const auto&l=p.solver.limits;
 for(double v:{m.wheelbase,m.rear_to_center,m.steering_tau,m.steering_delay,m.steering_gain,m.speed_tau,m.speed_delay,m.speed_gain,m.yaw_gain,m.yaw_residual,m.longitudinal_residual,m.lateral_residual,w.lateral,w.heading,w.speed,w.smoothness,l.speed,l.steering,l.steering_rate,l.acceleration,l.deceleration,l.jerk,l.lateral_acceleration,p.speed_cap,p.solver.dt,p.solver.budget_ms})s<<'|'<<v;
 s<<'|'<<p.solver.steps<<'|'<<p.solver.knots<<'|'<<p.solver.iterations;
 std::uint64_t hash=1469598103934665603ULL;for(unsigned char c:s.str()){hash^=c;hash*=1099511628211ULL;}
 std::ostringstream o;o<<std::hex<<std::setw(16)<<std::setfill('0')<<hash;return o.str();
}
// Absence of a NEW observation is distinct from an invalid initial state.
// Exact receipt/source equality is required: no interpolation or timestamp repair.
inline bool p6UnchangedFeedbackObservation(double fw,double fi,double fs,
 double cw,double ci,double cs,double now,double apply){
 for(double t:{fw,fi,fs,cw,ci,cs,now,apply})if(!std::isfinite(t)||t<=0.)return false;
 return cw==fw&&ci==fi&&cs==fs&&cs<=ci&&now>=std::max(cw,ci)&&
   now-cs<=.12&&now-cw<=.12&&now<=apply;
}
class P6Controller {
 ReducedNMPC twelve_,blocked_;P6FlatnessNMPC flat_;p6::ResponseEstimator response_;bool bounded_feedback_=false,course_consistent_feedback_=false;
 double yaw_rate_tracking_weight_=0.,steering_rate_continuity_weight_=0.;
 bool turn_aware_rate_continuity_=false;
public:
 void setSteeringRateContinuityWeight(double value){steering_rate_continuity_weight_=value;}
 void setTurnAwareRateContinuity(bool value){turn_aware_rate_continuity_=value;}
 void setNearKnotRestoration(bool value){flat_.setNearKnotRestoration(value);}
 void setFeasibilityRecovery(bool value){flat_.setFeasibilityRecovery(value);}
 void setAccuracyRefinement(bool value){flat_.setAccuracyRefinement(value);}
 void setConditionedGeometry(bool value){flat_.setConditionedGeometry(value);}
 void setBaselineSeed(bool value){flat_.setBaselineSeed(value);}
 void setDelayAlignedReconstruction(bool value){flat_.setDelayAlignedReconstruction(value);}
 void setIndependentInputEncoding(bool value){flat_.setIndependentInputEncoding(value);}
 void setCommandProjection(bool value){flat_.setCommandProjection(value);}
 void setCausalReferenceSeed(bool value){flat_.setCausalReferenceSeed(value);}
 void setResponseKnotMonitor(bool value){flat_.setResponseKnotMonitor(value);}
 void setInputFeasibilitySQP(bool value){flat_.setInputFeasibilitySQP(value);}
 void setResponseBase(bool value){flat_.setResponseBase(value);}
 void setResponseWarmStart(bool value){flat_.setResponseWarmStart(value);}
 void setResponseNearSupport(bool value){flat_.setResponseNearSupport(value);}
 void setBudgetedPoll(bool value){flat_.setBudgetedPoll(value);}
 void setBoundedFeedback(bool value){bounded_feedback_=value;}
 void setCourseConsistentFeedback(bool value){course_consistent_feedback_=value;}
 void setYawRateTrackingWeight(double value){yaw_rate_tracking_weight_=value;}
 // The versioned strategy controls the objective; the P5 wire snapshot stays
 // unchanged. A fresh-state validation uses the same strategy as the solve.
 SolverConfig objectiveConfig(SolverConfig c,const std::string& mode)const{
  if(p6FlatMode(mode))c.yaw_rate_tracking_weight=yaw_rate_tracking_weight_;
  return c;
 }
 // The issued steering trend is useful along one turn. Across a reversal it
 // can oppose the next turn through the delayed steering response. Fade only
 // this performance term when both signs are present in the response window.
 static double turnAwareRateScale(const Reference& reference,const Model& model){
  const double window=2.*(model.steering_delay+model.steering_tau);
  if(!reference.valid||reference.count<2||reference.count>kMaxHorizon+1||
     !std::isfinite(window)||window<=0.)return 1.;
  constexpr double meaningful_curvature=.12; // m^-1, same as synchronized preview
  double left=0.,right=0.;
  for(int i=0;i<reference.count&&reference.points[i].t<=window;++i){
   const double curvature=reference.points[i].curvature;
   if(!std::isfinite(curvature))return 1.;
   const double phase=clamp((window-reference.points[i].t)/(.5*window),0.,1.);
   const double response_curvature=curvature*phase*phase*(3.-2.*phase);
   left=std::max(left,response_curvature);right=std::max(right,-response_curvature);
  }
  const double change=clamp(std::min(left,right)/meaningful_curvature,0.,1.);
  return 1.-change*change*(3.-2.*change);
 }
 struct Result{SolveResult solve;P6PolicyValidator::Policy feedback;double feedback_correction=0.,feedback_ms=0.,policy_build_ms=0.,total_ms=0.;bool nominal_validation_reused=false,feedback_nonzero=false,near_knot_restored=false,feasibility_recovery_attempted=false,feasibility_recovered=false;int tracking_directions=0,tracking_accepted=0;std::string reason;
  bool steering_rate_history_valid=false,steering_rate_continuity_active=false;
  double steering_rate_prior=missing(),steering_rate_continuity_weight=0.,steering_rate_continuity_cost=0.;
  double flat_knot=missing(),flat_reference_fit=missing();int flat_rejected_rollouts=0,flat_first_rejected_step=-1;
  const char* flat_first_rejection="none";
  int input_sqp_steps=0;double input_merit_before=missing(),input_merit_after=missing();
  bool response_recovery_seed_attempted=false,response_recovery_seed_used=false;
  bool flat_response_base_used=false;double flat_response_base_cost=missing();
  bool flat_response_warm_available=false,flat_response_warm_accepted=false;double flat_response_warm_cost_before=missing(),flat_response_warm_cost_after=missing();
  bool flat_coarse_poll_skipped=false;int flat_fine_poll_evaluations=0,flat_fine_poll_improvements=0,flat_coarse_poll_sweeps=0,flat_poll_stop_stage=0;
 };
 void reset(){twelve_.reset();blocked_.reset();flat_.reset();response_.reset();}
 P6EffectiveParameters parameters(const P6NominalProfile&nominal,p6::SensorSnapshot sensors,
  const std::vector<p6::PublishedSample>&history,p6::ResponsePolicy policy,const std::string&mode){
  P6EffectiveParameters out;out.solver=nominal.solver;out.scene=nominal.scene;out.nominal_config_hash=nominal.config_hash;
  out.stamp=sensors.now;out.cycle=sensors.cycle;out.speed_cap=nominal.speed_cap;
  sensors.scene=out.scene;sensors.nominal_config_hash=nominal.config_hash;
  auto scene_channel=[&](p6::ChannelPolicy&channel,const char*name){
   if(nominal.response_policy_kind!="synthetic_validation"||nominal.response_policy_sha.size()!=64||!nominal.response_channels.count(name))channel.enabled=false;
   if(std::isfinite(nominal.response_minimum_confidence))channel.min_confidence=std::max(channel.min_confidence,nominal.response_minimum_confidence);
  };
  scene_channel(policy.steering,"steering");scene_channel(policy.longitudinal,"longitudinal");
  scene_channel(policy.yaw,policy.yaw_additive_enabled?"yaw_disturbance":"yaw_gain");scene_channel(policy.vibration,"roughness");scene_channel(policy.slip,"slip");
  auto restrict_interval=[&](const char*key,double&lo,double&hi){auto i=nominal.metadata.find(key);if(i!=nominal.metadata.end()){lo=std::max(lo,i->second.lower);hi=std::min(hi,i->second.upper);}};
  restrict_interval("steering_gain",policy.steering.min_gain,policy.steering.max_gain);restrict_interval("steering_tau",policy.steering.min_tau,policy.steering.max_tau);
  restrict_interval("speed_gain",policy.longitudinal.min_gain,policy.longitudinal.max_gain);restrict_interval("speed_tau",policy.longitudinal.min_tau,policy.longitudinal.max_tau);restrict_interval("yaw_gain",policy.yaw.min_gain,policy.yaw.max_gain);
  if(!p6ResidualMode(mode)){policy.steering.enabled=policy.yaw.enabled=policy.longitudinal.enabled=false;policy.yaw_additive_enabled=false;}
  if(mode=="yaw_compensation"){policy.steering.enabled=policy.longitudinal.enabled=false;}
  if(mode=="steering_compensation"){policy.yaw.enabled=policy.longitudinal.enabled=false;}
  out.response=response_.observe(sensors,history,nominal.solver.model,policy);out.valid=out.response.valid;
  if(sensors.wheel_speed.valid&&sensors.wheel_speed.unit_verified&&std::isfinite(sensors.wheel_speed.value)){
   for(const auto&m:nominal.metadata)if(std::abs(sensors.wheel_speed.value)<m.second.minimum_speed||std::abs(sensors.wheel_speed.value)>m.second.maximum_speed){out.valid=false;out.response.reason="outside_scene_model_speed_domain:"+m.first;break;}
  }
  out.solver.model=out.response.effective;out.revision=out.response.effective_revision;out.history_base_sequence=out.response.history_base_sequence;
  if(!p6ResidualMode(mode)){
   out.solver.model=nominal.solver.model;out.response.effective=nominal.solver.model;
   for(auto&change:out.response.changes){change.effective=change.nominal;change.reason="fixed_mode_nominal_model";change.source="nominal_config";}
   out.response.effective_hash=p6::responseHash(out.response);
  }
  out.snapshot_hash=p6SnapshotIdentity(out);return out;
 }
 Result solve(const SolveRequest&r,const P6EffectiveParameters& effective,const std::string&mode,
  const State*validation_state=nullptr,double validation_stamp=0.,const std::vector<CommandSample>*validation_history=nullptr,double available_ms=missing()){
  const auto start=std::chrono::steady_clock::now();Result out;out.solve.command=r.previous;
  if(!effective.valid||effective.snapshot_hash!=p6SnapshotIdentity(effective)||!p6KnownMode(mode)){out.solve.reason="invalid_or_mutated_parameter_snapshot";return out;}
  if(!p6FlatMode(mode)&&mode!="blocked8"&&effective.solver.knots!=6){out.solve.reason="twelve_baseline_requires_six_control_knots";return out;}
  const double allowance=std::isfinite(available_ms)?std::min(available_ms,effective.solver.budget_ms):effective.solver.budget_ms;
  if(allowance<=0.){out.solve.timed_out=true;out.solve.reason="p6_candidate_budget_unavailable";return out;}
  const auto deadline=start+std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double,std::milli>(allowance));
  auto c=objectiveConfig(effective.solver,mode);c.budget_ms=allowance;const bool feedback=p6FeedbackMode(mode);
  // Reserve part of the SAME 8ms budget for local gains + full nonlinear validation.
  // This is computational scheduling, not relaxation of a physical boundary.
  if(feedback)c.budget_ms=std::max(.1,c.budget_ms-1.5);
  if(p6FlatMode(mode)){
   double rate_weight=steering_rate_continuity_weight_;
   if(rate_weight>0.&&turn_aware_rate_continuity_)rate_weight*=turnAwareRateScale(r.reference,c.model);
   flat_.setSteeringRateContinuityWeight(rate_weight);
   flat_.setNonuniformKnots(mode!="flat_nmpc_8_uniform");out.solve=flat_.solve(r,c);out.near_knot_restored=flat_.diagnostics().near_knot_restoration;out.feasibility_recovery_attempted=flat_.diagnostics().feasibility_recovery_attempted;out.feasibility_recovered=flat_.diagnostics().feasibility_recovered;out.tracking_directions=flat_.diagnostics().tracking_directions;out.tracking_accepted=flat_.diagnostics().tracking_accepted;
  }
  else if(mode=="blocked8"){c.knots=4;c.research_grid=3;out.solve=blocked_.solve(r,c);}
  else out.solve=twelve_.solve(r,c);
  if(p6FlatMode(mode)){const auto& d=flat_.diagnostics();out.flat_knot=d.interior_knot;
   out.flat_reference_fit=d.reference_fit_rmse;out.flat_rejected_rollouts=d.rejected_rollouts;
   out.flat_first_rejected_step=d.first_rejected_step;out.flat_first_rejection=d.first_rejection;
   out.input_sqp_steps=d.input_sqp_steps;out.input_merit_before=d.input_merit_before;out.input_merit_after=d.input_merit_after;
   out.response_recovery_seed_attempted=d.response_recovery_seed_attempted;out.response_recovery_seed_used=d.response_recovery_seed_used;
   out.flat_response_base_used=d.response_base_used;out.flat_response_base_cost=d.response_base_cost;
   out.flat_response_warm_available=d.response_warm_available;out.flat_response_warm_accepted=d.response_warm_accepted;out.flat_response_warm_cost_before=d.response_warm_cost_before;out.flat_response_warm_cost_after=d.response_warm_cost_after;
   out.flat_coarse_poll_skipped=d.coarse_poll_skipped;out.flat_fine_poll_evaluations=d.fine_poll_evaluations;out.flat_fine_poll_improvements=d.fine_poll_improvements;out.flat_coarse_poll_sweeps=d.coarse_poll_sweeps;out.flat_poll_stop_stage=d.local_poll_stop_stage;}
  if(p6FlatMode(mode)){const auto& d=flat_.diagnostics();
   out.steering_rate_history_valid=d.steering_rate_history_valid;out.steering_rate_continuity_active=d.steering_rate_continuity_active;
   out.steering_rate_prior=d.steering_rate_prior;out.steering_rate_continuity_weight=d.steering_rate_continuity_weight;out.steering_rate_continuity_cost=d.steering_rate_continuity_cost;}
  if(feedback&&out.solve.ok){
   const auto build_start=std::chrono::steady_clock::now();
   const bool built=P6PolicyValidator::build(r,c,out.solve,out.feedback,deadline,course_consistent_feedback_);
   out.policy_build_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-build_start).count();
   if(!built){out.solve.ok=out.solve.feasible=false;out.solve.reason=out.feedback.reason;}
   else if(!validation_state&&!validation_history){
    // The solver just validated this exact request and immutable snapshot.
    // At its own nominal initial state every local error/queue delta is zero.
    // Reuse only here; external/fresh-state validation must do a full rollout.
    out.nominal_validation_reused=true;
   }else{
    const auto checked=P6PolicyValidator::validate(r,c,out.solve,out.feedback,validation_state?*validation_state:r.state,
      validation_state?validation_stamp:r.now,validation_history?*validation_history:r.history,deadline,true,bounded_feedback_);
    out.solve=checked.solve;out.feedback_ms=checked.feedback_ms;out.feedback_correction=checked.feedback;out.feedback_nonzero=checked.feedback_nonzero;
   }
  }
  out.total_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
  if(out.total_ms>=allowance){out.solve.ok=out.solve.feasible=false;out.solve.timed_out=true;out.solve.command=r.previous;out.solve.reason="p6_complete_candidate_budget_exceeded";}
  out.solve.solve_ms=out.total_ms;out.reason=out.solve.reason;return out;
 }
};
}
