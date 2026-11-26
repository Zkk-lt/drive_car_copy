#pragma once
#include "control/research_types.h"
#include "control/p6_response_model.h"
#include <cstdint>
#include <map>
#include <set>

namespace research_control {
// Typed normalized configuration. YAML parsing stays in the existing Python
// loader; ROS adapters must construct this type, never mutate an active solve.
struct P6ParameterMetadata {
  std::string unit,source,version,calibration_status="unverified";
  double lower=missing(),upper=missing(),minimum_speed=0.,maximum_speed=.5;
  double standard_deviation=missing();
  bool uncertainty_known=false;
};
// The estimator's concrete policy type is the only runtime policy. No second
// set of estimator constants is hidden in the configuration adapter.
using P6ChannelPolicy = p6::ChannelPolicy;
inline const std::map<std::string,double P6ChannelPolicy::*>& p6PolicyScalarMembers(){
  static const std::map<std::string,double P6ChannelPolicy::*> fields={
    {"stale_s",&P6ChannelPolicy::stale_s},{"sync_s",&P6ChannelPolicy::sync_s},
    {"min_speed",&P6ChannelPolicy::min_speed},{"min_excitation",&P6ChannelPolicy::min_excitation},
    {"min_gain",&P6ChannelPolicy::min_gain},{"max_gain",&P6ChannelPolicy::max_gain},
    {"min_tau",&P6ChannelPolicy::min_tau},{"max_tau",&P6ChannelPolicy::max_tau},
    {"gain_rate",&P6ChannelPolicy::gain_rate},{"tau_rate",&P6ChannelPolicy::tau_rate},
    {"return_rate",&P6ChannelPolicy::return_rate},{"forgetting_s",&P6ChannelPolicy::forgetting_s},
    {"filter_s",&P6ChannelPolicy::filter_s},{"min_information",&P6ChannelPolicy::min_information},
    {"min_rank_ratio",&P6ChannelPolicy::min_rank_ratio},{"max_innovation",&P6ChannelPolicy::max_innovation},
    {"max_covariance",&P6ChannelPolicy::max_covariance},{"prediction_improvement",&P6ChannelPolicy::prediction_improvement},
    {"min_confidence",&P6ChannelPolicy::min_confidence},
    {"residual_bound",&P6ChannelPolicy::residual_bound},{"residual_rate",&P6ChannelPolicy::residual_rate},
    {"residual_filter_s",&P6ChannelPolicy::residual_filter_s},
    {"condition_enter_fraction",&P6ChannelPolicy::condition_enter_fraction},
    {"condition_exit_fraction",&P6ChannelPolicy::condition_exit_fraction},
    {"condition_dwell_s",&P6ChannelPolicy::condition_dwell_s}};
  return fields;
}
inline bool p6PolicyValid(const P6ChannelPolicy& p) {
  if(!p.enabled&&p.evidence_kind=="missing_evidence")return true;
  if(p.evidence_kind!="synthetic_validation"||p.evidence_sha.size()!=64||
     p.evidence_sha.find_first_not_of("0123456789abcdefABCDEF")!=std::string::npos)return false;
  const double nonnegative[]={p.sync_s,p.min_speed,p.min_excitation,p.gain_rate,p.tau_rate,p.return_rate,
    p.min_information,p.max_innovation,p.max_covariance,p.prediction_improvement,p.residual_bound,
    p.residual_rate,p.condition_enter_fraction,p.condition_exit_fraction,p.condition_dwell_s};
  for(double x:nonnegative)if(!std::isfinite(x)||x<0.)return false;
  return std::isfinite(p.stale_s)&&p.stale_s>0.&&p.sync_s<=p.stale_s&&
    std::isfinite(p.min_gain)&&std::isfinite(p.max_gain)&&p.min_gain>0.&&p.max_gain>=p.min_gain&&
    std::isfinite(p.min_tau)&&std::isfinite(p.max_tau)&&p.min_tau>0.&&p.max_tau>=p.min_tau&&
    std::isfinite(p.forgetting_s)&&p.forgetting_s>0.&&std::isfinite(p.filter_s)&&p.filter_s>0.&&
    std::isfinite(p.residual_filter_s)&&p.residual_filter_s>0.&&
    std::isfinite(p.min_rank_ratio)&&p.min_rank_ratio>0.&&p.min_rank_ratio<=1.&&p.min_samples>=2&&
    std::isfinite(p.min_confidence)&&p.min_confidence>0.&&p.min_confidence<=1.&&
    p.condition_exit_fraction<=p.condition_enter_fraction;
}
inline bool p6LoadChannelPolicy(const std::map<std::string,double>& values,
    const std::map<std::string,bool>& flags,const std::string& evidence_kind,
    const std::string& evidence_sha,P6ChannelPolicy& policy,std::string& reason){
  const std::set<std::string> expected_flags={"enabled","delay_verified","bias_verified","timing_verified","allow_receipt_identification"};
  std::set<std::string> actual;for(const auto& f:flags)actual.insert(f.first);
  if(actual!=expected_flags){reason="policy_boolean_fields";return false;}
  if(evidence_kind=="missing_evidence"){
    for(const auto& f:flags)if(f.second){reason="missing_policy_evidence";return false;}
    if(!values.empty()||!evidence_sha.empty()){reason="missing_evidence_cannot_supply_thresholds";return false;}
    policy=P6ChannelPolicy{};reason="missing_evidence";return true;
  }
  std::set<std::string> expected;for(const auto& f:p6PolicyScalarMembers())expected.insert(f.first);expected.insert("min_samples");
  actual.clear();for(const auto& f:values){actual.insert(f.first);if(!std::isfinite(f.second)){reason="nonfinite_policy_threshold";return false;}}
  if(actual!=expected){reason="missing_or_unknown_policy_threshold";return false;}
  P6ChannelPolicy out;out.enabled=flags.at("enabled");out.delay_verified=flags.at("delay_verified");
  out.bias_verified=flags.at("bias_verified");out.timing_verified=flags.at("timing_verified");
  out.allow_receipt_identification=flags.at("allow_receipt_identification");
  out.evidence_kind=evidence_kind;out.evidence_sha=evidence_sha;
  for(const auto& f:p6PolicyScalarMembers())out.*(f.second)=values.at(f.first);
  if(std::floor(values.at("min_samples"))!=values.at("min_samples")||values.at("min_samples")<2.||
     values.at("min_samples")>std::numeric_limits<int>::max()){reason="policy_min_samples_integer";return false;}
  out.min_samples=static_cast<int>(values.at("min_samples"));
  if(!p6PolicyValid(out)){reason="invalid_active_policy";return false;}
  policy=out;reason="normalized_estimator_policy_loaded";return true;
}
struct P6NominalProfile {
  SolverConfig solver;
  double speed_cap=.1;
  double physical_steering_limit=.38,collision_stop_distance=.5,follow_distance_min=.5;
  double reference_hard_timeout=1.2;
  std::string scene="unknown",version,config_hash,status="research_unvalidated";
  std::map<std::string,P6ParameterMetadata> metadata;
  std::set<std::string> response_channels;
  std::string response_policy_kind="missing_evidence",response_policy_sha;
  double response_minimum_confidence=missing();
  // A normalized profile is loaded atomically into a temporary. Unknown keys,
  // incomplete inputs and widened launch safety anchors are rejected.
  bool load(const std::map<std::string,double>& v,std::string& reason) {
    const std::set<std::string> expected={
      "wheelbase","rear_to_center","wheel_speed_scale","steering_tau","steering_delay",
      "steering_gain","speed_tau","speed_delay","speed_gain","yaw_gain","control_rate",
      "prediction_dt","horizon_steps","control_knots","solver_max_iterations","solver_budget_ms",
      "weight_lateral","weight_heading","weight_speed","weight_smoothness","speed_cap",
      "lateral_weight_scale","heading_weight_scale","speed_weight_scale","smoothness_weight_scale",
      "max_speed","max_steer","solver_max_steer","max_steer_rate","max_accel","max_decel",
      "max_jerk","max_lateral_accel","collision_stop_distance","follow_distance_min","reference_hard_timeout"};
    std::set<std::string> actual;
    for(const auto& p:v){actual.insert(p.first);if(!std::isfinite(p.second)){reason="nonfinite_parameter:"+p.first;return false;}}
    if(actual!=expected){reason="missing_or_unknown_parameters";return false;}
    const std::map<std::string,double> safety={{"max_speed",.5},{"max_steer",.38},{"solver_max_steer",.32},
      {"max_steer_rate",1.},{"max_accel",.28},{"max_decel",.36},{"max_jerk",.8},{"max_lateral_accel",.25},
      {"collision_stop_distance",.5},{"follow_distance_min",.5},{"reference_hard_timeout",1.2}};
    for(const auto& p:safety)if(v.at(p.first)!=p.second){reason="immutable_safety:"+p.first;return false;}
    if(v.at("wheelbase")!=.6||v.at("rear_to_center")!=.3||v.at("wheel_speed_scale")!=.001||v.at("control_rate")!=30.){
      reason="immutable_geometry_or_unit_contract";return false;}
    const auto bounded=[&](const char* k,double a,double b){return v.at(k)>=a&&v.at(k)<=b;};
    if(!bounded("steering_tau",.05,1.)||!bounded("speed_tau",.05,1.)||
       !bounded("steering_delay",0.,.7)||!bounded("speed_delay",0.,.7)||
       !bounded("steering_gain",.5,1.5)||!bounded("speed_gain",.5,1.5)||!bounded("yaw_gain",.5,1.5)||
       !bounded("prediction_dt",.05,.2)||!bounded("horizon_steps",10,40)||!bounded("control_knots",4,10)||
       !bounded("solver_max_iterations",1,5)||!bounded("solver_budget_ms",1,8)||!bounded("speed_cap",0.,scene=="smooth"?.4:.3)||
       !bounded("weight_lateral",1,100)||!bounded("weight_heading",1,100)||
       !bounded("weight_speed",1,40)||!bounded("weight_smoothness",1,100)||
       !bounded("lateral_weight_scale",.75,1.5)||!bounded("heading_weight_scale",.75,1.5)||
       !bounded("speed_weight_scale",.5,1.25)||!bounded("smoothness_weight_scale",1.,2.)){
      reason="model_or_performance_outside_research_domain";return false;}
    if(std::floor(v.at("horizon_steps"))!=v.at("horizon_steps")||
       std::floor(v.at("control_knots"))!=v.at("control_knots")||
       std::floor(v.at("solver_max_iterations"))!=v.at("solver_max_iterations")||
       v.at("control_knots")>v.at("horizon_steps")||
       (scene=="unknown"&&v.at("speed_cap")>.1)){
      reason="invalid_discretization_or_unknown_speed";return false;}
    P6NominalProfile out=*this;
    Model& m=out.solver.model;Limits& l=out.solver.limits;Weights& w=out.solver.weights;
    m.wheelbase=v.at("wheelbase");m.rear_to_center=v.at("rear_to_center");
    m.steering_tau=v.at("steering_tau");m.steering_delay=v.at("steering_delay");m.steering_gain=v.at("steering_gain");
    m.speed_tau=v.at("speed_tau");m.speed_delay=v.at("speed_delay");m.speed_gain=v.at("speed_gain");m.yaw_gain=v.at("yaw_gain");
    out.solver.dt=v.at("prediction_dt");out.solver.steps=static_cast<int>(v.at("horizon_steps"));
    out.solver.knots=static_cast<int>(v.at("control_knots"));out.solver.iterations=static_cast<int>(v.at("solver_max_iterations"));
    out.solver.budget_ms=v.at("solver_budget_ms");
    w.lateral=v.at("weight_lateral")*v.at("lateral_weight_scale");w.heading=v.at("weight_heading")*v.at("heading_weight_scale");
    w.speed=v.at("weight_speed")*v.at("speed_weight_scale");w.smoothness=v.at("weight_smoothness")*v.at("smoothness_weight_scale");
    l.speed=v.at("max_speed");l.steering=v.at("solver_max_steer");l.steering_rate=v.at("max_steer_rate");
    l.acceleration=v.at("max_accel");l.deceleration=v.at("max_decel");l.jerk=v.at("max_jerk");l.lateral_acceleration=v.at("max_lateral_accel");
    out.speed_cap=v.at("speed_cap");out.physical_steering_limit=v.at("max_steer");
    out.collision_stop_distance=v.at("collision_stop_distance");out.follow_distance_min=v.at("follow_distance_min");
    out.reference_hard_timeout=v.at("reference_hard_timeout");
    *this=out;reason="normalized_profile_loaded";return true;
  }
};
struct P6LayeredConfig {
  std::array<P6NominalProfile,4> profiles;
  std::map<std::string,P6ChannelPolicy> policies;
  std::string config_hash;
  bool weights_online_enabled=false,real_vehicle_control_allowed=false;
  P6LayeredConfig(){profiles[0].scene="smooth";profiles[1].scene="grass_bumpy";profiles[2].scene="asphalt";profiles[3].scene="unknown";}
  bool validate(std::string& reason) const {
    if(weights_online_enabled||real_vehicle_control_allowed){reason="research_permission_guard";return false;}
    if(config_hash.empty()){reason="missing_config_identity";return false;}
    const std::set<std::string> expected={"steering","yaw_gain","yaw_disturbance","longitudinal","roughness","slip"};
    std::set<std::string> actual;for(const auto& p:policies){actual.insert(p.first);if(!p6PolicyValid(p.second)){reason="invalid_policy:"+p.first;return false;}}
    if(actual!=expected){reason="missing_or_unknown_channels";return false;}
    if(policies.at("slip").enabled){reason="slip_independent_contract_not_reviewed";return false;}
    if(policies.at("yaw_gain").enabled&&policies.at("yaw_disturbance").enabled){reason="duplicate_yaw_compensation";return false;}
    for(const auto&p:profiles)if(!p.response_channels.empty()){
      if(p.scene=="unknown"||p.response_policy_kind!="synthetic_validation"||p.response_policy_sha.size()!=64||
        !std::isfinite(p.response_minimum_confidence)||p.response_minimum_confidence<=0.||p.response_minimum_confidence>1.){reason="scene_response_evidence_missing";return false;}
      for(const auto&name:p.response_channels)if(!expected.count(name)){reason="unknown_scene_response_channel";return false;}
    }
    reason="isolated_config_valid";return true;
  }
};
inline p6::ResponsePolicy p6ResponsePolicy(const P6LayeredConfig& config) {
  p6::ResponsePolicy p;
  p.steering=config.policies.at("steering");p.longitudinal=config.policies.at("longitudinal");
  const auto& gain=config.policies.at("yaw_gain");const auto& disturbance=config.policies.at("yaw_disturbance");
  p.yaw=disturbance.enabled?disturbance:gain;p.yaw_additive_enabled=disturbance.enabled;
  p.vibration=config.policies.at("roughness");p.slip=config.policies.at("slip");return p;
}
struct P6EffectiveParameters {
  // Passed by const value/reference for one complete prediction + solve cycle.
  SolverConfig solver;
  std::string scene="unknown",nominal_config_hash,snapshot_hash;
  std::uint64_t revision=0,cycle=0,history_base_sequence=0;
  double stamp=missing(),speed_cap=.1;
  bool valid=false;
  p6::ParameterSnapshot response;
};
// Scene selection is independent of composable response flags. The three
// transition constants are inherited, explicitly unvalidated research settings.
class P6SceneManager {
 P6NominalProfile current_;std::string pending_;double pending_since_=0.,last_=0.;bool initialized_=false;
public:
 bool enabled=false;double ramp_s=3.,dwell_s=2.,confidence_hysteresis=.15;
 P6NominalProfile update(const P6LayeredConfig&config,const std::string&requested,double now,double confidence=1.){
  int index=3;for(int i=0;i<4;++i)if(config.profiles[i].scene==requested)index=i;
  const auto&target=config.profiles[index];
  if(!initialized_){current_=target;last_=now;initialized_=true;return current_;}
  const double dt=std::max(0.,std::min(.25,now-last_));last_=now;
  if(target.scene!=current_.scene){
   if(target.scene!="unknown"&&(!enabled||confidence<1.-confidence_hysteresis)){current_.speed_cap=std::min(current_.speed_cap,target.speed_cap);current_.response_channels.clear();return current_;}
   if(pending_!=target.scene){pending_=target.scene;pending_since_=now;}
   // Conservative speed restriction takes effect while a request is pending.
   current_.speed_cap=std::min(current_.speed_cap,target.speed_cap);
   if(target.scene!="unknown"&&now-pending_since_<dwell_s)return current_;
   current_.scene=target.scene;current_.config_hash=target.config_hash;current_.version=target.version;
   current_.response_channels=target.response_channels;current_.response_policy_kind=target.response_policy_kind;current_.response_policy_sha=target.response_policy_sha;current_.response_minimum_confidence=target.response_minimum_confidence;
   current_.metadata=target.metadata;
  }else pending_.clear();
  const double alpha=1.-std::exp(-dt/std::max(.001,ramp_s));
  auto move=[&](double& a,double b){a+=alpha*(b-a);};
  auto&m=current_.solver.model;const auto&n=target.solver.model;
  move(m.steering_gain,n.steering_gain);move(m.steering_tau,n.steering_tau);move(m.steering_delay,n.steering_delay);
  move(m.speed_gain,n.speed_gain);move(m.speed_tau,n.speed_tau);move(m.speed_delay,n.speed_delay);move(m.yaw_gain,n.yaw_gain);
  move(m.yaw_residual,n.yaw_residual);move(m.longitudinal_residual,n.longitudinal_residual);move(m.lateral_residual,n.lateral_residual);
  move(current_.solver.weights.lateral,target.solver.weights.lateral);move(current_.solver.weights.heading,target.solver.weights.heading);
  move(current_.solver.weights.speed,target.solver.weights.speed);move(current_.solver.weights.smoothness,target.solver.weights.smoothness);
  if(target.speed_cap<current_.speed_cap)current_.speed_cap=target.speed_cap;else move(current_.speed_cap,target.speed_cap);
  if(current_.scene=="unknown")current_.speed_cap=std::min(.1,current_.speed_cap);
  return current_;
 }
};
inline Limits p6SafetyIntersection(const Limits& safety,const Limits& conservative){
  Limits r=safety;r.speed=std::min(r.speed,conservative.speed);r.steering=std::min(r.steering,conservative.steering);
  r.steering_rate=std::min(r.steering_rate,conservative.steering_rate);r.acceleration=std::min(r.acceleration,conservative.acceleration);
  r.deceleration=std::min(r.deceleration,conservative.deceleration);r.jerk=std::min(r.jerk,conservative.jerk);
  r.lateral_acceleration=std::min(r.lateral_acceleration,conservative.lateral_acceleration);return r;
}
inline Command p6ConservativeSpeedFallback(Command legacy,const Command&previous,double previous_a,
 double dt,double operating_cap,const Limits&physical){
 if(!std::isfinite(dt)||dt<=0.||!std::isfinite(operating_cap)||operating_cap<0.)return legacy;
 if(legacy.speed<=operating_cap&&previous.speed<=operating_cap)return legacy;
 double wanted=clamp((operating_cap-previous.speed)/dt,-physical.deceleration,physical.acceleration);
 if(previous.speed<operating_cap&&wanted>0.){
  const double jdt=physical.jerk*dt;
  wanted=std::min(wanted,std::sqrt(jdt*jdt+2.*physical.jerk*(operating_cap-previous.speed))-jdt);
 }
 const double a=clamp(wanted,previous_a-physical.jerk*dt,previous_a+physical.jerk*dt);
 // The operating target may be crossed temporarily while jerk-limited recovery
 // starts. Immutable physical speed remains a hard boundary throughout.
 legacy.speed=std::min(legacy.speed,clamp(previous.speed+a*dt,0.,physical.speed));return legacy;
}
} // namespace research_control
