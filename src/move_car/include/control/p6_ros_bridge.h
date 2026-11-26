#pragma once
#include "control/p6_controller.h"
#include <ros/ros.h>
#include <move_car/P6ControlRequest.h>
#include <move_car/P6ControlCandidate.h>
#include <stdexcept>
namespace research_control {
inline double p6Number(XmlRpc::XmlRpcValue&v){if(v.getType()==XmlRpc::XmlRpcValue::TypeInt)return static_cast<int>(v);if(v.getType()==XmlRpc::XmlRpcValue::TypeDouble)return static_cast<double>(v);throw std::runtime_error("P6 expected finite numeric field");}
inline P6LayeredConfig loadP6Runtime(const ros::NodeHandle&root,P6SceneManager&scenes){
 ros::NodeHandle n(root,"research_runtime");int schema=0;std::string deployment;n.getParam("schema_version",schema);n.getParam("deployment",deployment);
 if(schema!=2||deployment!="isolated_simulation_only"||n.param("real_vehicle_control_allowed",true)||n.param("weight_adaptation_enabled",true))throw std::runtime_error("P6 isolated schema/permission contract");
 P6LayeredConfig c;if(!n.getParam("config_hash",c.config_hash)||c.config_hash.size()!=64||c.config_hash.find_first_not_of("0123456789abcdef")!=std::string::npos)throw std::runtime_error("P6 nominal SHA missing");
 for(auto&p:c.profiles){ros::NodeHandle profile(n,"profiles/"+p.scene);XmlRpc::XmlRpcValue raw;
  if(!profile.getParam("values",raw)||raw.getType()!=XmlRpc::XmlRpcValue::TypeStruct)throw std::runtime_error("P6 profile values missing");
  std::map<std::string,double> values;for(auto i=raw.begin();i!=raw.end();++i)values[i->first]=p6Number(i->second);
  std::string why;if(!p.load(values,why))throw std::runtime_error("P6 invalid profile "+p.scene+":"+why);
  profile.getParam("version",p.version);p.config_hash=c.config_hash;
  XmlRpc::XmlRpcValue metadata;
  if(!profile.getParam("metadata",metadata)||metadata.getType()!=XmlRpc::XmlRpcValue::TypeStruct)throw std::runtime_error("P6 model provenance missing");
  for(auto it=metadata.begin();it!=metadata.end();++it){auto&entry=it->second;P6ParameterMetadata m;
   if(entry.getType()!=XmlRpc::XmlRpcValue::TypeStruct||!entry.hasMember("unit")||!entry.hasMember("bounds")||!entry.hasMember("valid_speed_range")||!entry.hasMember("uncertainty")||!entry.hasMember("calibration_status")||!entry.hasMember("source"))throw std::runtime_error("P6 model metadata incomplete");
   m.unit=static_cast<std::string>(entry["unit"]);m.calibration_status=static_cast<std::string>(entry["calibration_status"]);m.version=p.version;
   auto&bounds=entry["bounds"];auto&domain=entry["valid_speed_range"];
   if(bounds.getType()!=XmlRpc::XmlRpcValue::TypeArray||domain.getType()!=XmlRpc::XmlRpcValue::TypeArray||bounds.size()!=2||domain.size()!=2)throw std::runtime_error("P6 metadata domain malformed");
   m.lower=p6Number(bounds[0]);m.upper=p6Number(bounds[1]);m.minimum_speed=p6Number(domain[0]);m.maximum_speed=p6Number(domain[1]);
   auto&unc=entry["uncertainty"];if(!unc.hasMember("known")||unc["known"].getType()!=XmlRpc::XmlRpcValue::TypeBoolean)throw std::runtime_error("P6 uncertainty validity missing");
   m.uncertainty_known=static_cast<bool>(unc["known"]);if(m.uncertainty_known){if(!unc.hasMember("stddev"))throw std::runtime_error("P6 known uncertainty lacks value");m.standard_deviation=p6Number(unc["stddev"]);if(m.standard_deviation<0.||!std::isfinite(m.standard_deviation))throw std::runtime_error("P6 invalid uncertainty");}
   if(m.calibration_status=="real_validated"||!std::isfinite(m.lower)||!std::isfinite(m.upper)||m.lower>m.upper||m.minimum_speed<0.||m.maximum_speed>.5||m.minimum_speed>=m.maximum_speed||(values.count(it->first)&&(values.at(it->first)<m.lower||values.at(it->first)>m.upper)))throw std::runtime_error("P6 unapproved calibration or invalid domain");
   auto&source=entry["source"];if(!source.hasMember("kind")||!source.hasMember("reference"))throw std::runtime_error("P6 source evidence missing");m.source=static_cast<std::string>(source["kind"])+":"+static_cast<std::string>(source["reference"]);
   p.metadata[it->first]=m;
  }
  ros::NodeHandle response(profile,"response_policy");std::vector<std::string> channels;
  if(!response.getParam("allowed_channels",channels)||!response.getParam("evidence_kind",p.response_policy_kind)||response.param<std::string>("low_confidence_strategy","")!="bounded_return_to_nominal")throw std::runtime_error("P6 scene response policy missing");
  p.response_channels.insert(channels.begin(),channels.end());response.getParam("evidence_sha",p.response_policy_sha);response.getParam("minimum_confidence",p.response_minimum_confidence);
 }
 for(const std::string name:{"steering","yaw_gain","yaw_disturbance","longitudinal","roughness","slip"}){
  ros::NodeHandle channel(n,"channel_policies/"+name);std::string kind,sha,why;
  if(!channel.getParam("evidence_kind",kind))throw std::runtime_error("P6 policy evidence missing");channel.getParam("evidence_sha",sha);
  std::map<std::string,bool> flags;for(const auto key:{"enabled","delay_verified","bias_verified","timing_verified","allow_receipt_identification"}){
   bool v;if(!channel.getParam(key,v))throw std::runtime_error("P6 policy flag missing");flags[key]=v;}
  std::map<std::string,double> values;if(kind!="missing_evidence"){
   for(const auto&member:p6PolicyScalarMembers()){double v;if(!channel.getParam(member.first,v))throw std::runtime_error("P6 policy scalar missing:"+member.first);values[member.first]=v;}
   int count;if(!channel.getParam("min_samples",count))throw std::runtime_error("P6 min_samples missing");values["min_samples"]=count;
  }
  if(!p6LoadChannelPolicy(values,flags,kind,sha,c.policies[name],why))throw std::runtime_error("P6 policy "+name+":"+why);
 }
 scenes.enabled=n.param("scene_transition/enabled",false);scenes.ramp_s=n.param("scene_transition/ramp_time_s",3.);scenes.dwell_s=n.param("scene_transition/dwell_s",2.);scenes.confidence_hysteresis=n.param("scene_transition/confidence_hysteresis",.15);
 if(scenes.enabled){const auto kind=n.param<std::string>("scene_transition/evidence_kind","");const auto evidence=n.param<std::string>("scene_transition/evidence_reference","");if(kind!="synthetic_validation"||evidence.size()!=71||evidence.compare(0,7,"sha256:")!=0)throw std::runtime_error("P6 transition lacks isolated evidence");}
 if(!std::isfinite(scenes.ramp_s)||scenes.ramp_s<=0.||!std::isfinite(scenes.dwell_s)||scenes.dwell_s<0.||!std::isfinite(scenes.confidence_hysteresis)||scenes.confidence_hysteresis<0.||scenes.confidence_hysteresis>1.)throw std::runtime_error("P6 scene transition invalid");
 std::string why;if(!c.validate(why))throw std::runtime_error(why);return c;
}
inline p6::RawSample p6Raw(const move_car::P6RawMeasurement&m){
 p6::RawSample r;r.value=m.value_si;r.source_stamp=m.source_stamp.isZero()?missing():m.source_stamp.toSec();r.receipt_stamp=m.receipt_stamp.isZero()?missing():m.receipt_stamp.toSec();
 r.valid=m.valid;r.unit_verified=m.unit_verified;r.frame_verified=m.frame_verified;r.hardware_stamp_verified=m.hardware_stamp_verified;
 r.new_frame_known=m.new_frame_known;r.new_frame=m.new_frame;r.independent=m.independent;r.sequence=m.sequence;r.source=m.source;r.unit=m.unit_si;return r;
}
inline p6::SensorSnapshot p6Sensors(const move_car::P6ControlRequest&q){
 p6::SensorSnapshot s;s.now=q.base.state_observation_stamp.toSec();s.run_id=q.base.run_id;s.session_id=q.session_id;s.scene=q.base.terrain;
 s.nominal_config_hash=q.base.config_hash;s.cycle=q.base.cycle;s.synthetic=q.synthetic;s.history_complete=q.base.history_valid;
 s.wheel_speed=p6Raw(q.wheel_speed);s.steering=p6Raw(q.steering);s.yaw_rate=p6Raw(q.yaw_rate);
 s.accel_x=p6Raw(q.accel_x);s.accel_y=p6Raw(q.accel_y);s.accel_z=p6Raw(q.accel_z);s.imu_frame_verified=q.imu_frame_verified;
 s.ground_speed=p6Raw(q.ground_speed);s.ground_frame_verified=q.ground_frame_verified;s.ground_lever_arm_verified=q.ground_lever_arm_verified;return s;
}
inline SolverConfig p6SolverFromSnapshot(const move_car::P6ParameterSnapshot&p){
 SolverConfig c;auto&m=c.model;const auto&v=p.effective_model;
 m.wheelbase=v[0];m.rear_to_center=v[1];m.steering_tau=v[2];m.steering_delay=v[3];m.speed_tau=v[4];m.speed_delay=v[5];
 m.yaw_gain=v[6];m.steering_gain=v[7];m.speed_gain=v[8];m.longitudinal_residual=v[9];m.yaw_residual=v[10];m.lateral_residual=v[11];
 const auto&w=p.effective_weights;c.weights.lateral=w[0];c.weights.heading=w[1];c.weights.speed=w[2];c.weights.smoothness=w[3];
 const auto&l=p.effective_limits;c.limits.speed=l[0];c.limits.steering=l[1];c.limits.steering_rate=l[2];c.limits.acceleration=l[3];c.limits.deceleration=l[4];c.limits.jerk=l[5];c.limits.lateral_acceleration=l[6];
 c.dt=p.prediction_dt;c.budget_ms=p.solve_budget_ms;c.steps=p.horizon_steps;c.knots=p.control_knots;c.iterations=p.solver_iterations;return c;
}
template<class A>inline void p6Model(A&v,const Model&m){
 const double x[]={m.wheelbase,m.rear_to_center,m.steering_tau,m.steering_delay,m.speed_tau,m.speed_delay,m.yaw_gain,m.steering_gain,m.speed_gain,m.longitudinal_residual,m.yaw_residual,m.lateral_residual};
 for(int i=0;i<12;++i)v[i]=x[i];
}
inline void p6Snapshot(move_car::P6ParameterSnapshot&out,const P6EffectiveParameters&p){
 out.nominal_config_hash=p.nominal_config_hash;out.effective_snapshot_hash=p.snapshot_hash;out.scene=p.scene;
 out.cycle=p.cycle;out.history_base_sequence=p.history_base_sequence;out.effective_revision=p.revision;out.estimate_revision=p.response.estimate_revision;out.observation_cutoff.fromSec(p.stamp);
 out.prediction_dt=p.solver.dt;out.solve_budget_ms=p.solver.budget_ms;out.speed_cap=p.speed_cap;out.horizon_steps=p.solver.steps;out.control_knots=p.solver.knots;out.solver_iterations=p.solver.iterations;
 p6Model(out.nominal_model,p.response.nominal);p6Model(out.effective_model,p.solver.model);
 const auto&w=p.solver.weights;out.effective_weights={{w.lateral,w.heading,w.speed,w.smoothness}};
 const auto&l=p.solver.limits;out.effective_limits={{l.speed,l.steering,l.steering_rate,l.acceleration,l.deceleration,l.jerk,l.lateral_acceleration}};
 for(const auto&x:p.response.changes){out.parameter_names.push_back(x.name);out.previous_values.push_back(x.old_value);out.estimates.push_back(x.estimate);out.effective_values.push_back(x.effective);out.confidences.push_back(x.confidence);out.validity.push_back(x.validity);out.update_reasons.push_back(x.reason);}
 const p6::ChannelEstimate* channels[]={&p.response.steering,&p.response.yaw,&p.response.longitudinal};
 for(int i=0;i<3;++i){const auto&c=*channels[i];out.response_residuals[i]=c.residual.value;out.residual_confidences[i]=c.residual.confidence;out.residual_valid[i]=c.residual.valid;
  if(std::isfinite(c.residual.stamp)&&c.residual.stamp>=0.)out.residual_stamps[i].fromSec(c.residual.stamp);
  out.residual_reasons[i]=c.residual.reason;out.estimator_time_domains[i]=c.time_domain;out.estimator_information[i]=c.information;out.estimator_rank_ratios[i]=c.rank_ratio;out.estimator_covariance_traces[i]=c.covariance_trace;
  out.estimator_nominal_mse[i]=c.nominal_mse;out.estimator_candidate_mse[i]=c.candidate_mse;out.estimator_accepted[i]=c.accepted;out.estimator_rejected[i]=c.rejected;}
 for(int i=0;i<7;++i)out.dynamic_flags[i]=p.response.dynamic_flags[i];out.slip_valid=p.response.slip.valid;out.slip_value=p.response.slip.value;out.slip_confidence=p.response.slip.confidence;out.slip_reason=p.response.slip.reason;
 out.roughness_valid=p.response.roughness.valid;out.roughness_rms=p.response.roughness.value;out.roughness_confidence=p.response.roughness.confidence;out.roughness_reason=p.response.roughness.reason;out.valid=p.valid;out.reason=p.response.reason;
}
inline bool p6SnapshotShape(const move_car::P6ControlCandidate&q){
 const auto&p=q.parameters;const auto n=p.parameter_names.size();
 if(q.schema_version!=2||q.session_id.empty()||!p6KnownMode(q.algorithm_mode)||!p6KnownMode(q.computation_mode)||!p.valid||p.nominal_config_hash!=q.base.config_hash||p.cycle!=q.base.cycle||p.history_base_sequence!=q.base.previous_sequence||p.effective_snapshot_hash.size()!=16||n>16||
  p.previous_values.size()!=n||p.estimates.size()!=n||p.effective_values.size()!=n||p.confidences.size()!=n||p.validity.size()!=n||p.update_reasons.size()!=n)return false;
 for(double v:p.effective_model)if(!std::isfinite(v))return false;for(double v:p.effective_weights)if(!std::isfinite(v)||v<=0.)return false;
 for(double v:p.effective_limits)if(!std::isfinite(v)||v<=0.)return false;
 if(q.feedback_count>124||q.feedback_state_dimension>26||q.feedback_gains.size()!=q.feedback_count*q.feedback_state_dimension)return false;
 for(double v:q.feedback_gains)if(!std::isfinite(v))return false;
 if(p.observation_cutoff>q.base.snapshot_stamp||q.validation_observation_cutoff>q.base.header.stamp)return false;
 if(q.base.valid){
  const auto dimension=p6FlatMode(q.computation_mode)||q.computation_mode=="blocked8"?8u:12u;
  if(q.base.decision_dimension!=dimension||q.base.predicted_states.size()!=6*(p.horizon_steps+1)||q.base.predicted_commands.size()!=2*p.horizon_steps)return false;
 }
 P6EffectiveParameters effective;effective.nominal_config_hash=p.nominal_config_hash;effective.scene=p.scene;effective.cycle=p.cycle;effective.history_base_sequence=p.history_base_sequence;effective.stamp=p.observation_cutoff.toSec();
 auto&m=effective.solver.model;const auto&v=p.effective_model;m.wheelbase=v[0];m.rear_to_center=v[1];m.steering_tau=v[2];m.steering_delay=v[3];m.speed_tau=v[4];m.speed_delay=v[5];m.yaw_gain=v[6];m.steering_gain=v[7];m.speed_gain=v[8];m.longitudinal_residual=v[9];m.yaw_residual=v[10];m.lateral_residual=v[11];
 auto&w=effective.solver.weights;w.lateral=p.effective_weights[0];w.heading=p.effective_weights[1];w.speed=p.effective_weights[2];w.smoothness=p.effective_weights[3];
 auto&l=effective.solver.limits;l.speed=p.effective_limits[0];l.steering=p.effective_limits[1];l.steering_rate=p.effective_limits[2];l.acceleration=p.effective_limits[3];l.deceleration=p.effective_limits[4];l.jerk=p.effective_limits[5];l.lateral_acceleration=p.effective_limits[6];
 effective.solver.dt=p.prediction_dt;effective.solver.budget_ms=p.solve_budget_ms;effective.solver.steps=p.horizon_steps;effective.solver.knots=p.control_knots;effective.solver.iterations=p.solver_iterations;effective.speed_cap=p.speed_cap;
 return p.effective_snapshot_hash==p6SnapshotIdentity(effective)&&std::isfinite(q.feedback_correction)&&std::isfinite(q.feedback_ms)&&q.feedback_ms>=0.;
}
}
