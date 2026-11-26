#pragma once
// P6 response identification only. No ROS publisher and no dependency on NED.
// Threshold defaults are inert until the caller supplies evidence and explicitly
// enables a channel. A synthetic policy is usable only on synthetic observations.
#include "control/research_types.h"
#include <array>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <vector>

namespace research_control { namespace p6 {
struct RawSample {
 double value=missing(),source_stamp=missing(),receipt_stamp=missing();
 bool valid=false,unit_verified=false,frame_verified=false;
 bool hardware_stamp_verified=false,new_frame_known=false,new_frame=false,independent=false;
 std::uint64_t sequence=0;
 std::string source="unknown",unit="unknown";
};
struct SensorSnapshot {
 double now=0.; RawSample wheel_speed,steering,yaw_rate,accel_x,accel_y,accel_z;
 RawSample ground_speed,heading_velocity;
 bool synthetic=false,history_complete=false,imu_frame_verified=false;
 bool ground_frame_verified=false,ground_lever_arm_verified=false;
 std::string run_id,session_id,scene="unknown",nominal_config_hash;
 std::uint64_t cycle=0;
};
struct PublishedSample {
 std::uint64_t sequence=0; double stamp=0.; Command command;
 bool published=false,committed=false;
};
struct ChannelPolicy {
 bool enabled=false,delay_verified=false,bias_verified=false,timing_verified=false;
 bool allow_receipt_identification=false;
 std::string evidence_kind="missing_evidence",evidence_sha;
 // Values are legacy design bounds, NOT calibration claims; all updates are
 // disabled unless evidence_kind/SHA and sensor contracts permit the channel.
 double stale_s=.25,sync_s=.08,min_speed=.15,min_excitation=.005;
 double min_gain=.5,max_gain=1.5,min_tau=.05,max_tau=1.5,min_confidence=.8;
 double gain_rate=.05,tau_rate=.03,return_rate=.05;
 double forgetting_s=6.,filter_s=1.,min_information=.0001,min_rank_ratio=.0001;
 double max_innovation=.35,max_covariance=100.,prediction_improvement=.01;
 double residual_bound=.03,residual_rate=.01,residual_filter_s=.4;
 double condition_enter_fraction=.10,condition_exit_fraction=.05,condition_dwell_s=2.;
 int min_samples=30;
};
struct ResponsePolicy {
 ChannelPolicy steering,longitudinal,yaw,vibration,slip;
 bool yaw_additive_enabled=false; // Never simultaneous with free yaw-gain fit.
 double speed_domain_min=0.,speed_domain_max=.5;
 double slip_min_speed=.15,slip_epsilon=.05;
};
struct ChannelEstimate {
 Estimate gain,tau,residual;
 double nominal_mse=missing(),candidate_mse=missing(),rank_ratio=0.,information=0.;
 double covariance_trace=missing(); // RLS numerical uncertainty, not a calibrated physical CI.
 std::uint64_t accepted=0,rejected=0;
 bool update_allowed=false;
 std::string reason="unobserved",time_domain="unknown";
};
struct ParameterChange {
 std::string name,reason="nominal",validity="invalid",source="unknown";
 double old_value=missing(),nominal=missing(),estimate=missing(),effective=missing(),confidence=0.;
};
struct ParameterSnapshot {
 Model nominal,effective;
 ChannelEstimate steering,longitudinal,yaw;
 Estimate roughness,slip,sideslip;
 std::array<ParameterChange,9> changes{};
 std::array<bool,7> dynamic_flags{{false,false,false,false,false,false,true}};
 // Normal, yaw underresponse, steering lag, longitudinal weak, vibration,
 // credible slip, low confidence. Flags are composable, not terrain labels.
 bool valid=false; double reference_time=0.;
 std::uint64_t cycle=0,estimate_revision=0,effective_revision=0,history_base_sequence=0;
 std::string run_id,session_id,scene,nominal_config_hash,effective_hash,reason="uninitialized";
};

inline std::string responseHash(const ParameterSnapshot& s) {
 std::ostringstream o;o<<std::setprecision(17)<<s.nominal_config_hash<<'|'<<s.cycle<<'|'
 <<s.effective.steering_gain<<'|'<<s.effective.steering_tau<<'|'<<s.effective.steering_delay<<'|'
 <<s.effective.speed_gain<<'|'<<s.effective.speed_tau<<'|'<<s.effective.speed_delay<<'|'
 <<s.effective.yaw_gain<<'|'<<s.effective.yaw_residual<<'|'<<s.effective.longitudinal_residual<<'|'
 <<s.effective.wheelbase<<'|'<<s.effective.rear_to_center<<'|'<<s.effective.lateral_residual;
 std::uint64_t h=1469598103934665603ULL;for(unsigned char c:o.str()){h^=c;h*=1099511628211ULL;}
 std::ostringstream hex;hex<<std::hex<<h;return hex.str(); // Identity only, not a cryptographic approval.
}

class ResponseEstimator {
 struct RLS {
  double a=0.,b=0.,p00=100.,p01=0.,p11=100.;
  double g00=0.,g01=0.,g11=0.,candidate_mse=0.,nominal_mse=0.;
  std::uint64_t n=0,rejected=0; bool initialized=false;
  double last_update_time=missing();
 } steer_fit_,speed_fit_;
 struct Previous { RawSample value; bool valid=false; } steer_previous_,speed_previous_;
 Model effective_; bool effective_initialized_=false;
 double last_time_=missing(),yaw_fit_=1.,yaw_information_=0.,yaw_candidate_mse_=0.,yaw_nominal_mse_=0.;
 double yaw_last_fit_time_=missing();
 double yaw_filtered_=0.,vibration_mean_=0.,vibration_var_=0.;
 std::uint64_t yaw_samples_=0,yaw_rejected_=0,estimate_revision_=0,effective_revision_=0;
 double previous_yaw_stamp_=missing(); std::uint64_t previous_yaw_sequence_=0;
 double previous_vibration_stamp_=missing(); std::uint64_t previous_vibration_sequence_=0;
 std::string previous_scene_,previous_nominal_hash_;
 std::array<double,7> flag_since_{{0.,0.,0.,0.,0.,0.,0.}};
 std::array<bool,7> flags_{{false,false,false,false,false,false,true}};
 static bool finite(double v){return std::isfinite(v);}
 static double blend(double dt,double tau){return 1.-std::exp(-dt/std::max(1e-6,tau));}
 static double approach(double old,double target,double rate,double dt){return old+clamp(target-old,-rate*dt,rate*dt);}
 static bool modelValid(const Model& m) {
  return finite(m.wheelbase)&&m.wheelbase>0.&&finite(m.steering_gain)&&m.steering_gain>0.&&
   finite(m.speed_gain)&&m.speed_gain>0.&&finite(m.steering_tau)&&m.steering_tau>0.&&
   finite(m.speed_tau)&&m.speed_tau>0.&&finite(m.steering_delay)&&m.steering_delay>=0.&&
   finite(m.speed_delay)&&m.speed_delay>=0.&&finite(m.yaw_gain)&&m.yaw_gain>0.&&
   finite(m.yaw_residual)&&finite(m.longitudinal_residual)&&finite(m.lateral_residual);
 }
 static bool policyValid(const ChannelPolicy& p) {
  const std::array<double,25> a{{p.stale_s,p.sync_s,p.min_speed,p.min_excitation,p.min_gain,p.max_gain,
   p.min_tau,p.max_tau,p.gain_rate,p.tau_rate,p.return_rate,p.forgetting_s,p.filter_s,p.min_information,
   p.min_rank_ratio,p.max_innovation,p.max_covariance,p.prediction_improvement,p.residual_bound,
   p.residual_rate,p.residual_filter_s,p.condition_enter_fraction,p.condition_exit_fraction,p.condition_dwell_s,p.min_confidence}};
  for(double x:a)if(!finite(x)||x<0.)return false;
  return p.stale_s>0.&&p.min_gain>0.&&p.max_gain>=p.min_gain&&p.min_tau>0.&&p.max_tau>=p.min_tau&&
   p.forgetting_s>0.&&p.filter_s>0.&&p.residual_filter_s>0.&&p.min_rank_ratio<1.&&
   p.max_innovation>0.&&p.max_covariance>0.&&p.prediction_improvement<1.&&p.min_samples>=2&&
   p.condition_exit_fraction<=p.condition_enter_fraction&&p.min_confidence<=1.;
 }
 static double stamp(const RawSample& s){return s.hardware_stamp_verified?s.source_stamp:s.receipt_stamp;}
 static bool sampleValid(const RawSample& s,double now,const ChannelPolicy& p,std::string& reason) {
  if(!s.valid||!finite(s.value)){reason="raw_measurement_missing";return false;}
  if(!s.unit_verified||!s.frame_verified){reason="units_or_frame_unverified";return false;}
  if(!finite(s.receipt_stamp)||s.receipt_stamp>now){reason="future_or_missing_receipt";return false;}
  // A future source stamp is not silently replaced by receipt-domain timing.
  if(finite(s.source_stamp)&&s.source_stamp>now){reason="future_source_sample";return false;}
  const double t=stamp(s);
  if(!finite(t)||t>now||now-t>p.stale_s){reason="stale_or_unknown_sample_time";return false;}
  if(s.new_frame_known&&!s.new_frame){reason="not_a_new_hardware_frame";return false;}
  reason="valid_measurement";return true;
 }
 static bool evidence(const ChannelPolicy& p,const SensorSnapshot& s,std::string& reason) {
  if(!p.enabled){reason="channel_disabled";return false;}
  if(p.evidence_sha.size()!=64||p.evidence_sha.find_first_not_of("0123456789abcdefABCDEF")!=std::string::npos||p.evidence_kind=="missing_evidence"){reason="missing_evidence";return false;}
  if(p.evidence_kind=="synthetic_validation"&&!s.synthetic){reason="synthetic_policy_on_real_data";return false;}
  if(p.evidence_kind!="synthetic_validation"&&p.evidence_kind!="independently_validated"){
   reason="unsupported_evidence_status";return false;
  }
  return true;
 }
 static bool commandIntegral(const std::vector<PublishedSample>& history,double begin,double end,
                             bool steering,double now,double& result,std::string& reason) {
  if(!(end>begin)){reason="invalid_integral_interval";return false;}
  int first=-1;for(std::size_t i=0;i<history.size();++i){
   const auto& q=history[i];
   if(!q.published||!q.committed||!finite(q.stamp)||q.stamp>now||
      !finite(q.command.speed)||!finite(q.command.steering)){reason="uncommitted_or_future_history";return false;}
   if(i&&(q.sequence!=history[i-1].sequence+1||q.stamp<=history[i-1].stamp)){
    // A gap older than the integration interval does not corrupt a new window.
    if(q.stamp>begin&&history[i-1].stamp<end){reason="command_sequence_gap";return false;}
   }
   if(q.stamp<=begin)first=static_cast<int>(i);
  }
  if(first<0){reason="command_history_prefix_missing";return false;}
  result=0.;double cursor=begin;Command u=history[first].command;
  for(std::size_t i=static_cast<std::size_t>(first+1);i<history.size()&&history[i].stamp<end;++i){
   const double t=history[i].stamp;if(t>cursor){result+=(t-cursor)*(steering?u.steering:u.speed);cursor=t;}
   u=history[i].command;
  }
  result+=(end-cursor)*(steering?u.steering:u.speed);return finite(result);
 }
 static void invalid(ChannelEstimate& out,const std::string& reason){out.reason=reason;out.gain.reason=out.tau.reason=reason;if(!out.residual.valid)out.residual.reason=reason;}
 void integralFit(const SensorSnapshot& s,const RawSample& raw,const ChannelPolicy& p,
                  const std::vector<PublishedSample>& history,bool steering,const Model& nominal,
                  RLS& fit,Previous& previous,ChannelEstimate& out) {
  std::string reason; const double t=stamp(raw);
  if(!sampleValid(raw,s.now,p,reason)){invalid(out,reason);previous.valid=false;return;}
  out.time_domain=raw.hardware_stamp_verified?"verified_sensor_time":"command_to_receipt_domain";
  const double tau=steering?nominal.steering_tau:nominal.speed_tau;
  const double gain=steering?nominal.steering_gain:nominal.speed_gain;
  const double delay=steering?nominal.steering_delay:nominal.speed_delay;
  if(!previous.valid){previous.value=raw;previous.valid=true;invalid(out,"need_previous_feedback");return;}
  const RawSample old=previous.value;const double dt=t-stamp(old);
  if(!(dt>0.)||dt>p.stale_s){previous.value=raw;invalid(out,"duplicate_or_feedback_gap");return;}
  previous.value=raw;
  if(raw.sequence&&old.sequence&&raw.sequence<=old.sequence){invalid(out,"feedback_sequence_not_increasing");return;}
  if(!s.history_complete){invalid(out,"command_history_incomplete");return;}
  double integral=0.;
  if(!commandIntegral(history,stamp(old)-delay,t-delay,steering,s.now,integral,reason)){invalid(out,reason);return;}
  // Trapezoidal feedback integration is an explicit numerical approximation;
  // it does not create intermediate measured events or hardware timestamps.
  const double measured=.5*(old.value+raw.value)*dt;
  const double response=raw.value-old.value;
  // Known additive acceleration is part of the nominal plant, not a gain/tau mismatch.
  const double additive=steering?0.:nominal.longitudinal_residual;
  const double nominal_response=gain/tau*integral-measured/tau+additive*dt;
  out.residual.value=(response-nominal_response)/dt;out.residual.stamp=t;
  out.residual.valid=std::fabs(out.residual.value)<=p.max_innovation;
  out.residual.confidence=raw.hardware_stamp_verified?.8:.35;
  out.residual.reason=out.residual.valid?"integral_response_residual":"response_innovation_outlier";
  if(!out.residual.valid){++fit.rejected;invalid(out,"response_innovation_outlier");return;}
  if(!evidence(p,s,reason)){invalid(out,reason);return;}
  if(!p.delay_verified||!p.bias_verified){invalid(out,"delay_or_bias_not_independently_verified");return;}
  if(!p.timing_verified||(!raw.hardware_stamp_verified&&!p.allow_receipt_identification)){
   invalid(out,"physical_time_unverified");return;
  }
  if(!raw.new_frame_known){invalid(out,"new_frame_unverified");return;}
  if(!steering&&std::fabs(raw.value)<p.min_speed){invalid(out,"low_speed_unobservable");return;}
  if(std::fabs(integral/dt)<p.min_excitation&&std::fabs(raw.value-old.value)<p.min_excitation){
   invalid(out,"weak_excitation");return;
  }
  if(finite(fit.last_update_time)&&t-fit.last_update_time>p.stale_s)fit=RLS();
  if(!fit.initialized){fit.a=gain/tau;fit.b=1./tau;fit.initialized=true;}
  const double x0=integral/dt,x1=-measured/dt,y=response/dt-additive;
  const double candidate=fit.a*x0+fit.b*x1,baseline=gain/tau*x0+x1/tau;
  const double innovation=y-candidate;
  if(!finite(innovation)||std::fabs(innovation)>p.max_innovation){++fit.rejected;invalid(out,"rls_innovation_rejected");return;}
  const double lambda=std::exp(-dt/std::max(dt,p.forgetting_s));
  const double px0=fit.p00*x0+fit.p01*x1,px1=fit.p01*x0+fit.p11*x1;
  const double denom=lambda+x0*px0+x1*px1;
  if(!(denom>1e-12)){invalid(out,"rls_covariance_invalid");return;}
  const double k0=px0/denom,k1=px1/denom;
  fit.a+=k0*innovation;fit.b+=k1*innovation;
  fit.p00=(fit.p00-k0*px0)/lambda;fit.p01=(fit.p01-k0*px1)/lambda;
  fit.p11=(fit.p11-k1*px1)/lambda;
  // Gramian is normalized by regressor energy before the rank test, keeping
  // gain/tau confidence independent of the choice of physical angle units.
  const double energy=std::max(1e-12,x0*x0+x1*x1);
  fit.g00=lambda*fit.g00+x0*x0/energy;fit.g01=lambda*fit.g01+x0*x1/energy;
  fit.g11=lambda*fit.g11+x1*x1/energy;
  const double ema=blend(dt,p.filter_s);
  if(!fit.n){fit.candidate_mse=innovation*innovation;fit.nominal_mse=(y-baseline)*(y-baseline);}
  else{fit.candidate_mse+=ema*(innovation*innovation-fit.candidate_mse);fit.nominal_mse+=ema*((y-baseline)*(y-baseline)-fit.nominal_mse);}
  ++fit.n;++estimate_revision_;fit.last_update_time=t;
  const double trace=fit.g00+fit.g11,det=fit.g00*fit.g11-fit.g01*fit.g01;
  const double disc=std::sqrt(std::max(0.,trace*trace-4.*det));
  const double lo=.5*(trace-disc),hi=.5*(trace+disc);
  out.rank_ratio=hi>0.?lo/hi:0.;out.information=lo;
  out.covariance_trace=fit.p00+fit.p11;
  out.accepted=fit.n;out.rejected=fit.rejected;out.nominal_mse=fit.nominal_mse;out.candidate_mse=fit.candidate_mse;
  if(!(fit.b>0.)||!finite(fit.a)||!finite(fit.b)){invalid(out,"nonphysical_parameter_fit");return;}
  const double estimated_tau=1./fit.b,estimated_gain=fit.a/fit.b;
  // Projection prevents an inadmissible RLS candidate from poisoning all later
  // iterations. A projected boundary fit is not granted confidence this cycle.
  fit.b=1./clamp(estimated_tau,p.min_tau,p.max_tau);
  fit.a=clamp(estimated_gain,p.min_gain,p.max_gain)*fit.b;
  out.gain.value=estimated_gain;out.tau.value=estimated_tau;out.gain.stamp=out.tau.stamp=t;
  if(estimated_tau<p.min_tau||estimated_tau>p.max_tau||estimated_gain<p.min_gain||estimated_gain>p.max_gain){invalid(out,"parameter_outside_validated_domain");return;}
  if(fit.n<static_cast<std::uint64_t>(p.min_samples)||lo<p.min_information||out.rank_ratio<p.min_rank_ratio){invalid(out,"insufficient_persistent_excitation");return;}
  if(std::max(fit.p00,fit.p11)>p.max_covariance){invalid(out,"parameter_uncertainty_high");return;}
  if(!(fit.nominal_mse>1e-12&&fit.candidate_mse<(1.-p.prediction_improvement)*fit.nominal_mse)){
   invalid(out,"no_prequential_prediction_improvement");return;
  }
  out.gain.confidence=out.tau.confidence=clamp(1.-fit.candidate_mse/std::max(1e-12,fit.nominal_mse),0.,1.);
  if(!raw.hardware_stamp_verified)out.gain.confidence=out.tau.confidence=std::min(.35,out.gain.confidence);
  if(out.gain.confidence<p.min_confidence){invalid(out,"parameter_confidence_below_threshold");return;}
  out.gain.valid=out.tau.valid=out.update_allowed=true;
  out.reason=out.gain.reason=out.tau.reason="excited_prequentially_supported_fit";
 }
 void yawFit(const SensorSnapshot& s,const Model& m,const ResponsePolicy& policy,ChannelEstimate& out) {
  const auto& p=policy.yaw;std::string reason;
  if(!sampleValid(s.wheel_speed,s.now,p,reason)||!sampleValid(s.steering,s.now,p,reason)||!sampleValid(s.yaw_rate,s.now,p,reason)){
   invalid(out,reason);return;
  }
  const double t=stamp(s.yaw_rate);
  if((finite(previous_yaw_stamp_)&&t<=previous_yaw_stamp_)||
     (s.yaw_rate.sequence&&previous_yaw_sequence_&&s.yaw_rate.sequence<=previous_yaw_sequence_)){
   invalid(out,"duplicate_or_out_of_order_yaw_sample");return;
  }
  previous_yaw_stamp_=t;previous_yaw_sequence_=s.yaw_rate.sequence;
  if(std::fabs(stamp(s.wheel_speed)-t)>p.sync_s||std::fabs(stamp(s.steering)-t)>p.sync_s){invalid(out,"yaw_input_pair_unsynchronized");return;}
  const double speed=s.wheel_speed.value,phi=speed*std::tan(s.steering.value)/m.wheelbase;
  const double raw=s.yaw_rate.value-m.yaw_gain*phi-m.yaw_residual;
  out.time_domain=s.yaw_rate.hardware_stamp_verified?"verified_sensor_time":"receipt_conditioned_response";
  out.residual.value=raw;out.residual.stamp=t;
  out.residual.valid=finite(raw)&&std::fabs(raw)<=p.max_innovation;
  out.residual.confidence=s.yaw_rate.hardware_stamp_verified?.8:.35;
  out.residual.reason=out.residual.valid?"wheel_proxy_conditioned_yaw_residual_not_slip":"yaw_innovation_outlier";
  if(!out.residual.valid){++yaw_rejected_;invalid(out,"yaw_innovation_outlier");return;}
  if(!evidence(p,s,reason)){invalid(out,reason);return;}
  if(!p.timing_verified||(!s.yaw_rate.hardware_stamp_verified&&!p.allow_receipt_identification)){
   invalid(out,"physical_time_unverified");return;
  }
  if(!s.wheel_speed.new_frame_known||!s.steering.new_frame_known||!s.yaw_rate.new_frame_known){invalid(out,"new_frame_unverified");return;}
  if(std::fabs(speed)<p.min_speed||std::fabs(phi)<p.min_excitation){invalid(out,"yaw_low_speed_or_weak_excitation");return;}
  // Estimator time is the interval between eligible new sensor samples, not
  // the controller period. Repeated snapshots must not change forgetting or
  // residual filtering when IMU and control frequencies differ.
  double dt=finite(yaw_last_fit_time_)?t-yaw_last_fit_time_:0.;
  if(dt>p.stale_s){yaw_samples_=0;yaw_information_=0.;yaw_filtered_=0.;dt=0.;}
  yaw_last_fit_time_=t;
  // If an additive channel is selected, gain fitting is disabled. The two
  // channels cannot freely explain the same innovation in one fitting window.
  if(policy.yaw_additive_enabled){
   yaw_filtered_+=blend(dt,p.residual_filter_s)*(raw-yaw_filtered_);
   out.residual.value=clamp(yaw_filtered_,-p.residual_bound,p.residual_bound);
   out.residual.confidence=s.yaw_rate.hardware_stamp_verified?.8:.35;
   if(out.residual.confidence<p.min_confidence){invalid(out,"parameter_confidence_below_threshold");return;}
   out.residual.valid=out.update_allowed=true;
   out.reason="bounded_additive_yaw_only_gain_frozen";return;
  }
  if(!yaw_samples_)yaw_fit_=m.yaw_gain;
  // Fit only the gain around the SAME known additive nominal model.
  const double innovation=s.yaw_rate.value-m.yaw_residual-yaw_fit_*phi;
  const double lambda=std::exp(-dt/p.forgetting_s);
  const double before=yaw_information_;yaw_information_=lambda*yaw_information_+phi*phi;
  const double step=phi/(.01+lambda*before+phi*phi);
  yaw_fit_+=step*innovation;
  const double unconstrained_fit=yaw_fit_;
  yaw_fit_=clamp(yaw_fit_,p.min_gain,p.max_gain);
  const double ema=blend(dt,p.filter_s);
  if(!yaw_samples_){yaw_candidate_mse_=innovation*innovation;yaw_nominal_mse_=raw*raw;}
  else{yaw_candidate_mse_+=ema*(innovation*innovation-yaw_candidate_mse_);yaw_nominal_mse_+=ema*(raw*raw-yaw_nominal_mse_);}
  ++yaw_samples_;++estimate_revision_;
  out.accepted=yaw_samples_;out.rejected=yaw_rejected_;out.information=yaw_information_;
  out.nominal_mse=yaw_nominal_mse_;out.candidate_mse=yaw_candidate_mse_;
  out.gain.value=yaw_fit_;out.gain.stamp=t;
  if(unconstrained_fit<p.min_gain||unconstrained_fit>p.max_gain){invalid(out,"yaw_gain_outside_validated_domain");return;}
  if(yaw_samples_<static_cast<std::uint64_t>(p.min_samples)||yaw_information_<p.min_information){invalid(out,"yaw_insufficient_excitation");return;}
  if(!(yaw_nominal_mse_>1e-12&&yaw_candidate_mse_<(1.-p.prediction_improvement)*yaw_nominal_mse_)){
   invalid(out,"no_prequential_prediction_improvement");return;
  }
  out.gain.confidence=clamp(1.-yaw_candidate_mse_/yaw_nominal_mse_,0.,1.);
  if(!s.yaw_rate.hardware_stamp_verified)out.gain.confidence=std::min(.35,out.gain.confidence);
  if(out.gain.confidence<p.min_confidence){invalid(out,"parameter_confidence_below_threshold");return;}
  out.gain.valid=out.update_allowed=true;out.reason=out.gain.reason="wheel_proxy_conditioned_gain_not_tire_friction";
 }
 void vibration(const SensorSnapshot& s,const ResponsePolicy& p,Estimate& out) {
  std::string reason;
  if(!s.imu_frame_verified||!sampleValid(s.accel_x,s.now,p.vibration,reason)||
   !sampleValid(s.accel_y,s.now,p.vibration,reason)||!sampleValid(s.accel_z,s.now,p.vibration,reason)){
   out.reason=s.imu_frame_verified?reason:"imu_axes_frame_unverified";return;
  }
  if(std::fabs(stamp(s.accel_x)-stamp(s.accel_z))>p.vibration.sync_s||
     std::fabs(stamp(s.accel_y)-stamp(s.accel_z))>p.vibration.sync_s){out.reason="imu_acceleration_axes_unsynchronized";return;}
  const double t=stamp(s.accel_z);
  if((finite(previous_vibration_stamp_)&&t<=previous_vibration_stamp_)||
     (s.accel_z.sequence&&previous_vibration_sequence_&&s.accel_z.sequence<=previous_vibration_sequence_)){
   out.reason="duplicate_or_out_of_order_acceleration_sample";return;
  }
  const double norm=std::sqrt(s.accel_x.value*s.accel_x.value+s.accel_y.value*s.accel_y.value+s.accel_z.value*s.accel_z.value);
  if(!finite(norm)){out.reason="invalid_acceleration_norm";return;}
  double dt=finite(previous_vibration_stamp_)?t-previous_vibration_stamp_:0.;
  if(!finite(previous_vibration_stamp_)||dt>p.vibration.stale_s){vibration_mean_=norm;vibration_var_=0.;dt=0.;}
  previous_vibration_stamp_=t;previous_vibration_sequence_=s.accel_z.sequence;
  const double a=blend(dt,p.vibration.filter_s);vibration_mean_+=a*(norm-vibration_mean_);
  const double d=norm-vibration_mean_;vibration_var_+=a*(d*d-vibration_var_);
  out.value=std::sqrt(std::max(0.,vibration_var_));out.stamp=stamp(s.accel_z);out.valid=true;
  out.confidence=s.accel_z.hardware_stamp_verified?.8:.35;
  out.reason="acceleration_norm_highpass_rms_not_terrain_or_mu";
 }
 static void slip(const SensorSnapshot& s,const ResponsePolicy& p,Estimate& out,Estimate& sideslip) {
  out.reason=sideslip.reason="slip_invalid_independent_motion_unavailable";
  std::string reason;
  if(!s.ground_speed.independent||!s.ground_frame_verified||!s.ground_lever_arm_verified)return;
  if(!sampleValid(s.wheel_speed,s.now,p.slip,reason)||!sampleValid(s.ground_speed,s.now,p.slip,reason)){
   out.reason="slip_invalid_"+reason;return;
  }
  if(!s.wheel_speed.hardware_stamp_verified||!s.ground_speed.hardware_stamp_verified){out.reason="slip_invalid_physical_time_unverified";return;}
  if(!s.wheel_speed.new_frame_known||!s.ground_speed.new_frame_known){out.reason="slip_invalid_new_frame_unverified";return;}
  if(std::fabs(stamp(s.wheel_speed)-stamp(s.ground_speed))>p.slip.sync_s){out.reason="slip_invalid_speed_pair_unsynchronized";return;}
  const double w=s.wheel_speed.value,g=s.ground_speed.value;
  if(std::max(std::fabs(w),std::fabs(g))<p.slip_min_speed){out.reason="slip_invalid_low_speed";return;}
  if(w*g<0.){out.reason="slip_invalid_direction_disagreement";return;}
  out.value=(w-g)/std::max(p.slip_epsilon,std::max(std::fabs(w),std::fabs(g)));
  out.stamp=std::min(stamp(s.wheel_speed),stamp(s.ground_speed));out.valid=true;out.confidence=.8;
  out.reason="signed_body_axis_speed_discrepancy_independent_motion_verified";
  if(s.heading_velocity.independent&&sampleValid(s.heading_velocity,s.now,p.slip,reason)&&
     s.heading_velocity.hardware_stamp_verified&&std::fabs(stamp(s.heading_velocity)-out.stamp)<=p.slip.sync_s){
   sideslip.value=s.heading_velocity.value;sideslip.stamp=stamp(s.heading_velocity);sideslip.valid=true;sideslip.confidence=.8;
   sideslip.reason="independent_motion_direction_body_heading_difference";
  }
 }
 static void change(ParameterChange& c,const char* name,double old,double nominal,const Estimate& estimate,
                    double effective,const std::string& reason) {
  c.name=name;c.old_value=old;c.nominal=nominal;c.estimate=estimate.value;c.effective=effective;
  c.confidence=estimate.confidence;c.validity=estimate.valid?"valid":"invalid";c.reason=reason;
  c.source=estimate.valid?"runtime_estimate":(std::abs(effective-nominal)>1e-12?"bounded_nominal_recovery":"nominal_config");
 }
 void updateFlag(std::size_t index,bool condition,double now,double dwell){
  if(condition==flags_[index]){flag_since_[index]=0.;return;}
  if(flag_since_[index]==0.)flag_since_[index]=now;
  if(now-flag_since_[index]>=dwell){flags_[index]=condition;flag_since_[index]=0.;}
 }
public:
 void reset(){*this=ResponseEstimator();}
 ParameterSnapshot observe(const SensorSnapshot& s,const std::vector<PublishedSample>& history,
                           const Model& nominal,const ResponsePolicy& policy) {
  ParameterSnapshot out;out.nominal=nominal;out.effective=nominal;out.reference_time=s.now;
  out.run_id=s.run_id;out.session_id=s.session_id;out.scene=s.scene;out.cycle=s.cycle;
  out.nominal_config_hash=s.nominal_config_hash;
  if(!history.empty())out.history_base_sequence=history.back().sequence;
  if(!modelValid(nominal)){out.reason="invalid_nominal_model_reject_candidate";return out;}
  if(!policyValid(policy.steering)||!policyValid(policy.longitudinal)||!policyValid(policy.yaw)||
     !policyValid(policy.vibration)||!policyValid(policy.slip)||!finite(policy.speed_domain_min)||
     !finite(policy.speed_domain_max)||policy.speed_domain_min<0.||policy.speed_domain_max<policy.speed_domain_min||
     !finite(policy.slip_min_speed)||policy.slip_min_speed<=0.||!finite(policy.slip_epsilon)||policy.slip_epsilon<=0.){
   out.reason="invalid_response_policy_reject_candidate";return out;
  }
  const bool model_context_changed=previous_scene_!=s.scene||
      previous_nominal_hash_!=s.nominal_config_hash;
  if(!finite(s.now)||(finite(last_time_)&&s.now<=last_time_)){
   out.reason="clock_regressed_or_duplicate_freeze";return out;
  }
  const double dt=finite(last_time_)?std::min(.25,s.now-last_time_):1./30.;last_time_=s.now;
  // The scene manager owns blending of nominal values. Learned offsets
  // from a different profile have no evidence in the new profile.
  if(!effective_initialized_||model_context_changed){effective_=nominal;effective_initialized_=true;}
  const Model old=effective_;
  if(model_context_changed){
   steer_fit_=RLS();speed_fit_=RLS();steer_previous_=Previous();speed_previous_=Previous();
   yaw_samples_=0;yaw_information_=0.;yaw_filtered_=0.;previous_yaw_stamp_=missing();previous_yaw_sequence_=0;yaw_last_fit_time_=missing();
   previous_vibration_stamp_=missing();previous_vibration_sequence_=0;
   vibration_mean_=vibration_var_=0.;
   flags_.fill(false);flags_[6]=true;flag_since_.fill(0.);
   previous_scene_=s.scene;previous_nominal_hash_=s.nominal_config_hash;
  }
  std::string speed_reason;
  const bool known_speed=sampleValid(s.wheel_speed,s.now,policy.longitudinal,speed_reason);
  const bool speed_domain=known_speed&&std::fabs(s.wheel_speed.value)>=policy.speed_domain_min&&
   std::fabs(s.wheel_speed.value)<=policy.speed_domain_max;
  if(known_speed&&!speed_domain){
   invalid(out.steering,"outside_validated_speed_domain");steer_previous_.valid=false;
  }else integralFit(s,s.steering,policy.steering,history,true,nominal,steer_fit_,steer_previous_,out.steering);
  if(known_speed&&!speed_domain){
   invalid(out.longitudinal,"outside_validated_speed_domain");invalid(out.yaw,"outside_validated_speed_domain");
   speed_previous_.valid=false;
  }else{
   integralFit(s,s.wheel_speed,policy.longitudinal,history,false,nominal,speed_fit_,speed_previous_,out.longitudinal);
   yawFit(s,nominal,policy,out.yaw);
  }
  vibration(s,policy,out.roughness);slip(s,policy,out.slip,out.sideslip);
  // Steering dynamics remain observable while parked or with missing motion
  // sensing. Missing speed disables only the channels that use speed.
  // Rebase immutable model properties to the current scene; learned properties
  // are applied separately below and never expand geometry or physical limits.
  effective_=nominal;
  const auto apply=[&](double oldv,double nominalv,const Estimate& est,const ChannelEstimate& c,
                       const ChannelPolicy& p,bool tau){
   if(!p.enabled)return nominalv;
   const double target=c.update_allowed&&est.valid?est.value:nominalv;
   const double rate=c.update_allowed?(tau?p.tau_rate:p.gain_rate):p.return_rate;
   return approach(oldv,target,std::max(0.,rate),dt);
  };
  effective_.steering_gain=apply(old.steering_gain,nominal.steering_gain,out.steering.gain,out.steering,policy.steering,false);
  effective_.steering_tau=apply(old.steering_tau,nominal.steering_tau,out.steering.tau,out.steering,policy.steering,true);
  effective_.speed_gain=apply(old.speed_gain,nominal.speed_gain,out.longitudinal.gain,out.longitudinal,policy.longitudinal,false);
  effective_.speed_tau=apply(old.speed_tau,nominal.speed_tau,out.longitudinal.tau,out.longitudinal,policy.longitudinal,true);
  // Delays are not estimated here. Use the nominal scene manager's value
  // directly so an additional filter cannot introduce model inconsistency.
  effective_.steering_delay=nominal.steering_delay;
  effective_.speed_delay=nominal.speed_delay;
  effective_.yaw_gain=apply(old.yaw_gain,nominal.yaw_gain,out.yaw.gain,out.yaw,policy.yaw,false);
  const bool additive=policy.yaw_additive_enabled&&out.yaw.update_allowed&&out.yaw.residual.valid;
  effective_.yaw_residual=policy.yaw.enabled?approach(old.yaw_residual,
   additive?nominal.yaw_residual+out.yaw.residual.value:nominal.yaw_residual,
   additive?policy.yaw.residual_rate:policy.yaw.return_rate,dt):nominal.yaw_residual;
  effective_.longitudinal_residual=nominal.longitudinal_residual;
  out.effective=effective_;
  change(out.changes[0],"steering_gain",old.steering_gain,nominal.steering_gain,out.steering.gain,effective_.steering_gain,out.steering.reason);
  change(out.changes[1],"steering_tau",old.steering_tau,nominal.steering_tau,out.steering.tau,effective_.steering_tau,out.steering.reason);
  change(out.changes[2],"speed_gain",old.speed_gain,nominal.speed_gain,out.longitudinal.gain,effective_.speed_gain,out.longitudinal.reason);
  change(out.changes[3],"speed_tau",old.speed_tau,nominal.speed_tau,out.longitudinal.tau,effective_.speed_tau,out.longitudinal.reason);
  change(out.changes[4],"yaw_gain",old.yaw_gain,nominal.yaw_gain,out.yaw.gain,effective_.yaw_gain,out.yaw.reason);
  change(out.changes[5],"yaw_residual",old.yaw_residual,nominal.yaw_residual,out.yaw.residual,effective_.yaw_residual,out.yaw.reason);
  change(out.changes[6],"longitudinal_residual",old.longitudinal_residual,nominal.longitudinal_residual,out.longitudinal.residual,effective_.longitudinal_residual,"gain_tau_fit_residual_not_double_applied");
  auto delayReason=[](const ChannelPolicy&p){return !p.delay_verified?"unverified_nominal_delay_seed_no_online_estimation":
    (p.evidence_kind=="synthetic_validation"?"fixed_synthetic_delay_not_real_calibration":
      (p.allow_receipt_identification?"fixed_receipt_domain_delay_not_physical_dead_time":"fixed_verified_delay_no_online_estimation"));};
  Estimate fixed_delay;fixed_delay.reason=delayReason(policy.steering);
  change(out.changes[7],"steering_delay",old.steering_delay,nominal.steering_delay,fixed_delay,effective_.steering_delay,fixed_delay.reason);
  fixed_delay.reason=delayReason(policy.longitudinal);
  change(out.changes[8],"speed_delay",old.speed_delay,nominal.speed_delay,fixed_delay,effective_.speed_delay,fixed_delay.reason);
  // Conditions require evidence-supported fits; no single residual spike or
  // terrain name triggers a model update. Thresholds reuse validated fit bounds.
  updateFlag(1,out.yaw.gain.valid&&out.yaw.gain.value<nominal.yaw_gain*(1.-(flags_[1]?policy.yaw.condition_exit_fraction:policy.yaw.condition_enter_fraction)),s.now,policy.yaw.condition_dwell_s);
  updateFlag(2,out.steering.tau.valid&&out.steering.tau.value>nominal.steering_tau*(1.+(flags_[2]?policy.steering.condition_exit_fraction:policy.steering.condition_enter_fraction)),s.now,policy.steering.condition_dwell_s);
  updateFlag(3,out.longitudinal.gain.valid&&out.longitudinal.gain.value<nominal.speed_gain*(1.-(flags_[3]?policy.longitudinal.condition_exit_fraction:policy.longitudinal.condition_enter_fraction)),s.now,policy.longitudinal.condition_dwell_s);
  std::string condition_evidence;
  updateFlag(4,evidence(policy.vibration,s,condition_evidence)&&out.roughness.valid&&out.roughness.value>policy.vibration.residual_bound,s.now,policy.vibration.condition_dwell_s);
  updateFlag(5,evidence(policy.slip,s,condition_evidence)&&out.slip.valid&&std::fabs(out.slip.value)>policy.slip.residual_bound,s.now,policy.slip.condition_dwell_s);
  flags_[6]=!(out.steering.update_allowed||out.longitudinal.update_allowed||out.yaw.update_allowed);
  flags_[0]=!flags_[1]&&!flags_[2]&&!flags_[3]&&!flags_[4]&&!flags_[5]&&!flags_[6];
  out.dynamic_flags=flags_;out.estimate_revision=estimate_revision_;out.effective_revision=++effective_revision_;
  out.valid=modelValid(effective_);out.reason=out.valid?"immutable_effective_model_ready":"invalid_effective_model_reject_candidate";
  out.effective_hash=responseHash(out);return out;
 }
};
}} // research_control::p6
