#pragma once
#include "reduced_nmpc.h"
#include "unified_control.h"
#include "flatness_reduced_nmpc.h"
#include "vehicle_state_observer.h"
#include "terrain_condition_estimator.h"

namespace research_control {
struct TerrainProfile {
 double speed_cap=.30,lateral_scale=1.,heading_scale=1.,speed_scale=1.,smoothness_scale=1.;
};
struct RuntimeConfig {
 // Explicit research selections; no production YAML or default change.
 std::string research_backend="original12";
 bool research_strict_adaptation=false;
 // Independent P37 study branch. No ROS parameter enables it, and the default
 // preserves P36 arithmetic and the previously rejected strict candidate.
 bool research_validity_adaptation=false;
 SolverConfig solver; ObserverConfig observer;
 std::array<TerrainProfile,4> profiles;
 bool terrain_estimation_enabled=true,use_estimated_terrain=false;
 bool model_adaptation_enabled=true,weight_adaptation_enabled=true;
 double filter_tau=3.,minimum_confidence=.8,switch_dwell=2.,switch_hysteresis=.15;
 // Residual features have deliberately discounted heuristic confidence (.63-.72).
 // Keep a distinct threshold; do not inflate them to pass the B-parameter gate.
 double feature_minimum_confidence=.60;
 double yaw_gain_max_change=.15,tau_max_change=.10;
 RuntimeConfig(){profiles[3].speed_cap=.10;}
};
// All evidence scores compare a parameter frozen before the incoming sample.
// They are prediction checks, not a proof of closed-loop stability.
struct AdaptationEvidence {
 int samples=0; double nominal_mse=0.,candidate_mse=0.;
 bool active=false; std::string reason="insufficient_held_out_evidence";
};
struct RuntimeResult {
 SolveResult solve; Command command;
 std::string requested_mode="legacy_slqr",owner="legacy_slqr",reason="legacy_requested",terrain="unknown";
 Observation observation; TerrainEstimate terrain_estimate; SolverConfig effective;
 bool nmpc_selected=false;
 std::array<AdaptationEvidence,5> adaptation; // gain, tau, yaw, acceleration, lateral
 bool model_adaptation_active=false,weight_adaptation_active=false;
 double adaptation_risk=0.; std::string adaptation_reason="not_requested";
};

// Shared by the existing ROS node (shadow only) and the offline closed loop.
// All authority selection occurs here; SLQR and NMPC commands are never added.
class AdaptiveRuntime {
public:
 bool configure(const RuntimeConfig& c) {
   config_=c; valid_=validate(c); observer_.configure(c.observer);flat_solver_.reset();
   TerrainEstimatorConfig tc;tc.enabled=c.terrain_estimation_enabled;tc.dwell_seconds=c.switch_dwell;
   terrain_estimator_.configure(tc);solver_.reset();flat_solver_.reset();last_now_=0.;scene_=3;pending_=3;
   pending_since_=0.;model_=c.solver.model;weights_=c.solver.weights;
   speed_cap_=c.profiles[3].speed_cap;recovery_since_=0.;recovery_required_=false;mode_="legacy_slqr";
   resetEvidence();risk_=roughness_risk_=0.;last_solver_ok_=false;
   return valid_ && observer_.configValid();
 }
 VehicleStateObserver& observer(){return observer_;}
 bool requestMode(const std::string& mode) {
   if(mode!="legacy_slqr"&&mode!="nmpc_fixed"&&mode!="nmpc_model_adaptive"&&
      mode!="nmpc_weight_adaptive"&&mode!="nmpc_full_adaptive"&&mode!="auto_fallback")return false;
   if(mode!=mode_){solver_.reset();flat_solver_.reset();recovery_since_=0.;recovery_required_=false;last_solver_ok_=false;
     // Retain applied model/weights: the next cycle ramps toward the new mode.
     // Resetting to nominal here would introduce an instantaneous transition.
   }
   mode_=mode;return true;
 }
 bool requestTerrain(const std::string& terrain,double now) {
   const int i=index(terrain);if(i<0||!std::isfinite(now))return false;
   if(i!=pending_){pending_=i;pending_since_=now;}
   return true;
 }
 RuntimeResult cycle(const SolveRequest& original,double now,bool reference_valid,bool hard_stop=false, bool preserve_initialized_state=false, const Limits* external_limits=nullptr) {
   RuntimeResult out;out.requested_mode=mode_;out.command=original.baseline;
   out.observation=observer_.observe(now,config_.solver.model);
   out.terrain_estimate=terrain_estimator_.update(out.observation,now);
   const bool clock_ok=std::isfinite(now)&&now>0.&&(last_now_==0.||now>=last_now_);
   const double dt=clock_ok&&last_now_>0.?clamp(now-last_now_,0.,.2):0.;last_now_=now;
   if(config_.use_estimated_terrain) {
     const auto& e=out.terrain_estimate;
     // Operational switching requires calibrated confidence. The initial
     // roughness heuristic deliberately cannot unlock an aggressive profile.
     const int candidate=index(e.label);
     const double current_p=e.probabilities[scene_];
     requestTerrain(e.valid&&candidate>=0&&e.confidence>=config_.minimum_confidence&&
       (candidate==scene_||e.probabilities[candidate]>=current_p+config_.switch_hysteresis)?e.label:"unknown",now);
   }
   if(pending_==3 || now-pending_since_>=config_.switch_dwell)scene_=pending_;
   out.terrain=name(scene_);
   const double blend=dt>0.?1.-std::exp(-dt/config_.filter_tau):0.;
   const auto& profile=config_.profiles[scene_];
   // Restrictions engage immediately; relaxation is filtered and rate limited.
   speed_cap_=profile.speed_cap<speed_cap_?profile.speed_cap:
       speed_cap_+std::min(.04*dt,blend*(profile.speed_cap-speed_cap_));
   Model target=config_.solver.model;Weights wt=config_.solver.weights;
   wt.lateral*=profile.lateral_scale;wt.heading*=profile.heading_scale;
   wt.speed*=profile.speed_scale;wt.smoothness*=profile.smoothness_scale;
   const bool adapt_model=config_.model_adaptation_enabled&&
      (mode_=="nmpc_model_adaptive"||mode_=="nmpc_full_adaptive"||mode_=="auto_fallback");
   const bool adapt_weight=config_.weight_adaptation_enabled&&
      (mode_=="nmpc_weight_adaptive"||mode_=="nmpc_full_adaptive"||mode_=="auto_fallback");
   const auto& o=out.observation;
   updateEvidence(o,now,dt);
   auto usable=[&](const Estimate& e){return e.valid&&std::isfinite(e.value)&&
      e.confidence>=config_.minimum_confidence&&now-e.stamp<=config_.observer.stale_after&&e.stamp<=now+.02;};
   auto feature=[&](const Estimate& e){return e.valid&&std::isfinite(e.value)&&
      e.confidence>=config_.feature_minimum_confidence&&now-e.stamp<=config_.observer.stale_after&&e.stamp<=now+.02;};
   // The ROS bridge deliberately leaves measured components unknown. Resolve
   // them before validating: an absent raw field must not reject a verified
   // observation, nor may a stale/invalid observation replace a valid field.
   State measured_state=original.state;
   if(!preserve_initialized_state&&feature(o.speed))measured_state.speed=o.speed.value;
   if(!preserve_initialized_state&&feature(o.steering))measured_state.steering=o.steering.value;
   const bool state_ok=finiteState(measured_state);
   const bool eligible=clock_ok&&state_ok&&reference_valid&&original.reference.valid&&
      !hard_stop&&last_solver_ok_&&config_.observer.ground_independent;
   const auto gate=[&](int i){return eligible&&evidence_[i].active;};
   if(adapt_model) {
     if(gate(0)&&usable(o.yaw_gain)&&(!config_.research_strict_adaptation||
        std::fabs(model_.yaw_residual-config_.solver.model.yaw_residual)<1e-6))
       target.yaw_gain=clamp(o.yaw_gain.value,target.yaw_gain-config_.yaw_gain_max_change,target.yaw_gain+config_.yaw_gain_max_change);
     if(gate(1)&&usable(o.steering_tau))
       target.steering_tau=clamp(o.steering_tau.value,target.steering_tau-config_.tau_max_change,target.steering_tau+config_.tau_max_change);
     // Residual and gain are alternative descriptions until a joint bias/gain
     // estimator is observable. Do not stack a nominal-model yaw residual on a
     // nonnominal gain, including the slow transition back toward nominal.
     if(!config_.research_validity_adaptation&&gate(2)&&feature(o.yaw_residual)&&!usable(o.yaw_gain)&&
        std::fabs(model_.yaw_gain-config_.solver.model.yaw_gain)<.005)
       target.yaw_residual=clamp(o.yaw_residual.value,-.03,.03);
     if(gate(3)&&feature(o.longitudinal_residual))target.longitudinal_residual=clamp(o.longitudinal_residual.value,-.08,.08);
     if(gate(4)&&feature(o.lateral_residual))target.lateral_residual=clamp(o.lateral_residual.value,-.03,.03);
   }
   double risk_target=0.,roughness_target=0.;
   if(adapt_weight&&eligible) {
     // Vibration is not a friction estimate and cannot by itself tighten the
     // lateral-acceleration envelope, lower speed weight and raise feedback.
     if(feature(o.roughness)&&feature(o.speed)&&feature(o.steering))
       roughness_target=clamp((o.roughness.value-.20)/2.,0.,1.);
     if((gate(0)||gate(2))&&feature(o.yaw_residual)) {
       double unexplained_yaw=o.yaw_residual.value;
       if(config_.research_validity_adaptation) {
         // A calibrated gain can explain nominal-model yaw error. Do not turn
         // that same explained component into a putative low-friction signal.
         // Still require the independent, synchronized gyro/speed/steering and
         // prior-sample gain evidence; a terrain label never supplies a gain.
         if(gate(0)&&usable(o.yaw_gain)&&feature(o.yaw_rate)&&feature(o.speed)&&feature(o.steering)) {
           const double phi=o.speed.value*std::tan(o.steering.value)/config_.solver.model.wheelbase;
           const double gain=clamp(o.yaw_gain.value,config_.solver.model.yaw_gain-config_.yaw_gain_max_change,
                                                    config_.solver.model.yaw_gain+config_.yaw_gain_max_change);
           unexplained_yaw=o.yaw_rate.value-gain*phi;
         } else unexplained_yaw=0.; // No jointly identified additive yaw bias.
       }
       risk_target=std::max(risk_target,clamp((std::fabs(unexplained_yaw)-.008)/.08,0.,1.));
     }
     if(feature(o.slip)&&feature(o.speed)&&std::fabs(o.speed.value)>=config_.observer.min_slip_speed&&
        std::fabs(o.slip.value)>.05)
       risk_target=std::max(risk_target,clamp((std::fabs(o.slip.value)-.05)/.20,0.,1.));
   }
   auto change=[&](double& x,double y,double rate){x+=clamp(blend*(y-x),-rate*dt,rate*dt);};
   change(risk_,risk_target,.15);change(roughness_risk_,roughness_target,.15);
   wt.speed*=1.-.20*risk_;
   wt.smoothness*=1.+.25*std::max(risk_,roughness_risk_);
   // Tracking feedback weights remain the validated scene values; uncertainty
   // does not justify simultaneous stronger steering and stronger smoothing.
   change(model_.yaw_gain,target.yaw_gain,.05);change(model_.steering_tau,target.steering_tau,.03);
   change(model_.yaw_residual,target.yaw_residual,.01);change(model_.longitudinal_residual,target.longitudinal_residual,.03);
   change(model_.lateral_residual,target.lateral_residual,.01);
   change(weights_.lateral,wt.lateral,10.);change(weights_.heading,wt.heading,3.);
   change(weights_.speed,wt.speed,3.);change(weights_.smoothness,wt.smoothness,10.);
   out.effective=config_.solver;out.effective.model=model_;out.effective.weights=weights_;
   out.effective.limits.speed=std::min(config_.solver.limits.speed,speed_cap_);
   // Conservative engineering acceleration envelope, never an inferred mu.
   out.effective.limits.lateral_acceleration=config_.solver.limits.lateral_acceleration*(1.-.35*risk_);
   if(external_limits){
     if(!unified_control::validLimits(*external_limits)){fallback(out,original,"invalid_external_safety_limits");last_solver_ok_=false;return out;}
     out.effective.limits=unified_control::intersectLimits(out.effective.limits,*external_limits);
   }
   out.adaptation=evidence_;out.adaptation_risk=risk_;
   out.model_adaptation_active=adapt_model&&(target.yaw_gain!=config_.solver.model.yaw_gain||
      target.steering_tau!=config_.solver.model.steering_tau||target.yaw_residual!=config_.solver.model.yaw_residual||
      target.longitudinal_residual!=config_.solver.model.longitudinal_residual||target.lateral_residual!=config_.solver.model.lateral_residual);
   out.weight_adaptation_active=adapt_weight&&(risk_target>0.||roughness_target>0.);
   out.adaptation_reason=!(adapt_model||adapt_weight)?"not_requested":
      !eligible?"unverified_state_independence_or_prior_solver":
      (out.model_adaptation_active||out.weight_adaptation_active)?"bounded_prediction_evidence":"holding_scene_no_credible_effect";
   if(!valid_||!clock_ok){out.reason=!valid_?"invalid_runtime_config":"clock_invalid";out.command=Command();solver_.reset();flat_solver_.reset();last_solver_ok_=false;resetEvidence();return out;}
   if(hard_stop||!reference_valid||!original.reference.valid) {
     out.reason=hard_stop?"hard_stop":"reference_invalid";out.owner="safety_stop";out.command=Command();
     solver_.reset();flat_solver_.reset();recovery_since_=0.;recovery_required_=true;last_solver_ok_=false;resetEvidence();return out;
   }
   if(!state_ok||!finiteCommand(original.baseline)||!finiteCommand(original.previous)||
      !std::isfinite(original.previous_command_acceleration)||!std::isfinite(original.control_dt)||original.control_dt<=0.) {
     fallback(out,original,"invalid_state_or_command");last_solver_ok_=false;solver_.reset();flat_solver_.reset();return out;
   }
   if(mode_=="legacy_slqr")return out;
   SolveRequest request=original;request.now=now;
   request.speed_cap=std::min(original.speed_cap,out.effective.limits.speed);
   // Estimates replace only measured state components with verified observability.
   request.state=measured_state;
   if(config_.research_backend=="flat6")out.solve=flat_solver_.solve(request,out.effective);
   else if(config_.research_backend=="refine12"){
     const auto started=std::chrono::steady_clock::now();
     SolverConfig coarse=out.effective;coarse.knots=4;coarse.iterations=1;
     coarse.budget_ms=out.effective.budget_ms*.40;
     auto first=solver_.solve(request,coarse);
     const auto elapsed=[&](){return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();};
     SolverConfig fine=out.effective;fine.knots=6;fine.iterations=std::max(1,fine.iterations-1);
     fine.budget_ms=out.effective.budget_ms-elapsed()-.10;
     SolveResult second;
     if(fine.budget_ms>.05){
       if(first.ok){request.warm_hint_count=coarse.steps;request.warm_hint=first.controls;}
       second=solver_.solve(request,fine);
     }
     const bool use_second=second.ok&&(!first.ok||second.objective<first.objective);
     out.solve=use_second?second:first;
     out.solve.evaluations=first.evaluations+second.evaluations;
     out.solve.iterations=first.iterations+second.iterations;
     out.solve.decision_dimension=12;out.solve.solve_ms=elapsed();
     if(out.solve.solve_ms>=out.effective.budget_ms){out.solve.ok=out.solve.feasible=false;out.solve.timed_out=true;out.solve.reason="refinement_total_timeout";}
     else if(out.solve.ok)out.solve.reason=use_second?"refined_8_to_12":"retained_feasible_8_after_12_attempt";
   }else {
     SolverConfig selected=out.effective;
     if(config_.research_backend!="original12")selected.knots=4;
     if(config_.research_backend=="front8")selected.research_grid=1;
     if(config_.research_backend=="curve8")selected.research_grid=2;
     if(config_.research_backend=="hold8")selected.research_grid=3;
     if(config_.research_backend=="flatseed8")selected.research_consistent_flat_seed=true;
     if(config_.research_backend=="search8")selected.iterations=5;
     out.solve=solver_.solve(request,selected);
   }
   last_solver_ok_=out.solve.ok&&out.solve.feasible&&!out.solve.timed_out;
   if(last_solver_ok_) {
     if(mode_=="auto_fallback"&&recovery_required_) {
       if(recovery_since_==0.)recovery_since_=now;
       if(now-recovery_since_<.5){fallback(out,original,"recovery_dwell");return out;}
     }
     recovery_required_=false;
     out.command=out.solve.command;out.owner="nmpc";out.reason="feasible_solution";out.nmpc_selected=true;
   } else {recovery_since_=0.;recovery_required_=true;fallback(out,original,out.solve.reason);}
   return out;
 }
 static int index(const std::string& s){return s=="smooth"?0:s=="grass_bumpy"?1:s=="asphalt"?2:s=="unknown"?3:-1;}
 static std::string name(int i){return i==0?"smooth":i==1?"grass_bumpy":i==2?"asphalt":"unknown";}
 static bool validate(const RuntimeConfig& c) {
   if(c.research_backend!="original12"&&c.research_backend!="blocked8"&&c.research_backend!="flat6"&&
      c.research_backend!="front8"&&c.research_backend!="curve8"&&c.research_backend!="hold8"&&
      c.research_backend!="flatseed8"&&c.research_backend!="refine12"&&c.research_backend!="search8")return false;
   const auto& s=c.solver;
   if(!(c.filter_tau>=1.&&c.filter_tau<=10.&&c.minimum_confidence>=.7&&c.minimum_confidence<=1.&&
      c.feature_minimum_confidence>=.6&&c.feature_minimum_confidence<=1.&&
      c.switch_dwell>=1.&&c.switch_dwell<=10.&&c.switch_hysteresis>=.1&&c.switch_hysteresis<=.4&&
      c.yaw_gain_max_change>=0.&&c.yaw_gain_max_change<=.25&&c.tau_max_change>=0.&&c.tau_max_change<=.15&&
      s.model.wheelbase==.60&&s.model.rear_to_center==.30&&s.model.steering_tau>=.05&&s.model.steering_tau<=1.&&
      s.model.steering_delay>=0.&&s.model.steering_delay<=.70&&s.model.speed_tau>=.05&&s.model.speed_tau<=1.&&
      s.model.speed_delay>=0.&&s.model.speed_delay<=.7&&s.model.yaw_gain>=.5&&s.model.yaw_gain<=1.5&&
      s.limits.speed>0.&&s.limits.speed<=.5&&s.limits.steering>0.&&s.limits.steering<=.32&&
      s.limits.steering_rate>0.&&s.limits.steering_rate<=1.&&s.limits.acceleration>0.&&s.limits.acceleration<=.28&&
      s.limits.deceleration>0.&&s.limits.deceleration<=.36&&s.limits.jerk>0.&&s.limits.jerk<=.8&&
      s.limits.lateral_acceleration>0.&&s.limits.lateral_acceleration<=.25&&
      s.steps>=10&&s.steps<=40&&s.knots>=4&&s.knots<=10&&s.knots<=s.steps&&
      s.iterations>=1&&s.iterations<=5&&s.dt>=.05&&s.dt<=.20&&s.budget_ms>0.&&std::isfinite(s.budget_ms)&&
      s.weights.lateral>=1.&&s.weights.lateral<=100.&&s.weights.heading>=1.&&s.weights.heading<=100.&&
      s.weights.speed>=1.&&s.weights.speed<=40.&&s.weights.smoothness>=1.&&s.weights.smoothness<=100.))return false;
   for(const auto& p:c.profiles)if(!(p.speed_cap>=0.&&p.speed_cap<=.30&&p.lateral_scale>=.75&&p.lateral_scale<=1.5&&
      p.heading_scale>=.75&&p.heading_scale<=1.5&&p.speed_scale>=.5&&p.speed_scale<=1.25&&
      p.smoothness_scale>=1.&&p.smoothness_scale<=2.))return false;
   return c.profiles[3].speed_cap<=.10;
 }
private:
 static bool finiteCommand(const Command& c) {return std::isfinite(c.speed)&&std::isfinite(c.steering);}
 static bool finiteState(const State& s) {
   return std::isfinite(s.x)&&std::isfinite(s.y)&&std::isfinite(s.yaw)&&std::isfinite(s.speed)&&
     std::isfinite(s.steering)&&std::isfinite(s.acceleration);
 }
 void resetEvidence() {
   evidence_={};evidence_started_={};previous_observation_=Observation();last_evidence_at_=0.;
 }
 bool sampleUsable(const Estimate& e,double now,double confidence)const {
   return e.valid&&std::isfinite(e.value)&&std::isfinite(e.stamp)&&e.confidence>=confidence&&
     e.stamp<=now+config_.observer.future_tolerance&&now-e.stamp<=config_.observer.stale_after;
 }
 bool delayedCommand(double stamp,Command& command)const {
   const auto& h=observer_.commandHistory();
   for(auto i=h.rbegin();i!=h.rend();++i)if(i->stamp<=stamp) {
     if(stamp-i->stamp>config_.observer.stale_after)return false;
     command=i->command;return finiteCommand(command);
   }
   return false;
 }
 void scoreEvidence(int i,double nominal_error,double candidate_error,double noise,
                    bool observable,double now,const char* unavailable) {
   auto& e=evidence_[i];
   if(!observable||!std::isfinite(nominal_error)||!std::isfinite(candidate_error)) {
     e=AdaptationEvidence();e.reason=unavailable;evidence_started_[i]=0.;return;
   }
   if(e.samples==0)evidence_started_[i]=now;
   const double blend=e.samples==0?1.:.08;
   e.nominal_mse+=blend*(nominal_error*nominal_error-e.nominal_mse);
   e.candidate_mse+=blend*(candidate_error*candidate_error-e.candidate_mse);++e.samples;
   const bool informative=e.nominal_mse>=noise*noise;
   // Optional uncertainty margin: require the predicted improvement to exceed
   // the configured engineering noise floor, not just a ratio of tiny errors.
   const bool improved=e.candidate_mse<(e.active?.90:.80)*e.nominal_mse &&
      (!(config_.research_strict_adaptation||config_.research_validity_adaptation) ||
         e.nominal_mse-e.candidate_mse>=noise*noise);
   e.active=e.samples>=30&&now-evidence_started_[i]>=1.&&informative&&improved;
   e.reason=e.samples<30||now-evidence_started_[i]<1.?"held_out_window_warming":
     !informative?"effect_below_uncertainty_floor":!improved?"no_held_out_prediction_improvement":"held_out_prediction_improved";
 }
 void updateEvidence(const Observation& o,double now,double dt) {
   if(!std::isfinite(now)||dt<=0.||dt>config_.observer.stale_after) {
     resetEvidence();previous_observation_=o;last_evidence_at_=now;return;
   }
   if(now<=last_evidence_at_)return;
   const auto& p=previous_observation_;const auto& m=config_.solver.model;
   const auto feature=[&](const Estimate& e){return sampleUsable(e,now,config_.feature_minimum_confidence);};
   const auto parameter=[&](const Estimate& e){return sampleUsable(e,now,config_.minimum_confidence);};
   const bool independent=config_.observer.ground_independent&&feature(o.speed)&&feature(p.speed)&&
      o.speed.reason=="rear_axle_ground_velocity"&&p.speed.reason=="rear_axle_ground_velocity"&&
      std::fabs(o.speed.value)>=config_.observer.min_identification_speed;
   const bool yaw_observable=independent&&feature(o.steering)&&feature(o.yaw_rate)&&
      std::fabs(o.yaw_rate.stamp-o.speed.stamp)<=config_.observer.sync_tolerance&&
      std::fabs(o.yaw_rate.stamp-o.steering.stamp)<=config_.observer.sync_tolerance&&
      o.yaw_rate.stamp>p.yaw_rate.stamp;
   const double phi=o.speed.value*std::tan(o.steering.value)/m.wheelbase;
   const double yaw_error=o.yaw_rate.value-m.yaw_gain*phi;
   const bool turning=yaw_observable&&std::fabs(phi)>=config_.observer.yaw_excitation;
   const double gain_candidate=config_.research_validity_adaptation?
      clamp(p.yaw_gain.value,m.yaw_gain-config_.yaw_gain_max_change,m.yaw_gain+config_.yaw_gain_max_change):p.yaw_gain.value;
   scoreEvidence(0,yaw_error,o.yaw_rate.value-gain_candidate*phi,.005,
      turning&&parameter(o.yaw_gain)&&parameter(p.yaw_gain)&&std::fabs(p.yaw_gain.value-m.yaw_gain)>=.02,
      now,"yaw_gain_not_independently_observable");
   scoreEvidence(2,yaw_error,yaw_error-p.yaw_residual.value,.008,
      !config_.research_validity_adaptation&&turning&&feature(o.yaw_residual)&&feature(p.yaw_residual)&&!parameter(o.yaw_gain),
      now,config_.research_validity_adaptation?"yaw_residual_not_jointly_identified":"yaw_residual_unobservable_or_gain_has_priority");
   Command delayed_steering,delayed_speed;
   const double ds=o.steering.stamp-p.steering.stamp,dv=o.speed.stamp-p.speed.stamp;
   const bool steering_pair=independent&&feature(o.steering)&&feature(p.steering)&&
      ds>=.005&&ds<=config_.observer.stale_after&&
      delayedCommand(.5*(o.steering.stamp+p.steering.stamp)-m.steering_delay,delayed_steering);
   const double nominal_steering=delayed_steering.steering+
      (p.steering.value-delayed_steering.steering)*std::exp(-ds/m.steering_tau);
   const double tau_candidate=config_.research_validity_adaptation?
      clamp(p.steering_tau.value,m.steering_tau-config_.tau_max_change,m.steering_tau+config_.tau_max_change):p.steering_tau.value;
   const double candidate_steering=delayed_steering.steering+
      (p.steering.value-delayed_steering.steering)*std::exp(-ds/tau_candidate);
   scoreEvidence(1,o.steering.value-nominal_steering,o.steering.value-candidate_steering,.0005,
      steering_pair&&parameter(o.steering_tau)&&parameter(p.steering_tau)&&
      std::fabs(delayed_steering.steering-p.steering.value)>=config_.observer.steering_excitation&&
      std::fabs(p.steering_tau.value-m.steering_tau)>=.03,
      now,"tau_unobservable_or_fixed_delay_prediction_not_supported");
   const bool speed_pair=independent&&dv>=.005&&dv<=config_.observer.stale_after&&
      delayedCommand(.5*(o.speed.stamp+p.speed.stamp)-m.speed_delay,delayed_speed);
   const double measured_acceleration=(o.speed.value-p.speed.value)/dv;
   const double nominal_acceleration=(delayed_speed.speed-.5*(p.speed.value+o.speed.value))/m.speed_tau;
   scoreEvidence(3,measured_acceleration-nominal_acceleration,
      measured_acceleration-nominal_acceleration-p.longitudinal_residual.value,.03,
      speed_pair&&feature(o.longitudinal_residual)&&feature(p.longitudinal_residual),
      now,"acceleration_residual_unobservable");
   // This tests persistence of an independently measured rear-axle lateral
   // velocity. Tiny course differences are below an explicit provisional floor.
   scoreEvidence(4,o.lateral_residual.value,o.lateral_residual.value-p.lateral_residual.value,.008,
      independent&&feature(o.lateral_residual)&&feature(p.lateral_residual)&&o.speed.stamp>p.speed.stamp,
      now,"lateral_velocity_unobservable");
   previous_observation_=o;last_evidence_at_=now;
 }
 std::array<AdaptationEvidence,5> evidence_{};std::array<double,5> evidence_started_{};
 Observation previous_observation_;double last_evidence_at_=0.;
 void fallback(RuntimeResult& out,const SolveRequest& request,const std::string& reason) {
   out.owner="legacy_slqr";out.reason="fallback:"+reason;
   out.command=request.baseline;
   const auto& l=config_.solver.limits;
   if(!finiteCommand(request.previous)||!std::isfinite(request.previous_command_acceleration)||
      !std::isfinite(request.control_dt)||request.control_dt<=0.) {
     out.command=Command();out.owner="safety_stop";out.reason+="|invalid_command_history";return;
   }
   if(!std::isfinite(out.command.speed))out.command.speed=0.;
   if(!std::isfinite(out.command.steering))out.command.steering=0.;
   const double dt=clamp(request.control_dt,.001,.25);
   const double target=std::max(0.,std::min(out.command.speed,std::min(.10,out.effective.limits.speed)));
   // Failure is not itself a collision emergency. Decelerate to the conservative
   // cap with admissible acceleration/jerk; never claim the new cap is reached
   // instantly while the delayed physical vehicle is still moving faster.
   const double alo=std::max({-l.deceleration,request.previous_command_acceleration-l.jerk*dt,-request.previous.speed/dt});
   const double ahi=std::min({l.acceleration,request.previous_command_acceleration+l.jerk*dt,(l.speed-request.previous.speed)/dt});
   if(alo<=ahi&&std::isfinite(alo)&&std::isfinite(ahi))
     out.command.speed=request.previous.speed+dt*clamp((target-request.previous.speed)/dt,alo,ahi);
   else {
     // A prior hard stop may have deliberately violated the ordinary jerk
     // envelope. Hold the last bounded speed for one tick to reinitialize it.
     out.command.speed=clamp(request.previous.speed,0.,l.speed);
     out.reason+="|reset_after_safety_override";
   }
   if(out.command.speed>target+1e-6)out.reason+="|decelerating_to_cap";
   const double dlo=std::max(-l.steering,request.previous.steering-l.steering_rate*dt);
   const double dhi=std::min(l.steering,request.previous.steering+l.steering_rate*dt);
   out.command.steering=dlo<=dhi?clamp(out.command.steering,dlo,dhi):clamp(request.previous.steering,-l.steering,l.steering);
 }
 RuntimeConfig config_;VehicleStateObserver observer_;TerrainConditionEstimator terrain_estimator_;ReducedNMPC solver_;FlatnessReducedNMPC flat_solver_;
 bool last_solver_ok_=false;double risk_=0.,roughness_risk_=0.;
 bool valid_=true,recovery_required_=false;std::string mode_="legacy_slqr";int scene_=3,pending_=3;
 double pending_since_=0.,last_now_=0.,speed_cap_=.10,recovery_since_=0.;Model model_;Weights weights_;
};
} // namespace research_control
