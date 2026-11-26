#include "control/p6_response_model.h"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace research_control;
using namespace research_control::p6;
namespace {
RawSample raw(double v,double t,std::uint64_t sequence=1) {
 RawSample r;r.value=v;r.source_stamp=r.receipt_stamp=t;r.valid=r.unit_verified=r.frame_verified=true;
 r.hardware_stamp_verified=r.new_frame_known=r.new_frame=true;r.sequence=sequence;r.source="synthetic_fixture";r.unit="SI";return r;
}
ResponsePolicy policy() {
 ResponsePolicy p;
 for(auto* c:{&p.steering,&p.longitudinal,&p.yaw,&p.vibration,&p.slip}){
  c->enabled=c->delay_verified=c->bias_verified=c->timing_verified=true;
  c->evidence_kind="synthetic_validation";c->evidence_sha=std::string(64,'a');
  c->min_samples=60;c->min_excitation=.001;c->max_innovation=3.;c->max_covariance=1000.;
  c->min_rank_ratio=.00001;c->prediction_improvement=.01;c->gain_rate=.1;c->tau_rate=.1;
 }
 return p;
}
SensorSnapshot snapshot(double t,double speed=.3,double steering=.1,double yaw=.05,std::uint64_t seq=1) {
 SensorSnapshot s;s.now=t;s.synthetic=s.history_complete=s.imu_frame_verified=true;
 s.wheel_speed=raw(speed,t,seq);s.steering=raw(steering,t,seq);s.yaw_rate=raw(yaw,t,seq);
 s.accel_x=raw(0.,t,seq);s.accel_y=raw(0.,t,seq);s.accel_z=raw(9.81,t,seq);
 s.run_id="synthetic_response";s.session_id="synthetic_training";s.scene="smooth";
 s.nominal_config_hash="fixture_model";s.cycle=seq;return s;
}
PublishedSample command(std::uint64_t seq,double t,double speed=.3,double steer=.1) {
 PublishedSample p;p.sequence=seq;p.stamp=t;p.command.speed=speed;p.command.steering=steer;p.published=p.committed=true;return p;
}
void test_default_and_causality() {
 ResponseEstimator e;Model m;m.steering_delay=m.speed_delay=0.;ResponsePolicy p;
 std::vector<PublishedSample> h{command(1,0.)};
 auto s=snapshot(1.);auto a=e.observe(s,h,m,p);assert(a.valid&&!a.slip.valid);
 s=snapshot(1.03,.3,.1001,.04,2);h.push_back(command(2,1.));a=e.observe(s,h,m,p);
 assert(!a.steering.update_allowed&&a.steering.reason=="channel_disabled");
 assert(a.effective.steering_tau==m.steering_tau);
 auto future=snapshot(1.06);future.steering.source_stamp=2.;a=e.observe(future,h,m,policy());
 assert(a.steering.reason=="future_source_sample");
 future=snapshot(1.09);future.steering.receipt_stamp=2.;a=e.observe(future,h,m,policy());assert(a.steering.reason=="future_or_missing_receipt");
 auto duplicate=e.observe(future,h,m,policy());assert(!duplicate.valid&&duplicate.reason=="clock_regressed_or_duplicate_freeze");
 std::cout<<"PASS default disabled, raw/future/cycle causality and slip optional\n";
}
void test_evidence_and_history() {
 ResponseEstimator e;Model m;m.steering_delay=m.speed_delay=0.;auto p=policy();
 std::vector<PublishedSample> h{command(1,.0)};
 auto s=snapshot(1.);e.observe(s,h,m,p);
 s=snapshot(1.03,.3,.11,.04,2);s.synthetic=false;h.push_back(command(2,1.));auto a=e.observe(s,h,m,p);
 assert(a.steering.reason=="synthetic_policy_on_real_data");
 s=snapshot(1.06,.3,.12,.04,3);h.push_back(command(4,1.04));a=e.observe(s,h,m,p);assert(a.steering.reason=="command_sequence_gap");
 s=snapshot(1.09,.3,.13,.04,4);s.history_complete=false;a=e.observe(s,h,m,p);assert(a.steering.reason=="command_history_incomplete");
 ResponseEstimator receipts;auto r=snapshot(2.);r.steering.hardware_stamp_verified=false;receipts.observe(r,h,m,p);
 r=snapshot(2.03,.3,.14,.04,2);r.steering.hardware_stamp_verified=false;h.clear();h.push_back(command(1,0.));
 a=receipts.observe(r,h,m,p);assert(a.steering.reason=="physical_time_unverified");
 r=snapshot(2.06,.3,.14,.04,3);r.steering.new_frame_known=false;a=receipts.observe(r,h,m,p);assert(a.steering.reason=="new_frame_unverified");
 std::cout<<"PASS actual command sequence, evidence domain, timing/new-frame gates\n";
}
void test_rls() {
 ResponseEstimator e;auto p=policy();Model m;m.steering_delay=m.speed_delay=0.;
 double time=1.,steer=0.,speed=.2;std::vector<PublishedSample> h;
 h.push_back(command(1,0.,.25,0.));ParameterSnapshot a;
 bool steer_valid=false,speed_valid=false,yaw_valid=false;
 double tau_est=0.,gain_est=0.;
 for(int i=1;i<=2600;++i){
  const double dt=(i%3==0)?.021:((i%3==1)?.033:.027);
  const double target_steer=.19*std::sin(.85*time)+.045*std::sin(2.7*time);
  const double target_speed=.31+.075*std::sin(1.15*time)+.035*std::sin(2.3*time);
  h.push_back(command(i+1,time,target_speed,target_steer));
  steer=.80*target_steer+(steer-.80*target_steer)*std::exp(-dt/.63);
  speed=.82*target_speed+(speed-.82*target_speed)*std::exp(-dt/.48);time+=dt;
  auto s=snapshot(time,speed,steer,.76*speed*std::tan(steer)/m.wheelbase,i);
  s.accel_z.value+=.1*std::sin(time*16.);
  a=e.observe(s,h,m,p);
  steer_valid|=a.steering.update_allowed;speed_valid|=a.longitudinal.update_allowed;yaw_valid|=a.yaw.update_allowed;
  if(a.steering.update_allowed){tau_est=a.steering.tau.value;gain_est=a.steering.gain.value;}
 }
 std::cout<<"RLS gain="<<gain_est<<" tau="<<tau_est<<" speed="<<a.longitudinal.gain.value<<"/"<<a.longitudinal.tau.value
  <<" yaw="<<a.yaw.gain.value<<" reason="<<a.steering.reason<<" rank="<<a.steering.rank_ratio<<"\n";
 assert(steer_valid&&speed_valid&&yaw_valid);
 assert(std::fabs(tau_est-.63)<.06&&std::fabs(gain_est-.80)<.03);
 assert(std::fabs(a.effective.steering_tau-.63)<.08);
 assert(std::fabs(a.effective.speed_gain-.82)<.05);
 assert(std::fabs(a.effective.yaw_gain-.76)<.05);
 assert(a.slip.reason=="slip_invalid_independent_motion_unavailable");
 assert(a.roughness.valid&&a.roughness.value>0.);
 assert(a.effective_hash.size()>0&&a.changes[0].nominal==1.);
 const double before=a.effective.steering_tau;
 for(int i=0;i<500;++i){time+=.03;auto s=snapshot(time,speed,steer,.0,3000+i);s.steering.valid=false;s.wheel_speed.valid=false;s.yaw_rate.valid=false;
  a=e.observe(s,h,m,p);}
 assert(std::fabs(a.effective.steering_tau-m.steering_tau)<std::fabs(before-m.steering_tau));
 assert(std::fabs(a.effective.steering_tau-m.steering_tau)<1e-9);
 auto paused=p;paused.steering.enabled=false;
 time+=.03;a=e.observe(snapshot(time,speed,steer,0.,3600),h,m,paused);
 assert(a.effective.steering_tau==m.steering_tau);
 time+=.03;a=e.observe(snapshot(time,speed,steer,0.,3601),h,m,p);
 assert(!a.steering.update_allowed&&a.steering.accepted<static_cast<std::uint64_t>(p.steering.min_samples));
 std::cout<<"PASS variable-dt gain/tau RLS, proxy yaw, model influence and bounded nominal recovery\n";
}
void test_policy_and_rejected_update() {
 ResponseEstimator e;auto p=policy();Model m;m.steering_delay=m.speed_delay=0.;
 std::vector<PublishedSample> h{command(1,0.)};auto s=snapshot(1.);auto a=e.observe(s,h,m,p);
 p.steering.return_rate=missing();s=snapshot(1.03);a=e.observe(s,h,m,p);
 assert(!a.valid&&a.reason=="invalid_response_policy_reject_candidate");
 p=policy();ResponseEstimator bad_history;bad_history.observe(snapshot(2.),h,m,p);
 h.push_back(command(2,2.));h.back().committed=false;
 a=bad_history.observe(snapshot(2.03,.3,.11,.03,2),h,m,p);
 assert(!a.steering.update_allowed&&a.steering.reason=="uncommitted_or_future_history");
 ResponseEstimator duplicates;h.clear();h.push_back(command(1,0.));
 a=duplicates.observe(snapshot(3.,.3,.1,.04,1),h,m,p);
 s=snapshot(3.03,.3,.1,.04,1);a=duplicates.observe(s,h,m,p);
 assert(a.yaw.reason=="duplicate_or_out_of_order_yaw_sample");
 s=snapshot(3.06,.3,.1,.04,2);s.steering.new_frame=false;
 a=duplicates.observe(s,h,m,p);assert(a.steering.reason=="not_a_new_hardware_frame");
 std::cout<<"PASS invalid policy, candidate-history exclusion, duplicate yaw and old CAN frames\n";
}
void test_weak_and_slip() {
 ResponseEstimator e;auto p=policy();Model m;m.steering_delay=m.speed_delay=0.;std::vector<PublishedSample> h{command(1,0.,.3,0.)};ParameterSnapshot a;
 for(int i=0;i<200;++i){auto s=snapshot(1.+i*.03,.3,0.,0.,i+1);a=e.observe(s,h,m,p);}
 assert(!a.steering.gain.valid&&!a.yaw.gain.valid);assert(a.effective.yaw_gain==m.yaw_gain);
 auto s=snapshot(8.,.30,.1,.03,301);s.ground_speed=raw(.25,8.);s.ground_speed.independent=true;s.ground_frame_verified=s.ground_lever_arm_verified=true;
 a=e.observe(s,h,m,p);assert(a.slip.valid&&std::fabs(a.slip.value-1./6.)<1e-9);
 s.now=8.01;s.ground_speed.new_frame_known=false;a=e.observe(s,h,m,p);
 assert(!a.slip.valid&&a.slip.reason=="slip_invalid_new_frame_unverified");
 s=snapshot(8.03,.01,.1,.03,302);s.ground_speed=raw(.01,8.03);s.ground_speed.independent=true;s.ground_frame_verified=s.ground_lever_arm_verified=true;
 a=e.observe(s,h,m,p);assert(!a.slip.valid&&a.slip.reason=="slip_invalid_low_speed");
 Model bad=m;bad.steering_tau=missing();s.now=8.06;a=e.observe(s,h,bad,p);assert(!a.valid);
 std::cout<<"PASS weak-excitation freeze, optional independent slip, severe model invalid\n";
}
void test_receipt_domain_is_not_physical_delay(){
 ResponseEstimator e;auto p=policy();p.steering.allow_receipt_identification=true;p.steering.min_confidence=.3;
 Model m;m.steering_delay=.02;m.speed_delay=0.;std::vector<PublishedSample> h{command(1,0.,.3,0.)};double t=1.,delta=0.;bool activated=false;ParameterSnapshot fitted;
 for(int i=1;i<=1800;++i){const double dt=.03,u=.18*std::sin(.9*t)+.03*std::sin(2.7*t);h.push_back(command(i+1,t,.3,u));
  delta=.8*u+(delta-.8*u)*std::exp(-dt/.63);t+=dt;auto s=snapshot(t+.02,.3,delta,0.,i);
  s.steering.source_stamp=missing();s.steering.hardware_stamp_verified=false;s.steering.receipt_stamp=t+.02;
  fitted=e.observe(s,h,m,p);activated|=fitted.steering.update_allowed;
 }
 assert(activated&&fitted.steering.time_domain=="command_to_receipt_domain");assert(fitted.steering.gain.confidence<=.35);
 assert(std::abs(fitted.steering.gain.value-.8)<.03&&std::abs(fitted.steering.tau.value-.63)<.06);
 assert(fitted.changes[7].reason=="fixed_synthetic_delay_not_real_calibration");
 std::cout<<"PASS synthetic receipt-domain response, confidence cap and no physical-delay claim\n";
}
void test_sensor_time_not_control_period(){
 // The same 20 Hz measurements seen at different control frequencies must
 // produce the same identification/filter state. Extra callbacks contain only
 // repeated past measurements; they are not new physical observations.
 for(bool additive:{false,true}){
  ResponseEstimator sensor_rate,fast_controller;auto p=policy();p.yaw_additive_enabled=additive;
  Model m;m.steering_delay=m.speed_delay=0.;std::vector<PublishedSample> h{command(1,0.)};
  for(int i=1;i<=160;++i){
   const double t=1.+i*.05,steer=.12+.02*std::sin(t),phi=.3*std::tan(steer)/m.wheelbase;
   auto s=snapshot(t,.3,steer,.8*phi+.002*std::sin(2.*t),i);
   s.accel_z.value+=.2*std::sin(17.*t);
   auto a=sensor_rate.observe(s,h,m,p),b=fast_controller.observe(s,h,m,p);
   assert(a.yaw.accepted==b.yaw.accepted);
   assert(std::abs(a.yaw.information-b.yaw.information)<1e-14);
   assert(a.yaw.residual.valid&&b.yaw.residual.valid);
   assert(std::abs(a.yaw.residual.value-b.yaw.residual.value)<1e-14);
   if(std::isfinite(a.yaw.gain.value))assert(std::abs(a.yaw.gain.value-b.yaw.gain.value)<1e-14);
   if(std::isfinite(a.yaw.candidate_mse))assert(std::abs(a.yaw.candidate_mse-b.yaw.candidate_mse)<1e-14);
   assert(a.roughness.valid&&b.roughness.valid&&std::abs(a.roughness.value-b.roughness.value)<1e-14);
   s.now=t+.015;b=fast_controller.observe(s,h,m,p);
   assert(!b.roughness.valid&&b.roughness.reason=="duplicate_or_out_of_order_acceleration_sample");
   assert(b.yaw.reason=="duplicate_or_out_of_order_yaw_sample");
   s.now=t+.033;fast_controller.observe(s,h,m,p);
  }
 }
 std::cout<<"PASS sensor-time yaw gain/additive and vibration invariant to repeated faster control snapshots\n";
}
void test_known_additive_offsets_do_not_bias_parameter_fits(){
 // Synthetic estimator-contract fixture: the speed update exactly satisfies
 // the trapezoidal integral regression. This isolates offset accounting from
 // the separate numerical error of approximating a physical feedback integral.
 // Positive, varying yaw excitation deliberately makes an unremoved additive
 // yaw offset look like an erroneous gain increase.
 for(bool mismatch:{false,true}){
  ResponseEstimator estimator;auto p=policy();p.steering.enabled=p.vibration.enabled=p.slip.enabled=false;
  Model m;m.steering_delay=m.speed_delay=0.;m.speed_gain=.92;m.speed_tau=.36;
  m.longitudinal_residual=.06;m.yaw_gain=1.05;m.yaw_residual=.025;
  const double speed_gain=mismatch?.80:m.speed_gain,speed_tau=mismatch?.57:m.speed_tau;
  const double yaw_gain=mismatch?.78:m.yaw_gain;
  double t=1.,speed=speed_gain*.3+m.longitudinal_residual*speed_tau;
  std::vector<PublishedSample> history{command(1,0.,.3,0.)};ParameterSnapshot result;
  bool speed_updated=false,yaw_updated=false;
  for(int i=1;i<=1600;++i){
   const double dt=(i%3==0)?.017:((i%3==1)?.031:.043);
   const double u=.31+.09*std::sin(1.15*t)+.035*std::sin(2.3*t);
   history.push_back(command(i+1,t,u,0.));
   speed=((1.-dt/(2.*speed_tau))*speed+dt*(speed_gain/speed_tau*u+m.longitudinal_residual))/(1.+dt/(2.*speed_tau));
   t+=dt;const double steering=.15+.065*std::sin(.7*t)+.02*std::sin(2.3*t);
   const double phi=speed*std::tan(steering)/m.wheelbase;
   result=estimator.observe(snapshot(t,speed,steering,yaw_gain*phi+m.yaw_residual,i),history,m,p);
   speed_updated|=result.longitudinal.update_allowed;yaw_updated|=result.yaw.update_allowed;
   assert(result.effective.longitudinal_residual==m.longitudinal_residual);
   assert(result.effective.yaw_residual==m.yaw_residual);
   if(!mismatch){
    if(result.longitudinal.residual.valid)assert(std::fabs(result.longitudinal.residual.value)<1e-11);
    assert(std::fabs(result.yaw.residual.value)<1e-13);
    assert(result.effective.speed_gain==m.speed_gain&&result.effective.speed_tau==m.speed_tau);
    assert(result.effective.yaw_gain==m.yaw_gain);
   }
  }
  assert(result.longitudinal.accepted>=1000&&result.yaw.accepted>=1000);
  if(!mismatch){assert(!speed_updated&&!yaw_updated);}
  else{
   assert(speed_updated&&yaw_updated);
   assert(std::fabs(result.longitudinal.gain.value-speed_gain)<.01);
   assert(std::fabs(result.longitudinal.tau.value-speed_tau)<.02);
   assert(std::fabs(result.yaw.gain.value-yaw_gain)<.01);
   assert(std::fabs(result.effective.speed_gain-speed_gain)<.01);
   assert(std::fabs(result.effective.speed_tau-speed_tau)<.02);
   assert(std::fabs(result.effective.yaw_gain-yaw_gain)<.01);
   assert(result.longitudinal.candidate_mse<.01*result.longitudinal.nominal_mse);
   assert(result.yaw.candidate_mse<.01*result.yaw.nominal_mse);
  }
 }
 std::cout<<"PASS known nonzero speed/yaw offsets: no spurious nominal adaptation; mismatched gain/tau recovery (synthetic)\n";
}
}
void test_scene_and_channel_isolation() {
 ResponseEstimator estimator;
 Model nominal;
 auto rules=policy();
 rules.yaw.min_samples=12;
 ParameterSnapshot result;
 const std::vector<PublishedSample> history;
 for(int i=1;i<=300;++i){
  auto s=snapshot(1.+.03*i,.3,.15,.7*.3*std::tan(.15)/nominal.wheelbase,i);
  result=estimator.observe(s,history,nominal,rules);
 }
 assert(result.yaw.update_allowed&&result.effective.yaw_gain<.95);
 auto unknown=snapshot(10.03,.3,.15,0.,301);
 unknown.scene="unknown";
 nominal.yaw_gain=.95;
 nominal.steering_delay=.28;
 ResponsePolicy disabled;
 result=estimator.observe(unknown,history,nominal,disabled);
 assert(result.valid&&!result.yaw.update_allowed);
 assert(result.effective.yaw_gain==nominal.yaw_gain);
 assert(result.effective.steering_delay==nominal.steering_delay);
 assert(!result.dynamic_flags[1]);
}
int main(){test_default_and_causality();test_evidence_and_history();test_rls();test_policy_and_rejected_update();test_weak_and_slip();test_receipt_domain_is_not_physical_delay();test_sensor_time_not_control_period();test_known_additive_offsets_do_not_bias_parameter_fits();test_scene_and_channel_isolation();std::cout<<"P6 response model tests passed (synthetic only)\n";}
