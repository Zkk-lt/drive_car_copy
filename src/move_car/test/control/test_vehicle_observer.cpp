#include "control/vehicle_state_observer.h"
#include "control/terrain_condition_estimator.h"
#include <cassert>
#include <iostream>

using namespace research_control;
namespace {
ObserverConfig calibrated() {
 ObserverConfig c;c.wheel_scale=1.;c.wheel_calibrated=true;c.steering_calibrated=true;
 c.ground_calibrated=c.ground_frame_verified=c.ground_independent=true;
 c.ground_heading_verified=c.ground_lever_arm_verified=c.imu_frame_verified=true;
 c.source_timestamps_verified=true;
 c.steering_delay_verified=true;
 return c;
}
void samples(VehicleStateObserver& observer,double t,double wheel=.5,double ground=.4,
             double steering=.1,double gyro=.07,bool orientation=true) {
 observer.updateWheel(wheel,t,t);observer.updateSteering(steering,t,t);
 observer.updateGround(ground,0.,0.,t,t);
 observer.updateImu(gyro,0.,0.,9.81,0.,0.,t,t,true,true,orientation);
}
void raw_and_calibration() {
 VehicleStateObserver o;Model m;samples(o,1.);
 Observation x=o.observe(1.,m);
 assert(!x.speed.valid&&!x.slip.valid&&!x.steering.valid&&!x.yaw_rate.valid);
 assert(o.rawWheel().value==.5&&o.rawWheel().time.source_stamp==1.);
 ObserverConfig c=calibrated();c.wheel_scale=.001;c.wheel_sign=-1.;
 o.configure(c);samples(o,2.,-500.,.4);
 x=o.observe(2.,m);assert(x.speed.valid&&std::fabs(x.speed.value-.4)<1e-12);
 assert(x.slip.valid&&std::fabs(x.slip.value-.2)<1e-12);
 assert(o.rawWheel().value==-500.);
 c.ground_independent=false;o.configure(c);samples(o,3.,-500.,.4);
 x=o.observe(3.,m);assert(!x.slip.valid&&!x.sideslip.valid&&!x.yaw_gain.valid);
 assert(x.slip.reason=="ground_wheel_independence_unverified");
 c=calibrated();c.wheel_scale=0.;o.configure(c);assert(!o.configValid());
 x=o.observe(4.,m);assert(x.speed.reason=="invalid_observer_config");
 std::cout<<"PASS raw/calibration/independence gates\n";
}
void observability() {
 VehicleStateObserver o(calibrated());Model m;samples(o,1.,.02,.01);
 Observation x=o.observe(1.,m);assert(!x.slip.valid&&!x.sideslip.valid);
 samples(o,1.1,.5,.4,.1,.07,false);x=o.observe(1.1,m);
 assert(x.yaw_rate.valid&&!x.roughness.valid&&x.roughness.reason=="imu_tilt_unavailable");
 o.updateGround(.4,0.,missing(),1.2,1.2,false);
 o.updateWheel(.5,1.2,1.2);x=o.observe(1.2,m);
 assert(x.speed.valid&&x.speed.reason=="calibrated_wheel_speed"&&!x.slip.valid);
 // Reverse lambda remains signed body-x, not an invented torque label.
 samples(o,1.3,-.5,-.4);x=o.observe(1.3,m);
 assert(x.slip.valid&&std::fabs(x.slip.value+.2)<1e-12&&!x.sideslip.valid);
 o.reset();samples(o,2.,.5,.4);o.updateGround(.4,0.,0.,2.1,2.1);
 x=o.observe(2.1,m);assert(!x.slip.valid&&x.slip.reason=="speed_pair_unsynchronized");
 std::cout<<"PASS low speed/heading/IMU/reverse/synchronization\n";
}
void timing() {
 VehicleStateObserver o(calibrated());Model m;samples(o,1.);
 assert(o.observe(1.,m).speed.valid);
 assert(!o.updateWheel(.7,.9,1.01));assert(o.rawWheel().time.reason=="duplicate_or_out_of_order");
 assert(o.rawWheel().value==.7&&!o.observe(1.01,m).slip.valid);
 assert(!o.updateWheel(.7,2.,1.02));assert(o.rawWheel().time.reason=="future_source_stamp");
 assert(!o.updateWheel(.7,1.1,2.));assert(o.rawWheel().time.reason=="late_sensor_sample");
 Observation stale=o.observe(1.5,m);assert(!stale.speed.valid&&!stale.yaw_rate.valid);
 Observation reset=o.observe(.5,m);assert(reset.speed.reason=="clock_regressed");
 assert(o.commandHistory().empty());samples(o,.6);assert(o.observe(.6,m).speed.valid);
 o.reset();o.updateWheel(.5,0.,1.);Observation receipt=o.observe(1.,m);
 assert(receipt.speed.valid&&receipt.speed.confidence<.6);
 assert(!o.rawWheel().time.source_time_valid&&o.rawWheel().time.source_stamp==0.);
 assert(!o.recordCommand(Command{},missing()));assert(o.recordCommand(Command{},1.));
 assert(!o.recordCommand(Command{},1.));
 ObserverConfig unverified=calibrated();unverified.source_timestamps_verified=false;
 o.configure(unverified);samples(o,2.);Observation driver=o.observe(2.,m);
 assert(driver.speed.valid&&driver.speed.confidence<.6&&driver.slip.confidence<.5);
 assert(o.rawGround().time.source_time_valid&&o.rawGround().time.reason=="driver_timestamp_unverified");
 o.invalidateGround("ground_velocity_frame_contract_unverified");
 driver=o.observe(2.01,m);assert(!driver.slip.valid);
 assert(o.rawGround().vx==.4&&o.rawGround().time.reason=="ground_velocity_frame_contract_unverified");
 o.invalidateImu("imu_frame_contract_unverified");assert(!o.observe(2.02,m).yaw_rate.valid);
 std::cout<<"PASS timestamps/staleness/clock reset/receipt confidence\n";
}
void lever_arm() {
 ObserverConfig c=calibrated();c.ground_lever_x=.3;c.ground_lever_y=.1;
 VehicleStateObserver o(c);Model m;
 // Rear velocity (0.4,0.03), yaw rate .2; sensor velocity at (.3,.1)
 // is (0.38,0.09). Rotate into arbitrary world heading.
 const double h=.8,vx=.38,vy=.09;
 o.updateWheel(.4,1.,1.);o.updateSteering(.2,1.,1.);
 o.updateGround(std::cos(h)*vx-std::sin(h)*vy,std::sin(h)*vx+std::cos(h)*vy,h,1.,1.);
 o.updateImu(.2,0.,0.,9.81,0.,0.,1.,1.,true,true,true);
 Observation x=o.observe(1.,m);
 assert(x.speed.valid&&std::fabs(x.speed.value-.4)<1e-12);
 assert(x.slip.valid&&std::fabs(x.slip.value)<1e-12);
 assert(x.sideslip.valid&&std::fabs(x.sideslip.value-std::atan2(.03,.4))<1e-12);
 assert(x.lateral_residual.valid&&std::fabs(x.lateral_residual.value-.03)<1e-12);
 std::cout<<"PASS coordinate rotation and sensor lever correction\n";
}
void identification() {
 ObserverConfig c=calibrated();VehicleStateObserver o(c);Model m;
 m.steering_delay=0.;m.speed_delay=0.;
 const double true_tau=.65,true_bias=.018,true_gain=.72,dt=.02;
 double delta=0.;Observation x;bool gain_ever=false,tau_ever=false;
 // Piecewise persistent excitation distinguishes offset from inverse tau.
 for(int i=0;i<4000;++i) {
   const double t=1.+i*dt;
   const double command=.18*std::sin(.9*(t-1.))+.06*std::sin(2.3*(t-1.));
   if(i)delta=(command+true_bias)+(delta-command-true_bias)*std::exp(-dt/true_tau);
   o.recordCommand(Command{.45,command},t-dt*.5-1e-5);
   const double yaw=true_gain*.45*std::tan(delta)/m.wheelbase;
   samples(o,t,.45,.45,delta,yaw);
   x=o.observe(t,m);gain_ever=gain_ever||x.yaw_gain.valid;tau_ever=tau_ever||x.steering_tau.valid;
 }
 assert(gain_ever&&tau_ever&&x.yaw_gain.valid&&x.steering_tau.valid&&x.steering_bias.valid);
 assert(std::fabs(x.yaw_gain.value-true_gain)<.025);
 assert(std::fabs(x.steering_tau.value-true_tau)<.04);
 assert(std::fabs(x.steering_bias.value-true_bias)<.008);
 assert(std::fabs(x.yaw_residual.value)<=c.yaw_residual_bound);
 assert(x.longitudinal_residual.valid&&std::fabs(x.longitudinal_residual.value)<1e-9);
 std::cout<<"PASS excited RLS gain="<<x.yaw_gain.value<<" tau="<<x.steering_tau.value
          <<" bias="<<x.steering_bias.value<<"\n";
 const auto before=o.identificationDiagnostics();
 // A finite sensor jump used to be clipped into bounded model compensation.
 // It must invalidate the current fit and not add information to the RLS.
 o.recordCommand(Command{.45,0.},80.99);
 samples(o,81.,.45,.45,delta+.08,1.);
 x=o.observe(81.,m);
 assert(!x.yaw_gain.valid&&!x.yaw_residual.valid&&!x.steering_tau.valid);
 assert(x.yaw_residual.reason=="yaw_innovation_outlier");
 assert(x.steering_tau.reason=="steering_rls_innovation_rejected");
 const auto rejected=o.identificationDiagnostics();
 assert(rejected.yaw_rejected>before.yaw_rejected&&rejected.steering_rejected>before.steering_rejected);
 assert(rejected.yaw_samples==before.yaw_samples&&rejected.steering_samples==before.steering_samples);
 // Low-speed and zero-regressor operation immediately revokes yaw observability.
 samples(o,81.02,.02,.02,.1,.002);x=o.observe(81.02,m);
 assert(!x.yaw_gain.valid&&x.yaw_gain.reason=="low_speed_yaw_unobservable");
 samples(o,81.04,.45,.45,0.,0.);x=o.observe(81.04,m);assert(!x.yaw_gain.valid);
 std::cout<<"PASS finite outlier rejection and immediate low-excitation revocation\n";
 // No excitation must revoke current observability, even with an old fit.
 for(int i=0;i<100;++i) {double t=82.+i*dt;samples(o,t,.45,.45,0.,0.);x=o.observe(t,m);}
 assert(!x.yaw_gain.valid&&!x.steering_tau.valid);
 c.identify_yaw=c.identify_steering=false;o.configure(c);samples(o,1.);x=o.observe(1.,m);
 assert(!x.yaw_gain.valid&&!x.steering_tau.valid);
 std::cout<<"PASS excitation loss and independent identification switches\n";
}
void identification_ambiguity() {
 ObserverConfig c=calibrated();VehicleStateObserver o(c);Model m;
 Observation x;
 for(int i=0;i<1000;++i) {
   const double t=1.+i*.02;
   // A constant turn cannot distinguish a gain change from a constant gyro offset.
   samples(o,t,.45,.45,.12,.72*.45*std::tan(.12)/m.wheelbase);
   x=o.observe(t,m);
 }
 assert(!x.yaw_gain.valid&&x.yaw_gain.reason=="yaw_regressor_variation_insufficient");
 assert(o.identificationDiagnostics().yaw_samples>900);
 c.steering_delay_verified=false;o.configure(c);samples(o,1.);x=o.observe(1.,m);
 assert(!x.steering_tau.valid&&x.steering_tau.reason=="steering_deadtime_unverified");
 assert(x.steering.valid); // Calibration gate must not discard raw/actual steering.
 std::cout<<"PASS constant-turn ambiguity and tau/deadtime calibration gate\n";
}
void headerless_steering_confidence() {
 VehicleStateObserver o(calibrated());Model m;bool observed=false;
 double max_confidence=0.;
 // Verified ground/gyro acquisition times do not make headerless CAN steering
 // acquisition time verified. The yaw regressor contains actual steering.
 for(int i=0;i<4000;++i) {
   const double t=1.+i*.02;
   const double delta=.18*std::sin(.9*t)+.06*std::sin(2.3*t);
   o.updateWheel(.45,t,t);o.updateGround(.45,0.,0.,t,t);
   o.updateSteering(delta,0.,t);
   o.updateImu(.72*.45*std::tan(delta)/m.wheelbase,0.,0.,9.81,0.,0.,t,t,true,true,true);
   const Observation x=o.observe(t,m);
   assert(x.steering.valid&&std::fabs(x.steering.confidence-.65)<1e-12);
   if(x.yaw_gain.valid) {
     observed=true;max_confidence=std::max(max_confidence,x.yaw_gain.confidence);
     assert(x.yaw_gain.confidence<=.9*.65+1e-12);
   }
 }
 assert(observed&&max_confidence>.58&&max_confidence<.8);
 std::cout<<"PASS headerless steering propagates to yaw-gain confidence="<<max_confidence<<"\n";
}
void roughness_and_terrain() {
 VehicleStateObserver o(calibrated());TerrainConditionEstimator terrain;Model m;
 Observation x;TerrainEstimate estimate;
 for(int i=0;i<400;++i) {
   const double t=1.+i*.02;
   o.updateWheel(.4,t,t);o.updateSteering(0.,t,t);o.updateGround(.4,0.,0.,t,t);
   o.updateImu(0.,0.,0.,9.81+2.*std::sin(2.*3.141592653589793*7.*t),0.,0.,t,t,true,true,true);
   x=o.observe(t,m);estimate=terrain.update(x,t);
 }
 assert(x.roughness.valid&&x.roughness.value>1.2);
 assert(estimate.valid&&estimate.label=="grass_bumpy");
 double sum=0.;for(double p:estimate.probabilities){assert(p>=0.&&p<=1.);sum+=p;}
 assert(std::fabs(sum-1.)<1e-12);
 for(int i=0;i<300;++i) {
   const double t=9.+i*.02;samples(o,t,.4,.4,0.,0.);x=o.observe(t,m);estimate=terrain.update(x,t);
 }
 assert(estimate.label=="unknown"&&!estimate.valid);
 assert(estimate.reason=="smooth_materials_unidentifiable");
 TerrainEstimatorConfig tc;tc.enabled=false;terrain.configure(tc);
 assert(terrain.update(x,15.).reason=="terrain_estimation_disabled");
 terrain.configure(TerrainEstimatorConfig());x.roughness.valid=false;
 assert(terrain.update(x,15.).label=="unknown");
 std::cout<<"PASS roughness/window/dwell/material ambiguity/disabled\n";
}
}
int main() {
 raw_and_calibration();observability();timing();lever_arm();identification();identification_ambiguity();headerless_steering_confidence();roughness_and_terrain();
 std::cout<<"Observer and terrain tests passed\n";
}
