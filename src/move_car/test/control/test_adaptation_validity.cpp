#include "control/adaptive_runtime.h"
#include <iostream>
#include <stdexcept>

namespace rc=research_control;
std::string validity_backend="original12";
void require_validity(bool condition,const char* message) {
 if(!condition)throw std::runtime_error(message);
}
rc::RuntimeConfig validityConfig() {
 rc::RuntimeConfig c;c.research_validity_adaptation=true;c.solver.budget_ms=1000.;
 c.research_backend=validity_backend;
 c.observer.source_timestamps_verified=true;
 c.observer.steering_delay_verified=true;
 c.observer.wheel_calibrated=c.observer.steering_calibrated=true;
 c.observer.ground_calibrated=c.observer.ground_frame_verified=c.observer.ground_independent=true;
 c.observer.ground_heading_verified=c.observer.ground_lever_arm_verified=c.observer.imu_frame_verified=true;
 c.observer.identify_steering=false;
 return c;
}
rc::SolveRequest validityRequest(double now,double speed,double steering) {
 rc::SolveRequest r;r.now=now;r.state.x=-.3;r.state.speed=speed;r.state.steering=steering;
 r.previous.speed=r.baseline.speed=speed;r.previous.steering=r.baseline.steering=steering;
 r.speed_cap=.30;r.history.push_back({now-1.,r.previous});
 r.reference.valid=true;r.reference.stamp=now;r.reference.count=21;
 for(int i=0;i<21;++i){auto& p=r.reference.points[i];p.t=i*.1;p.x=p.s=speed*p.t;p.speed=speed;}
 return r;
}
void feedValidity(rc::AdaptiveRuntime& runtime,double now,double speed,double steering,double gain) {
 auto& observer=runtime.observer();observer.updateWheel(speed*1000.,now,now);
 observer.updateSteering(steering,now,now);observer.updateGround(speed,0.,0.,now,now);
 observer.updateImu(gain*speed*std::tan(steering)/.6,0.,0.,9.80665,0.,0.,now,now,true,true,true);
}
int main(int argc,char** argv){try{
 if(argc>1)validity_backend=argv[1];
 require_validity(validity_backend=="original12"||validity_backend=="curve8","unsupported validity test backend");
 require_validity(!rc::RuntimeConfig().research_validity_adaptation,"validity enabled by default");
 auto config=validityConfig();rc::AdaptiveRuntime runtime;
 require_validity(runtime.configure(config),"validity configuration rejected");
 rc::RuntimeResult out;double previous_gain=1.;int evidence=0,applied=0;
 // Identifying gain requires variable, independent measurements. RLS is never
 // supplied the plant's gain; it sees only synthetic raw gyro/steering/speed.
 runtime.requestTerrain("smooth",1.);
 for(int i=0;i<1000;++i) {
   const double now=1.+i/30.,d=.22*std::sin(.95*i/30.),v=.24;
   feedValidity(runtime,now,v,d,.75);
   if(i==100)runtime.requestMode("nmpc_model_adaptive");
   out=runtime.cycle(validityRequest(now,v,d),now,true);
   if(i>=100)require_validity(out.solve.decision_dimension==(validity_backend=="curve8"?8:12),
       "adaptation mode did not enter selected optimizer dimension");
   runtime.observer().recordCommand(out.command,now);
   require_validity(!out.adaptation[2].active&&out.effective.model.yaw_residual==0.,
       "unidentified additive yaw bias entered model");
   require_validity(out.adaptation[2].reason=="yaw_residual_not_jointly_identified"||i==0,
       "missing per-parameter rejection reason");
   require_validity(std::fabs(out.effective.model.yaw_gain-previous_gain)<=.05/30.+1e-10,
       "gain update exceeded existing rate bound");
   require_validity(out.effective.model.yaw_gain>=.85-1e-12&&out.effective.model.yaw_gain<=1.15+1e-12,
       "gain exceeded existing bounded target");
   previous_gain=out.effective.model.yaw_gain;
   evidence+=out.adaptation[0].active;applied+=out.effective.model.yaw_gain<.999;
 }
 require_validity(evidence>0&&applied>0,"validity branch only changes labels");
 const double credible_gain=out.effective.model.yaw_gain;
 runtime.requestMode("nmpc_fixed");
 const double now=35.;feedValidity(runtime,now,.24,.1,.75);
 out=runtime.cycle(validityRequest(now,.24,.1),now,true);
 require_validity(!out.model_adaptation_active,"fixed mode activates target");
 require_validity(std::fabs(out.effective.model.yaw_gain-credible_gain)<=.05*.2+1e-12,
     "fixed transition jumped model");
 for(bool independent:{false,true}) {
   auto c=validityConfig();c.observer.ground_independent=independent;
   rc::AdaptiveRuntime rt;require_validity(rt.configure(c),"gate fixture configuration");
   rt.requestMode("nmpc_full_adaptive");
   // Independent but low-speed, or moving with an unverified independent
   // source: neither can identify yaw gain or a true slip response.
   const double speed=independent?.08:.24;
   for(int i=0;i<90;++i) {
     double t=1.+i/30.;feedValidity(rt,t,speed,.1*std::sin(t),.6);
     auto z=rt.cycle(validityRequest(t,speed,.1*std::sin(t)),t,true);
     require_validity(z.effective.model.yaw_gain==1.&&z.effective.model.steering_tau==.44&&
       z.effective.model.yaw_residual==0.,"unobservable low-speed parameter changed");
     for(const auto& e:z.adaptation)require_validity(!e.active,"low-speed evidence accepted");
     require_validity(!z.observation.yaw_gain.valid,"unobservable yaw measurement marked valid");
   }
 }
 for(bool verified_clock:{false,true}) {
   auto c=validityConfig();c.observer.identify_steering=true;
   c.observer.source_timestamps_verified=verified_clock;
   c.observer.steering_delay_verified=false;
   rc::AdaptiveRuntime rt;require_validity(rt.configure(c),"clock/delay gate fixture configuration");
   rt.requestMode("nmpc_full_adaptive");
   for(int i=0;i<120;++i) {
     double t=1.+i/30.,d=.22*std::sin(.9*t);
     feedValidity(rt,t,.24,d,.75);
     auto z=rt.cycle(validityRequest(t,.24,d),t,true);
     rt.observer().recordCommand(z.command,t);
     require_validity(!z.observation.steering_tau.valid&&z.effective.model.steering_tau==.44,
         "unverified deadtime produced a valid time constant");
     require_validity(!z.adaptation[1].active,"unverified deadtime accepted prediction evidence");
     if(!verified_clock) {
       require_validity(z.effective.model.yaw_gain==1.,"unverified acquisition time adapted gain");
       require_validity(!z.adaptation[0].active,"unverified acquisition time accepted gain evidence");
     }
   }
 }
 std::cout<<"PASS "<<validity_backend<<" validity default-off, identifiable gain activation, additive-yaw rejection, rate, low-speed/independence, acquisition-time and deadtime gates\n";
 return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
