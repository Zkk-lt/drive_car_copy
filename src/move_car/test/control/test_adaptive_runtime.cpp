#include "control/adaptive_runtime.h"
#include <iostream>
#include <stdexcept>
namespace rc=research_control;
void check(bool c,const char* what){if(!c)throw std::runtime_error(what);}
rc::RuntimeConfig calibrated() {
 rc::RuntimeConfig c;c.solver.budget_ms=1000.;
 c.observer.source_timestamps_verified=true;
 c.observer.steering_delay_verified=true;
 c.observer.wheel_calibrated=c.observer.steering_calibrated=true;
 c.observer.ground_calibrated=c.observer.ground_frame_verified=c.observer.ground_independent=true;
 c.observer.ground_heading_verified=c.observer.ground_lever_arm_verified=c.observer.imu_frame_verified=true;
 return c;
}
rc::SolveRequest request(double now) {
 rc::SolveRequest r;r.now=now;r.state.x=-.3;r.state.speed=.08;
 r.previous.speed=r.baseline.speed=.08;r.speed_cap=.10;
 r.history.push_back({now-1.,r.previous});r.reference.valid=true;r.reference.stamp=now;r.reference.count=21;
 for(int i=0;i<21;++i){auto& p=r.reference.points[i];p.t=i*.1;p.x=p.s=.08*p.t;p.speed=.08;}
 return r;
}
int main(){try{
 rc::AdaptiveRuntime runtime;auto c=calibrated();check(runtime.configure(c),"configuration");
 auto r=request(1.);auto out=runtime.cycle(r,1.,true);
 check(out.owner=="legacy_slqr"&&!out.nmpc_selected&&out.solve.reason=="not_run","default legacy mode");
 { // Match the ROS bridge's unknown measured fields; independent observations
   // must resolve them before state validity is decided.
   rc::AdaptiveRuntime rt;check(rt.configure(c),"bridge measurement config");rt.requestMode("nmpc_fixed");
   auto q=request(1.);q.state.speed=q.state.steering=rc::missing();
   auto& o=rt.observer();o.updateWheel(80.,1.,1.);o.updateSteering(0.,1.,1.);
   o.updateGround(.08,0.,0.,1.,1.);o.updateImu(0.,0.,0.,9.80665,0.,0.,1.,1.,true,true,true);
   auto z=rt.cycle(q,1.,true);check(z.nmpc_selected,"verified measurements rejected before state reconstruction");
   q=request(2.);q.state.speed=q.state.steering=rc::missing();
   z=rt.cycle(q,2.,true);check(!z.nmpc_selected,"stale measurements resolved unknown state");
 }
 check(!runtime.requestMode("bogus")&&!runtime.requestTerrain("mud",1.),"invalid selection accepted");
 check(runtime.requestMode("nmpc_fixed"),"fixed mode");
 out=runtime.cycle(request(1.1),1.1,true);check(out.nmpc_selected&&out.solve.feasible,"fixed solver selected");
 r=request(1.2);r.force_timeout=true;out=runtime.cycle(r,1.2,true);
 check(!out.nmpc_selected&&out.command.speed<=.1&&out.solve.timed_out,"timeout fallback");
 out=runtime.cycle(request(1.3),1.3,false);check(out.command.speed==0.&&out.owner=="safety_stop","reference loss stop");
 out=runtime.cycle(request(1.4),1.4,true,true);check(out.command.speed==0.,"hard stop");
 runtime.requestTerrain("smooth",2.);out=runtime.cycle(request(2.),2.,true);
 check(out.terrain=="unknown","manual scene bypassed dwell");
 for(int i=1;i<121;++i){double now=2.+i/30.;out=runtime.cycle(request(now),now,true);}
 check(out.terrain=="smooth"&&out.effective.limits.speed>.1&&out.effective.limits.speed<.30,"scene ramp");
 runtime.requestTerrain("unknown",6.1);out=runtime.cycle(request(6.1),6.1,true);
 check(out.effective.limits.speed<=.1,"unknown restriction delayed");
 check(!runtime.cycle(request(5.),5.,true).nmpc_selected,"clock reset used old solution");
 { // Ordinary failures must not introduce an emergency speed discontinuity.
   rc::AdaptiveRuntime rt;check(rt.configure(c),"fallback test config");rt.requestMode("nmpc_fixed");
   rc::Command previous{.24,0.};double last_a=0.;
   for(int i=0;i<100;++i){double now=8.+i/30.;auto q=request(now);q.previous=previous;
     q.previous_command_acceleration=last_a;q.baseline={.24,.2};q.force_timeout=true;
     auto z=rt.cycle(q,now,true);const double acceleration=(z.command.speed-previous.speed)*30.;
     check(acceleration<=.28+1e-8&&acceleration>=-.36-1e-8,"fallback acceleration");
     check(std::fabs(acceleration-last_a)<=.8/30.+1e-8,"fallback jerk");
     check(std::fabs(z.command.steering-previous.steering)<=1./30.+1e-8,"fallback steering rate");
     previous=z.command;last_a=acceleration;
   }
   check(previous.speed<=.1001,"fallback cap not reached");
 }
 { // Invalid inputs cannot leak NaN from either the baseline or warm history.
   rc::AdaptiveRuntime rt;check(rt.configure(c),"invalid input config");
   for(const std::string mode:{"legacy_slqr","nmpc_fixed","auto_fallback"}) {
     rt.requestMode(mode);auto q=request(20.);q.state.yaw=rc::missing();q.baseline.speed=rc::missing();
     auto z=rt.cycle(q,20.,true);
     check(std::isfinite(z.command.speed)&&std::isfinite(z.command.steering)&&!z.nmpc_selected,"NaN state/baseline escaped");
     q.previous.steering=rc::missing();z=rt.cycle(q,20.1,true);
     check(z.owner=="safety_stop"&&z.command.speed==0.&&z.command.steering==0.,"NaN history escaped");
   }
 }
 for(double bad:{.51,1.}){auto invalid=c;invalid.solver.limits.speed=bad;check(!runtime.configure(invalid),"safety loosen accepted");}
 auto invalid=c;invalid.profiles[3].speed_cap=.30;check(!runtime.configure(invalid),"aggressive unknown accepted");
 invalid=c;invalid.feature_minimum_confidence=.2;check(!runtime.configure(invalid),"feature threshold accepted");
 double fixed_cost=0.,adaptive_cost=0.;
 for(const std::string mode:{"nmpc_fixed","nmpc_model_adaptive","nmpc_weight_adaptive","nmpc_full_adaptive","auto_fallback"}) {
   rc::AdaptiveRuntime rt;check(rt.configure(c),"reset config");rt.requestMode(mode);
   for(int i=1;i<=180;++i){const double now=10.+i/30.;auto& obs=rt.observer();
     obs.updateWheel(80.,now,now);obs.updateSteering(0.,now,now);
     obs.updateGround(.08,.005,0.,now,now);obs.updateImu(0.,0.,0.,9.80665+2.*std::sin(i*1.7),0.,0.,now,now,true,true,true);
     out=rt.cycle(request(now),now,true);obs.recordCommand(out.command,now);
   }
   const bool weight=mode=="nmpc_weight_adaptive"||mode=="nmpc_full_adaptive"||mode=="auto_fallback";
   const bool model=mode=="nmpc_model_adaptive"||mode=="nmpc_full_adaptive"||mode=="auto_fallback";
   check((out.effective.weights.smoothness>c.solver.weights.smoothness+1e-5)==weight,"weight switch inactive or leaking");
   // Lateral residual observable only above minimum identification speed; no
   // fabricated yaw gain at this deliberately unexcited low-speed fixture.
   check(out.effective.model.yaw_gain==1.,"unobservable yaw adapted");
   check(out.effective.weights.lateral==c.solver.weights.lateral&&
     out.effective.weights.heading==c.solver.weights.heading,"vibration changed feedback weights");
   check(out.effective.weights.speed==c.solver.weights.speed&&
     out.effective.limits.lateral_acceleration==c.solver.limits.lateral_acceleration,
     "vibration alone changed dynamics safety envelope");
   for(const auto& e:out.adaptation)check(!e.active,"unobservable low-speed evidence accepted");
   (void)model;
   if(mode=="nmpc_fixed")fixed_cost=out.solve.objective;
   if(mode=="nmpc_weight_adaptive")adaptive_cost=out.solve.objective;
 }
 check(std::isfinite(fixed_cost)&&std::isfinite(adaptive_cost),"mode solver failed");
 { // An excited independent yaw sequence is evaluated with the previous
   // candidate, never the candidate trained on the same sample. No road label
   // appears in the gate. Small nominal errors must not unlock compensation.
   rc::AdaptiveRuntime rt;auto cfg=c;cfg.observer.identify_steering=false;
   check(rt.configure(cfg),"held-out test config");rt.requestTerrain("smooth",1.);
   for(int i=0;i<600;++i)rt.cycle(request(1.+i/30.),1.+i/30.,true);
   rt.requestMode("nmpc_model_adaptive");
   for(int i=0;i<600;++i) {
     const double now=30.+i/30.,d=.22*std::sin(.95*i/30.),v=.24;
     auto& obs=rt.observer();obs.updateWheel(v*1000.,now,now);obs.updateSteering(d,now,now);
     obs.updateGround(v,0.,0.,now,now);obs.updateImu(.75*v*std::tan(d)/.6,0.,0.,9.80665,0.,0.,now,now,true,true,true);
     auto q=request(now);q.state.speed=q.previous.speed=q.baseline.speed=v;
     q.state.steering=q.previous.steering=q.baseline.steering=d;q.speed_cap=.30;q.history={{now-1.,q.previous}};
     for(auto& point:q.reference.points){point.speed=v;point.x=point.s=v*point.t;}
     out=rt.cycle(q,now,true);obs.recordCommand(out.command,now);
   }
   check(out.effective.model.yaw_gain<.995,"credible yaw evidence never changed model");
   const double gain_before=out.effective.model.yaw_gain;
   // Mode changes preserve model continuity and then ramp to nominal. Sensor
   // loss immediately prevents new adaptation, without trusting stale evidence.
   rt.requestMode("nmpc_fixed");auto q=request(50.);q.force_timeout=true;
   out=rt.cycle(q,50.,true);
   check(std::fabs(out.effective.model.yaw_gain-gain_before)<=.05/30.+1e-6,"mode reset jumped model");
   check(!out.model_adaptation_active,"fixed mode adapts model");
   out=rt.cycle(request(51.),51.,true);
   for(const auto& e:out.adaptation)check(!e.active,"sensor loss retained adaptation evidence");
 }
 // P36 backend integration uses the same supervisor and fails closed.
 for(const std::string backend:{"original12","blocked8","flat6"}) {
   rc::AdaptiveRuntime rt;auto cfg=calibrated();cfg.research_backend=backend;
   check(rt.configure(cfg),"backend configure");rt.requestMode("nmpc_fixed");
   auto q=request(1.);auto z=rt.cycle(q,1.,true);
   check(z.solve.decision_dimension==(backend=="flat6"?6:backend=="blocked8"?8:12),"wrong real decision dimension");
   check(z.nmpc_selected,"straight backend feasible");
   q=request(1.1);q.force_timeout=true;z=rt.cycle(q,1.1,true);
   check(!z.nmpc_selected&&z.solve.timed_out,"backend timeout retained ownership");
   z=rt.cycle(request(1.2),1.2,true,true);check(z.command.speed==0.&&z.owner=="safety_stop","backend delayed emergency stop");
 }
 {auto cfg=calibrated();cfg.research_backend="invalid";check(!rc::AdaptiveRuntime::validate(cfg),"invalid backend accepted");}
 // Independently disabled adaptation stays nominal even in full mode.
 c.model_adaptation_enabled=c.weight_adaptation_enabled=false;check(runtime.configure(c),"switch config");
 runtime.requestMode("nmpc_full_adaptive");out=runtime.cycle(request(20.),20.,true);
 check(out.effective.weights.lateral==c.solver.weights.lateral&&out.effective.model.yaw_gain==1.,"ablation toggles");
 std::cout<<"PASS runtime modes, sensor gates, scene ramp/dwell, immutable bounds and fault supervisor\n";
 return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
