#include "control/p6_controller.h"
#include <cassert>
#include <iostream>
using namespace research_control;
int main(){
 assert(p6UnchangedFeedbackObservation(9.98,9.99,9.985,9.98,9.99,9.985,10.,10.02));
 assert(!p6UnchangedFeedbackObservation(9.98,9.99,9.985,9.99,9.99,9.985,10.,10.02));
 assert(!p6UnchangedFeedbackObservation(9.98,9.99,9.985,9.98,9.99,9.985,10.2,10.22));
 assert(!p6UnchangedFeedbackObservation(9.98,9.99,9.985,9.98,9.99,9.985,10.04,10.02));
 assert(!p6UnchangedFeedbackObservation(9.98,9.99,9.985,9.98,9.99,9.991,10.,10.02));
 Reference bend;bend.valid=true;bend.count=3;
 bend.points[0].curvature=.30;bend.points[1].t=.6;bend.points[1].curvature=.3;
 bend.points[2].t=2.;bend.points[2].curvature=-.3;
 assert(P6Controller::turnAwareRateScale(bend,Model{})==1.);
 bend.points[1].curvature=-.06;
 assert(std::abs(P6Controller::turnAwareRateScale(bend,Model{})-.5)<1e-9);
 bend.points[1].curvature=-.30;
 assert(P6Controller::turnAwareRateScale(bend,Model{})==0.);
 P6Controller controller;P6NominalProfile nominal;nominal.scene="smooth";nominal.speed_cap=.3;nominal.config_hash=std::string(64,'a');
 p6::SensorSnapshot observed;observed.now=10.;observed.run_id="p5_p6_unit";observed.session_id="synthetic";observed.synthetic=observed.history_complete=true;observed.cycle=1;
 std::vector<p6::PublishedSample> published;for(int k=0;k<40;++k){p6::PublishedSample p;p.sequence=k+1;p.stamp=8.+k*.04;p.command.speed=.24;p.published=p.committed=true;published.push_back(p);}
 auto effective=controller.parameters(nominal,observed,published,p6::ResponsePolicy{},"nmpc_12_fixed");assert(effective.valid);
 const auto hash=effective.snapshot_hash;nominal.solver.model.yaw_gain=.7;assert(effective.solver.model.yaw_gain==1.&&effective.snapshot_hash==hash);
 SolveRequest r;r.now=10.;r.state.speed=.24;r.previous.speed=.24;r.baseline=r.previous;r.speed_cap=.3;
 for(const auto&p:published)r.history.push_back({p.stamp,p.command});r.reference.valid=true;r.reference.count=21;
 for(int i=0;i<21;++i){auto&p=r.reference.points[i];p.t=.1*i;p.x=.3+.24*p.t;p.s=.24*p.t;p.speed=.24;}
 auto a=controller.solve(r,effective,"nmpc_12_fixed");assert(a.solve.ok&&a.solve.decision_dimension==12);
 auto b=controller.solve(r,effective,"blocked8");assert(b.solve.ok&&b.solve.decision_dimension==8);
 auto f=controller.solve(r,effective,"flat_nmpc_8");assert(f.solve.ok&&f.solve.decision_dimension==8);
 // A same-context nominal path needs no second zero-error rollout, but an
 // explicitly supplied state/history must still take the validation branch.
 controller.setResponseBase(true);controller.setDelayAlignedReconstruction(true);
 controller.setConditionedGeometry(true);controller.setBaselineSeed(true);
 auto generous=effective;generous.solver.budget_ms=1000.;generous.snapshot_hash=p6SnapshotIdentity(generous);
 const auto reuse=controller.solve(r,generous,"flat_nmpc_slqr");
 assert(reuse.solve.ok&&reuse.feedback.valid&&reuse.nominal_validation_reused);
 assert(reuse.feedback_ms==0.&&!reuse.feedback_nonzero&&reuse.policy_build_ms>0.);
 const auto explicit_state=controller.solve(r,generous,"flat_nmpc_slqr",&r.state,r.now,&r.history);
 assert(explicit_state.solve.ok&&!explicit_state.nominal_validation_reused&&explicit_state.feedback_ms>0.);
 auto modified=effective;modified.solver.model.yaw_gain=.6;
 assert(controller.solve(r,modified,"nmpc_12_fixed").solve.reason=="invalid_or_mutated_parameter_snapshot");
 const auto expired=controller.solve(r,effective,"nmpc_12_fixed",nullptr,0.,nullptr,0.);
 assert(!expired.solve.ok&&expired.solve.timed_out);
 P6LayeredConfig config;for(auto&p:config.profiles)p.config_hash=nominal.config_hash;
 config.profiles[0].speed_cap=.3;config.profiles[0].solver.model.steering_tau=.44;
 config.profiles[1].speed_cap=.18;config.profiles[1].solver.model.steering_tau=.7;
 config.profiles[3].speed_cap=.1;
 P6SceneManager manager;manager.enabled=true;auto scene=manager.update(config,"smooth",0.);
 scene=manager.update(config,"grass_bumpy",.1);assert(scene.scene=="smooth"&&scene.speed_cap<=.18);
 for(int i=1;i<80;++i)scene=manager.update(config,"grass_bumpy",.1+i*.05);
 assert(scene.scene=="grass_bumpy"&&scene.solver.model.steering_tau>.44&&scene.solver.model.steering_tau<.7);
 scene=manager.update(config,"unknown",4.2);assert(scene.scene=="unknown"&&scene.speed_cap<=.1);
 {Command previous{.24,.1};double previous_a=0.;Limits limits;
  for(int k=0;k<180;++k){const auto u=p6ConservativeSpeedFallback({.24,.1},previous,previous_a,1./30.,.1,limits);
   const double a=(u.speed-previous.speed)*30.;assert(a>=-limits.deceleration-1e-9&&a<=limits.acceleration+1e-9);assert(std::abs(a-previous_a)<=limits.jerk/30.+1e-9);
   assert(u.steering==.1&&u.speed<=limits.speed);previous=u;previous_a=a;}
  assert(previous.speed<=.1+1e-9);
 }
 std::cout<<"PASS P6 actual backend dimensions, immutable snapshot, budget, scene dwell/ramp/unknown cap\n";
}
