#include "control/p7_reference_revalidation.h"
#include "control/p7_publication_guard.h"
#include "control/unified_control.h"
#include <cassert>
#include <iostream>
using namespace research_control;
int main(){
 ReferencePoint p;p.x=1.;p.y=.2;p.yaw=.1;
 const auto translated=P7ReferenceRevalidation::transportReference(p,{{0.,0.,0.}},{{.01,0.,0.}});
 assert(std::abs(translated.x-.99)<1e-12&&std::abs(translated.y-.2)<1e-12);
 const auto rotated=P7ReferenceRevalidation::transportReference(p,{{0.,0.,0.}},{{0.,0.,1.5707963267948966}});
 assert(std::abs(rotated.x+.1)<1e-12&&std::abs(rotated.y+1.3)<1e-12);
 SolverConfig config;SolveRequest r;r.now=10.;r.speed_cap=.3;r.previous={.24,0.};r.baseline=r.previous;
 r.state={-.30,0.,0.,.24,0.,0.};r.history.push_back({9.,r.previous});
 std::vector<ReferencePoint>old_path,new_path;
 for(int i=0;i<=60;++i){ReferencePoint p;p.s=p.x=.05*i;old_path.push_back(p);p.y=.001;new_path.push_back(p);}
 std::array<Command,kMaxHorizon>commands{};for(auto&c:commands)c=r.previous;
 auto deadline=[](){return std::chrono::steady_clock::now()+std::chrono::milliseconds(100);};
 const auto ok=P7ReferenceRevalidation::validate(r,config,old_path,new_path,commands,deadline());
 assert(ok.valid&&ok.rollout.count==config.steps+1&&ok.rollout.max_violation<=1e-7);
 assert(ok.rollout.command.speed==r.previous.speed&&ok.position_change>.0009);
 P7ReferenceRevalidation::Workspace workspace;
 const auto buffered=P7ReferenceRevalidation::validate(r,config,old_path,new_path,commands,deadline(),
     false,false,false,0.,false,false,false,&workspace);
 assert(buffered.valid&&buffered.rollout.objective==ok.rollout.objective);
 assert(buffered.rollout.command.steering==ok.rollout.command.steering);
 auto invalid_path=new_path;invalid_path[1].s=invalid_path[0].s;
 assert(!P7ReferenceRevalidation::validate(r,config,old_path,invalid_path,commands,deadline(),
     false,false,false,0.,false,false,false,&workspace).valid);
 assert(P7ReferenceRevalidation::validate(r,config,old_path,new_path,commands,deadline(),
     false,false,false,0.,false,false,false,&workspace).valid);
 std::vector<ReferencePoint> suffix;
 auto ahead=r.state;ahead.x=.025-config.model.rear_to_center;
 assert(FlatnessReferenceModel::projectSpatialSuffix(old_path,ahead,config.model,suffix));
 assert(std::abs(suffix.front().s-.025)<1e-12&&std::abs(suffix.front().x-.025)<1e-12);
 ahead.x=missing();
 assert(!FlatnessReferenceModel::projectSpatialSuffix(old_path,ahead,config.model,suffix));
 // A queued steering step takes effect inside the first interval. Even
 // though its boundary position is unchanged, its new derivative is unsafe.
 auto event_config=config;event_config.model.steering_tau=.10;
 auto event_request=r;event_request.previous.steering=.20;
 event_request.history.push_back({r.now-event_config.model.steering_delay+.014,event_request.previous});
 auto event_commands=commands;
 for(auto& cmd:event_commands)cmd.steering=event_request.previous.steering;
 const auto event=P7ReferenceRevalidation::validate(event_request,event_config,old_path,new_path,event_commands,deadline());
 assert(!event.valid&&event.reason=="delayed_physical_state_constraint_violation");
 auto jump=new_path;for(auto&p:jump)p.y=.031;
 assert(P7ReferenceRevalidation::validate(r,config,old_path,jump,commands,deadline()).reason=="revalidation_reference_jump_outside_local_domain");
 auto brake=r;brake.reference.braking=true;
 assert(!P7ReferenceRevalidation::validate(brake,config,old_path,new_path,commands,deadline()).valid);
 auto lower=r;lower.speed_cap=.2;
 assert(!P7ReferenceRevalidation::validate(lower,config,old_path,new_path,commands,deadline()).valid);
 auto jerk=commands;jerk[0].speed+=.01;
 const auto rejected=P7ReferenceRevalidation::validate(r,config,old_path,new_path,jerk,deadline());
 assert(!rejected.valid&&rejected.reason=="reconstructed_input_constraint_violation");
 auto slower=r;slower.speed_cap=.26;
 auto crossing=commands;
 for(int i=1;i<config.steps;++i)crossing[i].speed+=std::min(.04,.004*i);
 assert(P7ReferenceRevalidation::validate(r,config,old_path,new_path,crossing,deadline()).valid);
 assert(!P7ReferenceRevalidation::validate(slower,config,old_path,new_path,crossing,deadline()).valid);
 const auto fitted=P7ReferenceRevalidation::validate(slower,config,old_path,new_path,crossing,deadline(),
     false,false,false,slower.control_dt,false,false,false,&workspace,true);
 assert(fitted.valid&&fitted.reason=="current_reference_tightened_speed_full_rollout_feasible");
 assert(fitted.rollout.max_violation<=1e-7);
 for(int i=0;i<config.steps;++i){
   assert(fitted.rollout.controls[i].speed<=slower.speed_cap+1e-10);
   assert(fitted.rollout.controls[i].steering==crossing[i].steering);
 }
 slower.state.speed=.27;
 assert(!P7ReferenceRevalidation::validate(slower,config,old_path,new_path,crossing,deadline(),
     false,false,false,slower.control_dt,false,false,false,&workspace,true).valid);
 // The geometry anchors are the same, although absolute station origins
 // and the sample locations along each linear segment are different.
 auto line=[](double origin,const std::vector<double>& stations){
   std::vector<ReferencePoint> path;
   for(double s:stations){
     ReferencePoint point;
     point.s=origin+s;point.x=s;
     path.push_back(point);
   }
   return path;
 };
 const auto sparse=line(2.,{0.,.20,.40,.60,.80});
 auto dense=line(5.,{0.,.10,.35,.55,.80});
 P7ReferenceRevalidation::Result same_arc;
 assert(P7ReferenceRevalidation::compareSpatial(sparse,dense,.70,same_arc));
 assert(same_arc.position_change<1e-12&&same_arc.heading_change<1e-12);
 dense[2].y=.040;dense[2].yaw=.11;
 P7ReferenceRevalidation::Result deviation;
 assert(P7ReferenceRevalidation::compareSpatial(sparse,dense,.70,deviation));
 assert(std::abs(deviation.position_change-.040)<1e-12);
 assert(std::abs(deviation.heading_change-.11)<1e-12);
 P7ReferenceRevalidation::Result unsupported;
 assert(!P7ReferenceRevalidation::compareSpatial(sparse,dense,.81,unsupported));

 // Separate the time-law change from the spatial comparison. A preview
 // curvature restriction reduces travel on the same centerline, while
 // every accepted command still passes the current limits and delayed roll.
 auto slower_preview=old_path;
 slower_preview[8].curvature=1.;
 const auto timed=P7ReferenceRevalidation::validate(r,config,old_path,slower_preview,commands,deadline(),
     true,false,false,.025);
 assert(!timed.valid&&timed.reason=="revalidation_reference_jump_outside_local_domain");
 const auto spatial=P7ReferenceRevalidation::validate(r,config,old_path,slower_preview,commands,deadline(),
     true,false,false,.025,false,false,true);
 assert(spatial.valid&&spatial.rollout.max_violation<=1e-7);
 const auto spatial_jerk=P7ReferenceRevalidation::validate(r,config,old_path,slower_preview,jerk,deadline(),
     true,false,false,.025,false,false,true);
 assert(!spatial_jerk.valid&&spatial_jerk.reason=="reconstructed_input_constraint_violation");
 const auto spatial_jump=P7ReferenceRevalidation::validate(r,config,old_path,jump,commands,deadline(),
     true,false,false,.025,false,false,true);
 assert(!spatial_jump.valid&&spatial_jump.reason=="revalidation_reference_jump_outside_local_domain");
 auto early=r;early.control_dt=.022;
 auto planned=commands;for(auto& c:planned)c.speed+=.0005;
 assert(!P7ReferenceRevalidation::validate(early,config,old_path,new_path,planned,deadline()).valid);
 const auto retimed=P7ReferenceRevalidation::validate(early,config,old_path,new_path,planned,deadline(),
     false,false,false,.025);
 assert(retimed.valid&&retimed.reason=="current_reference_retimed_full_rollout_feasible");
 assert(std::abs(retimed.rollout.command_acceleration)<=config.limits.jerk*early.control_dt+1e-8);
 assert(!P7ReferenceRevalidation::validate(early,config,old_path,new_path,jerk,deadline(),
     false,false,false,.025).valid);
 auto increments=commands;
 increments[0].speed=r.previous.speed+.025*.015;
 increments[1].speed=increments[0].speed+config.dt*(.015-config.limits.jerk*config.dt);
 for(int i=2;i<config.steps;++i)increments[i].speed=increments[1].speed;
 assert(!P7ReferenceRevalidation::validate(early,config,old_path,new_path,increments,deadline(),
     false,false,false,.025).valid);
 const auto preserved=P7ReferenceRevalidation::validate(early,config,old_path,new_path,increments,deadline(),
     false,false,false,.025,false,true);
 assert(preserved.valid);
 assert(std::abs((preserved.rollout.controls[1].speed-preserved.rollout.controls[0].speed)-
                 (increments[1].speed-increments[0].speed))<1e-12);
 unified_control::P7PublicationGuard publication_guard;
 std::uint64_t steady=1000000000;
 for(int i=0;i<12;++i,steady+=25000000)
   assert(publication_guard.allow(steady,steady-25000000,true)==(i==11));
 // The field trace at sequence 1139 had one 33.559472 ms gap followed
 // by a normal release. It must not use the late optimized command.
 using Release=unified_control::P7PublicationGuard::Release;
 unified_control::ControlRequest field;
 field.dt=.02500002;
 field.history_valid=field.state.valid=field.state.causal=field.reference.valid=true;
 field.previous={.3902586961,-.0299061225};
 field.previous_acceleration=.0292924028;
 field.state.speed=.386;
 field.speed_cap=.392;
 field.limits={.4,.32,1.,.28,.36,.8,.35};
 unified_control::Command recovery_command;
 const double field_gap=.033559472;
 assert(publication_guard.brakingCommand(field,field_gap,2.292,.1,.3,recovery_command));
 assert(recovery_command.speed>.389&&recovery_command.speed<.392&&recovery_command.steering==field.previous.steering);
 for(double dt:{field.dt,field_gap}) {
   const double a=(recovery_command.speed-field.previous.speed)/dt;
   assert(std::abs(a-field.previous_acceleration)<=field.limits.jerk*dt+1e-9);
 }
 const auto first_late=steady+33559472;
 assert(publication_guard.assess(first_late,steady,false,true)==Release::BoundedBrake);
 assert(publication_guard.assess(first_late+15670918,first_late,false,true)==Release::OnTime);
 steady=first_late+15670918;
 assert(publication_guard.assess(steady+33559472,steady,false,true)==Release::BoundedBrake);
 steady+=33559472;
 assert(publication_guard.assess(steady+33559472,steady,false,true)==Release::Stop);
 field.reference.braking=true;
 assert(!publication_guard.brakingCommand(field,field_gap,2.292,.1,.3,recovery_command));
 field.reference.braking=false;
 assert(!publication_guard.brakingCommand(field,field_gap,.25,.1,.3,recovery_command));
 field.following_stop=true;
 assert(!publication_guard.brakingCommand(field,field_gap,2.292,.1,.3,recovery_command));
 steady+=85000000;
 assert(!publication_guard.allow(steady,steady-60000000,false));
 for(int i=0;i<16;++i,steady+=25000000)
   assert(!publication_guard.allow(steady,steady-25000000,false));
 for(int i=0;i<12;++i,steady+=25000000)
   assert(publication_guard.allow(steady,steady-25000000,true)==(i==11));
 auto future=r;future.history.push_back({r.now+.001,r.previous});
 assert(!P7ReferenceRevalidation::validate(future,config,old_path,new_path,commands,deadline()).valid);
 const auto timeout=P7ReferenceRevalidation::validate(r,config,old_path,new_path,commands,std::chrono::steady_clock::now());
 assert(!timeout.valid&&timeout.timed_out);
 unified_control::ControlRequest request;request.mode=unified_control::ControlMode::NmpcFixed;
 request.now=10.;request.state={};auto&s=request.state;s.stamp=s.receipt_stamp=10.;s.speed=.24;s.confidence=1.;
 s.valid=s.causal=s.nominal_model_valid=true;request.reference={9.99,2,true,false};
 request.previous=request.legacy=r.previous;request.epoch=request.config_version=1;request.history_sequence=17;request.history_valid=true;
 request.guard={true,true,"/control_research/p5_test/cmd","p5_revalidation"};
 unified_control::ControlCandidate candidate;candidate.command=r.previous;candidate.sequence=4;candidate.epoch=candidate.config_version=1;
 candidate.history_base_sequence=17;candidate.reference_sequence=1;candidate.snapshot_stamp=9.97;candidate.created_stamp=9.98;
 candidate.expected_apply_stamp=10.;candidate.solve_ms=1.;candidate.max_violation=0.;candidate.propagated_speed=.24;
 candidate.valid=candidate.feasible=candidate.complete_substep_constraints=candidate.state_pair_validated=true;
 assert(!unified_control::ControlArbiter::decide(request,&candidate).selected_nmpc);
 candidate.reference_revalidated=true;candidate.validated_reference_sequence=2;candidate.validation_stamp=10.;candidate.revalidation_ms=ok.elapsed_ms;
 assert(unified_control::ControlArbiter::decide(request,&candidate).selected_nmpc);
 candidate.validation_stamp=9.999;
 assert(!unified_control::ControlArbiter::decide(request,&candidate).selected_nmpc);
 candidate.validation_stamp=10.;candidate.history_base_sequence=16;
 assert(!unified_control::ControlArbiter::decide(request,&candidate).selected_nmpc);
 candidate.history_base_sequence=17;candidate.solve_ms=8.;
 assert(!unified_control::ControlArbiter::decide(request,&candidate).selected_nmpc);
 std::cout<<"PASS current-reference nonlinear revalidation, jumps/brake/caps/jerk/future/history/deadline rejection; original optimized revision preserved; elapsed_ms="<<ok.elapsed_ms<<"\n";
}
