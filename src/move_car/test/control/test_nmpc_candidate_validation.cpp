#include "control/nmpc_candidate_validation.h"
#include "control/reduced_nmpc.h"
#include <cassert>
#include <iostream>

using namespace research_control;
struct Fixture {
 SolveRequest r;SolverConfig c;SolveResult s;CandidateProvenance p;CandidateAdmission a;
 Fixture(){
   r.now=10.;r.control_dt=1./30.;r.speed_cap=.3;r.previous={.2,0.};r.baseline=r.previous;
   r.state.speed=.2;r.reference.valid=true;r.reference.stamp=9.9;r.reference.count=21;
   for(int i=0;i<=20;++i){auto& q=r.reference.points[i];q.t=.1*i;q.s=.02*i;q.x=.3+q.s;q.speed=.2;}
   for(int i=0;i<=20;++i)r.history.push_back({9.4+i*.03,{.2,0.}});
   s.ok=s.feasible=true;s.count=c.steps+1;s.solve_ms=.5;s.objective=1.;s.max_violation=0.;
   s.command=r.previous;s.command_acceleration=0.;
   for(int i=0;i<c.steps;++i)s.controls[i]=r.previous;
   for(int i=0;i<s.count;++i)s.predicted[i]=r.state;
   p.cycle=20;p.reference_revision=7;p.configuration_revision=3;p.snapshot_ros=r.now;
   p.snapshot_steady=30.;p.started_steady=30.001;p.finished_steady=30.002;
   a.cycle=20;a.reference_revision=7;a.configuration_revision=3;a.now_ros=r.now;
   a.now_steady=30.003;a.actual_previous=r.previous;
   a.local_frame_verified=a.measured_state_verified=a.history_verified=true;
   a.solver_substep_constraints_checked=true;
 }
 CandidateValidation validate(){return NMPCandidateValidator::validate(r,c,s,p,a);}
 void enable(){a.authorized=true;}
};
int main(){
 int cases=0;
 {Fixture f;assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();assert(f.validate().accepted);++cases;}
 {Fixture f;f.enable();f.a.safety_stop=true;assert(std::string(f.validate().reason)=="existing_safety_owner");++cases;}
 {Fixture f;f.enable();f.a.reference_braking=true;assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();f.a.local_frame_verified=false;assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();f.a.measured_state_verified=false;assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();f.a.history_verified=false;assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();f.a.solver_substep_constraints_checked=false;assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();f.c.limits.speed=.6;assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();f.p.snapshot_steady=missing();assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();f.p.finished_steady=f.a.now_steady+.001;assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();f.a.now_steady=30.1;assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();++f.p.cycle;assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();++f.p.reference_revision;assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();++f.p.configuration_revision;assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();f.p.snapshot_ros=9.;assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();f.a.now_ros=9.;assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();f.r.reference.stamp=10.1;assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();f.r.reference.count=kMaxHorizon+2;assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();f.r.reference.points[3].curvature=missing();assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();f.r.reference.points[3].t=f.r.reference.points[2].t;assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();f.r.reference.count=3;assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();f.r.previous.speed=.21;assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();f.r.history.clear();assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();f.r.history[3].stamp=f.r.history[2].stamp;assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();f.r.history[3].stamp=10.1;assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();f.s.timed_out=true;assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();f.s.solve_ms=8.;assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();f.p.finished_steady=30.010;f.a.now_steady=30.011;assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();f.s.max_violation=missing();assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();f.s.feasible=false;assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();f.s.count=kMaxHorizon+2;assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();f.s.command.speed=.21;assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();f.s.predicted[0].speed=.21;assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();f.s.controls[1].speed=.6;assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();f.s.controls[1].steering=.11;assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();f.s.controls[1].speed=.22;assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();f.s.command_acceleration=.01;assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();f.s.predicted[3].yaw=missing();assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();f.s.predicted[3].speed=.31;assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();f.c.model.yaw_gain=missing();assert(!f.validate().accepted);++cases;}
 {Fixture f;f.enable();f.c.model.speed_tau=missing();assert(!f.validate().accepted);++cases;}
 // Actual ReducedNMPC solution under the identical contract. This is a
 // synthetic unit scenario, not hardware timing or real control authority.
 {Fixture f;f.enable();f.c.budget_ms=100.;f.a.maximum_solve_ms=100.;
  ReducedNMPC solver;f.s=solver.solve(f.r,f.c);assert(f.s.ok);
  f.p.finished_steady=f.p.started_steady+f.s.solve_ms/1000.;
  f.a.now_steady=f.p.finished_steady+.001;assert(f.validate().accepted);++cases;}
 std::cout<<"PASS "<<cases<<" candidate validation cases; no publisher or actual owner integration\n";
}
