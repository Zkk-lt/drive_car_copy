#include "control/p7_speed_policy.h"
#include "control/reduced_nmpc.h"
#include <cassert>
#include <iostream>
using namespace research_control;
P7SpeedPolicy policy(bool on=true,const std::string&w="base",double follow=.4,double curve=.4,double cte=.4,double actuator=.4){return P7SpeedPolicy::make(on,.25,.4,.4,follow,.4,curve,cte,actuator,false,.4,w,false,true,false,false);}
SolveRequest request(double cap){SolveRequest r;r.now=100.;r.control_dt=1./30.;r.state={-.3,0.,0.,.30,0.,0.};r.speed_cap=cap;r.target_speed=.25;r.previous={.30,0.};r.baseline={.25,0.};r.reference.valid=true;r.reference.count=21;
 for(int i=0;i<21;++i){auto&p=r.reference.points[i];p.t=i*.1;p.x=.25*p.t;p.s=p.x;p.speed=.25;}
 for(int i=30;i>0;--i)r.history.push_back({100.-i/30.,r.previous});return r;}
int main(){
 const auto preview=P7SpeedPolicy::make(true,.1,.4,.4,.33,.28,.39,.38,.37,true,.30,
       "slqr_sequence_fallback",false,true,false,false,true);
 assert(preview.separated&&preview.target==.1&&preview.hard_cap==.28);
 const auto veto=P7SpeedPolicy::make(true,.1,.4,.4,.33,.28,.39,.38,.37,true,.30,
       "slqr_sequence_fallback",false,true,false,true,true);
 assert(!veto.separated&&veto.hard_cap==.1);
 auto p=policy();assert(p.separated&&p.target==.25&&p.hard_cap==.4);
 assert(!policy(false).separated&&policy(false).hard_cap==.25);
 assert(policy(true,"base",.2).hard_cap==.2);assert(policy(true,"curve",.4,.21).hard_cap==.21);
 assert(policy(true,"cte",.4,.4,.19).hard_cap==.19);assert(policy(true,"actuator",.4,.4,.4,.17).hard_cap==.17);
 assert(!policy(true,"bad_geometry").separated&&!policy(true,"follow").separated);
 assert(!P7SpeedPolicy::make(true,.25,.4,.4,.4,.4,.4,.4,.4,false,.4,"base",true,true,false,false).separated);
 assert(!P7SpeedPolicy::make(true,.25,.4,.4,.4,.4,.4,.4,.4,false,.4,"base",false,false,false,false).separated);
 assert(P7SpeedPolicy::make(true,.25,.4,.4,.4,.4,.4,.4,.4,true,.2,"base",false,true,false,false).hard_cap==.196);
 SolverConfig c;c.budget_ms=8.;c.iterations=2;ReducedNMPC old_solver,new_solver;
 auto old=old_solver.solve(request(.25),c),now=new_solver.solve(request(p.hard_cap),c);
 assert(!old.ok&&now.ok&&now.feasible&&now.max_violation<=1e-7&&now.command.speed<.30&&now.command.speed>=0.);
 auto risk=request(policy(true,"base",.2).hard_cap);ReducedNMPC risk_solver;assert(!risk_solver.solve(risk,c).ok);
 auto physical=request(.4);physical.state.speed=.41;ReducedNMPC physical_solver;assert(!physical_solver.solve(physical,c).ok);
 std::cout<<"PASS base-target .25/current .30/hard .40: old rejected, new feasible deceleration="<<now.command.speed<<"; all curve/cte/actuator/follow/horizon/geometry/physical caps retained; protected stops unchanged\n";
}
