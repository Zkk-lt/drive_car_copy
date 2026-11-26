#include "control/flatness_reduced_nmpc.h"
#include <cassert>
#include <iostream>
using namespace research_control;
int main(){
 State s;s.speed=.24;s.yaw=.4;s.steering=.1;Model m;
 FlatnessReducedNMPC::Coefficients z{{.1,-.2,.3,.2,-.3,.1}};
 const double t=.6,h=1e-5,H=2.;
 const auto a=FlatnessReducedNMPC::jet(s,m,t-h,H,z),b=FlatnessReducedNMPC::jet(s,m,t,H,z),c=FlatnessReducedNMPC::jet(s,m,t+h,H,z);
 assert(std::abs((c.x-a.x)/(2*h)-b.dx)<1e-8);assert(std::abs((c.dy-a.dy)/(2*h)-b.ddy)<1e-8);
 assert(std::abs((c.ddx-a.ddx)/(2*h)-b.dddx)<1e-8);
 const auto initial=FlatnessReducedNMPC::jet(s,m,0.,H,z);auto flat=FlatnessReferenceModel::reconstruct(initial,m);
 assert(flat.valid&&std::abs(flat.state.steering-s.steering)<1e-12);
 SolverConfig cfg;cfg.budget_ms=1000.;SolveRequest r;r.now=10.;r.state.speed=.24;r.state.x=-.3;r.previous.speed=.24;r.baseline=r.previous;
 r.history.push_back({9.,r.previous});r.reference.valid=true;r.reference.count=21;r.speed_cap=.3;
 for(int i=0;i<=20;++i){auto& p=r.reference.points[i];p.t=.1*i;p.x=.24*p.t;p.s=p.x;p.speed=.24;}
 FlatnessReducedNMPC solver;auto out=solver.solve(r,cfg);assert(out.ok&&out.decision_dimension==6&&out.max_violation<=1e-7);
 r.state.speed=0.;assert(solver.solve(r,cfg).reason=="flatness_low_speed_invalid");
 r.state.speed=.24;r.force_timeout=true;assert(solver.solve(r,cfg).timed_out);r.force_timeout=false;
 r.state.x=missing();assert(!solver.solve(r,cfg).ok);r.state.x=-.3;
 r.previous.steering=.32;r.state.steering=-.32;assert(!solver.solve(r,cfg).ok);
 std::cout<<"PASS flat6 derivatives, nominal reconstruction, dimension, constraints and invalid/fault handling\n";
}
