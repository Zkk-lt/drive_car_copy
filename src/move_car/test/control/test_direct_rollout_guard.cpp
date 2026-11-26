// Regression: coordinate tolerance must not admit a physical jerk violation.
#include "control/reduced_nmpc.h"
#include <iostream>
#include <iomanip>
#include <stdexcept>
using namespace research_control;
int main(){SolverConfig c;c.budget_ms=1000.;SolveRequest r;r.now=10.;r.control_dt=1./30.;r.previous.speed=.24;r.baseline=r.previous;r.state.speed=.24;r.state.x=-.3;r.speed_cap=.3;r.history.push_back({8.,r.previous});r.reference.valid=true;r.reference.count=21;
for(int i=0;i<=20;++i){auto& p=r.reference.points[i];p.t=.1*i;p.x=p.s=.24*p.t;p.speed=.24;}
ReducedNMPC core;SolveProfile profile;core.prepare(r,c,profile);std::array<Command,kMaxHorizon> direct{};const double a=c.limits.jerk*r.control_dt+5e-8;double speed=r.previous.speed;
for(int i=0;i<c.steps;++i){speed+=a*(i==0?r.control_dt:c.dt);direct[i].speed=speed;}
bool stopped=false;auto roll=ReducedNMPC::rollout(ReducedNMPC::Decision{},r,c,core.prepared_,std::chrono::steady_clock::now()+std::chrono::seconds(1),stopped,profile,&direct);
std::cout<<std::setprecision(17)<<"feasible="<<roll.feasible<<" command_jerk="<<a/r.control_dt<<" jerk_limit="<<c.limits.jerk<<"\n";
if(roll.feasible){throw std::runtime_error("direct inputs exceeded physical jerk but were accepted");}
return 0;}
