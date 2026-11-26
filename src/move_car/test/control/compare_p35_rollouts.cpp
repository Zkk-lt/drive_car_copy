// Deterministic candidate-by-candidate proof that caching preserves the original
// event partition, delayed dynamics, constraints and objective. Test-only access
// is enabled by -fno-access-control; this binary does not connect to ROS.
#include "control/reduced_nmpc.h"
#include <cassert>
#include <iomanip>
#include <iostream>
#include <random>
using namespace research_control;
int main(){
  std::mt19937 random(3517);std::uniform_real_distribution<double> u(-1.,1.);
  std::cout<<std::setprecision(17);
#ifdef P35_OPTIMIZED
  { // Warm-start knot zero must sample by elapsed time, including skipped cycles.
    ReducedNMPC solver;SolverConfig c;SolveRequest r;r.control_dt=.04;r.speed_cap=.35;
    r.previous.speed=.2;solver.warm_stamp_=10.;solver.warm_first_interval_=.033;
    solver.warm_dt_=.1;solver.warm_count_=20;
    for(int j=0;j<20;++j)solver.warm_controls_[j]={.2,.001*j};
    for(double elapsed:{.01,.035,.15,.35,.73}){
      r.now=10.+elapsed;const auto z=solver.makeSeed(r,c,3);
      const int expected=elapsed<.033?0:1+int((elapsed-.033+1e-10)/.1);
      const double first_rate=.001*expected/r.control_dt;
      assert(std::abs(z[1]-first_rate)<1e-12);
    }
  }
#endif

  for(int sample=0;sample<240;++sample){
    SolverConfig c;c.budget_ms=1000.;c.dt=sample%3==0?.03:(sample%3==1?.1:.25);
    c.steps=sample%4==0?40:20;c.knots=sample%5==0?10:6;
    c.model.steering_delay=sample%6==0?0.:(sample%6==1?.015:.325+.05*u(random));
    c.model.speed_delay=sample%5==0?.001:.20+.04*u(random);
    c.model.steering_tau=.44+.12*u(random);c.model.speed_tau=.20+.05*u(random);
    c.model.yaw_gain=1.+.12*u(random);
    c.model.longitudinal_residual=.005*u(random);
    c.model.yaw_residual=.005*u(random);c.model.lateral_residual=.002*u(random);
    SolveRequest r;r.now=10.+sample*.07;r.control_dt=.033+.008*u(random);
    r.state.speed=.19+.07*u(random);r.state.steering=.10*u(random);r.state.y=.025*u(random);
    r.state.yaw=.04*u(random);r.previous={r.state.speed,r.state.steering};r.baseline=r.previous;
    r.speed_cap=.35;
    for(int j=0;j<60;++j)r.history.push_back({r.now-1.2+.02*j,r.previous});
    std::vector<ReferencePoint> path(501);double yaw=0.;
    for(int j=0;j<501;++j){
      auto& p=path[j];p.s=.02*j;p.curvature=.30*std::sin(.04*j+.5*sample);
      if(j){yaw+=.02*p.curvature;p.x=path[j-1].x+.02*std::cos(yaw);p.y=path[j-1].y+.02*std::sin(yaw);}
      else p.x=c.model.rear_to_center;
      p.yaw=yaw;
    }
    r.reference=FlatnessReferenceModel::buildTimeReference(path,r.state.speed,.23,r.now,c);
    ReducedNMPC solver;ReducedNMPC::Decision z{};
    for(int j=0;j<2*c.knots;++j)z[j]=.8*u(random);
    bool expired=false;
#ifdef P35_OPTIMIZED
    SolveProfile profile;solver.prepare(r,c,profile);
    auto v=solver.rollout(z,r,c,solver.prepared_,ReducedNMPC::Clock::now()+std::chrono::seconds(5),expired,profile);
#else
    auto v=solver.rollout(z,r,c,ReducedNMPC::Clock::now()+std::chrono::seconds(5),expired);
#endif
    if(expired)return 2;
    std::cout<<sample<<' '<<v.feasible<<' '<<v.cost<<' '<<v.violation;
    for(int j=0;j<c.steps;++j){
      auto s=v.states[j+1];auto x=v.commands[j];
      std::cout<<' '<<s.x<<' '<<s.y<<' '<<s.yaw<<' '<<s.speed<<' '<<s.steering<<' '<<s.acceleration
               <<' '<<x.speed<<' '<<x.steering<<' '<<v.command_accelerations[j];
    }
    std::cout<<'\n';
  }
}
