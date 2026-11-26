// Exact causal historical-request regression. This is not a changed real-vehicle
// feedback experiment. Numeric fixture is emitted from original request,
// immutable model snapshot and P5 publication history; no interpolation.
#include "control/p6_flatness_nmpc.h"
#include "control/p5_nominal_propagation.h"
#include <cassert>
#include <fstream>
#include <iomanip>
#include <iostream>
using namespace research_control;
bool readExample(std::istream& in,int& cycle,SolveRequest&r,SolverConfig&c){
 if(!(in>>cycle))return false;
 int admissible,braking,nh,np;double pu,yu,observed,snapshot,target,apply;
 in>>admissible>>braking>>pu>>yu>>observed>>snapshot>>r.control_dt>>r.speed_cap;
 in>>r.previous.speed>>r.previous.steering>>r.previous_command_acceleration>>r.baseline.speed>>r.baseline.steering;
 in>>r.state.x>>r.state.y>>r.state.yaw>>r.state.speed>>r.state.steering>>r.state.acceleration;
 auto&m=c.model;in>>m.wheelbase>>m.rear_to_center>>m.steering_tau>>m.steering_delay>>m.speed_tau>>m.speed_delay>>m.yaw_gain>>m.steering_gain>>m.speed_gain>>m.longitudinal_residual>>m.yaw_residual>>m.lateral_residual;
 auto&l=c.limits;in>>l.speed>>l.steering>>l.steering_rate>>l.acceleration>>l.deceleration>>l.jerk>>l.lateral_acceleration;
 auto&w=c.weights;in>>w.lateral>>w.heading>>w.speed>>w.smoothness>>c.steps>>c.knots>>c.iterations>>c.dt>>nh>>np;
 for(int i=0;i<nh;++i){CommandSample h;unsigned long long sequence;in>>h.stamp>>h.command.speed>>h.command.steering>>sequence;r.history.push_back(h);}
 std::vector<ReferencePoint> path;
 for(int i=0;i<np;++i){ReferencePoint p;in>>p.s>>p.x>>p.y>>p.yaw>>p.curvature;path.push_back(p);}
 in>>target>>apply;assert(in&&admissible);
 auto initial=p5PropagateNominal(r.state,observed,apply,m,l,r.history,pu,yu,.04,.08,snapshot);
 assert(initial.valid&&initial.position_uncertainty<=.03&&initial.yaw_uncertainty<=.1);r.state=initial.state;r.now=apply;r.target_speed=target;
 FlatnessReferenceModel::recoverMissingSpatialCurvature(path);
 const double cx=r.state.x+m.rear_to_center*std::cos(r.state.yaw),cy=r.state.y+m.rear_to_center*std::sin(r.state.yaw);
 std::size_t index=0;double fraction=0.,distance=INFINITY;
 for(std::size_t i=0;i+1<path.size();++i){const auto&a=path[i];const auto&b=path[i+1];double dx=b.x-a.x,dy=b.y-a.y,d=dx*dx+dy*dy;
  double f=d>1e-12?clamp(((cx-a.x)*dx+(cy-a.y)*dy)/d,0.,1.):0.;double e=std::hypot(a.x+f*dx-cx,a.y+f*dy-cy);
  if(e<distance){distance=e;index=i;fraction=f;}}
 std::vector<ReferencePoint> spatial;auto a=path[index];const auto&b=path[index+1];a.s+=fraction*(b.s-a.s);a.x+=fraction*(b.x-a.x);a.y+=fraction*(b.y-a.y);a.yaw=wrap(a.yaw+fraction*wrap(b.yaw-a.yaw));a.curvature+=fraction*(b.curvature-a.curvature);spatial.push_back(a);
 for(std::size_t i=index+1;i<path.size();++i)if(path[i].s>spatial.back().s+1e-9)spatial.push_back(path[i]);
 r.reference=FlatnessReferenceModel::buildTimeReference(spatial,r.state.speed,target,r.now,c,braking!=0);assert(r.reference.valid);return true;
}
void configure(P6FlatnessNMPC& solver){solver.setConditionedGeometry(true);solver.setBaselineSeed(true);solver.setDelayAlignedReconstruction(true);solver.setCommandProjection(true);solver.setResponseBase(true);solver.setResponseNearSupport(true);}
int main(int argc,char**argv){
 if(argc<2)return 2;
 std::ifstream in(argv[1]);assert(in);const double budget=argc>2?std::atof(argv[2]):50.;
 std::cout<<std::setprecision(17)<<"cycle,ok,reason,variables,solve_ms,objective,speed,steering,validated,violation,recovery_attempted,recovery_used\n";
 int cycle,n=0;bool saw_recovery=false;
 while(true){SolveRequest r;SolverConfig c;if(!readExample(in,cycle,r,c))break;++n;c.budget_ms=budget;
  P6FlatnessNMPC solver;configure(solver);const auto out=solver.solve(r,c);assert(out.decision_dimension==8);
  bool validated=false;double violation=missing();
  if(out.ok){const auto checked=ReducedNMPC::validateControls(r,c,out.controls,std::chrono::steady_clock::now()+std::chrono::milliseconds(100));validated=checked.feasible;violation=checked.violation;assert(validated&&violation<=1e-7);}
#ifdef ENTRY_BASELINE
  const bool attempted=false,used=false;
#else
  const bool attempted=solver.diagnostics().response_recovery_seed_attempted,used=solver.diagnostics().response_recovery_seed_used;
  if(cycle==2136&&budget>=50.){assert(used&&out.ok);saw_recovery=true;
   // A genuinely empty first-input interval cannot be repaired by a new seed.
   auto invalid=r;invalid.previous_command_acceleration=c.limits.acceleration+1.;P6FlatnessNMPC check;configure(check);
   const auto bad=check.solve(invalid,c);assert(!bad.ok&&!check.diagnostics().response_recovery_seed_attempted);
   invalid=r;invalid.history.clear();assert(!check.solve(invalid,c).ok);
   invalid=r;invalid.force_timeout=true;assert(check.solve(invalid,c).timed_out);
   invalid=r;invalid.speed_cap=r.state.speed-.01;assert(!check.solve(invalid,c).ok);
  }
#endif
  std::cout<<cycle<<','<<out.ok<<','<<out.reason<<','<<out.decision_dimension<<','<<out.solve_ms<<','<<out.objective<<','<<out.command.speed<<','<<out.command.steering<<','<<validated<<','<<violation<<','<<attempted<<','<<used<<'\n';
 }
 assert(n==9);
#ifndef ENTRY_BASELINE
 if(budget>=50.)assert(saw_recovery);
#endif
}
