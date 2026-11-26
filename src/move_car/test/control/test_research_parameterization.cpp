#include "control/reduced_nmpc.h"
#include "control/adaptive_runtime.h"
#include <cassert>
#include <iostream>
using namespace research_control;
int main(){
 SolverConfig c;c.knots=4;std::array<ReferencePoint,kMaxHorizon> ref{};
 for(int i=0;i<20;++i){ref[i].curvature=i<5?0.:.3;ref[i].speed=.24;}
 for(int grid=1;grid<=3;++grid){
  c.research_grid=grid;auto p=researchKnotPositions(c,ref,1./30.);
  for(int k=1;k<4;++k)assert(p[k]>p[k-1]&&p[k]<=19.);
  for(int i=0;i<20;++i){int left;double f;researchKnotCoordinate(i,p,c,left,f);assert(left>=0&&left<3&&f>=0.&&f<=1.);}
 }
 const double l=.3,theta=.4,co=std::cos(theta),si=std::sin(theta),v=.25,a=.04,j=.02,r=.1,rd=.03,rdd=-.02;
 FlatOutputJet q;q.x=1.;q.y=2.;q.dx=v*co;q.dy=v*si;q.ddx=a*co-v*r*si;q.ddy=a*si+v*r*co;
 q.dddx=(j-v*r*r)*co-(2*a*r+v*rd)*si;q.dddy=(j-v*r*r)*si+(2*a*r+v*rd)*co;
 auto pc=q;pc.x+=l*co;pc.y+=l*si;pc.dx-=l*r*si;pc.dy+=l*r*co;
 pc.ddx-=l*rd*si+l*r*r*co;pc.ddy+=l*rd*co-l*r*r*si;
 pc.dddx-=l*(rdd-r*r*r)*si+3*l*r*rd*co;pc.dddy+=l*(rdd-r*r*r)*co-3*l*r*rd*si;
 auto recovered=rearJetFromCenter(pc,theta,l);
 assert(std::abs(recovered.x-q.x)<1e-12&&std::abs(recovered.dx-q.dx)<1e-12&&std::abs(recovered.ddy-q.ddy)<1e-12&&std::abs(recovered.dddx-q.dddx)<1e-12);
 for(int grid=0;grid<=3;++grid){
  c.research_grid=grid;c.budget_ms=1000.;SolveRequest req;req.now=1.;req.state.speed=.24;req.state.x=-.3;req.previous.speed=.24;req.baseline=req.previous;req.history.push_back({0.,req.previous});req.speed_cap=.3;req.reference.valid=true;req.reference.count=21;
  for(int i=0;i<=20;++i){auto& p=req.reference.points[i];p.t=i*.1;p.x=p.s=.24*p.t;p.speed=.24;}
  ReducedNMPC solver;auto result=solver.solve(req,c);assert(result.ok&&result.decision_dimension==8&&result.max_violation<=1e-7);
  req.warm_hint=result.controls;req.warm_hint_count=c.steps;assert(solver.solve(req,c).ok);
  req.warm_hint[0].speed=missing();assert(!solver.solve(req,c).ok);
 }
 for(const std::string name:{"original12","blocked8","front8","curve8","hold8","flatseed8","refine12","search8"}){
  RuntimeConfig config;config.research_backend=name;assert(AdaptiveRuntime::validate(config));
 }
 std::cout<<"PASS grids, exact center/rear jet inversion, eight-variable feasibility and validated warm hints\n";
}
