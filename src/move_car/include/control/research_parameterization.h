#pragma once
#include "flatness_reference_model.h"
namespace research_control {
// All grids parameterize the SAME bounded demand chart. These are not extra
// physical states, actuator delays, or a claim of flat-output optimization.
inline std::array<double,kMaxKnots> researchKnotPositions(const SolverConfig& c,
    const std::array<ReferencePoint,kMaxHorizon>& ref,double first_interval){
 std::array<double,kMaxKnots> p{};
 // Knots locate command START times (step 0 at t=0), not state end times.
 const double horizon=first_interval+(c.steps-2)*c.dt;
 if(c.research_grid==3){for(int k=0;k<c.knots;++k)p[k]=double(k)*c.steps/c.knots;return p;}
 std::array<double,kMaxHorizon> cumulative{};
 if(c.research_grid==2){
  for(int i=1;i<c.steps;++i){
   const double t=first_interval+(i-1)*c.dt;
   const double variation=std::min(1.,std::abs(ref[i].curvature-ref[i-1].curvature)/.05);
   cumulative[i]=cumulative[i-1]+1.+1.5*std::exp(-t/.45)+3.*variation;
  }
 }
 for(int k=0;k<c.knots;++k){
  const double fraction=double(k)/(c.knots-1);
  if(c.research_grid==1){
   const double t=std::pow(fraction,1.5)*horizon;
   p[k]=t<=first_interval?t/first_interval:1.+(t-first_interval)/c.dt;
   p[k]=clamp(p[k],0.,double(c.steps-1));
  }else if(c.research_grid==2){
   const double target=fraction*cumulative[c.steps-1];int i=0;
   while(i+1<c.steps-1&&cumulative[i+1]<target)++i;
   p[k]=i+(target-cumulative[i])/(cumulative[i+1]-cumulative[i]);
  }else p[k]=fraction*(c.steps-1);
 }
 p[0]=0.;p[c.knots-1]=c.steps-1;return p;
}
inline void researchKnotCoordinate(double step,const std::array<double,kMaxKnots>& p,
    const SolverConfig& c,int& left,double& fraction){
 if(c.research_grid==3){
  const int block=std::min(c.knots-1,int(step*c.knots/c.steps));
  left=std::min(block,c.knots-2);fraction=block==c.knots-1?1.:0.;return;
 }
 left=0;while(left+1<c.knots-1&&p[left+1]<step)++left;
 fraction=clamp((step-p[left])/(p[left+1]-p[left]),0.,1.);
}
// Kinematic inversion of a body-center jet given body heading. Heading evolves
// through an ODE: center position alone is NOT asserted to be a flat output.
inline FlatOutputJet rearJetFromCenter(const FlatOutputJet& c,double heading,double l){
 const double co=std::cos(heading),si=std::sin(heading);
 const double vn=-si*c.dx+co*c.dy,ve=co*c.dx+si*c.dy;
 const double an=-si*c.ddx+co*c.ddy,ae=co*c.ddx+si*c.ddy;
 const double jn=-si*c.dddx+co*c.dddy;
 const double r=vn/l,rd=(an-r*ve)/l,rdd=(jn-2*r*ae-rd*ve-r*r*vn)/l;
 FlatOutputJet q;
 q.x=c.x-l*co;q.y=c.y-l*si;
 q.dx=c.dx+l*r*si;q.dy=c.dy-l*r*co;
 q.ddx=c.ddx+l*rd*si+l*r*r*co;q.ddy=c.ddy-l*rd*co+l*r*r*si;
 q.dddx=c.dddx+l*(rdd-r*r*r)*si+3*l*r*rd*co;
 q.dddy=c.dddy-l*(rdd-r*r*r)*co+3*l*r*rd*si;
 return q;
}
}
