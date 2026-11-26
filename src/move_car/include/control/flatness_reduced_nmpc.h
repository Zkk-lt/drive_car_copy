#pragma once
#include "reduced_nmpc.h"
namespace research_control {
// Research-only six-coefficient flat-output search. Reuses the original delayed
// plant, objective and feasible rollout; no second dynamics implementation.
class FlatnessReducedNMPC {
 using Core=ReducedNMPC;
 using Clock=std::chrono::steady_clock;
public:
 using Coefficients=std::array<double,6>;
 void reset(){warm_=false;}
 static FlatOutputJet jet(const State& s,const Model& m,double t,double horizon,const Coefficients& z){
   FlatOutputJet j;const double co=std::cos(s.yaw),si=std::sin(s.yaw);
   const double normal=s.speed*s.speed*std::tan(s.steering)/m.wheelbase;
   const double ax=s.acceleration*co-normal*si,ay=s.acceleration*si+normal*co;
   j.x=s.x+s.speed*co*t+.5*ax*t*t;j.y=s.y+s.speed*si*t+.5*ay*t*t;
   j.dx=s.speed*co+ax*t;j.dy=s.speed*si+ay*t;j.ddx=ax;j.ddy=ay;
   const double u=t/horizon;
   for(int k=0;k<3;++k){const int n=k+3;
     const double x=.03*(co*z[k]-si*z[k+3]),y=.03*(si*z[k]+co*z[k+3]);
     const double p=std::pow(u,n),d=n*std::pow(u,n-1)/horizon;
     const double dd=n*(n-1)*std::pow(u,n-2)/(horizon*horizon);
     const double ddd=n*(n-1)*(n-2)*std::pow(u,n-3)/(horizon*horizon*horizon);
     j.x+=x*p;j.y+=y*p;j.dx+=x*d;j.dy+=y*d;j.ddx+=x*dd;j.ddy+=y*dd;j.dddx+=x*ddd;j.dddy+=y*ddd;
   }
   return j;
 }
 SolveResult solve(const SolveRequest& r,const SolverConfig& c){
   const auto start=Clock::now();SolveResult out;out.decision_dimension=6;out.command=r.previous;
   auto finish=[&](const char* why){out.reason=why;out.solve_ms=Core::milliseconds(start);
     if(out.solve_ms>=c.budget_ms && std::isfinite(c.budget_ms)){
       out.ok=out.feasible=false;out.timed_out=true;out.command=r.previous;out.reason="solver_timeout";warm_=false;
     }return out;};
   if(!Core::validConfig(c))return finish("invalid_config");
   if(!Core::validState(r.state)||!Core::finiteCommand(r.previous)||!Core::finiteCommand(r.baseline)||
      !std::isfinite(r.now)||!std::isfinite(r.control_dt)||r.control_dt<.001||r.control_dt>.25||
      !std::isfinite(r.speed_cap)||r.speed_cap<0.||!std::isfinite(r.previous_command_acceleration))
     return finish("invalid_state_or_request");
   if(!Core::validReference(r.reference,c))return finish("reference_invalid");
   if(r.force_timeout){out.timed_out=true;return finish("injected_timeout");}
   if(r.force_infeasible)return finish("injected_infeasible");
   if(r.force_numerical_failure)return finish("injected_numerical_failure");
   if(r.state.speed<.015)return finish("flatness_low_speed_invalid");
   if(r.history.size()>512)return finish("command_history_too_large");
   double stamp=-std::numeric_limits<double>::infinity();
   for(const auto& h:r.history){
     if(!std::isfinite(h.stamp)||h.stamp<stamp||h.stamp>r.now+1e-6||!Core::finiteCommand(h.command))return finish("invalid_command_history");
     stamp=h.stamp;
   }
   const double delay=std::max(c.model.steering_delay,c.model.speed_delay);
   if(delay>1e-9&&(r.history.empty()||r.history.front().stamp>r.now-delay+1e-6))return finish("history_incomplete");
   const auto deadline=start+Core::duration(c.budget_ms);
   const auto search_deadline=deadline-Core::duration(std::min(.8,.12*c.budget_ms+.05));
   if(!core_.prepare(r,c,out.profile))return finish("schedule_capacity_exceeded");
   const double horizon=r.control_dt+(c.steps-1)*c.dt;
   Core::Rollout best;Coefficients selected{};bool stopped=false;
   auto evaluate=[&](const Coefficients& z){
     Core::Rollout roll;if(Clock::now()>=search_deadline){stopped=true;return roll;}
     ++out.evaluations;std::array<Command,kMaxHorizon> commands{};
     for(int i=0;i<c.steps;++i){
       // Commands are sampled at end of each interval, then held by the actual
       // delayed predictor. Continuous nominal reconstruction != actual state.
       const auto q=jet(r.state,c.model,Core::startTime(i,r,c)+Core::interval(i,r,c),horizon,z);
       const auto f=FlatnessReferenceModel::reconstruct(q,c.model);
       if(!f.valid||q.dx*std::cos(r.state.yaw)+q.dy*std::sin(r.state.yaw)<=.015)return roll;
       commands[i]=f.feedforward;
     }
     return Core::rollout(Core::Decision{},r,c,core_.prepared_,search_deadline,stopped,out.profile,&commands);
   };
   best=evaluate(selected);out.initial_objective=best.feasible?best.cost:missing();
   // Reusing coefficients is a warm candidate, not an exact time-shifted flat
   // curve: initial jet is re-anchored and full feasibility is checked again.
   out.warm_available=warm_&&r.now>=stamp_&&r.now-stamp_<horizon;
   if(out.warm_available&&!stopped){const auto q=evaluate(previous_);if(q.feasible&&q.cost<best.cost){best=q;selected=previous_;out.warm_started=true;}}
   for(int it=0;it<c.iterations&&!stopped;++it){
     const double radius=.35*std::pow(.5,it);
     for(int k=0;k<6&&!stopped;++k){const auto base=selected;
       for(int sign:{-1,1}){auto z=base;z[k]=clamp(z[k]+sign*radius,-1.,1.);
         auto roll=evaluate(z);if(roll.feasible&&!stopped&&roll.cost<best.cost){best=roll;selected=z;}
         if(stopped)break;
       }
     }out.iterations=it+1;
   }
   out.budget_limited=stopped;
   if(Clock::now()>=deadline)return finish("solver_timeout");
   if(!best.feasible)return finish("flatness_no_feasible_rollout");
   out.ok=out.feasible=true;out.objective=best.cost;out.max_violation=best.violation;
   out.command=best.commands[0];out.command_acceleration=best.command_accelerations[0];
   out.controls=best.commands;out.predicted=best.states;out.count=c.steps+1;
   previous_=selected;warm_=true;stamp_=r.now;
   return finish(stopped?"flatness_feasible_budget_stop":"flatness_feasible_pattern_search");
 }
private:
 Core core_;Coefficients previous_{};bool warm_=false;double stamp_=0.;
};
}
