#pragma once
#include "unified_control.h"
namespace research_control {
struct P5PropagationResult{State state;bool valid=false;std::string reason="invalid_propagation_input";double position_uncertainty=0.,yaw_uncertainty=0.;};
inline P5PropagationResult p5PropagateNominal(State state,double observed,double target,const Model&m,const Limits&l,const std::vector<CommandSample>&history,double position_uncertainty,double yaw_uncertainty,double position_rate=.04,double yaw_rate=.08,double available=0.){
 P5PropagationResult out;out.state=state;
 if(!unified_control::validLimits(l)||!std::isfinite(observed)||!std::isfinite(target)||target<observed||target-observed>.075||available<observed||available>target||history.empty()||history.size()>512||!std::isfinite(position_uncertainty)||position_uncertainty<0.||!std::isfinite(yaw_uncertainty)||yaw_uncertainty<0.||m.speed_tau<=0.||m.steering_tau<=0.||m.wheelbase<=0.)return out;
 for(double v:{state.x,state.y,state.yaw,state.speed,state.steering,state.acceleration,m.speed_tau,m.steering_tau,m.speed_delay,m.steering_delay,m.wheelbase})if(!std::isfinite(v))return out;
 double last=-INFINITY;for(const auto&h:history){if(!std::isfinite(h.stamp)||h.stamp<=last||h.stamp>available||!unified_control::finiteCommand(h.command))return out;last=h.stamp;}
 if(history.front().stamp>observed-std::max(m.speed_delay,m.steering_delay)){out.reason="actual_delay_history_incomplete";return out;}
 auto held=[&](double stamp){auto it=std::upper_bound(history.begin(),history.end(),stamp,[](double t,const CommandSample&h){return t<h.stamp;});return it==history.begin()?history.front().command:(it-1)->command;};
 // Publication stamps are strictly ordered. Within each delayed command
 // channel only the first future event can shorten the current substep.
 // Keep the original time expression, tolerance and held-command lookup.
 std::size_t speed_event=0, steering_event=0;
 auto shortenAtEvent=[&](double t,double delay,std::size_t& next,double& dt){
  while(next<history.size()&&history[next].stamp+delay-t<=1e-10)++next;
  if(next<history.size())dt=std::min(dt,history[next].stamp+delay-t);
 };
 for(double t=observed;t<target-1e-10;){
  double dt=std::min(.005,target-t);
  shortenAtEvent(t,m.speed_delay,speed_event,dt);
  shortenAtEvent(t,m.steering_delay,steering_event,dt);
  const auto speed=held(t-m.speed_delay),steer=held(t-m.steering_delay);
  const double a=(m.speed_gain*speed.speed-state.speed)/m.speed_tau+m.longitudinal_residual;
  const double dr=(m.steering_gain*steer.steering-state.steering)/m.steering_tau;
  const double r=m.yaw_gain*state.speed*std::tan(state.steering)/m.wheelbase+m.yaw_residual;
  const double err=1e-7;if(state.speed<0.||state.speed>l.speed+err||std::fabs(state.steering)>l.steering+err||a>l.acceleration+err||a<-l.deceleration-err||std::fabs(dr)>l.steering_rate+err||std::fabs(state.speed*r)>l.lateral_acceleration+err){out.reason="propagation_physical_envelope_exceeded";return out;}
  state.x+=dt*(state.speed*std::cos(state.yaw)-m.lateral_residual*std::sin(state.yaw));state.y+=dt*(state.speed*std::sin(state.yaw)+m.lateral_residual*std::cos(state.yaw));state.yaw=wrap(state.yaw+r*dt);
  state.speed=m.speed_gain*speed.speed+m.longitudinal_residual*m.speed_tau+(state.speed-m.speed_gain*speed.speed-m.longitudinal_residual*m.speed_tau)*std::exp(-dt/m.speed_tau);
  state.steering=m.steering_gain*steer.steering+(state.steering-m.steering_gain*steer.steering)*std::exp(-dt/m.steering_tau);state.acceleration=(m.speed_gain*speed.speed-state.speed)/m.speed_tau+m.longitudinal_residual;t+=dt;
 }
 out.state=state;out.position_uncertainty=position_uncertainty+(target-observed)*position_rate;out.yaw_uncertainty=yaw_uncertainty+(target-observed)*yaw_rate;out.valid=true;out.reason="bounded_nominal_actual_command_hold_not_measurement";return out;
}
}
