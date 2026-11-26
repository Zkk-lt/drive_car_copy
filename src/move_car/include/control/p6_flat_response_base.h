#pragma once
#include "control/flatness_reference_model.h"

namespace research_control {
// Immutable samples of a NOMINAL response-model prediction. These are not
// measured states. Recorded commands are used before issue time zero; the
// supplied, still-unexecuted seed is used afterwards. No command is published.
// Sample jets use right limits at ZOH activation events, including the tail.
struct P6FlatResponseBase {
  static constexpr int kCapacity=3*kMaxHorizon+4;
  std::array<double,kCapacity> times{};
  std::array<FlatOutputJet,kCapacity> jets{};
  int count=0;
  bool valid=false;
  const char* reason="uninitialized_response_base";

  static P6FlatResponseBase build(const SolveRequest& r,const SolverConfig& c,
      const std::array<Command,kMaxHorizon>& issued,
      const std::array<double,kCapacity>& requested,int requested_count){
    P6FlatResponseBase out;
    if(requested_count<0||requested_count>kCapacity||c.steps<2||c.steps>kMaxHorizon||
       !std::isfinite(r.control_dt)||r.control_dt<=0.||
       !std::isfinite(c.dt)||c.dt<=0.||!std::isfinite(r.now)){
      out.reason="response_base_invalid_request";return out;
    }
    const auto& m=c.model;
    const double parameters[]={m.wheelbase,m.speed_tau,m.steering_tau,m.speed_delay,
      m.steering_delay,m.speed_gain,m.steering_gain,m.yaw_gain,
      m.longitudinal_residual,m.yaw_residual,m.lateral_residual};
    for(double value:parameters)if(!std::isfinite(value)){
      out.reason="response_base_nonfinite_model";return out;
    }
    if(m.wheelbase<=0.||m.speed_tau<=0.||m.steering_tau<=0.||
       m.speed_delay<0.||m.steering_delay<0.||m.speed_gain<=0.||
       m.steering_gain<=0.||m.yaw_gain<=0.||std::abs(m.lateral_residual)>1e-9){
      out.reason="response_base_unsupported_model";return out;
    }
    if(!finiteState(r.state)||!finiteCommand(r.previous)){
      out.reason="response_base_invalid_initial_state";return out;
    }
    for(int i=0;i<c.steps;++i)if(!finiteCommand(issued[i])){
      out.reason="response_base_nonfinite_seed";return out;
    }
    if(r.history.size()>512){out.reason="response_base_history_too_large";return out;}
    double history_last=-std::numeric_limits<double>::infinity();
    for(const auto& sample:r.history){
      if(!std::isfinite(sample.stamp)||sample.stamp<history_last||
         sample.stamp>r.now+1e-6||!finiteCommand(sample.command)){
        out.reason="response_base_invalid_history";return out;
      }
      history_last=sample.stamp;
    }
    const double delay=std::max(m.speed_delay,m.steering_delay);
    if(delay>1e-9&&(r.history.empty()||r.history.front().stamp>r.now-delay+1e-6)){
      out.reason="response_base_history_incomplete";return out;
    }
    const double horizon=r.control_dt+(c.steps-1)*c.dt;
    const double end=horizon+delay;
    for(int i=0;i<requested_count;++i){
      const double t=requested[i];
      if(!std::isfinite(t)||t<0.||t>end+1e-9){
        out.reason="response_base_sample_outside_prediction";return out;
      }
      out.times[out.count++]=t;
    }
    // All needed integration storage is fixed-size. Sorting changes no
    // timestamp; equal requested events are represented by one right-side jet.
    std::sort(out.times.begin(),out.times.begin()+out.count);
    int unique=0;
    for(int i=0;i<out.count;++i)
      if(unique==0||std::abs(out.times[i]-out.times[unique-1])>1e-10)
        out.times[unique++]=out.times[i];
    out.count=unique;
    if(out.count==0||out.times[0]>1e-10){
      if(out.count==kCapacity){out.reason="response_base_capacity";return out;}
      for(int i=out.count;i>0;--i)out.times[i]=out.times[i-1];
      out.times[0]=0.;++out.count;
    }else out.times[0]=0.;

    State state=r.state;double time=0.;
    for(int i=0;i<out.count;++i){
      const double sample_time=out.times[i];
      while(time<sample_time-1e-11){
        const Command uv=source(time-m.speed_delay,r,c,issued);
        const Command ud=source(time-m.steering_delay,r,c,issued);
        double next=std::min(sample_time,time+.01);
        next=std::min(next,nextEvent(time,m.speed_delay,r,c));
        next=std::min(next,nextEvent(time,m.steering_delay,r,c));
        const double h=next-time;
        if(!(h>0.)||!std::isfinite(h)){
          out.reason="response_base_invalid_event_partition";return out;
        }
        const double veq=m.speed_gain*uv.speed+m.speed_tau*m.longitudinal_residual;
        const double v=veq+(state.speed-veq)*std::exp(-h/m.speed_tau);
        const double deq=m.steering_gain*ud.steering;
        const double delta=deq+(state.steering-deq)*std::exp(-h/m.steering_tau);
        const double vm=.5*(state.speed+v),dm=.5*(state.steering+delta);
        const double omega=m.yaw_gain*vm*std::tan(dm)/m.wheelbase+m.yaw_residual;
        const double heading=state.yaw+.5*h*omega;
        state.x+=h*vm*std::cos(heading);state.y+=h*vm*std::sin(heading);
        state.yaw=wrap(state.yaw+h*omega);state.speed=v;state.steering=delta;
        state.acceleration=(m.speed_gain*uv.speed-v)/m.speed_tau+m.longitudinal_residual;
        if(!finiteState(state)||state.speed<.015){
          out.reason="response_base_invalid_or_low_speed_prediction";return out;
        }
        time=next;
      }
      time=sample_time;
      // Select again AT the sample, so an activation exactly at t uses the
      // new command rather than the command from the integration left limit.
      const Command uv=source(sample_time-m.speed_delay,r,c,issued);
      const Command ud=source(sample_time-m.steering_delay,r,c,issued);
      if(!makeJet(state,uv,ud,m,out.jets[i])){
        out.reason="response_base_invalid_or_low_speed_jet";return out;
      }
    }
    out.valid=true;out.reason="nominal_response_right_limit_jets";return out;
  }

 private:
  static bool finiteCommand(const Command& u){
    return std::isfinite(u.speed)&&std::isfinite(u.steering);
  }
  static bool finiteState(const State& s){
    return std::isfinite(s.x)&&std::isfinite(s.y)&&std::isfinite(s.yaw)&&
      std::isfinite(s.speed)&&std::isfinite(s.steering)&&std::isfinite(s.acceleration);
  }
  static double issueTime(int index,const SolveRequest& r,const SolverConfig& c){
    return index==0?0.:r.control_dt+(index-1)*c.dt;
  }
  static Command source(double relative_time,const SolveRequest& r,const SolverConfig& c,
      const std::array<Command,kMaxHorizon>& issued){
    if(relative_time>=-1e-10){
      int index=0;
      while(index+1<c.steps&&issueTime(index+1,r,c)<=relative_time+1e-10)++index;
      return issued[index]; // The last issued demand remains held past H.
    }
    const double stamp=r.now+relative_time;
    const auto it=std::upper_bound(r.history.begin(),r.history.end(),stamp+1e-10,
      [](double t,const CommandSample& sample){return t<sample.stamp;});
    return it==r.history.begin()?r.previous:(it-1)->command;
  }
  static double nextEvent(double time,double delay,const SolveRequest& r,const SolverConfig& c){
    double next=std::numeric_limits<double>::infinity();
    auto it=std::upper_bound(r.history.begin(),r.history.end(),r.now+time-delay+1e-9,
      [](double t,const CommandSample& sample){return t<sample.stamp;});
    // A large ROS epoch has coarser ULPs than relative prediction times. Never
    // return a zero/negative event interval after subtracting that epoch.
    while(it!=r.history.end()&&it->stamp-r.now+delay<=time+1e-9)++it;
    if(it!=r.history.end())next=it->stamp-r.now+delay;
    for(int i=0;i<c.steps;++i){const double activation=issueTime(i,r,c)+delay;
      if(activation>time+1e-9){next=std::min(next,activation);break;}}
    return next;
  }
  static bool makeJet(const State& s,const Command& uv,const Command& ud,
      const Model& m,FlatOutputJet& q){
    if(!finiteState(s)||s.speed<.015||std::abs(s.steering)>=1.5707963267948966)return false;
    const double co=std::cos(s.yaw),si=std::sin(s.yaw),v=s.speed;
    const double delta_cos=std::cos(s.steering);
    if(std::abs(delta_cos)<1e-12)return false;
    const double a=(m.speed_gain*uv.speed-v)/m.speed_tau+m.longitudinal_residual;
    const double delta_rate=(m.steering_gain*ud.steering-s.steering)/m.steering_tau;
    const double omega=m.yaw_gain*v*std::tan(s.steering)/m.wheelbase+m.yaw_residual;
    const double jerk=-a/m.speed_tau; // Right-side derivative within a ZOH hold.
    const double omega_rate=m.yaw_gain*(a*std::tan(s.steering)+
      v*delta_rate/(delta_cos*delta_cos))/m.wheelbase;
    q.x=s.x;q.y=s.y;q.dx=v*co;q.dy=v*si;
    q.ddx=a*co-v*omega*si;q.ddy=a*si+v*omega*co;
    const double tangent=jerk-v*omega*omega,normal=2.*a*omega+v*omega_rate;
    q.dddx=tangent*co-normal*si;q.dddy=tangent*si+normal*co;
    const double values[]={q.x,q.y,q.dx,q.dy,q.ddx,q.ddy,q.dddx,q.dddy};
    for(double value:values)if(!std::isfinite(value))return false;
    return true;
  }
};
} // namespace research_control
