#pragma once
#include "control/research_types.h"

namespace research_control {
// Flat output of the NOMINAL forward, no-slip rear-axle bicycle: (x_r,y_r).
// Delays, tire slip and model residuals are NOT claimed to be flat.
struct FlatOutputJet {
  double x=0.,y=0.,dx=0.,dy=0.,ddx=0.,ddy=0.,dddx=0.,dddy=0.;
};
struct FlatReconstruction {
  bool valid=false;
  State state;
  Command feedforward;
  double curvature=0.,yaw_rate=0.,steering_rate=0.,jerk=0.;
  const char* reason="uninitialized";
};
class FlatnessReferenceModel {
 public:
  // The worker and the publication validator must crop from the same
  // body-center projection, after curvature has been prepared.
  static bool projectSpatialSuffix(const std::vector<ReferencePoint>& path,
      const State& state,const Model& model,std::vector<ReferencePoint>& out){
    out.clear();
    if(path.size()<2||path.size()>1024)return false;
    double last=-INFINITY;
    for(const auto& p:path){
      for(double v:{p.s,p.x,p.y,p.yaw,p.curvature})if(!std::isfinite(v))return false;
      if(p.s<=last)return false;
      last=p.s;
    }
    const double x=state.x+model.rear_to_center*std::cos(state.yaw);
    const double y=state.y+model.rear_to_center*std::sin(state.yaw);
    if(!std::isfinite(x)||!std::isfinite(y))return false;
    std::size_t best=0;
    double fraction=0.,distance2=INFINITY;
    for(std::size_t i=0;i+1<path.size();++i){
      const auto& a=path[i];const auto& b=path[i+1];
      const double dx=b.x-a.x,dy=b.y-a.y,length2=dx*dx+dy*dy;
      const double f=length2>1e-12?clamp(((x-a.x)*dx+(y-a.y)*dy)/length2,0.,1.):0.;
      const double ex=a.x+f*dx-x,ey=a.y+f*dy-y;
      const double error2=ex*ex+ey*ey;
      if(error2<distance2){distance2=error2;best=i;fraction=f;}
    }
    if(!std::isfinite(distance2))return false;
    auto point=path[best];const auto& next=path[best+1];
    point.s+=fraction*(next.s-point.s);
    point.x+=fraction*(next.x-point.x);point.y+=fraction*(next.y-point.y);
    point.yaw=wrap(point.yaw+fraction*wrap(next.yaw-point.yaw));
    point.curvature+=fraction*(next.curvature-point.curvature);
    out.reserve(path.size()-best);
    out.push_back(point);
    for(std::size_t i=best+1;i<path.size();++i)
      if(path[i].s>out.back().s+1e-9)out.push_back(path[i]);
    return out.size()>1;
  }

  static constexpr unsigned kInverseSpeed=1,kInverseSteering=2;
  static FlatReconstruction reconstruct(const FlatOutputJet& q,const Model& m,
                                        double minimum_speed=.015) {
    return reconstructJet<true>(q,m,minimum_speed);
  }

  // The inverse inputs and their Jacobians do not use the absolute heading.
  // All finite-value and low-speed tests are shared with the full reconstruction.
  static FlatReconstruction reconstructForInputs(const FlatOutputJet& q,const Model& m) {
    return reconstructJet<false>(q,m,.015);
  }

  static bool boundedInputModel(const Model& m){
    for(double v:{m.wheelbase,m.steering_tau,m.speed_tau,m.steering_gain,m.speed_gain,m.yaw_gain})
      if(!(v>=1./kInverseRange&&v<=kInverseRange))return false;
    return std::abs(m.yaw_residual)<=kInverseRange&&std::abs(m.longitudinal_residual)<=kInverseRange;
  }

  // Compute only demanded channels. Within this arithmetic domain (jet and
  // model values <=2^16, gain divisors >=2^-16 and speed >=.015), all unused
  // inverse results and jerk are finite. Other data take the full map.
  static bool reconstructSelectedInputs(const FlatOutputJet& q,const Model& m,
      bool bounded_model,unsigned mask,Command& result){
    bool bounded=bounded_model;
    for(double v:{q.x,q.y,q.dx,q.dy,q.ddx,q.ddy,q.dddx,q.dddy})bounded=bounded&&std::abs(v)<=kInverseRange;
    if(!bounded){
      const auto full=reconstructForInputs(q,m);
      result=full.feedforward;return full.valid;
    }
    const double v2=q.dx*q.dx+q.dy*q.dy,v=std::sqrt(v2);
    if(v<.015)return false;
    if(!mask)return true;
    const double dot=q.dx*q.ddx+q.dy*q.ddy,acceleration=dot/v;
    if(mask&kInverseSpeed)result.speed=speedInverse(m,v,acceleration);
    if(mask&kInverseSteering){
      FlatReconstruction r;
      steeringInverse(q,m,v2,v,dot,acceleration,r);
      result.steering=r.feedforward.steering;
    }
    return true;
  }

 private:
  static constexpr double kInverseRange=65536.;
  static double speedInverse(const Model& m,double speed,double acceleration){
    return (speed+m.speed_tau*(acceleration-m.longitudinal_residual))/m.speed_gain;
  }
  static void steeringInverse(const FlatOutputJet& q,const Model& m,double v2,
      double v,double dot,double acceleration,FlatReconstruction& r){
    const double cross=q.dx*q.ddy-q.dy*q.ddx;
    r.curvature=cross/(v2*v);
    r.yaw_rate=cross/v2;
    const double curvature_rate=(q.dx*q.dddy-q.dy*q.dddx)/(v2*v)
                                -3.*cross*dot/(v2*v2*v);
    const double effective_curvature=(r.curvature-m.yaw_residual/v)/m.yaw_gain;
    const double effective_rate=(curvature_rate+m.yaw_residual*acceleration/v2)/m.yaw_gain;
    r.state.steering=std::atan(m.wheelbase*effective_curvature);
    r.steering_rate=m.wheelbase*effective_rate/
                    (1.+m.wheelbase*m.wheelbase*effective_curvature*effective_curvature);
    r.feedforward.steering=(r.state.steering+m.steering_tau*r.steering_rate)/m.steering_gain;
  }
  template <bool IncludeHeading>
  static FlatReconstruction reconstructJet(const FlatOutputJet& q,const Model& m,
                                           double minimum_speed) {
    FlatReconstruction r;
    const double data[]={q.x,q.y,q.dx,q.dy,q.ddx,q.ddy,q.dddx,q.dddy,
                         m.wheelbase,m.steering_tau,m.speed_tau,m.steering_gain,m.speed_gain,m.yaw_gain,m.yaw_residual,m.longitudinal_residual,minimum_speed};
    for(double x:data) if(!std::isfinite(x)){r.reason="nonfinite_flat_jet";return r;}
    if(m.wheelbase<=0. || m.steering_tau<0. || m.speed_tau<0. ||
       m.steering_gain<=0. || m.speed_gain<=0. || m.yaw_gain<=0. || minimum_speed<=0.) {
      r.reason="invalid_flat_model";return r;
    }
    const double v2=q.dx*q.dx+q.dy*q.dy,v=std::sqrt(v2);
    if(v<minimum_speed){r.reason="flatness_low_speed_singularity";return r;}
    const double dot=q.dx*q.ddx+q.dy*q.ddy;
    r.state.x=q.x;r.state.y=q.y;
    if(IncludeHeading)r.state.yaw=std::atan2(q.dy,q.dx);
    r.state.speed=v;r.state.acceleration=dot/v;
    // Frozen response-model feedforward inversion. Gain=1/residual=0 recovers
    // the strict nominal bicycle map. Unknown tire/slip dynamics are not flat.
    steeringInverse(q,m,v2,v,dot,r.state.acceleration,r);
    r.jerk=(q.ddx*q.ddx+q.ddy*q.ddy+q.dx*q.dddx+q.dy*q.dddy)/v-dot*dot/(v2*v);
    // Inverse nominal first-order actuators, BEFORE constraints; explicit delay
    // compensation is done by the NMPC rollout rather than this local formula.
    r.feedforward.speed=speedInverse(m,v,r.state.acceleration);
    r.valid=std::isfinite(r.jerk)&&std::isfinite(r.feedforward.speed)&&
            std::isfinite(r.feedforward.steering);
    r.reason=r.valid?"nominal_rear_flat_reconstruction":"nonfinite_flat_reconstruction";
    return r;
  }

 public:
  // Research adapter for geometry-only paths whose curvature field is absent
  // (all zero) despite changing tangent headings. This derives d(yaw)/ds from
  // the supplied LOCAL geometry; it is neither a physical curvature sensor nor
  // evidence of a measured vehicle response. Already supplied nonzero curvature
  // is preserved in its entirety. Invalid/duplicate stations leave input intact.
  // Call before cropping to the nearest point so its derivative has both sides.
  static bool recoverMissingSpatialCurvature(std::vector<ReferencePoint>& spatial){
    if(spatial.size()<2)return false;
    bool missing_curvature=true,changing_heading=false;
    for(std::size_t i=0;i<spatial.size();++i){const auto& p=spatial[i];
      if(!std::isfinite(p.s)||!std::isfinite(p.x)||!std::isfinite(p.y)||
         !std::isfinite(p.yaw)||!std::isfinite(p.curvature)||
         (i&&p.s<=spatial[i-1].s))return false;
      if(p.curvature!=0.)missing_curvature=false;
      if(i&&wrap(p.yaw-spatial[i-1].yaw)!=0.)changing_heading=true;
    }
    if(!missing_curvature||!changing_heading)return false;
    std::vector<double> curvature(spatial.size());
    auto slope=[&](std::size_t left,std::size_t right){
      return wrap(spatial[right].yaw-spatial[left].yaw)/(spatial[right].s-spatial[left].s);
    };
    curvature.front()=slope(0,1);curvature.back()=slope(spatial.size()-2,spatial.size()-1);
    for(std::size_t i=1;i+1<spatial.size();++i){
      const double left=spatial[i].s-spatial[i-1].s,right=spatial[i+1].s-spatial[i].s;
      // Derivative of the local quadratic interpolant on a NONuniform grid.
      // Adjacent wrapped increments avoid a false 2*pi spike at yaw wrapping.
      curvature[i]=(right*slope(i-1,i)+left*slope(i,i+1))/(left+right);
    }
    for(double value:curvature)if(!std::isfinite(value))return false;
    for(std::size_t i=0;i<spatial.size();++i)spatial[i].curvature=curvature[i];
    return true;
  }

  // Spatial body-center reference -> explicitly SYNTHESIZED nominal time law.
  // It does not turn a planner geometry-only message into a measured time law.
  // Input begins at the caller's selected closest path station. yaw is tangent.
  static Reference buildTimeReference(const std::vector<ReferencePoint>& spatial,
      double initial_speed,double target_speed,double stamp,const SolverConfig& c,
      bool braking=false,bool speed_preview=false,double initial_acceleration=0.,
      bool synchronized_preview=false,bool terminal_approach=false,
      double initial_body_yaw=missing()) {
    Reference out;out.stamp=stamp;out.braking=braking;
    if(spatial.size()<2 || c.steps<2 || c.steps>kMaxHorizon ||
       !std::isfinite(initial_speed)||!std::isfinite(target_speed)||
       !std::isfinite(stamp)||!std::isfinite(c.dt)||c.dt<=0. ||
       !std::isfinite(c.limits.acceleration)||c.limits.acceleration<=0. ||
       !std::isfinite(c.limits.deceleration)||c.limits.deceleration<=0. ||
       !std::isfinite(c.limits.speed)||c.limits.speed<=0. ||
       !std::isfinite(c.limits.jerk)||c.limits.jerk<=0.) {
      out.reason="invalid_spatial_reference_or_schedule";return out;
    }
    double last_s=spatial.front().s;
    for(std::size_t i=0;i<spatial.size();++i){
      const auto& p=spatial[i];
      if(!std::isfinite(p.s)||!std::isfinite(p.x)||!std::isfinite(p.y)||
         !std::isfinite(p.yaw)||!std::isfinite(p.curvature)||
         (i && p.s<=last_s)){out.reason="invalid_spatial_geometry";return out;}
      last_s=p.s;
    }
    const bool preview=speed_preview&&!braking;
    bool sync=preview&&synchronized_preview;
    bool actuator_aligned=false;
    // Backward spatial envelope: w_i^2 <= w_(i+1)^2 + 2 b ds.
    // Use the original lateral/deceleration limits; no tyre-friction estimate.
    // Bounded stack storage, nothing added to the eight online variables.
    constexpr std::size_t kMaxSpatial=1024;
    std::array<double,kMaxSpatial> squared_cap,segment_cap,body_beta; // only [0,spatial.size()) is written/read
    const double cruise=clamp(target_speed,0.,c.limits.speed);
    if(preview){
      if(spatial.size()>kMaxSpatial||!std::isfinite(initial_acceleration)||
         !std::isfinite(c.model.speed_delay)||c.model.speed_delay<0.||
         !std::isfinite(c.model.speed_tau)||c.model.speed_tau<=0.||
         !std::isfinite(c.limits.lateral_acceleration)||c.limits.lateral_acceleration<=0.||
         !std::isfinite(c.limits.steering_rate)||c.limits.steering_rate<=0.||
         !std::isfinite(c.model.wheelbase)||c.model.wheelbase<=0.||
         !std::isfinite(c.model.rear_to_center)||c.model.rear_to_center<0.||
         !std::isfinite(c.model.yaw_gain)||c.model.yaw_gain<=0.||
         (sync&&(!std::isfinite(c.limits.steering)||c.limits.steering<=0.||c.limits.steering>=1.3))){
        out.reason="invalid_preview_model_or_spatial_capacity";return out;
      }
      // Select one schedule for the whole suffix. Mixing a less conservative
      // lift on the straight approach with the old restriction only at the
      // tight apex can remove the braking lead needed by the old chart.
      const double kr_limit=sync ? c.model.yaw_gain*std::tan(c.limits.steering)/c.model.wheelbase:0.;
      const double center_limit=kr_limit/std::hypot(1.,c.model.rear_to_center*kr_limit);
      // Lift the same center path as the geometric steering-rate limit,
      // starting at the causally propagated body heading. Under the nominal
      // no-slip model d(psi)/ds_center=sin(tangent-psi)/ell. A midpoint step
      // yields the demand at both ends of each confirmed segment. The old
      // linearized steady-curvature lift remains available outside this domain.
      const double planning_speed=c.limits.speed;
      const double brake_lead=c.model.speed_delay+c.model.speed_tau+
          (std::max(0.,initial_acceleration)+c.limits.deceleration)/c.limits.jerk;
      const double terminal_guard_distance=planning_speed*(c.dt*c.steps+brake_lead)+
          planning_speed*planning_speed/(2.*c.limits.deceleration);
      const bool terminal_in_view=terminal_approach&&
          spatial.back().s-spatial.front().s<=terminal_guard_distance;
      bool approaching_bend=false;
      if(sync&&!terminal_in_view&&std::isfinite(initial_body_yaw)){
        // The curve option uses the smooth study's straight/curve analysis
        // split. Use the segment heading change, not an isolated curvature
        // sample or the experiment's route label.
        constexpr double kBendCurvature=.12; // m^-1
        const double visible_arc=std::max(initial_speed,cruise)*(c.dt*c.steps+
          std::max(c.model.speed_delay+c.model.speed_tau,c.model.steering_delay+c.model.steering_tau));
        for(std::size_t i=1;i<spatial.size()&&spatial[i-1].s-spatial.front().s<visible_arc;++i){
          const auto& a=spatial[i-1];const auto& b=spatial[i];
          if(std::abs(wrap(b.yaw-a.yaw))>=kBendCurvature*(b.s-a.s)){
            approaching_bend=true;break;
          }
        }
      }
      if(approaching_bend&&
         c.model.rear_to_center>1e-6&&std::abs(c.model.yaw_residual)<1e-12&&
         std::abs(c.model.lateral_residual)<1e-12&&
         std::isfinite(c.model.steering_delay)&&c.model.steering_delay>=0.&&
         std::isfinite(c.model.steering_tau)&&c.model.steering_tau>=0.){
        const double ell=c.model.rear_to_center;
        const double ratio=c.model.wheelbase/(ell*c.model.yaw_gain);
        double body=initial_body_yaw;
        actuator_aligned=true;
        for(std::size_t i=0;i<spatial.size();++i){
          const double beta=wrap(spatial[i].yaw-body);
          if(std::abs(beta)>.5){actuator_aligned=false;break;}
          const double delta=std::atan(ratio*std::tan(beta));
          if(std::abs(delta)>c.limits.steering+1e-7){actuator_aligned=false;break;}
          body_beta[i]=delta;
          if(i+1<spatial.size()){
            const auto& a=spatial[i];const auto& b=spatial[i+1];
            const double ds=b.s-a.s;
            if(ds>ell){actuator_aligned=false;break;}
            const double course=a.yaw+.5*wrap(b.yaw-a.yaw);
            const double half=body+.5*ds*std::sin(beta)/ell;
            body+=ds*std::sin(course-half)/ell;
          }
        }
      }
      if(sync&&!actuator_aligned){
        for(const auto& p:spatial)if(std::abs(p.curvature)>=center_limit){sync=false;break;}
      }
      if(sync&&!actuator_aligned&&c.model.rear_to_center>1e-6){
        const double ell=c.model.rear_to_center;
        body_beta[0]=std::asin(clamp(ell*spatial.front().curvature,-.95,.95));
        for(std::size_t i=1;i<spatial.size();++i){
          const auto& a=spatial[i-1];const auto& b=spatial[i];const double ds=b.s-a.s;
          const double k=wrap(b.yaw-a.yaw)/ds;
          // Exact spatial integration of the linearized lift beta'=k-beta/ell.
          // Used ONLY to condition the speed preview; the seed and forward
          // constraint simulation below use the nonlinear model.
          body_beta[i]=k*ell+(body_beta[i-1]-k*ell)*std::exp(-ds/ell);
        }
      }
      squared_cap[spatial.size()-1]=0.;
      for(std::size_t j=spatial.size()-1;j>0;--j){
        const double ds=spatial[j].s-spatial[j-1].s;
        // Bound the WHOLE interpolated segment using its maximum |curvature|.
        const double k=std::max(std::abs(spatial[j-1].curvature),
                                std::abs(spatial[j].curvature));
        const double curve=k>1e-9?c.limits.lateral_acceleration/k:cruise*cruise;
        // For the steady center->rear lift, |d(delta)/ds| is bounded
        // by (L / g_y) |d(k)/ds| / (1 - (ell*k_max)^2)^(3/2).
        // At sharp transitions this constraint can bind before v^2*k.
        const double lk=c.model.rear_to_center*k;
        const double dk=std::abs(spatial[j].curvature-spatial[j-1].curvature)/ds;
        const double factor=std::max(0.,1.-lk*lk);
        double by_rate=dk>1e-9? c.limits.steering_rate*c.model.yaw_gain*factor*std::sqrt(factor)/
          (c.model.wheelbase*dk):cruise;
        if(actuator_aligned){
          const double ddelta=std::abs(body_beta[j]-body_beta[j-1])/ds;
          // The planning speed follows center arc while the delayed rollout
          // checks actual steer and steer rate in rear-axle state coordinates.
          by_rate=ddelta>1e-9?.98*c.limits.steering_rate/ddelta:cruise;
        }else if(sync&&c.model.rear_to_center>1e-6&&factor>.09){
          // d[atan(L tan(beta)/(ell*g))]/ds. A change in center curvature
          // changes beta', not beta itself. Use the larger of the endpoint
          // rates, with the measured geometry tangent on this whole segment.
          const double ell=c.model.rear_to_center;
          const double k_tangent=wrap(spatial[j].yaw-spatial[j-1].yaw)/ds;
          const double ratio=c.model.wheelbase/(ell*c.model.yaw_gain);
          auto slope=[&](double beta){
            const double cb=std::cos(beta),sb=std::sin(beta);
            return ratio*std::abs(k_tangent-sb/ell)/(cb*cb+ratio*ratio*sb*sb);
          };
          const double ddelta=std::max(slope(body_beta[j-1]),slope(body_beta[j]));
          // No standstill spuriously introduced just because one input path
          // point was sampled more densely on the same centerline.
          by_rate=ddelta>1e-9?c.limits.steering_rate/ddelta:cruise;
        }
        segment_cap[j-1]=std::min({cruise*cruise,curve,by_rate*by_rate});
        squared_cap[j-1]=std::min(segment_cap[j-1],squared_cap[j]+2.*c.limits.deceleration*ds);
      }
    }
    const double end_s=spatial.back().s;
    const double length=end_s-spatial.front().s;
    const double physical_horizon=c.dt*c.steps;
    // Only near a finite endpoint, on a suffix with a consistent end
    // tangent. A tangent half-plane is not meaningful on a folded route.
    const bool terminal=preview&&terminal_approach&&length<=
      std::max(initial_speed,cruise)*(physical_horizon+c.model.speed_delay+c.model.speed_tau);
    if(terminal){
      const auto& end=spatial.back();
      const double co=std::cos(end.yaw),si=std::sin(end.yaw);
      bool monotone=true;double previous=-INFINITY;
      for(const auto& p:spatial){
        const double along=(p.x-end.x)*co+(p.y-end.y)*si;
        if(along<previous-1e-6||std::cos(p.yaw-end.yaw)<.5){monotone=false;break;}
        previous=along;
      }
      if(monotone){out.terminal_approach=true;out.endpoint=end;}
    }
    double s=spatial.front().s,v=clamp(initial_speed,0.,c.limits.speed),a=0.;
    std::size_t segment=0,look_segment=0;
    if(preview)a=clamp(initial_acceleration,-c.limits.deceleration,c.limits.acceleration);
    for(int i=0;i<=c.steps;++i){
      if(s>=end_s-1e-9&&v>.015){
        if(!out.terminal_approach){out.reason="reference_horizon_exhausted_while_moving";return out;}
        // Saturate the *reference*, not the predicted vehicle: its held goal
        // is the last known pose at zero speed. Every actual state still has
        // to remain before the endpoint under the unmodified delayed rollout.
        v=0.;a=0.;
      }
      while(segment+2<spatial.size() && spatial[segment+1].s<s) ++segment;
      const auto& p=spatial[segment];const auto& q=spatial[segment+1];
      const double f=clamp((s-p.s)/(q.s-p.s),0.,1.);
      auto& r=out.points[i];r.t=i*c.dt;r.s=s;
      r.x=p.x+f*(q.x-p.x);r.y=p.y+f*(q.y-p.y);
      r.yaw=wrap(p.yaw+f*wrap(q.yaw-p.yaw));
      r.curvature=p.curvature+f*(q.curvature-p.curvature);
      r.speed=v;r.acceleration=a;
      const double stop_bound=std::sqrt(std::max(0.,2.*c.limits.deceleration*(end_s-s)));
      double wanted=braking?0.:std::min(cruise,stop_bound);
      if(preview){
        // Query future spatial limits before actuator actuation. The time
        // allowance covers the configured dead time, one time constant and
        // the current acceleration-to-deceleration jerk ramp. This is a
        // planning margin, not a physical certificate for an arbitrary plant.
        const double speed_lead=c.model.speed_delay+c.model.speed_tau+
            (std::max(0.,a)+c.limits.deceleration)/c.limits.jerk;
        // Start the speed adjustment before the slower steering response
        // reaches the segment that needs it. Both channels retain their
        // separate dead times in the nonlinear constraint rollout.
        const double steering_lead=c.model.speed_delay+c.model.steering_delay+c.model.steering_tau;
        const double lead=actuator_aligned?std::max(speed_lead,steering_lead):speed_lead;
        const double query=std::min(end_s,s+v*(lead+c.dt));
        // v may decrease; keep bounded lookup correct even if query moves back.
        while(look_segment>0&&spatial[look_segment].s>query)--look_segment;
        while(look_segment+2<spatial.size()&&spatial[look_segment+1].s<query)++look_segment;
        const double x0=spatial[look_segment].s,x1=spatial[look_segment+1].s;
        const double alpha=clamp((query-x0)/(x1-x0),0.,1.);
        const double w0=squared_cap[look_segment],w1=squared_cap[look_segment+1];
        // A narrow upcoming turn-rate restriction must not disappear as soon
        // as the far query goes beyond it. Retain restrictions along the
        // entire preview interval, using the backward envelope at each node.
        double cap2=std::min(segment_cap[look_segment],(1.-alpha)*w0+alpha*w1);
        for(std::size_t k=segment;k<look_segment;++k)
          cap2=std::min({cap2,segment_cap[k],squared_cap[k+1]});
        wanted=std::min(wanted,std::sqrt(std::max(0.,cap2)));
        // Return acceleration toward zero as the target is approached,
        // including when the target crossed the current planned speed.
        if(a>0.&&wanted>=v)wanted=std::max(v,wanted-a*a/(2.*c.limits.jerk));
        if(a<0.&&wanted<=v)wanted=std::min(v,wanted+a*a/(2.*c.limits.jerk));
      }
      double next_a=clamp((wanted-v)/c.dt,-c.limits.deceleration,c.limits.acceleration);
      next_a=clamp(next_a,a-c.limits.jerk*c.dt,a+c.limits.jerk*c.dt);
      const double next_v=clamp(v+next_a*c.dt,0.,c.limits.speed);
      s=std::min(end_s,s+.5*(v+next_v)*c.dt);a=(next_v-v)/c.dt;v=next_v;
    }
    if(actuator_aligned)out.initial_body_yaw=initial_body_yaw;
    out.count=c.steps+1;out.valid=true;out.speed_preview=preview;out.synchronized_preview=sync;
    out.reason=actuator_aligned?"synthesized_actuator_aligned_preview":sync?"synthesized_synchronized_preview":
      (preview?"synthesized_curvature_braking_preview":"synthesized_nominal_body_time_law");
    return out;
  }
};
}
