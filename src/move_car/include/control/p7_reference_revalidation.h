#pragma once
#include "control/reduced_nmpc.h"

namespace research_control {
// No new optimization. A first-step timing correction is allowed only when
// the whole delayed trajectory passes validation again on the current reference.
class P7ReferenceRevalidation {
 public:
  struct Result {
    bool valid=false,timed_out=false;double elapsed_ms=0.,position_change=0.,heading_change=0.;
    int rejected_step=-1;
    std::string reason="not_revalidated";SolveResult rollout;
  };
  struct Workspace {
    std::vector<ReferencePoint> previous_suffix,latest_suffix;
  };
  struct FrameTransform {
    std::array<double,3> source,current;
    double co,si,cn,sn;
    FrameTransform(const std::array<double,3>& from,const std::array<double,3>& to)
        :source(from),current(to),co(std::cos(from[2])),si(std::sin(from[2])),
         cn(std::cos(to[2])),sn(std::sin(to[2])){}
    ReferencePoint apply(ReferencePoint p) const {
      const double x=p.x+.30,y=p.y;
      const double dx=source[0]+co*x-si*y-current[0],dy=source[1]+si*x+co*y-current[1];
      p.x=cn*dx+sn*dy-.30;p.y=-sn*dx+cn*dy;
      p.yaw=wrap(p.yaw+source[2]-current[2]);
      return p;
    }
  };
  static ReferencePoint transportReference(ReferencePoint p,
      const std::array<double,3>&source,const std::array<double,3>&current){
    return FrameTransform(source,current).apply(p);
  }
  // Both suffixes start at the projection of the same measured body center.
  // Compare the piecewise-linear paths at equal distance from that anchor.
  // Sweeping both sets of knots bounds the position error over each interval.
  static bool compareSpatial(const std::vector<ReferencePoint>& previous,
      const std::vector<ReferencePoint>& latest,double extent,Result& result){
    if(previous.size()<2||latest.size()<2)return false;
    const double old_start=previous.front().s,new_start=latest.front().s;
    constexpr double tolerance=1e-8;
    if(!std::isfinite(extent)||extent<0.||
       extent>std::min(previous.back().s-old_start,latest.back().s-new_start)+tolerance)return false;
    auto sample=[](const std::vector<ReferencePoint>& path,std::size_t upper,double s){
      const auto& a=path[upper-1];const auto& b=path[upper];
      const double f=clamp((s-a.s)/(b.s-a.s),0.,1.);
      ReferencePoint point;
      point.x=a.x+f*(b.x-a.x);point.y=a.y+f*(b.y-a.y);
      point.yaw=a.yaw+f*wrap(b.yaw-a.yaw);
      return point;
    };
    std::size_t i=1,j=1;
    double distance=0.,last_old=previous.front().yaw,last_new=latest.front().yaw;
    double yaw_difference=wrap(last_old-last_new);
    result.heading_change=std::abs(yaw_difference);
    for(;;){
      const auto a=sample(previous,i,old_start+distance),b=sample(latest,j,new_start+distance);
      result.position_change=std::max(result.position_change,std::hypot(a.x-b.x,a.y-b.y));
      // Accumulate the unwrapped difference to also detect opposite heading
      // changes within an interval whose endpoints agree modulo 2*pi.
      yaw_difference+=wrap(a.yaw-last_old)-wrap(b.yaw-last_new);
      result.heading_change=std::max(result.heading_change,std::abs(yaw_difference));
      if(distance>=extent)return true;
      last_old=a.yaw;last_new=b.yaw;
      if(i+1<previous.size()&&previous[i].s-old_start<=distance)++i;
      if(j+1<latest.size()&&latest[j].s-new_start<=distance)++j;
      const double old_next=i+1<previous.size()?previous[i].s-old_start:extent;
      const double new_next=j+1<latest.size()?latest[j].s-new_start:extent;
      distance=std::min({extent,old_next,new_next});
    }
  }
  static Result validate(SolveRequest current,const SolverConfig&c,
      const std::vector<ReferencePoint>&previous_reference_in_current_frame,
      const std::vector<ReferencePoint>&latest_reference,
      const std::array<Command,kMaxHorizon>&commands,
      std::chrono::steady_clock::time_point deadline,bool speed_preview=false,bool synchronized_preview=false,bool terminal_approach=false,
      double planned_dt=0.,bool actuator_aligned_preview=false,bool retime_suffix=false,bool spatial_continuity=false,
      Workspace* scratch=nullptr,bool cap_tightened=false){
    const auto start=std::chrono::steady_clock::now();Result out;
    auto finish=[&](const std::string&why){out.reason=why;out.elapsed_ms=
      std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();return out;};
    if(current.reference.braking)return finish("revalidation_braking_reference");
    Workspace local;
    auto& workspace=scratch?*scratch:local;
    auto& old_path=workspace.previous_suffix;
    auto& new_path=workspace.latest_suffix;
    if(!FlatnessReferenceModel::projectSpatialSuffix(previous_reference_in_current_frame,current.state,c.model,old_path)||
       !FlatnessReferenceModel::projectSpatialSuffix(latest_reference,current.state,c.model,new_path))return finish("revalidation_geometry_invalid");
    const double target=std::isfinite(current.target_speed)?std::min(current.target_speed,current.speed_cap):current.speed_cap;
    // P7 local acceleration is explicitly a model derivative, not an IMU
    // measurement. Recompute it with this immutable model and the actual
    // DELAYED held command, as the worker propagation does. Using the latest
    // undelayed command here would create a different prediction model.
    if(!current.history.empty()&&std::isfinite(c.model.speed_tau)&&c.model.speed_tau>0.){
      const double cutoff=current.now-c.model.speed_delay;
      const auto i=std::upper_bound(current.history.begin(),current.history.end(),cutoff,
        [](double stamp,const CommandSample&h){return stamp<h.stamp;});
      if(i!=current.history.begin())current.state.acceleration=
        (c.model.speed_gain*(i-1)->command.speed-current.state.speed)/c.model.speed_tau+c.model.longitudinal_residual;
    }
    const double initial_body_yaw=actuator_aligned_preview?current.state.yaw:missing();
    const auto old_ref=FlatnessReferenceModel::buildTimeReference(old_path,current.state.speed,target,current.now,c,false,speed_preview,current.state.acceleration,synchronized_preview,terminal_approach,initial_body_yaw);
    current.reference=FlatnessReferenceModel::buildTimeReference(new_path,current.state.speed,target,current.now,c,false,speed_preview,current.state.acceleration,synchronized_preview,terminal_approach,initial_body_yaw);
    if(!old_ref.valid||!current.reference.valid)return finish("revalidation_reference_horizon_invalid");
    if(spatial_continuity){
      const double extent=std::max(old_ref.points[old_ref.count-1].s-old_path.front().s,
          current.reference.points[current.reference.count-1].s-new_path.front().s);
      if(!compareSpatial(old_path,new_path,extent,out))return finish("revalidation_common_spatial_support_insufficient");
    }else{
      for(int i=0;i<current.reference.count;++i){const auto&a=old_ref.points[i];const auto&b=current.reference.points[i];
        out.position_change=std::max(out.position_change,std::hypot(a.x-b.x,a.y-b.y));
        out.heading_change=std::max(out.heading_change,std::abs(wrap(a.yaw-b.yaw)));
      }
    }
    // Inherited P6 local-feedback engineering domain, not a new physical
    // calibration. Larger reference revisions require a new optimization.
    if(out.position_change>.03||out.heading_change>.10)return finish("revalidation_reference_jump_outside_local_domain");
    const auto checked=ReducedNMPC::validateControls(current,c,commands,deadline,planned_dt,retime_suffix,cap_tightened);
    out.rejected_step=checked.rejected_step;
    out.timed_out=checked.timed_out;out.rollout=checked.result;out.valid=checked.feasible;
    return finish(checked.reason);
  }
};
}
