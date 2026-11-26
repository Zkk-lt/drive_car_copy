#pragma once
#include "research_types.h"
#include <cstdint>

namespace research_control {
// Pure research admission check. This header has no publisher, ROS dependency,
// parameter writes, history mutation or actuator authority. A caller-supplied
// authorization is evidence to check, never authorization created by this API.
// The existing solver's integration-substep checks remain mandatory: endpoint
// checks below cannot certify constraints between nodes or real vehicle motion.
struct CandidateProvenance {
 std::uint64_t cycle=0,reference_revision=0,configuration_revision=0;
 double snapshot_ros=missing(),snapshot_steady=missing(),started_steady=missing(),finished_steady=missing();
};
struct CandidateAdmission {
 bool authorized=false,safety_stop=false,reference_braking=false;
 bool local_frame_verified=false,measured_state_verified=false,history_verified=false;
 bool solver_substep_constraints_checked=false;
 std::uint64_t cycle=0,reference_revision=0,configuration_revision=0;
 double now_ros=missing(),now_steady=missing(),maximum_snapshot_age=.05,maximum_history_gap=.05;
 double maximum_reference_age=1.2,maximum_solve_ms=8.;
 Command actual_previous;
 double actual_previous_acceleration=0.;
 // Must come from the owner's immutable safety contract, never the candidate.
 Limits hard_limits;
};
struct CandidateValidation {
 bool accepted=false;
 const char* reason="not_checked";
};

class NMPCandidateValidator {
 static bool finite(double x){return std::isfinite(x);}
 static bool command(const Command& u){return finite(u.speed)&&finite(u.steering);}
 static bool state(const State& s){return finite(s.x)&&finite(s.y)&&finite(s.yaw)&&
   finite(s.speed)&&finite(s.steering)&&finite(s.acceleration);}
 static bool limits(const Limits& l){return finite(l.speed)&&l.speed>=0.&&
   finite(l.steering)&&l.steering>0.&&finite(l.steering_rate)&&l.steering_rate>0.&&
   finite(l.acceleration)&&l.acceleration>0.&&finite(l.deceleration)&&l.deceleration>0.&&
   finite(l.jerk)&&l.jerk>0.&&finite(l.lateral_acceleration)&&l.lateral_acceleration>0.;}
 static bool narrower(const Limits& a,const Limits& b){return a.speed<=b.speed&&
   a.steering<=b.steering&&a.steering_rate<=b.steering_rate&&
   a.acceleration<=b.acceleration&&a.deceleration<=b.deceleration&&
   a.jerk<=b.jerk&&a.lateral_acceleration<=b.lateral_acceleration;}
 static bool same(double a,double b){return finite(a)&&finite(b)&&std::fabs(a-b)<=1e-9;}
 static bool same(const Command& a,const Command& b){return same(a.speed,b.speed)&&same(a.steering,b.steering);}
 static CandidateValidation reject(const char* reason){return {false,reason};}
public:
 static CandidateValidation validate(const SolveRequest& r,const SolverConfig& c,
    const SolveResult& s,const CandidateProvenance& p,const CandidateAdmission& a){
   // Safety output remains with the existing S8 safety branch. In particular,
   // normal comfort jerk checks may never postpone an emergency stop.
   if(a.safety_stop||a.reference_braking)return reject("existing_safety_owner");
   if(!a.authorized)return reject("vehicle_authority_not_authorized");
   if(!a.local_frame_verified||!a.measured_state_verified)return reject("local_state_unverified");
   if(!a.history_verified)return reject("actual_command_history_unverified");
   if(!a.solver_substep_constraints_checked)return reject("substep_constraint_evidence_missing");
   if(!limits(c.limits)||!limits(a.hard_limits)||!narrower(c.limits,a.hard_limits))
     return reject("safety_limits_invalid_or_relaxed");
   if(!finite(a.maximum_solve_ms)||a.maximum_solve_ms<=0.||!finite(c.budget_ms)||
      c.budget_ms<=0.||c.budget_ms>a.maximum_solve_ms||
      !finite(a.maximum_snapshot_age)||a.maximum_snapshot_age<=0.||
      !finite(a.maximum_history_gap)||a.maximum_history_gap<=0.||
      !finite(a.maximum_reference_age)||a.maximum_reference_age<=0.)
     return reject("admission_time_limits_invalid");
   if(!finite(a.now_steady)||!finite(p.snapshot_steady)||!finite(p.started_steady)||
      !finite(p.finished_steady)||p.snapshot_steady>p.started_steady||
      p.started_steady>p.finished_steady||p.finished_steady>a.now_steady)
     return reject("noncausal_monotonic_timestamps");
   if(a.now_steady-p.snapshot_steady>a.maximum_snapshot_age)
     return reject("candidate_snapshot_stale");
   if(p.cycle!=a.cycle||p.reference_revision!=a.reference_revision||
      p.configuration_revision!=a.configuration_revision)
     return reject("cycle_reference_or_configuration_revision_mismatch");
   if(!finite(r.now)||!same(p.snapshot_ros,r.now)||!finite(a.now_ros)||a.now_ros<r.now||
      !r.reference.valid||r.reference.braking||!finite(r.reference.stamp)||
      r.reference.stamp>r.now||a.now_ros-r.reference.stamp>a.maximum_reference_age||
      r.reference.count<2||r.reference.count>kMaxHorizon+1)
     return reject("reference_invalid_stale_or_future");
   if(!state(r.state)||!command(r.previous)||!command(r.baseline)||
      !same(r.previous,a.actual_previous)||
      !same(r.previous_command_acceleration,a.actual_previous_acceleration)||
      !finite(r.control_dt)||r.control_dt<.001||r.control_dt>.25||
      !finite(r.speed_cap)||r.speed_cap<0.||
      !finite(c.dt)||c.dt<=0.||c.steps<1||c.steps>kMaxHorizon)
     return reject("state_previous_or_grid_invalid");
   double reference_time=-1.;
   for(int i=0;i<r.reference.count;++i){
     const auto& q=r.reference.points[i];
     if(!finite(q.t)||!finite(q.s)||!finite(q.x)||!finite(q.y)||!finite(q.yaw)||
        !finite(q.curvature)||!finite(q.speed)||!finite(q.acceleration)||
        q.t<0.||q.t<=reference_time||q.speed<0.)return reject("reference_samples_invalid");
     reference_time=q.t;
   }
   if(reference_time+1e-6<(c.steps-1)*c.dt)return reject("reference_horizon_incomplete");
   if(!finite(c.model.wheelbase)||c.model.wheelbase<=0.||
      !finite(c.model.yaw_gain)||c.model.yaw_gain<=0.||!finite(c.model.yaw_residual)||
      !finite(c.model.rear_to_center)||c.model.rear_to_center<0.||
      !finite(c.model.steering_tau)||c.model.steering_tau<.01||
      !finite(c.model.speed_tau)||c.model.speed_tau<.01||
      !finite(c.model.longitudinal_residual)||!finite(c.model.lateral_residual))
     return reject("prediction_model_invalid");
   const double delay=std::max(c.model.steering_delay,c.model.speed_delay);
   if(!finite(c.model.steering_delay)||c.model.steering_delay<0.||
      !finite(c.model.speed_delay)||c.model.speed_delay<0.||r.history.size()>512)
     return reject("delay_or_history_invalid");
   double last=-std::numeric_limits<double>::infinity();
   for(const auto& h:r.history){
     if(!finite(h.stamp)||!command(h.command)||h.stamp<=last||h.stamp>r.now||
        (finite(last)&&h.stamp-last>a.maximum_history_gap))
       return reject("actual_command_history_noncausal_or_gapped");
     last=h.stamp;
   }
   if(delay>0.&&(r.history.empty()||r.history.front().stamp>r.now-delay||
      r.now-r.history.back().stamp>a.maximum_history_gap))return reject("delay_history_incomplete");
   const double measured_ms=1000.*(p.finished_steady-p.started_steady);
   if(s.timed_out||!finite(s.solve_ms)||s.solve_ms<0.||s.solve_ms>=c.budget_ms||
      measured_ms>=c.budget_ms)return reject("solver_wall_budget_exceeded");
   if(!s.ok||!s.feasible||!finite(s.objective)||!finite(s.max_violation)||
      s.max_violation<0.||s.max_violation>1e-7)return reject("solver_not_feasible");
   if(s.count!=c.steps+1||!command(s.command)||!same(s.command,s.controls[0])||
      !finite(s.command_acceleration))return reject("solution_shape_or_first_command_invalid");
   if(!same(s.predicted[0].x,r.state.x)||!same(s.predicted[0].y,r.state.y)||
      !same(s.predicted[0].yaw,r.state.yaw)||!same(s.predicted[0].speed,r.state.speed)||
      !same(s.predicted[0].steering,r.state.steering)||
      !same(s.predicted[0].acceleration,r.state.acceleration))
     return reject("prediction_initial_state_mismatch");
   constexpr double tolerance=1e-7;
   const double cap=std::min(r.speed_cap,c.limits.speed);
   Command previous=r.previous;double previous_acceleration=r.previous_command_acceleration;
   for(int i=0;i<c.steps;++i){
     const Command& u=s.controls[i];const double dt=i==0?r.control_dt:c.dt;
     if(!command(u)||u.speed<-tolerance||u.speed>cap+tolerance||
        std::fabs(u.steering)>c.limits.steering+tolerance)
       return reject("command_absolute_constraint_violation");
     const double acceleration=(u.speed-previous.speed)/dt;
     if(acceleration>c.limits.acceleration+tolerance||acceleration<-c.limits.deceleration-tolerance||
        std::fabs(u.steering-previous.steering)>c.limits.steering_rate*dt+tolerance||
        std::fabs(acceleration-previous_acceleration)>c.limits.jerk*dt+tolerance)
       return reject("command_transition_constraint_violation");
     if(i==0&&!same(acceleration,s.command_acceleration))
       return reject("first_command_acceleration_mismatch");
     previous=u;previous_acceleration=acceleration;
   }
   for(int i=0;i<s.count;++i){
     const State& x=s.predicted[i];
     if(!state(x))return reject("prediction_nonfinite");
     const double yaw_rate=c.model.yaw_gain*x.speed*std::tan(x.steering)/c.model.wheelbase+c.model.yaw_residual;
     if(x.speed<-tolerance||x.speed>cap+tolerance||std::fabs(x.steering)>c.limits.steering+tolerance||
        x.acceleration>c.limits.acceleration+tolerance||x.acceleration<-c.limits.deceleration-tolerance||
        std::fabs(x.speed*yaw_rate)>c.limits.lateral_acceleration+tolerance)
       return reject("prediction_endpoint_constraint_violation");
   }
   return {true,"candidate_valid_for_explicit_experimental_owner"};
 }
};
}
