#pragma once
#include "control/flatness_reference_model.h"
#include "control/p7_turn_timing.h"
#include "research_parameterization.h"
#include <chrono>

namespace research_control {
// Fixed-memory, feasible single-shooting NMPC. Numerical reduction is 2*M
// move-blocked demand coordinates; nominal flatness supplies a separate seed.
// No flatness assertion is made for the augmented delayed/residual model.
class ReducedNMPC {
  friend class FlatnessReducedNMPC;
  friend class P6FlatnessNMPC;
  friend class P6PolicyValidator;
  friend class P6ClosedLoopAdapter;
  using Clock=std::chrono::steady_clock;
  using Decision=std::array<double,2*kMaxKnots>;
  struct Rollout {
    bool feasible=false,dominated=false;
    const char* rejection="not_evaluated";int rejected_step=-1;
    double cost=std::numeric_limits<double>::infinity(),violation=0.;
    std::array<State,kMaxHorizon+1> states{};
    std::array<Command,kMaxHorizon> commands{};
    std::array<double,kMaxHorizon> command_accelerations{};
  };
  struct Source { int index=-1; Command fixed; };
  struct Substep {
    double dt=0.,speed_decay=0.,steering_decay=0.;
    Source speed,steering;
  };
  struct FixedPrefix {
    struct Stage {
      State state;
      int end=0;
      double violation=0.;
    };
    std::array<Stage,kMaxHorizon> stages{};
    int stage_count=0;
    double speed_command=missing(),steering_command=missing();
  };
  // At most 1000 regular 10 ms subdivisions + 2*512 history events +
  // 2*40 future command events + 40 interval boundaries for allowed horizons.
  static constexpr int kMaxSubsteps=2304;
  struct Prepared {
    std::array<Substep,kMaxSubsteps> substeps{};
    std::array<int,kMaxHorizon+1> begin{};
    std::array<ReferencePoint,kMaxHorizon> reference{};
    std::array<double,kMaxHorizon> ref_sin{},ref_cos{},h{},knot_fraction{};
    std::array<double,kMaxHorizon> body_yaw_rate{};
    std::array<Command,kMaxHorizon> synchronized_demand{};
    std::array<bool,kMaxHorizon> synchronized_demand_valid{};
    double yaw_rate_weight=0.;
    // Endpoint constraint uses the already-issued snapshot, not reference
    // resampling or a modified command after optimization.
    double end_cos=0.,end_sin=0.,end_x=0.,end_y=0.,terminal_jerk_scale=1.;
    bool terminal_approach=false;
    std::array<int,kMaxHorizon> knot_left{};
    std::array<double,kMaxKnots> knot_positions{};
    int count=0;
  };
 public:
  struct DirectValidation {
    bool feasible=false,timed_out=false;double cost=missing(),violation=missing(),elapsed_ms=0.;
    const char* reason="not_validated";int rejected_step=-1;
    SolveResult result;
  };

  // Construct a new input sequence for a cap that tightened after the solve.
  // Preserve the planned steering slope at the first issue boundary and the
  // planned steering increments thereafter. Each speed step reserves space
  // to unwind its acceleration to zero at the existing jerk limit.
  static bool projectTightenedSpeed(const SolveRequest& r,const SolverConfig& c,
      const std::array<Command,kMaxHorizon>& plan,double planned_dt,
      std::array<Command,kMaxHorizon>& adjusted){
    if(!std::isfinite(planned_dt)||planned_dt<.001||
       std::abs(r.control_dt-planned_dt)>.5*planned_dt)return false;
    const double cap=std::min(c.limits.speed,r.speed_cap);
    if(r.previous.speed>cap||r.previous.speed<0.||r.state.speed>cap)return false;
    Command previous=r.previous;
    double a=r.previous_command_acceleration;
    const double steering_offset=(plan[0].steering-r.previous.steering)*(r.control_dt/planned_dt-1.);
    for(int i=0;i<c.steps;++i){
      const double h=interval(i,r,c);
      double lo,hi,dlo,dhi;
      if(!finiteCommand(plan[i])||!demandBounds(previous,a,h,cap,c.limits,lo,hi,dlo,dhi))return false;
      auto recoverable=[&](double room){
        room=std::max(0.,room);
        return 2.*room/(h+std::sqrt(h*h+2.*room/c.limits.jerk));
      };
      lo=std::max(lo,-recoverable(previous.speed));
      hi=std::min(hi,recoverable(cap-previous.speed));
      if(lo>hi)return false;
      const double wanted=i==0?previous.speed+h*(plan[0].speed-previous.speed)/planned_dt:plan[i].speed;
      const double steering=plan[i].steering+steering_offset;
      a=clamp((wanted-previous.speed)/h,lo,hi);
      const double rate=(steering-previous.steering)/h;
      if(rate<dlo-1e-10||rate>dhi+1e-10)return false;
      previous={previous.speed+h*a,steering};
      adjusted[i]=previous;
    }
    return true;
  }
  // Every version is validated across the full delayed model and constraints.
  // The optional timing correction preserves the first planned rate inside
  // the actual publication interval, with the same model and reference.
  static DirectValidation validateControls(const SolveRequest&r,const SolverConfig&c,
      const std::array<Command,kMaxHorizon>&commands,Clock::time_point deadline,double planned_dt=0.,bool retime_suffix=false,
      bool cap_tightened=false){
    const auto start=Clock::now();DirectValidation out;
    auto finish=[&](const char*reason){out.reason=reason;out.elapsed_ms=milliseconds(start);return out;};
    if(Clock::now()>=deadline){out.timed_out=true;return finish("revalidation_budget_exhausted");}
    if(!validConfig(c)||!validState(r.state)||!finiteCommand(r.previous)||
       !std::isfinite(r.now)||!std::isfinite(r.control_dt)||r.control_dt<.001||r.control_dt>.25||
       !std::isfinite(r.speed_cap)||r.speed_cap<0.||!std::isfinite(r.previous_command_acceleration)||
       !validReference(r.reference,c)||r.history.empty()||r.history.size()>512)return finish("revalidation_input_invalid");
    double last=-INFINITY;for(const auto&h:r.history){if(!std::isfinite(h.stamp)||h.stamp<=last||h.stamp>r.now||!finiteCommand(h.command))return finish("revalidation_history_invalid");last=h.stamp;}
    if(r.history.front().stamp>r.now-std::max(c.model.speed_delay,c.model.steering_delay)+1e-6)return finish("revalidation_history_incomplete");
    ReducedNMPC core;SolveProfile profile;
    if(!core.prepare(r,c,profile,false))return finish("revalidation_schedule_capacity");
    bool stopped=false;bool retimed=false;bool cap_projected=false;
    auto roll=rollout(Decision{},r,c,core.prepared_,deadline,stopped,profile,&commands);
    if(!stopped&&!roll.feasible&&roll.rejected_step==0&&
       std::string(roll.rejection)=="reconstructed_input_constraint_violation"&&
       std::isfinite(planned_dt)&&planned_dt>=.001&&std::abs(r.control_dt-planned_dt)<=.5*planned_dt){
      const double h=core.prepared_.h[0];
      double lo,hi,dlo,dhi;
      if(demandBounds(r.previous,r.previous_command_acceleration,h,std::min(c.limits.speed,r.speed_cap),
                      c.limits,lo,hi,dlo,dhi)){
        auto adjusted=commands;
        const double planned_a=(commands[0].speed-r.previous.speed)/planned_dt;
        const double planned_d=(commands[0].steering-r.previous.steering)/planned_dt;
        const double a=clamp(planned_a,lo,hi),d=clamp(planned_d,dlo,dhi);
        // Only correct a mismatch that can be explained by a difference in
        // application step. An arbitrary infeasible command stays rejected.
        const double mismatch=std::abs(h-planned_dt);
        if(std::abs(a-planned_a)<=c.limits.jerk*mismatch+1e-8&&
           std::abs(d-planned_d)<=1e-8){
          adjusted[0]={r.previous.speed+h*a,r.previous.steering+h*d};
          roll=rollout(Decision{},r,c,core.prepared_,deadline,stopped,profile,&adjusted);
          retimed=roll.feasible;
        }
      }
    }
    if(retime_suffix&&!stopped&&!roll.feasible&&roll.rejected_step<=1&&
       std::string(roll.rejection)=="reconstructed_input_constraint_violation"&&
       std::isfinite(planned_dt)&&planned_dt>=.001&&
       std::abs(r.control_dt-planned_dt)<=.5*planned_dt&&
       std::abs(r.control_dt-planned_dt)>1e-9){
      // Certify the original sequence at its proposed issue interval first.
      // Shift every subsequent absolute input by the same first-step change,
      // preserving the optimized increments, then check the entire new rollout.
      auto nominal=r;nominal.control_dt=planned_dt;
      ReducedNMPC nominal_core;SolveProfile nominal_profile;
      if(nominal_core.prepare(nominal,c,nominal_profile,false)){
        const auto certificate=rollout(Decision{},nominal,c,nominal_core.prepared_,deadline,stopped,nominal_profile,&commands);
        if(!stopped&&certificate.feasible){
          const double ratio=r.control_dt/planned_dt;
          const Command offset{(commands[0].speed-r.previous.speed)*(ratio-1.),
                               (commands[0].steering-r.previous.steering)*(ratio-1.)};
          auto adjusted=commands;
          for(int i=0;i<c.steps;++i){adjusted[i].speed+=offset.speed;adjusted[i].steering+=offset.steering;}
          auto alternative=rollout(Decision{},r,c,core.prepared_,deadline,stopped,profile,&adjusted);
          if(alternative.feasible){roll=alternative;retimed=true;}
        }
      }
    }
    if(cap_tightened&&!stopped&&!roll.feasible&&std::isfinite(roll.violation)&&
       std::string(roll.rejection)=="reconstructed_input_constraint_violation"&&Clock::now()<deadline){
      std::array<Command,kMaxHorizon> adjusted;
      if(projectTightenedSpeed(r,c,commands,planned_dt,adjusted)){
        auto alternative=rollout(Decision{},r,c,core.prepared_,deadline,stopped,profile,&adjusted);
        if(alternative.feasible){roll=alternative;cap_projected=true;}
      }
    }
    out.violation=roll.violation;out.rejected_step=roll.rejected_step;out.cost=roll.cost;
    if(stopped||Clock::now()>=deadline){out.timed_out=true;return finish("revalidation_budget_exhausted");}
    if(!roll.feasible)return finish(roll.rejection);
    out.feasible=true;auto&s=out.result;s.ok=s.feasible=true;s.command=roll.commands[0];
    s.command_acceleration=roll.command_accelerations[0];s.controls=roll.commands;s.predicted=roll.states;
    s.count=c.steps+1;s.objective=roll.cost;s.max_violation=roll.violation;
    return finish(cap_projected?"current_reference_tightened_speed_full_rollout_feasible":
                  retimed?"current_reference_retimed_full_rollout_feasible":
                          "current_reference_full_rollout_feasible");
  }
  void reset(){warm_valid_=false;warm_count_=0;}

  SolveResult solve(const SolveRequest& r,const SolverConfig& c){
    const auto start=Clock::now();
    SolveResult out;out.command=r.previous;out.decision_dimension=2*c.knots;
    auto finish=[&](const char* why){
      out.reason=why;out.solve_ms=milliseconds(start);
      if(out.ok && out.solve_ms>=c.budget_ms){
        out.ok=out.feasible=false;out.timed_out=true;out.command=r.previous;
        out.reason="solver_timeout";warm_valid_=false;
      }
      if(c.profile){
        const auto& p=out.profile;
        out.profile.other_ms=std::max(0.,out.solve_ms-p.initialization_ms-p.reference_ms-
          p.prediction_ms-p.constraint_cost_ms-p.direction_ms-p.candidate_selection_ms);
      }
      return out;
    };
    if(!validConfig(c))return finish("invalid_config");
    if(!validState(r.state)||!finiteCommand(r.previous)||!finiteCommand(r.baseline)||
       !std::isfinite(r.now)||!std::isfinite(r.control_dt)||r.control_dt<.001||
       r.control_dt>.25||!std::isfinite(r.speed_cap)||r.speed_cap<0.||
       !std::isfinite(r.previous_command_acceleration))return finish("invalid_state_or_request");
    if(!validReference(r.reference,c))return finish("reference_invalid");
    if(r.warm_hint_count!=0&&r.warm_hint_count!=c.steps)return finish("invalid_warm_hint_count");
    for(int i=0;i<r.warm_hint_count;++i)if(!finiteCommand(r.warm_hint[i]))return finish("invalid_warm_hint");
    if(r.force_numerical_failure)return finish("injected_numerical_failure");
    if(r.force_infeasible)return finish("injected_infeasible");
    if(r.force_timeout){out.timed_out=true;return finish("injected_timeout");}
    const double max_delay=std::max(c.model.steering_delay,c.model.speed_delay);
    double stamp=-std::numeric_limits<double>::infinity();
    if(r.history.size()>512)return finish("command_history_too_large");
    for(const auto& h:r.history){
      if(!std::isfinite(h.stamp)||!finiteCommand(h.command)||h.stamp<stamp||
         h.stamp>r.now+1e-6)return finish("invalid_command_history");
      stamp=h.stamp;
    }
    if(max_delay>1e-9 && (r.history.empty()||r.history.front().stamp>r.now-max_delay+1e-6))
      return finish("history_incomplete");
    const auto deadline=start+duration(c.budget_ms);
    // Reserve time for result copies and the final hard deadline check. This is
    // an internal search limit, not an increase of the caller's wall budget.
    const auto search_deadline=deadline-duration(std::min(.8,.12*c.budget_ms+.05));
    if(c.profile)out.profile.initialization_ms=milliseconds(start);
    if(!prepare(r,c,out.profile))return finish("schedule_capacity_exceeded");
    const double elapsed=r.now-warm_stamp_;
    const double warm_horizon=warm_first_interval_+(warm_count_-1)*warm_dt_;
    out.warm_available=warm_valid_ && warm_count_==c.steps &&
      std::abs(warm_dt_-c.dt)<1e-12 && elapsed>=0. && elapsed<warm_horizon;
    if(!out.warm_available)warm_valid_=false;
    Rollout best;Decision selected{};bool expired=false,search_stopped=false;
    double worst_rollout_ms=0.;
    auto evaluate=[&](const Decision& x,Rollout& roll){
      if(search_stopped)return;
      const auto tick=Clock::now();
      if(tick>=deadline){expired=true;return;}
      if(tick>=search_deadline || (best.feasible &&
         tick+duration(worst_rollout_ms*1.2)>=search_deadline)){
        search_stopped=true;out.budget_limited=true;return;
      }
      roll=rollout(x,r,c,prepared_,search_deadline,search_stopped,out.profile);
      if(search_stopped)out.budget_limited=true;
      worst_rollout_ms=std::max(worst_rollout_ms,milliseconds(tick));++out.evaluations;
      if(Clock::now()>=deadline)expired=true;
    };
    auto seedFor=[&](int kind){
      const auto tick=c.profile?Clock::now():Clock::time_point{};
      const auto x=makeSeed(r,c,kind);
      if(c.profile)out.profile.direction_ms+=milliseconds(tick);
      return x;
    };
    // Original SLQR supplies one independent feasible-searched seed. The final
    // steering command is never a sum of independent controller outputs.
    Decision seed=seedFor(0);Rollout candidate;evaluate(seed,candidate);
    out.initial_objective=candidate.feasible?candidate.cost:missing();
    best=candidate;selected=seed;
    const auto trySeed=[&](const Decision& x,bool warm){
      Rollout roll;evaluate(x,roll);
      const auto tick=c.profile?Clock::now():Clock::time_point{};
      if(!expired && !search_stopped && roll.feasible && (!best.feasible||roll.cost<best.cost)){
        best=roll;selected=x;out.warm_started=warm;
      }
      if(c.profile)out.profile.candidate_selection_ms+=milliseconds(tick);
    };
    if(out.warm_available)trySeed(seedFor(3),true);
    if(r.warm_hint_count==c.steps&&!search_stopped&&!expired)trySeed(seedFor(4),true);
    if(!search_stopped&&!expired)trySeed(seedFor(1),false);
    if(!search_stopped&&!expired)trySeed(seedFor(2),false);
    // Feasible-only deterministic coordinate pattern search. 2*M variables,
    // at most (4+4*M*iterations) rollouts, each O(N*S). Candidate-independent
    // event partitioning, delay lookup, exponentials and references are cached.
    int small_improvement_passes=0;
    for(int iteration=0;iteration<c.iterations && !expired && !search_stopped;++iteration){
      const double old_cost=best.cost,radius=.35*std::pow(.5,iteration);
      for(int j=0;j<2*c.knots && !expired && !search_stopped;++j){
        const Decision base=selected;
        for(int sign:{-1,1}){
          if(expired||search_stopped)break;
          const auto direction_start=c.profile?Clock::now():Clock::time_point{};
          Decision x=base;x[j]=clamp(base[j]+sign*radius,-1.,1.);
          if(c.profile)out.profile.direction_ms+=milliseconds(direction_start);
          if(std::abs(x[j]-base[j])<1e-12)continue;
          Rollout roll;evaluate(x,roll);
          const auto tick=c.profile?Clock::now():Clock::time_point{};
          if(!expired && !search_stopped && roll.feasible && (!best.feasible||roll.cost<best.cost)){
            best=roll;selected=x;
          }
          if(c.profile)out.profile.candidate_selection_ms+=milliseconds(tick);
        }
      }
      out.iterations=iteration+1;
      const double improvement=old_cost-best.cost;
      if(best.feasible && std::isfinite(improvement) && improvement<=1e-7*std::max(1.,std::abs(old_cost)))
        ++small_improvement_passes;
      else small_improvement_passes=0;
      if(small_improvement_passes>=2){out.early_stopped=true;break;}
    }
    if(expired || Clock::now()>=deadline){
      out.timed_out=true;out.max_violation=best.violation;
      return finish("solver_timeout");
    }
    if(!best.feasible){out.max_violation=best.violation;
      return finish(out.budget_limited?"budget_exhausted_without_feasible":"no_feasible_rollout");}
    out.ok=true;out.feasible=true;out.objective=best.cost;
    out.max_violation=best.violation;out.command=best.commands[0];
    out.command_acceleration=best.command_accelerations[0];out.count=c.steps+1;
    out.predicted=best.states;out.controls=best.commands;
    warm_controls_=best.commands;warm_count_=c.steps;warm_valid_=true;
    warm_stamp_=r.now;warm_first_interval_=r.control_dt;warm_dt_=c.dt;
    // A preempted process can exceed its wall budget even after optimization.
    // Never return an accepted result whose measured finish crossed the budget.
    if(Clock::now()>=deadline){out.ok=out.feasible=false;out.timed_out=true;warm_valid_=false;
      out.command=r.previous;return finish("solver_timeout");}
    return finish(out.budget_limited?"feasible_budget_stop":
                  (out.early_stopped?"feasible_converged_pattern_search":"feasible_pattern_search"));
  }

 private:
  Prepared prepared_;
  std::array<Command,kMaxHorizon> warm_controls_{};
  bool warm_valid_=false;int warm_count_=0;
  double warm_stamp_=0.,warm_first_interval_=0.,warm_dt_=0.;
  static Clock::duration duration(double ms){
    return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double,std::milli>(ms));
  }
  static double milliseconds(Clock::time_point start){
    return std::chrono::duration<double,std::milli>(Clock::now()-start).count();
  }
  static bool finiteCommand(const Command& u){return std::isfinite(u.speed)&&std::isfinite(u.steering);}
  static bool validState(const State& s){
    return std::isfinite(s.x)&&std::isfinite(s.y)&&std::isfinite(s.yaw)&&
           std::isfinite(s.speed)&&std::isfinite(s.steering)&&std::isfinite(s.acceleration);
  }
  static bool validConfig(const SolverConfig& c){
    const double positive[]={c.dt,c.budget_ms,c.model.wheelbase,c.model.steering_tau,
      c.model.speed_tau,c.model.yaw_gain,c.model.steering_gain,c.model.speed_gain,c.limits.speed,c.limits.steering,
      c.limits.steering_rate,c.limits.acceleration,c.limits.deceleration,c.limits.jerk,
      c.limits.lateral_acceleration};
    for(double v:positive)if(!std::isfinite(v)||v<=0.)return false;
    const double nonnegative[]={c.model.rear_to_center,c.model.steering_delay,c.model.speed_delay,
      c.weights.lateral,c.weights.heading,c.weights.speed,c.weights.smoothness,c.yaw_rate_tracking_weight};
    for(double v:nonnegative)if(!std::isfinite(v)||v<0.)return false;
    return c.research_grid>=0&&c.research_grid<=3&&c.steps>=2&&c.steps<=kMaxHorizon&&c.knots>=2&&c.knots<=kMaxKnots&&
      c.knots<=c.steps&&c.iterations>=0&&c.iterations<=30&&c.dt>=.005&&c.dt<=.25&&
      c.limits.steering<1.3&&c.model.steering_delay<=2.&&c.model.speed_delay<=2.&&
      c.model.steering_tau>=.01&&c.model.speed_tau>=.01&&
      std::isfinite(c.model.longitudinal_residual)&&std::isfinite(c.model.yaw_residual)&&
      std::isfinite(c.model.lateral_residual)&&
      c.weights.lateral+c.weights.heading+c.weights.speed>0.;
  }
  static bool validReference(const Reference& ref,const SolverConfig& c){
    if(!ref.valid||ref.count<2||ref.count>kMaxHorizon+1||!std::isfinite(ref.stamp))return false;
    if(ref.terminal_approach&&(!std::isfinite(ref.endpoint.x)||
       !std::isfinite(ref.endpoint.y)||!std::isfinite(ref.endpoint.yaw)||ref.braking))return false;
    double t=-1.;
    for(int i=0;i<ref.count;++i){
      const auto& p=ref.points[i];
      const double fields[]={p.t,p.s,p.x,p.y,p.yaw,p.curvature,p.speed,p.acceleration};
      for(double v:fields)if(!std::isfinite(v))return false;
      if(p.t<=t||p.t<0.||p.speed<0.)return false;
      t=p.t;
    }
    return ref.points[ref.count-1].t+1e-6>=(c.steps-1)*c.dt;
  }
  static double interval(int i,const SolveRequest& r,const SolverConfig& c){return i==0?r.control_dt:c.dt;}
  static double startTime(int i,const SolveRequest& r,const SolverConfig& c){return i==0?0.:r.control_dt+(i-1)*c.dt;}
  static ReferencePoint referenceAt(const Reference& ref,double t){
    int j=0;while(j+2<ref.count&&ref.points[j+1].t<t)++j;
    const auto& a=ref.points[j];const auto& b=ref.points[j+1];
    const double f=clamp((t-a.t)/(b.t-a.t),0.,1.);ReferencePoint p;
    p.t=t;p.s=a.s+f*(b.s-a.s);p.x=a.x+f*(b.x-a.x);p.y=a.y+f*(b.y-a.y);
    p.yaw=wrap(a.yaw+f*wrap(b.yaw-a.yaw));p.curvature=a.curvature+f*(b.curvature-a.curvature);
    p.speed=a.speed+f*(b.speed-a.speed);p.acceleration=a.acceleration+f*(b.acceleration-a.acceleration);
    return p;
  }
  // The bounded chart is part of the optimization parameterization, NOT an
  // unconstrained output clipped after optimization. Every candidate is rolled
  // out and rejected if any actual-state constraint fails.
  static bool demandBounds(const Command& previous,double previous_a,double h,
      double cap,const Limits& l,double& alo,double& ahi,double& dlo,double& dhi){
    alo=std::max(-l.deceleration,std::max(previous_a-l.jerk*h,-previous.speed/h));
    ahi=std::min(l.acceleration,std::min(previous_a+l.jerk*h,(cap-previous.speed)/h));
    dlo=std::max(-l.steering_rate,(-l.steering-previous.steering)/h);
    dhi=std::min(l.steering_rate,(l.steering-previous.steering)/h);
    return alo<=ahi+1e-12&&dlo<=dhi+1e-12;
  }
  static double directDemandViolation(const Command& u,double a,double d,double previous_a,
      double h,double alo,double ahi,double dlo,double dhi,const Limits& limits){
    // Input-coordinate tolerance grows by 1/h in the physical jerk.
    const double jerk=std::abs((a-previous_a)/h);
    if(!finiteCommand(u)||a<alo-1e-10||a>ahi+1e-10||
       d<dlo-1e-10||d>dhi+1e-10||jerk>limits.jerk+1e-9)
      return std::max({1e-6,alo-a,a-ahi,dlo-d,d-dhi,jerk-limits.jerk});
    return 0.;
  }
  // This proves only the command-side bounds. A passing sequence still needs
  // the entire delayed state rollout, including terminal approach constraints.
  struct IssuedInput {
    Command command;
    double acceleration=0.,steering_rate=0.;
  };
  using IssuedInputs=std::array<IssuedInput,kMaxHorizon>;
  static bool prepareDirectInput(const Command& direct,const Command& previous,
      double previous_a,double h,double cap,const Limits& limits,IssuedInput& issued,Rollout& rejection){
    double alo,ahi,dlo,dhi;
    if(!demandBounds(previous,previous_a,h,cap,limits,alo,ahi,dlo,dhi)){
      rejection.rejection="command_speed_acceleration_jerk_bounds_empty";
      rejection.violation=std::max(alo-ahi,dlo-dhi);return false;
    }
    const double a=(direct.speed-previous.speed)/h;
    const double d=(direct.steering-previous.steering)/h;
    const double violation=directDemandViolation(direct,a,d,previous_a,h,alo,ahi,dlo,dhi,limits);
    if(violation>0.){
      rejection.rejection="reconstructed_input_constraint_violation";
      rejection.violation=violation;return false;
    }
    issued={{previous.speed+h*a,previous.steering+h*d},a,d};
    return true;
  }
  static double encode(double target,double lo,double hi){
    return hi-lo<1e-12?0.:clamp(2.*(target-lo)/(hi-lo)-1.,-1.,1.);
  }
  static double decode(double z,double lo,double hi){return lo+.5*(z+1.)*std::max(0.,hi-lo);}
  static double coordinate(const Decision& z,int channel,int step,const SolverConfig& c){
    const double k=double(step)*(c.knots-1)/(c.steps-1);
    const int left=std::min(c.knots-2,int(k));const double f=k-left;
    return (1.-f)*z[2*left+channel]+f*z[2*(left+1)+channel];
  }
  Decision makeSeed(const SolveRequest& r,const SolverConfig& c,int kind,
      std::array<Command,kMaxHorizon>* scheduled=nullptr,bool* complete=nullptr)const{
    Decision z{};Command prev=r.previous;double a=r.previous_command_acceleration;
    double flat_heading=r.state.yaw;
    std::array<std::array<double,2>,kMaxHorizon> dense{};
    const double cap=std::min(c.limits.speed,r.speed_cap);
    if(complete)*complete=false;
    for(int i=0;i<c.steps;++i){
      const double h=interval(i,r,c);Command wanted=r.baseline;
      if(kind==2)wanted=r.previous;
      if(kind==4)wanted=r.warm_hint[i];
      if(kind==3){
        // A solve may follow a skipped callback or a changed first interval.
        // Resample the previous piecewise-constant policy at physical time.
        const double old_t=r.now-warm_stamp_+startTime(i,r,c);
        int j=0;
        if(old_t>=warm_first_interval_)j=1+int((old_t-warm_first_interval_+1e-10)/warm_dt_);
        wanted=warm_controls_[std::min(j,warm_count_-1)];
      }
      const bool synchronized=scheduled&&kind==1&&r.reference.synchronized_preview&&prepared_.synchronized_demand_valid[i];
      if(kind==1&&!synchronized){
        const auto& p=prepared_.reference[i];
        // Steady-curve body-center -> rear curvature initialization. Variable
        // curvature is handled by the optimizer's explicit center prediction.
        const double lk=clamp(c.model.rear_to_center*p.curvature,-.95,.95);
        const double kr=p.curvature/std::sqrt(1.-lk*lk);
        const double v=p.speed/std::sqrt(1.+c.model.rear_to_center*c.model.rear_to_center*kr*kr);
        const double yaw=p.yaw-std::atan(c.model.rear_to_center*kr);
        FlatOutputJet jet;jet.dx=v*std::cos(yaw);jet.dy=v*std::sin(yaw);
        jet.ddx=p.acceleration*std::cos(yaw)-v*v*kr*std::sin(yaw);
        jet.ddy=p.acceleration*std::sin(yaw)+v*v*kr*std::cos(yaw);
        jet.dddx=-v*v*v*kr*kr*std::cos(yaw)-3.*v*p.acceleration*kr*std::sin(yaw);
        jet.dddy=-v*v*v*kr*kr*std::sin(yaw)+3.*v*p.acceleration*kr*std::cos(yaw);
        const auto flat=FlatnessReferenceModel::reconstruct(jet,c.model);
        wanted=flat.valid?flat.feedforward:Command{};
        if(c.research_consistent_flat_seed&&c.model.rear_to_center>.001){
          const double h=interval(i,r,c),t=startTime(i,r,c)+h;
          // Integrate body heading consistently with the specified center
          // tangent; no instantaneous constant-curvature conversion assumed.
          for(int sub=0;sub<4;++sub){
            const double ts=t-h+(sub+.5)*h/4.;
            const auto pc=referenceAt(r.reference,ts);
            const double r0=pc.speed*std::sin(pc.yaw-flat_heading)/c.model.rear_to_center;
            const double mid=flat_heading+.5*(h/4.)*r0;
            flat_heading=wrap(flat_heading+(h/4.)*pc.speed*std::sin(pc.yaw-mid)/c.model.rear_to_center);
          }
          const auto before=referenceAt(r.reference,std::max(0.,t-c.dt));
          const auto after=referenceAt(r.reference,std::min(r.reference.points[r.reference.count-1].t,t+c.dt));
          const double width=after.t-before.t;
          const double kd=width>1e-9?(after.curvature-before.curvature)/width:0.;
          const double j=width>1e-9?(after.acceleration-before.acceleration)/width:0.;
          const double co=std::cos(p.yaw),si=std::sin(p.yaw),v=p.speed,a=p.acceleration,k=p.curvature;
          FlatOutputJet center;center.x=p.x;center.y=p.y;center.dx=v*co;center.dy=v*si;
          center.ddx=a*co-v*v*k*si;center.ddy=a*si+v*v*k*co;
          center.dddx=(j-v*v*v*k*k)*co-(3*v*a*k+v*v*kd)*si;
          center.dddy=(j-v*v*v*k*k)*si+(3*v*a*k+v*v*kd)*co;
          const auto consistent=FlatnessReferenceModel::reconstruct(rearJetFromCenter(center,flat_heading,c.model.rear_to_center),c.model);
          wanted=consistent.valid?consistent.feedforward:r.baseline;
        }
      }
      if(synchronized){
        wanted=prepared_.synchronized_demand[i];
      }else if(kind==1&&r.reference.speed_preview){
        // Seed the issued longitudinal demand from the reference at its
        // activation time. The real command history and exact delay are
        // still applied by rollout. Steering seeds and hard bounds remain.
        const auto pv=referenceAt(r.reference,startTime(i,r,c)+c.model.speed_delay+h);
        const double lk=clamp(c.model.rear_to_center*pv.curvature,-.95,.95);
        wanted.speed=(pv.speed*std::sqrt(1.-lk*lk)+
          c.model.speed_tau*(pv.acceleration-c.model.longitudinal_residual))/c.model.speed_gain;
      }
      double lo,hi,dlo,dhi;
      if(!demandBounds(prev,a,h,cap,c.limits,lo,hi,dlo,dhi))break;
      const double za=encode((wanted.speed-prev.speed)/h,lo,hi);
      const double zd=encode((wanted.steering-prev.steering)/h,dlo,dhi);
      a=decode(za,lo,hi);prev.speed+=a*h;prev.steering+=decode(zd,dlo,dhi)*h;
      dense[i]={{za,zd}};
      if(scheduled)(*scheduled)[i]=prev;
      if(complete&&i+1==c.steps)*complete=true;
    }
    // Dense commands are only an initialization for the eight coefficient
    // chart, never issued. They retain distinct activation times; fitting
    // them to six input nodes here would change phase again.
    if(scheduled)return z;
    for(int k=0;k<c.knots;++k){
      if(c.research_grid==0){
        const int j=int(std::round(double(k)*(c.steps-1)/(c.knots-1)));
        z[2*k]=dense[j][0];z[2*k+1]=dense[j][1];
      }else{
        const double pos=clamp(prepared_.knot_positions[k],0.,double(c.steps-1));
        const int j=std::min(c.steps-2,int(pos));const double f=pos-j;
        z[2*k]=(1.-f)*dense[j][0]+f*dense[j+1][0];
        z[2*k+1]=(1.-f)*dense[j][1]+f*dense[j+1][1];
      }
    }
    return z;
  }
  static Source commandSource(double relative_t,const SolveRequest& r,const SolverConfig& c,int available){
    Source source;
    if(relative_t>=-1e-10){
      int j=0;while(j+1<=available&&startTime(j+1,r,c)<=relative_t+1e-10)++j;
      source.index=j;return source;
    }
    const double stamp=r.now+relative_t;
    const auto it=std::upper_bound(r.history.begin(),r.history.end(),stamp+1e-10,
      [](double time,const CommandSample& sample){return time<sample.stamp;});
    source.fixed=it==r.history.begin()?r.previous:(it-1)->command;return source;
  }
  // One centerline clock for the two inverse actuators. Reuse the body anchor
  // of the time-law lift when present. The delayed rollout still uses actual
  // queued commands independently of this seed.
  void prepareSynchronizedDemand(const SolveRequest& r,const SolverConfig& c){
    prepared_.synchronized_demand_valid.fill(false);
    if(!r.reference.synchronized_preview||c.model.rear_to_center<=1e-6)return;
    const double ell=c.model.rear_to_center;
    const auto initial=referenceAt(r.reference,0.);
    const bool body_anchor=std::isfinite(r.reference.initial_body_yaw);
    if(!body_anchor&&std::abs(ell*initial.curvature)>=.95)return;
    // Fixed integration grid, independent of both actuator query times.
    // Moving one actuator's latency cannot perturb the other's feedforward.
    std::array<double,4*kMaxHorizon+1> body{};
    body[0]=body_anchor?r.reference.initial_body_yaw:wrap(initial.yaw-std::asin(ell*initial.curvature));
    const double last=r.reference.points[r.reference.count-1].t;
    const int n=4*(r.reference.count-1);
    if(last<=0.||n<=0)return;
    const double h=last/n;
    auto step=[&](double yaw,double t,double dt){
      const auto p=referenceAt(r.reference,t+.5*dt);
      const double r0=p.speed*std::sin(p.yaw-yaw)/ell;
      return wrap(yaw+dt*p.speed*std::sin(p.yaw-yaw-.5*dt*r0)/ell);
    };
    for(int j=0;j<n;++j)body[j+1]=step(body[j],j*h,h);
    for(int i=0;i<c.steps;++i)for(bool steering:{false,true}){
      const double delay=steering?c.model.steering_delay:c.model.speed_delay;
      const double t=std::min(last,startTime(i,r,c)+.5*interval(i,r,c)+delay);
      const int j=std::min(n-1,static_cast<int>(t/h));
      const double body_yaw=step(body[j],j*h,t-j*h);
      const auto p=referenceAt(r.reference,t);
      const double beta=wrap(p.yaw-body_yaw),sb=std::sin(beta),cb=std::cos(beta);
      const double rear=p.speed*cb,omega=p.speed*sb/ell;
      auto& u=prepared_.synchronized_demand[i];
      double result=missing();
      if(rear>=.015&&cb>.5){
        // A centered integral of the *tangent* over an actuator bandwidth,
        // with wrap protection. This is seed conditioning only: pose/error
        // costs and hard rollout still use the unmodified path.
        const double half=std::max(.5*c.dt,.5*c.model.steering_tau);
        const auto lo=referenceAt(r.reference,std::max(0.,t-half));
        const auto hi=referenceAt(r.reference,std::min(last,t+half));
        const double yaw_dot=hi.t>lo.t?wrap(hi.yaw-lo.yaw)/(hi.t-lo.t):0.;
        const double beta_dot=yaw_dot-omega;
        const double a=p.acceleration*cb-p.speed*sb*beta_dot;
        if(!steering)result=(rear+c.model.speed_tau*(a-c.model.longitudinal_residual))/c.model.speed_gain;
        else{
          const double omega_dot=(p.acceleration*sb+p.speed*cb*beta_dot)/ell;
          const double x=c.model.wheelbase*(omega-c.model.yaw_residual)/(c.model.yaw_gain*rear);
          const double xdot=(c.model.wheelbase*omega_dot/c.model.yaw_gain-x*a)/rear;
          result=(std::atan(x)+c.model.steering_tau*xdot/(1.+x*x))/c.model.steering_gain;
        }
      }
      if(steering)u.steering=result;else u.speed=result;
    }
    for(int i=0;i<c.steps;++i)prepared_.synchronized_demand_valid[i]=finiteCommand(prepared_.synchronized_demand[i]);
  }
  bool prepare(const SolveRequest& r,const SolverConfig& c,SolveProfile& profile,bool seed=true){
    prepared_.terminal_approach=r.reference.terminal_approach;
    prepared_.terminal_jerk_scale=1.;
    if(prepared_.terminal_approach){
      const auto& end=r.reference.endpoint;
      prepared_.end_cos=std::cos(end.yaw);prepared_.end_sin=std::sin(end.yaw);
      // Use endpoint-relative coordinates for a numerically stable boundary.
      prepared_.end_x=end.x;prepared_.end_y=end.y;
      // Ramp the performance penalty in as usable length decreases.
      // No change to the physical jerk bound, and no emergency activation.
      const double length=std::max(0.,end.s-r.reference.points[0].s);
      const double horizon=r.control_dt+(c.steps-1)*c.dt+c.model.speed_delay+c.model.speed_tau;
      const double free_distance=std::max(.015,r.state.speed)*horizon;
      const double x=clamp(1.-length/free_distance,0.,1.);
      prepared_.terminal_jerk_scale=1.+4.*x*x*(3.-2.*x);
    }
    const auto reference_start=c.profile?Clock::now():Clock::time_point{};
    for(int i=0;i<c.steps;++i){
      const double h=interval(i,r,c),t=startTime(i,r,c);
      prepared_.h[i]=h;prepared_.reference[i]=referenceAt(r.reference,t+h);
      prepared_.ref_sin[i]=std::sin(prepared_.reference[i].yaw);
      prepared_.ref_cos[i]=std::cos(prepared_.reference[i].yaw);
      const double k=double(i)*(c.knots-1)/(c.steps-1);
      prepared_.knot_left[i]=std::min(c.knots-2,int(k));
      prepared_.knot_fraction[i]=k-prepared_.knot_left[i];
    }
    prepared_.yaw_rate_weight=0.;
    const double course_weight=!r.reference.terminal_approach&&r.supported_course_yaw_weight>0.&&
      r.supported_course_yaw_weight<=4.?r.supported_course_yaw_weight:0.;
    const double tracking_yaw_weight=c.yaw_rate_tracking_weight+course_weight;
    if(tracking_yaw_weight>0. && !r.reference.braking){
      const double lever=c.model.rear_to_center;
      const auto p0=referenceAt(r.reference,0.);
      const double beta_sin=lever*p0.curvature;
      // Lift the reference from its center course to rear-axle body yaw.
      // Seed from reference geometry so current yaw error cannot move the target.
      if(std::abs(beta_sin)<1.){
        double yaw=wrap(p0.yaw-std::asin(beta_sin));
        for(int i=0;i<c.steps;++i){
          const auto& p=prepared_.reference[i];
          if(lever<=1e-6){prepared_.body_yaw_rate[i]=p.speed*p.curvature;continue;}
          const double t=startTime(i,r,c), h=prepared_.h[i]/4.;
          for(int j=0;j<4;++j){
            const auto middle=referenceAt(r.reference,t+(j+.5)*h);
            const double course_error=middle.yaw-yaw;
            const double rate=middle.speed*std::sin(course_error)/lever;
            yaw=wrap(yaw+h*middle.speed*std::sin(course_error-.5*h*rate)/lever);
          }
          prepared_.body_yaw_rate[i]=p.speed*std::sin(p.yaw-yaw)/lever;
        }
        prepared_.yaw_rate_weight=tracking_yaw_weight;
      }
    }
    if(!r.reference.braking&&!r.reference.terminal_approach&&
       std::isfinite(r.straight_yaw_damping_weight)&&r.straight_yaw_damping_weight>0.){
      // w*(omega-ref)^2+d*omega^2 is equivalent, up to a trajectory-
      // independent constant, to (w+d)*(omega-w*ref/(w+d))^2.
      // Use this same prepared cost in the nonlinear rollout and analytic
      // flat8 derivatives. It also covers the zero-lift case cheaply.
      const double total=prepared_.yaw_rate_weight+r.straight_yaw_damping_weight;
      if(prepared_.yaw_rate_weight>0.){
        const double scale=prepared_.yaw_rate_weight/total;
        for(int i=0;i<c.steps;++i)prepared_.body_yaw_rate[i]*=scale;
      }else{
        for(int i=0;i<c.steps;++i)prepared_.body_yaw_rate[i]=0.;
      }
      prepared_.yaw_rate_weight=total;
    }
    if(!r.reference.braking&&!r.reference.terminal_approach&&r.straight_yaw_damping_weight==0.&&
       r.turn_release.weight>0.&&r.turn_release.weight<=4.){
      const double old_weight=prepared_.yaw_rate_weight;
      const double total=old_weight+r.turn_release.weight;
      for(int i=0;i<c.steps;++i){
        const double target=P7TurnTiming::releaseYawRate(r.turn_release,prepared_.reference[i]);
        if(!std::isfinite(target))return false;
        prepared_.body_yaw_rate[i]=((old_weight>0.?old_weight*prepared_.body_yaw_rate[i]:0.)+
                                    r.turn_release.weight*target)/total;
      }
      prepared_.yaw_rate_weight=total;
    }
    if(r.reference.synchronized_preview&&seed)prepareSynchronizedDemand(r,c);
    if(c.research_grid!=0){
      prepared_.knot_positions=researchKnotPositions(c,prepared_.reference,r.control_dt);
      for(int i=0;i<c.steps;++i)researchKnotCoordinate(i,prepared_.knot_positions,c,prepared_.knot_left[i],prepared_.knot_fraction[i]);
    }
    if(c.profile)profile.reference_ms+=milliseconds(reference_start);
    const auto schedule_start=c.profile?Clock::now():Clock::time_point{};
    prepared_.count=0;
    for(int i=0;i<c.steps;++i){
      prepared_.begin[i]=prepared_.count;
      const double h=prepared_.h[i],t=startTime(i,r,c);
      double elapsed=0.;
      while(elapsed<h-1e-10){
        const double absolute=t+elapsed;
        double sub=std::min(.01,h-elapsed);
        for(double delay:{c.model.speed_delay,c.model.steering_delay}){
          const auto next=std::upper_bound(r.history.begin(),r.history.end(),
            r.now+absolute-delay+1e-9,
            [](double time,const CommandSample& sample){return time<sample.stamp;});
          if(next!=r.history.end())sub=std::min(sub,next->stamp-r.now+delay-absolute);
          for(int j=0;j<=i;++j){const double event=startTime(j,r,c)+delay-absolute;
            if(event>1e-9)sub=std::min(sub,event);}
        }
        if(!(sub>0.) || prepared_.count>=kMaxSubsteps)return false;
        auto& step=prepared_.substeps[prepared_.count++];step.dt=sub;
        step.speed_decay=std::exp(-sub/c.model.speed_tau);
        step.steering_decay=std::exp(-sub/c.model.steering_tau);
        step.speed=commandSource(absolute-c.model.speed_delay,r,c,i);
        step.steering=commandSource(absolute-c.model.steering_delay,r,c,i);
        elapsed+=sub;
      }
    }
    prepared_.begin[c.steps]=prepared_.count;
    if(c.profile)profile.initialization_ms+=milliseconds(schedule_start);
    return true;
  }
  static double stateViolation(const State& s,double accel,double steer_rate,double cap,const SolverConfig& c){
    const double omega=c.model.yaw_gain*s.speed*std::tan(s.steering)/c.model.wheelbase+c.model.yaw_residual;
    return std::max({0.,-s.speed,s.speed-cap,std::abs(s.steering)-c.limits.steering,
      accel-c.limits.acceleration,-accel-c.limits.deceleration,
      std::abs(steer_rate)-c.limits.steering_rate,
      std::abs(s.speed*omega)-c.limits.lateral_acceleration});
  }
  // Most delayed substeps stay in the local principal interval. Only an
  // actual branch crossing needs the general trigonometric reduction.
  static double predictionAngle(double angle) {
    return angle>-3.14159265358979323846 && angle<3.14159265358979323846 ? angle : wrap(angle);
  }
  static void advanceSubstep(State& s,const Substep& step,const Model& model,
      const Command& speed,const Command& steering){
    const double veq=model.speed_gain*speed.speed+model.speed_tau*model.longitudinal_residual;
    const double next_v=veq+(s.speed-veq)*step.speed_decay;
    const double deq=model.steering_gain*steering.steering;
    const double next_d=deq+(s.steering-deq)*step.steering_decay;
    const double vm=.5*(s.speed+next_v),dm=.5*(s.steering+next_d);
    const double omega=model.yaw_gain*vm*std::tan(dm)/model.wheelbase+model.yaw_residual;
    const double mid_yaw=s.yaw+.5*step.dt*omega;
    s.x+=step.dt*(vm*std::cos(mid_yaw)-model.lateral_residual*std::sin(mid_yaw));
    s.y+=step.dt*(vm*std::sin(mid_yaw)+model.lateral_residual*std::cos(mid_yaw));
    s.yaw=predictionAngle(s.yaw+step.dt*omega);s.speed=next_v;s.steering=next_d;
    s.acceleration=(model.speed_gain*speed.speed-s.speed)/model.speed_tau+model.longitudinal_residual;
  }
  static double endpointOverrun(const State& s,const Prepared& p,const SolverConfig& c){
    return (s.x-p.end_x+c.model.rear_to_center*std::cos(s.yaw))*p.end_cos+
           (s.y-p.end_y+c.model.rear_to_center*std::sin(s.yaw))*p.end_sin;
  }
  // Bounded C1 braking chart for the issued command. Use the precomputed
  // exact delayed speed channel to pay for already queued motion before the
  // first new command can act. No measurement/history is synthesized.
  static bool terminalBrakeSeed(const SolveRequest& r,const SolverConfig& c,const Prepared& p,
      const std::array<Command,kMaxHorizon>& steering_seed,double duration_scale,
      std::array<Command,kMaxHorizon>& issued){
    if(!p.terminal_approach||!r.published_history_contiguous||r.history.empty()||
       std::abs(r.history.back().command.speed-r.previous.speed)>1e-9||
       r.history.back().stamp>r.now||r.now-r.history.back().stamp>.12)return false;
    double v=r.state.speed,drift=0.;
    for(int j=0;j<p.count&&p.substeps[j].speed.index<0;++j){
      const auto& q=p.substeps[j];const auto& m=c.model;
      const double eq=m.speed_gain*q.speed.fixed.speed+m.speed_tau*m.longitudinal_residual;
      drift+=eq*q.dt+(v-eq)*m.speed_tau*(1.-q.speed_decay);
      v=eq+(v-eq)*q.speed_decay;
    }
    const double available=-endpointOverrun(r.state,p,c)-drift-c.model.speed_tau*std::max(0.,v);
    if(!std::isfinite(available)||available<=0.)return false;
    const auto& l=c.limits;const double v0=r.previous.speed,a0=r.previous_command_acceleration;
    // Keep the flat chart nonsingular while tracking a finite braking
    // horizon. It is not a standstill controller; the existing reference-stop
    // has priority. The seed's positive floor does not relax endpoint checks.
    const double floor=std::min(v0,.025),dv=v0-floor;
    if(dv<1e-5)return false;
    const double gain=c.model.speed_gain;
    // Allocate remaining distance to the area of the cubic command pulse:
    // D(T)=g*(dv*T/2 + max(a0,0)*T^2/12). Reserve floor travel over the horizon.
    const double horizon=r.control_dt+(c.steps-1)*c.dt;
    const double d=available/gain-floor*horizon;
    if(d<=0.)return false;
    const double b=.5*dv, a=std::max(0.,a0)/12.;
    const double duration=duration_scale*2.*d/(b+std::sqrt(b*b+4.*a*d));
    if(!std::isfinite(duration)||duration<r.control_dt)return false;
    Command prev=r.previous;double accel=a0;
    for(int i=0;i<c.steps;++i){
      const double h=interval(i,r,c),t=startTime(i,r,c)+h;
      const double f=clamp(t/duration,0.,1.),u=1.-f;
      const double desired=floor+dv*u*u*(1.+2.*f)+a0*duration*f*u*u;
      double lo,hi,dlo,dhi;
      if(!demandBounds(prev,accel,h,std::min(l.speed,r.speed_cap),l,lo,hi,dlo,dhi))return false;
      accel=clamp((desired-prev.speed)/h,lo,hi);
      // Retain room for the next jerk-limited recovery toward zero accel.
      // At 0.1 s steps this bound is conservative for the sampled ramp.
      const double portion=std::max(0.,prev.speed-floor);
      const double safe=-2.*portion/(h+std::sqrt(h*h+2.*portion/l.jerk));
      if(safe>hi+1e-12)return false;
      accel=std::max(accel,safe);
      prev.speed+=h*accel;
      prev.steering+=h*clamp((steering_seed[i].steering-prev.steering)/h,dlo,dhi);
      issued[i]=prev;
    }
    return true;
  }
  static Rollout rollout(const Decision& z,const SolveRequest& r,const SolverConfig& c,
      const Prepared& cache,Clock::time_point search_deadline,bool& stopped,SolveProfile& profile,
      const std::array<Command,kMaxHorizon>* direct=nullptr,const IssuedInputs* issued=nullptr,
      const FixedPrefix* fixed=nullptr,double incumbent_cost=INFINITY){
    Rollout roll;roll.states[0]=r.state;
    const double cap=std::min(c.limits.speed,r.speed_cap);
    Command previous=r.previous;double previous_a=r.previous_command_acceleration;
    State s=r.state;double objective=0.;
    // The previous substep already checked this exact boundary state. Its
    // derivative constraints change only at a delayed command discontinuity.
    double checked_speed_command=missing(),checked_steering_command=missing();
    auto timer=c.profile?Clock::now():Clock::time_point{};
    roll.violation=stateViolation(s,s.acceleration,0.,cap,c);
    if(c.profile)profile.constraint_cost_ms+=milliseconds(timer);
    if(roll.violation>1e-7){roll.rejection="initial_state_constraint_violation";return roll;}
    if(cache.terminal_approach){
      roll.violation=endpointOverrun(s,cache,c);
      if(roll.violation>1e-7){roll.rejection="terminal_initial_past_reference";return roll;}
    }
    for(int i=0;i<c.steps;++i){
      roll.rejected_step=i;
      if(Clock::now()>=search_deadline){stopped=true;return roll;}
      const double h=cache.h[i];
      timer=c.profile?Clock::now():Clock::time_point{};
      IssuedInput input;
      if(issued)input=(*issued)[i];
      else if(direct){
        if(!prepareDirectInput((*direct)[i],previous,previous_a,h,cap,c.limits,input,roll))return roll;
      }else{
        double alo,ahi,dlo,dhi;
        if(!demandBounds(previous,previous_a,h,cap,c.limits,alo,ahi,dlo,dhi)){
          roll.rejection="command_speed_acceleration_jerk_bounds_empty";
          roll.violation=std::max(alo-ahi,dlo-dhi);
          if(c.profile)profile.constraint_cost_ms+=milliseconds(timer);
          return roll;
        }
        const int left=cache.knot_left[i];const double f=cache.knot_fraction[i];
        input.acceleration=decode((1.-f)*z[2*left]+f*z[2*(left+1)],alo,ahi);
        input.steering_rate=decode((1.-f)*z[2*left+1]+f*z[2*(left+1)+1],dlo,dhi);
        input.command={previous.speed+h*input.acceleration,previous.steering+h*input.steering_rate};
      }
      const auto& u=input.command;
      const double a=input.acceleration,d=input.steering_rate;
      roll.commands[i]=u;roll.command_accelerations[i]=a;
      if(c.profile)profile.constraint_cost_ms+=milliseconds(timer);
      // Event partition and actuator exponentials are candidate-independent.
      // Reuse already verified states only until a candidate command can act.
      int first=cache.begin[i];
      if(fixed&&i<fixed->stage_count){
        const auto& stage=fixed->stages[i];
        s=stage.state;first=stage.end;
        roll.violation=std::max(roll.violation,stage.violation);
        if(i+1==fixed->stage_count){
          checked_speed_command=fixed->speed_command;checked_steering_command=fixed->steering_command;
        }
      }
      for(int j=first;j<cache.begin[i+1];++j){
        const auto& step=cache.substeps[j];
        const Command& ud=step.steering.index<0?step.steering.fixed:roll.commands[step.steering.index];
        const Command& uv=step.speed.index<0?step.speed.fixed:roll.commands[step.speed.index];
        if(uv.speed!=checked_speed_command||ud.steering!=checked_steering_command){
          const double acc=(c.model.speed_gain*uv.speed-s.speed)/c.model.speed_tau+c.model.longitudinal_residual;
          const double dr=(c.model.steering_gain*ud.steering-s.steering)/c.model.steering_tau;
          timer=c.profile?Clock::now():Clock::time_point{};
          roll.violation=std::max(roll.violation,stateViolation(s,acc,dr,cap,c));
          if(c.profile)profile.constraint_cost_ms+=milliseconds(timer);
          if(roll.violation>1e-7){roll.rejection="delayed_physical_state_constraint_violation";return roll;}
        }
        timer=c.profile?Clock::now():Clock::time_point{};
        advanceSubstep(s,step,c.model,uv,ud);
        if(c.profile)profile.prediction_ms+=milliseconds(timer);
        timer=c.profile?Clock::now():Clock::time_point{};
        if(!validState(s)){roll.violation=std::numeric_limits<double>::infinity();
          if(c.profile)profile.constraint_cost_ms+=milliseconds(timer);
        roll.rejection="predicted_state_nonfinite";return roll;}
        if(cache.terminal_approach){
          const double overrun=endpointOverrun(s,cache,c);
          if(overrun>1e-7){roll.violation=overrun;roll.rejection="terminal_prediction_past_reference";return roll;}
        }
        roll.violation=std::max(roll.violation,stateViolation(s,s.acceleration,
                             (c.model.steering_gain*ud.steering-s.steering)/c.model.steering_tau,cap,c));
        if(c.profile)profile.constraint_cost_ms+=milliseconds(timer);
        if(roll.violation>1e-7){roll.rejection="predicted_physical_state_constraint_violation";return roll;}
        checked_speed_command=uv.speed;checked_steering_command=ud.steering;
      }
      timer=c.profile?Clock::now():Clock::time_point{};
      const auto& p=cache.reference[i];
      const double center_x=s.x+c.model.rear_to_center*std::cos(s.yaw);
      const double center_y=s.y+c.model.rear_to_center*std::sin(s.yaw);
      const double ex=center_x-p.x,ey=center_y-p.y;
      const double lateral=-cache.ref_sin[i]*ex+cache.ref_cos[i]*ey;
      const double along=cache.ref_cos[i]*ex+cache.ref_sin[i]*ey;
      const double omega=c.model.yaw_gain*s.speed*std::tan(s.steering)/c.model.wheelbase+c.model.yaw_residual;
      const double center_vy=c.model.lateral_residual+c.model.rear_to_center*omega;
      const double tangent=s.yaw+(std::abs(s.speed)>.015?std::atan2(center_vy,s.speed):0.);
      const double heading=predictionAngle(tangent-p.yaw);
      const double velocity=std::hypot(s.speed,center_vy)-p.speed;
      const double jerk=(a-previous_a)/h;
      double stage=c.weights.lateral*(lateral*lateral+.15*along*along)+
         c.weights.heading*heading*heading+c.weights.speed*velocity*velocity;
      if(cache.yaw_rate_weight>0.){
        const double rate_error=omega-cache.body_yaw_rate[i];
        stage+=cache.yaw_rate_weight*rate_error*rate_error;
      }
      objective+=h*(stage+c.weights.smoothness*(.01*d*d+.02*a*a+.002*cache.terminal_jerk_scale*jerk*jerk));
      if(i==c.steps-1)objective+=.4*stage;
      if(c.profile)profile.constraint_cost_ms+=milliseconds(timer);
      // Each remaining cost term is nonnegative. This prefix cannot win,
      // and only complete, fully checked trajectories are ever feasible.
      if(i+1<c.steps&&incumbent_cost<INFINITY&&objective>=incumbent_cost){
        roll.dominated=true;return roll;
      }
      roll.states[i+1]=s;previous=u;previous_a=a;
    }
    roll.cost=objective;roll.feasible=std::isfinite(objective);roll.rejection=roll.feasible?"feasible":"objective_nonfinite";roll.rejected_step=-1;return roll;
  }
};
}
