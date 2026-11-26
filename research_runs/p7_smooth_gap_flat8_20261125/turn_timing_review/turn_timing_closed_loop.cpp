// Deterministic, hardware-free P7 comparison on mirrored variable curvature.
// The plant and fallback below are synthetic; trajectory acceptance uses the
// production nonlinear optimizer and P5 arbiter, not a ROS command publisher.
#include "control/p6_controller.h"
#include "control/p7_straight_tracking_reference.h"
#include "control/p7_turn_timing.h"
#include "control/unified_control.h"
#include <iomanip>
#include <iostream>
#include <numeric>
#include <stdexcept>

namespace rc = research_control;
namespace uc = unified_control;
namespace {
constexpr double dt = .025;
constexpr double ds = .005;
constexpr double cruise = .40;
constexpr double synthetic_recovery_speed_ceiling = .50;
constexpr double tight_begin = .82, open_begin = 1.83;

void require(bool ok, const char* what) {
    if (!ok) throw std::runtime_error(what);
}
double ramp(double u) {
    const double x = rc::clamp(u, 0., 1.);
    return x*x*(3.-2.*x);
}

struct Track {
    const int side;
    const bool transition;
    const std::string cap_profile;
    std::vector<rc::ReferencePoint> path;

    Track(int curve_sign, bool use_transition, std::string profile="smooth")
        : side(curve_sign), transition(use_transition), cap_profile(std::move(profile)) {
        rc::ReferencePoint point;
        for (int i=0; i<=1400; ++i) {
            const double s=i*ds;
            point.s=s;
            point.curvature=curvature(s);
            path.push_back(point);
            const double turn=curvature(s+.5*ds)*ds;
            point.x+=ds*std::cos(point.yaw+.5*turn);
            point.y+=ds*std::sin(point.yaw+.5*turn);
            point.yaw=rc::wrap(point.yaw+turn);
        }
    }

    double curvature(double s) const {
        if (!transition) return side*.36;
        if (cap_profile=="compound") {
            // A supported opening before a second bend in the same direction.
            // The .51/m apex stays within the nominal steering geometry.
            return side*(.33-.15*ramp((s-.20)/.42)+.33*ramp((s-.72)/.34)-
                         .32*ramp((s-1.76)/.43));
        }
        // Near .5/m at the tight apex, then deliberately opening to a
        // shallow bend. Scale and operating speeds follow the 16:59 run.
        return side*(.17+.35*ramp((s-tight_begin)/.34)-.34*ramp((s-open_begin)/.44));
    }
    double cap(double s) const {
        // Synthetic *causal* dynamic-cap schedule to exercise the behavior of
        // a cap that drops when the vehicle reaches the tighter geometry.
        // Real cap values are not treated as a validated map.
        if (!transition||cap_profile=="compound") return cruise;
        const bool rapid=cap_profile=="rapid";
        const double drop=rapid ? .07 : .04;
        return cruise-drop*ramp((s-tight_begin)/(rapid ? .20 : .44))+
            drop*ramp((s-open_begin)/(rapid ? .22 : .44));
    }
    double target(double s) const {
        return cap_profile=="compound" ? .385 : cap(s);
    }
    rc::ReferencePoint at(double s) const {
        const double clamped=rc::clamp(s,0.,path.back().s);
        const auto i=std::min(path.size()-2,static_cast<std::size_t>(clamped/ds));
        auto p=path[i]; const auto& next=path[i+1];
        const double f=(clamped-p.s)/ds;
        p.s=clamped;p.x+=f*(next.x-p.x);p.y+=f*(next.y-p.y);
        p.yaw=rc::wrap(p.yaw+f*rc::wrap(next.yaw-p.yaw));
        p.curvature=curvature(clamped);
        return p;
    }
    double project(double x,double y,double previous) const {
        const std::size_t first=static_cast<std::size_t>(std::max(0.,previous-.10)/ds);
        const std::size_t last=std::min(path.size(),std::size_t((previous+.5)/ds)+1);
        double nearest=std::numeric_limits<double>::infinity();
        std::size_t best=first;
        for (std::size_t i=first;i<last;++i) {
            const double xx=path[i].x-x, yy=path[i].y-y;
            if (xx*xx+yy*yy<nearest) {
                nearest=xx*xx+yy*yy;
                best=i;
            }
        }
        return path[best].s;
    }
};

rc::Command held(const std::vector<rc::CommandSample>& history,double t) {
    for (auto i=history.rbegin();i!=history.rend();++i) {
        if (i->stamp<=t+1e-10) return i->command;
    }
    throw std::runtime_error("plant requested unknown history");
}
double steeringFor(double curvature,const rc::Model& m) {
    const double beta_sin=m.rear_to_center*curvature;
    return std::atan(m.wheelbase*curvature/(m.yaw_gain*std::sqrt(1.-beta_sin*beta_sin)));
}
double yawRate(const rc::State& s,const rc::Model& m) {
    return m.yaw_gain*s.speed*std::tan(s.steering)/m.wheelbase;
}
void independentStep(rc::State& s,const rc::Model& m,double now,
                     const std::vector<rc::CommandSample>& history) {
    // Independent .005 s RK midpoint for longitudinal/steering first-order
    // responses. This is separate from ReducedNMPC rollout and derivatives.
    constexpr int n=5;
    const double h=dt/n;
    for (int k=0;k<n;++k) {
        const double t=now+(k+.5)*h;
        const auto uz=held(history,t-m.speed_delay),ud=held(history,t-m.steering_delay);
        const double vn=uz.speed+(s.speed-uz.speed)*std::exp(-h/m.speed_tau);
        const double dn=ud.steering+(s.steering-ud.steering)*std::exp(-h/m.steering_tau);
        const double vm=(vn+s.speed)/2., dm=(dn+s.steering)/2.;
        const double delta_yaw=h*m.yaw_gain*vm*std::tan(dm)/m.wheelbase;
        s.x+=h*vm*std::cos(s.yaw+.5*delta_yaw);
        s.y+=h*vm*std::sin(s.yaw+.5*delta_yaw);
        s.yaw=rc::wrap(s.yaw+delta_yaw);
        s.speed=vn;s.steering=dn;
        s.acceleration=(uz.speed-vn)/m.speed_tau;
    }
}

uc::PublishedCommandRecord published(uc::CommandLedger& ledger,double now,rc::Command command,
                                    double a,bool nmpc) {
    uc::PublishedCommandRecord record;
    record.sequence=ledger.nextSequence();record.epoch=1;record.cycle_stamp=now;
    record.stamp=record.publish_ros_start=record.publish_ros_end=now;
    record.publish_steady_start_ns=record.publish_steady_end_ns=std::llround(now*1e9);
    record.command=command;record.acceleration=a;
    record.ros_publish_returned=true;record.selected_nmpc=nmpc;
    return record;
}

void run(const char* scenario,const Track& track,double reserve,double release,
         std::ostream& out) {
    constexpr const char* mode="flat_nmpc_slqr";
    rc::SolverConfig c;
    c.iterations=2;c.model.yaw_gain=.9569791928640006;
    c.weights.lateral=39.;c.weights.heading=8.678570922328328;
    c.limits.speed=cruise;
    const auto& m=c.model; const auto& l=c.limits;
    rc::P6Controller solver;
    solver.setConditionedGeometry(true);solver.setBaselineSeed(true);
    solver.setDelayAlignedReconstruction(true);solver.setCommandProjection(true);
    solver.setResponseBase(true);solver.setResponseNearSupport(true);
    solver.setBoundedFeedback(true);solver.setTurnAwareRateContinuity(true);
    solver.setSteeringRateContinuityWeight(.05);
    rc::P6EffectiveParameters effective;
    effective.valid=true;effective.scene="smooth";effective.solver=c;
    effective.nominal_config_hash=std::string(64,'a');
    effective.speed_cap=cruise;
    rc::State state;
    state.speed=track.cap_profile=="compound" ? .355 : .38;
    state.steering=steeringFor(track.curvature(0.),m);
    state.yaw=-std::asin(m.rear_to_center*track.curvature(0.))+.004*track.side;
    state.x=-m.rear_to_center*std::cos(state.yaw);
    state.y=-m.rear_to_center*std::sin(state.yaw)-.015*(track.side ? track.side : 1);
    double progress=0., a=0.;
    rc::Command last{state.speed,state.steering};
    uc::CommandLedger ledger(1);
    std::vector<rc::CommandSample> history;
    for (int i=40;i>0;--i) {
        const double time=100.-i*dt;
        require(ledger.commit(published(ledger,time,last,0.,false)).accepted,"pre-roll");
        history.push_back({time,last});
    }
    constexpr int ticks=420; // 10.5 s at the existing 40 Hz control cadence.
    for (int k=0;k<ticks;++k) {
        const double now=100.+k*dt;
        const double x=state.x+m.rear_to_center*std::cos(state.yaw);
        const double y=state.y+m.rear_to_center*std::sin(state.yaw);
        progress=track.project(x,y,progress);
        const auto here=track.at(progress);
        const double error=-std::sin(here.yaw)*(x-here.x)+std::cos(here.yaw)*(y-here.y);
        const double yaw_error=rc::wrap(state.yaw-here.yaw+
                                    std::asin(m.rear_to_center*here.curvature));
        const auto ahead=track.at(progress+.18);
        const double cap=track.cap(progress);
        // Only the synthetic fallback: preserve longitudinal and steering
        // rate bounds while recovering from an incompatible reference cap.
        // Report speed above the tighter operational cap separately.
        const double upper_a=std::min({l.acceleration,a+l.jerk*dt,
             (synthetic_recovery_speed_ceiling-last.speed)/dt});
        const double lower_a=std::max({-l.deceleration,a-l.jerk*dt,-last.speed/dt});
        require(lower_a<=upper_a+1e-9,"synthetic fallback cannot satisfy physical speed/jerk bounds");
        double wanted_a=rc::clamp(3.*(track.target(progress)-last.speed),lower_a,upper_a);
        const double cap_margin=synthetic_recovery_speed_ceiling-last.speed;
        if (wanted_a>0.)
            wanted_a=std::max(lower_a,std::min(wanted_a,
              std::sqrt(l.jerk*l.jerk*dt*dt+2.*l.jerk*std::max(0.,cap_margin))-l.jerk*dt));
        const double want_delta=rc::clamp(steeringFor(ahead.curvature,m)
            -.85*error-.65*yaw_error,-l.steering,l.steering);
        const rc::Command baseline{last.speed+wanted_a*dt,last.steering+
            rc::clamp(want_delta-last.steering,-l.steering_rate*dt,l.steering_rate*dt)};
        rc::SolveRequest r;
        r.now=now;r.control_dt=dt;r.state=state;r.previous=last;r.baseline=baseline;
        r.previous_command_acceleration=a;r.speed_cap=cap;r.target_speed=track.target(progress);
        r.history=history;r.published_history_contiguous=ledger.valid();
        std::vector<rc::ReferencePoint> spatial;
        for (int j=0;j<=90;++j) spatial.push_back(track.at(progress+.02*j));
        r.reference=rc::FlatnessReferenceModel::buildTimeReference(spatial,state.speed,r.target_speed,
            now,c,false,true,state.acceleration,true,true,state.yaw);
        const double blend=rc::P7StraightTrackingReference::condition(spatial,r.reference);
        r.straight_yaw_damping_weight=rc::P7StraightTrackingReference::yawDampingWeight(blend,2.);
        const auto prior=r.target_speed;
        const auto timing=blend>0. ? rc::P7TurnTiming::Result{} :
            rc::P7TurnTiming::apply(spatial,c,r,reserve,release);
        require(r.speed_cap==cap,"turn timing changed the hard speed cap");
        require(r.target_speed<=prior+1e-10,"turn timing raised the target");
        effective.history_base_sequence=ledger.lastSequence();
        effective.cycle=k+1;effective.stamp=now;
        effective.snapshot_hash=rc::p6SnapshotIdentity(effective);
        const auto solved=solver.solve(r,effective,mode).solve;
        require(solved.decision_dimension==8,"not an eight-coordinate solve");
        uc::ControlRequest request;
        request.mode=uc::ControlMode::AutoFallback;
        request.now=now;request.dt=dt;request.legacy=baseline;request.previous=last;
        request.previous_acceleration=a;request.speed_cap=cap;request.limits=l;
        request.epoch=1;request.config_version=1;request.history_sequence=ledger.lastSequence();
        request.history_valid=ledger.valid();request.reference={now,std::uint64_t(k+1),r.reference.valid,false};
        request.guard={true,true,"/control_research/p5_test/turn_timing","p5_synthetic_turn_timing"};
        request.state={now,now,state.x,state.y,state.yaw,state.speed,state.steering,yawRate(state,m),
                       state.acceleration,1.,0.,true,true,true};
        uc::ControlCandidate candidate;
        candidate.sequence=k+1;candidate.epoch=1;candidate.config_version=1;
        candidate.reference_sequence=k+1;candidate.history_base_sequence=ledger.lastSequence();
        candidate.created_stamp=candidate.snapshot_stamp=candidate.expected_apply_stamp=now;
        candidate.command=solved.command;candidate.command_acceleration=solved.command_acceleration;
        candidate.valid=solved.ok;candidate.feasible=solved.feasible;
        candidate.timed_out=solved.timed_out;candidate.max_violation=solved.max_violation;
        candidate.solve_ms=solved.solve_ms;candidate.state_pair_validated=true;
        candidate.complete_substep_constraints=solved.feasible&&solved.max_violation<=1e-7;
        candidate.propagated_speed=state.speed;candidate.propagated_steering=state.steering;
        candidate.propagated_yaw_rate=yawRate(state,m);
        const auto decision=uc::ControlArbiter::decide(request,&candidate);
        const auto issued=decision.command;
        const double next_a=(issued.speed-last.speed)/dt;
        bool violation=issued.speed<0.||issued.speed>synthetic_recovery_speed_ceiling+1e-7||
            std::abs(issued.steering)>l.steering+1e-7||
            std::abs(issued.steering-last.steering)>l.steering_rate*dt+1e-7||
            next_a>l.acceleration+1e-7||next_a<-l.deceleration-1e-7||
            std::abs(next_a-a)>l.jerk*dt+1e-7;
        const char* phase=!track.transition?"control":
             (progress<open_begin-.16 ? (progress<tight_begin-.50?"approach":"tighten"):"open");
        out<<scenario<<','<<reserve<<','<<release<<','<<k<<','<<progress<<','<<phase<<','
           <<(track.side ? -track.side*error : error)<<','<<yaw_error<<','<<state.speed<<','
           <<state.steering<<','<<yawRate(state,m)<<','<<cap<<','<<r.target_speed<<','
           <<issued.speed<<','<<issued.steering<<','<<decision.selected_nmpc<<','<<solved.timed_out<<','
           <<solved.solve_ms<<','<<violation<<','<<timing.speed_reserve<<','<<timing.release_weight
           <<','<<timing.tightening<<','<<timing.opening<<','<<solved.reason<<'\n';
        require(ledger.commit(published(ledger,now,issued,next_a,decision.selected_nmpc)).accepted,
                "one publication per tick");
        history.push_back({now,issued});
        while (history.size()>50) history.erase(history.begin());
        independentStep(state,m,now,history);
        last=issued;a=next_a;
    }
}
} // namespace

int main(int argc,char**argv) {
    try {
        if (argc!=4) throw std::runtime_error("usage: closed_loop SPEED_RESERVE RELEASE_PRIOR CAP_PROFILE");
        const double reserve=std::stod(argv[1]),release=std::stod(argv[2]);
        const std::string profile=argv[3];
        require(profile=="smooth"||profile=="rapid"||profile=="compound","invalid synthetic cap profile");
        std::cout<<std::setprecision(16)<<"scenario,speed_reserve,release_prior,tick,station,phase,"
             "outward_error,heading_error,actual_speed,actual_steering,yaw_rate,hard_cap,target_speed,"
             "command_speed,command_steering,selected_nmpc,timeout,solve_ms,violation,"
             "applied_reserve,applied_release_weight,tightening,opening,reason\n";
        run("straight",Track(0,false),reserve,release,std::cout);
        run("steady_right",Track(-1,false),reserve,release,std::cout);
        run("tight_open_left",Track(1,true,profile),reserve,release,std::cout);
        run("tight_open_right",Track(-1,true,profile),reserve,release,std::cout);
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n';return 1; }
}
