// Hardware-free comparison. The pursuer, noisy observation and fallback are
// synthetic; the eight-variable solve and final arbitration use P7/P5 code.
#include "control/p6_controller.h"
#include "control/p7_straight_tracking_reference.h"
#include "control/unified_control.h"
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace rc = research_control;
namespace uc = unified_control;
namespace {
constexpr double dt = .025;
constexpr double cap = .40;
constexpr double target = .340;

rc::ReferencePoint route(double s, double k) {
    rc::ReferencePoint p;
    p.s=s;p.x=s;p.y=0.;p.yaw=k*s;p.curvature=k;
    if(k!=0.){p.x=std::sin(k*s)/k;p.y=(1.-std::cos(k*s))/k;}
    return p;
}

void applyConnector(rc::ReferencePoint& point,double s,double amplitude) {
    constexpr double width=.69;
    if(s>=width)return;
    const double rate=std::acos(-1.)/width,z=rate*s;
    const double shift=.5*amplitude*(1.+std::cos(z));
    const double dy=-.5*amplitude*rate*std::sin(z);
    const double ddy=-.5*amplitude*rate*rate*std::cos(z);
    const double k=point.curvature,tangent=1.-k*shift;
    point.x-=std::sin(point.yaw)*shift;
    point.y+=std::cos(point.yaw)*shift;
    point.yaw+=std::atan2(dy,tangent);
    point.curvature=(tangent*(k*tangent+ddy)+2.*k*dy*dy)/
        std::pow(tangent*tangent+dy*dy,1.5);
}

rc::Command held(const std::vector<rc::CommandSample>& history,double t) {
    auto i=std::upper_bound(history.begin(),history.end(),t+1.e-10,
        [](double time,const rc::CommandSample& sample){return time<sample.stamp;});
    if(i==history.begin())throw std::runtime_error("plant history incomplete");
    return (i-1)->command;
}

void simulate(rc::State& s,const rc::Model& m,double now,
              const std::vector<rc::CommandSample>& history) {
    constexpr double h=.005;
    for(int j=0;j<5;++j){
        const double t=now+(j+.5)*h;
        const double u=held(history,t-m.speed_delay).speed;
        const double d=held(history,t-m.steering_delay).steering;
        const double v1=u+(s.speed-u)*std::exp(-h/m.speed_tau);
        const double d1=d+(s.steering-d)*std::exp(-h/m.steering_tau);
        const double v=.5*(s.speed+v1),yaw_increment=h*m.yaw_gain*v*
            std::tan(.5*(s.steering+d1))/m.wheelbase;
        s.x+=h*v*std::cos(s.yaw+.5*yaw_increment);
        s.y+=h*v*std::sin(s.yaw+.5*yaw_increment);
        s.yaw=rc::wrap(s.yaw+yaw_increment);s.speed=v1;s.steering=d1;
        s.acceleration=(u-v1)/m.speed_tau;
    }
}

void run(const char* scenario,double curve,double pulse,bool candidate) {
    rc::SolverConfig c;
    c.iterations=2;c.model.yaw_gain=.9569791928640006;
    c.model.speed_tau=.20;c.model.steering_tau=.44;c.model.steering_delay=.325;
    c.weights.lateral=39.;c.weights.heading=8.678570922328328;
    c.limits.speed=cap;
    const auto& m=c.model;const auto& l=c.limits;
    rc::P6Controller solver;
    solver.setConditionedGeometry(true);solver.setBaselineSeed(true);
    solver.setDelayAlignedReconstruction(true);solver.setCommandProjection(true);
    solver.setResponseBase(true);solver.setResponseNearSupport(true);
    solver.setBoundedFeedback(true);solver.setTurnAwareRateContinuity(true);
    solver.setSteeringRateContinuityWeight(.05);
    rc::P6EffectiveParameters effective;
    effective.valid=true;effective.scene="smooth";effective.solver=c;
    effective.nominal_config_hash=std::string(64,'a');effective.speed_cap=cap;
    rc::State s;
    s.yaw=-std::asin(m.rear_to_center*curve);
    s.x=-m.rear_to_center*std::cos(s.yaw);s.y=-m.rear_to_center*std::sin(s.yaw);
    s.speed=target;s.steering=std::atan(m.wheelbase*curve/(m.yaw_gain*std::cos(s.yaw)));
    rc::Command previous{target,s.steering};
    constexpr double start=100.;
    std::vector<rc::CommandSample> history;
    for(int i=40;i>0;--i)history.push_back({start-i*dt,previous});
    double a=0.;
    for(int tick=0;tick<200;++tick){
        const double now=start+tick*dt;
        const double x=s.x+m.rear_to_center*std::cos(s.yaw);
        const double y=s.y+m.rear_to_center*std::sin(s.yaw);
        const double distance=curve==0.?x:std::atan2(curve*x,1.-curve*y)/curve;
        const auto center=route(distance,curve);
        const double lateral=std::cos(center.yaw)*(y-center.y)-std::sin(center.yaw)*(x-center.x);
        const double heading=rc::wrap(s.yaw-center.yaw+std::asin(m.rear_to_center*curve));
        const double amplitude=pulse*std::sin(tick*dt*2.65);
        std::vector<rc::ReferencePoint> spatial;
        for(int i=0;i<=100;++i){
            const double station=i*.02;
            auto point=route(distance+station,curve);
            if(pulse!=0.)applyConnector(point,station,amplitude);
            spatial.push_back(point);
        }
        const auto& upcoming=spatial[14];
        const auto true_upcoming=route(distance+.28,curve);
        const double offset=std::cos(true_upcoming.yaw)*(upcoming.y-true_upcoming.y)-
            std::sin(true_upcoming.yaw)*(upcoming.x-true_upcoming.x);
        const double wanted=std::atan(m.wheelbase*upcoming.curvature/m.yaw_gain)-
            .6*(lateral-offset)-.8*rc::wrap(s.yaw-upcoming.yaw+std::asin(m.rear_to_center*curve));
        rc::SolveRequest r;
        r.now=now;r.control_dt=dt;r.state=s;r.previous=previous;r.previous_command_acceleration=a;
        const double lo=std::max({-l.deceleration,a-l.jerk*dt,-previous.speed/dt});
        const double hi=std::min({l.acceleration,a+l.jerk*dt,(cap-previous.speed)/dt});
        if(lo>hi+1.e-8)throw std::runtime_error("synthetic fallback viability");
        const double acceleration=rc::clamp(3.*(target-previous.speed),lo,std::max(lo,hi));
        r.baseline={previous.speed+acceleration*dt,rc::clamp(previous.steering+rc::clamp(wanted-previous.steering,
            -l.steering_rate*dt,l.steering_rate*dt),-l.steering,l.steering)};
        r.speed_cap=cap;r.target_speed=target;r.history=history;r.published_history_contiguous=true;
        const auto raw=rc::FlatnessReferenceModel::buildTimeReference(spatial,s.speed,target,
            now,c,false,true,s.acceleration,true,true,s.yaw);
        r.reference=raw;
        double supported=0.,straight=rc::P7StraightTrackingReference::condition(spatial,r.reference);
        double coarse_curvature=0.;
        if(candidate&&straight<.25){
            auto fitted=raw;
            const auto fit=rc::P7StraightTrackingReference::conditionSupportedCourse(spatial,fitted,straight);
            if(fit.blend>straight){
                r.reference=fitted;supported=fit.blend;straight=fit.straight_blend;
                coarse_curvature=fit.curvature;
                r.supported_course_yaw_weight=rc::P7StraightTrackingReference::courseYawWeight(fit,2.);
            }
        }
        r.straight_yaw_damping_weight=rc::P7StraightTrackingReference::yawDampingWeight(straight,2.);
        effective.history_base_sequence=tick+40;effective.cycle=tick+1;effective.stamp=now;
        effective.snapshot_hash=rc::p6SnapshotIdentity(effective);
        const auto solved=solver.solve(r,effective,"flat_nmpc_slqr").solve;
        uc::ControlRequest request;
        request.mode=uc::ControlMode::AutoFallback;
        request.now=now;request.dt=dt;request.legacy=r.baseline;request.previous=previous;
        request.previous_acceleration=a;request.speed_cap=cap;request.limits=l;
        request.epoch=1;request.config_version=1;request.history_sequence=tick+40;
        request.history_valid=true;request.reference={now,static_cast<std::uint64_t>(tick+1),raw.valid,false};
        request.guard={true,true,"/control_research/p5_test/connector","p5_synthetic_connector"};
        const double yaw_rate=m.yaw_gain*s.speed*std::tan(s.steering)/m.wheelbase;
        request.state={now,now,s.x,s.y,s.yaw,s.speed,s.steering,yaw_rate,s.acceleration,1.,0.,true,true,true};
        uc::ControlCandidate proposal;
        proposal.sequence=tick+1;proposal.epoch=1;proposal.config_version=1;
        proposal.reference_sequence=tick+1;proposal.history_base_sequence=tick+40;
        proposal.created_stamp=proposal.snapshot_stamp=proposal.expected_apply_stamp=now;
        proposal.command=solved.command;proposal.command_acceleration=solved.command_acceleration;
        proposal.valid=solved.ok;proposal.feasible=solved.feasible;
        proposal.timed_out=solved.timed_out;proposal.max_violation=solved.max_violation;
        proposal.solve_ms=solved.solve_ms;proposal.state_pair_validated=true;
        proposal.complete_substep_constraints=solved.feasible&&solved.max_violation<=1.e-7;
        proposal.propagated_speed=s.speed;proposal.propagated_steering=s.steering;
        proposal.propagated_yaw_rate=yaw_rate;
        const auto decision=uc::ControlArbiter::decide(request,&proposal);
        const auto command=decision.command;
        const double na=(command.speed-previous.speed)/dt;
        const double jerk=(na-a)/dt,steer_rate=(command.steering-previous.steering)/dt;
        const bool violation=command.speed<0.||command.speed>cap+1.e-7||
            std::abs(command.steering)>l.steering+1.e-7||std::abs(steer_rate)>l.steering_rate+1.e-7||
            na>l.acceleration+1.e-7||na<-l.deceleration-1.e-7||std::abs(jerk)>l.jerk+1.e-7;
        std::cout<<scenario<<','<<candidate<<','<<tick<<','<<distance<<','<<lateral<<','<<heading<<','
            <<yaw_rate<<','<<s.steering<<','<<command.steering<<','<<command.speed<<','<<steer_rate<<','
            <<jerk<<','<<decision.selected_nmpc<<','<<solved.solve_ms<<','<<solved.timed_out<<','
            <<violation<<','<<supported<<','<<straight<<','<<coarse_curvature<<','
            <<r.supported_course_yaw_weight<<'\n';
        history.push_back({now,command});
        if(history.size()>64)history.erase(history.begin());
        simulate(s,m,now,history);previous=command;a=na;
    }
}
} // namespace

int main(int argc,char** argv) {
    const bool gentle_only=argc==2&&std::string(argv[1])=="--gentle-only";
    if(argc>1&&!gentle_only){std::cerr<<"usage: p7_connector_course [--gentle-only]\n";return 2;}
    std::cout<<std::setprecision(12)<<"scene,candidate,tick,station,cross_track_error,heading_error,"
        "yaw_rate,measured_steering,command_steering,command_speed,steering_rate,jerk,nmpc_selected,"
        "solve_ms,timeout,violation,fit_blend,straight_blend,coarse_curvature,course_yaw_weight\n";
    for(bool candidate:{false,true}){
        if(!gentle_only){
            run("connector_left",0.,.033,candidate);
            run("connector_right",0.,-.033,candidate);
        }
        run("gentle_left",.12,.033,candidate);
        run("gentle_right",-.12,-.033,candidate);
        if(!gentle_only)run("steady_right",-.36,0.,candidate);
    }
}
