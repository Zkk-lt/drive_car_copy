#include "control/p7_straight_tracking_reference.h"
#include "control/reduced_nmpc.h"
#include <cassert>
#include <iostream>

using research_control::P7StraightTrackingReference;
using research_control::Reference;
using research_control::ReferencePoint;
using research_control::wrap;

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kYaw = 0.09;
constexpr double kOffset = 0.035;

ReferencePoint transform(ReferencePoint p,double angle,double dx,double dy) {
    const double x = p.x;
    const double y = p.y;
    p.x = dx + std::cos(angle)*x - std::sin(angle)*y;
    p.y = dy + std::sin(angle)*x + std::cos(angle)*y;
    p.yaw = wrap(p.yaw + angle);
    return p;
}

// Millimetre-sized deviations can still create large second derivatives
// and alternating steering feedforward on a nearly straight route.
ReferencePoint wavyPoint(double s) {
    constexpr double frequency = 2.0*kPi/0.50;
    const double slope = 0.004*frequency*std::cos(frequency*s);
    ReferencePoint p;
    p.s = s;
    p.x = s;
    p.y = kOffset + 0.004*std::sin(frequency*s);
    p.yaw = std::atan(slope);
    p.curvature = -0.004*frequency*frequency*std::sin(frequency*s)/
        std::pow(1.0+slope*slope, 1.5);
    return transform(p,kYaw,0.,0.);
}

std::vector<ReferencePoint> nearLine(double end=1.40) {
    std::vector<ReferencePoint> points;
    for(int i=0;i<=std::lround(end/.02);++i)points.push_back(wavyPoint(.02*i));
    return points;
}

std::vector<ReferencePoint> curvedPath(int shape) {
    std::vector<ReferencePoint> points;
    ReferencePoint p;
    p.y = kOffset;
    p.yaw = kYaw;
    constexpr double ds=.02;
    for(int i=0;i<=70;++i) {
        p.s = i*ds;
        if(shape==0)p.curvature = 1./3.5;
        else if(shape==1)p.curvature = .55*std::cos(2.*kPi*p.s/1.18);
        else if(shape==2)p.curvature = p.s<.52 ? 0. : .40;
        // A shallow alternating bend can point along its fitted line while
        // still requiring steering: test the entering positive lobe.
        else p.curvature = .37*std::sin(2.8*(p.s+.50));
        points.push_back(p);
        p.x += ds*std::cos(p.yaw + .5*ds*p.curvature);
        p.y += ds*std::sin(p.yaw + .5*ds*p.curvature);
        p.yaw += ds*p.curvature;
    }
    return points;
}

Reference sampled(const std::vector<ReferencePoint>& spatial) {
    Reference result;
    result.valid = true;
    result.stamp = 17.;
    result.reason = "fixture_reference";
    result.speed_preview = true;
    result.synchronized_preview = true;
    result.endpoint = spatial.back();
    for(std::size_t i=0;i<spatial.size()&&i<=40;i+=2) {
        auto p = spatial[i];
        p.t = .1*result.count;
        p.speed = .40 - .003*result.count;
        p.acceleration = -.03;
        result.points[result.count++] = p;
    }
    return result;
}

bool near(double a,double b,double tolerance=1e-8) {
    return std::abs(a-b)<=tolerance;
}

void unchangedGeometry(const ReferencePoint& a,const ReferencePoint& b) {
    assert(a.x==b.x&&a.y==b.y&&a.yaw==b.yaw&&a.curvature==b.curvature);
}

void unchangedTiming(const Reference& a,const Reference& b) {
    assert(a.count==b.count&&a.stamp==b.stamp&&a.valid==b.valid);
    assert(a.braking==b.braking&&a.terminal_approach==b.terminal_approach);
    assert(a.speed_preview==b.speed_preview&&a.synchronized_preview==b.synchronized_preview);
    assert(a.reason==b.reason);
    assert(a.initial_body_yaw==b.initial_body_yaw||
           (std::isnan(a.initial_body_yaw)&&std::isnan(b.initial_body_yaw)));
    for(int i=0;i<a.count;++i) {
        const auto& x=a.points[i];
        const auto& y=b.points[i];
        assert(x.t==y.t&&x.s==y.s&&x.speed==y.speed&&x.acceleration==y.acceleration);
    }
    unchangedGeometry(a.endpoint,b.endpoint);
    assert(a.endpoint.t==b.endpoint.t&&a.endpoint.s==b.endpoint.s&&
           a.endpoint.speed==b.endpoint.speed&&a.endpoint.acceleration==b.endpoint.acceleration);
}

void verifyBypass(const std::vector<ReferencePoint>& points,Reference ref) {
    const Reference before = ref;
    assert(P7StraightTrackingReference::condition(points,ref)==0.);
    unchangedTiming(before,ref);
    for(int i=0;i<ref.count;++i)unchangedGeometry(before.points[i],ref.points[i]);
}

double curvatureEnergy(const Reference& r) {
    double energy=0.;
    for(int i=0;i<r.count;++i) energy += r.points[i].curvature*r.points[i].curvature;
    return energy;
}

void checkSupportedCourse() {
    std::vector<ReferencePoint> spatial;
    for(int i=0;i<=75;++i){
        const double x=.02*i;
        const double b=std::max(0.,1.-x/.40);
        const double y=.038*b*b*b+.065*x*x;
        const double dy=-.114/.40*b*b+.130*x;
        const double d2=.228/(.40*.40)*b+.130;
        const double g=std::hypot(1.,dy);
        ReferencePoint p;
        p.s=p.x=x;p.y=y;p.yaw=std::atan(dy);p.curvature=d2/(g*g*g);
        spatial.push_back(p);
    }
    auto reference=sampled(spatial);reference.initial_body_yaw=0.;const auto original=reference;
    const auto course=P7StraightTrackingReference::conditionSupportedCourse(spatial,reference);
    assert(course.blend>.60);
    assert(course.curvature>.09&&course.curvature<.16&&course.straight_blend<.1);
    assert(curvatureEnergy(reference)<.4*curvatureEnergy(original));
    assert(near(reference.points[15].y,original.points[15].y,.01));
    unchangedTiming(original,reference);
    auto established=original;
    assert(P7StraightTrackingReference::conditionSupportedCourse(spatial,established,.85).blend==0.);
    for(int i=0;i<established.count;++i)unchangedGeometry(original.points[i],established.points[i]);
    for(int shape=0;shape<4;++shape){
        auto curve=curvedPath(shape);
        // Provide complete support beyond 1.42 m so these are genuine shape
        // rejections. Keep each original pattern's final local curvature.
        while(curve.back().s<1.48){
            auto p=curve.back();
            p.s+=.02;p.x+=.02*std::cos(p.yaw);p.y+=.02*std::sin(p.yaw);p.yaw+=.02*p.curvature;
            curve.push_back(p);
        }
        auto ref=sampled(curve);ref.initial_body_yaw=kYaw;
        assert(P7StraightTrackingReference::conditionSupportedCourse(curve,ref).blend==0.);
    }
    Reference stopped=original;stopped.terminal_approach=true;
    assert(P7StraightTrackingReference::conditionSupportedCourse(spatial,stopped).blend==0.);
    auto const short_path=nearLine(.60);
    auto short_reference=sampled(short_path);
    assert(P7StraightTrackingReference::conditionSupportedCourse(short_path,short_reference).blend==0.);
}

void checkYawRateCost() {
    using namespace research_control;
    assert(P7StraightTrackingReference::yawDampingWeight(.55, 2.)==0.);
    assert(near(P7StraightTrackingReference::yawDampingWeight(.75, 2.),1.));
    assert(P7StraightTrackingReference::yawDampingWeight(.95, 2.)==2.);
    assert(P7StraightTrackingReference::yawDampingWeight(missing(), 2.)==0.);
    assert(P7StraightTrackingReference::courseYawWeight({1.,1.,0.},2.)==0.);
    assert(near(P7StraightTrackingReference::courseYawWeight({.8,0.,.12},2.),1.6));
    SolverConfig config;
    SolveRequest request;
    request.now=10.;request.speed_cap=.30;
    request.previous=request.baseline={.24,.08};
    request.state={-.3,0.,0.,.24,.08,0.};
    request.history.push_back({9.,request.previous});
    request.reference.valid=true;request.reference.count=config.steps+1;
    for(int i=0;i<request.reference.count;++i) {
        auto& p=request.reference.points[i];
        p.t=i*config.dt;p.x=p.s=.24*p.t;p.speed=.24;
    }
    std::array<Command,kMaxHorizon> commands;
    commands.fill(request.previous);
    const auto rollout=[&](){
        return ReducedNMPC::validateControls(request,config,commands,
            std::chrono::steady_clock::now()+std::chrono::milliseconds(100));
    };
    const auto base=rollout();
    request.straight_yaw_damping_weight=2.;
    const auto damped=rollout();
    assert(base.feasible&&damped.feasible&&base.violation==damped.violation);
    assert(base.result.command.steering==damped.result.command.steering);
    const double omega=.24*std::tan(.08)/config.model.wheelbase;
    const double horizon=request.control_dt+(config.steps-1)*config.dt;
    assert(near(damped.result.objective-base.result.objective,
                2.*(horizon+.4)*omega*omega,1e-9));
    request.straight_yaw_damping_weight=0.;
    request.supported_course_yaw_weight=2.;
    const auto course=rollout();
    assert(course.feasible&&course.violation==base.violation);
    assert(near(course.result.objective,damped.result.objective));
}
}

int main() {
    checkYawRateCost();
    checkSupportedCourse();
    const auto spatial = nearLine();
    const Reference before = sampled(spatial);
    Reference straight = before;
    const double blend=P7StraightTrackingReference::condition(spatial,straight);
    assert(blend>0. && blend<=1.);
    assert(curvatureEnergy(straight) < .65*curvatureEnergy(before));
    unchangedTiming(before,straight);
    for(int i=0;i<straight.count;++i) {
        const auto& p=straight.points[i];
        const auto local=transform(p,-kYaw,0.,0.);
        // Condition the tracking reference toward the observed lead line,
        // never the follower's own zero lateral offset or heading.
        assert(near(local.y,kOffset,.007));
        assert(near(local.yaw,0.,.060));
    }

    // The result must not depend on the local frame or the yaw wrap boundary.
    constexpr double theta=3.11, dx=.77, dy=-.42;
    auto spatial_rotated=spatial;
    auto ref_rotated=before;
    for(auto& p:spatial_rotated)p=transform(p,theta,dx,dy);
    for(int i=0;i<ref_rotated.count;++i)
        ref_rotated.points[i]=transform(ref_rotated.points[i],theta,dx,dy);
    ref_rotated.endpoint=transform(ref_rotated.endpoint,theta,dx,dy);
    assert(near(P7StraightTrackingReference::condition(spatial_rotated,ref_rotated),blend));
    for(int i=0;i<ref_rotated.count;++i) {
        const auto rotated=transform(straight.points[i],theta,dx,dy);
        const auto& checked=ref_rotated.points[i];
        assert(near(rotated.x,checked.x)&&near(rotated.y,checked.y));
        assert(near(wrap(rotated.yaw-checked.yaw),0.));
        assert(near(rotated.curvature,checked.curvature));
    }

    // Preserve genuine bends, including an S with near-zero net turn and
    // a turn after a straight entry.
    for(int shape=0;shape<4;++shape) {
        const auto curved=curvedPath(shape);
        verifyBypass(curved,sampled(curved));
    }
    const auto short_path=nearLine(.60);
    verifyBypass(short_path,sampled(short_path));
    for(int mode=0;mode<3;++mode) {
        Reference guarded=before;
        if(mode==0)guarded.terminal_approach=true;
        if(mode==1)guarded.braking=true;
        if(mode==2)guarded.valid=false;
        verifyBypass(spatial,guarded);
    }
    std::cout<<"PASS straight tracking curvature attenuation, observed offset, frame invariance, bend and guard bypasses\n";
}
