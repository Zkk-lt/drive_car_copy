#include "control/p6_flatness_nmpc.h"
#include <cassert>
#include <iostream>
#include <iomanip>
using namespace research_control;
namespace {
using Flat=P6FlatnessNMPC;
bool near(double a,double b,double tolerance=1e-9){return std::abs(a-b)<tolerance;}
SolveRequest request(){
  SolveRequest r;r.now=10.;r.state.speed=.24;r.state.x=-.30;r.previous.speed=.24;r.baseline=r.previous;
  r.speed_cap=.30;r.history.push_back({9.,r.previous});r.reference.valid=true;r.reference.count=21;
  for(int i=0;i<=20;++i){auto& p=r.reference.points[i];p.t=.1*i;p.x=.24*p.t;p.s=p.x;p.speed=.24;}
  return r;
}
int rank(double J[8][8]){
  int n=0;for(int k=0;k<8;++k){int pivot=n;for(int i=n;i<8;++i)if(std::abs(J[i][k])>std::abs(J[pivot][k]))pivot=i;
    if(std::abs(J[pivot][k])<1e-10)continue;
    for(int j=k;j<8;++j)std::swap(J[pivot][j],J[n][j]);
    const double p=J[n][k];for(int j=k;j<8;++j)J[n][j]/=p;
    for(int i=0;i<8;++i)if(i!=n){const double f=J[i][k];for(int j=k;j<8;++j)J[i][j]-=f*J[n][j];}
    ++n;
  }return n;
}
void testBasis(){
  State s;s.x=.3;s.y=-.2;s.yaw=.4;s.speed=.24;s.steering=.1;s.acceleration=.04;Model m;
  for(double fraction:{.2,.5,.8}){
    auto curve=Flat::anchoredCurve(s,m,2.,fraction);
    const auto a=Flat::jet(curve,0.);const double co=std::cos(s.yaw),si=std::sin(s.yaw),normal=s.speed*s.speed*std::tan(s.steering)/m.wheelbase;
    assert(near(a.x,s.x)&&near(a.y,s.y));assert(near(a.dx,s.speed*co)&&near(a.dy,s.speed*si));
    assert(near(a.ddx,s.acceleration*co-normal*si)&&near(a.ddy,s.acceleration*si+normal*co));
    assert(near(a.dddx,0.)&&near(a.dddy,0.));
    const auto flat=FlatnessReferenceModel::reconstruct(a,m);assert(flat.valid&&near(flat.state.steering,s.steering));
    for(double t:{0.,.23,.74,1.31,2.}){
      const auto B=Flat::basis(curve,t);for(int d=0;d<=3;++d){double sum=0.;for(double b:B.d[d])sum+=b;assert(near(sum,d?0.:1.,1e-8));}
      const auto q=Flat::jet(curve,t);assert(near(q.x,s.x+s.speed*co*t+.5*(s.acceleration*co-normal*si)*t*t));
    }
    curve.control[0][3]+=.013;curve.control[1][4]-=.017;curve.control[1][6]+=.008;
    const double t=.63,h=1e-5;const auto before=Flat::jet(curve,t-h),at=Flat::jet(curve,t),after=Flat::jet(curve,t+h);
    assert(near((after.x-before.x)/(2*h),at.dx,1e-8));assert(near((after.dy-before.dy)/(2*h),at.ddy,1e-8));
    assert(near((after.ddx-before.ddx)/(2*h),at.dddx,1e-7));
    const double knot=curve.knots[6],eps=1e-8;const auto left=Flat::jet(curve,knot-eps),right=Flat::jet(curve,knot+eps);
    assert(near(left.x,right.x,1e-7)&&near(left.dddy,right.dddy,1e-5));
  }
}
void testGeometryOnlyCurvatureRecovery(){
  std::vector<ReferencePoint> circle;
  for(double s:{0.,.10,.35,.80,1.40}){ReferencePoint p;p.s=s;
    p.x=std::sin(.4*s)/.4;p.y=(1.-std::cos(.4*s))/.4;p.yaw=.4*s;circle.push_back(p);}
  const auto original=circle;
  assert(FlatnessReferenceModel::recoverMissingSpatialCurvature(circle));
  for(std::size_t i=0;i<circle.size();++i){
    assert(near(circle[i].curvature,.4,1e-12));
    assert(circle[i].x==original[i].x&&circle[i].y==original[i].y&&
      circle[i].s==original[i].s&&circle[i].yaw==original[i].yaw);}
  // A second call preserves existing supplied/derived curvature exactly.
  assert(!FlatnessReferenceModel::recoverMissingSpatialCurvature(circle));
  auto wrapped=original;for(auto& p:wrapped)p.yaw=wrap(3.05+.4*p.s);
  assert(FlatnessReferenceModel::recoverMissingSpatialCurvature(wrapped));
  for(const auto& p:wrapped)assert(near(p.curvature,.4,1e-12));
  // Nonuniform stations need a weighted three-point derivative: an arithmetic
  // average of adjacent secants is incorrect for this quadratic heading.
  auto quadratic=original;for(auto& p:quadratic)p.yaw=.1*p.s*p.s;
  assert(FlatnessReferenceModel::recoverMissingSpatialCurvature(quadratic));
  for(std::size_t i=1;i+1<quadratic.size();++i)
    assert(near(quadratic[i].curvature,.2*quadratic[i].s,1e-12));
  assert(near(quadratic.front().curvature,.01,1e-12));
  assert(near(quadratic.back().curvature,.22,1e-12));
  auto supplied=original;supplied[2].curvature=.123;
  assert(!FlatnessReferenceModel::recoverMissingSpatialCurvature(supplied));
  for(std::size_t i=0;i<supplied.size();++i)assert(supplied[i].curvature==(i==2?.123:0.));
  auto bad=original;bad[2].s=bad[1].s;
  assert(!FlatnessReferenceModel::recoverMissingSpatialCurvature(bad));
  for(const auto& p:bad)assert(p.curvature==0.);
  bad=original;bad[2].yaw=missing();
  assert(!FlatnessReferenceModel::recoverMissingSpatialCurvature(bad));
  auto straight=original;for(auto& p:straight)p.yaw=.3;
  assert(!FlatnessReferenceModel::recoverMissingSpatialCurvature(straight));
  std::cout<<"PASS research-only geometry curvature derivation: circle, nonuniform stations, wrapped yaw, existing curvature and invalid geometry preserved\n";
}
SolveRequest responseReference(int samples,double turn_time,SolverConfig& c){
  auto r=request();c.steps=samples;c.dt=2./samples;r.control_dt=c.dt;
  r.reference.count=samples+1;
  const double speed=.24,width=.12,amplitude=.40;
  double x=0.,y=0.;
  auto heading=[&](double t){return .5*amplitude*(1.+std::tanh((t-turn_time)/width));};
  for(int i=0;i<=samples;++i){const double t=i*c.dt;
    if(i){const double yaw=heading(t-.5*c.dt);x+=speed*c.dt*std::cos(yaw);y+=speed*c.dt*std::sin(yaw);}
    auto& p=r.reference.points[i];p.t=t;p.s=speed*t;p.x=x;p.y=y;p.speed=speed;
    p.yaw=heading(t);const double a=std::tanh((t-turn_time)/width);
    p.curvature=.5*amplitude*(1.-a*a)/(width*speed);
  }
  return r;
}
void testCausalRearReferenceAndResponseKnot(){
  SolverConfig c;auto r=responseReference(20,.40,c);
  const auto lifted=Flat::localRearReference(r,c);assert(lifted.valid);
  // The rear-axle lift must commute with a rigid local-frame change. This
  // checks the center/body convention, including wrapped body/tangent yaw.
  const double angle=2.8,co=std::cos(angle),si=std::sin(angle),tx=1.7,ty=-.8;
  auto transformed=r;
  const double sx=r.state.x,sy=r.state.y;
  transformed.state.x=tx+co*sx-si*sy;transformed.state.y=ty+si*sx+co*sy;
  transformed.state.yaw=wrap(r.state.yaw+angle);
  for(int i=0;i<transformed.reference.count;++i){auto& p=transformed.reference.points[i];
    const double x=p.x,y=p.y;p.x=tx+co*x-si*y;p.y=ty+si*x+co*y;p.yaw=wrap(p.yaw+angle);}
  const auto moved=Flat::localRearReference(transformed,c);assert(moved.valid);
  for(int i=0;i<c.steps;++i){
    assert(near(moved.times[i],lifted.times[i],1e-12));
    assert(near(moved.positions[i][0],tx+co*lifted.positions[i][0]-si*lifted.positions[i][1],1e-11));
    assert(near(moved.positions[i][1],ty+si*lifted.positions[i][0]+co*lifted.positions[i][1],1e-11));
    assert(near(moved.body_rates[i],lifted.body_rates[i],1e-11));
  }
  // A finite tangent trajectory needs no instantaneous steady-circle inverse.
  // Deliberately inconsistent high curvature metadata is NOT a promise that
  // a reconstructed/optimized control will satisfy physical constraints.
  auto high_curvature=r;for(int i=0;i<high_curvature.reference.count;++i)high_curvature.reference.points[i].curvature=10.;
  assert(c.model.rear_to_center*high_curvature.reference.points[0].curvature>1.);
  const auto no_steady_inverse=Flat::localRearReference(high_curvature,c);assert(no_steady_inverse.valid);
  for(int i=0;i<c.steps;++i){
    assert(near(no_steady_inverse.positions[i][0],lifted.positions[i][0],1e-12));
    assert(near(no_steady_inverse.positions[i][1],lifted.positions[i][1],1e-12));
    assert(near(no_steady_inverse.body_rates[i],lifted.body_rates[i],1e-12));
  }
  // Early and late visible heading changes should move the one knot without
  // consulting a route/terrain label. Refining the SAME smooth signal should
  // retain its time-variation mass, rather than reverting toward H/2.
  SolverConfig fine,late_config;const auto fine_request=responseReference(40,.40,fine);
  const auto late=responseReference(20,1.40,late_config);
  const double early_knot=Flat::responseKnotFraction(r,c,2.,lifted);
  const double fine_knot=Flat::responseKnotFraction(fine_request,fine,2.,Flat::localRearReference(fine_request,fine));
  const double late_knot=Flat::responseKnotFraction(late,late_config,2.,Flat::localRearReference(late,late_config));
  assert(early_knot<.5&&late_knot>.5&&late_knot-early_knot>.20);
  assert(std::abs(fine_knot-early_knot)<.035);
  assert(near(early_knot,Flat::responseKnotFraction(transformed,c,2.,moved),1e-11));
  auto straight=r;for(int i=0;i<straight.reference.count;++i){auto& p=straight.reference.points[i];
    p.x=.24*p.t;p.y=0.;p.yaw=0.;p.curvature=0.;}
  const auto straight_rear=Flat::localRearReference(straight,c);assert(straight_rear.valid);
  assert(near(Flat::responseKnotFraction(straight,c,2.,straight_rear),.5,1e-12));
  auto invalid=r;invalid.state.yaw=missing();assert(!Flat::localRearReference(invalid,c).valid);
  assert(near(Flat::responseKnotFraction(invalid,c,2.,Flat::LocalRearReference{}),.5));
  std::cout<<"PASS causal center/rear lift SE2 covariance, finite high-curvature seed, visible turn response knots "
    <<early_knot<<"/"<<late_knot<<", N20/N40 consistency "<<fine_knot<<" and straight half-horizon\n";
}
void testEightDimensionalChart(){
  auto r=request();SolverConfig c;const double H=r.control_dt+(c.steps-1)*c.dt;
  auto anchor=Flat::anchoredCurve(r.state,c.model,H);Flat::Coefficients z{};Flat::Curve base;double error=0.;
  assert(Flat::curveFromDecision(anchor,z,r,c,base,true,&error)&&error<1e-10);
  const auto first=FlatnessReferenceModel::reconstruct(Flat::jet(base,r.control_dt),c.model);
  assert(first.valid&&near(first.feedforward.speed,r.previous.speed,1e-10)&&near(first.feedforward.steering,r.previous.steering,1e-10));
  double jacobian[8][8]{};const double eps=1e-5;
  for(int coordinate=0;coordinate<8;++coordinate){
    auto shifted=z;shifted[coordinate]+=eps;Flat::Curve q;assert(Flat::curveFromDecision(anchor,shifted,r,c,q));
    for(int axis=0;axis<2;++axis)for(int cp=3;cp<7;++cp)jacobian[axis*4+cp-3][coordinate]=(q.control[axis][cp]-base.control[axis][cp])/eps;
    const auto input=FlatnessReferenceModel::reconstruct(Flat::jet(q,r.control_dt),c.model);assert(input.valid);
    if(coordinate==0)assert(std::abs(input.feedforward.speed-first.feedforward.speed)>1e-10);
    if(coordinate==4)assert(std::abs(input.feedforward.steering-first.feedforward.steering)>1e-8);
  }
  assert(rank(jacobian)==8);
  // All remaining coefficients really change spline shape, with the two first
  // input coordinates preserved by inversion rather than by output clipping.
  for(int k:{1,2,3,5,6,7}){auto shifted=z;shifted[k]+=.001;Flat::Curve q;assert(Flat::curveFromDecision(anchor,shifted,r,c,q));
    const auto mid=Flat::jet(q,H*.7),original=Flat::jet(base,H*.7);assert(std::hypot(mid.x-original.x,mid.y-original.y)>1e-11);
    const auto first_q=FlatnessReferenceModel::reconstruct(Flat::jet(q,r.control_dt),c.model);
    assert(near(first_q.feedforward.speed,first.feedforward.speed,1e-9)&&near(first_q.feedforward.steering,first.feedforward.steering,1e-9));
  }
}
void testIndependentFirstInputEncoding(){
  auto r=request();SolverConfig c;
  for(bool delayed:{false,true}){
    auto anchor=Flat::anchoredCurve(r.state,c.model,r.control_dt+(c.steps-1)*c.dt);
    anchor.delay_aligned=delayed;anchor.decision_scale={{.001,.001}};
    Flat::Coefficients desired{};desired[0]=.10;desired[4]=.05;Flat::Curve q;
    assert(Flat::curveFromDecision(anchor,desired,r,c,q));
    // P6 has no support at the first command sample. A large far-point fit
    // therefore provides a direct counterexample to whole-vector rescaling.
    q.control[0][6]+=.05;q.control[1][6]-=.025;
    const auto old=Flat::encodeCurve(anchor,q,r,c);
    const auto fixed=Flat::encodeCurve(anchor,q,r,c,false,true);
    assert(near(fixed[0],desired[0],1e-8)&&near(fixed[4],desired[4],1e-8));
    assert(std::abs(old[0]-desired[0])>.01&&std::abs(old[4]-desired[4])>.01);
    for(int j:{1,2,3,5,6,7})assert(std::abs(fixed[j])<=1.+1e-12);
    Flat::Curve reconstructed;assert(Flat::curveFromDecision(anchor,fixed,r,c,reconstructed));
    Command first,roundtrip;const double t=delayed?0.:r.control_dt;
    assert(Flat::commandAt(q,t,c.model,first)&&Flat::commandAt(reconstructed,t,c.model,roundtrip));
    assert(near(first.speed,roundtrip.speed,1e-9)&&near(first.steering,roundtrip.steering,1e-9));
  }
}
void testPhysicalInputProjection(){
  auto r=request();SolverConfig c;
  auto q=Flat::anchoredCurve(r.state,c.model,r.control_dt+(c.steps-1)*c.dt);
  q.delay_aligned=true;auto target_curve=q;
  target_curve.control[1][4]+=.000002;target_curve.control[1][5]+=.000004;
  std::array<Command,kMaxHorizon> target{};
  for(int i=0;i<c.steps;++i){const double t=i? r.control_dt+(i-1)*c.dt:0.;
    assert(Flat::commandAt(target_curve,t,c.model,target[i]));}
  auto error=[&](const Flat::Curve& curve){double result=0.;for(int i=0;i<c.steps;++i){
    const double t=i?r.control_dt+(i-1)*c.dt:0.;Command value;assert(Flat::commandAt(curve,t,c.model,value));
    result+=std::pow(value.speed-target[i].speed,2)+std::pow(value.steering-target[i].steering,2);}
    return result;};
  const double before=error(q);assert(before>1e-10);
  assert(Flat::projectCommands(q,target,r,c));const double after=error(q);
  assert(after<before*.1);
  for(int axis=0;axis<2;++axis)for(int cp=0;cp<3;++cp)
    assert(q.control[axis][cp]==target_curve.control[axis][cp]);
  assert(Flat::dimension()==8);
  std::cout<<"PASS flat8 analytic issued-command projection "<<before<<" -> "<<after<<" preserves initial jet\n";
}
void testJitteredWarmStartAndUncontrollablePrefix(){
  auto r=request();r.state.y=-.01;SolverConfig c;c.budget_ms=1000.;
  Flat solver;solver.setConditionedGeometry(true);solver.setBaselineSeed(true);
  solver.setDelayAlignedReconstruction(true);solver.setCommandProjection(true);
  const auto first=solver.solve(r,c);assert(first.ok&&first.feasible);
  auto next=r;next.now+=r.control_dt;next.control_dt=.034;
  next.state=first.predicted[1];next.previous=first.command;
  next.previous_command_acceleration=first.command_acceleration;
  next.history.push_back({r.now,first.command});
  for(int i=0;i<next.reference.count;++i)next.reference.points[i].x+=.24*r.control_dt;
  const auto shifted=solver.solve(next,c);
  assert(shifted.warm_available&&solver.diagnostics().warm_geometry_preserved&&
    std::isfinite(solver.diagnostics().warm_fit_rmse));
  assert(shifted.ok&&shifted.feasible&&shifted.decision_dimension==8);
  const auto valid=ReducedNMPC::validateControls(next,c,shifted.controls,
    std::chrono::steady_clock::now()+std::chrono::seconds(1));
  assert(valid.feasible);
  auto impossible=request();impossible.history.front().command.speed=.36;
  const auto rejected=solver.solve(impossible,c);
  assert(!rejected.ok&&!rejected.feasible&&rejected.evaluations==0&&
    rejected.reason=="flatness_uncontrollable_history_infeasible");
  std::array<Command,kMaxHorizon> any{};for(auto& u:any)u=impossible.previous;
  const auto oracle=ReducedNMPC::validateControls(impossible,c,any,
    std::chrono::steady_clock::now()+std::chrono::seconds(1));
  assert(!oracle.feasible&&oracle.reason=="delayed_physical_state_constraint_violation");
  impossible=request();impossible.state.acceleration=c.limits.acceleration+.01;
  const auto initial=solver.solve(impossible,c);
  assert(!initial.ok&&initial.evaluations==0&&initial.reason=="flatness_initial_physical_state_infeasible");
  std::cout<<"PASS jittered actual dt warm refit, full constraint revalidation, proven uncontrollable-prefix rejection without spline polls\n";
}
void testSaturatedSpeedCoupledSplineSeed(){
  auto r=request();r.state.speed=r.previous.speed=.4;r.baseline=r.previous;r.speed_cap=.4;
  SolverConfig c;auto curve=Flat::anchoredCurve(r.state,c.model,r.control_dt+(c.steps-1)*c.dt);
  curve.delay_aligned=true;curve.control[1][5]+=.00001;curve.control[1][6]+=.00002;
  const auto fixed=curve;
  auto maxSpeed=[&](const Flat::Curve& q){double value=0.;for(int i=0;i<c.steps;++i){
    Command u;assert(Flat::commandAt(q,i?r.control_dt+(i-1)*c.dt:0.,c.model,u));value=std::max(value,u.speed);}return value;};
  const double before=maxSpeed(curve);assert(before>r.speed_cap);
  assert(Flat::coupledSpeedRetreat(curve,r,c));
  assert(maxSpeed(curve)<before);
  for(int axis=0;axis<2;++axis)for(int cp=0;cp<3;++cp)assert(curve.control[axis][cp]==fixed.control[axis][cp]);
  assert(Flat::dimension()==8);
  // A commanded corrective turn at the saturated speed must have a coupled
  // feasible descent direction, without relying on timeout/S8 fallback to move
  // the vehicle out of a zero-steering fixed point. Synthetic nominal fixture.
  r.state={-.3,.035,.025,.4,0.,0.};r.previous.steering=0.;r.baseline.steering=-.10;
  r.history.clear();r.history.push_back({9.,r.previous});
  for(int i=0;i<r.reference.count;++i){auto& p=r.reference.points[i];p.x=.4*p.t;p.s=p.x;p.speed=.4;}
  c.budget_ms=1000.;Flat solver;solver.setConditionedGeometry(true);solver.setBaselineSeed(true);
  solver.setDelayAlignedReconstruction(true);solver.setCommandProjection(true);
  const auto result=solver.solve(r,c);
  assert(result.ok&&result.feasible&&result.command.steering< -1e-7);
  const auto checked=ReducedNMPC::validateControls(r,c,result.controls,
    std::chrono::steady_clock::now()+std::chrono::seconds(1));assert(checked.feasible);
  std::cout<<"PASS saturated-speed normal/tangent seed lowers second-order speed overshoot, preserves eight CP DOFs and initial jet\n";
}
void testTrackingGaussNewtonGradient(){
  auto r=request();r.state.y=.015;r.state.yaw=.025;
  SolverConfig c;c.weights.speed=12.;c.weights.smoothness=32.;
  auto curve=Flat::anchoredCurve(r.state,c.model,r.control_dt+(c.steps-1)*c.dt);
  curve.delay_aligned=true;curve.decision_scale={{.001,.001}};
  // Exercise unequal near/far CP conditioning; analytic columns and physical
  // updates must use the SAME scales rather than a shared P3-dominated value.
  curve.geometry_scale[1]=.002;curve.geometry_scale[2]=.003;curve.geometry_scale[3]=.004;
  curve.geometry_scale[5]=.0025;curve.geometry_scale[6]=.0035;curve.geometry_scale[7]=.0045;
  const auto coordinate_scales=Flat::trackingScales(curve,r,c);
  assert(coordinate_scales[0]==.001&&coordinate_scales[4]==.001&&
         coordinate_scales[3]==.004&&coordinate_scales[7]==.0045);
  auto oracle=[&](const Flat::Curve& q){std::array<Command,kMaxHorizon> commands{};
    for(int i=0;i<c.steps;++i)assert(Flat::commandAt(q,i?r.control_dt+(i-1)*c.dt:0.,c.model,commands[i]));
    return ReducedNMPC::validateControls(r,c,commands,std::chrono::steady_clock::now()+std::chrono::seconds(1));};
  const auto original=oracle(curve);assert(original.feasible);
  Flat::Coefficients direction{},gradient{};double objective=missing();
  assert(Flat::trackingDirection(curve,r,c,direction,objective,&gradient));
  assert(near(objective,original.cost,1e-11));
  const double epsilon=1e-4;
  for(int coordinate=0;coordinate<8;++coordinate){
    auto before=curve,after=curve;Flat::Coefficients perturb{};perturb[coordinate]=1.;
    Flat::applyTrackingDirection(before,perturb,-epsilon,r,c);
    Flat::applyTrackingDirection(after,perturb,epsilon,r,c);
    // The oracle computes the identical nonlinear cost on both perturbations.
    // Its result may be physically infeasible on a constraint boundary, so use
    // the independently propagated analytic residual sum there, never call an
    // infeasible perturbation an accepted control.
    Flat::Coefficients unused{};double a=missing(),b=missing();
    assert(Flat::trackingDirection(before,r,c,unused,a)&&Flat::trackingDirection(after,r,c,unused,b));
    const double numerical=(b-a)/(2.*epsilon);
    assert(std::abs(numerical-gradient[coordinate])<1e-6*std::max(1.,std::abs(numerical)));
    const auto left=oracle(before),right=oracle(after);
    if(left.feasible&&right.feasible)assert(std::abs((right.cost-left.cost)/(2.*epsilon)-gradient[coordinate])<1e-6);
  }
  bool improved=false;
  for(double alpha:{1.,.5,.25,.125,.0625}){auto candidate=curve;
    Flat::applyTrackingDirection(candidate,direction,alpha,r,c);
    const auto checked=oracle(candidate);
    for(int axis=0;axis<2;++axis)for(int cp=0;cp<3;++cp)assert(candidate.control[axis][cp]==curve.control[axis][cp]);
    if(checked.feasible&&checked.cost<original.cost)improved=true;
  }
  assert(improved);
  std::cout<<"PASS analytic preconditioned eight-CP delayed tracking objective/gradient, shared update scales, common Core cost, initial jet and feasible descent\n";
}
void testCachedFlatCommandDifferential(){
  auto r=request();SolverConfig c;
  for(bool delayed:{false,true}){
    auto q=Flat::anchoredCurve(r.state,c.model,r.control_dt+(c.steps-1)*c.dt);
    q.delay_aligned=delayed;q.control[1][4]+=.000002;q.control[0][6]-=.000001;
    for(double t:{0.,.033,.1,.7,1.9}){
      const auto B=Flat::basis(q,Flat::inputTime(q,t,c.model.speed_delay));
      const auto D=delayed?Flat::basis(q,Flat::inputTime(q,t,c.model.steering_delay)):B;
      Flat::CommandDifferentialCache cache;assert(Flat::prepareCommandDifferential(q,B,D,c.model,cache));
      Command original;assert(Flat::commandAt(q,t,c.model,original));
      assert(original.speed==cache.speed_flat.feedforward.speed);
      assert(original.steering==(delayed?cache.steering_flat.feedforward.steering:cache.speed_flat.feedforward.steering));
      for(int cp=3;cp<7;++cp){double before[2][2]{},after[2][2]{};
        assert(Flat::commandJacobian(q,B,D,c.model,before,cp));
        assert(Flat::cachedCommandJacobian(q,B,D,c.model,cache,after,cp));
        for(int a=0;a<2;++a)for(int b=0;b<2;++b)assert(before[a][b]==after[a][b]);
      }
    }
  }
  std::cout<<"PASS shared nominal jet/reconstruction cache matches uncached commands and all CP Jacobians exactly\n";
}

void testAnalyticFirstInputJacobian(){
  // A central-difference oracle checks both coupled actuator inverses across
  // speed, heading, gains and additive residuals. These are synthetic calculus
  // checks, not physical calibration or closed-loop tracking evidence.
  int checked=0;
  for(double speed:{.025,.10,.24,.39})for(double yaw:{-.8,0.,.7}){
    Model m;m.steering_gain=.87;m.speed_gain=1.12;m.yaw_gain=.92;
    m.yaw_residual=.003;m.longitudinal_residual=-.004;
    State s;s.speed=speed;s.yaw=yaw;s.steering=.08;s.acceleration=.012;
    const auto curve=Flat::anchoredCurve(s,m,1.933333333333,.05);
    const auto B=Flat::basis(curve,1./30.);double J[2][2]{};
    assert(Flat::firstInputJacobian(Flat::jet(curve,B),B,m,J));
    for(int axis=0;axis<2;++axis){
      auto left=curve,right=curve;const double h=1e-9;
      left.control[axis][3]-=h;right.control[axis][3]+=h;
      const auto lo=FlatnessReferenceModel::reconstruct(Flat::jet(left,B),m);
      const auto hi=FlatnessReferenceModel::reconstruct(Flat::jet(right,B),m);
      assert(lo.valid&&hi.valid);
      const double oracle[2]={(hi.feedforward.speed-lo.feedforward.speed)/(2.*h),
        (hi.feedforward.steering-lo.feedforward.steering)/(2.*h)};
      for(int channel=0;channel<2;++channel)
        assert(std::abs(J[channel][axis]-oracle[channel])<1e-4*(1.+std::abs(oracle[channel])));
      ++checked;
    }
  }
  assert(checked==24);
}
void testDelayAlignedChart(){
  auto r=request();SolverConfig c;c.budget_ms=1000.;
  auto anchor=Flat::anchoredCurve(r.state,c.model,r.control_dt+(c.steps-1)*c.dt);
  anchor.delay_aligned=true;
  // Two distinct activation times produce a two-row actuator Jacobian.
  const auto Bv=Flat::basis(anchor,Flat::inputTime(anchor,0.,c.model.speed_delay));
  const auto Bd=Flat::basis(anchor,Flat::inputTime(anchor,0.,c.model.steering_delay));
  for(int cp=3;cp<=6;++cp){double J[2][2]{};assert(Flat::commandJacobian(anchor,Bv,Bd,c.model,J,cp));
    for(int axis=0;axis<2;++axis){auto left=anchor,right=anchor;const double h=1e-8;
      left.control[axis][cp]-=h;right.control[axis][cp]+=h;Command lo,hi;
      assert(Flat::commandAt(left,0.,c.model,lo)&&Flat::commandAt(right,0.,c.model,hi));
      const double oracle[2]={(hi.speed-lo.speed)/(2.*h),(hi.steering-lo.steering)/(2.*h)};
      for(int channel=0;channel<2;++channel)
        assert(std::abs(oracle[channel]-J[channel][axis])<1e-4*(1.+std::abs(oracle[channel])));
    }
  }
  Flat::Coefficients z{};Flat::Curve base;double error=0.;
  assert(Flat::curveFromDecision(anchor,z,r,c,base,true,&error,true)&&error<1e-10);
  Command first;assert(Flat::commandAt(base,0.,c.model,first));
  assert(near(first.speed,r.previous.speed,1e-10)&&near(first.steering,r.previous.steering,1e-10));
  double J[8][8]{};const double eps=1e-5;
  for(int coordinate=0;coordinate<8;++coordinate){auto changed=z;changed[coordinate]+=eps;Flat::Curve q;
    assert(Flat::curveFromDecision(anchor,changed,r,c,q,true,nullptr,true));
    for(int axis=0;axis<2;++axis)for(int cp=3;cp<=6;++cp)
      J[axis*4+cp-3][coordinate]=(q.control[axis][cp]-base.control[axis][cp])/eps;
    Command value;assert(Flat::commandAt(q,0.,c.model,value));
    if(coordinate!=0&&coordinate!=4)assert(near(value.speed,first.speed,1e-9)&&near(value.steering,first.steering,1e-9));
  }
  assert(rank(J)==8);
  Command endpoint,tail;assert(Flat::commandAt(base,base.horizon,c.model,endpoint));
  assert(Flat::commandAt(base,base.horizon+100.,c.model,tail));
  assert(near(endpoint.speed,tail.speed)&&near(endpoint.steering,tail.steering));
  Flat solver;solver.setDelayAlignedReconstruction(true);solver.setConditionedGeometry(true);
  const auto solved=solver.solve(r,c);
  assert(solved.ok&&solved.feasible&&solved.decision_dimension==8&&solver.diagnostics().delay_aligned_reconstruction);
  const auto independently=ReducedNMPC::validateControls(r,c,solved.controls,std::chrono::steady_clock::now()+std::chrono::seconds(1));
  assert(independently.feasible);
  c.model.speed_delay=0.;assert(solver.solve(r,c).reason=="flatness_delay_aligned_speed_channel_singular");
  std::cout<<"PASS delay-aligned separate actuator Jacobian, rank=8, first-input invariance, endpoint ZOH, full physical rollout\n";
}
void constraints(const SolveResult& out,const SolveRequest& r,const SolverConfig& c){
  assert(out.ok&&out.feasible&&out.decision_dimension==8&&out.count==c.steps+1&&out.max_violation<=1e-7);
  Command previous=r.previous;double a=r.previous_command_acceleration;
  for(int i=0;i<c.steps;++i){const double h=i?c.dt:r.control_dt;const auto& u=out.controls[i];
    const double next_a=(u.speed-previous.speed)/h;
    assert(u.speed>=-1e-10&&u.speed<=std::min(r.speed_cap,c.limits.speed)+1e-10);
    assert(std::abs(u.steering)<=c.limits.steering+1e-10&&std::abs(u.steering-previous.steering)<=c.limits.steering_rate*h+1e-10);
    assert(next_a<=c.limits.acceleration+1e-10&&next_a>=-c.limits.deceleration-1e-10);
    assert(std::abs(next_a-a)<=c.limits.jerk*h+1e-8);previous=u;a=next_a;
  }
}
void testConditionedGeometryAndBaselineSeed(){
  auto r=request();r.state.y=-.04;r.baseline.steering=.08;
  SolverConfig c;c.budget_ms=1000.;
  Flat old,improved;improved.setConditionedGeometry(true);improved.setBaselineSeed(true);
  const auto a=old.solve(r,c),b=improved.solve(r,c);constraints(a,r,c);constraints(b,r,c);
  assert(improved.diagnostics().conditioned_geometry&&improved.diagnostics().baseline_seed_accepted);
  assert(b.objective<a.objective-1e-5&&b.decision_dimension==a.decision_dimension);
  const auto stored=improved.lastCurve();
  auto anchor=Flat::anchoredCurve(r.state,c.model,stored.horizon,stored.knots[6]/stored.horizon);
  anchor.geometry_scale=stored.geometry_scale;
  Flat::Coefficients zero{};Flat::Curve origin;
  assert(Flat::curveFromDecision(anchor,zero,r,c,origin));
  const auto initial=FlatnessReferenceModel::reconstruct(Flat::jet(origin,r.control_dt),c.model);
  double J[8][8]{};const double eps=1e-5;
  for(int k=0;k<8;++k){
    auto z=zero;z[k]=eps;Flat::Curve shifted;assert(Flat::curveFromDecision(anchor,z,r,c,shifted));
    for(int axis=0;axis<2;++axis)for(int cp=3;cp<7;++cp)
      J[axis*4+cp-3][k]=(shifted.control[axis][cp]-origin.control[axis][cp])/eps;
    if(k!=0&&k!=4){
      assert(std::isfinite(anchor.geometry_scale[k])&&anchor.geometry_scale[k]>0.);
      const auto first=FlatnessReferenceModel::reconstruct(Flat::jet(shifted,r.control_dt),c.model);
      assert(first.valid&&near(first.feedforward.speed,initial.feedforward.speed,1e-10)&&near(first.feedforward.steering,initial.feedforward.steering,1e-10));
    }
  }
  assert(rank(J)==8);
  const auto independently=ReducedNMPC::validateControls(r,c,b.controls,std::chrono::steady_clock::now()+std::chrono::seconds(1));
  assert(independently.feasible);
  std::cout<<"PASS conditioned spline chart rank=8, first-input invariance, S8 trajectory seed improves objective "<<a.objective<<" -> "<<b.objective<<" with unchanged constraints\n";
}
void testResponseBaseFlatOptimization(){
  auto r=request();r.state.y=-.02;r.control_dt=1./30.;SolverConfig c;c.budget_ms=1000.;c.iterations=2;
  Flat solver;solver.setConditionedGeometry(true);solver.setBaselineSeed(true);
  solver.setDelayAlignedReconstruction(true);solver.setCommandProjection(true);solver.setResponseBase(true);
  const auto result=solver.solve(r,c);constraints(result,r,c);
  assert(solver.diagnostics().response_base_used);
  const double base_cost=solver.diagnostics().response_base_cost;
  assert(result.objective+1e-12<base_cost);
  const auto frozen=solver.lastCurve();assert(frozen.response_base&&frozen.response_base->valid);
  const auto first=Flat::jet(frozen,0.);assert(near(first.x,r.state.x)&&near(first.y,r.state.y));
  assert(near(first.dx,r.state.speed)&&near(first.dy,0.));
  const auto base_position=frozen.response_base->jets[0].x;
  auto next=r;next.now+=r.control_dt;next.state=result.predicted[1];next.previous=result.command;
  next.previous_command_acceleration=result.command_acceleration;next.history.push_back({r.now,result.command});
  solver.setResponseNearSupport(true);
  const auto second=solver.solve(next,c);constraints(second,next,c);
  assert(near(solver.diagnostics().interior_knot,std::max(c.model.steering_delay,c.model.speed_delay)+c.dt));
  assert(second.decision_dimension==8);
  assert(solver.diagnostics().response_base_used);
  assert(frozen.response_base->jets[0].x==base_position);
  assert(solver.lastCurve().response_base.get()!=frozen.response_base.get());
  std::cout<<"PASS response-model flat base, eight optimized coefficients, feasible zero correction, common-cost descent "
    <<base_cost<<" -> "<<result.objective<<" and immutable double-buffer snapshots\n";
}
void testResponseBaseWarmStart(){
  // Synthetic functional test with a wide CPU budget, not a realtime claim.
  // Only the response seed differs; both optimizers retain eight decisions.
  auto r=request();r.state.y=-.02;r.control_dt=1./30.;
  SolverConfig c;c.budget_ms=1000.;c.iterations=2;
  auto configure=[](Flat& f){f.setConditionedGeometry(true);f.setBaselineSeed(true);
    f.setDelayAlignedReconstruction(true);f.setCommandProjection(true);
    f.setResponseBase(true);f.setResponseNearSupport(true);};
  auto sameChart=[](const Flat::Curve& x,const Flat::Curve& y){
    assert(x.knots==y.knots&&x.response_base&&y.response_base);
    assert(x.response_base->count==y.response_base->count);
    for(int axis=0;axis<2;++axis)for(int k=0;k<3;++k)assert(near(x.control[axis][k],y.control[axis][k],1e-12));
    for(int i=0;i<x.response_base->count;++i){
      assert(x.response_base->times[i]==y.response_base->times[i]);
      const auto&a=x.response_base->jets[i];const auto&b=y.response_base->jets[i];
      const double av[]={a.x,a.y,a.dx,a.dy,a.ddx,a.ddy,a.dddx,a.dddy};
      const double bv[]={b.x,b.y,b.dx,b.dy,b.ddx,b.ddy,b.dddx,b.dddy};
      for(int j=0;j<8;++j)assert(near(av[j],bv[j],1e-12));
    }
  };
  Flat cold,warm;configure(cold);configure(warm);warm.setResponseWarmStart(true);
  const auto a=cold.solve(r,c),b=warm.solve(r,c);constraints(a,r,c);constraints(b,r,c);
  assert(near(a.objective,b.objective,1e-12)&&!warm.diagnostics().response_warm_available);
  auto next=r;next.now+=r.control_dt;next.control_dt=.037;
  next.state=b.predicted[1];next.previous=b.command;
  next.previous_command_acceleration=b.command_acceleration;
  next.history.push_back({r.now,b.command});const auto history=next.history;
  const auto current=cold.solve(next,c),shifted=warm.solve(next,c);
  constraints(current,next,c);constraints(shifted,next,c);
  sameChart(cold.lastCurve(),warm.lastCurve());
  const auto unselected=warm.diagnostics();
  assert(unselected.response_warm_available&&unselected.response_warm_evaluations==1&&shifted.warm_available);
  assert(unselected.response_warm_cost_after<=unselected.response_warm_cost_before);
  if(!unselected.response_warm_accepted)assert(near(unselected.response_warm_cost_before,
    unselected.response_warm_cost_after,1e-12)&&near(current.objective,shifted.objective,1e-12));
  auto third=next;third.now+=next.control_dt;third.state=shifted.predicted[1];
  third.previous=shifted.command;third.previous_command_acceleration=shifted.command_acceleration;
  third.history.push_back({next.now,shifted.command});
  const auto third_cold=cold.solve(third,c),used=warm.solve(third,c);
  constraints(third_cold,third,c);constraints(used,third,c);
  sameChart(cold.lastCurve(),warm.lastCurve());
  const auto d=warm.diagnostics();
  assert(d.response_warm_available&&d.response_warm_evaluations==1&&used.warm_available);
  std::cerr<<"same-chart warm seed: accepted "<<unselected.response_warm_accepted<<"/"<<d.response_warm_accepted
    <<" cost "<<unselected.response_warm_cost_before<<" -> "<<unselected.response_warm_cost_after
    <<" then "<<d.response_warm_cost_before<<" -> "<<d.response_warm_cost_after<<"\n";
  assert(d.response_warm_accepted||unselected.response_warm_accepted);
  assert(d.response_warm_cost_after<=d.response_warm_cost_before);
  if(d.response_warm_accepted)assert(d.response_warm_cost_after<d.response_warm_cost_before);
  assert(near(d.response_warm_cost_before,cold.diagnostics().response_base_cost,1e-12));
  assert(near(d.response_warm_cost_before,d.response_base_cost,1e-12));
  assert(shifted.decision_dimension==8&&current.decision_dimension==8);
  assert(next.history.size()==history.size());
  for(std::size_t i=0;i<history.size();++i){assert(next.history[i].stamp==history[i].stamp);
    assert(next.history[i].command.speed==history[i].command.speed&&
      next.history[i].command.steering==history[i].command.steering);}
  const auto checked=ReducedNMPC::validateControls(third,c,used.controls,
    std::chrono::steady_clock::now()+std::chrono::seconds(1));
  assert(checked.feasible&&near(checked.cost,used.objective,1e-12));
  const auto initial=Flat::jet(warm.lastCurve(),0.);
  assert(near(initial.x,third.state.x)&&near(initial.y,third.state.y));
  assert(near(initial.dx,third.state.speed*std::cos(third.state.yaw))&&
    near(initial.dy,third.state.speed*std::sin(third.state.yaw)));
  // Expired predictions, reset, a failed solve and incomplete REAL history
  // cannot become warm execution history or bypass the original gates.
  auto expired=next;expired.now+=3.;assert(warm.solve(expired,c).ok);
  assert(!warm.diagnostics().response_warm_available);
  warm.reset();assert(warm.solve(r,c).ok&&!warm.diagnostics().response_warm_available);
  auto failed=next;failed.force_infeasible=true;assert(!warm.solve(failed,c).ok);
  assert(warm.solve(next,c).ok&&!warm.diagnostics().response_warm_available);
  failed=next;failed.now+=next.control_dt;failed.force_timeout=true;
  assert(!warm.solve(failed,c).ok);failed.force_timeout=false;
  assert(warm.solve(failed,c).ok&&!warm.diagnostics().response_warm_available);
  failed.history.clear();assert(warm.solve(failed,c).reason=="history_incomplete");
  assert(!warm.diagnostics().response_warm_available);
  std::cout<<"PASS same-chart ZOH warm seed projection, changed first interval, strict seed cost "
    <<d.response_warm_cost_before<<" -> "<<d.response_warm_cost_after
    <<", eight decisions, actual history untouched, independent full validation and invalidation\n";
}
void testBudgetedFeasiblePoll(){
  // A fixed synthetic near-cap straight response, not a tuned route search.
  // Both solvers use exactly the same state, published history and constraints.
  auto r=request();r.state.y=-.02;r.control_dt=1./30.;r.speed_cap=r.state.speed;
  SolverConfig c;c.budget_ms=1000.;c.iterations=2;
  auto configure=[](Flat& f){f.setConditionedGeometry(true);f.setBaselineSeed(true);
    f.setDelayAlignedReconstruction(true);f.setCommandProjection(true);
    f.setResponseBase(true);f.setResponseNearSupport(true);};
  Flat original,bounded;configure(original);configure(bounded);bounded.setBudgetedPoll(true);
  const auto full=original.solve(r,c),shorter=bounded.solve(r,c);
  constraints(full,r,c);constraints(shorter,r,c);
  const auto& d=bounded.diagnostics();
  assert(!original.diagnostics().coarse_poll_skipped);
  assert(d.response_base_used&&d.tracking_accepted>0&&d.fine_poll_evaluations==16);
  // This fixture naturally improves during the fine poll. Its complete first
  // coarse sweep makes no further improvement, so only the second is omitted.
  assert(d.fine_poll_improvements>0&&d.local_poll_stop_stage==2&&d.coarse_poll_sweeps==1);
  assert(d.coarse_poll_skipped&&shorter.early_stopped&&shorter.reason=="flat8_feasible_local_poll_stop");
  assert(shorter.evaluations<full.evaluations);
  assert(near(shorter.objective,full.objective,1e-12));
  assert(shorter.decision_dimension==8&&full.decision_dimension==8);
  const auto initial=Flat::jet(bounded.lastCurve(),0.);
  assert(near(initial.x,r.state.x)&&near(initial.y,r.state.y)&&near(initial.dx,r.state.speed)&&near(initial.dy,0.));
  assert(near(initial.ddx,0.)&&near(initial.ddy,0.));
  const auto verified=ReducedNMPC::validateControls(r,c,shorter.controls,
    std::chrono::steady_clock::now()+std::chrono::seconds(1));
  assert(verified.feasible&&near(verified.cost,shorter.objective,1e-12));
  for(int i=0;i<c.steps;++i){assert(near(full.controls[i].speed,shorter.controls[i].speed,1e-12));
    assert(near(full.controls[i].steering,shorter.controls[i].steering,1e-12));}
  // No successful GN/actual response base: the opt-in rule cannot manufacture
  // a feasible command, or bypass the ordinary invalid/timeout mechanisms.
  auto zero=request();zero.control_dt=r.control_dt;
  Flat no_update;configure(no_update);no_update.setBudgetedPoll(true);
  assert(no_update.solve(zero,c).ok&&!no_update.diagnostics().coarse_poll_skipped);
  auto invalid=r;invalid.force_infeasible=true;const auto infeasible=bounded.solve(invalid,c);
  assert(!infeasible.ok&&!bounded.diagnostics().coarse_poll_skipped);
  invalid=r;invalid.force_timeout=true;assert(bounded.solve(invalid,c).timed_out);
  invalid=r;invalid.history.clear();assert(bounded.solve(invalid,c).reason=="history_incomplete");
  invalid=r;invalid.state.speed=.001;assert(bounded.solve(invalid,c).reason=="flatness_low_speed_invalid");
  invalid=r;c.budget_ms=.000001;const auto timed=bounded.solve(invalid,c);
  assert(!timed.ok&&timed.timed_out&&!bounded.diagnostics().coarse_poll_skipped);
  std::cout<<"PASS bounded feasible flat8 poll: complete fine+one coarse, exact common-cost/command/initial-jet preservation, evaluations "
    <<full.evaluations<<" -> "<<shorter.evaluations<<", independent physical rollout and invalid/timeout rejection\n";
}
void testInputConstraintPhaseI(){
  auto r=request();SolverConfig c;r.control_dt=1./30.;
  r.previous_command_acceleration=.12;r.state.acceleration=.12;
  const double H=r.control_dt+(c.steps-1)*c.dt;
  auto anchor=Flat::anchoredCurve(r.state,c.model,H);anchor.delay_aligned=true;
  Flat::Coefficients z{};z[1]=.15;z[2]=-.05;z[5]=.10;
  Flat::Curve q;assert(Flat::curveFromDecision(anchor,z,r,c,q));
  double merit=0.,maximum=0.;Flat::Coefficients direction{},gradient{};
  assert(Flat::inputFeasibilityDirection(q,r,c,merit,maximum,&direction,
    std::chrono::steady_clock::now()+std::chrono::seconds(1),&gradient));
  assert(merit>1e-10&&maximum>0.);
  const double epsilon=1e-6;double dot=0.;
  for(int n=0;n<8;++n){
    auto plus=z,minus=z;plus[n]+=epsilon;minus[n]-=epsilon;Flat::Curve a,b;
    assert(Flat::curveFromDecision(anchor,plus,r,c,a)&&Flat::curveFromDecision(anchor,minus,r,c,b));
    double ap,am,mp,mm;
    assert(Flat::inputFeasibilityDirection(a,r,c,ap,mp));
    assert(Flat::inputFeasibilityDirection(b,r,c,am,mm));
    const double numerical=(ap-am)/(2.*epsilon);
    assert(std::abs(numerical-gradient[n])<1e-4*std::max(1.,std::abs(numerical)));
    dot+=gradient[n]*direction[n];
  }
  assert(dot<0.);bool decreased=false;
  for(double alpha:{1.,.5,.25,.125,.0625,.03125,.015625}){
    auto trial=z;for(int n=0;n<8;++n)trial[n]+=alpha*direction[n];
    Flat::Curve next;if(!Flat::curveFromDecision(anchor,trial,r,c,next))continue;
    double value,max_value;if(!Flat::inputFeasibilityDirection(next,r,c,value,max_value))continue;
    if(value<merit&&max_value<=maximum){
      const auto initial=Flat::jet(q,0.),after=Flat::jet(next,0.);
      assert(near(initial.x,after.x)&&near(initial.dy,after.dy)&&near(initial.ddx,after.ddx));
      decreased=true;break;
    }
  }
  assert(decreased);
  std::cout<<"PASS eight-coordinate implicit-first-input Phase-I merit gradient, full-horizon inequalities, descent and initial jet preservation\n";
}
void testSolver(){
  auto r=request();SolverConfig c;c.budget_ms=1000.;Flat solver;
  auto solved=solver.solve(r,c);constraints(solved,r,c);
  assert(solved.evaluations>1&&solver.diagnostics().reference_fit_samples==c.steps);
  Flat recovery;recovery.setFeasibilityRecovery(true);
  const auto already_feasible=recovery.solve(r,c);constraints(already_feasible,r,c);
  assert(!recovery.diagnostics().feasibility_recovery_attempted);
  assert(near(already_feasible.command.speed,solved.command.speed)&&
         near(already_feasible.command.steering,solved.command.steering));
  // The opt-in accuracy strategy must refit a time-shifted warm curve in its
  // original chart and revalidate it against REAL next-cycle command history.
  Flat accurate;accurate.setFeasibilityRecovery(true);accurate.setAccuracyRefinement(true);
  const auto first=accurate.solve(r,c);constraints(first,r,c);
  auto next=r;next.now+=r.control_dt;next.state=first.predicted[1];next.previous=first.command;
  next.previous_command_acceleration=first.command_acceleration;
  next.history.push_back({r.now,first.command});
  for(int i=0;i<next.reference.count;++i)next.reference.points[i].x+=.24*r.control_dt;
  const auto shifted=accurate.solve(next,c);constraints(shifted,next,c);
  assert(shifted.warm_available&&accurate.diagnostics().warm_geometry_preserved&&
    std::isfinite(accurate.diagnostics().warm_fit_rmse));
  const auto independent=ReducedNMPC::validateControls(next,c,shifted.controls,
    std::chrono::steady_clock::now()+std::chrono::seconds(1));
  assert(independent.feasible);
  solver.setNonuniformKnots(false);const auto uniform=solver.solve(r,c);constraints(uniform,r,c);
  assert(near(solver.diagnostics().interior_knot,.5*(r.control_dt+(c.steps-1)*c.dt)));
  // Actual time-shift / re-fit warm candidate, not old coefficient reuse.
  r.now+=r.control_dt;r.history.push_back({r.now-r.control_dt,solved.command});r.previous=solved.command;r.previous_command_acceleration=solved.command_acceleration;
  r.state=solved.predicted[1];for(int i=0;i<r.reference.count;++i)r.reference.points[i].x+=.24*r.control_dt;
  const auto warm=solver.solve(r,c);constraints(warm,r,c);assert(warm.warm_available&&std::isfinite(solver.diagnostics().warm_fit_rmse));
  // Same request and objective, ordinary twelve-coordinate control baseline.
  ReducedNMPC full;const auto twelve=full.solve(r,c);assert(twelve.ok&&twelve.decision_dimension==12);
  // Nonzero local error must affect a genuine spline coefficient and optimized
  // trajectory.  No precomputed reference-only control is returned.
  auto disturbed=request();disturbed.state.y=-.02;Flat response;const auto changed=response.solve(disturbed,c);constraints(changed,disturbed,c);
  double lateral_command=0.;for(int i=0;i<c.steps;++i)lateral_command+=std::abs(changed.controls[i].steering);
  std::cerr<<"response debug objective="<<changed.objective<<" evaluations="<<changed.evaluations<<" lateral="<<lateral_command<<" inversefail="<<response.diagnostics().inverse_failures<<" fit="<<response.diagnostics().reference_fit_rmse<<"\n";
  for(int k=4;k<8;++k){Flat::Coefficients z{};z[k]=.0875;Flat::Curve q;const auto a=Flat::anchoredCurve(disturbed.state,c.model,disturbed.control_dt+(c.steps-1)*c.dt);
    bool valid=Flat::curveFromDecision(a,z,disturbed,c,q);double maxj=0.,maxd=0.,maxrate=0.,va=0.;Command u=disturbed.previous;
    if(valid)for(int i=0;i<c.steps;++i){const double h=i?c.dt:disturbed.control_dt,t=disturbed.control_dt+i*c.dt;const auto f=FlatnessReferenceModel::reconstruct(Flat::jet(q,t),c.model);const double a1=(f.feedforward.speed-u.speed)/h;maxj=std::max(maxj,std::abs(a1-va)/h);maxd=std::max(maxd,std::abs(f.feedforward.steering));maxrate=std::max(maxrate,std::abs(f.feedforward.steering-u.steering)/h);va=a1;u=f.feedforward;}
    std::cerr<<"k="<<k<<" valid="<<valid<<" jerk="<<maxj<<" delta="<<maxd<<" rate="<<maxrate<<"\n";
  }
  assert(lateral_command>1e-7);
  std::cout<<std::setprecision(12)<<"flat8 straight="<<solved.objective<<" twelve="<<twelve.objective
    <<" error_case="<<changed.objective<<" evals="<<changed.evaluations<<" inverse_failures="<<response.diagnostics().inverse_failures<<"\n";
  auto invalid=request();invalid.state.speed=.001;assert(response.solve(invalid,c).reason=="flatness_low_speed_invalid");
  invalid=request();invalid.reference.valid=false;assert(!response.solve(invalid,c).ok);
  invalid=request();invalid.history.clear();assert(response.solve(invalid,c).reason=="history_incomplete");
  invalid=request();invalid.history.push_back({invalid.now+.1,invalid.previous});assert(!response.solve(invalid,c).ok);
  invalid=request();invalid.force_timeout=true;assert(response.solve(invalid,c).timed_out);
  invalid=request();invalid.force_infeasible=true;assert(!response.solve(invalid,c).ok);
  invalid=request();invalid.state.x=missing();assert(!response.solve(invalid,c).ok);
  invalid=request();c.budget_ms=.000001;const auto timeout=response.solve(invalid,c);assert(!timeout.ok&&timeout.timed_out);
  // Restoration may enlarge geometric search coordinates, never a hard speed
  // cap or the empty acceleration/jerk domain inherited from actual history.
  c.budget_ms=1000.;invalid=request();invalid.speed_cap=.20;
  assert(!recovery.solve(invalid,c).ok);
  invalid=request();invalid.previous_command_acceleration=.28;
  invalid.speed_cap=invalid.previous.speed;
  assert(recovery.solve(invalid,c).reason=="flatness_first_command_speed_acceleration_jerk_bounds_empty");
}
void testSelectedInputReconstruction(){
  using Map=FlatnessReferenceModel;
  Model model;
  model.yaw_residual=.012;model.longitudinal_residual=-.04;
  const std::array<FlatOutputJet,6> jets{{
    {.2,-.03,.34,-.02,.07,-.08,.033,.11},
    {0.,0.,.015,0.,0.,0.,0.,0.},
    {0.,0.,.0149,0.,0.,0.,0.,0.},
    {0.,0.,.34,0.,0.,0.,0.,missing()},
    {65537.,0.,.33,.04,.03,-.03,0.,.03},
    {65536.,-65536.,.015,0.,65536.,-65536.,-65536.,65536.}
  }};
  for(double gain:{1.,1e-18,1e10}){
    model.speed_gain=gain;
    const bool ordinary=Map::boundedInputModel(model);
    assert(ordinary==(gain==1.));
    for(const auto& jet:jets){
      const auto full=Map::reconstructForInputs(jet,model);
      for(unsigned mask:{0u,Map::kInverseSpeed,Map::kInverseSteering,Map::kInverseSpeed|Map::kInverseSteering}){
        Command command;
        assert(Map::reconstructSelectedInputs(jet,model,ordinary,mask,command)==full.valid);
        if(!full.valid)continue;
        if(mask&Map::kInverseSpeed)assert(command.speed==full.feedforward.speed);
        if(mask&Map::kInverseSteering)assert(command.steering==full.feedforward.steering);
      }
    }
  }
  std::cout<<"PASS channel-selective flat inverse matches complete validity and demanded inputs\n";
}
}
int main(){testBasis();testSelectedInputReconstruction();testGeometryOnlyCurvatureRecovery();testCausalRearReferenceAndResponseKnot();testEightDimensionalChart();testIndependentFirstInputEncoding();testPhysicalInputProjection();testJitteredWarmStartAndUncontrollablePrefix();testSaturatedSpeedCoupledSplineSeed();testTrackingGaussNewtonGradient();testCachedFlatCommandDifferential();testAnalyticFirstInputJacobian();testDelayAlignedChart();testConditionedGeometryAndBaselineSeed();testResponseBaseFlatOptimization();testResponseBaseWarmStart();testBudgetedFeasiblePoll();testInputConstraintPhaseI();testSolver();std::cout<<"PASS P6 quintic B-spline analytic jets/Jacobian, eight-dimensional chart, original constraint rollout, response, warm shift and invalid/fault handling\n";}
