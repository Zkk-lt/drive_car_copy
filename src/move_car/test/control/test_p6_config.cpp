#include "control/p6_config.h"
#include "control/reduced_nmpc.h"
#include <cassert>
#include <iostream>
using namespace research_control;
static std::map<std::string,double> fixture(){return {
 {"wheelbase",.6},{"rear_to_center",.3},{"wheel_speed_scale",.001},{"steering_tau",.44},{"steering_delay",.325},
 {"steering_gain",1.},{"speed_tau",.2},{"speed_delay",.2},{"speed_gain",1.},{"yaw_gain",1.},{"control_rate",30.},
 {"prediction_dt",.1},{"horizon_steps",20},{"control_knots",6},{"solver_max_iterations",3},{"solver_budget_ms",8.},
 {"weight_lateral",26.},{"weight_heading",7.},{"weight_speed",6.},{"weight_smoothness",24.},{"speed_cap",.3},
 {"lateral_weight_scale",1.},{"heading_weight_scale",1.},{"speed_weight_scale",1.},{"smoothness_weight_scale",1.},
 {"max_speed",.5},{"max_steer",.38},{"solver_max_steer",.32},{"max_steer_rate",1.},{"max_accel",.28},
 {"max_decel",.36},{"max_jerk",.8},{"max_lateral_accel",.25},{"collision_stop_distance",.5},{"follow_distance_min",.5},{"reference_hard_timeout",1.2}};}
static SolveRequest request(const SolverConfig& c){
 SolveRequest r;r.now=10.;r.state.speed=.2;r.state.y=.035;r.previous={.2,0.};r.baseline={.2,-.08};r.speed_cap=.3;
 r.history.push_back({9.,r.previous});std::vector<ReferencePoint> path;
 for(int i=0;i<100;++i){ReferencePoint p;p.s=.02*i;p.x=c.model.rear_to_center+p.s;path.push_back(p);}
 r.reference=FlatnessReferenceModel::buildTimeReference(path,.2,.25,10.,c);return r;
}
int main(){
 auto values=fixture();P6NominalProfile p;p.scene="smooth";std::string reason;
 assert(p.load(values,reason));assert(p.solver.model.speed_gain==1.);assert(p.solver.model.steering_gain==1.);
 values["speed_cap"]=.4;assert(p.load(values,reason)&&p.speed_cap==.4);
 values["speed_cap"]=.40001;assert(!p.load(values,reason));
 values["speed_cap"]=.4;p.scene="asphalt";assert(!p.load(values,reason));p.scene="unknown";assert(!p.load(values,reason));
 p.scene="smooth";values=fixture();assert(p.load(values,reason));
 auto old=p.solver;values["max_speed"]=.6;assert(!p.load(values,reason));assert(p.solver.limits.speed==old.limits.speed);
 values=fixture();values["unknown_key"]=1.;assert(!p.load(values,reason));
 values=fixture();values["solver_budget_ms"]=9.;assert(!p.load(values,reason));
 values=fixture();values["steering_gain"]=missing();assert(!p.load(values,reason));
 values=fixture();values["yaw_gain"]=.85;values["steering_gain"]=.9;values["speed_tau"]=.3;values["speed_gain"]=.9;
 assert(p.load(values,reason));assert(p.solver.model.yaw_gain==.85);assert(p.solver.model.steering_gain==.9);
 assert(p.solver.model.speed_tau==.3&&p.solver.model.speed_gain==.9);
 {FlatOutputJet q;q.dx=.2;q.ddy=.01;Model m;const auto a=FlatnessReferenceModel::reconstruct(q,m);
  m.steering_gain=.8;m.speed_gain=.9;const auto b=FlatnessReferenceModel::reconstruct(q,m);
  assert(a.valid&&b.valid);assert(a.state.steering==b.state.steering);
  assert(std::abs(b.feedforward.steering-a.feedforward.steering/.8)<1e-14);
  assert(std::abs(b.feedforward.speed-a.feedforward.speed/.9)<1e-14);}
 {SolverConfig c;c.budget_ms=1000.;auto r=request(c);ReducedNMPC a,b;
  auto x=a.solve(r,c);c.model.steering_gain=.8;c.model.speed_gain=.9;auto y=b.solve(r,c);
  assert(x.ok&&y.ok);double model_change=0.,command_change=0.;
  for(int i=0;i<c.steps;++i){model_change+=std::abs(x.predicted[i+1].y-y.predicted[i+1].y)+std::abs(x.predicted[i+1].speed-y.predicted[i+1].speed);
   command_change+=std::abs(x.controls[i].steering-y.controls[i].steering)+std::abs(x.controls[i].speed-y.controls[i].speed);}
  assert(model_change>1e-5&&command_change>1e-5);}
 {Limits safe,wide;wide.speed=2.;wide.steering=1.;auto a=p6SafetyIntersection(safe,wide);assert(a.speed==safe.speed&&a.steering==safe.steering);}
 {P6LayeredConfig cfg;cfg.config_hash="synthetic-fixture";for(auto name:{"steering","yaw_gain","yaw_disturbance","longitudinal","roughness","slip"})cfg.policies[name]=P6ChannelPolicy{};
  assert(cfg.validate(reason));cfg.real_vehicle_control_allowed=true;assert(!cfg.validate(reason));}
 std::cout<<"P6 config: strict normalized load, gain model/control sensitivity, inverse consistency, immutable safety passed\n";
}
