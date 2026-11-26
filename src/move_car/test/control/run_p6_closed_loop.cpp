// P6 uses the real S8 SLQR and independent P3 plant, with the P5 arbiter/ledger.
// No ROS master, command publisher or hardware driver is created here.
#define P36_RESEARCH
#define P6_EMBED_RUNNER
#include "run_closed_loop.cpp"
#include "control/unified_control.h"
#include "control/p6_controller.h"
#include <map>

namespace p6test {
using Clock=std::chrono::steady_clock;
struct Row {
 Samples ey,heading,velocity,solve,loop,rate,acceleration,jerk,plantjerk,prediction;
 Samples nominal_yaw_prediction,effective_yaw_prediction,nominal_speed_prediction,effective_speed_prediction;
 Samples normal_plant_jerk,stop_command_jerk,stop_plant_jerk;
 int normal_plant_jerk_violations=0,stop_periods=0;
 int selected=0,fallback=0,commits=0,invalid=0,violations=0,plant_violations=0;
 int timeouts=0,feasible=0,dimension=0;double progress=0.;
 int model_updates=0,feedback_active=0,slip_valid=0,estimator_valid=0;
 std::string run_id,session_id,config_hash;
};
rc::p6::RawSample rawSample(double value,double now,std::uint64_t sequence){
 rc::p6::RawSample s;s.value=value;s.source_stamp=s.receipt_stamp=now;s.sequence=sequence;
 s.valid=s.unit_verified=s.frame_verified=s.hardware_stamp_verified=s.new_frame_known=s.new_frame=true;s.source="synthetic_plant";s.unit="SI";return s;
}
rc::p6::ResponsePolicy syntheticPolicy(){
 rc::p6::ResponsePolicy p;const char*sha=std::getenv("P6_SYNTHETIC_EVIDENCE_SHA");
 if(!sha)return p;
 for(auto*c:{&p.steering,&p.longitudinal,&p.yaw,&p.vibration}){
  c->enabled=c->delay_verified=c->bias_verified=c->timing_verified=true;c->evidence_kind="synthetic_validation";c->evidence_sha=sha;
  // Exact policy exercised in test_p6_response on independent variable-dt
  // synthetic identification. It does not authorize real adaptation.
  c->min_samples=60;c->min_excitation=.001;c->max_innovation=3.;c->max_covariance=1000.;
  c->min_rank_ratio=.00001;c->prediction_improvement=.01;c->gain_rate=c->tau_rate=.1;
 }return p;
}
rc::State independentPrediction(rc::State s,double now,const rc::Model&m,const std::vector<rc::CommandSample>&history){
 for(int k=0;k<20;++k){const double h=DT/20.,t=now+k*h;const auto v=commandAt(history,t-m.speed_delay),d=commandAt(history,t-m.steering_delay);
  const double a=(m.speed_gain*v.speed-s.speed)/m.speed_tau+m.longitudinal_residual;
  const double r=m.yaw_gain*s.speed*std::tan(s.steering)/m.wheelbase+m.yaw_residual;
  s.x+=h*(s.speed*std::cos(s.yaw)-m.lateral_residual*std::sin(s.yaw));s.y+=h*(s.speed*std::sin(s.yaw)+m.lateral_residual*std::cos(s.yaw));s.yaw=rc::wrap(s.yaw+h*r);
  s.speed+=h*a;s.steering+=h*(m.steering_gain*d.steering-s.steering)/m.steering_tau;s.acceleration=a;
 }return s;
}
rc::Reference reference(const Track& track,double progress,double now,const rc::SolverConfig& c,double wanted=TARGET){
 rc::Reference r;r.valid=true;r.stamp=now;r.count=c.steps+1;
 for(int i=0;i<r.count;++i){auto p=track.at(progress+i*c.dt*wanted);auto&v=r.points[i];
  v.t=i*c.dt;v.s=p.s;v.x=p.x;v.y=p.y;v.yaw=p.yaw;v.curvature=p.kappa;v.speed=wanted;}
 return r;
}
unified_control::PublishedCommandRecord publication(unified_control::CommandLedger& l,
 double now,rc::Command u,double a,bool selected){
 unified_control::PublishedCommandRecord p;p.sequence=l.nextSequence();p.epoch=1;
 p.cycle_stamp=now;p.stamp=now;p.publish_ros_start=now;p.publish_ros_end=now;
 p.publish_steady_start_ns=static_cast<std::uint64_t>(now*1.e9);p.publish_steady_end_ns=p.publish_steady_start_ns;
 p.command=u;p.acceleration=a;p.selected_nmpc=selected;p.ros_publish_returned=true;return p;
}
Row run(const std::string& shape,const std::string& mode,double seconds,double budget,
 const std::string& fault,std::ostream& ticks){
 Track track(shape);Plant plant(track);Ground terrain=ground("smooth");SpatialSLQR slqr;configureBaseline(slqr);
 rc::P6Controller controller;
 const bool preview=std::getenv("P6_SPEED_PREVIEW")&&std::string(std::getenv("P6_SPEED_PREVIEW"))=="1";
 const bool sync=std::getenv("P6_SYNC_PREVIEW")&&std::string(std::getenv("P6_SYNC_PREVIEW"))=="1";
#ifndef P6_FROZEN_ENTRY
 const bool actuator_aligned=std::getenv("P6_ACTUATOR_ALIGNED_PREVIEW")&&std::string(std::getenv("P6_ACTUATOR_ALIGNED_PREVIEW"))=="1";
#endif
 const bool terminal=std::getenv("P6_TERMINAL_APPROACH")&&std::string(std::getenv("P6_TERMINAL_APPROACH"))=="1";
 const bool terminal_handoff=terminal&&std::getenv("P7_TERMINAL_HANDOFF")&&mode=="flat_nmpc_slqr";
 unified_control::TerminalHandoff parking;
 const bool short_ref=std::getenv("P6_SHORT_REFERENCE")&&std::string(std::getenv("P6_SHORT_REFERENCE"))=="1";
 if(const char* value=std::getenv("P6_YAW_RATE_TRACKING_WEIGHT"))controller.setYawRateTrackingWeight(std::stod(value));
 if(const char* flag=std::getenv("P6_BOUNDED_FEEDBACK"))controller.setBoundedFeedback(std::string(flag)=="1");
 if(const char* flag=std::getenv("P6_COURSE_CONSISTENT_FEEDBACK"))controller.setCourseConsistentFeedback(std::string(flag)=="1");
 if(const char* flag=std::getenv("P6_CONDITIONED_GEOMETRY"))controller.setConditionedGeometry(std::string(flag)=="1");
 if(const char* flag=std::getenv("P6_BASELINE_SEED"))controller.setBaselineSeed(std::string(flag)=="1");
 if(const char* flag=std::getenv("P6_DELAY_ALIGNED"))controller.setDelayAlignedReconstruction(std::string(flag)=="1");
 if(const char* flag=std::getenv("P6_INDEPENDENT_INPUTS"))controller.setIndependentInputEncoding(std::string(flag)=="1");
 if(const char* flag=std::getenv("P6_COMMAND_PROJECTION"))controller.setCommandProjection(std::string(flag)=="1");
#ifndef P6_FROZEN_ENTRY
 if(const char* flag=std::getenv("P6_CAUSAL_REFERENCE_SEED"))controller.setCausalReferenceSeed(std::string(flag)=="1");
 if(const char* flag=std::getenv("P6_RESPONSE_KNOT_MONITOR"))controller.setResponseKnotMonitor(std::string(flag)=="1");
 if(const char* flag=std::getenv("P6_INPUT_FEASIBILITY_SQP"))controller.setInputFeasibilitySQP(std::string(flag)=="1");
#endif
 if(const char* flag=std::getenv("P6_RESPONSE_BASE"))controller.setResponseBase(std::string(flag)=="1");
 if(const char* flag=std::getenv("P6_RESPONSE_NEAR_SUPPORT"))controller.setResponseNearSupport(std::string(flag)=="1");
#ifndef P6_FROZEN_ENTRY
 if(const char* flag=std::getenv("P6_BUDGETED_POLL"))controller.setBudgetedPoll(std::string(flag)=="1");
 if(const char* flag=std::getenv("P6_RESPONSE_WARM_START"))controller.setResponseWarmStart(std::string(flag)=="1");
#endif
 if(const char* value=std::getenv("P6_STEERING_RATE_CONTINUITY_WEIGHT"))controller.setSteeringRateContinuityWeight(std::stod(value));
 if(const char* flag=std::getenv("P6_TURN_AWARE_RATE_CONTINUITY"))controller.setTurnAwareRateContinuity(std::string(flag)=="1");
 rc::SolverConfig c;c.budget_ms=budget;
 if(const char* value=std::getenv("P6_MODEL_YAW_GAIN"))c.model.yaw_gain=std::stod(value);
 if(const char* value=std::getenv("P6_MAX_ITERATIONS"))c.iterations=std::stoi(value);
 if(const char* value=std::getenv("P6_LATERAL_WEIGHT"))c.weights.lateral=std::stod(value);
 if(const char* value=std::getenv("P6_HEADING_WEIGHT"))c.weights.heading=std::stod(value);
 if(const char* value=std::getenv("P6_SPEED_WEIGHT"))c.weights.speed=std::stod(value);
 if(const char* value=std::getenv("P6_SMOOTHNESS_WEIGHT"))c.weights.smoothness=std::stod(value);
 rc::P6NominalProfile profile;profile.scene="smooth";profile.speed_cap=TARGET;profile.config_hash=std::string(64,'6');profile.solver=c;
 if(const char*hash=std::getenv("P6_NOMINAL_CONFIG_SHA"))profile.config_hash=hash;
 auto response_policy=syntheticPolicy();
 if(response_policy.yaw.enabled){profile.response_channels={"steering","yaw_gain","longitudinal","roughness"};profile.response_policy_kind="synthetic_validation";profile.response_policy_sha=response_policy.yaw.evidence_sha;profile.response_minimum_confidence=.8;}
 if(mode=="blocked8"){c.knots=4;c.research_grid=3;}
 unified_control::CommandLedger ledger(1);std::vector<rc::CommandSample> history;
 rc::Command previous{TARGET,plant.state.steering};double previous_a=0.;
 // Explicitly published synthetic pre-roll; these are not recovered real events.
 for(int j=0;j<36;++j){double t=100.-(36-j)*DT;auto p=publication(ledger,t,previous,0.,false);
  require(ledger.commit(p).accepted,"pre-roll ledger");history.push_back({t,previous});}
 Row m;m.run_id=(std::getenv("P6_RUN_ID")?std::getenv("P6_RUN_ID"):"p5_p6_synthetic")+std::string("_")+shape+"_"+mode+"_"+fault;
 m.session_id=p37_validation?"synthetic_heldout":"synthetic_development";m.config_hash=profile.config_hash;
 if(const char*session=std::getenv("P6_SESSION_ID"))m.session_id=session;
 double progress=0.,last_plant_a=plant.state.acceleration;
 for(int tick=0;tick<int(seconds/DT);++tick){
  const auto start=Clock::now();const double now=100.+tick*DT;
  const double center_x=plant.state.x+.3*std::cos(plant.state.yaw),center_y=plant.state.y+.3*std::sin(plant.state.yaw);
  progress=track.nearest(center_x,center_y,progress);const auto p=track.at(progress);
  const bool loss=fault=="reference_loss"&&tick>=60&&tick<75;
  const bool emergency=fault=="emergency"&&tick>=90&&tick<96;
  const bool conservative_scene=fault=="scene_switch"&&tick>=60&&tick<150;
  const double desired_speed=conservative_scene?.1:TARGET;
  if(tick>=60){
   if(fault=="yaw_gain_step"||fault=="combined")terrain.yaw_gain=.83-.015*p6_fixture_variant;
   if(fault=="steering_lag"||fault=="combined"){terrain.steering_tau=.62+.02*p6_fixture_variant;terrain.steering_gain=.86-.01*p6_fixture_variant;}
   if(fault=="longitudinal_weak"||fault=="combined"){terrain.speed_gain=.87-.01*p6_fixture_variant;terrain.speed_tau=.43+.01*p6_fixture_variant;}
   if(fault=="vibration"||fault=="combined")terrain.vibration=.8;
  }
  Metrics base_metrics;rc::Command base=baselineCommand(slqr,track,plant,progress,now,previous,previous_a,history,base_metrics,loss||emergency);
  if(fault=="scene_switch")base=rc::p6ConservativeSpeedFallback(base,previous,previous_a,DT,desired_speed,c.limits);
  unified_control::ControlRequest q;q.mode=mode=="legacy_slqr"?unified_control::ControlMode::LegacySlqr:(mode=="nmpc_shadow"?unified_control::ControlMode::NmpcShadow:unified_control::ControlMode::NmpcFixed);
  q.now=now;q.dt=DT;q.legacy=base;q.previous=previous;q.previous_acceleration=previous_a;
  q.guard={true,true,"/control_research/p5_test/p6/command",m.run_id};
  q.epoch=1;q.config_version=1;q.history_sequence=ledger.lastSequence();q.history_valid=ledger.valid();
  q.reference={now,static_cast<std::uint64_t>(tick+1),!loss,false};q.safety_stop=emergency;q.speed_cap=std::max(desired_speed,std::max(previous.speed,base.speed));q.limits=c.limits;
  q.state.stamp=q.state.receipt_stamp=now;q.state.x=plant.state.x;q.state.y=plant.state.y;q.state.yaw=plant.state.yaw;
  q.state.speed=plant.state.speed;q.state.steering=plant.state.steering;q.state.acceleration=plant.state.acceleration;
  q.state.yaw_rate=plant.yaw_rate;q.state.confidence=1.;q.state.valid=q.state.causal=q.state.nominal_model_valid=true;
  q.maximum_solve_ms=budget;
  rc::SolveRequest request;request.state=plant.state;request.now=now;request.previous=previous;
  request.previous_command_acceleration=previous_a;request.baseline=base;request.speed_cap=desired_speed;
  request.control_dt=DT;request.history=history;request.published_history_contiguous=ledger.valid();request.reference=reference(track,progress,now,c,desired_speed);
  if((preview||std::getenv("P6_SHARED_SPATIAL_REFERENCE"))&&!loss){
   const double end=short_ref?std::min(track.p.back().s,1.35):std::min(track.p.back().s,progress+1.8);
   std::vector<rc::ReferencePoint> spatial;spatial.reserve(96);
   for(double s=progress;s<end+1e-9;s+=.02){const auto p=track.at(s);
    rc::ReferencePoint z;z.s=p.s;z.x=p.x;z.y=p.y;z.yaw=p.yaw;z.curvature=p.kappa;spatial.push_back(z);}
   if(end>progress&&(spatial.empty()||end-spatial.back().s>1e-6)){
    const auto p=track.at(end);rc::ReferencePoint z;z.s=p.s;z.x=p.x;z.y=p.y;z.yaw=p.yaw;z.curvature=p.kappa;spatial.push_back(z);}
   request.target_speed=desired_speed;
   request.reference=rc::FlatnessReferenceModel::buildTimeReference(spatial,request.state.speed,
     request.target_speed,now,c,false,preview,request.state.acceleration,sync,terminal
#ifndef P6_FROZEN_ENTRY
     ,actuator_aligned?request.state.yaw:rc::missing()
#endif
     );
  }else if(loss)request.reference.valid=false;
  q.reference.valid=request.reference.valid;
  request.force_timeout=fault=="timeout"&&tick>=60&&tick<70;
  request.force_infeasible=fault=="infeasible"&&tick>=60&&tick<70;
  request.force_numerical_failure=fault=="numerical"&&tick>=60&&tick<70;
  rc::p6::SensorSnapshot sensors;sensors.now=now;sensors.synthetic=sensors.history_complete=sensors.imu_frame_verified=true;
  sensors.run_id=m.run_id;sensors.session_id=m.session_id;sensors.cycle=tick+1;sensors.scene="smooth";
  sensors.wheel_speed=rawSample(plant.state.speed,now,tick+1);sensors.steering=rawSample(plant.state.steering,now,tick+1);sensors.yaw_rate=rawSample(plant.yaw_rate,now,tick+1);
  sensors.accel_x=rawSample(plant.state.acceleration,now,tick+1);sensors.accel_y=rawSample(plant.state.speed*plant.yaw_rate,now,tick+1);sensors.accel_z=rawSample(9.81+terrain.vibration*std::sin(now*17.),now,tick+1);
  if(fault=="sensor_dropout"&&tick>=60&&tick<80){sensors.steering.valid=sensors.yaw_rate.valid=false;q.state.valid=false;}
  std::vector<rc::p6::PublishedSample> published;std::array<unified_control::PublishedCommandRecord,256> records;
  const auto count=ledger.copyHistory(records);for(std::size_t i=0;i<count;++i){rc::p6::PublishedSample h;h.sequence=records[i].sequence;h.stamp=records[i].stamp;h.command=records[i].command;h.published=h.committed=true;published.push_back(h);}
  profile.solver=c;profile.scene=conservative_scene?"unknown":"smooth";profile.speed_cap=desired_speed;
  if(conservative_scene)profile.response_channels.clear();else if(response_policy.yaw.enabled)profile.response_channels={"steering","yaw_gain","longitudinal","roughness"};
  auto effective=controller.parameters(profile,sensors,published,response_policy,mode);effective.history_base_sequence=ledger.lastSequence();effective.snapshot_hash=rc::p6SnapshotIdentity(effective);
  q.adaptive_gate_enabled=true;
  if(rc::p6ResidualMode(mode))q.mode=unified_control::ControlMode::NmpcAdaptive;
  if(mode=="research_auto_fallback")q.mode=unified_control::ControlMode::AutoFallback;
  // Identical external, synthetic physical event for every mode, occurring after
  // the optimization measurement and before validation. It is not estimator truth.
  const bool asynchronous=(fault=="async_disturbance"||fault=="combined")&&tick>=60&&tick%20==0;
  if(asynchronous){plant.state.y+=.0005;plant.state.yaw=rc::wrap(plant.state.yaw+.0003);}
  rc::SolveResult result;rc::P6Controller::Result controlled;
  if(mode!="legacy_slqr"&&!emergency&&!(terminal_handoff&&parking.latched())){controlled=controller.solve(request,effective,mode,asynchronous?&plant.state:nullptr,now,asynchronous?&history:nullptr);result=controlled.solve;}
  m.feedback_active+=controlled.feedback_nonzero;
  for(const auto&change:effective.response.changes)if(std::isfinite(change.effective)&&std::abs(change.effective-change.nominal)>1e-9){++m.model_updates;break;}
  m.estimator_valid+=effective.response.yaw.update_allowed||effective.response.steering.update_allowed||effective.response.longitudinal.update_allowed;
  m.slip_valid+=effective.response.slip.valid;
  unified_control::ControlCandidate candidate;candidate.command=result.command;candidate.command_acceleration=result.command_acceleration;
  candidate.solve_ms=result.solve_ms;candidate.max_violation=result.max_violation;candidate.created_stamp=candidate.snapshot_stamp=candidate.expected_apply_stamp=now;
  candidate.sequence=tick+1;candidate.epoch=1;candidate.reference_sequence=q.reference.sequence;candidate.config_version=1;candidate.history_base_sequence=ledger.lastSequence();
  candidate.valid=result.ok;candidate.feasible=result.feasible;candidate.timed_out=result.timed_out;
  candidate.complete_substep_constraints=result.ok&&result.max_violation<=1e-7;candidate.state_pair_validated=true;
  candidate.propagated_speed=plant.state.speed;candidate.propagated_steering=plant.state.steering;candidate.propagated_yaw_rate=plant.yaw_rate;
  auto decision=unified_control::ControlArbiter::decide(q,mode=="legacy_slqr"?nullptr:&candidate);
  if(terminal_handoff)decision=parking.apply(q,decision,short_ref?std::max(0.,1.35-progress):1.8,
     .10,effective.solver.model.speed_delay+effective.solver.model.speed_tau,!loss,q.speed_cap,profile.speed_cap);
  const rc::Command applied=decision.command;const double a=(applied.speed-previous.speed)/DT,j=(a-previous_a)/DT;
  if(!emergency&&!loss&&(applied.speed<0.||applied.speed>c.limits.speed+1e-7||std::abs(applied.steering)>c.limits.steering+1e-7||
    std::abs(applied.steering-previous.steering)>c.limits.steering_rate*DT+1e-7||a>c.limits.acceleration+1e-7||a< -c.limits.deceleration-1e-7||std::abs(j)>c.limits.jerk+1e-7))++m.violations;
  auto committed=publication(ledger,now,applied,a,decision.selected_nmpc);
  require(ledger.commit(committed).accepted,"actual command commit failed");++m.commits;
  history.push_back({now,applied});while(history.size()>256)history.erase(history.begin());
  if(decision.selected_nmpc)++m.selected;if(decision.fallback)++m.fallback;
  if(result.feasible)++m.feasible;if(result.timed_out)++m.timeouts;
  m.dimension=result.decision_dimension;m.solve.add(result.solve_ms);m.rate.add((applied.steering-previous.steering)/DT);
  m.acceleration.add(a);if(!emergency&&!loss)m.jerk.add(j);
  const double ey=-(center_x-p.x)*std::sin(p.yaw)+(center_y-p.y)*std::cos(p.yaw);
  m.ey.add(ey);m.heading.add(rc::wrap(plant.state.yaw-p.yaw));m.velocity.add(plant.state.speed-desired_speed);
  const auto before=plant.state;
  const double prior_yaw_rate=plant.yaw_rate;
  const auto nominal_prediction=independentPrediction(before,now,c.model,history),effective_prediction=independentPrediction(before,now,effective.solver.model,history);plant.advance(now,terrain,history);
  m.nominal_yaw_prediction.add(rc::wrap(nominal_prediction.yaw-plant.state.yaw)/DT);m.effective_yaw_prediction.add(rc::wrap(effective_prediction.yaw-plant.state.yaw)/DT);
  m.nominal_speed_prediction.add(nominal_prediction.speed-plant.state.speed);m.effective_speed_prediction.add(effective_prediction.speed-plant.state.speed);
  if(result.ok)m.prediction.add(std::hypot(result.predicted[1].x-plant.state.x,result.predicted[1].y-plant.state.y));
  const double pj=(plant.state.acceleration-last_plant_a)/DT;last_plant_a=plant.state.acceleration;m.plantjerk.add(pj);
  if(emergency||loss){++m.stop_periods;m.stop_command_jerk.add(j);m.stop_plant_jerk.add(pj);}
  else {m.normal_plant_jerk.add(pj);if(std::abs(pj)>c.limits.jerk+1e-7)++m.normal_plant_jerk_violations;}
  if(std::abs(plant.state.steering)>c.limits.steering+1e-7||plant.state.speed>c.limits.speed+1e-7||plant.state.acceleration>c.limits.acceleration+1e-7||plant.state.acceleration< -c.limits.deceleration-1e-7)++m.plant_violations;
  ticks<<shape<<','<<mode<<','<<fault<<','<<tick<<','<<now<<','<<(decision.selected_nmpc?"nmpc":decision.terminal_handoff?"terminal_handoff":"s8")<<','<<decision.reason<<','<<ledger.lastSequence()<<','
    <<ey<<','<<rc::wrap(before.yaw-p.yaw)<<','<<before.speed-TARGET<<','<<applied.speed<<','<<applied.steering<<','<<a<<','<<j<<','<<pj<<','<<result.solve_ms<<','<<result.decision_dimension<<','<<result.ok<<','
    <<effective.solver.model.yaw_gain<<','<<effective.solver.model.steering_gain<<','<<effective.solver.model.steering_tau<<','<<effective.solver.model.speed_gain<<','<<effective.solver.model.speed_tau<<','<<effective.snapshot_hash<<','<<controlled.feedback_correction<<','<<effective.response.yaw.reason<<','<<effective.response.steering.reason<<','<<effective.response.longitudinal.reason<<','<<effective.response.slip.reason<<','
    <<effective.response.yaw.gain.value<<','<<effective.response.yaw.gain.confidence<<','<<effective.response.steering.gain.value<<','<<effective.response.steering.tau.value<<','<<effective.response.steering.gain.confidence<<','<<effective.response.steering.rank_ratio<<','<<effective.response.longitudinal.gain.value<<','<<effective.response.longitudinal.tau.value;
  for(bool flag:effective.response.dynamic_flags)ticks<<','<<flag;
  ticks<<','<<m.run_id<<','<<m.session_id<<','<<m.config_hash<<','<<emergency<<','<<(!loss)<<','<<desired_speed<<','<<result.iterations<<','<<result.evaluations;
#ifndef P6_FROZEN_ENTRY
  ticks<<','<<controlled.input_sqp_steps<<','<<controlled.input_merit_before<<','<<controlled.input_merit_after<<','<<controlled.flat_response_base_used<<','<<controlled.flat_response_base_cost<<','<<controlled.flat_coarse_poll_skipped<<','<<controlled.flat_fine_poll_evaluations<<','<<controlled.flat_fine_poll_improvements;
#else
  ticks<<",0,nan,nan,0,nan,0,0,0";
#endif
  ticks<<','<<prior_yaw_rate<<','<<desired_speed*p.kappa<<','<<progress<<','<<request.reference.points[std::min(c.steps,5)].speed<<','<<plant.state.speed<<','<<request.reference.terminal_approach<<','
    <<(short_ref?1.35-progress:1.8)<<','<<result.reason<<'\n';
  previous=applied;previous_a=a;m.loop.add(std::chrono::duration<double,std::milli>(Clock::now()-start).count());
 }
 require(!ledger.commit(*ledger.latest()).accepted,"duplicate publication was accepted");
 m.progress=progress;return m;
}
}
int main(int argc,char**argv){try{ros::Time::init();
 const std::string output=argc>1?argv[1]:"/tmp/p6_stage1.csv";const double seconds=argc>2?std::stod(argv[2]):8.;
 const std::string mode_filter=argc>3?argv[3]:"";const std::string fault=argc>4?argv[4]:"none";
 const double budget=argc>5?std::stod(argv[5]):8.;require(budget>0.&&budget<=8.,"8ms budget may not be relaxed");
 std::ofstream out(output),ticks(output+".ticks.csv");require(bool(out)&&bool(ticks),"evidence output unavailable");
 out<<"synthetic,path,mode,fault,nmpc_selected,fallback,commits,feasible,timeouts,dimension,command_violations,plant_violations,progress,lateral_rmse,lateral_p95,lateral_max,heading_rmse,speed_rmse,steering_rate_rms,command_jerk_rms,plant_jerk_max,prediction_rmse,solve_p50,solve_p95,solve_p99,solve_max,loop_p50,loop_p95,loop_p99,loop_max,model_updates,feedback_nonzero,slip_valid,estimator_valid,nominal_yaw_prediction_rmse,effective_yaw_prediction_rmse,nominal_speed_prediction_rmse,effective_speed_prediction_rmse,normal_plant_jerk_max,normal_plant_jerk_violations,stop_periods,stop_command_jerk_max,stop_plant_jerk_max\n";
 ticks<<"path,mode,fault,tick,time,owner,reason,sequence,ey,eheading,ev,speed,steering,command_a,command_jerk,plant_jerk,solve_ms,dimension,solver_ok,yaw_gain,steering_gain,steering_tau,speed_gain,speed_tau,snapshot_hash,feedback_correction,yaw_reason,steering_reason,longitudinal_reason,slip_reason,yaw_gain_estimate,yaw_confidence,steering_gain_estimate,steering_tau_estimate,steering_confidence,steering_rank_ratio,speed_gain_estimate,speed_tau_estimate,condition_normal,condition_yaw_underresponse,condition_steering_lag,condition_longitudinal_weak,condition_vibration,condition_slip,condition_low_confidence,run_id,session_id,nominal_config_sha,emergency,reference_valid,constrained_target_speed,iterations,evaluations,input_sqp_steps,input_merit_before,input_merit_after,response_base_used,response_base_cost,coarse_poll_skipped,fine_poll_evaluations,fine_poll_improvements,plant_body_yaw_rate,steady_curve_yaw_rate_baseline,station,preview_speed_at_0p5s,actual_rear_speed,terminal_approach,distance_left,solve_reason\n";
 int cases=0;for(const std::string shape:{"straight","circle","s_curve","curvature_jump"})for(const std::string mode:{"legacy_slqr","nmpc_12_fixed","blocked8","flat_nmpc_8","flat_nmpc_8_uniform","flat_nmpc_residual","flat_nmpc_slqr","flat_nmpc_residual_slqr","scene_fixed","yaw_compensation","steering_compensation","multi_residual_12","research_auto_fallback","nmpc_shadow"}){
  if(!mode_filter.empty()&&mode!=mode_filter)continue;auto m=p6test::run(shape,mode,seconds,budget,fault,ticks);
  require(m.commits==int(seconds/DT),"not every applied command committed");
  if((mode=="nmpc_12_fixed"||mode=="blocked8")&&fault=="none")require(m.selected>0,"control NMPC did not drive plant");
  out<<1<<','<<shape<<','<<mode<<','<<fault<<','<<m.selected<<','<<m.fallback<<','<<m.commits<<','<<m.feasible<<','<<m.timeouts<<','<<m.dimension<<','<<m.violations<<','<<m.plant_violations<<','<<m.progress<<','<<m.ey.rmse()<<','<<m.ey.percentile(.95)<<','<<m.ey.maximum()<<','<<m.heading.rmse()<<','<<m.velocity.rmse()<<','<<m.rate.rmse()<<','<<m.jerk.rmse()<<','<<m.plantjerk.maximum()<<','<<m.prediction.rmse()<<','<<m.solve.percentile(.5)<<','<<m.solve.percentile(.95)<<','<<m.solve.percentile(.99)<<','<<m.solve.maximum()<<','<<m.loop.percentile(.5)<<','<<m.loop.percentile(.95)<<','<<m.loop.percentile(.99)<<','<<m.loop.maximum()<<','<<m.model_updates<<','<<m.feedback_active<<','<<m.slip_valid<<','<<m.estimator_valid<<','<<m.nominal_yaw_prediction.rmse()<<','<<m.effective_yaw_prediction.rmse()<<','<<m.nominal_speed_prediction.rmse()<<','<<m.effective_speed_prediction.rmse()<<','<<m.normal_plant_jerk.maximum()<<','<<m.normal_plant_jerk_violations<<','<<m.stop_periods<<','<<m.stop_command_jerk.maximum()<<','<<m.stop_plant_jerk.maximum()<<'\n';++cases;
 }
 require(cases>0,"unknown mode filter: no case executed");
 std::cout<<"P6 isolated P5 closed-loop cases="<<cases<<" synthetic=1 vehicle_authority=0\n";return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
