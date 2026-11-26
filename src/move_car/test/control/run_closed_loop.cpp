// Offline simulation only: no NodeHandle, publisher, subscriber or vehicle driver.
// The baseline below calls the real production SpatialSLQR, not a reimplementation.
#define main control_pub_node_main
#include "../../src/control_pub.cpp"
#undef main
#include "control/adaptive_runtime.h"
#include <fstream>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <cstdlib>

namespace rc = research_control;
namespace {
constexpr double DT=1./30., DS=.005;
#ifdef P6_EMBED_RUNNER
// A paired fixture input, not a production speed parameter. Default unchanged.
const double TARGET=std::getenv("P6_TARGET_SPEED")?std::stod(std::getenv("P6_TARGET_SPEED")):.24;
#else
constexpr double TARGET=.24;
#endif
#ifdef P6_EMBED_RUNNER
const int p6_fixture_variant=std::getenv("P6_VARIANT")?std::atoi(std::getenv("P6_VARIANT")):0;
#endif
double test_budget_ms=1000.;
bool test_profile=false;
#ifdef P36_RESEARCH
std::string test_backend="original12";
bool test_strict_adaptation=false;
bool test_validity_adaptation=false;
const bool p37_validation=std::getenv("P37_VALIDATION_SET")&&std::string(std::getenv("P37_VALIDATION_SET"))=="1";
bool calibrated_fixture=false;
double fitted_gain=1.,fitted_tau=.44,calibrated_delay=.325,candidate_lateral=1.,candidate_smoothness=1.;
#endif
void require(bool condition,const std::string& what) {
    if(!condition) throw std::runtime_error(what);
}
struct TrackPoint {double s=0.,x=0.,y=0.,yaw=0.,kappa=0.;};
struct Track {
    std::vector<TrackPoint> p;
    explicit Track(const std::string& shape) {
        auto curvature=[&](double s) {
#ifdef P36_RESEARCH
            // Independent validation routes; never supplied to the controller
            // as a mode, ground truth label, or adaptation answer.
            if(p37_validation){
#ifdef P6_EMBED_RUNNER
              if(shape=="circle")return .28+.01*p6_fixture_variant;
              if(shape=="s_curve")return .34*std::sin((2.2+.07*p6_fixture_variant)*s+.35+.11*p6_fixture_variant);
              if(shape=="curvature_jump")return s<.73+.08*p6_fixture_variant?0.:(s<1.72+.04*p6_fixture_variant?.36+.02*p6_fixture_variant:-.29+.01*p6_fixture_variant);
#endif
              if(shape=="circle")return .28;
              if(shape=="s_curve")return .34*std::sin(2.2*s+.35);
              if(shape=="curvature_jump")return s<.73?0.:(s<1.72?.36:-.29);
              return 0.;
            }
#endif
            if(shape=="circle")return .32;
            if(shape=="s_curve")return .37*std::sin(2.8*s);
            if(shape=="curvature_jump")return s<.55 ? 0. : (s<1.45 ? .42 : -.32);
            return 0.;
        };
        TrackPoint a;a.kappa=curvature(0.);p.push_back(a);
        for(int i=1;i<=4000;++i) {
            const double k=curvature((i-.5)*DS),mid=a.yaw+.5*DS*k;
            a.x+=DS*std::cos(mid);a.y+=DS*std::sin(mid);a.yaw=rc::wrap(a.yaw+DS*k);
            a.s=i*DS;a.kappa=curvature(a.s);p.push_back(a);
        }
    }
    TrackPoint at(double s)const {
        s=rc::clamp(s,0.,p.back().s);size_t i=std::min(p.size()-2,size_t(s/DS));
        const double f=(s-p[i].s)/DS;TrackPoint a=p[i],b=p[i+1];
        a.s=s;a.x+=(b.x-a.x)*f;a.y+=(b.y-a.y)*f;
        a.yaw=rc::wrap(a.yaw+rc::wrap(b.yaw-a.yaw)*f);return a;
    }
    double nearest(double x,double y,double previous)const {
        size_t begin=size_t(std::max(0.,previous-.12)/DS);
        size_t end=std::min(p.size(),size_t((previous+.70)/DS)+1),best=begin;
        double distance=1e100;
        for(size_t i=begin;i<end;++i) {
            double d=std::hypot(x-p[i].x,y-p[i].y);
            if(d<distance){distance=d;best=i;}
        }
        return p[best].s;
    }
};
struct Ground {
    double steering_tau=.44,steering_delay=.325,speed_tau=.20,speed_delay=.20;
    double yaw_gain=1.,slip=0.,drift=0.,vibration=.035;
    double steering_gain=1.,speed_gain=1.; // P6 independent synthetic mismatch only.
};
Ground ground(const std::string& name) {
    Ground p;
    if(name=="grass_bumpy") {
        p.steering_tau=.65;p.steering_delay=.43;p.speed_tau=.30;
        p.yaw_gain=.82;p.slip=.10;p.drift=.004;p.vibration=1.6;
    } else if(name=="asphalt") {
        p.steering_tau=.34;p.steering_delay=.22;p.speed_tau=.17;
        p.yaw_gain=1.06;p.slip=.015;p.drift=-.002;p.vibration=.11;
    }
#ifdef P36_RESEARCH
    if(p37_validation){p.yaw_gain*=.97;p.steering_tau*=1.08;p.vibration*=1.05;}
#ifdef P6_EMBED_RUNNER
    if(p6_fixture_variant>1){p.yaw_gain*=.98;p.steering_tau*=1.04;p.speed_tau*=1.03;}
#endif
#endif
    return p; // Labels specify test perturbations, never a friction ordering.
}
rc::Command commandAt(const std::vector<rc::CommandSample>& h,double t) {
    for(auto i=h.rbegin();i!=h.rend();++i)if(i->stamp<=t+1e-12)return i->command;
    return h.front().command;
}
struct Plant {
    rc::State state;
    double yaw_rate=0.,lateral_velocity=0.;
    explicit Plant(const Track& track) {
        state.x=-.3;state.y=.035;state.yaw=.025;state.speed=TARGET;
        state.steering=std::atan(.60*track.at(0.).kappa);
#ifdef P36_RESEARCH
        if(p37_validation){state.y=.05;state.yaw=-.018;}
#endif
    }
    // Independent small-step plant: no use of the optimizer's transition helper.
    void advance(double now,const Ground& p,const std::vector<rc::CommandSample>& history) {
        for(int j=0;j<4;++j) {
            const double dt=DT/4.,t=now+j*dt;
            const rc::Command steering=commandAt(history,t-p.steering_delay);
            const rc::Command speed=commandAt(history,t-p.speed_delay);
            state.steering+=dt*rc::clamp((p.steering_gain*steering.steering-state.steering)/p.steering_tau,-.90,.90);
            const double desired=rc::clamp((p.speed_gain*speed.speed-state.speed)/p.speed_tau,-.38,.30);
            state.acceleration+=rc::clamp(desired-state.acceleration,-1.1*dt,1.1*dt);
            state.speed=std::max(0.,state.speed+dt*state.acceleration);
            yaw_rate=p.yaw_gain*state.speed*std::tan(state.steering)/.60;
            lateral_velocity=p.drift*(1.+.2*std::sin(1.2*t));
            const double yaw=state.yaw+.5*dt*yaw_rate;
            state.x+=dt*(state.speed*std::cos(yaw)-lateral_velocity*std::sin(yaw));
            state.y+=dt*(state.speed*std::sin(yaw)+lateral_velocity*std::cos(yaw));
            state.yaw=rc::wrap(state.yaw+dt*yaw_rate);
        }
    }
};
// Independent validation model, evaluated on EVERY tick, including fallback and
// stops. Compare the same physical motion to nominal and adapted predictions;
// this is prediction validation rather than a counterfactual control claim.
rc::State validationPrediction(rc::State s,double now,const rc::Model& m,
                              const std::vector<rc::CommandSample>& history) {
    for(int i=0;i<20;++i){const double h=DT/20.,t=now+i*h;
        auto uv=commandAt(history,t-m.speed_delay),ud=commandAt(history,t-m.steering_delay);
        const double a=(uv.speed-s.speed)/m.speed_tau+m.longitudinal_residual;
        const double r=m.yaw_gain*s.speed*std::tan(s.steering)/m.wheelbase+m.yaw_residual;
        s.x+=h*(s.speed*std::cos(s.yaw)-m.lateral_residual*std::sin(s.yaw));
        s.y+=h*(s.speed*std::sin(s.yaw)+m.lateral_residual*std::cos(s.yaw));s.yaw=rc::wrap(s.yaw+h*r);
        s.speed+=h*a;s.steering+=h*(ud.steering-s.steering)/m.steering_tau;s.acceleration=a;
    }return s;
}
struct Samples {
    std::vector<double> values;
    void add(double x){if(std::isfinite(x))values.push_back(x);}
    double rmse()const {double x=0.;for(double a:values)x+=a*a;return values.empty()?0.:std::sqrt(x/values.size());}
    double percentile(double p)const {
        if(values.empty())return 0.;auto sorted=values;for(double& a:sorted)a=std::fabs(a);
        std::sort(sorted.begin(),sorted.end());return sorted[size_t(p*(sorted.size()-1))];
    }
    double maximum()const{return percentile(1.);}
};
struct Scenario {std::string name,terrain,shape;bool delay=false,switching=false,loss=false,faults=false;};
struct Metrics {
    Samples ey,heading,speed,steer_rate,accel,jerk,yaw_error,solve,slqr_time,loop;
    int steps=0,selected=0,fallback=0,failed=0,timeout=0,infeasible=0,numerical=0;
    int saturation=0,command_violation=0,prediction_violation=0,plant_violation=0;
    int safety_command_violation=0,safety_stops=0,reference_recovered=0;
    int slip_valid=0,tau_valid=0,gain_valid=0,terrain_valid=0,terrain_matches=0;
    int model_changed=0,weights_changed=0,unknown=0;
    double progress=0.,last_gain=1.,last_tau=.44,first_cap=.3,last_cap=.3;
};

void configureBaseline(SpatialSLQR& solver) {
    // Values read from the current movecar_improved.launch. Remaining defaults
    // are the same as configure(); no ROS parameter server is required here.
    solver.horizon_steps_=20;solver.ds_=.10;solver.wheelbase_=.60;
    solver.q_y_=26.;solver.q_theta_=7.;solver.q_terminal_y_=90.;solver.q_terminal_theta_=40.;
    solver.r_delta_=.55;solver.r_delta_rate_=24.;solver.r_feedforward_=.90;
    solver.command_rate_=1.;solver.actuator_tau_=.44;solver.setRuntimeMode(1);
}

rc::Command baselineCommand(SpatialSLQR& slqr,const Track& track,const Plant& plant,
                           double progress,double now,rc::Command previous,double previous_accel,
                           const std::vector<rc::CommandSample>& history,Metrics& metrics,bool lost) {
    const auto p=track.at(progress);const auto& s=plant.state;
    const double cx=s.x+.3*std::cos(s.yaw),cy=s.y+.3*std::sin(s.yaw);
    SpatialSLQR::CenterState z;
    z<<-(cx-p.x)*std::sin(p.yaw)+(cy-p.y)*std::cos(p.yaw),rc::wrap(s.yaw-p.yaw),s.steering,previous.steering;
    double delta_progress=0.;
    // Same nominal queued-command propagation concept as the existing controller.
    for(double t=0.;t<.325-1e-9;t+=.01) {
        const double dt=std::min(.01,.325-t);
        SpatialSLQR::advanceCenter(track.at(progress+delta_progress).kappa,s.speed,
            commandAt(history,now+t-.325).steering,dt,.44,.60,z,delta_progress);
    }
    std::vector<double> curvatures(20);
    for(size_t i=0;i<curvatures.size();++i)curvatures[i]=track.at(progress+delta_progress+i*.10).kappa;
    slqr.setRuntimeEnvelope(s.speed,.45,9.80665);slqr.setActuatorState(z[2],DT);
    SpatialSLQR::SolveInfo info;const auto started=std::chrono::steady_clock::now();
    rc::Command u;u.steering=slqr.solve(z[0],z[1],curvatures,previous.steering,info);
    metrics.slqr_time.add(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count());
    const double target=lost?0.:TARGET;
    double a=rc::clamp((target-previous.speed)/DT,-.36,.28);
    a=rc::clamp(a,previous_accel-.80*DT,previous_accel+.80*DT);
    u.speed=std::max(0.,previous.speed+a*DT);if(lost)u.speed=0.;return u;
}

rc::RuntimeConfig runtimeConfig();

Metrics run(const Scenario& scenario,const std::string& mode,double seconds,std::ostream* trace,std::ostream* ticks) {
    Track track(scenario.shape);Plant plant(track);SpatialSLQR slqr;configureBaseline(slqr);
    rc::AdaptiveRuntime runtime;auto config=runtimeConfig();require(runtime.configure(config),"runtime config invalid");
    runtime.requestTerrain(scenario.terrain,0.);
    // Load/settle the manual scene before movement, as in an experiment setup.
    // These calls run legacy mode without solving and are excluded from metrics.
    rc::SolveRequest preflight;preflight.reference.valid=true;
    for(int i=1;i<=900;++i)runtime.cycle(preflight,i*DT,true,false);
    require(runtime.requestMode(mode),"mode rejected: "+mode);
    rc::Command previous;previous.speed=TARGET;previous.steering=plant.state.steering;
    double previous_acceleration=0.,last_acceleration=0.,progress=0.;
    std::vector<rc::CommandSample> history{{-2.,previous}};
    Metrics metrics;bool lost_before=false;
    for(int tick=0;tick<int(seconds/DT);++tick) {
        const auto loop_start=std::chrono::steady_clock::now();double now=31.+tick*DT;
        const double phase=tick*DT/seconds;
        std::string actual_terrain=scenario.terrain;
        if(scenario.switching)actual_terrain=phase<.33?"smooth":(phase<.70?"grass_bumpy":"asphalt");
        Ground g=ground(actual_terrain);
        #ifdef P36_RESEARCH
        if(calibrated_fixture){g.yaw_gain=fitted_gain;g.steering_tau=fitted_tau;g.steering_delay=calibrated_delay;}
        #endif
        if(scenario.delay){g.steering_delay=.58;g.steering_tau=.85;g.yaw_gain=.72;}
        if(scenario.switching && (tick==int(.33*seconds/DT)||tick==int(.70*seconds/DT)))
            runtime.requestTerrain(actual_terrain,now);
        const bool sensor_dropout=scenario.switching && phase>.52 && phase<.67;
        if(scenario.switching&&tick==int(.52*seconds/DT))runtime.requestTerrain("unknown",now);
        const bool lost=scenario.loss&&phase>.35&&phase<.52;
        auto& observer=runtime.observer();
        const double vx=plant.state.speed*std::cos(plant.state.yaw)-
            (plant.lateral_velocity+.3*plant.yaw_rate)*std::sin(plant.state.yaw);
        const double vy=plant.state.speed*std::sin(plant.state.yaw)+
            (plant.lateral_velocity+.3*plant.yaw_rate)*std::cos(plant.state.yaw);
        observer.updateWheel(plant.state.speed/(1.-g.slip),now,now);
        observer.updateSteering(plant.state.steering,now,now);
        if(!sensor_dropout) {
            observer.updateGround(vx,vy,plant.state.yaw,now,now,true);
            observer.updateImu(plant.yaw_rate,plant.state.acceleration,
                plant.state.speed*plant.yaw_rate,9.80665+g.vibration*std::sin(52.*now),
                0.,0.,now,now,true,true,true);
        }
        double cx=plant.state.x+.3*std::cos(plant.state.yaw),cy=plant.state.y+.3*std::sin(plant.state.yaw);
        progress=track.nearest(cx,cy,progress);const auto near=track.at(progress);
        rc::SolveRequest request;request.state=plant.state;request.previous=previous;
        request.previous_command_acceleration=previous_acceleration;request.now=now;
        request.control_dt=DT;request.speed_cap=.30;
        request.baseline=baselineCommand(slqr,track,plant,progress,now,previous,previous_acceleration,history,metrics,lost);
        request.history=history;request.reference.valid=!lost;request.reference.stamp=now;
        request.reference.reason=lost?"injected_reference_loss":"simulation_geometry_and_time_law";
        request.reference.count=config.solver.steps+1;
        for(int i=0;i<request.reference.count;++i) {
            const double t=i==0?0.:DT+(i-1)*config.solver.dt;
            auto p=track.at(progress+TARGET*t);auto& r=request.reference.points[i];
            r.t=t;r.s=p.s;r.x=p.x;r.y=p.y;r.yaw=p.yaw;r.curvature=p.kappa;r.speed=TARGET;
        }
        request.force_timeout=scenario.faults&&phase>.20&&phase<.25;
        request.force_infeasible=scenario.faults&&phase>.40&&phase<.45;
        request.force_numerical_failure=scenario.faults&&phase>.60&&phase<.65;
        const auto result=runtime.cycle(request,now,!lost,false);
        const auto u=result.command;const auto& limits=result.effective.limits;
        require(std::isfinite(u.speed)&&std::isfinite(u.steering),"nonfinite final command");
        require(std::fabs(u.steering)<=.32+1e-8&&u.speed>=-1e-8&&u.speed<=.50+1e-8,"physical command bound");
        require(!result.owner.empty(),"missing single command owner");
        require(!(result.nmpc_selected&&(!result.solve.ok||!result.solve.feasible||lost)),"unsafe NMPC ownership");
        const bool safety_transition=lost||lost_before;
        if(lost) {require(u.speed==0.,"reference interruption did not stop");++metrics.safety_stops;}
        if(!lost&&lost_before)++metrics.reference_recovered;
        lost_before=lost;
        if((request.force_timeout||request.force_infeasible||request.force_numerical_failure)&&mode!="legacy_slqr")
            require(!result.nmpc_selected,"injected failure retained NMPC ownership");
        const double command_acceleration=(u.speed-previous.speed)/DT;
        const double command_jerk=(command_acceleration-previous_acceleration)/DT;
        const double steering_rate=(u.steering-previous.steering)/DT;
        const bool violation=std::fabs(u.steering)>limits.steering+1e-6||
            std::fabs(steering_rate)>limits.steering_rate+1e-6||
            command_acceleration>limits.acceleration+1e-6||command_acceleration< -limits.deceleration-1e-6||
            std::fabs(command_jerk)>limits.jerk+1e-6;
        if(violation){if(safety_transition)++metrics.safety_command_violation;else ++metrics.command_violation;}
        if(result.solve.ok&&(!result.solve.feasible||result.solve.max_violation>1e-6))++metrics.prediction_violation;
        if(result.nmpc_selected)require(!violation,"accepted NMPC command failed explicit input constraints");
        ++metrics.steps;metrics.selected+=result.nmpc_selected;metrics.fallback+=mode!="legacy_slqr"&&!result.nmpc_selected;
        metrics.failed+=mode!="legacy_slqr"&&!result.solve.ok&&!lost;
        metrics.timeout+=result.solve.timed_out;
        metrics.infeasible+=result.solve.reason.find("infeasib")!=std::string::npos;
        metrics.numerical+=result.solve.reason.find("numer")!=std::string::npos;
        metrics.saturation+=std::fabs(u.steering)>.995*limits.steering;
        metrics.ey.add(-(cx-near.x)*std::sin(near.yaw)+(cy-near.y)*std::cos(near.yaw));
        metrics.heading.add(rc::wrap(plant.state.yaw+std::atan2(plant.lateral_velocity+.3*plant.yaw_rate,
            std::max(.02,plant.state.speed))-near.yaw));
        metrics.speed.add(plant.state.speed-TARGET);metrics.steer_rate.add(steering_rate);
        metrics.accel.add(plant.state.acceleration);metrics.jerk.add((plant.state.acceleration-last_acceleration)/DT);
        metrics.yaw_error.add(plant.yaw_rate-plant.state.speed*near.kappa);
        if(mode!="legacy_slqr"&&result.solve.solve_ms>0.)metrics.solve.add(result.solve.solve_ms);
        metrics.slip_valid+=result.observation.slip.valid;metrics.tau_valid+=result.observation.steering_tau.valid;
        metrics.gain_valid+=result.observation.yaw_gain.valid;metrics.terrain_valid+=result.terrain_estimate.valid;
        metrics.terrain_matches+=result.terrain_estimate.valid&&result.terrain_estimate.label==actual_terrain;
        metrics.model_changed+=std::fabs(result.effective.model.yaw_gain-config.solver.model.yaw_gain)>1e-6||
            std::fabs(result.effective.model.steering_tau-config.solver.model.steering_tau)>1e-6;
        metrics.weights_changed+=std::fabs(result.effective.weights.lateral-config.solver.weights.lateral)>1e-6||
            std::fabs(result.effective.weights.smoothness-config.solver.weights.smoothness)>1e-6;
        metrics.unknown+=result.terrain=="unknown";metrics.last_gain=result.effective.model.yaw_gain;
        metrics.last_tau=result.effective.model.steering_tau;metrics.last_cap=limits.speed;
        if(tick==0)metrics.first_cap=limits.speed;
        if(plant.state.acceleration>limits.acceleration+1e-6||plant.state.acceleration< -limits.deceleration-1e-6||
            std::fabs(plant.state.speed*plant.yaw_rate)>limits.lateral_acceleration+1e-6)++metrics.plant_violation;
        if(trace&&tick%3==0)*trace<<scenario.name<<','<<mode<<','<<now<<','<<actual_terrain<<','
            <<result.terrain_estimate.label<<','<<result.terrain_estimate.confidence<<','<<result.owner<<','
            <<result.reason<<','<<metrics.ey.values.back()<<','<<plant.state.speed<<','<<u.speed<<','<<u.steering<<','
            <<result.effective.model.yaw_gain<<','<<result.effective.model.steering_tau<<','
            <<result.effective.weights.lateral<<','<<result.effective.weights.smoothness<<','
            <<result.solve.solve_ms<<','<<result.solve.max_violation<<'\n';
        observer.recordCommand(u,now);history.push_back({now,u});
        if(history.size()>120)history.erase(history.begin());
        const auto validation_nominal=validationPrediction(plant.state,now,config.solver.model,history);
        const auto validation_adapted=validationPrediction(plant.state,now,result.effective.model,history);
        last_acceleration=plant.state.acceleration;previous_acceleration=command_acceleration;previous=u;
        plant.advance(now,g,history);metrics.progress=progress;
        if(ticks) {
            const auto& z=plant.state;double px=rc::missing(),py=rc::missing(),pv=rc::missing(),pd=rc::missing(),pa=rc::missing();
            if(result.solve.count>1){const auto& p=result.solve.predicted[1];px=z.x-p.x;py=z.y-p.y;
                pv=z.speed-p.speed;pd=z.steering-p.steering;pa=z.acceleration-p.acceleration;}
            // Reference safety override and delayed braking response are separate
            // from nominal periods; state error is measured on an independent plant.
            const bool emergency=lost||safety_transition;
            const bool physical_violation=z.acceleration>limits.acceleration+1e-6||z.acceleration< -limits.deceleration-1e-6||
                std::fabs(z.speed*plant.yaw_rate)>limits.lateral_acceleration+1e-6;
            *ticks<<scenario.name<<','<<mode<<','<<tick<<','<<now<<','<<emergency<<','<<result.owner<<','<<result.reason<<','
                <<result.solve.ok<<','<<result.solve.timed_out<<','<<result.solve.evaluations<<','<<result.solve.iterations<<','
                <<result.solve.solve_ms<<','<<command_acceleration<<','<<command_jerk<<','<<steering_rate<<','<<violation<<','
                <<z.acceleration<<','<<(z.acceleration-last_acceleration)/DT<<','<<physical_violation<<','
                <<px<<','<<py<<','<<pv<<','<<pd<<','<<pa<<','<<result.solve.warm_started;
#ifdef P35_SOLVER_PROFILE
            const auto& timing=result.solve.profile;
            *ticks<<','<<result.solve.budget_limited<<','<<result.solve.early_stopped<<','<<timing.initialization_ms<<','
                <<timing.reference_ms<<','<<timing.prediction_ms<<','<<timing.derivative_ms<<','<<timing.constraint_cost_ms<<','
                <<timing.direction_ms<<','<<timing.candidate_selection_ms<<','<<timing.other_ms;
#else
            *ticks<<",0,0,nan,nan,nan,nan,nan,nan,nan,nan";
#endif
            *ticks<<','<<result.effective.model.yaw_gain<<','<<result.effective.model.steering_tau<<','
                <<result.effective.model.yaw_residual<<','<<result.effective.model.longitudinal_residual<<','
                <<result.effective.model.lateral_residual<<','<<result.effective.weights.lateral<<','
                <<result.effective.weights.smoothness<<','<<result.effective.limits.lateral_acceleration<<','
                <<result.terrain_estimate.label<<','<<result.terrain_estimate.confidence<<','
                <<z.speed-validation_nominal.speed<<','<<z.speed-validation_adapted.speed<<','
                <<rc::wrap(z.yaw-validation_nominal.yaw)<<','<<rc::wrap(z.yaw-validation_adapted.yaw)<<','
                <<z.steering-validation_nominal.steering<<','<<z.steering-validation_adapted.steering<<','
                <<std::hypot(z.x-validation_nominal.x,z.y-validation_nominal.y)<<','
                <<std::hypot(z.x-validation_adapted.x,z.y-validation_adapted.y);
#ifdef P35_PROFILE
            *ticks<<','<<result.model_adaptation_active<<','<<result.weight_adaptation_active<<','<<result.adaptation_risk<<','<<result.adaptation_reason;
            for(const auto& e:result.adaptation)*ticks<<','<<e.samples<<','<<e.nominal_mse<<','<<e.candidate_mse<<','<<e.active<<','<<e.reason;
#else
            *ticks<<",0,0,nan,unavailable";
            for(int i=0;i<5;++i)*ticks<<",0,nan,nan,0,unavailable";
#endif
            *ticks<<'\n';
        }
        metrics.loop.add(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-loop_start).count());
    }
    return metrics;
}

rc::RuntimeConfig runtimeConfig() {
    rc::RuntimeConfig c;
#ifdef P36_RESEARCH
    c.research_backend=test_backend;c.research_strict_adaptation=test_strict_adaptation;
    c.research_validity_adaptation=test_validity_adaptation;
    if(calibrated_fixture){
      c.solver.model.yaw_gain=fitted_gain;c.solver.model.steering_tau=fitted_tau;c.solver.model.steering_delay=calibrated_delay;
      for(auto& p:c.profiles){p.lateral_scale=candidate_lateral;p.smoothness_scale=candidate_smoothness;}
    }
#endif
    c.solver.steps=20;c.solver.knots=6;c.solver.iterations=3;
    // Ample host allowance isolates algorithm comparisons from host scheduling.
    // It is deliberately NOT the production shadow budget (8 ms).
    c.solver.budget_ms=test_budget_ms;
#ifdef P35_SOLVER_PROFILE
    c.solver.profile=test_profile;
#endif
#ifdef P35_PROFILE
    c.observer.source_timestamps_verified=true;
    c.observer.steering_delay_verified=true;
#endif
    c.observer.wheel_scale=1.;c.observer.wheel_calibrated=true;
    c.observer.steering_calibrated=true;c.observer.ground_calibrated=true;
    c.observer.ground_frame_verified=true;c.observer.ground_independent=true;
    c.observer.ground_heading_verified=true;c.observer.ground_lever_arm_verified=true;
    c.observer.ground_lever_x=.30;c.observer.imu_frame_verified=true;
    c.terrain_estimation_enabled=true;
    // Synthetic calibrated channels provide the observer's conservative 0.7
    // feature confidence. Production YAML is not changed by this fixture.
    c.minimum_confidence=.70;
    return c;
}

void print(std::ostream& o,const Scenario& s,const std::string& mode,const Metrics& m) {
    o<<s.name<<','<<s.terrain<<','<<s.shape<<','<<mode<<','<<m.steps<<','<<m.progress<<','
     <<m.ey.rmse()<<','<<m.ey.percentile(.95)<<','<<m.ey.maximum()<<','
     <<m.heading.rmse()<<','<<m.heading.percentile(.95)<<','<<m.speed.rmse()<<','
     <<m.steer_rate.rmse()<<','<<m.steer_rate.maximum()<<','<<m.accel.rmse()<<','<<m.jerk.rmse()<<','
     <<m.yaw_error.rmse()<<','<<m.saturation<<','<<m.selected<<','<<m.fallback<<','<<m.failed<<','
     <<m.timeout<<','<<m.infeasible<<','<<m.numerical<<','<<m.command_violation<<','
     <<m.safety_command_violation<<','<<m.prediction_violation<<','<<m.plant_violation<<','
     <<m.safety_stops<<','<<m.reference_recovered<<','<<m.solve.percentile(.50)<<','
     <<m.solve.percentile(.95)<<','<<m.solve.percentile(.99)<<','<<m.solve.maximum()<<','
     <<m.slqr_time.percentile(.50)<<','<<m.slqr_time.percentile(.95)<<','<<m.slqr_time.percentile(.99)<<','<<m.slqr_time.maximum()<<','
     <<m.loop.percentile(.50)<<','<<m.loop.percentile(.95)<<','<<m.loop.percentile(.99)<<','<<m.loop.maximum()<<','
     <<m.slip_valid<<','<<m.tau_valid<<','<<m.gain_valid<<','<<m.terrain_valid<<','<<m.terrain_matches<<','
     <<m.model_changed<<','<<m.weights_changed<<','<<m.unknown<<','<<m.last_gain<<','<<m.last_tau<<','
     <<m.first_cap<<','<<m.last_cap<<'\n';
}
} // namespace

#ifndef P6_EMBED_RUNNER
int main(int argc,char** argv) {
    try {
        ros::Time::init();
        const std::string destination=argc>1?argv[1]:"/tmp/control_closed_loop.csv";
        const double seconds=argc>2?std::stod(argv[2]):12.;
        const std::string filter=argc>3?argv[3]:"";
        test_budget_ms=argc>4?std::stod(argv[4]):1000.;
        test_profile=argc>5?std::stoi(argv[5])!=0:false;
#ifdef P36_RESEARCH
        test_backend=argc>6?argv[6]:"original12";
        test_strict_adaptation=argc>7&&std::stoi(argv[7])==1;
        test_validity_adaptation=argc>7&&std::stoi(argv[7])==2;
        if(argc>13){calibrated_fixture=true;fitted_gain=std::stod(argv[9]);fitted_tau=std::stod(argv[10]);
          calibrated_delay=std::stod(argv[11]);candidate_lateral=std::stod(argv[12]);candidate_smoothness=std::stod(argv[13]);
          require(fitted_gain>=.5&&fitted_gain<=1.5&&fitted_tau>=.05&&fitted_tau<=1.&&calibrated_delay>=0.&&calibrated_delay<=.7&&
            candidate_lateral>=.95&&candidate_lateral<=1.05&&candidate_smoothness>=1.&&candidate_smoothness<=1.1,"invalid calibrated candidate");
        }
#endif
        require(std::isfinite(test_budget_ms)&&test_budget_ms>0.&&test_budget_ms<=1000.,"invalid test budget");
        std::ofstream out(destination),trace(destination+".trace.csv"),ticks(destination+".ticks.csv");
        ticks<<std::setprecision(12);
        ticks<<"scenario,mode,tick,time,emergency,owner,reason,ok,timed_out,evaluations,iterations,solve_ms,command_acceleration,command_jerk,command_steering_rate,command_violation,plant_acceleration,plant_jerk,plant_violation,prediction_error_x,prediction_error_y,prediction_error_speed,prediction_error_steering,prediction_error_acceleration,warm_started,budget_limited,early_stopped,setup_ms,reference_ms,prediction_ms,derivative_ms,evaluation_ms,direction_ms,feasibility_search_ms,other_ms,yaw_gain,steering_tau,yaw_residual,longitudinal_residual,lateral_residual,lateral_weight,smoothness_weight,lateral_acceleration_limit,terrain_estimate,terrain_confidence,validation_nominal_speed_error,validation_adapted_speed_error,validation_nominal_yaw_error,validation_adapted_yaw_error,validation_nominal_steering_error,validation_adapted_steering_error,validation_nominal_position_error,validation_adapted_position_error,model_adaptation_active,weight_adaptation_active,adaptation_risk,adaptation_reason";
        for(const char* n:{"yaw_gain","steering_tau","yaw_residual","longitudinal_residual","lateral_residual"})
            ticks<<','<<n<<"_evidence_samples,"<<n<<"_nominal_mse,"<<n<<"_candidate_mse,"<<n<<"_active,"<<n<<"_reason";
        ticks<<'\n';
        require(bool(out)&&bool(trace),"cannot write simulation result");out<<std::setprecision(10);trace<<std::setprecision(10);
        out<<"scenario,terrain,path,mode,steps,progress_m,lateral_rmse_m,lateral_p95_m,lateral_max_m,heading_rmse_rad,heading_p95_rad,speed_rmse_mps,steering_rate_rms_radps,steering_rate_max_radps,accel_rms_mps2,jerk_rms_mps3,yaw_residual_rms_radps,saturation_count,nmpc_selected,fallback_count,solver_failures,timeouts,infeasible,numerical,command_constraint_violations,safety_command_constraint_violations,prediction_constraint_violations,plant_constraint_violations,safety_stops,reference_recoveries,solve_p50_ms,solve_p95_ms,solve_p99_ms,solve_max_ms,slqr_p50_ms,slqr_p95_ms,slqr_p99_ms,slqr_max_ms,loop_p50_ms,loop_p95_ms,loop_p99_ms,loop_max_ms,slip_valid,tau_valid,yaw_gain_valid,terrain_valid,terrain_matches,model_changed,weights_changed,unknown_count,final_yaw_gain,final_steering_tau,initial_limit_speed,final_limit_speed\n";
        trace<<"scenario,mode,t,terrain_truth,terrain_estimate,terrain_confidence,owner,reason,lateral_error,speed,speed_command,steering_command,yaw_gain,steering_tau,weight_lateral,weight_smoothness,solve_ms,max_violation\n";
        std::vector<Scenario> scenarios;
        for(const std::string g:{"smooth","grass_bumpy","asphalt"})
            for(const std::string shape:{"straight","circle","s_curve","curvature_jump"})
                scenarios.push_back({g+"_"+shape,g,shape,false,false,false,false});
        scenarios.push_back({"steering_delay_underresponse","smooth","s_curve",true,false,false,false});
        scenarios.push_back({"terrain_switch_sensor_dropout","smooth","s_curve",false,true,false,false});
        scenarios.push_back({"reference_loss_recovery","smooth","s_curve",false,false,true,false});
        scenarios.push_back({"solver_fault_injection","smooth","curvature_jump",false,false,false,true});
        int count=0;
        for(const auto& s:scenarios) {
            if(!filter.empty()&&s.name.find(filter)==std::string::npos)continue;
            for(const std::string mode:{"legacy_slqr","nmpc_fixed","nmpc_model_adaptive","nmpc_weight_adaptive","nmpc_full_adaptive","auto_fallback"}) {
                #ifdef P36_RESEARCH
                if(argc>8&&std::string(argv[8])!=""&&mode!=argv[8])continue;
#endif
                const auto metrics=run(s,mode,seconds,&trace,&ticks);print(out,s,mode,metrics);out.flush();
                std::cerr<<s.name<<" / "<<mode<<": ey_rmse="<<metrics.ey.rmse()
                    <<" selected="<<metrics.selected<<'/'<<metrics.steps<<" failures="<<metrics.failed<<'\n';++count;
            }
        }
        require(count>0,"no scenarios selected");std::cout<<"PASS "<<count<<" offline closed-loop cases; "<<destination<<'\n';
        return 0;
    } catch(const std::exception& e) {std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}
}
#endif
