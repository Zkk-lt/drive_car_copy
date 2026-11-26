#pragma once
#include "research_types.h"
#include <deque>

namespace research_control {

// Every calibration/independence claim is opt-in. NED means a coordinate convention,
// not evidence that its velocity is independent from wheel odometry.
struct ObserverConfig {
 double wheel_scale=.001,wheel_sign=1.; bool wheel_calibrated=false;
 double steering_scale=1.,steering_sign=1.; bool steering_calibrated=false;
 double ground_scale=1.,ground_lever_x=0.,ground_lever_y=0.;
 bool ground_calibrated=false,ground_frame_verified=false,ground_independent=false;
 bool ground_heading_verified=false,ground_lever_arm_verified=false;
 double gyro_scale=1.,gyro_sign=1.,accel_scale=1.; bool imu_frame_verified=false;
 // A nonzero ROS Header can contain driver publication time. Only a measured
 // acquisition/transport timing calibration earns full temporal confidence.
 bool source_timestamps_verified=false;
 // tau and deadtime are correlated. This RLS conditions on a calibrated fixed
 // deadtime; it does not identify transport delay from these two regressors.
 bool steering_delay_verified=false;
 double stale_after=.25,sync_tolerance=.08,future_tolerance=.02;
 double min_slip_speed=.15,min_identification_speed=.15;
 double yaw_excitation=.025,steering_excitation=.015;
 double yaw_gain_min=.4,yaw_gain_max=1.6,tau_min=.08,tau_max=1.5,bias_max=.08;
 double yaw_residual_bound=.20,longitudinal_residual_bound=.40,lateral_residual_bound=.20;
 double parameter_filter_tau=1.,residual_filter_tau=.4,roughness_filter_tau=.5;
 double yaw_gain_rate=.15,tau_rate=.15,bias_rate=.02,forgetting=.995;
 bool identify_yaw=true,identify_steering=true;
};

struct RawTiming {
 double source_stamp=0.,receipt_stamp=0.,stamp=0.;
 bool accepted=false,source_time_valid=false;
 std::string reason="unobserved";
};
struct RawWheel { double value=missing(); RawTiming time; };
struct RawSteering { double value=missing(); RawTiming time; };
struct RawGround {
 double vx=missing(),vy=missing(),heading=missing(); bool heading_valid=false;
 RawTiming time;
};
struct RawImu {
 double gyro_z=missing(),ax=missing(),ay=missing(),az=missing(),roll=missing(),pitch=missing();
 bool gyro_valid=false,accel_valid=false,orientation_valid=false; RawTiming time;
};
struct IdentificationDiagnostics {
 unsigned yaw_samples=0,yaw_rejected=0,steering_samples=0,steering_rejected=0;
 double yaw_information=0.,yaw_regressor_variance=0.,steering_regressor_variance=0.;
 double yaw_innovation_rms=0.,steering_innovation_rms=0.;
};

class VehicleStateObserver {
public:
 explicit VehicleStateObserver(const ObserverConfig& config=ObserverConfig()) { configure(config); }
 void configure(const ObserverConfig& config) {
   config_=config; config_valid_=validConfig(config_); reset();
 }
 const ObserverConfig& config() const { return config_; }
 bool configValid() const { return config_valid_; }
 const RawWheel& rawWheel() const { return wheel_; }
 const RawSteering& rawSteering() const { return steering_; }
 const RawGround& rawGround() const { return ground_; }
 const RawImu& rawImu() const { return imu_; }
 const IdentificationDiagnostics& identificationDiagnostics() const { return diagnostics_; }
 const std::deque<CommandSample>& commandHistory() const { return history_; }
 void reset() {
   wheel_=RawWheel(); steering_=RawSteering(); ground_=RawGround(); imu_=RawImu();
   wheel_order_=steer_order_=ground_order_=imu_order_=0.;
   history_.clear(); last_now_=0.; clock_epoch_=0.; resetIdentification();
 }
 bool updateWheel(double raw,double source,double receipt) {
   wheel_.value=raw;
   return accept(wheel_.time,wheel_order_,source,receipt,std::isfinite(raw));
 }
 bool updateSteering(double raw,double source,double receipt) {
   steering_.value=raw;
   return accept(steering_.time,steer_order_,source,receipt,std::isfinite(raw));
 }
 // vx,vy and heading MUST use the same verified world axes. heading is body
 // heading from an independent attitude source, never course atan2(vy,vx).
 bool updateGround(double vx,double vy,double body_heading,double source,double receipt,
                   bool heading_valid=true) {
   ground_.vx=vx; ground_.vy=vy; ground_.heading=body_heading;
   ground_.heading_valid=heading_valid && std::isfinite(body_heading);
   return accept(ground_.time,ground_order_,source,receipt,
                 std::isfinite(vx)&&std::isfinite(vy));
 }
 // Adapters can invalidate a coordinate/time contract after preserving the
 // received values. No frame name is itself proof of a calibrated transform.
 void invalidateGround(const std::string& reason) { ground_.time.accepted=false;ground_.time.reason=reason; }
 void invalidateImu(const std::string& reason) { imu_.time.accepted=false;imu_.time.reason=reason; }
 // Calibrated IMU convention: FLU, positive left yaw, acceleration in m/s^2;
 // roll/pitch rotate body acceleration to world up. Missing orientation does
 // not invalidate angular velocity, but disables vertical roughness.
 bool updateImu(double gz,double ax,double ay,double az,double roll,double pitch,
                double source,double receipt,bool gyro_valid=true,
                bool accel_valid=true,bool orientation_valid=false) {
   imu_.gyro_z=gz; imu_.ax=ax; imu_.ay=ay; imu_.az=az; imu_.roll=roll; imu_.pitch=pitch;
   imu_.gyro_valid=gyro_valid&&std::isfinite(gz);
   imu_.accel_valid=accel_valid&&std::isfinite(ax)&&std::isfinite(ay)&&std::isfinite(az);
   imu_.orientation_valid=orientation_valid&&std::isfinite(roll)&&std::isfinite(pitch);
   return accept(imu_.time,imu_order_,source,receipt,imu_.gyro_valid||imu_.accel_valid);
 }
 bool recordCommand(const Command& command,double stamp) {
   if (!std::isfinite(stamp)||stamp<=0.||!std::isfinite(command.speed)||
       !std::isfinite(command.steering)||(!history_.empty()&&stamp<=history_.back().stamp))
     return false;
   history_.push_back(CommandSample{stamp,command});
   while(history_.size()>512 || (!history_.empty() && stamp-history_.front().stamp>12.))
     history_.pop_front();
   return true;
 }

 Observation observe(double now,const Model& model) {
   Observation out; out.stamp=now;
   if(!config_valid_) { invalidateAll(out,"invalid_observer_config",now); return out; }
   if(!std::isfinite(now)||now<=0.) { invalidateAll(out,"invalid_clock",now); return out; }
   if(last_now_>0. && now<last_now_) {
     // ROS simulated-time reset starts a new epoch. Old derivatives and command
     // delays must never bridge the discontinuity. Fresh channel samples needed.
     reset(); last_now_=now; clock_epoch_=now;
     invalidateAll(out,"clock_regressed",now); return out;
   }
   const double dt=last_now_>0.?clamp(now-last_now_,0.,config_.stale_after):0.;
   last_now_=now;
   if(!std::isfinite(model.wheelbase)||model.wheelbase<=.05 ||
      !std::isfinite(model.steering_delay)||model.steering_delay<0. ||
      !std::isfinite(model.speed_delay)||model.speed_delay<0. ||
      !std::isfinite(model.speed_tau)||model.speed_tau<=0. ||
      !std::isfinite(model.yaw_gain)) {
     invalidateAll(out,"invalid_nominal_model",now); return out;
   }
   Estimate wheel_speed=channel(wheel_.time,now);
   if(wheel_speed.valid&&!config_.wheel_calibrated)
     wheel_speed=invalid("wheel_calibration_unverified",wheel_.time.stamp);
   if(wheel_speed.valid) {
     wheel_speed.value=wheel_.value*config_.wheel_scale*config_.wheel_sign;
     wheel_speed.confidence*=.70; wheel_speed.reason="calibrated_wheel_speed";
     if(std::fabs(wheel_speed.value)>10.) wheel_speed=invalid("wheel_range",wheel_.time.stamp);
   }
   out.steering=channel(steering_.time,now);
   if(out.steering.valid&&!config_.steering_calibrated)
     out.steering=invalid("steering_calibration_unverified",steering_.time.stamp);
   if(out.steering.valid) {
     out.steering.value=steering_.value*config_.steering_scale*config_.steering_sign;
     out.steering.reason="calibrated_steering";
     if(std::fabs(out.steering.value)>.8) out.steering=invalid("steering_range",steering_.time.stamp);
   }
   out.yaw_rate=channel(imu_.time,now);
   if(out.yaw_rate.valid&&!config_.imu_frame_verified)
     out.yaw_rate=invalid("imu_frame_unverified",imu_.time.stamp);
   if(out.yaw_rate.valid&&!imu_.gyro_valid)
     out.yaw_rate=invalid("gyro_unavailable",imu_.time.stamp);
   if(out.yaw_rate.valid) {
     out.yaw_rate.value=imu_.gyro_z*config_.gyro_scale*config_.gyro_sign;
     out.yaw_rate.reason="calibrated_gyro";
     if(std::fabs(out.yaw_rate.value)>5.) out.yaw_rate=invalid("gyro_range",imu_.time.stamp);
   }
   Estimate ground_speed=channel(ground_.time,now);
   double lateral=missing();
   if(ground_speed.valid) {
     if(!config_.ground_calibrated) ground_speed=invalid("ground_calibration_unverified",ground_.time.stamp);
     else if(!config_.ground_frame_verified) ground_speed=invalid("ground_frame_unverified",ground_.time.stamp);
     else if(!config_.ground_heading_verified||!ground_.heading_valid)
       ground_speed=invalid("independent_body_heading_unverified",ground_.time.stamp);
     else if(!config_.ground_lever_arm_verified)
       ground_speed=invalid("ground_lever_arm_unverified",ground_.time.stamp);
     else {
       const double c=std::cos(ground_.heading),s=std::sin(ground_.heading);
       ground_speed.value=config_.ground_scale*(c*ground_.vx+s*ground_.vy);
       lateral=config_.ground_scale*(-s*ground_.vx+c*ground_.vy);
       if(std::hypot(config_.ground_lever_x,config_.ground_lever_y)>1e-6) {
         if(!out.yaw_rate.valid||!synced(ground_speed,out.yaw_rate))
           ground_speed=invalid("lever_arm_yaw_unavailable",ground_.time.stamp);
         else { ground_speed.value+=out.yaw_rate.value*config_.ground_lever_y;
                lateral-=out.yaw_rate.value*config_.ground_lever_x; }
       }
       if(ground_speed.valid) {
         ground_speed.confidence*=.9; ground_speed.reason="rear_axle_ground_velocity";
         if(std::hypot(ground_speed.value,lateral)>10.)
           ground_speed=invalid("ground_velocity_range",ground_.time.stamp);
       }
     }
   }
   out.speed=ground_speed.valid?ground_speed:wheel_speed;
   if(!out.speed.valid) out.speed.reason="speed_unavailable:"+ground_speed.reason+":"+wheel_speed.reason;
   out.slip=invalid("independent_speed_pair_unavailable",now);
   if(!config_.ground_independent) out.slip.reason="ground_wheel_independence_unverified";
   else if(ground_speed.valid&&wheel_speed.valid) {
     if(!synced(ground_speed,wheel_speed)) out.slip.reason="speed_pair_unsynchronized";
     else if(std::max(std::fabs(wheel_speed.value),std::fabs(ground_speed.value))<config_.min_slip_speed)
       out.slip.reason="low_speed_unobservable";
     else {
       // Signed body-x convention: positive excess wheel velocity is positive
       // lambda, including braking while reversing. A drive/brake label needs
       // torque information, which this estimator does not invent.
       out.slip=estimate((wheel_speed.value-ground_speed.value)/
          std::max({std::fabs(wheel_speed.value),std::fabs(ground_speed.value),config_.min_slip_speed}),
          std::min(wheel_speed.confidence,ground_speed.confidence),
          std::max(wheel_speed.stamp,ground_speed.stamp),"signed_kinematic_slip");
     }
   }
   out.sideslip=invalid("ground_heading_or_motion_unavailable",now);
   out.lateral_residual=invalid("ground_heading_or_motion_unavailable",now);
   if(ground_speed.valid) {
     if(!config_.ground_independent) {
       out.sideslip.reason=out.lateral_residual.reason="independent_ground_motion_unverified";
     } else if(ground_speed.value<config_.min_slip_speed) {
       out.sideslip.reason=out.lateral_residual.reason="low_speed_or_reverse_unobservable";
     } else {
       out.sideslip=estimate(std::atan2(lateral,ground_speed.value),ground_speed.confidence*.8,
                           ground_speed.stamp,"rear_axle_motion_direction_deviation");
       out.lateral_residual=estimate(clamp(lateral,-config_.lateral_residual_bound,
                                          config_.lateral_residual_bound),
                           ground_speed.confidence*.7,ground_speed.stamp,"bounded_lateral_velocity");
     }
   }
   updateYaw(out,ground_speed,model,dt,now);
   updateSteeringModel(out,model,dt,now);
   updateLongitudinal(out,ground_speed,model,dt,now);
   updateRoughness(out,now);
   return out;
 }

private:
 struct ScalarRLS {
   double theta=1.,p=100.,information=0.,s0=0.,sx=0.; int samples=0;
   void update(double x,double y,double forgetting,double lo,double hi) {
     const double k=p*x/(forgetting+x*x*p);
     theta=clamp(theta+k*(y-x*theta),lo,hi);
     p=clamp((p-k*x*p)/forgetting,1e-5,1e6);
     information=forgetting*information+x*x;
     s0=forgetting*s0+1.;sx=forgetting*sx+x;++samples;
   }
   double variance() const {return s0>0.?std::max(0.,information/s0-(sx/s0)*(sx/s0)):0.;}
 } yaw_rls_;
 struct SteeringRLS {
   double a=1./.44,b=0.,p00=100.,p01=0.,p11=100.;
   double s0=0.,sx=0.,sxx=0.; int samples=0;
   void update(double x,double y,double forgetting,const ObserverConfig& cfg) {
     const double px=p00*x+p01,py=p01*x+p11;
     const double den=forgetting+x*px+py,k0=px/den,k1=py/den;
     const double residual=y-a*x-b;
     a=clamp(a+k0*residual,1./cfg.tau_max,1./cfg.tau_min);
     b=clamp(b+k1*residual,-cfg.bias_max*a,cfg.bias_max*a);
     const double q00=(p00-k0*px)/forgetting,q01=(p01-k0*py)/forgetting;
     const double q11=(p11-k1*py)/forgetting;
     p00=clamp(q00,1e-7,1e6);p01=q01;p11=clamp(q11,1e-7,1e6);
     // Numerical round-off cannot turn covariance indefinite.
     p01=clamp(p01,-.999*std::sqrt(p00*p11),.999*std::sqrt(p00*p11));
     s0=forgetting*s0+1.;sx=forgetting*sx+x;sxx=forgetting*sxx+x*x;++samples;
   }
   double variance() const { return s0>0.?std::max(0.,sxx/s0-(sx/s0)*(sx/s0)):0.; }
 } steer_rls_;
 ObserverConfig config_; bool config_valid_=false;
 RawWheel wheel_; RawSteering steering_; RawGround ground_; RawImu imu_;
 std::deque<CommandSample> history_;
 double wheel_order_=0.,steer_order_=0.,ground_order_=0.,imu_order_=0.;
 double last_now_=0.,clock_epoch_=0.;
 double last_yaw_stamp_=0.,last_steer_stamp_=0.,last_steer_=0.,last_ground_stamp_=0.,last_ground_=0.;
 double last_rough_stamp_=0.,rough_mean_=0.,rough_variance_=0.; int rough_samples_=0;
 double filtered_yaw_gain_=1.,filtered_tau_=.44,filtered_bias_=0.;
 double filtered_yaw_residual_=0.,filtered_acceleration_residual_=0.;
 double yaw_last_excited_=0.,steer_last_excited_=0.;
 Estimate acceleration_residual_,yaw_residual_,roughness_;
 IdentificationDiagnostics diagnostics_;
 bool yaw_sample_rejected_=false,steer_sample_rejected_=false;

 static bool validConfig(const ObserverConfig& c) {
   const double positive[]={c.wheel_scale,c.steering_scale,c.ground_scale,c.gyro_scale,c.accel_scale,
      c.stale_after,c.sync_tolerance,c.min_slip_speed,c.min_identification_speed,c.yaw_excitation,
      c.steering_excitation,c.yaw_gain_min,c.yaw_gain_max,c.tau_min,c.tau_max,c.bias_max,
      c.yaw_residual_bound,c.longitudinal_residual_bound,c.lateral_residual_bound,
      c.parameter_filter_tau,c.residual_filter_tau,c.roughness_filter_tau,c.yaw_gain_rate,
      c.tau_rate,c.bias_rate,c.forgetting};
   for(double v:positive) if(!std::isfinite(v)||v<=0.) return false;
   return std::fabs(std::fabs(c.wheel_sign)-1.)<1e-8 &&
     std::fabs(std::fabs(c.steering_sign)-1.)<1e-8 && std::fabs(std::fabs(c.gyro_sign)-1.)<1e-8 &&
     std::isfinite(c.ground_lever_x)&&std::isfinite(c.ground_lever_y)&&
     std::isfinite(c.future_tolerance)&&c.future_tolerance>=0.&&c.yaw_gain_min<c.yaw_gain_max&&
     c.tau_min<c.tau_max&&c.forgetting<=1.;
 }
 void resetIdentification() {
   yaw_rls_=ScalarRLS();steer_rls_=SteeringRLS();
   filtered_yaw_gain_=clamp(1.,config_.yaw_gain_min,config_.yaw_gain_max);
   filtered_tau_=clamp(.44,config_.tau_min,config_.tau_max);filtered_bias_=0.;
   last_yaw_stamp_=last_steer_stamp_=last_ground_stamp_=last_rough_stamp_=0.;
   last_steer_=last_ground_=rough_mean_=rough_variance_=0.;rough_samples_=0;
   filtered_yaw_residual_=filtered_acceleration_residual_=0.;
   yaw_last_excited_=steer_last_excited_=0.;
   acceleration_residual_=Estimate();yaw_residual_=Estimate();roughness_=Estimate();
   diagnostics_=IdentificationDiagnostics();yaw_sample_rejected_=steer_sample_rejected_=false;
 }
 bool accept(RawTiming& t,double& ordering,double source,double receipt,bool finite_values) {
   t.source_stamp=source;t.receipt_stamp=receipt;t.source_time_valid=source>0.;
   t.stamp=t.source_time_valid?source:receipt;t.accepted=false;
   if(!std::isfinite(source)||!std::isfinite(receipt)||source<0.||receipt<=0.) t.reason="invalid_stamp";
   else if(source>receipt+config_.future_tolerance) t.reason="future_source_stamp";
   else if(!finite_values) t.reason="nonfinite_sensor";
   else if(t.stamp<=ordering) t.reason="duplicate_or_out_of_order";
   else if(receipt-t.stamp>config_.stale_after) t.reason="late_sensor_sample";
   else { ordering=t.stamp;t.accepted=true;t.reason=t.source_time_valid?
       (config_.source_timestamps_verified?"source_timestamp_verified":"driver_timestamp_unverified"):
       "receipt_timestamp_only"; }
   return t.accepted;
 }
 static Estimate estimate(double value,double confidence,double stamp,const std::string& reason) {
   Estimate e;e.value=value;e.confidence=clamp(confidence,0.,1.);e.stamp=stamp;
   e.valid=std::isfinite(value);e.reason=e.valid?reason:"nonfinite_estimate";return e;
 }
 static Estimate invalid(const std::string& reason,double stamp) {
   Estimate e;e.stamp=stamp;e.reason=reason;return e;
 }
 static void invalidateAll(Observation& o,const std::string& reason,double stamp) {
   Estimate* fields[]={&o.speed,&o.slip,&o.yaw_rate,&o.yaw_reference,&o.yaw_residual,
     &o.steering_tau,&o.steering_bias,&o.sideslip,&o.roughness,&o.yaw_gain,
     &o.longitudinal_residual,&o.lateral_residual,&o.steering};
   for(Estimate* p:fields) *p=invalid(reason,stamp);
 }
 Estimate channel(const RawTiming& t,double now) const {
   if(!t.accepted)return invalid(t.reason,t.stamp);
   if(t.receipt_stamp>now+config_.future_tolerance||t.stamp>now+config_.future_tolerance)
     return invalid("future_measurement",t.stamp);
   if(now-t.stamp>config_.stale_after||now-t.receipt_stamp>config_.stale_after)
     return invalid("stale_measurement",t.stamp);
   return estimate(0.,t.source_time_valid&&config_.source_timestamps_verified?1.:.65,t.stamp,t.reason);
 }
 bool synced(const Estimate& a,const Estimate& b) const {
   return a.valid&&b.valid&&std::fabs(a.stamp-b.stamp)<=config_.sync_tolerance;
 }
 bool commandAt(double stamp,Command& command) const {
   for(auto it=history_.rbegin();it!=history_.rend();++it) {
     if(it->stamp<=stamp) {
       if(stamp-it->stamp>config_.stale_after)return false;
       command=it->command;return true;
     }
   }
   return false;
 }
 static double smoothBounded(double old,double target,double dt,double tau,double max_rate) {
   if(dt<=0.)return old;
   const double change=(1.-std::exp(-dt/tau))*(target-old);
   return old+clamp(change,-max_rate*dt,max_rate*dt);
 }
 void updateYaw(Observation& o,const Estimate& ground_speed,const Model& model,double dt,double now) {
   o.yaw_reference=invalid("synchronized_speed_steering_unavailable",now);
   o.yaw_residual=invalid("synchronized_gyro_unavailable",now);
   o.yaw_gain=invalid(config_.identify_yaw?"yaw_excitation_insufficient":"yaw_identification_disabled",now);
   if(!synced(o.speed,o.steering))return;
   const double phi=o.speed.value*std::tan(o.steering.value)/model.wheelbase;
   o.yaw_reference=estimate(model.yaw_gain*phi,
      std::min(o.speed.confidence,o.steering.confidence),std::max(o.speed.stamp,o.steering.stamp),
      "nominal_bicycle_yaw_rate");
   if(!synced(o.yaw_reference,o.yaw_rate))return;
   const double innovation=o.yaw_rate.value-o.yaw_reference.value;
   if(o.yaw_rate.stamp>last_yaw_stamp_) {
     yaw_sample_rejected_=false;
     // A bounded residual must not make an implausible gyro sample appear to
     // be valid model error. Retain the raw gyro for audit, reject the fit.
     if(std::fabs(innovation)>config_.yaw_residual_bound) {
       yaw_sample_rejected_=true;++diagnostics_.yaw_rejected;
       yaw_residual_=invalid("yaw_innovation_outlier",o.yaw_rate.stamp);
     } else {
     filtered_yaw_residual_=smoothBounded(filtered_yaw_residual_,
        clamp(innovation,-config_.yaw_residual_bound,config_.yaw_residual_bound),
        dt,config_.residual_filter_tau,1.);
     yaw_residual_=estimate(filtered_yaw_residual_,
         std::min(o.yaw_rate.confidence,o.yaw_reference.confidence)*.8,o.yaw_rate.stamp,
         "bounded_yaw_model_residual_not_friction");
     }
     // Wheel-only speed cannot identify yaw gain separately from longitudinal
     // slip. Independent ground speed, synchronized steering, and excitation
     // are prerequisites. This scalar gain is not a tire friction coefficient.
     if(!yaw_sample_rejected_&&config_.identify_yaw&&config_.ground_independent&&ground_speed.valid&&
        synced(ground_speed,o.yaw_rate)&&std::fabs(ground_speed.value)>=config_.min_identification_speed&&
        std::fabs(phi)>=config_.yaw_excitation) {
       const double lower=std::min(config_.yaw_gain_min*phi,config_.yaw_gain_max*phi);
       const double upper=std::max(config_.yaw_gain_min*phi,config_.yaw_gain_max*phi);
       const double noise_margin=.5*config_.yaw_excitation;
       if(o.yaw_rate.value<lower-noise_margin||o.yaw_rate.value>upper+noise_margin) {
         yaw_sample_rejected_=true;++diagnostics_.yaw_rejected;
       } else {
         const double error=o.yaw_rate.value-yaw_rls_.theta*phi;
         yaw_rls_.update(phi,o.yaw_rate.value,config_.forgetting,config_.yaw_gain_min,config_.yaw_gain_max);
         diagnostics_.yaw_samples=yaw_rls_.samples;
         diagnostics_.yaw_information=yaw_rls_.information;
         diagnostics_.yaw_regressor_variance=yaw_rls_.variance();
         diagnostics_.yaw_innovation_rms=std::sqrt(.98*diagnostics_.yaw_innovation_rms*
             diagnostics_.yaw_innovation_rms+.02*error*error);
         yaw_last_excited_=now;
       }
     }
     last_yaw_stamp_=o.yaw_rate.stamp;
   }
   o.yaw_residual=yaw_residual_;
   if(!config_.identify_yaw)return;
   if(!config_.ground_independent||!ground_speed.valid) {
     o.yaw_gain.reason="yaw_gain_requires_independent_speed";return;
   }
   if(std::fabs(ground_speed.value)<config_.min_identification_speed) {
     o.yaw_gain.reason="low_speed_yaw_unobservable";return;
   }
   if(std::fabs(phi)<config_.yaw_excitation) return;
   if(yaw_sample_rejected_) {o.yaw_gain.reason="yaw_rls_innovation_rejected";return;}
   if(yaw_rls_.samples<20||yaw_rls_.information<.03||now-yaw_last_excited_>.5)return;
   // At a single steady turn a gyro offset and gain error are inseparable.
   // This scalar fit does not claim to estimate that offset; demand changing
   // steering/speed as an additional conservative validation gate.
   if(yaw_rls_.variance()<.1*config_.yaw_excitation*config_.yaw_excitation) {
     o.yaw_gain.reason="yaw_regressor_variation_insufficient";return;
   }
   if(yaw_rls_.theta<=config_.yaw_gain_min+1e-6||yaw_rls_.theta>=config_.yaw_gain_max-1e-6) {
     o.yaw_gain.reason="yaw_rls_at_parameter_bound";return;
   }
   filtered_yaw_gain_=smoothBounded(filtered_yaw_gain_,yaw_rls_.theta,dt,
                                   config_.parameter_filter_tau,config_.yaw_gain_rate);
   o.yaw_gain=estimate(filtered_yaw_gain_,
      std::min(.9,yaw_rls_.information/.3)*
      std::min({ground_speed.confidence,o.yaw_rate.confidence,o.steering.confidence}),
      last_yaw_stamp_,"excitation_gated_bounded_rls_gain");
 }
 void updateSteeringModel(Observation& o,const Model& model,double dt,double now) {
   const std::string disabled=config_.identify_steering?"steering_excitation_insufficient":"steering_identification_disabled";
   o.steering_tau=invalid(disabled,now);o.steering_bias=invalid(disabled,now);
   if(config_.identify_steering&&!config_.steering_delay_verified) {
     o.steering_tau.reason=o.steering_bias.reason="steering_deadtime_unverified";return;
   }
   if(!o.steering.valid)return;
   if(o.steering.stamp>last_steer_stamp_) {
     steer_sample_rejected_=false;
     const double ds=o.steering.stamp-last_steer_stamp_;
     Command delayed;
     if(config_.identify_steering&&last_steer_stamp_>0.&&ds>=.005&&ds<=config_.stale_after&&
        commandAt(.5*(o.steering.stamp+last_steer_stamp_)-model.steering_delay,delayed)) {
       const double delta_mid=.5*(last_steer_+o.steering.value);
       const double x=delayed.steering-delta_mid;
       const double derivative=(o.steering.value-last_steer_)/ds;
       const double expected=steer_rls_.a*x+steer_rls_.b,error=derivative-expected;
       // Gross innovations are rejected after warm-up; accepting a clipped
       // inverse-tau fit would otherwise hide a sensor step as fast actuation.
       if(std::fabs(derivative)>=3. || (steer_rls_.samples>=30 &&
          std::fabs(error)>.35+.5*std::fabs(expected))) {
         steer_sample_rejected_=true;++diagnostics_.steering_rejected;
       } else if(std::fabs(x)>=config_.steering_excitation) {
         steer_rls_.update(x,derivative,config_.forgetting,config_);
         diagnostics_.steering_samples=steer_rls_.samples;
         diagnostics_.steering_regressor_variance=steer_rls_.variance();
         diagnostics_.steering_innovation_rms=std::sqrt(.98*diagnostics_.steering_innovation_rms*
             diagnostics_.steering_innovation_rms+.02*error*error);
         steer_last_excited_=now;
       }
     }
     last_steer_stamp_=o.steering.stamp;last_steer_=o.steering.value;
   }
   if(!config_.identify_steering)return;
   if(steer_sample_rejected_) {
     o.steering_tau.reason=o.steering_bias.reason="steering_rls_innovation_rejected";return;
   }
   if(steer_rls_.samples<30||steer_rls_.variance()<.0001||now-steer_last_excited_>.5)return;
   if(steer_rls_.a<=1./config_.tau_max+1e-6||steer_rls_.a>=1./config_.tau_min-1e-6) {
     o.steering_tau.reason=o.steering_bias.reason="steering_rls_at_parameter_bound";return;
   }
   filtered_tau_=smoothBounded(filtered_tau_,1./steer_rls_.a,dt,config_.parameter_filter_tau,config_.tau_rate);
   filtered_bias_=smoothBounded(filtered_bias_,steer_rls_.b/steer_rls_.a,dt,
                               config_.parameter_filter_tau,config_.bias_rate);
   const double confidence=std::min(.9,steer_rls_.variance()/.003)*o.steering.confidence;
   o.steering_tau=estimate(filtered_tau_,confidence,last_steer_stamp_,"rls_tau_given_fixed_command_delay");
   o.steering_bias=estimate(filtered_bias_,confidence,last_steer_stamp_,"rls_steering_offset_not_road_bias");
 }
 void updateLongitudinal(Observation& o,const Estimate& ground,const Model& model,double dt,double now) {
   o.longitudinal_residual=invalid("independent_ground_derivative_unavailable",now);
   if(!config_.ground_independent||!ground.valid)return;
   if(ground.stamp>last_ground_stamp_) {
     const double ds=ground.stamp-last_ground_stamp_;Command delayed;
     if(last_ground_stamp_>0.&&ds>=.005&&ds<=config_.stale_after&&
        commandAt(.5*(ground.stamp+last_ground_stamp_)-model.speed_delay,delayed)) {
       const double derivative=(ground.value-last_ground_)/ds;
       const double predicted=(delayed.speed-.5*(ground.value+last_ground_))/model.speed_tau;
       if(std::fabs(derivative)<4.) {
         filtered_acceleration_residual_=smoothBounded(filtered_acceleration_residual_,
             clamp(derivative-predicted,-config_.longitudinal_residual_bound,
                   config_.longitudinal_residual_bound),dt,config_.residual_filter_tau,1.);
         acceleration_residual_=estimate(filtered_acceleration_residual_,ground.confidence*.7,
             ground.stamp,"bounded_acceleration_model_residual");
       } else acceleration_residual_=invalid("ground_derivative_outlier",ground.stamp);
     } else acceleration_residual_=invalid("delayed_command_or_derivative_unavailable",ground.stamp);
     last_ground_stamp_=ground.stamp;last_ground_=ground.value;
   }
   if(acceleration_residual_.valid&&now-acceleration_residual_.stamp<=config_.stale_after)
     o.longitudinal_residual=acceleration_residual_;
 }
 void updateRoughness(Observation& o,double now) {
   o.roughness=invalid("vertical_acceleration_unavailable",now);
   if(!channel(imu_.time,now).valid||!config_.imu_frame_verified||!imu_.accel_valid)return;
   if(!imu_.orientation_valid) {o.roughness.reason="imu_tilt_unavailable";return;}
   const double vertical=config_.accel_scale*(-std::sin(imu_.pitch)*imu_.ax+
       std::sin(imu_.roll)*std::cos(imu_.pitch)*imu_.ay+
       std::cos(imu_.roll)*std::cos(imu_.pitch)*imu_.az);
   if(std::fabs(vertical)>50.) {o.roughness.reason="acceleration_range";return;}
   if(imu_.time.stamp>last_rough_stamp_) {
     const double ds=imu_.time.stamp-last_rough_stamp_;
     if(last_rough_stamp_<=0.||ds>config_.stale_after) {
       rough_mean_=vertical;rough_variance_=0.;rough_samples_=1;
     } else {
       const double a=1.-std::exp(-ds/config_.roughness_filter_tau),d=vertical-rough_mean_;
       rough_mean_+=a*d;rough_variance_=(1.-a)*(rough_variance_+a*d*d);++rough_samples_;
     }
     last_rough_stamp_=imu_.time.stamp;
   }
   if(rough_samples_<20) {o.roughness.reason="roughness_window_warming";return;}
   o.roughness=estimate(std::sqrt(std::max(0.,rough_variance_)),
        channel(imu_.time,now).confidence*.7,imu_.time.stamp,"detrended_vertical_acceleration_rms");
 }
};
} // namespace research_control
