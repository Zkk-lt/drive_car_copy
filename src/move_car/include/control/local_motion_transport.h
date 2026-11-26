#pragma once
#include <array>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace research_control {

// Research-only, receipt-causal short-window transport. It estimates relative
// motion from wheel-speed proxy and gyro, never independent ground velocity.
// Bounds below are engineering assumptions to be calibrated, not a proof of
// physical uncertainty or a grant of real-vehicle control authority.
struct LocalMotionConfig {
  double wheel_scale=.001,wheel_sign=1.,steering_scale=1.,steering_sign=1.;
  double gyro_scale=1.,gyro_sign=1.,wheelbase=.60;
  bool wheel_calibrated=false,steering_calibrated=false,imu_frame_verified=false;
  bool source_timestamps_verified=false,allow_bicycle_fallback=true;
  double history_seconds=3.,max_transport_s=.50,max_sample_age_s=.15;
  double max_source_delay_s=.25,max_step_s=.02;
  double max_abs_speed=2.,max_abs_steering=.60,max_abs_yaw_rate=1.5;
  double low_speed_threshold=.04;
  // Offset of the transported frame from the rear/integration anchor, in FLU.
  // Zero matches S8's integration anchor; .30 m is an explicit center mapping.
  double reference_offset_x=0.,reference_offset_y=0.;
  double speed_uncertainty_mps=.05,gyro_uncertainty_radps=.02;
  double bicycle_uncertainty_radps=.10;
  double max_position_uncertainty_m=.10,max_yaw_uncertainty_rad=.10;
};

struct LocalRawStatus {
  double raw=std::numeric_limits<double>::quiet_NaN();
  double value=std::numeric_limits<double>::quiet_NaN();
  double source_stamp=0.,receipt_stamp=0.;
  bool available=false,source_time_present=false,source_time_verified=false;
  const char* reason="unobserved";
};

struct LocalTransportResult {
  bool valid=false;
  const char* reason="uninitialized";
  // q_new = R(yaw) q_old + [tx,ty]. No global pose is required or returned.
  double tx=0.,ty=0.,yaw=0.,duration=0.;
  double position_uncertainty_m=0.,yaw_uncertainty_rad=0.;
  bool gyro_measured_entire_window=true,bicycle_fallback_used=false;
  bool source_timestamps_verified=true,low_speed_observed=false;
  // Headerless CAN receipt cannot establish acquisition freshness/new frames.
  bool hardware_freshness_verified=false,independent_ground_speed=false;
  bool real_vehicle_control_eligible=false;
  LocalRawStatus wheel,steering,imu;
  unsigned steps=0;
  std::uint64_t rejected_samples=0,overwritten_samples=0;
  double pointPositionUncertainty(double radius)const {
    if(!std::isfinite(radius)||radius<0.)return std::numeric_limits<double>::infinity();
    return position_uncertainty_m+radius*yaw_uncertainty_rad;
  }
};

template<std::size_t Capacity=512> class LocalMotionTransport {
  static_assert(Capacity>1,"relative transport needs bounded channel history");
  struct Sample {double raw=0.,source=0.,receipt=0.;};
  struct Buffer {
    std::array<Sample,Capacity> data{};
    std::size_t next=0,count=0;
    double latest_receipt=0.,latest_source=0.;
    std::uint64_t rejected=0,overwritten=0;
    const char* rejection="none";
  } wheel_,steering_,imu_;
  LocalMotionConfig config_;
  bool config_valid_=false;
  double last_control_=0.;
  static constexpr double eps_=1e-10;

  static bool finitePositive(double x) {return std::isfinite(x)&&x>0.;}
  static bool validConfig(const LocalMotionConfig& c) {
    const double positive[]={c.wheel_scale,c.steering_scale,c.gyro_scale,c.wheelbase,
      c.history_seconds,c.max_transport_s,c.max_sample_age_s,c.max_source_delay_s,
      c.max_step_s,c.max_abs_speed,c.max_abs_steering,c.max_abs_yaw_rate,
      c.speed_uncertainty_mps,c.gyro_uncertainty_radps,c.bicycle_uncertainty_radps,
      c.max_position_uncertainty_m,c.max_yaw_uncertainty_rad};
    for(double x:positive)if(!finitePositive(x))return false;
    return c.wheelbase>.05&&c.max_transport_s<=c.history_seconds&&c.max_step_s<=.02&&
      c.max_transport_s/c.max_step_s<=2000.&&
      std::isfinite(c.reference_offset_x)&&std::isfinite(c.reference_offset_y)&&
      std::isfinite(c.low_speed_threshold)&&c.low_speed_threshold>=0.&&
      std::fabs(std::fabs(c.wheel_sign)-1.)<eps_&&
      std::fabs(std::fabs(c.steering_sign)-1.)<eps_&&
      std::fabs(std::fabs(c.gyro_sign)-1.)<eps_;
  }
  bool push(Buffer& b,double raw,double source,double receipt) {
    const char* reason=nullptr;
    if(!std::isfinite(raw))reason="nonfinite_raw_value";
    else if(!std::isfinite(source)||source<0.||!finitePositive(receipt))reason="invalid_sample_time";
    else if(source>receipt)reason="future_source_stamp";
    else if(receipt<=b.latest_receipt)reason="duplicate_or_out_of_order_receipt";
    else if(source>0.&&source<=b.latest_source)reason="duplicate_or_out_of_order_source";
    else if(source>0.&&receipt-source>config_.max_source_delay_s)reason="late_source_sample";
    if(reason){++b.rejected;b.rejection=reason;return false;}
    if(b.count==Capacity)++b.overwritten;else ++b.count;
    Sample& s=b.data[b.next];s.raw=raw;s.source=source;s.receipt=receipt;
    b.next=(b.next+1)%Capacity;b.latest_receipt=receipt;
    if(source>0.)b.latest_source=source;
    return true;
  }
  LocalRawStatus select(const Buffer& b,double t,double scale,double sign,
                        bool calibrated,const char* unverified,double& next)const {
    const Sample* selected=nullptr;
    for(std::size_t i=0;i<b.count;++i){const Sample& s=b.data[i];
      if(s.receipt>t){next=std::min(next,s.receipt);continue;}
      // No interpolation and no backdating of late arrivals to source time.
      if(s.source>t)continue;
      if(!selected||s.receipt>selected->receipt)selected=&s;
    }
    LocalRawStatus o;
    if(!selected){o.reason="no_causal_sample";return o;}
    o.raw=selected->raw;o.value=scale*sign*o.raw;
    o.source_stamp=selected->source;o.receipt_stamp=selected->receipt;
    o.source_time_present=o.source_stamp>0.;
    o.source_time_verified=o.source_time_present&&config_.source_timestamps_verified;
    const double stamp=o.source_time_present?o.source_stamp:o.receipt_stamp;
    if(!calibrated){o.reason=unverified;return o;}
    // Compare absolute deadlines: subtracting epoch-sized ROS timestamps can
    // round a sample just below its age limit even at the exact expiry time.
    const double expiry=std::min(stamp,o.receipt_stamp)+config_.max_sample_age_s;
    if(t>=expiry){o.reason="stale_sample";return o;}
    o.available=true;o.reason=o.source_time_verified?"verified_source_time":
      (o.source_time_present?"driver_source_time_unverified":"receipt_only_acquisition_unknown");
    if(expiry>t+eps_)next=std::min(next,expiry);
    return o;
  }
  void diagnostics(LocalTransportResult& out)const {
    out.rejected_samples=wheel_.rejected+steering_.rejected+imu_.rejected;
    out.overwritten_samples=wheel_.overwritten+steering_.overwritten+imu_.overwritten;
  }

public:
  explicit LocalMotionTransport(const LocalMotionConfig& c=LocalMotionConfig()) {configure(c);}
  bool configure(const LocalMotionConfig& c){config_=c;config_valid_=validConfig(c);reset();return config_valid_;}
  const LocalMotionConfig& config()const{return config_;}
  bool configValid()const{return config_valid_;}
  void reset(){wheel_=Buffer();steering_=Buffer();imu_=Buffer();last_control_=0.;}
  bool updateWheel(double raw,double source,double receipt){return push(wheel_,raw,source,receipt);}
  bool updateSteering(double raw,double source,double receipt){return push(steering_,raw,source,receipt);}
  bool updateImu(double raw_yaw_rate,double source,double receipt){return push(imu_,raw_yaw_rate,source,receipt);}
  const char* wheelRejection()const{return wheel_.rejection;}
  const char* steeringRejection()const{return steering_.rejection;}
  const char* imuRejection()const{return imu_.rejection;}

  LocalTransportResult relative(double reference_stamp,double control_stamp) {
    LocalTransportResult out;diagnostics(out);
    if(!config_valid_){out.reason="invalid_transport_configuration";return out;}
    if(!finitePositive(reference_stamp)||!finitePositive(control_stamp)){
      out.reason="invalid_transform_time";return out;}
    if(last_control_>0.&&control_stamp<last_control_){
      reset();out.reason="control_clock_regressed_requires_new_samples";return out;}
    last_control_=control_stamp;
    if(reference_stamp>control_stamp){out.reason="reference_in_future";return out;}
    out.duration=control_stamp-reference_stamp;
    if(out.duration>config_.history_seconds){out.reason="reference_outside_history_bound";return out;}
    if(out.duration>config_.max_transport_s){out.reason="transport_window_exceeded";return out;}
    if(out.duration==0.){
      out.valid=true;out.reason="identity_only_state_not_validated";
      out.gyro_measured_entire_window=false;out.source_timestamps_verified=false;return out;}
    double x=0.,y=0.,yaw=0.,t=reference_stamp;
    while(t<control_stamp-eps_){
      if(++out.steps>2000){out.reason="integration_step_bound";return out;}
      double next=std::min(control_stamp,t+config_.max_step_s);
      out.wheel=select(wheel_,t,config_.wheel_scale,config_.wheel_sign,
                      config_.wheel_calibrated,"wheel_units_unverified",next);
      out.steering=select(steering_,t,config_.steering_scale,config_.steering_sign,
                      config_.steering_calibrated,"steering_units_unverified",next);
      out.imu=select(imu_,t,config_.gyro_scale,config_.gyro_sign,
                      config_.imu_frame_verified,"imu_frame_unverified",next);
      if(!out.wheel.available){out.reason=out.wheel.reason;return out;}
      const double v=out.wheel.value;
      if(!std::isfinite(v)||std::fabs(v)>config_.max_abs_speed){out.reason="wheel_proxy_range";return out;}
      out.low_speed_observed=out.low_speed_observed||std::fabs(v)<config_.low_speed_threshold;
      double r=0.,sigma_r=config_.gyro_uncertainty_radps;
      if(out.imu.available){
        r=out.imu.value;
        if(!std::isfinite(r)||std::fabs(r)>config_.max_abs_yaw_rate){out.reason="gyro_range";return out;}
      }else{
        out.gyro_measured_entire_window=false;
        if(!config_.allow_bicycle_fallback){out.reason="gyro_unavailable_bicycle_fallback_disabled";return out;}
        if(!out.steering.available){out.reason="gyro_and_steering_unavailable";return out;}
        if(!std::isfinite(out.steering.value)||std::fabs(out.steering.value)>config_.max_abs_steering){
          out.reason="steering_range";return out;}
        r=v*std::tan(out.steering.value)/config_.wheelbase;
        if(!std::isfinite(r)||std::fabs(r)>config_.max_abs_yaw_rate){out.reason="bicycle_yaw_range";return out;}
        out.bicycle_fallback_used=true;sigma_r=config_.bicycle_uncertainty_radps;
      }
      out.source_timestamps_verified=out.source_timestamps_verified&&out.wheel.source_time_verified&&
        (out.imu.available?out.imu.source_time_verified:out.steering.source_time_verified);
      const double dt=next-t;
      if(!std::isfinite(dt)||dt<=eps_){out.reason="nonpositive_integration_interval";return out;}
      // Same midpoint SE(2) rule and <=20 ms substeps as S8 tracking transport.
      x+=v*std::cos(yaw+.5*r*dt)*dt;y+=v*std::sin(yaw+.5*r*dt)*dt;
      const double yaw_bound_mid=out.yaw_uncertainty_rad+.5*sigma_r*dt;
      out.position_uncertainty_m+=(config_.speed_uncertainty_mps+
        (std::fabs(v)+config_.speed_uncertainty_mps)*yaw_bound_mid)*dt;
      out.yaw_uncertainty_rad+=sigma_r*dt;
      yaw=std::atan2(std::sin(yaw+r*dt),std::cos(yaw+r*dt));t=next;
      // Include uncertain rotation of the configured rigid-body lever arm.
      const double lever=std::hypot(config_.reference_offset_x,config_.reference_offset_y);
      if(out.position_uncertainty_m+lever*out.yaw_uncertainty_rad>config_.max_position_uncertainty_m||
         out.yaw_uncertainty_rad>config_.max_yaw_uncertainty_rad){
        out.reason="configured_uncertainty_envelope_exceeded";return out;}
    }
    const double c=std::cos(yaw),s=std::sin(yaw);
    x+=c*config_.reference_offset_x-s*config_.reference_offset_y-config_.reference_offset_x;
    y+=s*config_.reference_offset_x+c*config_.reference_offset_y-config_.reference_offset_y;
    out.tx=-c*x-s*y;out.ty=s*x-c*y;out.yaw=-yaw;
    // Inverting the motion transform rotates its translation too. Account for
    // that angular uncertainty separately from forward integration and lever.
    out.position_uncertainty_m+=(std::hypot(config_.reference_offset_x,config_.reference_offset_y)+
      std::hypot(x,y))*out.yaw_uncertainty_rad;
    if(out.position_uncertainty_m>config_.max_position_uncertainty_m){
      out.reason="configured_uncertainty_envelope_exceeded";return out;}
    out.valid=std::isfinite(out.tx)&&std::isfinite(out.ty)&&std::isfinite(out.yaw);
    out.reason=out.valid?(out.bicycle_fallback_used?"wheel_bicycle_model_transport":
      "wheel_gyro_local_transport"):"nonfinite_relative_transform";
    return out;
  }
};
} // namespace research_control
