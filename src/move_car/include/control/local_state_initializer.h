#pragma once
#include "unified_control.h"
namespace unified_control {
struct LocalSignal{double value=0.,receipt_stamp=0.,source_stamp=0.;bool valid=false,source_stamp_known=false,hardware_sample_time_verified=false;};
struct LocalInitializationInput{double now=0.;LocalSignal wheel,steering,imu_yaw;bool reference_local_frame_verified=false;};
struct LocalInitializationContract{double wheel_scale=.001,steering_scale=1.,imu_yaw_sign=1.,maximum_age=.12,maximum_skew=.06;bool wheel_units_verified=false,steering_units_verified=false,imu_body_frame_verified=false;};
struct LocalInitializationResult{LocalControlState state;const char*reason="unverified_contract";};
class LocalStateInitializer{public:static LocalInitializationResult initialize(const LocalInitializationInput&i,const LocalInitializationContract&c){LocalInitializationResult r;auto&s=r.state;s.stamp=i.now;
 if(!c.wheel_units_verified||!c.steering_units_verified||!c.imu_body_frame_verified||!i.reference_local_frame_verified)return r;
 if(!std::isfinite(i.now)||!std::isfinite(c.wheel_scale)||c.wheel_scale==0.||!std::isfinite(c.steering_scale)||c.steering_scale==0.||!std::isfinite(c.imu_yaw_sign)||std::fabs(c.imu_yaw_sign)!=1.||!std::isfinite(c.maximum_age)||c.maximum_age<=0.||!std::isfinite(c.maximum_skew)||c.maximum_skew<0.){r.reason="invalid_contract";return r;}
 double oldest=i.now,newest=0.;for(const auto*p:{&i.wheel,&i.steering,&i.imu_yaw}){if(!p->valid||!std::isfinite(p->value)||!std::isfinite(p->receipt_stamp)||p->receipt_stamp<=0.||p->receipt_stamp>i.now||i.now-p->receipt_stamp>c.maximum_age){r.reason="invalid_stale_or_future_receipt";return r;}if(p->source_stamp_known&&(!std::isfinite(p->source_stamp)||p->source_stamp<=0.||p->source_stamp>p->receipt_stamp||i.now-p->source_stamp>c.maximum_age)){r.reason="invalid_stale_or_future_source";return r;}oldest=std::min(oldest,p->receipt_stamp);newest=std::max(newest,p->receipt_stamp);}
 if(newest-oldest>c.maximum_skew){r.reason="sensor_receipt_skew";return r;}
 s.speed=i.wheel.value*c.wheel_scale;s.steering=i.steering.value*c.steering_scale;s.yaw_rate=i.imu_yaw.value*c.imu_yaw_sign;
 if(std::fabs(s.speed)>2.||std::fabs(s.steering)>.6||std::fabs(s.yaw_rate)>1.5){r.reason="nominal_support_exceeded";return r;}
 s.receipt_stamp=newest;s.maximum_source_age=i.now-oldest;s.causal=true;s.valid=true;s.nominal_model_valid=true;s.yaw_rate_measured=true;s.confidence=.35;r.reason="receipt_proxy_nominal_only";return r;}};
}
