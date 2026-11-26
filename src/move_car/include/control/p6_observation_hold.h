#pragma once
#include <algorithm>
#include <cmath>
namespace research_control {
// A feedback innovation is new relative to the FROZEN STATE observation,
// not to the later request-serialization stamp. S8 computation can take
// several milliseconds; a sample received in that interval is still a new,
// causal measurement. Every source/receipt/skew/propagation gate remains.
inline const char* p6FeedbackTimingRejection(double observed,double snapshot,double old_source,
 double wheel_receipt,double imu_receipt,double imu_source,double now,double apply){
 for(double t:{observed,snapshot,old_source,wheel_receipt,imu_receipt,imu_source,now,apply})
  if(!std::isfinite(t)||t<=0.)return "feedback_sample_missing";
 if(observed>snapshot||snapshot>now)return "feedback_observation_interval_invalid";
 if(imu_source<=old_source)return "feedback_no_new_imu_source";
 const double cutoff=std::max(wheel_receipt,imu_receipt);
 if(imu_source>imu_receipt||imu_source>now||cutoff>now)return "feedback_future_source_or_receipt";
 if(cutoff<=observed)return "feedback_no_new_receipt";
 if(cutoff>apply)return "feedback_receipt_after_apply";
 if(std::abs(wheel_receipt-imu_receipt)>.06)return "feedback_sensor_receipt_skew";
 if(cutoff-observed>.075)return "feedback_observation_interval_invalid";
 return nullptr;
}
// Controller and worker receipts belong to DIFFERENT ROS subscribers. They
// must be causal/fresh, but cannot be required equal. IMU source time is the
// shared sample identity; CAN has no hardware identity in the present driver.
inline bool p6NominalHoldTiming(double frozen_wheel,double frozen_imu,double source,
 double wheel_receipt,double imu_receipt,double imu_source,double snapshot,double now,double apply){
 for(double t:{frozen_wheel,frozen_imu,source,wheel_receipt,imu_receipt,imu_source,snapshot,now,apply})
  if(!std::isfinite(t)||t<=0.)return false;
 return imu_source==source&&source<=frozen_imu&&imu_source<=imu_receipt&&
   frozen_wheel<=snapshot&&frozen_imu<=snapshot&&wheel_receipt<=snapshot&&
   std::max(wheel_receipt,imu_receipt)<=now&&snapshot<=now&&now<=apply&&
   now-std::min({frozen_wheel,frozen_imu,source,wheel_receipt})<=.12;
}

// CAN has no source clock in this driver. This qualifies only a new local
// receipt after the frozen cutoff, with a concurrently available held IMU.
// Identical IMU source identities must not create a second yaw innovation.
inline bool p6CanOnlyTiming(double observed,double snapshot,double frozen_imu_receipt,
 double frozen_imu_source,double wheel_receipt,double qualified_steer_receipt,
 double imu_receipt,double imu_source,double now,double apply){
 for(double t:{observed,snapshot,frozen_imu_receipt,frozen_imu_source,wheel_receipt,
               qualified_steer_receipt,imu_receipt,imu_source,now,apply})
  if(!std::isfinite(t)||t<=0.)return false;
 return observed<=snapshot&&snapshot<=now&&now<=apply&&apply-observed<=.075&&
   wheel_receipt>observed&&wheel_receipt<=now&&qualified_steer_receipt==wheel_receipt&&
   imu_source==frozen_imu_source&&imu_source<=frozen_imu_receipt&&frozen_imu_receipt<=observed&&
   imu_source<=imu_receipt&&imu_receipt<=now&&now-imu_source<=.12&&
   std::abs(wheel_receipt-imu_receipt)<=.06;
}
}
