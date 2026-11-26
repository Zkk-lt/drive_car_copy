#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace research_control {
// The legacy carTop stream emitted isolated zero-battery motion reports with
// impossible steering steps. Zero alone does not invalidate motion. Keep
// chassis fault handling independent of this motion qualification.
inline bool discontinuousZeroBatteryMotion(int battery,double steering,double receipt,
    double accepted,double accepted_stamp,double maximum_rate) {
  if(battery!=0||!std::isfinite(steering)||!std::isfinite(receipt)||
     !std::isfinite(accepted)||!std::isfinite(accepted_stamp)||accepted_stamp<=0.||
     !std::isfinite(maximum_rate)||maximum_rate<=0.)return false;
  const double elapsed=receipt-accepted_stamp;
  return elapsed>=0.&&elapsed<=.12&&std::abs(steering-accepted)>.02+maximum_rate*elapsed;
}
// Receipt-domain qualification of one raw steering channel.  This is the
// existing S8 pulse-confirmation rule, without S8's subsequent low-pass filter.
// An unconfirmed pulse never changes the accepted value OR its receipt time.
// It is not a hardware timestamp, frame counter, or actuator identification.
struct QualifiedSteeringSample {
  double raw=0.,accepted=0.,pending=0.;
  double accepted_stamp=0.,seen_stamp=0.,pending_stamp=0.;
  std::uint64_t deferred_count=0;
  bool initialized=false,pending_valid=false,deferred=false;
  double update(double value,double stamp,double maximum_rate) {
    raw=value;deferred=false;
    if(!std::isfinite(value)||!std::isfinite(stamp)||stamp<=0.) {
      deferred=true;return accepted;
    }
    auto accept=[&]() {
      initialized=true;accepted=value;accepted_stamp=seen_stamp=stamp;
      pending_valid=false;return accepted;
    };
    // Backwards receipt time is invalid, even after a long gap.  S8 stays
    // untouched; this stricter research contract never accepts future data.
    if(initialized&&stamp<seen_stamp) {deferred=true;return accepted;}
    if(!initialized||stamp-seen_stamp>.20)return accept();
    seen_stamp=stamp;
    const double rate=std::max(.05,maximum_rate);
    const double allowance=.02+rate*std::min(.10,stamp-accepted_stamp);
    if(std::abs(value-accepted)<=allowance)return accept();
    const double dt=pending_valid?stamp-pending_stamp:0.;
    const bool consistent=pending_valid&&
      std::abs(value-pending)<=.02+rate*std::min(.10,dt);
    if(consistent&&dt>=.04)return accept();
    if(!consistent) {pending=value;pending_stamp=stamp;pending_valid=true;++deferred_count;}
    deferred=true;return accepted;
  }
  bool usable(double now,double maximum_age=.12) const {
    return initialized&&std::isfinite(now)&&accepted_stamp<=now&&
      now-accepted_stamp<=maximum_age;
  }
  bool discontinuousZeroBatteryMotion(int battery,double steering,double receipt,double maximum_rate) const {
    return initialized&&research_control::discontinuousZeroBatteryMotion(
        battery,steering,receipt,accepted,accepted_stamp,maximum_rate);
  }
};
} // namespace research_control
