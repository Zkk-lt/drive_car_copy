#pragma once
#include <algorithm>
#include <cmath>
#include <string>

namespace research_control {
// The S8 base cruise/catchup speed is an objective. Every curve, tracking-error,
// actuator, following, reference-horizon and geometry-rate cap remains hard. This adapter
// never alters a physical limit or permits an initial state above a hard cap.
struct P7SpeedPolicy {
  double target=0.,hard_cap=0.;
  bool separated=false;
  const char* reason="legacy_speed_contract";
  static P7SpeedPolicy make(bool enabled,double planned,double physical,
      double scene,double following,double horizon,double curve,double cte,double actuator,
      bool rate_matched,double rate,
      const std::string& winner,bool protected_stop,bool reference_usable,
      bool braking_reference,bool bad_geometry,bool preview_target_only=false) {
    P7SpeedPolicy p;p.target=p.hard_cap=std::max(0.,std::min(physical,planned));
    if(!enabled||protected_stop||!reference_usable||braking_reference||bad_geometry)return p;
    // The independently validated NMPC may reproduce the gradual reduction
    // toward the S8 preview failure's target. All physical/safety caps still
    // enter the same intersection below and apply immediately.
    const bool preview_fallback=preview_target_only&&winner=="slqr_sequence_fallback";
    if(winner!="base"&&winner!="curve"&&winner!="cte"&&winner!="actuator"&&!preview_fallback)return p;
    for(double v:{planned,physical,scene,following,horizon,curve,cte,actuator})
      if(!std::isfinite(v)||v<0.){p.reason="speed_policy_invalid_input";return p;}
    if(rate_matched&&(!std::isfinite(rate)||rate<0.)){
      p.reason="speed_policy_invalid_geometry_rate";return p;
    }
    p.hard_cap=std::min({physical,scene,following,horizon,curve,cte,actuator});
    // Same 0.98 factor used by S8 ReferenceRateConstraint::limitSpeed.
    if(rate_matched)p.hard_cap=std::min(p.hard_cap,.98*rate);
    p.target=std::min(planned,p.hard_cap);
    p.separated=true;p.reason="performance_target_with_unchanged_safety_caps";
    return p;
  }
};
} // namespace research_control
