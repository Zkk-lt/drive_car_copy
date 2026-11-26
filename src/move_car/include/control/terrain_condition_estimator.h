#pragma once
#include "research_types.h"

namespace research_control {
struct TerrainEstimatorConfig {
 bool enabled=true;
 double rough_enter=.80,rough_exit=.45,dwell_seconds=1.,max_age=.3,min_speed=.10;
};

// A deliberately conservative feature classifier. A high vibration score is
// compatible with grass_bumpy, but not proof of material or tire friction.
// Smooth grass and smooth asphalt cannot be distinguished by these channels;
// their mass remains ambiguous and the operational output is unknown.
class TerrainConditionEstimator {
public:
 explicit TerrainConditionEstimator(const TerrainEstimatorConfig& c=TerrainEstimatorConfig()) {configure(c);}
 void configure(const TerrainEstimatorConfig& c) {config_=c;reset();}
 void reset() {last_now_=candidate_since_=0.;candidate_=label_="unknown";}
 TerrainEstimate update(const Observation& o,double now) {
   TerrainEstimate out;out.stamp=now;
   if(!config_.enabled) {out.reason="terrain_estimation_disabled";reset();return out;}
   if(!std::isfinite(now)||now<=0. || !std::isfinite(config_.rough_enter)||
      !std::isfinite(config_.rough_exit)||!std::isfinite(config_.dwell_seconds)||
      !std::isfinite(config_.max_age)||!std::isfinite(config_.min_speed)||
      config_.rough_enter<=config_.rough_exit||config_.rough_exit<0.||
      config_.dwell_seconds<=0.||config_.max_age<=0.||config_.min_speed<=0.) {
     out.reason="invalid_terrain_config_or_clock";reset();return out;
   }
   if(last_now_>0.&&now<last_now_) {out.reason="clock_regressed";reset();return out;}
   last_now_=now;
   if(!fresh(o.roughness,now)||!fresh(o.speed,now)||std::fabs(o.speed.value)<config_.min_speed) {
     out.reason="terrain_unobservable_or_low_speed";label_=candidate_="unknown";candidate_since_=now;return out;
   }
   const double feature_confidence=std::min(o.speed.confidence,o.roughness.confidence);
   if(feature_confidence<.25) {
     out.reason="terrain_feature_confidence_low";label_=candidate_="unknown";candidate_since_=now;return out;
   }
   const bool rough=o.roughness.value>=(label_=="grass_bumpy"?config_.rough_exit:config_.rough_enter);
   const std::string desired=rough?"grass_bumpy":"unknown";
   if(desired!=candidate_) {candidate_=desired;candidate_since_=now;}
   if(candidate_!=label_ && now-candidate_since_>=config_.dwell_seconds)label_=candidate_;
   const double rough_score=clamp((o.roughness.value-config_.rough_exit)/
                                (config_.rough_enter-config_.rough_exit),0.,1.);
   const double bumpy=.05+.65*rough_score,unknown=.45-.20*rough_score;
   out.probabilities={{(1.-bumpy-unknown)/2.,bumpy,(1.-bumpy-unknown)/2.,unknown}};
   out.label=label_;out.confidence=label_=="grass_bumpy"?.65*feature_confidence:0.;
   out.valid=label_=="grass_bumpy";
   out.reason=label_=="grass_bumpy"?"roughness_heuristic_uncalibrated_not_friction":
      (candidate_=="grass_bumpy"?"roughness_candidate_waiting_dwell":"smooth_materials_unidentifiable");
   return out;
 }
private:
 TerrainEstimatorConfig config_;double last_now_=0.,candidate_since_=0.;
 std::string label_="unknown",candidate_="unknown";
 bool fresh(const Estimate& e,double now) const {
   return e.valid&&std::isfinite(e.value)&&std::isfinite(e.confidence)&&
       e.confidence>0.&&e.stamp<=now+.02&&now-e.stamp<=config_.max_age;
 }
};
} // namespace research_control
