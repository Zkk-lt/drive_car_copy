#pragma once
#include "research_types.h"
#include "p7_supervised_test.h"
#include <array>
#include <cstdint>
#include <mutex>
namespace unified_control {
using research_control::Command;using research_control::Limits;
enum class ControlMode{LegacySlqr,NmpcShadow,NmpcFixed,NmpcAdaptive,AutoFallback};
inline ControlMode parseMode(const std::string&s){if(s=="nmpc_shadow")return ControlMode::NmpcShadow;
 if(s=="nmpc_fixed")return ControlMode::NmpcFixed;
 if(s=="nmpc_adaptive")return ControlMode::NmpcAdaptive;
 if(s=="auto_fallback")return ControlMode::AutoFallback;
 return ControlMode::LegacySlqr;}
inline const char* modeName(ControlMode m){switch(m){case ControlMode::LegacySlqr:return "legacy_slqr";case ControlMode::NmpcShadow:return "nmpc_shadow";case ControlMode::NmpcFixed:return "nmpc_fixed";case ControlMode::NmpcAdaptive:return "nmpc_adaptive";case ControlMode::AutoFallback:return "auto_fallback";}return "legacy_slqr";}
struct LocalControlState{
 double stamp=0.,receipt_stamp=0.,x=0.,y=0.,yaw=0.,speed=0.,steering=0.,yaw_rate=0.,acceleration=0.,confidence=0.,maximum_source_age=0.;
 bool valid=false,causal=false,nominal_model_valid=false,physical_calibrated=false,slip_valid=false,speed_is_ground_measurement=false,yaw_rate_measured=false,acceleration_observed=false;
};
struct ControlReference{double stamp=0.;std::uint64_t sequence=0;bool valid=false,braking=false;};
struct SimulationGuard{bool isolated_test=false,allow_test_nmpc=false;std::string command_topic,run_id;
 bool permits()const{return isolated_test&&allow_test_nmpc&&run_id.compare(0,3,"p5_")==0&&command_topic.compare(0,26,"/control_research/p5_test/")==0&&command_topic.size()>26;}};
inline bool finiteCommand(const Command& c){return std::isfinite(c.speed)&&std::isfinite(c.steering);}
inline bool validLimits(const Limits& l){return std::isfinite(l.speed)&&l.speed>=0.&&std::isfinite(l.steering)&&l.steering>0.&&std::isfinite(l.steering_rate)&&l.steering_rate>0.&&std::isfinite(l.acceleration)&&l.acceleration>0.&&std::isfinite(l.deceleration)&&l.deceleration>0.&&std::isfinite(l.jerk)&&l.jerk>0.&&std::isfinite(l.lateral_acceleration)&&l.lateral_acceleration>0.;}
inline Limits intersectLimits(const Limits&a,const Limits&b){Limits l;l.speed=std::min(a.speed,b.speed);l.steering=std::min(a.steering,b.steering);l.steering_rate=std::min(a.steering_rate,b.steering_rate);l.acceleration=std::min(a.acceleration,b.acceleration);l.deceleration=std::min(a.deceleration,b.deceleration);l.jerk=std::min(a.jerk,b.jerk);l.lateral_acceleration=std::min(a.lateral_acceleration,b.lateral_acceleration);
 return l;}
struct ControlRequest{
 double target_speed=research_control::missing();
 bool speed_target_separated=false;
 ControlMode mode=ControlMode::LegacySlqr;LocalControlState state;ControlReference reference;Command legacy,previous;Limits limits;SimulationGuard guard;SupervisedGuard supervised;double steady_now=0.;
 double now=0.,dt=1./30.,speed_cap=.3,previous_acceleration=0.,maximum_candidate_age=.075,maximum_reference_age=1.2,maximum_state_age=.12,maximum_apply_time_error=.05,maximum_solve_ms=8.;
 double engineering_state_speed_tolerance=.12,engineering_state_steering_tolerance=.12,engineering_state_yaw_rate_tolerance=.30;
 std::uint64_t epoch=0,config_version=0,history_sequence=0;bool history_valid=false,safety_stop=false,following_stop=false,adaptive_gate_enabled=false;
};
struct ControlCandidate{
 Command command;double command_acceleration=0.,solve_ms=0.,max_violation=0.,created_stamp=0.,snapshot_stamp=0.,expected_apply_stamp=0.,propagated_speed=0.,propagated_steering=0.,propagated_yaw_rate=0.;
 std::uint64_t sequence=0,epoch=0,reference_sequence=0,config_version=0,history_base_sequence=0;
 bool valid=false,feasible=false,timed_out=false,numerical_failure=false,complete_substep_constraints=false,state_pair_validated=false,model_adaptation_active=false,weight_adaptation_active=false;
 // The original optimized identity above is never overwritten by revalidation.
 bool reference_revalidated=false;std::uint64_t validated_reference_sequence=0;
 double validation_stamp=0.,revalidation_ms=0.;
};
struct ControlDecision{Command command;bool selected_nmpc=false,fallback=false,safety_stop=false,legacy_contract_warning=false,terminal_handoff=false;
 const char*reason="legacy_default";std::uint64_t candidate_sequence=0,base_sequence=0,epoch=0;
 bool bounded_prefix_following=false;};
class ControlArbiter{
 static ControlDecision reject(ControlDecision d,const char*r){d.fallback=true;d.reason=r;
 return d;}
public: static ControlDecision decide(const ControlRequest&r,const ControlCandidate*c=nullptr){
 ControlDecision d;d.command=r.legacy;d.base_sequence=r.history_sequence;d.epoch=r.epoch;
 if(r.supervised.enabled){
  d.command.speed=r.supervised.boundedSpeed(d.command.speed);
  if(!r.supervised.armed(r.steady_now)){if(!finiteCommand(d.command))d.command=Command{};d.command.speed=0.;d.safety_stop=true;d.reason="supervised_operator_lease_required";return d;}
 }
 if(!finiteCommand(r.legacy)){d.command=Command{};d.safety_stop=true;d.legacy_contract_warning=true;d.reason="legacy_nonfinite_stop";
 return d;}
 if(r.safety_stop||r.following_stop||r.reference.braking){d.safety_stop=true;d.reason="existing_safety_priority";
 if(r.safety_stop||r.following_stop){d.legacy_contract_warning=r.legacy.speed!=0.;d.command.speed=0.;}return d;}
 if(r.mode==ControlMode::LegacySlqr){d.reason="legacy_slqr";
 return d;}if(r.mode==ControlMode::NmpcShadow){d.reason="shadow_has_no_authority";
 return d;}
 if(!r.guard.permits()&&!r.supervised.armed(r.steady_now))return reject(d,"isolated_simulation_authority_required");
 if(r.supervised.enabled&&(r.mode==ControlMode::NmpcAdaptive||r.adaptive_gate_enabled||(c&&(c->model_adaptation_active||c->weight_adaptation_active))))return reject(d,"supervised_online_adaptation_forbidden");
 if(!r.reference.valid||!std::isfinite(r.reference.stamp)||r.reference.stamp>r.now||r.now-r.reference.stamp>r.maximum_reference_age)return reject(d,"reference_invalid_or_stale");
 const auto&s=r.state;
 if(!s.valid||!s.causal||!s.nominal_model_valid||!std::isfinite(s.stamp)||s.stamp>r.now||r.now-s.stamp>r.maximum_state_age||!finiteCommand({s.speed,s.steering})||!std::isfinite(s.yaw_rate)||!std::isfinite(s.confidence)||s.confidence<=0.||s.confidence>1.||!std::isfinite(s.receipt_stamp)||s.receipt_stamp>r.now||r.now-s.receipt_stamp>r.maximum_state_age||!std::isfinite(s.maximum_source_age)||s.maximum_source_age<0.||s.maximum_source_age>r.maximum_state_age)return reject(d,"local_state_invalid_or_noncausal");
 for(double value:{s.x,s.y,s.yaw,s.acceleration})if(!std::isfinite(value))return reject(d,"local_state_nonfinite");
 if(!r.history_valid||!finiteCommand(r.previous)||!std::isfinite(r.previous_acceleration))return reject(d,"actual_history_invalid");
 if(!c)return reject(d,"candidate_missing");
 const bool revalidated=c->reference_revalidated&&c->validated_reference_sequence==r.reference.sequence&&
   c->validation_stamp==r.now&&std::isfinite(c->revalidation_ms)&&c->revalidation_ms>=0.;
 if(c->sequence==0||c->epoch!=r.epoch||(!revalidated&&c->reference_sequence!=r.reference.sequence)||c->config_version!=r.config_version||c->history_base_sequence!=r.history_sequence)return reject(d,"candidate_context_mismatch");
 if(!std::isfinite(r.now)||!std::isfinite(r.dt)||r.dt<=0.||r.dt>.25||!std::isfinite(r.maximum_candidate_age)||r.maximum_candidate_age<=0.||!std::isfinite(r.maximum_apply_time_error)||r.maximum_apply_time_error<=0.||!std::isfinite(r.maximum_solve_ms)||r.maximum_solve_ms<=0.||!std::isfinite(r.maximum_state_age)||r.maximum_state_age<=0.||!std::isfinite(r.maximum_reference_age)||r.maximum_reference_age<=0.||!std::isfinite(r.engineering_state_speed_tolerance)||r.engineering_state_speed_tolerance<0.||!std::isfinite(r.engineering_state_steering_tolerance)||r.engineering_state_steering_tolerance<0.||!std::isfinite(r.engineering_state_yaw_rate_tolerance)||r.engineering_state_yaw_rate_tolerance<0.)return reject(d,"request_contract_invalid");
 if(!std::isfinite(c->snapshot_stamp)||!std::isfinite(c->created_stamp)||!std::isfinite(c->expected_apply_stamp)||c->snapshot_stamp>c->created_stamp||c->created_stamp>r.now||c->snapshot_stamp>r.now||r.reference.stamp>(revalidated?c->validation_stamp:c->snapshot_stamp)||c->expected_apply_stamp<c->snapshot_stamp||r.now-c->snapshot_stamp>r.maximum_candidate_age||std::fabs(c->expected_apply_stamp-r.now)>r.maximum_apply_time_error)return reject(d,"candidate_stale_or_noncausal");
 if(c->timed_out||!std::isfinite(c->solve_ms)||c->solve_ms<0.||c->solve_ms>=r.maximum_solve_ms)return reject(d,"candidate_solver_timeout");
 if(!c->valid||!c->feasible||c->numerical_failure||!finiteCommand(c->command)||!std::isfinite(c->command_acceleration)||!std::isfinite(c->max_violation)||c->max_violation<0.||c->max_violation>1e-7)return reject(d,"candidate_invalid_or_infeasible");
 if(!c->complete_substep_constraints)return reject(d,"prediction_constraints_unverified");
 if(!c->state_pair_validated||!finiteCommand({c->propagated_speed,c->propagated_steering})||!std::isfinite(c->propagated_yaw_rate)||std::fabs(c->propagated_speed-s.speed)>r.engineering_state_speed_tolerance||std::fabs(c->propagated_steering-s.steering)>r.engineering_state_steering_tolerance||std::fabs(c->propagated_yaw_rate-s.yaw_rate)>r.engineering_state_yaw_rate_tolerance)return reject(d,"candidate_initial_state_inconsistent");
 if(r.mode==ControlMode::NmpcAdaptive&&!r.adaptive_gate_enabled)return reject(d,"adaptive_gate_disabled");
 if(!r.adaptive_gate_enabled&&(c->model_adaptation_active||c->weight_adaptation_active))return reject(d,"untrusted_adaptation_active");
 if(!validLimits(r.limits)||!std::isfinite(r.speed_cap)||r.speed_cap<0.)return reject(d,"immutable_safety_limits_invalid");
 constexpr double eps=1e-7;
 const double a=(c->command.speed-r.previous.speed)/r.dt;
 if(c->command.speed<0.||(r.supervised.enabled&&c->command.speed>r.supervised.speed_cap+eps)||c->command.speed>std::min(r.speed_cap,r.limits.speed)+eps||std::fabs(c->command.steering)>r.limits.steering+eps)return reject(d,"candidate_absolute_constraint_violation");
 if(std::fabs(s.speed*s.yaw_rate)>r.limits.lateral_acceleration+eps)return reject(d,"measured_lateral_envelope_exceeded");
 if(a>r.limits.acceleration+eps||a<-r.limits.deceleration-eps||std::fabs(c->command.steering-r.previous.steering)>r.limits.steering_rate*r.dt+eps||std::fabs(a-r.previous_acceleration)>r.limits.jerk*r.dt+eps||std::fabs(a-c->command_acceleration)>eps)return reject(d,"candidate_transition_constraint_violation");
 d.command=c->command;d.selected_nmpc=true;d.reason=r.supervised.enabled?"supervised_nmpc_selected":"isolated_nmpc_selected";d.candidate_sequence=c->sequence;
 return d;
 }};
struct PublishedCommandRecord{
 std::uint64_t sequence=0,epoch=0,candidate_sequence=0;double stamp=0.,cycle_stamp=0.,publish_ros_start=0.,publish_ros_end=0.;std::uint64_t publish_steady_start_ns=0,publish_steady_end_ns=0;Command command;double acceleration=0.;bool ros_publish_returned=false,physical_execution_confirmed=false,selected_nmpc=false;
};
struct CommitResult{bool accepted=false;
 const char*reason="not_committed";};
class CommandLedger{
 static constexpr std::size_t capacity_=256;std::array<PublishedCommandRecord,capacity_>records_{};std::size_t size_=0,head_=0;std::uint64_t epoch_=0,last_sequence_=0,rejections_=0;bool valid_=false;double maximum_gap_=.12;
 CommitResult fail(const char*r){valid_=false;++rejections_;
 return {false,r};}
public: explicit CommandLedger(std::uint64_t epoch=0,double maximum_gap=.12):epoch_(epoch),maximum_gap_(maximum_gap){}
 std::uint64_t nextSequence()const{return last_sequence_+1;}std::uint64_t lastSequence()const{return last_sequence_;}std::uint64_t rejectionCount()const{return rejections_;}bool valid()const{return valid_;}std::size_t size()const{return size_;}
 const PublishedCommandRecord*latest()const{return size_?&records_[(head_+capacity_-1)%capacity_]:nullptr;}
 const PublishedCommandRecord*preceding()const{return size_>=2?&records_[(head_+capacity_-2)%capacity_]:nullptr;}
 CommitResult commit(const PublishedCommandRecord&r){
 if(!r.ros_publish_returned)return fail("publish_did_not_return");
 if(r.epoch!=epoch_)return fail("publication_epoch_mismatch");
 if(r.sequence<=last_sequence_)return fail("duplicate_or_out_of_order_sequence");
 if(r.sequence!=nextSequence())return fail("publication_sequence_gap");
 if(!std::isfinite(maximum_gap_)||maximum_gap_<=0.)return fail("history_gap_contract_invalid");
 if(!finiteCommand(r.command)||!std::isfinite(r.acceleration)||!std::isfinite(r.cycle_stamp)||!std::isfinite(r.stamp)||!std::isfinite(r.publish_ros_start)||!std::isfinite(r.publish_ros_end)||r.cycle_stamp>r.publish_ros_start||r.publish_ros_end<r.publish_ros_start||r.stamp<r.publish_ros_start||r.stamp>r.publish_ros_end||r.publish_steady_start_ns==0||r.publish_steady_end_ns<r.publish_steady_start_ns||r.physical_execution_confirmed)return fail("publication_boundary_invalid");
 const auto*b=latest();
 if(b&&(r.cycle_stamp<=b->cycle_stamp||r.publish_steady_start_ns<b->publish_steady_end_ns))return fail("publication_time_out_of_order");
 const bool gap=b&&r.cycle_stamp-b->cycle_stamp>maximum_gap_;
 records_[head_]=r;head_=(head_+1)%capacity_;
 if(size_<capacity_)++size_;
 last_sequence_=r.sequence;
 if(size_==1&&rejections_==0&&!gap)valid_=true;
 if(gap)valid_=false;
 return {true,gap?"published_recorded_history_gap":"published_recorded_once"};}
 bool certifyRecovery(std::uint64_t sequence,double from,double horizon){if(!size_||sequence!=last_sequence_||!std::isfinite(from)||!std::isfinite(horizon)||horizon<0.)return false;
 const auto*last=latest();double first=last->cycle_stamp;for(std::size_t i=1;i<size_;++i){const auto&n=records_[(head_+capacity_-i)%capacity_];
 const auto&o=records_[(head_+capacity_-i-1)%capacity_];
 if(n.sequence!=o.sequence+1||n.cycle_stamp-o.cycle_stamp>maximum_gap_)break;
 first=o.cycle_stamp;}if(first>from||last->cycle_stamp-from<horizon)return false;
 valid_=true;
 return true;}
 std::size_t copyHistory(std::array<PublishedCommandRecord,256>&out)const{for(std::size_t i=0;i<size_;++i)out[i]=records_[(head_+capacity_-size_+i)%capacity_];
 return size_;}
};
class LatestCandidateMailbox{std::mutex mutex_;ControlCandidate candidate_;bool present_=false;public:
 bool tryPublish(const ControlCandidate&c){std::unique_lock<std::mutex>l(mutex_,std::try_to_lock);
 if(!l.owns_lock())return false;
 candidate_=c;present_=true;
 return true;}
 bool tryRead(ControlCandidate&c){std::unique_lock<std::mutex>l(mutex_,std::try_to_lock);
 if(!l.owns_lock()||!present_)return false;
 c=candidate_;
 return true;}
 template<class F>void withLockedSlotForTest(F f){std::lock_guard<std::mutex>l(mutex_);f();}
};
}
