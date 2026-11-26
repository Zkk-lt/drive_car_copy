#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace research_control {
// Evidence about observed command snapshots, not CAN acquisition freshness.
// A missing or overwritten snapshot invalidates the delayed command history.
class ControlSnapshotContinuity {
public:
 struct Update { bool reset_history; const char* reason; };
 explicit ControlSnapshotContinuity(double delay=0.,double maximum_interval=.05)
   :delay_(delay),maximum_interval_(maximum_interval) {}
 // pending_lost means the displaced actual command could NOT be recorded.
 // A displaced snapshot retained without prediction is not a history loss.
 Update observe(std::uint32_t sequence,double stamp,bool pending_lost) {
   const char* failure=nullptr;
   if(!std::isfinite(stamp)||stamp<=0.)failure="invalid_control_stamp";
   else if(!seen_)failure="command_history_warming";
   else if(sequence!=std::uint32_t(last_sequence_+1u))failure="control_sequence_gap";
   else if(stamp<=last_stamp_||stamp-last_stamp_>maximum_interval_)failure="control_time_gap";
   if(pending_lost)failure="pending_control_snapshot_not_retained";
   if(failure)invalidate(stamp,failure);
   else ++consecutive_;
   seen_=std::isfinite(stamp)&&stamp>0.;last_stamp_=stamp;last_sequence_=sequence;
   return {failure!=nullptr,failure?failure:"continuous_control_snapshots"};
 }
 void invalidate(double stamp,const char* reason) {
   // A rejected older pending snapshot must not move a newer gap's recovery
   // boundary backwards. Clock reversal remains invalid until time catches up.
   if(std::isfinite(stamp))continuous_since_=std::max(continuous_since_,stamp);
   consecutive_=1;reason_=reason;
 }
 bool ready(double stamp)const {
   return seen_&&consecutive_>=2&&std::isfinite(delay_)&&delay_>=0.&&
     std::isfinite(stamp)&&std::isfinite(continuous_since_)&&
     stamp>=continuous_since_+delay_&&stamp<=last_stamp_+1e-9;
 }
 const char* reason()const{return reason_;}
 double remaining(double stamp)const {
   return std::max(0.,continuous_since_+delay_-stamp);
 }
private:
 double delay_,maximum_interval_,last_stamp_=0.,continuous_since_=0.;
 std::uint32_t last_sequence_=0;unsigned consecutive_=0;bool seen_=false;
 const char* reason_="command_history_warming";
};
}
