#include "control/control_snapshot_continuity.h"
#include <cassert>
#include <iostream>
#include <limits>

int main(){
 using research_control::ControlSnapshotContinuity;
 ControlSnapshotContinuity c(.325);
 assert(c.observe(10,1.,false).reset_history);assert(!c.ready(1.));
 for(unsigned i=1;i<=10;++i){assert(!c.observe(10+i,1.+i/30.,false).reset_history);if(i<10)assert(!c.ready(1.+i/30.));}
 assert(c.ready(1.+10./30.));
 // A single missed 30 Hz snapshot must invalidate a previously warm history.
 auto lost=c.observe(22,1.+12./30.,false);assert(lost.reset_history);assert(!c.ready(1.+12./30.));
 for(unsigned i=1;i<=10;++i)c.observe(22+i,1.+(12+i)/30.,false);
 assert(c.ready(1.+22./30.));
 c.invalidate(1.5,"older_pending_rejected");assert(!c.ready(1.5));
 // Sequence continuity alone cannot establish temporal continuity.
 assert(c.observe(33,2.,false).reset_history);assert(!c.ready(2.));
 // A snapshot displaced without retaining its command invalidates history.
 assert(c.observe(34,2.+1./30.,true).reset_history);assert(!c.ready(2.+1./30.));
 for(unsigned i=1;i<=10;++i)c.observe(34+i,2.+(1+i)/30.,false);
 assert(c.ready(2.+11./30.));
 assert(!c.ready(2.));assert(!c.ready(3.));
 assert(c.observe(45,2.,false).reset_history);assert(!c.ready(2.));
 assert(c.observe(46,std::numeric_limits<double>::quiet_NaN(),false).reset_history);
 assert(!c.ready(3.));
 ControlSnapshotContinuity wrap(0.);
 assert(wrap.observe(std::numeric_limits<std::uint32_t>::max(),4.,false).reset_history);
 assert(!wrap.observe(0,4.+1./30.,false).reset_history);assert(wrap.ready(4.+1./30.));
 wrap.invalidate(4.+1./30.,"nonfinite_actual_command");assert(!wrap.ready(4.+1./30.));
 ControlSnapshotContinuity older(.325);
 older.observe(1,10.,false);older.observe(9,10.4,false);
 older.invalidate(10.1,"older_pending_rejected");
 for(unsigned i=1;i<=9;++i){older.observe(9+i,10.4+i/30.,false);assert(!older.ready(10.4+i/30.));}
 older.observe(19,10.4+10./30.,false);assert(older.ready(10.4+10./30.));
 // Recording each displaced actual command without predicting it preserves
 // the continuous delayed history. Only a genuinely rejected/dropped command
 // passes pending_lost=true (stale/nonfinite checks live in the ROS adapter).
 ControlSnapshotContinuity retained(.325);
 retained.observe(1,20.,false);
 for(unsigned i=1;i<=10;++i)assert(!retained.observe(1+i,20.+i/30.,false).reset_history);
 assert(retained.ready(20.+10./30.));
 assert(retained.observe(12,20.+11./30.,true).reset_history);
 assert(!retained.ready(20.+11./30.));
 std::cout<<"PASS command snapshot sequence/time/drop detection, delay-window recovery and rollover\n";
}
