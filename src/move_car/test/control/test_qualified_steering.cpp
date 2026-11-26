#include "control/qualified_steering_sample.h"
#include <cassert>
#include <iostream>
#include <limits>
using research_control::QualifiedSteeringSample;
int main(int argc,char**argv){
 if(argc>1){double t,v,rate;QualifiedSteeringSample q;std::cout.precision(17);
  while(std::cin>>t>>v>>rate){q.update(v,t,rate);std::cout<<t<<','<<q.accepted<<','<<q.accepted_stamp<<','<<q.deferred<<','<<q.usable(t)<<'\n';}return 0;}
 QualifiedSteeringSample q;assert(q.update(-.30,100.,1.)==-.30);
 assert(q.update(.01,100.05,1.)==-.30&&q.deferred&&q.accepted_stamp==100.);
 assert(q.update(-.30,100.10,1.)==-.30&&!q.deferred&&q.accepted_stamp==100.10);
 q.update(.0,100.15,1.);assert(q.deferred&&q.accepted_stamp==100.10);
 q.update(.0,100.151,1.);assert(q.deferred); // duplicate deliveries cannot confirm
 q.update(.0,100.20,1.);assert(!q.deferred&&q.accepted==0.&&q.accepted_stamp==100.20);
 q.update(.01,99.,1.);assert(q.deferred&&q.accepted_stamp==100.20);
 q.update(std::numeric_limits<double>::quiet_NaN(),100.25,1.);
 assert(q.deferred&&q.accepted_stamp==100.20&&!q.usable(100.33));
 assert(!q.usable(100.19));
 QualifiedSteeringSample motion;
 assert(!motion.discontinuousZeroBatteryMotion(0,.05,100.,1.));
 motion.update(-.217,100.,1.);
 assert(motion.discontinuousZeroBatteryMotion(0,.051,100.05,1.));
 assert(motion.accepted_stamp==100.);
 assert(!motion.discontinuousZeroBatteryMotion(71,.051,100.05,1.));
 assert(!motion.discontinuousZeroBatteryMotion(0,-.232,100.05,1.));
 assert(!motion.discontinuousZeroBatteryMotion(0,.051,100.13,1.));
 assert(!motion.discontinuousZeroBatteryMotion(0,.051,99.95,1.));
 motion.update(-.230,100.10,1.);
 assert(motion.accepted==-.230&&motion.accepted_stamp==100.10);
 QualifiedSteeringSample slew;for(int i=0;i<100;++i){const double t=100.+i*.05;slew.update(.001*i,t,1.);assert(!slew.deferred);}
 std::cout<<"PASS: isolated pulse, real transition, duplicate, backward, NaN, stale, future, continuous slew\n";
}
