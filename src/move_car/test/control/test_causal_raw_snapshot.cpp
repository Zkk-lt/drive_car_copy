#include "control/causal_raw_snapshot.h"
#include "control/vehicle_state_observer.h"
#include <cassert>
#include <iostream>
#include <limits>

using namespace research_control;
struct Sample {double value=0.,source=0.;};

int main(){
 {
  CausalRawSnapshot<Sample,4> buffer;Sample last;double receipt=0.;unsigned feeds=0;
  auto feed=[&](const Sample& sample,double t){last=sample;receipt=t;++feeds;};
  assert(buffer.push({1.,1.},1.01));assert(buffer.push({2.,1.1},1.11));
  assert(!buffer.feed(1.,feed)); // It was not yet received at this control time.
  assert(buffer.feed(1.05,feed));assert(last.value==1.&&last.source==1.&&receipt==1.01);
  assert(!buffer.feed(1.05,feed));assert(feeds==1); // No duplicate observer update.
  assert(buffer.feed(1.12,feed));assert(last.value==2.&&last.source==1.1&&receipt==1.11);
  buffer.resetFed();assert(buffer.feed(1.12,feed));assert(feeds==3&&receipt==1.11);
  assert(!buffer.push({3.,1.05},1.10));assert(buffer.rejected()==1);
  assert(!buffer.push({3.,1.2},std::numeric_limits<double>::quiet_NaN()));
  std::cout<<"PASS receipt causality, unchanged clocks, duplicate suppression and reset\n";
 }
 {
  CausalRawSnapshot<int,3> buffer;int value=0;
  for(int i=1;i<=4;++i)assert(buffer.push(i,double(i)));
  assert(buffer.size()==3&&buffer.overwritten()==1);
  assert(!buffer.feed(1.5,[&](int v,double){value=v;})); // Expired history stays unknown.
  assert(buffer.feed(2.5,[&](int v,double){value=v;}));assert(value==2);
  assert(buffer.feed(4.,[&](int v,double){value=v;}));assert(value==4);
  std::cout<<"PASS bounded overwrite and unavailable historical sample\n";
 }
 {
  ObserverConfig config;config.wheel_scale=1.;config.wheel_calibrated=true;
  config.source_timestamps_verified=true;VehicleStateObserver observer(config);Model model;
  CausalRawSnapshot<Sample,4> buffer;
  assert(buffer.push({.3,1.2},1.));
  buffer.feed(1.05,[&](const Sample& x,double receipt){observer.updateWheel(x.value,x.source,receipt);});
  assert(observer.rawWheel().time.source_stamp==1.2&&observer.rawWheel().time.receipt_stamp==1.);
  assert(!observer.rawWheel().time.accepted);
  assert(observer.rawWheel().time.reason=="future_source_stamp");
  assert(!observer.observe(1.05,model).speed.valid);
  observer.reset();buffer.resetFed();
  assert(buffer.push({.3,1.1},2.));
  buffer.feed(2.01,[&](const Sample& x,double receipt){observer.updateWheel(x.value,x.source,receipt);});
  assert(!observer.rawWheel().time.accepted&&observer.rawWheel().time.reason=="late_sensor_sample");
  std::cout<<"PASS source-future and late acquisition still rejected by unchanged observer\n";
 }
 {
  ObserverConfig config;config.wheel_scale=1.;config.wheel_calibrated=true;
  VehicleStateObserver observer(config);Model model;CausalRawSnapshot<Sample,4> buffer;
  assert(buffer.push({.3,0.},1.));
  auto feed=[&](const Sample& x,double receipt){observer.updateWheel(x.value,x.source,receipt);};
  assert(buffer.feed(1.02,feed));assert(observer.rawWheel().time.accepted);
  assert(!buffer.feed(1.03,feed));assert(observer.rawWheel().time.accepted);
  observer.reset();buffer.resetFed();assert(buffer.feed(1.04,feed));
  assert(observer.rawWheel().time.receipt_stamp==1.&&observer.rawWheel().time.source_stamp==0.);
  assert(observer.observe(1.04,model).speed.valid);
  assert(!observer.observe(2.,model).speed.valid); // Rebuild cannot refresh old evidence.
  observer.configure(ObserverConfig());buffer.resetFed();assert(buffer.feed(1.04,feed));
  assert(!observer.observe(1.04,model).speed.valid); // Default calibration stays disabled.
  std::cout<<"PASS headerless receipt retained, stale/reset recovery and default calibration disabled\n";
 }
}
