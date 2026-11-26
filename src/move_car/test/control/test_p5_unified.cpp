#include "control/local_state_initializer.h"
#include "control/p5_nominal_propagation.h"
#include "control/adaptive_runtime.h"
#include <cassert>
#include <iostream>
#include <thread>
#include <atomic>
using namespace unified_control;
int main(){
 int checks=0;auto check=[&](bool b){++checks;if(!b)std::cerr<<"failed check "<<checks<<"\n";assert(b);};
 ControlRequest r;r.now=10.;r.dt=1./30.;r.guard={true,true,"/control_research/p5_test/command","p5_unit"};r.mode=ControlMode::NmpcFixed;r.legacy={.1,.01};r.previous={.1,.01};r.history_valid=true;r.history_sequence=50;r.config_version=1;r.epoch=1;r.reference={9.99,10,true,false};r.state.valid=r.state.causal=r.state.nominal_model_valid=true;r.state.stamp=r.state.receipt_stamp=9.99;r.state.speed=.1;r.state.steering=.01;r.state.confidence=.35;
 ControlCandidate c;c.valid=c.feasible=c.complete_substep_constraints=c.state_pair_validated=true;c.sequence=1;c.epoch=1;c.config_version=1;c.reference_sequence=10;c.history_base_sequence=50;c.command=r.previous;c.created_stamp=9.999;c.snapshot_stamp=9.99;c.expected_apply_stamp=10.;c.solve_ms=1.;c.propagated_speed=.1;c.propagated_steering=.01;
 check(ControlArbiter::decide(r,&c).selected_nmpc);
 for(ControlMode mode:{ControlMode::LegacySlqr,ControlMode::NmpcShadow}){auto q=r;q.mode=mode;auto x=c;x.command={.5,.32};auto d=ControlArbiter::decide(q,&x);check(!d.selected_nmpc&&d.command.speed==q.legacy.speed&&d.command.steering==q.legacy.steering);}
 for(int i=0;i<16;++i){auto q=r;auto x=c;switch(i){case 0:q.guard.command_topic="/cmd_vel";break;case 1:x.timed_out=true;break;case 2:x.valid=false;break;case 3:x.command.speed=INFINITY;break;case 4:x.history_base_sequence=49;break;case 5:x.snapshot_stamp=9.;break;case 6:q.state.valid=false;break;case 7:q.state.stamp=11.;break;case 8:q.history_valid=false;break;case 9:q.reference.valid=false;break;case 10:x.reference_sequence=11;break;case 11:x.state_pair_validated=false;break;case 12:x.complete_substep_constraints=false;break;case 13:x.command.steering=.30;break;case 14:q.mode=ControlMode::NmpcAdaptive;break;case 15:q.state.yaw_rate=10.;break;}check(!ControlArbiter::decide(q,&x).selected_nmpc);}
 auto emergency=r;emergency.safety_stop=true;auto d=ControlArbiter::decide(emergency,&c);check(d.safety_stop&&d.command.speed==0.&&!d.selected_nmpc);
 CommandLedger ledger(1);PublishedCommandRecord record;record.epoch=1;record.ros_publish_returned=true;record.command={.1,.02};record.acceleration=0.;
 auto commit=[&](int n,double t){record.sequence=n;record.cycle_stamp=t;record.publish_ros_start=t+.001;record.publish_ros_end=record.stamp=t+.002;record.publish_steady_start_ns=static_cast<uint64_t>(t*1e9);record.publish_steady_end_ns=record.publish_steady_start_ns+1000;return ledger.commit(record);};
 check(commit(1,10.).accepted&&ledger.valid());check(!ledger.commit(record).accepted&&!ledger.valid());check(commit(2,10.033).accepted&&!ledger.valid());
 for(int i=3;i<=40;++i)check(commit(i,10.+(i-1)/30.).accepted);
 check(ledger.certifyRecovery(40,10.3,.6));record.sequence=41;record.ros_publish_returned=false;check(!ledger.commit(record).accepted&&ledger.lastSequence()==40&&!ledger.valid());record.ros_publish_returned=true;check(commit(41,11.34).accepted);check(!commit(43,11.37).accepted);check(ledger.lastSequence()==41);
 LocalInitializationContract contract;contract.wheel_units_verified=contract.steering_units_verified=contract.imu_body_frame_verified=true;LocalInitializationInput in;in.now=10.;in.reference_local_frame_verified=true;in.wheel={100.,9.99,0.,true,false,false};in.steering={.02,9.99,0.,true,false,false};in.imu_yaw={.01,9.99,9.98,true,true,false};
 auto init=LocalStateInitializer::initialize(in,contract);check(init.state.valid&&init.state.speed==.1&&!init.state.slip_valid&&!init.state.physical_calibrated);in.imu_yaw.source_stamp=10.1;check(!LocalStateInitializer::initialize(in,contract).state.valid);in.imu_yaw.source_stamp=9.98;in.wheel.receipt_stamp=9.;check(!LocalStateInitializer::initialize(in,contract).state.valid);
 research_control::Model m;research_control::Limits l;std::vector<research_control::CommandSample>history;for(int i=0;i<30;++i)history.push_back({9.+i/30.,{.1,.02}});research_control::State s;s.speed=.1;s.steering=.02;
 auto p=research_control::p5PropagateNominal(s,10.,10.033,m,l,history,0.,0.,.04,.08,10.);check(p.valid&&p.state.x>0.&&!p.reason.empty());history.back().stamp=10.1;check(!research_control::p5PropagateNominal(s,10.,10.033,m,l,history,0.,0.,.04,.08,10.).valid);
 auto tight=l;tight.speed=.08;tight.jerk=.4;auto both=intersectLimits(l,tight);check(both.speed==.08&&both.jerk==.4&&both.steering==l.steering);
 research_control::RuntimeConfig cfg;research_control::AdaptiveRuntime runtime;check(runtime.configure(cfg));runtime.requestMode("nmpc_fixed");research_control::SolveRequest req;req.now=10.;req.reference.valid=true;req.state.speed=.1;req.baseline={.1,0.};auto result=runtime.cycle(req,10.,true,false,true,&tight);check(result.effective.limits.speed<=.08&&result.effective.limits.jerk<=.4);tight.jerk=NAN;check(runtime.cycle(req,10.033,true,false,true,&tight).reason=="fallback:invalid_external_safety_limits");
 LatestCandidateMailbox mailbox;check(!mailbox.tryRead(c));check(mailbox.tryPublish(c));std::atomic<bool> nonblocking{false};mailbox.withLockedSlotForTest([&](){std::thread th([&](){ControlCandidate value;nonblocking=!mailbox.tryRead(value)&&!mailbox.tryPublish(value);});th.join();});check(nonblocking);
 for(int i=0;i<1000;++i){check(mailbox.tryPublish(c));ControlCandidate copy;check(mailbox.tryRead(copy)&&copy.sequence==c.sequence);}
 std::cout<<"P5 unified contract checks="<<checks<<" PASS\n";
}
