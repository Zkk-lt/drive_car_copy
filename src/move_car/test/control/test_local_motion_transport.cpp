#include "control/local_motion_transport.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

using namespace research_control;
namespace {
unsigned checks=0,cases=0;
void check(bool ok,const char* message){++checks;if(!ok)throw std::runtime_error(message);}
void near(double observed,double expected,double tolerance,const char* message){
  check(std::isfinite(observed)&&std::fabs(observed-expected)<=tolerance,message);
}
void pass(const char* name){++cases;std::cout<<"PASS "<<name<<'\n';}
LocalMotionConfig verified(){
  LocalMotionConfig c;c.wheel_scale=1.;c.wheel_calibrated=true;
  c.steering_calibrated=true;c.imu_frame_verified=true;c.source_timestamps_verified=true;
  return c;
}
template<std::size_t N> void fill(LocalMotionTransport<N>& m,double speed,double steer,double gyro,
                                 double begin=1.,double end=1.5){
  for(unsigned i=0;;++i){const double t=begin+.05*i;if(t>end+1e-10)break;
    check(m.updateWheel(speed,t,t),"fill wheel rejected");
    check(m.updateSteering(steer,t,t),"fill steering rejected");
    check(m.updateImu(gyro,t,t),"fill gyro rejected");
  }
}
}

int main(){try{
  {
    LocalMotionTransport<> m(verified());fill(m,.3,0.,0.);const auto r=m.relative(1.,1.5);
    check(r.valid,"constant straight invalid");near(r.tx,-.15,1e-12,"straight tx");
    near(r.ty,0.,1e-12,"straight ty");near(r.yaw,0.,1e-12,"straight yaw");
    check(r.gyro_measured_entire_window&&!r.bicycle_fallback_used,"gyro classification");
    check(r.source_timestamps_verified,"verified timestamps lost");
    check(!r.independent_ground_speed&&!r.hardware_freshness_verified&&!r.real_vehicle_control_eligible,
          "transport incorrectly certified hardware/ground/authority");
    check(r.steps<=40,"straight step bound");pass("constant straight, measured gyro and authority boundary");
  }
  {
    LocalMotionTransport<> m(verified());fill(m,.3,.2,.2);const auto r=m.relative(1.,1.5);
    check(r.valid,"circle invalid");near(r.tx,-1.5*std::sin(.1),3e-7,"circle tx");
    near(r.ty,1.5*(1.-std::cos(.1)),3e-7,"circle ty");near(r.yaw,-.1,1e-12,"circle yaw");
    pass("constant-circle SE2 and midpoint integration accuracy");
  }
  {
    auto c=verified();c.reference_offset_x=.3;LocalMotionTransport<> m(c);fill(m,.3,.2,.2);
    const auto r=m.relative(1.,1.5);check(r.valid,"lever mapping invalid");
    near(r.tx,-1.5*std::sin(.1)+.3*(std::cos(.1)-1.),3e-7,"center lever tx");
    near(r.ty,1.5*(1.-std::cos(.1))-.3*std::sin(.1),3e-7,"center lever ty");
    pass("explicit rear-to-reference rigid lever, no global pose");
  }
  {
    LocalMotionTransport<> m(verified());fill(m,.3,0.,0.,1.,1.);
    check(!m.updateWheel(.9,1.2,1.1),"future source accepted");
    check(std::string(m.wheelRejection())=="future_source_stamp","future source reason");
    const auto r=m.relative(1.,1.05);check(r.valid,"rejected sample destroyed usable old history");
    near(r.tx,-.015,1e-12,"future source polluted motion");
    check(r.rejected_samples==1,"future sample rejection count");pass("future source rejection without backdating");
  }
  {
    LocalMotionTransport<> m(verified());fill(m,.3,0.,0.,1.,1.);
    check(m.updateWheel(99.,1.10,1.10),"future arrival should remain in buffer");
    const auto r=m.relative(1.,1.05);check(r.valid,"future arrival incorrectly consumed");
    near(r.tx,-.015,1e-12,"future arrival leaked into past control");
    near(r.wheel.raw,.3,0.,"selected future raw sample");pass("receipt-causal historical control cutoff");
  }
  {
    LocalMotionTransport<> m(verified());check(m.updateWheel(.3,1.,1.),"first sample");
    check(!m.updateWheel(.4,.99,1.10),"out-of-order source accepted");
    check(std::string(m.wheelRejection())=="duplicate_or_out_of_order_source","source order reason");
    check(!m.updateWheel(.4,0.,.9),"out-of-order receipt accepted");
    check(!m.updateWheel(.4,0.,1.),"duplicate receipt accepted");
    check(std::string(m.wheelRejection())=="duplicate_or_out_of_order_receipt","receipt order reason");
    check(!m.updateImu(0.,1.,1.3),"late source accepted");
    check(std::string(m.imuRejection())=="late_source_sample","late source reason");
    pass("out-of-order, duplicate and late source/receipt diagnostics");
  }
  {
    LocalMotionTransport<> m(verified());m.updateImu(0.,1.,1.);m.updateSteering(0.,1.,1.);
    const auto r=m.relative(1.,1.05);check(!r.valid,"missing wheel accepted");
    check(!r.wheel.available,"missing wheel state");pass("missing wheel is invalid, gyro is not a speed source");
  }
  {
    LocalMotionTransport<> m(verified());fill(m,.3,0.,0.,1.,1.);
    const auto r=m.relative(1.,1.20);check(!r.valid,"stale wheel extrapolated");
    check(std::string(r.reason)=="stale_sample","stale wheel reason");pass("stale feedback rejected during interval");
  }
  {
    auto c=verified();LocalMotionTransport<> m(c);
    for(unsigned i=0;i<=10;++i){const double t=1.+.05*i;m.updateWheel(.3,t,t);m.updateSteering(.1,t,t);}
    m.updateImu(0.,1.,1.);const auto r=m.relative(1.,1.5);
    check(r.valid&&r.bicycle_fallback_used&&!r.gyro_measured_entire_window,"stale gyro fallback classification");
    check(r.yaw<0.,"bicycle fallback missing turn");
    check(r.yaw_uncertainty_rad>.5*c.gyro_uncertainty_radps,"fallback not more uncertain");
    pass("gyro expiry switches to explicitly model-based bicycle fallback");
  }
  {
    auto c=verified();c.allow_bicycle_fallback=false;LocalMotionTransport<> m(c);
    for(unsigned i=0;i<=4;++i){const double t=1.+.05*i;m.updateWheel(.3,t,t);m.updateSteering(.1,t,t);}
    m.updateImu(0.,1.,1.);const auto r=m.relative(1.,1.2);
    check(!r.valid&&std::string(r.reason)=="gyro_unavailable_bicycle_fallback_disabled","disabled fallback bypassed");
    pass("disabled fallback makes expired gyro invalid");
  }
  {
    LocalMotionTransport<> m(verified());m.updateWheel(.3,1.,1.);
    const auto r=m.relative(1.,1.05);check(!r.valid,"missing gyro and steer accepted");
    check(std::string(r.reason)=="gyro_and_steering_unavailable","missing fallback reason");
    pass("missing model fallback input stays unknown");
  }
  {
    LocalMotionTransport<> m(verified());
    for(unsigned i=0;i<=10;++i){const double t=1.+.05*i;m.updateWheel(0.,0.,t);m.updateSteering(.2,0.,t);}
    const auto r=m.relative(1.,1.5);check(r.valid&&r.low_speed_observed,"stopped transport invalid");
    near(r.tx,0.,0.,"stop translated");near(r.ty,0.,0.,"stop lateral drift");near(r.yaw,0.,0.,"stop bicycle turned");
    check(r.bicycle_fallback_used&&!r.gyro_measured_entire_window&&!r.source_timestamps_verified,"stop mislabeled measured");
    pass("stopped wheel proxy, receipt-only timing and no invented yaw measurement");
  }
  {
    LocalMotionTransport<> m(verified());fill(m,.01,.2,.02);const auto r=m.relative(1.,1.5);
    check(r.valid&&r.low_speed_observed,"low-speed flag");near(r.yaw,-.01,1e-12,"low speed suppressed actual gyro");
    check(!r.independent_ground_speed,"low speed gained false ground independence");
    pass("low speed retains measured gyro but no slip/ground-speed claim");
  }
  {
    LocalMotionTransport<> m(verified());fill(m,.3,.2,2.,1.,1.);
    const auto r=m.relative(1.,1.05);check(!r.valid&&std::string(r.reason)=="gyro_range","bad fresh gyro hidden by fallback");
    LocalMotionTransport<> w(verified());fill(w,3.,0.,0.,1.,1.);
    check(!w.relative(1.,1.05).valid,"wheel range accepted");
    pass("nonphysical raw ranges reject rather than clip into validity");
  }
  {
    LocalMotionTransport<> m;fill(m,.3,0.,0.,1.,1.);
    auto r=m.relative(1.,1.05);check(!r.valid&&std::string(r.reason)=="wheel_units_unverified","default calibration silently enabled");
    auto c=verified();c.steering_calibrated=false;c.imu_frame_verified=false;LocalMotionTransport<> x(c);fill(x,.3,0.,0.,1.,1.);
    check(!x.relative(1.,1.05).valid,"unverified axes/steer became valid");
    pass("independent channel calibration defaults fail closed");
  }
  {
    auto c=verified();c.source_timestamps_verified=false;LocalMotionTransport<> m(c);fill(m,.3,0.,0.,1.,1.);
    const auto r=m.relative(1.,1.05);check(r.valid&&!r.source_timestamps_verified,"driver header certified acquisition time");
    check(r.imu.source_time_present&&!r.imu.source_time_verified,"source and verified time conflated");
    pass("nonzero header is preserved without acquisition-time certification");
  }
  {
    LocalMotionTransport<> m(verified());fill(m,.3,0.,0.);
    check(!m.relative(1.6,1.5).valid,"future reference accepted");
    auto r=m.relative(.9,1.5);check(!r.valid&&std::string(r.reason)=="transport_window_exceeded","transport window limit");
    r=m.relative(1.,5.);check(!r.valid&&std::string(r.reason)=="reference_outside_history_bound","history age limit");
    pass("future reference and distinct transport/history limits");
  }
  {
    LocalMotionTransport<> m(verified());fill(m,.3,0.,0.);check(m.relative(1.,1.5).valid,"pre-reset transform");
    auto r=m.relative(.4,.5);check(!r.valid&&std::string(r.reason)=="control_clock_regressed_requires_new_samples","clock regression not detected");
    check(!m.relative(.4,.5).valid,"old epoch data survived clock reset");
    fill(m,.2,0.,0.,.4,.5);r=m.relative(.4,.5);check(r.valid,"fresh new epoch cannot recover");
    near(r.tx,-.02,1e-12,"new epoch transform");pass("clock reset clears history and requires fresh epoch samples");
  }
  {
    LocalMotionTransport<4> m(verified());
    for(unsigned i=0;i<8;++i){const double t=1.+.01*i;m.updateWheel(.3,t,t);m.updateImu(0.,t,t);}
    const auto r=m.relative(1.,1.08);check(!r.valid,"overwritten historical sample fabricated");
    check(r.overwritten_samples==8,"bounded overwrite count");pass("fixed-capacity overwrite cannot restore absent history");
  }
  {
    LocalMotionTransport<> m(verified());const auto r=m.relative(1.,1.);
    check(r.valid&&std::string(r.reason)=="identity_only_state_not_validated","zero interval identity");
    check(!r.gyro_measured_entire_window&&!r.real_vehicle_control_eligible,"identity certified measured state");
    check(!m.relative(std::numeric_limits<double>::quiet_NaN(),1.).valid,"NaN time accepted");
    check(!m.updateWheel(std::numeric_limits<double>::infinity(),1.,1.),"nonfinite raw accepted");
    pass("identity is geometry only; invalid clocks/raw remain invalid");
  }
  {
    auto c=verified();c.max_position_uncertainty_m=.001;LocalMotionTransport<> m(c);fill(m,.3,0.,0.);
    const auto r=m.relative(1.,1.5);check(!r.valid&&std::string(r.reason)=="configured_uncertainty_envelope_exceeded","uncertainty gate bypassed");
    c=verified();c.max_step_s=1e-8;LocalMotionTransport<> bad(c);check(!bad.configValid(),"unbounded integration work accepted");
    pass("engineering uncertainty and integration-work bounds reject unsafe assumptions");
  }
  {
    LocalMotionTransport<> m(verified());m.updateWheel(.3,1.,1.);m.updateImu(0.,1.,1.);
    m.updateWheel(.5,1.025,1.025);m.updateWheel(.9,1.05,1.05);
    const auto r=m.relative(1.,1.05);check(r.valid,"piecewise causal transform invalid");
    near(r.tx,-.02,1e-12,"future interpolation changed prior motion");
    pass("piecewise ZOH input changes, no future interpolation");
  }
  {
    LocalMotionTransport<> m(verified());m.updateWheel(.1,1.,1.);m.updateImu(0.,1.,1.);
    m.updateWheel(.5,1.01,1.04);const auto r=m.relative(1.,1.05);
    check(r.valid,"late-but-causal sample rejected");near(r.tx,-.009,1e-12,"late arrival backdated to source");
    near(r.wheel.source_stamp,1.01,0.,"source timestamp mutated");
    near(r.wheel.receipt_stamp,1.04,0.,"receipt timestamp mutated");
    pass("late arrival affects motion only after receipt, original clocks unchanged");
  }
  {
    const double epoch=1794219581.;LocalMotionTransport<> m(verified());
    m.updateWheel(.3,epoch,epoch);m.updateSteering(.1,epoch,epoch);m.updateImu(0.,epoch,epoch);
    const auto r=m.relative(epoch,epoch+.2);
    check(!r.valid&&std::string(r.reason)=="stale_sample","epoch rounding held stale wheel an extra step");
    LocalMotionTransport<> x(verified());fill(x,.3,0.,0.,epoch,epoch+.5);
    const auto a=x.relative(epoch,epoch+.5);check(a.valid,"epoch-sized fresh sample integration invalid");
    near(a.tx,-.15,1e-6,"epoch-sized straight transform");
    check(a.pointPositionUncertainty(5.)>=a.position_uncertainty_m+5.*a.yaw_uncertainty_rad,
          "point-distance angular uncertainty missing");
    check(!std::isfinite(a.pointPositionUncertainty(-1.)),"invalid point radius uncertainty accepted");
    pass("epoch-sized ROS timestamp expiry and point-distance uncertainty");
  }
  std::cout<<"SUMMARY cases="<<cases<<" checks="<<checks<<"\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL after "<<cases<<" cases: "<<e.what()<<'\n';return 1;}}
