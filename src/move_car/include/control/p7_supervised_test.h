#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
namespace unified_control {
// An explicit operator/deadman contract for a supervised research test.  This
// conveys software authority only; it is not physical calibration or approval.
struct SupervisedGuard {
 bool enabled=false,mapping_confirmed=false,synthetic=false;
 std::string command_topic,run_id,terrain,operator_id,mapping_evidence,imu_frame;
 double speed_cap=0.,degraded_speed_limit=.10,wheel_scale=.001,steering_sign=1.,imu_yaw_sign=1.;
 double lease_receipt_steady=0.;std::uint64_t lease_sequence=0;
 // Normal operating request, below the immutable S8 0.50m/s limit. This is
 // separate from S8's unchanged degraded/solver-failure speed settings.
 static constexpr double maximum_speed=.40,lease_ttl=.12;
 bool configured()const {
  return enabled&&mapping_confirmed&&!synthetic&&command_topic=="/cmd_vel"&&
   run_id.size()>7&&run_id.compare(0,7,"p7real_")==0&&run_id.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-")==std::string::npos&&
   terrain=="smooth"&&operator_id.find_first_not_of(" \t\r\n")!=std::string::npos&&imu_frame.find_first_not_of(" \t\r\n")!=std::string::npos&&mapping_evidence.size()==64&&
   mapping_evidence.find_first_not_of("0123456789abcdef")==std::string::npos&&
   std::isfinite(speed_cap)&&speed_cap>=0.&&speed_cap<=maximum_speed&&
   std::isfinite(degraded_speed_limit)&&degraded_speed_limit>0.&&degraded_speed_limit<=.10&&
   std::isfinite(wheel_scale)&&std::abs(wheel_scale-.001)<1e-12&&
   std::isfinite(steering_sign)&&std::abs(steering_sign)==1.&&imu_yaw_sign==1.;
 }
 bool acceptLease(const std::string&payload,double receipt_steady,bool latched=false){
  if(!configured()||latched||!std::isfinite(receipt_steady)||receipt_steady<=0.||receipt_steady<lease_receipt_steady)return false;
  const std::string prefix=run_id+"|";if(payload.compare(0,prefix.size(),prefix)!=0)return false;
  const std::string seq=payload.substr(prefix.size());if(seq.empty()||seq[0]=='0'||seq.find_first_not_of("0123456789")!=std::string::npos)return false;
  std::uint64_t n=0;for(char c:seq){const auto digit=static_cast<unsigned>(c-'0');if(n>(std::numeric_limits<std::uint64_t>::max()-digit)/10)return false;n=n*10+digit;}
  if(n<=lease_sequence)return false;
  lease_sequence=n;
  lease_receipt_steady=receipt_steady;
  return true;
 }
 bool armed(double now_steady)const{return configured()&&lease_sequence>0&&std::isfinite(now_steady)&&now_steady>=lease_receipt_steady&&now_steady-lease_receipt_steady<=lease_ttl;}
 double boundedSpeed(double v)const{return std::isfinite(v)&&configured()?std::max(0.,std::min(v,speed_cap)):0.;}
};
// Only fixed nominal models during first supervised collection. Optional SLQR
// feedback requires a fresh causal observation and full nonlinear validation.
inline bool p7FixedMode(const std::string&m){return m=="legacy_slqr"||m=="nmpc_shadow"||m=="nmpc_12_fixed"||m=="flat_nmpc_8"||m=="flat_nmpc_8_uniform"||m=="blocked8"||m=="flat_nmpc_slqr";}
}
