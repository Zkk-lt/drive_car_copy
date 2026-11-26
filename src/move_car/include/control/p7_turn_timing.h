#pragma once
#include "control/flatness_reference_model.h"

namespace research_control {

// Performance preview for supported tightening and opening turns. It cannot
// change the hard speed cap, path geometry, endpoint or physical constraints.
class P7TurnTiming {
  static double fade(double value,double low,double high) {
    const double f=clamp((value-low)/(high-low),0.,1.);
    return f*f*(3.-2.*f);
  }
public:
  struct Result {
    double speed_reserve=0.,release_weight=0.;
    bool tightening=false,opening=false;
  };
  static Result apply(const std::vector<ReferencePoint>& path,const SolverConfig& config,
                      SolveRequest& request,double speed_reserve,double release_weight) {
    Result result;
    if(!request.reference.valid||!request.reference.speed_preview||
       request.reference.braking||request.reference.terminal_approach||
       !request.published_history_contiguous||path.size()<2||path.size()>1024||
       !std::isfinite(speed_reserve)||speed_reserve<0.||speed_reserve>.05||
       !std::isfinite(release_weight)||release_weight<0.||release_weight>4.||
       (speed_reserve==0.&&release_weight==0.))return result;
    const auto& model=config.model;
    if(!(request.state.speed>=.15&&request.state.speed<=.50&&model.wheelbase>0.&&
         model.rear_to_center>=0.&&model.yaw_gain>0.&&std::isfinite(request.state.steering)&&
         std::isfinite(request.target_speed)&&std::isfinite(request.speed_cap)))return result;

    const double s0=path.front().s;
    // All chords use positions, not differentiated tangent fields near the
    // ego-anchored connector. Retain at least .20 m past the last chord.
    if(!std::isfinite(s0)||!(path.back().s-s0>=1.18))return result;
    constexpr std::array<double,5> center{{.21,.35,.49,.64,.78}};
    constexpr std::array<double,5> half{{.15,.15,.15,.18,.20}};
    struct XY{double x,y;};
    auto sample=[&](double station,XY& p){
      station+=s0;
      const auto b=std::lower_bound(path.begin()+1,path.end(),station,
          [](const ReferencePoint& q,double s){return q.s<s;});
      if(b==path.end())return false;
      const auto& a=*(b-1);const double ds=b->s-a.s;
      if(!(ds>0.&&ds<=.15))return false;
      const double f=clamp((station-a.s)/ds,0.,1.);
      p={a.x+f*(b->x-a.x),a.y+f*(b->y-a.y)};
      return std::isfinite(p.x)&&std::isfinite(p.y);
    };
    auto chordCurvature=[&](double station,double radius,double& k){
      XY a{},b{},c{};
      if(!sample(station-radius,a)||!sample(station,b)||!sample(station+radius,c))return false;
      const XY u{b.x-a.x,b.y-a.y},v{c.x-b.x,c.y-b.y};
      const double lengths=std::hypot(u.x,u.y)+std::hypot(v.x,v.y);
      if(!(lengths>=1.8*radius)||u.x*v.x+u.y*v.y<=0.)return false;
      k=2.*std::atan2(u.x*v.y-u.y*v.x,u.x*v.x+u.y*v.y)/lengths;
      return std::abs(k)<.90;
    };
    std::array<double,5> curve{};
    for(std::size_t i=0;i<curve.size();++i)
      if(!chordCurvature(center[i],half[i],curve[i]))return result;
    const double side=(std::abs(curve[2])>=.14?curve[2]:curve[0])>=0.?1.:-1.;
    for(double& k:curve)k*=side;
    for(double k:curve)if(k<-.09)return result;
    const bool local_turn=curve[2]>=.14&&curve[3]>=.13&&curve[4]>=.10;
    const double near=.5*(curve[0]+curve[1]);
    const double far=.5*(curve[3]+curve[4]);
    const double lasting_turn=.5*(curve[2]+curve[3]);
    double strength=0.;
    if(speed_reserve>0.){
      if(local_turn&&lasting_turn>.20&&far>.25&&curve[2]>=near-.04&&curve[4]>=curve[2]-.09)
        strength=fade(far-near,.09,.27);
      // The near arc can open briefly before a second same-side tightening.
      // Give the delayed speed channel time to respond using two supported
      // chords beyond the nominal steering horizon. No hard cap changes.
      if(strength<1.&&near>=.20&&curve[1]>=.12&&path.back().s-s0>=1.46){
        double later1,later2;
        if(chordCurvature(1.05,.20,later1)&&chordCurvature(1.20,.20,later2)){
          later1*=side;later2*=side;
          if(later1>=.28&&later2>=.45&&later2>=later1&&later2>=near+.12)
            strength=std::max(strength,fade(later2-near,.12,.24));
        }
      }
    }
    if(strength>0.){
      const double ceiling=std::min(request.speed_cap,config.limits.speed);
      const double target=std::min(request.target_speed,std::max(.15,ceiling-speed_reserve*strength));
      if(target<request.target_speed-1e-5){
        // Preserve the original projected path and every stop/constraint
        // field. Only the nominal time law is rebuilt with headroom.
        auto revised=FlatnessReferenceModel::buildTimeReference(path,request.state.speed,target,
            request.now,config,false,true,request.state.acceleration,
            request.reference.synchronized_preview,false,request.reference.initial_body_yaw);
        if(revised.valid&&!revised.terminal_approach){
          result.tightening=true;result.speed_reserve=request.target_speed-target;
          request.target_speed=target;request.reference=std::move(revised);
        }
      }
    }
    const double rear_k=side*(model.yaw_gain*std::tan(request.state.steering)/model.wheelbase+
                              model.yaw_residual/request.state.speed);
    const double body_k=rear_k/std::hypot(1.,model.rear_to_center*rear_k);
    if(release_weight>0.&&local_turn&&near>.20&&near-far>.06&&curve[4]<curve[2]-.035&&
       curve[2]<=near+.07&&curve[3]<=curve[2]+.065&&body_k>near+.04){
      // Weight follows both sustained opening and excess measured curvature.
      // The target is sampled at each predicted stage's traveled station,
      // never from the far probe directly at the current vehicle pose.
      result.release_weight=release_weight*fade(near-far,.06,.18)*fade(body_k-near,.04,.17);
      if(result.release_weight>1e-5){
        result.opening=true;
        request.turn_release.weight=result.release_weight;
        request.turn_release.start_s=s0;
        request.turn_release.station=center;
        for(std::size_t i=0;i<curve.size();++i)
          request.turn_release.curvature[i]=side*curve[i];
      }
    }
    return result;
  }

  static double releaseYawRate(const SolveRequest::TurnRelease& profile,const ReferencePoint& point) {
    const auto& s=profile.station;const auto& k=profile.curvature;
    const double d=point.s-profile.start_s;
    if(d<=s.front())return point.speed*k.front();
    if(d>=s.back())return point.speed*k.back();
    const auto upper=std::upper_bound(s.begin(),s.end(),d);
    const std::size_t i=upper-s.begin();
    const double f=(d-s[i-1])/(s[i]-s[i-1]);
    return point.speed*((1.-f)*k[i-1]+f*k[i]);
  }
};

} // namespace research_control
