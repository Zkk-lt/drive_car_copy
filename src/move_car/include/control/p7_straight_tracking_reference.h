#pragma once

#include "control/research_types.h"

namespace research_control {

// Condition only the tracking objective, AFTER the original spatial reference
// has set speed, endpoint and feasibility data. A line is inferred from
// supported positions over a physical distance, not differentiated point yaw.
class P7StraightTrackingReference {
public:
  struct Course {
    double blend=0.,straight_blend=0.,curvature=0.;
  };

  // Smooth the tracking objective of a supported straight or gentle course.
  // Fit positions beyond the ego-anchored splice. A quadratic retains broad
  // turning while suppressing the short alternating derivative of the splice.
  // Coordinates in the fit frame are in metres and no history is accumulated.
  static Course conditionSupportedCourse(const std::vector<ReferencePoint>& spatial, Reference& tracking,
                                         double line_blend=0.) {
    if(!tracking.valid||tracking.braking||tracking.terminal_approach||tracking.count<2||
       tracking.count>kMaxHorizon+1||spatial.size()<2||spatial.size()>1024||
       !(line_blend>=0.&&line_blend<.25)||!std::isfinite(tracking.initial_body_yaw))return {};
    constexpr double fit_begin=.38,fit_end=1.38,guard_end=1.48;
    const double start=spatial.front().s;
    const double travelled=tracking.points[tracking.count-1].s-start;
    if(!std::isfinite(start)||!(travelled>=.10&&travelled<=fit_end)||spatial.back().s-start<guard_end)return {};

    struct Point{double x,y;};
    constexpr int count=9;
    std::array<Point,count> samples{};
    std::size_t j=0;
    for(int i=0;i<count;++i){
      const double s=start+fit_begin+(fit_end-fit_begin)*i/(count-1);
      while(j+2<spatial.size()&&spatial[j+1].s<s)++j;
      const auto& a=spatial[j];const auto& b=spatial[j+1];
      const double ds=b.s-a.s;
      if(!(ds>0.&&ds<=.15))return {};
      const double f=(s-a.s)/ds;
      samples[i]={a.x+f*(b.x-a.x),a.y+f*(b.y-a.y)};
      if(!std::isfinite(samples[i].x)||!std::isfinite(samples[i].y))return {};
    }
    const auto anchor=samples.front();
    const double dx=samples.back().x-anchor.x,dy=samples.back().y-anchor.y;
    const double chord=std::hypot(dx,dy);
    if(!(chord>=.95&&chord<=1.04))return {};
    const double ux=dx/chord,uy=dy/chord,yaw=std::atan2(uy,ux);
    const Point center{anchor.x+.5*dx,anchor.y+.5*dy};
    const auto tangent=[&](double x,double y){return ux*(x-center.x)+uy*(y-center.y);};
    const auto normal=[&](double x,double y){return -uy*(x-center.x)+ux*(y-center.y);};
    double mean_x=0.,mean_y=0.;
    for(auto& p:samples){p={tangent(p.x,p.y),normal(p.x,p.y)};mean_x+=p.x/count;mean_y+=p.y/count;}
    double mean_sq=0.;
    for(auto& p:samples){p.x-=mean_x;mean_sq+=p.x*p.x/count;}
    double xx=0.,xz=0.,zz=0.,xy=0.,zy=0.;
    for(const auto& p:samples){
      const double z=p.x*p.x-mean_sq,v=p.y-mean_y;
      xx+=p.x*p.x;xz+=p.x*z;zz+=z*z;xy+=p.x*v;zy+=z*v;
    }
    const double determinant=xx*zz-xz*xz;
    if(!(determinant>1e-4))return {};
    const double b=(zz*xy-xz*zy)/determinant,c=(xx*zy-xz*xy)/determinant;
    if(!(std::abs(wrap(yaw+std::atan(b)-tracking.initial_body_yaw))<.25))return {};
    const auto offset=[&](double x){return mean_y+b*x+c*(x*x-mean_sq);};
    const auto slope=[&](double x){return b+2.*c*x;};
    const auto fade=[](double value,double low,double high){
      const double f=clamp((high-value)/(high-low),0.,1.);
      return f*f*(3.-2.*f);
    };
    double corridor=0.,connector=0.;
    // Check the dense preview too. An actual turn after the fitted samples
    // must not be hidden by a narrow fitted interval or sparse knots.
    double last_x=-std::numeric_limits<double>::infinity();
    for(const auto& p:spatial){
      if(p.s>start+guard_end)break;
      const double x=tangent(p.x,p.y)-mean_x,y=normal(p.x,p.y);
      if(!(x>last_x)||!std::isfinite(y))return {};
      last_x=x;
      const double error=std::abs(y-offset(x));
      if(p.s<start+fit_begin)connector=std::max(connector,error);
      else corridor=std::max(corridor,error);
    }
    const double curvature=2.*c;
    const double blend=fade(corridor,.010,.020)*fade(connector,.032,.059)*
                       fade(std::abs(curvature),.10,.21)*fade(line_blend,0.,.25);
    if(!(blend>1e-6))return {};
    for(int i=0;i<tracking.count;++i){
      const auto& p=tracking.points[i];const double x=tangent(p.x,p.y)-mean_x;
      if(!std::isfinite(p.curvature)||!(std::abs(offset(x)-normal(p.x,p.y))<=.055)||
         !(std::abs(wrap(yaw+std::atan(slope(x))-p.yaw))<.33))return {};
    }
    for(int i=0;i<tracking.count;++i){
      auto& p=tracking.points[i];const double x=tangent(p.x,p.y)-mean_x;
      const double d=slope(x),g=std::hypot(1.,d);
      const double shift=blend*(offset(x)-normal(p.x,p.y));
      p.x-=uy*shift;p.y+=ux*shift;
      p.yaw=wrap(p.yaw+blend*wrap(yaw+std::atan(d)-p.yaw));
      p.curvature+=blend*(curvature/(g*g*g)-p.curvature);
    }
    return {blend,blend*fade(std::abs(curvature),.035,.10),curvature};
  }

  // A weak or partial line fit must not add a zero-turn objective at bend entry.
  static double yawDampingWeight(double blend,double maximum) {
    if (!std::isfinite(blend) || !std::isfinite(maximum) || maximum<=0.) return 0.;
    const double f=clamp((blend-.55)/.40,0.,1.);
    return maximum*f*f*(3.-2.*f);
  }

  // Share one weight between the zero-yaw prior and the fitted course's
  // nominal body yaw. A gentle bend must retain its nonzero reference rate.
  static double courseYawWeight(const Course& course,double maximum) {
    if(!(course.blend>0.&&course.blend<=1.)||!(maximum>0.&&maximum<=4.))return 0.;
    return std::max(0.,maximum*course.blend-yawDampingWeight(course.straight_blend,maximum));
  }

  static double condition(const std::vector<ReferencePoint>& spatial, Reference& tracking) {
    if (!tracking.valid || tracking.braking || tracking.terminal_approach ||
        tracking.count < 2 || tracking.count > kMaxHorizon+1 || spatial.size() < 2)
      return 0.;

    const double start = spatial.front().s;
    const double travelled = tracking.points[tracking.count-1].s-start;
    // The extra .30 m out front covers approximately one steering response
    // interval at the smooth trial's .40 m/s cap. In both recorded runs this
    // kept the fit out of clear bends already visible beyond the solve mesh.
    const double support = std::max(1.25, travelled+.30);
    if (!std::isfinite(start) || !std::isfinite(travelled) || travelled < .10 ||
        support > 1.35 || spatial.back().s-start < support) return 0.;

    struct Point { double x, y; };
    constexpr std::size_t n = 9;
    const double step = support/(n-1);
    std::array<Point,n> samples{};
    std::size_t j = 0;
    for (std::size_t i=0; i<n; ++i) {
      const double station = start+i*step;
      while (j+2<spatial.size() && spatial[j+1].s<station) ++j;
      const auto& a = spatial[j]; const auto& b = spatial[j+1];
      const double ds = b.s-a.s;
      // Do not let an interpolated long gap count as independent line support.
      if (!(ds>0. && ds<=.20)) return 0.;
      const double f = clamp((station-a.s)/ds,0.,1.);
      samples[i] = {a.x+f*(b.x-a.x), a.y+f*(b.y-a.y)};
      if (!std::isfinite(samples[i].x) || !std::isfinite(samples[i].y)) return 0.;
    }

    // The first reconstructed point can be tied to the current follower
    // pose. Estimate the line from preview positions, but still require the
    // near origin to lie inside its corridor.
    Point center{0.,0.}, direction{0.,0.};
    for (std::size_t i=1; i<n; ++i) {
      const double t = i-.5*n;
      center.x += samples[i].x/(n-1);
      center.y += samples[i].y/(n-1);
      direction.x += t*(samples[i].x-samples.front().x);
      direction.y += t*(samples[i].y-samples.front().y);
    }
    const double norm = std::hypot(direction.x,direction.y);
    if (!(norm>2.5*step)) return 0.;
    const double ux=direction.x/norm, uy=direction.y/norm;
    const auto normal = [&](double x,double y){return -uy*(x-center.x)+ux*(y-center.y);};
    double offset=0., sin_turn=0., chord_change=0.;
    // Check the actual polyline, not just the fitted points. A short sharp
    // projection segment does not veto line fitting unless it leaves the
    // corridor; a sustained turn is detected by overlapping two-step chords.
    for (const auto& p:spatial) {
      if (p.s>start+support) break;
      offset=std::max(offset,std::abs(normal(p.x,p.y)));
    }
    offset=std::max(offset,std::abs(normal(samples.back().x,samples.back().y)));
    std::array<Point,n-2> chords{};
    for (std::size_t i=0; i+2<n; ++i) {
      const double dx=samples[i+2].x-samples[i].x, dy=samples[i+2].y-samples[i].y;
      const double length=std::hypot(dx,dy);
      if (!(length>=1.7*step && ux*dx+uy*dy>0.)) return 0.;
      chords[i]={dx/length,dy/length};
      sin_turn=std::max(sin_turn,std::abs(-uy*chords[i].x+ux*chords[i].y));
      if (i>=2) {
        // Opposite bends can cancel against the global line. Check heading
        // changes between successive non-overlapping chords independently.
        const auto& a=chords[i-2];const auto& b=chords[i];
        chord_change=std::max(chord_change,std::abs(a.x*b.y-a.y*b.x));
      }
    }
    const auto fade = [](double v,double low,double high) {
      const double a=clamp((high-v)/(high-low),0.,1.);
      return a*a*(3.-2.*a);
    };
    const double blend=fade(offset,.012,.027)*fade(sin_turn,.045,.095)*
        fade(chord_change,.045,.070);
    if (blend<=1e-6) return 0.;
    const double yaw=std::atan2(uy,ux);
    for (int i=0; i<tracking.count; ++i) {
      const auto& p=tracking.points[i];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.yaw) ||
          !std::isfinite(p.curvature) || !(std::abs(wrap(yaw-p.yaw))<.25) ||
          !(std::abs(normal(p.x,p.y))<=.027)) return 0.;
    }
    for (int i=0; i<tracking.count; ++i) {
      auto& p=tracking.points[i];
      const double shift=blend*normal(p.x,p.y);
      p.x+=uy*shift;
      p.y-=ux*shift;
      p.yaw=wrap(p.yaw+blend*wrap(yaw-p.yaw));
      p.curvature*=1.-blend;
    }
    return blend;
  }
};

} // namespace research_control
