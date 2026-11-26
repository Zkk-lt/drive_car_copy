#pragma once

#include <cstdint>
#include "unified_control.h"

namespace unified_control {

// A shorter release period provides scheduling headroom. This bound remains
// the original 30 Hz deadline, measured at the actual publication boundary.
class P7PublicationGuard {
    static constexpr std::uint64_t maximum_gap_ns=33333334;
    // One missed 40 Hz release. The NMPC deadline stays at 33.33 ms.
    static constexpr std::uint64_t single_release_gap_ns=50000000;
    static constexpr unsigned recovery_intervals=12;
    unsigned clean_=0;
    bool blocked_=true, last_release_late_=false;
public:
    enum class Release { OnTime, BoundedBrake, Stop };
    static bool deadlineMissed(std::uint64_t now,std::uint64_t last) {
        return last!=0&&now>last&&now-last>maximum_gap_ns;
    }
    Release assess(std::uint64_t now,std::uint64_t last,bool stopped,bool bounded_brake_available) {
        if(last==0 || now<=last || deadlineMissed(now,last)){
            const bool single=!blocked_&&!last_release_late_&&last!=0&&now>last&&
                now-last<=single_release_gap_ns&&bounded_brake_available;
            last_release_late_=true;
            if(single)return Release::BoundedBrake;
            requireStoppedRecovery();
            return Release::Stop;
        }
        last_release_late_=false;
        if(blocked_){
            if(!stopped)clean_=0;
            else if(++clean_>=recovery_intervals)blocked_=false;
        }
        return blocked_?Release::Stop:Release::OnTime;
    }
    void requireStoppedRecovery() {
        clean_=0;
        blocked_=true;
    }
    bool allow(std::uint64_t now,std::uint64_t last,bool stopped) {
        return assess(now,last,stopped,false)==Release::OnTime;
    }
    static bool brakingCommand(const ControlRequest& r,double elapsed,double arc,
                               double reserve,double response,Command& out) {
        if(!validLimits(r.limits)||!r.history_valid||!r.state.valid||!r.state.causal||
           !r.reference.valid||r.reference.braking||r.safety_stop||r.following_stop||
           !finiteCommand(r.previous)||r.previous.speed<=.015||
           std::abs(r.previous.steering)>r.limits.steering||
           !std::isfinite(r.previous_acceleration)||!std::isfinite(r.speed_cap)||r.speed_cap<0.||
           !std::isfinite(r.state.speed)||r.state.speed<0.||
           !std::isfinite(r.state.yaw_rate)||std::abs(r.state.speed*r.state.yaw_rate)>r.limits.lateral_acceleration||
           !std::isfinite(r.dt)||r.dt<=0.||r.dt>.25||!std::isfinite(elapsed)||elapsed<=0.||
           !std::isfinite(reserve)||reserve<0.||!std::isfinite(response)||response<0.)return false;
        const double v=std::max(r.previous.speed,r.state.speed);
        const double reaction=response+elapsed+
            (std::max(0.,r.previous_acceleration)+r.limits.deceleration)/r.limits.jerk;
        if(v>std::min(r.speed_cap,r.limits.speed)||!std::isfinite(arc)||
           arc<=reserve+v*reaction+v*v/(2.*r.limits.deceleration))return false;
        // Intersect the physical publication interval and the interval used
        // by the command ledger. Choose the strongest reachable deceleration.
        double low=0.,high=std::min(r.speed_cap,r.limits.speed);
        for(double dt:{elapsed,r.dt}){
            const double step=r.limits.jerk*dt;
            low=std::max(low,r.previous.speed+dt*std::max(-r.limits.deceleration,r.previous_acceleration-step));
            high=std::min(high,r.previous.speed+dt*std::min(r.limits.acceleration,r.previous_acceleration));
        }
        if(low>high)return false;
        out={low,r.previous.steering};
        return true;
    }
};

}
