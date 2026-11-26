#pragma once

#include "unified_control.h"

namespace unified_control {

struct PrefixTrackingSupport {
    bool verified = false;
    double progress_stamp = 0.;
    double restart_speed = 0.;
};

class TerminalHandoff {
    static constexpr double stopped_speed = .015; // engineering wheel-speed proxy, m/s
    static constexpr int recovery_frames = 3;
    bool approaching_ = false, latched_ = false;
    int fresh_frames_ = 0;
    std::uint64_t stop_revision_ = 0, last_fresh_revision_ = 0;
    double recovery_distance_ = 0.;
    double stop_stamp_ = 0.;

    static double stopDistance(double v, double a, double response, const Limits& limits) {
        // Distance for the reference reserve and decision lead time. The
        // longitudinal response allowance is frozen from the scene model.
        return v * (response + (std::max(0., a) + limits.deceleration) / limits.jerk) +
               v * v / (2. * limits.deceleration);
    }

    static bool continuePrefix(const ControlRequest& r, ControlDecision& d, double arc,
                               double reserve, double response, double hard_cap) {
        if (!finiteCommand(d.command) ||
            std::abs(d.command.steering) > r.limits.steering ||
            std::abs(d.command.steering-r.previous.steering) > r.limits.steering_rate*r.dt+1e-7)
            return false;
        const double jdt = r.limits.jerk*r.dt;
        const double min_a = std::max(-r.limits.deceleration,r.previous_acceleration-jdt);
        const double max_a = std::min(r.limits.acceleration,r.previous_acceleration+jdt);
        if (min_a > max_a) return false;
        const double hard_speed = std::min({r.limits.speed,hard_cap,d.command.speed});
        const double preview = prefixPreviewTarget(hard_speed,arc,reserve,response,r.limits);
        const double speed = std::min(r.previous.speed+r.dt*max_a,
                                     std::max(preview,r.previous.speed+r.dt*min_a));
        if (speed <= 1e-7 || speed > hard_speed+1e-9) return false;
        const double next_a = (speed-r.previous.speed)/r.dt;
        const double moving_speed = std::max({r.state.speed,r.previous.speed,speed});
        // Check actual approach speed, rather than the cruise request. The
        // soft preview can be exceeded while jerk-limited braking catches up
        // only when the confirmed arc still contains the complete stop.
        if (arc <= std::max(.10,reserve)+stopDistance(moving_speed,
                std::max(r.previous_acceleration,next_a),response+2.*r.dt,r.limits))
            return false;
        d.command.speed = speed;
        d.reason = "short_prefix_bounded_following";
        d.bounded_prefix_following = true;
        return true;
    }

public:
    bool latched() const { return latched_; }

    static double prefixPreviewTarget(double speed,double arc,double reserve,double response,
                                      const Limits& limits) {
        if(!validLimits(limits)||!std::isfinite(speed)||speed<0.||
           !std::isfinite(arc)||!std::isfinite(reserve)||reserve<0.||
           !std::isfinite(response)||response<0.)return speed;
        // Invert the reconstruction prefix stopping rule. Use its one-second
        // reaction reserve and the frozen longitudinal response allowance.
        const double reaction=1.+response+(limits.acceleration+limits.deceleration)/limits.jerk;
        const double distance=std::max(0.,arc-std::max(.10,reserve));
        const double term=limits.deceleration*reaction;
        const double bound=2.*limits.deceleration*distance/
            (std::sqrt(term*term+2.*limits.deceleration*distance)+term);
        return std::min(speed,bound);
    }

    ControlDecision apply(const ControlRequest& r, ControlDecision d, double usable_arc,
                          double reserve, double response, bool fresh_geometry,
                          double hard_speed_cap, double nominal_speed_cap,
                          bool advancing_reconnection=false, bool prefix_preview_active=false,
                          PrefixTrackingSupport prefix={}) {
        if (r.mode == ControlMode::LegacySlqr || r.mode == ControlMode::NmpcShadow ||
            (!r.guard.permits() && !r.supervised.enabled)) return d;

        const bool limits_valid = validLimits(r.limits) && std::isfinite(reserve) && reserve >= 0. &&
            std::isfinite(response) && response >= 0. && std::isfinite(r.dt) && r.dt > 0. && r.dt <= .25 &&
            std::isfinite(hard_speed_cap) && hard_speed_cap >= 0. &&
            std::isfinite(nominal_speed_cap) && nominal_speed_cap >= 0.;
        const bool history_valid = r.history_valid && finiteCommand(r.previous) && r.previous.speed >= 0. &&
            std::isfinite(r.previous_acceleration);
        const bool geometry_valid = r.reference.valid && r.reference.sequence > 0 &&
            std::isfinite(r.reference.stamp) && r.reference.stamp <= r.now &&
            r.now - r.reference.stamp <= r.maximum_reference_age &&
            std::isfinite(usable_arc) && usable_arc >= 0.;
        const bool observation_valid = r.state.valid && r.state.causal && std::isfinite(r.state.speed) &&
            std::isfinite(r.state.stamp) && r.state.stamp <= r.now && r.now - r.state.stamp <= r.maximum_state_age &&
            std::isfinite(r.state.receipt_stamp) && r.state.receipt_stamp <= r.now &&
            r.now - r.state.receipt_stamp <= r.maximum_state_age;
        const bool armed = r.guard.permits() || r.supervised.armed(r.steady_now);
        const bool verified_prefix = prefix.verified && fresh_geometry && geometry_valid &&
            limits_valid && history_valid && observation_valid && r.state.speed >= 0. &&
            !r.reference.braking && !d.safety_stop && !r.safety_stop && !r.following_stop;

        if (!latched_ && armed) {
            if (geometry_valid && limits_valid && history_valid) {
                const double measured = observation_valid ? std::max(0., r.state.speed) : 0.;
                const double speed = std::max(r.previous.speed, measured);
                // Enter early enough for the falling reference-horizon cap.
                // The calibrated operating cap is independent of the moving
                // safety cap. A held but usable suffix can request braking.
                const double operating_speed = std::min(r.limits.speed, nominal_speed_cap);
                const double approach_speed = std::max(speed, operating_speed);
                approaching_ = speed > stopped_speed &&
                    usable_arc <= reserve + stopDistance(approach_speed, r.previous_acceleration,
                                                         response + r.dt, r.limits);
                const bool reverses_braking = r.previous_acceleration < -r.limits.jerk * r.dt &&
                    d.command.speed > r.previous.speed;
                // When a verified prefix is still advancing and its preview
                // already commands braking, a failed solve need not terminate
                // optimization. Recheck actual stopping space on every cycle.
                const double fallback_a=(d.command.speed-r.previous.speed)/r.dt;
                const bool continues_preview=prefix_preview_active&&advancing_reconnection&&
                    observation_valid&&!r.reference.braking&&!d.safety_stop&&fallback_a<0.&&
                    fallback_a>=-r.limits.deceleration-1e-7&&
                    std::abs(fallback_a-r.previous_acceleration)<=r.limits.jerk*r.dt+1e-7&&
                    usable_arc>reserve+stopDistance(speed,r.previous_acceleration,
                       response+2.*r.dt,r.limits);
                const bool prefix_continues = approaching_ && !d.selected_nmpc && verified_prefix &&
                    continuePrefix(r,d,usable_arc,reserve,response,hard_speed_cap);
                if (approaching_ && ((!d.selected_nmpc && !continues_preview && !prefix_continues) ||
                                    (reverses_braking && !prefix_continues))) {
                    latched_ = true;
                    stop_stamp_ = r.now;
                }
                if (approaching_) {
                    stop_revision_ = r.reference.sequence;
                    recovery_distance_ = reserve + stopDistance(operating_speed, 0., response + r.dt, r.limits);
                }
            } else if (approaching_) {
                latched_ = true;
                stop_stamp_ = r.now;
            }
        }
        if (!latched_) return d;

        const bool at_rest = r.previous.speed <= 1e-7 && observation_valid &&
            std::abs(r.state.speed) <= stopped_speed;
        // A rolling reference can regain a well-supported continuation
        // before the vehicle stops. Keep braking through this publication;
        // the next controller starts from the actually committed transition.
        const bool controlled_rejoin = advancing_reconnection && observation_valid &&
            r.state.speed >= 0. && std::max(r.state.speed,r.previous.speed) <= hard_speed_cap &&
            r.previous_acceleration <= 0. && r.previous_acceleration >= -r.limits.deceleration;
        const double restart_speed = std::min({prefix.restart_speed, hard_speed_cap, r.limits.speed});
        const bool short_rejoin = verified_prefix && advancing_reconnection &&
            std::isfinite(prefix.progress_stamp) && prefix.progress_stamp > stop_stamp_ &&
            prefix.progress_stamp <= r.now && std::isfinite(restart_speed) && restart_speed > stopped_speed &&
            usable_arc > std::max(.10,reserve)+stopDistance(
                std::max({r.state.speed,r.previous.speed,restart_speed}),r.previous_acceleration,
                1.+response+2.*r.dt,r.limits);
        const bool extended = fresh_geometry && geometry_valid && limits_valid && history_valid && armed && !d.safety_stop &&
            (at_rest || controlled_rejoin) &&
            r.reference.sequence > stop_revision_ && (usable_arc > recovery_distance_ || short_rejoin);
        bool released=false;
        if (!extended) {
            fresh_frames_ = 0;
            last_fresh_revision_ = 0;
        } else if (r.reference.sequence > last_fresh_revision_) {
            last_fresh_revision_ = r.reference.sequence;
            if (++fresh_frames_ == recovery_frames) {
                *this = TerminalHandoff{};
                released=true;
            }
        }
        // Chassis errors, collision, command lease and braking-only reference
        // retain their original priority. Keep the terminal latch through them.
        if (d.safety_stop) return d;

        d.selected_nmpc = false;
        d.candidate_sequence = 0;
        d.terminal_handoff = true;
        if (!limits_valid || !history_valid) {
            d.command.speed = 0.;
            d.safety_stop = true;
            d.reason = "terminal_history_or_limits_invalid_stop";
            return d;
        }
        const double v = r.previous.speed;
        const double jdt = r.limits.jerk * r.dt;
        const double cap = std::min({v, r.limits.speed, hard_speed_cap});
        // Hold enough speed to unwind deceleration to zero at the jerk bound.
        // With b=-a >= 0 require v-dt*b >= b^2/(2*j).
        const double unwind = v > 0. ?
            2. * v / (r.dt + std::sqrt(r.dt * r.dt + 2. * v / r.limits.jerk)) : 0.;
        // A verified post-brake continuation can unwind deceleration
        // during the confirmation window. The command still cannot
        // accelerate, and a failed next verification resumes braking.
        const double goal = extended && short_rejoin ? 0. :
            -std::min(r.limits.deceleration, unwind);
        const double comfort_a = std::max(r.previous_acceleration - jdt,
                                          std::min(r.previous_acceleration + jdt, goal));
        // A previous hard-cap override can put acceleration outside the
        // comfort interval. Restore the acceleration bound at the first
        // opportunity instead of propagating excessive braking.
        const double a = std::max(-r.limits.deceleration, std::min(0., comfort_a));
        const double speed = std::max(0., std::min(cap, v + a * r.dt));
        const double applied_a = (speed - v) / r.dt;
        d.command.speed = speed;
        const double rate_step = r.limits.steering_rate * r.dt;
        const bool stationary_hold = v <= 1e-7 && observation_valid &&
            std::abs(r.state.speed) <= stopped_speed;
        const double steering = stationary_hold ? r.previous.steering : r.legacy.steering;
        d.command.steering = std::max(-r.limits.steering, std::min(r.limits.steering,
            std::max(r.previous.steering - rate_step, std::min(r.previous.steering + rate_step, steering))));
        if (std::abs(applied_a - r.previous_acceleration) > jdt + 1e-7 ||
            applied_a < -r.limits.deceleration - 1e-7) {
            d.reason = "terminal_hard_limit_override";
        } else {
            d.reason = released && short_rejoin ? "terminal_short_prefix_release" :
                (released && !at_rest ? "terminal_rejoin_brake_release" :
                 (speed <= 1e-7 ? "terminal_hold" : "terminal_braking_handoff"));
        }
        return d;
    }
};

} // namespace unified_control
