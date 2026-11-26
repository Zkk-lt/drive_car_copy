#pragma once

#include "unified_control.h"

namespace unified_control {

// Shape only the first strong reversal of a healthy NMPC-to-SLQR handoff.
// The result stays between the last published angle and the current SLQR
// proposal. It does not replay the rejected trajectory or affect speed.
inline bool shapeP7SteeringHandoff(const ControlRequest& request,
                                   const PublishedCommandRecord& older,
                                   const PublishedCommandRecord& previous,
                                   std::uint64_t steady_now,
                                   double feedforward_steering,
                                   double active_steer_limit,
                                   ControlDecision& decision) {
    if (!request.history_valid || !request.reference.valid || request.reference.braking ||
        request.safety_stop || request.following_stop || !decision.fallback ||
        decision.selected_nmpc || decision.safety_stop || decision.terminal_handoff ||
        decision.bounded_prefix_following || decision.legacy_contract_warning ||
        !previous.selected_nmpc || !previous.ros_publish_returned || !older.ros_publish_returned ||
        previous.sequence != request.history_sequence || older.sequence + 1 != previous.sequence ||
        previous.epoch != request.epoch || older.epoch != request.epoch ||
        older.publish_steady_start_ns == 0 ||
        previous.publish_steady_start_ns <= older.publish_steady_start_ns ||
        steady_now <= previous.publish_steady_start_ns ||
        !validLimits(request.limits) || !std::isfinite(request.dt) || request.dt <= 0. ||
        !std::isfinite(feedforward_steering) || !std::isfinite(active_steer_limit) ||
        active_steer_limit <= 0. ||
        !finiteCommand(older.command) || !finiteCommand(previous.command) ||
        !finiteCommand(decision.command) || previous.command.speed <= .02 ||
        decision.command.speed <= .02) return false;

    const double previous_dt =
        (previous.publish_steady_start_ns - older.publish_steady_start_ns) * 1e-9;
    const double elapsed = (steady_now - previous.publish_steady_start_ns) * 1e-9;
    constexpr double deadline = 1. / 30.;
    if (previous_dt < .01 || previous_dt > deadline || elapsed < .01 ||
        elapsed > deadline || request.dt > deadline) return false;

    const double limit = std::min(active_steer_limit, request.limits.steering);
    const double last = previous.command.steering;
    if (std::abs(last) > limit || std::abs(decision.command.steering) > limit) return false;

    const double prior_rate = (last - older.command.steering) / previous_dt;
    const double step = decision.command.steering - last;
    const double goal_rate = step / request.dt;
    constexpr double material_rate = .20; // Strong reversal threshold in the run audit.
    if (prior_rate * goal_rate >= 0. || std::abs(prior_rate) < material_rate ||
        std::abs(goal_rate) < material_rate) return false;

    // Preserve a purposeful transition toward a material feedforward move
    // such as turn entry or unwinding on turn exit.
    const double feedforward_step = feedforward_steering - last;
    if (step * feedforward_step > 0. && std::abs(feedforward_step) >= .035) return false;

    constexpr double first_reverse_rate = .40; // Near continuous flat8 RMS, not a physical limit.
    const double maximum_step =
        std::min(first_reverse_rate, request.limits.steering_rate) * std::min(request.dt, elapsed);
    if (std::abs(step) <= maximum_step) return false;
    decision.command.steering = last + std::copysign(maximum_step, step);
    return true;
}

} // namespace unified_control
