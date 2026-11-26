#include "control/p7_steering_handoff.h"
#include <cassert>
#include <iomanip>
#include <iostream>

using namespace unified_control;

struct Handoff {
    ControlRequest request;
    PublishedCommandRecord older, previous;
    ControlDecision decision;
    std::uint64_t now = 1050000000;

    Handoff() {
        request.dt = .025;
        request.history_valid = request.reference.valid = true;
        request.epoch = 1;
        request.history_sequence = 2;
        older.epoch = previous.epoch = 1;
        older.sequence = 1;
        previous.sequence = 2;
        older.ros_publish_returned = previous.ros_publish_returned = true;
        previous.selected_nmpc = true;
        older.publish_steady_start_ns = 1000000000;
        previous.publish_steady_start_ns = 1025000000;
        older.command = {.35, 0.};
        previous.command = {.35, .025};
        decision.command = {.34, 0.};
        decision.fallback = true;
    }

    bool apply(double feedforward = .025, double limit = .32) {
        return shapeP7SteeringHandoff(request, older, previous, now, feedforward, limit, decision);
    }
};

int main(int argc, char**) {
    if (argc == 2) {
        // Evaluate paired command windows from a frozen real run.
        double feedforward, limit;
        Handoff row;
        std::cout << std::setprecision(17);
        while (std::cin >> row.older.publish_steady_start_ns >> row.previous.publish_steady_start_ns >>
               row.now >> row.request.dt >> row.older.command.steering >> row.previous.command.steering >>
               row.decision.command.steering >> row.previous.command.speed >> row.decision.command.speed >>
               feedforward >> limit) {
            const bool changed = row.apply(feedforward, limit);
            std::cout << changed << ' ' << row.decision.command.steering << '\n';
        }
        return std::cin.eof() ? 0 : 1;
    }

    Handoff reverse;
    assert(reverse.apply());
    assert(std::abs(reverse.decision.command.steering - .015) < 1e-12);
    assert(reverse.decision.command.speed == .34 && !reverse.decision.selected_nmpc);
    Handoff same_direction;
    same_direction.decision.command.steering = .05;
    assert(!same_direction.apply());
    Handoff planned_turn;
    assert(!planned_turn.apply(-.10));
    Handoff tightened;
    assert(!tightened.apply(.025, .01));
    Handoff stale;
    stale.now += 10000000;
    assert(!stale.apply());
    Handoff gap;
    gap.older.sequence = 0;
    assert(!gap.apply());
    Handoff continuous_legacy;
    continuous_legacy.previous.selected_nmpc = false;
    assert(!continuous_legacy.apply());
    Handoff safety;
    safety.decision.safety_stop = true;
    assert(!safety.apply());
    safety.decision.safety_stop = false;
    safety.request.reference.braking = true;
    assert(!safety.apply());
    safety.request.reference.braking = false;
    safety.decision.terminal_handoff = true;
    assert(!safety.apply());
    safety.decision.terminal_handoff = false;
    safety.decision.bounded_prefix_following = true;
    assert(!safety.apply());
    std::cout << "P7 first-reversal shaping and bypasses passed\n";
}
