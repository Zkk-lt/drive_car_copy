#define main control_pub_node_main
#include "../src/control_pub.cpp"
#undef main
#include <iostream>

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

int main() {
    FollowDistanceSupervisor supervisor;
    LeadTargetState lead;
    lead.valid = true;
    for (double speed : {0.0, 0.1, 0.3}) {
        for (double distance : {0.0, 0.49, 0.5, 0.75, 1.0, 1.5, 1.75, 2.0}) {
            lead.distance = distance;
            const auto result = supervisor.evaluate(lead, 0.4, speed);
            const double expected = std::max(0.0, 0.18 * (distance - 0.5) / 1.5);
            require(std::fabs(result.cap - expected) < 1e-9, "linear cap mismatch");
            require(result.hard_stop == (distance <= 0.5), "stop boundary mismatch");
        }
        for (double boundary : {0.5, 2.0, 2.3}) {
            lead.distance = boundary - 1e-7;
            const auto a = supervisor.evaluate(lead, 0.4, speed);
            lead.distance = boundary + 1e-7;
            const auto b = supervisor.evaluate(lead, 0.4, speed);
            require(std::fabs(a.cap - b.cap) < 1e-6, "boundary discontinuity");
        }
    }
    lead.distance = 1.9;
    require(supervisor.evaluate(lead, 0.05, 0.3).cap <= 0.05, "max speed exceeded");
    lead.valid = false;
    require(!supervisor.evaluate(lead, 0.4, 0.0).active, "lost-target behavior changed");
    std::cout << "linear envelope, stop boundary, continuity, speed cap, target loss PASS\n";
}
