#define main control_pub_node_main
#include "../src/control_pub.cpp"
#undef main
#include <iostream>

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

int main() {
    PreviewGeometryTracker tracker;
    UnifiedTrackingSignal tracking;
    PreviewGeometrySignal preview;
    preview.valid = true;
    preview.geometric_curve_evidence = true;
    for (double sign : {-1.0, 1.0}) {
        preview.kappa_cmd = sign * 0.4;
        preview.front_axle_e_ct = sign * 0.1;
        const auto output = tracker.compute(tracking, preview, 0.25, 0.0, 0.0,
                                             0.0, -sign * 0.1, sign * 0.2,
                                             1.0, true, false);
        const double limit = std::min(0.95 * std::fabs(output.delta_ff),
                                      0.05 + 0.35 * std::fabs(output.delta_ff));
        require(std::fabs(output.slqr_feedback_applied + sign * limit) < 1e-9,
                "previous-version SLQR cancellation bound lost");
        require(output.delta_ref * sign >= 0.0,
                "opposing feedback reversed a trusted curve");
        require(std::fabs(output.delta_lateral) <=
                    0.05 + 0.35 * std::fabs(output.delta_ff) + 1e-9,
                "previous-version lateral cancellation bound lost");
        require(std::fabs(output.delta_ref) <= tracker.deltaMax(),
                "total steering limit lost");
        require(output.diagnostic_branch == "slqr_replaced_feedforward",
                "diagnostic steering branch does not match executed branch");
    }
    PathBuffer path;
    path.valid = true;
    for (int i = 0; i <= 40; ++i) {
        PathPoint point;
        point.s = point.x = i * 0.05;
        path.points.push_back(point);
    }
    for (double speed : {0.03, 0.1, 0.3, 0.6}) {
        const auto signal = tracker.observe(path, tracking, speed);
        require(signal.valid, "straight path invalid");
        require(signal.steer_delay_distance <= 0.14 + 1e-9,
                "configured delay distance cap bypassed");
        require(std::fabs(signal.kappa_cmd) < 1e-9, "straight path gained curvature");
    }
    std::cout << "left/right feedback, steering limit, preview cap and straight path PASS\n";
}
