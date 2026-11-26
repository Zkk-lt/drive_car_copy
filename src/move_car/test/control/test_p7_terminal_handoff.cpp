#include "control/p7_terminal_handoff.h"

#include <cassert>
#include <iostream>

int main() {
    namespace uc = unified_control;
    uc::TerminalHandoff handoff;
    uc::ControlRequest q;
    q.mode = uc::ControlMode::NmpcFixed;
    q.guard = {true, true, "/control_research/p5_test/command", "p5_terminal_unit"};
    q.now = 10.;
    q.dt = 1. / 30.;
    q.history_valid = true;
    q.limits.speed = .4;
    q.limits.deceleration = .35;
    q.limits.jerk = .8;
    const auto prefix_target=[&](double arc) {
        return uc::TerminalHandoff::prefixPreviewTarget(.4,arc,.119,.4,q.limits);
    };
    assert(prefix_target(2.3)==.4);
    assert(prefix_target(1.10)<.4 && prefix_target(.85)<prefix_target(1.10));
    assert(prefix_target(.10)==0.);
    const double target=prefix_target(.85);
    const double reaction=1.4+(q.limits.acceleration+q.limits.deceleration)/q.limits.jerk;
    assert(std::abs(target*reaction+target*target/(2.*q.limits.deceleration)-(.85-.119))<1e-9);
    q.previous = {.30, .04};
    q.previous_acceleration = -.16;
    q.legacy = {.4, .05};
    q.state.valid = q.state.causal = true;
    q.state.stamp = q.state.receipt_stamp = q.now;
    q.state.speed = .32;
    q.reference = {q.now, 1, true, false};
    uc::ControlDecision nominal;
    nominal.command = {.296, .04};
    nominal.selected_nmpc = true;
    assert(handoff.apply(q, nominal, .27, .10, .2, true, .4, .4).selected_nmpc);
    auto fallback = uc::ControlArbiter::decide(q, nullptr);
    auto out = handoff.apply(q, fallback, .27, .10, .2, true, .4, .4);
    assert(handoff.latched() && out.terminal_handoff && out.command.speed < q.previous.speed);
    for(int i=0; i<180; ++i) {
        const double a = (out.command.speed - q.previous.speed)/q.dt;
        assert(a <= 1e-10 && a >= -q.limits.deceleration-1e-9);
        assert(std::abs(a-q.previous_acceleration) <= q.limits.jerk*q.dt+1e-9);
        q.previous = out.command;
        q.previous_acceleration = a;
        q.now += q.dt;
        q.reference = {q.now, static_cast<std::uint64_t>(i+2), true, false};
        q.state.stamp = q.state.receipt_stamp = q.now;
        q.state.speed = q.previous.speed;
        out = handoff.apply(q, fallback, .15, .10, .2, true, .4, .4);
        assert(out.terminal_handoff);
    }
    assert(out.command.speed <= 1e-7);
    // Many publications of one path revision, a future path, stale feedback
    // and an unverified extension each fail to release the terminal hold.
    q.reference = {q.now+.05, 200, true, false};
    assert(handoff.apply(q, fallback, 2., .10, .2, true, .4, .4).terminal_handoff);
    q.reference.stamp = q.now;
    q.state.receipt_stamp -= .5;
    assert(handoff.apply(q, fallback, 2., .10, .2, true, .4, .4).terminal_handoff);
    q.state.receipt_stamp = q.now;
    for(int i=0; i<5; ++i)
        assert(handoff.apply(q, fallback, 2., .10, .2, true, .4, .4).terminal_handoff);
    assert(handoff.latched());
    q.reference.sequence = 201;
    assert(handoff.apply(q, fallback, 2., .10, .2, false, .4, .4).terminal_handoff);
    for(int i=202; i<=204; ++i) {
        q.reference.sequence = i;
        assert(handoff.apply(q, fallback, 2., .10, .2, true, .4, .4).terminal_handoff);
    }
    assert(!handoff.latched());
    q.previous = {.30, .04};
    q.previous_acceleration = -.12;
    q.history_valid = true;
    assert(handoff.apply(q, fallback, .25, .10, .2, true, .4, .4).terminal_handoff);
    q.history_valid = false;
    assert(handoff.apply(q, fallback, .25, .10, .2, false, .4, .4).safety_stop);
    q.history_valid = true;
    out = handoff.apply(q, fallback, .25, .10, .2, true, .1, .4);
    assert(out.command.speed == .1 && std::string(out.reason) == "terminal_hard_limit_override");
    q.previous_acceleration = (out.command.speed-q.previous.speed)/q.dt;
    q.previous = out.command;
    out = handoff.apply(q, fallback, .25, .10, .2, true, .4, .4);
    assert((out.command.speed-q.previous.speed)/q.dt >= -q.limits.deceleration-1e-9);
    assert(std::string(out.reason) == "terminal_hard_limit_override");
    q.safety_stop = true;
    out = handoff.apply(q, uc::ControlArbiter::decide(q), .25, .10, .2, true, .4, .4);
    assert(out.safety_stop && out.command.speed == 0. && !out.terminal_handoff && handoff.latched());
    q.mode = uc::ControlMode::NmpcShadow;
    assert(!handoff.apply(q, fallback, .25, .10, .2, true, .4, .4).terminal_handoff);

    uc::TerminalHandoff rolling;
    q.mode = uc::ControlMode::NmpcFixed;
    q.safety_stop = false;
    q.previous = {.25, .04};
    q.previous_acceleration = -.12;
    q.state.speed = .24;
    q.reference.sequence = 300;
    const auto moving_brake = rolling.apply(q, fallback, .4, .10, .2, true, .4, .4);
    assert(rolling.latched() && moving_brake.command.speed < q.previous.speed);
    // New ROS publication numbers alone do not carry recovery authority.
    for (int i=301; i<=304; ++i) {
        q.reference.sequence=i;
        assert(rolling.apply(q, fallback, 2.1, .10, .2, true, .4, .4).terminal_handoff);
    }
    assert(rolling.latched());
    q.reference.sequence=305;
    assert(rolling.apply(q, fallback, 2.1, .10, .2, true, .1, .4, true).terminal_handoff);
    assert(rolling.latched());
    for (int i=306; i<=308; ++i) {
        q.reference.sequence=i;
        out=rolling.apply(q, fallback, 2.1, .10, .2, true, .4, .4, true);
        assert(out.terminal_handoff && out.command.speed<q.previous.speed);
    }
    assert(!rolling.latched() && std::string(out.reason)=="terminal_rejoin_brake_release");
    uc::TerminalHandoff preview_handoff;
    q.previous={.25,.04};
    q.previous_acceleration=-.12;
    q.state.speed=.24;
    auto preview_braking=fallback;
    preview_braking.command.speed=q.previous.speed+q.previous_acceleration*q.dt;
    auto preview=preview_handoff.apply(q,preview_braking,.44,.10,.2,true,.4,.4,true,true);
    assert(!preview.terminal_handoff && !preview_handoff.latched());
    preview=preview_handoff.apply(q,preview_braking,.30,.10,.2,true,.4,.4,true,true);
    assert(preview.terminal_handoff && preview_handoff.latched());

    // Field-scale usable prefix. A failed solve with adequate measured
    // stopping space should continue following as the preview slows down.
    uc::TerminalHandoff short_following;
    q.dt = .025;
    q.previous = {.22, .04};
    q.previous_acceleration = -.08;
    q.state.speed = .22;
    q.reference.sequence = 400;
    const uc::PrefixTrackingSupport verified{true,q.now-.1,.10};
    auto short_fallback = fallback;
    short_fallback.command = {.219,.05};
    out=short_following.apply(q,short_fallback,.65,.10,.4,true,.26,.4,false,false,verified);
    assert(out.bounded_prefix_following && !short_following.latched() && !out.selected_nmpc);
    const double applied=(out.command.speed-q.previous.speed)/q.dt;
    assert(out.command.speed>0. && out.command.speed<=short_fallback.command.speed);
    assert(std::abs(applied-q.previous_acceleration)<=q.limits.jerk*q.dt+1e-8);
    out=short_following.apply(q,short_fallback,.18,.10,.4,true,.26,.4,false,false,verified);
    assert(out.terminal_handoff && short_following.latched() && out.command.speed<q.previous.speed);

    // Stop release requires progress after the actual latch, independent
    // of how often the old prefix is republished.
    const double stopped_at=q.now;
    q.now+=.7;
    q.state.stamp=q.state.receipt_stamp=q.now;
    q.reference.stamp=q.now;
    q.previous.speed=q.previous_acceleration=q.state.speed=0.;
    for(int i=401;i<=404;++i) {
        q.reference.sequence=i;
        out=short_following.apply(q,short_fallback,.65,.10,.4,true,.26,.4,true,false,verified);
        assert(out.terminal_handoff && out.command.speed==0. && short_following.latched());
    }
    const uc::PrefixTrackingSupport recovered{true,stopped_at+.4,.10};
    q.reference.sequence=405;
    q.reference.braking=true;
    out=short_following.apply(q,uc::ControlArbiter::decide(q),.65,.10,.4,true,.26,.4,true,false,recovered);
    assert(out.safety_stop && short_following.latched());
    q.reference.braking=false;
    for(int i=406;i<=408;++i) {
        q.reference.sequence=i;
        out=short_following.apply(q,short_fallback,.65,.10,.4,true,.26,.4,true,false,recovered);
        assert(out.terminal_handoff && out.command.speed==0.);
    }
    assert(!short_following.latched() && std::string(out.reason)=="terminal_short_prefix_release");

    // The softened follower cannot override a newly lower hard cap,
    // a stale geometry result, or a measured speed beyond stopping range.
    q.previous={.22,.04};
    q.previous_acceleration=-.08;
    q.state.speed=.22;
    for(int i=0;i<3;++i) {
        uc::TerminalHandoff denied;
        if(i==2)q.state.speed=.42;
        out=denied.apply(q,short_fallback,.60,.10,.4,i!=1,i==0?.10:.26,.4,false,false,recovered);
        assert(!out.bounded_prefix_following && denied.latched());
        if(i==0)assert(out.command.speed<=.10);
    }
    uc::TerminalHandoff ease_brake;
    q.state.speed=.22;
    assert(ease_brake.apply(q,short_fallback,.18,.10,.4,true,.26,.4,false,false,recovered).terminal_handoff);
    const double braking_stamp=q.now;
    q.now+=.2;
    q.state.stamp=q.state.receipt_stamp=q.reference.stamp=q.now;
    ++q.reference.sequence;
    q.previous={.10,.04};
    q.previous_acceleration=-.15;
    q.state.speed=.12;
    out=ease_brake.apply(q,short_fallback,.63,.10,.4,true,.26,.4,true,false,
                        {true,braking_stamp+.1,.10});
    const double eased_a=(out.command.speed-q.previous.speed)/q.dt;
    assert(ease_brake.latched() && out.terminal_handoff);
    assert(std::abs(eased_a-(q.previous_acceleration+q.limits.jerk*q.dt))<1e-9);
    assert(out.command.speed<=q.previous.speed);
    std::cout << "terminal handoff, jerk, frozen/future input, recovery, and stop priority passed\n";
}
