#!/usr/bin/env python3
"""Read-only offline delay/lag fits. No ROS imports or controller parameter writes."""

import argparse
import bisect
import csv
from dataclasses import dataclass
import hashlib
import io
import json
import math
from pathlib import Path
import statistics
import sys


ROOT = Path(__file__).resolve().parents[3]
ANGLE_SCALES = {"rad": 1.0, "deg": math.pi / 180.0}
SPEED_SCALES = {"m/s": 1.0, "mm/s": 0.001, "km/h": 1.0 / 3.6}
MAX_DELAY = 0.7
MAX_TAU = 0.6


@dataclass
class Segment:
    ct: list
    steering: list
    speed_command: list
    t: list
    feedback: list
    wheel_speed: list
    suspect: list

    def clip(self, lo, hi):
        t = self.t[lo:hi]
        if not t:
            return None
        j = bisect.bisect_right(self.ct, t[0]) - 1
        k = bisect.bisect_right(self.ct, t[-1])
        # The command held at the boundary is known; no earlier events are reused.
        return Segment([t[0]] + self.ct[j + 1:k],
                       [self.steering[j]] + self.steering[j + 1:k],
                       [self.speed_command[j]] + self.speed_command[j + 1:k],
                       t, self.feedback[lo:hi], self.wheel_speed[lo:hi],
                       self.suspect[lo:hi])


def read_stream(path, kind, angle_unit="rad", speed_unit="m/s",
                min_speed=0.05, max_gap=0.25):
    """Preserve file order; invalid rows, stops, gaps, and resets break runs."""
    payload = Path(path).read_bytes()
    reader = csv.DictReader(io.StringIO(payload.decode("utf-8-sig")))
    columns = reader.fieldnames or []
    if len(columns) != len(set(columns)):
        raise ValueError("duplicate CSV columns in {}".format(path))
    if kind == "command":
        angle_col, speed_col = "cmd_delta", "cmd_v"
    else:
        angle_col = next((c for c in ("steer_scaled", "steer_feedback_for_control",
                                     "steer_feedback") if c in columns), None)
        speed_col = "wheel_speed"
    if any(c not in columns for c in ("stamp", angle_col, speed_col)):
        raise ValueError("{} lacks {} steering/speed/stamp columns".format(path, kind))
    rows = list(reader)
    periods, previous_stamp = [], None
    for row in rows:
        try:
            stamp = float(row["stamp"])
            if not math.isfinite(stamp):
                continue
        except (TypeError, ValueError):
            continue
        if previous_stamp is not None and stamp > previous_stamp:
            periods.append(stamp - previous_stamp)
        previous_stamp = stamp
    nominal_period = statistics.median(periods) if periods else None
    gap_limit = min(max_gap, 1.75 * nominal_period) if nominal_period else max_gap
    counts = dict(rows=0, invalid=0, stopped=0, gaps=0, time_resets=0,
                  duplicate_stamps=0)
    runs, run, previous, epoch = [], [], None, 0

    def flush():
        nonlocal run
        if run:
            runs.append((epoch, run))
        run = []

    for row in rows:
        counts["rows"] += 1
        try:
            t = float(row["stamp"])
            if not math.isfinite(t):
                raise ValueError("invalid time")
        except (TypeError, ValueError):
            flush()
            counts["invalid"] += 1
            continue
        if previous is not None:
            if t == previous:
                # Neither value at an ambiguous duplicate stamp is trustworthy.
                if run and run[-1][0] == t:
                    run.pop()
                flush()
                counts["duplicate_stamps"] += 1
                continue
            if t < previous:
                flush()
                epoch += 1
                counts["time_resets"] += 1
            elif t - previous > gap_limit:
                flush()
                counts["gaps"] += 1
        previous = t
        try:
            angle = float(row[angle_col]) * ANGLE_SCALES[angle_unit]
            speed = float(row[speed_col]) * SPEED_SCALES[speed_unit]
            suspect = float(row.get("steer_scale_suspect", "0"))
            if (row.get(None) or not all(map(math.isfinite, (angle, speed, suspect)))
                    or abs(angle) > math.pi / 2):
                raise ValueError("invalid values or implausible steering units")
        except (TypeError, ValueError):
            flush()
            counts["invalid"] += 1
            continue
        # Forward-only analysis avoids mixing reverse conventions or stop transitions.
        if speed <= min_speed:
            flush()
            counts["stopped"] += 1
            continue
        run.append((t, angle, speed, bool(suspect)))
    flush()
    return runs, dict(path=str(Path(path).resolve()), sha256=hashlib.sha256(payload).hexdigest(),
                      angle_column=angle_col, speed_column=speed_col,
                      input_angle_unit=angle_unit, input_speed_unit=speed_unit,
                      nominal_period_s=nominal_period, effective_gap_limit_s=gap_limit,
                      counts=counts, runs=len(runs))


def align_streams(commands, feedback):
    """Intersect valid runs without extrapolating beyond either run's last stamp."""
    segments = []
    for epoch, cr in commands:
        ct = [r[0] for r in cr]
        for fe, fr in feedback:
            if fe != epoch or fr[-1][0] < ct[0] or fr[0][0] > ct[-1]:
                continue
            rows = [r for r in fr if ct[0] <= r[0] <= ct[-1]]
            if len(rows) < 2:
                continue
            s = Segment(ct, [r[1] for r in cr], [r[2] for r in cr],
                        [r[0] for r in rows], [r[1] for r in rows],
                        [r[2] for r in rows], [r[3] for r in rows])
            segments.append(s.clip(0, len(rows)))
    return segments


def read_can_log(path, min_speed=0.05, max_gap=0.25):
    """Read one driver's appended process runs, using only its host steady clock."""
    payload = Path(path).read_bytes()
    rows = list(csv.reader(io.StringIO(payload.decode("utf-8-sig"))))
    required = {"steady_time", "new_command", "encoding_valid", "tx_attempted", "tx_result",
                "encoded_speed_mm_s", "encoded_steering_centideg", "tx_start_steady",
                "tx_end_steady", "fresh_valid", "rx_start_steady", "rx_end_steady",
                "feedback_speed_mm_s", "feedback_steering_centideg"}
    if not rows:
        raise ValueError("empty CAN diagnostic CSV")

    def header(values):
        if len(set(values)) != len(values) or not required.issubset(values):
            raise ValueError("CAN diagnostic CSV lacks required columns or has duplicate columns")
        return values

    columns = header(rows[0])
    records, periods, previous = [], [], None
    for values in rows[1:]:
        if not values:
            records.append(None)
            previous = None
            continue
        if values == columns or values[0] == "ros_time" or required.issubset(values):
            columns = header(values)
            records.append("header")
            previous = None
            continue
        if len(values) != len(columns):
            records.append(None)
            previous = None
            continue
        row = dict(zip(columns, values))
        records.append(row)
        try:
            stamp = float(row["steady_time"])
            if not math.isfinite(stamp):
                raise ValueError("invalid steady time")
        except ValueError:
            previous = None
            continue
        if previous is not None and stamp > previous:
            periods.append(stamp - previous)
        previous = stamp
    nominal_period = statistics.median(periods) if periods else None
    gap_limit = min(max_gap, 1.75 * nominal_period) if nominal_period else max_gap
    counts = dict(rows=0, repeated_headers=0, steady_resets=0, duplicate_stamps=0,
                  invalid_rows=0, logging_gaps=0, feedback_gaps=0, successful_tx=0,
                  failed_or_invalid_tx=0, idle_rows=0, nonfresh_rx=0, fresh_rx=0,
                  unknown_actuation_rx=0, stopped_rx=0, saturated_successful_tx=0)
    segments, active, held, previous = [], None, None, None

    def flush():
        nonlocal active
        if active is not None and len(active.t) >= 2:
            end = bisect.bisect_right(active.ct, active.t[-1])
            active.ct = active.ct[:end]
            active.steering = active.steering[:end]
            active.speed_command = active.speed_command[:end]
            segments.append(active)
        active = None

    def finite(row, key):
        value = float(row[key])
        if not math.isfinite(value) or value < 0:
            raise ValueError("invalid steady time")
        return value

    def flag(row, key):
        value = int(row[key])
        if value not in (0, 1):
            raise ValueError("invalid boolean")
        return value

    def decode(row, speed_key, angle_key):
        speed, angle = int(row[speed_key]), int(row[angle_key])
        if not (-32768 <= speed <= 32767 and -32768 <= angle <= 32767):
            raise ValueError("not signed int16 wire units")
        return speed / 1000.0, angle * math.pi / 18000.0

    for row in records:
        if row == "header":
            flush()
            held, previous = None, None
            counts["repeated_headers"] += 1
            continue
        counts["rows"] += 1
        if row is None:
            flush()
            held = None
            counts["invalid_rows"] += 1
            continue
        try:
            stamp = finite(row, "steady_time")
            if previous is not None:
                if stamp == previous:
                    flush()
                    held = None
                    counts["duplicate_stamps"] += 1
                    continue
                if stamp < previous:
                    flush()
                    held, previous = None, None
                    counts["steady_resets"] += 1
                elif stamp - previous > gap_limit:
                    flush()
                    held = None  # Missing rows might conceal a TX, not just an idle cycle.
                    counts["logging_gaps"] += 1
            rx_start, rx_end = finite(row, "rx_start_steady"), finite(row, "rx_end_steady")
            if not ((previous is None or previous <= rx_start) and rx_start <= rx_end <= stamp):
                raise ValueError("inconsistent same-process RX steady timestamps")
            new, attempted = flag(row, "new_command"), flag(row, "tx_attempted")
            valid, fresh = flag(row, "encoding_valid"), flag(row, "fresh_valid")
            result = int(row["tx_result"])
            successful = attempted == 1 and result == 1 and valid == 1
            if attempted:
                tx_start, tx_end = finite(row, "tx_start_steady"), finite(row, "tx_end_steady")
                if not (0 < tx_start <= tx_end <= rx_start
                        and (previous is None or previous <= tx_start)):
                    raise ValueError("inconsistent same-process TX steady timestamps")
            previous = stamp
            if successful:
                speed, angle = decode(row, "encoded_speed_mm_s", "encoded_steering_centideg")
                held = (tx_end, angle, speed)
                counts["successful_tx"] += 1
                counts["saturated_successful_tx"] += flag(row, "saturated") if "saturated" in row else 0
                if speed <= min_speed:
                    flush()
                elif active is not None:
                    active.ct.append(tx_end)
                    active.steering.append(angle)
                    active.speed_command.append(speed)
            elif new or attempted:
                flush()
                held = None
                counts["failed_or_invalid_tx"] += 1
            else:
                # encoding_valid=0 and tx_result=-2 are normal when there is no callback.
                counts["idle_rows"] += 1
            if not fresh:
                counts["nonfresh_rx"] += 1
                continue
            speed, angle = decode(row, "feedback_speed_mm_s", "feedback_steering_centideg")
            counts["fresh_rx"] += 1
            if held is None:
                counts["unknown_actuation_rx"] += 1
                continue
            if speed <= min_speed or held[2] <= min_speed:
                flush()
                counts["stopped_rx"] += 1
                continue
            if active is not None and rx_end - active.t[-1] > gap_limit:
                flush()
                counts["feedback_gaps"] += 1
            if active is None:
                active = Segment([held[0]], [held[1]], [held[2]], [], [], [], [])
            active.t.append(rx_end)
            active.feedback.append(angle)
            active.wheel_speed.append(speed)
            active.suspect.append(False)
        except (ValueError, OverflowError):
            flush()
            held = None
            counts["invalid_rows"] += 1
    flush()
    metadata = dict(path=str(Path(path).resolve()), sha256=hashlib.sha256(payload).hexdigest(),
                    mode="can_log", clock="same-process steady seconds, separated by process/run",
                    command_time_column="tx_end_steady", feedback_time_column="rx_end_steady",
                    command_columns=["encoded_speed_mm_s", "encoded_steering_centideg"],
                    feedback_columns=["feedback_speed_mm_s", "feedback_steering_centideg"],
                    speed_conversion="signed mm/s / 1000 -> m/s",
                    steering_conversion="signed centidegrees * pi / 18000 -> rad",
                    nominal_period_s=nominal_period, effective_gap_limit_s=gap_limit,
                    counts=counts, process_runs=1 + counts["repeated_headers"] + counts["steady_resets"])
    return segments, metadata


def chronological_split(segments, fraction=0.7):
    """One global prefix/suffix split, never random or independently per run."""
    cut = int(sum(len(s.t) for s in segments) * fraction)
    train, validation, offset = [], [], 0
    for s in segments:
        k = max(0, min(len(s.t), cut - offset))
        if k:
            train.append(s.clip(0, k))
        if k < len(s.t):
            validation.append(s.clip(k, len(s.t)))
        offset += len(s.t)
    return train, validation


def predict(ct, u, t, initial, delay, tau):
    """Exact first-order integration at every delayed ZOH command event."""
    if not t:
        return []
    j = bisect.bisect_right(ct, t[0] - delay) - 1
    if j < 0:
        raise ValueError("prediction needs command history for the full delay")
    y, left, result = initial, t[0], [initial]
    for right in t[1:]:
        while j + 1 < len(ct) and ct[j + 1] + delay <= right:
            event = ct[j + 1] + delay
            if tau > 0:
                y = u[j] + (y - u[j]) * math.exp(-(event - left) / tau)
            left = event
            j += 1
        if tau > 0:
            y = u[j] + (y - u[j]) * math.exp(-(right - left) / tau)
        else:
            y = u[j]
        left = right
        result.append(y)
    return result


def prepare(segments, channel):
    prepared = []
    for s in segments:
        # Common history reserve for all candidates; measured seed is never scored.
        start = bisect.bisect_left(s.t, s.t[0] + MAX_DELAY + 1e-9)
        if len(s.t) - start < 12 or s.t[-1] - s.t[start] < 1.0:
            continue
        u = s.steering if channel == "steering" else s.speed_command
        y = s.feedback if channel == "steering" else s.wheel_speed
        prepared.append((s.ct, u, s.t[start:], y[start:]))
    return prepared


def metrics(prepared, delay, tau):
    errors, observations = [], []
    for ct, u, t, y in prepared:
        prediction = predict(ct, u, t, y[0], delay, tau)
        errors.extend(a - b for a, b in zip(prediction[1:], y[1:]))
        observations.extend(y[1:])
    if not errors:
        return None
    mse = sum(e * e for e in errors) / len(errors)
    variance = statistics.pvariance(observations)
    return dict(samples=len(errors), segments=len(prepared), rmse=math.sqrt(mse),
                mae=sum(map(abs, errors)) / len(errors), bias=statistics.mean(errors),
                r_squared=1 - mse / variance if variance > 1e-15 else None)


def excitation(prepared, threshold):
    commands = []
    steps = []
    for ct, u, t, _ in prepared:
        lo = bisect.bisect_right(ct, t[0] - MAX_DELAY) - 1
        hi = bisect.bisect_right(ct, t[-1])
        values = u[max(lo, 0):hi]
        commands.extend(values)
        steps.extend(abs(b - a) for a, b in zip(values, values[1:]))
    return dict(command_std=statistics.pstdev(commands) if commands else 0,
                command_range=max(commands) - min(commands) if commands else 0,
                informative_steps=sum(x >= threshold for x in steps),
                step_threshold=threshold,
                scored_duration_s=sum(t[-1] - t[0] for _, _, t, _ in prepared))


def fit_channel(train_segments, validation_segments, channel, verified=False):
    train, validation = prepare(train_segments, channel), prepare(validation_segments, channel)
    support = {"train": excitation(train, 0.01), "validation": excitation(validation, 0.01)}
    if not train or not validation:
        return dict(status="insufficient_support", support=support,
                    reason="both chronological partitions need a >=1s scored segment plus 0.7s history")
    candidates = {}

    def evaluate(delay, tau):
        key = (round(delay, 6), round(tau, 6))
        if key not in candidates:
            candidates[key] = metrics(train, *key)["rmse"]

    for di in range(15):
        for ti in range(13):
            evaluate(di * 0.05, ti * 0.05)
    coarse = min(candidates, key=candidates.get)
    for di in range(-5, 6):
        for ti in range(-5, 6):
            d, tau = coarse[0] + di * 0.01, coarse[1] + ti * 0.01
            if 0 <= d <= MAX_DELAY and 0 <= tau <= MAX_TAU:
                evaluate(d, tau)
    best = min(candidates, key=candidates.get)
    # Refine the independent pure-delay baseline too, using training data only.
    for di in range(71):
        evaluate(di * 0.01, 0.0)
    baseline = min((key for key in candidates if key[1] == 0), key=candidates.get)
    best = min(candidates, key=candidates.get)
    fitted = {part: metrics(p, *best) for part, p in (("train", train), ("validation", validation))}
    zero_tau = {part: metrics(p, *baseline) for part, p in (("train", train), ("validation", validation))}
    gains = {part: 1 - fitted[part]["rmse"] / zero_tau[part]["rmse"]
             if zero_tau[part]["rmse"] > 1e-12 else 0.0 for part in fitted}
    fixed_delay_candidates = {round(i * 0.01, 6): metrics(train, 0.5, i * 0.01)["rmse"]
                              for i in range(61)}
    fixed_tau = min(fixed_delay_candidates, key=fixed_delay_candidates.get)
    fixed_metrics = {part: metrics(p, 0.5, fixed_tau)
                     for part, p in (("train", train), ("validation", validation))}
    configured_metrics = {part: metrics(p, 0.5, 0)
                          for part, p in (("train", train), ("validation", validation))}
    near = [key for key, error in candidates.items()
            if error <= candidates[best] * 1.05 + 1e-10]
    ranges = {name: [min(k[i] for k in near), max(k[i] for k in near)]
              for i, name in enumerate(("delay_s", "tau_s"))}
    reasons = []
    if not verified:
        reasons.append("low-level feedback provenance/calibration not verified")
    if any(s["informative_steps"] < 10 or s["scored_duration_s"] < 10 for s in support.values()):
        reasons.append("limited independent excitation or duration")
    if gains["validation"] < 0.05:
        reasons.append("less than 5% held-out RMSE improvement over pure-delay baseline")
    if ranges["tau_s"][1] - ranges["tau_s"][0] > 0.15:
        reasons.append("broad delay/tau tradeoff in training objective")
    if best[0] in (0.0, MAX_DELAY) or best[1] in (0.0, MAX_TAU):
        reasons.append("optimum touches search boundary")
    if fitted["validation"]["r_squared"] is None or fitted["validation"]["r_squared"] < 0.5:
        reasons.append("weak held-out explanatory power")
    return dict(status="fit", units="rad" if channel == "steering" else "m/s",
                best=dict(delay_s=best[0], tau_s=best[1]), metrics=fitted,
                zero_tau_baseline=dict(delay_s=baseline[0], tau_s=0.0, metrics=zero_tau),
                configured_delay_zero_tau=dict(delay_s=0.5, tau_s=0.0,
                                               metrics=configured_metrics),
                configured_delay_tau_fit=dict(delay_s=0.5, tau_s=fixed_tau, metrics=fixed_metrics,
                    validation_rmse_fractional_improvement=(
                        1 - fixed_metrics["validation"]["rmse"] / configured_metrics["validation"]["rmse"]
                        if configured_metrics["validation"]["rmse"] > 1e-12 else 0.0),
                    selection="training-only tau grid 0..0.6s at 0.01s, delay held at 0.5s"),
                validation_rmse_fractional_improvement=gains["validation"], support=support,
                confidence=dict(level="candidate_only" if not reasons else "low", reasons=reasons,
                    near_optimal_training_ranges=ranges,
                    range_definition="evaluated candidates within 5% train RMSE; not statistical confidence intervals"),
                evaluated_candidates=len(candidates))


def constant_command_windows(segments, min_duration=2.0, tolerance=0.005,
                             max_speed=0.5, settle=1.3):
    """Disjoint low-speed plateaus, bounded by total command range, not slope."""
    windows = []
    for s in segments:
        begin, low, high = 0, s.speed_command[0], s.speed_command[0]

        def record(a, b):
            end = s.ct[b] if b < len(s.ct) else s.t[-1]
            start = s.ct[a]
            if end - start < min_duration:
                return
            indices = [i for i, t in enumerate(s.t) if start + settle <= t < end]
            if len(indices) < 3:
                return
            values = [s.wheel_speed[i] for i in indices]
            cmd = statistics.mean(s.speed_command[a:b])
            mean = statistics.mean(values)
            windows.append(dict(start_stamp=start, end_stamp=end, duration_s=end - start,
                                settled_samples=len(values), command_mean_mps=cmd,
                                command_range_mps=max(s.speed_command[a:b]) - min(s.speed_command[a:b]),
                                wheel_speed_mean_mps=mean, wheel_speed_std_mps=statistics.pstdev(values),
                                wheel_speed_min_mps=min(values), wheel_speed_max_mps=max(values),
                                mean_error_mps=mean - cmd, measured_command_ratio=mean / cmd))

        for i, value in enumerate(s.speed_command):
            if value > max_speed or max(high, value) - min(low, value) > tolerance:
                record(begin, i)
                begin, low, high = i, value, value
            else:
                low, high = min(low, value), max(high, value)
            if value > max_speed:
                begin = i + 1
                if begin < len(s.speed_command):
                    low = high = s.speed_command[begin]
        if begin < len(s.ct):
            record(begin, len(s.ct))
    return dict(count=len(windows), windows=windows,
                criteria=dict(min_duration_s=min_duration, command_range_mps=tolerance,
                              max_command_mps=max_speed, settling_exclusion_s=settle),
                interpretation="Descriptive wheel-speed tracking only; not identification of powertrain nonlinearity.")


def analyze(args):
    if args.can_log:
        segments, can_metadata = read_can_log(args.can_log, args.min_speed, args.max_gap)
        inputs = {"can_log": can_metadata}
        raw_feedback = True
    else:
        cr, cm = read_stream(args.command_csv, "command", args.command_steer_unit,
                             args.command_speed_unit, args.min_speed, args.max_gap)
        fr, fm = read_stream(args.feedback_csv, "feedback", args.feedback_steer_unit,
                             args.feedback_speed_unit, args.min_speed, args.max_gap)
        if (Path(args.command_csv).resolve() != Path(args.feedback_csv).resolve()
                and (cm["counts"]["time_resets"] or fm["counts"]["time_resets"])):
            raise ValueError("cross-file clock resets are ambiguous; supply matching single-epoch logs")
        segments = align_streams(cr, fr)
        inputs = {"command": cm, "feedback": fm}
        raw_feedback = fm["angle_column"] == "steer_scaled"
    train, validation = chronological_split(segments, args.train_fraction)
    suspect = sum(sum(s.suspect) for s in segments)
    verified = args.feedback_calibrated and not suspect and raw_feedback
    report = dict(schema_version=1, input=inputs,
                policy=dict(read_only=True, controller_delay_s_unchanged=0.5,
                            live_tau_default_s=0.0, auto_apply=False),
                method=dict(model="unity-gain pure delay plus first-order lag, fitted separately per channel",
                            command_hold="zero_order_hold at original command timestamps",
                            integration="exact at delayed command events; no measured derivatives",
                            split="global contiguous file-order prefix train / suffix validation",
                            train_fraction=args.train_fraction,
                            initialization="measured feedback after common 0.7s command-history reserve in each partition/run; seed excluded from metrics",
                            gap_limit_s=args.max_gap, forward_motion_threshold_mps=args.min_speed,
                            adaptive_gap_rule="also split each stream above 1.75 times its median positive sample interval",
                            grid=dict(delay_s=[0, 0.7], tau_s=[0, 0.6], coarse_step_s=0.05,
                                      local_refinement_step_s=0.01)),
                support=dict(aligned_segments=len(segments), aligned_samples=sum(len(s.t) for s in segments),
                             train_samples=sum(len(s.t) for s in train),
                             validation_samples=sum(len(s.t) for s in validation),
                             train_last_stamp=train[-1].t[-1] if train else None,
                             validation_first_stamp=validation[0].t[0] if validation else None,
                             steer_scale_suspect_samples=suspect),
                steering=fit_channel(train, validation, "steering", verified),
                speed=fit_channel(train, validation, "speed", args.feedback_calibrated),
                constant_command_speed=constant_command_windows(segments),
                limitations=["Feedback may be echoed, filtered, stale or incorrectly scaled; physical sensor provenance is not established by this CSV.",
                             "CSV stamps are logging times, not verified hardware acquisition timestamps; fitted delay includes transport/logging latency.",
                             "Closed-loop, quantized and correlated data provide limited independent excitation; no gain, offset, hysteresis or saturation is fitted.",
                             "Forward-moving runs only; starts/stops and missing gaps are excluded. No cross-file clock-reset pairing is guessed.",
                             "Constant-command windows do not establish powertrain nonlinearity or causality.",
                             "Keep live delay at 0.5s and live tau at 0 until reliable calibrated identification is independently reviewed."])
    if args.can_log:
        report["method"].update(
            clock="same-process/run steady clock only; ROS, wall, callback and CAN clocks ignored",
            command_hold="successful encoded TX at tx_end_steady; held across no-new-command cycles",
            adaptive_gap_rule="split logging and fresh-RX gaps above min(max-gap, 1.75x median cycle period)",
            tx_gate="tx_attempted==1 and tx_result==1 and encoding_valid==1",
            rx_gate="fresh_valid==1",
            unknown_actuation="failed/invalid TX or missing/invalid log rows discard held command until next success",
            run_boundaries="repeated process headers or steady-clock resets; never join across runs")
        report["limitations"][1] = (
            "TX/RX end-of-call steady timestamps bound host calls, not physical actuation/acquisition. "
            "fresh_valid means newly accepted, not proven physically recent; CAN clock is not synchronized.")
        report["limitations"].append(
            "Encoded TX success is vendor-reported submission success, not measured actuation. "
            "End-of-call timing convention has uncertainty up to call duration; saturated wire commands may be present.")
    return report


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--can-log", type=Path,
                   help="prefer one driver diagnostic CSV for both TX/RX; runtime paths and unit overrides ignored")
    p.add_argument("--command-csv", type=Path, default=ROOT / "cdata/runtime_debug.csv")
    p.add_argument("--feedback-csv", type=Path, default=ROOT / "data/runtime_debug.csv")
    p.add_argument("--command-steer-unit", choices=ANGLE_SCALES, default="rad")
    p.add_argument("--feedback-steer-unit", choices=ANGLE_SCALES, default="rad")
    p.add_argument("--command-speed-unit", choices=SPEED_SCALES, default="m/s")
    p.add_argument("--feedback-speed-unit", choices=SPEED_SCALES, default="m/s")
    p.add_argument("--min-speed", type=float, default=0.05)
    p.add_argument("--max-gap", type=float, default=0.25,
                   help="absolute gap cap in seconds; also split above 1.75x stream median period")
    p.add_argument("--train-fraction", type=float, default=0.7)
    p.add_argument("--feedback-calibrated", action="store_true",
                   help="annotate independently verified physical feedback; never enables application")
    p.add_argument("--output", type=Path, help="explicit NEW JSON file; existing files are never overwritten")
    return p


def main(argv=None):
    p = parser()
    args = p.parse_args(argv)
    if (not math.isfinite(args.max_gap) or args.max_gap <= 0
            or not math.isfinite(args.min_speed) or args.min_speed < 0
            or not 0 < args.train_fraction < 1):
        p.error("require finite max-gap > 0, min-speed >= 0 and 0 < train-fraction < 1")
    try:
        if args.output and args.output.exists():
            raise ValueError("--output must be a new file; refusing to overwrite any existing file")
        report = analyze(args)
        serialized = json.dumps(report, indent=2, allow_nan=False) + "\n"
        if args.output:
            with args.output.open("x", encoding="utf-8") as stream:
                stream.write(serialized)
        else:
            sys.stdout.write(serialized)
    except (OSError, ValueError, csv.Error) as exc:
        sys.stderr.write("fit_actuator_response: {}\n".format(exc))
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
