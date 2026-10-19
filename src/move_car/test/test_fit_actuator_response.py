"""Focused offline tests: python3 -B -m unittest discover -s src/move_car/test -p 'test_fit_actuator_response.py'."""

import contextlib
import csv
import io
import math
from pathlib import Path
import random
import tempfile
import unittest

import fit_actuator_response as fit


CAN_COLUMNS = (
    "ros_time,wall_time,steady_time,cmd_seq,cmd_ros_time,cmd_wall_time,cmd_steady_time,"
    "cmd_speed_m_s,cmd_steering_rad,new_command,encoding_valid,saturated,encoded_speed_mm_s,"
    "encoded_steering_centideg,tx_attempted,tx_result,tx_start_steady,tx_end_steady,rx_count,"
    "rx_count_error,rx_valid_count,rx_stale_count,fresh_valid,rx_start_steady,rx_end_steady,"
    "feedback_id,can_timestamp_raw,can_timestamp_valid,feedback_speed_mm_s,"
    "feedback_steering_centideg,battery,error").split(",")


def can_row(t, tx=True, fresh=True, **overrides):
    row = dict.fromkeys(CAN_COLUMNS, 0)
    row.update(steady_time=t + 0.004, new_command=int(tx), encoding_valid=int(tx),
               tx_attempted=int(tx), tx_result=1 if tx else -2,
               tx_start_steady=t + 0.001 if tx else 0,
               tx_end_steady=t + 0.002 if tx else 0,
               rx_start_steady=t + 0.003, rx_end_steady=t + 0.0035,
               encoded_speed_mm_s=200 if tx else 0,
               encoded_steering_centideg=-1000 if tx else 0,
               fresh_valid=int(fresh), feedback_speed_mm_s=200 if fresh else 0,
               feedback_steering_centideg=-900 if fresh else 0,
               ros_time=900000 - t, wall_time=1700000000 - t,
               cmd_ros_time="nan", cmd_wall_time="inf", cmd_steady_time=-500,
               cmd_speed_m_s="nan", cmd_steering_rad="inf", can_timestamp_raw=4294967295)
    row.update(overrides)
    return row


def synthetic(delay=0.23, tau=0.18, seconds=40):
    rng = random.Random(724)
    ct = [i * 0.4 for i in range(int(seconds / 0.4) + 1)]
    u = [rng.uniform(-0.25, 0.25) for _ in ct]
    t = [i * 0.05 for i in range(int(seconds / 0.05) + 1)]
    # Independent analytic superposition of delayed step responses.
    y = []
    for stamp in t:
        value = u[0]
        for i in range(1, len(ct)):
            event = ct[i] + delay
            age = stamp - event
            if stamp >= event:
                value += (u[i] - u[i - 1]) * (1 - math.exp(-age / tau) if tau else 1)
        y.append(value)
    return fit.Segment(ct, u, [0.2] * len(ct), t, y, [0.2] * len(t), [False] * len(t))


class ActuatorFitTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.segment = synthetic()
        cls.train, cls.validation = fit.chronological_split([cls.segment])
        cls.report = fit.fit_channel(cls.train, cls.validation, "steering")

    def write_csv(self, directory, name, rows, columns=None):
        path = Path(directory) / name
        with path.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=columns or list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        return path

    def test_known_delay_and_lag_recovered_separately(self):
        self.assertAlmostEqual(self.report["best"]["delay_s"], 0.23, delta=0.02)
        self.assertAlmostEqual(self.report["best"]["tau_s"], 0.18, delta=0.02)
        self.assertLess(self.report["metrics"]["validation"]["rmse"], 0.002)
        self.assertGreater(self.report["validation_rmse_fractional_improvement"], 0.9)
        self.assertEqual(self.report["zero_tau_baseline"]["tau_s"], 0)
        self.assertEqual(self.report["confidence"]["level"], "low")

    def test_exact_zoh_integration_and_measured_initialization(self):
        prediction = fit.predict([-1, 0.25, 0.75], [0, 1, -1], [0, 0.5, 1], 0.3, 0.1, 0.2)
        first = 0.3 * math.exp(-0.35 / 0.2)
        first = 1 + (first - 1) * math.exp(-0.15 / 0.2)
        final = 1 + (first - 1) * math.exp(-0.35 / 0.2)
        final = -1 + (final + 1) * math.exp(-0.15 / 0.2)
        self.assertEqual(prediction[0], 0.3)
        self.assertAlmostEqual(prediction[1], first)
        self.assertAlmostEqual(prediction[2], final)

    def test_zero_tau_is_instantaneous_delayed_command_not_interpolation(self):
        self.assertEqual(fit.predict([0, 0.25, 0.75], [0, 1, -1],
                                     [0.2, 0.5, 1], 0.7, 0.1, 0), [0.7, 1, -1])
        with self.assertRaises(ValueError):
            fit.predict([0], [1], [0], 0, 0.2, 0.1)

    def test_chronological_split_and_no_validation_command_history(self):
        self.assertLess(self.train[-1].t[-1], self.validation[0].t[0])
        self.assertEqual(self.validation[0].ct[0], self.validation[0].t[0])
        prepared = fit.prepare(self.validation, "steering")
        expected = sum(len(t) - 1 for _, _, t, _ in prepared)
        self.assertEqual(self.report["metrics"]["validation"]["samples"], expected)
        self.assertGreaterEqual(prepared[0][2][0] - self.validation[0].t[0], 0.7)

    def test_validation_does_not_select_parameters(self):
        changed = self.validation[0].clip(0, len(self.validation[0].t))
        changed.feedback = [0.25 * math.sin(t * 7) for t in changed.t]
        result = fit.fit_channel(self.train, [changed], "steering")
        self.assertEqual(result["best"], self.report["best"])
        self.assertEqual(result["metrics"]["train"], self.report["metrics"]["train"])
        self.assertEqual(result["configured_delay_tau_fit"]["tau_s"],
                         self.report["configured_delay_tau_fit"]["tau_s"])
        self.assertGreater(result["metrics"]["validation"]["rmse"], 0.1)

    def test_pure_delay_does_not_invent_lag(self):
        train, validation = fit.chronological_split([synthetic(0.2, 0)])
        result = fit.fit_channel(train, validation, "steering")
        self.assertEqual(result["best"], {"delay_s": 0.2, "tau_s": 0.0})

    def test_unit_conversion_and_preferred_raw_feedback(self):
        with tempfile.TemporaryDirectory() as directory:
            path = self.write_csv(directory, "feedback.csv", [
                dict(stamp=i * 0.1, steer_scaled=30, steer_feedback_for_control=999,
                     wheel_speed=200, steer_scale_suspect=0) for i in range(3)])
            runs, meta = fit.read_stream(path, "feedback", "deg", "mm/s")
            self.assertEqual(meta["angle_column"], "steer_scaled")
            self.assertAlmostEqual(runs[0][1][0][1], math.pi / 6)
            self.assertAlmostEqual(runs[0][1][0][2], 0.2)
            command = self.write_csv(directory, "command.csv", [
                dict(stamp=i * 0.1, cmd_delta=30, cmd_v=0.72) for i in range(3)])
            runs, _ = fit.read_stream(command, "command", "deg", "km/h")
            self.assertAlmostEqual(runs[0][1][0][1], math.pi / 6)
            self.assertAlmostEqual(runs[0][1][0][2], 0.2)

    def test_invalid_gap_stop_reset_and_duplicate_break_runs(self):
        rows = [dict(stamp=t, steer_scaled=a, wheel_speed=v)
                for t, a, v in [(0, 0, 0.2), (0.1, 0, 0.2), (0.2, "nan", 0.2),
                                (0.3, 0, 0.2), (1.0, 0, 0.2), (1.1, 0, 0),
                                (1.2, 0, 0.2), (0.1, 0, 0.2), (0.2, 0, 0.2),
                                (0.2, 0, 0.2), (0.3, 0, 0.2)]]
        with tempfile.TemporaryDirectory() as directory:
            path = self.write_csv(directory, "feedback.csv", rows)
            runs, meta = fit.read_stream(path, "feedback")
        self.assertEqual(meta["counts"]["invalid"], 1)
        self.assertEqual(meta["counts"]["stopped"], 1)
        self.assertEqual(meta["counts"]["gaps"], 1)
        self.assertEqual(meta["counts"]["time_resets"], 1)
        self.assertEqual(meta["counts"]["duplicate_stamps"], 1)
        for _, run in runs:
            self.assertTrue(all(0 < b[0] - a[0] <= 0.25 for a, b in zip(run, run[1:])))

    def test_alignment_does_not_bridge_command_gaps(self):
        cr = [(0, [(0, 0, 0.2, False), (0.1, 0, 0.2, False)]),
              (0, [(1, 1, 0.2, False), (1.1, 1, 0.2, False)])]
        fr = [(0, [(i * 0.05, 0, 0.2, False) for i in range(23)])]
        segments = fit.align_streams(cr, fr)
        self.assertEqual(len(segments), 2)
        self.assertLessEqual(segments[0].t[-1], 0.1)
        self.assertGreaterEqual(segments[1].t[0], 1)
        result = fit.fit_channel(segments[:1], segments[1:], "steering")
        self.assertEqual(result["status"], "insufficient_support")

    def test_single_missing_sample_breaks_nominal_cadence(self):
        with tempfile.TemporaryDirectory() as directory:
            path = self.write_csv(directory, "command.csv", [
                dict(stamp=t, cmd_delta=0, cmd_v=0.2) for t in (0, 0.1, 0.2, 0.4, 0.5, 0.6)])
            runs, meta = fit.read_stream(path, "command")
        self.assertEqual(len(runs), 2)
        self.assertEqual(meta["counts"]["gaps"], 1)
        self.assertAlmostEqual(meta["effective_gap_limit_s"], 0.175)

    def test_speed_model_uses_speed_commands_and_wheel_feedback(self):
        s = synthetic(delay=0.15, tau=0.25, seconds=20)
        s.speed_command = [0.2 + value * 0.3 for value in s.steering]
        s.wheel_speed = [0.2 + value * 0.3 for value in s.feedback]
        s.steering = [0] * len(s.ct)
        s.feedback = [0] * len(s.t)
        train, validation = fit.chronological_split([s])
        result = fit.fit_channel(train, validation, "speed")
        self.assertEqual(result["units"], "m/s")
        self.assertAlmostEqual(result["best"]["delay_s"], 0.15, delta=0.02)
        self.assertAlmostEqual(result["best"]["tau_s"], 0.25, delta=0.02)

    def test_split_is_global_not_per_segment(self):
        a, b = synthetic(seconds=5), synthetic(seconds=5)
        b.t = [t + 10 for t in b.t]
        b.ct = [t + 10 for t in b.ct]
        train, validation = fit.chronological_split([a, b])
        self.assertEqual(len(train), 2)
        self.assertEqual(len(validation), 1)
        self.assertEqual(train[0].t, a.t)
        self.assertLess(train[-1].t[-1], validation[0].t[0])

    def test_constant_command_stats_and_no_drift_chain(self):
        s = synthetic(seconds=10)
        s.speed_command = [0.2] * len(s.ct)
        s.wheel_speed = [0.24] * len(s.t)
        result = fit.constant_command_windows([s])
        self.assertEqual(result["count"], 1)
        self.assertAlmostEqual(result["windows"][0]["mean_error_mps"], 0.04)
        self.assertAlmostEqual(result["windows"][0]["measured_command_ratio"], 1.2)
        s.speed_command = [0.2 + i * 0.004 for i in range(len(s.ct))]
        self.assertEqual(fit.constant_command_windows([s])["count"], 0)
        s.speed_command = [0.8] * len(s.ct)
        self.assertEqual(fit.constant_command_windows([s])["count"], 0)

    def test_low_excitation_is_flagged(self):
        s = synthetic(seconds=12)
        s.steering = [0.1] * len(s.ct)
        s.feedback = [0.1] * len(s.t)
        train, validation = fit.chronological_split([s])
        result = fit.fit_channel(train, validation, "steering", verified=True)
        self.assertEqual(result["confidence"]["level"], "low")
        self.assertEqual(result["support"]["train"]["informative_steps"], 0)

    def test_cli_stdout_read_only_and_explicit_output_no_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            rows = [dict(stamp=i * 0.1, cmd_delta=0.1, cmd_v=0.2,
                         steer_scaled=0.1, wheel_speed=0.2) for i in range(5)]
            path = self.write_csv(directory, "both.csv", rows)
            before = path.read_bytes()
            argv = ["--command-csv", str(path), "--feedback-csv", str(path)]
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                self.assertEqual(fit.main(argv), 0)
            report = fit.json.loads(output.getvalue())
            self.assertEqual(report["steering"]["status"], "insufficient_support")
            self.assertEqual(report["policy"]["live_tau_default_s"], 0)
            self.assertEqual(report["policy"]["controller_delay_s_unchanged"], 0.5)
            self.assertEqual(path.read_bytes(), before)
            with contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(fit.main(argv + ["--output", str(path)]), 2)
            self.assertEqual(path.read_bytes(), before)
            new = Path(directory) / "report.json"
            self.assertEqual(fit.main(argv + ["--output", str(new)]), 0)
            self.assertTrue(new.exists())
            with contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(fit.main(argv + ["--output", str(new)]), 2)

    def test_missing_columns_and_unconverted_degrees_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = self.write_csv(directory, "bad.csv", [dict(stamp=0, steer_raw=30, wheel_speed=0.2)])
            with self.assertRaises(ValueError):
                fit.read_stream(path, "feedback")
            path = self.write_csv(directory, "degrees.csv", [dict(stamp=0, steer_scaled=30, wheel_speed=0.2)])
            runs, meta = fit.read_stream(path, "feedback")
            self.assertEqual(runs, [])
            self.assertEqual(meta["counts"]["invalid"], 1)

    def test_cross_file_reset_pairing_refused(self):
        with tempfile.TemporaryDirectory() as directory:
            command = self.write_csv(directory, "command.csv", [dict(stamp=t, cmd_delta=0, cmd_v=0.2)
                                                                  for t in (0, 0.1, 0, 0.1)])
            feedback = self.write_csv(directory, "feedback.csv", [dict(stamp=t, steer_scaled=0, wheel_speed=0.2)
                                                                    for t in (0, 0.1, 0, 0.1)])
            args = fit.parser().parse_args(["--command-csv", str(command), "--feedback-csv", str(feedback)])
            with self.assertRaisesRegex(ValueError, "clock resets"):
                fit.analyze(args)


class CanLogTests(unittest.TestCase):
    def write_log(self, directory, rows):
        path = Path(directory) / "can.csv"
        with path.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=CAN_COLUMNS)
            writer.writeheader()
            for row in rows:
                if row == "header":
                    writer.writeheader()
                else:
                    writer.writerow(row)
        return path

    def read(self, rows):
        with tempfile.TemporaryDirectory() as directory:
            path = self.write_log(directory, rows)
            return fit.read_can_log(path)

    def test_encoded_signed_units_and_same_process_event_clocks(self):
        rows = [can_row(100 + i * 0.05, encoded_speed_mm_s=253,
                        encoded_steering_centideg=-1234, feedback_speed_mm_s=241,
                        feedback_steering_centideg=987) for i in range(5)]
        segments, meta = self.read(rows)
        self.assertEqual(len(segments), 1)
        s = segments[0]
        self.assertAlmostEqual(s.ct[0], 100.002)
        self.assertAlmostEqual(s.t[0], 100.0035)
        self.assertAlmostEqual(s.speed_command[0], 0.253)
        self.assertAlmostEqual(s.wheel_speed[0], 0.241)
        self.assertAlmostEqual(s.steering[0], -1234 * math.pi / 18000)
        self.assertAlmostEqual(s.feedback[0], 987 * math.pi / 18000)
        self.assertEqual(meta["command_time_column"], "tx_end_steady")
        self.assertEqual(meta["feedback_time_column"], "rx_end_steady")
        # ROS/wall/callback/CAN clocks may reset, wrap, be nonfinite or disagree.
        for i, row in enumerate(rows):
            row.update(ros_time="nan", wall_time=-i * 1000, cmd_steady_time=1e20,
                       can_timestamp_raw=i % 2, can_timestamp_valid=i % 2)
        changed, _ = self.read(rows)
        self.assertEqual(changed, segments)

    def test_no_new_command_holds_successful_tx_without_replaying_zero_fields(self):
        rows = [can_row(100 + i * 0.05, tx=(i == 0)) for i in range(61)]
        rows[-1].update(fresh_valid=0, feedback_speed_mm_s="nan", feedback_steering_centideg="nan")
        segments, meta = self.read(rows)
        self.assertEqual(len(segments), 1)
        self.assertEqual(len(segments[0].ct), 1)
        self.assertEqual(len(segments[0].t), 60)
        self.assertEqual(meta["counts"]["failed_or_invalid_tx"], 0)
        self.assertEqual(meta["counts"]["idle_rows"], 60)
        self.assertEqual(meta["counts"]["nonfresh_rx"], 1)
        self.assertEqual(fit.constant_command_windows(segments)["count"], 1)

    def test_failed_and_invalid_tx_exclude_until_next_success(self):
        failures = [dict(tx_result=0), dict(tx_result=4294967295), dict(tx_result=-1),
                    dict(encoding_valid=0),
                    dict(tx_attempted=0, tx_result=-2, encoding_valid=0),
                    dict(tx_attempted=0, tx_result=1, encoding_valid=1)]
        for failure in failures:
            with self.subTest(failure=failure):
                rows = [can_row(100 + i * 0.05, tx=(i in (0, 3, 6))) for i in range(10)]
                rows[3].update(failure)
                segments, meta = self.read(rows)
                self.assertEqual(len(segments), 2)
                self.assertLess(segments[0].t[-1], 100.15)
                self.assertGreaterEqual(segments[1].t[0], 100.30)
                self.assertEqual(meta["counts"]["failed_or_invalid_tx"], 1)
                self.assertEqual(meta["counts"]["unknown_actuation_rx"], 3)
                self.assertEqual(meta["counts"]["successful_tx"], 2)

    def test_appended_process_header_breaks_hold_even_without_steady_reset(self):
        rows = [can_row(100 + i * 0.05, tx=(i == 0)) for i in range(4)]
        rows += ["header"]
        rows += [can_row(101 + i * 0.05, tx=(i == 2)) for i in range(6)]
        segments, meta = self.read(rows)
        self.assertEqual(len(segments), 2)
        self.assertLess(segments[0].t[-1], 101)
        self.assertGreaterEqual(segments[1].t[0], 101.1)
        self.assertEqual(meta["counts"]["repeated_headers"], 1)
        self.assertEqual(meta["counts"]["unknown_actuation_rx"], 2)
        self.assertEqual(meta["process_runs"], 2)

    def test_steady_reset_starts_new_run_without_cross_run_commands(self):
        rows = [can_row(100 + i * 0.05, tx=(i == 0)) for i in range(4)]
        rows += [can_row(10 + i * 0.05, tx=(i == 2)) for i in range(6)]
        segments, meta = self.read(rows)
        self.assertEqual(len(segments), 2)
        self.assertGreater(segments[0].t[0], segments[1].t[-1])
        self.assertGreaterEqual(segments[1].ct[0], 10.1)
        self.assertEqual(meta["counts"]["steady_resets"], 1)
        self.assertEqual(meta["counts"]["unknown_actuation_rx"], 2)

    def test_nonfresh_samples_not_fabricated_and_fresh_gap_reseeds(self):
        rows = [can_row(100 + i * 0.05, tx=(i == 0), fresh=(i not in (3, 4))) for i in range(9)]
        for i in (3, 4):
            rows[i].update(feedback_speed_mm_s="nan", feedback_steering_centideg="nan")
        segments, meta = self.read(rows)
        self.assertEqual([len(s.t) for s in segments], [3, 4])
        self.assertEqual(meta["counts"]["fresh_rx"], 7)
        self.assertEqual(meta["counts"]["feedback_gaps"], 1)
        self.assertEqual(meta["counts"]["successful_tx"], 1)
        self.assertEqual(meta["counts"]["unknown_actuation_rx"], 0)

    def test_missing_log_cycle_invalidates_hold_until_success(self):
        rows = [can_row(100 + i * 0.05, tx=(i in (0, 7))) for i in range(10) if i != 3]
        segments, meta = self.read(rows)
        self.assertEqual(len(segments), 2)
        self.assertEqual(meta["counts"]["logging_gaps"], 1)
        self.assertEqual(meta["counts"]["unknown_actuation_rx"], 3)
        self.assertGreaterEqual(segments[1].t[0], 100.35)

    def test_invalid_wire_values_and_incompatible_steady_times_invalidate_hold(self):
        invalid = [dict(encoded_speed_mm_s=32768), dict(encoded_steering_centideg="nan"),
                   dict(fresh_valid=2), dict(rx_end_steady=1e9), dict(tx_end_steady=1e9)]
        for change in invalid:
            with self.subTest(change=change):
                rows = [can_row(100 + i * 0.05, tx=(i in (0, 3, 6))) for i in range(9)]
                rows[3].update(change)
                segments, meta = self.read(rows)
                self.assertEqual(len(segments), 2)
                self.assertEqual(meta["counts"]["invalid_rows"], 1)
                self.assertGreaterEqual(segments[1].t[0], 100.3)

    def test_cli_can_precedence_stdout_and_input_protection(self):
        with tempfile.TemporaryDirectory() as directory:
            path = self.write_log(directory, [can_row(100 + i * 0.05) for i in range(5)])
            before = path.read_bytes()
            argv = ["--can-log", str(path), "--command-csv", "/does/not/exist",
                    "--feedback-csv", "/also/missing", "--feedback-steer-unit", "deg",
                    "--command-speed-unit", "km/h"]
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                self.assertEqual(fit.main(argv), 0)
            report = fit.json.loads(output.getvalue())
            self.assertEqual(set(report["input"]), {"can_log"})
            self.assertEqual(report["input"]["can_log"]["mode"], "can_log")
            self.assertEqual(report["support"]["aligned_samples"], 5)
            self.assertEqual(report["policy"]["live_tau_default_s"], 0)
            with contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(fit.main(argv + ["--output", str(path)]), 2)
            self.assertEqual(before, path.read_bytes())

    def test_missing_can_schema_refused(self):
        with tempfile.TemporaryDirectory() as directory:
            path = ActuatorFitTests.write_csv(self, directory, "bad.csv", [dict(steady_time=1)])
            with self.assertRaises(ValueError):
                fit.read_can_log(path)

    def test_known_delay_lag_through_quantized_can_pipeline(self):
        source = synthetic(seconds=24)
        encoded = [round(u * 18000 / math.pi) for u in source.steering]
        commands = [value * math.pi / 18000 for value in encoded]
        rows = []
        for i in range(481):
            t, j = i * 0.05, i // 8
            sample_time = t + 0.0015  # RX end minus first TX end, same steady clock.
            value = commands[0]
            for k in range(1, len(commands)):
                age = sample_time - (source.ct[k] + 0.23)
                if age >= 0:
                    value += (commands[k] - commands[k - 1]) * (1 - math.exp(-age / 0.18))
            rows.append(can_row(100 + t, tx=(i % 8 == 0),
                                encoded_steering_centideg=encoded[j],
                                feedback_steering_centideg=round(value * 18000 / math.pi)))
        segments, meta = self.read(rows)
        self.assertEqual(len(segments), 1)
        self.assertEqual(meta["counts"]["invalid_rows"], 0)
        train, validation = fit.chronological_split(segments)
        result = fit.fit_channel(train, validation, "steering")
        self.assertAlmostEqual(result["best"]["delay_s"], 0.23, delta=0.02)
        self.assertAlmostEqual(result["best"]["tau_s"], 0.18, delta=0.02)
        self.assertLess(result["metrics"]["validation"]["rmse"], 0.002)


if __name__ == "__main__":
    unittest.main()
