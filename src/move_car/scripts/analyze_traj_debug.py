#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path


def load_csv(path):
    with path.open("r", newline="") as f:
        return list(csv.DictReader(f))


def to_float(row, key, default=0.0):
    value = row.get(key, "")
    if value in ("", None):
        return default
    try:
        return float(value)
    except ValueError:
        return default


def to_int(row, key, default=0):
    value = row.get(key, "")
    if value in ("", None):
        return default
    try:
        return int(float(value))
    except ValueError:
        return default


def load_anchor_gate(config_path):
    gate = 0.15
    if not config_path.exists():
        return gate
    for line in config_path.read_text().splitlines():
        if line.startswith("anchor_pollution_gate:"):
            try:
                return float(line.split(":", 1)[1].strip())
            except ValueError:
                return gate
    return gate


def unique_confirmed_points(history_rows):
    confirmed = [r for r in history_rows if r.get("queue_type") == "confirmed"]
    confirmed.sort(key=lambda r: (to_float(r, "t_obs"), to_int(r, "source_obs_seq"), to_int(r, "frame_seq")))

    unique = []
    seen = set()
    for row in confirmed:
        key = (
            to_int(row, "source_obs_seq"),
            round(to_float(row, "t_obs"), 6),
            round(to_float(row, "x_world"), 6),
            round(to_float(row, "y_world"), 6),
        )
        if key in seen:
            continue
        seen.add(key)
        unique.append(row)
    return unique


def main():
    parser = argparse.ArgumentParser(description="Summarize trajectory reconstruction debug exports.")
    parser.add_argument("scenario_dir", help="Directory containing frame_debug.csv/history_world.csv/path_output.csv")
    parser.add_argument("--output", default="", help="Optional output CSV path")
    args = parser.parse_args()

    scenario_dir = Path(args.scenario_dir).expanduser().resolve()
    frame_rows = load_csv(scenario_dir / "frame_debug.csv")
    history_rows = load_csv(scenario_dir / "history_world.csv")
    path_rows = load_csv(scenario_dir / "path_output.csv")
    anchor_gate = load_anchor_gate(scenario_dir / "metrics_config.yaml")

    if not frame_rows:
        raise SystemExit("frame_debug.csv is empty")

    scenario_id = frame_rows[0].get("scenario_id", scenario_dir.name)
    frame_anchor = {
        to_int(row, "frame_seq"): (to_float(row, "anchor_x"), to_float(row, "anchor_y"))
        for row in frame_rows
    }

    confirmed_unique = unique_confirmed_points(history_rows)
    anchor_distances = []
    pollution_count = 0
    for row in confirmed_unique:
        anchor_x, anchor_y = frame_anchor.get(to_int(row, "frame_seq"), (0.0, 0.0))
        dist = math.hypot(to_float(row, "x_world") - anchor_x, to_float(row, "y_world") - anchor_y)
        anchor_distances.append(dist)
        if dist > anchor_gate:
            pollution_count += 1

    false_trail_length = 0.0
    for prev, cur in zip(confirmed_unique, confirmed_unique[1:]):
        false_trail_length += math.hypot(
            to_float(cur, "x_world") - to_float(prev, "x_world"),
            to_float(cur, "y_world") - to_float(prev, "y_world"),
        )

    ref_valid_values = [to_int(row, "ref_valid") for row in frame_rows]
    ref_spans = [to_float(row, "ref_span") for row in frame_rows]
    ref_spans_sorted = sorted(ref_spans)
    p10_index = int(0.1 * max(0, len(ref_spans_sorted) - 1))
    ref_span_p10 = ref_spans_sorted[p10_index] if ref_spans_sorted else 0.0

    curvature_by_frame = {}
    for row in path_rows:
        if row.get("path_type") != "reference":
            continue
        frame_seq = to_int(row, "frame_seq")
        curvature_by_frame.setdefault(frame_seq, []).append((to_int(row, "point_index"), to_float(row, "kappa")))

    curvature_jumps = []
    for values in curvature_by_frame.values():
        values.sort(key=lambda item: item[0])
        if len(values) < 2:
            continue
        jumps = [abs(values[i + 1][1] - values[i][1]) for i in range(len(values) - 1)]
        curvature_jumps.append(sum(jumps) / len(jumps))

    summary = {
        "scenario_id": scenario_id,
        "anchor_drift_max": max(anchor_distances) if anchor_distances else 0.0,
        "anchor_drift_mean": sum(anchor_distances) / len(anchor_distances) if anchor_distances else 0.0,
        "false_trail_length": false_trail_length,
        "history_pollution_rate": pollution_count / len(anchor_distances) if anchor_distances else 0.0,
        "valid_output_rate": sum(ref_valid_values) / len(ref_valid_values) if ref_valid_values else 0.0,
        "ref_span_mean": sum(ref_spans) / len(ref_spans) if ref_spans else 0.0,
        "ref_span_p10": ref_span_p10,
        "ref_zero_count": sum(1 for span in ref_spans if span <= 1e-6),
        "curvature_jump_mean": sum(curvature_jumps) / len(curvature_jumps) if curvature_jumps else 0.0,
    }

    output_path = Path(args.output).expanduser().resolve() if args.output else scenario_dir / "metrics_summary.csv"
    with output_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(summary.keys()))
        writer.writeheader()
        writer.writerow(summary)

    print(output_path)


if __name__ == "__main__":
    main()
