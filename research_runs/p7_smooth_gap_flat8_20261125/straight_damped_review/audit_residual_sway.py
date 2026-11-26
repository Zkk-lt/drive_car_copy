#!/usr/bin/env python3
"""Read an archived smooth run; use the actual published owner and causal CAN/IMU."""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import re

import numpy as np
import pandas as pd


def stamp(message):
    return message['secs'] + message['nsecs'] * 1e-9


def number_or_none(value):
    value = float(value)
    return value if np.isfinite(value) else None


def near_line_reference(points):
    path = np.array([(p['s'], p['x'], p['y'], p['tangent_yaw']) for p in points])
    if len(path) <= 10 or not np.all(np.diff(path[:, 0]) > 0):
        return np.nan, np.nan, np.nan
    s = path[:, 0]
    support = path[(s >= .40) & (s <= 1.40)]
    tangent_support = (s >= .30) & (s <= 1.30)
    if len(support) <= 5 or not np.any(tangent_support):
        return np.nan, np.nan, np.nan
    dx, dy = support[-1, 1:3] - support[0, 1:3]
    px, py = (support[:, 1:3] - support[0, 1:3]).T
    distance = np.max(np.abs(dx * py - dy * px)) / max(np.hypot(dx, dy), 1e-6)
    heading = abs(np.unwrap(support[:, 3])[-1] - support[0, 3])
    curvature = np.median(np.abs(np.gradient(np.unwrap(path[:, 3]), s)[tangent_support]))
    return distance, heading, curvature


def read_events(events):
    data = defaultdict(list)
    pattern = re.compile(rb'"topic": "([^"]+)"')
    pubs, diag, req, cand, imu, can = topics = tuple(k.encode() for k in (
        '/control_research/p5/published', '/control_research/p5/diagnostics',
        '/control_research/p5_test/p6/request', '/control_research/p5_test/p6/candidate',
        '/imu', '/car_message'))
    with events.open('rb') as stream:
        for raw in stream:
            key = pattern.search(raw[:1024])
            if key is None or key[1] not in topics:
                continue
            event = json.loads(raw)
            p = event['payload']
            receipt = event['receipt_ros_ns'] * 1e-9
            if key[1] == pubs and p['publish_returned'] and p['history_committed']:
                data['published'].append((p['cycle'], p['sequence'], stamp(p['publish_ros_start']),
                    stamp(p['cycle_stamp']), p['owner'], p['steering'], p['speed'], p['reason'].split(';')[0]))
            elif key[1] == diag:
                data['diagnostics'].append((p['cycle'], p['candidate_cycle'], p['publish_interval_ms']))
            elif key[1] == req:
                b = p['base']
                data['requests'].append((b['cycle'], b['reference_valid'], b['baseline_steering'],
                    *near_line_reference(b['reference'])))
            elif key[1] == cand:
                b = p['base']
                fields = dict(field.split('=', 1) for field in b['reason'].split(';') if '=' in field)
                data['candidates'].append((b['cycle'], b['previous_sequence'], b['valid'], b['solve_ms'],
                    p['feedback_correction'], float(fields.get('straight_tracking_blend', 'nan')),
                    float(fields.get('steering_rate_continuity_weight', 'nan'))))
            elif key[1] == imu:
                data['imu'].append((receipt, p['angular_velocity']['z']))
            elif key[1] == can:
                data['can'].append((receipt, p['turn_angle'], .001 * p['back_wheel_speed']))
    return data


def audit(folder):
    events = next(folder.glob('raw/*/events.jsonl'))
    data = read_events(events)
    d = pd.DataFrame(data['published'], columns='cycle seq t cycle_stamp owner cmd speed reason'.split())
    d = d.sort_values('seq')
    consecutive = d.seq.diff().eq(1) & d.t.diff().gt(0) & d.t.diff().lt(.08)
    d['rate'] = (d.cmd.diff() / d.t.diff()).where(consecutive)
    d['switch'] = d.owner.ne(d.owner.shift()) & consecutive
    previous = d.rate.shift()
    d['reverse'] = (d.rate * previous < 0) & (d.rate.abs() > .20) & (previous.abs() > .20)
    for key, columns in (('imu', 't imu'), ('can', 't delta wheel_speed')):
        other = pd.DataFrame(data[key], columns=columns.split()).sort_values('t')
        d = pd.merge_asof(d.sort_values('t'), other, on='t', direction='backward', tolerance=.15)
    for key, columns in (
        ('diagnostics', 'cycle candidate_cycle publish_ms'),
        ('requests', 'cycle reference_valid baseline chord heading_diff median_k'),
        ('candidates', 'candidate_cycle previous_seq candidate_valid solve_ms correction blend weight')):
        names = columns.split()
        right = pd.DataFrame(data[key], columns=names).drop_duplicates(names[0], keep='last')
        d = d.merge(right, on=names[0], how='left')
    runtime_columns = 'stamp track_signed_e_y e_theta_control straight_lock reference_status_mode'.split()
    runtime = pd.read_csv(folder / 's8_csv_live/runtime_debug.csv', usecols=runtime_columns).sort_values('stamp')
    d = pd.merge_asof(d.sort_values('cycle_stamp'), runtime, left_on='cycle_stamp', right_on='stamp',
                      direction='nearest', tolerance=.05)
    d = d.sort_values('seq')
    moving = (d.speed > .18) & (d.wheel_speed > .18) & d.reference_valid.eq(True)
    straight = moving & (d.chord < .018) & (d.heading_diff < .12) & (d.median_k < .15)
    bend = moving & ((d.chord > .04) | (d.heading_diff > .35))
    nmpc = d.owner.eq('nmpc')
    selected = nmpc & d.candidate_valid.eq(True) & d.previous_seq.eq(d.seq - 1)
    t0 = d.loc[moving, 't'].min()
    d['since_moving_start_s'] = d.t - t0

    def summarize(mask):
        r = d[mask]
        sq = np.sqrt(r.rate.pow(2).mean())
        return dict(cycles=len(r), nmpc_cycles=int(r.owner.eq('nmpc').sum()),
                    steering_rate_rms_rad_s=number_or_none(sq),
                    cte_abs_p95_m=number_or_none(r.track_signed_e_y.abs().quantile(.95)),
                    command_angle_abs_p95_rad=number_or_none(r.cmd.abs().quantile(.95)),
                    can_angle_abs_p95_rad=number_or_none(r.delta.abs().quantile(.95)),
                    imu_yaw_rate_abs_p95_rad_s=number_or_none(r.imu.abs().quantile(.95)),
                    slqr_correction_abs_p95_rad=number_or_none(r.correction.abs().quantile(.95)),
                    slqr_correction_abs_max_rad=number_or_none(r.correction.abs().max()),
                    blend_median=number_or_none(r.blend.median()),
                    effective_weight_median=number_or_none(r.weight.median()),
                    solve_ms_p95=number_or_none(r.solve_ms.quantile(.95)),
                    publish_interval_ms_max=number_or_none(r.publish_ms.max()),
                    strong_reversals=int(r.reverse.sum()))

    window_mask = d.since_moving_start_s.between(5.74, 7.005)
    window = d[window_mask & selected]
    samples = []
    for seconds in (5.75, 6.25, 6.50, 6.75, 7.00) if len(window) else ():
        row = window.loc[(window.since_moving_start_s - seconds).abs().idxmin()]
        samples.append({**{'time_utc8': pd.to_datetime(row.t + 8*3600, unit='s').strftime('%H:%M:%S.%f')[:-3]},
                      **{k: number_or_none(row[k]) for k in
                         'since_moving_start_s cmd baseline delta imu blend weight chord heading_diff track_signed_e_y'.split()}})
    manifest = json.loads(events.with_name('manifest.json').read_text())
    archive = json.loads((folder / 'RAM_ARCHIVE_MANIFEST.json').read_text())
    trial = json.loads((folder / 'trial.json').read_text())
    return dict(run=folder.name, data_origin='real_vehicle', route_ground_truth=False,
                reference_rule='published and CAN speed proxy >0.18 m/s, valid request; near line 0.40–1.40 m chord deviation<0.018 m, end tangent difference<0.12 rad, 0.30–1.30 m median absolute tangent derivative<0.15 rad/m',
                sensor_times='last receipt up to 0.15 s before current command; no future feedback',
                selected_candidate_identity='same diagnostic cycle and candidate previous publication sequence plus one equals actual publication sequence',
                original_event_sha256=archive['files'][str(events.relative_to(folder))]['sha256'],
                binaries=trial['binaries'], all_publications=len(d), all_nmpc=int(nmpc.sum()),
                recorder_application_queue_drops=manifest['writer']['queue_dropped'],
                moving_valid=summarize(moving), near_line=summarize(straight),
                near_line_selected_flat8=summarize(straight & selected),
                near_line_selected_flat8_high_blend=summarize(straight & selected & (d.blend >= .8)),
                near_line_same_owner_flat8_high_blend=summarize(straight & selected & ~d.switch & (d.blend >= .8)),
                near_line_owner_switch=summarize(straight & d.switch),
                near_line_same_owner=summarize(straight & ~d.switch),
                near_line_fallback_reasons=d.loc[straight & ~nmpc, 'reason'].value_counts().to_dict(),
                clearly_curved=summarize(bend),
                continuous_flat8_window={**summarize(window_mask),
                    'matched_nmpc_samples':len(window), 'examples':samples},
                note='Route groups are proxies from the request path. Saturation and cause are not randomized. Compare with older runs only descriptively; unlike a paired treatment experiment.')


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('archive', type=Path)
    args = ap.parse_args()
    print(json.dumps(audit(args.archive), ensure_ascii=False, indent=2, allow_nan=False))
