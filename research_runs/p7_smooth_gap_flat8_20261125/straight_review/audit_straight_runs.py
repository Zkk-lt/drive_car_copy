#!/usr/bin/env python3
"""Analyze preserved read-only real runs. Requires numpy/pandas."""
import argparse
from collections import defaultdict
import hashlib
import json
import re
from pathlib import Path
import subprocess

import numpy as np
import pandas as pd


def stamp(m):
    return m['secs'] + m['nsecs'] / 1e9


def p95(x):
    return float(x.abs().quantile(.95)) if len(x) else None


def rms(x):
    return float(np.sqrt(np.mean(x*x))) if len(x) else None


def motion_summary(data, selector):
    x = data[selector]
    accepted = x[x.owner.eq('nmpc')]
    stable = accepted[~accepted.switch]
    return dict(cycles=len(x), nmpc_selected=len(accepted),
                tracking_abs_error_p95_m=p95(x.track_signed_e_y),
                steer_command_abs_p95_rad=p95(x.cmd), can_steer_abs_p95_rad=p95(x.can_delta),
                imu_yaw_rate_abs_p95_rad_s=p95(x.imu_z), command_steer_rate_rms_rad_s=rms(x.steer_rate),
                candidate_slqr_abs_correction_p95_rad=p95(accepted.correction),
                selected_p95_solve_ms=p95(accepted.solve_ms),
                selected_max_solve_ms=float(accepted.solve_ms.max()) if len(accepted) else None,
                publish_p95_ms=p95(x.publish_ms), max_publish_ms=float(x.publish_ms.max()) if len(x) else None,
                lookahead_chord_median_deviation_m=float(x.chord.median()) if len(x) else None,
                observed_can_yaw_kinematic_correlation=float(x.imu_z.corr(x.can_v*np.tan(x.can_delta)/.60)),
                same_owner_nmpc_cycles=len(stable),
                same_owner_nmpc_strong_rate_reversals=int(stable.reversal.sum()),
                switch_cycles=int(x.switch.sum()), strong_rate_reversals_on_switch=int((x.switch&x.reversal).sum()),
                nonswitch_cycles=int((~x.switch).sum()), strong_rate_reversals_without_switch=int((~x.switch&x.reversal).sum()),
                legacy_straight_lock_cycles=int(x.straight_lock.eq(1).sum()),
                tracking_reference_class=x.ref_tracking_class.value_counts().to_dict(),
                geometry_probe=geometry_summary(x))


def geometry_summary(x):
    if 'geo_valid' not in x:
        return None
    x = x[x.geo_valid.eq(1)]
    if x.empty:
        return dict(valid_probe_cycles=0)
    return dict(valid_probe_cycles=len(x), terminal_approach_cycles=int(x.geo_terminal.sum()),
                nonzero_blend_cycles=int(x.blend.gt(0).sum()), blend_over_half_cycles=int(x.blend.gt(.5).sum()),
                peak_position_change_m=float(x.shift_m.max()),
                peak_heading_change_rad=float(x.shift_yaw.max()))


def geometry_metrics(probe, paths, candidate_values):
    blocks = []
    for cycle, values in candidate_values.items():
        if cycle not in paths:
            continue
        target, braking, points = paths[cycle]
        cap, params = values
        blocks.append(' '.join(map(str, [cycle, len(points), min(cap, target) if target is not None else cap,
                                            int(braking), *params, *points.ravel()])))
    result = subprocess.run([str(probe)], input='\n'.join(blocks)+'\n',
                            text=True, capture_output=True, check=True)
    return pd.DataFrame([[float(v) for v in row.split(',')] for row in result.stdout.splitlines()],
                        columns='cycle geo_valid geo_terminal blend shift_m shift_yaw support_m'.split())


def audit(folder, probe):
    events = next(folder.glob('raw/*/events.jsonl'))
    data, paths, candidate_values = defaultdict(list), {}, {}
    name = re.compile(r'"topic": "([^"]+)"')
    selected_topics = {'/control_research/p5/published', '/control_research/p5/diagnostics',
                       '/control_research/p5_test/p6/request', '/control_research/p5_test/p6/candidate',
                       '/imu', '/car_message', '/pointpillars/tracked_objects'}
    with events.open() as stream:
        for line in stream:
            match = name.search(line)
            if not match or match[1] not in selected_topics:
                continue
            x = json.loads(line)
            t, p, receipt = x['topic'], x['payload'], x['receipt_ros_ns']/1e9
            if t.endswith('/published') and p['history_committed']:
                data['cmd'].append((stamp(p['publish_ros_start']),p['sequence'],p['cycle'],
                                    p['owner'],p['speed'],p['steering']))
            elif t.endswith('/diagnostics'):
                data['diag'].append((p['cycle'],p['candidate_cycle'],p['publish_interval_ms'],p['callback_ms']))
            elif t.endswith('/request'):
                b = p['base']
                a = np.array([(v['s'],v['x'],v['y'],v['tangent_yaw'],v['curvature']) for v in b['reference']])
                chord, heading, median_k = np.nan, np.nan, np.nan
                if len(a)>10 and np.all(np.diff(a[:,0])>0.):
                    inner, outer = (a[:,0]>=.3)&(a[:,0]<=1.3), (a[:,0]>=.4)&(a[:,0]<=1.4)
                    z = a[outer]
                    if len(z)>5 and np.any(inner):
                        delta = z[-1,1:3]-z[0,1:3]
                        chord = max(abs(np.cross(delta,z[:,1:3]-z[0,1:3])))/max(np.linalg.norm(delta),1e-6)
                        heading = abs(np.unwrap(z[:,3])[-1]-z[0,3])
                        median_k = np.median(abs(np.gradient(np.unwrap(a[:,3]),a[:,0])[inner]))
                    if probe and b['reference_valid']:
                        paths[b['cycle']] = (p['target_speed'] if p['speed_target_separated'] else None,
                                              b['reference_braking'], a)
                data['req'].append((b['cycle'],b['reference_valid'],chord,heading,median_k))
            elif t.endswith('/candidate'):
                b, q = p['base'], p['parameters']
                data['candidate'].append((b['cycle'],b['solve_ms'],p['feedback_correction']))
                if probe and b['valid'] and len(b['predicted_states'])>=6:
                    candidate_values[b['cycle']] = (q['speed_cap'], [*b['predicted_states'][:6],
                        q['horizon_steps'],q['prediction_dt'],*q['effective_model'],*q['effective_limits']])
            elif t=='/imu':
                data['imu'].append((receipt,p['angular_velocity']['z']))
            elif t=='/car_message':
                data['can'].append((receipt,p['turn_angle'],p['back_wheel_speed']/1000.))
            elif t=='/pointpillars/tracked_objects':
                data['tracked'].append((receipt,len(p['poses'])))
    frame = pd.DataFrame(data['cmd'],columns='t sequence cycle owner speed cmd'.split()).sort_values('sequence')
    frame['steer_rate']=frame.cmd.diff()/frame.t.diff()
    last=frame.steer_rate.shift()
    frame['switch']=frame.owner.ne(frame.owner.shift())
    frame['reversal']=(frame.steer_rate*last<0.)&(frame.steer_rate.abs()>.2)&(last.abs()>.2)
    stamp_cols = [('can','can_delta can_v'),('imu','imu_z'),('tracked','detections')]
    for key, columns in stamp_cols:
        other=pd.DataFrame(data[key],columns=('t '+columns).split()).sort_values('t')
        frame=pd.merge_asof(frame,other,on='t',direction='backward',tolerance=.15)
    for key, columns, left_key in (
        ('diag','cycle selected_cycle publish_ms callback_ms','cycle'),
        ('req','cycle reference_valid chord heading_diff kappa_median','cycle'),
        ('candidate','selected_cycle solve_ms correction','selected_cycle')):
        other=pd.DataFrame(data[key],columns=columns.split()).drop_duplicates(left_key,keep='last')
        frame=frame.merge(other,on=left_key,how='left')
    runtime=pd.read_csv(folder/'s8_csv_live/runtime_debug.csv',usecols=
                        'stamp track_signed_e_y straight_lock ref_tracking_class'.split())
    frame=pd.merge_asof(frame,runtime.sort_values('stamp'),left_on='t',right_on='stamp',tolerance=.10,direction='nearest')
    if probe:
        frame=frame.merge(geometry_metrics(probe,paths,candidate_values),on='cycle',how='left')
    moving=frame.speed.gt(.18)&frame.can_v.gt(.18)&frame.reference_valid.eq(True)
    straight=moving&frame.chord.lt(.018)&frame.heading_diff.lt(.12)&frame.kappa_median.lt(.15)
    bend=moving&(frame.chord.gt(.04)|frame.heading_diff.gt(.35))
    manifest=json.loads(events.with_name('manifest.json').read_text())
    recon=pd.read_csv(next(folder.glob('reconstructor_csv/origin_reference*.csv')),usecols=['compute_ms'])
    lags={}
    for signal in ('can_delta','imu_z'):
        cors=[]
        for k in range(39):
            mask=straight&frame.t.diff(k).lt(.0255*k+.020) if k else straight
            cors.append(frame.cmd.shift(k)[mask].corr(frame[signal][mask]))
        peak=int(np.nanargmax(cors))
        lags[signal]=dict(apparent_lag_s=peak*.025, correlation=cors[peak])
    return dict(run_id=folder.name, ground_truth=False,
        recorder_application_queue_drops=manifest['writer']['queue_dropped'],
        recorded_committed_commands=len(frame),
        recorded_nmpc_selected=int(frame.owner.eq('nmpc').sum()),
        recorder_can_imu_backwards_aligned=True,
        straight_apparent_command_response_receipt_domain=lags,
        whole_run_timing_ms=dict(published_interval_max=float(frame.publish_ms.max()),
            control_callback_max=float(frame.callback_ms.max()), reconstructor_compute_p95=p95(recon.compute_ms),
            reconstructor_compute_max=float(recon.compute_ms.max())),
        moving_all=motion_summary(frame,moving), near_line=motion_summary(frame,straight),
        clearly_curved=motion_summary(frame,bend), other=motion_summary(frame,moving&~(straight|bend)))


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('runs',nargs='+',type=Path)
    p.add_argument('--geometry-probe',type=Path)
    args=p.parse_args()
    output={'geometry_probe_sha256':hashlib.sha256(args.geometry_probe.read_bytes()).hexdigest() if args.geometry_probe else None,
        'method': {'moving':'published SI command & CAN speed proxy > 0.18 m/s, request valid reference',
         'near_line':'request 0.40-1.40 m chord error < 0.018 m and endpoint yaw difference < 0.12 rad; 0.30-1.30 m median absolute tangent derivative < 0.15 rad/m',
         'clearly_curved':'request chord error > 0.04 m or endpoint yaw difference > 0.35 rad',
         'geometry_probe':'matches valid candidate archived state/model to the same request; tests geometry applicability and unchanged schedule only, not physical closed loop',
         'reversal':'consecutive opposite command angular increments with each angular rate beyond 0.20 rad/s'},
         'runs':[audit(folder,args.geometry_probe) for folder in args.runs]}
    print(json.dumps(output,indent=2,allow_nan=False))
