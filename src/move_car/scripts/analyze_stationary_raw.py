#!/usr/bin/env python3
"""Category-A descriptive raw statistics. Never approves or applies zero bias.

No interpolation, merged control/feedback table, sensor configuration or ROS.
Hardware acquisition times remain unknown when the driver did not publish them.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

from control_research.dataset import write_new


def stats(values):
    a = sorted(float(x) for x in values if isinstance(x, (int, float)) and
               not isinstance(x, bool) and math.isfinite(x))
    if not a:
        return dict(count=0, mean=None, std=None, minimum=None, maximum=None, p95_abs=None)
    mean = sum(a)/len(a); b = sorted(abs(v) for v in a); position = .95*(len(b)-1)
    lo = int(position); hi = min(lo+1, len(b)-1)
    return dict(count=len(a), mean=mean, std=math.sqrt(sum((x-mean)**2 for x in a)/len(a)),
                minimum=a[0], maximum=a[-1], p95_abs=b[lo]+(b[hi]-b[lo])*(position-lo))


def field(payload, name):
    value = payload
    for key in name.split('.'):
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    return value


def analyze(events, manifest):
    grouped = {}
    for event in events:
        grouped.setdefault(event.get('topic'), []).append(event)
    topic_metrics = {}
    for topic, rows in grouped.items():
        mono = [r.get('receipt_monotonic_ns') for r in rows]
        valid_mono = [v for v in mono if isinstance(v, int) and not isinstance(v, bool)]
        source = [r.get('source_stamp_ns') for r in rows]
        positive_source = [v for v in source if isinstance(v, int) and v > 0]
        span = (max(valid_mono)-min(valid_mono))*1e-9 if len(valid_mono)>1 else None
        topic_metrics[topic] = dict(count=len(rows), publishers=dict(Counter(r.get('publisher') for r in rows)),
            receipt_span_s=span, observed_rate_hz=(len(rows)-1)/span if span and span>0 else None,
            receipt_interval_ms=stats((b-a)*1e-6 for a,b in zip(valid_mono,valid_mono[1:])),
            receipt_nonincreasing=sum(b<=a for a,b in zip(valid_mono,valid_mono[1:])),
            missing_source_stamp=len(rows)-len(positive_source),
            duplicate_source_stamp=len(positive_source)-len(set(positive_source)),
            nonincreasing_source=sum(b<=a for a,b in zip(positive_source,positive_source[1:])),
            source_to_collector_age_ms=stats((r['receipt_ros_ns']-r['source_stamp_ns'])*1e-6 for r in rows
                if isinstance(r.get('receipt_ros_ns'), int) and isinstance(r.get('source_stamp_ns'), int) and r['source_stamp_ns']>0),
            source_future_count=sum(isinstance(r.get('source_stamp_ns'), int) and r['source_stamp_ns']>r['receipt_ros_ns'] for r in rows),
            raw_identical_adjacent_payloads=sum(a.get('payload')==b.get('payload') for a,b in zip(rows,rows[1:])),
            hardware_acquisition_timestamp=None,
            repeated_payload_interpretation='May be stationary or cached; no hardware acquisition/new-frame inference')
    imu = grouped.get('/imu', []); wheel = grouped.get('/car_message', []); command = grouped.get('/cmd_vel', [])
    axes = ('x', 'y', 'z')
    gyro = {k: stats(field(r.get('payload', {}), 'angular_velocity.'+k) for r in imu) for k in axes}
    accel = {k: stats(field(r.get('payload', {}), 'linear_acceleration.'+k) for r in imu) for k in axes}
    w = stats(field(r.get('payload', {}), 'back_wheel_speed') for r in wheel)
    steering = stats(field(r.get('payload', {}), 'turn_angle') for r in wheel)
    qnorm = []
    for r in imu:
        q = [field(r.get('payload', {}), 'orientation.'+k) for k in ('x','y','z','w')]
        if all(isinstance(v, (int,float)) and math.isfinite(v) for v in q):
            qnorm.append(sum(v*v for v in q))
    warnings = []
    if not wheel:
        warnings.append('wheel_feedback_missing')
    elif w['count']!=len(wheel) or max(abs(w['minimum']), abs(w['maximum']))>5.:
        warnings.append('reported_wheel_not_consistent_with_stationary_or_invalid')
    if not imu:
        warnings.append('IMU_missing')
    elif any(v['count']!=len(imu) or max(abs(v['minimum']),abs(v['maximum']))>.05 for v in gyro.values()):
        warnings.append('gyro_not_consistent_with_low_rotation_or_invalid')
    for r in command:
        if abs(field(r['payload'],'linear.x') or 0.)>.001:
            warnings.append('nonzero_speed_command_observed');break
    imu_health=topic_metrics.get('/imu', {})
    if imu_health.get('source_future_count',0) or imu_health.get('nonincreasing_source',0):
        warnings.append('IMU_driver_Header_clock_anomaly')
    writer=manifest.get('writer', {})
    complete=bool(manifest.get('state')=='closed' and writer.get('accepted')==writer.get('written')==len(events)
                  and writer.get('queue_dropped')==0 and writer.get('unwritten')==0 and writer.get('writer_error') is None)
    if not complete:
        warnings.append('incomplete_local_recording')
    blocks=[]
    if imu:
        begin=min(r['receipt_monotonic_ns'] for r in imu)
        for i in range(3):
            selected=[r for r in imu if i*10 <= (r['receipt_monotonic_ns']-begin)*1e-9 < (i+1)*10]
            blocks.append(dict(index=i, elapsed_window_s=[i*10,(i+1)*10], samples=len(selected),
                gyro_mean_std={k:stats(field(r['payload'],'angular_velocity.'+k) for r in selected) for k in axes}))
    return dict(schema_version=1, run_id=manifest.get('run_id'), continuous_session_id=manifest.get('continuous_session_id'),
        data_origin=manifest.get('data_origin'), category='A_stationary_raw_trial',
        local_recording_complete=complete, event_count=len(events), topics=topic_metrics,
        reported_wheel_raw=w, reported_steering_rad_driver_convention=steering,
        chassis_error_flags=dict(Counter(str(field(r['payload'],'error_flag')) for r in wheel)),
        chassis_battery_reported=stats(field(r['payload'],'battery_level') for r in wheel),
        IMU_driver_frame_ids=sorted(set(r.get('frame_id','') for r in imu)),
        angular_velocity_driver_reported_radps=gyro, acceleration_driver_reported_mps2=accel,
        IMU_orientation_quaternion_norm_squared=stats(qnorm), gyro_ten_second_blocks=blocks,
        observed_stationary_consistency=not warnings, stationary_consistency_warnings=warnings,
        stationary_independently_verified=False,
        gyro_bias_calibration=dict(value=None,valid=False,
            reason='Driver Header is not device acquisition; body installation, independent stationarity and temperature/stability repetitions unverified. Means above are descriptive only.'),
        accelerometer_bias_calibration=dict(value=None,valid=False,reason='gravity_vector_orientation_and_independent_acceleration_reference_unverified'),
        NED_observations=len(grouped.get('/NED_odometry',[])), NED_velocity_independence_verified=False,
        command_events=len(command), control_history_reconstructed=False,
        physical_calibration_completed=False,
        feasibility=('raw_wheel_steering_IMU_capture_and_immediate_audit_available'
                     if complete and imu and wheel else 'capture_chain_incomplete'),
        scope='No claim of full category-A calibration, command completeness, transport losslessness or hardware timing. No interpolation or parameter application.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    manifest=json.loads((args.run_dir/'manifest.json').read_text())
    raw=(args.run_dir/'events.jsonl').read_bytes()
    rows=[json.loads(x) for x in raw.decode().splitlines() if x.strip()]
    out=analyze(rows,manifest)
    out.update(events_sha256=hashlib.sha256(raw).hexdigest(),manifest_sha256=hashlib.sha256((args.run_dir/'manifest.json').read_bytes()).hexdigest())
    write_new(args.output,out)
    print(out['feasibility'])


if __name__=='__main__':
    main()
