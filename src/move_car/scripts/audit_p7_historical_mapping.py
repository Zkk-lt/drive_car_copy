#!/usr/bin/env python3
"""Read-only, causal receipt-domain audit of historical real smooth sensors.

Writes a NEW evidence directory; never publishes ROS, calibrates physical
hardware, modifies Config, or reconstructs absent command events.
"""
import argparse
import bisect
import collections
import datetime
import hashlib
import json
import math
from pathlib import Path
import re


WORKSPACE = Path(__file__).resolve().parents[3]
DEFAULT_MANIFESTS = (
    'research_runs/p42_stationary_20261109T084222Z/runs/p42_A_20261109T084222Z_r1/manifest.json',
    'research_runs/p42_s8_20261109T091200Z/runs/p42_s8_20261109T091200Z_r1/manifest.json',
    'research_runs/p42_s8_60s_20261109T091603Z/runs/p42_s8_60s_20261109T091603Z_r1/manifest.json',
    'research_runs/p42_s8_extended_20261109T092533Z/runs/p42_s8_extended_20261109T092533Z_r1/manifest.json',
    'research_runs/p42_smooth_final_20261109T094400Z/runs/p42_smooth_final_20261109T094400Z_r1/manifest.json',
)
TOPICS = ('/imu', '/car_message', '/cmd_vel')
TOPIC_RE = re.compile(rb'"topic"\s*:\s*"/(?:imu|car_message|cmd_vel)"')


def digest(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def correlation(x, y):
    if len(x) < 2:
        return None
    mx, my = sum(x) / len(x), sum(y) / len(y)
    xx = sum((v-mx)**2 for v in x)
    yy = sum((v-my)**2 for v in y)
    return sum((a-mx)*(b-my) for a, b in zip(x, y)) / math.sqrt(xx*yy) if xx*yy > 0 else None


def statistics(values):
    if not values:
        return dict(count=0, minimum=None, maximum=None, mean=None)
    return dict(count=len(values), minimum=min(values), maximum=max(values), mean=sum(values)/len(values))


def direction_stats(x, y):
    return dict(count=len(x), same_sign_count=sum(a*b > 0 for a, b in zip(x, y)),
                same_sign_fraction=sum(a*b > 0 for a, b in zip(x, y))/len(x) if x else None,
                pearson=correlation(x, y))


def read_run(manifest_path):
    manifest_path = Path(manifest_path).resolve()
    manifest = json.loads(manifest_path.read_text())
    if manifest.get('data_origin') != 'real_vehicle' or manifest.get('terrain_label') != 'smooth':
        raise ValueError('explicit real_vehicle smooth manifest required: ' + str(manifest_path))
    writer = manifest.get('writer', {})
    if (manifest.get('state') != 'closed' or writer.get('queue_dropped') != 0 or
            writer.get('unwritten') != 0 or writer.get('writer_error') or
            writer.get('accepted') != writer.get('written')):
        raise ValueError('closed locally complete raw writer required: ' + str(manifest_path))
    run_id = manifest['run_id']
    event_path = manifest_path.with_name('events.jsonl')
    frames = collections.Counter()
    publishers = {name: collections.Counter() for name in TOPICS}
    rows = {name: [] for name in TOPICS}
    events_hash = hashlib.sha256()
    event_count = 0
    for line in event_path.open('rb'):
        events_hash.update(line)
        if not line.strip():
            continue
        event_count += 1
        if not TOPIC_RE.search(line):
            continue
        event = json.loads(line)
        if event.get('run_id') != run_id:
            raise ValueError('mixed raw run identity')
        topic, payload = event['topic'], event['payload']
        if topic not in rows:
            continue
        receipt = event.get('receipt_monotonic_ns')
        if not isinstance(receipt, int) or isinstance(receipt, bool) or receipt <= 0:
            raise ValueError('positive monotonic receipt clock required')
        stamp = receipt * 1e-9
        publishers[topic][event.get('publisher', '<unknown>')] += 1
        if topic == '/imu':
            frames[payload.get('header', {}).get('frame_id', '')] += 1
            row = (stamp, payload['angular_velocity']['z'], payload['angular_velocity']['x'],
                   payload['angular_velocity']['y'], payload['linear_acceleration']['z'])
        elif topic == '/car_message':
            row = (stamp, payload['back_wheel_speed'], payload['turn_angle'],
                   payload.get('battery_level', 0), payload.get('error_flag', -1))
        else:
            row = (stamp, payload['linear']['x'], payload['angular']['z'])
        if not all(finite(value) for value in row):
            raise ValueError('nonfinite raw audit input: ' + topic)
        rows[topic].append(row)
    if event_count != writer['written'] or events_hash.hexdigest() != manifest.get('events_sha256'):
        raise ValueError('event count/hash integrity mismatch: ' + str(event_path))
    for stream in rows.values():
        stream.sort(key=lambda row: row[0])
    imu, car, command = (rows[topic] for topic in TOPICS)
    out = dict(run_id=run_id, continuous_session_id=manifest.get('continuous_session_id'),
               data_origin='real_vehicle', terrain='smooth', manifest=str(manifest_path),
               manifest_sha256=digest(manifest_path), events=str(event_path),
               events_sha256=events_hash.hexdigest(), event_count=event_count,
               local_writer_complete=True, transport_losslessness_verified=False,
               imu_frame_counts=dict(frames), publishers={t: dict(p) for t, p in publishers.items()},
               topic_counts={t: len(stream) for t, stream in rows.items()},
               raw_wheel_driver_mmps=statistics([row[1] for row in car]),
               raw_steering_driver_rad=statistics([row[2] for row in car]),
               command_speed_driver_mps=statistics([row[1] for row in command]),
               command_steering_driver_rad=statistics([row[2] for row in command]),
               imu_z_driver_radps=statistics([row[1] for row in imu]),
               imu_acceleration_z_driver_mps2=statistics([row[4] for row in imu]))
    if not command:
        out['dynamic_direction_analysis'] = dict(valid=False, reason='no_control_commands_stationary_run')
        return out
    # Reject invalid/error feedback; accepted rows are still not certified CAN-new frames.
    car = [row for row in car if row[3] > 0 and row[4] == 0]
    imu_times = [row[0] for row in imu]
    command_times = [row[0] for row in command]
    steer, yaw, proxy = [], [], []
    for stamp, wheel, delta, _, _ in car:
        index = bisect.bisect_right(imu_times, stamp)-1
        if index < 0 or stamp-imu_times[index] > .05:
            continue
        r = imu[index][1]
        if wheel < 100 or abs(delta) < .03 or abs(r) < .01:
            continue
        steer.append(delta)
        yaw.append(r)
        proxy.append(.001*wheel*math.tan(delta))
    dynamic = dict(valid=bool(steer), feedback_steer_vs_imu_z=direction_stats(steer, yaw),
                   wheel_proxy_tansteer_vs_imu_z_pearson=correlation(proxy, yaw),
                   interpretation='causal collector receipt-domain engineering association only; no independent world-left turn measurement')
    dynamic['command_vs_feedback_by_receipt_delay_s'] = {}
    for delay in (0., .2, .4, .6):
        x, y = [], []
        for stamp, wheel, delta, _, _ in car:
            index = bisect.bisect_right(command_times, stamp-delay)-1
            if index < 0 or stamp-delay-command_times[index] > .1:
                continue
            cmd = command[index][2]
            if wheel < 100 or abs(delta) < .03 or abs(cmd) < .03:
                continue
            x.append(cmd)
            y.append(delta)
        dynamic['command_vs_feedback_by_receipt_delay_s'][str(delay)] = direction_stats(x, y)
    matched = []
    for stamp, wheel, _, _, _ in car:
        index = bisect.bisect_right(command_times, stamp-.35)-1
        if (index >= 0 and stamp-.35-command_times[index] < .1 and
                command[index][1] > .1 and abs(wheel) > 50):
            matched.append(wheel)
    dynamic['positive_speed_command_vs_wheel_proxy'] = dict(
        count=len(matched), positive_wheel_count=sum(value > 0 for value in matched),
        positive_wheel_fraction=sum(value > 0 for value in matched)/len(matched) if matched else None)
    out['dynamic_direction_analysis'] = dynamic
    return out


def build_report(manifests):
    if len(set(str(Path(p).resolve()) for p in manifests)) != len(manifests):
        raise ValueError('duplicate manifests are not independent evidence')
    runs = [read_run(path) for path in manifests]
    if len({run['run_id'] for run in runs}) != len(runs):
        raise ValueError('duplicate run identity')
    if not runs or any(set(run['imu_frame_counts']) != {'gyro_link'} for run in runs):
        raise ValueError('expected historical gyro_link frame evidence missing or inconsistent')
    expected_publishers = {'/imu': '/ahrs_driver', '/car_message': '/carTop', '/cmd_vel': '/publish_speed_turn'}
    for run in runs:
        for topic, expected in expected_publishers.items():
            if set(run['publishers'][topic]) - {expected}:
                raise ValueError('unexpected historical publisher for ' + topic)
    maxima = [run['command_speed_driver_mps']['maximum'] for run in runs
              if run['command_speed_driver_mps']['count']]
    sources = [
        'research_runs/p42_stationary_20261109T084222Z/LIVE_SENSOR_PROVENANCE.md',
        'research_runs/p42_stationary_20261109T084222Z/hardware_protocol_0.txt',
        'research_runs/p42_stationary_20261109T084222Z/hardware_document_sources.json',
        'research_runs/p42_stationary_20261109T084222Z/driver_source_snapshot/home/ubuntu/drive_car/src/fdilink_ahrs/src/ahrs_driver.cpp',
        'research_runs/p42_stationary_20261109T084222Z/driver_source_snapshot/home/ubuntu/drive_car/src/fdilink_ahrs/include/fdilink_data_struct.h',
        'research_runs/p42_stationary_20261109T084222Z/driver_source_snapshot/home/ubuntu/drive_car/src/move_car/src/movecar.cpp',
        'src/move_car/launch/movecar_improved.launch',
    ]
    return dict(schema_version=1, kind='historical_engineering_mapping_audit',
                generated_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                tool=str(Path(__file__).resolve()), tool_sha256=digest(__file__),
                totals={topic: sum(run['topic_counts'][topic] for run in runs) for topic in TOPICS},
                historical_command_speed_max_mps=max(maxima) if maxima else None,
                requested_speed_mps=.4, requested_speed_historical_closed_loop_verified=False,
                units=dict(imu_gyro='rad/s_driver_declared', imu_accel='m/s^2_driver_declared',
                           wheel='mm/s_driver_declared_signed_proxy', feedback_steering='rad_driver_conversion',
                           command_speed='m/s_S8_driver_convention', command_angular_z='steering_rad_not_yaw_rate'),
                declared_axes=dict(device_type=1, driver_gyro_accel_map='(x,-y,-z)',
                                   downstream_imu_yaw_sign=1., protocol_steering='left_positive_right_negative'),
                selection=dict(imu_lookup='latest receipt <= feedback receipt, no interpolation',
                               imu_max_age_s=.05, minimum_forward_raw_wheel=100,
                               minimum_abs_feedback_steering_rad=.03, minimum_abs_yaw_radps=.01,
                               purpose='descriptive signal excitation selection, not calibrated confidence thresholds'),
                evidence_sources=[dict(path=str(WORKSPACE/path), sha256=digest(WORKSPACE/path)) for path in sources],
                limitations=[
                    'Historical engineering mapping is not independent physical calibration or deployment approval.',
                    'Positive steering/yaw association plus driver protocol does not independently certify world-left turn.',
                    'CAN hardware sampling, true-new-frame flag and actuator physical reception time remain unknown.',
                    'Driver feedback angle may be actual angle or setpoint echo; not independently established.',
                    'No independent ground speed; wheel proxy cannot establish true slip or 0.4 m/s real speed.',
                    'No absent NED samples, lost control events or timestamps are invented or interpolated.',
                    'Receipt lag associations are not physical pure actuator delay; no dynamic model fit is applied.',
                    'Historical S8 command max is below requested 0.4 m/s; new-speed performance remains unverified.',
                    'Independent physical sessions, installation transform and driver binary/source identity remain unverified.',
                    'Stationary means are descriptive; gyro/accelerometer bias is not automatically calibrated.',
                ], runs=runs)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', action='append', help='Explicit real smooth manifest; repeat per run. Defaults to five frozen P42 real runs, never discovery/globbing.')
    parser.add_argument('--output', required=True, help='New evidence directory; refuses overwrite.')
    args = parser.parse_args(argv)
    output = Path(args.output).resolve()
    if output.exists():
        parser.error('output already exists; evidence is immutable: ' + str(output))
    report = build_report(args.manifest or [WORKSPACE/path for path in DEFAULT_MANIFESTS])
    output.mkdir(parents=True)
    report_path = output/'report.json'
    report_path.write_text(json.dumps(report, ensure_ascii=False, sort_keys=True, indent=2)+'\n')
    source_files = {source['path']: source['sha256'] for source in report['evidence_sources']}
    for run in report['runs']:
        source_files[run['manifest']] = run['manifest_sha256']
        source_files[run['events']] = run['events_sha256']
    profile = dict(schema_version=1, kind='historical_engineering_mapping', imu_frame='gyro_link',
                   wheel_speed_scale=.001, feedback_steer_sign=1., imu_yaw_sign=1.,
                   cmd_steer_scale=1., cmd_steer_sign=1., engineering_mapping_supported=True,
                   physical_calibration_certified=False, absolute_left_turn_independently_verified=False,
                   declared_ground_speed_independent=False, hardware_sample_time_verified=False,
                   can_new_frame_verified=False, source_clock_alignment_verified=False,
                   terrain='smooth', imu_angular_velocity_unit='rad/s', imu_acceleration_unit='m/s^2',
                   wheel_raw_unit='mm/s_driver_signed_proxy', feedback_steering_unit='rad',
                   evidence_report=str(report_path), evidence_report_sha256=digest(report_path),
                   tool_sha256=digest(__file__), source_files=source_files, requested_speed_mps=.4,
                   requested_speed_historical_closed_loop_verified=False,
                   declaration='Source/protocol plus causal historical response consistency; no authority, physical calibration, independent slip, or performance certification.')
    (output/'mapping_profile.json').write_text(json.dumps(profile, ensure_ascii=False, sort_keys=True, indent=2)+'\n')
    print(json.dumps(dict(output=str(output), totals=report['totals'], historical_command_speed_max_mps=report['historical_command_speed_max_mps'], engineering_mapping_supported=True, physical_calibration_certified=False), sort_keys=True))


if __name__ == '__main__':
    main()
