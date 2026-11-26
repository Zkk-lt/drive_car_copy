"""Passive raw-event audit; no ROS imports, no state fusion or control outputs.

An event_record receipt is the *collector callback* time, not a hardware
acquisition time and not proof that S8 had received that sample.  This module
keeps those distinctions even when a caller supplies a calibration contract.
"""
from __future__ import division
import base64
import bisect
import csv
import hashlib
import json
import math
import os
from collections import Counter


RAW_TOPICS = ('/car_message', '/carvel', '/NED_odometry', '/imu', '/cmd_vel',
              '/planner/reference_path', '/control_research/controller',
              '/control_research/adaptive', '/control_research/validation')
STATE_TOPICS = ('/car_message', '/NED_odometry', '/imu')
CONTRACT_NAME = 'p42_raw_event_contract_v1'
LOCAL_USE_REQUIREMENTS = {
    'local_relative_reference_and_command': (
        ('/car_message', ('back_wheel_speed', 'turn_angle')),
        ('/imu', ('angular_velocity.z',)), ('/cmd_vel', ('linear.x', 'angular.z')),
        ('/planner/reference_path', ())),
    'local_yaw_response_signals': (
        ('/car_message', ('back_wheel_speed', 'turn_angle')), ('/imu', ('angular_velocity.z',))),
    'local_steering_response_signals': (
        ('/car_message', ('turn_angle',)), ('/cmd_vel', ('angular.z',))),
}
LOCAL_SIGNAL_TYPES = {'/car_message': 'move_car/car_parameter', '/imu': 'sensor_msgs/Imu',
                      '/cmd_vel': 'geometry_msgs/Twist', '/planner/reference_path': 'nav_msgs/Path'}


def positive_integer(value):
    return isinstance(value, int) and not isinstance(value, bool) and value > 0


def finite_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def default_contract():
    """Unverified by default. Values are reported conventions, not calibration."""
    return {'schema_version': 1, 'contract_name': CONTRACT_NAME,
            'source_clock_verified': False,
            'receipt_ros_clock_verified': False,
            'collector_receipt_bounds_controller_availability_verified': False,
            'session_identity_verified': False,
            'independent_ground_speed_verified': False,
            'control_stamp_semantics': 'controller_callback_entry_ros',
            'pose_transport_strictly_causal_verified': False,
            'evidence': [],
            'topics': {
                '/car_message': {'max_age_ns': 150000000, 'source_time_kind': 'unknown',
                    'reported_units': {'back_wheel_speed': 'mm/s_driver_convention_unverified',
                                       'turn_angle': 'rad_driver_convention_unverified'},
                    'units_verified': False, 'frame_verified': False,
                    'hardware_freshness_verified': False},
                '/carvel': {'max_age_ns': 150000000, 'source_time_kind': 'unknown',
                    'reported_units': {'linear.x': 'mm/s_same_cached_CAN',
                                       'angular.z': 'steering_rad_NOT_yaw_rate'},
                    'units_verified': False, 'frame_verified': False,
                    'hardware_freshness_verified': False},
                '/NED_odometry': {'max_age_ns': 150000000,
                    'source_time_kind': 'driver_receipt_unverified',
                    'reported_units': {'position': 'm_assumed', 'linear_velocity': 'm/s_assumed'},
                    'twist_expression': 'unverified', 'units_verified': False,
                    'frame_verified': False, 'hardware_freshness_verified': False},
                '/imu': {'max_age_ns': 150000000, 'source_time_kind': 'driver_receipt_unverified',
                    'reported_units': {'linear_acceleration': 'm/s^2_assumed', 'angular_velocity': 'rad/s_assumed'},
                    'units_verified': False, 'frame_verified': False,
                    'hardware_freshness_verified': False}}}


def _get(value, path):
    for key in path.split('.'):
        if not isinstance(value, dict) or key not in value:
            return None
        value = value[key]
    return value


def local_signal_reasons(event, logical_topic, fields):
    """Descriptive-field contract only; does not verify units or physical state."""
    reasons = []
    if event.get('ros_type') != LOCAL_SIGNAL_TYPES.get(logical_topic):
        reasons.append('unexpected_signal_ROS_type:' + logical_topic)
    payload = event.get('payload')
    if not isinstance(payload, dict) or any(not finite_number(_get(payload, f)) for f in fields):
        reasons.append('missing_or_nonfinite_signal:' + logical_topic)
    if logical_topic == '/planner/reference_path' and not isinstance(_get(payload, 'poses'), list):
        reasons.append('missing_path_pose_array:' + logical_topic)
    return reasons


def _raw_fields(topic, payload):
    # Deliberately retain independent source values, never vehicle.speed.
    paths = {'/car_message': ('back_wheel_speed', 'turn_angle'),
             '/carvel': ('linear.x', 'linear.y', 'linear.z', 'angular.z'),
             '/cmd_vel': ('linear.x', 'angular.z'),
             '/imu': tuple(p + '.' + axis for p in ('linear_acceleration', 'angular_velocity')
                           for axis in ('x', 'y', 'z')),
             '/NED_odometry': tuple(p + '.' + axis for p in
                    ('pose.pose.position', 'twist.twist.linear', 'twist.twist.angular')
                    for axis in ('x', 'y', 'z')) +
                    tuple('pose.pose.orientation.' + axis for axis in ('x', 'y', 'z', 'w'))}
    return {path: _get(payload, path) for path in paths.get(topic, ())}


def _topic_aliases(manifest):
    settings = (manifest.get('ros_parameters_start') or {}).get('/publish_speed_turn') or {}
    if not isinstance(settings, dict):
        settings = {}
    keys = {'/car_message': 'car_topic', '/carvel': 'carvel_topic', '/imu': 'imu_topic',
            '/NED_odometry': 'ned_odom_topic', '/cmd_vel': 'cmd_topic',
            '/planner/reference_path': 'reference_topic'}
    return {name: settings.get(key, name) for name, key in keys.items()}


def _schema_reasons(event, manifest):
    reasons = []
    if event.get('schema_version') != 1:
        reasons.append('unsupported_event_schema')
    if not event.get('run_id') or event.get('run_id') != manifest.get('run_id'):
        reasons.append('run_id_mismatch')
    for name in ('receipt_ros_ns', 'receipt_wall_ns', 'receipt_monotonic_ns'):
        if not positive_integer(event.get(name)):
            reasons.append('invalid_' + name)
    source = event.get('source_stamp_ns')
    if source is not None and source != 0 and not positive_integer(source):
        reasons.append('invalid_source_stamp_ns')
    if positive_integer(source) and positive_integer(event.get('receipt_ros_ns')) and source > event['receipt_ros_ns']:
        reasons.append('source_stamp_after_collector_receipt')
    if not isinstance(event.get('payload'), dict):
        reasons.append('missing_raw_payload')
    try:
        raw = base64.b64decode(event.get('ros_serialized_b64', '').encode('ascii'), validate=True)
        if not raw:
            reasons.append('missing_exact_ros_serialization')
    except (ValueError, TypeError, UnicodeError, AttributeError):
        reasons.append('invalid_exact_ros_serialization')
    return reasons


def _sample_reasons(event, contract, stamp_ns, reused=False):
    topic = event.get('_logical_topic', event.get('topic'))
    cfg = contract.get('topics', {}).get(event.get('topic'), contract.get('topics', {}).get(topic, {}))
    reasons = list(event.get('_audit_reasons', []))
    receipt = event.get('receipt_ros_ns')
    source = event.get('source_stamp_ns')
    if not positive_integer(receipt) or receipt > stamp_ns:
        reasons.append('future_or_missing_collector_receipt')
    max_age = cfg.get('max_age_ns', 150000000)
    if not positive_integer(max_age):
        reasons.append('invalid_max_age_contract')
    elif positive_integer(receipt) and stamp_ns - receipt > max_age:
        reasons.append('collector_receipt_stale')
    if positive_integer(source):
        if source > stamp_ns:
            reasons.append('source_after_control_stamp')
        if positive_integer(max_age) and stamp_ns - source > max_age:
            reasons.append('source_stamp_stale')
    else:
        reasons.append('source_stamp_unknown')
    if contract.get('receipt_ros_clock_verified') is not True:
        reasons.append('receipt_ros_clock_unverified')
    if contract.get('source_clock_verified') is not True:
        reasons.append('source_clock_unverified')
    if contract.get('collector_receipt_bounds_controller_availability_verified') is not True:
        reasons.append('controller_availability_unverified')
    if cfg.get('source_time_kind') != 'hardware_acquisition':
        reasons.append('hardware_acquisition_time_unknown')
    if cfg.get('hardware_freshness_verified') is not True:
        reasons.append('hardware_sample_freshness_unverified')
    if cfg.get('units_verified') is not True:
        reasons.append('units_unverified')
    if cfg.get('frame_verified') is not True:
        reasons.append('coordinate_frame_unverified')
    if topic == '/imu' and not event.get('frame_id'):
        reasons.append('imu_frame_id_missing')
    expected_frame = cfg.get('expected_frame_id')
    if expected_frame and event.get('frame_id') != expected_frame:
        reasons.append('frame_id_contract_mismatch')
    if topic == '/NED_odometry':
        payload = event.get('payload', {})
        if not event.get('frame_id') or not payload.get('child_frame_id'):
            reasons.append('odometry_frame_ids_missing')
        if cfg.get('expected_child_frame_id') and payload.get('child_frame_id') != cfg['expected_child_frame_id']:
            reasons.append('child_frame_id_contract_mismatch')
        if cfg.get('twist_expression') not in ('child_body', 'parent_world'):
            reasons.append('odometry_twist_expression_unknown')
        q = [_get(payload, 'pose.pose.orientation.' + a) for a in ('x', 'y', 'z', 'w')]
        if not all(finite_number(v) for v in q) or abs(sum(v*v for v in q)-1.) > .02:
            reasons.append('odometry_orientation_invalid')
        if contract.get('independent_ground_speed_verified') is not True:
            reasons.append('ground_speed_independence_unverified')
    if reused:
        reasons.append('reused_without_new_sample')
    raw_fields = _raw_fields(topic, event.get('payload', {}))
    if raw_fields and not all(finite_number(v) for v in raw_fields.values()):
        reasons.append('raw_numeric_field_missing_or_nonfinite')
    return sorted(set(reasons))


def _event_reference(event):
    return {'line': event['_line'], 'event_sha256': event['_line_sha256'],
            'source_stamp_ns': event.get('source_stamp_ns'),
            'hardware_acquisition_stamp_ns': event.get('_hardware_stamp_ns'),
            'receipt_ros_ns': event.get('receipt_ros_ns'),
            'receipt_wall_ns': event.get('receipt_wall_ns'),
            'receipt_monotonic_ns': event.get('receipt_monotonic_ns'),
            'receipt_provenance': 'passive_collector_callback_entry_not_driver_or_controller_receive',
            'frame_id': event.get('frame_id'), 'publisher': event.get('publisher'), 'topic': event.get('topic'),
            'ros_type': event.get('ros_type'), 'raw_values': _raw_fields(event.get('_logical_topic', event.get('topic')), event.get('payload', {})),
            'ros_serialized_sha256': event.get('_serialized_sha256')}


def audit_events(events, manifest, contract=None, include_snapshots=False):
    """Audit event_record dicts. Contract assertions do not certify hardware.

    Selection uses collector receipt <= the control entry stamp AND source <=
    that stamp. No pose interpolation or backdating is performed. Reusing a
    sample is exposed and rejected as a fresh observation, rather than being
    relabelled with the current snapshot time.
    """
    contract = default_contract() if contract is None else contract
    if not isinstance(contract, dict) or contract.get('schema_version') != 1:
        raise ValueError('unsupported causal data contract')
    if not isinstance(contract.get('topics', {}), dict) or any(
            not isinstance(value, dict) for value in contract.get('topics', {}).values()):
        raise ValueError('contract topics must be objects')
    coverage, by_topic, invalid_events, snapshots = {}, {}, [], []
    aliases = _topic_aliases(manifest)
    inverse_aliases = {actual: logical for logical, actual in aliases.items()}
    seen, last_source, last_receipt, last_mono = set(), {}, {}, {}
    all_events = []
    for index, original in enumerate(events, 1):
        event = dict(original)
        topic = event.get('topic', '<missing_topic>')
        if not isinstance(topic, str):
            topic = '<invalid_topic>'
            event['topic'] = topic
        event['_logical_topic'] = inverse_aliases.get(topic, topic)
        event.setdefault('_line', index)
        encoded = json.dumps(original, sort_keys=True, allow_nan=True).encode('utf8')
        event.setdefault('_line_sha256', hashlib.sha256(encoded).hexdigest())
        reasons = _schema_reasons(event, manifest)
        try:
            serialized = base64.b64decode(event.get('ros_serialized_b64', '').encode('ascii'), validate=True)
            event['_serialized_sha256'] = hashlib.sha256(serialized).hexdigest()
        except (ValueError, TypeError, UnicodeError, AttributeError):
            event['_serialized_sha256'] = None
        source, receipt, mono = (event.get(k) for k in
                               ('source_stamp_ns', 'receipt_ros_ns', 'receipt_monotonic_ns'))
        identity = json.dumps([topic, source, receipt, mono, event.get('_serialized_sha256')], sort_keys=True)
        if identity in seen:
            reasons.append('duplicate_event_identity')
        seen.add(identity)
        if positive_integer(source):
            if topic in last_source and source <= last_source[topic]:
                reasons.append('repeated_or_reversed_source_stamp')
            last_source[topic] = source
        if positive_integer(receipt):
            if topic in last_receipt and receipt < last_receipt[topic]:
                reasons.append('receipt_ros_clock_reversal')
            last_receipt[topic] = receipt
        if positive_integer(mono):
            if topic in last_mono and mono < last_mono[topic]:
                reasons.append('receipt_monotonic_reversal')
            last_mono[topic] = mono
        cfg = contract.get('topics', {}).get(topic, contract.get('topics', {}).get(event['_logical_topic'], {}))
        # A Header is not promoted into acquisition time without an explicit
        # per-topic contract. Headerless CAN remains null even under a generic
        # source-clock assertion.
        event['_hardware_stamp_ns'] = source if positive_integer(source) and \
            cfg.get('source_time_kind') == 'hardware_acquisition' else None
        event['_audit_reasons'] = sorted(set(reasons))
        state = coverage.setdefault(topic, {'count': 0, 'ros_types': Counter(),
            'source_stamp_count': 0, 'hardware_acquisition_stamp_count': 0,
            'raw_serialization_count': 0, 'source_frames': Counter(), 'invalid_events': Counter(),
            'unit_contract': cfg.get('reported_units', {}),
            'units_verified_asserted': cfg.get('units_verified') is True,
            'source_time_kind_asserted': cfg.get('source_time_kind', 'unknown')})
        state['count'] += 1
        state['ros_types'][str(event.get('ros_type'))] += 1
        state['source_frames'][str(event.get('frame_id'))] += 1
        state['source_stamp_count'] += int(positive_integer(source))
        state['hardware_acquisition_stamp_count'] += int(positive_integer(event['_hardware_stamp_ns']))
        state['raw_serialization_count'] += int(bool(event.get('ros_serialized_b64')) and not any(
            reason in reasons for reason in ('missing_exact_ros_serialization', 'invalid_exact_ros_serialization')))
        state['invalid_events'].update(reasons)
        if reasons:
            invalid_events.append(dict(_event_reference(event), topic=topic, reasons=event['_audit_reasons']))
        by_topic.setdefault(topic, []).append(event)
        all_events.append(event)
    for state in coverage.values():
        for key in ('ros_types', 'source_frames', 'invalid_events'):
            state[key] = dict(state[key])
    source_lists = {}
    for topic in STATE_TOPICS:
        records = [e for e in by_topic.get(aliases.get(topic, topic), []) if positive_integer(e.get('receipt_ros_ns'))]
        records.sort(key=lambda e: (e['receipt_ros_ns'], e['_line']))
        source_lists[topic] = ([e['receipt_ros_ns'] for e in records], records)
    controls = [e for e in all_events if e.get('ros_type') == 'move_car/ControlResearchTelemetry']
    selected_counts, rejected_counts, future_excluded = Counter(), Counter(), Counter()
    previous_selected = {}
    invalid_controls = 0
    for control in controls:
        stamp = control.get('source_stamp_ns')
        if not positive_integer(stamp) or control.get('_audit_reasons'):
            invalid_controls += 1
            continue
        row = {'control': _event_reference(control), 'control_stamp_semantics':
               contract.get('control_stamp_semantics', 'unverified'), 'samples': {}}
        for topic in STATE_TOPICS:
            times, records = source_lists[topic]
            idx = bisect.bisect_right(times, stamp) - 1
            # Source-future messages are not substituted as historical data.
            # Record how many were excluded at the leading edge of the history.
            excluded = []
            while idx >= 0:
                source = records[idx].get('source_stamp_ns')
                if not positive_integer(source) or source <= stamp:
                    break
                excluded.append(records[idx]['_line'])
                idx -= 1
            future_excluded[topic] += len(excluded)
            if idx < 0:
                reasons = ['no_arrival_causal_sample']
                selected = None
            else:
                event = records[idx]
                reused = previous_selected.get(topic) == event['_line']
                previous_selected[topic] = event['_line']
                reasons = _sample_reasons(event, contract, stamp, reused)
                selected = _event_reference(event)
                selected['age_from_collector_receipt_ns'] = stamp - event['receipt_ros_ns']
                cfg = contract.get('topics', {}).get(event.get('topic'), contract.get('topics', {}).get(topic, {}))
                selected['unit_contract'] = cfg.get('reported_units', {})
                selected['source_time_kind_asserted'] = cfg.get('source_time_kind', 'unknown')
                selected_counts[topic] += 1
            rejected_counts.update(reasons)
            row['samples'][topic] = {'selected': selected, 'eligible_fresh_raw_observation': not reasons,
                                     'invalid_reasons': reasons, 'source_future_excluded_lines': excluded}
        snapshots.append(row)
    missing = []
    if not all_events:
        missing.append('missing_raw_events')
    expected = set(aliases.get(topic, topic) for topic in RAW_TOPICS)
    expected.update(t.get('name') for t in manifest.get('topics', []) if t.get('name'))
    missing += ['missing_topic:' + t for t in sorted(expected - set(coverage))]
    session_id = manifest.get('continuous_session_id') or manifest.get('session_id')
    if not session_id or contract.get('session_identity_verified') is not True:
        missing.append('session_identity_unverified')
    if manifest.get('data_origin') != 'real_vehicle':
        missing.append('real_vehicle_provenance_not_verified')
    for flag, reason in (('source_clock_verified', 'source_clock_unverified'),
                         ('receipt_ros_clock_verified', 'receipt_ros_clock_unverified'),
                         ('independent_ground_speed_verified', 'independent_ground_speed_unverified'),
                         ('collector_receipt_bounds_controller_availability_verified', 'controller_availability_unverified'),
                         ('pose_transport_strictly_causal_verified', 'pose_transport_not_strictly_causal_verified')):
        if contract.get(flag) is not True:
            missing.append(reason)
    for topic in STATE_TOPICS:
        cfg = contract.get('topics', {}).get(aliases.get(topic, topic), contract.get('topics', {}).get(topic, {}))
        if cfg.get('units_verified') is not True:
            missing.append('units_unverified:' + topic)
        if cfg.get('frame_verified') is not True:
            missing.append('coordinate_frame_unverified:' + topic)
        if cfg.get('hardware_freshness_verified') is not True:
            missing.append('hardware_sample_freshness_unverified:' + topic)
    can_contract = contract.get('topics', {}).get(aliases.get('/car_message', '/car_message'),
                    contract.get('topics', {}).get('/car_message', {}))
    if can_contract.get('source_time_kind') != 'hardware_acquisition' or any(
            e.get('_logical_topic') == '/car_message' and not e.get('_hardware_stamp_ns') for e in all_events):
        missing.append('CAN_acquisition_time_unknown')
    writer = manifest.get('writer', {})
    if manifest.get('state') != 'closed' or not writer or any(writer.get(k) for k in
            ('writer_error', 'unwritten', 'queue_dropped')):
        missing.append('recording_completeness_not_established')
    temporal_eligible = sum(s['eligible_fresh_raw_observation']
                            for row in snapshots for s in row['samples'].values())
    result = {'schema_version': 1, 'run_id': manifest.get('run_id'), 'session_id': session_id,
              'data_origin': manifest.get('data_origin', 'unknown'),
              'terrain_label': manifest.get('terrain_label', 'unknown'),
              'status': 'diagnostic_only', 'identification_ready': False,
              'missing': sorted(set(missing)), 'topic_coverage': coverage,
              'topic_aliases': aliases,
              'causal_summary': {'control_snapshot_count': len(controls), 'invalid_control_snapshots': invalid_controls,
                'audited_snapshots': len(snapshots), 'selected_samples': dict(selected_counts),
                'eligible_fresh_raw_observations': temporal_eligible,
                'rejection_reasons': dict(rejected_counts), 'source_future_excluded': dict(future_excluded),
                'selection_clock': 'collector_receipt_ros_ns <= control_entry_stamp; source_stamp <= control_entry_stamp',
                'controller_availability_proven': False,
                'pose_interpolation_performed': False},
              'invalid_events': invalid_events,
              'contract_assertions_are_not_certificates': True,
              'contract_sha256': hashlib.sha256(json.dumps(contract, sort_keys=True, allow_nan=False).encode('utf8')).hexdigest(),
              'raw_preservation': 'original events.jsonl is read-only; references include original line SHA and serialized SHA',
              'physical_calibration_or_closed_loop_claim': False}
    # Keep strict source-time/independent-ground diagnostics above unchanged.
    # They do not invalidate unrelated raw relative-feedback observations.
    complete = (manifest.get('state') == 'closed' and
                writer.get('accepted') == writer.get('written') == len(all_events) and
                isinstance(writer.get('written'), int) and not isinstance(writer.get('written'), bool) and
                writer.get('queue_dropped') == 0 and writer.get('unwritten') == 0 and
                writer.get('writer_error') is None)
    def local_use(requirements):
        reasons = []
        if not complete:
            reasons.append('local_recording_completeness_not_established')
        counts = {}
        for logical, fields in requirements:
            actual = aliases.get(logical, logical)
            rows = by_topic.get(actual, [])
            counts[logical] = len(rows)
            if len(rows) < 2:
                reasons.append('insufficient_raw_samples:' + logical)
            elif any(e.get('_audit_reasons') for e in rows):
                reasons.append('invalid_raw_event_or_receipt_time:' + logical)
            for e in rows:
                reasons.extend(local_signal_reasons(e, logical, fields))
        return dict(available_for_descriptive_signal_analysis=not reasons,
                    required_raw_sample_counts=counts, missing=sorted(set(reasons)),
                    NED_required=False, independent_ground_speed_required=False,
                    physical_parameter_identified=False, controller_state_certified=False,
                    limitation='Raw receipt-domain signals only. Check excitation, signal filtering, '
                    'axis/units/zero and per-interval timing before fitting; no physical delay or control-clock claim.')
    result['data_use_assessments'] = {
        'local_relative_reference_and_command': local_use(LOCAL_USE_REQUIREMENTS['local_relative_reference_and_command']),
        'local_yaw_response_signals': local_use(LOCAL_USE_REQUIREMENTS['local_yaw_response_signals']),
        'local_steering_response_signals': local_use(LOCAL_USE_REQUIREMENTS['local_steering_response_signals']),
        'physical_actuator_calibration': dict(valid=False, NED_required=False,
            reason='Requires verified actual command/feedback semantics, input completeness, endpoint timing, '
                   'excitation and held-out validation. Independent ground speed is not required for steering.'),
        'true_longitudinal_slip_extension': dict(valid=False, required_for_local_control=False,
            requires='independent ground speed, wheel scale/sign, frame/time alignment and low-speed observability; '
                     'NED topic presence alone does not satisfy this'),
    }
    result['missing_scope'] = 'legacy_full_state_source_time_contract_not_global_local_data_rejection'
    result['global_pose_required_for_local_feedback'] = False
    if include_snapshots:
        result['snapshots'] = snapshots
    return result


def audit_run(run_directory, contract=None, include_snapshots=False):
    """Audit the existing passive-recorder directory; never rewrite its files."""
    manifest_path = os.path.join(run_directory, 'manifest.json')
    event_path = os.path.join(run_directory, 'events.jsonl')
    manifest = {}
    if os.path.isfile(manifest_path):
        with open(manifest_path) as stream:
            manifest = json.load(stream)
    events, parse_failures = [], []
    if os.path.isfile(event_path):
        with open(event_path, 'rb') as stream:
            for line_no, line in enumerate(stream, 1):
                try:
                    def reject_constant(value):
                        raise ValueError('nonfinite JSON token: ' + value)
                    event = json.loads(line.decode('utf8'), parse_constant=reject_constant)
                    if not isinstance(event, dict):
                        raise ValueError('event is not an object')
                    event['_line'] = line_no
                    event['_line_sha256'] = hashlib.sha256(line).hexdigest()
                    events.append(event)
                except (ValueError, UnicodeError) as exc:
                    parse_failures.append({'line': line_no, 'error': str(exc), 'sha256': hashlib.sha256(line).hexdigest()})
    result = audit_events(events, manifest, contract, include_snapshots)
    result.update({'run_directory': os.path.abspath(run_directory),
                   'events_sha256': sha256_file(event_path) if os.path.isfile(event_path) else None,
                   'manifest_sha256': sha256_file(manifest_path) if os.path.isfile(manifest_path) else None,
                   'parse_failures': parse_failures})
    if parse_failures:
        result['missing'].append('corrupt_or_incomplete_event_lines')
    for item in result.get('data_use_assessments', {}).values():
        if 'available_for_descriptive_signal_analysis' not in item:
            continue
        if parse_failures:
            item['missing'].append('corrupt_or_incomplete_event_lines')
        if manifest.get('events_sha256') and manifest['events_sha256'] != result['events_sha256']:
            item['missing'].append('raw_event_hash_mismatch')
        item['available_for_descriptive_signal_analysis'] = not item['missing']
    return result


def _csv_summary(path):
    fields, count, time_min, time_max = [], 0, None, None
    try:
        with open(path, newline='') as stream:
            reader = csv.DictReader(stream)
            fields = reader.fieldnames or []
            for row in reader:
                count += 1
                value = row.get('stamp', row.get('time'))
                try:
                    value = float(value)
                    if math.isfinite(value):
                        time_min = value if time_min is None else min(value, time_min)
                        time_max = value if time_max is None else max(value, time_max)
                except (TypeError, ValueError):
                    pass
        return {'columns': fields, 'rows': count, 'time_min': time_min, 'time_max': time_max}
    except (OSError, UnicodeError, csv.Error) as exc:
        return {'read_error': str(exc)}


def workspace_inventory(root):
    """Hash data candidates and classify by evidence, not a real_vehicle flag alone.

    Only the explicit P37 legacy hash allowlist is recognized as user-confirmed
    indoor evidence. Other old folders are quarantined until provenance is
    reviewed. Build/results fixtures cannot self-certify as real data.
    """
    root = os.path.abspath(root)
    scope = os.path.join(root, 'src/move_car/docs/control_research/results/p37/calibration/review_20261109/legacy_scene_scope.json')
    legacy = {}
    if os.path.isfile(scope):
        with open(scope) as stream:
            legacy = {r['path']: r for r in json.load(stream).get('runs', [])}
    rows = []
    for directory, names, files in os.walk(root):
        names[:] = sorted(n for n in names if n not in ('.git', '.codex', '.agents', '__pycache__'))
        for name in sorted(files):
            path = os.path.join(directory, name)
            relative = os.path.relpath(path, root).replace(os.sep, '/')
            # No generated audit evidence is recursively inventoried.
            if relative.startswith('src/move_car/docs/control_research/results/p42/') or relative.startswith('build/p42/'):
                continue
            legacy_text_root = relative.split('/')[0].startswith(('data', 'cdata', 'run_archives', 'run_manifests'))
            if not (name.endswith(('.csv', '.bag', '.jsonl')) or name in ('manifest.json', 'tracking_run_manifest.txt') or
                    (legacy_text_root and name.endswith(('.txt', '.log')))):
                continue
            if not os.path.isfile(path) or os.path.islink(path):
                continue
            digest = sha256_file(path)
            if relative in legacy and digest == legacy[relative].get('sha256'):
                origin, reason = 'user_confirmed_indoor_smooth_legacy_control', 'P37_allowlist_hash_matches_user_scope'
            elif relative.startswith('build/') or '/results/' in relative or '/test/' in relative or '/synthetic' in relative:
                origin, reason = 'software_test_or_fixture_not_vehicle_evidence', 'test_build_or_archived_test_location'
            elif relative.startswith(('data/', 'cdata/')):
                origin, reason = 'quarantined_legacy_unverified', 'directory_or_timestamp_does_not_establish_provenance_or_session'
            else:
                origin, reason = 'unverified_data_candidate', 'requires_manual_provenance_review'
            row = {'path': relative, 'bytes': os.path.getsize(path), 'sha256': digest,
                   'data_class': origin, 'classification_reason': reason,
                   'eligible_for_parameter_identification': False, 'session_id': None}
            if name.endswith('.csv'):
                row.update(_csv_summary(path))
            if name == 'manifest.json':
                try:
                    with open(path) as stream:
                        m = json.load(stream)
                    row['declared_manifest'] = {k: m.get(k) for k in ('run_id', 'continuous_session_id',
                        'terrain_label', 'data_origin', 'actual_control_mode', 'state')}
                except (ValueError, UnicodeError) as exc:
                    row['read_error'] = str(exc)
            rows.append(row)
    classes = Counter(r['data_class'] for r in rows)
    return {'schema_version': 1, 'workspace': root, 'files': rows, 'file_count': len(rows),
            'classes': dict(classes), 'legacy_scope_sha256': sha256_file(scope) if os.path.isfile(scope) else None,
            'user_scene_scope': 'all_confirmed_real_vehicle_logs_same_indoor_smooth; not grass/asphalt evidence',
            'session_inference': 'never inferred from filename/timestamp; unconfirmed files quarantined',
            'identification_ready_runs': 0, 'physical_calibration_claim': False}
