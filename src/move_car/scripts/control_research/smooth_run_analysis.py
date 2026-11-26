"""Read-only per-run smooth diagnostics, never Config promotion or calibration.

Raw command events and publisher records remain separate. Receipt-domain
derivatives are communication diagnostics, not physical acceleration/jerk.
"""
import csv
import hashlib
import json
import math
from collections import Counter
from pathlib import Path

from .analysis import number, stats, summarize
from .causal_data_audit import audit_run


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write('\n')


def stamp(value):
    if isinstance(value, dict):
        a, b = value.get('secs'), value.get('nsecs')
        return a+b*1e-9 if number(a) and number(b) else None
    return value if number(value) else None


def absolute(values):
    return stats(abs(x) for x in values if number(x))


def command_diagnostics(rows):
    """Do not resample, interpolate, or infer missing source sequence IDs."""
    intervals, rates, acceleration, jerk = [], [], [], []
    previous = None
    previous_acceleration = None
    nonpositive = 0
    for event in rows:
        p = event.get('payload', {})
        t = event.get('receipt_monotonic_ns')
        v = p.get('linear', {}).get('x')
        d = p.get('angular', {}).get('z')
        if not all(number(x) for x in (t, v, d)):
            previous = previous_acceleration = None
            continue
        t *= 1e-9
        if previous:
            dt = t-previous[0]
            if dt <= 0.:
                nonpositive += 1
                previous_acceleration = None
            else:
                intervals.append(dt*1000.)
                rates.append((d-previous[2])/dt)
                a = (v-previous[1])/dt
                acceleration.append(a)
                if previous_acceleration is not None:
                    jerk.append((a-previous_acceleration)/dt)
                previous_acceleration = a
        previous = (t, v, d)
    return dict(samples=len(rows), receipt_interval_ms=stats(intervals),
        nonpositive_receipt_intervals=nonpositive,
        receipt_domain_steering_rate_abs=absolute(rates),
        receipt_domain_acceleration=stats(acceleration), receipt_domain_jerk=stats(jerk),
        source_sequence_known=False, physical_jerk_identified=False,
        normal_vs_emergency_separation='unavailable_without_aligned_explicit_safety_state',
        limitation='Collector receipt times are not publisher times or physical execution times. No resampling.')


def ledger_diagnostics(events, run_id):
    committed, seen, duplicate, gaps, failed, foreign = [], set(), [], [], [], 0
    sequence_previous = None
    for e in events:
        p = e.get('payload', {})
        if p.get('run_id') != run_id:
            foreign += 1
            continue
        if not p.get('publish_returned') or not p.get('history_committed'):
            failed.append(dict(sequence=p.get('sequence'), reason=p.get('failure_reason')))
            continue
        key = (p.get('epoch'), p.get('sequence'))
        if key in seen:
            duplicate.append(list(key))
            continue
        seen.add(key)
        if sequence_previous and (key[0] != sequence_previous[0] or key[1] != sequence_previous[1]+1):
            gaps.append(dict(previous=list(sequence_previous), current=list(key)))
        sequence_previous = key
        committed.append(p)
    return dict(samples=len(events), committed=len(committed), duplicates=duplicate,
        observed_sequence_discontinuities=gaps, failed_publications=failed,
        foreign_run_events=foreign,
        owners=dict(Counter(p.get('owner', 'unknown') for p in committed)),
        reasons=dict(Counter(p.get('reason', 'unknown') for p in committed)),
        physical_execution_claims=sum(p.get('physical_execution_confirmed') is True for p in committed),
        strict_observed_history_contiguous=bool(committed) and not (duplicate or gaps or failed or foreign),
        unobserved_run_prefix_or_suffix='not established by subscriber sequence continuity alone')


def csv_metrics(paths, start, end):
    values = {k: [] for k in ('e_y', 'e_theta', 'yaw_rate_error', 'speed_error')}
    sources = []
    for path in paths or []:
        source = Path(path)
        accepted = 0
        with source.open(newline='') as stream:
            reader = csv.DictReader(stream)
            fields = reader.fieldnames or []
            if not all(k in fields for k in ('stamp', 'path_valid', 'e_y', 'e_theta')):
                continue
            for row in reader:
                try:
                    t = float(row['stamp'])
                    valid = str(row['path_valid']).lower() in ('1', 'true')
                    if start is None or end is None or not start <= t <= end or not valid:
                        continue
                    accepted += 1
                    for key in values:
                        x = float(row[key]) if row.get(key) else None
                        if number(x):values[key].append(x)
                except (ValueError, TypeError):
                    continue
        sources.append(dict(path=str(source), sha256=sha(source), accepted_rows=accepted))
    return dict(sources=sources, overlap_ros_seconds=[start, end],
        metrics={key: absolute(value) for key, value in values.items()},
        scope='Controller internal path-error estimates in captured time overlap, not independent ground truth. CSV command columns are not actual published history.')


def analyze_run(run_directory, output_dir, csv_snapshots=None):
    run_directory, output_dir = Path(run_directory), Path(output_dir)
    manifest = json.loads((run_directory/'manifest.json').read_text())
    events, parse_errors = [], []
    with (run_directory/'events.jsonl').open() as stream:
        for line, raw in enumerate(stream, 1):
            try:
                e = json.loads(raw, parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))
                if not isinstance(e, dict):raise ValueError('event must be object')
                events.append(e)
            except (ValueError, TypeError) as exc:
                parse_errors.append(dict(line=line, error=str(exc)))
    audit = audit_run(str(run_directory), include_snapshots=False)
    output_dir.mkdir(parents=True, exist_ok=False)
    by_type = {}
    for e in events:by_type.setdefault(e.get('ros_type', 'unknown'), []).append(e)
    def rows(kind):return [e.get('payload', {}) for e in by_type.get(kind, [])]
    candidates = rows('move_car/P6ControlCandidate')
    bases = [r.get('base', {}) for r in candidates]
    attempted = [r for r in bases if number(r.get('evaluations')) and r['evaluations'] > 0]
    parameters = [r.get('parameters', {}) for r in candidates]
    diag = rows('move_car/P5ControlDiagnostics')
    budget = 8.
    solver = dict(candidates=len(candidates), attempted=len(attempted),
        local_feedback_active=sum(r.get('feedback_active') is True for r in candidates),
        local_feedback_fresh=sum(r.get('feedback_used_new_observation') is True for r in candidates),
        local_feedback_correction_abs=absolute(r.get('feedback_correction') for r in candidates),
        valid=sum(r.get('valid') is True for r in bases),
        modes=dict(Counter(r.get('algorithm_mode', 'unknown') for r in candidates)),
        decision_dimensions=dict(Counter(str(r.get('decision_dimension')) for r in attempted)),
        reasons=dict(Counter(r.get('reason', 'unknown') for r in bases)),
        solve_ms=stats(r.get('solve_ms') for r in attempted),
        worker_ms=stats(r.get('worker_ms') for r in attempted),
        solve_budget_ms=budget,
        measured_solve_over_budget=sum(r['solve_ms'] > budget for r in attempted if number(r.get('solve_ms'))),
        timeout_reports=sum(r.get('timed_out') is True for r in bases),
        nominal_config_hashes=sorted({p.get('nominal_config_hash', '') for p in parameters}),
        effective_snapshot_hashes=sorted({p.get('effective_snapshot_hash', '') for p in parameters}),
        slip_valid_samples=sum(p.get('slip_valid') is True for p in parameters),
        parameter_snapshots=len(parameters),
        prediction_accuracy=None,
        prediction_accuracy_reason='Requires causal time-aligned observed response and explicit measurement contract; never compare a prediction to arbitrary nearest future data.')
    imu = rows('sensor_msgs/Imu')
    wheel = rows('move_car/car_parameter')
    sensor = dict(imu_samples=len(imu), raw_chassis_samples=len(wheel),
        imu_frames=sorted({r.get('header', {}).get('frame_id', '') for r in imu}),
        raw_wheel_value=stats(r.get('back_wheel_speed') for r in wheel),
        raw_steering_value=stats(r.get('turn_angle') for r in wheel),
        imu_yaw_rate_raw=stats(r.get('angular_velocity', {}).get('z') for r in imu),
        imu_accel_z_raw=stats(r.get('linear_acceleration', {}).get('z') for r in imu),
        independent_ground_speed_required_for_local_analysis=False,
        slip_valid=False, slip_reason='independent ground velocity not established',
        physical_sampling_time='unknown for headerless chassis feedback; receipt retained separately',
        roughness=None, roughness_reason='Raw acceleration retained; coordinate/gravity/filtering contract not established')
    times = [e.get('receipt_ros_ns')*1e-9 for e in events if number(e.get('receipt_ros_ns'))]
    timing = {key: stats(r.get(key) for r in diag) for key in (
        'callback_ms', 'previous_callback_total_ms', 'publish_interval_ms',
        'algorithm_ms', 'arbitration_ms', 'history_commit_ms', 'publish_ms',
        'queue_delay_ms', 'wakeup_lateness_ms', 'callback_wait_proxy_ms')}
    ledger = ledger_diagnostics(by_type.get('move_car/P5PublishedCommand', []), manifest.get('run_id'))
    writer = manifest.get('writer', {})
    complete = (manifest.get('state') == 'closed' and not parse_errors and
        writer.get('accepted') == writer.get('written') == len(events) and
        writer.get('queue_dropped') == 0 and writer.get('unwritten') == 0 and
        writer.get('writer_error') is None and
        manifest.get('events_sha256') == sha(run_directory/'events.jsonl'))
    review = dict(schema_version=1, run_id=manifest.get('run_id'),
        session_id=manifest.get('continuous_session_id'), terrain='smooth',
        eligible_for_offline_review=complete,
        strict_physical_parameter_identification_ready=False,
        candidate_parameters={}, production_config_written=False,
        automatic_promotion=False, claimed_real_closed_loop_improvement=False,
        physical_calibration_claim=False,
        missing=['independent complete held-out session', 'per-channel identification contract and excitation evidence'],
        next_comparisons=['12 fixed vs flat8 same route, direction and declared speed',
                          'flat8 nonuniform vs flat8 uniform vs blocked8; freeze all other Config values'],
        forbidden_inference='One run or fixed-log counterfactual is not evidence of improved closed-loop control.',
        collection_complete=complete, observed_command_history_contiguous=ledger['strict_observed_history_contiguous'])
    result = dict(schema_version=1, run_id=manifest.get('run_id'),
        session_id=manifest.get('continuous_session_id'), terrain_label=manifest.get('terrain_label'),
        data_origin=manifest.get('data_origin', 'unknown'),
        source_manifest_sha256=sha(run_directory/'manifest.json'),
        source_events_sha256=sha(run_directory/'events.jsonl'),
        collection_complete=complete, parse_errors=parse_errors,
        raw_command=command_diagnostics([e for e in events if e.get('topic') == '/cmd_vel']),
        published_history=ledger, nmpc=solver, sensors=sensor, control_timing_ms=timing,
        csv_tracking=csv_metrics(csv_snapshots, min(times) if times else None, max(times) if times else None),
        telemetry=summarize(events, manifest), physical_calibration_complete=False,
        nmpc_real_vehicle_approval=False)
    write(output_dir/'raw_audit.json', audit)
    write(output_dir/'summary.json', result)
    write(output_dir/'candidate_review.json', review)
    lines = ['# Smooth run review', '', 'run_id: '+str(result['run_id']),
        'session_id: '+str(result['session_id']),
        'Collection complete: '+str(complete),
        'Actual committed command owners: '+json.dumps(ledger['owners']),
        'NMPC candidates / valid: %d / %d' % (solver['candidates'], solver['valid']),
        '', 'See summary.json for tracking, timing, raw sensor and solver metrics.',
        'See raw_audit.json for missing fields, source time and causal availability.',
        'See candidate_review.json before preparing a separate candidate Config.',
        'No physical calibration, performance promotion, or production Config write performed.']
    (output_dir/'README.md').write_text('\n'.join(lines)+'\n')
    return result
