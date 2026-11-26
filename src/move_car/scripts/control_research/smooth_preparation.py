"""P4-2 smooth-only data eligibility. Offline, no controller or YAML writes.

The existing whole-run loader and metrics are retained. This layer adds explicit
session-boundary evidence and rejects temporal/content leakage before model work.
Operator declarations and review references do not authenticate measurements.
"""
import hashlib
import json
from pathlib import Path

from .dataset import digest, index_runs, load_run


def split_template():
    return {
        'schema_version': 1, 'terrain': 'smooth', 'runs': [],
        'session_reviews': [], 'status': 'requires_operator_plan',
        'run_entry_fields': ['run_id', 'continuous_session_id', 'split'],
        'session_review_fields': ['continuous_session_id', 'run_ids', 'reviewer',
                                  'session_boundary_verified', 'evidence'],
        'evidence_entry_fields': ['path', 'sha256'],
        'rule': 'All runs in one continuous acquisition session belong to one split. Never randomize adjacent cycles.',
        'allowed_splits': ['train', 'validation', 'test'],
        'data_origin_authenticated': False,
    }


def _values_only(obj):
    if isinstance(obj, dict):
        return {k: _values_only(v) for k, v in obj.items()
                if k not in ('run_id', 'stamp', 'seq', 'geometry_stamp_ns', 'source_stamp_ns')}
    if isinstance(obj, list):
        return [_values_only(x) for x in obj]
    return obj


def measurement_fingerprint(events):
    """Supplement to full-event SHA: renaming IDs or shifting clocks is no split.

    Conservative collisions, e.g. identical stationary runs, require manual
    review. Never silently use them as independent test evidence.
    """
    raw_types = {'move_car/car_parameter', 'nav_msgs/Odometry', 'sensor_msgs/Imu'}
    payload = [(e.get('topic'), e.get('ros_type'), _values_only(e.get('payload', {})))
               for e in events if e.get('ros_type') in raw_types]
    if not payload:
        return None
    return hashlib.sha256(json.dumps(payload, sort_keys=True, allow_nan=False,
                                     separators=(',', ':')).encode()).hexdigest()


def _session_reviews(plan, base):
    reviews = {}
    for r in plan.get('session_reviews', []):
        sid = r.get('continuous_session_id')
        if not sid or sid in reviews:
            raise ValueError('missing_or_duplicate_session_review')
        if r.get('session_boundary_verified') is not True or not str(r.get('reviewer', '')).strip():
            raise ValueError('session_boundary_review_required:' + sid)
        evidence = r.get('evidence', [])
        if not isinstance(evidence, list) or not evidence:
            raise ValueError('session_evidence_required:' + sid)
        for e in evidence:
            p = Path(e.get('path', ''))
            if not p.is_absolute():
                p = base / p
            if not p.is_file() or digest(p) != e.get('sha256'):
                raise ValueError('session_evidence_missing_or_hash_mismatch:' + sid)
        reviews[sid] = r
    return reviews


def audit_dataset(root, plan, evidence_base=None):
    root = Path(root)
    out = dict(schema_version=1, terrain='smooth', status='not_ready', runs=[], missing=[],
               split_unit='whole_run_and_continuous_session', data_origin_authenticated=False,
               candidate_generation_allowed=False, vehicle_improvement_claim=False)
    if not isinstance(plan, dict) or not plan.get('runs'):
        out['missing'] = ['explicit_nonempty_whole_run_split_plan', 'reviewed_session_boundaries']
        return out
    try:
        old_index = index_runs(root, plan)
        reviews = _session_reviews(plan, Path(evidence_base or root))
        values_seen = {}; rows = []; intervals = []
        for row in old_index['runs']:
            rid, sid = row['run_id'], row['continuous_session_id']
            manifest, events, metrics = load_run(root / rid)
            missing = []
            if row['terrain'] != 'smooth':
                missing.append('only_current_real_smooth_scene_allowed')
            if row['data_origin'] != 'real_vehicle':
                missing.append('real_vehicle_origin_declaration_required')
            if manifest.get('use_sim_time') is not False:
                missing.append('explicit_non_simulated_clock_required')
            hashes = manifest.get('formal_config_hashes', {})
            if not isinstance(hashes, dict) or not hashes or any(
                    not isinstance(v, str) or len(v) != 64 or any(c not in '0123456789abcdef' for c in v)
                    for v in hashes.values()):
                missing.append('formal_S8_configuration_hashes_required')
            if manifest.get('continuous_session_id') != sid:
                missing.append('explicit_manifest_session_required')
            if sid not in reviews or rid not in reviews[sid].get('run_ids', []):
                missing.append('run_not_covered_by_reviewed_session')
            if manifest.get('actual_control_mode') != 'legacy_slqr':
                missing.append('S8_control_mode_required')
            for event in events:
                p = event.get('payload', {})
                if event.get('ros_type') == 'move_car/AdaptiveControlTelemetry':
                    if p.get('actual_owner') != 'legacy_slqr' or p.get('shadow_only') is not True:
                        missing.append('telemetry_control_authority_conflict')
                if p.get('nmpc_active') is True:
                    missing.append('telemetry_control_authority_conflict')
            vehicle = manifest.get('vehicle_id')
            if not vehicle or vehicle == 'unspecified':
                missing.append('vehicle_identity_required_for_overlap_audit')
            ts = [e.get('receipt_ros_ns') for e in events]
            if not ts or any(isinstance(t, bool) or not isinstance(t, int) or t <= 0 for t in ts):
                missing.append('valid_receipt_time_range_required')
                interval = None
            else:
                interval = [min(ts), max(ts)]
                intervals.append((vehicle, interval, row['split'], rid))
            fingerprint = measurement_fingerprint(events)
            if fingerprint is None:
                missing.append('independent_raw_channel_payloads_missing')
            elif fingerprint in values_seen:
                raise ValueError('duplicate_measurement_content_even_after_relabel:' + rid + ':' + values_seen[fingerprint])
            else:
                values_seen[fingerprint] = rid
            rows.append(dict(row, missing=sorted(set(missing)), vehicle_id=vehicle,
                             receipt_time_range_ros_ns=interval,
                             measurement_values_sha256=fingerprint))
            if row['split'] == 'test':
                rows[-1]['metrics'] = {'status': 'sealed_final_test_performance',
                                      'reason': 'Do not disclose tuning objectives before candidates and protocol are frozen.'}
        for i, (vehicle, a, split, rid) in enumerate(intervals):
            for other_vehicle, b, other_split, other_rid in intervals[i+1:]:
                if vehicle == other_vehicle and split != other_split and max(a[0], b[0]) <= min(a[1], b[1]):
                    raise ValueError('overlapping_vehicle_time_ranges_across_splits:' + rid + ':' + other_rid)
        for split in ('train', 'validation', 'test'):
            if not any(r['split'] == split and not r['missing'] for r in rows):
                out['missing'].append('eligible_' + split + '_session_missing')
        out['runs'] = rows
        out['coverage'] = old_index['coverage']
        out['missing'].extend(sorted(set(x for r in rows for x in r['missing'])))
        if not out['missing']:
            out['status'] = 'session_partition_reviewed'
        out['physical_identification_ready'] = False
        out['next_gate'] = 'independent_raw_source_contract_and_identification_validation'
    except (ValueError, KeyError, TypeError, OSError) as e:
        out['missing'].append(str(e))
    return out


def quarantine_legacy(workspace):
    # Reuse P4-0 raw CSV inventory/metrics; add only the latest user scope.
    from audit_legacy_control_data import audit
    old = audit(Path(workspace))
    for run in old['runs']:
        run.update(terrain_label='smooth',
                   terrain_evidence='user declaration: existing real runs are one indoor smooth scene; not independent sensor certification',
                   continuous_session_id=None, split='quarantine',
                   missing=['independent_raw_ground_speed', 'CAN_acquisition_time',
                            'reviewed_continuous_session', 'per_run_formal_config_identity',
                            'raw_imu_and_actual_steering_with_verified_source_time'])
    return dict(schema_version=1, terrain='smooth', status='not_ready',
                runs=old['runs'], run_count=old['run_count'],
                accepted_S8_pair=old['accepted_S8_pair'],
                eligible_train_runs=0, eligible_validation_runs=0, eligible_test_runs=0,
                physical_calibration_completed=False, candidate_generation_allowed=False,
                limitation='Legacy controller diagnostics remain descriptive S8 evidence, not raw physical identification data. No guessed session grouping.')
