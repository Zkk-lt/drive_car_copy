"""Smooth evidence by purpose, plus the backward-compatible physical-model gate.

Passing means eligible to prepare unapproved offline candidates; it never grants
real vehicle control. The legacy ``ready`` gate covers the originally specified
ground-referenced model bundle, not a universal requirement for relative local
tracking. Local raw/receipt-domain descriptions do not require NED or independent
ground speed. Their availability cannot certify acquisition clocks, physical
delay, tire slip, parameter updates or closed-loop improvement.
"""
import json
import math
import hashlib
from pathlib import Path
from .dataset import digest
from .causal_data_audit import LOCAL_USE_REQUIREMENTS, local_signal_reasons

MODELS = ('speed_affine', 'longitudinal_first_order', 'yaw_gain', 'steering_actuator')


def _json(path):
    return json.loads(Path(path).read_text())


def _count(value):
    return isinstance(value, int) and not isinstance(value, bool) and value > 0


def _signal_field_scan(path, aliases):
    """Stream closed raw files at bounded memory; recheck fields even for old audits.

    The digest covers the same bytes as this field scan. No sample interpolation,
    state fusion or physical/measurement certification is performed here.
    """
    checks = {key: dict(missing=set(), examined_samples=0) for key in LOCAL_USE_REQUIREMENTS}
    wanted = {}
    for key, requirements in LOCAL_USE_REQUIREMENTS.items():
        for logical, fields in requirements:
            wanted.setdefault(aliases.get(logical, logical), []).append((key, logical, fields))
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for line in stream:
            h.update(line)
            event = json.loads(line)
            if not isinstance(event, dict):
                raise ValueError('raw event object required for signal field scan')
            for key, logical, fields in wanted.get(event.get('topic'), []):
                checks[key]['examined_samples'] += 1
                checks[key]['missing'].update(local_signal_reasons(event, logical, fields))
    for check in checks.values():
        check['missing'] = sorted(check['missing'])
        check['required_signal_fields_present'] = not check['missing'] and check['examined_samples'] > 0
    return h.hexdigest(), checks


def _local_audit(path, audit):
    """Bind an existing raw audit to its closed files; never alter the files.

    Hash binding detects stale reports, not maliciously fabricated provenance.
    We retain the original audit's physical rejection reasons without allowing
    missing optional ground/NED measurements to reject local descriptions.
    """
    row = dict(path=str(path), sha256=digest(path), run_id=audit.get('run_id'),
               data_origin=audit.get('data_origin', 'unknown'), missing=[],
               usable_topics=[], physical_or_control_availability_claim=False)
    if audit.get('schema_version') != 1 or not isinstance(audit.get('topic_coverage'), dict):
        row['missing'].append('supported_raw_audit_required')
        return row
    if audit.get('terrain_label') != 'smooth':
        row['missing'].append('smooth_scope_required')
    if audit.get('parse_failures') != []:
        row['missing'].append('raw_parse_checks_must_pass')
    # A malformed optional stream must not invalidate unrelated local uses.
    # Topic-specific schema/clock failures exclude that topic below. Unknown
    # event attribution is a file-level failure and remains fail-closed.
    invalid = audit.get('invalid_events')
    if not isinstance(invalid, list) or any(
            not isinstance(e, dict) or not isinstance(e.get('topic'), str)
            for e in invalid):
        row['missing'].append('raw_invalid_event_topic_attribution_required')
    row['invalid_event_topics'] = sorted({e['topic'] for e in (invalid if isinstance(invalid, list) else [])
                                         if isinstance(e, dict) and isinstance(e.get('topic'), str)})
    try:
        directory = Path(audit['run_directory'])
        manifest = _json(directory / 'manifest.json')
        aliases = audit.get('topic_aliases', {})
        if not isinstance(aliases, dict):
            raise ValueError('raw_topic_alias_mapping_required')
        scanned_sha, row['signal_field_checks'] = _signal_field_scan(directory / 'events.jsonl', aliases)
        if digest(directory / 'manifest.json') != audit.get('manifest_sha256') or \
                scanned_sha != audit.get('events_sha256'):
            row['missing'].append('audit_input_hash_mismatch')
        if manifest.get('run_id') != audit.get('run_id') or not audit.get('run_id'):
            row['missing'].append('audit_manifest_run_identity_mismatch')
        if manifest.get('terrain_label') != 'smooth' or manifest.get('data_origin') != audit.get('data_origin'):
            row['missing'].append('audit_manifest_scene_or_origin_mismatch')
        row['events_sha256'] = audit.get('events_sha256')
        row['manifest_sha256'] = audit.get('manifest_sha256')
        row['run_directory'] = str(directory)
        writer = manifest.get('writer') or {}
        if manifest.get('state') != 'closed' or not _count(writer.get('written')) or not _count(writer.get('accepted')) or \
                writer.get('accepted') != writer.get('written') or \
                writer.get('queue_dropped') != 0 or writer.get('unwritten') != 0 or \
                writer.get('writer_error') is not None or 'writer_error' not in writer:
            row['missing'].append('closed_loss_free_local_writer_required')
        total = sum(v.get('count', 0) for v in audit['topic_coverage'].values()
                    if isinstance(v, dict) and _count(v.get('count')))
        if total != writer.get('written'):
            row['missing'].append('audit_writer_event_count_mismatch')
        for topic, coverage in audit['topic_coverage'].items():
            if isinstance(coverage, dict) and _count(coverage.get('count')) and \
                    coverage['count'] > 1 and coverage.get('raw_serialization_count') == coverage['count'] and \
                    coverage.get('invalid_events') == {} and topic not in row['invalid_event_topics']:
                row['usable_topics'].append(topic)
        row['topic_aliases'] = audit.get('topic_aliases', {})
        if not isinstance(row['topic_aliases'], dict):
            row['missing'].append('raw_topic_alias_mapping_required')
            row['topic_aliases'] = {}
        row['original_strict_missing'] = audit.get('missing', [])
        row['source_acquisition_and_transport_completeness'] = 'not_certified_by_local_writer_or_receipt_audit'
    except (OSError, KeyError, ValueError, TypeError) as exc:
        row['missing'].append('raw_audit_input_unavailable:' + str(exc))
    return row


def assess_local_uses(evidence_paths=()):
    """Purpose-specific availability, not data promotion or model approval."""
    evidence, unsupported, responses = [], [], []
    for path in evidence_paths:
        try:
            item = _json(path)
            if isinstance(item, dict) and 'topic_coverage' in item:
                evidence.append(_local_audit(path, item))
            elif isinstance(item, dict) and item.get('tool') == 'analyze_raw_response':
                responses.append((path, item))
            else:
                unsupported.append(dict(path=str(path), missing=['raw_audit_or_receipt_response_report_required']))
        except (OSError, ValueError, TypeError) as exc:
            unsupported.append(dict(path=str(path), missing=[str(exc)]))

    def use(signal_key, topics, scope, additional=()):
        eligible, rejected = [], []
        for row in evidence:
            aliases = row.get('topic_aliases') or {}
            missing = list(row['missing']) + [
                'missing_valid_raw_topic:' + topic for topic in topics
                if aliases.get(topic, topic) not in row['usable_topics']]
            field_check = row.get('signal_field_checks', {}).get(signal_key, {})
            missing += field_check.get('missing', [])
            if field_check.get('required_signal_fields_present') is not True:
                missing.append('required_numeric_signal_field_check_not_passed')
            (rejected if missing else eligible).append(dict(
                run_id=row['run_id'], evidence_sha256=row['sha256'], missing=missing,
                data_origin=row['data_origin']))
        return dict(status='available_diagnostic_only' if eligible else 'not_established',
                    available_for_stated_scope=bool(eligible), scope=scope,
                    eligible_runs=eligible, rejected_runs=rejected,
                    missing=[] if eligible else ['verified_closed_raw_streams_for_this_purpose_required'],
                    ready_for_parameter_update=False, physical_calibration_certified=False,
                    controller_availability_certified=False, real_vehicle_provenance_authenticated=False,
                    limitations=list(additional))

    uses = {
        'relative_local_tracking_description': use(
            'local_relative_reference_and_command',
            ('/planner/reference_path', '/cmd_vel', '/car_message', '/imu'),
            'Describe body/local reference, actual command and raw response streams; '
            'no global pose/ground trajectory reconstruction or independent truth error.',
            ('Local tracking-error metrics additionally require recorded controller error samples or audited S8 CSV; '
             'path presence alone does not provide tracking error.',)),
        'local_yaw_response_diagnostic': use(
            'local_yaw_response_signals',
            ('/car_message', '/imu'),
            'Describe measured IMU angular rate and reported wheel/steering response; '
            'wheel-speed kinematic proxy is not ground-speed yaw calibration.',
            ('SI units, installed IMU axes, steering zero and synchronized sample freshness remain separate checks.',
             'Availability permits stream/residual investigation, not a trusted yaw-gain fit.')),
        'local_steering_receipt_response_diagnostic': use(
            'local_steering_response_signals',
            ('/car_message', '/cmd_vel'),
            'Recorded command/feedback events in collector receipt time; '
            'keep full command events rather than downsampling commands to feedback rate.',
            ('No claim that all emitted or actuator-received commands were captured.',
             'Receipt-domain delay includes cache, transport and scheduling; it is not physical actuator delay.',
             'Only command events available before a receipt-domain prediction anchor may be selected.')),
        'physical_actuator_calibration': dict(
            status='not_established', available_for_stated_scope=False,
            missing=['verified_command_publish_and_actuator_receipt_timing',
                     'fresh_CAN_feedback_and_acquisition_timing', 'SI_zero_gain_and_excitation_validation',
                     'independent_session_parameter_uncertainty_validation'],
            scope='Physical steering gain/time constant/delay; NED is not required.',
            ned_required=False, ready_for_parameter_update=False),
        'independent_ground_speed_and_true_slip': dict(
            status='not_established', available_for_stated_scope=False,
            missing=['optional_independent_ground_speed_measurement_and_source_provenance',
                     'frame_unit_time_alignment_and_low_speed_observability'],
            scope='Optional true longitudinal slip research; wheel feedback alone is not independent ground speed.',
            ned_required=False, required_for_relative_local_control=False,
            ready_for_parameter_update=False),
    }
    verified = {row.get('run_id'): row for row in evidence if not row['missing']}
    receipt_reports = []
    for path, report in responses:
        failures = []
        training = report.get('training_runs', [])
        holdout = report.get('diagnostic_holdout_runs', [])
        if not isinstance(training, list) or not isinstance(holdout, list):
            failures.append('response_input_lists_required')
            training, holdout = [], []
        inputs = training + holdout
        if not inputs:
            failures.append('response_raw_input_bindings_required')
        for source in inputs:
            if not isinstance(source, dict):
                failures.append('response_raw_input_object_required')
                continue
            match = verified.get(source.get('run_id'))
            if not match or any(source.get(k) != match.get(k) for k in ('events_sha256', 'manifest_sha256')):
                failures.append('response_input_not_bound_to_verified_raw_audit:' + str(source.get('run_id')))
            elif any((match.get('topic_aliases') or {}).get(topic, topic) not in match['usable_topics']
                     for topic in ('/cmd_vel', '/car_message')):
                failures.append('response_input_requires_valid_command_and_steering_streams:' + str(source.get('run_id')))
            elif match.get('signal_field_checks', {}).get('local_steering_response_signals', {}).get('required_signal_fields_present') is not True:
                failures.append('response_input_requires_numeric_steering_fields:' + str(source.get('run_id')))
        if report.get('physical_delay_identified') is not False or \
                report.get('eligible_for_runtime') is not False or \
                report.get('time_basis') != 'each_run_recorder_receipt_monotonic_ns_only_not_source_publish_or_actuator_time':
            failures.append('receipt_diagnostic_scope_required')
        receipt_reports.append(dict(path=str(path), sha256=digest(path), missing=sorted(set(failures)),
                                    usable_as_receipt_diagnostic_reference=not failures,
                                    grants_parameter_or_control_authority=False))
    return dict(use_case_assessments=uses, local_evidence=evidence,
                unsupported_local_evidence=unsupported, receipt_response_references=receipt_reports,
                local_evidence_changes_legacy_ready_gate=False,
                assessment_boundary='Use-specific diagnostic availability does not promote old data '
                'to calibrated measurements, authenticate provenance, or grant NMPC vehicle output.')


def assess(dataset_path, model_paths=(), review_path=None, speed_range=None, local_evidence_paths=()):
    out = dict(schema_version=1, terrain='smooth', status='not_ready',
               ready_for_offline_candidate_search=False, ready_for_vehicle_NMPC=False,
               vehicle_improvement_claim=False, production_changed=False,
               final_test_used_for_parameter_selection=False, missing=[], models={},
               provenance_authenticated=False)
    try:
        data = _json(dataset_path)
        out['dataset_sha256'] = digest(dataset_path)
        if data.get('terrain') != 'smooth' or data.get('status') != 'session_partition_reviewed':
            out['missing'].append('reviewed_complete_smooth_train_validation_test_sessions')
        rows = data.get('runs', [])
        by_id = {r.get('run_id'): r for r in rows}
        for split in ('train', 'validation', 'test'):
            if not any(r.get('split') == split and not r.get('missing') for r in rows):
                out['missing'].append('eligible_' + split + '_session')
        speed_ok = isinstance(speed_range, (tuple, list)) and len(speed_range) == 2 and all(
            isinstance(v, (float, int)) and not isinstance(v, bool) and math.isfinite(v) for v in speed_range)
        if not speed_ok or not 0. <= speed_range[0] <= speed_range[1] <= .5:
            out['missing'].append('explicit_requested_offline_speed_domain_0_to_0p5_mps')
            speed_ok = False
        out['requested_speed_domain_mps'] = speed_range if speed_ok else None
        for path in model_paths:
            model = _json(path); kind = model.get('model_kind'); failures = []
            if kind not in MODELS or kind in out['models']:
                raise ValueError('unknown_or_duplicate_model_kind:' + str(kind))
            if model.get('status') != 'held_out_equation_validated' or model.get('valid') is not True:
                failures.append('held_out_equation_validation_required')
            if model.get('data_origin') != 'real_vehicle' or model.get('eligible_for_model_review') is not True:
                failures.append('real_vehicle_model_review_eligibility_required')
            train = by_id.get(model.get('training_run_id'), {})
            valid = by_id.get(model.get('validation_run_id'), {})
            if train.get('split') != 'train' or valid.get('split') != 'validation':
                failures.append('model_training_validation_runs_must_match_dataset_roles')
            if not model.get('training_session_id') or model.get('training_session_id') == model.get('validation_session_id'):
                failures.append('independent_model_sessions_required')
            if train.get('continuous_session_id') != model.get('training_session_id') or valid.get('continuous_session_id') != model.get('validation_session_id'):
                failures.append('model_session_dataset_mismatch')
            if not model.get('training_samples_fingerprint') or not model.get('validation_samples_fingerprint') or model.get('training_samples_fingerprint') == model.get('validation_samples_fingerprint'):
                failures.append('independent_model_measurement_fingerprints_required')
            if not model.get('uncertainty') or not model.get('validation_residual'):
                failures.append('uncertainty_and_independent_residuals_required')
            if kind != 'steering_actuator' and speed_ok:
                for domain in ('model_domain', 'validation_model_domain'):
                    support = (model.get(domain) or {}).get('rear_speed_mps')
                    if not isinstance(support, list) or len(support) != 2 or not all(
                            isinstance(v, (float, int)) and not isinstance(v, bool) and math.isfinite(v) for v in support) or not (
                            support[0] <= speed_range[0] <= speed_range[1] <= support[1]):
                        failures.append(domain + '_does_not_support_requested_speed_range')
            out['models'][kind] = dict(path=str(path), sha256=digest(path), missing=failures,
                                       physical_delay_identified=model.get('physical_delay_identified', False))
        out['missing'].extend('validated_' + m + '_model' for m in MODELS if m not in out['models'])
        out['missing'].extend(kind + ':' + reason for kind, value in out['models'].items() for reason in value['missing'])
        if review_path is None:
            out['missing'].append('explicit_nominal_model_and_data_review')
        else:
            review = _json(review_path)
            out['review_sha256'] = digest(review_path)
            expected = {kind: value['sha256'] for kind, value in out['models'].items()}
            if review.get('approved') is not True or not str(review.get('reviewer', '')).strip() or review.get('scope') != 'nominal_model_for_smooth_offline_simulation':
                out['missing'].append('approved_named_reviewer_and_offline_scope_required')
            if review.get('dataset_sha256') != out['dataset_sha256'] or review.get('models_sha256') != expected:
                out['missing'].append('review_dataset_or_model_hash_mismatch')
            if review.get('final_test_locked') is not True:
                out['missing'].append('final_independent_test_must_remain_locked')
            if review.get('model_domain_and_uncertainty_reviewed') is not True:
                out['missing'].append('model_domain_uncertainty_review_required')
            if review.get('common_model_same_state_actual_input_prediction_protocol') is not True:
                out['missing'].append('fair_12_and_8_prediction_protocol_required')
        out['missing'] = sorted(set(out['missing']))
        if not out['missing']:
            out['status'] = 'eligible_for_unapproved_offline_candidates_only'
            out['ready_for_offline_candidate_search'] = True
    except (OSError, ValueError, KeyError, TypeError) as e:
        out['missing'].append(str(e))
    out['strict_gate_scope'] = 'Legacy ground-referenced calibrated model bundle for offline candidates; not a universal requirement for relative local control.'
    out.update(assess_local_uses(local_evidence_paths))
    out['next_step'] = ('Run offline bounded C-class search and calibrated-domain simulation; freeze candidates before final test.'
                        if out['ready_for_offline_candidate_search'] else
                        'Use the separate purpose-specific assessments for local analysis; collect only the '
                        'missing evidence needed by the chosen study with S8. Missing optional NED/ground '
                        'speed does not invalidate local relative tracking descriptions.')
    out['limitations'] = ['Passing is not a provenance certificate or physical safety validation.',
                          'A-class safety limits and B-class calibration cannot be optimized as performance weights.',
                          'No grass/asphalt candidate or production configuration is generated.',
                          'Historical fixed-state replay is not feedback closed-loop evidence.',
                          'Full vehicle timing admission and actual actuator jerk remain separate gates.']
    return out
