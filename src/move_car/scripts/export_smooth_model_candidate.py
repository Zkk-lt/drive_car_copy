#!/usr/bin/env python3
"""Package apparent smooth response parameters for offline investigation only.

Reads the existing diagnostic/config schemas and registry; never imports ROS,
loads parameters into a controller, or writes an existing directory.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil

import yaml

from control_research.config import CATALOG, load_config
from control_research.terrain_experiment import scene_library


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def finite(value):
    return (not isinstance(value, bool) and isinstance(value, (int, float))
            and math.isfinite(value))


def load_diagnostic(path):
    value = json.loads(Path(path).read_text())
    if value.get('schema_version') != 1 or value.get('status') != 'diagnostic_only':
        raise ValueError('unsupported_diagnostic_schema')
    for split in ('training_runs', 'diagnostic_holdout_runs'):
        runs = value.get(split)
        if not isinstance(runs, list) or not runs:
            raise ValueError('missing_' + split)
        if any(r.get('terrain_label') != 'smooth' or r.get('data_origin') != 'real_vehicle'
               or not r.get('run_id') for r in runs):
            raise ValueError('requires_declared_real_vehicle_smooth_runs')
    train = {r['run_id'] for r in value['training_runs']}
    holdout = {r['run_id'] for r in value['diagnostic_holdout_runs']}
    if train & holdout:
        raise ValueError('same_run_in_training_and_holdout')
    model = value.get('models', {}).get('steering', {})
    family = model.get('families', {}).get('unit_gain_zero_bias', {})
    parameters = family.get('parameters', {})
    if (family.get('nmpc_equation_compatible') is not True
            or parameters.get('command_gain') != 1.
            or parameters.get('output_bias') != 0.):
        raise ValueError('incompatible_steering_equation')
    for key, source in (('steering_delay', 'apparent_delay_s'),
                        ('steering_tau', 'apparent_tau_s')):
        number = parameters.get(source)
        _, _, low, high = CATALOG[key]
        if not finite(number) or not low <= number <= high:
            raise ValueError('parameter_outside_catalog:' + key)
    return value, model, family


def holdout_gates(family, label):
    """Recheck numeric tails; do not rely on a status/debug string alone."""
    checks = []
    rows = family.get('per_run_diagnostic_holdout')
    if not isinstance(rows, list) or not rows:
        return [dict(review=label, passed=False, reasons=['holdout_metrics_missing'])]
    for row in rows:
        reasons = []
        steps = [('one_step', row.get('candidate', {}), row.get('nominal', {}))]
        for horizon in ('0.5s', '1.0s'):
            candidate = row.get('candidate_free_prediction', {}).get(horizon, {})
            nominal = row.get('nominal_free_prediction', {}).get(horizon, {})
            if (candidate.get('valid') is not True or nominal.get('valid') is not True
                    or candidate.get('nonoverlapping_anchor_count', 0) < 3
                    or nominal.get('nonoverlapping_anchor_count', 0) < 3):
                reasons.append(horizon + ':insufficient_valid_nonoverlapping_anchors')
            steps.append((horizon, candidate.get('residual', {}), nominal.get('residual', {})))
        metrics = {}
        for horizon, candidate, nominal in steps:
            metrics[horizon] = {}
            for key in ('rms', 'p99', 'maximum'):
                c = candidate.get('absolute', {}).get(key)
                n = nominal.get('absolute', {}).get(key)
                metrics[horizon][key] = dict(candidate=c, nominal=n)
                if not finite(c) or not finite(n) or c < 0. or n < 0.:
                    reasons.append(horizon + ':' + key + ':missing_or_invalid')
                elif c > n + 1e-12:
                    reasons.append(horizon + ':' + key + ':worse_than_nominal')
                elif horizon == 'one_step' and key == 'rms' and c >= n:
                    reasons.append('one_step:rms:does_not_improve_nominal')
        if family.get('status') == 'rejected':
            reasons.append('source_diagnostic_rejected:' + str(family.get('reason')))
        if family.get('at_hard_search_boundary') is not False:
            reasons.append('search_boundary_unresolved')
        checks.append(dict(review=label, run_id=row.get('run_id'), passed=not reasons,
                           reasons=reasons, metrics=metrics))
    return checks


def build_candidate(pooled_path, prior_path, config_dir, baseline_evidence=None):
    config_dir = Path(config_dir).resolve()
    config = load_config(str(config_dir), 'smooth')
    pooled, model, family = load_diagnostic(pooled_path)
    prior, _, previous_family = load_diagnostic(prior_path)
    package_root = config_dir.parent.parent
    source_paths = [package_root/'include/control/research_types.h',
                    package_root/'src/control_pub.cpp',
                    package_root/'src/trajectory_reconstructor.cpp',
                    Path(__file__).resolve(),
                    Path(__file__).resolve().parent/'control_research/config.py']
    hashes = {str(p): digest(p) for p in source_paths}
    nominal_source = model.get('nominal_comparison', {}).get('source', {})
    if nominal_source.get('sha256') != hashes[str(source_paths[0])]:
        raise ValueError('nominal_source_hash_mismatch')
    if (prior['models']['steering'].get('nominal_comparison', {}).get('source', {}).get('sha256')
            != hashes[str(source_paths[0])]):
        raise ValueError('prior_nominal_source_hash_mismatch')
    entries = {}
    for name, source in (('steering_delay', 'apparent_delay_s'),
                         ('steering_tau', 'apparent_tau_s')):
        cls, unit, low, high = CATALOG[name]
        bounds = config['layers']['base']['parameters'][name]['bounds']
        val = family['parameters'][source]
        if not bounds[0] <= val <= bounds[1]:
            raise ValueError('parameter_outside_base_layer_bounds:' + name)
        entries[name] = dict(value=val, unit=unit,
            bounds=list(bounds), catalog_bounds=[low, high],
            nominal_value=config['values'][name],
            interpretation='recorder_receipt_domain_apparent_response_not_physical_calibration')
        entries[name]['class'] = cls
    checks = holdout_gates(family, 'pooled_diagnostic') + holdout_gates(previous_family, 'prior_fixed_run_review')
    reasons = ['no_independent_physical_session_validation',
               'hardware_source_time_and_new_CAN_frame_unknown',
               'no_certified_ground_speed_or_physical_actuator_delay',
               'no_new_NMPC_vehicle_feedback_closed_loop_validation',
               'full_control_timing_admission_not_established']
    if any(not c['passed'] for c in checks):
        reasons.append('all_run_diagnostic_gate_failed')
    if family['parameters'] != previous_family['parameters']:
        reasons.append('changed_candidate_requires_prior_run_revalidation')
    baseline = None
    if baseline_evidence is not None:
        path = Path(baseline_evidence).resolve()
        if path.is_dir():
            path = path/'evidence_summary.json'
        value = json.loads(path.read_text())
        if value.get('schema_version') != 1 or value.get('terrain_label') != 'smooth':
            raise ValueError('unsupported_baseline_evidence')
        baseline = dict(path=str(path), sha256=digest(path),
                        accepted_S8_pair=value.get('accepted_S8_pair'),
                        certifies_new_NMPC_parameters=False)
    parameters = model['nominal_comparison']['parameters']
    if (parameters.get('apparent_tau_s') != config['values']['steering_tau']
            or parameters.get('apparent_delay_s') != config['values']['steering_delay']):
        raise ValueError('nominal_diagnostic_config_mismatch')
    return dict(schema_version=1, kind='offline_model_candidate', terrain='smooth',
        status='research_candidate_or_rejected', diagnostic_status=family.get('status'),
        deployment='offline_only', runtime=False, eligible_for_runtime=False,
        promotion_blocked=True, promotion_block_reasons=reasons,
        production_changed=False, physical_parameter_certification=False,
        nmpc_closed_loop_improvement_claim=False, independent_sessions=False,
        source_time_status='unknown_hardware_acquisition_and_actuator_reception_time',
        evidence_scope='recorder_receipt_domain_response_investigation_only',
        parameters=entries, unchanged_nominal_configuration=config['values'],
        unchanged_model_quantities=['speed_tau', 'speed_delay', 'yaw_gain',
                                    'all_safety_limits', 'all_cost_weights'],
        baseline_config_hash=config['config_hash'],
        baseline_config_file_hashes=config['file_hashes'], source_hashes=hashes,
        baseline_evidence=baseline,
        diagnostic_inputs=[dict(role=role, path=str(Path(path).resolve()), sha256=digest(path))
            for role, path in [('pooled', pooled_path), ('prior_fixed_run', prior_path)]],
        all_run_gates=checks,
        measurement_support=dict(training_runs=pooled['training_runs'],
            diagnostic_holdout_runs=pooled['diagnostic_holdout_runs'],
            prior_diagnostic_holdout_runs=prior['diagnostic_holdout_runs'],
            training_signal_ranges=model.get('training_support'),
            diagnostic_holdout_signal_ranges=model.get('holdout_support'),
            parameter_profile_delay_s=family.get('profile_delay_range_s'),
            parameter_profile_tau_s=family.get('profile_tau_range_s'),
            uncertainty='loss_profile_not_statistical_or_physical_confidence_interval',
            ground_speed_independence_verified=False,
            cross_scene_validation=False),
        limitations=pooled.get('limitations', []),
        loader_contract='Evidence artifact, not a layered control configuration or a ROS parameter namespace')


def export_candidate(pooled_path, prior_path, config_dir, output_dir, baseline_evidence=None):
    output = Path(output_dir).resolve()
    config_dir = Path(config_dir).resolve()
    if output.exists():
        raise ValueError('output_directory_must_be_new')
    if config_dir == output or config_dir in output.parents:
        raise ValueError('output_must_not_be_in_configuration_directory')
    candidate = build_candidate(pooled_path, prior_path, config_dir, baseline_evidence)
    output.mkdir(parents=True, exist_ok=False)
    for suffix, content in [('json', json.dumps(candidate, indent=2, sort_keys=True, allow_nan=False)+'\n'),
                            ('yaml', yaml.safe_dump(candidate, sort_keys=True))]:
        (output/('model_candidate.'+suffix)).write_text(content)
    inputs = output/'inputs'
    inputs.mkdir()
    for entry in candidate['diagnostic_inputs']:
        destination = inputs/(entry['role']+'.json')
        shutil.copyfile(entry['path'], str(destination))
        if digest(destination) != entry['sha256']:
            raise ValueError('input_changed_during_packaging')
    library = scene_library([], [], [], [dict(terrain='smooth',
        path=str(output/'model_candidate.json'), eligible_for_runtime=False,
        promotion_blocked=True)], [])
    (output/'terrain_library.json').write_text(json.dumps(library, indent=2, sort_keys=True)+'\n')
    manifest = dict(schema_version=1, immutable_by_contract=True,
        replacement_or_production_write=False,
        files={str(p.relative_to(output)): digest(p)
               for p in sorted(output.rglob('*')) if p.is_file()})
    (output/'package_manifest.json').write_text(json.dumps(manifest, indent=2, sort_keys=True)+'\n')
    return candidate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pooled-diagnostic', required=True, type=Path)
    parser.add_argument('--prior-review', required=True, type=Path)
    parser.add_argument('--config-dir', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--baseline-evidence', type=Path)
    args = parser.parse_args()
    try:
        candidate = export_candidate(args.pooled_diagnostic, args.prior_review,
            args.config_dir, args.output_dir, args.baseline_evidence)
    except (ValueError, KeyError, OSError, TypeError) as error:
        parser.exit(2, 'candidate_export_failed: ' + str(error) + '\n')
    print(json.dumps(dict(output_dir=str(args.output_dir.resolve()), terrain='smooth',
        promotion_blocked=candidate['promotion_blocked'], runtime=False)))


if __name__ == '__main__':
    main()
