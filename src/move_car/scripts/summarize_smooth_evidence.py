#!/usr/bin/env python3
"""Immutable smooth evidence index; no ROS, tuning, or production writes.

Only listed historical CSVs and closed run summaries are read. Raw JSONL is
never scanned. A matching hash authenticates bytes, not physical measurement.
"""
import argparse
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess

from control_research.analysis import stats
from control_research.terrain_experiment import scene_library


CORE_FILES = (
    'src/move_car/src/control_pub.cpp',
    'src/move_car/src/trajectory_reconstructor.cpp',
    'src/move_car/launch/movecar_improved.launch',
    'src/move_car/launch/trajectory_reconstructor.launch',
)
METRICS = {
    'ey': 'origin_projected_lateral_error_abs_m',
    'body_heading_error': 'body_heading_error_abs_rad',
    'course_error': 'course_error_abs_rad',
    'front_axle_e_ct': 'front_axle_reference_error_abs_m',
    'e_theta': 'runtime_heading_error_abs_rad',
    'yaw_rate_error': 'internal_yaw_response_error_abs_radps',
}


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def finite(value):
    try:
        value = float(value)
        return value if math.isfinite(value) else None
    except (TypeError, ValueError):
        return None


def workspace_path(root, relative):
    path = (root/relative).resolve()
    if root not in path.parents:
        raise ValueError('historical_path_outside_workspace:' + str(relative))
    return path


def git_bytes(root, *args):
    return subprocess.check_output(['git', '-C', str(root)] + list(args),
                                   stderr=subprocess.PIPE)


def csv_evidence(root, old):
    """Recount only the exact allowlisted file; changed data is not old data."""
    path = workspace_path(root, old['path'])
    out = dict(path=old['path'], run_id=old.get('run_id'),
        expected_sha256=old.get('sha256'), expected_rows=old.get('rows'),
        expected_code_revisions=old.get('code_revisions', []),
        accepted_S8=old.get('accepted_S8') is True, terrain_label='smooth',
        terrain_evidence='user_confirmed_same_indoor_smooth_scene',
        eligibility='legacy_descriptive_only', physical_calibration_valid=False,
        continuous_session_id=None, split='quarantine')
    if not path.is_file():
        return dict(out, integrity='missing', observed_rows=None,
                    observed_sha256=None, statistics=None)
    before = path.stat()
    observed_hash = digest(path)
    values = {key: [] for key in METRICS}
    rates = []; last = None; rows = tracked = malformed = 0
    revisions = Counter(); run_ids = Counter(); modes = Counter(); stops = Counter()
    stamps = []; nonmonotonic = gaps = 0
    with path.open(newline='') as stream:
        reader = csv.DictReader(stream)
        columns = list(reader.fieldnames or [])
        for row in reader:
            rows += 1
            if None in row:
                malformed += 1
            revisions[row.get('code_revision') or 'unrecorded'] += 1
            run_ids[row.get('run_id') or 'unrecorded'] += 1
            modes[row.get('mode') or 'unrecorded'] += 1
            stops[row.get('stop_reason') or 'unrecorded'] += 1
            t, u = finite(row.get('stamp')), finite(row.get('cmd'))
            if t is not None:
                stamps.append(t)
            if last is not None and t is not None and u is not None:
                dt = t-last[0]
                if 0 < dt <= .2:
                    rates.append(abs((u-last[1])/dt))
                elif dt <= 0:
                    nonmonotonic += 1
                else:
                    gaps += 1
            last = (t, u) if t is not None and u is not None else None
            # Same historical P42 selection: path_valid == 1, not a new mask
            # chosen to improve a result. New raw-run metrics remain separate.
            if row.get('path_valid') != '1':
                continue
            tracked += 1
            for key in values:
                value = finite(row.get(key))
                if value is not None:
                    values[key].append(abs(value))
    after = path.stat()
    stable = (before.st_size, before.st_mtime_ns) == (after.st_size, after.st_mtime_ns)
    if not stable:
        integrity = 'changed_during_read'
    elif observed_hash != old.get('sha256'):
        integrity = 'changed'
    elif rows != old.get('rows'):
        integrity = 'inventory_row_count_mismatch'
    elif sorted(revisions) != sorted(old.get('code_revisions', [])):
        integrity = 'inventory_revision_mismatch'
    elif malformed:
        integrity = 'malformed_csv'
    elif sorted(run_ids) not in ([str(old.get('run_id'))], ['unrecorded']):
        integrity = 'run_id_mismatch'
    else:
        integrity = 'matched'
    out.update(integrity=integrity, observed_sha256=observed_hash,
        observed_rows=rows, observed_code_revisions=dict(revisions),
        observed_run_ids=dict(run_ids), columns=columns,
        run_id_recorded_in_rows=sorted(run_ids) == [str(old.get('run_id'))],
        run_id_basis=('csv_rows' if sorted(run_ids) == [str(old.get('run_id'))]
                      else 'frozen_inventory_reference_only'),
        tracking_rows=tracked, malformed_rows=malformed,
        source_time_range=[min(stamps), max(stamps)] if stamps else None,
        nonmonotonic_stamps=nonmonotonic, gaps_over_0p2s=gaps,
        modes=dict(modes), stop_reasons=dict(stops), statistics=None)
    if integrity == 'matched':
        out['statistics'] = {METRICS[key]: stats(value) for key, value in values.items()
                             if key in columns}
        out['statistics']['command_steering_rate_abs_radps'] = stats(rates)
    else:
        out['exclusion_reason'] = 'current_bytes_are_not_the_frozen_historical_evidence'
    out['metric_scope'] = (
        'Per-file path_valid==1 internal diagnostics. ey is origin projection; '
        'wheel_speed is fused speed. No independent tracking truth, slip, '
        'physical jerk, cross-version winner, or pooled controller score.')
    return out


def code_evidence(root, release):
    snapshots = {}; files = []; errors = []
    try:
        commit = git_bytes(root, 'rev-parse', 'S8^{commit}').decode().strip()
    except subprocess.CalledProcessError:
        commit = None
        errors.append('S8_tag_missing')
    for relative in CORE_FILES:
        expected = release.get('sha256', {}).get(relative)
        path = workspace_path(root, relative)
        current = path.read_bytes() if path.is_file() else None
        try:
            tag = git_bytes(root, 'show', 'S8:' + relative)
        except subprocess.CalledProcessError:
            tag = None
        tag_hash = hashlib.sha256(tag).hexdigest() if tag is not None else None
        current_hash = hashlib.sha256(current).hexdigest() if current is not None else None
        files.append(dict(path=relative, release_sha256=expected,
            tag_sha256=tag_hash, current_sha256=current_hash,
            tag_matches_release=bool(expected and tag_hash == expected),
            current_matches_tag=current is not None and tag is not None and current == tag))
        if tag is not None:
            snapshots['tag_S8/' + relative] = tag
        if current is not None:
            snapshots['current_instrumented/' + relative] = current
    return dict(tag='S8', tag_commit=commit, files=files, errors=errors,
        current_identity='current_research_source_snapshot_equivalence_unverified',
        tag_identity='user_accepted_S8_20261104',
        all_tag_files_match_release=all(f['tag_matches_release'] for f in files),
        current_core_is_byte_identical_to_tag=all(f['current_matches_tag'] for f in files),
        running_binary_identity_verified=False,
        source_difference_is_not_automatic_behavior_equivalence=True), snapshots


def closed_summary(path):
    path = Path(path).resolve()
    value = json.loads(path.read_text())
    if value.get('terrain_label') != 'smooth' or value.get('data_origin') != 'real_vehicle':
        raise ValueError('only_explicit_real_vehicle_smooth_summaries:' + str(path))
    writer = value.get('writer') or {}
    if not (value.get('local_recording_complete') is True and
            writer.get('accepted') == writer.get('written') and
            type(writer.get('written')) is int and writer['written'] > 0 and
            writer.get('queue_dropped') == 0 and writer.get('unwritten') == 0 and
            writer.get('writer_error') is None):
        raise ValueError('closed_locally_complete_run_required:' + str(path))
    rid = value.get('run_id')
    if not isinstance(rid, str) or not rid or any(x not in
            'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_-' for x in rid):
        raise ValueError('safe_run_id_required')
    events_hash = value.get('events_sha256')
    if (not isinstance(events_hash, str) or len(events_hash) != 64 or
            any(c not in '0123456789abcdef' for c in events_hash)):
        raise ValueError('valid_declared_events_sha256_required:' + rid)
    manifest_path = path.parent/'runs'/rid/'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    if (manifest.get('state') != 'closed' or manifest.get('run_id') != rid or
            manifest.get('terrain_label') != 'smooth' or
            manifest.get('data_origin') != 'real_vehicle' or
            manifest.get('events_sha256') != value.get('events_sha256') or
            not value.get('events_sha256') or
            manifest.get('writer') != writer):
        raise ValueError('closed_manifest_summary_conflict:' + rid)
    if (manifest.get('session_id') is not None and
            manifest['session_id'] != value.get('session_id')):
        raise ValueError('closed_manifest_summary_session_conflict:' + rid)
    if value.get('vehicle_commands_generated_by_tools') is not False:
        raise ValueError('passive_collection_required:' + rid)
    # Keep published summaries, not reconstructed pseudo-30Hz feedback. Empty
    # graph snapshots during start/stop do not by themselves imply takeover.
    graphs = value.get('graph_samples_cmd_publishers', [])
    external = []
    graph_publishers = []
    graph_format_valid = isinstance(graphs, list)
    for graph in graphs if graph_format_valid else []:
        pubs = graph.get('publishers', []) if isinstance(graph, dict) else graph
        if isinstance(pubs, list) and all(isinstance(p, str) for p in pubs):
            graph_publishers.extend(pubs)
            external.extend(p for p in pubs if p != '/publish_speed_turn')
        else:
            graph_format_valid = False
    only_s8_observed = bool(graph_format_valid and graph_publishers and not external)
    return dict(run_id=rid, session_id=value.get('session_id'),
        continuous_session_id=manifest.get('continuous_session_id'),
        terrain_label='smooth', terrain_evidence='user_confirmed',
        summary_path=str(path), summary_sha256=digest(path),
        manifest_path=str(manifest_path), manifest_sha256=digest(manifest_path),
        declared_events_sha256=value['events_sha256'], raw_events_rescanned=False,
        local_recording_complete=True, writer=writer,
        duration_observed_s=value.get('duration_observed_s'),
        raw_events=value.get('raw_events'),
        observed_external_command_publishers=sorted(set(external)),
        only_S8_observed_publishing=only_s8_observed,
        actual_control_mode='legacy_slqr' if only_s8_observed else 'unverified',
        controller_source_identity='run_summary_declaration_binary_not_verified',
        source_files_untouched=value.get('source_files_untouched'),
        software_freeze_changes=value.get('software_freeze_changes'),
        local_time_window=value.get('local_time_window'),
        legacy_S8_windowed_metrics=value.get('legacy_S8_windowed_metrics', {}),
        command_receipt_derivatives=value.get('command_receipt_derivatives'),
        raw_capture_timing=value.get('timing_in_raw_capture_window'),
        session_independence_verified=False, split='quarantine',
        nmpc_closed_loop_improvement_claim=False,
        limitation='Summary bytes and closed manifest checked; raw events not '
                   'rescanned, collector IDs do not certify independent sessions.')


def build_summary(workspace, inventory_path, summary_paths):
    root = Path(workspace).resolve()
    inventory_path = Path(inventory_path).resolve()
    inventory = json.loads(inventory_path.read_text())
    old_runs = inventory.get('runs')
    if not isinstance(old_runs, list) or len(old_runs) != inventory.get('run_count'):
        raise ValueError('legacy_inventory_count_mismatch')
    paths = [r.get('path') for r in old_runs]
    if len(paths) != len(set(paths)):
        raise ValueError('duplicate_legacy_path')
    release_path = root/'releases/S8/manifest.json'
    release = json.loads(release_path.read_text())
    code, snapshots = code_evidence(root, release)
    runs = [csv_evidence(root, r) for r in old_runs]
    accepted_id = release.get('accepted_run', {}).get('control')
    accepted = [r for r in runs if r['run_id'] == accepted_id and r['accepted_S8']]
    reconstruction_id = release.get('accepted_run', {}).get('reconstruction')
    reconstruction = root/'data'/('origin_reference_' + str(reconstruction_id) + '.csv')
    pair = dict(control_run_id=accepted_id, reconstruction_run_id=reconstruction_id,
        release_manifest_sha256=digest(release_path),
        accepted_control_integrity=accepted[0]['integrity'] if len(accepted) == 1 else 'not_unique',
        reconstruction_path=str(reconstruction.relative_to(root)),
        reconstruction_sha256=digest(reconstruction) if reconstruction.is_file() else None,
        reconstruction_expected_sha256=None,
        reconstruction_bytes_authenticated=False,
        baseline_eligibility_scope='accepted tag/code, frozen control evidence and '
            'declared reconstruction revision; no authenticated historical '
            'reconstruction bytes or running binary',
        pairing_basis='explicit releases/S8/manifest.json user-accepted pairing; '
                      'not guessed by timestamps or file adjacency')
    accepted_revisions = release.get('runtime_revisions', {})
    control_revision_matches = len(accepted) == 1 and (
        sorted(accepted[0].get('observed_code_revisions', {})) ==
        [accepted_revisions.get('control_pub')])
    reconstruction_revisions = set()
    if reconstruction.is_file():
        with reconstruction.open() as stream:
            for row in csv.DictReader(stream):
                reconstruction_revisions.add(row.get('code_revision') or 'unrecorded')
    pair['runtime_revisions'] = accepted_revisions
    pair['control_revision_matches'] = control_revision_matches
    pair['reconstruction_observed_revisions'] = sorted(reconstruction_revisions)
    pair['reconstruction_revision_matches'] = sorted(reconstruction_revisions) == [accepted_revisions.get('trajectory_reconstructor')]
    pair['accepted_baseline_reference_eligible'] = bool(
        code['all_tag_files_match_release'] and pair['accepted_control_integrity'] == 'matched'
        and pair['control_revision_matches'] and pair['reconstruction_revision_matches'])
    summaries = [closed_summary(p) for p in summary_paths]
    if len({r['run_id'] for r in summaries}) != len(summaries):
        raise ValueError('duplicate_closed_run_summary')
    if len({r['declared_events_sha256'] for r in summaries}) != len(summaries):
        raise ValueError('duplicate_closed_raw_event_declaration')
    counts = Counter(r['integrity'] for r in runs)
    result = dict(schema_version=1, terrain_label='smooth',
        terrain_evidence='user_confirmed_same_indoor_smooth_scene',
        legacy_inventory_path=str(inventory_path), legacy_inventory_sha256=digest(inventory_path),
        historical_run_count=len(runs), historical_integrity_counts=dict(counts),
        historical_matched_row_count=sum(r['observed_rows'] for r in runs if r['integrity'] == 'matched'),
        historical_runs=runs, accepted_S8_pair=pair, code_provenance=code,
        closed_current_runs=summaries,
        current_run_group=dict(independence_verified=False,
            collector_session_ids=[r['session_id'] for r in summaries],
            split_unit='whole_physical_session_not_collector_ID',
            classification='same_day_smooth_collection_group_independence_unverified',
            independent_test_session_count=0),
        nmpc_control_authority=False, production_changed=False,
        dynamics_calibration_certified=False, parameter_candidate_promotion=False,
        nmpc_closed_loop_improvement_claim=False,
        metric_comparison_policy='Per CSV/run and exact field/selection only. '
            'Do not pool origin ey and front-axle errors, compare different code '
            'revisions as matched experiments, or label historical replay a new closed loop.',
        other_terrain_policy='grass_bumpy/asphalt/unknown slots have no real-vehicle '
            'validated dynamics parameters in this evidence package.')
    return result, snapshots, release_path


def write_package(workspace, inventory_path, summary_paths, output_dir):
    output = Path(output_dir).resolve()
    if output.exists():
        raise ValueError('output_directory_must_be_new')
    result, snapshots, release_path = build_summary(workspace, inventory_path, summary_paths)
    output.mkdir(parents=True, exist_ok=False)
    for relative, content in snapshots.items():
        path = output/'.source'/relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(content)
    frozen_release = output/'.source'/'releases/S8/manifest.json'
    frozen_release.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(str(release_path), str(frozen_release))
    (output/'code_provenance.json').write_text(json.dumps(result['code_provenance'], indent=2, sort_keys=True)+'\n')
    summary_file = output/'evidence_summary.json'
    summary_file.write_text(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False)+'\n')
    baselines = []
    if result['accepted_S8_pair']['accepted_baseline_reference_eligible']:
        baselines.append(dict(terrain='smooth', path=str(frozen_release),
            verification_kind='user_accepted_S8', actual_control_mode='legacy_slqr',
            tag='S8', accepted_run=result['accepted_S8_pair']['control_run_id'],
            verification_scope='existing accepted S8 code/config pairing; no new physical calibration'))
    experiments = [dict(terrain='smooth', path=r['summary_path'],
        actual_control_mode=r['actual_control_mode'],
        controller_source_identity=r['controller_source_identity'],
        run_id=r['run_id'], session_independence_verified=False, nmpc_control_authority=False)
        for r in result['closed_current_runs']]
    library = scene_library(baselines, [], experiments, [], [])
    library.update(terrain_label_basis='user_confirmed', nmpc_control_authority=False,
        new_physical_model_parameters={}, independent_test_session_count=0)
    (output/'terrain_library.json').write_text(json.dumps(library, indent=2, sort_keys=True)+'\n')
    matched = result['historical_integrity_counts'].get('matched', 0)
    lines = ['# smooth S8 evidence index', '',
        'S8 remains the only vehicle controller. NMPC has no command authority.', '',
        'Historical allowlist: %d files, %d hash/row/revision matches.' %
        (result['historical_run_count'], matched),
        'Closed current raw-run summaries: %d. Raw JSONL was not rescanned.' % len(experiments),
        'Collector session IDs do not establish independent physical sessions.', '',
        'The accepted S8 tag and current instrumented source are separate hashes in '
        '`code_provenance.json`; copied launches are under hidden `.source/`.', '',
        'Per-file internal errors are descriptive. Origin ey, front-axle error, '
        'physical tracking truth, and command/physical jerk must not be pooled.', '',
        'Only smooth has user-confirmed real runs. Other terrain slots contain no '
        'fabricated validated model parameters. No YAML or production code was changed.', '',
        '[Full evidence](evidence_summary.json) · [Terrain registry](terrain_library.json)', '']
    (output/'README.md').write_text('\n'.join(lines))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', required=True, type=Path)
    parser.add_argument('--legacy-inventory', required=True, type=Path)
    parser.add_argument('--run-summary', action='append', default=[], type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    args = parser.parse_args()
    try:
        result = write_package(args.workspace, args.legacy_inventory, args.run_summary, args.output_dir)
    except (ValueError, OSError, KeyError, TypeError, csv.Error) as error:
        parser.exit(2, 'ERROR: %s\n' % error)
    print('Indexed %d historical files and %d closed smooth runs; no parameter or authority changes.' %
          (result['historical_run_count'], len(result['closed_current_runs'])))


if __name__ == '__main__':
    main()
