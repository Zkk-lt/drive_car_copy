#!/usr/bin/env python3
"""Replay frozen P7 requests offline with the actually committed command history.

Stages:
  extract ARCHIVE OUTPUT
  compare FIXTURE BINARY OUTPUT
The files are derivative evidence; the real archive is never modified. Replay
does not update state/history using new commands and cannot measure tracking
improvements, future fallback frequency, or real end-to-end scheduling.
"""
import argparse
from collections import Counter
import csv
import datetime
import hashlib
import json
import math
from pathlib import Path
import re
from statistics import median
import subprocess

TIME_ZONE = datetime.timezone(datetime.timedelta(hours=8))
REQUEST = '/control_research/p5_test/p6/request'
CANDIDATE = '/control_research/p5_test/p6/candidate'
PUBLISHED = '/control_research/p5/published'
TOPIC = re.compile(rb'"topic": "([^"]+)"')
# Predefined episodes identified by audit_bend_phase.py and the straight
# comparison. We include the approach, before the cap becomes infeasible.
DEFAULT_WINDOWS = [
    ('near_straight', '17:01:01.873', '17:01:03.473'),
    ('turn_earlier_preview', '17:01:19.000', '17:01:20.599'),
    ('turn_tightening_approach', '17:01:20.600', '17:01:21.797'),
    ('turn_tightening', '17:01:21.798', '17:01:23.470'),
    ('turn_opening', '17:01:31.425', '17:01:33.265'),
    ('sustained_right', '17:02:00.548', '17:02:02.690'),
]


def read_ns(stamp):
    return stamp['secs'] * 10**9 + stamp['nsecs']


def read_time(stamp):
    return read_ns(stamp) * 1e-9


def wall_clock(stamp):
    return datetime.datetime.fromtimestamp(stamp, TIME_ZONE).strftime('%H:%M:%S.%f')[:-3]


def finite(values):
    return all(isinstance(x, (float, int)) and not isinstance(x, bool) and math.isfinite(x)
               for x in values)


def timing_distribution(rows, field):
    values = sorted(float(r[field]) for r in rows if float(r[field]) > 0.)
    if not values:
        return None
    def rank(p):
        return values[max(0, math.ceil(p * len(values)) - 1)]
    return dict(count=len(values), p50=rank(.5), p95=rank(.95), p99=rank(.99), maximum=values[-1])


def valid_local_state(q):
    observed = read_ns(q['state_observation_stamp'])
    snapshot = read_ns(q['header']['stamp'])
    receipts = [read_ns(q[k]) for k in ('wheel_receipt_stamp', 'steering_receipt_stamp', 'imu_receipt_stamp')]
    imu_source = read_ns(q['imu_source_stamp'])
    reference_stamp = read_ns(q['reference_stamp'])
    return (q['nominal_state_contract'] and q['state_valid'] and q['yaw_rate_valid'] and
            0. < q['state_confidence'] <= 1. and 0. < reference_stamp <= observed and
            0 <= snapshot - observed <= 25_000_000 and
            q['reference_frame_stamp'] == q['state_observation_stamp'] and
            all(t > 0 and 0 <= observed - t <= 120_000_000 for t in receipts) and
            max(receipts) - min(receipts) <= 60_000_000 and
            0 < imu_source <= receipts[-1] and observed - imu_source <= 120_000_000)


def extract(args):
    folder = args.archive
    raw = folder / 'raw' / folder.name / 'events.jsonl'
    manifest = json.loads((folder / 'RAM_ARCHIVE_MANIFEST.json').read_text())
    trial = json.loads((folder / 'trial.json').read_text())
    expected_sha = manifest['files'][str(raw.relative_to(folder))]['sha256']
    windows = args.window or DEFAULT_WINDOWS
    names = {REQUEST.encode(), CANDIDATE.encode(), PUBLISHED.encode()}
    requests, candidates, publications = {}, {}, {}
    stream_sha = hashlib.sha256()
    counts = Counter()
    with raw.open('rb') as stream:
        for line in stream:
            stream_sha.update(line)
            tag = TOPIC.search(line[:800])
            if tag is None or tag[1] not in names:
                continue
            event = json.loads(line)
            kind, payload = tag[1].decode(), event['payload']
            if kind == PUBLISHED:
                if (payload['run_id'] != folder.name or not payload['history_committed'] or
                        not payload['publish_returned']):
                    continue
                ident = payload['epoch'], payload['sequence']
                if ident in publications:
                    raise ValueError('duplicate committed publication')
                publications[ident] = payload
                continue
            base = payload['base']
            stamp = read_time(base['state_observation_stamp'] if kind == REQUEST else base['snapshot_stamp'])
            if base['run_id'] != folder.name or payload['computation_mode'] != 'flat_nmpc_slqr':
                continue
            time = wall_clock(stamp)
            window = next((name for name, start, end in windows if start <= time <= end), None)
            if window is None:
                continue
            ident = base['epoch'], base['cycle']
            current = requests if kind == REQUEST else candidates
            if ident in current:
                raise ValueError('duplicate request or candidate identity')
            current[ident] = (window, time, payload)
    if stream_sha.hexdigest() != expected_sha:
        raise ValueError('archived raw SHA did not match')
    packed, meta = [], []
    for ident, (window, time, request) in sorted(requests.items()):
        q = request['base']
        counts[(window, 'observed')] += 1
        if (not q['state_valid'] or not q['history_valid'] or not q['reference_valid'] or
                q['hard_stop'] or q['reference_braking'] or len(q['reference']) < 2 or
                not valid_local_state(q)):
            counts[(window, 'request_guard')] += 1
            continue
        if ident not in candidates:
            counts[(window, 'no_worker_result')] += 1
            continue
        _, _, candidate = candidates[ident]
        b, p = candidate['base'], candidate['parameters']
        # Replay only snapshots that were truly constructed by the controller,
        # even if the later solve found no feasible steering/speed sequence.
        if (b['previous_sequence'] != q['previous_sequence'] or
                b['reference_revision'] != q['reference_revision'] or
                b['config_revision'] != q['config_revision'] or
                b['config_hash'] != q['config_hash'] or
                b['snapshot_stamp'] != q['header']['stamp'] or
                b['model_adaptation_active'] or b['weight_adaptation_active'] or
                p['cycle'] != q['cycle'] or
                not finite(p['effective_limits']) or min(p['effective_limits']) <= 0. or
                not finite(p['effective_model']) or not finite(p['effective_weights']) or
                p['prediction_dt'] <= 0. or p['horizon_steps'] < 2):
            counts[(window, 'no_matching_effective_snapshot')] += 1
            continue
        sequence = q['previous_sequence']
        history = [publications.get((q['epoch'], i)) for i in range(max(sequence-255, 1), sequence+1)]
        # The archive's initial recorder-edge prefix is unknown. Require a
        # complete window of 256 records, exactly as in the live worker here.
        if len(history) != 256 or any(h is None for h in history):
            counts[(window, 'missing_commit')] += 1
            continue
        history_stamps = [read_ns(h['header']['stamp']) for h in history]
        snapshot = read_ns(q['header']['stamp'])
        if (history[-1]['speed'] != q['previous_speed'] or history[-1]['steering'] != q['previous_steering'] or
                history[-1]['acceleration'] != q['previous_acceleration'] or
                history[-1]['header']['stamp'] != q['previous_publish_stamp'] or
                history_stamps[-1] > snapshot or q['state_history_sequence'] >= sequence or
                any(not 0 < b-a <= 120_000_000 for a,b in zip(history_stamps,history_stamps[1:]))):
            counts[(window, 'noncausal_or_gap')] += 1
            continue
        if not all(poly['geometry_valid'] and finite([poly[k] for k in ('s','x','y','tangent_yaw','curvature')])
                   for poly in q['reference']) or any(a['s'] >= b['s'] for a,b in zip(q['reference'],q['reference'][1:])):
            counts[(window, 'invalid_geometry')] += 1
            continue
        cap = p['speed_cap']
        target = min(request['target_speed'], cap) if request['speed_target_separated'] else cap
        fields = [read_time(q['header']['stamp']), read_time(q['state_observation_stamp']), q['control_dt'],
                  cap, target, q['position_uncertainty'], q['yaw_uncertainty'], *q['state'],
                  q['previous_speed'], q['previous_steering'], q['previous_acceleration'],
                  q['baseline_speed'], q['baseline_steering'], p['horizon_steps'], p['control_knots'],
                  p['solver_iterations'], p['prediction_dt'], p['solve_budget_ms'],
                  *p['effective_model'], *p['effective_limits'], *p['effective_weights']]
        if not finite(fields):
            counts[(window, 'invalid_scalar')] += 1
            continue
        if q['cycle'] % args.stride:
            counts[(window, 'stride_skip')] += 1
            continue
        packed.append(' '.join(map(str, [q['cycle'], len(q['reference']), len(history), *fields])))
        packed.extend(' '.join(str(poly[k]) for k in ('s','x','y','tangent_yaw','curvature'))
                      for poly in q['reference'])
        packed.extend(' '.join(map(str, [stamp * 1e-9, h['speed'], h['steering']]))
                      for stamp,h in zip(history_stamps, history))
        meta.append(dict(cycle=q['cycle'], time=time, window=window, previous_sequence=sequence,
                         original_worker_reason=b['reason'].split(';')[0], original_candidate_valid=b['valid']))
        counts[(window, 'fixture_rows')] += 1
    if not meta:
        raise ValueError('no causal fixture rows')
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'requests.txt').write_text('\n'.join(packed)+'\n')
    metadata = dict(run_id=folder.name, original_event_sha256=expected_sha, binaries=trial['binaries'],
                    windows=windows, fixture_stride=args.stride,
                    counts={f'{w}/{k}':v for (w,k),v in sorted(counts.items())}, rows=meta)
    (args.output/'fixture.json').write_text(json.dumps(metadata, indent=2)+'\n')
    print(dict(rows=len(meta), counts=metadata['counts']))


def compare(args):
    meta = json.loads((args.fixture/'fixture.json').read_text())
    expected = {int(r['cycle']):r for r in meta['rows']}
    args.output.mkdir(parents=True, exist_ok=False)
    target = args.output/'paired.csv'
    options = [str(args.binary.resolve()), str(args.speed_reserve), str(args.release_prior), str(args.budget)]
    with (args.fixture/'requests.txt').open() as inp, target.open('x') as out:
        subprocess.run(options, stdin=inp, stdout=out, check=True)
    with target.open() as f:
        rows = list(csv.DictReader(f))
    if len(rows) != len(expected) or {int(r['cycle']) for r in rows} != set(expected):
        raise ValueError('fixture cycles differ from paired replay')
    with target.open('w') as f:
        writer = csv.DictWriter(f, fieldnames=['time', 'window', *rows[0].keys()])
        writer.writeheader()
        for row in rows:
            key = expected[int(row['cycle'])]
            writer.writerow(dict(time=key['time'], window=key['window'], **row))
    group = {}
    for name, _, _ in meta['windows']:
        sample = [r for r in rows if expected[int(r['cycle'])]['window'] == name]
        both = [r for r in sample if r['old_ok'] == r['new_ok'] == '1']
        reserve_enabled = [r for r in both if float(r['reserve_mps'])>1e-8]
        release_enabled = [r for r in both if r['release_active']=='1']

        def delta(subset, stem, absolute=False):
            values = [[float(r['old_'+stem]), float(r['new_'+stem])] for r in subset]
            if absolute:
                values = [[abs(a),abs(b)] for a,b in values]
            return median([b-a for a,b in values]) if values else None

        group[name] = dict(
            cycles=len(sample), valid_before=sum(r['old_ok']=='1' for r in sample),
            valid_after=sum(r['new_ok']=='1' for r in sample),
            archived_worker_candidates_valid=sum(expected[int(r['cycle'])]['original_candidate_valid'] for r in sample),
            infeasible_before=dict(Counter(r['old_reason'] for r in sample if r['old_ok']!='1')),
            infeasible_after=dict(Counter(r['new_reason'] for r in sample if r['new_ok']!='1')),
            unchanged_cap=sum(r['cap_unchanged']=='1' for r in sample),
            same_terminal_contract=sum(r['terminal_unchanged']=='1' for r in sample),
            target_speed_reduced=sum(float(r['reserve_mps'])>1e-8 for r in sample),
            unwind_enabled=sum(r['release_active']=='1' for r in sample),
            first_reduced_target_time=next((expected[int(r['cycle'])]['time'] for r in sample
                                            if float(r['reserve_mps'])>1e-8), None),
            first_reduced_target_feasible_time=next((expected[int(r['cycle'])]['time'] for r in reserve_enabled), None),
            max_speed_reserve_mps=max((float(r['reserve_mps']) for r in sample), default=None),
            max_release_weight=max((float(r['release_weight']) for r in sample), default=None),
            tightening_signal_count=sum(float(r['tightening'])>0. for r in sample),
            opening_signal_count=sum(float(r['opening'])>0. for r in sample),
            independently_revalidated_before=sum(r['old_validated']=='1' for r in sample),
            independently_revalidated_after=sum(r['new_validated']=='1' for r in sample),
            paired_feasible=len(both),
            first_command_steering_changed=sum(abs(float(r['old_steering'])-float(r['new_steering']))>1e-9 for r in both),
            first_command_speed_changed=sum(abs(float(r['old_speed'])-float(r['new_speed']))>1e-9 for r in both),
            reserve_paired_feasible=len(reserve_enabled),
            reserve_first_speed_delta_mps=delta(reserve_enabled,'speed'),
            reserve_first_steer_delta_rad=delta(reserve_enabled,'steering'),
            reserve_first_steer_magnitude_delta_rad=delta(reserve_enabled,'steering',absolute=True),
            reserve_predicted_speed_after_0p6s_delta_mps=delta(reserve_enabled,'speed_at_0p6s'),
            reserve_predicted_steer_after_0p6s_delta_rad=delta(reserve_enabled,'steer_at_0p6s'),
            release_paired_feasible=len(release_enabled),
            release_first_steer_magnitude_delta_rad=delta(release_enabled,'steering',absolute=True),
            release_predicted_steer_magnitude_after_0p6s_delta_rad=delta(release_enabled,'steer_at_0p6s',absolute=True),
            old_timeouts=sum(r['old_timeout']=='1' for r in sample),
            new_timeouts=sum(r['new_timeout']=='1' for r in sample),
            measured_nominal_policy_before_ms=timing_distribution(sample,'old_solve_ms'),
            measured_nominal_policy_after_ms=timing_distribution(sample,'new_solve_ms'),
        )
    result = dict(data_origin='archived_real_frozen_requests_open_loop', archive=meta['run_id'],
                  original_event_sha256=meta['original_event_sha256'], archive_binaries=meta['binaries'],
                  replay_binary_sha256=hashlib.sha256(args.binary.read_bytes()).hexdigest(),
                  replay_fixture_sha256=hashlib.sha256((args.fixture/'requests.txt').read_bytes()).hexdigest(),
                  phase_windows=meta['windows'], inclusion=meta['counts'], budget_ms=args.budget,
                  reserve_option=args.speed_reserve, release_option=args.release_prior,
                  limits='Identical archived hard cap/model/limits/terminal endpoint/command history in each pair',
                  note=('Recomputed nominal flat8+SLQR policy build; does not reconstruct asynchronous feedback, '
                        'ROS candidate acceptance or closed-loop response. Frozen records are replayed '
                        'without previous optimizer warm state, which the archive does not preserve.'),
                  groups=group)
    (args.output/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
    print(args.output/'summary.json')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='action', required=True)
    run = commands.add_parser('extract')
    run.add_argument('archive', type=Path)
    run.add_argument('output', type=Path)
    run.add_argument('--stride', type=int, choices=range(1, 21), default=3)
    run.add_argument('--window', action='append', nargs=3, metavar=('LABEL','START_CST','END_CST'))
    run = commands.add_parser('compare')
    run.add_argument('fixture', type=Path)
    run.add_argument('binary', type=Path)
    run.add_argument('output', type=Path)
    run.add_argument('--speed-reserve', type=float, required=True)
    run.add_argument('--release-prior', type=float, required=True)
    run.add_argument('--budget', type=float, default=8.)
    args = parser.parse_args()
    (extract if args.action == 'extract' else compare)(args)


if __name__ == '__main__':
    main()
