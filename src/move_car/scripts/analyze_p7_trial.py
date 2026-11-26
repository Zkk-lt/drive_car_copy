#!/usr/bin/env python3
"""Stream a recorded P7 trial into a new offline review directory.

No ROS imports, vehicle actions, Config writes, interpolation, or event repair.
Large message arrays and serialized/base64 payloads are never retained.
"""
import argparse
import csv
import hashlib
import gzip
import json
import math
from collections import Counter, defaultdict
from contextlib import ExitStack
from pathlib import Path
from control_research import p7_bend_metrics


def finite(x):
    return not isinstance(x, bool) and isinstance(x, (int, float)) and math.isfinite(x)


def statistics(values):
    a = sorted(float(v) for v in values if finite(v))
    if not a:
        return dict(n=0, p50=None, p95=None, p99=None, max=None)
    def quantile(p):
        z = p*(len(a)-1)
        i = int(z)
        return a[i]+(a[min(i+1, len(a)-1)]-a[i])*(z-i)
    return dict(n=len(a), min=a[0], mean=sum(a)/len(a), p50=quantile(.5),
                p95=quantile(.95), p99=quantile(.99), max=a[-1])


def write_json(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write('\n')


def prefix(reason):
    return str(reason).split(';', 1)[0]


def selected_straight_conditioning(published, diagnostics, candidates):
    """Prove activation from a matched candidate, never from configured weight."""
    counts = Counter()
    for row in published:
        if (row.get('owner') != 'nmpc' or row.get('history_committed') is not True or
                not isinstance(row.get('sequence'), int) or isinstance(row['sequence'], bool) or
                not finite(row.get('speed')) or row['speed'] <= .18):
            continue
        counts['moving_nmpc_publications'] += 1
        candidate = candidates.get(diagnostics.get(row.get('cycle'), {}).get('candidate_cycle'))
        if (not candidate or candidate.get('valid') is not True or
                candidate.get('run_id') != row.get('run_id') or candidate.get('epoch') != row.get('epoch') or
                candidate.get('previous_sequence') != row['sequence']-1):
            counts['unmatched_candidate'] += 1
            continue
        fields = dict(field.split('=', 1) for field in str(candidate.get('reason', '')).split(';') if '=' in field)
        try:
            blend = float(fields['straight_tracking_blend'])
            damping = float(fields['straight_yaw_damping_weight'])
        except (KeyError, ValueError):
            counts['unlogged_conditioning'] += 1
            continue
        if not (0. <= blend <= 1. and 0. <= damping <= 4.):
            counts['unlogged_conditioning'] += 1
            continue
        counts['matched_with_conditioning_fields'] += 1
        counts['line_fit_positive'] += blend > 0.
        counts['yaw_damping_active_gt_0p01'] += damping > .01
    return dict(counts)


def raw_directory(trial):
    folders = [p.parent for p in (trial/'raw').glob('*/manifest.json')]
    if len(folders) != 1:
        raise ValueError('expected exactly one raw/*/manifest.json under --trial')
    return folders[0]


def publication_derivatives(published, diagnostics, requests):
    """Use publish-call timestamps and contiguous history; never callback dt."""
    groups = defaultdict(list)
    rows, previous, previous_previous, previous_a, previous_dt = [], None, None, None, None
    skipped = Counter()
    for p in published:
        if p.get('publish_returned') is not True or p.get('history_committed') is not True:
            skipped['publication_failed_or_history_uncommitted'] += 1
            previous = previous_previous = previous_a = previous_dt = None
            continue
        if not all(isinstance(p.get(k), int) and not isinstance(p.get(k), bool) and p[k] >= 0
                   for k in ('epoch', 'sequence')):
            skipped['published_record_identity_invalid'] += 1
            previous = previous_previous = previous_a = previous_dt = None
            continue
        t = p.get('publish_steady_start_ns')
        if not (finite(t) and finite(p.get('speed')) and finite(p.get('steering'))):
            skipped['missing_publish_timestamp_or_command'] += 1
            previous = previous_previous = previous_a = previous_dt = None
            continue
        key = (p.get('epoch'), p.get('sequence'))
        d = diagnostics.get(p.get('cycle'), {})
        stop = d.get('emergency_stop')
        now = dict(p=p, key=key, t=t*1e-9, v=p['speed'], steering=p['steering'], stop=stop)
        if previous:
            dt = now['t']-previous['t']
            consecutive = key[0] == previous['key'][0] and finite(key[1]) and key[1] == previous['key'][1]+1
            if not consecutive or dt <= 0:
                skipped['history_gap_or_nonpositive_publish_interval'] += 1
                previous_previous = previous_a = previous_dt = None
            else:
                a = (now['v']-previous['v'])/dt
                rate = (now['steering']-previous['steering'])/dt
                jerk = None if previous_a is None else (a-previous_a)/(.5*(dt+previous_dt))
                stage = ('normal_pair' if stop is False and previous['stop'] is False else
                         'safety_stop_pair' if stop is True or previous['stop'] is True else 'safety_state_unknown')
                owner = p.get('owner', 'unknown')
                switch = 'switch' if owner != previous['p'].get('owner', 'unknown') else 'stable_pair'
                q = requests.get(p.get('cycle'), {})
                limits = [q.get(k) for k in ('max_steering_rate', 'max_acceleration', 'max_deceleration', 'max_jerk')]
                limits_known = all(finite(v) and v > 0 for v in limits)
                row = dict(cycle=p.get('cycle'), sequence=p.get('sequence'), owner=owner,
                           previous_owner=previous['p'].get('owner'), stage=stage,
                           publish_interval_ms=dt*1000, steering_rate=rate, acceleration=a, jerk=jerk,
                           limits_known=limits_known, steering_rate_violated=None,
                           acceleration_violated=None, jerk_violated=None)
                if limits_known:
                    row.update(steering_rate_violated=abs(rate)>limits[0]+1e-6,
                               acceleration_violated=a>limits[1]+1e-6 or a < -limits[2]-1e-6,
                               jerk_violated=None if jerk is None else abs(jerk)>limits[3]+1e-6)
                groups[stage+'/'+owner+'/'+switch].append(row)
                if (previous_previous and stage == 'normal_pair' and previous_previous['stop'] is False and
                        owner == previous['p'].get('owner') == previous_previous['p'].get('owner')):
                    groups['normal_stable_three/'+owner].append(row)
                rows.append(row)
                previous_a, previous_dt = a, dt
                previous_previous = previous
        previous = now
    summary = {}
    for name, values in groups.items():
        summary[name] = dict(n=len(values), limits_known=sum(r['limits_known'] for r in values),
            steering_rate_violations=sum(r['steering_rate_violated'] is True for r in values),
            acceleration_violations=sum(r['acceleration_violated'] is True for r in values),
            jerk_violations=sum(r['jerk_violated'] is True for r in values),
            steering_rate_abs=statistics(abs(r['steering_rate']) for r in values),
            acceleration=statistics(r['acceleration'] for r in values), jerk=statistics(r['jerk'] for r in values))
    return dict(groups=summary, skipped=dict(skipped),
        method='Actual publish_steady_start_ns; a=delta_v/dt; jerk=delta_a/(0.5*(dt+previous_dt)). Never bridge missing command sequences.',
        scope='Software command finite differences, not physical vehicle acceleration/jerk. Safety-stop windows remain separate.'), rows


def analyze_trial(trial, output):
    trial, output = Path(trial).resolve(), Path(output).resolve()
    analyzer_source_sha256 = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    raw = raw_directory(trial)
    source, manifest_path = raw/'events.jsonl', raw/'manifest.json'
    if not source.is_file() and (raw/'events.jsonl.gz').is_file():
        source = raw/'events.jsonl.gz'
    manifest_bytes = manifest_path.read_bytes()
    manifest = json.loads(manifest_bytes)
    if not source.is_file():
        raise ValueError('raw events.jsonl missing')
    # Never create analysis files inside the evidence being reviewed.
    if output == trial or trial in output.parents:
        raise ValueError('--output must be outside the original trial directory')
    output.mkdir(parents=True, exist_ok=False)
    counts, errors, failed, duplicate, gaps = Counter(), [], [], [], []
    published, diagnostics, requests, candidates = [], {}, {}, {}
    metrics, solver, candidate_reasons, dimensions = defaultdict(list), defaultdict(list), Counter(), Counter()
    first = last = None
    digest, seen, previous_key = hashlib.sha256(), set(), None
    parsed_lines = invalid_lines = foreign_events = physical_claims = 0
    n_candidates = attempted = valid_candidates = feasible_candidates = timeouts = 0
    feedback_active = feedback_fresh = slip_valid = 0
    command_publishers, imu_frames = Counter(), Counter()
    keep_published = ('run_id', 'epoch', 'sequence', 'cycle', 'owner', 'reason', 'speed', 'steering', 'acceleration',
                      'control_dt', 'publish_steady_start_ns', 'publish_steady_end_ns', 'publish_returned',
                      'history_committed', 'physical_execution_confirmed')
    keep_candidate = ('cycle', 'valid', 'feasible', 'reason', 'evaluations', 'decision_dimension', 'solve_ms', 'worker_ms', 'timed_out')
    timing_keys = ('callback_ms', 'previous_callback_total_ms', 'algorithm_ms', 'algorithm_cpu_ms', 'callback_cpu_ms',
                   'arbitration_ms', 'history_commit_ms', 'publish_ms', 'publish_interval_ms', 'queue_delay_ms',
                   'wakeup_lateness_ms', 'callback_wait_proxy_ms')
    with ExitStack() as stack:
        def writer(name, fields):
            stream = stack.enter_context((output/name).open('x', newline=''))
            out = csv.DictWriter(stream, fieldnames=fields, extrasaction='ignore')
            out.writeheader()
            return out
        pub_writer = writer('published.csv', keep_published)
        cand_writer = writer('candidates.csv', keep_candidate)
        diag_writer = writer('diagnostics.csv', ('cycle', 'candidate_cycle', 'emergency_stop', 'selected_owner', 'reason')+timing_keys)
        stream = stack.enter_context(gzip.open(source,'rb') if source.suffix=='.gz' else source.open('rb'))
        for line_number, line in enumerate(stream, 1):
            digest.update(line)
            try:
                event = json.loads(line, parse_constant=lambda value: (_ for _ in ()).throw(ValueError('nonfinite '+value)))
                if not isinstance(event, dict) or not isinstance(event.get('payload'), dict):
                    raise ValueError('event/payload must be objects')
            except (ValueError, TypeError) as exc:
                invalid_lines += 1
                if len(errors) < 100:
                    errors.append(dict(line=line_number, error=str(exc)))
                continue
            parsed_lines += 1
            topic, payload = event.get('topic', 'unknown'), event['payload']
            counts[topic] += 1
            mono = event.get('receipt_monotonic_ns')
            if finite(mono):
                first = mono if first is None else min(first, mono)
                last = mono if last is None else max(last, mono)
            if topic == '/cmd_vel':
                command_publishers[event.get('publisher', 'unknown')] += 1
            elif topic == '/imu':
                imu_frames[event.get('frame_id', '')] += 1
            elif topic == '/control_research/p5/published':
                row = {key: payload.get(key) for key in keep_published}
                pub_writer.writerow(row)
                if row['run_id'] != manifest.get('run_id'):
                    foreign_events += 1
                    continue
                # Keep the ROS cycle stamp for the geometry-only review.
                # Neither the stable publication CSV nor command timing changes.
                row['cycle_stamp'] = payload.get('cycle_stamp')
                row['publish_ros_start'] = payload.get('publish_ros_start')
                published.append(row)
                physical_claims += payload.get('physical_execution_confirmed') is True
                if payload.get('publish_returned') is not True or payload.get('history_committed') is not True:
                    failed.append(dict(epoch=row['epoch'], sequence=row['sequence'], reason=payload.get('failure_reason')))
                    continue
                if not all(isinstance(row[k], int) and not isinstance(row[k], bool) and row[k] >= 0
                           for k in ('epoch', 'sequence')):
                    failed.append(dict(epoch=row['epoch'], sequence=row['sequence'], reason='published_record_identity_invalid'))
                    continue
                key = (row['epoch'], row['sequence'])
                if key in seen:
                    duplicate.append(list(key))
                    continue
                seen.add(key)
                if previous_key and (key[0] != previous_key[0] or not finite(key[1]) or key[1] != previous_key[1]+1):
                    gaps.append(dict(previous=list(previous_key), current=list(key)))
                previous_key = key
            elif topic == '/control_research/p5/diagnostics':
                row = {key: payload.get(key) for key in ('cycle', 'candidate_cycle', 'emergency_stop', 'selected_owner', 'reason')+timing_keys}
                diagnostics[payload.get('cycle')] = row
                diag_writer.writerow(row)
                for key in timing_keys:
                    if finite(payload.get(key)):
                        metrics[key].append(payload[key])
            elif topic == '/control_research/p5_test/p6/request':
                base = payload.get('base', {})
                requests[base.get('cycle')] = {key: base.get(key) for key in ('max_steering_rate', 'max_acceleration', 'max_deceleration', 'max_jerk')}
            elif topic == '/control_research/p5_test/p6/candidate':
                base = payload.get('base', {})
                row = {key: base.get(key) for key in keep_candidate}
                candidates[base.get('cycle')] = row
                cand_writer.writerow(row)
                row.update((k, base.get(k)) for k in ('previous_sequence', 'run_id', 'epoch'))
                n_candidates += 1
                valid_candidates += base.get('valid') is True
                feasible_candidates += base.get('feasible') is True
                timeouts += base.get('timed_out') is True
                feedback_active += payload.get('feedback_active') is True
                feedback_fresh += payload.get('feedback_used_new_observation') is True
                slip_valid += payload.get('parameters', {}).get('slip_valid') is True
                candidate_reasons[prefix(base.get('reason', 'unknown'))] += 1
                if finite(base.get('evaluations')) and base['evaluations'] > 0:
                    attempted += 1
                    dimensions[str(base.get('decision_dimension'))] += 1
                    for key in ('solve_ms', 'worker_ms'):
                        if finite(base.get(key)):
                            solver[key].append(base[key])
    raw_sha = digest.hexdigest()
    writer_status = manifest.get('writer', {})
    complete = (manifest.get('state') == 'closed' and invalid_lines == 0 and
        writer_status.get('accepted') == writer_status.get('written') == parsed_lines and
        writer_status.get('queue_dropped') == writer_status.get('unwritten') == 0 and
        writer_status.get('writer_error') is None and manifest.get('events_sha256') == raw_sha)
    def group(rows):
        return dict(n=len(rows), owners=dict(Counter(p.get('owner', 'unknown') for p in rows)),
                    reasons=dict(Counter(prefix(p.get('reason')) for p in rows)),
                    speed=statistics(p.get('speed') for p in rows))
    actual_published = [p for p in published if p.get('publish_returned') is True]
    normal, stop, unknown = [], [], []
    for p in actual_published:
        flag = diagnostics.get(p.get('cycle'), {}).get('emergency_stop')
        (normal if flag is False else stop if flag is True else unknown).append(p)
    context = Counter()
    for p in published:
        if prefix(p.get('reason')) == 'candidate_context_mismatch':
            d = diagnostics.get(p.get('cycle'), {})
            c = candidates.get(d.get('candidate_cycle'), {})
            context[prefix(c.get('reason', 'candidate_not_observed'))] += 1
    boundary, boundary_rows = publication_derivatives(published, diagnostics, requests)
    if boundary_rows:
        with (output/'publication_boundaries.csv').open('x', newline='') as stream:
            out = csv.DictWriter(stream, fieldnames=list(boundary_rows[0]))
            out.writeheader()
            out.writerows(boundary_rows)
    result = dict(schema_version=1, run_id=manifest.get('run_id'), session_id=manifest.get('continuous_session_id'),
        terrain=manifest.get('terrain_label'), data_origin=manifest.get('data_origin', 'unknown'),
        raw_events=parsed_lines, topic_counts=dict(counts), duration_s=(last-first)*1e-9 if first is not None else None,
        source_events_sha256=raw_sha, source_manifest_sha256=hashlib.sha256(manifest_bytes).hexdigest(),
        analyzer_source_sha256=analyzer_source_sha256,
        collection_complete=complete, invalid_lines=invalid_lines, parse_errors_first_100=errors,
        recorder=dict(writer_status), upstream_transport_loss='unknown; recorder zero-drop and contiguous observed ledger do not certify upstream transport, unobserved prefix/suffix, or hardware sampling',
        publication_groups=dict(all=group(actual_published), positive_output=group([p for p in actual_published if finite(p.get('speed')) and p['speed']>.01]), normal=group(normal), safety_stop=group(stop), safety_state_unknown=group(unknown)),
        published_history=dict(samples=len(published), committed_unique=len(seen), first=[published[0]['epoch'],published[0]['sequence']] if published else None,
            last=[published[-1]['epoch'],published[-1]['sequence']] if published else None, duplicates=duplicate,
            observed_sequence_discontinuities=gaps, failed_publications=failed, foreign_run_events=foreign_events,
            strict_observed_history_contiguous=bool(seen) and not (duplicate or gaps or failed or foreign_events),
            physical_execution_claims=physical_claims, prefix_suffix_completeness='unknown'),
        candidates=dict(n=n_candidates, attempted=attempted, valid=valid_candidates, feasible=feasible_candidates,
            timed_out=timeouts, dimensions=dict(dimensions), reasons=dict(candidate_reasons), solve={k:statistics(v) for k,v in solver.items()},
            feedback_active=feedback_active, feedback_new_observation=feedback_fresh, slip_valid=slip_valid),
        context_mismatch_original_candidate_reasons=dict(context), timing_ms={k:statistics(v) for k,v in metrics.items()},
        callback_over_33p333=sum(x>1000/30 for x in metrics['callback_ms']),
        publish_interval_over_50=sum(x>50 for x in metrics['publish_interval_ms']),
        publication_boundary_constraints=boundary, command_publishers=dict(command_publishers), imu_frames=dict(imu_frames),
        bend_geometry=p7_bend_metrics.analyze_bend_geometry(trial, actual_published),
        bend_metrics_source_sha256=hashlib.sha256(Path(p7_bend_metrics.__file__).read_bytes()).hexdigest(),
        selected_straight_conditioning=selected_straight_conditioning(actual_published, diagnostics, candidates),
        limitations=['No physical calibration, true slip, controller promotion, or Config change is inferred.',
                     'Stop stages are retained and separated; ordinary jerk must not delay necessary braking.',
                     'Selected-owner subsets are not matched controller comparisons.',
                     'Published ROS commands are not confirmed physical execution.'])
    write_json(output/'summary.json', result)
    (output/'README.md').write_text('# P7 recorded trial review\n\nRun: '+str(result['run_id'])+
        '\n\nRecorder complete: '+str(complete)+'. See summary.json for retained safety-stop stages, actual published owners, history gaps, timing, publication boundaries and signed bend geometry. Front axle versus centerline and center versus observed history are reported separately.\n\n'+
        'Read-only offline analysis; no ROS node, lease, vehicle command, calibration conclusion or Config promotion. Upstream transport and unobserved history endpoints remain unknown.\n')
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial', required=True, help='completed P7 trial containing raw/RUN/manifest.json')
    parser.add_argument('--output', required=True, help='new analysis directory outside trial; never overwritten')
    args = parser.parse_args(argv)
    try:
        result = analyze_trial(args.trial, args.output)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    print('ANALYSIS_DIRECTORY='+str(Path(args.output).resolve()))
    print('actual owners='+json.dumps(result['publication_groups']['all']['owners'], sort_keys=True))
    print('collection_complete='+str(result['collection_complete'])+'; upstream transport=unknown; Config unchanged')


if __name__ == '__main__':
    main()
