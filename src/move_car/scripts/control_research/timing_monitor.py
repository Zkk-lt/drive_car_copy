"""Passive timing evidence, Python 2/3, no ROS or control output imports.

Collector receipt is not a publisher timestamp. Only the optional diagnostic
probe can expose timer queue/release or the instrumented publish call boundary.
"""
from __future__ import division
import csv
import hashlib
import json
import math
import os
import time

from .recording import monotonic_ns


def finite(value):
    try:
        return not (math.isnan(float(value)) or math.isinf(float(value)))
    except (TypeError, ValueError, OverflowError):
        return False


def distribution(values):
    values = sorted(float(v) for v in values if finite(v))
    if not values:
        return {'count': 0, 'p50': None, 'p95': None, 'p99': None, 'max': None}
    def q(p):
        index = (len(values) - 1) * p
        lo = int(math.floor(index)); hi = int(math.ceil(index))
        return values[lo] + (index - lo) * (values[hi] - values[lo])
    return {'count': len(values), 'p50': q(.5), 'p95': q(.95),
            'p99': q(.99), 'max': values[-1]}


def file_sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _text(path):
    with open(path) as stream:
        return stream.read().strip()


def _stat(path):
    text = _text(path)
    # comm can contain whitespace or parentheses; numeric fields start after
    # the last closing parenthesis. Index zero below is kernel stat field 3.
    tail = text[text.rfind(')') + 2:].split()
    if len(tail) < 39:
        raise ValueError('truncated proc stat')
    return {'comm': text[text.find('(') + 1:text.rfind(')')],
            'state': tail[0], 'user_ticks': int(tail[11]),
            'system_ticks': int(tail[12]), 'priority': int(tail[15]),
            'nice': int(tail[16]), 'threads': int(tail[17]),
            'starttime_ticks': int(tail[19]), 'last_cpu': int(tail[36]),
            'rt_priority': int(tail[37]), 'policy': int(tail[38])}


def proc_sample(pids, root='/proc', cpu_root='/sys/devices/system/cpu', max_threads=256):
    """Read at low frequency outside callbacks; disappearing PIDs stay explicit."""
    sample = {'kind': 'resources', 'receipt_monotonic_ns': monotonic_ns(),
              'receipt_wall_ns': int(time.time() * 1000000000),
              'cpu_stat': None, 'loadavg': None, 'frequency_khz': {},
              'processes': {}, 'errors': []}
    for key, name in [('cpu_stat', 'stat'), ('loadavg', 'loadavg')]:
        try:
            value = _text(os.path.join(root, name))
            sample[key] = value.splitlines()[0] if key == 'cpu_stat' else value
        except (IOError, OSError, ValueError) as exc:
            sample['errors'].append(key + ': ' + str(exc))
    try:
        for name in sorted(os.listdir(cpu_root)):
            if name.startswith('cpu') and name[3:].isdigit():
                path = os.path.join(cpu_root, name, 'cpufreq/scaling_cur_freq')
                try:
                    sample['frequency_khz'][name] = int(_text(path))
                except (IOError, OSError, ValueError):
                    pass
    except OSError as exc:
        sample['errors'].append('frequency: ' + str(exc))
    for pid in pids:
        data = {'tasks': {}, 'errors': []}
        sample['processes'][str(pid)] = data
        base = os.path.join(root, str(pid))
        try:
            data['stat'] = _stat(os.path.join(base, 'stat'))
            for name in ('status', 'io', 'schedstat'):
                try:
                    data[name] = _text(os.path.join(base, name))
                except (IOError, OSError) as exc:
                    data['errors'].append(name + ': ' + str(exc))
            tids = sorted(os.listdir(os.path.join(base, 'task')), key=int)
            data['task_count'] = len(tids)
            data['task_truncated'] = len(tids) > max_threads
            for tid in tids[:max_threads]:
                try:
                    task_base = os.path.join(base, 'task', tid)
                    data['tasks'][tid] = {'stat': _stat(os.path.join(task_base, 'stat')),
                                         'schedstat': _text(os.path.join(task_base, 'schedstat'))}
                except (IOError, OSError, ValueError) as exc:
                    data['errors'].append('task ' + tid + ': ' + str(exc))
        except (IOError, OSError, ValueError) as exc:
            data['errors'].append('process unavailable: ' + str(exc))
    return sample


def telemetry_event(kind, msg, topic, receipt_ros_ns, receipt_monotonic_ns, wall_ns):
    """Copy bounded scalar fields. Never reinterpret header as publish time."""
    header = getattr(msg, 'header', None)
    source = header.stamp.to_nsec() if header is not None else None
    data = {'kind': kind, 'topic': topic, 'receipt_ros_ns': int(receipt_ros_ns),
            'receipt_monotonic_ns': int(receipt_monotonic_ns), 'receipt_wall_ns': int(wall_ns),
            'source_stamp_ns': source or None,
            'source_stamp_semantics': ('S8_control_entry' if kind == 'controller' else
                                       'historical_control_snapshot' if kind == 'shadow' else
                                       'unknown_no_header'),
            'publisher': getattr(msg, '_connection_header', {}).get('callerid', 'unknown'),
            'actual_command_publish_ns': None, 'hardware_sample_ns': None,
            'timer_expected_ns': None, 'timer_expired_ns': None,
            'timer_dispatch_ns': None, 'callback_entry_ns': None}
    if kind == 'command':
        data.update({'command_speed_mps': msg.linear.x,
                     'command_steering_rad': msg.angular.z,
                     'command_stamp_semantics': 'collector_receipt_only; Twist.angular.z is steering, not yaw rate'})
    elif kind == 'controller':
        for name in ('mode', 'nmpc_active', 'safe_stop', 'collision_stop', 'tracking_valid',
                     'solve_ms', 'compute_ms', 'callback_before_telemetry_ms',
                     'timer_lateness_ms', 'fallback_reason', 'stop_reason'):
            data[name] = getattr(msg, name, None)
        data['callback_duration_semantics'] = 'before telemetry serialization/publication; not complete callback'
    elif kind == 'shadow':
        for name in ('run_id', 'config_hash', 'shadow_only', 'actual_owner', 'selected_owner',
                     'requested_mode', 'feasible', 'timed_out', 'reason', 'evaluations',
                     'iterations', 'solve_ms', 'shadow_cycle_ms', 'warm_started', 'max_violation'):
            data[name] = getattr(msg, name, None)
    else:
        raise ValueError('unsupported timing event kind')
    for key, value in list(data.items()):
        if isinstance(value, float) and not finite(value):
            data[key] = None
            data.setdefault('nonfinite_fields', []).append(key)
    return data


def _intervals(events, key):
    values = []; nonmonotonic = 0
    for a, b in zip(events, events[1:]):
        av, bv = a.get(key), b.get(key)
        if av is None or bv is None:
            continue
        if bv <= av:
            nonmonotonic += 1
        else:
            values.append((bv - av) / 1000000.)
    return {'ms': distribution(values), 'nonmonotonic_or_duplicate': nonmonotonic}


def _proc_summary(samples):
    busy = []; processes = {}; invalid = []
    for a, b in zip(samples, samples[1:]):
        if b.get('receipt_monotonic_ns', 0) <= a.get('receipt_monotonic_ns', 0):
            invalid.append('nonmonotonic_collector_resources'); continue
        try:
            va = [int(v) for v in a['cpu_stat'].split()[1:9]]
            vb = [int(v) for v in b['cpu_stat'].split()[1:9]]
            delta = [v - u for u, v in zip(va, vb)]
            if len(delta) != 8 or min(delta) < 0:
                raise ValueError('CPU counters reset/incomplete')
            if sum(delta):
                busy.append(100. * (sum(delta) - delta[3] - delta[4]) / sum(delta))
        except (KeyError, ValueError, TypeError, AttributeError) as exc:
            invalid.append(str(exc))
        for pid in set(a.get('processes', {})) & set(b.get('processes', {})):
            pa = a['processes'][pid]; pb = b['processes'][pid]
            bucket = processes.setdefault(pid, {'intervals': 0, 'summed_thread_runtime_ms': 0.,
                                                'summed_thread_runqueue_wait_ms': 0., 'invalid_intervals': 0,
                                                'observed_policy_nice': [], 'io_delta': {}, 'last_status': pb.get('status')})
            if (not pa.get('stat') or not pb.get('stat') or
                    pa['stat']['starttime_ticks'] != pb['stat']['starttime_ticks']):
                bucket['invalid_intervals'] += 1; continue
            policy = [pb['stat']['policy'], pb['stat']['rt_priority'], pb['stat']['nice']]
            if policy not in bucket['observed_policy_nice']:
                bucket['observed_policy_nice'].append(policy)
            bucket['intervals'] += 1
            try:
                ia = dict((k, int(v)) for k, v in (line.split(':', 1) for line in pa.get('io', '').splitlines()))
                ib = dict((k, int(v)) for k, v in (line.split(':', 1) for line in pb.get('io', '').splitlines()))
                for key in set(ia) & set(ib):
                    delta = ib[key] - ia[key]
                    if delta < 0:
                        bucket['invalid_intervals'] += 1
                    else:
                        bucket['io_delta'][key] = bucket['io_delta'].get(key, 0) + delta
            except ValueError:
                bucket['invalid_intervals'] += 1
            for tid in set(pa['tasks']) & set(pb['tasks']):
                ta = pa['tasks'][tid]; tb = pb['tasks'][tid]
                try:
                    if ta['stat']['starttime_ticks'] != tb['stat']['starttime_ticks']:
                        raise ValueError('thread identity changed')
                    da = [int(v) for v in ta['schedstat'].split()[:3]]
                    db = [int(v) for v in tb['schedstat'].split()[:3]]
                    delta = [v - u for u, v in zip(da, db)]
                    if len(delta) != 3 or min(delta) < 0:
                        raise ValueError('thread schedstat reset')
                    bucket['summed_thread_runtime_ms'] += delta[0] / 1000000.
                    bucket['summed_thread_runqueue_wait_ms'] += delta[1] / 1000000.
                except (KeyError, ValueError):
                    bucket['invalid_intervals'] += 1
    frequency = [v / 1000. for s in samples for v in s.get('frequency_khz', {}).values() if finite(v)]
    return {'sample_count': len(samples), 'aggregate_cpu_busy_percent': distribution(busy),
            'sampled_frequency_mhz': distribution(frequency), 'processes': processes,
            'invalid_intervals': invalid,
            'interpretation': 'schedstat sums over sampled concurrent threads are not callback queue or wall latency; transient/new/exited threads may be missed'}


def summarize_events(events):
    by_kind = {}
    for event in events:
        by_kind.setdefault(event.get('kind', 'unknown'), []).append(event)
    controller = by_kind.get('controller', []); shadow = by_kind.get('shadow', [])
    active = [r for r in shadow if (r.get('evaluations') or 0) > 0]
    result = {'schema_version': 1, 'event_counts': {k: len(v) for k, v in by_kind.items()},
              'controller_partial_callback_ms': distribution(r.get('callback_before_telemetry_ms') for r in controller),
              'controller_compute_ms': distribution(r.get('compute_ms') for r in controller),
              'controller_reported_timer_lateness_ms': distribution(r.get('timer_lateness_ms') for r in controller),
              'controller_entry_source_intervals': _intervals(controller, 'source_stamp_ns'),
              'command_collector_receipt_intervals': _intervals(by_kind.get('command', []), 'receipt_monotonic_ns'),
              'shadow_all_messages_solve_ms': distribution(r.get('solve_ms') for r in shadow),
              'shadow_active_solves_ms': distribution(r.get('solve_ms') for r in active),
              'shadow_timed_out_count': sum(r.get('timed_out') is True for r in shadow),
              'shadow_feasible_count': sum(r.get('feasible') is True for r in shadow),
              'shadow_actual_owner_violations': sum(r.get('actual_owner') != 'legacy_slqr' for r in shadow),
              'controller_active_nmpc_violations': sum(r.get('nmpc_active') is True for r in controller),
              'resources': _proc_summary(by_kind.get('resources', [])),
              'unknown': {'actual_command_publish_intervals': 'Twist has no header; collector receipt is only a transport observation',
                          'timer_wakeup_and_queue': 'not exposed by original S8 telemetry; optional independent diagnostic probe required',
                          'full_callback_duration': 'original telemetry excludes its serialization/publication and destructor tail',
                          'hardware_sensor_to_actuator_latency': 'acquisition/CAN execution clocks not verified'},
              'scope': 'passive software observation; no hardware real-time guarantee'}
    return result


def summarize_probe(path, start_ros_ns=None, end_ros_ns=None):
    """Existing P41 CSV, no source edits. Old traces keep missing expired unknown."""
    with open(path) as stream:
        reader = csv.DictReader(stream)
        required = set(('name', 'begin_ns', 'end_ns', 'cpu_begin_ns', 'cpu_end_ns',
                        'expected_ros_ns', 'timer_real_ros_ns', 'entry_ros_ns', 'tid'))
        if not required.issubset(reader.fieldnames or []):
            raise ValueError('missing required probe columns')
        rows = [{k: v if k == 'name' else int(v) for k, v in row.items()} for row in reader]
    if len(rows) >= 65536:
        raise ValueError('probe capacity reached; completeness is unknown')
    for row in rows:
        if row['end_ns'] < row['begin_ns'] or row['cpu_end_ns'] < row['cpu_begin_ns']:
            raise ValueError('negative/incomplete probe duration')
    callbacks = [r for r in rows if r['name'] == 'control_timer' and
                 (start_ros_ns is None or r['entry_ros_ns'] >= start_ros_ns) and
                 (end_ros_ns is None or r['entry_ros_ns'] < end_ros_ns)]
    release = []; queue = []; wrapping = []; combined = []; timer_invalid = 0
    for row in callbacks:
        expected = row['expected_ros_ns']; real = row['timer_real_ros_ns']; entry = row['entry_ros_ns']
        expired = row.get('timer_expired_ros_ns', 0)
        if not expected or not real or not entry or not expected <= real <= entry:
            timer_invalid += 1; continue
        combined.append((real - expected) / 1000000.)
        wrapping.append((entry - real) / 1000000.)
        if expired:
            if not expected <= expired <= real:
                timer_invalid += 1; continue
            release.append((expired - expected) / 1000000.)
            queue.append((real - expired) / 1000000.)
    # Select by the containing callback's ROS entry stamp, then compute publish
    # intervals from the publish call's MONOTONIC begin, never header stamps.
    publishes = sorted([r for r in rows if r['name'] == 'command_publish' and
                        any(c['tid'] == r['tid'] and c['begin_ns'] <= r['begin_ns'] and
                            r['end_ns'] <= c['end_ns'] for c in callbacks)],
                       key=lambda r: r['begin_ns'])
    interval_peaks = []
    for a, b in zip(publishes, publishes[1:]):
        owner = next(c for c in callbacks if c['tid'] == b['tid'] and
                     c['begin_ns'] <= b['begin_ns'] and b['end_ns'] <= c['end_ns'])
        interval_peaks.append({'interval_ms': (b['begin_ns'] - a['begin_ns']) / 1000000.,
                               'from_publish_begin_monotonic_ns': a['begin_ns'],
                               'to_publish_begin_monotonic_ns': b['begin_ns'],
                               'to_callback_entry_ros_ns': owner['entry_ros_ns'],
                               'to_callback_wall_ms': (owner['end_ns'] - owner['begin_ns']) / 1000000.,
                               'to_entry_to_publish_begin_ms': (b['begin_ns'] - owner['begin_ns']) / 1000000.})
    modules = {}
    for row in rows:
        modules.setdefault(row['name'], []).append((row['end_ns'] - row['begin_ns']) / 1000000.)
    return {'path': os.path.abspath(path), 'sha256': file_sha256(path), 'row_count': len(rows),
            'window_by_callback_entry_ros_ns': {'start_inclusive': start_ros_ns, 'end_exclusive': end_ros_ns},
            'selected_callback_count': len(callbacks), 'selected_publish_count': len(publishes),
            'full_callback_wall_ms': distribution((r['end_ns'] - r['begin_ns']) / 1000000. for r in callbacks),
            'full_callback_thread_cpu_ms': distribution((r['cpu_end_ns'] - r['cpu_begin_ns']) / 1000000. for r in callbacks),
            'full_callback_over_33p333ms': sum(r['end_ns'] - r['begin_ns'] > 100000000 / 3. for r in callbacks),
            'release_lateness_ms': distribution(release), 'expired_to_dispatch_ms': distribution(queue),
            'dispatch_to_entry_ms': distribution(wrapping), 'release_queue_combined_ms': distribution(combined),
            'timer_invalid_count': timer_invalid,
            'command_publish_call_begin_intervals': _intervals(publishes, 'begin_ns'),
            'command_publish_call_interval_peaks': sorted(interval_peaks, key=lambda r: r['interval_ms'], reverse=True)[:5],
            'command_publish_intervals_over_33p333ms': sum(r['interval_ms'] > 1000./30. for r in interval_peaks),
            'command_publish_intervals_over_36p667ms': sum(r['interval_ms'] > 1100./30. for r in interval_peaks),
            'command_publish_call_duration_ms': distribution((r['end_ns'] - r['begin_ns']) / 1000000. for r in publishes),
            'module_wall_ms_whole_csv': {k: distribution(v) for k, v in modules.items()},
            'interpretation': 'callback/publish window uses callback entry stamps but publish intervals use monotonic call starts; instrumented publish boundary is not CAN transmission/actuator execution; whole-CSV nested modules must not be summed; absent expired means release/queue unknown'}
