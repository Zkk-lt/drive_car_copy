#!/usr/bin/env python2
"""P4-2 read-only ROS timing monitor and Python 2/3 offline summarizer.

Never starts controllers, changes scheduling, or publishes any ROS message.
ROS collection uses the existing Melodic Python 2 environment. Offline commands
also work under Python 3 without rospy or generated message packages.
"""
from __future__ import print_function
import argparse
import json
import os
import threading
import time

from control_research.config import validate_run_id
from control_research.recording import AsyncJsonlWriter, monotonic_ns
from control_research.timing_monitor import (file_sha256, proc_sample, summarize_events,
                                           summarize_probe, telemetry_event)


def write_new(path, value):
    # O_EXCL is available in Python 2; never silently overwrite an experiment.
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    with os.fdopen(fd, 'w') as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write('\n')


def collect(args):
    if not .2 <= args.resource_interval <= 60.:
        raise ValueError('resource interval must be in [0.2,60] seconds')
    if not 0. <= args.duration <= 86400.:
        raise ValueError('duration must be in [0,86400] seconds; zero means until manual stop')
    if len(args.pid) > 32 or any(pid <= 0 for pid in args.pid):
        raise ValueError('at most 32 positive explicitly selected PIDs')
    if len(set((args.controller_topic, args.shadow_topic, args.command_topic))) != 3:
        raise ValueError('timing topics must be distinct')
    run_id = validate_run_id(args.run_id); session_id = validate_run_id(args.session_id)
    # Only collection imports ROS; self-test/offline analysis have no ROS import.
    import rospy
    from geometry_msgs.msg import Twist
    from move_car.msg import ControlResearchTelemetry, AdaptiveControlTelemetry
    rospy.init_node('passive_control_timing_monitor', anonymous=True)
    os.makedirs(args.output)
    event_path = os.path.join(args.output, 'timing_events.jsonl')
    writer = AsyncJsonlWriter(event_path, capacity=2048)
    stop = threading.Event()
    from control_research import timing_monitor
    import inspect
    manifest = {'schema_version': 1, 'run_id': run_id, 'session_id': session_id,
                'terrain_label': args.terrain_label, 'label_source': 'operator CLI; not inferred from measurements',
                'data_origin': args.data_origin,
                'state': 'recording', 'monitor_only': True, 'controller_mode_expected': 'legacy_slqr',
                'git_revision': args.git_revision, 'config_sha256': args.config_sha256,
                'pids': args.pid, 'resource_interval_s': args.resource_interval,
                'use_sim_time': rospy.get_param('/use_sim_time', False),
                'script_sha256': file_sha256(__file__),
                'module_sha256': file_sha256(inspect.getsourcefile(timing_monitor)),
                'proc_clock_ticks_per_second': os.sysconf('SC_CLK_TCK'),
                'unknown': ['hardware_sample_times', 'CAN_send_execute_time',
                            'actual_command_publish_time', 'timer_expected_expired_dispatch_without_probe'],
                'subscriber_queue_size': 200, 'transport_drops': 'unknown',
                'topics': {'controller': args.controller_topic, 'shadow': args.shadow_topic,
                           'command': args.command_topic}}
    write_new(os.path.join(args.output, 'manifest_start.json'), manifest)
    def callback(msg, context):
        # Capture receipt BEFORE waiting for asynchronous serialization/disk IO.
        ros_ns = rospy.Time.now().to_nsec(); steady_ns = monotonic_ns()
        wall_ns = int(time.time() * 1000000000)
        kind, topic = context
        event = telemetry_event(kind, msg, topic, ros_ns, steady_ns, wall_ns)
        event.update({'run_id': run_id, 'session_id': session_id})
        writer.submit(event)
    subscribers = [rospy.Subscriber(topic, cls, callback, callback_args=(kind, topic),
                                    queue_size=200, buff_size=1024 * 1024)
                   for kind, topic, cls in [
                       ('controller', args.controller_topic, ControlResearchTelemetry),
                       ('shadow', args.shadow_topic, AdaptiveControlTelemetry),
                       ('command', args.command_topic, Twist)]]
    # One low-rate monitor thread. No /proc reads or file writes in callbacks.
    def resources():
        while not stop.is_set() and not rospy.is_shutdown():
            sample = proc_sample(args.pid)
            sample.update({'run_id': run_id, 'session_id': session_id,
                           'receipt_ros_ns': rospy.Time.now().to_nsec()})
            writer.submit(sample)
            stop.wait(args.resource_interval)
    monitor = threading.Thread(target=resources); monitor.daemon = True; monitor.start()
    start = monotonic_ns()
    try:
        while not rospy.is_shutdown() and (not args.duration or
                                          monotonic_ns() - start < args.duration * 1e9):
            if writer.error:
                raise RuntimeError('timing writer failed: ' + writer.error)
            stop.wait(.2)
    finally:
        stop.set(); monitor.join()
        for subscriber in subscribers:
            subscriber.unregister()
        manifest['writer'] = writer.close()
        manifest['state'] = 'failed' if writer.error else 'closed'
        manifest['event_file_sha256'] = file_sha256(event_path)
        write_new(os.path.join(args.output, 'manifest_closed.json'), manifest)
    return {'directory': args.output, 'state': manifest['state'], 'writer': manifest['writer']}


def analyze(args):
    result = {'schema_version': 1}
    if args.events:
        with open(args.events) as stream:
            events = [json.loads(line) for line in stream if line.strip()]
        result['events'] = summarize_events(events)
        result['events_source'] = {'path': os.path.abspath(args.events), 'sha256': file_sha256(args.events)}
        if args.manifest:
            with open(args.manifest) as stream:
                manifest = json.load(stream)
            writer = manifest.get('writer', {})
            checked = (manifest.get('event_file_sha256') == result['events_source']['sha256'])
            result['collection_integrity'] = {
                'event_sha256_matches_closed_manifest': checked,
                'state': manifest.get('state'), 'writer': writer,
                'collector_complete': bool(checked and manifest.get('state') == 'closed' and
                                           writer.get('accepted') == writer.get('written') == len(events) and
                                           writer.get('queue_dropped') == 0 and
                                           writer.get('unwritten') == 0 and
                                           writer.get('writer_error') is None),
                'ROS_transport_completeness': 'unknown; local writer counters do not prove absence of subscriber/network loss'}
        else:
            result['collection_integrity'] = {'collector_complete': None,
                                              'reason': 'closed manifest not supplied; event count alone cannot prove completeness'}
    if args.probe:
        result['probe'] = summarize_probe(args.probe, args.probe_start_ros_ns, args.probe_end_ros_ns)
    if not args.events and not args.probe:
        raise ValueError('analyze requires --events and/or --probe')
    if args.output:
        write_new(args.output, result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command')
    record = commands.add_parser('collect', help='passive subscriptions only; manual controlled vehicle start remains separate')
    record.add_argument('--output', required=True)
    record.add_argument('--run-id', required=True); record.add_argument('--session-id', required=True)
    record.add_argument('--terrain-label', choices=('smooth', 'unknown'), default='unknown')
    record.add_argument('--data-origin', choices=('real_vehicle', 'synthetic_tool_test', 'unverified'), default='unverified')
    record.add_argument('--pid', type=int, action='append', default=[], help='PID explicitly selected for read-only /proc monitoring')
    record.add_argument('--resource-interval', type=float, default=.5)
    record.add_argument('--duration', type=float, default=0.)
    record.add_argument('--controller-topic', default='/control_research/controller')
    record.add_argument('--shadow-topic', default='/control_research/adaptive')
    record.add_argument('--command-topic', default='/cmd_vel')
    record.add_argument('--git-revision', default='unverified')
    record.add_argument('--config-sha256', default='unverified')
    offline = commands.add_parser('analyze', help='no ROS required; source files are read-only')
    offline.add_argument('--events'); offline.add_argument('--manifest'); offline.add_argument('--probe'); offline.add_argument('--output')
    offline.add_argument('--probe-start-ros-ns', type=int)
    offline.add_argument('--probe-end-ros-ns', type=int)
    resources = commands.add_parser('resources', help='single passive /proc sample, no ROS required')
    resources.add_argument('--pid', type=int, action='append', default=[])
    args = parser.parse_args()
    try:
        if args.command == 'collect':
            result = collect(args)
        elif args.command == 'analyze':
            result = analyze(args)
        elif args.command == 'resources':
            if len(args.pid) > 32 or any(pid <= 0 for pid in args.pid):
                raise ValueError('at most 32 positive PIDs')
            result = proc_sample(args.pid)
        else:
            parser.error('choose collect, analyze or resources')
        print(json.dumps(result, sort_keys=True, allow_nan=False, indent=2))
    except (ValueError, OSError, IOError, RuntimeError) as exc:
        parser.exit(1, str(exc) + '\n')


if __name__ == '__main__':
    main()
