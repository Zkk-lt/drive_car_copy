#!/usr/bin/env python2
"""Bounded passive capture with a collector-local run ID.

No ROS parameter writes, command publishers, controller/driver starts or stops.
Reuses the P4-2 raw event schema and asynchronous writer. No resampling/fusion.
"""
from __future__ import print_function
import argparse
import hashlib
import json
import math
import os
import threading
import time

from control_research.collection_contract import collection_contract
from control_research.config import TERRAINS, validate_run_id
from control_research.recording import AsyncJsonlWriter, event_record, monotonic_ns


def digest(path):
    h = hashlib.sha256()
    with open(path, 'rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def write_json(path, value):
    with open(path, 'w') as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write('\n')


def terrain_metadata(label):
    """An operator label is metadata, never a friction/model declaration."""
    if label is not None and label not in TERRAINS:
        raise ValueError('unsupported terrain label')
    return dict(terrain_label=label if label is not None else 'unknown',
                terrain_label_source='operator_cli' if label is not None else 'unspecified_default_unknown',
                terrain_label_infers_friction=False)


def experiment_metadata(phase, route=None, target_speed_mps=None,
                        direction=None, repetition=1):
    """Validate operator descriptions without changing a controller setting."""
    if phase not in ('stationary', 's8_tracking'):
        raise ValueError('unsupported experiment phase')
    if route is not None and (not isinstance(route, str) or not route.strip()):
        raise ValueError('route must be a nonempty operator description')
    if direction is not None and direction not in ('forward', 'reverse', 'mixed'):
        raise ValueError('unsupported direction')
    if isinstance(repetition, bool) or not isinstance(repetition, int) or repetition < 1:
        raise ValueError('repetition must be a positive integer')
    if target_speed_mps is not None:
        if isinstance(target_speed_mps, bool):
            raise ValueError('target speed must be a finite nonnegative number')
        try:
            speed = float(target_speed_mps)
        except (ValueError, TypeError):
            raise ValueError('target speed must be a finite nonnegative number')
        if math.isnan(speed) or math.isinf(speed) or speed < 0.:
            raise ValueError('target speed must be a finite nonnegative number')
        if phase == 'stationary' and speed != 0.:
            raise ValueError('stationary recording cannot declare a nonzero target speed')
    else:
        speed = 0. if phase == 'stationary' else None
    return dict(route=route if route is not None else
                ('stationary_pad' if phase == 'stationary' else 'operator_route_not_yet_documented'),
                target_speed_mps=speed, direction=direction or 'unspecified',
                repetition=repetition,
                experiment_metadata_role='operator_requested_recording_not_controller_authority',
                target_speed_is_controller_parameter=False)


def p6_event_topics(enabled):
    """Extra passive topics only; physical /cmd_vel remains a separate raw event."""
    if not enabled:
        return []
    return [('/control_research/p5/published', 'move_car/P5PublishedCommand'),
            ('/control_research/p5/diagnostics', 'move_car/P5ControlDiagnostics'),
            ('/control_research/p5/request', 'move_car/P5ControlRequest'),
            ('/control_research/p5/candidate', 'move_car/P5ControlCandidate'),
            ('/control_research/p5_test/p6/request', 'move_car/P6ControlRequest'),
            ('/control_research/p5_test/p6/candidate', 'move_car/P6ControlCandidate')]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--session-id', required=True)
    parser.add_argument('--freeze', required=True)
    parser.add_argument('--duration', type=float, default=35.)
    parser.add_argument('--terrain-label', choices=TERRAINS, default=None,
                        help='explicit operator label; omission records unknown, never silently smooth')
    parser.add_argument('--phase', choices=('stationary', 's8_tracking'), default='stationary',
                        help='s8_tracking records operator-started S8 motion; never starts a controller')
    parser.add_argument('--route', default=None, help='operator route description; metadata only')
    parser.add_argument('--target-speed-mps', type=float, default=None,
                        help='requested experiment target, not a controller parameter write')
    parser.add_argument('--direction', choices=('forward', 'reverse', 'mixed'), default=None)
    parser.add_argument('--repetition', type=int, default=1)
    parser.add_argument('--p6-events', action='store_true',
                        help='also record existing P5/P6 messages; does not start a worker or grant control')
    parser.add_argument('--data-origin', choices=('real_vehicle', 'synthetic_tool_test'), default='real_vehicle',
                        help='explicit test-fixture provenance; real_vehicle is only an operator declaration')
    parser.add_argument('--exclude-ned', action='store_true',
                        help='operator excludes unusable NED/system_speed; never substitutes wheel-derived ground speed')
    parser.add_argument('--until-control-stops', action='store_true',
                        help='record until both S8 and reconstructor have been observed then absent in two graph polls')
    args = parser.parse_args()
    try:
        experiment = experiment_metadata(args.phase, args.route, args.target_speed_mps,
                                         args.direction, args.repetition)
    except ValueError as exc:
        parser.error(str(exc))
    maximum_duration = 120. if args.phase == 'stationary' else 1800.
    if args.until_control_stops and args.phase != 's8_tracking':
        parser.error('--until-control-stops requires --phase s8_tracking')
    indefinite = args.until_control_stops and args.duration == 0.
    if not indefinite and not 5. <= args.duration <= maximum_duration:
        parser.error('duration must be 5..%g seconds for this phase' % maximum_duration)
    rid = validate_run_id(args.run_id); sid = validate_run_id(args.session_id)
    if not os.path.isfile(args.freeze):
        parser.error('existing software freeze required')
    import rospy
    import rosgraph
    from geometry_msgs.msg import Twist, PoseArray
    from nav_msgs.msg import Odometry, Path
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from move_car.msg import (car_parameter, ReferenceInfo, ControlResearchTelemetry,
                              AdaptiveControlTelemetry, ResearchValidationTelemetry)
    node_name = 'p42_stationary_raw_recorder' if args.phase == 'stationary' else 'p42_s8_raw_recorder'
    rospy.init_node(node_name, anonymous=True, disable_signals=False)
    master = rosgraph.Master(rospy.get_name())
    folder = os.path.join(os.path.abspath(args.output_dir), rid)
    os.makedirs(folder)  # Never overwrite or reuse an earlier capture.
    namespaces = ['/carTop', '/ahrs_driver', '/publish_speed_turn', '/trajectory_reconstructor',
                  '/control_research', '/research_external_shadow']
    def snapshot():
        result = {'receipt_wall_ns': int(time.time()*1e9),
                  'receipt_monotonic_ns': monotonic_ns(), 'receipt_ros_ns': rospy.Time.now().to_nsec()}
        try:
            result['system_state'] = master.getSystemState()
            result['published_types'] = master.getPublishedTopics('')
            result['parameters'] = {n: rospy.get_param(n, None) for n in namespaces}
            result['global_run_id'] = rospy.get_param('/control_research/run_id', None)
        except Exception as exc:
            result['error'] = str(exc)
        return result
    start_graph = snapshot()
    if 'error' in start_graph:
        raise RuntimeError('live ROS graph unavailable before collection')
    settings = start_graph['parameters'].get('/publish_speed_turn') or {}
    reconstruction_settings = start_graph['parameters'].get('/trajectory_reconstructor') or {}
    watched_nodes = {'/publish_speed_turn', '/trajectory_reconstructor'}
    def observed_nodes(graph):
        if 'error' in graph or 'system_state' not in graph:
            return None
        return {node for group in graph['system_state'] for _, nodes in group for node in nodes}
    armed = watched_nodes.issubset(observed_nodes(start_graph) or set())
    absent_polls = 0
    command_topic = settings.get('cmd_topic', '/cmd_vel')
    topics = [(settings.get('car_topic', '/car_message'), car_parameter),
              (settings.get('carvel_topic', '/carvel'), Twist),
              (settings.get('ned_odom_topic', '/NED_odometry'), Odometry),
              (settings.get('imu_topic', '/imu'), Imu),
              (settings.get('system_speed_topic', '/system_speed'), Twist),
              (command_topic, Twist),
              (settings.get('reference_topic', '/planner/reference_path'), Path),
              (reconstruction_settings.get('raw_topic', '/planner/raw_path'), Path),
              (reconstruction_settings.get('tracked_topic', '/pointpillars/tracked_objects'), PoseArray),
              ('/planner/reference_status', String), ('/control_research/reference', ReferenceInfo),
              ('/control_research/controller', ControlResearchTelemetry),
              ('/control_research/adaptive', AdaptiveControlTelemetry),
              ('/control_research/validation', ResearchValidationTelemetry),
              ('/control_research/shadow_health', String), ('/tf', TFMessage), ('/tf_static', TFMessage)]
    if args.exclude_ned:
        excluded = {settings.get('ned_odom_topic', '/NED_odometry'),
                    settings.get('system_speed_topic', '/system_speed')}
        topics = [(n, c) for n, c in topics if n not in excluded]
    extra_topics = p6_event_topics(args.p6_events)
    if extra_topics:
        import roslib.message
        for name, kind in extra_topics:
            cls = roslib.message.get_message_class(kind)
            if cls is None:
                raise RuntimeError('requested passive P6 recording needs generated message class: ' + kind)
            topics.append((name, cls))
    pubs = dict(start_graph['system_state'][0])
    manifest = dict(schema_version=1, run_id=rid, continuous_session_id=sid,
        run_id_scope='collector_local; global ROS parameter not set or changed',
        data_origin=args.data_origin,
        provenance='Live chassis and sensors requested by operator; not authenticated independent measurement',
        experiment_phase=args.phase,
        operator_stationary_request=args.phase == 'stationary', stationary_independently_verified=False,
        ned_excluded_by_operator=args.exclude_ned,
        until_control_stops=args.until_control_stops,
        stop_watch_nodes=sorted(watched_nodes) if args.until_control_stops else [],
        stop_watch_semantics='arm only after both nodes observed together; stop after both absent in two successful 5s graph polls; master errors never count as node exit',
        actual_control_mode='no_controller_observed' if not pubs.get(command_topic) else 'unverified_read_only_graph',
        initial_command_publishers=pubs.get(command_topic, []),
        topics=[dict(name=n, type=c._type) for n, c in topics],
        collection_contract=collection_contract(), state='recording',
        hardware_calibration_verified=False, time_sync_verified=False,
        ground_speed_independence_verified=False, vehicle_id='current_operator_vehicle_unverified_identifier',
        use_sim_time=rospy.get_param('/use_sim_time', False),
        duration_requested_s=args.duration, software_freeze_path=os.path.abspath(args.freeze),
        software_freeze_sha256=digest(args.freeze), collector_source_sha256=digest(__file__),
        ros_parameters_start=start_graph['parameters'], start_graph=start_graph,
        control_commands_generated=False, parameter_writes_performed=False,
        source_contracts=collection_contract()['raw_channels'],
        p6_events_requested=args.p6_events,
        command_history_contract=dict(
            cmd_vel_source_sequence=None,
            cmd_vel_source_sequence_status='unknown_headerless_Twist',
            cmd_vel_collector_receipt_is_publish_time=False,
            recorded_event_order_is_source_command_sequence=False,
            published_command_sequence_source='P5PublishedCommand.sequence only when independently observed and audited',
            published_command_observation_is_physical_execution=False),
        endpoint_times_unknown=['CAN_hardware_sample', 'driver_receive', 'actual_command_publish',
                                'actuator_execution', 'controller_receive'])
    manifest.update(terrain_metadata(args.terrain_label))
    manifest.update(experiment)
    if args.p6_events and settings.get('p7_supervised_test') is True:
        manifest['collection_contract']['controller_authority'] = 'operator_supervised_P7_research; actual owners require P5 published-record audit'
        manifest['requested_algorithm_mode'] = settings.get('p6_algorithm_mode')
        manifest['execution_context'] = 'operator_supervised_unvalidated_vehicle_trial'
        manifest['engineering_mapping_is_physical_calibration'] = False
    write_json(os.path.join(folder, 'manifest.json'), manifest)
    event_file = os.path.join(folder, 'events.jsonl')
    writer = AsyncJsonlWriter(event_file, capacity=2048)
    counts = {}; lock = threading.Lock(); suspected = []
    def callback(msg, topic):
        ros_ns = rospy.Time.now().to_nsec(); wall_ns = int(time.time()*1e9); steady_ns = monotonic_ns()
        # No interpolation: wheel and every command are distinct raw messages.
        writer.submit(lambda: event_record(msg, topic, rid, ros_ns, wall_ns, steady_ns),topic=topic)
        with lock:
            counts[topic] = counts.get(topic, 0)+1
            if args.phase == 'stationary' and topic == command_topic and (abs(msg.linear.x) > .001 or abs(msg.angular.z) > .005):
                suspected.append({'topic': topic, 'receipt_ros_ns': ros_ns, 'reason': 'nonzero_command_during_requested_stationary_capture'})
            if args.phase == 'stationary' and getattr(msg, '_type', '') == 'move_car/car_parameter' and abs(msg.back_wheel_speed) > 5.:
                suspected.append({'topic': topic, 'receipt_ros_ns': ros_ns,
                                  'reason': 'nonzero_reported_wheel_during_requested_stationary_capture'})
    subs = [rospy.Subscriber(n, c, callback, callback_args=n, queue_size=200, buff_size=1024*1024)
            for n, c in topics]
    # Small periodic graph snapshots are taken by the main collector thread,
    # never in sensor callbacks. The collector alone is stopped on suspected motion.
    start = monotonic_ns(); next_report = start; next_health = start; graphs = []; exit_reason = 'duration_completed'
    try:
        while not rospy.is_shutdown() and (indefinite or monotonic_ns()-start < args.duration*1e9):
            if monotonic_ns() >= next_health:
                with lock:
                    observed_counts=dict(counts)
                health=dict(run_id=rid,collector_pid=os.getpid(),state='recording',
                    steady_ns=monotonic_ns(),topic_event_counts=observed_counts,
                    writer=writer.queue_health(),guarantees_transport_delivery=False)
                temporary=os.path.join(folder,'capture_health.json.tmp')
                write_json(temporary,health)
                os.rename(temporary,os.path.join(folder,'capture_health.json'))
                next_health=monotonic_ns()+1000000000
            if writer.error:
                exit_reason = 'writer_error';break
            if suspected:
                exit_reason = 'stationary_request_violated_recording_stopped_no_vehicle_action';break
            if monotonic_ns() >= next_report:
                graph = snapshot(); graphs.append(graph)
                nodes = observed_nodes(graph)
                if args.until_control_stops and nodes is not None:
                    armed = armed or watched_nodes.issubset(nodes)
                    absent_polls = absent_polls+1 if armed and not watched_nodes.intersection(nodes) else 0
                    if absent_polls >= 2:
                        exit_reason = 'both_S8_and_reconstructor_stopped_observed_by_read_only_graph';break
                print('elapsed_s=%.1f counts=%s writer=%s' % ((monotonic_ns()-start)/1e9,
                    json.dumps(counts, sort_keys=True),json.dumps(writer.queue_health(),sort_keys=True)))
                import sys
                sys.stdout.flush();next_report = monotonic_ns()+5*1000000000
            time.sleep(.05)
        if rospy.is_shutdown():
            exit_reason = 'ROS_shutdown_or_manual_interrupt'
    finally:
        for sub in subs:
            sub.unregister()
        manifest['writer'] = writer.close()
        manifest['duration_observed_s'] = (monotonic_ns()-start)/1e9
        manifest['exit_reason'] = exit_reason
        manifest['stop_watch_armed'] = armed
        manifest['stop_watch_absent_polls'] = absent_polls
        manifest['state'] = 'failed' if writer.error else 'closed'
        temporary=os.path.join(folder,'capture_health.json.tmp')
        write_json(temporary,dict(run_id=rid,state=manifest['state'],steady_ns=monotonic_ns(),
            collector_pid=os.getpid(),writer=manifest['writer'],topic_event_counts=dict(counts)))
        os.rename(temporary,os.path.join(folder,'capture_health.json'))
        manifest['topic_event_counts'] = counts
        manifest['command_history_contract']['p5_published_events_observed'] = counts.get(
            '/control_research/p5/published', 0)
        manifest['command_history_contract']['p5_actual_history_validated'] = False
        manifest['stationary_warning_events'] = suspected
        manifest['end_graph'] = snapshot()
        manifest['ros_parameters_end'] = manifest['end_graph'].get('parameters')
        manifest['events_sha256'] = digest(event_file)
        manifest['missing_topics'] = [n for n, _ in topics if counts.get(n, 0) == 0]
        write_json(os.path.join(folder, 'graph_samples.json'), graphs)
        write_json(os.path.join(folder, 'manifest.json'), manifest)
    print(json.dumps({'run_directory': folder, 'state': manifest['state'], 'exit_reason': exit_reason,
                      'duration_s': manifest['duration_observed_s'], 'writer': manifest['writer'],
                      'missing_topics': manifest['missing_topics']}, sort_keys=True))
    normal_exit = exit_reason in ('duration_completed', 'both_S8_and_reconstructor_stopped_observed_by_read_only_graph')
    # A closed stream with local drops is evidence of an incomplete recording,
    # never a successful calibration capture. Keep its original bytes/counts.
    return 0 if manifest['state'] == 'closed' and normal_exit and not manifest['writer']['queue_dropped'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
