"""Passive-recording time and raw-channel semantics; Python 2/3, no ROS.

This describes available evidence. It does not turn driver conventions into
physical calibration, nor a subscriber receipt into the controller's receipt.
"""


def collection_contract():
    return {
        'schema_version': 1,
        'controller_authority': 'S8_only_NMPC_shadow',
        'authority_is_experiment_requirement_not_hardware_certificate': True,
        'architecture': {
            'feedback_scope': 'relative_reference_and_short_horizon_ego_motion',
            'NED_required_for_local_control': False,
            'global_pose_required_for_local_control': False,
            'independent_ground_speed_role': 'optional_true_slip_and_independent_physical_calibration',
            'current_external_shadow_NED_pose_dependency': 'adapter_implementation_limitation_not_method_requirement',
        },
        'timestamp_fields': {
            'source_stamp_ns': 'message Header stamp when present; acquisition semantics unverified',
            'receipt_ros_ns': 'recorder callback entry ROS clock; not transport or S8 receive time',
            'receipt_wall_ns': 'recorder callback wall clock; not for duration measurement',
            'receipt_monotonic_ns': 'recorder callback CLOCK_MONOTONIC; same-host duration clock',
            'earliest_available_recorded_receive': 'recorder_callback_entry',
            'driver_receive_time': None,
            'hardware_acquisition_time': None,
            'controller_receive_time': None,
            'ros_clock_to_monotonic_mapping_verified': False,
        },
        'command_time': {
            'controller_telemetry_header': 'control_callback_entry_before_compute',
            'cmd_vel_header': None,
            'actual_publish_time': None,
            'collector_receipt_is_publish_time': False,
            'actuator_receive_time': None,
        },
        'raw_channels': {
            'tracked_objects': {
                'fields': ['header', 'poses'],
                'role': 'front_vehicle_relative_observations_for_reconstruction',
                'frame_and_tracking_identity_verified': False,
                'global_position_required': False,
            },
            'relative_reference': {
                'fields': ['header', 'poses', 'reference_status'],
                'role': 'local_reconstructed_reference_not_independent_ground_truth',
                'global_position_required': False,
            },
            'car_message': {
                'fields': ['back_wheel_speed', 'turn_angle'],
                'driver_declared_units': ['mm/s', 'rad'],
                'physical_units_calibrated': False,
                'wheel_aggregation_sign_scale_verified': False,
                'hardware_acquisition_time_available': False,
                'driver_cache_republication_may_occur': True,
            },
            'carvel': {
                'fields': ['linear.x', 'angular.z'],
                'driver_declared_units': ['mm/s', 'steering_rad'],
                'independent_of_car_message': False,
                'angular_z_is_yaw_rate': False,
                'hardware_acquisition_time_available': False,
            },
            'NED_odometry': {
                'role': 'optional_auxiliary_when_valid_not_required_local_feedback',
                'fields': ['pose', 'twist', 'header.frame_id', 'child_frame_id'],
                'driver_declared_velocity_axes': 'world_NED',
                'actual_velocity_frame_verified': False,
                'ground_speed_independence_verified': False,
                'independent_body_heading_verified': False,
                'sensor_to_rear_axle_transform_verified': False,
            },
            'imu': {
                'fields': ['linear_acceleration.xyz', 'angular_velocity.xyz', 'orientation'],
                'driver_declared_units': ['m/s^2', 'rad/s', 'quaternion'],
                'installation_axes_units_and_zero_verified': False,
                'header_is_hardware_acquisition_time_verified': False,
            },
        },
        'causal_snapshot': {
            'recorder_receipt_before_control_stamp_proves_S8_availability': False,
            'requires': ['verified_clock_mapping', 'controller_local_receive_or_sample_id_evidence'],
            'external_shadow_raw': 'own-receipt cutoff only; preserves original source/receipt',
            'external_shadow_pose': 'historical bracket interpolation may use later observations',
            'strict_online_controller_causality_certified': False,
        },
        'physical_calibration_verified': False,
        'production_configuration_changed': False,
    }


def collection_preflight(snapshot, expected_run_id, s8_node='/publish_speed_turn'):
    """Read-only graph/parameter eligibility, never a sensor health certificate."""
    parameters = snapshot.get('parameters', {})
    control = parameters.get(s8_node) or {}
    reconstruct = parameters.get('/trajectory_reconstructor') or {}
    publishers = dict(snapshot.get('publishers', []))
    types = dict(snapshot.get('published_types', []))
    requested = [(control.get('car_topic', '/car_message'), 'move_car/car_parameter'),
                 (control.get('carvel_topic', '/carvel'), 'geometry_msgs/Twist'),
                 (control.get('ned_odom_topic', '/NED_odometry'), 'nav_msgs/Odometry'),
                 (control.get('imu_topic', '/imu'), 'sensor_msgs/Imu'),
                 (control.get('cmd_topic', '/cmd_vel'), 'geometry_msgs/Twist'),
                 (control.get('reference_topic', '/planner/reference_path'), 'nav_msgs/Path'),
                 ('/control_research/reference', 'move_car/ReferenceInfo'),
                 ('/control_research/controller', 'move_car/ControlResearchTelemetry'),
                 ('/control_research/adaptive', 'move_car/AdaptiveControlTelemetry'),
                 ('/control_research/validation', 'move_car/ResearchValidationTelemetry')]
    topics = []; missing = []
    optional_ned = control.get('ned_odom_topic', '/NED_odometry')
    for topic, message_type in requested:
        actual = publishers.get(topic, [])
        ready = bool(actual) and types.get(topic) == message_type
        topics.append({'topic': topic, 'publishers': actual, 'type': types.get(topic),
                       'expected_type': message_type, 'present_with_expected_type': ready,
                       'required_for_research_telemetry_snapshot': topic != optional_ned})
        if not ready and topic != optional_ned:
            missing.append('missing_or_wrong_type:' + topic)
    if control.get('enable_research_telemetry') is not True:
        missing.append('S8_research_telemetry_not_enabled_default_false')
    if reconstruct.get('enable_research_telemetry') is not True:
        missing.append('reference_metadata_not_enabled_default_false')
    if snapshot.get('run_id') != expected_run_id:
        missing.append('global_run_id_mismatch_or_missing')
    if snapshot.get('use_sim_time') is not False:
        missing.append('non_simulated_ROS_clock_not_confirmed')
    command_topic = control.get('cmd_topic', '/cmd_vel')
    if publishers.get(command_topic) != [s8_node]:
        missing.append('unique_expected_S8_command_publisher_not_confirmed')
    # The extended telemetry snapshot is not the S8/local-feedback input
    # contract. Global run metadata and optional ground data are not needed
    # simply to observe the local raw/relative channel graph.
    local_requested = [(control.get('car_topic', '/car_message'), 'move_car/car_parameter'),
                       (control.get('imu_topic', '/imu'), 'sensor_msgs/Imu'),
                       (command_topic, 'geometry_msgs/Twist'),
                       (control.get('reference_topic', '/planner/reference_path'), 'nav_msgs/Path'),
                       (control.get('reference_status_topic', '/planner/reference_status'), 'std_msgs/String'),
                       (reconstruct.get('tracked_topic', '/pointpillars/tracked_objects'), 'geometry_msgs/PoseArray')]
    local_missing = ['missing_or_wrong_type:' + topic for topic, cls in local_requested
                     if not publishers.get(topic) or types.get(topic) != cls]
    if publishers.get(command_topic) != [s8_node]:
        local_missing.append('unique_expected_S8_command_publisher_not_confirmed')
    return {'schema_version': 1, 'scope': 'graph_and_parameter_snapshot_only',
            'expected_run_id': expected_run_id, 'observed_run_id': snapshot.get('run_id'),
            'topics': topics, 'missing': missing, 'full_chain_topics_ready': not missing,
            'full_chain_scope': 'extended_research_telemetry_snapshot_not_S8_operational_readiness; NED optional',
            'local_tracking_topics_ready': not local_missing,
            'local_missing': local_missing,
            'local_required_topics': [topic for topic, cls in local_requested],
            'optional_ground_input': {'topic': optional_ned, 'publishers': publishers.get(optional_ned, []),
                                      'required_for_local_control': False,
                                      'measurement_validity_verified': False,
                                      'registration_proves_message_traffic': False},
            'sensor_measurement_validity_verified': False,
            'physical_calibration_verified': False, 'controller_receive_causality_verified': False,
            'vehicle_motion_authorized': False, 'parameters_changed': False,
            'remap_limitation': 'Configured topic names may be ROS-remapped. Audit the running graph; provide an explicit snapshot of resolved names if needed.',
            'next_step': 'Do not claim full data coverage when missing. Any diagnostic startup change needs operator review and timing validation; production defaults are retained.'}
