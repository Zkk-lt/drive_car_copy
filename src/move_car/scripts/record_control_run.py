#!/usr/bin/env python2
"""ROS Melodic passive subscriber; no chassis publishers or online configuration writes."""
from __future__ import print_function
import hashlib
import json
import os
import shutil
import subprocess

from control_research.config import load_config, validate_run_id
from control_research.recording import AsyncJsonlWriter, event_record, monotonic_ns
from control_research.collection_contract import collection_contract

def dump(path, obj):
    with open(path, 'w') as f:
        json.dump(obj, f, indent=2, sort_keys=True, allow_nan=False)

def git(root, args):
    try:
        return subprocess.check_output(['git','-C',root]+args, stderr=subprocess.STDOUT).decode('utf8').strip()
    except Exception as e:
        return 'unavailable: '+str(e)

def main():
    import rospy
    from geometry_msgs.msg import Twist, PoseArray
    from sensor_msgs.msg import Imu, PointCloud2
    from nav_msgs.msg import Odometry, Path
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from move_car.msg import car_parameter, ReferenceInfo, ControlResearchTelemetry, AdaptiveControlTelemetry, ResearchValidationTelemetry
    rospy.init_node('control_run_recorder')
    run_id=validate_run_id(rospy.get_param('/control_research/run_id'))
    root=os.path.abspath(rospy.get_param('~workspace_root'))
    config_root=rospy.get_param('~config_dir')
    terrain=rospy.get_param('~terrain','unknown')
    config=load_config(config_root, terrain)
    destination=os.path.join(os.path.abspath(rospy.get_param('~output_dir')),run_id)
    # Atomic refusal to overwrite an existing experiment, even after a crash.
    os.makedirs(destination)
    config_dest=os.path.join(destination,'config');os.makedirs(config_dest)
    for rel in config['file_hashes']:
        target=os.path.join(config_dest,rel)
        if not os.path.isdir(os.path.dirname(target)):os.makedirs(os.path.dirname(target))
        shutil.copyfile(os.path.join(config_root,rel),target)
    calibration_file=os.path.join(config_root,'sensor_calibration.yaml')
    if os.path.isfile(calibration_file):
        shutil.copyfile(calibration_file,os.path.join(config_dest,'sensor_calibration.yaml'))
    namespaces=['/publish_speed_turn','/trajectory_reconstructor','/ahrs_driver','/control_research','/research_external_shadow']
    parameters={n:rospy.get_param(n,{}) for n in namespaces}
    encoded=json.dumps(parameters,sort_keys=True).encode('utf8')
    source_hashes={};binary_hashes={}
    sources=['src/move_car/src/control_pub.cpp','src/move_car/src/trajectory_reconstructor.cpp',
             'src/move_car/src/research_shadow_node.cpp','src/move_car/scripts/record_control_run.py',
             'src/move_car/launch/movecar_improved.launch','src/move_car/launch/trajectory_reconstructor.launch']
    import glob
    sources += [os.path.relpath(p,root) for p in glob.glob(os.path.join(root,'src/move_car/include/control/*.h'))]
    sources += [os.path.relpath(p,root) for p in glob.glob(os.path.join(root,'src/move_car/scripts/control_research/*.py'))]
    for rel in sources:
        path=os.path.join(root,rel)
        if os.path.isfile(path):
            with open(path,'rb') as f:source_hashes[rel]=hashlib.sha256(f.read()).hexdigest()
            snapshot=os.path.join(destination,'source',rel)
            if not os.path.isdir(os.path.dirname(snapshot)):os.makedirs(os.path.dirname(snapshot))
            shutil.copyfile(path,snapshot)
    for name in ('control_pub','trajectory_reconstructor','research_shadow_node'):
        path=os.path.join(root,'devel','lib','move_car',name)
        if os.path.isfile(path):
            digest=hashlib.sha256()
            with open(path,'rb') as f:
                for block in iter(lambda:f.read(1024*1024),b''):digest.update(block)
            binary_hashes[name]=digest.hexdigest()
    # P4-1 experiment metadata only; subscribers/queue/control chain unchanged.
    formal_config_hashes={}
    for rel in rospy.get_param('~formal_config_paths',[]):
        path=os.path.realpath(os.path.join(root,rel))
        if not path.startswith(os.path.realpath(root)+os.sep) or not os.path.isfile(path):
            raise ValueError('formal_config_paths must name existing workspace files')
        with open(path,'rb') as f:formal_config_hashes[os.path.relpath(path,root)]=hashlib.sha256(f.read()).hexdigest()
    direction=rospy.get_param('~direction','unspecified')
    if direction not in ('forward','reverse','mixed','unspecified'):raise ValueError('invalid direction')
    repetition=rospy.get_param('~repetition',1)
    if isinstance(repetition,bool) or not isinstance(repetition,int) or repetition<1:raise ValueError('invalid repetition')
    manifest={'schema_version':1,'run_id':run_id,'terrain_label':terrain,
              'collection_contract':collection_contract(),
              'route':rospy.get_param('~route','unspecified'),
              'experiment_batch_id':validate_run_id(rospy.get_param('~experiment_batch_id',run_id)),
              'direction':direction,'repetition':repetition,
              'experiment_phase':rospy.get_param('~experiment_phase','unspecified'),
              'matching_conditions_id':rospy.get_param('~matching_conditions_id','unspecified'),
              'formal_config_hashes':formal_config_hashes,
              'formal_config_version':rospy.get_param('~formal_config_version','unverified'),
              'continuous_session_id':validate_run_id(rospy.get_param('~continuous_session_id',run_id)),
              'data_origin':rospy.get_param('~data_origin','unverified'),
              'vehicle_id':rospy.get_param('~vehicle_id','unspecified'),
              'actual_control_mode':'legacy_slqr',
              'target_speed_mps':rospy.get_param('~target_speed_mps',0.0),
              'conditions':rospy.get_param('~conditions','unspecified'),
              'git_revision':git(root,['rev-parse','HEAD']),
              'git_source_status':git(root,['status','--short','--','src']),
              'source_hashes':source_hashes,'built_binary_hashes':binary_hashes,
              'running_binary_identity_verified':False,'config':config,'ros_parameters_start':parameters,
              'ros_parameters_sha256':hashlib.sha256(encoded).hexdigest(),
              'use_sim_time':rospy.get_param('/use_sim_time',False),
              'state':'recording','hardware_calibration_verified':False,
              'time_sync_verified':False,'ground_speed_independence_verified':False,
              'terrain_estimate':{'label':'unknown','valid':False,'confidence':0.,'reason':'see_adaptive_events'},
              'external_shadow_parameters':parameters['/research_external_shadow'],
              'adaptive_runtime_snapshot':parameters['/publish_speed_turn'].get('research_runtime',{}),
              'adaptive_shadow_enabled':parameters['/publish_speed_turn'].get('enable_adaptive_shadow',False)}
    with open(os.path.join(destination,'source.diff'),'wb') as f:
        f.write(git(root,['diff','--','src']).encode('utf8'))
    writer=AsyncJsonlWriter(os.path.join(destination,'events.jsonl'),capacity=1024)
    # Use actual configured topic names when available; changes after startup require a new run.
    c=parameters['/publish_speed_turn']
    topics=[(c.get('car_topic','/car_message'),car_parameter),
            (c.get('carvel_topic','/carvel'),Twist),(c.get('imu_topic','/imu'),Imu),
            (c.get('ned_odom_topic','/NED_odometry'),Odometry),
            (c.get('system_speed_topic','/system_speed'),Twist),
            (c.get('cmd_topic','/cmd_vel'),Twist),
            (c.get('reference_topic','/planner/reference_path'),Path),
            (c.get('raw_topic','/planner/raw_path'),Path),
            (c.get('reference_status_topic','/planner/reference_status'),String),
            (c.get('tracked_topic','/pointpillars/tracked_objects'),PoseArray),
            (c.get('obstacle_poses_topic','/pointpillars/poses'),PoseArray),
            ('/control_research/reference',ReferenceInfo),
            ('/control_research/controller',ControlResearchTelemetry),
            ('/control_research/adaptive',AdaptiveControlTelemetry),
            ('/control_research/validation',ResearchValidationTelemetry),
            ('/control_research/shadow_health',String),('/control_research/request_mode',String),('/control_research/request_terrain',String),
            ('/tf',TFMessage),('/tf_static',TFMessage)]
    manifest['source_contracts']={
        'car_message':{'speed_unit':'mm/s','steering_unit':'rad','speed_scale_to_mps':0.001,
                       'source':'CAN rear-wheel reported speed; aggregation/calibration unverified'},
        'carvel':{'speed_unit':'mm/s','angular_z_unit':'steering rad','independent_of_car_message':False},
        'NED_odometry':{'velocity_unit':'m/s assumed by consumers; protocol calibration unverified',
                        'axes':'North East Down per driver; frame IDs absent; pose quaternion not filled'},
        'system_speed':{'velocity_unit':'m/s assumed','axes':'body per driver; no Header',
                        'independent_of_NED':False},
        'imu':{'gyro_unit':'rad/s assumed by consumers','acceleration_unit':'m/s^2 assumed',
               'axes':'device_type transform and installation must be verified'},
        'cmd_vel':{'linear_x_unit':'m/s','angular_z_unit':'steering rad; NOT yaw rate'},
        'timestamps':'header may be driver receipt, not acquisition; receipt_monotonic_ns is CLOCK_MONOTONIC'}
    manifest['record_clouds']=rospy.get_param('~record_clouds',False)
    if manifest['record_clouds']:
        topics.extend([(c.get('filtered_cloud_topic','/filtered_cloud'),PointCloud2),
                       (c.get('cluster_cloud_topic','/cluster_cloud'),PointCloud2)])
    manifest['topics']=[{'name':name,'type':cls._type} for name,cls in topics if name]
    manifest_path=os.path.join(destination,'manifest.json');dump(manifest_path,manifest)
    def callback(msg, topic):
        receipt=rospy.Time.now().to_nsec()
        # WallTime has no rospy wrapper; time.time is wall receipt only, never duration timing.
        import time
        wall_ns=int(time.time()*1000000000);steady=monotonic_ns()
        writer.submit(lambda:event_record(msg,topic,run_id,receipt,wall_ns,steady))
    subscribers=[rospy.Subscriber(topic,cls,callback,callback_args=topic,
                   queue_size=100,buff_size=8*1024*1024) for topic,cls in topics if topic]
    stopped=[False]
    def finish():
        if stopped[0]:return
        stopped[0]=True
        for sub in subscribers:sub.unregister()
        manifest['writer']=writer.close()
        manifest['state']='failed' if writer.error else 'closed'
        manifest['ros_parameters_end']={n:rospy.get_param(n,{}) for n in namespaces} if not rospy.is_shutdown() else None
        dump(manifest_path,manifest)
    rospy.on_shutdown(finish)
    def health(_event):
        if writer.error:
            rospy.logerr('Recorder failed: %s',writer.error)
            rospy.signal_shutdown('recorder disk/serialization failure')
        elif writer.dropped:
            rospy.logwarn_throttle(10,'Recorder queue dropped %d events',writer.dropped)
    health_timer=rospy.Timer(rospy.Duration(1),health)
    rospy.loginfo('Passive recording run_id=%s directory=%s',run_id,destination)
    try:rospy.spin()
    finally:finish()

if __name__=='__main__':main()
