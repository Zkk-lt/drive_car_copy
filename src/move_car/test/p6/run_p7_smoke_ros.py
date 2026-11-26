#!/usr/bin/env python3
"""Short P7 software-contract test on a private ROS master, never real hardware.

The request deliberately has synthetic=false to exercise the P7 receipt-domain
path. Every artifact is labelled synthetic_tool_test: its mapping SHA is a test
constant, not calibration or permission for a real vehicle. No production master
is queried, mutated or used by any child process.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import xml.etree.ElementTree as ET
from xmlrpc.client import ServerProxy
import yaml

P = Path(__file__).resolve().parents[2]
W = P.parents[1]

FIXTURE = r'''#!/usr/bin/env python2
from __future__ import print_function
import json,math,os,signal,sys,threading,time
import rospy
from geometry_msgs.msg import Twist,PoseArray,Pose,PoseStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path
from std_msgs.msg import String
from move_car.msg import car_parameter,P5PublishedCommand,P5ControlDiagnostics,P6ControlRequest,P6ControlCandidate
rospy.init_node('p7_synthetic_tool_fixture',anonymous=False)
raw=dict(data_origin='synthetic_tool_test',real_hardware=False,requests=[],candidates=[],published=[],diagnostics=[],commands=[],phases=[])
lock=threading.RLock();last_command=[0.,0.];plant=[0.,0.,0.,0.,0.]
def encode(v):
 if hasattr(v,'secs') and hasattr(v,'nsecs'):return v.to_sec()
 if hasattr(v,'__slots__'):return dict((k,encode(getattr(v,k))) for k in v.__slots__)
 if isinstance(v,(list,tuple)):return [encode(x) for x in v]
 return v
subs=[]
def watch(key,typ,topic):
 def callback(m):
  item=encode(m);item['receipt_wall']=time.time()
  with lock:raw[key].append(item)
 subs.append(rospy.Subscriber(topic,typ,callback,queue_size=2048))
for key,typ,topic in [('published',P5PublishedCommand,'/control_research/p5/published'),('diagnostics',P5ControlDiagnostics,'/control_research/p5/diagnostics'),('requests',P6ControlRequest,'/control_research/p5_test/p6/request'),('candidates',P6ControlCandidate,'/control_research/p5_test/p6/candidate')]:watch(key,typ,topic)
def command(m):
 with lock:last_command[:]=[m.linear.x,m.angular.z];raw['commands'].append(dict(receipt_wall=time.time(),speed=m.linear.x,steering=m.angular.z))
subs.append(rospy.Subscriber('/cmd_vel',Twist,command,queue_size=2048))
wheel=rospy.Publisher('/car_message',car_parameter,queue_size=8)
imu=rospy.Publisher('/imu',Imu,queue_size=8)
path=rospy.Publisher('/planner/reference_path',Path,queue_size=2)
status=rospy.Publisher('/planner/reference_status',String,queue_size=2)
lead=rospy.Publisher('/tracked_pose_array',PoseArray,queue_size=2)
lease=rospy.Publisher('/control_research/p7/operator_lease',String,queue_size=1,latch=False)
time.sleep(.6);start=time.time();seq=0;killed=False;previous=start
run=sys.argv[2];worker=int(sys.argv[3]);duration=8.6
try:
 for i in range(int(duration*100)):
  t=i/100.;delay=start+t-time.time()
  if delay>0:time.sleep(delay)
  wall=time.time();dt=min(.05,max(0.,wall-previous));previous=wall
  active=1.4<=t<5.2 or 6.2<=t<7.6
  if active and i%3==0:seq+=1;lease.publish(String(run+'|'+str(seq)))
  if t>=6.6 and not killed:
   # This PID was created by the parent on this private master only.
   os.kill(worker,signal.SIGTERM);raw['worker_exit_wall']=wall;killed=True
  with lock:
   speed,delta=last_command
   plant[3]+=dt*(speed-plant[3])/.20;plant[4]+=dt*(delta-plant[4])/.44
   r=plant[3]*math.tan(plant[4])/.60
   plant[0]+=dt*plant[3]*math.cos(plant[2]);plant[1]+=dt*plant[3]*math.sin(plant[2]);plant[2]+=dt*r
   x,y,yaw,v,d=plant
  now=rospy.Time.now()
  im=Imu();im.header.stamp=now;im.header.frame_id='wrong_test_frame' if 3.0<=t<3.6 else 'base_link';im.orientation.w=1.;im.angular_velocity.z=r+.01*math.sin(19.*t);im.linear_acceleration.z=9.80665;imu.publish(im)
  if i%5==0:
   w=car_parameter();w.back_wheel_speed=max(0.,v)*1000.;w.turn_angle=d;w.error_flag=0;wheel.publish(w)
  if i%3==0:
   pa=PoseArray();pa.header.stamp=now;pa.header.frame_id='base_link';target=Pose();target.position.x=3.;target.orientation.w=1.;pa.poses=[target];lead.publish(pa)
  if int((i+1)*int(sys.argv[5])/100)!=int(i*int(sys.argv[5])/100):
   p=Path();p.header.stamp=now;p.header.frame_id='base_link'
   for j in range(61):
    s=.05*j;wy=.015+.01*math.sin(.8*(x+s));dy=.008*math.cos(.8*(x+s));q=PoseStamped();q.header=p.header;q.pose.position.x=s*math.cos(yaw)+(wy-y)*math.sin(yaw);q.pose.position.y=-s*math.sin(yaw)+(wy-y)*math.cos(yaw);a=math.atan(dy)-yaw;q.pose.orientation.z=math.sin(a/2.);q.pose.orientation.w=math.cos(a/2.);p.poses.append(q)
   path.publish(p)
   if sys.argv[4]=='healthy':
    # All geometry is generated above; this is an explicit synthetic healthy
    # status, using the real reconstructor vocabulary rather than unknown TRACK.
    st='mode=TRACK_GOOD age=%.3f quality=1 reference_geometry_valid=1 reference_usable=1 accepted_source_age=0 splice_preserved_suffix_span=3 reference_estimator_confidence=1 reference_full_backtrack_count=0 reference_stale=0'%t
   else:st='mode=TRACK quality=1 reference_geometry_valid=1 reference_usable=1 accepted_source_age=0'
   status.publish(String(st))
  raw['phases'].append(dict(wall=wall,t=t,lease_published=active,imu_frame=im.header.frame_id))
finally:
 raw['origin_wall']=start;time.sleep(.2)
 with lock:open(sys.argv[1],'w').write(json.dumps(raw,sort_keys=True))
'''


def stop(proc):
    if proc.poll() is not None:
        return
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=4)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--algorithm', choices=('nmpc_12_fixed', 'flat_nmpc_8', 'flat_nmpc_8_uniform', 'blocked8','flat_nmpc_slqr'), default='nmpc_12_fixed')
    parser.add_argument('--trial', type=Path, help='Optional manifest provenance only; never authorizes this test or real motion')
    parser.add_argument('--capture',action='store_true',help='Also exercise the actual passive recorder and post-run analyser, synthetic data only')
    parser.add_argument('--speed-cap',type=float,default=.10,choices=(.10,.40))
    parser.add_argument('--reference-quality',choices=('degraded','healthy'),default='degraded')
    parser.add_argument('--reference-hz',type=int,choices=(10,30),default=10)
    parser.add_argument('--revalidate-reference',action='store_true')
    parser.add_argument('--flat-feasibility-recovery',action='store_true')
    parser.add_argument('--flat-accuracy-refinement',action='store_true')
    parser.add_argument('--flat-conditioned-geometry',action='store_true')
    parser.add_argument('--flat-baseline-seed',action='store_true')
    parser.add_argument('--flat-delay-aligned',action='store_true')
    parser.add_argument('--flat-independent-input-encoding',action='store_true')
    parser.add_argument('--flat-command-projection',action='store_true')
    parser.add_argument('--shadow-only',action='store_true')
    parser.add_argument('--config-root',type=Path)
    parser.add_argument('--disable-near-knot-restoration',action='store_true')
    parser.add_argument('--inject-conflicting-canonical-flags',action='store_true')
    parser.add_argument('--bounded-slqr-feedback',action='store_true')
    parser.add_argument('--live-preview-contract',action='store_true')
    parser.add_argument('--steering-rate-weight',type=float,default=0.)
    parser.add_argument('--turn-aware-rate-continuity',action='store_true')
    parser.add_argument('--smooth-steering-handoff',action='store_true')
    parser.add_argument('--straight-tracking-reference',action='store_true')
    parser.add_argument('--supported-course',action='store_true')
    parser.add_argument('--straight-yaw-damping-weight',type=float,default=0.)
    parser.add_argument('--turn-speed-reserve',type=float,default=0.)
    parser.add_argument('--turn-release-weight',type=float,default=0.)
    parser.add_argument('--yaw-rate-weight',type=float,default=0.)
    args = parser.parse_args()
    if args.flat_feasibility_recovery and not args.revalidate_reference:
        parser.error('flat feasibility recovery requires current-reference revalidation')
    if args.flat_accuracy_refinement and not (args.flat_feasibility_recovery and args.revalidate_reference):
        parser.error('flat accuracy refinement requires recovery and current-reference revalidation')
    if args.bounded_slqr_feedback and (not args.revalidate_reference or args.algorithm!='flat_nmpc_slqr'):
        parser.error('bounded SLQR feedback requires flat_nmpc_slqr and current-reference revalidation')
    if args.live_preview_contract and (not args.revalidate_reference or args.shadow_only or args.algorithm!='flat_nmpc_slqr'):
        parser.error('live preview contract requires revalidated flat_nmpc_slqr on the private plant')
    if args.smooth_steering_handoff and not args.live_preview_contract:
        parser.error('smooth steering handoff requires the live preview contract')
    if args.straight_tracking_reference and not args.live_preview_contract:
        parser.error('straight tracking reference requires the live preview contract')
    if args.supported_course and not args.straight_tracking_reference:
        parser.error('supported course requires straight tracking reference')
    if not 0.<=args.straight_yaw_damping_weight<=4. or (args.straight_yaw_damping_weight and not args.straight_tracking_reference):
        parser.error('straight yaw damping weight requires a straight reference and range [0,4]')
    if not 0.<=args.turn_speed_reserve<=.05 or not 0.<=args.turn_release_weight<=4. or (
            (args.turn_speed_reserve or args.turn_release_weight) and not args.live_preview_contract):
        parser.error('turn timing requires the live preview contract and bounded weights')
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    processes = []
    result = dict(pass_test=False, data_origin='synthetic_tool_test', private_master=True,
                  real_hardware=False, real_vehicle_test=False, calibration_claim=False,
                  production_modified=False, test_mapping_sha='a' * 64)
    result['algorithm'] = args.algorithm
    result['reference_quality'] = args.reference_quality
    result['reference_hz'] = args.reference_hz
    result['reference_revalidation'] = args.revalidate_reference
    result['flat_feasibility_recovery'] = args.flat_feasibility_recovery
    result['flat_accuracy_refinement'] = args.flat_accuracy_refinement
    result['bounded_slqr_feedback'] = args.bounded_slqr_feedback
    result['shadow_only'] = args.shadow_only
    result['smooth_steering_handoff'] = args.smooth_steering_handoff
    result['straight_tracking_reference'] = args.straight_tracking_reference
    result['supported_course'] = args.supported_course
    result['straight_yaw_damping_weight'] = args.straight_yaw_damping_weight
    result['turn_speed_reserve'] = args.turn_speed_reserve
    result['turn_release_weight'] = args.turn_release_weight
    result['steering_rate_continuity_weight'] = args.steering_rate_weight
    log = (out / 'process.log').open('w')
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    env = os.environ.copy()
    env.update(ROS_MASTER_URI='http://127.0.0.1:%d' % port, ROS_HOSTNAME='127.0.0.1',
               ROS_HOME=str(out / 'ros_home'), ROS_LOG_DIR=str(out / 'ros_logs'))
    env.pop('ROS_IP', None)
    result['private_master_uri'] = env['ROS_MASTER_URI']
    try:
        if args.trial:
            result['optional_trial_manifest'] = dict(path=str(args.trial.resolve()), sha256=sha(args.trial))
        devel = Path(os.environ.get('CONTROL_RESEARCH_DEVEL',str(W / ('devel_p7' if (W/'devel_p7').is_dir() else 'devel'))))
        controller = devel / 'lib/move_car/control_pub'
        worker = devel / 'lib/move_car/p5_nmpc_worker_node'
        result['binary_sha'] = dict(control_pub=sha(controller), p5_nmpc_worker_node=sha(worker))
        result['source_sha'] = sha(__file__)
        master_proc = subprocess.Popen(['roscore', '-p', str(port)], env=env, stdout=log, stderr=log)
        processes.append(master_proc)
        master = ServerProxy(env['ROS_MASTER_URI'])
        deadline = time.monotonic() + 15
        while True:
            try:
                if master.getPid('/p7_smoke_runner')[0] == 1:
                    break
            except OSError:
                pass
            if time.monotonic() > deadline:
                raise RuntimeError('private master startup failed')
            time.sleep(.1)
        run = 'p7real_software_smoke_' + str(time.time_ns())
        result['run_id'] = run
        master.setParam('/p7_smoke_runner', '/control_research/run_id', run)
        import sys
        sys.path.insert(0, str(P/'scripts'))
        from prepare_control_runtime import prepare
        config_root=args.config_root or P/('config/control/p7_smooth_0p4' if args.speed_cap>.30 else 'config/control/p6')
        runtime = prepare(str(config_root),str(P/'config/control/sensor_calibration.yaml'),
                          'legacy_slqr','smooth')['research_runtime']
        params = {e.attrib['name']: yaml.safe_load(e.attrib['value']) for e in
                  ET.parse(str(P / 'launch/movecar_improved.launch')).getroot().iter('param')}
        params.update(cmd_topic='/cmd_vel', tracked_topic='/tracked_pose_array',
                      enable_debug_csv_logs=False, enable_research_telemetry=False,
                      enable_adaptive_shadow=False, research_runtime=runtime,
                      p5_config_hash=runtime['config_hash'], p5_isolated_test=False,
                      p5_allow_test_nmpc=False, p5_nominal_state_contract=False,
                      p5_enable_adaptation=False, p5_diagnostics_enabled=True,
                      p5_terrain='smooth', p6_enabled=True, p6_algorithm_mode='nmpc_shadow' if args.shadow_only else args.algorithm,
                      p6_shadow_algorithm=args.algorithm,
                      p6_session_id=run+'_synthetic_tool_session',
                      p6_synthetic_observation_contract=False, enable_p6_adaptation=False,
                      enable_adaptation=False, p7_supervised_test=True,
                      p7_mapping_confirmed=True, p7_run_id=run,
                      p7_operator_id='synthetic_tool_test_operator',
                      p7_mapping_evidence='a'*64, p7_imu_frame='base_link', p7_speed_cap=args.speed_cap)
        params['p7_reference_revalidation']=args.revalidate_reference
        params['p7_flat_feasibility_recovery']=args.flat_feasibility_recovery
        params['p7_flat_accuracy_refinement']=args.flat_accuracy_refinement
        params['p7_flat_conditioned_geometry']=args.flat_conditioned_geometry
        params['p7_flat_baseline_seed']=args.flat_baseline_seed
        params['p7_flat_delay_aligned_reconstruction']=args.flat_delay_aligned
        params['p7_flat_independent_input_encoding']=args.flat_independent_input_encoding
        params['p7_flat_command_projection']=args.flat_command_projection
        if args.disable_near_knot_restoration:params['p7_flat_near_knot_restoration']=False
        if args.inject_conflicting_canonical_flags:
            for key in ('flat_conditioned_geometry','flat_baseline_seed','flat_delay_aligned_reconstruction'):
                params['p6_'+key]=not params['p7_'+key]
        params['p7_bounded_slqr_feedback']=args.bounded_slqr_feedback
        params.update(p6_steering_rate_continuity_weight=args.steering_rate_weight,
                      p6_turn_aware_rate_continuity=args.turn_aware_rate_continuity,
                      p7_smooth_steering_handoff=args.smooth_steering_handoff,
                      p7_straight_tracking_reference=args.straight_tracking_reference,
                      p7_supported_course=args.supported_course,
                      p7_straight_yaw_damping_weight=args.straight_yaw_damping_weight,
                      p7_turn_speed_reserve=args.turn_speed_reserve,
                      p7_turn_release_weight=args.turn_release_weight,
                      p6_yaw_rate_tracking_weight=args.yaw_rate_weight)
        if args.live_preview_contract:
            params.update(control_rate=40.,p7_live_handoff=True,p7_async_preview=True,
                p7_candidate_control_trial=True,p7_terminal_handoff=True,p6_terminal_approach=True,
                p6_speed_preview=True,p6_synchronized_preview=True,p6_actuator_aligned_preview=True,
                p6_separated_speed_target=True,p6_qualified_steering=True,p6_can_only_feedback=True,
                p7_qualify_chassis_motion=True,p7_nominal_observation_hold=True,
                p7_flat_response_base=True,p7_flat_response_near_support=True,
                p7_retime_command_increments=True,p7_spatial_reference_continuity=True)
        params.update(max_speed=args.speed_cap)
        if args.speed_cap>.10:
            params.update(cruise_speed=args.speed_cap,continuous_cruise_speed=args.speed_cap,
                continuous_catchup_max_speed=args.speed_cap,continuous_curve_max_speed=args.speed_cap)
        # The fixed source seed remains uncalibrated. These test mappings are
        # artificial and may never be copied to a real trial manifest.
        master.setParam('/p7_smoke_runner', '/p7_smoke_controller', params)
        worker_params = dict(params, run_id=run)
        master.setParam('/p7_smoke_runner', '/p7_smoke_worker', worker_params)
        (out / 'test_parameters.json').write_text(json.dumps(params, sort_keys=True, indent=2)+'\n')
        wp = subprocess.Popen([str(worker), '__name:=p7_smoke_worker'], env=env, stdout=log, stderr=log)
        processes.append(wp)
        cp = subprocess.Popen([str(controller), '__name:=p7_smoke_controller'], env=env, stdout=log, stderr=log)
        processes.append(cp)
        time.sleep(.8)
        assert wp.poll() is None and cp.poll() is None, 'P7 process failed before fixture; see process.log'
        capture=None
        if args.capture:
            freeze=out/'synthetic_freeze.json';freeze.write_text(json.dumps(dict(synthetic_tool_test=True,binary_sha=result['binary_sha'])))
            capture=subprocess.Popen(['/usr/bin/python2',str(P/'scripts/collect_stationary_raw.py'),
                '--output-dir',str(out/'capture'),'--run-id',run,'--session-id','synthetic_tool_session',
                '--freeze',str(freeze),'--phase','s8_tracking','--exclude-ned','--p6-events',
                '--terrain-label','smooth','--duration','30','--data-origin','synthetic_tool_test'],
                env=env,stdout=log,stderr=log)
            processes.append(capture)
            time.sleep(.6)
        fixture = out / 'synthetic_tool_fixture.py'
        fixture.write_text(FIXTURE)
        subprocess.run(['/usr/bin/python2', str(fixture), str(out/'raw.json'), run, str(wp.pid),args.reference_quality,str(args.reference_hz)],
                       env=env, stdout=log, stderr=log, timeout=20, check=True)
        graph = master.getSystemState('/p7_smoke_runner')[2]
        (out / 'private_graph.json').write_text(json.dumps(graph, indent=2)+'\n')
        pubs, subs = dict(graph[0]), dict(graph[1])
        result['command_publishers'] = pubs.get('/cmd_vel', [])
        assert result['command_publishers'] == ['/p7_smoke_controller'], 'multiple command publishers'
        passive={n for n in subs.get('/cmd_vel',[]) if args.capture and n.startswith('/p42_s8_raw_recorder_')}
        result['passive_command_subscribers']=sorted(passive)
        assert set(subs.get('/cmd_vel', [])) <= {'/p7_synthetic_tool_fixture'}|passive, 'unexpected actuator subscriber'
        assert cp.poll() is None, 'controller exited with worker'
        raw = json.loads((out/'raw.json').read_text())
        if capture is not None:
            stop(capture)
            from control_research.smooth_run_analysis import analyze_run
            summary=analyze_run(out/'capture'/run,out/'capture_analysis')
            result['capture']=dict(complete=summary['collection_complete'],
                candidates=summary['nmpc']['candidates'],owners=summary['published_history']['owners'],
                raw_chassis_samples=summary['sensors']['raw_chassis_samples'],
                imu_samples=summary['sensors']['imu_samples'],data_origin=summary['data_origin'])
            assert summary['collection_complete'] and summary['nmpc']['candidates']>0, 'passive raw capture or analysis failed'
            assert summary['data_origin']=='synthetic_tool_test', 'software tool fixture mislabeled real'
        start = raw['origin_wall']
        def phase(rows, lo, hi):
            return [r for r in rows if start+lo <= r['receipt_wall'] <= start+hi]
        commands = raw['commands']
        records = [r for r in raw['published'] if r['publish_returned'] and r['history_committed']]
        assert commands and records, 'no observed commands or committed history'
        result['commands'] = len(commands)
        result['published_records'] = len(records)
        intervals = sorted((b['publish_steady_start_ns'] - a['publish_steady_start_ns']) / 1e6
                           for a, b in zip(records, records[1:]))
        if intervals:
            result['publish_intervals_ms'] = {
                key: intervals[round((len(intervals)-1)*quantile)]
                for key, quantile in (('p50', .50), ('p95', .95), ('p99', .99), ('max', 1.))}
            result['intervals_over_33_333_ms'] = sum(interval > 1000./30. for interval in intervals)
        result['maximum_speed'] = max(r['speed'] for r in commands)
        assert all(0. <= r['speed'] <= args.speed_cap+1e-7 for r in commands), 'trial cap bypassed'
        result['zero_speed_windows'] = {}
        for name, lo, hi in [('initial_no_lease', .25, 1.3), ('expired_lease', 5.5, 6.1), ('worker_exited_lease_expired', 7.9, 8.5)]:
            rows = phase(commands, lo, hi)
            result['zero_speed_windows'][name] = dict(n=len(rows), maximum_speed=max((abs(r['speed']) for r in rows), default=None))
            assert len(rows) >= 5 and all(abs(r['speed']) < 1e-12 for r in rows), name
        result['commands_after_worker_exit'] = len([r for r in commands if r['receipt_wall'] > raw['worker_exit_wall']+.15])
        assert result['commands_after_worker_exit'] >= 10, 'worker exit blocked S8 output'
        result['physical_execution_claims'] = sum(r['physical_execution_confirmed'] for r in records)
        assert result['physical_execution_claims'] == 0
        result['request_count'] = len(raw['requests'])
        result['candidate_count'] = len(raw['candidates'])
        result['real_labelled_requests'] = sum(not r['synthetic'] for r in raw['requests'])
        assert raw['requests'], 'no P7 request received; check generated ROS message compatibility in process.log'
        assert all(not r['synthetic'] for r in raw['requests']), 'P7 claimed synthetic sensor contract'
        result['solver_attempts'] = sum(r['base']['solve_ms'] > 0. for r in raw['candidates'])
        result['valid_candidates'] = sum(r['base']['valid'] for r in raw['candidates'])
        blend = [float(field.split('=',1)[1]) for r in raw['candidates'] if r['base']['valid']
                 for field in r['base']['reason'].split(';') if field.startswith('straight_tracking_blend=')]
        result['straight_conditioned_valid_candidates'] = sum(v>0. for v in blend)
        if args.straight_tracking_reference:
            assert result['straight_conditioned_valid_candidates']>0, 'straight tracking reference did not run'
        damping = [float(field.split('=',1)[1]) for r in raw['candidates'] if r['base']['valid']
                   for field in r['base']['reason'].split(';') if field.startswith('straight_yaw_damping_weight=')]
        result['straight_damped_valid_candidates'] = sum(v>0. for v in damping)
        if args.straight_yaw_damping_weight:
            assert result['straight_damped_valid_candidates']>0, 'straight yaw cost did not run'
        turn_active = [float(field.split('=',1)[1]) for r in raw['candidates'] if r['base']['valid']
                       for field in r['base']['reason'].split(';')
                       if field.startswith(('turn_speed_reserve=','turn_release_weight='))]
        result['none_turn_timing_on_straight_plant']=bool(turn_active) and all(v==0. for v in turn_active)
        if args.turn_speed_reserve or args.turn_release_weight:
            assert result['none_turn_timing_on_straight_plant'], 'turn timing changed the near-straight test plant'
        result['decision_dimensions'] = sorted({r['base']['decision_dimension'] for r in raw['candidates'] if r['base']['valid']})
        assert result['decision_dimensions'] == ([12] if args.algorithm == 'nmpc_12_fixed' else [8]), 'wrong actual optimization dimension'
        result['nmpc_selected'] = sum(r['owner'] == 'nmpc' for r in records)
        if args.algorithm!='nmpc_12_fixed':
            expected={'flat_conditioned_geometry':args.flat_conditioned_geometry,'flat_baseline_seed':args.flat_baseline_seed,'flat_delay_aligned_reconstruction':args.flat_delay_aligned,
                      'flat_independent_input_encoding':args.flat_independent_input_encoding,'flat_command_projection':args.flat_command_projection}
            valid=[r for r in raw['candidates'] if r['base']['valid']]
            result['strategy_matches_frozen_flags']=bool(valid) and all(all((';%s=%d'%(k,v)) in r['base']['reason'] for k,v in expected.items()) for r in valid)
            assert result['strategy_matches_frozen_flags'],'stale canonical flag overrode frozen P7 strategy'
        result['normal_nmpc_above_previous_point_three_two'] = sum(r['owner']=='nmpc' and r['speed']>.32 for r in records)
        if not args.shadow_only and args.speed_cap>.32 and args.reference_quality=='healthy':
            assert result['normal_nmpc_above_previous_point_three_two']>0, 'higher Config cap never reached the final NMPC command'
        result['fresh_local_feedback_candidates'] = sum(r['base']['valid'] and r['feedback_used_new_observation'] for r in raw['candidates'])
        result['nonzero_local_feedback_candidates'] = sum(r['base']['valid'] and r['feedback_used_new_observation'] and abs(r['feedback_correction'])>1e-10 for r in raw['candidates'])
        if args.algorithm=='flat_nmpc_slqr':
            assert result['fresh_local_feedback_candidates']>0 and result['nonzero_local_feedback_candidates']>0, 'SLQR feedback never used a fresh nonzero error'
        result['leased_nonzero_commands'] = sum(r['speed'] > 1e-5 for r in phase(commands, 1.7, 5.1))
        assert result['solver_attempts'] > 0, 'engineering path never ran NMPC solve'
        if args.shadow_only:
            assert result['nmpc_selected']==0 and all(r['owner']!='nmpc' for r in records), 'shadow candidate obtained output authority'
        else:
            assert result['nmpc_selected'] > 0, 'NMPC solved but never became the selected software-plant control source'
        assert result['leased_nonzero_commands'] > 0, 'operator lease never allowed controller command'
        wrong = phase(raw['diagnostics'], 3.15, 3.55)
        result['wrong_frame_nmpc_accepted'] = sum(r['candidate_accepted'] for r in wrong)
        assert wrong and result['wrong_frame_nmpc_accepted'] == 0, 'wrong frame accepted'
        seq = [r['sequence'] for r in records]
        result['single_commit'] = seq == list(range(seq[0], seq[-1]+1))
        assert result['single_commit'], 'published-history sequence gap'
        result['candidate_reasons'] = {reason: sum(r['base']['reason'] == reason for r in raw['candidates']) for reason in sorted({r['base']['reason'] for r in raw['candidates']})}
        result['pass_test'] = True
    except Exception as exc:
        result['failure'] = repr(exc)
        raise
    finally:
        for proc in reversed(processes):
            stop(proc)
        log.close()
        (out/'result.json').write_text(json.dumps(result, sort_keys=True, indent=2)+'\n')
    print(json.dumps(result, sort_keys=True, indent=2))


if __name__ == '__main__':
    main()
