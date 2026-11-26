#!/usr/bin/env python3
"""Prepare/start an explicitly operator-supervised smooth trial, not deployment.

prepare is offline. start requires no existing command publisher and launches
only the independent controller/worker. It never starts the chassis or lease.
"""
import argparse
from datetime import datetime, timedelta, timezone
import json
import os
from pathlib import Path
import shutil
import shlex
import signal
import subprocess
import sys
import uuid
import xml.etree.ElementTree as ET

import yaml
from check_control_collection import read_master
from prepare_control_runtime import prepare as prepare_runtime
from run_smooth_experiment import prepare, sha, write_new, live_provenance

PACKAGE=Path(__file__).resolve().parents[1]
WORKSPACE=PACKAGE.parents[1]
PRODUCTION=Path('/home/ubuntu/drive_car_copy')
MODES=('nmpc_12_fixed','flat_nmpc_8','flat_nmpc_8_uniform','blocked8','legacy_slqr','nmpc_shadow','flat_nmpc_slqr')


def binaries():
    default=WORKSPACE/'devel'
    return Path(os.environ.get('CONTROL_RESEARCH_DEVEL',str(default)))/'lib/move_car'


def require(value, reason):
    if not value:raise ValueError(reason)


def candidate_strategy(root, algorithm):
    """Optional P7 algorithm overlay in the SAME frozen Config directory.

    It selects tested search structure, never changes physical/model limits or
    grants authority. Absence preserves existing trial preparation behavior.
    """
    path=Path(root)/'p7_strategy.yaml'
    if not path.exists():return None
    from control_research.config import UniqueLoader
    data=yaml.load(path.read_text(),Loader=UniqueLoader)
    keys={'schema_version','kind','status','terrain','algorithm','source_run_id',
          'source_events_sha256','reference_revalidation','flat_feasibility_recovery',
          'safety_limits_unchanged','automatic_promotion','evidence'}
    optional={'flat_accuracy_refinement','bounded_slqr_feedback','smooth_steering_handoff','straight_tracking_reference','supported_course',
              'turn_aware_rate_continuity',
              'retime_command_increments','spatial_reference_continuity',
              'flat_conditioned_geometry','flat_baseline_seed','flat_delay_aligned_reconstruction','flat_near_knot_restoration','flat_independent_input_encoding','flat_command_projection',
              'flat_causal_reference_seed','flat_response_knot_monitor','flat_input_feasibility_sqp','flat_response_base','flat_response_near_support','nominal_observation_hold','flat_budgeted_poll','flat_response_warm_start','qualified_steering','qualify_chassis_motion','can_only_feedback','course_consistent_feedback','separated_speed_target','speed_preview','synchronized_preview','actuator_aligned_preview','terminal_approach','terminal_handoff','async_preview','live_handoff'}
    require(isinstance(data,dict) and keys<=set(data) and set(data)<=keys|optional|
        {'steering_rate_continuity_weight','yaw_rate_tracking_weight','straight_yaw_damping_weight',
         'turn_speed_reserve','turn_release_weight'},
        'invalid P7 candidate strategy schema')
    require(all(type(data.get(k,False)) is bool for k in optional), 'P7 strategy options must be boolean')
    continuity=data.get('steering_rate_continuity_weight',0.)
    require(type(continuity) in (int,float) and 0.<=continuity<=1.,'invalid steering rate continuity weight')
    yaw_rate_weight=data.get('yaw_rate_tracking_weight',0.)
    require(type(yaw_rate_weight) in (int,float) and 0.<=yaw_rate_weight<=4.,'invalid yaw-rate tracking weight')
    straight_yaw_damping=data.get('straight_yaw_damping_weight',0.)
    require(type(straight_yaw_damping) in (int,float) and 0.<=straight_yaw_damping<=4. and
        (straight_yaw_damping==0. or data.get('straight_tracking_reference',False)),
        'invalid straight yaw damping weight or reference gate')
    for name,upper in (('turn_speed_reserve',.05),('turn_release_weight',4.)):
        value=data.get(name,0.)
        require(type(value) in (int,float) and 0.<=value<=upper and
            (value==0. or data.get('speed_preview',False)), 'invalid turn timing '+name)
    require(type(data['schema_version']) is int and data['schema_version']==1 and
        data['kind']=='p7_algorithm_strategy' and data['status']=='research_candidate' and
        data['terrain']=='smooth' and data['algorithm']==algorithm and
        algorithm in ('flat_nmpc_8','flat_nmpc_8_uniform','flat_nmpc_slqr'),
        'P7 candidate strategy algorithm/terrain mismatch')
    require(data['safety_limits_unchanged'] is True and data['automatic_promotion'] is False and
        data['reference_revalidation'] is True and type(data['flat_feasibility_recovery']) is bool,
        'P7 strategy cannot disable reference validation or alter safety/authority')
    require(isinstance(data['source_run_id'],str) and data['source_run_id'].startswith('p7real_') and
        isinstance(data['source_events_sha256'],str) and len(data['source_events_sha256'])==64 and
        all(x in '0123456789abcdef' for x in data['source_events_sha256']) and
        isinstance(data['evidence'],list) and bool(data['evidence']), 'P7 strategy evidence required')
    return data


def candidate_trial_manifest(root, algorithm):
    """Explicit, unvalidated supervised-trial intent; never deployment approval."""
    path=(Path(root)/'CANDIDATE_TRIAL.json').resolve()
    require(path.is_file(),'candidate control trial requires CANDIDATE_TRIAL.json')
    data=json.loads(path.read_text())
    require(isinstance(data,dict) and type(data.get('schema_version')) is int and
        data['schema_version']==1 and data.get('kind')=='operator_supervised_unvalidated_candidate' and
        data.get('terrain')=='smooth' and data.get('algorithm')==algorithm=='flat_nmpc_slqr' and
        data.get('automatic_promotion') is False and data.get('safety_limits_unchanged') is True and
        data.get('validated_deployment') is False and
        isinstance(data.get('evidence'),list) and bool(data['evidence']),
        'invalid unvalidated candidate control trial manifest')
    return dict(path=str(path),sha256=sha(path),values=data)


def apply_historical_mapping(a):
    path=getattr(a,'mapping_profile',None)
    if path is None:return None
    path=Path(path).resolve();profile=json.loads(path.read_text())
    require(profile.get('schema_version')==1 and profile.get('kind')=='historical_engineering_mapping' and
        profile.get('engineering_mapping_supported') is True and
        profile.get('physical_calibration_certified') is False and
        profile.get('declared_ground_speed_independent') is False,
        'unsupported historical engineering profile; no physical calibration promotion')
    require(profile.get('imu_frame')=='gyro_link','historical IMU mapping must bind observed gyro_link')
    expected=dict(wheel_speed_scale=.001,feedback_steer_sign=1.,imu_yaw_sign=1.,cmd_steer_scale=1.,cmd_steer_sign=1.)
    for key,value in expected.items():require(profile.get(key)==value,'historical mapping mismatch: '+key)
    sources=profile.get('source_files')
    require(isinstance(sources,dict) and bool(sources),'mapping profile needs pinned actual evidence')
    retained = PACKAGE / 'scripts/smooth_trial_support/sensor_mapping/mapping_profile.json'
    if path == retained:
        # The published release pinned this report and profile before its raw
        # historical inputs were retired. It remains engineering evidence only.
        require(sha(path) == 'd17e91424891b639e774a2120be403efaab85af3985ce0ae18eb1cb8072d61f9',
                'retained engineering mapping changed')
        report = retained.with_name('report.json')
    else:
        for name, digest in sources.items():
            source = Path(name)
            require(source.is_absolute() and source.is_file() and sha(source) == digest,
                    'historical mapping source changed: ' + name)
        report = Path(profile['evidence_report']) if profile.get('evidence_report') else None
    if report is not None:
        require(report.is_file() and sha(report)==profile.get('evidence_report_sha256'),
            'historical mapping report changed')
        profile['maximum_observed_command_speed_mps']=json.loads(report.read_text()).get('historical_command_speed_max_mps')
    require(not a.imu_frame or a.imu_frame==profile['imu_frame'],'explicit IMU frame disagrees with historical evidence')
    a.imu_frame=profile['imu_frame'];a.mapping_evidence=path
    # These are the inherited S8 protocol conventions supported by real response
    # consistency, not an independently measured world-frame/left-turn calibration.
    a.confirm_forward_wheel_positive=a.confirm_left_steering_positive=a.confirm_left_yaw_positive=True
    return profile


def make_trial(a):
    observed=apply_historical_mapping(a)
    require(a.confirm_forward_wheel_positive and a.confirm_left_steering_positive and
            a.confirm_left_yaw_positive, 'confirm each observed mapping separately; no automatic calibration')
    require(a.operator.strip() and a.imu_frame and a.imu_frame.strip(), 'operator and actual IMU frame required')
    require(a.mapping_evidence is not None and a.mapping_evidence.is_file(), 'existing operator mapping notes required')
    require(0.<a.speed_cap<=.40, 'supervised normal operating cap must be >0 and <=0.40 m/s')
    default_config=PACKAGE/('config/control/p7_smooth_0p4' if a.speed_cap>.30 else 'config/control/p6')
    config_root=Path(getattr(a,'config_root',None) or default_config).resolve()
    require(config_root.is_dir(), 'existing complete P6 Schema 2 Config directory required')
    shadow_only=bool(getattr(a,'shadow_only',False))
    candidate_trial=bool(getattr(a,'candidate_control_trial',False))
    require(not (candidate_trial and shadow_only),'candidate control trial and shadow-only are mutually exclusive')
    require(not candidate_trial or a.algorithm=='flat_nmpc_slqr',
        'candidate control trial requires flat_nmpc_slqr')
    require(not shadow_only or a.algorithm in ('flat_nmpc_8','flat_nmpc_8_uniform','flat_nmpc_slqr','nmpc_12_fixed','blocked8'),
        'shadow-only requires an explicit computation algorithm')
    require(shadow_only or not (config_root/'ISOLATED_ONLY.json').exists(),
        'candidate is isolated-research-only; cannot prepare a real vehicle trial')
    strategy=candidate_strategy(config_root,a.algorithm)
    revalidate=getattr(a,'revalidate_reference',False) or bool(strategy and strategy['reference_revalidation'])
    recovery=getattr(a,'flat_feasibility_recovery',False) or bool(strategy and strategy['flat_feasibility_recovery'])
    near_knot=bool(strategy.get('flat_near_knot_restoration',revalidate)) if strategy else revalidate
    accuracy=bool(strategy and strategy.get('flat_accuracy_refinement',False))
    bounded=bool(strategy and strategy.get('bounded_slqr_feedback',False))
    conditioned=bool(strategy and strategy.get('flat_conditioned_geometry',False))
    baseline_seed=bool(strategy and strategy.get('flat_baseline_seed',False))
    delay_aligned=bool(strategy and strategy.get('flat_delay_aligned_reconstruction',False))
    independent_inputs=bool(strategy and strategy.get('flat_independent_input_encoding',False))
    command_projection=bool(strategy and strategy.get('flat_command_projection',False))
    causal_seed=bool(strategy and strategy.get('flat_causal_reference_seed',False))
    response_knots=bool(strategy and strategy.get('flat_response_knot_monitor',False))
    input_sqp=bool(strategy and strategy.get('flat_input_feasibility_sqp',False))
    response_base=bool(strategy and strategy.get('flat_response_base',False))
    near_support=bool(strategy and strategy.get('flat_response_near_support',False))
    nominal_hold=bool(strategy and strategy.get('nominal_observation_hold',False))
    budgeted_poll=bool(strategy and strategy.get('flat_budgeted_poll',False))
    response_warm=bool(strategy and strategy.get('flat_response_warm_start',False))
    qualified_steering=bool(strategy and strategy.get('qualified_steering',False))
    qualify_chassis_motion=bool(strategy and strategy.get('qualify_chassis_motion',False))
    retime_command_increments=bool(strategy and strategy.get('retime_command_increments',False))
    spatial_continuity=bool(strategy and strategy.get('spatial_reference_continuity',False))
    can_only_feedback=bool(strategy and strategy.get('can_only_feedback',False))
    course_feedback=bool(strategy and strategy.get('course_consistent_feedback',False))
    separated_speed=bool(strategy and strategy.get('separated_speed_target',False))
    speed_preview=bool(strategy and strategy.get('speed_preview',False))
    synchronized_preview=bool(strategy and strategy.get('synchronized_preview',False))
    actuator_aligned_preview=bool(strategy and strategy.get('actuator_aligned_preview',False))
    terminal_approach=bool(strategy and strategy.get('terminal_approach',False))
    terminal_handoff=bool(strategy and strategy.get('terminal_handoff',False))
    async_preview=bool(strategy and strategy.get('async_preview',False))
    live_handoff=bool(strategy and strategy.get('live_handoff',False))
    smooth_steering_handoff=bool(strategy and strategy.get('smooth_steering_handoff',False))
    straight_tracking_reference=bool(strategy and strategy.get('straight_tracking_reference',False))
    supported_course=bool(strategy and strategy.get('supported_course',False))
    require(not supported_course or straight_tracking_reference,'supported course requires straight tracking reference')
    straight_yaw_damping=float(strategy.get('straight_yaw_damping_weight',0.)) if strategy else 0.
    turn_timing={name:float(strategy.get(name,0.)) if strategy else 0.
                 for name in ('turn_speed_reserve','turn_release_weight')}
    require(not any(turn_timing.values()) or (live_handoff and revalidate and candidate_trial and speed_preview),
        'turn timing requires a supervised candidate with speed preview and reference revalidation')
    require(not smooth_steering_handoff or (live_handoff and revalidate and candidate_trial),
        'smooth steering handoff requires the supervised revalidated candidate trial')
    require(not straight_tracking_reference or (live_handoff and revalidate and candidate_trial),
        'straight tracking reference requires the supervised revalidated candidate trial')
    require(not live_handoff or (candidate_trial and async_preview),
        'live handoff requires an explicit staged flat8 control trial')
    require(not async_preview or terminal_handoff,'staged preview requires terminal handoff')
    require(not terminal_handoff or (terminal_approach and a.algorithm=='flat_nmpc_slqr'),
        'terminal handoff requires the flat_nmpc_slqr terminal approach')
    require(not terminal_approach or speed_preview,'terminal approach requires speed preview')
    require(not synchronized_preview or speed_preview,'synchronized preview requires speed preview')
    require(not actuator_aligned_preview or (synchronized_preview and revalidate),
        'actuator-aligned preview requires synchronized preview and reference revalidation')
    require(not speed_preview or revalidate,'speed preview requires current-reference validation')
    continuity_weight=float(strategy.get('steering_rate_continuity_weight',0.)) if strategy else 0.
    turn_aware_rate=bool(strategy and strategy.get('turn_aware_rate_continuity',False))
    require(not turn_aware_rate or (continuity_weight>0. and revalidate),
        'turn-aware steering continuity requires a positive weight and reference revalidation')
    yaw_rate_weight=float(strategy.get('yaw_rate_tracking_weight',0.)) if strategy else 0.
    require(not response_warm or (response_base and revalidate and (shadow_only or candidate_trial)),
        'response warm start requires response base and explicit candidate/shadow current-reference validation')
    require(not candidate_trial or (response_base and near_support and nominal_hold and revalidate),
        'candidate control trial requires response base, near support, nominal observation hold and reference revalidation')
    require(not budgeted_poll or response_base,'budgeted flat poll requires response base')
    require(not budgeted_poll or (revalidate and shadow_only),
        'budgeted flat poll currently requires reference revalidation and shadow-only collection')
    require(not near_support or response_base,'near correction support requires response base')
    require(not (near_support or nominal_hold) or (revalidate and (shadow_only or candidate_trial)),
        'response refinement requires shadow/current-reference validation or explicit candidate control trial')
    require(not near_knot or revalidate,'near-knot restoration requires reference revalidation')
    require(not recovery or revalidate,'flat feasibility recovery requires reference revalidation')
    require(not accuracy or (recovery and revalidate),'flat accuracy refinement requires recovery and reference revalidation')
    require(not bounded or (revalidate and a.algorithm=='flat_nmpc_slqr'),
        'bounded SLQR feedback requires flat_nmpc_slqr and reference revalidation')
    require(not can_only_feedback or (bounded and qualified_steering and revalidate),
        'CAN-only feedback requires qualified steering, bounded SLQR and reference revalidation')
    require(not qualify_chassis_motion or qualified_steering,
        'chassis motion qualification requires qualified steering')
    require(not retime_command_increments or (revalidate and live_handoff),
        'increment retiming requires live handoff and reference revalidation')
    require(not spatial_continuity or revalidate,'spatial continuity requires reference revalidation')
    require(not (causal_seed or response_knots) or (revalidate and shadow_only),
        'causal flat reference strategies currently require shadow-only collection')
    require(not input_sqp or (revalidate and shadow_only),
        'input feasibility SQP currently requires reference revalidation and shadow-only collection')
    require(not response_base or (revalidate and (shadow_only or candidate_trial)),
        'flat response base currently requires reference revalidation and shadow-only collection or explicit candidate control trial')
    trial_manifest=candidate_trial_manifest(config_root,a.algorithm) if candidate_trial else None
    if live_handoff:
        admitted=trial_manifest['values'].get('software_acceptance',{})
        evidence=Path(admitted.get('path',''))
        require(evidence.is_file() and sha(evidence)==admitted.get('sha256'),
                'live handoff software acceptance evidence changed')
        tested=json.loads(evidence.read_text())
        require(tested.get('supervised_software_qualified') is True and
                tested.get('binary_sha')=={name:sha(binaries()/name) for name in ('control_pub','p5_nmpc_worker_node')},
                'live handoff must use the tested binaries')
    if revalidate:
        require(b'revalidation_original_context_mismatch' in (binaries()/'control_pub').read_bytes(),
            'selected controller binary lacks reference revalidation; build/source the separate candidate workspace')
    if candidate_trial:
        require(b'p7_candidate_control_trial' in (binaries()/'p5_nmpc_worker_node').read_bytes(),
            'selected worker binary lacks p7_candidate_control_trial; build/source the explicit candidate trial workspace')
    if recovery:
        require(b'flat8_feasible_bounded_restoration' in (binaries()/'p5_nmpc_worker_node').read_bytes(),
            'selected worker binary lacks bounded flat feasibility recovery; build/source the postrun candidate workspace')
    if accuracy:
        require(b'p7_flat_accuracy_refinement' in (binaries()/'p5_nmpc_worker_node').read_bytes(),
            'selected worker binary lacks flat accuracy refinement; build/source the new candidate workspace')
    if bounded:
        require(b'feedback_bounded_rollout_feasible' in (binaries()/'p5_nmpc_worker_node').read_bytes(),
            'selected worker binary lacks constrained SLQR feedback; build/source the new candidate workspace')
    for enabled,key in ((conditioned,'p7_flat_conditioned_geometry'),
                        (baseline_seed,'p7_flat_baseline_seed'),
                        (delay_aligned,'p7_flat_delay_aligned_reconstruction'),
                        (independent_inputs,'p7_flat_independent_input_encoding'),
                        (command_projection,'p7_flat_command_projection'),
                        (causal_seed,'p7_flat_causal_reference_seed'),
                        (response_knots,'p7_flat_response_knot_monitor'),
                        (input_sqp,'p7_flat_input_feasibility_sqp'),
                        (response_base,'p7_flat_response_base'),
                        (near_support,'p7_flat_response_near_support'),
                        (nominal_hold,'p7_nominal_observation_hold'),
                        (budgeted_poll,'p7_flat_budgeted_poll'),
                        (response_warm,'p7_flat_response_warm_start')):
        if enabled:
            require(key.encode('ascii') in (binaries()/'p5_nmpc_worker_node').read_bytes(),
                'selected worker binary lacks '+key+'; build/source the new candidate workspace')
    for enabled,key,names in ((smooth_steering_handoff,'p7_smooth_steering_handoff',('control_pub',)),
                              (straight_tracking_reference,'p7_straight_tracking_reference',('p5_nmpc_worker_node',)),
                              (supported_course,'p7_supported_course',('p5_nmpc_worker_node',)),
                              (straight_yaw_damping>0.,'p7_straight_yaw_damping_weight',('p5_nmpc_worker_node',)),
                              (turn_timing['turn_speed_reserve']>0.,'p7_turn_speed_reserve',('p5_nmpc_worker_node',)),
                              (turn_timing['turn_release_weight']>0.,'p7_turn_release_weight',('p5_nmpc_worker_node',)),
                              (qualified_steering,'p6_qualified_steering',('control_pub','p5_nmpc_worker_node')),
                              (can_only_feedback,'p6_can_only_feedback',('p5_nmpc_worker_node',)),
                              (qualify_chassis_motion,'p7_qualify_chassis_motion',('control_pub','p5_nmpc_worker_node')),
                              (retime_command_increments,'p7_retime_command_increments',('control_pub',)),
                              (spatial_continuity,'p7_spatial_reference_continuity',('control_pub',)),
                              (course_feedback,'p6_course_consistent_feedback',('p5_nmpc_worker_node',)),
                              (separated_speed,'p6_separated_speed_target',('control_pub','p5_nmpc_worker_node')),
                              (continuity_weight>0.,'p6_steering_rate_continuity_weight',('p5_nmpc_worker_node',)),
                              (turn_aware_rate,'p6_turn_aware_rate_continuity',('p5_nmpc_worker_node',)),
                              (yaw_rate_weight>0.,'p6_yaw_rate_tracking_weight',('p5_nmpc_worker_node',)),
                              (speed_preview,'p6_speed_preview',('control_pub','p5_nmpc_worker_node')),
                              (synchronized_preview,'p6_synchronized_preview',('control_pub','p5_nmpc_worker_node')),
                              (actuator_aligned_preview,'p6_actuator_aligned_preview',('control_pub','p5_nmpc_worker_node')),
                              (terminal_approach,'p6_terminal_approach',('control_pub','p5_nmpc_worker_node')),
                              (terminal_handoff,'p7_terminal_handoff',('control_pub',)),
                              (async_preview,'p7_async_preview',('control_pub',)),
                              (live_handoff,'p7_live_handoff',('control_pub',))):
        if enabled:
            for name in names:
                require(key.encode('ascii') in (binaries()/name).read_bytes(),
                    'selected '+name+' lacks '+key+'; build/source the matching research overlay')
    if strategy and 'flat_near_knot_restoration' in strategy:
        require(b'p7_flat_near_knot_restoration' in (binaries()/'p5_nmpc_worker_node').read_bytes(),
            'selected worker binary lacks p7_flat_near_knot_restoration')
    require(a.speed_cap<=.10 or observed is not None,'above 0.10 m/s requires the pinned historical engineering mapping profile')
    text=a.mapping_evidence.read_text().strip();require(bool(text), 'empty mapping notes')
    run='p7real_'+datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%dT%H%M%S')+'_'+uuid.uuid4().hex[:8]
    folder,experiment=prepare(a.output_root,a.session_id,a.route,a.speed_cap,a.production_workspace,run_id=run)
    notes=folder/'mapping_operator_notes.txt';shutil.copyfile(str(a.mapping_evidence),str(notes))
    experiment.update(execution_context=('operator_supervised_s8_shadow_collection' if shadow_only else
        ('operator_supervised_unvalidated_candidate' if candidate_trial else 'operator_supervised_unvalidated_vehicle_trial')),
        candidate_control_trial=candidate_trial,
        requested_algorithm=a.algorithm, nmpc_vehicle_authority=False if shadow_only else 'requires_live_operator_lease_and_all_candidate_checks')
    (folder/'experiment.json').write_text(json.dumps(experiment,indent=2,sort_keys=True)+'\n')
    formal=Path(a.production_workspace)/'src/move_car/launch/movecar_improved.launch'
    tree=ET.parse(str(formal));params={p.attrib['name']:yaml.safe_load(p.attrib['value']) for p in tree.iter('param') if 'value' in p.attrib}
    require(params.get('wheel_speed_scale')==.001, 'only audited S8 0.001 raw-speed convention supported')
    require(params.get('feedback_steer_sign',1.)==1., 'positive-left first test requires S8 feedback sign +1; other mapping needs explicit adapter')
    require(params.get('cmd_steer_sign',1.)==1. and params.get('cmd_steer_scale',1.)==1., 'first test requires original identity steering command mapping')
    runtime=prepare_runtime(str(config_root),str(PACKAGE/'config/control/sensor_calibration.yaml'),'legacy_slqr','smooth')
    rr=runtime['research_runtime'];require(not rr['scene_transition']['enabled'], 'initial supervised scene transition must stay off')
    require(a.speed_cap<=rr['profiles']['smooth']['values']['speed_cap'],
        'requested speed exceeds selected smooth Config cap; choose a matching candidate Config')
    require(not any(c['enabled'] for c in rr['channel_policies'].values()), 'initial supervised adaptation policies must stay off')
    freeze_path=folder/'software_freeze.json'
    if freeze_path.is_file():
        freeze=json.loads(freeze_path.read_text())
        freeze['base_layer_config_hash']=freeze.get('nominal_config_hash')
        freeze['nominal_config_hash']=rr['config_hash']
        freeze['nominal_config_hash_scope']='actual immutable P6 runtime aggregate identity'
        freeze['selected_config_root']=str(config_root)
        freeze.setdefault('files',{}).update({str(p):sha(p) for p in sorted(config_root.rglob('*.yaml'))})
        if trial_manifest:
            freeze['candidate_trial_manifest']=trial_manifest
            freeze['files'][trial_manifest['path']]=trial_manifest['sha256']
        freeze_path.write_text(json.dumps(freeze,indent=2,sort_keys=True)+'\n')
        experiment['software_freeze_sha256']=sha(freeze_path)
        experiment['nominal_config_hash']=rr['config_hash']
        experiment['selected_config_root']=str(config_root)
        (folder/'experiment.json').write_text(json.dumps(experiment,indent=2,sort_keys=True)+'\n')
    with (folder/'runtime.yaml').open('x') as f:yaml.safe_dump(runtime,f,sort_keys=True)
    common=dict(p7_supervised_test=True,p7_mapping_confirmed=True,p7_run_id=run,
        p7_operator_id=a.operator,p7_mapping_evidence=sha(notes),p7_imu_frame=a.imu_frame,
        p7_speed_cap=a.speed_cap,p5_terrain='smooth',degraded_speed_limit=.10,
        wheel_speed_scale=.001,feedback_steer_sign=1.,p6_enabled=True,
        p6_synthetic_observation_contract=False,enable_p6_adaptation=False,
        enable_adaptation=False,p5_enable_adaptation=False)
    common['p7_candidate_control_trial']=candidate_trial
    common['p7_reference_revalidation']=revalidate
    common['p7_flat_feasibility_recovery']=recovery
    common['p7_flat_accuracy_refinement']=accuracy
    common['p7_bounded_slqr_feedback']=bounded
    common['p7_flat_near_knot_restoration']=near_knot
    common['p7_flat_conditioned_geometry']=conditioned
    common['p7_flat_baseline_seed']=baseline_seed
    common['p7_flat_delay_aligned_reconstruction']=delay_aligned
    common['p7_flat_independent_input_encoding']=independent_inputs
    common['p7_flat_command_projection']=command_projection
    common['p7_flat_causal_reference_seed']=causal_seed
    common['p7_flat_response_knot_monitor']=response_knots
    common['p7_flat_input_feasibility_sqp']=input_sqp
    common['p7_flat_response_base']=response_base
    common['p7_flat_response_near_support']=near_support
    common['p7_nominal_observation_hold']=nominal_hold
    common['p7_flat_budgeted_poll']=budgeted_poll
    common['p7_flat_response_warm_start']=response_warm
    common['p6_qualified_steering']=qualified_steering
    common['p7_qualify_chassis_motion']=qualify_chassis_motion
    common['p7_retime_command_increments']=retime_command_increments
    common['p7_spatial_reference_continuity']=spatial_continuity
    common['p6_can_only_feedback']=can_only_feedback
    common['p6_course_consistent_feedback']=course_feedback
    common['p6_separated_speed_target']=separated_speed
    common['p6_steering_rate_continuity_weight']=continuity_weight
    common['p6_turn_aware_rate_continuity']=turn_aware_rate
    common['p6_yaw_rate_tracking_weight']=yaw_rate_weight
    if smooth_steering_handoff:
        common['p7_smooth_steering_handoff']=True
    if straight_tracking_reference:
        common['p7_straight_tracking_reference']=True
    if supported_course:
        common['p7_supported_course']=True
    if straight_yaw_damping>0.:
        common['p7_straight_yaw_damping_weight']=straight_yaw_damping
    common.update(('p7_'+name,value) for name,value in turn_timing.items() if value>0.)
    common['p6_speed_preview']=speed_preview
    common['p6_synchronized_preview']=synchronized_preview
    common['p6_actuator_aligned_preview']=actuator_aligned_preview
    common['p6_terminal_approach']=terminal_approach
    common['p7_terminal_handoff']=terminal_handoff
    common['p7_async_preview']=async_preview
    common['p7_live_handoff']=live_handoff
    controller=dict(common,p6_algorithm_mode='nmpc_shadow' if shadow_only else a.algorithm,
        p6_shadow_algorithm=a.algorithm if shadow_only else 'nmpc_12_fixed',
        p6_session_id=a.session_id,p5_config_hash=rr['config_hash'],p5_diagnostics_enabled=True,
        p5_isolated_test=False,p5_allow_test_nmpc=False,p5_nominal_state_contract=False,
        p5_mode='legacy_slqr',cmd_topic='/cmd_vel',max_speed=a.speed_cap,
        debug_log_dir=str(folder/'s8_csv_live'),enable_debug_csv_logs=True,
        enable_adaptive_shadow=False,enable_research_telemetry=False)
    if live_handoff:controller['control_rate']=40.
    if a.speed_cap>.10:
        # Operating targets only. Original lateral-acceleration, error/geometry,
        # actuator, following, braking, solver-failure and emergency caps remain.
        controller.update(cruise_speed=a.speed_cap,continuous_cruise_speed=a.speed_cap,
            continuous_catchup_max_speed=a.speed_cap,continuous_curve_max_speed=a.speed_cap)
    worker=dict(common,run_id=run)
    configuration=dict(research_runtime=runtime['research_runtime'],**controller)
    with (folder/'controller.yaml').open('x') as f:yaml.safe_dump(configuration,f,sort_keys=True)
    with (folder/'worker.yaml').open('x') as f:yaml.safe_dump(dict(research_runtime=runtime['research_runtime'],**worker),f,sort_keys=True)
    launch=ET.Element('launch')
    # The original S8 parameter file is included unchanged. The research overlay
    # resolves the executable from this independent workspace, never production.
    include = ET.SubElement(launch,'include',{'file':str(formal)})
    ET.SubElement(include, 'arg', {'name': 'smooth', 'value': 'false'})
    ET.SubElement(launch,'param',{'name':'/control_research/run_id','value':run})
    ET.SubElement(launch,'rosparam',{'command':'load','file':str(folder/'controller.yaml'),'ns':'publish_speed_turn'})
    node=ET.SubElement(launch,'node',{'pkg':'move_car','type':'p5_nmpc_worker_node','name':'p7_nmpc_worker','output':'screen'})
    ET.SubElement(node,'rosparam',{'command':'load','file':str(folder/'worker.yaml')})
    ET.ElementTree(launch).write(str(folder/'trial.launch'),encoding='utf-8',xml_declaration=True)
    trial=dict(schema_version=1,run_id=run,session_id=a.session_id,terrain='smooth',
        algorithm='nmpc_shadow' if shadow_only else a.algorithm,computation_algorithm=a.algorithm,
        shadow_only=shadow_only,nmpc_vehicle_authority=False if shadow_only else 'requires_live_operator_lease_and_all_candidate_checks',
        candidate_control_trial=candidate_trial,candidate_trial_manifest=trial_manifest,
        operator_id=a.operator,imu_frame=a.imu_frame,speed_cap=a.speed_cap,
        engineering_mapping_confirmed=True,physical_calibration_certified=False,
        mapping_basis='historical_S8_engineering_mapping' if observed else 'operator_engineering_confirmation',
        absolute_left_turn_independently_verified=False,
        historical_maximum_command_mps=observed.get('maximum_observed_command_speed_mps') if observed else None,
        requested_speed_is_not_measured_validation=True,
        degraded_speed_limit_unchanged=.10,
        execution_context=experiment['execution_context'],
        nominal_model_status='P6 research seed; not physical calibration or validated production Config',
        production_changed=False,automatic_arming=False,validated_deployment=False,
        mapping_evidence_sha256=sha(notes),nominal_config_hash=rr['config_hash'],
        nominal_config_directory=str(config_root),
        nominal_config_files={str(p):sha(p) for p in sorted(config_root.rglob('*.yaml'))},
        sampling_time='receipt-domain engineering proxy; hardware time unknown',
        online_adaptation=False,live_slqr_fresh_state_feedback=a.algorithm=='flat_nmpc_slqr',
        reference_revalidation=revalidate,
        flat_feasibility_recovery=recovery,flat_accuracy_refinement=accuracy,
        bounded_slqr_feedback=bounded,flat_near_knot_restoration=near_knot,flat_conditioned_geometry=conditioned,
        flat_baseline_seed=baseline_seed,flat_delay_aligned_reconstruction=delay_aligned,
        flat_independent_input_encoding=independent_inputs,
        flat_command_projection=command_projection,
        flat_causal_reference_seed=causal_seed,flat_response_knot_monitor=response_knots,
        flat_input_feasibility_sqp=input_sqp,
        flat_response_base=response_base,flat_response_near_support=near_support,nominal_observation_hold=nominal_hold,flat_budgeted_poll=budgeted_poll,flat_response_warm_start=response_warm,
        qualified_steering=qualified_steering,qualify_chassis_motion=qualify_chassis_motion,
        retime_command_increments=retime_command_increments,
        spatial_reference_continuity=spatial_continuity,
        can_only_feedback=can_only_feedback,course_consistent_feedback=course_feedback,
        separated_speed_target=separated_speed,steering_rate_continuity_weight=continuity_weight,
        turn_aware_rate_continuity=turn_aware_rate,
        yaw_rate_tracking_weight=yaw_rate_weight,speed_preview=speed_preview,synchronized_preview=synchronized_preview,terminal_approach=terminal_approach,
        actuator_aligned_preview=actuator_aligned_preview,
        terminal_handoff=terminal_handoff,async_preview=async_preview,live_handoff=live_handoff,
        smooth_steering_handoff=smooth_steering_handoff,
        straight_tracking_reference=straight_tracking_reference,
        supported_course=supported_course,
        straight_yaw_damping_weight=straight_yaw_damping,
        **turn_timing,
        algorithm_strategy=strategy,
        source=experiment, trial_files={str(p.relative_to(folder)):sha(p) for p in folder.iterdir() if p.is_file()},
        binary_paths={name:str(binaries()/name) for name in ['control_pub','p5_nmpc_worker_node']},
        binaries={name:sha(binaries()/name) for name in ['control_pub','p5_nmpc_worker_node']},
        production_launch_sha256=sha(formal))
    write_new(folder/'trial.json',trial)
    print('TRIAL_DIRECTORY='+str(folder))
    if shadow_only:
        print('MODE=nmpc_shadow; ACTUAL_OUTPUT=S8; COMPUTATION='+a.algorithm+'; NMPC_selected is zero by design; inspect candidate solve/valid counts separately.')
    print('No controller or vehicle started. Start/record/lease are separate explicit commands.')
    print('prepare cannot change TRIAL in your shell. Copy this assignment into each terminal:')
    print('TRIAL='+shlex.quote(str(folder)))
    return folder


def read_trial(folder):
    folder=Path(folder).resolve();trial=json.loads((folder/'trial.json').read_text())
    require(trial.get('schema_version')==1 and trial.get('terrain')=='smooth' and
            trial.get('run_id','').startswith('p7real_') and trial.get('algorithm') in MODES,
            'invalid trial identity')
    require(type(trial.get('candidate_control_trial',False)) is bool,
        'invalid candidate control trial flag')
    # Each file includes the full P6 runtime tree. Parse it once per preflight
    # and use the same snapshot for every flag and weight check.
    loader=getattr(yaml,'CSafeLoader',yaml.SafeLoader)
    controller,worker=(yaml.load((folder/name).read_text(),Loader=loader)
                       for name in ('controller.yaml','worker.yaml'))
    parameters=(controller,worker)
    if 'flat_response_warm_start' in trial:
        require(type(trial['flat_response_warm_start']) is bool,'invalid response warm-start flag')
        for params in parameters:
            require(params.get('p7_flat_response_warm_start',False) is trial['flat_response_warm_start'],
                'response warm-start frozen flag mismatch')
    for option,key in (('qualified_steering','p6_qualified_steering'),
                       ('can_only_feedback','p6_can_only_feedback'),('qualify_chassis_motion','p7_qualify_chassis_motion'),
                       ('retime_command_increments','p7_retime_command_increments'),
                       ('spatial_reference_continuity','p7_spatial_reference_continuity'),
                       ('course_consistent_feedback','p6_course_consistent_feedback'),
                       ('separated_speed_target','p6_separated_speed_target'),('speed_preview','p6_speed_preview'),
                       ('synchronized_preview','p6_synchronized_preview'),('terminal_approach','p6_terminal_approach'),
                       ('actuator_aligned_preview','p6_actuator_aligned_preview'),
                       ('terminal_handoff','p7_terminal_handoff'),('async_preview','p7_async_preview'),
                       ('turn_aware_rate_continuity','p6_turn_aware_rate_continuity'),
                       ('smooth_steering_handoff','p7_smooth_steering_handoff'),
                       ('straight_tracking_reference','p7_straight_tracking_reference'),
                       ('supported_course','p7_supported_course'),
                       ('live_handoff','p7_live_handoff')):
        if option in trial:
            require(type(trial[option]) is bool,'invalid '+option+' flag')
            for params in parameters:
                require(params.get(key,False) is trial[option],option+' frozen flag mismatch')
    require(not trial.get('supported_course') or trial.get('straight_tracking_reference') is True,
            'frozen supported course requires straight tracking reference')
    if 'steering_rate_continuity_weight' in trial:
        weight=trial['steering_rate_continuity_weight']
        require(type(weight) in (float,int) and 0.<=weight<=1.,'invalid frozen steering rate continuity weight')
        for params in parameters:
            require(params.get('p6_steering_rate_continuity_weight',0.)==weight,'steering rate continuity frozen weight mismatch')
    if 'yaw_rate_tracking_weight' in trial:
        weight=trial['yaw_rate_tracking_weight']
        require(type(weight) in (float,int) and 0.<=weight<=4.,'invalid frozen yaw-rate tracking weight')
        for params in parameters:
            require(params.get('p6_yaw_rate_tracking_weight',0.)==weight,'yaw-rate tracking frozen weight mismatch')
    if 'straight_yaw_damping_weight' in trial:
        weight=trial['straight_yaw_damping_weight']
        require(type(weight) in (float,int) and 0.<=weight<=4. and
            (weight==0. or trial.get('straight_tracking_reference') is True),
            'invalid frozen straight yaw damping weight')
        for params in parameters:
            require(params.get('p7_straight_yaw_damping_weight',0.)==weight,
                'straight yaw damping frozen weight mismatch')
    for name,upper in (('turn_speed_reserve',.05),('turn_release_weight',4.)):
        if name in trial:
            value=trial[name]
            require(type(value) in (int,float) and 0.<=value<=upper and
                (value==0. or (trial.get('candidate_control_trial') is True and
                 trial.get('reference_revalidation') is True and trial.get('speed_preview') is True)),
                'invalid frozen turn timing '+name)
            for params in parameters:
                require(params.get('p7_'+name,0.)==value,'turn timing frozen mismatch: '+name)
    for rel,digest in trial['trial_files'].items():
        p=(folder/rel).resolve();require(folder in p.parents and p.is_file() and sha(p)==digest,'trial input changed: '+rel)
    if 'candidate_control_trial' in trial:
        for params in parameters:
            require(params.get('p7_candidate_control_trial',False) is trial['candidate_control_trial'],
                'candidate control trial frozen flag mismatch')
        require(trial['candidate_control_trial'] or trial.get('candidate_trial_manifest') is None,
            'candidate control trial manifest without explicit trial flag')
    if trial.get('shadow_only'):
        require(trial['algorithm']=='nmpc_shadow' and trial['nmpc_vehicle_authority'] is False and
            controller['p6_algorithm_mode']=='nmpc_shadow' and controller['p5_mode']=='legacy_slqr' and
            controller['p5_allow_test_nmpc'] is False and
            controller['p6_shadow_algorithm']==trial['computation_algorithm'],
            'shadow-only trial must keep S8 output authority')
    if trial.get('candidate_control_trial',False):
        require(not trial.get('shadow_only') and trial['algorithm']=='flat_nmpc_slqr' and
            trial['computation_algorithm']=='flat_nmpc_slqr' and
            trial['nmpc_vehicle_authority']=='requires_live_operator_lease_and_all_candidate_checks' and
            trial['reference_revalidation'] is True and trial['flat_response_base'] is True and
            trial['flat_response_near_support'] is True and trial['nominal_observation_hold'] is True and
            trial.get('flat_budgeted_poll',False) is False,
            'candidate control trial mode/strategy mismatch')
        root=Path(trial['nominal_config_directory'])
        require(not (root/'ISOLATED_ONLY.json').exists(), 'candidate control trial cannot use isolated-only Config')
        require(candidate_trial_manifest(root,trial['computation_algorithm'])==trial.get('candidate_trial_manifest'),
            'candidate control trial manifest changed; prepare a new trial')
        for params in parameters:
            require(params.get('p7_candidate_control_trial') is True and
                params.get('p7_reference_revalidation') is True and
                params.get('p7_flat_response_base') is True and
                params.get('p7_flat_response_near_support') is True and
                params.get('p7_nominal_observation_hold') is True and
                params.get('p7_flat_budgeted_poll',False) is False,
                'candidate control trial frozen worker/controller strategy mismatch')
        require(controller.get('p6_algorithm_mode')=='flat_nmpc_slqr' and
            controller.get('p5_mode')=='legacy_slqr' and controller.get('p5_allow_test_nmpc') is False,
            'candidate control trial must retain P7 single-output/lease arbitration')
        if trial.get('live_handoff'):
            accepted=trial['candidate_trial_manifest']['values'].get('software_acceptance',{})
            evidence=Path(accepted.get('path',''))
            require(evidence.is_file() and sha(evidence)==accepted.get('sha256') and
                controller.get('control_rate')==40. and controller.get('p7_async_preview') is True,
                'live handoff tested timing contract changed; prepare a new trial')
    for name,digest in trial['binaries'].items():
        require(sha(binaries()/name)==digest,'binary changed; prepare a new trial')
    for path,digest in trial.get('nominal_config_files',{}).items():
        require(Path(path).is_file() and sha(path)==digest,'Config changed; prepare a new trial: '+path)
    formal=Path(trial['source']['production_workspace'])/'src/move_car/launch/movecar_improved.launch'
    require(sha(formal)==trial['production_launch_sha256'],'formal S8 configuration changed; prepare a new trial')
    return folder,trial


def start(a):
    folder,trial=read_trial(a.trial)
    require(not (folder/'started.json').exists(),
        'trial already started: '+str(folder)+' (started.json exists). '
        'Keep this record; prepare a new run. prepare does not update your shell\'s TRIAL. '
        'Set TRIAL to the new TRIAL_DIRECTORY printed by prepare before running start; '
        'do not remove started.json or reuse this run.')
    graph=read_master(a.master_uri,'/publish_speed_turn')
    publishers=dict(graph['publishers']).get('/cmd_vel',[])
    require(not publishers,'stop the previous controller manually first; cmd_vel publishers='+repr(publishers))
    package=subprocess.check_output(['rospack','find','move_car'],text=True).strip()
    require(Path(package).resolve()==PACKAGE.resolve(),'source the independent research devel/setup.bash first')
    # Package identity alone does not prove roslaunch resolves the new executable.
    # A sourced old devel space must never launch S8 under P7 parameters.
    code='import json,roslib.packages; print(json.dumps({n:roslib.packages.find_node("move_car",n) for n in ["control_pub","p5_nmpc_worker_node"]}))'
    found=json.loads(subprocess.check_output(['/usr/bin/python2','-c',code],text=True))
    for name,digest in trial['binaries'].items():
        candidates=found.get(name) or []
        require(len(candidates)==1 and sha(Path(candidates[0]))==digest,
            'ROS executable resolution does not match prepared binary: '+name+'; source devel_p7/setup.bash')
    write_new(folder/'started.json',dict(run_id=trial['run_id'],master_uri=a.master_uri,
        preflight_publishers=publishers,automatic_arming=False,operator_initiated_start=True))
    env=os.environ.copy();env['ROS_MASTER_URI']=a.master_uri
    proc=subprocess.Popen(['roslaunch',str(folder/'trial.launch')],env=env,start_new_session=True)
    try:return proc.wait()
    except KeyboardInterrupt:
        proc.send_signal(signal.SIGINT)
        return proc.wait(timeout=30.)


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);s=p.add_subparsers(dest='op',required=True)
    q=s.add_parser('prepare');q.add_argument('--session-id',required=True);q.add_argument('--route',required=True)
    q.add_argument('--operator',required=True);q.add_argument('--imu-frame')
    q.add_argument('--mapping-evidence',type=Path)
    q.add_argument('--mapping-profile',type=Path,help='pinned real-history engineering mapping; fills gyro_link, units and inherited S8 signs')
    for name in ['forward-wheel-positive','left-steering-positive','left-yaw-positive']:
        q.add_argument('--confirm-'+name,action='store_true')
    q.add_argument('--algorithm',choices=MODES,default='nmpc_12_fixed')
    q.add_argument('--shadow-only',action='store_true',help='S8 is the sole real command source; selected algorithm computes candidates only; no NMPC selection')
    q.add_argument('--candidate-control-trial',action='store_true',
        help='explicit unvalidated supervised flat_nmpc_slqr trial; requires matching Config manifest, worker, live lease and all candidate checks; never arms automatically')
    q.add_argument('--config-root',type=Path,help='Schema 2 directory; >0.30 defaults to the independent smooth_0p4 candidate')
    q.add_argument('--speed-cap',type=float,default=.10)
    q.add_argument('--revalidate-reference',action='store_true',help='experimental current-state/current-reference nonlinear revalidation; requires matching separate candidate binary')
    q.add_argument('--flat-feasibility-recovery',action='store_true',help='original-search-first bounded flat8 restoration; requires reference revalidation and matching postrun worker')
    q.add_argument('--production-workspace',type=Path,default=PRODUCTION)
    q.add_argument('--output-root',type=Path,default=WORKSPACE/'experiment_runs')
    q=s.add_parser('start');q.add_argument('--trial',type=Path,required=True)
    q.add_argument('--master-uri',default=os.environ.get('ROS_MASTER_URI','http://localhost:11311'))
    a=p.parse_args(argv)
    try:
        if a.op=='prepare':make_trial(a)
        else:raise SystemExit(start(a))
    except ValueError as exc:
        p.error(str(exc))


if __name__=='__main__':main()
