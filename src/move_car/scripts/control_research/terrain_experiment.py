"""P4-1 matched-run comparison and immutable scene library. No controller writes."""
import hashlib
import json
import math
import re
from pathlib import Path
from .dataset import load_run,digest
from .config import TERRAINS,validate_run_id
from .analysis import stats,number


def prepare(workspace,run_id,session,batch,terrain,route,speed,direction,repetition,phase,conditions,formal_paths):
    root=Path(workspace).resolve()
    for v in (run_id,session,batch):validate_run_id(v)
    if terrain not in TERRAINS or direction not in ('forward','reverse'):raise ValueError('explicit terrain/direction required')
    if isinstance(repetition,bool) or not isinstance(repetition,int) or repetition<1:raise ValueError('positive repetition required')
    if isinstance(speed,bool) or not isinstance(speed,(int,float)) or not math.isfinite(speed) or not 0.<=speed<=.5:raise ValueError('plan outside currently documented low-speed research range; does not authorize driving')
    if not route or route=='unspecified' or not conditions or conditions=='unspecified' or not phase or phase=='unspecified':raise ValueError('route and matching conditions required')
    hashes={}
    for rel in formal_paths:
        path=(root/rel).resolve()
        if root not in path.parents or not path.is_file():raise ValueError('existing formal workspace files required')
        hashes[str(path.relative_to(root))]=digest(path)
    if not hashes:raise ValueError('explicit formal configuration provenance required')
    return dict(schema_version=1,status='collection_plan_only',run_id=run_id,continuous_session_id=session,
        experiment_batch_id=batch,terrain_label=terrain,route=route,target_speed_mps=speed,direction=direction,
        repetition=repetition,experiment_phase=phase,matching_conditions_id=conditions,
        formal_config_hashes=hashes,actual_control_mode='legacy_slqr',nmpc_authority=False,
        data_origin='unverified',calibration_contract_status='all_unverified_until_evidenced',
        plan_does_not_start_vehicle=True,
        recorder_parameters={'experiment_batch_id':batch,'continuous_session_id':session,'terrain':terrain,
            'route':route,'target_speed_mps':speed,'direction':direction,'repetition':repetition,
            'experiment_phase':phase,'matching_conditions_id':conditions,'formal_config_paths':list(hashes)},
        limitation='Operator must confirm S8 baseline identity, safe route and matching conditions; plan is not motion authorization.')


def compare_runs(run_paths):
    rows=[];excluded=[]
    required=('experiment_batch_id','route','direction','matching_conditions_id','formal_config_hashes','experiment_phase','continuous_session_id')
    seen=set();content=set()
    for path in run_paths:
        m,events,metrics=load_run(path)
        rid=m['run_id']
        if rid in seen:raise ValueError('duplicate run')
        seen.add(rid)
        fingerprint=digest(Path(path)/'events.jsonl')
        if fingerprint in content:raise ValueError('duplicate event content')
        content.add(fingerprint)
        missing=[k for k in required if not m.get(k) or m.get(k)=='unspecified']
        hashes=m.get('formal_config_hashes')
        if not isinstance(hashes,dict) or not hashes or any(not isinstance(k,str) or not isinstance(v,str) or not re.fullmatch('[0-9a-f]{64}',v) for k,v in hashes.items()):missing.append('valid_formal_sha256')
        speed=m.get('target_speed_mps')
        if isinstance(speed,bool) or not isinstance(speed,(int,float)) or not math.isfinite(speed) or not 0.<=speed<=.5:missing.append('explicit_finite_target_speed')
        repetition=m.get('repetition')
        if isinstance(repetition,bool) or not isinstance(repetition,int) or repetition<1:missing.append('positive_integer_repetition')
        if m.get('direction') not in ('forward','reverse'):missing.append('explicit_direction')
        if m.get('data_origin')!='real_vehicle':missing.append('real_vehicle_provenance')
        if m.get('actual_control_mode')!='legacy_slqr':missing.append('S8_actual_owner')
        if any((e.get('ros_type')=='move_car/ControlResearchTelemetry' and e.get('payload',{}).get('nmpc_active') is True) or
            (e.get('ros_type')=='move_car/AdaptiveControlTelemetry' and e.get('payload',{}).get('actual_owner') not in (None,'','legacy_slqr')) for e in events):
            missing.append('telemetry_owner_conflicts_with_S8_declaration')
        if missing:
            excluded.append(dict(run_id=rid,reason='unmatched_or_unverified_metadata',missing=missing));continue
        config=json.dumps(m['formal_config_hashes'],sort_keys=True)
        key=json.dumps([m[k] for k in ('experiment_batch_id','route','direction','matching_conditions_id','experiment_phase')]+
                       [round(float(m.get('target_speed_mps',0.)),6),config],sort_keys=True)
        command_metrics={}
        controller=[e.get('payload',{}) for e in events if e.get('ros_type')=='move_car/ControlResearchTelemetry']
        for phase,safe in (('normal',False),('safe_stop',True)):
            selected=[p for p in controller if p.get('safe_stop') is safe]
            command_metrics[phase]={'samples':len(selected),
                'command_acceleration_mps2':stats(p.get('command_acceleration') for p in selected),
                'command_jerk_mps3':stats(p.get('command_jerk') for p in selected),
                'actual_physical_jerk':None}
        row=dict(separated_command_metrics=command_metrics,run_id=rid,continuous_session_id=m.get('continuous_session_id'),terrain=m['terrain_label'],
            repetition=m.get('repetition'),group_key=hashlib.sha256(key.encode()).hexdigest()[:20],
            matching_metadata=dict((k,m.get(k)) for k in required),target_speed_mps=m.get('target_speed_mps'),
            metrics=metrics,manifest_sha256=digest(Path(path)/'manifest.json'),events_sha256=digest(Path(path)/'events.jsonl'))
        rows.append(row)
    groups=[]
    for key in sorted(set(r['group_key'] for r in rows)):
        rs=[r for r in rows if r['group_key']==key]
        terrains={t:[r['run_id'] for r in rs if r['terrain']==t] for t in TERRAINS}
        replicated=all(len(set(r['repetition'] for r in rs if r['terrain']==t))>=3 and
            len(set(r['continuous_session_id'] for r in rs if r['terrain']==t))>=3 for t in ('smooth','grass_bumpy','asphalt'))
        comparison={}
        metric_keys=('lateral_error_abs_m','heading_error_abs_rad','speed_error_fused_mps',
            'yaw_response_error_radps','command_steering_rate_abs_radps','compute_ms','callback_before_telemetry_ms')
        for terrain in TERRAINS:
            tr=[r for r in rs if r['terrain']==terrain];tm={}
            for name in metric_keys:
                vals=[r['metrics'][name] for r in tr]
                rms=[v['rmse'] for v in vals if number(v.get('rmse'))]
                p95=[v['p95'] for v in vals if number(v.get('p95'))]
                maximum=[v['max'] for v in vals if number(v.get('max'))]
                tm[name]=dict(valid_runs=len(rms),mean_run_rmse=sum(rms)/len(rms) if rms else None,
                    worst_run_rmse=max(rms) if rms else None,worst_run_p95=max(p95) if p95 else None,
                    worst_run_max=max(maximum) if maximum else None)
            comparison[terrain]=tm
        groups.append(dict(group_key=key,runs_by_terrain=terrains,per_terrain_run_metrics=comparison,
            repeated_three_terrain_coverage=replicated,
            independent_sessions_by_terrain={t:len(set(r['continuous_session_id'] for r in rs if r['terrain']==t)) for t in TERRAINS},
            interpretation='Matched nominal route/speed/direction/config metadata only; independent trajectory truth and terrain causality not certified'))
    return dict(schema_version=1,runs=rows,excluded_runs=excluded,matched_groups=groups,
        valid_for_descriptive_comparison=any(r['metrics']['control_samples']>0 for r in rows),terrain_causal_effect_established=False,
        nmpc_closed_loop_improvement_claim=False,production_changed=False,
        pending=['Independent physical tracking ground truth','Run/session training-validation-test separation',
                 'Operator audit of weather/load/tire/route matching and repeated-run independence'])


def scene_library(baselines,calibrations,experiments,candidates,validations):
    # References are immutable artifacts, not merged ROS parameters. Approved S8
    # releases remain an explicit fixed baseline; unknown scenes are not assigned
    # guessed best parameters. Candidate review is handled by existing registry.
    out={t:dict(verified_fixed_baseline=[],dynamics_calibrations=[],current_experiment=[],
               unapproved_candidates=[],independent_validation_history=[]) for t in TERRAINS}
    def add(entries,slot):
        for e in entries:
            t=e.get('terrain')
            if t not in TERRAINS:raise ValueError('entry requires explicit terrain')
            path=Path(e['path'])
            if not path.is_file():raise ValueError('referenced artifact missing')
            item=dict(e,sha256=digest(path))
            if slot=='verified_fixed_baseline' and not (e.get('verification_kind')=='user_accepted_S8' and e.get('actual_control_mode')=='legacy_slqr'):
                raise ValueError('only existing explicitly reviewed S8 baseline can occupy fixed baseline slot')
            if slot=='dynamics_calibrations':
                payload=json.loads(path.read_text())
                item['calibration_status']=payload.get('status','invalid')
                eligible=(payload.get('status')=='held_out_equation_validated' and payload.get('data_origin')=='real_vehicle' and
                    payload.get('run_id') and payload.get('continuous_session_id') and payload.get('validation_run') and payload.get('validation_session') and
                    payload['run_id']!=payload['validation_run'] and payload['continuous_session_id']!=payload['validation_session'] and
                    payload.get('training_rows_sha256') and payload.get('validation_rows_sha256') and
                    payload['training_rows_sha256']!=payload['validation_rows_sha256'] and payload.get('model_contract_sha256'))
                item['eligible_for_offline_model_review']=bool(eligible)
                item['available_for_model_use']=bool(eligible and e.get('model_review_approved') is True)
                item['review_scope']='held-out one-step equation; not vehicle closed-loop approval'
                item['physical_delay_identified']=payload.get('physical_delay_identified') is True
            if slot=='unapproved_candidates':item['production_approval']=False
            out[t][slot].append(item)
    add(baselines,'verified_fixed_baseline');add(calibrations,'dynamics_calibrations');add(experiments,'current_experiment')
    add(candidates,'unapproved_candidates');add(validations,'independent_validation_history')
    return dict(schema_version=1,scenes=out,production_changed=False,candidate_promotion='existing terrain_dataset registry plus explicit human review',
        cross_scene_validation_required=['smooth','grass_bumpy','asphalt','unknown'],
        limitation='Hash registry records supplied evidence; does not authenticate approval or certify vehicle performance.')
