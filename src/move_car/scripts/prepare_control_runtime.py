#!/usr/bin/env python3
"""Validate P1 layers, export a SHADOW-only snapshot; never overwrite inputs."""
import argparse
import hashlib
import json
import os
import yaml
from control_research.config import load_config, TERRAINS, UniqueLoader, BASELINE, finite_number

BOOLS = ('wheel_calibrated','steering_calibrated','ground_calibrated','ground_frame_verified',
         'ground_independent','ground_heading_verified','ground_lever_arm_verified',
         'imu_frame_verified','identify_yaw','identify_steering')
OPTIONAL_BOOLS=('source_timestamps_verified','steering_delay_verified')
BOUNDS = {'wheel_scale':(.0005,.002),'wheel_sign':(-1.,1.),'steering_scale':(.8,1.2),
          'steering_sign':(-1.,1.),'ground_scale':(.8,1.2),'ground_lever_x':(-2.,2.),
          'ground_lever_y':(-2.,2.),'gyro_scale':(.8,1.2),'gyro_sign':(-1.,1.),
          'accel_scale':(.8,1.2),'stale_after':(.05,.5),'sync_tolerance':(.005,.15)}
MODES = ('legacy_slqr','nmpc_fixed','nmpc_model_adaptive','nmpc_weight_adaptive',
         'nmpc_full_adaptive','auto_fallback')

def prepare(root, calibration, mode='legacy_slqr', terrain='unknown'):
    if mode not in MODES or terrain not in TERRAINS:
        raise ValueError('unknown mode/terrain')
    layers = {t:load_config(root,t) for t in TERRAINS}
    with open(calibration,'rb') as f:
        raw=f.read();cal=yaml.load(raw,Loader=UniqueLoader)
    required_fields=set(('schema_version','kind','baseline_version','status','evidence','values'))
    if (not isinstance(cal,dict) or not required_fields<=set(cal) or set(cal)-required_fields-set(('frame_contract',)) or
        cal['schema_version']!=1 or cal['kind']!='sensor_calibration' or cal['baseline_version']!=BASELINE or
        cal['status'] not in ('unverified','calibrated') or not isinstance(cal['evidence'],list) or not cal['evidence']):
        raise ValueError('invalid calibration schema/evidence')
    v=cal['values']
    if not isinstance(v,dict):raise ValueError('calibration values')
    for k in OPTIONAL_BOOLS:v.setdefault(k,False)
    if set(v)!=set(BOOLS)|set(BOUNDS)|set(OPTIONAL_BOOLS):raise ValueError('calibration fields')
    for k in BOOLS+OPTIONAL_BOOLS:
        if type(v[k]) is not bool:raise ValueError('boolean required: '+k)
        if cal['status']=='unverified' and k not in ('identify_yaw','identify_steering') and v[k]:
            raise ValueError('calibration claim needs status=calibrated and evidence: '+k)
    for k,(lo,hi) in BOUNDS.items():
        if not finite_number(v[k]) or not lo<=v[k]<=hi:raise ValueError('calibration bounds: '+k)
        if k.endswith('_sign') and v[k] not in (-1.,1.):raise ValueError('sign must be +/-1')
    if v['sync_tolerance']>v['stale_after']:raise ValueError('sync tolerance exceeds freshness')
    frames=cal.get('frame_contract',{'ground_twist_convention':'unverified','ground_world_frame':'',
                                  'ground_body_frame':'','imu_body_frame':''})
    if not isinstance(frames,dict) or set(frames)!=set(('ground_twist_convention','ground_world_frame','ground_body_frame','imu_body_frame')):
        raise ValueError('frame contract fields')
    if frames['ground_twist_convention'] not in ('unverified','world_enu'):
        raise ValueError('NED/body twist needs a calibrated upstream adapter; supported observer input is world ENU')
    for k in ('ground_world_frame','ground_body_frame','imu_body_frame'):
        if not isinstance(frames[k],str) or len(frames[k])>128 or any(x.isspace() for x in frames[k]):raise ValueError('invalid frame name')
    if frames['ground_twist_convention']=='world_enu' and (not v['ground_frame_verified'] or
        not frames['ground_world_frame'] or not frames['ground_body_frame']):raise ValueError('ENU contract requires calibration and explicit frames')
    values=dict(layers['unknown']['values'])
    if layers['unknown']['schema_version']==2:
        return prepare_p6(layers, raw, v, frames, mode, terrain)
    profiles={t:{k:layers[t]['values'][k] for k in ('speed_cap','lateral_weight_scale','heading_weight_scale',
              'speed_weight_scale','smoothness_weight_scale')} for t in TERRAINS}
    if profiles['unknown']['speed_cap']>.1:raise ValueError('unknown cap must be <= .1 m/s')
    hashes={t:layers[t]['config_hash'] for t in TERRAINS}
    hashes['calibration']=hashlib.sha256(raw).hexdigest()
    snapshot={'schema_version':1,'deployment':'shadow_only','mode':mode,'terrain':terrain,
              'values':values,'profiles':profiles,'sensors':v,'frame_contract':frames,'source_hashes':hashes,
              'terrain_estimation_enabled':True,'use_estimated_terrain':False,
              'feature_minimum_confidence':0.60,
              'model_adaptation_enabled':True,'weight_adaptation_enabled':True}
    snapshot['config_hash']=hashlib.sha256(json.dumps(snapshot,sort_keys=True).encode('utf8')).hexdigest()
    return {'research_runtime':snapshot}

def prepare_p6(layers, calibration_raw, sensors, frames, mode, terrain):
    """Same audited loader, explicit isolated schema. Never accepted as P5/v1.

    No calibration flag is promoted here. Missing evidence leaves the relevant
    channel disabled even if a caller selects an adaptive research mode.
    """
    if any(x['schema_version']!=2 for x in layers.values()):
        raise ValueError('mixed P6 profile schemas')
    profiles={t:{'values':dict(layers[t]['values']),
                 'version':layers[t]['layers']['terrain']['version'],
                 'config_hash':layers[t]['config_hash'],
                 'response_policy':layers[t]['layers']['terrain']['response_policy'],
                 'metadata':dict(layers[t]['layers']['base']['parameters'],
                                 **layers[t]['layers']['terrain']['parameters'])}
              for t in TERRAINS}
    if profiles['unknown']['values']['speed_cap']>.1:
        raise ValueError('unknown cap must be <= .1 m/s')
    adaptation=layers['unknown']['layers']['adaptation']
    hashes={t:layers[t]['config_hash'] for t in TERRAINS}
    hashes['calibration']=hashlib.sha256(calibration_raw).hexdigest()
    snapshot={'schema_version':2, 'deployment':'isolated_simulation_only',
              'mode':mode, 'terrain':terrain, 'profiles':profiles,
              'values':dict(layers[terrain]['values']), 'sensors':sensors,
              'frame_contract':frames, 'source_hashes':hashes,
              'channel_policies':adaptation['channel_policies'],
              'dynamic_conditions':adaptation['dynamic_conditions'],
              'scene_transition':adaptation['scene_transition'],
              'weight_adaptation_enabled':False,
              'real_vehicle_control_allowed':False,
              'parameter_states':{'nominal_config':'research_unvalidated',
                                  'runtime_estimate':'unobserved',
                                  'effective_parameters':'nominal_only',
                                  'candidate_config':'not_promoted',
                                  'validated_config':None}}
    # ROS1 XMLRPC cannot encode null. Keep explicit unknown/evidence flags, omit
    # absent numeric thresholds instead of manufacturing zero measurements.
    def ros_values(value):
        if isinstance(value,dict):return {k:ros_values(v) for k,v in value.items() if v is not None}
        if isinstance(value,list):return [ros_values(v) for v in value]
        return value
    snapshot=ros_values(snapshot)
    snapshot['parameter_states']['validated_config']={'status':'not_validated'}
    snapshot['config_hash']=hashlib.sha256(json.dumps(snapshot,sort_keys=True).encode('utf8')).hexdigest()
    return {'research_runtime':snapshot}

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config-dir',required=True);p.add_argument('--calibration')
    p.add_argument('--mode',choices=MODES,default='legacy_slqr')
    p.add_argument('--terrain',choices=TERRAINS,default='unknown');p.add_argument('--output',required=True)
    a=p.parse_args();out=prepare(a.config_dir,a.calibration or os.path.join(a.config_dir,'sensor_calibration.yaml'),a.mode,a.terrain)
    # Exclusive create: no production config or existing snapshot can be overwritten.
    with open(a.output,'x') as f:yaml.safe_dump(out,f,default_flow_style=False,sort_keys=True)
    print(out['research_runtime']['config_hash'])
if __name__=='__main__':main()
