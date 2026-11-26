"""Strict, offline-only layered configuration. Compatible with Python 2/3."""
from __future__ import division
import hashlib
import json
import math
import os
import re
import yaml

TERRAINS = ('smooth', 'grass_bumpy', 'asphalt', 'unknown')
# A: an audited snapshot of current launch constraints, NOT certified hardware ratings.
# Exact anchors intentionally cannot be loosened by editing YAML or a candidate.
SAFETY = {'max_speed': (0.50, 'm/s'), 'max_steer': (0.38, 'rad'),
          'solver_max_steer': (0.32, 'rad'), 'max_steer_rate': (1.0, 'rad/s'),
          'max_accel': (0.28, 'm/s^2'), 'max_decel': (0.36, 'm/s^2'),
          'max_jerk': (0.80, 'm/s^3'), 'collision_stop_distance': (0.50, 'm'),
          'follow_distance_min': (0.50, 'm'), 'reference_hard_timeout': (1.20, 's')}
# key -> class, units, hard lower/upper bounds; YAML may narrow, never widen.
CATALOG = {
 'wheelbase': ('B','m',0.60,0.60), 'rear_to_center': ('B','m',0.30,0.30),
 'wheel_speed_scale': ('B','m/s per raw',0.001,0.001),
 'steering_tau': ('B','s',0.05,1.0), 'steering_delay': ('B','s',0.,0.70),
 'yaw_gain': ('B','1',0.5,1.5),
 'control_rate': ('C','Hz',30.,30.),
 'prediction_dt': ('C','s',0.05,0.20), 'horizon_steps': ('C','1',10,40),
 'control_knots': ('C','1',4,10), 'solver_max_iterations': ('C','1',1,5),
 'solver_budget_ms': ('C','ms',1.,12.),
 'weight_lateral': ('C','1',1.,100.), 'weight_heading': ('C','1',1.,100.),
 'weight_speed': ('C','1',1.,40.), 'weight_smoothness': ('C','1',1.,100.),
 'speed_cap': ('C','m/s',0.,0.30),
 'lateral_weight_scale': ('C','1',0.75,1.50),
 'heading_weight_scale': ('C','1',0.75,1.50),
 'speed_weight_scale': ('C','1',0.50,1.25),
 'smoothness_weight_scale': ('C','1',1.,2.),
 'yaw_gain_max_change': ('B','1',0.,0.25),
 'tau_max_change': ('B','s',0.,0.15),
 'filter_tau': ('C','s',1.,10.), 'min_confidence': ('C','1',0.7,1.),
 'switch_dwell': ('C','s',1.,10.), 'switch_hysteresis': ('C','1',0.1,0.4)}
LAYERS = {
 'base': set(('wheelbase','rear_to_center','wheel_speed_scale','steering_tau','steering_delay',
              'yaw_gain','control_rate','prediction_dt','horizon_steps','control_knots',
              'solver_max_iterations','solver_budget_ms','weight_lateral','weight_heading',
              'weight_speed','weight_smoothness')),
 'terrain': set(('speed_cap','lateral_weight_scale','heading_weight_scale','speed_weight_scale','smoothness_weight_scale')),
 'adaptation': set(('yaw_gain_max_change','tau_max_change','filter_tau','min_confidence','switch_dwell','switch_hysteresis')),
 'safety': set(SAFETY)}
BASELINE = '695f1665feec86fd82e97f82c42fa85930c6bef5'

class UniqueLoader(yaml.SafeLoader):
    pass

def unique_mapping(loader, node, deep=False):
    result = {}
    for key, value in node.value:
        k = loader.construct_object(key, deep=deep)
        if k in result:
            raise ValueError('duplicate YAML key: '+str(k))
        result[k] = loader.construct_object(value, deep=deep)
    return result
UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping)

def finite_number(value):
    return not isinstance(value, bool) and isinstance(value, (int, float)) and not (math.isnan(value) or math.isinf(value))

def read_layer(path, kind, terrain='all'):
    with open(path, 'rb') as f:
        raw = f.read()
    d = yaml.load(raw, Loader=UniqueLoader)
    if isinstance(d, dict) and d.get('schema_version') == 2:
        return _read_layer_v2(d, raw, path, kind, terrain)
    required = set(('schema_version','kind','terrain','baseline_version','status','evidence',
                    'valid_range','parameters','features'))
    if not isinstance(d, dict) or set(d) != required:
        raise ValueError('invalid top-level schema: '+path)
    if type(d['schema_version']) is not int or d['schema_version'] != 1 or d['kind'] != kind or d['terrain'] != terrain:
        raise ValueError('schema/kind/terrain mismatch: '+path)
    if d['baseline_version'] != BASELINE or d['status'] != 'research_unvalidated':
        raise ValueError('unsupported baseline/status: '+path)
    if not isinstance(d['evidence'], list) or not d['evidence']:
        raise ValueError('evidence required: '+path)
    if d['valid_range'] != {'direction': 'forward', 'max_speed_mps': 0.5,
                             'deployment': 'offline_or_shadow_only'}:
        raise ValueError('unsupported operating range: '+path)
    if d['features'] != {'nmpc_enabled': False, 'terrain_estimation_enabled': False,
                         'adaptation_enabled': False} or any(type(v) is not bool for v in d['features'].values()):
        raise ValueError('P1 features must remain disabled: '+path)
    if not isinstance(d['parameters'], dict) or set(d['parameters']) != LAYERS[kind]:
        raise ValueError('missing/unknown parameters: '+path)
    values = {}
    for name, entry in d['parameters'].items():
        if not isinstance(entry, dict) or set(entry) != set(('value','unit','class','bounds','basis')):
            raise ValueError('invalid parameter entry: '+name)
        if kind == 'safety':
            val, unit = SAFETY[name]; cls, lo, hi = 'A', val, val
        else:
            cls, unit, lo, hi = CATALOG[name]
        bounds = entry['bounds']; val = entry['value']
        if (entry['class'] != cls or entry['unit'] != unit or not entry['basis'] or
            not isinstance(bounds, list) or len(bounds) != 2 or
            not all(finite_number(x) for x in bounds+[val]) or
            not lo <= bounds[0] <= val <= bounds[1] <= hi):
            raise ValueError('class/unit/bounds/value rejected: '+name)
        if name in ('horizon_steps','control_knots','solver_max_iterations') and type(val) is not int:
            raise ValueError('integer required: '+name)
        values[name] = val
    return d, values, hashlib.sha256(raw).hexdigest()

def load_config(root, terrain='unknown'):
    if terrain not in TERRAINS:
        raise ValueError('unsupported terrain')
    layers = {}; values = {}; hashes = {}
    for rel, kind, scene in [('base.yaml','base','all'),('safety.yaml','safety','all'),
                             ('terrain/'+terrain+'.yaml','terrain',terrain),
                             ('adaptation.yaml','adaptation','all')]:
        d, v, digest = read_layer(os.path.join(root, rel), kind, scene)
        layers[kind] = d; values.update(v); hashes[rel] = digest
    versions = set(d['schema_version'] for d in layers.values())
    if len(versions) != 1:
        raise ValueError('mixed configuration schema versions')
    if values['control_knots'] > values['horizon_steps']:
        raise ValueError('control knots exceed horizon')
    fingerprint = hashlib.sha256(json.dumps(hashes, sort_keys=True).encode('utf8')).hexdigest()
    return {'schema_version':versions.pop(), 'terrain':terrain, 'values':values, 'layers':layers,
            'file_hashes':hashes, 'config_hash':fingerprint, 'applied_to_controller':False}

# Schema 2 is normalized through the SAME load_config entry point. Schema 1's
# validator and merge semantics above are intentionally unchanged.
P6_MODEL_KEYS = set(('steering_tau', 'steering_delay', 'steering_gain',
                     'speed_tau', 'speed_delay', 'speed_gain', 'yaw_gain'))
P6_CATALOG = dict(CATALOG)
P6_CATALOG.update({
    'solver_budget_ms': ('C', 'ms', 1.0, 8.0),
    'steering_gain': ('B', '1', 0.5, 1.5),
    'speed_gain': ('B', '1', 0.5, 1.5),
    'speed_tau': ('B', 's', 0.05, 1.0),
    'speed_delay': ('B', 's', 0.0, 0.70)})
P6_SAFETY = dict(SAFETY, max_lateral_accel=(0.25, 'm/s^2'))
P6_LAYERS = dict((k, set(v)) for k, v in LAYERS.items())
P6_LAYERS['base'].update(P6_MODEL_KEYS)
P6_LAYERS['terrain'].update(P6_MODEL_KEYS)
# v1's six legacy adaptation knobs do not describe the new per-channel
# estimators. Do not export unused scalar knobs as apparently active controls.
P6_LAYERS['adaptation'] = set()
P6_LAYERS['safety'] = set(P6_SAFETY)
P6_CHANNELS = ('steering', 'yaw_gain', 'yaw_disturbance', 'longitudinal', 'roughness', 'slip')
P6_DYNAMIC_FLAGS = ('yaw_underresponse', 'steering_lag', 'longitudinal_weak',
                    'vibration_high', 'low_confidence')
P6_POLICY_NUMERIC = ('stale_s','sync_s','min_speed','min_excitation','min_gain','max_gain',
    'min_tau','max_tau','gain_rate','tau_rate','return_rate','forgetting_s','filter_s',
    'min_information','min_rank_ratio','max_innovation','max_covariance',
    'prediction_improvement','min_confidence','residual_bound','residual_rate','residual_filter_s',
    'condition_enter_fraction','condition_exit_fraction','condition_dwell_s','min_samples')
P6_POLICY_BOOLS = ('enabled','delay_verified','bias_verified','timing_verified','allow_receipt_identification')

def _read_layer_v2(d, raw, path, kind, terrain):
    fields = set(('schema_version', 'kind', 'terrain', 'baseline_version', 'status',
                  'evidence', 'valid_range', 'parameters', 'features', 'version'))
    if kind == 'adaptation':
        fields.update(('channel_policies', 'dynamic_conditions', 'weights_online_enabled', 'scene_transition'))
    if kind == 'terrain':fields.add('response_policy')
    if set(d) != fields or type(d['schema_version']) is not int or d['kind'] != kind or d['terrain'] != terrain:
        raise ValueError('schema2 top-level/kind/terrain mismatch: '+path)
    if d['baseline_version'] != BASELINE or d['status'] != 'research_unvalidated' or not isinstance(d['version'], str) or not d['version']:
        raise ValueError('schema2 baseline/status/version: '+path)
    if not isinstance(d['evidence'], list) or not d['evidence'] or not all(isinstance(x, str) and x for x in d['evidence']):
        raise ValueError('schema2 evidence required: '+path)
    if d['valid_range'] != {'direction':'forward', 'max_speed_mps':0.5, 'deployment':'isolated_simulation_only'}:
        raise ValueError('schema2 deployment must stay isolated: '+path)
    if d['features'] != {'nmpc_enabled':False, 'terrain_estimation_enabled':False, 'adaptation_enabled':False} or any(type(x) is not bool for x in d['features'].values()):
        raise ValueError('schema2 real-vehicle features must remain disabled: '+path)
    if not isinstance(d['parameters'], dict) or set(d['parameters']) != P6_LAYERS[kind]:
        raise ValueError('schema2 missing/unknown parameters: '+path)
    values = {}
    for name, entry in d['parameters'].items():
        required = set(('value', 'unit', 'class', 'bounds', 'basis', 'source',
                        'calibration_status', 'uncertainty', 'valid_speed_range'))
        if not isinstance(entry, dict) or set(entry) != required:
            raise ValueError('schema2 parameter metadata: '+name)
        if kind == 'safety':
            anchor, unit = P6_SAFETY[name]; cls, lo, hi = 'A', anchor, anchor
        else:
            cls, unit, lo, hi = P6_CATALOG[name]
            # Only Schema 2 smooth research candidates may request 0.40m/s.
            # Schema 1 and all other scenes retain their original catalog.
            if name=='speed_cap' and kind=='terrain' and terrain=='smooth':hi=.4
        val, bounds = entry['value'], entry['bounds']
        if (entry['class'] != cls or entry['unit'] != unit or not entry['basis'] or
            not isinstance(bounds, list) or len(bounds) != 2 or
            not all(finite_number(x) for x in bounds+[val]) or
            not lo <= bounds[0] <= val <= bounds[1] <= hi):
            raise ValueError('schema2 class/unit/bounds/value: '+name)
        if name in ('horizon_steps','control_knots','solver_max_iterations') and type(val) is not int:
            raise ValueError('integer required: '+name)
        source = entry['source']; domain = entry['valid_speed_range']; uncertainty = entry['uncertainty']
        if (not isinstance(source, dict) or set(source) != set(('kind','reference')) or
            source['kind'] not in ('source_audit','synthetic','real_calibration') or
            not isinstance(source['reference'],str) or not source['reference']):
            raise ValueError('schema2 parameter source: '+name)
        if entry['calibration_status'] not in ('unverified','synthetic_validated','real_validated'):
            raise ValueError('schema2 calibration status: '+name)
        if entry['calibration_status']=='real_validated':
            # A text claim cannot promote a research candidate. Promotion must
            # be done by a separate reviewed artifact bound to evidence hashes.
            raise ValueError('real calibration promotion requires external review: '+name)
        if entry['calibration_status']=='synthetic_validated' and source['kind']!='synthetic':
            raise ValueError('synthetic status requires synthetic source: '+name)
        if (not isinstance(domain,list) or len(domain)!=2 or
            not all(finite_number(x) for x in domain) or not 0. <= domain[0] < domain[1] <= 0.5):
            raise ValueError('schema2 speed domain: '+name)
        if (not isinstance(uncertainty,dict) or set(uncertainty)!=set(('known','stddev')) or
            type(uncertainty['known']) is not bool or
            (uncertainty['known'] and (not finite_number(uncertainty['stddev']) or uncertainty['stddev']<0.)) or
            (not uncertainty['known'] and uncertainty['stddev'] is not None)):
            raise ValueError('schema2 uncertainty must distinguish unknown: '+name)
        values[name] = val
    if kind == 'adaptation':
        _validate_p6_policies(d)
    if kind == 'terrain':
        p=d['response_policy']
        if (not isinstance(p,dict) or set(p)!=set(('allowed_channels','evidence_kind','evidence_sha','minimum_confidence','low_confidence_strategy')) or
            not isinstance(p['allowed_channels'],list) or len(set(p['allowed_channels']))!=len(p['allowed_channels']) or
            set(p['allowed_channels'])-set(P6_CHANNELS) or p['low_confidence_strategy']!='bounded_return_to_nominal'):
            raise ValueError('scene response policy shape')
        if p['evidence_kind']=='missing_evidence':
            if p['allowed_channels'] or p['evidence_sha'] is not None or p['minimum_confidence'] is not None:
                raise ValueError('unvalidated scene cannot activate parameter updates')
        elif (p['evidence_kind']!='synthetic_validation' or not isinstance(p['evidence_sha'],str) or
              not re.match(r'^[a-f0-9]{64}\Z',p['evidence_sha']) or not finite_number(p['minimum_confidence']) or not 0.<p['minimum_confidence']<=1.):
            raise ValueError('scene response policy evidence')
        if terrain=='unknown' and p['allowed_channels']:raise ValueError('unknown scene freezes adaptation')
    return d, values, hashlib.sha256(raw).hexdigest()

def _validate_p6_policies(d):
    if d['weights_online_enabled'] is not False:
        raise ValueError('online performance weights are frozen')
    transition=d['scene_transition']
    if (not isinstance(transition,dict) or set(transition)!=set(('enabled','ramp_time_s','dwell_s',
        'confidence_hysteresis','evidence_kind','evidence_reference')) or type(transition['enabled']) is not bool or
        not all(finite_number(transition[k]) for k in ('ramp_time_s','dwell_s','confidence_hysteresis')) or
        transition['ramp_time_s']<=0. or transition['dwell_s']<0. or not 0.<=transition['confidence_hysteresis']<=1. or
        not isinstance(transition['evidence_reference'],str) or not transition['evidence_reference']):
        raise ValueError('scene transition policy malformed')
    if transition['evidence_kind']=='inherited_unvalidated_design_seed':
        if transition['enabled'] or transition['ramp_time_s']!=3. or transition['dwell_s']!=2. or transition['confidence_hysteresis']!=.15:
            raise ValueError('unvalidated scene transition remains disabled')
    elif transition['evidence_kind']!='synthetic_validation' or not re.match(r'^sha256:[a-f0-9]{64}\Z',transition['evidence_reference']):
        raise ValueError('scene transition activation needs synthetic test evidence')
    policies=d['channel_policies']
    if not isinstance(policies,dict) or set(policies)!=set(P6_CHANNELS):
        raise ValueError('schema2 residual channels')
    numeric=P6_POLICY_NUMERIC
    for name,p in policies.items():
        if (not isinstance(p,dict) or set(p)!=set(numeric+P6_POLICY_BOOLS+('evidence_kind','evidence_sha','reason')) or
            any(type(p[k]) is not bool for k in P6_POLICY_BOOLS) or p['evidence_kind'] not in ('missing_evidence','synthetic_validation') or
            not isinstance(p['reason'],str) or not p['reason']):
            raise ValueError('schema2 channel policy: '+name)
        if p['evidence_kind']=='missing_evidence':
            if any(p[k] for k in P6_POLICY_BOOLS) or p['evidence_sha'] is not None or any(p[k] is not None for k in numeric):
                raise ValueError('missing evidence must disable update: '+name)
        else:
            if not isinstance(p['evidence_sha'],str) or not re.match(r'^[a-f0-9]{64}\Z',p['evidence_sha']) or any(not finite_number(p[k]) or p[k]<0. for k in numeric):
                raise ValueError('synthetic policy evidence/numeric: '+name)
            if (p['filter_s']<=0. or p['forgetting_s']<=0. or p['stale_s']<=0. or
                p['sync_s']>p['stale_s'] or p['min_gain']<=0. or p['min_gain']>p['max_gain'] or
                p['min_tau']<=0. or p['min_tau']>p['max_tau'] or p['min_samples']<2 or
                type(p['min_samples']) is not int or not 0.<p['min_rank_ratio']<=1. or
                not 0.<p['min_confidence']<=1. or
                p['condition_exit_fraction']>p['condition_enter_fraction']):
                raise ValueError('synthetic policy domain: '+name)
        if name=='slip' and p['enabled']:
            raise ValueError('slip requires separately reviewed independent-source contract')
    if policies['yaw_gain']['enabled'] and policies['yaw_disturbance']['enabled']:
        raise ValueError('free yaw gain and additive yaw compensation cannot update together')
    dynamic=d['dynamic_conditions']
    if (not isinstance(dynamic,dict) or set(dynamic)!=set(('composition','flags','classification_status')) or
        dynamic['composition']!='independent_flags' or dynamic['flags']!=list(P6_DYNAMIC_FLAGS) or
        dynamic['classification_status']!='unvalidated'):
        raise ValueError('dynamic conditions must remain composable research flags')

def validate_run_id(value):
    if not re.match(r'^[A-Za-z0-9][A-Za-z0-9_-]{0,79}\Z', value):
        raise ValueError('run_id must contain 1..80 letters, digits, underscore or hyphen')
    return value
