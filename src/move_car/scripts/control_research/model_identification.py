"""P4-2 conditional SI model identification; no ROS, publisher or config writes.

Contracts are explicit operator declarations, not source authentication. A valid
fit is an offline equation candidate. It never certifies physical parameters or
closed-loop controller improvement. See P42_VEHICLE_MODEL_IDENTIFICATION.md.
"""
import bisect
import hashlib
import json
import math
from pathlib import Path
from .vehicle_calibration import (template as calibration_template, analyze_si,
    finite, summary, fit_actuator, validate_actuator, _timing_limits, _evidence, _steering_samples)

MODELS = ('speed_affine', 'longitudinal_first_order', 'yaw_gain', 'steering_actuator')
_FIELDS = ('wheel_mps', 'ground_vx_mps', 'ground_vy_mps', 'heading_rad', 'yaw_rate_rps',
           'steering_rad', 'command_speed_mps', 'command_steering_rad')
_TIMES = ('time_s', 'receipt_time_s', 'wheel_source_time_s', 'ground_source_time_s',
          'heading_source_time_s', 'yaw_source_time_s', 'steering_source_time_s',
          'command_source_time_s', 'feedback_source_time_s')
_ID_FIELDS = ('run_id', 'continuous_session_id', 'session_id', 'data_origin',
              'experiment_phase', 'limitation')


def template():
    out = calibration_template()
    out.update(model_identification_schema=1, scene='smooth',
        longitudinal_slip_negligible_verified=False, speed_scale_evidence=[],
        ground_acquisition_timestamp_verified=False,
        speed_command_units_verified=False, speed_command_semantics='unknown',
        velocity_feedback_filter_phase_verified=False,
        known_velocity_filter_delay_s=None,
        yaw_quasi_steady_verified=False, yaw_quasi_steady_evidence=[],
        nominal_speed_delay_s=.20,nominal_speed_tau_s=.20,
        nominal_command_gain=1.,nominal_speed_bias=0.,
        nominal_steering_delay_s=.325,nominal_steering_tau_s=.44,
        nominal_model_source_kind='research_types_default_unvalidated_assumption',
        nominal_model_source=_nominal_source(),
        identification_note='Unknown is invalid; evidence declarations are not authenticated by this tool.')
    return out


def _nominal_source():
    source=Path(__file__).resolve().parents[2]/'include/control/research_types.h'
    if not source.is_file():
        return dict(path='include/control/research_types.h',sha256=None,snapshot=None,
            verified_actual_runtime_parameters=False)
    raw=source.read_bytes()
    return dict(path='include/control/research_types.h',sha256=hashlib.sha256(raw).hexdigest(),
        snapshot=[line.strip() for line in raw.decode().splitlines() if 'double wheelbase=' in line or 'double speed_tau=' in line],
        verified_actual_runtime_parameters=False)


def _nominal(contract):
    keys=('nominal_speed_delay_s','nominal_speed_tau_s','nominal_command_gain','nominal_speed_bias',
        'nominal_steering_delay_s','nominal_steering_tau_s')
    if not all(finite(contract.get(k)) for k in keys):raise ValueError('explicit_nominal_model_comparison_parameters_required')
    delay,tau,gain,bias,sd,st=[float(contract[k]) for k in keys]
    if not 0<=delay<=.4 or not .1<=tau<=2. or not .5<=gain<=1.5 or not -.15<=bias<=.15 or not 0<=sd<=.4 or not .08<=st<=1.2:
        raise ValueError('nominal_model_outside_identification_domain')
    source=contract.get('nominal_model_source',{})
    if not isinstance(source,dict) or not isinstance(source.get('sha256'),str) or len(source['sha256'])!=64:
        raise ValueError('nominal_model_source_snapshot_and_hash_required')
    return delay,tau,gain,bias,sd,st


def _hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False,
                                    separators=(',', ':')).encode()).hexdigest()


def sample_fingerprints(rows, model=None):
    """Reject copied/relabelled data, including an absolute clock translation.

    Value-only SHA is deliberately conservative: exactly repeated noiseless
    experiments cannot serve as independent evidence. This does not authenticate
    arbitrary modified data or prove independence of two physical sessions.
    """
    if not rows or not finite(rows[0].get('time_s')):
        raise ValueError('missing_finite_time_s')
    origin = float(rows[0]['time_s'])
    normalized = []; values = []
    fields = _FIELDS
    if model in ('speed_affine','yaw_gain','longitudinal_first_order'):
        fields = tuple(k for k in _FIELDS if k not in ('command_steering_rad','command_speed_mps','steering_rad'))
        if model=='yaw_gain':fields += ('steering_rad',)
        if model=='longitudinal_first_order':fields += ('command_speed_mps',)
    elif model=='steering_actuator':fields=('steering_rad','command_steering_rad')
    for row in rows:
        v = {k: float(row[k]) for k in fields if finite(row.get(k))}
        t = {k: round(float(row[k])-origin, 8) for k in _TIMES if finite(row.get(k))}
        normalized.append(dict(values=v, relative_times=t)); values.append(v)
    return dict(normalized_rows_sha256=_hash(normalized), measurement_values_sha256=_hash(values))


def _contract_hash(contract):
    return _hash({k: v for k, v in contract.items() if k not in _ID_FIELDS})


def _identity(contract):
    for key in ('run_id', 'continuous_session_id'):
        value = contract.get(key)
        if not isinstance(value, str) or not value.strip() or value.startswith('REPLACE'):
            raise ValueError('explicit_run_and_complete_session_identity_required')
    if contract.get('scene') != 'smooth':
        raise ValueError('P42_supports_only_declared_smooth_scene')
    if contract.get('data_origin') not in ('real_vehicle', 'synthetic'):
        raise ValueError('explicit_real_vehicle_or_synthetic_provenance_required')


def _flags(contract, keys):
    missing = [key for key in keys if contract.get(key) is not True]
    if missing:
        raise ValueError('missing_evidence:' + ','.join(missing))


def _base(model, contract):
    return dict(schema_version=1, model=model, status='invalid', valid=False,
        run_id=contract.get('run_id'), continuous_session_id=contract.get('continuous_session_id'),
        data_origin=contract.get('data_origin', 'unverified'), scene=contract.get('scene', 'unknown'),
        parameters=None, missing_measurements=[], provenance_authenticated=False,
        evidence_scope='conditional_offline_equation_only', physical_parameter_certification=False,
        vehicle_closed_loop_improvement_claim=False, safety_or_production_config_changed=False,
        uncertainty=None,nominal_comparison=dict(parameters={k:contract.get(k) for k in
            ('nominal_speed_delay_s','nominal_speed_tau_s','nominal_command_gain','nominal_speed_bias','nominal_steering_delay_s','nominal_steering_tau_s')},
            source=contract.get('nominal_model_source'),source_kind=contract.get('nominal_model_source_kind'),
            interpretation='declared nominal comparison assumptions, not identified vehicle physics'))


def _prepared(rows, contract, model):
    _identity(contract)
    if contract.get('model_identification_schema') != 1:
        raise ValueError('model_identification_schema_1_required')
    _timing_limits(contract)
    _flags(contract, ('ground_acquisition_timestamp_verified',))
    if len(rows) < 150 or len(rows) > 3000:
        raise ValueError('require_150_to_3000_samples_per_complete_experiment_phase')
    states = analyze_si(rows, contract)
    invalid = [s for s in states['states'] if not s['rear_speed']['valid']]
    if invalid:
        raise ValueError('invalid_rear_speed:' + invalid[0]['rear_speed']['reason'])
    ts = [float(r['time_s']) for r in rows]
    if ts[-1]-ts[0] < 5. or max(b-a for a, b in zip(ts, ts[1:])) > .1+1e-9:
        raise ValueError('insufficient_duration_or_measurement_gap')
    speeds = [s['rear_speed']['value'] for s in states['states']]
    if max(abs(v) for v in speeds) > 1.5:
        raise ValueError('outside_low_speed_identification_domain_1_5_mps')
    if model == 'speed_affine':
        _flags(contract, ('longitudinal_slip_negligible_verified',))
        if not _evidence(contract.get('speed_scale_evidence')):
            raise ValueError('speed_scale_independent_reference_and_low_slip_evidence_required')
        if max(abs(float(r['yaw_rate_rps'])) for r in rows) > .05:
            raise ValueError('speed_scale_requires_straight_motion')
    elif model == 'yaw_gain':
        _flags(contract, ('steering_units_verified', 'steering_zero_bias_verified',
            'yaw_gyro_zero_verified', 'yaw_quasi_steady_verified'))
        if not _evidence(contract.get('yaw_quasi_steady_evidence')):
            raise ValueError('yaw_quasi_steady_evidence_required')
        _, skew = _timing_limits(contract)
        previous = None
        for t, row in zip(ts, rows):
            st = row.get('steering_source_time_s')
            if not finite(st) or not finite(row.get('steering_rad')):
                raise ValueError('actual_steering_source_timestamp_and_SI_angle_required')
            st = float(st)
            if previous is not None and st <= previous:
                raise ValueError('steering_source_timestamp_not_increasing')
            if abs(st-t) > skew or not -.002 <= float(row['receipt_time_s'])-st <= _timing_limits(contract)[0]:
                raise ValueError('asynchronous_stale_or_future_actual_steering')
            if abs(float(row['steering_rad'])) > .35:
                raise ValueError('outside_small_steering_identification_domain_0_35_rad')
            previous = st
        if min(speeds) < .15:
            raise ValueError('yaw_gain_low_speed_or_reverse_unobservable')
        steering = [float(r['steering_rad']) for r in rows]
        if max(abs((b-a)/(tb-ta)) for a,b,ta,tb in zip(steering, steering[1:], ts, ts[1:])) > .12:
            raise ValueError('yaw_gain_steering_transient_not_quasi_steady')
    elif model == 'longitudinal_first_order':
        _flags(contract, ('speed_command_units_verified', 'command_history_complete_verified'))
        if contract.get('speed_command_semantics') != 'target_speed_mps':
            raise ValueError('speed_command_must_be_target_speed_mps_not_acceleration_or_raw_DAC')
        if ts[-1]-ts[0] < 8.:
            raise ValueError('longitudinal_experiment_requires_8_seconds')
        if max(abs(float(r['yaw_rate_rps'])) for r in rows) > .05:
            raise ValueError('longitudinal_fit_requires_straight_motion')
    return ts, speeds, states


def _spread(values):
    mean = sum(values)/len(values)
    return math.sqrt(sum((x-mean)**2 for x in values)/len(values))


def residual_statistics(residuals):
    if not residuals:
        raise ValueError('no_prediction_pairs')
    mean = sum(residuals)/len(residuals)
    centered = [x-mean for x in residuals]
    denom = sum(x*x for x in centered)
    lag1 = sum(a*b for a,b in zip(centered, centered[1:]))/denom if denom > 1e-24 else None
    out = summary(residuals)
    out.update(mse=sum(x*x for x in residuals)/len(residuals),
        absolute=summary([abs(x) for x in residuals]), lag1_correlation=lag1,
        interpretation='Source-time equation residuals; serial correlation invalidates naive IID confidence coverage')
    return out


def _ols(x, y, intercept, weights=None):
    n = len(x)
    if n <= 3 or _spread(x) < 1e-5:
        raise ValueError('weak_regressor_excitation')
    weighted=weights is not None
    weights=[1.]*n if weights is None else weights
    if len(weights)!=n or any(not finite(w) or w<=0 for w in weights):raise ValueError('positive_finite_regression_weights_required')
    total=sum(weights)
    mx, my = sum(w*a for w,a in zip(weights,x))/total, sum(w*b for w,b in zip(weights,y))/total
    denom = sum(w*(a-mx)**2 for w,a in zip(weights,x)) if intercept else sum(w*a*a for w,a in zip(weights,x))
    gain = (sum(w*(a-mx)*(b-my) for w,a,b in zip(weights,x,y)) if intercept else sum(w*a*b for w,a,b in zip(weights,x,y)))/denom
    bias = my-gain*mx if intercept else 0.
    errors = [b-gain*a-bias for a,b in zip(x,y)]
    p = 2 if intercept else 1
    variance = sum(w*e*e for w,e in zip(weights,errors))/(n-p)
    gain_se = math.sqrt(variance/denom)
    bias_se = math.sqrt(variance*(1/total+mx*mx/denom)) if intercept else None
    scale = math.sqrt(sum(e*e for e in errors)/n)
    outliers = sum(abs(e) > max(6*scale, 1e-8) for e in errors)
    return gain, bias, errors, dict(kind='conditional_weighted_OLS_assuming_fixed_regressor_and_known_relative_precision' if weighted else 'conditional_OLS_assuming_fixed_regressor_and_IID_errors',
        statistical_coverage_validated=False,
        gain_standard_error=gain_se, bias_standard_error=bias_se,
        gain_conditional_95_interval=[gain-1.96*gain_se, gain+1.96*gain_se],
        bias_conditional_95_interval=[bias-1.96*bias_se,bias+1.96*bias_se] if intercept else None,
        serial_dependence_or_sensor_systematics_not_covered=True, residual_outlier_count=outliers)


def _command_history(rows, ts):
    ct=[]; u=[]
    for t,row in zip(ts, rows):
        if not finite(row.get('command_source_time_s')) or not finite(row.get('command_speed_mps')):
            raise ValueError('complete_command_speed_and_own_source_time_required')
        at = float(row['command_source_time_s']); val = float(row['command_speed_mps'])
        if at > t+1e-9 or t-at > .2 or (ct and at < ct[-1]):
            raise ValueError('future_old_or_reversed_command_history')
        if abs(val)>1.5:
            raise ValueError('command_outside_low_speed_identification_domain')
        if ct and at == ct[-1]:
            if val != u[-1]:
                raise ValueError('conflicting_values_for_same_command_timestamp')
        else:
            ct.append(at); u.append(val)
    if len(ct)<20 or _spread(u)<.08 or max(u)-min(u)<.3:
        raise ValueError('weak_longitudinal_command_excitation')
    return ct,u


def _integrated_input(begin, end, ct, u, delay, tau):
    """Integral of exp kernel times complete delayed command ZOH (no lookahead)."""
    j=bisect.bisect_right(ct, begin-delay)-1
    if j<0:
        return None
    at=begin; driven=0.
    while at<end:
        if j+1<len(ct) and ct[j+1]+delay<=at:
            j+=1; continue
        until=min(end, ct[j+1]+delay) if j+1<len(ct) else end
        a=math.exp(-(until-at)/tau)
        driven=a*driven+(1-a)*u[j]
        at=until
        if at<end:j+=1
    return driven


def _longitudinal_pairs(ts, speed, ct, u, delay, tau):
    x=[];y=[];pairs=[]
    for i in range(1,len(ts)):
        # All candidates use identical post-0.4 s window, not differing sample sets.
        if ts[i-1]-ts[0] < .4-1e-9:
            continue
        duration=ts[i]-ts[i-1]; a=math.exp(-duration/tau)
        driven=_integrated_input(ts[i-1],ts[i],ct,u,delay,tau)
        if driven is None:
            raise ValueError('initial_command_history_does_not_cover_delay')
        b=1-a
        x.append(driven/b); y.append((speed[i]-a*speed[i-1])/b)
        pairs.append((i,a,driven,b))
    return x,y,pairs


def _steering_errors(ts,y,ct,u,delay,tau):
    errors=[]
    for i in range(1,len(ts)):
        if ts[i-1]-ts[0]<.4:continue
        driven=_integrated_input(ts[i-1],ts[i],ct,u,delay,tau)
        if driven is None:continue
        a=math.exp(-(ts[i]-ts[i-1])/tau)
        errors.append(y[i]-(a*y[i-1]+driven))
    return errors


def _fit_longitudinal(ts, speed, ct, u):
    if _spread(speed)<.05:
        raise ValueError('weak_longitudinal_response_excitation')
    candidates=[]
    for di in range(21):
        delay=di*.02
        for ti in range(39):
            tau=.10+ti*.05
            x,y,pairs=_longitudinal_pairs(ts,speed,ct,u,delay,tau)
            gain,bias,_,uncertainty=_ols(x,y,True,weights=[b*b for _,_,_,b in pairs])
            if not .5<gain<1.5 or not -.15<bias<.15:
                continue
            errors=[speed[i]-(a*speed[i-1]+gain*driven+bias*b) for i,a,driven,b in pairs]
            mse=sum(e*e for e in errors)/len(errors)
            candidates.append((mse,delay,tau,gain,bias,errors,uncertainty))
    if not candidates:
        raise ValueError('no_longitudinal_candidate_inside_research_bounds')
    candidates.sort(key=lambda c:c[0]); best=candidates[0]
    mse,delay,tau,gain,bias,errors,uncertainty=best
    if delay<=1e-9 or delay>=.4-1e-9 or tau<=.1+1e-9 or tau>=2.-1e-9:
        raise ValueError('fit_at_grid_boundary_unresolved')
    alt_d=[c[0] for c in candidates if abs(c[1]-delay)>=.04-1e-9]
    alt_t=[c[0] for c in candidates if abs(c[2]-tau)>=.10-1e-9]
    sep_d=(min(alt_d)-mse)/max(mse,1e-12) if alt_d else 0.
    sep_t=(min(alt_t)-mse)/max(mse,1e-12) if alt_t else 0.
    if min(sep_d,sep_t)<.05:
        raise ValueError('delay_tau_gain_bias_jointly_unidentifiable_or_filter_confounded')
    near=[c for c in candidates if c[0]<=mse+max(.05*mse,1e-12)]
    uncertainty.update(delay_tau_profile_kind='5_percent_loss_set_not_statistical_confidence_interval',
        delay_profile_range_s=[min(c[1] for c in near),max(c[1] for c in near)],
        tau_profile_range_s=[min(c[2] for c in near),max(c[2] for c in near)],
        delay_grid_s=.02,tau_grid_s=.05,delay_separation_ratio=sep_d,tau_separation_ratio=sep_t,
        gain_bias_OLS_conditioned_on_selected_delay_tau=True,
        gain_bias_weighting='(1-exp(-dt/tau))^2 so least squares minimizes actual one-step speed error even for unequal sampling')
    return dict(command_gain=gain, speed_bias_mps=bias,
                apparent_speed_tau_s=tau, apparent_command_delay_s=delay),errors,uncertainty


def _fit_model(model, rows, contract):
    base=_base(model,contract)
    try:
        if model not in MODELS:
            raise ValueError('unsupported_model')
        _identity(contract)
        if contract.get('model_identification_schema')!=1:raise ValueError('model_identification_schema_1_required')
        _nominal(contract)
        if model=='steering_actuator':
            result=fit_actuator(rows,contract)
            steering_training_residual=None
            if result['status']=='candidate_fit_only':
                ts,y,ct,u=_steering_samples(rows,contract)
                steering_training_residual=residual_statistics(_steering_errors(ts,y,ct,u,result['apparent_command_to_feedback_delay_s'],result['steering_tau_s']))
            return dict(base, status=result['status'], valid=result['status']=='candidate_fit_only',
                reason=result.get('reason'), actuator_fit=result,
                missing_measurements=[] if result['status']=='candidate_fit_only' else [result.get('reason')],
                training_residual=steering_training_residual,
                effective_domain=dict(support='steering_response_only_no_speed_domain',
                    steering_rad=[min(float(r['steering_rad']) for r in rows),max(float(r['steering_rad']) for r in rows)]) if result['status']=='candidate_fit_only' else None,
                parameters=dict(apparent_command_to_feedback_delay_s=result.get('apparent_command_to_feedback_delay_s'),
                    steering_tau_s=result.get('steering_tau_s')) if result['status']=='candidate_fit_only' else None,
                uncertainty=dict(kind='bounded_grid_profile_not_statistical_CI',
                    delay_resolution_s=.02,tau_resolution_s=.04,
                    delay_profile_separation_ratio=result.get('delay_profile_separation_ratio'),
                    tau_profile_separation_ratio=result.get('tau_profile_separation_ratio')),
                **sample_fingerprints(rows,model),model_contract_sha256=_contract_hash(contract))
        ts,speed,_=_prepared(rows,contract,model)
        if model=='speed_affine':
            x=[float(r['wheel_mps']) for r in rows]
            if _spread(x)<.08 or max(x)-min(x)<.3:
                raise ValueError('weak_speed_scale_excitation')
            gain,bias,errors,uncertainty=_ols(x,speed,True)
            if not .5<gain<1.5 or not -.15<bias<.15:
                raise ValueError('speed_scale_or_bias_outside_research_bounds')
            parameters=dict(wheel_speed_scale=gain,wheel_speed_bias_mps=bias)
        elif model=='yaw_gain':
            wb=float(contract.get('wheelbase_m',.6))
            x=[v*math.tan(float(r['steering_rad']))/wb for v,r in zip(speed,rows)]
            y=[float(r['yaw_rate_rps']) for r in rows]
            if _spread(x)<.03 or max(x)<.04 or min(x)>-.04:
                raise ValueError('weak_or_one_sided_yaw_excitation')
            gain,_,errors,uncertainty=_ols(x,y,False)
            if not .5<gain<1.5:
                raise ValueError('yaw_gain_outside_research_bounds')
            parameters=dict(effective_quasi_steady_yaw_gain=gain)
        else:
            ct,u=_command_history(rows,ts)
            parameters,errors,uncertainty=_fit_longitudinal(ts,speed,ct,u)
        if uncertainty.get('residual_outlier_count',0):
            raise ValueError('outlier_dominated_residual_requires_sensor_review_no_automatic_trimming')
        domain=dict(rear_speed_mps=[min(speed),max(speed)],support='observed_training_range_not_global_model_validity',duration_s=ts[-1]-ts[0],sample_count=len(rows),
            sample_interval_s=summary([b-a for a,b in zip(ts,ts[1:])]))
        if model=='speed_affine':domain['wheel_mps']=[min(x),max(x)]
        if model=='yaw_gain':domain.update(steering_rad=[min(float(r['steering_rad']) for r in rows),max(float(r['steering_rad']) for r in rows)],
            yaw_regressor_rps=[min(x),max(x)])
        if model=='longitudinal_first_order':domain['command_speed_mps']=[min(u),max(u)]
        return dict(base,status='candidate_fit_only',valid=True,reason='requires_independent_complete_session_validation',
            parameters=parameters,training_residual=residual_statistics(errors),uncertainty=uncertainty,
            effective_domain=domain,model_contract_sha256=_contract_hash(contract),
            **sample_fingerprints(rows,model),source_time_kind='declared_verified_acquisition_time_not_receipt_substitution',
            parameter_interpretation='apparent low-speed input-to-measured response including unresolved transport/filter effects'
                if model=='longitudinal_first_order' else 'conditional calibration equation; not friction or tire slip identification')
    except (ValueError,KeyError,TypeError,OverflowError,ZeroDivisionError) as error:
        return dict(base,reason=str(error),missing_measurements=[str(error)],next_measurement='See P42_SENSOR_CALIBRATION_PROTOCOL.md; do not substitute receipt time for acquisition time')


def _predict_errors(fit, rows, contract):
    model=fit['model'];ts,speed,_=_prepared(rows,contract,model);p=fit['parameters']
    if model=='speed_affine':
        scale,bias=p['wheel_speed_scale'],p['wheel_speed_bias_mps']
        if not .5<scale<1.5 or not -.15<bias<.15:raise ValueError('invalid_training_parameters')
        errors=[v-scale*float(r['wheel_mps'])-bias for v,r in zip(speed,rows)]
        nominal=[v-float(r['wheel_mps']) for v,r in zip(speed,rows)]
    elif model=='yaw_gain':
        gain=p['effective_quasi_steady_yaw_gain']
        if not .5<gain<1.5:raise ValueError('invalid_training_parameters')
        x=[v*math.tan(float(r['steering_rad']))/float(contract.get('wheelbase_m',.6)) for v,r in zip(speed,rows)]
        errors=[float(r['yaw_rate_rps'])-gain*a for r,a in zip(rows,x)]
        nominal=[float(r['yaw_rate_rps'])-a for r,a in zip(rows,x)]
    else:
        ct,u=_command_history(rows,ts)
        delay,tau,gain,bias=[p[k] for k in ('apparent_command_delay_s','apparent_speed_tau_s','command_gain','speed_bias_mps')]
        if not all(finite(v) for v in (delay,tau,gain,bias)) or not 0<delay<.4 or not .1<tau<2 or not .5<gain<1.5 or not -.15<bias<.15:
            raise ValueError('invalid_training_parameters')
        _,_,pairs=_longitudinal_pairs(ts,speed,ct,u,delay,tau)
        errors=[speed[i]-(a*speed[i-1]+gain*driven+bias*b) for i,a,driven,b in pairs]
        nominal_delay,nominal_tau,nominal_gain,nominal_bias,_,_=_nominal(contract)
        _,_,baseline=_longitudinal_pairs(ts,speed,ct,u,nominal_delay,nominal_tau)
        nominal=[speed[i]-(a*speed[i-1]+nominal_gain*driven+nominal_bias*b) for i,a,driven,b in baseline]
    if not all(finite(v) for v in errors):raise ValueError('nonfinite_prediction')
    domain=dict(rear_speed_mps=[min(speed),max(speed)],support='observed_held_out_range_not_global_model_validity',
        duration_s=ts[-1]-ts[0],sample_count=len(rows))
    return errors,nominal,domain


def _independence(fit, rows, contract, final_test=False):
    _identity(contract)
    fingerprints=sample_fingerprints(rows,fit.get('model'))
    if fit.get('data_origin') != contract.get('data_origin'):
        raise ValueError('provenance_mismatch')
    if _contract_hash(contract) != fit.get('model_contract_sha256'):
        raise ValueError('measurement_contract_differs')
    sessions=[fit.get('continuous_session_id')]; runs=[fit.get('run_id')]
    if final_test:
        sessions.append(fit.get('validation_session')); runs.append(fit.get('validation_run'))
    if contract['continuous_session_id'] in sessions or contract['run_id'] in runs:
        raise ValueError('independent_complete_run_AND_continuous_session_required')
    for key,value in fingerprints.items():
        if value in (fit.get(key),fit.get('validation_'+key)):
            raise ValueError('copied_or_relabelled_measurements_not_independent')
    return fingerprints


def _validate_model(fit, rows, contract):
    base=_base(fit.get('model','unknown'),contract)
    try:
        if fit.get('status')!='candidate_fit_only' or fit.get('valid') is not True:
            raise ValueError('valid_candidate_fit_required')
        fingerprints=_independence(fit,rows,contract)
        if fit['model']=='steering_actuator':
            checked=validate_actuator(fit['actuator_fit'],rows,contract)
            _,_,_,_,nominal_delay,nominal_tau=_nominal(contract)
            ts,y,ct,u=_steering_samples(rows,contract)
            nominal_residual=residual_statistics(_steering_errors(ts,y,ct,u,nominal_delay,nominal_tau))
            candidate_residual=residual_statistics(_steering_errors(ts,y,ct,u,fit['parameters']['apparent_command_to_feedback_delay_s'],fit['parameters']['steering_tau_s']))
            nominal_mse=nominal_residual['mse']
            accepted=candidate_residual['mse']<=nominal_mse+1e-10
            return dict(fit,status='held_out_equation_validated' if accepted else 'validation_rejected',
                valid=accepted,reason='independent_equation_validation_only_not_vehicle_approval',
                legacy_actuator_validation_status=checked['status'],
                validation_improves_nominal=checked['held_out_mse']<.8*nominal_mse,
                actuator_fit=checked,validation_run=contract['run_id'],validation_session=contract['continuous_session_id'],
                validation_residual=candidate_residual,
                nominal_validation_residual=nominal_residual,
                validation_model_domain=dict(support='steering_response_only_no_speed_domain'),
                **{'validation_'+k:v for k,v in fingerprints.items()})
        errors,nominal,domain=_predict_errors(fit,rows,contract)
        candidate=residual_statistics(errors);baseline=residual_statistics(nominal)
        accepted=candidate['mse']<=baseline['mse']+1e-10
        return dict(fit,status='held_out_equation_validated' if accepted else 'validation_rejected',valid=accepted,
            validation_run=contract['run_id'],validation_session=contract['continuous_session_id'],
            validation_residual=candidate,nominal_validation_residual=baseline,validation_model_domain=domain,
            validation_improves_nominal=candidate['mse']<.8*baseline['mse'],
            **{'validation_'+k:v for k,v in fingerprints.items()},
            reason='independent_equation_validation_only_not_vehicle_approval',
            validation_acceptance='no larger source-time MSE than nominal + 1e-10 numerical tolerance; no safety/closed-loop criterion')
    except (ValueError,KeyError,TypeError,OverflowError,ZeroDivisionError) as error:
        return dict(base,reason=str(error),missing_measurements=[str(error)])


def _evaluate_one_step(fit, rows, contract):
    """Final held-out measured-input evaluation, common to both parameterizations.

    It intentionally does not substitute controller predictions or replay new
    commands on historical actual trajectories. Static speed/yaw equations are
    labelled static calibration, not dynamic forecasting.
    """
    base=_base(fit.get('model','unknown'),contract)
    try:
        if fit.get('status')!='held_out_equation_validated' or fit.get('valid') is not True:
            raise ValueError('independently_validated_fit_required')
        fingerprints=_independence(fit,rows,contract,final_test=True)
        if fit['model']=='steering_actuator':
            # The existing validator accepts only the original candidate state.
            candidate=dict(fit['actuator_fit'],status='candidate_fit_only')
            out=validate_actuator(candidate,rows,contract)
            ts,y,ct,u=_steering_samples(rows,contract)
            residual=residual_statistics(_steering_errors(ts,y,ct,u,fit['parameters']['apparent_command_to_feedback_delay_s'],fit['parameters']['steering_tau_s']))
        else:
            errors,_,domain=_predict_errors(fit,rows,contract);residual=residual_statistics(errors)
        return dict(base,status='final_held_out_equation_evaluated',valid=True,
            training_run=fit['run_id'],training_session=fit['continuous_session_id'],
            validation_run=fit.get('validation_run'),validation_session=fit.get('validation_session'),test_run=contract['run_id'],
            test_session=contract['continuous_session_id'],test_fingerprints=fingerprints,
            common_model_sha256=_hash(fit),shared_residual=residual,
            variants={'original12':{'optimization_decisions':12,'shared_model_residual':residual},
                      'curve8':{'optimization_decisions':8,'shared_model_residual':residual}},
            prediction_kind='one_step_measured_previous_state_and_actual_command_ZOH' if fit['model'] in ('steering_actuator','longitudinal_first_order') else 'same_time_static_calibration_equation',
            interpretation='Both share actual inputs and the SAME calibrated model; equal errors by construction do not compare optimized trajectories or closed-loop control',
            causal_live_control_certified=False, uses_historical_realized_inputs=True,
            offline_equation_origin_not_live_available_snapshot=True)
    except (ValueError,KeyError,TypeError,OverflowError,ZeroDivisionError) as error:
        return dict(base,reason=str(error),missing_measurements=[str(error)])


def _schema(result):
    """Stable artifact keys shared with the P4-2 dataset readiness checker."""
    result.update(model_kind=result.get('model'),
        training_run_id=result.get('training_run',result.get('run_id') if result.get('parameters') else None),
        training_session_id=result.get('training_session',result.get('continuous_session_id') if result.get('parameters') else None),
        validation_run_id=result.get('validation_run'), validation_session_id=result.get('validation_session'),
        training_samples_fingerprint=result.get('normalized_rows_sha256'),
        validation_samples_fingerprint=result.get('validation_normalized_rows_sha256'),
        contract_sha256=result.get('model_contract_sha256'),
        model_domain=result.get('effective_domain'), physical_delay_identified=False,
        eligible_for_model_review=(result.get('status')=='held_out_equation_validated' and
            result.get('valid') is True and result.get('data_origin')=='real_vehicle'),
        model_review_approved=False,
        confidence_kind='conditional_equation_validation_not_probability',
        confidence='held_out_equation_only' if result.get('status')=='held_out_equation_validated' else
            'candidate_only' if result.get('status')=='candidate_fit_only' else 'invalid_or_test_only')
    units={'wheel_speed_scale':'1','wheel_speed_bias_mps':'m/s','command_gain':'1','speed_bias_mps':'m/s',
        'effective_quasi_steady_yaw_gain':'1','apparent_speed_tau_s':'s','apparent_command_delay_s':'s',
        'apparent_command_to_feedback_delay_s':'s','steering_tau_s':'s'}
    result['parameter_states']={key:dict(value=value if result.get('valid') else None,
        valid=result.get('valid') is True,units=units.get(key,'unknown'),
        reason=result.get('reason'),confidence=result['confidence'])
        for key,value in (result.get('parameters') or {}).items()}
    return result


def fit_model(model, rows, contract):
    if not isinstance(contract,dict) or not isinstance(rows,list) or any(not isinstance(r,dict) for r in rows):
        return _schema(dict(_base(model,contract if isinstance(contract,dict) else {}),reason='object_contract_and_list_of_sample_objects_required'))
    return _schema(_fit_model(model,rows,contract))


def validate_model(fit, rows, contract):
    if not isinstance(fit,dict) or not isinstance(contract,dict) or not isinstance(rows,list) or any(not isinstance(r,dict) for r in rows):
        return _schema(dict(_base('unknown',{}),reason='object_fit_contract_and_list_of_sample_objects_required'))
    return _schema(_validate_model(fit,rows,contract))


def evaluate_one_step(fit, rows, contract):
    if not isinstance(fit,dict) or not isinstance(contract,dict) or not isinstance(rows,list) or any(not isinstance(r,dict) for r in rows):
        return _schema(dict(_base('unknown',{}),reason='object_fit_contract_and_list_of_sample_objects_required'))
    return _schema(_evaluate_one_step(fit,rows,contract))
