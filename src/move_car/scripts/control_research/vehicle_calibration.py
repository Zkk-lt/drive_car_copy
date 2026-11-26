"""Passive P4-1 calibration diagnostics; SI input, explicit evidence gates, no ROS."""
import bisect
import hashlib
import json
import math
from collections import defaultdict


def estimate(value=None, valid=False, reason='unobservable', confidence=0., units='', **extra):
    return dict(value=value if valid else None, valid=valid, reason=reason,
                confidence=confidence if valid else 0., units=units,
                confidence_kind='evidence_gate_heuristic_not_probability', **extra)


def finite(value):
    if isinstance(value,bool):return False
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def summary(values):
    values = sorted(float(x) for x in values if finite(x))
    if not values:
        return dict(count=0, mean=None, p50=None, p95=None, p99=None, maximum=None, rms=None)
    def p(q):
        z=(len(values)-1)*q;lo=int(z);hi=min(lo+1,len(values)-1)
        return values[lo]+(values[hi]-values[lo])*(z-lo)
    return dict(count=len(values), mean=sum(values)/len(values), p50=p(.5), p95=p(.95),
                p99=p(.99), maximum=values[-1], rms=math.sqrt(sum(x*x for x in values)/len(values)))


def template():
    return dict(schema_version=1, data_origin='unverified', run_id='REPLACE_RUN',
        continuous_session_id='REPLACE_SESSION', experiment_phase='stationary',
        units='SI', velocity_frame='unknown', heading_frame='unknown',
        wheel_units_verified=False, steering_units_verified=False, steering_zero_bias_verified=False,
        ground_independent=False, ground_independence_evidence=[],
        body_heading_independent=False, coordinate_transform_verified=False, planar_motion_verified=False,
        yaw_gyro_zero_verified=False,
        lever_arm_verified=False, rear_axle_to_sensor_m=[None,None],
        imu_body_transform_verified=False, gravity_vector_body_mps2=None,
        stationary_verified=False, source_time_verified=False,
        source_time_evidence=[], source_age_limit_s=.10, synchronized_max_skew_s=.02,
        command_timestamp_at_actuator_verified=False, command_history_complete_verified=False,
        feedback_timestamp_acquisition_verified=False, feedback_filter_phase_verified=False,
        delay_evidence=[], known_feedback_filter_delay_s=None,
        wheelbase_m=.6, low_speed_mps=.15,
        limitation='Declarations require independent calibration evidence; this tool does not certify authenticity.')


def audit_events(events, manifest):
    topics=defaultdict(list)
    for e in events:
        topics[e.get('topic','unknown')].append(e)
    report={}
    for topic, samples in sorted(topics.items()):
        receipts=[e.get('receipt_monotonic_ns') for e in samples]
        deltas=[(b-a)*1e-6 for a,b in zip(receipts,receipts[1:]) if finite(a) and finite(b)]
        ages=[]; sources=[]; values=[]; frames=set(); quaternions=[]
        for e in samples:
            src=e.get('source_stamp_ns');receipt=e.get('receipt_ros_ns')
            if finite(src) and src and finite(receipt):ages.append((receipt-src)*1e-9);sources.append(src)
            p=e.get('payload',{});frames.add(e.get('frame_id',''))
            values.append(json.dumps(p,sort_keys=True))
            if e.get('ros_type')=='nav_msgs/Odometry':
                q=p.get('pose',{}).get('pose',{}).get('orientation',{})
                quaternions.append(sum(float(q.get(k,0.))**2 for k in ('x','y','z','w')))
        repeated=sum(a==b for a,b in zip(values,values[1:]))
        report[topic]=dict(count=len(samples),receipt_interval_ms=summary(deltas),
            header_to_ros_receipt_age_s=summary(ages), missing_source_stamp=len(samples)-len(sources),
            duplicate_source_stamps=len(sources)-len(set(sources)),
            source_not_increasing=sum(b<=a for a,b in zip(sources,sources[1:])),
            repeated_payload_pairs=repeated,frames=sorted(frames),
            invalid_odom_quaternions=sum(not .9<=q<=1.1 for q in quaternions),
            acquisition_freshness=estimate(reason='hardware_acquisition_time_or_sequence_not_recorded'),
            interpretation='Repeated payload may be stationary or stale cache; neither inferred. Header age may be driver receipt age.')
    return dict(schema_version=1,run_id=manifest['run_id'],data_origin=manifest.get('data_origin','unverified'),
        topics=report,physical_calibration_status='invalid',
        reasons=['NED independence and coordinate/extrinsic evidence required',
                 'Headerless CAN cannot establish acquisition freshness',
                 'No automatic time alignment, body-heading substitution, or slip reconstruction from mixed raw sources'],
        known_aliases={'/carvel':'same cached wheel/steering source as /car_message; angular.z is steering rad'})


def _checked_rows(rows, contract):
    if len(rows)<2:
        raise ValueError('at least two SI samples required')
    if contract.get('units')!='SI':
        raise ValueError('explicit calibrated SI units required')
    times=[float(r['time_s']) for r in rows]
    if any(not finite(x) for x in times) or any(b<=a for a,b in zip(times,times[1:])):
        raise ValueError('nonfinite/repeated/out-of-order sample time')
    return times


def _evidence(value):
    return isinstance(value,list) and bool(value) and all(isinstance(v,str) and v.strip() and not v.startswith('REPLACE') for v in value)


def _timing_limits(contract):
    # Research tools may tighten these checks, never silently enlarge them.
    age=float(contract.get('source_age_limit_s',.10))
    skew=float(contract.get('synchronized_max_skew_s',.02))
    if not finite(age) or not .002<=age<=.10 or not finite(skew) or not .001<=skew<=.02:
        raise ValueError('invalid_or_relaxed_source_time_limits')
    return age,skew


def _row_fingerprint(rows):
    # Content guard supplements declared run/session separation. It does not
    # authenticate separately recorded runs or independently prove provenance.
    return hashlib.sha256(json.dumps(rows,sort_keys=True,allow_nan=False,separators=(',',':')).encode()).hexdigest()


def _actuator_contract(contract):
    return {k:contract.get(k) for k in ('units','steering_units_verified','steering_zero_bias_verified','source_time_verified',
        'feedback_timestamp_acquisition_verified','command_timestamp_at_actuator_verified','command_history_complete_verified','feedback_filter_phase_verified',
        'known_feedback_filter_delay_s','source_time_evidence','delay_evidence',
        'source_age_limit_s','synchronized_max_skew_s')}


def _gate(contract, keys):
    return [k for k in keys if contract.get(k) is not True]


def analyze_si(rows, contract):
    times=_checked_rows(rows,contract)
    age_limit,skew_limit=_timing_limits(contract)
    outputs=[];previous_stamps={}
    mandatory=('wheel_units_verified','ground_independent','body_heading_independent',
               'coordinate_transform_verified','planar_motion_verified','lever_arm_verified','source_time_verified')
    missing=_gate(contract,mandatory)
    if contract.get('ground_independent') and not _evidence(contract.get('ground_independence_evidence')):
        missing.append('ground_independence_evidence')
    if contract.get('source_time_verified') and not _evidence(contract.get('source_time_evidence')):
        missing.append('source_time_evidence')
    lever=contract.get('rear_axle_to_sensor_m',[None,None])
    if not isinstance(lever,list) or len(lever)!=2 or not all(finite(x) and abs(float(x))<=3. for x in lever):
        missing.append('rear_axle_to_sensor_m')
    if contract.get('heading_frame')!='world_enu_body_flu':missing.append('heading_frame')
    if contract.get('velocity_frame') not in ('world_ned','world_enu','body_flu'):missing.append('velocity_frame')
    low=float(contract.get('low_speed_mps',.15))
    if not .15<=low<=1.:raise ValueError('low-speed gate must remain at least .15 m/s')
    for t,r in zip(times,rows):
        reason=','.join(missing)
        needed=('wheel_mps','ground_vx_mps','ground_vy_mps','heading_rad','yaw_rate_rps',
                'wheel_source_time_s','ground_source_time_s','heading_source_time_s','yaw_source_time_s','receipt_time_s')
        if not all(finite(r.get(k)) for k in needed):reason=reason or 'missing_or_nonfinite_independent_SI_samples'
        if not reason and (abs(float(r['wheel_mps']))>3. or math.hypot(float(r['ground_vx_mps']),float(r['ground_vy_mps']))>3. or abs(float(r['yaw_rate_rps']))>3.):
            reason='outside_low_speed_research_model_domain'
        if not reason:
            stamps=[float(r[k]) for k in ('wheel_source_time_s','ground_source_time_s','heading_source_time_s','yaw_source_time_s')]
            ages=[float(r['receipt_time_s'])-s for s in stamps]
            if any(k in previous_stamps and s<=previous_stamps[k] for k,s in enumerate(stamps)):
                reason='source_timestamp_not_increasing'
            elif abs(t-stamps[1])>1e-6:
                reason='time_s_must_be_ground_acquisition_time'
            elif max(stamps)-min(stamps)>skew_limit:
                reason='asynchronous_sources'
            elif min(ages)<-.002 or max(ages)>age_limit:
                reason='stale_or_future_source'
            previous_stamps=dict(enumerate(stamps))
        if reason:
            outputs.append(dict(time_s=t,slip=estimate(reason=reason),motion_beta=estimate(reason=reason),
                                rear_speed=estimate(reason=reason,units='m/s'),
                                yaw_reference=estimate(reason=reason,units='rad/s'),
                                yaw_residual=estimate(reason=reason,units='rad/s')));continue
        x,y=float(r['ground_vx_mps']),float(r['ground_vy_mps']);heading=float(r['heading_rad']);yaw=float(r['yaw_rate_rps'])
        if contract['velocity_frame']=='world_ned':x,y=y,x
        if contract['velocity_frame']!='body_flu':x,y=math.cos(heading)*x+math.sin(heading)*y,-math.sin(heading)*x+math.cos(heading)*y
        x+=yaw*float(lever[1]);y-=yaw*float(lever[0]);wheel=float(r['wheel_mps'])
        speed=estimate(x,True,'independent_calibrated_rear_axle_speed',.9,'m/s')
        slip=estimate(reason='low_speed_unobservable')
        if min(abs(wheel),abs(x))>=low:
            slip=estimate((wheel-x)/max(abs(wheel),abs(x),low),True,'signed_velocity_difference_not_drive_brake_classification',.8,'1')
        beta=estimate(reason='low_speed_or_reverse_unobservable',units='rad')
        if x>=low:beta=estimate(math.atan2(y,x),True,'rear_axle_motion_direction_not_tire_slip_angle',.8,'rad')
        yaw_reference=estimate(reason='steering_source_zero_bias_or_low_speed_unverified',units='rad/s')
        yaw_residual=estimate(reason=yaw_reference['reason'],units='rad/s')
        if contract.get('steering_units_verified') is True and contract.get('steering_zero_bias_verified') is True and contract.get('yaw_gyro_zero_verified') is True and \
                finite(r.get('steering_rad')) and finite(r.get('steering_source_time_s')) and abs(x)>=low:
            st=float(r['steering_source_time_s']);wb=float(contract.get('wheelbase_m',.6))
            if not .2<=wb<=2.:raise ValueError('wheelbase outside research calibration bounds')
            if abs(float(r['steering_rad']))>.6:
                yaw_reference=estimate(reason='steering_outside_research_model_domain',units='rad/s')
                yaw_residual=estimate(reason=yaw_reference['reason'],units='rad/s')
            elif max(abs(st-z) for z in stamps)<=skew_limit and -.002<=float(r['receipt_time_s'])-st<=age_limit:
                predicted=x*math.tan(float(r['steering_rad']))/wb
                yaw_reference=estimate(predicted,True,'nominal_kinematic_model_prediction_not_measurement',.7,'rad/s')
                yaw_residual=estimate(yaw-predicted,True,'measured_yaw_minus_nominal_not_friction',.7,'rad/s')
        outputs.append(dict(time_s=t,slip=slip,motion_beta=beta,rear_speed=speed,
                            yaw_reference=yaw_reference,yaw_residual=yaw_residual))
    static_keys=('imu_gx_rps','imu_gy_rps','imu_gz_rps','imu_ax_mps2','imu_ay_mps2','imu_az_mps2')
    stationary=contract.get('stationary_verified') is True and contract.get('experiment_phase')=='stationary'
    motion_conflict=any(s['rear_speed']['valid'] and abs(s['rear_speed']['value'])>.03 for s in outputs)
    if motion_conflict:stationary=False
    zero={}
    imu_times=[float(r['imu_source_time_s']) for r in rows if finite(r.get('imu_source_time_s'))]
    imu_time_valid=(len(imu_times)==len(rows) and all(b>a for a,b in zip(imu_times,imu_times[1:])) and
        contract.get('source_time_verified') is True and _evidence(contract.get('source_time_evidence')) and
        all(finite(r.get('receipt_time_s')) and -.002<=float(r['receipt_time_s'])-it<=age_limit
            and abs(it-t)<=skew_limit for r,it,t in zip(rows,imu_times,times)))
    for key in static_keys:
        vals=[float(r[key]) for r in rows if finite(r.get(key))]
        enough=len(vals)==len(rows) and len(vals)>=100 and times[-1]-times[0]>=5.
        valid=stationary and enough and imu_time_valid and contract.get('imu_body_transform_verified') is True
        zero[key]=estimate(sum(vals)/len(vals) if vals else None,valid,
            ('stationary_gyro_measured_mean' if '_rps' in key else 'stationary_acceleration_mean_includes_gravity') if valid else 'stationary_source_time_evidence_transform_or_duration_missing',
            .8, 'rad/s' if '_rps' in key else 'm/s^2', statistics=summary(vals))
    return dict(schema_version=1,run_id=contract.get('run_id'),data_origin=contract.get('data_origin','unverified'),
        estimate_kind='offline_state_estimate',sample_count=len(rows),states=outputs,stationary_raw_means=zero,
        stationary_motion_conflict=motion_conflict,
        gyro_zero_bias_valid=all(zero[k]['valid'] for k in static_keys[:3]),
        accelerometer_bias=estimate(reason='gravity_orientation_and_reference_acceleration_not_identified',units='m/s^2'),
        slip_valid_ratio=sum(s['slip']['valid'] for s in outputs)/len(outputs),
        beta_valid_ratio=sum(s['motion_beta']['valid'] for s in outputs)/len(outputs),
        physical_jerk=estimate(reason='independent_high_rate_velocity_noise_filter_and_source_time_not_validated',units='m/s^3'),
        limitations=['Evidence declarations are operator supplied, not certified by this tool',
                     'No tire friction estimate; no body-heading replacement by course; no implicit interpolation'])


def _steering_samples(rows,contract):
    ts=_checked_rows(rows,contract)
    age_limit,_=_timing_limits(contract)
    for key in ('run_id','continuous_session_id'):
        value=contract.get(key)
        if not isinstance(value,str) or not value.strip() or value.startswith('REPLACE'):
            raise ValueError('explicit_run_and_session_identity_required')
    required=('steering_units_verified','steering_zero_bias_verified','source_time_verified','feedback_timestamp_acquisition_verified','command_history_complete_verified')
    missing=_gate(contract,required)
    if not _evidence(contract.get('source_time_evidence')):missing.append('source_time_evidence')
    if missing:raise ValueError('unverified_actuator_measurement:'+','.join(missing))
    if len(rows)<150 or ts[-1]-ts[0]<5.:raise ValueError('insufficient_actuator_samples')
    command_times=[]
    for t,r in zip(ts,rows):
        keys=('feedback_source_time_s','command_source_time_s','receipt_time_s')
        if not all(finite(r.get(k)) for k in keys):raise ValueError('missing_actuator_source_or_receipt_time')
        ft,ct,receipt=[float(r[k]) for k in keys]
        if abs(ft-t)>1e-6:raise ValueError('time_s_must_be_feedback_acquisition_time')
        if ct>t+1e-9 or t-ct>.2:raise ValueError('asynchronous_command_history')
        if command_times and ct<command_times[-1]:raise ValueError('command_history_time_reversal')
        command_times.append(ct)
        if receipt-ft<-.002 or receipt-ft>age_limit:
            raise ValueError('stale_or_future_steering_feedback')
    if max(b-a for a,b in zip(ts,ts[1:]))>.1:raise ValueError('actuator_sampling_gaps')
    if not all(finite(r.get(k)) for r in rows for k in ('steering_rad','command_steering_rad')):
        raise ValueError('missing_steering_or_command')
    if any(abs(float(r[k]))>.6 for r in rows for k in ('steering_rad','command_steering_rad')):
        raise ValueError('steering_outside_research_model_domain')
    y=[float(r['steering_rad']) for r in rows];u=[float(r['command_steering_rad']) for r in rows]
    def std(x):return math.sqrt(sum((v-sum(x)/len(x))**2 for v in x)/len(x))
    if std(u)<.02 or std(y)<.01:raise ValueError('weak_steering_excitation')
    if len(rows)>3000:raise ValueError('split_experiment_into_complete_phases_max_3000_samples')
    command_ts=[];command_values=[]
    for ct,value in zip(command_times,u):
        if command_ts and ct==command_ts[-1]:
            if value!=command_values[-1]:raise ValueError('conflicting_values_for_same_command_timestamp')
        else:command_ts.append(ct);command_values.append(value)
    if len(command_ts)<20:raise ValueError('insufficient_independent_command_samples')
    return ts,y,command_ts,command_values


def _model_mse(ts,y,command_ts,u,delay,tau):
    # Integrate the calibrated command ZOH on its OWN acquisition clock, including
    # every delayed command change inside a feedback interval. No future input.
    loss=0.;n=0
    for i in range(1,len(ts)):
        begin,end=ts[i-1],ts[i]
        if begin-ts[0]<.4:continue
        j=bisect.bisect_right(command_ts,begin-delay)-1
        if j<0:continue
        predicted=y[i-1];at=begin
        while at<end:
            if j+1<len(command_ts) and command_ts[j+1]+delay<=at:
                j+=1;continue
            until=min(end,command_ts[j+1]+delay) if j+1<len(command_ts) else end
            a=math.exp(-(until-at)/tau)
            predicted=a*predicted+(1-a)*u[j]
            at=until
            if at<end:j+=1
        loss+=(y[i]-predicted)**2;n+=1
    return loss/n if n else float('inf')


def fit_actuator(rows,contract):
    base=dict(schema_version=1,status='invalid',run_id=contract.get('run_id'),
        continuous_session_id=contract.get('continuous_session_id'),data_origin=contract.get('data_origin','unverified'),
        apparent_command_to_feedback_delay_s=None,steering_tau_s=None,
        physical_delay_identified=False,confidence='invalid',safety_or_production_config_changed=False)
    try:ts,y,command_ts,u=_steering_samples(rows,contract)
    except (ValueError,KeyError) as e:return dict(base,reason=str(e))
    candidates=[]
    # Fixed grid bounds are research fit bounds, not changes to physical limits.
    for d in range(21):
        delay=d*.02
        best=min((_model_mse(ts,y,command_ts,u,delay,.08+j*.04),.08+j*.04) for j in range(29))
        candidates.append((best[0],delay,best[1]))
    candidates.sort();mse,delay,tau=candidates[0]
    adjacent=[m for m,d,_ in candidates if abs(d-delay)>=.04]
    separation=(min(adjacent)-mse)/max(mse,1e-12)
    if delay<=1e-9 or delay>=.4-1e-9 or tau<=.08+1e-9 or tau>=1.2-1e-9:return dict(base,reason='fit_at_grid_boundary_unresolved',candidate_mse=mse)
    tau_alternatives=[_model_mse(ts,y,command_ts,u,delay,.08+j*.04) for j in range(29) if abs(.08+j*.04-tau)>=.08-1e-12]
    tau_separation=(min(tau_alternatives)-mse)/max(mse,1e-12)
    if min(separation,tau_separation)<.05:return dict(base,reason='delay_tau_unidentifiable_or_filter_confounded',candidate_mse=mse)
    physical=contract.get('data_origin')=='real_vehicle' and all(contract.get(k) is True for k in ('command_timestamp_at_actuator_verified',
        'feedback_timestamp_acquisition_verified','feedback_filter_phase_verified')) and _evidence(contract.get('delay_evidence')) and contract.get('known_feedback_filter_delay_s')==0.
    model_contract=_actuator_contract(contract)
    return dict(base,status='candidate_fit_only',training_rows_sha256=_row_fingerprint(rows),
        model_contract_sha256=hashlib.sha256(json.dumps(model_contract,sort_keys=True).encode()).hexdigest(),reason='requires_independent_session_validation',
        apparent_command_to_feedback_delay_s=delay,steering_tau_s=tau,one_step_training_mse=mse,
        delay_profile_separation_ratio=separation,tau_profile_separation_ratio=tau_separation,physical_delay_identified=physical,
        confidence='unvalidated',sample_count=len(rows),
        time_resolution_s=summary([b-a for a,b in zip(ts,ts[1:])]),grid_resolution_s=dict(delay=.02,tau=.04),
        interpretation='endpoint response deadtime conditional on verified clock/filter evidence' if physical else
                       'ROS command-to-feedback apparent delay, includes transport/filter effects; not pure actuator deadtime')


def validate_actuator(fit,rows,contract):
    if fit.get('status')!='candidate_fit_only':raise ValueError('candidate fit required')
    if fit.get('run_id')==contract.get('run_id') or fit.get('continuous_session_id')==contract.get('continuous_session_id'):
        raise ValueError('independent complete run AND continuous session required')
    if fit.get('data_origin')!=contract.get('data_origin','unverified'):raise ValueError('provenance mismatch')
    model_contract=_actuator_contract(contract)
    if hashlib.sha256(json.dumps(model_contract,sort_keys=True).encode()).hexdigest()!=fit.get('model_contract_sha256'):
        raise ValueError('measurement contract differs between training and validation')
    delay,tau=fit.get('apparent_command_to_feedback_delay_s'),fit.get('steering_tau_s')
    if not finite(delay) or not 0.<float(delay)<.4 or not finite(tau) or not .08<float(tau)<1.2:
        raise ValueError('invalid_or_out_of_domain_training_fit')
    if not fit.get('training_rows_sha256') or _row_fingerprint(rows)==fit['training_rows_sha256']:
        raise ValueError('independent_sample_content_required')
    ts,y,command_ts,u=_steering_samples(rows,contract)
    mse=_model_mse(ts,y,command_ts,u,fit['apparent_command_to_feedback_delay_s'],fit['steering_tau_s'])
    nominal=_model_mse(ts,y,command_ts,u,.10,.44)
    accepted=mse<.8*nominal
    return dict(fit,status='held_out_equation_validated' if accepted else 'validation_rejected',
        validation_run=contract.get('run_id'),validation_session=contract.get('continuous_session_id'),
        held_out_mse=mse,nominal_held_out_mse=nominal,validation_rows_sha256=_row_fingerprint(rows),
        confidence='held_out_one_step_only' if accepted else 'low_confidence',
        reason='independent_equation_fit_only_not_closed_loop_or_real_vehicle_approval',
        vehicle_closed_loop_improvement_claim=False)


def audit_legacy(rows):
    """Describe legacy internal diagnostics without promoting them to raw sensors."""
    keys=set(rows[0]) if rows else set()
    reasons=['independent_ground_velocity_missing','raw_imu_and_heading_source_provenance_missing',
             'CAN_acquisition_timestamp_and_sequence_missing','command_execution_timestamp_missing',
             'configuration_delay_and_tau_are_assumptions_not_observed_identification']
    ages=[float(r['steering_receive_age']) for r in rows if finite(r.get('steering_receive_age')) and float(r['steering_receive_age'])>=0.]
    times=[float(r['stamp']) for r in rows if finite(r.get('stamp'))]
    return dict(schema_version=1,status='invalid_for_physical_calibration',
        evidence_scope='limited_legacy_controller_internal_diagnostics',rows=len(rows),
        run_ids=sorted(set(r.get('run_id','unknown') for r in rows)),columns=sorted(keys),
        receipt_steering_age_s=summary(ages),record_interval_s=summary([b-a for a,b in zip(times,times[1:])]),
        reasons=reasons,slip=estimate(reason=reasons[0]),motion_beta=estimate(reason=reasons[1]),
        yaw_gain=estimate(reason='independent_speed_and_synchronized_actual_steering_missing'),
        actuator_delay=estimate(reason=reasons[2]),steering_tau=estimate(reason=reasons[4]),
        actual_jerk=estimate(reason='independent_actual_velocity_and_filter_phase_missing'),
        retained_for='S8 historical internal tracking/command descriptive baseline only',
        baseline_controller_quality_claim=False,production_changed=False)
