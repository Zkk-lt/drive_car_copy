"""Read-only metrics. Missing evidence is null, never a successful zero."""
import math
from collections import Counter

def number(v):
    return isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v)

def stats(values):
    a=sorted(float(x) for x in values if number(x))
    if not a:return {'count':0,'rmse':None,'p50':None,'p95':None,'p99':None,'max':None}
    def percentile(q):
        f=(len(a)-1)*q;i=int(f);j=min(i+1,len(a)-1)
        return a[i]+(a[j]-a[i])*(f-i)
    return {'count':len(a),'rmse':math.sqrt(sum(x*x for x in a)/len(a)),
            'p50':percentile(.50),'p95':percentile(.95),'p99':percentile(.99),'max':max(a)}

def ratio(rows, key):
    usable=[r[key] for r in rows if type(r.get(key)) is bool]
    return sum(usable)/len(usable) if usable else None

def summarize(events, manifest):
    health={};controllers=[];adaptive=[];validation=[];mismatches=0
    for event in events:
        if event.get('run_id') != manifest.get('run_id'):
            mismatches+=1;continue
        topic=event['topic'];state=health.setdefault(topic,{'count':0,'source_missing':0,'nonmonotonic_source':0,
             'source_age_ms':[],'receipt_intervals_ms':[],'reasons':Counter()})
        state['count']+=1
        state['reasons'].update(event.get('validity',{}).get('reasons',[]))
        stamp=event.get('source_stamp_ns');receive=event.get('receipt_ros_ns')
        steady=event.get('receipt_monotonic_ns')
        if stamp:
            if state.get('previous_source') is not None and stamp <= state['previous_source']:
                state['nonmonotonic_source']+=1
            state['previous_source']=stamp
            if number(receive):state['source_age_ms'].append((receive-stamp)/1e6)
        else:state['source_missing']+=1
        if number(steady):
            if 'previous_receive' in state:
                state['receipt_intervals_ms'].append((steady-state['previous_receive'])/1e6)
            state['previous_receive']=steady
        if event.get('ros_type')=='move_car/ControlResearchTelemetry':
            p=event['payload']
            if p.get('run_id') != manifest.get('run_id'):
                mismatches+=1;continue
            p=dict(p);p['_time']=stamp/1e9 if stamp else None
            controllers.append(p)
        if event.get('ros_type')=='move_car/AdaptiveControlTelemetry':
            p=event['payload']
            if p.get('run_id')!=manifest.get('run_id'):mismatches+=1;continue
            adaptive.append(p)
        if event.get('ros_type')=='move_car/ResearchValidationTelemetry':
            p=event['payload']
            if p.get('run_id')!=manifest.get('run_id'):mismatches+=1;continue
            validation.append(p)
    tracked=[r for r in controllers if r.get('tracking_valid') is True]
    attempted=[r for r in controllers if r.get('solver_attempted') is True]
    def collect(rows,key,absolute=False):
        return [abs(r[key]) if absolute else r[key] for r in rows if number(r.get(key))]
    steering_rates=[];last=None;reversals=0;prev_rate=None
    for row in controllers:
        t=row['_time'];delta=row.get('command_steering')
        if last and number(t) and number(delta):
            dt=t-last[0]
            if 0 < dt <= .20:
                rate=(delta-last[1])/dt;steering_rates.append(abs(rate))
                if abs(rate)>.01:
                    if prev_rate is not None and rate*prev_rate<0:reversals+=1
                    prev_rate=rate
            else:prev_rate=None
        last=(t,delta) if number(t) and number(delta) else None
    for state in health.values():
        for k in ('previous_source','previous_receive'):state.pop(k,None)
        for k in ('source_age_ms','receipt_intervals_ms'):state[k]=stats(state[k])
        state['reasons']=dict(state['reasons'])
    errors=[r['measured_speed_fused']-r['target_speed'] for r in tracked
            if number(r.get('measured_speed_fused')) and number(r.get('target_speed'))]
    budget=manifest.get('config',{}).get('values',{}).get('solver_budget_ms',8.)
    solve_times=collect(attempted,'solve_ms')
    estimates={}
    for row in adaptive:
        for name,e in zip(row.get('estimate_names',[]),row.get('estimates',[])):
            estimates.setdefault(name,[]).append(e)
    adaptive_attempts=[r for r in adaptive if r.get('evaluations',0)>0 or r.get('timed_out')]
    adaptive_summary={'samples':len(adaptive),'shadow_only':all(r.get('shadow_only') is True for r in adaptive) if adaptive else None,
      'requested_modes':dict(Counter(r.get('requested_mode') for r in adaptive)),
      'selected_owners':dict(Counter(r.get('selected_owner') for r in adaptive)),
      'actual_owners':dict(Counter(r.get('actual_owner') for r in adaptive)),
      'reasons':dict(Counter(r.get('reason') for r in adaptive)),
      'config_hashes':sorted(set(r.get('config_hash','') for r in adaptive)),
      'solve_ms':stats(collect(adaptive_attempts,'solve_ms')),
      'shadow_cycle_ms':stats(collect(adaptive,'shadow_cycle_ms')),
      'timeout_ratio':ratio(adaptive_attempts,'timed_out'),'feasible_ratio':ratio(adaptive_attempts,'feasible'),
      'estimates':{k:{'valid_ratio':ratio(v,'valid'),
          'values_valid_only':stats(e.get('value') for e in v if e.get('valid') is True),
          'invalid_reasons':dict(Counter(e.get('reason') for e in v if not e.get('valid')))} for k,v in estimates.items()}}
    phases=('initialization_ms','reference_ms','prediction_ms','derivative_ms','constraint_cost_ms',
            'direction_ms','candidate_selection_ms','other_ms')
    profiled=[r for r in validation if r.get('detailed_profiling_enabled') is True]
    validation_summary={'samples':len(validation),'profiled_samples':len(profiled),
      'profile_ms':{k:stats(collect(profiled,k)) for k in phases},
      'budget_limited_ratio':ratio(validation,'budget_limited'),
      'model_adaptation_active_ratio':ratio(validation,'model_adaptation_active'),
      'weight_adaptation_active_ratio':ratio(validation,'weight_adaptation_active'),
      'reasons':{k:dict(Counter(r.get(k,'missing') for r in validation)) for k in
        ('adaptation_reason','ground_frame_status','imu_frame_status','source_time_status','wheel_freshness_status')},
      'evidence_reasons':dict(Counter(reason for r in validation for reason in r.get('adaptation_reasons',[])))}
    return {'schema_version':1,'run_id':manifest.get('run_id'),'terrain_label':manifest.get('terrain_label'),
      'provenance':'sample-weighted diagnostic metrics; no independent ground truth; no closed-loop counterfactual',
      'run_id_mismatch_events':mismatches,'sensor_health':health,'control_samples':len(controllers),
      'tracking_samples':len(tracked),'solver_attempts':len(attempted),
      'lateral_error_abs_m':stats(collect(tracked,'lateral_error',True)),
      'heading_error_abs_rad':stats(collect(tracked,'heading_error',True)),
      'speed_error_fused_mps':stats(errors),'yaw_response_error_radps':stats(collect(tracked,'yaw_rate_error',True)),
      'command_steering_rate_abs_radps':stats(steering_rates),'command_steering_reversals':reversals if steering_rates else None,
      'command_acceleration_mps2':stats(collect(controllers,'command_acceleration')),
      'command_jerk_mps3':stats(collect(controllers,'command_jerk')),
      'steering_saturation_sample_ratio':ratio(controllers,'steering_saturated'),
      'steering_rate_limited_sample_ratio':ratio(controllers,'steering_rate_limited'),
      'safe_stop_sample_ratio':ratio(controllers,'safe_stop'),
      'collision_stop_sample_ratio':ratio(controllers,'collision_stop'),
      'solver_failure_ratio':1.-ratio(attempted,'solver_ok') if attempted else None,
      'solve_ms':stats(solve_times),'compute_ms':stats(collect(controllers,'compute_ms')),
      'callback_before_telemetry_ms':stats(collect(controllers,'callback_before_telemetry_ms')),
      'timer_lateness_ms':stats(collect(controllers,'timer_lateness_ms')),
      'solve_over_proposed_budget_ratio':sum(t>budget for t in solve_times)/len(solve_times) if solve_times else None,
      'proposed_budget_ms':budget,'budget_enforced':False,
      'adaptive_runtime':adaptive_summary,
      'validation':validation_summary,
      'fallback_reason_counts':dict(Counter(r.get('fallback_reason','unknown') for r in controllers)),
      'stop_reason_counts':dict(Counter(r.get('stop_reason','unknown') for r in controllers if r.get('safe_stop'))),
      'unavailable':{'physical_acceleration_jerk':'requires calibrated frame/gravity compensation',
        'route_completion':'requires route endpoint and independent progress definition',
        'physical_constraint_violations':'predicted feasibility is recorded in adaptive events; requires independent measured verification',
        'nmpc_closed_loop_performance':'shadow diagnostics are not applied control; use independent closed-loop tests',
        'slip_friction_sideslip':'independence, frames, excitation and time alignment not verified',
        'closed_loop_improvement':'requires separate closed-loop validation'},
      'recording_status':manifest.get('state'),'writer':manifest.get('writer')}
