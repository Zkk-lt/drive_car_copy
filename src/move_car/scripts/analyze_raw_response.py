#!/usr/bin/env python3
"""Offline receipt-clock response diagnostics; no ROS or production writes.

These equations describe the recorder's observations, not actuator physics.
Hardware sampling, command publication and actuator reception times remain
unknown. This tool deliberately does not relax identify_vehicle_model's gates.
"""
import argparse
import bisect
import collections
import hashlib
import json
import math
from pathlib import Path

from control_research.dataset import digest, write_new
from control_research.model_identification import residual_statistics as _source_residual_statistics, _nominal_source

MODELS = {
    'steering': dict(command='steering', feedback='steering', nominal_delay=.325,
                     nominal_tau=.44, input_std=.02, output_std=.01, bias_limit=.08,
                     unit='rad', parameter='steering'),
    'reported_wheel_speed': dict(command='speed', feedback='speed', nominal_delay=.20,
                     nominal_tau=.20, input_std=.03, output_std=.02, bias_limit=.15,
                     unit='driver_convention_mps_not_ground_speed', parameter='speed'),
}


def residual_statistics(values):
    # Reuse arithmetic only; the strict identifier's source-clock explanation
    # must not be copied into this explicitly receipt-clock diagnostic.
    result = _source_residual_statistics(values)
    result['interpretation'] = ('Recorder receipt-domain equation residuals; not source/acquisition-time '
        'or actuator physical residuals. Serial correlation and systematic clock/cache/filter errors '
        'invalidate naive IID confidence coverage.')
    return result


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def spread(values):
    if not values:
        return 0.
    mean = sum(values)/len(values)
    return math.sqrt(sum((x-mean)**2 for x in values)/len(values))


def load_raw_run(directory, command_publisher='/publish_speed_turn'):
    """Read immutable, closed raw runs; do not reinterpret receipt as source."""
    directory = Path(directory).resolve()
    manifest_path = directory/'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    writer = manifest.get('writer', {})
    if manifest.get('state') != 'closed':
        raise ValueError('closed_run_required:' + str(directory))
    if (writer.get('writer_error') or writer.get('queue_dropped') != 0 or
            writer.get('unwritten') != 0 or writer.get('accepted') != writer.get('written')):
        raise ValueError('complete_local_writer_required:' + str(directory))
    if manifest.get('data_origin') not in ('real_vehicle', 'synthetic'):
        raise ValueError('explicit_real_vehicle_or_synthetic_origin_required')
    if manifest.get('terrain_label') != 'smooth':
        raise ValueError('current_receipt_response_research_supports_only_smooth')
    commands, feedback, barriers = [], [], []
    counts = collections.Counter()
    h = hashlib.sha256()
    with (directory/'events.jsonl').open('rb') as stream:
        for line in stream:
            h.update(line)
            if not line.strip():
                continue
            event = json.loads(line)
            counts['all_events'] += 1
            if event.get('run_id') != manifest.get('run_id'):
                raise ValueError('mixed_run_id')
            topic = event.get('topic')
            if topic not in ('/cmd_vel', '/car_message'):
                continue
            counts[topic + '_events'] += 1
            stamp = event.get('receipt_monotonic_ns')
            if not isinstance(stamp, int) or isinstance(stamp, bool) or stamp <= 0:
                raise ValueError('missing_positive_monotonic_receipt_time')
            payload = event.get('payload', {})
            if topic == '/cmd_vel':
                if event.get('publisher') != command_publisher:
                    raise ValueError('unexpected_command_publisher_not_S8')
                speed = payload.get('linear', {}).get('x')
                steering = payload.get('angular', {}).get('z')
                if not all(finite(x) for x in (speed, steering)) or abs(speed) > 1.5 or abs(steering) > .6:
                    counts['invalid_command_samples'] += 1
                    barriers.append(stamp)
                    continue
                commands.append((stamp, float(steering), float(speed)))
            else:
                number = counts['/car_message_events']
                if not finite(payload.get('battery_level')) or payload['battery_level'] <= 0:
                    counts['feedback_battery_zero_or_invalid'] += 1
                    continue
                if payload.get('error_flag') != 0:
                    counts['feedback_error_flag_nonzero_or_missing'] += 1
                    continue
                steering, wheel = payload.get('turn_angle'), payload.get('back_wheel_speed')
                if not all(finite(x) for x in (steering, wheel)) or abs(steering) > .6 or abs(wheel) > 1500:
                    counts['feedback_nonfinite_or_outside_research_signal_domain'] += 1
                    continue
                feedback.append((stamp, float(steering), .001*float(wheel), number))
    if counts['all_events'] != writer.get('written'):
        raise ValueError('writer_and_event_count_mismatch')
    actual_hash = h.hexdigest()
    declared = manifest.get('events_sha256')
    if declared is not None and declared != actual_hash:
        raise ValueError('events_sha256_mismatch')
    if not commands or not feedback:
        raise ValueError('raw_commands_and_feedback_required')
    for name, rows in (('command', commands), ('feedback', feedback)):
        counts[name + '_file_order_inversions'] = sum(b[0] < a[0] for a, b in zip(rows, rows[1:]))
        rows.sort(key=lambda r: r[0])
        if any(b[0] == a[0] for a, b in zip(rows, rows[1:])):
            raise ValueError('duplicate_' + name + '_receipt_timestamp')
    origin = min(commands[0][0], feedback[0][0])
    commands = [((t-origin)*1e-9, d, v) for t, d, v in commands]
    feedback = [((t-origin)*1e-9, d, v, n) for t, d, v, n in feedback]
    return dict(run_id=manifest['run_id'],
        continuous_session_id=manifest.get('continuous_session_id'),
        data_origin=manifest['data_origin'], terrain_label=manifest.get('terrain_label'),
        directory=str(directory), manifest_sha256=digest(manifest_path), events_sha256=actual_hash,
        receipt_origin_monotonic_ns=origin, counts=dict(counts), commands=commands,
        feedback=feedback, command_barriers=sorted((t-origin)*1e-9 for t in barriers))


def prepare_pairs(run, model, maximum_delay=.7, maximum_feedback_gap=.15, maximum_command_gap=.20):
    """Use a common, delay-independent mask, so grid losses are comparable."""
    column = 1 if model == 'steering' else 2
    commands = [(r[0], r[column]) for r in run['commands']]
    feedback = [(r[0], r[column], r[3]) for r in run['feedback']]
    ct = [r[0] for r in commands]
    gap_prefix = [0]
    for a, b in zip(ct, ct[1:]):
        gap_prefix.append(gap_prefix[-1] + int(b-a > maximum_command_gap))
    rejected = collections.Counter()
    pairs = []
    barriers = run.get('command_barriers', [])
    for previous, current in zip(feedback, feedback[1:]):
        begin, before, old_index = previous
        end, observed, new_index = current
        first = bisect.bisect_right(ct, begin-maximum_delay)-1
        last = bisect.bisect_right(ct, end)-1
        reason = None
        if new_index != old_index+1:
            reason = 'invalid_feedback_between_pair'
        elif end-begin <= 0 or end-begin > maximum_feedback_gap:
            reason = 'feedback_gap_or_nonpositive_interval'
        elif first < 0 or last < 0:
            reason = 'insufficient_complete_delay_history'
        elif end-ct[last] > maximum_command_gap or gap_prefix[last]-gap_prefix[first] > 0:
            reason = 'command_history_gap'
        elif bisect.bisect_right(barriers, end) > bisect.bisect_left(barriers, begin-maximum_delay):
            reason = 'invalid_command_in_delay_history'
        if reason:
            rejected[reason] += 1
        else:
            pairs.append((begin, end, before, observed))
    return dict(commands=commands, command_times=ct, pairs=pairs, rejected=dict(rejected),
        input_std=spread([v for _, v in commands]), output_std=spread([v for _, v, _ in feedback]),
        support=dict(command_range=[min(v for _, v in commands), max(v for _, v in commands)],
                     reported_feedback_range=[min(v for _, v, _ in feedback), max(v for _, v, _ in feedback)],
                     usable_pair_count=len(pairs), usable_interval_sum_s=sum(b-a for a,b,_,_ in pairs)))


def zoh_basis(command_times, commands, begin, end, delay, tau):
    """Exact convolution of every held command within ONE feedback interval."""
    index = bisect.bisect_right(command_times, begin-delay)-1
    if index < 0 or end <= begin or tau <= 0 or delay < 0:
        raise ValueError('invalid_ZOH_interval_or_missing_command_history')
    at, convolved = begin, 0.
    while at < end:
        change = command_times[index+1]+delay if index+1 < len(commands) else end
        until = min(end, change)
        if until <= at:
            index += 1
            continue
        decay = math.exp(-(until-at)/tau)
        convolved = decay*convolved + (1-decay)*commands[index][1]
        at = until
        if at < end:
            index += 1
    return math.exp(-(end-begin)/tau), convolved


def equations(prepared, delay, tau):
    rows = []
    for p in prepared:
        for begin, end, before, observed in p['pairs']:
            decay, x = zoh_basis(p['command_times'], p['commands'], begin, end, delay, tau)
            rows.append((x, 1-decay, observed-decay*before))
    return rows


def fit_gain_bias(rows, bias_limit):
    xx = sum(x*x for x,z,y in rows)
    zz = sum(z*z for x,z,y in rows)
    xz = sum(x*z for x,z,y in rows)
    det = xx*zz-xz*xz
    if det <= 1e-8*max(xx*zz, 1e-24):
        return None
    xy = sum(x*y for x,z,y in rows)
    zy = sum(z*y for x,z,y in rows)
    gain, bias = (xy*zz-zy*xz)/det, (zy*xx-xy*xz)/det
    if not .5 <= gain <= 1.5 or not -bias_limit <= bias <= bias_limit:
        return None
    return gain, bias


def error_values(prepared, parameters):
    return [y-parameters['command_gain']*x-parameters['output_bias']*z
        for x,z,y in equations(prepared, parameters['apparent_delay_s'], parameters['apparent_tau_s'])]


def persistence_errors(prepared):
    return [observed-before for p in prepared for _,_,before,observed in p['pairs']]


def free_prediction(prepared, parameters):
    """0.5/1 s response predictions, never resetting to intermediate feedback.

    Anchors use only the initial measured state. Subsequent actual commands are
    integrated on their receipt clock; this is historical response validation,
    not an online forecast of commands not yet chosen. All horizons use the same
    nonoverlapping anchors, and invalid feedback/command gaps break each chain.
    """
    errors = {.5: [], 1.: []}
    persistent = {.5: [], 1.: []}
    actual_horizons = {.5: [], 1.: []}
    for p in prepared:
        chains = []
        for pair in p['pairs']:
            if not chains or abs(chains[-1][-1][1]-pair[0]) > 1e-9:
                chains.append([])
            chains[-1].append(pair)
        for chain in chains:
            anchor = 0
            while anchor < len(chain):
                begin = chain[anchor][0]
                ends = {}
                for index in range(anchor, len(chain)):
                    elapsed = chain[index][1]-begin
                    for horizon in (.5,1.):
                        if horizon not in ends and elapsed >= horizon-1e-9:
                            ends[horizon] = index
                    if len(ends) == 2:
                        break
                if len(ends) != 2:
                    break
                initial = chain[anchor][2]
                predicted = initial
                for index in range(anchor, ends[1.]+1):
                    at, until, _, observed = chain[index]
                    decay, x = zoh_basis(p['command_times'], p['commands'], at, until,
                        parameters['apparent_delay_s'], parameters['apparent_tau_s'])
                    predicted = (decay*predicted + parameters['command_gain']*x +
                                 parameters['output_bias']*(1-decay))
                    for horizon, end_index in ends.items():
                        if index == end_index:
                            errors[horizon].append(observed-predicted)
                            persistent[horizon].append(observed-initial)
                            actual_horizons[horizon].append(until-begin)
                anchor = ends[1.]+1  # next anchor begins at or after prior 1 s endpoint
    return {('%.1fs' % horizon): dict(valid=bool(values), nonoverlapping_anchor_count=len(values),
        residual=residual_statistics(values) if values else None,
        persistence_residual=residual_statistics(persistent[horizon]) if values else None,
        actual_horizon_range_s=[min(actual_horizons[horizon]),max(actual_horizons[horizon])] if values else None,
        prediction_kind='no_feedback_reset_historical_actual_command_ZOH_not_future_command_forecast')
        for horizon,values in errors.items()}


def free_prediction_tails_no_worse(candidate, nominal, tolerance=1e-12):
    """Protect both high-percentile and worst error at every common horizon."""
    for horizon, result in candidate.items():
        baseline = nominal.get(horizon, {})
        if not result.get('valid') or not baseline.get('valid'):
            return False
        for statistic in ('p99', 'maximum'):
            if result['residual']['absolute'][statistic] > baseline['residual']['absolute'][statistic]+tolerance:
                return False
    return bool(candidate)


def fit_response(training_runs, holdout_runs, model, delays=None, taus=None):
    settings = MODELS[model]
    full_train = [prepare_pairs(r, model) for r in training_runs]
    holdout = [prepare_pairs(r, model) for r in holdout_runs]
    pair_count = sum(len(p['pairs']) for p in full_train)
    # Bound offline search cost on the vehicle computer. Each retained interval
    # keeps ALL original command changes; this is not command resampling. Final
    # selected-model metrics use every usable original feedback interval.
    stride = max(1, math.ceil(pair_count/3000))
    train, global_index = [], 0
    for prepared in full_train:
        selected = []
        for pair in prepared['pairs']:
            if global_index % stride == 0:
                selected.append(pair)
            global_index += 1
        train.append(dict(prepared, pairs=selected))
    base = dict(model=model, status='invalid', reason='insufficient_excitation_or_pairs',
        evidence_scope='recorder_receipt_clock_apparent_response_only',
        eligible_for_runtime=False, physical_parameter_certification=False,
        physical_delay_identified=False, ground_speed_calibration=False,
        independent_validation=False, vehicle_closed_loop_improvement_claim=False,
        unit=settings['unit'], training_support=[p['support'] for p in full_train],
        training_pair_exclusions=[p['rejected'] for p in full_train],
        holdout_support=[p['support'] for p in holdout], holdout_pair_exclusions=[p['rejected'] for p in holdout],
        excitation=[dict(input_std=p['input_std'], output_std=p['output_std']) for p in full_train],
        fitting_interval_selection=dict(original_usable_pair_count=pair_count,
            selected_pair_count=sum(len(p['pairs']) for p in train), maximum_selected_pairs=3000,
            stride=stride, policy='deterministic_interval_subselection_all_command_events_retained',
            final_selected_model_metrics='all_usable_original_feedback_intervals'))
    if (pair_count < 100 or
        sum(p['support']['usable_interval_sum_s'] for p in full_train) < 5 or
        not all(p['input_std'] >= settings['input_std'] and p['output_std'] >= settings['output_std'] for p in full_train)):
        return base
    refine = delays is None and taus is None
    delays = sorted(set(delays if delays is not None else [round(i*.1,8) for i in range(8)]+[settings['nominal_delay']]))
    taus = sorted(set(taus if taus is not None else [.05,.1,.2,.35,.5,.75,1.,settings['nominal_tau']]))
    if not all(finite(d) and 0 <= d <= .7 for d in delays) or not all(finite(t) and .05 <= t <= 1. for t in taus):
        raise ValueError('search_cannot_relax_existing_model_parameter_bounds')
    families = {'unit_gain_zero_bias': [], 'gain_bias_proxy': []}
    evaluated = set()
    def evaluate_grid_point(delay, tau):
        if (delay,tau) in evaluated:
            return
        evaluated.add((delay,tau))
        rows = equations(train, delay, tau)
        for name, gb in (('unit_gain_zero_bias', (1.,0.)),
                         ('gain_bias_proxy', fit_gain_bias(rows, settings['bias_limit']))):
            if gb is None:
                continue
            gain, bias = gb
            mse = sum((y-gain*x-bias*z)**2 for x,z,y in rows)/len(rows)
            families[name].append((mse, delay, tau, gain, bias))
    for delay in delays:
        for tau in taus:
            evaluate_grid_point(delay, tau)
    if refine:
        # One bounded local refinement per family; never tune against holdout.
        centers = [min(values)[1:3] for values in families.values() if values]
        for delay, tau in centers:
            for dd in (-.05,-.025,0.,.025,.05):
                for dt in (-.05,-.025,0.,.025,.05):
                    d,t = round(delay+dd,8), round(tau+dt,8)
                    if 0 <= d <= .7 and .05 <= t <= 1.:
                        evaluate_grid_point(d,t)
    nominal = dict(apparent_delay_s=settings['nominal_delay'], apparent_tau_s=settings['nominal_tau'],
                   command_gain=1., output_bias=0.)
    out = dict(base, status='diagnostic_complete', reason='see_each_family_acceptance',
        nominal_comparison=dict(parameters=nominal, source=_nominal_source(),
            source_kind='research_types.h defaults; not runtime or physical certification'),
        families={}, grid=dict(coarse_delays_s=delays, coarse_taus_s=taus,
            single_refinement_enabled=refine, refinement_step_s=.025 if refine else None,
            evaluated_points=[list(p) for p in sorted(evaluated)], evaluation_count=len(evaluated),
            hard_delay_bounds_s=[0.,.7], hard_tau_bounds_s=[.05,1.]),
        uncertainty=dict(kind='five_percent_training_loss_profile_not_statistical_confidence_interval',
                         accounts_for_hardware_clock_or_systematic_errors=False))
    for name, candidates in families.items():
        if not candidates:
            out['families'][name] = dict(status='rejected', reason='weak_regression_or_gain_bias_outside_bounds')
            continue
        candidates.sort()
        mse, delay, tau, gain, bias = candidates[0]
        parameters = dict(apparent_delay_s=delay, apparent_tau_s=tau, command_gain=gain, output_bias=bias)
        near = [c for c in candidates if c[0] <= mse*1.05+1e-14]
        boundary = delay <= 1e-9 or delay >= .7-1e-9 or tau <= .05+1e-9 or tau >= 1.-1e-9
        item = dict(status='candidate_diagnostic_only', reason='no_independent_session_or_physical_time_validation',
            parameters=parameters, at_hard_search_boundary=boundary,
            confidence='unvalidated_receipt_equation_only', eligible_for_runtime=False,
            training_residual=residual_statistics(error_values(full_train, parameters)),
            nominal_training_residual=residual_statistics(error_values(full_train, nominal)),
            training_persistence_residual=residual_statistics(persistence_errors(full_train)),
            training_free_prediction=free_prediction(full_train, parameters),
            nominal_training_free_prediction=free_prediction(full_train, nominal),
            profile_delay_range_s=[min(c[1] for c in near), max(c[1] for c in near)],
            profile_tau_range_s=[min(c[2] for c in near), max(c[2] for c in near)],
            nmpc_equation_compatible=name == 'unit_gain_zero_bias',
            parameter_transfer_approved=False,
            per_run_diagnostic_holdout=[])
        if name == 'gain_bias_proxy':
            item['nmpc_mapping_limitation'] = 'Current NMPC has command_gain=1 and output_bias=0; copying only tau/delay changes the fitted equation.'
        if boundary:
            item.update(status='rejected', reason='hard_search_boundary_unresolved_do_not_transfer')
        if not holdout or any(len(p['pairs']) < 100 or p['support']['usable_interval_sum_s'] < 5 for p in holdout):
            item.update(status='rejected', reason='sufficient_diagnostic_holdout_required')
        else:
            for run, prepared in zip(holdout_runs, holdout):
                candidate_stats = residual_statistics(error_values([prepared], parameters))
                nominal_stats = residual_statistics(error_values([prepared], nominal))
                candidate_free = free_prediction([prepared], parameters)
                nominal_free = free_prediction([prepared], nominal)
                train_input = [p['support']['command_range'] for p in train]
                train_output = [p['support']['reported_feedback_range'] for p in train]
                input_domain = [min(p[0] for p in train_input), max(p[1] for p in train_input)]
                output_domain = [min(p[0] for p in train_output), max(p[1] for p in train_output)]
                holdout_input = prepared['support']['command_range']
                holdout_output = prepared['support']['reported_feedback_range']
                item['per_run_diagnostic_holdout'].append(dict(run_id=run['run_id'],
                    candidate=candidate_stats, nominal=nominal_stats,
                    persistence=residual_statistics(persistence_errors([prepared])),
                    candidate_free_prediction=candidate_free, nominal_free_prediction=nominal_free,
                    enough_nonoverlapping_free_anchors=all(v['nonoverlapping_anchor_count'] >= 3 for v in candidate_free.values()),
                    free_prediction_no_worse_than_nominal=all(v['valid'] and
                        v['residual']['mse'] <= nominal_free[h]['residual']['mse']+1e-15
                        for h,v in candidate_free.items()),
                    free_prediction_p99_and_max_no_worse_than_nominal=free_prediction_tails_no_worse(candidate_free, nominal_free),
                    one_step_p99_and_max_no_worse_than_nominal=all(
                        candidate_stats['absolute'][key] <= nominal_stats['absolute'][key]+1e-12
                        for key in ('p99','maximum')),
                    non_independent_validation=True,
                    context_note=('Diagnostic holdout only; continuous-session independence is unverified. '
                        'Signal-range coverage is context, not independent model validation or runtime eligibility.'),
                    improves_nominal=candidate_stats['mse'] < nominal_stats['mse'],
                    training_signal_domain=dict(command=input_domain, reported_feedback=output_domain),
                    holdout_outside_training_signal_range=(holdout_input[0] < input_domain[0] or
                        holdout_input[1] > input_domain[1] or holdout_output[0] < output_domain[0] or
                        holdout_output[1] > output_domain[1])))
            if not all(r['improves_nominal'] for r in item['per_run_diagnostic_holdout']):
                item.update(status='rejected', reason='does_not_improve_each_diagnostic_holdout')
            elif not all(r['enough_nonoverlapping_free_anchors'] for r in item['per_run_diagnostic_holdout']):
                item.update(status='rejected', reason='insufficient_nonoverlapping_multistep_holdout_anchors')
            elif not all(r['free_prediction_no_worse_than_nominal'] for r in item['per_run_diagnostic_holdout']):
                item.update(status='rejected', reason='multistep_diagnostic_holdout_worse_than_nominal')
            elif not all(r['free_prediction_p99_and_max_no_worse_than_nominal'] for r in item['per_run_diagnostic_holdout']):
                item.update(status='rejected', reason='multistep_diagnostic_tail_worse_than_nominal')
            elif not all(r['one_step_p99_and_max_no_worse_than_nominal'] for r in item['per_run_diagnostic_holdout']):
                item.update(status='rejected', reason='one_step_diagnostic_tail_worse_than_nominal')
        item['diagnostic_acceptance_gate'] = dict(one_step_must_improve_each_holdout=True,
            one_step_absolute_p99_and_max_must_not_worsen_nominal=True,
            half_and_one_second_response_must_not_worsen_nominal=True,
            each_horizon_absolute_p99_and_max_must_not_worsen_nominal=True,
            absolute_tail_comparison_tolerance=1e-12,
            minimum_nonoverlapping_holdout_anchors_per_horizon=3,
            persistence_comparison='reported_separately_not_claim_of_independence_or_physical_validation')
        out['families'][name] = item
    return out


def analyze(training_runs, holdout_runs, models):
    fingerprints = [r['events_sha256'] for r in training_runs+holdout_runs]
    if len(set(fingerprints)) != len(fingerprints):
        raise ValueError('duplicate_training_or_holdout_raw_content')
    if {r['data_origin'] for r in training_runs+holdout_runs} not in ({'real_vehicle'}, {'synthetic'}):
        raise ValueError('do_not_mix_synthetic_and_real_measurements')
    def metadata(run):
        return {k:v for k,v in run.items() if k not in ('commands','feedback','command_barriers')}
    return dict(schema_version=1, tool='analyze_raw_response', status='diagnostic_only',
        tool_source_sha256=digest(__file__),
        training_runs=[metadata(r) for r in training_runs], diagnostic_holdout_runs=[metadata(r) for r in holdout_runs],
        continuous_session_independence='unverified_operator_grouping_required',
        holdout_is_independent_session_validation=False, eligible_for_runtime=False,
        physical_parameter_certification=False, physical_delay_identified=False,
        production_or_safety_config_written=False, vehicle_closed_loop_improvement_claim=False,
        time_basis='each_run_recorder_receipt_monotonic_ns_only_not_source_publish_or_actuator_time',
        command_history='every_raw_command_event_piecewise_ZOH_exact_exponential_integral_no_20Hz_command_resampling',
        limitations=['CAN hardware acquisition/new-frame, command publication and actuator reception clocks remain unknown',
            'Recorder receipt order cannot certify data available to the S8 control callback',
            'A monotonic receipt delay combines cache, ROS transport, scheduling, filtering and physical response',
            'Wheel speed uses driver convention 0.001; it is not independently calibrated ground speed',
            'All results are historical one/multi-step response diagnostics, not new feedback closed-loop performance',
            'Half/one-second response checks use historical commands and nonoverlapping anchors; they never reset to intermediate feedback',
            'Different run IDs do not prove independent continuous physical sessions',
            'Battery<=0/error/nonfinite feedback excluded conservatively, not proof remaining packets are fresh',
            'Five-percent loss profiles do not cover systematic uncertainty or certify confidence probability'],
        models={model:fit_response(training_runs, holdout_runs, model) for model in models})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--train-run', action='append', required=True)
    parser.add_argument('--diagnostic-holdout-run', action='append', default=[])
    parser.add_argument('--model', action='append', choices=sorted(MODELS))
    parser.add_argument('--command-publisher', default='/publish_speed_turn')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    try:
        train = [load_raw_run(p, args.command_publisher) for p in args.train_run]
        holdout = [load_raw_run(p, args.command_publisher) for p in args.diagnostic_holdout_run]
        result = analyze(train, holdout, args.model or sorted(MODELS))
    except (OSError, ValueError, TypeError, KeyError) as error:
        result = dict(schema_version=1, status='invalid', reason=str(error), eligible_for_runtime=False,
            physical_parameter_certification=False, production_or_safety_config_written=False,
            vehicle_closed_loop_improvement_claim=False)
    write_new(args.output, result)
    print(result['status'])


if __name__ == '__main__':
    main()
