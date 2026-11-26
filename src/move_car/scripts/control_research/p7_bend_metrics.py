"""Causal bend diagnostics for archived P7 runs, without a vehicle model fit.

The front axle and the body center are different tracking points.  In a turn,
the front axle can be outside the center route even when the center follows it.
The two logged distances below are to different trajectories as well. Neither
is an independently measured tire-to-tire tracking error.
"""

from bisect import bisect_left, bisect_right
from collections import Counter, defaultdict
import csv
from datetime import datetime, timedelta, timezone
from decimal import Decimal, InvalidOperation
import math
from pathlib import Path


_NS = 10 ** 9
_PUBLISH_MATCH_NS = 10 ** 6
_MAX_ORIGIN_AGE_NS = 55 * 10 ** 6
_MAX_RUN_GAP_NS = 160 * 10 ** 6


def _stamp(value):
    """Preserve the full ROS nanoseconds from CSV text or a message stamp."""
    try:
        if isinstance(value, dict):
            return int(value['secs']) * _NS + int(value['nsecs'])
        return int(Decimal(str(value)) * _NS)
    except (TypeError, ValueError, KeyError, InvalidOperation, OverflowError):
        return None


def _number(row, key):
    try:
        value = float(row[key])
    except (TypeError, ValueError, KeyError):
        return None
    return value if math.isfinite(value) else None


def _timed_csv(path, fields):
    with path.open(newline='') as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            stamp = _stamp(row.get('stamp'))
            if stamp is not None:
                yield stamp, {name: row.get(name) for name in fields}


def _summary(values):
    if not values:
        return dict(samples=0, median_m=None, signed_p95_m=None,
                    abs_p95_m=None, max_signed_m=None)

    def percentile(ordered, q):
        index = (len(ordered) - 1) * q
        low = int(index)
        return ordered[low] + (index - low) * (ordered[math.ceil(index)] - ordered[low])

    signed = sorted(values)
    return dict(samples=len(values), median_m=percentile(signed, .5),
                signed_p95_m=percentile(signed, .95),
                abs_p95_m=percentile(sorted(abs(value) for value in values), .95),
                max_signed_m=signed[-1])


def _publication_index(publications):
    index = {}
    for message in publications:
        payload = message.get('payload', message) if isinstance(message, dict) else {}
        if payload.get('history_committed') is False or payload.get('publish_returned') is False:
            continue
        cycle = _stamp(payload.get('cycle_stamp'))
        published = _stamp(payload.get('publish_ros_start'))
        owner = payload.get('owner')
        if cycle is not None and published is not None and cycle <= published and isinstance(owner, str):
            index[cycle] = owner
    return index


def _nominal_offset(curve, turn_sign):
    """Near-steady axle/center radius difference, not a compensated sensor."""
    if len(curve) < 6:
        return None
    stations = [0.]
    distance, closest = math.inf, 0.
    for a, b in zip(curve, curve[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        step = math.hypot(dx, dy)
        if step <= 1e-6 or step > .12:
            return None
        if stations[-1] < .85:
            # Project the .30 m front lever onto the nearby execution geometry.
            f = max(0., min(1., ((.30 - a[0])*dx - a[1]*dy)/(step*step)))
            error = math.hypot(a[0] + f*dx - .30, a[1] + f*dy)
            if error < distance:
                distance, closest = error, stations[-1] + f*step
        stations.append(stations[-1] + step)
        if stations[-1] > 1.10:
            break
    if closest < .20 or closest + .20 > stations[-1]:
        return None

    def interpolate(s):
        i = min(len(stations) - 1, bisect_right(stations, s))
        f = (s - stations[i - 1])/(stations[i] - stations[i - 1])
        a, b = curve[i - 1], curve[i]
        return tuple(a[j] + f*(b[j] - a[j]) for j in range(2))

    a, b, c, d, e = [interpolate(closest + f) for f in (-.2, -.1, 0., .1, .2)]
    angles = [math.atan2(q[1] - p[1], q[0] - p[0]) for p, q in zip((a,b,c,d),(b,c,d,e))]
    curvatures = [turn_sign*math.remainder(y - x, 2.*math.pi)/.10
                  for x, y in zip(angles, angles[1:])]
    mean = sum(curvatures)/len(curvatures)
    if min(curvatures) < .12 or mean > .90 or max(curvatures) - min(curvatures) > .12:
        return None
    radius = 1./mean
    # R_f² = R_c² + L² - ell² for the .60/.30 m steady bicycle.
    radius_delta = .27/(math.hypot(radius, math.sqrt(.27)) + radius)
    return radius_delta


def analyze_bend_geometry(trial: Path, publications: list):
    """Compare front/center lateral error estimates at matched archived cycles.

    Runtime CSV rows are sparse, but timestamped at the control cycle. Match
    the publication owner for that exact cycle within 1 ms. Use only the last
    reconstructor origin CSV row at or before that cycle, no older than 55 ms.
    """
    trial = Path(trial)
    runtime = trial/'s8_csv_live'/'runtime_debug.csv'
    origin = next(iter(sorted((trial/'reconstructor_csv').glob('origin_reference*.csv'))), None)
    result = dict(
        status='unavailable', independent_position_truth=False,
        frames='right/left sign is corroborated by feedforward curvature, measured steer and raw IMU; positive is outward in each turn',
        matching=('runtime cycle and committed command within 1 ms; origin reconstruction and '
                  'pose timestamps must be at or before both matched cycle stamps. '
                  'This CSV join does not establish when an origin row arrived at the controller.'),
        limitations=('The original front-axle error is relative to the ego-anchored execution CENTER path. '
                     'Observed-history error uses the body center and measured-history support. '
                     'The steady-circle axle radius offset is a geometry diagnostic, not an actual tire-to-tire error or an instruction to lower right-turn gain.'),
        right={}, left={},
    )
    if not runtime.is_file() or origin is None:
        result['reason'] = 'runtime_or_observed_origin_csv_missing'
        return result
    result['coverage'] = ('Strict agreement group, not every real turn. Entries and exits '
                          'can have a small immediate feedforward curvature while a bend is '
                          'visible farther along the path.')
    result['limits'] = dict(speed_proxy_min_m_s=.15, preview_curvature_abs_min_inv_m=.12,
                            steering_abs_min_rad=.08, raw_imu_yaw_abs_min_rad_s=.045,
                            sustained_window_min_s=.15, max_gap_between_runtime_samples_s=.16)
    origin_fields = ('pose_stamp', 'source_stamp', 'speed', 'delta', 'historical_ey', 'historical_yaw',
                     'history_supported', 'using_history_cache', 'recovery_braking_only')
    runtime_fields = ('reference_geometry_valid', 'reference_status_mode', 'reference_held',
                      'kappa_ff', 'yaw_rate_raw_imu', 'yaw_rate_valid',
                      'imu_yaw_rate_unavailable', 'front_axle_e_ct')
    history = sorted(_timed_csv(origin, origin_fields), key=lambda x: x[0])
    origin_stamps = [stamp for stamp, _ in history]
    index = _publication_index(publications)
    publish_cycles = sorted(index)
    frames = {'right': [], 'left': []}
    start = previous = direction = previous_history_yaw = None
    for cycle, runtime_row in sorted(_timed_csv(runtime, runtime_fields), key=lambda x: x[0]):
        # Near identifier match is allowed, but reject owner ambiguity.
        j = bisect_left(publish_cycles, cycle - _PUBLISH_MATCH_NS)
        matches = publish_cycles[j:bisect_right(publish_cycles, cycle + _PUBLISH_MATCH_NS)]
        owner = index[matches[0]] if len(matches) == 1 else 'unmatched'
        cutoff = min(cycle, matches[0]) if len(matches) == 1 else cycle
        i = bisect_right(origin_stamps, cutoff) - 1
        if i < 0 or cutoff - origin_stamps[i] > _MAX_ORIGIN_AGE_NS:
            start = previous = direction = None
            continue
        source_stamp, row = history[i]
        pose_time, observed = _stamp(row['pose_stamp']), _stamp(row['source_stamp'])
        if (pose_time is None or observed is None or not 0 < pose_time <= cutoff or
                not 0 < observed <= source_stamp or row['history_supported'] != '1' or
                row['using_history_cache'] != '0' or row['recovery_braking_only'] == '1' or
                runtime_row['reference_geometry_valid'] != '1' or
                runtime_row['reference_held'] != '0' or runtime_row['reference_status_mode'] != 'TRACK_GOOD'):
            start = previous = direction = None
            continue
        speed, delta, center = (_number(row, key) for key in ('speed','delta','historical_ey'))
        kappa, yaw, front = (_number(runtime_row, key)
                             for key in ('kappa_ff','yaw_rate_raw_imu','front_axle_e_ct'))
        if (None in (speed, delta, center, kappa, yaw, front) or speed <= .15 or
                runtime_row['yaw_rate_valid'] != '1' or runtime_row['imu_yaw_rate_unavailable'] != '0'):
            start = previous = direction = None
            continue
        sign = -1 if kappa < 0. else 1
        if sign*kappa < .12 or sign*delta < .08 or sign*yaw < .045:
            start = previous = direction = None
            continue
        history_yaw = _number(row, 'historical_yaw')
        consecutive = direction == sign and previous is not None and cycle - previous <= _MAX_RUN_GAP_NS
        if not consecutive:
            start = cycle
        tangent_jump = (consecutive and history_yaw is not None and
                        previous_history_yaw is not None and
                        abs(math.remainder(history_yaw - previous_history_yaw, 2.*math.pi)) > .25)
        previous, direction = cycle, sign
        previous_history_yaw = history_yaw
        frames['right' if sign < 0 else 'left'].append(
            dict(cycle=cycle, source_stamp=source_stamp, outward_center=-sign*center,
                 outward_front=-sign*front, sustained=(cycle - start >= 150 * 10 ** 6),
                 historical_tangent_changed_gt_0p25_rad=tangent_jump,
                 wheel_steering_rad=delta, imu_yaw_rate_rad_s=yaw,
                 preview_kappa_inv_m=kappa, speed_proxy_m_s=speed, owner=owner))

    # Point snapshots are decimated. Evaluate circle offsets only where the
    # matching origin row has an exactly timestamped execution polyline.
    need = {row['source_stamp'] for samples in frames.values() for row in samples}
    point_file = next(iter(sorted((trial/'reconstructor_csv').glob('origin_points*.csv'))), None)
    curves = defaultdict(list)
    if need and point_file is not None:
        with point_file.open(newline='') as stream:
            for record in csv.DictReader(stream):
                if record.get('kind') != 'execution':
                    continue
                stamp = _stamp(record.get('stamp'))
                if stamp not in need:
                    continue
                point = (_number(record, 'center_x'), _number(record, 'center_y'))
                if None not in point:
                    curves[stamp].append(point)
    tz = timezone(timedelta(hours=8))

    def describe(samples, turn_sign):
        outward_center = [row['outward_center'] for row in samples]
        outward_front = [row['outward_front'] for row in samples]
        nominal = []
        for row in samples:
            estimate = _nominal_offset(curves.get(row['source_stamp'], []), turn_sign)
            if estimate is not None:
                nominal.append((estimate, row['outward_front'] - estimate))

        def example(key, choose=max):
            if not samples:
                return None
            row = choose(samples, key=lambda a: a[key])
            stamp = datetime.fromtimestamp(row['cycle'] / _NS, tz).isoformat(timespec='milliseconds')
            return dict(time_utc8=stamp, owner=row['owner'],
                        front_axle_vs_execution_center_outward_m=row['outward_front'],
                        observed_history_body_center_outward_m=row['outward_center'],
                        preview_kappa_inv_m=row['preview_kappa_inv_m'],
                        measured_steering_rad=row['wheel_steering_rad'],
                        raw_imu_yaw_rate_rad_s=row['imu_yaw_rate_rad_s'])

        return dict(
            sampled_cycles=len(samples), publication_owners=dict(Counter(row['owner'] for row in samples)),
            front_axle_vs_execution_center_outward=_summary(outward_front),
            observed_history_body_center_outward=_summary(outward_center),
            observed_history_center_outward_ge_4cm=sum(value >= .04 for value in outward_center),
            observed_history_center_inward_ge_4cm=sum(value <= -.04 for value in outward_center),
            historical_tangent_jumps_gt_0p25_rad=sum(
                row['historical_tangent_changed_gt_0p25_rad'] for row in samples),
            nominal_steady_front_axle_outward=_summary([x[0] for x in nominal]),
            approx_front_outward_minus_steady_offset=_summary([x[1] for x in nominal]),
            largest_front_outward_example=example('outward_front'),
            largest_body_center_outward_example=example('outward_center'),
            largest_body_center_inward_example=example('outward_center', choose=min),
        )

    for side, sign in (('right', -1), ('left', 1)):
        samples = frames[side]
        result[side] = dict(all_directional_samples=describe(samples, sign),
                            sustained_samples=describe([row for row in samples if row['sustained']], sign))
    result['status'] = 'ok' if any(frames.values()) else 'insufficient_supported_bend_samples'
    return result
