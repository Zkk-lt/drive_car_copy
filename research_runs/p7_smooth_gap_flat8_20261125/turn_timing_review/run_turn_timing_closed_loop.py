#!/usr/bin/env python3
"""A/B synthetic smooth test; no ROS master, topic, driver or real data writes."""
import argparse
from collections import Counter, defaultdict
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
import subprocess


def scalar(row, name):
    return float(row[name])


def quantile(values, q):
    v = sorted(values)
    i = (len(v)-1)*q
    return v[math.floor(i)] + (i-math.floor(i))*(v[math.ceil(i)]-v[math.floor(i)])


def summarize(rows):
    def rms(values):
        return statistics.mean(x*x for x in values)**.5
    solver_ms = [scalar(row,'solve_ms') for row in rows]
    speeds = [scalar(row,'command_speed') for row in rows]
    steering = [scalar(row,'command_steering') for row in rows]
    signed_errors = [scalar(row,'outward_error') for row in rows]
    dt = .025

    return dict(
        ticks=len(rows),
        valid_selected=sum(r['selected_nmpc']=='1' for r in rows),
        solver_timeouts=sum(r['timeout']=='1' for r in rows),
        hard_command_violations=sum(r['violation']=='1' for r in rows),
        physical_above_dynamic_cap=sum(scalar(r,'actual_speed')>scalar(r,'hard_cap')+1e-7 for r in rows),
        published_above_dynamic_cap=sum(scalar(r,'command_speed')>scalar(r,'hard_cap')+1e-7 for r in rows),
        reserve_active=sum(scalar(r,'applied_reserve')>0 for r in rows),
        opening_weight_active=sum(scalar(r,'applied_release_weight')>0 for r in rows),
        release_weight_peak=max(scalar(r,'applied_release_weight') for r in rows),
        reserve_peak_mps=max(scalar(r,'applied_reserve') for r in rows),
        not_selected_reasons=dict(Counter(r['reason'] for r in rows if r['selected_nmpc']=='0')),
        absolute_center_error_p95_m=quantile([abs(e) for e in signed_errors],.95),
        absolute_center_error_max_m=max(abs(e) for e in signed_errors),
        signed_outer_p95_m=quantile(signed_errors,.95),
        center_error_rms_m=rms(signed_errors),
        heading_error_rms_rad=rms(scalar(r,'heading_error') for r in rows),
        absolute_speed_median_mps=quantile([scalar(r,'actual_speed') for r in rows],.50),
        final_station_m=scalar(rows[-1],'station'),
        command_steering_rate_rms_rad_s=rms((b-a)/dt for a,b in zip(steering,steering[1:])),
        command_jerk_rms_m_s3=rms((c-2*b+a)/dt**2 for a,b,c in zip(speeds,speeds[1:],speeds[2:])),
        solver_ms_p50=quantile(solver_ms,.50),
        solver_ms_p95=quantile(solver_ms,.95),
        solver_ms_p99=quantile(solver_ms,.99),
        solver_ms_max=max(solver_ms),
    )


def station_matched(old, new):
    def errors_by_station(rows):
        by_station = defaultdict(list)
        for row in rows:
            bin_index = int(scalar(row,'station')/.02 + 1e-9)
            by_station[bin_index].append(scalar(row,'outward_error'))
        return {index: statistics.mean(errors) for index, errors in by_station.items()}

    old_errors, new_errors = errors_by_station(old), errors_by_station(new)
    shared = sorted(old_errors.keys() & new_errors.keys())
    pairs = [[errors[i] for i in shared] for errors in (old_errors, new_errors)]
    return {
        'common_bins_of_2_cm':len(shared),
        'center_error_rms_m':[statistics.mean(error**2 for error in p)**.5 for p in pairs],
        'absolute_center_error_p95_m':[quantile([abs(error) for error in p],.95) for p in pairs],
        'outward_signed_p95_m':[quantile(p,.95) for p in pairs],
        'order':['old','new'],
    }


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--compiler',default='g++')
    parser.add_argument('--binary',type=Path,help='reuse existing compiled harness')
    parser.add_argument('--reserve',type=float,default=.025)
    parser.add_argument('--release',type=float,default=0.)
    parser.add_argument('--cap-profile',choices=('smooth','rapid','compound'),default='compound')
    args=parser.parse_args()
    out=args.output_dir.resolve()
    out.mkdir(parents=True,exist_ok=False)
    here=Path(__file__).resolve().parent
    binary=args.binary
    if binary is None:
        include=here.parents[2]/'src/move_car/include'
        binary=out/'turn_timing_closed_loop'
        subprocess.run([args.compiler,'-std=c++14','-O2','-DNDEBUG',
                        '-I'+str(include),'-I/usr/include/eigen3',
                        str(here/'turn_timing_closed_loop.cpp'),'-o',str(binary)],check=True)
    cases=(('old',0.,0.),('new',args.reserve,args.release))
    rows={}
    for label, speed, release in cases:
        tick_path=out/(label+'.csv')
        with tick_path.open('w') as f:
            subprocess.run([str(binary.resolve()),str(speed),str(release),args.cap_profile],stdout=f,check=True)
        with tick_path.open() as f:
            rows[label]=list(csv.DictReader(f))
    comparisons={}
    for scenario in dict.fromkeys(r['scenario'] for r in rows['old']):
        aa,bb=([x for x in rows[label] if x['scenario']==scenario] for label,_,_ in cases)
        phases=dict.fromkeys(x['phase'] for x in aa)
        comparisons[scenario]={'old':summarize(aa),'new':summarize(bb),
                               'station_matched':station_matched(aa,bb),
                               'phases':{phase:{'old':summarize([r for r in aa if r['phase']==phase]),
                                                'new':summarize([r for r in bb if r['phase']==phase])}
                                         for phase in phases if any(r['phase']==phase for r in bb)}}
        if scenario in ('straight','steady_right'):
            if any(scalar(r,'applied_reserve') or scalar(r,'applied_release_weight')
                   for r in aa+bb):
                raise AssertionError('turn timing activated in '+scenario)
            fields=('station','command_steering','command_speed','hard_cap','target_speed')
            comparisons[scenario]['exact_commands_and_station']=all(
                all(a[k]==b[k] for k in fields) for a,b in zip(aa,bb))
    timing={
        'scope':'Synthetic delayed bicycle; uses the production flat8 solver and P5 arbiter with a bounded synthetic fallback; not real-gain identification.',
        'dynamic_cap':{'profile':args.cap_profile,'scope':'Injected by simulated station in both sides, never changed by candidate.'},
        'hard_command_violations':'Synthetic recovery ceiling 0.50 m/s, acceleration, jerk, steering and steering rate bounds. The operational cap (≤0.40 m/s) is immutable for NMPC and counted separately for the fallback and plant.',
        'compiled_binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),
        'ablation_controls':{label:{'speed_reserve_mps':speed,'release_weight':release}
                             for label,speed,release in cases},
        'comparison':comparisons,
    }
    (out/'summary.json').write_text(json.dumps(timing,indent=2)+'\n')
    for scenario, q in comparisons.items():
        metric='center_error_rms_m'
        print('{} RMS [m] {:.5f} -> {:.5f}; active reserve/release {}/{}; exact {}'.format(
              scenario,q['old'][metric],q['new'][metric],q['new']['reserve_active'],
              q['new']['opening_weight_active'],q.get('exact_commands_and_station')))


if __name__=='__main__':
    main()
