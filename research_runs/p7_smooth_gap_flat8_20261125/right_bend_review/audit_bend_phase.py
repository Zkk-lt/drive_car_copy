#!/usr/bin/env python3
"""Read a verified real smooth archive; compare right-bend Center and axle metrics.

Requires numpy and pandas, does not modify the archive.
"""
import argparse
from pathlib import Path
import json
import re
from collections import defaultdict
import pandas as pd,numpy as np
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('archive',type=Path)
parser.add_argument('--episode',action='append',nargs=3,metavar=('LABEL','START_CST','END_CST'),
                    help='Optional wall-time range HH:MM:SS[.sss], use same recording day')
args=parser.parse_args()
p=args.archive
f=next(p.glob('raw/*/events.jsonl'))
data=defaultdict(list)
def ts(t):return t['secs']+1e-9*t['nsecs']
def val(d,k):return float(d.get(k,'nan'))
def geom(z,lo,hi):
 if len(z)<4 or z[0,0]>lo or z[-1,0]<hi:return np.nan
 s=np.linspace(lo,hi,3)
 xy=np.array([np.interp(s,z[:,0],z[:,1]),np.interp(s,z[:,0],z[:,2])]).T
 a,b=np.diff(xy,axis=0)
 return np.arctan2(np.cross(a,b),a@b)/(.5*(hi-lo))
with f.open('rb') as inp:
 for l in inp:
  mo=re.search(rb'"topic": "([^"]+)"',l[:800])
  if not mo:continue
  k=mo[1];k=k.rsplit(b'/',1)[-1]
  if k not in (b'published', b'candidate', b'diagnostics', b'request', b'imu',b'car_message'):continue
  o=json.loads(l);d=o['payload'];t=o['receipt_ros_ns']*1e-9
  if k==b'published' and d['publish_returned'] and d['history_committed']:
   data[k].append((d['cycle'],d['sequence'],ts(d['publish_ros_start']),ts(d['cycle_stamp']),d['owner'],d['steering'],d['speed'],d['reason']))
  elif k==b'diagnostics':
   data[k].append((d['cycle'],d['candidate_cycle'],d['publish_interval_ms'],d['reason']))
  elif k==b'request':
   d0=d['base'];z=np.array([(q['s'],q['x'],q['y']) for q in d0['reference']])
   data[k].append((d0['cycle'],d0['reference_valid'],d0['baseline_steering'],d0['speed_cap'],
                  *(geom(z,lo,hi) for lo,hi in ((0.05,.55),(.3,.9),(.6,1.2),(.9,1.5)))))
  elif k==b'candidate':
   reason=d['base']['reason'];fields=dict(x.split('=',1) for x in reason.split(';') if '=' in x)
   d0=d['base'];data[k].append((d0['cycle'],d0['previous_sequence'],d0['valid'],d0['steering'],d0['solve_ms'],
                   val(fields,'straight_tracking_blend'),val(fields,'straight_yaw_damping_weight'),d['feedback_correction'],reason))
  elif k==b'imu': data[k].append((t,d['angular_velocity']['z']))
  elif k==b'car_message':data[k].append((t,d['turn_angle'],.001*d['back_wheel_speed']))

d=pd.DataFrame(data[b'published'],columns='cycle seq t cycle_stamp owner cmd speed reason'.split()).sort_values('seq')
d['rate']=d.cmd.diff()/d.t.diff()
d['switch']=d.owner.ne(d.owner.shift())
d['reverse']=(d.rate*d.rate.shift()<0)&(d.rate.abs()>.2)&(d.rate.shift().abs()>.2)&d.seq.diff().eq(1)&d.t.diff().between(.001,.08)
for k,cs in ((b'imu','t imu'),(b'car_message','t delta wheel_speed')):
 a=pd.DataFrame(data[k],columns=cs.split()).sort_values('t'); d=pd.merge_asof(d.sort_values('t'),a,on='t',direction='backward',tolerance=.15)
for k,cs in ((b'diagnostics','cycle ccycle pub_ms diagnostic'),(b'request','cycle v baseline cap k05 k30 k60 k90'),(b'candidate','ccycle prev valid pred solve blend damp fb candidate_reason')):
 names=cs.split()
 d=d.merge(pd.DataFrame(data[k],columns=names).drop_duplicates(names[0],keep='last'),on=names[0],how='left')
cs='stamp path_valid front_axle_e_ct track_signed_e_y e_theta_control delta_ref delta_ref_lateral delta_ff straight_lock kappa_ff kappa_exec_raw kappa_speed_preview kappa_path kappa_cmd kappa0 kappa_ref_delay curve_evidence_reliable ref_tracking_class reference_status_mode yaw_rate_ref near_curve_yaw_change preview_x preview_y inside_error_front active_steer_limit'.split()
a=pd.read_csv(p/'s8_csv_live/runtime_debug.csv',usecols=cs).sort_values('stamp')
d=pd.merge_asof(d.sort_values('cycle_stamp'),a,left_on='cycle_stamp',right_on='stamp',direction='nearest',tolerance=.05)
cols='stamp historical_ey historical_yaw history_supported initial_center_kappa coordinate_kappa_max residual mode reject selected_remaining'.split();a=pd.read_csv(next(p.glob('reconstructor_csv/origin_reference*.csv')),usecols=lambda c:c in cols).sort_values('stamp')
d=pd.merge_asof(d.sort_values('cycle_stamp'),a,left_on='cycle_stamp',right_on='stamp',direction='backward',tolerance=.1,suffixes=('','_recon'))
d=d.sort_values('seq')
d['wall']=pd.to_datetime(d.t,unit='s',utc=True).dt.tz_convert('Asia/Shanghai').dt.strftime('%H:%M:%S.%f')
d['curv_yaw']=d.imu/d.wheel_speed.where(d.wheel_speed.gt(.18))
d['axle_nominal_outer']=np.hypot(
    .60/np.tan(d.delta.where(d.delta.abs().gt(.02))),.60)-np.hypot(
    .60/np.tan(d.delta.where(d.delta.abs().gt(.02))),.30)
d['applied_vs_fallback']=d.cmd-d.baseline
d['nm_selected_match']=(d.owner.eq('nmpc') & d.valid.eq(True) & d.prev.eq(d.seq-1))
moving=d.speed.gt(.16)&d.wheel_speed.gt(.18)&d.v.eq(True)&d.path_valid.eq(1)
supported=moving&d.history_supported.eq(1)&d.reference_status_mode.eq('TRACK_GOOD')
right=supported&d.k30.lt(-.17)&d.k60.lt(-.17)&d.delta.lt(-.08)
left=supported&d.k30.gt(.17)&d.k60.gt(.17)&d.delta.gt(.08)

def q(series, quantile):
    v=float(series.quantile(quantile))
    return v if np.isfinite(v) else None

def summary(mask):
    x=d[mask]
    fields='front_axle_e_ct historical_ey k05 k30 k60 k90 curv_yaw wheel_speed cmd delta imu damp applied_vs_fallback axle_nominal_outer'.split()
    def counts(field): return {str(k):int(v) for k,v in x[field].value_counts().items()}
    out=dict(cycles=len(x),owner=counts('owner'),logger_mode=counts('mode'),
             flat8_candidate_sequence_match=int(x.nm_selected_match.sum()),
             passes_nonzero_yaw_damping=int(x.loc[x.nm_selected_match,'damp'].gt(0).sum()),
             both_runtime_and_history_available=int((x.front_axle_e_ct.notna()&x.historical_ey.notna()).sum()),
             fields={f:{k:q(x[f],p) for k,p in [('min',0),('p10',.1),('median',.5),('p90',.9),('max',1)]}
                     for f in fields})
    return out

manifest=json.loads((p/'RAM_ARCHIVE_MANIFEST.json').read_text())
meta=json.loads((p/'trial.json').read_text())
episodes=args.episode or ([('tightening_entry','17:01:21.798','17:01:23.470'),
                           ('reacquired_curve_releasing_steer','17:01:31.425','17:01:33.265'),
                           ('curve_release_subset','17:01:32.848','17:01:33.265')]
                          if p.name=='p7real_20261126T165921_e62f8981' else [])
outputs={}
for label,start,end in episodes:
    m=d.wall.ge(start)&d.wall.le(end+'9'*(15-len(end))) if '.' in end else d.wall.ge(start)&d.wall.lt(end)
    # Transition windows are separately shown before the stricter sustained-
    # curve filter; intended to include mode switching and entry/exit phase.
    outputs[label]=dict(wall_time_cst=[start,end],
        supported_episode=summary(m&d.v.eq(True)&d.history_supported.eq(1)),
        sustained_right_subset=summary(m&right))
result=dict(run_id=p.name,binaries=meta['binaries'],
    original_event_sha256=manifest['files'][str(f.relative_to(p))]['sha256'],
    actual_published_commands=len(d),
    time_matching=dict(CAN_IMU='latest receipt no later than publish, age≤0.15s',
      runtime='nearest stamp within 0.05s; sampled debug estimate, not higher-rate truth',
      reconstructor='latest timestamp at/before current cycle within 0.10s'),
    sustained_curve_gate='published speed>0.16 m/s and receipt-domain wheel speed>0.18 m/s; valid request/runtime path and TRACK_GOOD; measured history supported; two requested-path position chord curvature windows 0.30–0.90 m and 0.60–1.20 m each <−0.17 m⁻¹ for right, >+0.17 for left; actual CAN steering same sign and |steer|>0.08 rad',
    center_error='historical_ey reflects left-positive vehicle-center projection onto supported point history; not ground truth and selected edge can switch',
    front_error='left-positive projection of front axle onto executable CENTER reference; do not substitute for center tracking error',
    geometry_front_offset='hypot(L/tan(delta),L)−hypot(L/tan(delta),rear_to_center), steady no-slip nominal magnitudes only',
    curve_metric='k05,k30,k60,k90 are signed spatial chord turn in .05–.55m, .30–.90m, .60–1.20m, .90–1.50m of requested body-center path; not applied-time curvature; curv_yaw uses IMU yaw rate divided by the wheel-speed proxy and is not independent ground truth',
    right_all=summary(right),right_flat8=summary(right&d.owner.eq('nmpc')),
    right_fallback=summary(right&d.owner.eq('legacy_slqr')),
    left_all=summary(left),
    right_front_over_4cm=summary(right&d.front_axle_e_ct.gt(.04)),
    right_history_center_outside_over_4cm=summary(right&d.historical_ey.gt(.04)),
    episodes=outputs)
print(json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False))
