#!/usr/bin/env python3
"""Read-only legacy CSV inventory and metrics; never invent raw sensor channels."""
import argparse,csv,hashlib,json,math
from collections import Counter
from pathlib import Path
from control_research.analysis import stats
from control_research.dataset import write_new

def finite(value):
 try:x=float(value);return x if math.isfinite(x) else None
 except (TypeError,ValueError):return None

def audit(root):
 release=json.loads((root/'releases/S8/manifest.json').read_text());accepted=release['accepted_run']['control'];runs=[]
 for path in sorted((root/'cdata').glob('origin_control_*.csv')):
  with path.open() as f:rows=list(csv.DictReader(f))
  ids=sorted(set(r.get('run_id','') for r in rows));expected=path.stem[len('origin_control_'):]
  if ids!=[expected]:raise ValueError('mixed/missing run_id: '+str(path))
  valid=[r for r in rows if r.get('path_valid')=='1'];rates=[];last=None;nonmonotonic=0;gaps=0
  for r in rows:
   t,u=finite(r.get('stamp')),finite(r.get('cmd'))
   if last and t is not None and u is not None:
    dt=t-last[0]
    if 0<dt<=.2:rates.append(abs((u-last[1])/dt))
    elif dt<=0:nonmonotonic+=1
    else:gaps+=1
   last=(t,u) if t is not None and u is not None else None
  def metric(key):return stats(abs(x) for r in valid for x in [finite(r.get(key))] if x is not None)
  timestamps=[finite(r.get('stamp')) for r in rows];timestamps=[x for x in timestamps if x is not None]
  revisions=sorted(set(r.get('code_revision','unrecorded') for r in rows))
  runs.append(dict(run_id=expected,path=str(path.relative_to(root)),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
    rows=len(rows),tracking_rows=len(valid),source_time_range=[min(timestamps),max(timestamps)] if timestamps else None,
    nonmonotonic_stamps=nonmonotonic,gaps_over_0p2s=gaps,code_revisions=revisions,
    accepted_S8=expected==accepted,terrain_label='smooth' if expected==accepted else 'unknown',
    terrain_evidence='user confirmed S8 indoor smooth accepted run + releases/S8/manifest.json' if expected==accepted else 'needs operator annotation',
    lateral_error_abs_m=metric('ey'),body_heading_error_abs_rad=metric('body_heading_error'),course_error_abs_rad=metric('course_error'),
    command_steering_rate_abs_radps=stats(rates),steering_receipt_age_s=metric('steering_receive_age'),
    modes=dict(Counter(r.get('mode') for r in rows)),stop_reasons=dict(Counter(r.get('stop_reason') for r in rows)),
    unavailable={'independent_speed_error':'no independent raw ground-speed channel','slip':'wheel/NED independence and raw paired stamps absent',
      'roughness':'raw vertical IMU unavailable in per-run control CSV','solver_runtime':'not recorded in this schema',
      'physical_jerk':'command derivatives cannot substitute physical acceleration','config_hash':'per-run full configuration hash absent'},
    source_kind='legacy_controller_diagnostic_csv',eligible_for_parameter_identification=False,eligible_for_candidate_tuning=False,
    split='quarantined_until_session_and_provenance_review'))
 inventory=[]
 for dirname in ('data','cdata'):
  for path in sorted((root/dirname).rglob('*')):
   if path.is_file():inventory.append({'path':str(path.relative_to(root)),'bytes':path.stat().st_size})
 return dict(schema_version=1,run_count=len(runs),runs=runs,inventory=inventory,
   accepted_S8_pair=release['accepted_run'],release_manifest_sha256=hashlib.sha256((root/'releases/S8/manifest.json').read_bytes()).hexdigest(),
   limitation='Historical controller metrics, not independent physical ground truth. Do not match overwritten global manifest to per-run CSV by directory alone. Raw observations are tracked target points, not independent vehicle motion. Whole-run quarantine prevents leakage and fabricated calibration.')

def main():
 p=argparse.ArgumentParser();p.add_argument('--workspace',required=True,type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args()
 out=audit(a.workspace.resolve());write_new(a.output,out);print('PASS',out['run_count'],'legacy runs audited; no production data modified')
if __name__=='__main__':main()
