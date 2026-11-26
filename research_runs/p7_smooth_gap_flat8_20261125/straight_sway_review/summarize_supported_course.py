from pathlib import Path
import argparse,hashlib,json
import pandas as pd
p=argparse.ArgumentParser()
p.add_argument('fixture', type=Path)
p.add_argument('pairs', type=Path)
p.add_argument('--output', type=Path)
args=p.parse_args()
fixture=args.fixture
meta=json.loads((fixture/'fixture.json').read_text())
d=pd.DataFrame(meta['rows']).merge(pd.read_csv(args.pairs),on='cycle',validate='one_to_one')
assert len(d)==len(meta['rows'])
assert d[['cap_unchanged','terminal_unchanged','mesh_unchanged']].eq(1).all().all()
assert (d.old_ok==d.old_validated).all() and (d.new_ok==d.new_validated).all()
def summary(z):
 a=z[(z.course_selected==1)&(z.old_ok==1)&(z.new_ok==1)].copy()
 quantiles=lambda s:{k:t for k,t in zip(('p50','p95','p99','max'),s.quantile([.5,.95,.99,1]).to_list())}
 def delta(field,absolute=False):
  old,new=(a[col+'_'+field] for col in ('old','new'))
  if absolute:old,new=old.abs(),new.abs()
  r=new-old
  return {'paired_changed':int(r.abs().gt(1e-6).sum()),'mean':r.mean(),'median':r.median(),'min':r.min(),'max':r.max()}
 r=dict(rows=len(z),old_ok=int(z.old_ok.sum()),new_ok=int(z.new_ok.sum()),old_timeouts=int(z.old_timeout.sum()),
        new_timeouts=int(z.new_timeout.sum()),course_selected=int(z.course_selected.sum()),changed_with_both_feasible=len(a),
        old_weight_active=int(z.old_weight.gt(0).sum()),new_weight_active=int(z.new_weight.gt(0).sum()),
        course_nominal_yaw_cost_active=int(z.course_rate_weight.gt(0).sum()),
        curvature_abs_p95=z.loc[z.course_selected==1,'course_curvature'].abs().quantile(.95),
        old_solve_ms=quantiles(z.old_solve_ms),new_solve_ms=quantiles(z.new_solve_ms),
        old_validated=int(z.old_validated.sum()),new_validated=int(z.new_validated.sum()),
        steering_change_rad=delta('steering'),steering_abs_change_rad=delta('steering',True),
        predicted_abs_yaw_rate_change_rad_s=delta('yaw_rate_at_0p6',True))
 return r
groups={name:summary(z) for name,z in d.groupby('window')}
headline=['time','window','old_blend','new_blend','course_curvature','course_selected','old_weight','new_weight','course_rate_weight','old_steering',
          'new_steering','old_yaw_rate_at_0p6','new_yaw_rate_at_0p6']
events=d.loc[d.time.isin(['17:56:10.799','17:56:11.024','17:56:11.249','17:56:12.149','17:56:12.599','17:56:13.049','17:56:13.274','17:57:38.849','17:57:39.074']),headline].to_dict('records')
report=dict(origin='frozen archived real command context; two cold starts; no trajectory feedback',
   run_id=meta['run_id'],raw_event_sha256=meta['original_event_sha256'],
   fixture_sha256=hashlib.sha256((fixture/'requests.txt').read_bytes()).hexdigest(),
   windows=meta['windows'],groups=groups,events=events,counts=meta['counts'],pairs_sha256=hashlib.sha256(args.pairs.read_bytes()).hexdigest(),
   raw_hard_cap_mesh_terminal_all_preserved=True)
def clean(x):
 if isinstance(x,float) and pd.isna(x):return None
 if isinstance(x,dict):return {k:clean(v) for k,v in x.items()}
 if isinstance(x,list):return [clean(v) for v in x]
 return x
if args.output:args.output.write_text(json.dumps(clean(report),ensure_ascii=False,allow_nan=False,indent=2)+'\n')
for key,z in groups.items():print(key,z['old_ok'],z['new_ok'],z['course_selected'],z['changed_with_both_feasible'],z['old_weight_active'],z['new_weight_active'],z['steering_abs_change_rad'],z['predicted_abs_yaw_rate_change_rad_s'],z['old_solve_ms']['p95'],z['new_solve_ms']['p95'])
print(pd.DataFrame(events).to_string(index=False,float_format=lambda x:f'{x:.4f}'))
