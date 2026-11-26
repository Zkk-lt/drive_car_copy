"""P4-0 immutable whole-run dataset and candidate contracts. Python 3, no ROS."""
import csv,hashlib,json,math
from pathlib import Path
from .analysis import summarize
from .config import load_config,validate_run_id,TERRAINS

def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def write_new(path,value):
 path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
 with path.open('x') as f:json.dump(value,f,indent=2,sort_keys=True,allow_nan=False);f.write('\n')
def load_run(path):
 path=Path(path);m=json.loads((path/'manifest.json').read_text());validate_run_id(m['run_id'])
 if m.get('state')!='closed':raise ValueError('run must be closed')
 w=m.get('writer',{})
 if w.get('writer_error') or w.get('queue_dropped',1)!=0 or w.get('unwritten',1)!=0:raise ValueError('incomplete recording')
 if m.get('terrain_label') not in TERRAINS:raise ValueError('unknown terrain label')
 events=[json.loads(x) for x in (path/'events.jsonl').read_text().splitlines() if x.strip()]
 if len(events)!=w.get('written') or w.get('accepted')!=w.get('written'):raise ValueError('event count mismatch')
 if any(e.get('run_id')!=m['run_id'] for e in events):raise ValueError('mixed run_id')
 metrics=summarize(events,m)
 if metrics['run_id_mismatch_events']:raise ValueError('payload run_id mismatch')
 return m,events,metrics

def index_runs(root,plan):
 # Splits are an explicit experiment plan, never adjacent random tick slicing.
 rows=[];sessions={};seen=set();content=set()
 for entry in plan['runs']:
  rid=validate_run_id(entry['run_id']);session=validate_run_id(entry['continuous_session_id']);split=entry['split']
  if rid in seen or split not in ('train','validation','test'):raise ValueError('duplicate run or invalid split')
  seen.add(rid)
  if session in sessions and sessions[session]!=split:raise ValueError('continuous-session leakage')
  sessions[session]=split;m,events,metrics=load_run(Path(root)/rid)
  if m['run_id']!=rid:raise ValueError('directory/run mismatch')
  if m.get('continuous_session_id') and m['continuous_session_id']!=session:raise ValueError('session declaration mismatch')
  fingerprint=digest(Path(root)/rid/'events.jsonl')
  if fingerprint in content:raise ValueError('duplicate event content')
  content.add(fingerprint)
  rows.append(dict(run_id=rid,continuous_session_id=session,split=split,terrain=m['terrain_label'],
    data_origin=m.get('data_origin','unverified'),config_hash=m.get('config',{}).get('config_hash'),
    manifest_sha256=digest(Path(root)/rid/'manifest.json'),events_sha256=fingerprint,metrics=metrics))
 coverage={terrain:{s:sum(r['terrain']==terrain and r['split']==s for r in rows) for s in ('train','validation','test')} for terrain in TERRAINS}
 return dict(schema_version=1,runs=rows,coverage=coverage,split_unit='whole_run_and_continuous_session',
   limitation='Session identity is operator declared; cross-session route/time leakage requires audit. No independent ground truth implied.')

def generate_candidates(index,config_root,terrain):
 runs=[r for r in index['runs'] if r['terrain']==terrain]
 if any(not any(r['split']==s for r in runs) for s in ('train','validation','test')):raise ValueError('three held-out run groups required')
 if any(r['data_origin']!='real_vehicle' for r in runs):raise ValueError('real-vehicle provenance declaration required; synthetic data cannot generate research candidates')
 config=load_config(str(config_root),terrain)
 # Initial bounded local search: only two C-class performance scales. All
 # physical/safety limits and B-class identification are outside search space.
 candidates=[]
 for lateral in (.95,1.,1.05):
  for smooth in (1.,1.10):
   params={'lateral_weight_scale':lateral,'smoothness_weight_scale':smooth}
   payload={'schema_version':1,'terrain':terrain,'status':'unapproved','parameters':params,
    'baseline_config_hash':config['config_hash'],'safety_hash':config['file_hashes']['safety.yaml'],
    'source_runs':{s:[{'run_id':r['run_id'],'sha256':r['events_sha256']} for r in runs if r['split']==s] for s in ('train','validation','test')},
    'model_calibration_status':'required_before_closed_loop','closed_loop_evidence':None,'vehicle_improvement_claim':False}
   payload['candidate_id']=hashlib.sha256(json.dumps(payload,sort_keys=True).encode()).hexdigest()[:20]
   candidates.append(payload)
 return candidates

def calibration_fit(rows,contract):
 required=('ground_independent','world_to_body_verified','wheel_units_verified','steering_units_verified','source_time_verified','deadtime_verified')
 invalid=[k for k in required if contract.get(k) is not True]
 result={'status':'invalid','reason':'unverified_contract','missing':invalid,'yaw_gain':None,'steering_tau':None,'physical_delay_identified':False}
 if invalid:return result
 # Rows are externally synchronized/calibrated SI data; no fused speed reuse.
 if len(rows)<60:return dict(result,reason='insufficient_samples')
 if len(set(r['run_id'] for r in rows))!=1:return dict(result,reason='fit_one_complete_training_run_at_a_time')
 times=[float(r['time']) for r in rows]
 if any(not math.isfinite(x) for x in times) or any(b<=a for a,b in zip(times,times[1:])):return dict(result,reason='bad_timestamps')
 x=[];y=[];sx=[];sy=[]
 for r in rows:
  try:v,steer,yaw,delayed,rate=[float(r[k]) for k in ('ground_speed','steering','yaw_rate','delayed_steering_command','steering_rate')]
  except (KeyError,ValueError):return dict(result,reason='missing_calibrated_SI_columns')
  if not all(math.isfinite(q) for q in (v,steer,yaw,delayed,rate)):return dict(result,reason='nonfinite_sample')
  if abs(v)>=.15:
   phi=v*math.tan(steer)/.6
   if abs(phi)>=.025:x.append(phi);y.append(yaw)
  if abs(delayed-steer)>=.015:sx.append(delayed-steer);sy.append(rate)
 def regression(a,b):
  if len(a)<30:return None
  mean=sum(a)/len(a);var=sum((q-mean)**2 for q in a)/len(a)
  if var<1e-4:return None
  my=sum(b)/len(b);slope=sum((q-mean)*(v-my) for q,v in zip(a,b))/(len(a)*var);bias=my-slope*mean
  return dict(slope=slope,bias=bias,regressor_variance=var,n=len(a),residual_rmse=math.sqrt(sum((v-slope*q-bias)**2 for q,v in zip(a,b))/len(a)))
 yaw=regression(x,y);steering=regression(sx,sy)
 if not yaw or not steering:return dict(result,reason='weak_excitation')
 if not .5<=yaw['slope']<=1.5 or not 1.<=steering['slope']<=20.:return dict(result,reason='fit_out_of_model_bounds')
 return dict(result,status='candidate_fit_only',reason='requires_independent_validation_run',yaw_gain=yaw['slope'],steering_tau=1./steering['slope'],yaw_fit=yaw,steering_fit=steering,confidence='unvalidated',source_run=rows[0]['run_id'])

def validate_fit(fit,rows,contract):
 check=calibration_fit(rows,contract)
 if check['status']!='candidate_fit_only' or fit.get('status')!='candidate_fit_only':raise ValueError('observable calibrated samples and training fit required')
 if check['source_run']==fit['source_run']:raise ValueError('validation run must be independent')
 yg,tau=fit['yaw_gain'],fit['steering_tau'];nominal=[];candidate=[]
 for r in rows:
  v,d,y,u,rate=[float(r[k]) for k in ('ground_speed','steering','yaw_rate','delayed_steering_command','steering_rate')]
  if abs(v)<.15:continue
  phi=v*math.tan(d)/.6
  nominal.append((y-phi)**2+(rate-(u-d)/.44)**2)
  candidate.append((y-yg*phi-fit['yaw_fit']['bias'])**2+(rate-(u-d)/tau-fit['steering_fit']['bias'])**2)
 if not nominal:raise ValueError('no validation samples')
 base=sum(nominal)/len(nominal);actual=sum(candidate)/len(candidate)
 out=dict(fit,validation_run=check['source_run'],nominal_validation_mse=base,candidate_validation_mse=actual,
   validation_scope='one-step yaw/steering equations, not vehicle closed-loop performance')
 out['status']='validated_on_independent_runs' if actual<.8*base else 'validation_rejected'
 out['confidence']='held_out_equation_fit_only' if actual<.8*base else 'low_confidence'
 out['reason']='held_out_equation_mse_improved' if actual<.8*base else 'insufficient_held_out_improvement'
 return out

def registry(candidates,reviews):
 # Immutable review events are supplied by an operator; no controller YAML or
 # existing review is ever overwritten. Historical approvals remain visible.
 by_id={c['candidate_id']:c for c in candidates};best={t:None for t in TERRAINS};history=[]
 for e in reviews:
  c=by_id.get(e.get('candidate_id'))
  if c is None or e.get('decision') not in ('accepted','rejected'):raise ValueError('invalid review')
  if e['decision']=='accepted':
   if not e.get('reviewer') or e.get('validation_kind')!='real_vehicle_closed_loop' or e.get('shadow_only') is not False:raise ValueError('shadow/replay cannot establish best vehicle config')
   if e.get('safety_violations')!=0 or len(set(e.get('independent_test_runs',[])))<3:raise ValueError('independent repeated safe closed-loop evidence required')
   score=e.get('score')
   if isinstance(score,bool) or not isinstance(score,(int,float)) or not math.isfinite(score) or score<0.:raise ValueError('invalid reviewed objective')
   prior=best[c['terrain']]
   if prior is None or score<prior['score']:best[c['terrain']]=e
  history.append(e)
 return {'schema_version':1,'best_verified':best,'review_history':history,'production_changed':False,
   'limitation':'Review claims require human audit of referenced data; this registry does not certify authenticity.'}
