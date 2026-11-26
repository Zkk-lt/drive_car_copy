#!/usr/bin/env python3
"""Offline P4-0 CLI; never publishes commands or overwrites a configuration."""
import argparse,csv,json
from pathlib import Path
from control_research.dataset import index_runs,generate_candidates,calibration_fit,write_new,load_run,digest,validate_fit,registry

def main():
 p=argparse.ArgumentParser(description=__doc__);s=p.add_subparsers(dest='command',required=True)
 i=s.add_parser('index');i.add_argument('--runs',required=True);i.add_argument('--split-plan',required=True)
 c=s.add_parser('candidates');c.add_argument('--index',required=True);c.add_argument('--config',required=True);c.add_argument('--terrain',required=True)
 f=s.add_parser('calibrate');f.add_argument('--samples',required=True);f.add_argument('--contract',required=True)
 e=s.add_parser('inspect');e.add_argument('--run',required=True)
 v=s.add_parser('validate-fit');v.add_argument('--fit',required=True);v.add_argument('--samples',required=True);v.add_argument('--contract',required=True)
 g=s.add_parser('registry');g.add_argument('--candidates',required=True);g.add_argument('--reviews',required=True)
 for sub in (i,c,f,e,v,g):sub.add_argument('--output',required=True)
 a=p.parse_args()
 if a.command=='index':out=index_runs(a.runs,json.loads(Path(a.split_plan).read_text()))
 elif a.command=='candidates':
  out=generate_candidates(json.loads(Path(a.index).read_text()),a.config,a.terrain)
  destination=Path(a.output);destination.mkdir(parents=True,exist_ok=False)
  for item in out:write_new(destination/(item['candidate_id']+'.json'),item)
  print('Created',len(out),'unapproved candidates');return
 elif a.command=='registry':
  candidates=[json.loads(f.read_text()) for f in Path(a.candidates).glob('*.json')]
  reviews=[json.loads(f.read_text()) for f in Path(a.reviews).glob('*.json')]
  out=registry(candidates,reviews)
 elif a.command in ('calibrate','validate-fit'):
  with open(a.samples) as stream:rows=list(csv.DictReader(stream))
  contract=json.loads(Path(a.contract).read_text())
  out=calibration_fit(rows,contract) if a.command=='calibrate' else validate_fit(json.loads(Path(a.fit).read_text()),rows,contract)
  if a.command=='calibrate':out['data_origin']=contract.get('data_origin','unverified')
  else:
   out['training_fit_sha256']=digest(a.fit)
   if out.get('data_origin')!=contract.get('data_origin','unverified'):raise ValueError('training/validation provenance mismatch')
  out['samples_sha256']=digest(a.samples);out['contract_sha256']=digest(a.contract)
 else:
  m,events,metrics=load_run(a.run);out={'metrics':metrics,'source_contracts':m.get('source_contracts'),
   'unverified':['NED independence','CAN acquisition time','body heading','pure delay'],'raw_topics':sorted(set(e['topic'] for e in events))}
 write_new(a.output,out)
if __name__=='__main__':main()
