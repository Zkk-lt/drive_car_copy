#!/usr/bin/env python3
"""Read-only calibration CLI. No ROS, command publisher, or production YAML write."""
import argparse,csv,json
from pathlib import Path
from control_research.dataset import load_run,write_new,digest
from control_research.vehicle_calibration import template,audit_events,analyze_si,fit_actuator,validate_actuator,audit_legacy

def main():
    p=argparse.ArgumentParser(description=__doc__);s=p.add_subparsers(dest='command',required=True)
    t=s.add_parser('template')
    a=s.add_parser('audit-run');a.add_argument('--run',required=True)
    l=s.add_parser('audit-legacy');l.add_argument('--samples',required=True)
    commands=[]
    for name in ('analyze-si','fit-actuator','validate-actuator'):
        q=s.add_parser(name);q.add_argument('--samples',required=True);q.add_argument('--contract',required=True);commands.append(q)
        if name=='validate-actuator':q.add_argument('--fit',required=True)
    for q in [t,a,l]+commands:q.add_argument('--output',required=True)
    x=p.parse_args()
    if x.command=='template':out=template()
    elif x.command=='audit-legacy':
        with open(x.samples) as stream:out=audit_legacy(list(csv.DictReader(stream)))
        out['samples_sha256']=digest(x.samples)
    elif x.command=='audit-run':
        m,e,_=load_run(x.run);out=audit_events(e,m)
        out['manifest_sha256']=digest(Path(x.run)/'manifest.json');out['events_sha256']=digest(Path(x.run)/'events.jsonl')
    else:
        with open(x.samples) as stream:rows=list(csv.DictReader(stream))
        c=json.loads(Path(x.contract).read_text())
        if x.command=='analyze-si':out=analyze_si(rows,c)
        elif x.command=='fit-actuator':out=fit_actuator(rows,c)
        else:out=validate_actuator(json.loads(Path(x.fit).read_text()),rows,c);out['training_fit_sha256']=digest(x.fit)
        out['samples_sha256']=digest(x.samples);out['contract_sha256']=digest(x.contract)
    write_new(x.output,out)
    print(out.get('status',out.get('physical_calibration_status','created')))
if __name__=='__main__':main()
