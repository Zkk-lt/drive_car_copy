#!/usr/bin/env python3
"""Prepare passive experiment metadata, compare complete matched runs, index scene artifacts."""
import argparse,json
from pathlib import Path
from control_research.dataset import write_new
from control_research.terrain_experiment import prepare,compare_runs,scene_library

def main():
    p=argparse.ArgumentParser(description=__doc__);s=p.add_subparsers(dest='command',required=True)
    q=s.add_parser('prepare')
    for key in ('workspace','run-id','session','batch','terrain','route','direction','phase','conditions'):q.add_argument('--'+key,required=True)
    q.add_argument('--speed',type=float,required=True);q.add_argument('--repetition',type=int,required=True);q.add_argument('--formal-path',action='append',required=True)
    c=s.add_parser('compare');c.add_argument('--run',action='append',required=True)
    l=s.add_parser('library');l.add_argument('--references',required=True)
    for z in (q,c,l):z.add_argument('--output',required=True)
    a=p.parse_args()
    if a.command=='prepare':out=prepare(a.workspace,a.run_id,a.session,a.batch,a.terrain,a.route,a.speed,a.direction,a.repetition,a.phase,a.conditions,a.formal_path)
    elif a.command=='compare':out=compare_runs(a.run)
    else:
        d=json.loads(Path(a.references).read_text());out=scene_library(*[d.get(k,[]) for k in ('baselines','calibrations','experiments','candidates','validations')])
    write_new(a.output,out)
    print('Created immutable',a.command,'artifact; no vehicle command or production YAML written')
if __name__=='__main__':main()
