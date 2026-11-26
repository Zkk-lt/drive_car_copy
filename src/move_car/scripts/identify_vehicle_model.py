#!/usr/bin/env python3
"""P4-2 offline equation tools. No ROS node, command output or YAML modification."""
import argparse
import csv
import json
from pathlib import Path
from control_research.dataset import digest, write_new
from control_research.model_identification import MODELS, template, fit_model, validate_model, evaluate_one_step


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    commands=parser.add_subparsers(dest='command',required=True)
    t=commands.add_parser('template');t.add_argument('--output',required=True)
    for name in ('fit','validate','predict'):
        p=commands.add_parser(name)
        p.add_argument('--samples',required=True);p.add_argument('--contract',required=True)
        p.add_argument('--output',required=True)
        if name=='fit':p.add_argument('--model',required=True,choices=MODELS)
        else:p.add_argument('--fit',required=True)
    args=parser.parse_args()
    if args.command=='template':out=template()
    else:
        try:
            contract=json.loads(Path(args.contract).read_text())
            if not isinstance(contract,dict):raise ValueError('object_contract_required')
            with open(args.samples) as stream:rows=list(csv.DictReader(stream))
            if args.command=='fit':out=fit_model(args.model,rows,contract)
            else:
                fit=json.loads(Path(args.fit).read_text())
                if not isinstance(fit,dict):raise ValueError('object_fit_required')
                out=(validate_model if args.command=='validate' else evaluate_one_step)(fit,rows,contract)
                out['training_fit_sha256']=digest(args.fit)
            out.update(samples_sha256=digest(args.samples),contract_sha256=digest(args.contract))
        except (OSError,ValueError,TypeError,KeyError) as error:
            out=dict(schema_version=1,status='invalid',valid=False,reason=str(error),
                missing_measurements=[str(error)],physical_parameter_certification=False,
                vehicle_closed_loop_improvement_claim=False,safety_or_production_config_changed=False)
    write_new(args.output,out)
    print(out.get('status','template_created'))

if __name__=='__main__':main()
