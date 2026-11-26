#!/usr/bin/env python3
"""Read-only raw recorder audit; does not start ROS or publish commands."""
from __future__ import print_function
import argparse
import json
import os
from control_research.causal_data_audit import audit_run, default_contract, workspace_inventory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action')
    run = sub.add_parser('run', help='audit existing manifest.json/events.jsonl directory')
    run.add_argument('--run-dir', required=True)
    run.add_argument('--contract')
    run.add_argument('--include-snapshots', action='store_true')
    run.add_argument('--output', required=True)
    inv = sub.add_parser('inventory', help='hash existing data; quarantine unknown provenance')
    inv.add_argument('--workspace', required=True)
    inv.add_argument('--output', required=True)
    template = sub.add_parser('template', help='write unverified raw-event audit contract')
    template.add_argument('--output', required=True)
    args = parser.parse_args()
    if not args.action:
        parser.error('choose run, inventory or template')
    if os.path.exists(args.output):
        parser.error('refusing to overwrite existing evidence: ' + args.output)
    if args.action == 'inventory':
        value = workspace_inventory(args.workspace)
    elif args.action == 'template':
        value = default_contract()
    else:
        contract = None
        if args.contract:
            with open(args.contract) as stream:
                contract = json.load(stream)
        value = audit_run(args.run_dir, contract, args.include_snapshots)
    parent = os.path.dirname(os.path.abspath(args.output))
    if not os.path.isdir(parent):
        os.makedirs(parent)
    with open(args.output, 'x') as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write('\n')
    print(json.dumps({'output': os.path.abspath(args.output), 'status': value.get('status'),
                      'identification_ready': value.get('identification_ready', False)}, sort_keys=True))


if __name__ == '__main__':
    main()
