#!/usr/bin/env python3
"""P4-2 passive smooth dataset gate; no ROS and no production YAML writes."""
import argparse
import json
from pathlib import Path
from control_research.dataset import write_new
from control_research.smooth_preparation import split_template, audit_dataset, quarantine_legacy


def main():
    p = argparse.ArgumentParser(description=__doc__)
    s = p.add_subparsers(dest='command', required=True)
    t = s.add_parser('template')
    a = s.add_parser('audit')
    a.add_argument('--runs', type=Path, required=True)
    a.add_argument('--plan', type=Path, required=True)
    q = s.add_parser('legacy')
    q.add_argument('--workspace', type=Path, required=True)
    for command in (t, a, q):
        command.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if args.command == 'template':
        result = split_template()
    elif args.command == 'legacy':
        result = quarantine_legacy(args.workspace.resolve())
    else:
        result = audit_dataset(args.runs, json.loads(args.plan.read_text()), args.plan.parent)
    write_new(args.output, result)
    print(result.get('status', 'written'))


if __name__ == '__main__':
    main()
