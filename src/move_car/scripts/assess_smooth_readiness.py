#!/usr/bin/env python3
"""Assess P4-2 offline readiness, never run a search or change a controller."""
import argparse
from pathlib import Path
from control_research.dataset import write_new
from control_research.smooth_readiness import assess


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset', type=Path, required=True)
    p.add_argument('--model', type=Path, action='append', default=[])
    p.add_argument('--review', type=Path)
    p.add_argument('--speed-range', type=float, nargs=2)
    p.add_argument('--local-evidence', type=Path, action='append', default=[],
                   help='Existing raw audit or receipt-response report; assess each local use separately without changing the legacy physical-model gate.')
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    result = assess(a.dataset, a.model, a.review, a.speed_range, a.local_evidence)
    write_new(a.output, result)
    print('legacy_calibrated_model_bundle: ' + result['status'])
    for name, value in sorted(result['use_case_assessments'].items()):
        print(name + ': ' + value['status'])


if __name__ == '__main__':
    main()
