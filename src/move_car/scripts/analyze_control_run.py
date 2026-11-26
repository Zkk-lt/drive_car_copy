#!/usr/bin/env python3
"""Offline analysis only; never searches parameters or writes approved YAML."""
import argparse
import csv
import json
from pathlib import Path
from control_research.analysis import summarize, stats

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run_dir',nargs='?')
    p.add_argument('--legacy-reconstructor',type=Path)
    p.add_argument('--legacy-controller',type=Path)
    p.add_argument('--output',type=Path)
    a=p.parse_args()
    if a.run_dir:
        root=Path(a.run_dir);manifest=json.loads((root/'manifest.json').read_text())
        with (root/'events.jsonl').open() as f:
            result=summarize((json.loads(line) for line in f if line.strip()),manifest)
    elif a.legacy_reconstructor or a.legacy_controller:
        import hashlib
        result={'schema_version':1,'provenance':'legacy logs; run_id, terrain and hardware timing unverified','files':{}}
        mapping={'e_y':'lateral_error_abs_m','e_theta':'heading_error_abs_rad',
                 'reconstruction_compute_ms':'reconstruction_compute_ms'}
        for path in (a.legacy_controller,a.legacy_reconstructor):
            if path is None:continue
            with path.open() as f:rows=list(csv.DictReader(f))
            metrics={}
            for field,name in mapping.items():
                values=[]
                for row in rows:
                    if field in row:
                        try:
                            value=float(row[field]);values.append(value if field.endswith('_ms') else abs(value))
                        except (ValueError,TypeError):pass
                if values:metrics[name]=stats(values)
            result['files'][str(path)]={'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
                                       'rows':len(rows),'metrics':metrics}
    else:p.error('provide run_dir or a legacy CSV')
    encoded=json.dumps(result,indent=2,sort_keys=True,allow_nan=False)+'\n'
    if a.output:
        # Do not overwrite a run artifact or a production YAML by accident.
        if a.output.suffix != '.json':p.error('output must be a new .json file')
        with a.output.open('x') as f:f.write(encoded)
    else:print(encoded,end='')
if __name__=='__main__':main()
