#!/usr/bin/env python3
"""Bind the traditional ROS launch entry points to the prepared smooth run."""
import json
import os
from pathlib import Path
import sys
import time

from check_control_collection import read_master
from p7_smooth_trial import read_trial, require, sha, write_new


def main():
    role, folder, executable, *arguments = sys.argv[1:]
    folder, trial = read_trial(folder)
    run = trial['run_id']
    storage = json.loads((folder / 'RAM_STORAGE.json').read_text())
    require(storage['run_id'] == run, 'prepared run identity mismatch')
    uri = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
    graph = read_master(uri, '/smooth_launch_preflight')
    publishers = dict(graph['publishers'])
    if role == 'control':
        require(sha(folder / 'run_identity.yaml') ==
                storage['reconstructor_launch']['run_identity_yaml_sha256'],
                'control run identity parameters changed')
        require(graph['run_id'] == run, 'controller launch run identity mismatch')
        require(not publishers.get('/cmd_vel'), 'stop the existing /cmd_vel publisher first')
        require((folder / 'reconstructor_started.json').is_file()
                and '/trajectory_reconstructor' in publishers.get('/planner/reference_path', [])
                and (graph['parameters'].get('/trajectory_reconstructor') or {}).get('research_run_id') == run,
                'start reconstruction for this run first')
        require(sha(Path(executable)) == trial['binaries']['control_pub'], 'controller binary identity mismatch')
        write_new(folder / 'started.json', dict(run_id=run, master_uri=uri, preflight_publishers=[],
                  automatic_arming=False, operator_initiated_start=True))
    elif role == 'reconstruct':
        require('/trajectory_reconstructor' not in publishers.get('/planner/reference_path', []),
                'stop the existing trajectory_reconstructor first')
        contract = storage['reconstructor_launch']
        require(sha(folder / 'reconstructor.yaml') == contract['runtime_yaml_sha256'],
                'reconstructor runtime parameters changed')
        require(sha(Path(contract['formal_launch'])) == contract['formal_launch_sha256'],
                'reconstructor launch changed')
        require(sha(Path(executable)) == storage['reconstructor_binary_sha256'], 'reconstructor binary identity mismatch')
        write_new(folder / 'reconstructor_started.json', dict(run_id=run, manual_action=True,
                  started_wall_ns=time.time_ns(), launch=contract['formal_launch']))
    else:
        raise ValueError('unknown smooth launch role')
    os.execv(executable, [executable] + arguments)


if __name__ == '__main__':
    try:
        main()
    except (ValueError, OSError, KeyError) as exc:
        raise SystemExit('smooth launch: ' + str(exc))
