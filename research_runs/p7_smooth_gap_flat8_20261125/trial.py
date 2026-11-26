#!/usr/bin/env python3
"""Run the tested smooth handoff through the existing RAM trial tools."""
import importlib.util
import fcntl
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent
WORKSPACE = ROOT.parents[1]
PREVIOUS = WORKSPACE / 'src/move_car/scripts/smooth_trial_support/archive_trial.py'
SPEC = importlib.util.spec_from_file_location('p7_ram_trial', PREVIOUS)
trial = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(trial)
trial.ROOT = ROOT
trial.CONFIG = WORKSPACE / 'src/move_car/config/control/tuning/candidates/p7_smooth_supported_course_20261126'
DEFAULT_SESSION = 'smooth_supported_course_20261126'
SOURCE_BINARIES = WORKSPACE / 'devel/lib/move_car'
ACCEPTANCE = ROOT / 'straight_sway_review/SOFTWARE_ACCEPTANCE.json'


def accepted_source_runtime():
    manifest = json.loads((trial.CONFIG / 'CANDIDATE_TRIAL.json').read_text())
    proof = manifest['software_acceptance']
    trial.require(Path(proof['path']) == ACCEPTANCE and trial.sha(ACCEPTANCE) == proof['sha256'],
                  'smooth software acceptance identity changed')
    acceptance = json.loads(ACCEPTANCE.read_text())
    config = {str(p.relative_to(trial.CONFIG)): trial.sha(p) for p in trial.CONFIG.rglob('*.yaml')}
    trial.require(acceptance['supervised_software_qualified'] is True
                  and config == acceptance['config_sha256'],
                  'smooth trial Config differs from the software-tested configuration')
    return dict(acceptance['binary_sha'],
                trajectory_reconstructor=acceptance['reconstructor_sha256'])


def check_source_binary_identity():
    for name, digest in accepted_source_runtime().items():
        trial.require(trial.sha(SOURCE_BINARIES / name) == digest,
                      'source launch binary differs from the smooth trial: ' + name)


def source_reconstructor_identity():
    digest = accepted_source_runtime()['trajectory_reconstructor']
    trial.require(trial.sha(SOURCE_BINARIES / 'trajectory_reconstructor') == digest,
                  'source launch reconstructor differs from the smooth trial')
    return digest


trial.check_binary_identity = check_source_binary_identity
trial.reconstructor_binary_identity = source_reconstructor_identity


def freeze_reference(folder):
    import yaml

    info = trial.trial_info(folder)
    formal = WORKSPACE / 'src/move_car/launch/trajectory_reconstructor.launch'
    launch = folder / 'reconstructor_ram.launch'
    log = folder / 'reconstructor_csv/front_car_final_trajectory.txt'
    settings = dict(research_run_id=info['run_id'], debug_log_dir=str(log.parent),
                    final_traj_log_file=str(log))
    with (folder / 'reconstructor.yaml').open('x') as output:
        yaml.safe_dump(settings, output)
    with (folder / 'run_identity.yaml').open('x') as output:
        yaml.safe_dump({'run_id': info['run_id']}, output)
    document = ET.Element('launch')
    include = ET.SubElement(document, 'include', {'file': str(formal)})
    ET.SubElement(include, 'arg', {'name': 'trial_dir', 'value': str(folder)})
    ET.SubElement(include, 'arg', {'name': 'direct_preflight', 'value': 'false'})
    ET.ElementTree(document).write(str(launch), encoding='utf-8', xml_declaration=True)
    return dict(generated_launch=str(launch), generated_launch_sha256=trial.sha(launch),
                formal_launch=str(formal), formal_launch_sha256=trial.sha(formal),
                final_traj_log_file=str(log),
                research_geometry_overrides=yaml.safe_load((trial.CONFIG / 'reconstructor.yaml').read_text()),
                runtime_yaml_sha256=trial.sha(folder / 'reconstructor.yaml'),
                run_identity_yaml_sha256=trial.sha(folder / 'run_identity.yaml'))


trial.write_reconstructor_launch = freeze_reference


def select_reconstruction_trial(args):
    pointer = ROOT / 'next_trial_path.txt'
    folder = trial.trial_path() if pointer.is_file() else None
    if folder is not None and (folder / 'trial.json').is_file():
        info, _ = trial.closed_check(folder, allow_incomplete=True)
        used = any((folder / name).exists()
                   for name in ('reconstructor_started.json', 'started.json', 'raw'))
        if not used:
            return folder

        archive_dir = WORKSPACE / 'experiment_runs' / info['session_id'] / info['run_id']
        if archive_dir.exists():
            trial.verify_archived_source(folder, archive_dir)
            print('已核对上一轮归档 ' + str(archive_dir), flush=True)
        else:
            trial.archive(folder, allow_incomplete=True)
        experiment = json.loads((folder / 'experiment.json').read_text())
        args.session_id = args.session_id or info['session_id']
        args.route = args.route or experiment['route']
        args.operator = args.operator or info['operator_id']
    args.session_id = args.session_id or DEFAULT_SESSION
    args.route = args.route or 's8_indoor_same_route'
    args.operator = args.operator or 'ubuntu'
    return trial.prepare(args)


def run_cli():
    try:
        # Keep automatic selection and the start marker in the same critical
        # section. Python closes the lock descriptor on exec to roslaunch.
        with (ROOT / '.trial_selection.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            trial.main(default_session=DEFAULT_SESSION, implicit_reconstruct_trial=select_reconstruction_trial)
    except (ValueError, OSError) as exc:
        raise SystemExit(str(exc))


if __name__ == '__main__':
    run_cli()
