#!/usr/bin/env python3
"""Explicit RAM-backed research trial actions; never starts a driver or arms by itself.

All prepare/start/record/reconstruct/enable are distinct operator invocations.
Archive refuses active processes and an open raw manifest. It copies original
files without rewriting their absolute paths and leaves the RAM source intact.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent
WORKSPACE = ROOT.parents[3]
SCRIPTS = WORKSPACE / 'src/move_car/scripts'
RAM_ROOT = Path('/dev/shm/control_research_trials')
CONFIG = WORKSPACE / 'src/move_car/config/control/tuning/candidates/p7_smooth_causal_feedback_20261118'
MIN_FREE = 2 * 1024 ** 3


def require(ok, reason):
    if not ok:
        raise ValueError(reason)


def write_new(path, data):
    with Path(path).open('x') as stream:
        json.dump(data, stream, sort_keys=True, indent=2)
        stream.write('\n')


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def memory_check():
    available = None
    for line in Path('/proc/meminfo').read_text().splitlines():
        if line.startswith('MemAvailable:'):
            available = int(line.split()[1]) * 1024
    mounted = any(parts[1] == '/dev/shm' and parts[2] == 'tmpfs'
                  for parts in (line.split() for line in Path('/proc/mounts').read_text().splitlines()))
    free = shutil.disk_usage('/dev/shm').free
    require(mounted, '/dev/shm must be a tmpfs; refusing disk-backed research output')
    require(available is not None and available >= MIN_FREE,
            'at least 2 GiB MemAvailable required; available=' + str(available))
    require(free >= MIN_FREE, 'at least 2 GiB tmpfs free required; free=' + str(free))
    return dict(mem_available_bytes=available, tmpfs_free_bytes=free, minimum_bytes=MIN_FREE,
                output_root=str(RAM_ROOT), storage='tmpfs', power_loss_durability=False)


def trial_path(value=None):
    if value:
        return Path(value).resolve()
    return Path((ROOT / 'next_trial_path.txt').read_text().strip()).resolve()


def trial_info(path):
    path = Path(path).resolve()
    data = json.loads((path / 'trial.json').read_text())
    require(data.get('terrain') == 'smooth' and data.get('run_id', '').startswith('p7real_'),
            'invalid smooth P7 trial')
    require(path.name == data['run_id'], 'trial folder and run_id mismatch')
    return data


def proc_rows():
    """Read-only local process snapshot; no ROS mutation or node termination."""
    found = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            args = [item.decode('utf-8', 'replace') for item in
                    (entry / 'cmdline').read_bytes().split(b'\0') if item]
            if args:
                found.append(dict(pid=int(entry.name), argv=args))
        except (OSError, ValueError):
            # /proc processes can disappear between listing and reading.
            continue
    return found


def active_trial_processes(path, trial, processes=None):
    path = str(Path(path).resolve())
    binary_paths = set(trial.get('binary_paths', {}).values())
    relevant_scripts = {'p7_smooth_trial.py', 'p7_operator_lease.py', 'record_control_run.py',
                        'collect_stationary_raw.py',
                        'run_smooth_experiment.py', 'monitor_control_timing.py'}
    active = []
    for row in proc_rows() if processes is None else processes:
        argv = row['argv']
        basenames = {Path(arg).name for arg in argv}
        # Conservative local writer check also catches orphaned ROS children
        # whose argv[0] was resolved through a different overlay/symlink.
        is_binary = bool(binary_paths.intersection(argv) or
                         {'control_pub', 'p5_nmpc_worker_node', 'trajectory_reconstructor'}.intersection(basenames))
        is_lease = 'p7_operator_lease.py' in basenames
        trial_arg = any(arg == path or arg.startswith(path + '/') for arg in argv)
        is_task = trial_arg and bool(relevant_scripts.intersection(basenames) or 'roslaunch' in basenames)
        if is_binary or is_lease or is_task:
            active.append(row)
    return active


def closed_check(path, processes=None, *, allow_incomplete=False):
    trial = trial_info(path)
    active = active_trial_processes(path, trial, processes)
    require(not active, 'stop trial controller/recorder and remove the live lease manually before archive/review: ' + repr(active))
    raw = Path(path) / 'raw' / trial['run_id'] / 'manifest.json'
    if allow_incomplete and not raw.is_file():
        return trial, dict(state='missing')
    require(raw.is_file(), 'raw recording manifest missing; nothing completed to archive/review')
    manifest = json.loads(raw.read_text())
    require(manifest.get('run_id') == trial['run_id'], 'raw manifest run_id mismatch')
    require(allow_incomplete or manifest.get('state') == 'closed',
            'raw recorder has not closed; finish recording first')
    return trial, manifest


def tree_identity(folder):
    """Hash original bytes and link targets, never dereference outside the run."""
    identity = {}
    for path in sorted(Path(folder).rglob('*')):
        rel = path.relative_to(folder).as_posix()
        if path.is_symlink():
            identity[rel] = dict(kind='symlink', target=os.readlink(str(path)))
        elif path.is_file():
            identity[rel] = dict(kind='file', bytes=path.stat().st_size, sha256=sha(path))
    return identity


def archive(path, destination_root=None, processes=None, *, allow_incomplete=False):
    path = Path(path).resolve()
    trial, manifest = closed_check(path, processes, allow_incomplete=allow_incomplete)
    destination_root = Path(destination_root or WORKSPACE / 'experiment_runs').resolve()
    session = trial.get('session_id', '')
    require(session and Path(session).name == session and session not in ('.', '..'), 'invalid session_id')
    dest = destination_root / session / trial['run_id']
    require(not dest.exists(), 'archive destination already exists; refusing overwrite: ' + str(dest))
    source_id = tree_identity(path)
    total = sum(item.get('bytes', 0) for item in source_id.values())
    destination_root.mkdir(parents=True, exist_ok=True)
    require(shutil.disk_usage(str(destination_root)).free >= total + 128 * 1024 ** 2,
            'insufficient disk space for byte-verified archive plus 128 MiB reserve')
    dest.parent.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix='.' + trial['run_id'] + '.archive-', dir=str(dest.parent)))
    try:
        payload = stage / 'payload'
        shutil.copytree(str(path), str(payload), symlinks=True)
        require(tree_identity(path) == source_id, 'RAM source changed during archive; stop all writers first')
        require(tree_identity(payload) == source_id, 'archive SHA mismatch; source remains intact')
        # Original evidence paths remain verbatim; archive manifest is additional metadata.
        with (payload / 'RAM_ARCHIVE_MANIFEST.json').open('x') as stream:
            json.dump(dict(schema_version=1, source_directory=str(path),
                destination_directory=str(dest), files=source_id, copied_files=len(source_id),
                copied_bytes=total, original_absolute_paths_preserved=True,
                source_deleted=False, raw_state=manifest.get('state', 'unknown'),
                raw_manifest_closed=manifest.get('state') == 'closed',
                raw_writer=manifest.get('writer', {}), copied_wall_ns=time.time_ns()),
                stream, sort_keys=True, indent=2)
            stream.write('\n')
        # Same-filesystem rename exposes only a fully checked new run directory.
        require(not dest.exists(), 'archive destination appeared during copy; refusing overwrite')
        os.rename(str(payload), str(dest))
    finally:
        shutil.rmtree(str(stage), ignore_errors=True)
    print('ARCHIVED_DIRECTORY=' + str(dest))
    print('RAM source retained: ' + str(path))
    return dest


def verify_archived_source(folder, archived):
    proof = json.loads((archived / 'RAM_ARCHIVE_MANIFEST.json').read_text())
    require(proof['source_directory'] == str(folder), 'archive source identity mismatch')
    actual = tree_identity(archived)
    actual.pop('RAM_ARCHIVE_MANIFEST.json', None)
    require(actual == proof['files'] and tree_identity(folder) == proof['files'],
            'archive/source changed; do not overwrite prior evidence')


def check_binary_identity():
    identity = json.loads((ROOT / 'runtime_identity.json').read_text())
    for name, expected in identity['binaries'].items():
        require(sha(ROOT / 'ros_overlay/lib/move_car' / name) == expected,
                'pinned research binary changed: ' + name)


def reconstructor_binary_identity():
    identity = json.loads((ROOT / 'runtime_identity.json').read_text())
    expected = identity.get('binaries', {}).get('trajectory_reconstructor')
    binary = ROOT / 'ros_overlay/lib/move_car/trajectory_reconstructor'
    require(expected and binary.is_file() and sha(binary) == expected,
            'pinned reconstructor binary missing or changed')
    return expected


def write_reconstructor_launch(folder):
    """Freeze a wrapper that changes only the two research log destinations."""
    folder = Path(folder).resolve()
    formal = WORKSPACE / 'src/move_car/launch/trajectory_reconstructor.launch'
    formal_sha = sha(formal)
    generated = folder / 'reconstructor_ram.launch'
    final_log = folder / 'reconstructor_csv/front_car_final_trajectory.txt'
    launch = ET.Element('launch')
    ET.SubElement(launch, 'include', {'file': str(formal)})
    ET.SubElement(launch, 'param', {'name': '/trajectory_reconstructor/debug_log_dir',
                                  'value': str(folder / 'reconstructor_csv')})
    ET.SubElement(launch, 'param', {'name': '/trajectory_reconstructor/final_traj_log_file',
                                  'value': str(final_log)})
    require(not generated.exists(), 'generated reconstructor launch already exists')
    ET.ElementTree(launch).write(str(generated), encoding='utf-8', xml_declaration=True)
    require(sha(formal) == formal_sha, 'formal reconstructor launch changed during preparation')
    return dict(generated_launch=str(generated), generated_launch_sha256=sha(generated),
                formal_launch=str(formal), formal_launch_sha256=formal_sha,
                final_traj_log_file=str(final_log))


def check_reconstructor_launch_identity(folder, storage):
    """Do not run a modified wrapper or a changed included production launch."""
    folder = Path(folder).resolve()
    contract = storage.get('reconstructor_launch', {})
    generated = folder / 'reconstructor_ram.launch'
    formal = WORKSPACE / 'src/move_car/launch/trajectory_reconstructor.launch'
    require(contract.get('generated_launch') == str(generated) and
            contract.get('formal_launch') == str(formal) and
            contract.get('final_traj_log_file') == str(folder / 'reconstructor_csv/front_car_final_trajectory.txt'),
            'missing or mismatched frozen reconstructor launch contract; prepare a new run')
    if 'runtime_yaml_sha256' in contract:
        require(sha(folder / 'reconstructor.yaml') == contract['runtime_yaml_sha256'],
                'reconstructor runtime parameters changed; prepare a new run')
    require(generated.is_file() and sha(generated) == contract.get('generated_launch_sha256'),
            'generated reconstructor launch changed; prepare a new run')
    require(formal.is_file() and sha(formal) == contract.get('formal_launch_sha256'),
            'formal reconstructor launch changed since prepare; prepare a new run')


def prepare(args, *, select=True):
    storage = memory_check()
    require(CONFIG.is_dir(), 'new causal-feedback candidate Config missing: ' + str(CONFIG))
    check_binary_identity()
    oldfile = ROOT / 'next_trial_path.txt'
    if select and oldfile.exists():
        previous = Path(oldfile.read_text().strip())
        if previous.exists():
            previous_trial = trial_info(previous)
            require(not active_trial_processes(previous, previous_trial),
                    'previous trial is active; stop it manually before preparing another run')
            if (previous / 'started.json').exists():
                archived = WORKSPACE / 'experiment_runs' / previous_trial['session_id'] / previous_trial['run_id']
                require((archived / 'RAM_ARCHIVE_MANIFEST.json').is_file(),
                        'archive the previous started RAM run before preparing another run')
    cmd = [sys.executable, str(SCRIPTS / 'p7_smooth_trial.py'), 'prepare',
        '--session-id', args.session_id, '--route', args.route, '--operator', args.operator,
        '--mapping-profile', str(Path(__file__).resolve().parent / 'sensor_mapping/mapping_profile.json'),
        '--algorithm', 'flat_nmpc_slqr', '--candidate-control-trial', '--speed-cap', '.40',
        '--production-workspace', str(WORKSPACE), '--config-root', str(CONFIG), '--output-root', str(RAM_ROOT)]
    output = subprocess.check_output(cmd, universal_newlines=True)
    matches = [line.split('=', 1)[1] for line in output.splitlines() if line.startswith('TRIAL_DIRECTORY=')]
    require(len(matches) == 1, 'prepare did not return one trial directory')
    folder = Path(matches[0]).resolve()
    require(RAM_ROOT.resolve() in folder.parents, 'prepared path escaped RAM output root')
    trial = trial_info(folder)
    for name in ('reconstructor_csv', 'ros_logs'):
        (folder / name).mkdir()
    launch_identity = write_reconstructor_launch(folder)
    storage.update(run_id=trial['run_id'], session_id=trial['session_id'],
        prepared_wall_ns=time.time_ns(), ros_log_dir=str(folder / 'ros_logs'),
        controller_csv_dir=str(folder / 's8_csv_live'),
        reconstructor_csv_dir=str(folder / 'reconstructor_csv'),
        reconstructor_launch=launch_identity,
        reconstructor_binary_sha256=reconstructor_binary_identity(),
        formal_reconstructor_launch_unchanged=True,
        reconstruction_geometry_parameters_unchanged=not launch_identity.get('research_geometry_overrides'),
        explicit_reconstruct_action_required=True,
        archive_required_before_power_off=True)
    write_new(folder / 'RAM_STORAGE.json', storage)
    if select:
        tmp = ROOT / ('next_trial_path.' + str(os.getpid()) + '.tmp')
        tmp.write_text(str(folder) + '\n')
        selector = WORKSPACE / 'src/move_car/.smooth_trial'
        if selector.exists() and not selector.is_symlink():
            raise ValueError('smooth trial selector must be a symbolic link')
        selector_tmp = selector.with_name(selector.name + '.' + str(os.getpid()))
        selector_tmp.symlink_to(folder, target_is_directory=True)
        os.replace(str(selector_tmp), str(selector))
        os.replace(str(tmp), str(oldfile))
    print('TRIAL_DIRECTORY=' + str(folder), flush=True)
    print('ROS_LOG_DIR=' + str(folder / 'ros_logs'))
    print('RAM DATA ARE VOLATILE. After stopping controller and recording, run archive. No node or lease started.')
    return folder


def read_frozen_trial(path):
    sys.path.insert(0, str(SCRIPTS))
    from p7_smooth_trial import read_trial
    folder, trial = read_trial(path)
    require(trial.get('candidate_control_trial') and not trial.get('shadow_only')
            and trial.get('algorithm') == 'flat_nmpc_slqr', 'wrong trial algorithm/authority')
    require((folder / 'RAM_STORAGE.json').is_file(), 'missing RAM storage contract')
    storage = json.loads((folder / 'RAM_STORAGE.json').read_text())
    require(storage.get('run_id') == trial['run_id'] and
            storage.get('session_id') == trial['session_id'], 'RAM storage identity mismatch')
    check_reconstructor_launch_identity(folder, storage)
    require(storage.get('reconstructor_binary_sha256') == reconstructor_binary_identity(),
            'reconstructor binary differs from RAM preparation; prepare a new run')
    return folder, trial


def check_reconstructor_idle():
    sys.path.insert(0, str(SCRIPTS))
    from check_control_collection import read_master
    graph = read_master(os.environ.get('ROS_MASTER_URI', 'http://localhost:11311'), '/trajectory_reconstructor')
    nodes = {node for group in graph['publishers'] + graph['subscribers'] for node in group[1]}
    require('/trajectory_reconstructor' not in nodes,
            '请先结束仍在运行的 trajectory_reconstructor，再执行 reconstruct。')


def check_trial_reconstructor(folder, run_id):
    require(not (folder / 'started.json').exists(),
            '这轮控制已启动过。结束本轮进程后，从同一入口重新执行 reconstruct，再执行 start。')
    require((folder / 'reconstructor_started.json').is_file(),
            '当前轮次尚未启动重构。请先执行同一入口的 reconstruct，再执行 start。')
    sys.path.insert(0, str(SCRIPTS))
    from check_control_collection import read_master
    graph = read_master(os.environ.get('ROS_MASTER_URI', 'http://localhost:11311'), '/trajectory_reconstructor')
    publishers = dict(graph['publishers']).get('/planner/reference_path', [])
    reconstructor = graph['parameters']['/trajectory_reconstructor']
    require(isinstance(reconstructor, dict) and reconstructor.get('research_run_id') == run_id and
            '/trajectory_reconstructor' in publishers,
            '当前轮次的重构尚未就绪。请确认 reconstruct 正在运行并发布 /planner/reference_path。')


def main(*, default_session='smooth_causal_feedback_20261118', implicit_reconstruct_trial=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['prepare', 'reconstruct', 'start', 'record', 'enable', 'inspect', 'archive', 'review'])
    parser.add_argument('--trial')
    parser.add_argument('--session-id')
    parser.add_argument('--route')
    parser.add_argument('--operator')
    args = parser.parse_args()
    if args.action == 'reconstruct':
        check_reconstructor_idle()
        if args.trial is None and implicit_reconstruct_trial is not None:
            args.trial = str(implicit_reconstruct_trial(args))
    if args.action == 'prepare':
        args.session_id = args.session_id or default_session
        args.route = args.route or 's8_indoor_same_route'
        args.operator = args.operator or 'ubuntu'
        prepare(args)
        return
    folder = trial_path(args.trial)
    if args.action in ('archive', 'review'):
        closed_check(folder)
        if args.action == 'archive':
            archive(folder)
        else:
            # Preserve RAM evidence before an offline analysis can take minutes.
            archived = WORKSPACE / 'experiment_runs' / trial_info(folder)['session_id'] / folder.name
            if not archived.exists():
                archive(folder)
            else:
                verify_archived_source(folder, archived)
            output = ROOT / 'reviews' / folder.name
            require(not output.exists(), 'review output already exists; preserve prior result')
            raise SystemExit(subprocess.call([sys.executable, str(SCRIPTS / 'analyze_p7_trial.py'),
                                              '--trial', str(archived), '--output', str(output)]))
        return
    check_binary_identity()
    folder, trial = read_frozen_trial(folder)
    os.environ['ROS_LOG_DIR'] = str(folder / 'ros_logs')
    os.chdir(str(WORKSPACE))
    if args.action == 'inspect':
        print(json.dumps(trial, sort_keys=True, indent=2))
        return
    if args.action == 'reconstruct':
        require(not (folder / 'reconstructor_started.json').exists(),
                '这个试跑目录已启动过。去掉 --trial 并重跑 reconstruct，或为手动入口创建新轮次。')
        write_new(folder / 'reconstructor_started.json', dict(run_id=trial['run_id'],
            manual_action=True, started_wall_ns=time.time_ns(),
            launch=str(folder / 'reconstructor_ram.launch')))
        os.execvp('roslaunch', ['roslaunch', str(folder / 'reconstructor_ram.launch')])
    if args.action == 'start':
        if implicit_reconstruct_trial is not None:
            check_trial_reconstructor(folder, trial['run_id'])
        os.execv(sys.executable, [sys.executable, str(SCRIPTS / 'p7_smooth_trial.py'), 'start', '--trial', str(folder)])
    if args.action == 'record':
        os.execv(sys.executable, [sys.executable, str(SCRIPTS / 'run_smooth_experiment.py'),
            'record', '--experiment', str(folder), '--duration', '0'])
    if args.action == 'enable':
        os.execv('/usr/bin/python2', ['/usr/bin/python2', str(SCRIPTS / 'p7_operator_lease.py'), '--trial', str(folder)])


if __name__ == '__main__':
    try:
        main()
    except (ValueError, FileNotFoundError) as exc:
        raise SystemExit(str(exc))
