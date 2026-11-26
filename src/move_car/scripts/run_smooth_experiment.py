#!/usr/bin/env python3
"""Passive real smooth capture and post-run review. Never starts a controller.

The only child ROS nodes are subscribers/recorders. No setParam, command
publisher, scheduling change or configuration deployment exists in this tool.
"""
import argparse
from datetime import datetime, timedelta, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import uuid
import yaml

from control_research.config import load_config, validate_run_id
from check_control_collection import read_master
from check_control_collection import BoundedTransport
from xmlrpc.client import ServerProxy

PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[1]
PRODUCTION = Path('/home/ubuntu/drive_car_copy')


def binary_directory():
    """Separate devel space when integrated in S8, preserving its old binary."""
    default = WORKSPACE/('devel_p7' if (WORKSPACE/'devel_p7').is_dir() else 'devel')
    return Path(os.environ.get('CONTROL_RESEARCH_DEVEL', str(default)))/'lib/move_car'


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1048576), b''):
            digest.update(block)
    return digest.hexdigest()


def write_new(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write('\n')


def git_revision(root):
    try:
        return subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'],
                                       stderr=subprocess.DEVNULL, text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def check_output_root(path, production=PRODUCTION):
    path = Path(path).resolve()
    protected = [Path(production).resolve()/'src', WORKSPACE/'src', WORKSPACE/'devel', WORKSPACE/'build',
                 WORKSPACE/'devel_p7', WORKSPACE/'build_p7']
    if WORKSPACE.resolve() != Path(production).resolve():
        protected.append(Path(production).resolve())
    if path == Path(production).resolve():
        raise ValueError('experiment output cannot be the workspace root')
    if WORKSPACE.resolve() == Path(production).resolve() and WORKSPACE.resolve() in path.parents:
        allowed=WORKSPACE/'experiment_runs'
        if path!=allowed and allowed not in path.parents:
            raise ValueError('inside the integrated workspace, use experiment_runs only')
    if any(path == p or p in path.parents for p in protected):
        raise ValueError('experiment output must be outside production/source/build/devel')
    return path


def prepare(output_root, session, route, speed, production=PRODUCTION, run_id=None):
    session = validate_run_id(session)
    if not route.strip() or not math.isfinite(speed) or speed < 0.:
        raise ValueError('nonempty route and finite nonnegative declared speed required')
    output_root = check_output_root(output_root, production)
    local = datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%dT%H%M%S')
    run = validate_run_id(run_id) if run_id else 'smooth_'+local+'_'+uuid.uuid4().hex[:8]
    folder = output_root/session/run
    folder.mkdir(parents=True, exist_ok=False)
    production = Path(production).resolve()
    files = [production/'src/move_car/src/control_pub.cpp',
             production/'src/move_car/src/trajectory_reconstructor.cpp',
             production/'src/move_car/launch/movecar_improved.launch',
             production/'src/move_car/launch/trajectory_reconstructor.launch',
             production/'devel/lib/move_car/control_pub',
             production/'devel/lib/move_car/trajectory_reconstructor']
    nominal = PACKAGE/'config/control/p6'
    files += sorted(nominal.rglob('*.yaml'))
    files += [Path(__file__), PACKAGE/'scripts/collect_stationary_raw.py',
              PACKAGE/'scripts/control_research/recording.py']
    files += [binary_directory()/name for name in ('control_pub','p5_nmpc_worker_node')]
    files += list((PACKAGE/'include/control').glob('*.h'))
    files += [PACKAGE/'src/unified_nmpc_worker.cpp',PACKAGE/'scripts/p7_smooth_trial.py',
              PACKAGE/'scripts/p7_operator_lease.py',PACKAGE/'scripts/control_research/smooth_run_analysis.py']
    freeze = dict(schema_version=1, run_id=run, session_id=session,
        production_git=git_revision(production), research_git=git_revision(WORKSPACE),
        files={str(p): sha(p) for p in files if p.is_file()},
        missing_files=[str(p) for p in files if not p.is_file()],
        nominal_config_hash=load_config(str(nominal), 'smooth')['config_hash'],
        running_binary_identity_verified=False, production_changed=False)
    write_new(folder/'software_freeze.json', freeze)
    experiment = dict(schema_version=1, run_id=run, session_id=session, terrain='smooth',
        route=route, declared_target_speed_mps=speed,
        target_speed_is_metadata_only=True, actual_controller='unverified_see_raw_published_owner_and_binary_provenance',
        nmpc_vehicle_authority=False, p6_live_shadow_not_certified=True,
        raw_directory=str(folder/'raw'/run), production_workspace=str(production),
        software_freeze_sha256=sha(folder/'software_freeze.json'),
        session_semantics='operator declares continuous acquisition session; not independent merely because ID differs',
        dataset_split='unassigned', production_or_formal_config_written=False)
    write_new(folder/'experiment.json', experiment)
    return folder, experiment


def stop_owned(process):
    if process is None or process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=25.)
    except subprocess.TimeoutExpired:
        process.terminate()
        process.wait(timeout=5.)


def live_provenance(master_uri, production, expected_executables=None, frozen_digests=None):
    master=ServerProxy(master_uri, transport=BoundedTransport()); records=[]
    expected_executables=expected_executables or {}
    frozen_digests=frozen_digests or {}
    nodes={'/publish_speed_turn':'control_pub','/trajectory_reconstructor':'trajectory_reconstructor'}
    if '/p7_nmpc_worker' in expected_executables:
        nodes['/p7_nmpc_worker']='p5_nmpc_worker_node'
    for name,binary in nodes.items():
        row=dict(node=name, verified_running_binary=False)
        try:
            code, message, uri=master.lookupNode('/smooth_passive_runner',name)
            if code!=1:raise ValueError(message)
            code, message, pid=ServerProxy(uri,transport=BoundedTransport()).getPid('/smooth_passive_runner')
            if code!=1 or not isinstance(pid,int) or pid<=0:raise ValueError('invalid node PID')
            executable=Path('/proc')/str(pid)/'exe'
            actual=sha(executable);expected=Path(expected_executables.get(name,
                Path(production)/'devel/lib/move_car'/binary))
            installed=sha(expected) if expected.is_file() else None
            pinned=frozen_digests.get(name,installed)
            row.update(pid=pid,executable=os.readlink(str(executable)),sha256=actual,
                expected_binary=str(expected),expected_sha256=pinned,
                verified_running_binary=actual==pinned==installed)
        except Exception as exc:row['unknown_reason']=str(exc)
        records.append(row)
    return records


def csv_snapshot_settings(folder, startup_settings):
    """Use a prepared run's immutable log destination, never stale ROS params.

    Recording may start before its controller. ROS keeps the previous node's
    private params after shutdown, so the initial graph is not a log identity.
    This only selects files for passive copying; no runtime parameter is set.
    """
    trial_file=folder/'trial.json'
    if not trial_file.is_file():
        return startup_settings, dict(source='startup_parameters_without_prepared_trial',
                                      current_run_log_identity_verified=False)
    trial=json.loads(trial_file.read_text());config=folder/'controller.yaml'
    expected=trial.get('trial_files',{}).get('controller.yaml')
    if not expected or not config.is_file() or sha(config)!=expected:
        raise ValueError('prepared controller log configuration changed; refuse stale CSV fallback')
    selected=yaml.safe_load(config.read_text())
    if not isinstance(selected,dict) or selected.get('p7_run_id')!=trial.get('run_id'):
        raise ValueError('prepared controller log run identity mismatch')
    destination=Path(selected.get('debug_log_dir','')).resolve()
    if folder.resolve() not in destination.parents:
        raise ValueError('prepared CSV log destination must belong to this run')
    return selected, dict(source='immutable_prepared_controller_yaml',
        current_run_log_identity_verified=True, run_id=trial['run_id'],
        startup_log_dir=startup_settings.get('debug_log_dir'),
        selected_log_dir=str(destination), configuration_sha256=expected,
        physical_sampling_or_execution_verified=False)


def snapshot_csv(folder, settings, production):
    """Copy existing S8 logs before its next launch truncates them; never edit."""
    source_dir = Path(settings.get('debug_log_dir', str(production/'cdata')))
    output = folder/'s8_csv'; output.mkdir()
    result = []
    for key, default in [('runtime_debug_file', 'runtime_debug.csv'),
                         ('profile_debug_file', 'profile_debug.csv'),
                         ('path_debug_file', 'path_snapshot.csv')]:
        source = source_dir/settings.get(key, default)
        entry = dict(source=str(source), present=source.is_file())
        if source.is_file():
            before = source.stat(); target = output/default
            shutil.copyfile(str(source), str(target)); after = source.stat()
            entry.update(path=str(target), sha256=sha(target), bytes=target.stat().st_size,
                stable_during_copy=(before.st_size, before.st_mtime_ns)==(after.st_size, after.st_mtime_ns),
                scope='unmodified CSV snapshot; only capture ROS-time overlap may be analysed; not an independent raw sensor')
        result.append(entry)
    write_new(output/'copy_manifest.json', result)
    return [x['path'] for x in result if x.get('stable_during_copy') and x.get('path')]


def analyze(folder):
    from control_research.smooth_run_analysis import analyze_run
    folder = Path(folder).resolve()
    experiment = json.loads((folder/'experiment.json').read_text())
    manifest = folder/'s8_csv/copy_manifest.json'
    snapshots = json.loads(manifest.read_text()) if manifest.exists() else []
    csv_paths = [r['path'] for r in snapshots if r.get('stable_during_copy') and r.get('path')]
    return analyze_run(experiment['raw_directory'], folder/'analysis', csv_snapshots=csv_paths)


def check_recording_publishers(folder, run_id, graph):
    settings = graph['parameters'].get('/publish_speed_turn') or {}
    command_topic = settings.get('cmd_topic', '/cmd_vel')
    publishers = dict(graph['publishers']).get(command_topic, [])
    if publishers and publishers != ['/publish_speed_turn']:
        raise ValueError('expected a single command publisher /publish_speed_turn; observed '+repr(publishers))
    if (folder/'trial.json').is_file() and (
            not (folder/'started.json').is_file() or graph.get('run_id') != run_id or
            publishers != ['/publish_speed_turn']):
        raise ValueError('请先从同一入口执行 reconstruct 和 start。等当前轮次的控制器开始发布指令，再执行 record。')
    return settings, command_topic


def record(args):
    if args.experiment:
        folder=Path(args.experiment).resolve()
        experiment=json.loads((folder/'experiment.json').read_text())
        args.session_id=experiment['session_id'];args.route=experiment['route']
        args.target_speed_mps=experiment['declared_target_speed_mps']
        args.production_workspace=Path(experiment['production_workspace'])
        if (folder/'raw').exists() or (folder/'capture_exit.json').exists():
            raise ValueError('prepared experiment already recorded; prepare a new run')
    else:
        folder, experiment = prepare(args.output_root, args.session_id, args.route,
                                     args.target_speed_mps, args.production_workspace)
    # P7 records one controller run. Standalone passive capture still allows
    # a collector-local identity without a global research run ID.
    graph = read_master(args.master_uri, '/publish_speed_turn')
    settings,command_topic=check_recording_publishers(folder,experiment['run_id'],graph)
    print('RUN_DIRECTORY='+str(folder), flush=True)
    write_new(folder/'preflight_graph.json', graph)
    trial_file=folder/'trial.json';expected_executables={};frozen_digests={}
    if trial_file.is_file():
        prepared_trial=json.loads(trial_file.read_text())
        prepared_binaries=prepared_trial['binary_paths']
        expected_executables['/publish_speed_turn']=prepared_binaries['control_pub']
        expected_executables['/p7_nmpc_worker']=prepared_binaries['p5_nmpc_worker_node']
        frozen_digests['/publish_speed_turn']=prepared_trial['binaries']['control_pub']
        frozen_digests['/p7_nmpc_worker']=prepared_trial['binaries']['p5_nmpc_worker_node']
        frozen_storage=folder/'RAM_STORAGE.json'
        if frozen_storage.is_file():
            storage=json.loads(frozen_storage.read_text())
            expected_executables['/trajectory_reconstructor']=(
                Path(prepared_binaries['control_pub']).parent/'trajectory_reconstructor')
            frozen_digests['/trajectory_reconstructor']=storage['reconstructor_binary_sha256']
    provenance=live_provenance(args.master_uri,args.production_workspace,expected_executables,frozen_digests)
    write_new(folder/'live_node_provenance.json',provenance)
    collector = [args.ros_python, str(PACKAGE/'scripts/collect_stationary_raw.py'),
        '--output-dir', str(folder/'raw'), '--run-id', experiment['run_id'],
        '--session-id', args.session_id, '--freeze', str(folder/'software_freeze.json'),
        '--phase', 's8_tracking', '--exclude-ned', '--terrain-label', 'smooth',
        '--route', args.route, '--target-speed-mps', str(args.target_speed_mps),
        '--direction', args.direction, '--repetition', str(args.repetition),
        '--data-origin', args.data_origin,
        '--p6-events', '--duration', str(args.duration)]
    if args.duration == 0.:
        collector.append('--until-control-stops')
    monitor = [args.ros_python, str(PACKAGE/'scripts/monitor_control_timing.py'),
        'collect', '--output', str(folder/'timing'), '--run-id', experiment['run_id'],
        '--session-id', args.session_id, '--terrain-label', 'smooth',
        '--data-origin', 'real_vehicle', '--duration', '0', '--command-topic', command_topic,
        '--git-revision', git_revision(WORKSPACE) or 'unknown',
        '--config-sha256', json.loads((folder/'software_freeze.json').read_text())['nominal_config_hash']]
    for node in provenance:
        if node.get('pid'):monitor.extend(['--pid',str(node['pid'])])
    env = os.environ.copy(); env['ROS_MASTER_URI'] = args.master_uri
    env['PYTHONPATH'] = str(PACKAGE/'scripts')+os.pathsep+env.get('PYTHONPATH', '')
    owned=[]; manual=False
    try:
        # New process groups ensure Ctrl-C is handled here, then sent only to
        # these two passive children. S8/driver processes are never stopped.
        for command, name in [(collector, 'capture.log'), (monitor, 'timing.log')]:
            stream=(folder/name).open('w')
            try:proc=subprocess.Popen(command, env=env, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
            finally:stream.close()
            owned.append(proc)
        while owned[0].poll() is None:
            if owned[1].poll() is not None:
                print('Timing monitor exited: inspect timing.log; raw capture continues.', flush=True)
                owned[0].wait(); break
            time.sleep(.25)
    except KeyboardInterrupt:
        manual=True; print('Closing passive recorders; no vehicle command sent.', flush=True)
    finally:
        for process in reversed(owned):stop_owned(process)
        write_new(folder/'capture_exit.json', dict(manual_interrupt=manual,
            returncodes=[p.returncode for p in owned], controllers_started_or_stopped=False))
    snapshot_settings,binding=csv_snapshot_settings(folder,settings)
    write_new(folder/'csv_source_binding.json',binding)
    snapshot_csv(folder, snapshot_settings, Path(args.production_workspace))
    raw=Path(experiment['raw_directory'])
    if not (raw/'manifest.json').is_file():
        raise RuntimeError('raw collector failed; inspect '+str(folder/'capture.log'))
    if (folder/'timing/manifest_closed.json').exists():
        with (folder/'timing_analysis.log').open('w') as stream:
            subprocess.run([sys.executable, str(PACKAGE/'scripts/monitor_control_timing.py'),
                'analyze', '--events', str(folder/'timing/timing_events.jsonl'),
                '--manifest', str(folder/'timing/manifest_closed.json'),
                '--output', str(folder/'timing_summary.json')], stdout=stream, stderr=stream, check=True)
    result=analyze(folder)
    print('ANALYSIS_DIRECTORY='+str(folder/'analysis'), flush=True)
    manifest=json.loads((raw/'manifest.json').read_text());w=manifest.get('writer',{})
    if manifest.get('state')!='closed' or w.get('writer_error') or w.get('queue_dropped') or w.get('unwritten'):
        raise RuntimeError('incomplete capture retained and analysed; not eligible for strict model validation')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    commands=parser.add_subparsers(dest='op', required=True)
    p=commands.add_parser('record', help='S8 is manually operated; passive raw capture only')
    p.add_argument('--experiment', type=Path, help='new prepared trial directory; reuse its run/session metadata')
    p.add_argument('--session-id')
    p.add_argument('--route')
    p.add_argument('--target-speed-mps', type=float, help='experiment metadata, never a speed command')
    p.add_argument('--direction', choices=('forward','reverse','mixed'), default='forward')
    p.add_argument('--repetition', type=int, default=1)
    p.add_argument('--duration', type=float, default=0., help='0: until S8 and reconstructor stop, or manual Ctrl-C')
    p.add_argument('--output-root', type=Path, default=WORKSPACE/'experiment_runs')
    p.add_argument('--production-workspace', type=Path, default=PRODUCTION)
    p.add_argument('--master-uri', default=os.environ.get('ROS_MASTER_URI','http://localhost:11311'))
    p.add_argument('--ros-python', default='/usr/bin/python2')
    p.add_argument('--data-origin',choices=('real_vehicle','synthetic_tool_test'),default='real_vehicle')
    p=commands.add_parser('analyze', help='offline re-analysis; requires new analysis directory')
    p.add_argument('--experiment', type=Path, required=True)
    args=parser.parse_args()
    if args.op=='record':
        if not args.experiment and (not args.session_id or not args.route or args.target_speed_mps is None):
            parser.error('provide --experiment or session-id/route/target-speed-mps')
        if args.repetition<1 or (args.duration!=0. and not 5.<=args.duration<=1800.):
            parser.error('positive repetition and duration 0 or 5..1800 required')
        try:
            record(args)
        except ValueError as error:
            raise SystemExit(str(error))
    else:analyze(args.experiment)


if __name__=='__main__':
    main()
