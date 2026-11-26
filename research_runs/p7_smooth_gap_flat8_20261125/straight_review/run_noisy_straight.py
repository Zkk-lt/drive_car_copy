#!/usr/bin/env python3
"""Compare the tracking reference change on an isolated synthetic vehicle."""
import argparse
import csv
import math
import os
from pathlib import Path
import shlex
import statistics
import subprocess


def rms(values):
    return math.sqrt(statistics.mean(value**2 for value in values))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--build-dir', type=Path, default=Path('build_smooth_integrated'))
    args = parser.parse_args()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    here = Path(__file__).resolve().parent
    workspace = here.parents[2]
    test = workspace / 'src/move_car/test/control'
    build = args.build_dir.resolve() / 'move_car/CMakeFiles/p6_closed_loop.dir'
    flags = {key: shlex.split(value) for key, value in
             (line.split(' = ', 1) for line in (build/'flags.make').read_text().splitlines()
              if ' = ' in line)}
    link = shlex.split((build/'link.txt').read_text())
    source, binary = output/'closed_loop.cpp', output/'closed_loop'
    subprocess.run(['patch', '-o', str(source), str(test/'run_p6_closed_loop.cpp'),
                    str(here/'noisy_straight_fixture.patch')], check=True)
    compile_command = [link[0], *flags['CXX_DEFINES'], *flags['CXX_INCLUDES'], *flags['CXX_FLAGS'],
                       '-I'+str(test), str(source), '-o', str(binary), *link[link.index('-o')+2:]]
    subprocess.run(compile_command, check=True)
    env = {k: v for k, v in os.environ.items() if not k.startswith(('P6_', 'P7_'))}
    for key in ('P6_SPEED_PREVIEW', 'P6_SYNC_PREVIEW', 'P6_ACTUATOR_ALIGNED_PREVIEW',
                'P6_TERMINAL_APPROACH', 'P7_TERMINAL_HANDOFF', 'P6_BOUNDED_FEEDBACK',
                'P6_CONDITIONED_GEOMETRY', 'P6_BASELINE_SEED', 'P6_DELAY_ALIGNED',
                'P6_COMMAND_PROJECTION', 'P6_RESPONSE_BASE', 'P6_RESPONSE_NEAR_SUPPORT',
                'P6_TURN_AWARE_RATE_CONTINUITY'):
        env[key] = '1'
    env.update(P6_STEERING_RATE_CONTINUITY_WEIGHT='.05', P6_YAW_RATE_TRACKING_WEIGHT='0',
               P6_MAX_ITERATIONS='2', P6_LATERAL_WEIGHT='39', P6_HEADING_WEIGHT='8.678570922328328',
               P6_SPEED_WEIGHT='6', P6_SMOOTHNESS_WEIGHT='24', P6_TARGET_SPEED='.4',
               P6_MODEL_YAW_GAIN='.9569791928640006')
    rows = []
    for conditioned in ('0', '1'):
        raw = output / ('condition_'+conditioned+'.csv')
        subprocess.run([str(binary), str(raw), '10', 'flat_nmpc_slqr', 'none', '8'],
                       env=dict(env, P7_STRAIGHT_REFERENCE=conditioned), check=True)
        with raw.open(newline='') as summary, Path(str(raw)+'.ticks.csv').open(newline='') as cycles:
            ticks = list(csv.DictReader(cycles))
            for row in csv.DictReader(summary):
                matching = [tick for tick in ticks if tick['path'] == row['path']]
                steering = [float(tick['steering']) for tick in matching]
                yaw = [float(tick['plant_body_yaw_rate']) for tick in matching]
                row.update(condition_enabled=conditioned,
                           condition_active=sum(float(tick['straight_blend'])>0 for tick in matching),
                           steering_second_difference_rms_radps2=rms((c-2*b+a)*30**2 for a,b,c in
                             zip(steering,steering[1:],steering[2:])),
                           body_yaw_rate_rms_radps=rms(yaw),
                           body_yaw_acceleration_rms_radps2=rms((b-a)*30 for a,b in zip(yaw,yaw[1:])))
                rows.append(row)
    with (output/'paired_summary.csv').open('w', newline='') as target:
        writer = csv.DictWriter(target, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)


if __name__ == '__main__':
    main()
