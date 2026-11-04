# S8 Stable Baseline

Frozen on 2026-11-04 following the user's successful vehicle test.
Git tag: `S8`. Development branch: `sucess_run_demo`.

This release preserves the exact tested source and launch settings. Creating
the release does not retune the algorithms or rename the runtime revisions:

- Reconstruction: `history_continuity_20261104_r6s8b`.
- Control: `reference_continuity_20261104_r3`.
- Accepted reconstruction run: `1793777995320545639`.
- Matching controller run: `1793777996985064346`.

`manifest.json` pins the two production CPPs and their two independent launches
with SHA-256 checksums. Future experiments should use new commits/tags rather
than moving or replacing the `S8` tag.

## Preserved Configuration

| Setting | Value |
| --- | ---: |
| Wheelbase | 0.60 m |
| Rear axle to body center | 0.30 m |
| Rear axle to lidar | 0.60 m |
| Continuous cruise target | 0.30 m/s |
| Follow-distance envelope upper speed | 0.48 m/s at 2.00 m |
| Follow-distance stop boundary | 0.50 m |
| Lateral acceleration setting | 0.25 m/s^2 |
| Acceleration / deceleration | 0.28 / 0.36 m/s^2 |
| Jerk | 0.80 m/s^3 |
| Effective steering bound | 0.32 rad |
| Command steering-rate setting | 1.00 rad/s |
| Actuator prediction delay / time constant | 0.325 / 0.44 s |

## Scope

The source checkpoint includes history-continuity recovery, origin-based
executable references, the existing center-motion SLQR model, smooth
longitudinal acceleration requests and causal steering-feedback pulse
conditioning. Raw/validated/deferred steering diagnostics remain enabled.

Reconstruction and control remain independent ROS nodes. Perception, chassis,
feedback protocol, external TF and their existing startup procedures are not
changed by this release. Raw experiment logs, local validation artifacts and
newly generated build/devel files are not added to the release commit.
Older tracked generated files in repository history are not build authority.

## Build and Run

Environment: Ubuntu 18.04, ROS Melodic, existing workspace dependencies.
Rebuild the two targets in the configured workspace with:

```bash
cmake --build build --target trajectory_reconstructor control_pub -- -j2
```

Use separate terminals for the original independent launches, after sourcing
ROS and this workspace. Keep perception and chassis startup unchanged.

```bash
source /opt/ros/melodic/setup.bash
source /home/ubuntu/drive_car_copy/devel/setup.bash
roslaunch move_car trajectory_reconstructor.launch
```

```bash
source /opt/ros/melodic/setup.bash
source /home/ubuntu/drive_car_copy/devel/setup.bash
roslaunch move_car movecar_improved.launch
```

Stop the old module instances while stationary before restarting. Do not run
duplicate nodes with the same name. A fresh checkout needs its own configured
catkin build; do not reuse another machine's CMake cache or executables.

## Validation Boundary

The user accepted the real-vehicle behavior of the run identified above.
Before acceptance, both native targets compiled and 12 isolated planner/SLQR
model cases passed, covering straight, left, right and S curves with and
without injected feedback pulses. Guard input tests, follow-distance tests
and history-tracking tests passed. No hardware node was started by the agent.

The existing curve-control and collision-diagnostic regression sources use
obsolete interfaces and do not compile against either the pre-patch or this
controller. They are not counted as passing. Offline model results and the
user's acceptance do not establish a universal accuracy or safety guarantee.
