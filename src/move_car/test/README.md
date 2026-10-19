# Restored Source Environment (2026-10-15)

The current control/reconstruction CPP and their two launch files were restored
by the user. This environment update does not modify those four files or their
parameters. Both targets explicitly require C++14; ROS Melodic uses the system
Python 2 interpreter and installed Eigen3. The sections below are historical:
their diagnostic/prediction features and production-function tests must not be
assumed to apply to these restored sources.

Build without starting vehicle nodes:

```sh
source /opt/ros/melodic/setup.bash
source /home/ubuntu/drive_car_copy/devel/setup.bash
cmake --build /home/ubuntu/drive_car_copy/build --target control_pub trajectory_reconstructor -- -j2
rospack find move_car
```

The package must resolve to `/home/ubuntu/drive_car_copy/src/move_car`; both
executables must resolve under `/home/ubuntu/drive_car_copy/devel/lib/move_car`.
At a safe stop, stop the existing control/reconstruction launches before manually
starting the restored versions in separate terminals with the environment above:

```sh
roslaunch move_car trajectory_reconstructor.launch
# Separate terminal, with the same environment:
roslaunch move_car movecar_improved.launch
```

Recompiling does not replace the code already loaded by running processes.
Debug output directories remain `data` and `cdata` in this workspace; node
startup may truncate their existing logs, so archive needed runs before restart.
This setup never restarts the hardware driver. This package does not build a
`movecar` target: the independent hardware executable remains
`/home/ubuntu/drive_car/devel/lib/move_car/movecar`. In a driver-only terminal,
source `/home/ubuntu/drive_car/devel/setup.bash` to select its own package before
using `rosrun move_car movecar`. Do not start a second hardware driver.

# Historical Regression Notes

## History-tail regression

Run after configuring/building the ROS workspace:

```sh
python3 src/move_car/test/run_history_tail_regression.py
```

The test includes the production implementation and relaxes private access only
in the test executable. It does not start a ROS node or send vehicle commands.
It covers straight, left/right 90-degree bends, overlapping tail observations,
preservation of the existing smoothing output, and updates without new points.

## 2026-09-22 finding

In runtime_debug.csv, interval 1790043790--1790043855 contained 1950 frames.
Reconstruction compute p95 was 1.751 ms. Among 1455 frames reporting
reference_stitched_tail, 336 (23.09%) had connector_span / (connector_span +
suffix_span) above 0.8. Some suffixes were only 0.16--0.30 m while the
connector was 1.5--2.1 m. This supports the long-connector symptom, not a
complete causal diagnosis of every corner-cut event.

The direct-motion-tail merge previously rejected an overlapping timestamp only
when its position was also close to the committed endpoint. Older points farther
away were appended again, introducing a time reversal and extra arc length.
The production-function test failed on that reversal before the fix. It passes
after enforcing chronological append against the current combined endpoint.
No smoothing weights, geometry rejection thresholds or control gains changed.

New reconstruction CSV columns:
- tail_overlap_skipped: older/equal timestamp points skipped in direct-tail merge.
- raw_discarded_prefix_span: arc length before the selected stable raw segment.

Raw input point timestamps are not available in the supplied logs, so
the historical run cannot be replayed exactly through this merge. A new vehicle
run is required to measure actual corner-cut reduction and remaining failures.

## 2026-10-13 diagnostics and collision regression

```sh
python3 src/move_car/test/run_control_regression.py
```

This covers the existing left/right feedback bounds, follow-distance envelope,
real obstacles, missing targets, cloud timeout, stale source timestamps, failed
TF, malformed/NaN clouds, valid empty clouds, and event serialization. The test
does not start ROS nodes or send commands. Existing launch settings, cached-path
speed caps, self-filter dimensions and lateral gains remain unchanged.

Reconstruction runtime/history CSVs append:
- `tail_overlap_skipped`: count for this export, reset every timer cycle.
- `raw_discarded_prefix_span`: arc before the selected raw-history segment.
- `latest_observation_age`: age of the latest successfully projected observation;
  this does not imply that it has been committed to history.
- `diagnostic_candidate_age/stamp`: endpoint source time of this cycle's raw candidate.
- `diagnostic_accepted_stamp`: accepted cached endpoint source time, not publish time.
- `cycle_candidate_reason`: build result captured before publication rewrites it,
  replaced by `full_audit:<reason>` if a later candidate audit rejects/trims it.
  These new fields do not reuse stale candidate reason fields from earlier cycles.

Control runtime CSV appends `cached_base_speed_cap` (-1 if not applied),
`diagnostic_horizon_cap` (-1 if arbitration did not run), and
`slqr_feedback_before_limit`. Existing `slqr_feedback_applied` is the after-limit
value in the feedforward-replacement branch, not the complete steering command.
Existing `usable_arc_remaining` and `speed_cap_winner` complete the speed chain.

With existing `enable_debug_csv_logs=true`, the control log directory also gets:
- `collision_events_0..3.csv`: 10Hz plus transitions, 20,000 rows per rotating file.
  Includes validity, release reason, source stamp, input/kept/self-rejected point
  counts, near point and drop counters. `raw_nearest` uses the sensor frame;
  `effective_nearest` and near XY use the base frame. Input count is full cloud
  size; kept/self-rejected counts are after the configured stride.
- `collision_cloud_event_0..7.csv/.bin`: up to two seconds before and after a
  collision rising edge, sampled at most 5Hz per cloud source. The CSV gives
  byte offsets/lengths into `.bin` and the exact 4x4 base-from-cloud transform
  used for that sample, with TF validity/age. A failed TF matrix must not be used.
  Each payload is an unmodified ROS-serialized `sensor_msgs/PointCloud2`, including
  its original header and fields. This captures the applied transform, not the
  complete `/tf` tree. Events rising within the same four-second capture are merged.

Capture is diagnostic only: no disk writes from the control callback. The
prebuffer is at most 8MiB, each event at most 16MiB, each frame at most 2MiB.
There is at most one queued event and one event being written; point payload
memory is bounded by 56MiB plus the current frame. Eight rotating event slots
bound cloud disk payload to 128MiB. Oversized frames/full queues are dropped and
counted rather than blocking control. Process restarts reuse the rotating slots;
archive a run before another run. Unexpected termination can leave a partial
last event; rows referring beyond the binary file size are incomplete.

Decode a row with ROS Python (no master required):
```python
from sensor_msgs.msg import PointCloud2
msg = PointCloud2()
with open(binary_path, 'rb') as data:
    data.seek(int(row['offset']))
    msg.deserialize(data.read(int(row['length'])))
```

A cloud-triggered collision latch now requires a valid fresh cloud to release.
Cloud timeout, bad TF/layout or all-NaN samples retain the existing stop, but do
not initiate a new stop themselves. A valid empty ROI can still release it;
removal of false self-returns requires measured dimensions, not guessed masking.
Test real obstacles and sensor dropout on a secured vehicle before normal runs.

## 2026-10-15 fixed-history shadow diagnostics

This implements the first, diagnostic stage of the accepted yaw/history plan.
The existing reference publisher, control state, steering law, speed arbitration,
gains and launch settings keep their behavior. The new history is a parallel
measurement basis, not an independent ground-truth trajectory and not a new
control input. Control rows explicitly record `shadow_only=1`.

Both nodes default `enable_history_tracking_diagnostics=true`. The reconstructor
publishes `/planner/history_tracking_snapshot` (override with
`history_tracking_snapshot_topic` on both nodes). Its custom message carries the
fixed-frame history and the base pose/twist at a common timestamp, with a run
epoch, immutable point IDs and continuity segment IDs. New message definitions
are built through the existing catkin targets.

Only raw geometry associated with an accepted, non-held reference candidate is
added to the shadow ledger. Existing points are never moved or re-smoothed.
New geometry needs timestamp overlap, at most 0.10m overlap discrepancy, and
continuous segment geometry; otherwise a distinct segment is retained without
an artificial connecting line. These are shadow diagnostic rules, not extra
vehicle-control gates. A new tail never replaces the active unfinished segment
with two-point support. Isolated singletons have no tangent and may be skipped
with an explicit diagnostic status when a later supported segment appears.
Changing segments requires reaching the old end and the next start. Progress
uses a limited forward search after initial acquisition; only passed points
more than 1m behind progress are trimmed. The ledger keeps at most 2048 points;
capacity exhaustion refuses additions and preserves unpassed history.

With the existing CSV logging enabled, two additional files are created:

- `data/history_tracking_points.csv`: append/trim journal with fixed coordinates,
  source timestamps and IDs. Segment starts identify bootstrap, missing time
  overlap, changed overlap geometry, position gaps, heading or curvature jumps.
  Points are written before trimming so late accepted history remains auditable.
- `cdata/history_tracking_debug.csv`: sampled at the existing runtime-log stride.
  Includes current base pose, supported rear/front projections, historical source
  time, lateral/heading errors, active controller errors and steering branch,
  candidate acceptance and connector boundary, trimming and overlap diagnostics.
  Pose source, IMU timestamp/age and observation-vs-pose query time difference
  expose clock mismatch and endpoint-clamped observation poses without changing
  the existing reconstruction pose estimator.
  `candidate_same_history_*` compares the current raw candidate to the frozen
  projection at the same source time; lack of time coverage is marked invalid.
  Its lateral change is positive to the left of the frozen historical tangent;
  use the historical turn direction before labeling that change an inside cut.
  Candidate connector index/arc refer to the candidate, not a held or smoothed
  published reference. A boundary of -1 means unavailable, never a measured zero.

World-frame error is `heading = wrap(ego_yaw - historical_segment_tangent)` and
`lateral = left_normal dot (axle_position - historical_projection)`. All angles
are radians and distances meters. Missing or unsupported historical projections
have `valid=0`, a reason, and NaN errors. Before-start/after-end states do not
extrapolate a tangent line and report it as measured history. The projections
refer to the same active segment; rear/front values may have different along-path
locations and must not be treated as identical error definitions.

`history_diagnostic_rear_axle_x` defaults to 0m on both nodes;
`history_diagnostic_front_axle_x` defaults to rear offset plus wheelbase on the
controller. Offsets are along base-frame X. These reproduce the existing axle
assumption, not a verified physical measurement. Set
`history_diagnostic_axle_offsets_calibrated=true` only after measuring base_link
against the axles. A rear-offset mismatch between nodes invalidates diagnostics.
`history_diagnostic_base_frame` defaults to `base_link` on the controller.

The controller buffers reconstruction poses for 5s. Queries interpolate only
across gaps no larger than 0.10s; after the latest pose, at most 0.10s of measured
constant-twist extrapolation is allowed and labeled in `pose_method`. Older,
future or missing data cannot silently become zero error. The stored reference
message is transformed through its publication-time base pose to the current
time for a separate comparison; this does not replace the active control path
or duplicate the controller's actuator-delay prediction. Check both
`reference_rebase_valid` (pose transform) and `reference_projection_valid`
(historical support). The message comparison may differ from the controller's
held/selected path, so its columns explicitly use `reference_message_*` names.

The main control regression runner also covers left/right 90-degree fixed-history
rotation, yaw perturbation error signs, overlap drift, target loss, disconnected
tails, passed-prefix retention, bounded storage, source-time comparison, pose
query gaps and local-path time rebasing. Tests do not start vehicle nodes.
After restarting both nodes, capture these two files alongside the existing logs.
The shadow files are truncated on node startup; archive runs before restarting.
This stage does not claim a measured reduction in corner cutting. Its output
determines whether to change the live history projection or execution model next.
