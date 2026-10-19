# Offline Actuator Fit: Current Logs

## Decision For The Controller Owner

Keep the configured live steering delay at **0.5 s** and the live modeled tau
default at **0 s**. No controller, launch configuration, or hardware was changed.

The unconstrained steering fit prefers delay **0.19 s**, tau **0.41 s**, but that
tau must not simply be added on top of the existing 0.5 s delay. Holding delay at
0.5 s gives a training-selected tau of **0.16 s**, with only **4.1%** held-out RMSE
improvement. Feedback provenance/calibration remains unverified, and the joint
delay/tau training objective has a broad near-optimal region. These are offline
candidates, not calibrated parameters for application.

## Reproduction

Python 3.7+ standard library only. From the repository root:

```bash
python3 -B src/move_car/test/fit_actuator_response.py
python3 -B src/move_car/test/fit_actuator_response.py --output /tmp/actuator-fit-report.json
python3 -B -m unittest discover -s src/move_car/test -p test_fit_actuator_response.py -v
```

Default output is JSON on stdout. `--output` explicitly creates a NEW file and
refuses any existing destination, including input logs. There are no ROS imports,
hardware calls, parameter application, or edits to input files. The focused suite
contains 28 passing tests (17 runtime/model tests plus 11 CAN-log tests). The
existing regression runner is not modified.

Snapshot analyzed on 2026-10-15:

| Input | Rows | SHA-256 |
| --- | ---: | --- |
| `cdata/runtime_debug.csv` | 3,164 | `f6dcb5bb17ec59c6dc472b8b0d46f72ca0b7f9b545fe891cf24628397563ef84` |
| `data/runtime_debug.csv` | 9,643 | `63f206a7688a263af5f4bca0a93dde23a3688b277e1298e83713d74a84c879e1` |

Both input hashes were checked unchanged after analysis. JSON records the hashes,
input columns, units, invalid/stopped counts, cadence, and effective gap limits.

## Method And Support

- Commands are `cmd_delta` in radians and `cmd_v` in m/s. Preferred steering
  feedback is `steer_scaled` in radians; speed feedback is `wheel_speed` in m/s.
  The already-scaled logged fields are not scaled again. Separate explicit CLI
  angle/speed unit options support other exports; units and sign are not guessed.
- Steering and speed each fit a unity-gain pure time delay followed by first-order
  lag. The coarse grid covers delay 0..0.7 s and tau 0..0.6 s at 0.05 s spacing,
  with local 0.01 s refinement. Independent tau=0 delay baselines use 0.01 s
  spacing. Fixed-delay comparisons select tau on training data only.
- Commands use zero-order hold at original command timestamps. Integration is
  exact across delayed command changes, even between feedback samples. No measured
  derivatives, interpolation of command ramps, fitted gain, or fitted offset.
- Forward runs require both commanded and measured speed >0.05 m/s. Stops,
  nonfinite/invalid rows, duplicate timestamps, clock resets, and gaps break runs.
  Gaps exceed the smaller of 0.25 s and 1.75 times each stream's median positive
  period. Cross-file clock resets are refused as ambiguous; a combined same-file
  export can retain separate epochs without joining across resets.
- Intersections of valid command/feedback runs yield 12 segments and 9,137 aligned
  samples. A global contiguous 70% prefix trains; the remaining 30% validates.
  The boundary is stamps 1792031001.612849 / 1792031001.646214. No per-run shuffle.
- Each partition/run reserves 0.7 s of its own command history, then initializes
  prediction with measured feedback at that fitting segment's start. The initial
  seed is not scored, and no prior partition's history or fitted state is reused.
  Short segments are excluded. Scoring uses 6,114 train samples in 8 segments
  (203.8 s), and 2,720 validation samples in 1 segment (90.7 s).
- Steering command range is 0.64 rad in both partitions, with 1,090 / 493 command
  changes >=0.01 rad. Speed has 380 / 132 changes >=0.01 m/s. Counts are descriptive
  support, not independent statistical observations. Six aligned samples carry
  `steer_scale_suspect` flags.

## Results

All parameter choices below use training data only; RMSE is reported separately
on the chronological holdout.

| Channel / model | Delay (s) | Tau (s) | Train RMSE | Validation RMSE |
| --- | ---: | ---: | ---: | ---: |
| Steering joint fit | 0.19 | 0.41 | 0.036897 rad | 0.049728 rad |
| Steering fitted pure delay | 0.43 | 0 | 0.055555 rad | 0.064663 rad |
| Steering configured delay, zero tau | 0.50 | 0 | 0.057124 rad | 0.065091 rad |
| Steering fixed configured delay, fitted tau | 0.50 | 0.16 | 0.054603 rad | 0.062444 rad |
| Speed joint fit | 0.21 | 0.10 | 0.024476 m/s | 0.024406 m/s |
| Speed fitted pure delay | 0.29 | 0 | 0.025035 m/s | 0.024290 m/s |
| Speed fixed 0.5 s delay, fitted tau | 0.50 | 0 | 0.029623 m/s | 0.028394 m/s |

The steering joint fit improves held-out RMSE by 23.1% over the fitted pure-delay
baseline. Speed's added lag worsens held-out RMSE by 0.48%. Steering candidates
within 5% of the best evaluated training RMSE span delay 0.10..0.25 s and tau
0.30..0.55 s. These are grid support ranges, **not confidence intervals**. Both
channels report low confidence; neither supports automatic application.

## Constant-Command Speed Windows

28 disjoint low-speed windows total 84.91 s. Criteria: >=2 s duration, commanded
speed <=0.5 m/s, total command range <=0.005 m/s, and a 1.3 s initial exclusion.
The exclusion is a fixed descriptive transient guard, not proof of equilibrium.

Window command means cover only 0.09456..0.17389 m/s. Median window mean tracking
error is -0.00133 m/s; individual mean errors span -0.01539..0.00521 m/s. Within-
window wheel-speed standard deviations span 0.00141..0.04255 m/s. JSON includes
each window's timestamps, counts, command mean/range, wheel-speed mean/std/min/max,
mean error, and measured/command ratio.

These statistics do **not** identify powertrain nonlinearity. Command variation,
closed-loop steering/load, wheel-speed quantization, unverified feedback, unknown
sensor freshness, and narrow operating support limit interpretation. Logging
timestamps do not establish hardware acquisition latency, and feedback may be
filtered or echoed. Independent physical feedback calibration and further
held-out excitation are needed before reconsidering live tau.

## Optional CAN Driver Input

The original runtime mode and results above remain unchanged. To prefer successful
encoded TX and fresh raw RX from the new driver CSV:

```bash
python3 -B src/move_car/test/fit_actuator_response.py --can-log /path/to/movecar_can.csv
python3 -B src/move_car/test/fit_actuator_response.py --can-log /path/to/movecar_can.csv --output /tmp/can-actuator-fit.json
```

`--can-log` replaces both runtime inputs, not just steering or speed. Runtime CSV
paths and runtime unit overrides are ignored in this mode. No log is discovered
automatically and there is no fallback to runtime data for missing CAN support.
The schema was checked against `/home/ubuntu/drive_car/src/move_car/docs/CAN_DRIVER.md`
and `/home/ubuntu/drive_car/src/move_car/src/movecar.cpp`.

- Accept a command only when `tx_attempted==1`, `tx_result==1`, and
  `encoding_valid==1`. Use `encoded_speed_mm_s / 1000` in m/s and
  `encoded_steering_centideg * pi / 18000` in radians. Raw callback commands are
  not used, including commands coalesced away before transmission.
- Accept feedback only when `fresh_valid==1`. Decode `feedback_speed_mm_s` and
  `feedback_steering_centideg` with the same conversions. Nonfresh rows' placeholder
  zeros are never measurements; even malformed unused payload values are ignored.
- TX time is **`tx_end_steady`**; RX time is **`rx_end_steady`**. Both come from
  the same driver's steady-clock domain. Validate their ordering against the
  bracketing start times and `steady_time`. ROS, wall, callback receipt, and raw
  CAN timestamps are not used for fitting or synchronization.
- A failed/invalid TX makes actuation unknown. Exclude that cycle's RX and all
  following feedback until the next successful TX. There is no hold through this
  unknown interval or fitted-state carryover on recovery.
- `new_command==0` with no TX attempt is an ordinary idle cycle, not a failure.
  The previous successful encoded command stays held, even though this row's
  `encoding_valid` and encoded payload fields are zero. Do not impose a TX cadence
  or invent heartbeat transmissions.
- Each repeated process header or decreasing `steady_time` clears the held
  command and starts a new run. Duplicate steady stamps also break support.
  Missing/malformed log rows or logging gaps clear command knowledge until a new
  successful TX because the missing interval could conceal a transmission.
- Fresh-feedback gaps break fitting segments but preserve a known held command
  when all intervening cycles were logged. Logging/fresh-RX gap limits use the
  smaller of `--max-gap` and 1.75 times the median logged cycle period. Stops and
  short segments are excluded as in runtime mode.

JSON records the CAN input hash, conversions, event-clock columns, process/run
boundaries, and acceptance/exclusion counts. The chronological split, measured
segment initialization, grids, baselines, constant-command statistics, controller
delay policy, and zero-tau live default are shared with runtime mode.

Vendor-reported TX success is not measured actuation. End-of-call timestamps are
a consistent convention with uncertainty up to the respective call durations,
not physical acquisition/actuation times. `fresh_valid` means newly accepted, not
proven physically recent; firmware, feedback provenance, and saturation still
require review. No hardware was opened, started, or exercised. CAN validation is
synthetic; no measured CAN dataset or new calibrated live parameters are claimed.
