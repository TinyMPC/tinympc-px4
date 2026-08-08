# Full-state actuator-envelope experiment

This experiment uses the supplied 50 Hz hover-linearized quadrotor matrices to
test a stronger TinyMPC control boundary than the PX4 trajectory-setpoint demo.
It now has two deliberately separate layers:

- a deterministic native model-in-the-loop constraint benchmark; and
- an experimental **SITL-only** PX4 module that closes the loop through PX4's
  torque/thrust control-allocation boundary. It never publishes raw motor
  outputs and cannot be built for a flight target.

The point is to put constraints that are normally handled in separate PX4
layers into one prediction problem:

- a future position boundary;
- local attitude and body-rate bounds;
- velocity bounds;
- absolute authority for each of four motors;
- motor command slew between 20 ms samples; and
- an asymmetric reduced-authority motor.

PX4 has useful limits and constrained control allocation, but its normal
cascaded position/attitude/rate stack does not plan all of these quantities
together over a future horizon. This experiment tests whether the supplied
model makes that joint TinyMPC problem practical before changing the flight
control boundary.

## State and input convention

The supplied nominal state has 13 values:

```text
[position(3), quaternion wxyz(4), velocity(3), body rates(3)]
```

The linear model has 12 states because attitude is represented locally around
the hover quaternion `[1, 0, 0, 0]` with three quaternion-vector/Rodrigues
error coordinates:

```text
x = [position(3), local attitude error(3), velocity(3), body rates(3)]
```

At hover, the attitude coordinates integrate body rate with `dt/2 = 0.01`, as
shown by the supplied `A` matrix. These coordinates are **not** Euler
roll/pitch/yaw. The SITL module latches a yaw-only engagement quaternion (so Z
remains vertical), forms `q_relative = inverse(q_heading) * q_current`,
converts its vector part to Rodrigues coordinates `q_xyz/q_w`, and changes PX4 NED/FRD to the model's
X-forward/Y-left/Z-up convention with `C = diag(1,-1,-1)`. Estimator reset
counters are latched at the same time and any subsequent reset fails closed.

The supplied trim is:

```text
u_hover = [0.297485, 0.297485, 0.297485, 0.297485]
```

The `B` matrix acts on motor-command deviations from this trim. The public
experimental API returns absolute normalized motor commands.

The checked-in `A/B` values match
`examples/controller_tinympc_eigen_task/src/quadrotor_50hz_params.hpp` in the
Robotic Exploration Lab Crazyflie firmware at commit
`4628c515271465b5f9d58e75580e25058b223f6a`. That source also confirms the
state ordering and Rodrigues attitude representation. The separately supplied
`0.297485` trim was not found in that upstream snapshot, so its exact
airframe/command-scaling provenance is still an open hardware-integration
item.

## Why the model is augmented

TinyMPC's basic box constraints apply to states and inputs at each horizon
knot. A motor slew constraint couples two successive commands. The experiment
turns that coupling into ordinary boxes by augmenting the state with the
previous motor-command deviation `d` and optimizing the change `du`:

```text
z       = [x; d]
x_next  = A*x + B*(d + du)
d_next  = d + du
```

Equivalently:

```text
z_next = [A B; 0 I] z + [B; I] du
```

The future augmented state receives absolute motor boxes, and `du` receives
the slew box. The first slew knot is also intersected explicitly with the
current absolute motor authority. This prevents a fixed-budget projected
iterate from returning a first command that satisfies slew but crosses an
absolute motor limit.

## Checked-in experiment

All cases use a 25-knot (`0.5 s`) horizon. The request is intentionally
`x = 0.95 m`. The constrained modes use a planning boundary of `0.855 m`
inside the illustrative physical wall at `0.87 m`; the 1.5 cm difference is a
margin, not a real-world guarantee.

The simultaneous future boxes are:

These are deliberately conservative benchmark values, not limits identified
or validated for a particular PX4 airframe.

| Quantity | Bound |
| --- | --- |
| Position x | `[-0.75, 0.855] m` |
| Position y / z | `±0.50 m` / `±0.40 m` |
| Local attitude x / y | `±0.18` |
| Local attitude z | `±0.25` |
| Velocity x / y / z | `±1.20 / ±1.00 / ±0.80 m/s` |
| Body rate x / y / z | `±2.80 / ±2.80 / ±2.00 rad/s` |
| Normal motor command | `[0.05, 0.60]` |
| Motor command change | `±0.045` per 20 ms sample |
| Degraded motor 0 | `[0.05, 0.34]` |

Four deterministic cases run for 500 control samples:

1. `hover_integration_reference` checks the zero-reference trim used before a
   PX4 handoff.
2. `predictive_envelope` applies every state, motor, and slew bound.
3. `reactive_baseline` removes the position/attitude/velocity/rate boxes but
   keeps the identical model, cost, target, motor authority, and motor slew.
4. `degraded_motor_0` restores the flight envelope and reduces only motor 0's
   upper command to `0.34`.

Run them with:

```bash
./scripts/run_full_state_benchmark.sh
```

A representative optimized x86 run on August 1, 2026 produced:

| Case | Maximum x | Maximum motor 0 | Maximum slew | Fallbacks |
| --- | ---: | ---: | ---: | ---: |
| Hover integration reference | 0.000 m | 0.297485 | 0.000 | 0 |
| Predictive envelope | 0.853 m | 0.342 | 0.045 | 0 |
| Reactive baseline | 0.952 m | 0.342 | 0.045 | 0 |
| Degraded motor 0 | 0.853 m | 0.340 | 0.045 | 0 |

The benchmark asserts the physical-wall backoff, the matched-baseline
difference, projected boxes, actual first-command motor/slew limits, degraded
motor limit, finite outputs, and no fallback. It prints solver residuals,
iterations, and desktop timing percentiles. Timing varies by host and is not
Pixhawk timing evidence.

This is a deterministic constraint regression, not yet a stock-PX4
performance comparison. A publishable controller comparison still needs the
same feasible reference/disturbance in SIH, Gazebo, and hardware with stock
and tuned PX4 baselines, as described in
[`research_direction.md`](research_direction.md).

## PX4/Gazebo full-state integration

`px4_external/src/tinympc_fullstate` runs the same 16-state controller at
50 Hz in PX4 POSIX/SITL. PX4 still owns commander, arming, estimator health,
Offboard loss handling, control allocation, actuator saturation reporting,
output limiting, logging, and the Gazebo motor interface. The module publishes
`vehicle_torque_setpoint` and `vehicle_thrust_setpoint`; it does not publish
`actuator_motors`.

The module validates the X500 airframe and all four configured rotor geometry
parameters before declaring itself ready. Crazyflie model motors
`[front-right, rear-right, rear-left, front-left]` are mapped to PX4 X500
rotors `[front-right, rear-left, front-left, rear-right]`. Model deviations
about `0.297485` are placed around `MPC_THR_HOVER` with a configurable SITL
gain. PX4's normalized allocation matrix is reconstructed from the live
geometry, self-tested, and used to turn the four requested motors into one
wrench. A delayed round-trip check then compares the allocated
`actuator_motors` with the request.

The controller publishes only an Offboard heartbeat until the vehicle is both
armed and explicitly switched to Offboard. Runtime guards cover stale state,
estimator resets, state envelope, solver policy, an 18 ms wall-clock deadline,
motor mapping, allocator round-trip error, and allocator saturation. A guard
failure requests autonomous Loiter and stops advertising readiness.

Build it together with the existing MATLAB-generated app:

```bash
PX4_DIR=/path/to/PX4-Autopilot ./scripts/build_px4_sitl.sh
```

Start X500 SITL, wait for `Ready for takeoff!`, and use the PX4 shell:

```text
tinympc_fullstate start hover 1.0
tinympc_fullstate status
commander takeoff
# Wait for the stock-PX4 takeoff/Loiter to settle.
commander mode offboard
tinympc_fullstate status
listener actuator_motors -n 1
listener control_allocator_status -n 1
commander land
```

`wall` and `degraded` are accepted experiment selections, but only `hover` has
passed closed-loop PX4/Gazebo validation so far. Do not use this module on
hardware.

On August 6, 2026, the final X500 Gazebo hover remained in Offboard for
24.5 simulated seconds and landed normally. The ULog showed 0 PX4 failsafe
samples, failure-detector status 0, 0 unachieved torque/thrust allocation
samples, 0 motor-saturation events, 0.175 m maximum displacement, and 0.100 m
final displacement from the engagement point. Live module telemetry reported
815 consecutive solves by the late inspection, a 35 us sampled solve, and
425 us worst observed wall-clock solve time. The active allocator watchdog
observed the requested four motors one allocation cycle later with errors far
below its 0.05 threshold.

## Solve and failure policy

The experimental solver uses `rho = 20`, at most 250 ADMM iterations, and the
same diagnostic vector as `MPC_Step_Scenario`.

- A converged finite solution is policy `1`.
- A fixed-budget projected solution is policy `2` only when projected boxes,
  the actual first motor command, and a `0.02` maximum primal-residual gate all
  pass.
- Otherwise policy `-1` moves every motor back toward hover by no more than
  the allowed slew.
- Invalid/non-finite state is policy `-2` with the same bounded return.

The residual is mixed-unit and is not a distance or safety guarantee. The
separate wall margin and explicit first-command checks remain necessary.

## Remaining hardware-integration gate

The SITL adapter closes the software loop, but it does not validate the model
for a real PX4 airframe. Before hardware use, all of the following remain
required:

1. Identify the exact airframe/mass/inertia and the supplied trim's
   normalized-command definition; re-identify the input scaling for the target.
2. Validate the implemented NED/FRD, quaternion/Rodrigues, and motor-order
   transforms against independent test vectors and target logs.
3. Validate one-step and multi-step prediction against SIH/Gazebo and logged
   hardware data across the intended envelope.
4. Port the control-allocation-boundary module deliberately to the selected
   PX4 hardware target and test every fail-closed transition. Do not bypass
   PX4 commander or publish raw motors.
5. Measure p50/p95/p99/worst solve time, deadline misses, stack, static
   RAM, heap, and flash on the actual flight-controller target.
6. Add explicit infeasibility/recoverability handling and test estimator
   discontinuities, stale state, model mismatch, saturation, and motor loss.

Until those gates pass, the result is evidence that the constraint formulation
and PX4/Gazebo software boundary work—not evidence that the motor controller is
safe to fly or that it outperforms tuned stock PX4.
