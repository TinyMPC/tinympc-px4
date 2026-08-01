# Full-state actuator-envelope experiment

This experiment uses the supplied 50 Hz hover-linearized quadrotor matrices to
test a stronger TinyMPC control boundary than the PX4 trajectory-setpoint demo.
It is native model-in-the-loop code and is **not connected to PX4 actuators**.

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
shown by the supplied `A` matrix. These coordinates are **not** the Euler
roll/pitch/yaw values currently passed to `MPC_Step_Scenario` by Simulink.
That mismatch is one reason the motor-level API is not wired into the flight
model.

The supplied trim is:

```text
u_hover = [0.297485, 0.297485, 0.297485, 0.297485]
```

The `B` matrix acts on motor-command deviations from this trim. The public
experimental API returns absolute normalized motor commands.

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

Three deterministic cases run for 500 control samples:

1. `predictive_envelope` applies every state, motor, and slew bound.
2. `reactive_baseline` removes the position/attitude/velocity/rate boxes but
   keeps the identical model, cost, target, motor authority, and motor slew.
3. `degraded_motor_0` restores the flight envelope and reduces only motor 0's
   upper command to `0.34`.

Run them with:

```bash
./scripts/run_full_state_benchmark.sh
```

A representative optimized x86 run on August 1, 2026 produced:

| Case | Maximum x | Maximum motor 0 | Maximum slew | Fallbacks |
| --- | ---: | ---: | ---: | ---: |
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

## Flight-integration gate

Before this API can command a real PX4 airframe, all of the following are
required:

1. Confirm where the matrices came from, the airframe/mass/inertia, NED/FRD
   signs, motor numbering, and normalized-command definition.
2. Convert PX4's estimated quaternion into the exact local attitude-error
   coordinates used to identify the model; reset the local reference on
   estimator resets and controller engagement.
3. Validate one-step and multi-step prediction against SIH/Gazebo and logged
   hardware data across the intended envelope.
4. Integrate through a PX4 control-allocation boundary that retains arming,
   failsafes, actuator testing, and output limiting. Do not publish raw motors
   from an Offboard trajectory-setpoint block.
5. Measure p50/p95/p99/worst solve time, 20 ms deadline misses, stack, static
   RAM, heap, and flash on the actual flight-controller target.
6. Add explicit infeasibility/recoverability handling and test estimator
   discontinuities, stale state, model mismatch, saturation, and motor loss.

Until those gates pass, this benchmark is evidence that the constraint
formulation behaves correctly on the supplied linear model—not evidence that
the motor controller is safe to fly.
