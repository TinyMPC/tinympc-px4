# Constraint examples and solver policy

This document defines the examples in `MPC_Step_Scenario`. All position
references and bounds are relative to the state latched when a scenario first
runs. Coordinates use PX4 local NED; a one-metre climb is therefore `z - 1`.

## Scenario definitions

### Hover (`0`)

- Reference: current `x/y/yaw`, `z - 1 m`, zero velocity.
- Position: `x/y ±2 m`, `z ∈ [-1.5, +0.25] m` from engagement.
- Velocity: `vx/vy ±1.5 m/s`, `vz ±1.0 m/s`.
- Input: acceleration ±4 m/s² per axis, yaw rate ±1 rad/s.

### Virtual wall (`1`)

- Reference: `x + 0.85 m`, `z - 1 m`.
- Physical example wall: `x + 1.0 m`.
- MPC planning boundary: `x + 0.98 m`. The 2 cm backoff demonstrates that a
  real boundary needs robustness margin.
- Lateral position: `y ±0.75 m`.
- Horizontal velocity: ±1.2 m/s; vertical velocity: ±1.0 m/s.
- Input: acceleration ±4 m/s² per axis, yaw rate ±1 rad/s.

The native comparison lets both controllers settle, then injects the same
`+0.9 m/s` forward velocity disturbance. The matched baseline removes only the
wall/state bounds; it retains the model, costs, reference, horizon, and input
limits.

### Corridor (`2`)

- Reference: `x + 2.0 m`, `y + 0.25 m`, `z - 1 m`.
- Corridor: `y ±0.35 m` from engagement.
- Forward range: `[-0.5, +3.0] m`; vertical range:
  `[-1.5, +0.25] m`.
- Lateral velocity: ±0.8 m/s; vertical velocity: ±1.0 m/s.
- Input: acceleration ±4 m/s² per axis, yaw rate ±1 rad/s.

The native benchmark adds a `+0.5 m/s` lateral velocity disturbance toward the
near corridor wall.

### Reduced authority (`3`)

- Reference: `x + 1.0 m`, `z - 1 m`.
- Horizontal position: `x ∈ [-1,+2] m`, `y ±1 m`.
- Horizontal velocity: ±0.8 m/s; vertical velocity: ±1.0 m/s.
- Input: horizontal acceleration ±0.75 m/s², vertical acceleration ±1.5 m/s²,
  yaw rate ±0.5 rad/s.

This is the first degradation example. It constrains the plan to known reduced
authority instead of computing an aggressive command and clipping it later.

### Three-dimensional figure-eight with tilt/thrust cone (`5`)

- Reference: a six-second Gerono figure-eight with `x` amplitude `1.5 m`, `y`
  amplitude `0.70 m`, and altitude varying from `0.60` to `0.75 m` above the
  engagement origin.
- Engagement: a bounded absolute climb ramp is followed by a two-second smooth
  blend into the moving reference. The path activates after reaching the
  relative altitude or after a bounded four-second takeoff window, avoiding a
  dependency on PX4 local-origin reset offsets.
- State envelope: `x ±1.8 m`, `y ±1.0 m`, NED `z ∈ [-1.5,+0.25] m`, horizontal
  velocity `±2.2 m/s`, and vertical velocity `±1.0 m/s`.
- Internal input: `[a_x, a_y, T_z, yaw_rate]`, where `T_z = g - a_z` is positive
  upward vertical specific thrust.
- Coupled constraint at every input knot:
  `sqrt(a_x^2 + a_y^2) <= tan(15 deg) T_z`.
- Input envelope: `a_x/a_y ±4 m/s²`, `T_z ∈ [0,g+3] m/s²`, and yaw rate
  `±1 rad/s`. The box and cone share one feasible ADMM slack trajectory.

The matched `figure_eight_box` case (`6`) uses the same model, costs, reference,
state envelope, input envelope, takeoff, and blend, but removes the cone. No
wind, impulse, or other external disturbance is applied to either case. In the
current deterministic run, both complete both lobes. The SOC case stays at
`15.000 deg` equivalent tilt with zero cone violation and zero fallbacks; the
box-only case reaches `15.969 deg` and violates the cone metric by
`0.177 m/s²`.

## Constraint semantics

The boxes and cone are TinyMPC ADMM projection constraints, not output-only
clamps. They apply to future state columns `1..N-1` and all input columns.
Horizon column zero is fixed to the measurement and deliberately has broad
bounds; a measured violation can therefore enter a recovery solve rather than
making the problem infeasible immediately.

For the SOC scenario, input boxes and the cone are projected into one shared
input slack. This avoids returning a box-feasible trajectory that disagrees
with a separate cone-feasible trajectory. The wrapper independently verifies
both the complete projected input trajectory and state trajectory before
publishing the first command. Model uncertainty margins and a terminal
recoverability set remain appropriate follow-on work before physical safety
claims.

## Fixed-budget solve and fallback

The box scenarios have a 50-iteration budget at 50 Hz; the SOC scenario
currently reserves up to 100 iterations:

1. A converged, finite, box-feasible solution is published with policy `1`.
2. If the iteration budget expires, TinyMPC's finite projected iterate is used
   as policy `2`; primal/dual residuals remain visible and distinguish it from
   convergence.
3. A non-finite, malformed, or constraint-infeasible solution triggers policy
   `-1`. Box scenarios command zero acceleration (hover semantics at the PX4
   acceleration interface). The SOC figure-eight uses a bounded, cone-feasible
   return-to-hover command based on position and velocity error.
4. Invalid state/setup input returns policy `-2`.

This is a bounded best-known policy, not a claim that a max-iteration solution
satisfies the dynamics equality to arbitrary precision.

## C diagnostic vector

`MPC_Step_Scenario` returns eight `float` values:

| Index | Value |
| --- | --- |
| 0 | solve policy (`1`, `2`, `-1`, or `-2`) |
| 1 | ADMM iterations |
| 2 | maximum primal residual |
| 3 | maximum dual residual |
| 4 | maximum projected state-box violation |
| 5 | maximum projected input-box/cone violation |
| 6 | cumulative fallback count |
| 7 | cumulative solve count |

The native benchmark prints these metrics plus optimized desktop p50/p95/p99
and worst solve time. The current Simulink app computes the diagnostic vector
but does not yet publish it to uORB. Publishing it to a logged PX4 diagnostic
topic is a required hardware-validation task.

The supplied-matrix, motor-level experiment uses a separate API and solver so
it cannot be selected accidentally by the trajectory-setpoint flight demo.
Its augmented-state constraints and benchmark are documented in
[`full_state_actuator_constraints.md`](full_state_actuator_constraints.md).
