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

## Constraint semantics

The bounds are TinyMPC ADMM projection constraints, not output-only clamps.
They apply to future state columns `1..N-1` and all input columns. Horizon
column zero is fixed to the measurement and deliberately has broad bounds; a
measured violation can therefore enter a recovery solve rather than making
the problem infeasible immediately.

The current implementation uses axis-aligned boxes. A tilt/thrust envelope is
not exactly a per-axis acceleration box. Input second-order-cone constraints,
model uncertainty margins, and a terminal recoverability set are appropriate
follow-on work before physical geofence claims.

## Fixed-budget solve and fallback

The controller has a 50-iteration budget at 50 Hz:

1. A converged, finite, box-feasible solution is published with policy `1`.
2. If the iteration budget expires, TinyMPC's finite projected iterate is used
   as policy `2`; primal/dual residuals remain visible and distinguish it from
   convergence.
3. A non-finite, malformed, or non-box-feasible solution triggers policy `-1`.
   Acceleration is set to zero (hover semantics at the PX4 acceleration
   interface) and the last plan is retained for guidance mode.
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
| 5 | maximum projected input-box violation |
| 6 | cumulative fallback count |
| 7 | cumulative solve count |

The native benchmark prints these metrics plus optimized desktop p50/p95/p99
and worst solve time. The current Simulink app computes the diagnostic vector
but does not yet publish it to uORB. Publishing it to a logged PX4 diagnostic
topic is a required hardware-validation task.
