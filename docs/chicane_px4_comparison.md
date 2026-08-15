# Matched TinyMPC versus tuned PX4 cascaded-controller chicane

This demo isolates the useful difference between a horizon-aware constrained
controller and PX4's reactive cascaded position/velocity controller. There is
no wind, impulse, collision, or other external disturbance.

## Control boundaries

TinyMPC mode is an onboard PX4 integration, not direct motor control:

```text
PX4 EKF -> TinyMPC position/velocity horizon -> acceleration/yaw-rate
         -> PX4 acceleration-to-attitude -> attitude/rate loops
         -> PX4 control allocation -> Gazebo X500 motors
```

The matched baseline changes only the outer position/velocity controller and
uses the best zero-departure PX4 gain set found in the recorded tuning sweep:

```text
PX4 EKF -> stock PX4 cascaded position/velocity controller
         -> the same PX4 attitude/rate loops and allocation -> X500 motors
```

`tinympc_chicane start pid_tuned` verifies the four tuned horizontal gains and
the common 15-degree tilt setting before it will publish an Offboard heartbeat.
It does not replace or reimplement PX4's controller.

The separate `tinympc_fullstate` path goes deeper, from the PX4 EKF through a
12-state/four-motor TinyMPC horizon to PX4 torque/thrust allocation. Neither
path replaces PX4 estimation, commander, arming, failsafes, allocation, or
motor output handling.

## Matched experiment

- Common reference: two alternating 90-degree turns, finishing at local
  `(x,y)=(3,1) m` in 4.4 seconds.
- Center-position corridor: union of three 0.36 m-wide rectangles.
- Common airframe: Gazebo X500 using PX4 v1.15.3.
- Common limit: `MPC_TILTMAX_AIR=15 deg`; TinyMPC enforces the corresponding
  coupled specific-thrust cone over its 0.5 second horizon.
- Common estimator, attitude/rate controllers, allocation, and simulator.
- No wind or disturbance plugin. The course geometry is visual-only, so a
  miss is measured rather than converted into an impact.

The overlapping geometric legs receive a 0.30 second active-box handoff. This
does not widen the corridor union. It prevents a real inner-loop lag inside a
valid corner overlap from making the next 20 ms prediction artificially
infeasible.

## Results

The fair matched PX4/Gazebo comparison reports:

| Controller | Maximum outside corridor | RMS tracking error | Final tracking error | Completion |
| --- | ---: | ---: | ---: | --- |
| TinyMPC -> PX4 inner loops | 0.000 m | 0.113 m | 0.005 m | completed and landed normally |
| Tuned PX4 cascaded position/velocity | 0.000 m | 0.250 m | 0.130 m | completed and landed normally |

The successful TinyMPC run observed a 1.068 ms worst host solve. That is SITL
host timing, not Pixhawk timing evidence.

Both controllers satisfy the measured geometric corridor. Tuned PX4 does so
mainly by lagging the sharp reference; TinyMPC uses 25 state knots at 20 ms to
optimize 0.48 seconds of reference preview and has 2.2 times lower RMS tracking
error. PX4's stock cascaded controller is not claimed to be generally unable
to traverse the course.

## Tuning budget

No chicane-specific TinyMPC cost, horizon, ADMM penalty, or iteration setting
was tuned; the chicane reused the existing project solver configuration. The
TinyMPC/PX4 integration itself was engineered and debugged, so it should not be
described as an entirely out-of-the-box controller.

The tuned-PX4 study evaluated approximately 220,000 deterministic ideal/lag
model parameter samples and four PX4 SITL candidates. The best tested
zero-departure gains were:

```text
MPC_XY_P            0.21
MPC_XY_VEL_P_ACC    5.00
MPC_XY_VEL_I_ACC    0.17
MPC_XY_VEL_D_ACC    0.13
```

This is an exploratory tuned baseline, not a global PX4 optimum or a
hardware-safe gain recommendation.

The telemetry replay is checked in at
[`media/tinympc_chicane_px4_sitl_comparison.mp4`](../media/tinympc_chicane_px4_sitl_comparison.mp4).
It is rendered from the actual `vehicle_local_position` and `vehicle_status`
topics in the completed ULogs; it is not a nominal-model animation.

## Reproduction

Build PX4 with both SITL-only external modules:

```bash
./scripts/build_px4_sitl.sh
```

Start X500 Gazebo, spawn the visual course, and configure the common limit:

```bash
./scripts/spawn_gazebo_course.sh chicane
```

In the PX4 shell, run one controller per fresh vehicle instance. Let takeoff
fully settle before the handoff; verify near-zero `vx/vy/vz` first. TinyMPC
requires only the common tilt setting:

```text
param set MPC_TILTMAX_AIR 15
tinympc_chicane start mpc
commander takeoff
listener vehicle_local_position -n 1
commander mode offboard
tinympc_chicane status
commander land
```

For the tuned PX4 run, configure the recorded gains before starting the
verification mode:

```text
param set MPC_TILTMAX_AIR 15
param set MPC_XY_P 0.21
param set MPC_XY_VEL_P_ACC 5.0
param set MPC_XY_VEL_I_ACC 0.17
param set MPC_XY_VEL_D_ACC 0.13
tinympc_chicane start pid_tuned
```

Render any completed matched log pair with:

```bash
python scripts/render_chicane_comparison.py \
  --mpc-log /path/to/tinympc.ulg \
  --tuned-px4-log /path/to/px4_pid_tuned.ulg \
  --output media/tinympc_chicane_px4_sitl_comparison.mp4
```

The module is deliberately restricted to POSIX/SITL and fails closed to
autonomous Loiter on stale state, estimator reset, state-envelope departure,
invalid solver policy, or an 18 ms wall-clock deadline miss.
