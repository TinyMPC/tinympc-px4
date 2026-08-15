# TinyMPC-PX4

TinyMPC-PX4 integrates a constrained TinyMPC controller into PX4 at 50 Hz. The
optimizer is compiled into the PX4 application; no companion computer solves
the MPC problem. The native integration is currently restricted to PX4
POSIX/SITL and is not hardware-qualified.

## Configuration status

| Path | Purpose | Status |
| --- | --- | --- |
| `tinympc_chicane start mpc` | Native TinyMPC acceleration/yaw-rate outer loop with PX4 inner control | **Recommended SITL demonstration**; matched X500 run completed |
| `tinympc_chicane start pid_tuned` | Tuned stock PX4 cascaded outer-loop baseline on the same course | **Recommended fair baseline**; gain-verified matched X500 run completed |
| `tinympc_chicane start pid` | PX4 cascaded outer loop with the vehicle's current gains | Optional diagnostic; not the reported fair baseline |
| Native wrapper benchmarks | Deterministic constraint and regression checks | **Maintained CI path** |
| Simulink `guidance` / `direct` | Earlier generated PX4 integrations and maneuver reproductions | **Legacy reproduction path**; MATLAB is optional |
| `tinympc_fullstate` | TinyMPC attitude/rate/motor horizon to PX4 torque/thrust allocation | **Experimental SITL path**; only hover is closed-loop validated |

The path to reproduce first is the native acceleration-level comparison:

```text
PX4 EKF -> TinyMPC -> acceleration/yaw-rate -> PX4 attitude/rate control
         -> PX4 control allocation -> motors
```

The legacy generated app can instead publish a predicted position/velocity
plan (`guidance`) or acceleration/yaw-rate (`direct`). The experimental
full-state path uses the supplied hover-linearized `A/B` matrices to include
attitude, body rate, normalized motor authority, and slew in one horizon. It
rejoins PX4 at torque/thrust allocation and deliberately refuses flight-target
builds until the physical model and target timing are validated.

> [!CAUTION]
> This is an experimental research prototype, not a flight-certified
> controller. Reproduce the native and Software-in-the-Loop (SITL) tests before
> adapting it to hardware. Predicted MPC constraints are not, by themselves,
> guaranteed real-world safety boundaries; model error, solver residuals,
> estimator resets, and downstream saturation require margin and validation.

## Control architecture

```text
recommended native path
PX4 EKF state -> TinyMPC acceleration MPC -> PX4 attitude/rate control
                                                   |
                                                   v
                                       control allocation -> motors

experimental full-state path
PX4 EKF state -> TinyMPC state/motor MPC -> torque/thrust -> allocation
```

For the trajectory/acceleration API, the state is
`[x y z roll pitch yaw vx vy vz p q r]` in PX4's local NED frame. TinyMPC uses
25 state knots and 24 control intervals at 20 ms (0.48 s of control preview)
and returns
`[ax ay az yawspeed]`. In direct mode, zero vertical acceleration means hover
at PX4's acceleration interface; PX4 adds gravity compensation and converts
the acceleration vector to attitude and collective thrust.

The selected scenario latches the local state as an engagement-relative
origin. Hard state and input boxes are applied inside TinyMPC at every solve;
the initial state column is left unconstrained so an out-of-bounds measured
state does not make the problem infeasible by definition.

## Included constrained examples

| Scenario | Reference and constraints | What it demonstrates |
| --- | --- | --- |
| `hover` | Rise 1 m; local position box, velocity limits, ±4 m/s² acceleration, ±1 rad/s yaw rate | Safe integration and bounded hover |
| `virtual_wall` | Target 0.85 m forward; 0.98 m planning boundary inside a 1.0 m wall; velocity and input limits | Predictive braking before a geofence |
| `corridor` | Fly 2 m forward and 0.25 m laterally inside a ±0.35 m corridor | Coupled tracking with a lateral state constraint |
| `reduced_authority` | Move 1 m forward while horizontal acceleration is limited to ±0.75 m/s², vertical acceleration to ±1.5 m/s², and yaw rate to ±0.5 rad/s | Replanning with degraded control authority |
| `figure_eight_soc` | Fly a smooth 3D figure-eight while enforcing `sqrt(ax²+ay²) <= tan(15°) Tz` and a thrust/input envelope at every horizon knot | Joint trajectory and coupled tilt/thrust planning without an external disturbance |
| `figure_eight_box` | Same model, reference, costs, and boxes with only the cone removed | Matched intrinsic baseline for the SOC constraint |
| `chicane_soc` | Two sharp turns through a 0.36 m center-position corridor with the same 15° cone | Horizon-aware constrained planning versus tuned PX4 cascaded position/velocity control, without an external disturbance |

The native benchmark also includes `wall_baseline`, which uses the same model,
reference, input limits, and 0.9 m/s disturbance without the wall constraint.
The implemented scenario definitions and solver policy are documented in
[`docs/constraint_examples.md`](docs/constraint_examples.md).

The separate full-state benchmark compares the supplied motor-level model
with and without predictive flight-envelope bounds. Both receive the same
unsafe `x = 0.95 m` request. With a `0.855 m` planning boundary inside a
`0.87 m` physical wall, the constrained case reaches approximately `0.853 m`;
the reactive baseline reaches `0.952 m`. The same run checks normalized motor
commands in `[0.05, 0.60]`, command slew at `0.045/sample`, attitude/body-rate
bounds, and an asymmetric degraded-motor case capped at `0.340`. See
[`docs/full_state_actuator_constraints.md`](docs/full_state_actuator_constraints.md)
for the model convention, exact experiment, limitations, and flight-integration
gate.

## Current validation status

The following checks were run on August 6 and 11, 2026 with PX4 v1.15.3 and
MATLAB/Simulink R2026a:

- native C++ smoke test and all deterministic constraint assertions;
- Simulink update and code generation for `corridor + direct`;
- full PX4 SITL firmware compilation including TinyMPC source;
- SIH flight in direct acceleration mode: acceleration-only heartbeat,
  `nav_state=14` (Offboard), no PX4 failsafe, and normal landing.
- full-state X500 Gazebo hover at the PX4 torque/thrust allocation boundary:
  24.5 simulated seconds in Offboard, 0 failsafes, failure-detector status 0,
  0 unachieved allocator setpoints, 0 motor saturations, 0.175 m maximum and
  0.100 m final engagement-relative displacement, and normal landing/disarm;
- active requested-to-allocated motor round-trip watchdog and an observed
  425 us worst-case SITL host solve, below the 18 ms runtime deadline; and
- MATLAB model update, Simulink code generation, and a final PX4 link with
  both `px4_simulink_app` and `tinympc_fullstate` present.
- matched no-wind X500 chicane runs through the same PX4 inner loops: both
  TinyMPC and tuned PX4 remained inside the measured corridor and landed
  normally. TinyMPC completed 1,095 consecutive solves with a 1.068 ms worst
  host solve and achieved 0.113 m RMS tracking error, versus 0.250 m for tuned
  PX4. These are SITL host results, not flight-target timing evidence.

The earlier guidance-mode hover was also verified in SIH and Gazebo. In that
test the vehicle remained in Offboard for 151 seconds and held altitude within
about 3 cm. A recording is included at
[`media/tinympc_hover_gazebo.mp4`](media/tinympc_hover_gazebo.mp4).

Gazebo recordings of the direct-acceleration maneuvers are also included:

- [virtual wall](media/tinympc_virtual_wall_gazebo.mp4);
- [corridor](media/tinympc_corridor_gazebo.mp4), flown between visible,
  collision-enabled Gazebo walls surrounding the `y = ±0.35 m` planning
  corridor;
- [reduced authority](media/tinympc_reduced_authority_gazebo.mp4); and
- [no-wind SOC figure-eight](media/tinympc_figure_eight_soc_gazebo.mp4), flown
  through four visual trajectory arches.

The [matched chicane telemetry replay](media/tinympc_chicane_px4_sitl_comparison.mp4)
uses actual PX4 SITL ULogs and shows TinyMPC beside tuned PX4 on the same
no-wind course. The exact controller boundaries, quantitative results, and
reproduction steps are in
[`docs/chicane_px4_comparison.md`](docs/chicane_px4_comparison.md).

The corridor scene geometry is included at
[`quadtest/gazebo/tinympc_corridor.sdf`](quadtest/gazebo/tinympc_corridor.sdf).
The no-wind figure-eight course uses four visual arches located on the actual
trajectory. They intentionally have no collision geometry, so the SOC demo
does not introduce an unmodeled impact disturbance. The scene is included at
[`quadtest/gazebo/tinympc_figure_eight_course.sdf`](quadtest/gazebo/tinympc_figure_eight_course.sdf).

These are visual integration demonstrations, not evidence that the predicted
constraints are guaranteed in flight. The deterministic native benchmarks and
future ULog-based metrics remain the quantitative constraint checks.

The accepted PX4/Gazebo figure-eight take completed both lobes (`x` from
`-1.334` to `+1.487 m`, `y` from `-0.731` to `+0.936 m`), had zero commanded
cone violation, no PX4 failsafe, and a normal landing/disarm. PX4's downstream
attitude loop reached about `28.4 deg` roll, so the `15 deg` acceleration cone
is not a measured-airframe attitude guarantee. Closing that model/inner-loop
gap remains part of the motor-level integration gate.

The deterministic native benchmark currently shows the constrained wall case
remaining at approximately 1.001 m after the injected disturbance, versus
1.028 m for the matched unconstrained baseline. The corridor case remains
inside 0.35 m and the reduced-authority case respects its smaller input box.
In the no-disturbance 3D figure-eight, the SOC controller completes both lobes
at a maximum equivalent tilt of 15.000 degrees with zero cone violation and
zero fallbacks; the matched box-only controller reaches 15.969 degrees and
violates the cone metric by 0.177 m/s².
Reported desktop solve timing is included in the CSV output, but it is **not**
a substitute for Pixhawk-class target timing.

## Requirements

- Native tests: Ubuntu 22.04, Git, CMake, Ninja, and a C++17 compiler.
- PX4 SITL: Python 3 and the PX4 v1.15.3 build toolchain.
- Gazebo reproduction: Gazebo Harmonic.
- Telemetry-video rendering: `pyulog`, NumPy, Matplotlib, and FFmpeg.

MATLAB is **not required** for the recommended native chicane comparison. The
legacy generated-app reproduction requires MATLAB/Simulink R2026a, MATLAB
Coder, Simulink Coder, Embedded Coder, UAV Toolbox, the UAV Toolbox Support
Package for PX4 Autopilots, and `patch`.

The optional
[`tinympc-matlab`](https://github.com/TinyMPC/tinympc-matlab) package is useful
for MATLAB-only experiments but is not in the flight feedback path.

## Quick start

Clone the repository, build TinyMPC, and run the fast native checks:

```bash
git clone https://github.com/TinyMPC/tinympc-px4.git
cd tinympc-px4
./scripts/setup_tinympc_px4.sh
./scripts/run_constraint_benchmarks.sh
./scripts/run_full_state_benchmark.sh
```

The benchmark asserts finite outputs, solver-projected state/input bounds,
scenario-level closed-loop limits, and zero fallbacks. It reports iterations,
residuals, best-effort solves, and optimized desktop timing percentiles.

Fetch the pinned PX4 release and build SITL with both native external modules:

```bash
./scripts/setup_px4_firmware.sh
./scripts/build_px4_sitl.sh
```

Native p50/p95/p99 timing is only a host regression measurement; it does not
establish the 20 ms deadline on a flight-controller MCU.

### Recommended matched chicane

Run `tinympc_chicane start mpc` for TinyMPC or configure the documented gains
and run `tinympc_chicane start pid_tuned` for the fair PX4 baseline. The tuned
mode refuses to start if the gain set or 15-degree tilt setting does not match.
Use one fresh X500 instance per controller and let takeoff fully settle before
switching to Offboard. The complete commands, matched conditions, metrics, and
ULog replay instructions are in
[`docs/chicane_px4_comparison.md`](docs/chicane_px4_comparison.md).

### Experimental full-state PX4/Gazebo hover

Builds include the external `tinympc_fullstate` module automatically. Start
X500 Gazebo as described below, wait for `Ready for takeoff!`, then run:

```text
tinympc_fullstate start hover 1.0
tinympc_fullstate status
commander takeoff
# Let stock PX4 settle in Loiter before the explicit handoff.
commander mode offboard
tinympc_fullstate status
listener actuator_motors -n 1
listener control_allocator_status -n 1
commander land
```

This bypasses PX4's cascaded position/attitude/rate controllers, but not PX4's
commander, estimator, failsafes, control allocator, or actuator output limits.
The `wall` and `degraded` selections build, but only `hover` is closed-loop
validated. The exact state transform, motor mapping, guards, test result, and
remaining hardware gate are documented in
[`docs/full_state_actuator_constraints.md`](docs/full_state_actuator_constraints.md).

## Legacy MATLAB/Simulink app

Explicitly enable the generated app in the PX4 SITL board configuration. The
default native-only setup deliberately leaves it disabled:

```bash
ENABLE_SIMULINK_APP=1 ./scripts/setup_px4_firmware.sh
```

### Patch the MATLAB PX4 support package

The R2026a generated uORB writer leaves timestamps at zero and does not publish
the `offboard_control_mode` heartbeat PX4 requires. Apply the narrow,
idempotent compatibility patch:

```bash
./scripts/patch_matlab_support_package.sh
```

The patch stamps messages and derives the Offboard control level from finite
trajectory fields. It advertises position control in guidance mode and
acceleration control in direct mode. If support packages are installed in a
non-default location:

```bash
MATLAB_SUPPORT_ROOT=/path/to/SupportPackages/R2026a \
  ./scripts/patch_matlab_support_package.sh
```

Other MATLAB releases may contain different support-package source and have
not been validated with this patch.

### Conservative hover/guidance build

The default remains the previously verified configuration:

```bash
matlab -batch "cd('quadtest'); run_tinympc_px4_demo('update')"
matlab -batch "cd('quadtest'); run_tinympc_px4_demo('build')"
./scripts/build_px4_sitl.sh
```

### TinyMPC-owned direct build

Choose the scenario at code-generation time. Unknown values fail closed to
`hover + guidance`.

```bash
TINY_MPC_SCENARIO=corridor TINY_MPC_OUTPUT_MODE=direct \
  matlab -batch "cd('quadtest'); run_tinympc_px4_demo('update')"

TINY_MPC_SCENARIO=corridor TINY_MPC_OUTPUT_MODE=direct \
  matlab -batch "cd('quadtest'); run_tinympc_px4_demo('build')"

./scripts/build_px4_sitl.sh
```

Valid flight scenarios are `hover`, `virtual_wall`, `corridor`,
`reduced_authority`, `figure_eight_soc`, and `figure_eight_box`. Valid output
modes are `guidance` and `direct`.

## Legacy MATLAB/Simulink SIH reproduction

Start the generated firmware from the PX4 root filesystem:

```bash
cd third_party/PX4-Autopilot/build/px4_sitl_default/rootfs
PX4_SYS_AUTOSTART=10040 ../bin/px4 ../../px4_sitl_default/etc \
  -s etc/init.d-posix/rcS
```

Wait for `Ready for takeoff!`, then run in the PX4 shell:

```text
px4_simulink_app start
listener offboard_control_mode -n 1
commander mode offboard
commander arm
# Observe the selected engagement-relative scenario.
commander land
```

For a direct build, `listener offboard_control_mode` must show
`acceleration: True` and position/velocity false. Confirm the actual setpoint
and flight state with:

```text
listener trajectory_setpoint -n 1
listener vehicle_status -n 1
listener vehicle_local_position -n 1
```

Direct mode should show `NaN` position/velocity and finite acceleration.
PX4 logs are written under `build/px4_sitl_default/rootfs/log/`.

## Gazebo reproduction

Install Gazebo Harmonic from the OSRF packages, build SITL, and start X500:

```bash
./scripts/build_px4_sitl.sh

cd third_party/PX4-Autopilot/build/px4_sitl_default/rootfs
PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 HEADLESS=1 \
  ../bin/px4 ../../px4_sitl_default/etc -s etc/init.d-posix/rcS
```

Attach a GUI with `gz sim -g` if desired, then use the same PX4 shell commands.
Spawn the visible geometry into the running `default` world before arming:

```bash
./scripts/spawn_gazebo_course.sh figure_eight
# or: ./scripts/spawn_gazebo_course.sh corridor
# or: ./scripts/spawn_gazebo_course.sh chicane
```

The figure-eight course uses visual trajectory gates without collision geometry
and adds no wind or other disturbance plugin. The matched native SOC/box
benchmark provides the quantitative constraint comparison.
If arming reports a yaw-estimate error, wait for EKF convergence. If parameters
from an earlier run cause preflight failures, stop PX4, remove only
`rootfs/parameters*.bson`, and restart it.

## Scope and prior work

Onboard MPC in PX4 is not an unclaimed idea. A 2018 thesis implemented an MPC
angular-rate controller in PX4 on Pixhawk, and a 2023 thesis exported a model
predictive controller as a PX4 module for SITL. TinyMPC itself demonstrated MPC
on a resource-constrained Crazyflie-class platform. Relevant starting points:

- [TinyMPC: Model-Predictive Control on Resource-Constrained Microcontrollers](https://arxiv.org/abs/2310.16985)
- [Design and Implementation of Model Predictive Control on Pixhawk Flight Controller (2018)](https://core.ac.uk/download/pdf/188220406.pdf)
- [Model Predictive Control for a Quadcopter under Contact with Environment (2023)](https://repositum.tuwien.at/bitstream/20.500.12708/189009/1/Bauer%20Paul%20-%202023%20-%20Model%20Predictive%20Control%20for%20a%20Quadcopter%20under%20Contact%20with...pdf)

This project's narrower contribution is an open, reproducible TinyMPC/PX4
integration with an explicit direct-acceleration boundary, solver-enforced
constraints, deterministic comparisons, diagnostics, and failure policy. No
claim of “first onboard PX4 MPC” is made.

The next research steps and fair comparison protocol are in
[`docs/research_direction.md`](docs/research_direction.md).

## Repository layout

- `quadtest/quadtest.slx`: uORB state readers, TinyMPC controller call, and
  trajectory-setpoint writer.
- `quadtest/wrapper/`: C API, constraints, diagnostics, native smoke test, and
  deterministic benchmarks, including the opt-in full-state/motor experiment.
- `px4_external/`: native SITL-only chicane module at the acceleration boundary
  and experimental full-state module at the torque/thrust allocation boundary.
- `quadtest/tinympc/TinyMPC/`: vendored TinyMPC source used by native tests and
  the generated PX4 app.
- `quadtest/setup_tinympc_px4.m`: deterministic model and custom-code setup.
- `quadtest/init_tinympc_quad.m`: matching MATLAB model plus build selection.
- `scripts/`: pinned PX4 build, benchmark, and support-package helpers.
- `docs/`: constraint definitions, integration evidence, and research direction.
- `media/`: verified Gazebo recordings and the matched chicane ULog replay.
- `paper/`: editable source, renderer, and PDF for the workshop abstract draft.

Generated Simulink code, native build products, firmware, and virtual
environments are intentionally excluded. A fresh clone rebuilds them from the
checked-in model and source.

## License

This repository is available under the [MIT License](LICENSE). The vendored
TinyMPC snapshot retains its license in
[`quadtest/tinympc/TinyMPC/LICENSE`](quadtest/tinympc/TinyMPC/LICENSE).
See [`CONTRIBUTING.md`](CONTRIBUTING.md) before proposing changes.
Citation metadata is available in [`CITATION.cff`](CITATION.cff).
