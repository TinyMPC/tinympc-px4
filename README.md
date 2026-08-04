# TinyMPC-PX4

TinyMPC-PX4 runs a constrained TinyMPC controller at 50 Hz **on the PX4 flight
controller**. The optimization is compiled into the PX4 app; no companion
computer or offboard optimizer solves the MPC problem.

The repository supports two intentionally separate control boundaries:

- `guidance` publishes a predicted TinyMPC position/velocity plan that PX4's
  position controller tracks. This is the conservative, previously verified
  hover integration.
- `direct` publishes only TinyMPC acceleration and yaw-rate commands. Position
  and velocity are `NaN`, so PX4 bypasses its position/velocity feedback and
  retains acceleration-to-attitude conversion, attitude/rate control, control
  allocation, and safety handling.

There is also a **native-only experimental full-state path** built from the
supplied 50 Hz hover-linearized `A/B` matrices. It makes normalized motor
commands and motor slew part of the MPC state/control problem, allowing one
horizon to enforce position, attitude, body-rate, motor-authority, and slew
limits together. It is deliberately not connected to the PX4 actuator output:
the matrix airframe, motor order, command scaling, and local attitude-error
conversion still need target validation.

> [!CAUTION]
> This is an experimental research prototype, not a flight-certified
> controller. Reproduce the native and Software-in-the-Loop (SITL) tests before
> adapting it to hardware. Predicted MPC constraints are not, by themselves,
> guaranteed real-world safety boundaries; model error, solver residuals,
> estimator resets, and downstream saturation require margin and validation.

## Control architecture

```text
                             guidance mode
PX4 EKF state -> TinyMPC ----------------------> PX4 position/velocity control
      |             |                                      |
      |             | direct mode: acceleration only       |
      |             +--------------------------------------+
      |                                                    v
      +----------------------------------------> attitude/rate control
                                                           |
                                                           v
                                               control allocation -> motors
```

The state is `[x y z roll pitch yaw vx vy vz p q r]` in PX4's local NED
frame. TinyMPC uses a 25-knot, 0.5 s horizon and returns
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

The following checks were run on August 1, 2026 with PX4 v1.15.3 and
MATLAB/Simulink R2026a:

- native C++ smoke test and all deterministic constraint assertions;
- Simulink update and code generation for `corridor + direct`;
- full PX4 SITL firmware compilation including TinyMPC source;
- SIH flight in direct acceleration mode: acceleration-only heartbeat,
  `nav_state=14` (Offboard), no PX4 failsafe, and normal landing.

The earlier guidance-mode hover was also verified in SIH and Gazebo. In that
test the vehicle remained in Offboard for 151 seconds and held altitude within
about 3 cm. A recording is included at
[`media/tinympc_hover_gazebo.mp4`](media/tinympc_hover_gazebo.mp4).

Gazebo recordings of the direct-acceleration maneuvers are also included:

- [virtual wall](media/tinympc_virtual_wall_gazebo.mp4);
- [corridor](media/tinympc_corridor_gazebo.mp4), flown between visible,
  collision-enabled Gazebo walls surrounding the `y = ±0.35 m` planning
  corridor; and
- [reduced authority](media/tinympc_reduced_authority_gazebo.mp4).

The corridor scene geometry is included at
[`quadtest/gazebo/tinympc_corridor.sdf`](quadtest/gazebo/tinympc_corridor.sdf).

These are visual integration demonstrations, not evidence that the predicted
constraints are guaranteed in flight. The deterministic native benchmarks and
future ULog-based metrics remain the quantitative constraint checks.

The deterministic native benchmark currently shows the constrained wall case
remaining at approximately 1.001 m after the injected disturbance, versus
1.028 m for the matched unconstrained baseline. The corridor case remains
inside 0.35 m and the reduced-authority case respects its smaller input box.
Reported desktop solve timing is included in the CSV output, but it is **not**
a substitute for Pixhawk-class target timing.

## Requirements

- Ubuntu 22.04 (verified host platform).
- Git, CMake, a C++17 compiler, Python 3, and the PX4 v1.15.3 build toolchain.
- MATLAB/Simulink R2026a with MATLAB Coder, Simulink Coder, Embedded Coder,
  UAV Toolbox, and the UAV Toolbox Support Package for PX4 Autopilots.
- `patch` for the MathWorks support-package compatibility patch.
- Gazebo Harmonic only for the optional Gazebo reproduction.

The optional
[`tinympc-matlab`](https://github.com/TinyMPC/tinympc-matlab) package is useful
for MATLAB-only experiments but is not in the flight feedback path.

## Quick start

Clone the repository and fetch the pinned firmware:

```bash
git clone https://github.com/TinyMPC/tinympc-px4.git
cd tinympc-px4
./scripts/setup_px4_firmware.sh
./scripts/build_px4_sitl.sh
```

Build TinyMPC from source and run the wrapper smoke test:

```bash
./scripts/setup_tinympc_px4.sh
```

Run all deterministic constrained examples and print CSV metrics:

```bash
./scripts/run_constraint_benchmarks.sh
```

The benchmark asserts finite outputs, solver-projected state/input bounds,
scenario-level closed-loop limits, and zero fallbacks. It reports iterations,
residuals, best-effort solves, and optimized desktop timing percentiles.

Run the experimental supplied-matrix motor/flight-envelope comparison:

```bash
./scripts/run_full_state_benchmark.sh
```

This is a native model-in-the-loop experiment, not a PX4 motor-output build.
Its desktop p50/p95/p99 timing is only a regression measurement; it does not
establish the 20 ms deadline on a flight-controller MCU.

## Generate the PX4 app

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

Valid flight scenarios are `hover`, `virtual_wall`, `corridor`, and
`reduced_authority`. Valid output modes are `guidance` and `direct`.

## Fly in SIH SITL

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
- `quadtest/tinympc/TinyMPC/`: vendored TinyMPC source used by native tests and
  the generated PX4 app.
- `quadtest/setup_tinympc_px4.m`: deterministic model and custom-code setup.
- `quadtest/init_tinympc_quad.m`: matching MATLAB model plus build selection.
- `scripts/`: pinned PX4 build, benchmark, and support-package helpers.
- `docs/`: constraint definitions and research/validation direction.
- `media/`: verified Gazebo hover recording.

Generated Simulink code, native build products, firmware, and virtual
environments are intentionally excluded. A fresh clone rebuilds them from the
checked-in model and source.

## License

This repository is available under the [MIT License](LICENSE). The vendored
TinyMPC snapshot retains its license in
[`quadtest/tinympc/TinyMPC/LICENSE`](quadtest/tinympc/TinyMPC/LICENSE).
