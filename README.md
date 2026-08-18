# TinyMPC-PX4

TinyMPC-PX4 runs a constrained model-predictive controller inside PX4 at
50 Hz. TinyMPC plans acceleration and yaw-rate commands; PX4 keeps ownership
of state estimation, attitude/rate control, allocation, arming, and failsafes.
No companion computer solves the MPC problem.

> [!CAUTION]
> **Software validation only.** The PX4 integration has been tested in native
> benchmarks and PX4 SITL/Gazebo, not on flight hardware. Hardware validation
> on Pixhawk-class targets is ongoing work.

```text
PX4 EKF -> TinyMPC -> acceleration/yaw rate -> PX4 inner loops -> motors
```

The recommended implementation is native C++ and does not require MATLAB.

## Chicane comparison

The main demonstration flies an X500 through a narrow, two-turn corridor with
no wind or external disturbance. TinyMPC and the tuned PX4 baseline use the
same reference, estimator, 15-degree tilt limit, inner loops, allocation, and
vehicle.

| Controller | Maximum outside corridor | RMS tracking error | Completion |
| --- | ---: | ---: | --- |
| TinyMPC -> PX4 | 0.000 m | 0.113 m | Normal landing |
| Tuned PX4 cascaded control | 0.000 m | 0.250 m | Normal landing |

TinyMPC uses 25 state knots and 24 control intervals at 20 ms, giving 0.48 s
of preview. The recorded run completed 1,095 solver calls without a module
failure; the worst host solve was 1.068 ms. Host timing is not Pixhawk timing
evidence.

[Watch the matched PX4/Gazebo telemetry replay](media/tinympc_chicane_px4_sitl_comparison.mp4)
or read the [experiment and reproduction details](docs/chicane_px4_comparison.md).

## Quick start

Build TinyMPC and run the native regression benchmarks:

```bash
git clone https://github.com/TinyMPC/tinympc-px4.git
cd tinympc-px4
./scripts/setup_tinympc_px4.sh
./scripts/run_constraint_benchmarks.sh
./scripts/run_full_state_benchmark.sh
```

Build the native external modules against the pinned PX4 release:

```bash
./scripts/setup_px4_firmware.sh
./scripts/build_px4_sitl.sh
```

The two matched chicane modes are:

```text
tinympc_chicane start mpc
tinympc_chicane start pid_tuned
```

The tuned PX4 mode verifies the documented controller gains before starting.
Follow the complete setup and flight sequence in
[`docs/chicane_px4_comparison.md`](docs/chicane_px4_comparison.md).

## Included work

- Native acceleration-level TinyMPC/PX4 integration and a tuned PX4 baseline.
- Box-constrained hover, virtual-wall, corridor, and reduced-authority cases.
- Coupled tilt/thrust-cone figure-eight and chicane examples.
- Deterministic benchmarks with solver residual, iteration, fallback, and
  timing diagnostics.
- An experimental SITL-only full-state controller that jointly models
  position, attitude, body rate, motor authority, and motor slew.
- Legacy MATLAB/Simulink integration retained for reproducibility but not
  required by the recommended path.

Recorded demonstrations:
[hover](media/tinympc_hover_gazebo.mp4),
[virtual wall](media/tinympc_virtual_wall_gazebo.mp4),
[corridor](media/tinympc_corridor_gazebo.mp4),
[reduced authority](media/tinympc_reduced_authority_gazebo.mp4), and
[SOC figure-eight](media/tinympc_figure_eight_soc_gazebo.mp4).

## Documentation

- [Constraint definitions and solver policy](docs/constraint_examples.md)
- [Matched TinyMPC versus tuned PX4 experiment](docs/chicane_px4_comparison.md)
- [Full-state actuator-envelope experiment](docs/full_state_actuator_constraints.md)
- [Research direction and remaining validation](docs/research_direction.md)
- [Contributing](CONTRIBUTING.md)

## Requirements

- Ubuntu 22.04, CMake, Ninja, and a C++17 compiler for native tests
- PX4 v1.15.3 toolchain for SITL
- Gazebo Harmonic for flight reproduction
- Python, `pyulog`, NumPy, Matplotlib, and FFmpeg for telemetry rendering

MATLAB/Simulink R2026a is optional and only needed to reproduce the legacy
generated application.

## Safety and scope

This is an experimental research controller, not flight-certified software.
Successful SITL runs and predicted constraints do not establish real-world
safety. PX4 hardware validation is ongoing work and still requires target
timing and memory evidence, airframe/model validation, estimator-reset testing,
robustness margins, and staged physical testing. The full-state path
deliberately remains POSIX/SITL only.

## License and citation

Released under the [MIT License](LICENSE). The vendored TinyMPC source retains
its own [license](quadtest/tinympc/TinyMPC/LICENSE). Citation metadata is in
[`CITATION.cff`](CITATION.cff).
