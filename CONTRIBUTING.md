# Contributing to TinyMPC-PX4

Thanks for helping improve the integration. This repository is an experimental
robotics research project, so reproducibility and clear control ownership are
part of every contribution.

## Start with the supported research path

The recommended integration is the native acceleration-level controller:

```text
PX4 EKF -> TinyMPC -> acceleration/yaw-rate -> PX4 inner control -> motors
```

The Simulink-generated integrations are retained for legacy reproduction. The
full-state torque/thrust path is experimental and SITL-only. Changes should not
silently move functionality between those boundaries.

## Local checks

Run the same checks used by continuous integration:

```bash
./scripts/setup_tinympc_px4.sh
./scripts/run_constraint_benchmarks.sh
./scripts/run_full_state_benchmark.sh
```

Changes to `px4_external/`, PX4-facing messages, build scripts, or the vendored
solver must also pass a fresh SITL build:

```bash
./scripts/setup_px4_firmware.sh
./scripts/build_px4_sitl.sh
```

Keep generated build products, PX4 checkouts, ULogs, and local environments out
of commits. `git diff --check` must pass.

## Experiment contributions

An experiment should state:

- model, frame, units, sample time, and prediction horizon;
- reference information available to every compared controller;
- state, input, cone, and downstream PX4 limits;
- solver acceptance, fallback, and mode-transition behavior;
- simulator, airframe, estimator, PX4 version, and disturbance conditions;
- quantitative success criteria derived from logs, not only video; and
- whether timing came from a host, SITL, or an actual flight-controller target.

Matched comparisons must keep the vehicle, estimator, inner loops, allocation,
reference, and physical limits common unless the difference is the subject of
the experiment. Include tuned baselines when making performance claims.

## Safety and hardware claims

Do not enable a POSIX-only module on hardware merely by removing its CMake
guard. Hardware work requires target timing and memory measurements, physical
model validation, estimator-reset tests, deadline handling, conservative
envelopes, and staged tethered testing. Predicted constraints and successful
SITL runs are not evidence of flight safety.

## Pull requests

Keep pull requests focused. Describe the control boundary affected, tests run,
new dependencies, and any remaining limitations. Update documentation and
machine-readable assertions with behavior changes.
