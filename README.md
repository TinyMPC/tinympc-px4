# TinyMPC-PX4

TinyMPC-PX4 runs TinyMPC as a 50 Hz guidance controller inside PX4 while
retaining PX4's estimator, position tracking, attitude/rate controllers,
control allocation, and actuator pipeline. The controller is compiled into a
code-generated Simulink PX4 app; no companion computer solves the MPC problem.

> [!CAUTION]
> This is an experimental research prototype, not a flight-certified
> controller. Reproduce the Software-in-the-Loop (SITL) demo before adapting it
> to hardware, and use normal PX4 safety procedures for any physical tests.

The current hover result was verified on July 8, 2026 with PX4 v1.15.3 and
MATLAB/Simulink R2026a. In SIH SITL, the vehicle remained in Offboard mode for
151 seconds and held the requested altitude within approximately 3 cm. A
Gazebo recording is available at
[`media/tinympc_hover_gazebo.mp4`](media/tinympc_hover_gazebo.mp4).

## Control architecture

```text
                                  trajectory_setpoint
PX4 EKF state --> TinyMPC (50 Hz) --------------------> mc_pos_control
                    guidance plan                         |
                                                          v
                                    attitude controller -> rate controller
                                                          |
                                                          v
                                           control allocator -> motors
```

The state is
`[x y z roll pitch yaw vx vy vz p q r]` in PX4's local NED frame. Each solve
returns the first control input
`[ax ay az yawspeed]` and a predicted state about 200 ms ahead. The published
`trajectory_setpoint` combines:

- predicted position and velocity;
- bounded acceleration feed-forward (±4 m/s²);
- predicted yaw and bounded yaw-rate feed-forward (±1 rad/s).

PX4's position controller tracks this moving plan point. A one-step (20 ms)
plan point stays too close to the current state to provide useful tracking
drive, so the verified demo uses horizon column 10. The hover reference is
latched one metre above the local position when the app first runs, avoiding
dependence on the local z-origin while the vehicle waits on the ground. If the
solver produces a non-finite plan, the app republishes its last valid plan.

The current model and weights are a hover integration test, not a tuned X500
controller. This version does not replace PX4's inner-loop stabilization and
does not directly command motors.

## Requirements

- Ubuntu 22.04 (the verified host platform).
- Git, CMake, a C++17 compiler, Python 3, and the PX4 v1.15.3 build toolchain.
- MATLAB/Simulink R2026a with MATLAB Coder, Simulink Coder, Embedded Coder,
  UAV Toolbox, and the UAV Toolbox Support Package for PX4 Autopilots.
- `patch` for the required MathWorks support-package compatibility patch.
- Gazebo Harmonic only if you want to reproduce the recorded Gazebo flight.

The optional
[`tinympc-matlab`](https://github.com/TinyMPC/tinympc-matlab) package is useful
for MATLAB-only experiments but is not part of the flight feedback path. Set
`TINY_MPC_MATLAB_PATH` if you want the setup helper to load it.

## Reproduce the hover demo

### 1. Clone the repository

```bash
git clone https://github.com/TinyMPC/tinympc-px4.git
cd tinympc-px4
```

All commands below start at the repository root.

### 2. Fetch PX4 and build the native smoke test

The firmware helper pins PX4 v1.15.3 under `third_party/`. It does not install
system packages for you.

```bash
./scripts/setup_px4_firmware.sh
```

If the PX4 toolchain is not already installed, follow the command printed by
that script (`third_party/PX4-Autopilot/Tools/setup/ubuntu.sh`) and reboot if
the PX4 installer requests it. Then build the base SITL firmware and verify the
vendored TinyMPC controller independently of MATLAB:

```bash
./scripts/build_px4_sitl.sh
./scripts/setup_tinympc_px4.sh
```

The second command builds TinyMPC from source and runs a C++ smoke test that
checks finite outputs, control bounds, and upward NED motion for a one-metre
hover reference. If MATLAB is available, it also checks the Simulink and PX4
support-package environment.

To use a separate PX4 checkout or virtual environment, set `PX4_DIR` or
`PX4_VENV_DIR`. To select another firmware ref for development, set
`PX4_VERSION`; the documented and verified ref remains v1.15.3.

### 3. Patch the MATLAB PX4 support package

The R2026a generated uORB writer leaves message timestamps at zero and does not
publish the `offboard_control_mode` heartbeat required by PX4. Apply the
repository's narrow compatibility patch before generating the app:

```bash
./scripts/patch_matlab_support_package.sh
```

The helper is idempotent and defaults to
`~/Documents/MATLAB/SupportPackages/R2026a`. If your support packages are
elsewhere, point it at the release directory:

```bash
MATLAB_SUPPORT_ROOT=/path/to/SupportPackages/R2026a \
  ./scripts/patch_matlab_support_package.sh
```

If the patch does not apply cleanly, restore an unmodified R2026a PX4 support
package before continuing. Other MATLAB releases may contain different source
and have not been validated with this patch.

### 4. Generate and build the PX4 app

The setup function configures MATLAB's firmware path and PX4 target preferences
from the repository layout. First update the diagram, generate the app sources,
and then compile the generated module into the SITL firmware:

```bash
matlab -batch "cd('quadtest'); run_tinympc_px4_demo('update')"
matlab -batch "cd('quadtest'); run_tinympc_px4_demo('build')"
./scripts/build_px4_sitl.sh
```

`quadtest/setup_tinympc_px4.m` also normalizes the model's custom-code settings
so the generated PX4 app compiles the vendored TinyMPC sources directly.
`run_tinympc_px4_demo('build')` generates and installs the module sources into
the pinned PX4 checkout; the final SITL helper invocation produces the updated
`bin/px4`. That helper also removes any obsolete `libtinympcstatic.a` link line
left by an older generated app.

### 5. Fly in SIH SITL

Start the generated binary directly from the PX4 root filesystem:

```bash
cd third_party/PX4-Autopilot/build/px4_sitl_default/rootfs
PX4_SYS_AUTOSTART=10040 ../bin/px4 ../../px4_sitl_default/etc \
  -s etc/init.d-posix/rcS
```

Wait for `Ready for takeoff!`; EKF convergence can take up to about two
minutes. `commander check` must report that arming is allowed. In the PX4 shell,
run:

```text
px4_simulink_app start
commander mode offboard
commander arm
# The vehicle climbs 1 m above its engagement point and hovers.
commander land
```

Confirm that the app is actually flying the vehicle:

```text
listener trajectory_setpoint
listener vehicle_status
```

`trajectory_setpoint` should show moving plan points with fresh timestamps,
and `vehicle_status.nav_state` should be `14` (Offboard). PX4 writes flight logs
under `build/px4_sitl_default/rootfs/log/`.

## Gazebo reproduction

Install Gazebo Harmonic from the OSRF packages, rebuild SITL once so the Gazebo
bridge is available, and start the X500 model:

```bash
./scripts/build_px4_sitl.sh

cd third_party/PX4-Autopilot/build/px4_sitl_default/rootfs
PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 HEADLESS=1 \
  ../bin/px4 ../../px4_sitl_default/etc -s etc/init.d-posix/rcS
```

Attach a GUI with `gz sim -g` if desired, then use the same PX4 shell commands
as the SIH demo. If arming reports a yaw-estimate error, wait for EKF
convergence and ensure PX4 is paired with a freshly started Gazebo server. If
parameters left by an earlier run cause preflight failures, stop PX4, remove
only `rootfs/parameters*.bson`, and restart it.

## Repository layout

- `quadtest/quadtest.slx`: Simulink model containing the uORB readers,
  TinyMPC controller, and trajectory-setpoint writer.
- `quadtest/setup_tinympc_px4.m`: environment checks and deterministic model
  configuration used before every update or build.
- `quadtest/init_tinympc_quad.m`: matching MATLAB-side dynamics, horizon, and
  weight definitions.
- `quadtest/wrapper/`: the C++ TinyMPC/PX4 interface and native smoke test.
- `quadtest/tinympc/TinyMPC/`: the vendored TinyMPC snapshot used by both the
  native test and generated PX4 app.
- `scripts/`: pinned PX4 setup/build helpers and the R2026a support-package
  patch helper.
- `media/`: the verified Gazebo hover recording.

Generated Simulink code, native build products, prebuilt host libraries, PX4
firmware, and virtual environments are intentionally excluded from version
control. A fresh clone rebuilds them from the checked-in model and sources.

## License

This repository is available under the [MIT License](LICENSE). The vendored
TinyMPC source retains its own license in
`quadtest/tinympc/TinyMPC/LICENSE`.
