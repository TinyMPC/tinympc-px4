# Pixhawk 6X port

This port compiles the native acceleration-level TinyMPC module into PX4
v1.15.3 for `px4_fmu-v6x_default`. It does not use MATLAB or a companion
computer. TinyMPC runs at 50 Hz, reads PX4's local-position and attitude
estimates, and supplies acceleration plus yaw-rate setpoints. PX4 retains the
attitude/rate loops, control allocation, drivers, arming checks, and failsafes.

> **Status:** the firmware is cross-compiled but has not been run on a Pixhawk
> 6X or flight tested. A successful build is not timing or safety evidence.

## Build

Install the PX4 toolchain, prepare the pinned checkout, then build:

```bash
./scripts/setup_px4_firmware.sh
./scripts/build_pixhawk6x.sh
```

For an existing checkout, set `PX4_DIR=/path/to/PX4-Autopilot`. The flashable
file is
`build/px4_fmu-v6x_default/px4_fmu-v6x_default.px4` inside that checkout.
The script only builds the file; it never uploads it.

The verified v1.15.3 cross-build uses 1,960,676 of the 1,966,080-byte PX4
application flash region (99.73%). This is a valid image but leaves little
flash headroom; future module additions must repeat the size check. Static
AXI-SRAM use reported by the linker was 99,740 of 524,288 bytes (19.02%).

## Deliberately staged hardware modes

The 6X image permits only these commands:

```text
tinympc_chicane start monitor
tinympc_chicane start hold
```

`monitor` is the default and passive mode. It runs the position-hold problem
and records timing/solver diagnostics, but publishes neither an Offboard
heartbeat nor a trajectory setpoint. It therefore cannot take control.

`hold` publishes a zero-displacement acceleration-level hold only after the
vehicle is armed and actually in Offboard mode. It must not be used until the
monitor and props-off stages below pass. The moving chicane and PX4 comparison
modes are compile-time blocked on NuttX for this first hardware image.

Both modes must be started while disarmed. The hardware build accepts only the
PX4 generic quad-X airframe (`SYS_AUTOSTART=4001`) and requires
`MPC_TILTMAX_AIR=15`. A stale estimate,
estimator reset, envelope departure, rejected solve, or solve over the 18 ms
deadline causes the active mode to stop commanding and request PX4 Loiter.

## First-board validation order

1. Upload the `.px4` file through QGroundControl with the flight battery and
   propellers removed. Confirm the normal PX4 sensors and parameters first.
2. From the MAVLink shell, run `tinympc_chicane start monitor`. Keep the
   vehicle disarmed and provide a valid local-position estimate; otherwise the
   stale-state gate intentionally leaves the solve count at zero. Check
   `tinympc_chicane status`, `top`, and `free` for at least several minutes.
3. Verify zero deadline misses, accepted policies, bounded residuals, adequate
   free heap, and stack headroom. Stop with `tinympc_chicane stop` and repeat
   across reboots and estimator resets.
4. Only after those measurements pass, conduct a restrained, props-off
   Offboard test of `hold`, including deliberate Offboard loss and estimator
   reset tests.
5. Flight testing requires a controlled test area, a pilot-controlled mode
   switch, conservative geofence/altitude limits, and incremental tethered
   hover tests before any moving trajectory is enabled.

The embedded solver uses single precision, a maximum of 50 ADMM iterations,
and a generated Riccati cache for the exact checked-in model and weights. The
native check regenerates that cache and rejects any mismatch. The controller
allocates its Eigen workspaces during disarmed module startup; after setup, the
repeated solve path is checked for zero Eigen heap allocations by
`./scripts/check_no_runtime_allocation.sh`. Actual 6X solve time, free heap,
and stack headroom still must be measured on the board.

This first image contains the 12-state acceleration-level hold controller. It
does not contain the SITL-only full-state/motor controller or the moving
chicane/SOC solver. PX4 still enforces its configured tilt and inner-loop
limits on the acceleration setpoint.

## Diagnostics

`tinympc_chicane status` reports solve count, latest/worst solve time,
termination policy, iteration count, primal/dual residuals, and deadline
misses. The same data is published in `debug_array` with name `tmch`:

| Index | Value |
| ---: | --- |
| 0 | mode |
| 1 | elapsed course time |
| 2–3 | relative x/y |
| 4–5 | reference x/y |
| 6–7 | current/maximum corridor violation |
| 8 | solve time in microseconds |
| 9 | deadline-miss count |
| 10–17 | policy, iterations, residuals, constraint violations, fallbacks, solves |

The prediction horizon remains 25 state knots and 24 control intervals at
20 ms, or 0.48 s from the first to final state knot.
