# Pixhawk 6X port

This port compiles the native acceleration-level TinyMPC module into PX4
v1.15.3 for `px4_fmu-v6x_multicopter`. It does not use MATLAB or a companion
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
file is `build/px4_fmu-v6x_multicopter/px4_fmu-v6x_multicopter.px4` inside
that checkout. The script only builds the file; it never uploads it. Ubuntu
24.04 toolchain steps and the collaborator flashing procedure are in
[FLASHING.md](FLASHING.md).

The image uses PX4's multicopter-only 6X configuration, which drops the
fixed-wing, VTOL, and airspeed modules. The full `px4_fmu-v6x_default`
image no longer fits: with the PX4 v1.15 ARM GCC 9 toolchain it overflows
flash by 12,940 bytes.

| Target | Flash used of 1,966,080 bytes | Static AXI SRAM |
| --- | ---: | ---: |
| `px4_fmu-v6x_multicopter` | 1,894,628 (96.4%) | 99,740 of 524,288 (19.0%) |

Future module additions must repeat the size check.

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
PX4 generic quad-X (`SYS_AUTOSTART=4001`) or Holybro X500 V2
(`SYS_AUTOSTART=4019`) airframe and requires
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

The embedded solver uses single precision, a maximum of 15 ADMM iterations
(50 in SITL and the native benchmarks), and a generated Riccati cache for the exact checked-in model and weights. The
native check regenerates that cache and rejects any mismatch. The controller
allocates its Eigen workspaces during disarmed module startup; after setup, the
repeated solve path is checked for zero Eigen heap allocations by
`./scripts/check_no_runtime_allocation.sh`. Actual 6X solve time, free heap,
and stack headroom still must be measured on the board.

### Cortex-M7 cost estimate

The solver object files from this build, recompiled with identical flags
except a single-precision FPU, were run in a Cortex-M7 instruction emulator.
The emulator cannot execute the 6X's double-precision instructions, so those
became software calls; they accounted for under 0.01% of the count, so the
figures below are an accurate upper bound.

| Hold solve | ADMM iterations | Instructions | At 480 MHz, 1–2 cycles/instruction |
| --- | ---: | ---: | ---: |
| At the hold point | 1 | 0.20 M | 0.4–0.8 ms |
| Small offset | 1–10 | 0.20–1.62 M | 0.4–6.8 ms |
| Box edge or outside the box, 50-iteration cap | 50 | 7.94 M | 16.5–33 ms |
| Same, 15-iteration cap (this image) | 15 | 2.41 M | 5.0–10.1 ms |

Each iteration costs about 158k instructions, almost entirely Eigen
matrix-vector products. A 50-iteration cap could therefore exceed the 18 ms
deadline, and every deadline miss stops the module, so the hardware build
caps ADMM at 15 iterations. Capped iterates remained bound-feasible
(`policy=2`, accepted) in every emulated case. The emulated solve used 5.2 KB
of the 64 KB task stack, and `MPC_Init` requested 32 KB of heap in 59
allocations. Stage 1 must still confirm `max_us` on the board, since cache
behaviour, flash wait states, and preemption by PX4's higher-priority tasks
are not modelled.

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
