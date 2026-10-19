# Flashing and bench-testing handoff

This page is for the team that will load TinyMPC onto a Holybro X500 V2 with
a Pixhawk 6X and run the first hardware stages. It covers which file to flash, the parameters
to set, the commands to run, and what correct output looks like. Background
and the full staged-validation rationale are in
[the Pixhawk 6X port notes](pixhawk6x_port.md).

> **Status:** the image is cross-compiled and has passed the software checks
> listed below. It has not run on a physical Pixhawk. Bench stages
> 1–3 below are the first hardware evidence; do not fly until they pass.

## 1. Get the file

Flash `firmware/px4_fmu-v6x_multicopter.px4` (PX4 board ID 53, Pixhawk 6X).
See [`firmware/`](../firmware/README.md). Verify it before flashing:

```bash
sha256sum -c SHA256SUMS
```

The image is PX4 v1.15.3 with the fixed-wing, VTOL, and airspeed modules
removed (an X500 uses none of them) plus the `tinympc_chicane` module. It
uses 96.4% of flash. QGroundControl checks the board ID and refuses to load
it onto any board other than a Pixhawk 6X.

## 2. Flash

1. Remove the propellers and disconnect the flight battery. Power the board
   from USB only.
2. In QGroundControl open **Vehicle Setup → Firmware**, then unplug and
   replug USB.
3. Choose **PX4 Pro**, tick **Advanced settings**, select
   **Custom firmware file...**, and pick `px4_fmu-v6x_multicopter.px4`.
4. After the reboot, confirm the version reads v1.15.3 and complete the
   normal PX4 setup: airframe, sensor calibration, radio, flight modes.

## 3. Set parameters

| Parameter | Value | Why |
| --- | --- | --- |
| `SYS_AUTOSTART` | `4019` (Holybro X500 V2) | Module refuses any other airframe except `4001` |
| `MPC_TILTMAX_AIR` | `15` | Module refuses to start otherwise |

Select the airframe in **Vehicle Setup → Airframe → Quadrotor x → Holybro
X500 V2** rather than typing `SYS_AUTOSTART`, so PX4 applies the X500 rate
gains and motor mapping. Keep a radio-controlled mode switch configured so
the pilot can leave Offboard at any time.

## 4. Bench stages

Run these from **Analyze Tools → MAVLink Console** in QGroundControl. The
module is not started automatically; it only runs when commanded.

### Stage 1: monitor (props off, disarmed)

`monitor` solves the hold problem at 50 Hz and records timing but publishes
no Offboard heartbeat or setpoint. It cannot move the vehicle.

The module only solves with a valid local-position estimate. Indoors without
GPS or another position source, the solve count intentionally stays at zero.
Wait until the estimator has settled (position no longer jumping) before
starting, because monitor latches its hold origin at the first valid sample.

```text
tinympc_chicane start monitor
tinympc_chicane status
top
free
```

Healthy output after a few minutes looks like this (SITL example; solve
times there read 0 because simulated time does not advance during a solve):

```text
running ready=1 engaged=0 failed=0 mode=tinympc_monitor reason=none
time=0.00 violation=0.000 max=0.000 solves=1485 solve_us=0 max_us=0
policy=1 iter=6 residual=[0 0.000715] deadline_misses=0
```

Record and report:

| Field | Pass |
| --- | --- |
| `solves` | Rising by about 50 per second |
| `max_us` | Well under 18000 (the solve deadline); this is the key hardware number. Emulation predicts about 5–10 ms worst case |
| `deadline_misses` | `0` |
| `failed` | `0` |
| `policy` | `1` (converged) most of the time; occasional `2` (best effort, hit the 15-iteration cap) is acceptable |
| `free` | Report the free-heap figure |
| `top` | Report `tinympc_chicane` CPU load and stack headroom |

A sustained `policy=2` with a large first residual on a stationary board
means the position estimate has drifted more than 0.5 m horizontally or
0.4 m vertically from where monitor started. Run `tinympc_chicane stop`,
then start monitor again. Any `failed=1`, a `reject policy=` error, or a
deadline miss is a stop condition; send the log.

Stop with `tinympc_chicane stop`. Repeat across several reboots.

### Stage 2: hold, props off and restrained

Only after stage 1 passes. `hold` must be started **while disarmed**; it
refuses to start once armed.

```text
tinympc_chicane start hold
```

Then arm, and switch to Offboard only once the vehicle is steady (in flight
stages, after the takeoff has finished climbing). The module latches its hold
point at the moment Offboard engages; engaging mid-climb produced an envelope
trip in SITL. Confirm `engaged=1` in `tinympc_chicane status`. Also test, in
order:

- Switch out of Offboard with the radio. PX4 must take over immediately.
- `tinympc_chicane stop` while in Offboard. The module must hand the vehicle
  to PX4 Loiter.
- Power-cycle a GPS or force an estimator reset. The module must log
  `fail-closed to autonomous Loiter` and stop commanding.

### Stage 3 onward

Tethered hover in a controlled area with a pilot holding the mode switch,
conservative geofence and altitude limits, then free hover. The moving
chicane modes are deliberately compiled out of this image.

## Fail-closed behaviour

In `hold`, the module stops commanding and requests PX4 Loiter on any of: a
stale estimate, an estimator reset, more than 0.75 m horizontal departure
from the engage point, excessive altitude error or horizontal speed, a
rejected solve, or a solve over the 18 ms deadline.

An envelope trip logs `fail-closed to autonomous Loiter: state_envelope` and
is the intended safe outcome, not a crash. In SITL the hold wandered by about
±0.4 m horizontally, comparable to PX4's own position hold in the same
simulator, so the 0.75 m envelope leaves limited margin with a noisy
position estimate. Report how often it trips.

## Software checks already passed

| Check | Result |
| --- | --- |
| Native solver benchmarks and wrapper smoke test | Pass |
| No heap allocation in the repeated solve path | Pass |
| 6X cross-build, image fits flash, board ID correct | Pass |
| Airframe (`4001`, `4019`) and tilt-limit gates compiled into the image | Pass |
| SITL: wrong airframe and wrong `MPC_TILTMAX_AIR` are refused | Pass |
| SITL: monitor publishes no Offboard messages | Pass |
| SITL: hold refuses to start while armed | Pass |
| SITL: hold engaged in Offboard for 20–30 s, 50 Hz, all solves accepted (4 runs) | Pass |
| SITL: stopping the module in Offboard hands over to Loiter | Pass |
| SITL: envelope departure fails closed to Loiter | Pass |
| Cortex-M7 emulation of the 6X solver code: worst case 2.41M instructions (about 5–10 ms), 5.2 KB stack, 32 KB heap | Pass |
| Fresh clone of this branch: setup, all builds, and all checks above | Pass |
| Fresh 6X build matches the committed image (identical symbol sizes) | Pass |

SITL ran PX4's built-in SIH quadrotor, not an X500 model, and cannot measure
Pixhawk solve time, heap, or stack. The emulation estimate is in
[the port notes](pixhawk6x_port.md#cortex-m7-cost-estimate); stage 1
measures the real numbers.

## Rebuilding the image

On Ubuntu 22.04 or 24.04:

```bash
./scripts/setup_px4_firmware.sh
sudo apt install -y python3-venv automake binutils-dev bison build-essential flex \
  g++-multilib gcc-multilib gdb-multiarch genromfs gettext gperf libelf-dev \
  libexpat-dev libgmp-dev libisl-dev libmpc-dev libmpfr-dev libncurses-dev \
  libtool pkg-config screen texinfo u-boot-tools util-linux vim-common
mkdir -p ~/opt
wget -O /tmp/gcc-arm.tar.bz2 https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/9-2020q2/gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2
tar -jxf /tmp/gcc-arm.tar.bz2 -C ~/opt/
./scripts/build_pixhawk6x.sh
```

PX4's own `Tools/setup/ubuntu.sh` fails on Ubuntu 24.04 (system-wide `pip`
and a missing `libncurses5`), which is why the packages are listed directly.
The build scripts find the ARM compiler in `~/opt` or `/opt` automatically.
