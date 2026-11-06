# Onboard metrics and log collection

The staged Pixhawk 6X image contains metrics schema 1. Run
`tinympc_chicane status` to see its schema, source content identifier, base
commit, solver statistics and hardware resource samples. Match `source=` to
`firmware/build_manifest.json` before collecting evidence. The content
identifier covers the module, solver, model headers, message and module CMake
files, including local edits; the base commit alone does not identify those edits.

## Record the disarmed bench run

Use the bench procedure in [the flashing guide](FLASHING.md). Insert a working
SD card. In the MAVLink Console, set these parameters and reboot:

```text
param set SDLOG_MODE 2
param set SDLOG_BOOT_BAT 0
param save
reboot
```

These settings record from boot until shutdown, including USB-powered,
disarmed monitor runs. They are needed on an existing board because flashing
does not necessarily reset its saved parameters. The firmware registers
`tinympc_status` at every publication and `control_allocator_status` at 50 Hz
in the logger, including when an SD-card `logger_topics.txt` is present.
An SD-card file still controls all its other topics. No debug-profile bit is
required for the named metrics topic; the legacy `debug_array` remains available.

```text
logger status
tinympc_chicane start monitor
tinympc_chicane status
listener tinympc_status 1
```

Confirm `logger status` shows an active full file log, not just a running
logger task. Let the run collect several minutes of samples, then run:

```text
tinympc_chicane status
tinympc_chicane stop
logger status
logger stop
```

Download the complete `.ulg` through QGroundControl's log download or copy it
from the SD card. Let the logger finish writing before removing power. Return
the `.ulg`, status output, `top`, `free`, parameters, and the firmware manifest
with the test conditions. Restore the team's usual logging mode after the
bench campaign if continuous recording is no longer wanted.

## Analyze

Use a Python environment with `numpy` and `pyulog` (the repo's PX4 virtual
environment includes these when installed). From the repository root:

```bash
python scripts/summarize_tinympc_log.py flight.ulg --output metrics.json
```

The JSON contains separate module runs, p50/p95/p99/worst timing, iteration
and residual distributions, solver outcome rates, tracking errors, observed
time outside position bounds, allocation saturation, resource minima, failure
events and log coverage. A monitor run measures drift about its latched origin;
it does not establish flight tracking performance. Module restarts reset the
lifetime counters; `run_id` identifies each module instance, including failed
initialization. `session` increments when the solver/origin is reset within
a module run, including an Offboard re-engagement.

## Fields and interpretation

The authoritative field definitions and units are in
[`TinympcStatus.msg`](../px4_external/msg/TinympcStatus.msg).

| Group | Meaning |
| --- | --- |
| `solve_time_us`, `solve_max_us` | Wall time around the complete MPC wrapper call, including reference and constraint preparation. A solve over 18,000 us increments `solve_deadline_misses` and retains the existing failure behavior. |
| `cycle_time_us`, `cycle_max_us` | Controller work, legacy debug publication and resource sampling; excludes sleep and the final named-status publication. `loop_overruns` counts this work exceeding 20,000 us. |
| `start_lateness_us`, `cycle_interval_us`, `skipped_releases` | Release lateness, actual start spacing and releases skipped by the scheduler's existing resynchronization. These expose delays outside the timed solver call. |
| `solve_called`, `command_accepted` | Distinguish actual solves and commands accepted by integration checks from idle, stale-input or failed cycles. In monitor, accepted commands are evaluated but not published. |
| Outcome counters | Converged, best-effort and fallback describe solver policy; rejected describes the integration result and can overlap, for example a converged but late solve. An iteration-cap hit is not proof of infeasibility. |
| Residuals and planned violations | Primal/dual residuals and planned state/input bound violations. These mix units; do not interpret a state residual as metres. |
| `position_error`, `velocity_error` | Measured NED position/velocity minus the current reference. Only use when `tracking_valid`. |
| `hold_position_margin`, `hold_velocity_margin` | Signed minimum margin to the MPC hold box in metres and m/s. Negative means outside; these are distinct from the larger module failure envelope. Only hold/monitor samples have these margins. |
| `input_age_us[0..2]` | Position, attitude and vehicle-status message age. Missing messages use `UINT32_MAX`; future timestamps show zero age, and the existing freshness checks still reject excessive future timestamps. |
| `allocator_valid`, saturation/achievement flags | Fresh allocation feedback. Analyze saturation while engaged; disarmed motors do not establish control saturation. |
| CPU/RAM | System load from PX4 `cpuload`; `cpuload_timestamp` identifies the measurement time. Use `top` for individual task CPU usage. |
| Heap and stack | System free heap and this task's stack high-water headroom, sampled at 1 Hz on NuttX. Minimum heap is the minimum sampled free value, not a record of every allocation. `resource_timestamp` identifies the measurement time. |
| Initialization | Wrapper setup wall time and system-wide net heap change across setup. Other tasks can affect the heap delta. Static RAM and firmware size are in the build manifest. |
| `config[0..5]` | Horizon knots, iteration cap, single-precision flag, sample interval seconds, primal tolerance, dual tolerance. The summarizer expands these names. |

Unavailable floating values are NaN, unavailable unsigned resource sizes are
`UINT32_MAX` (4294967295), and an unavailable initialization heap delta is
`INT32_MIN`. In particular, SITL does not provide the NuttX heap/stack samples.
SITL solver/work timing uses the host monotonic clock; scheduling and message
timestamps use PX4 simulation time. Neither is Pixhawk timing evidence.

The topic is published after every cycle, including a failed solve or stale
state, and once when the task stops or initialization fails. A stopped sample
is not another solve. Check sequence gaps, solver capture fraction and ULog
dropouts before interpreting percentiles; the final counters include solves
that may have occurred before logging began. Time-outside and saturation
summaries count observed intervals capped at one 20 ms period, without filling
log gaps or crossing origin resets. Recovery time requires a defined
disturbance and settling threshold and is not inferred automatically.
