# Prebuilt firmware

`px4_fmu-v6x_multicopter.px4` is PX4 v1.15.3 for the Pixhawk 6X with the
`tinympc_chicane` module, built with ARM GCC 9-2020-q2 from this branch.

This image includes metrics schema 1: solver and cycle timing, outcomes,
constraint/tracking diagnostics, input freshness, allocation feedback, and
hardware heap/stack and system CPU/RAM samples. The named `tinympc_status`
topic is registered in the default logger, including failure cycles.

[`build_manifest.json`](build_manifest.json) identifies the image by SHA256,
source content identifier and base commit, and records image and static RAM
sizes. Match its `source_id` to `tinympc_chicane status` on the board.
The manifest also includes the compiler/solver build options and the results
of the final disarmed SITL log check, including its intentionally injected failure.

From this `firmware/` directory, run `sha256sum -c SHA256SUMS`, then follow
[the flashing guide](../docs/FLASHING.md). The image is software-tested only
and has not run on a physical Pixhawk.

Enable disarmed USB logging with `SDLOG_MODE=2` and `SDLOG_BOOT_BAT=0`, save
and reboot, and use a working SD card. See
[collection and analysis](../docs/onboard_metrics.md) for the complete procedure.
