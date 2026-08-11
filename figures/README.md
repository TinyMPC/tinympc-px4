# Chicane comparison figures

These figures use the completed, matched, no-wind PX4 SITL runs. Both runs use
the X500 model, the same chicane reference, and the same 15-degree tilt limit.

- `chicane_top_down.{pdf,svg,png}` shows the exact feasible-corridor union,
  common reference centerline, measured trajectories, direction arrows, and
  maximum measured violation.
- `chicane_violation_time.{pdf,svg,png}` shows Euclidean distance to the
  corridor union over the six-second evaluation window.
- `tinympc_chicane_timeseries.csv` and `stock_px4_chicane_timeseries.csv`
  contain the native-rate ULog timestamps, relative `x/y`, exact reference, and
  outside-corridor distance used for the plots.
- `chicane_metrics.json` records the corridor bounds, reference timing,
  evaluation definitions, run metrics, and available solver-status summary.

The source logs are `20_54_16.ulg` (TinyMPC) and `19_37_19.ulg` (stock PX4).
Regenerate the artifacts with a Python environment containing `numpy`,
`matplotlib`, and `pyulog`:

```bash
python scripts/plot_chicane_results.py \
  --tinympc-log /path/to/20_54_16.ulg \
  --px4-log /path/to/19_37_19.ulg \
  --output-dir figures
```

The TinyMPC ULog does not contain a solver/debug topic. The 1,095 solve count,
1.068 ms worst host solve time, and zero module failures came from the live
module-status summary captured for this run. Mean, median, p95, residual,
solver-failure, and fallback values are therefore left null rather than
inferred. This host result is not Pixhawk timing evidence.
