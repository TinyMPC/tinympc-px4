#!/usr/bin/env python3
"""Summarize named TinyMPC ULog diagnostics. Requires numpy and pyulog."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

MODE_NAMES = {0: "mpc", 1: "pid", 2: "pid_tuned", 3: "hold", 4: "monitor"}
FAILURE_NAMES = {1: "configuration", 2: "stale_state", 3: "estimator_reset",
                 4: "state_envelope", 5: "solver", 6: "deadline"}

def distribution(values) -> dict | None:
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return None
    return {"samples": len(values), "mean": float(np.mean(values)),
            "p50": float(np.percentile(values, 50)), "p95": float(np.percentile(values, 95)),
            "p99": float(np.percentile(values, 99)), "max": float(np.max(values))}


def text_field(data: dict, name: str, index: int) -> str:
    values = [int(data[f"{name}[{i}]"][index]) for i in range(64) if f"{name}[{i}]" in data]
    return bytes(v & 255 for v in values).split(b"\0", 1)[0].decode("ascii", errors="replace")


def available_min(values) -> int | None:
    values = np.asarray(values)
    values = values[values != np.iinfo(np.uint32).max]
    return int(np.min(values)) if len(values) else None


def summarize(data: dict) -> list[dict]:
    """Keep module restarts separate and expose missing samples instead of hiding them."""
    if len(data["timestamp"]) == 0:
        return []
    if np.any(np.asarray(data["schema_version"]) != 1):
        raise ValueError("Unsupported tinympc_status schema (expected 1)")
    if "run_id" in data:
        boundaries = np.flatnonzero(np.diff(np.asarray(data["run_id"], dtype=np.int64)) != 0) + 1
    else:
        boundaries = np.flatnonzero(np.diff(np.asarray(data["sequence"], dtype=np.int64)) <= 0) + 1
    runs = []
    for indices in np.split(np.arange(len(data["timestamp"])), boundaries):
        # A final stopped sample is useful for counters but is not another solve/cycle.
        d = {name: np.asarray(values)[indices] for name, values in data.items()}
        running = d["running"].astype(bool)
        solved = d["solve_called"].astype(bool) & running
        tracking = d["tracking_valid"].astype(bool) & running
        allocated = d["allocator_valid"].astype(bool) & d["engaged"].astype(bool) & running
        total = int(d["solve_count"][-1])
        counters = {name: int(d[name][-1]) for name in (
            "solve_count", "converged_count", "best_effort_count", "fallback_count",
            "rejected_count", "iteration_limit_count", "solve_deadline_misses", "loop_overruns",
            "skipped_releases", "stale_cycles", "estimator_reset_events")}
        errors = np.asarray(d["position_error_norm"][tracking], dtype=float)
        errors = errors[np.isfinite(errors)]
        # Count observed coverage only. Never bridge a dropout or a session reset.
        timestamps = d["timestamp"].astype(np.int64)
        dt = np.zeros(len(indices))
        if len(indices) > 1:
            dt[:-1] = np.clip(np.diff(timestamps) / 1e6, 0, 0.02)
            dt[:-1][np.diff(d["session"]) != 0] = 0
        hold = tracking & np.isfinite(d["hold_position_margin"])
        course = tracking & np.isin(d["mode"], [0, 1, 2])
        outside = (hold & (d["hold_position_margin"] < 0)) | (course & (d["corridor_violation"] > 0))
        source_id = text_field(d, "source_id", -1)
        schema = int(d["schema_version"][-1])
        run = {
            "schema_version": schema, "source_id": source_id,
            "source_commit": text_field(d, "source_commit", -1), "mode": int(d["mode"][-1]),
            "mode_name": MODE_NAMES.get(int(d["mode"][-1]), "unknown"),
            "recorded_samples": len(indices), "first_sequence": int(d["sequence"][0]),
            "missing_cycle_samples": int(np.maximum(np.diff(d["sequence"].astype(np.int64)) - 1, 0).sum()),
            "recorded_solve_samples": int(solved.sum()),
            "solve_capture_fraction": float(solved.sum() / total) if total else None,
            "counts_at_last_sample": counters,
            "outcome_percent_at_last_sample": {
                key: 100 * counters[key] / total if total else None for key in
                ("converged_count", "best_effort_count", "fallback_count", "rejected_count", "iteration_limit_count")},
            "solve_us": distribution(d["solve_time_us"][solved]),
            "cycle_work_us": distribution(d["cycle_time_us"][running]),
            "start_lateness_us": distribution(d["start_lateness_us"][running]),
            "cycle_interval_us": distribution(d["cycle_interval_us"][running & (d["cycle_interval_us"] > 0)]),
            "iterations": distribution(d["iterations"][solved]),
            "primal_residual": distribution(d["primal_residual"][solved]),
            "dual_residual": distribution(d["dual_residual"][solved]),
            "planned_state_violation_mixed_units": distribution(d["planned_state_violation"][solved]),
            "planned_input_violation_mixed_units": distribution(d["planned_input_violation"][solved]),
            "position_error_m": distribution(errors),
            "position_error_rms_m": float(np.sqrt(np.mean(errors ** 2))) if len(errors) else None,
            "velocity_error_norm_m_s": distribution(np.sqrt(sum(
                d[f"velocity_error[{axis}]"][tracking] ** 2 for axis in range(3)))),
            "minimum_hold_position_margin_m": float(np.min(d["hold_position_margin"][hold])) if hold.any() else None,
            "observed_tracking_coverage_s": float(dt[tracking].sum()),
            "observed_time_outside_position_bounds_s": float(dt[outside].sum()),
            "observed_actuator_saturation_s": float(dt[allocated & d["actuator_saturated"].astype(bool)].sum()),
            "system_cpu_load": distribution(d["system_cpu_load"]),
            "system_ram_usage": distribution(d["system_ram_usage"]),
            "input_age_us": {name: distribution(d[f"input_age_us[{i}]"][
                d[f"input_age_us[{i}]"] != np.iinfo(np.uint32).max])
                for i, name in enumerate(("position", "attitude", "vehicle_status"))},
            "minimum_sampled_heap_free_bytes": available_min(d["heap_min_free_bytes"]),
            "minimum_stack_free_bytes": available_min(d["stack_free_bytes"]),
            "init_time_us": int(d["init_time_us"][-1]),
            "init_heap_delta_bytes": None if int(d["init_heap_delta_bytes"][-1]) == -(2 ** 31) else int(d["init_heap_delta_bytes"][-1]),
            "configuration": {key: (float(d[f"config[{i}]"][-1]) if np.isfinite(d[f"config[{i}]"][-1]) else None)
                              for i, key in enumerate(("horizon_knots", "iteration_cap", "single_precision",
                                                       "sample_time_s", "primal_tolerance", "dual_tolerance"))},
            "failures": [],
        }
        last_reason = 0
        for i, reason in enumerate(d["failure_reason"]):
            if reason and int(reason) != last_reason:
                run["failures"].append({"timestamp_us": int(d["timestamp"][i]), "reason": int(reason),
                                        "reason_name": FAILURE_NAMES.get(int(reason), "unknown"),
                                        "solve_called": bool(solved[i]), "policy": int(d["policy"][i]),
                                        "solve_time_us": int(d["solve_time_us"][i]) if solved[i] else None})
            last_reason = int(reason)
        runs.append(run)
    return runs


def main() -> None:
    from pyulog import ULog

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--output", type=Path, help="Write JSON here as well as printing it")
    args = parser.parse_args()
    log = ULog(str(args.log))
    try:
        data = log.get_dataset("tinympc_status").data
    except (KeyError, IndexError, StopIteration) as error:
        raise SystemExit("No tinympc_status samples: check the image, logger status, SDLOG_MODE=2 and SDLOG_BOOT_BAT=0.") from error
    report = {"log": str(args.log), "ulog_dropouts": len(log.dropouts),
              "ulog_dropout_duration_ms": sum(drop.duration for drop in log.dropouts), "runs": summarize(data)}
    rendered = json.dumps(report, indent=2, allow_nan=False)
    print(rendered)
    if args.output:
        args.output.write_text(rendered + "\n")


if __name__ == "__main__":
    main()
