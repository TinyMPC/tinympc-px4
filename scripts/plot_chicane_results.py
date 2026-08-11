#!/usr/bin/env python3
"""Create publication figures and metrics from matched PX4 chicane ULogs."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np
from pyulog import ULog


CORRIDOR_RECTS = (
    (-0.30, 1.68, -0.18, 0.18),
    (1.32, 1.68, -0.18, 1.18),
    (1.32, 3.30, 0.82, 1.18),
)
CORRIDOR_POLYGON = np.array([
    [-0.30, -0.18],
    [1.68, -0.18],
    [1.68, 0.82],
    [3.30, 0.82],
    [3.30, 1.18],
    [1.32, 1.18],
    [1.32, 0.18],
    [-0.30, 0.18],
])
REFERENCE_WAYPOINTS = np.array([
    [0.0, 0.0],
    [1.5, 0.0],
    [1.5, 1.0],
    [3.0, 1.0],
])
CORNER_TIMES = (1.60, 2.80)
FINISH_TIME = 4.40
DEFAULT_DURATION = 6.0
OUTSIDE_TOLERANCE_M = 1.0e-6
GOAL_TOLERANCE_M = 0.25

TINY_COLOR = "#0072B2"
PX4_COLOR = "#D55E00"
REFERENCE_COLOR = "#4D4D4D"
CORRIDOR_COLOR = "#DCEFE6"
CORRIDOR_EDGE = "#458B74"


def corridor_violation(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Euclidean distance to the union of the three corridor rectangles."""
    distances = []
    for xmin, xmax, ymin, ymax in CORRIDOR_RECTS:
        dx = np.maximum.reduce((xmin - x, np.zeros_like(x), x - xmax))
        dy = np.maximum.reduce((ymin - y, np.zeros_like(y), y - ymax))
        distances.append(np.hypot(dx, dy))
    return np.min(np.vstack(distances), axis=0)


def reference_at(time_s: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Evaluate the exact piecewise-linear reference used by the chicane test."""
    time_s = np.maximum(np.asarray(time_s, dtype=float), 0.0)
    x_ref = np.empty_like(time_s)
    y_ref = np.empty_like(time_s)

    first = time_s < CORNER_TIMES[0]
    second = (time_s >= CORNER_TIMES[0]) & (time_s < CORNER_TIMES[1])
    third = (time_s >= CORNER_TIMES[1]) & (time_s < FINISH_TIME)
    finished = time_s >= FINISH_TIME

    x_ref[first] = 1.5 * time_s[first] / 1.6
    y_ref[first] = 0.0
    x_ref[second] = 1.5
    y_ref[second] = (time_s[second] - 1.6) / 1.2
    x_ref[third] = 1.5 + 1.5 * (time_s[third] - 2.8) / 1.6
    y_ref[third] = 1.0
    x_ref[finished] = 3.0
    y_ref[finished] = 1.0
    return x_ref, y_ref


def _time_average(values: np.ndarray, time_s: np.ndarray) -> float:
    duration = float(time_s[-1] - time_s[0])
    if duration <= 0.0:
        raise RuntimeError("ULog selection has no measurable duration")
    return float(np.trapezoid(values, time_s) / duration)


def load_run(path: Path, duration_s: float) -> dict:
    """Extract one course run, using Offboard entry as time and position origin."""
    ulog = ULog(str(path))
    status = ulog.get_dataset("vehicle_status").data
    offboard_indices = np.flatnonzero(status["nav_state"] == 14)
    if offboard_indices.size == 0:
        raise RuntimeError(f"{path}: no Offboard interval")
    start_us = int(status["timestamp"][offboard_indices[0]])

    local = ulog.get_dataset("vehicle_local_position").data
    timestamps_us = local["timestamp"].astype(np.int64)
    origin_index = int(np.searchsorted(timestamps_us, start_us))
    if origin_index >= len(timestamps_us):
        raise RuntimeError(f"{path}: no local-position sample after Offboard entry")

    selected = (
        (timestamps_us >= start_us)
        & (timestamps_us <= start_us + int(duration_s * 1.0e6))
    )
    selected_indices = np.flatnonzero(selected)
    if selected_indices.size < 2:
        raise RuntimeError(f"{path}: insufficient local-position samples")

    timestamp_us = timestamps_us[selected]
    time_s = (timestamp_us - start_us) * 1.0e-6
    origin_x = float(local["x"][origin_index])
    origin_y = float(local["y"][origin_index])
    x_m = local["x"][selected].astype(float) - origin_x
    y_m = local["y"][selected].astype(float) - origin_y
    violation_m = corridor_violation(x_m, y_m)
    x_ref_m, y_ref_m = reference_at(time_s)
    tracking_error_m = np.hypot(x_m - x_ref_m, y_m - y_ref_m)

    after_finish = time_s >= FINISH_TIME
    minimum_goal_error_m = float(
        np.min(np.hypot(x_m[after_finish] - 3.0, y_m[after_finish] - 1.0))
    )
    course_completed = minimum_goal_error_m <= GOAL_TOLERANCE_M

    land = ulog.get_dataset("vehicle_land_detected").data
    landed_after_course = bool(
        np.any(
            (land["timestamp"].astype(np.int64) > start_us + int(FINISH_TIME * 1.0e6))
            & land["landed"].astype(bool)
        )
    )
    disarmed_after_course = bool(
        np.any(
            (status["timestamp"].astype(np.int64) > start_us + int(FINISH_TIME * 1.0e6))
            & (status["arming_state"] == 1)
        )
    )

    metrics = {
        "maximum_outside_corridor_m": float(np.max(violation_m)),
        "mean_outside_corridor_m": _time_average(violation_m, time_s),
        "fraction_of_trajectory_outside": _time_average(
            (violation_m > OUTSIDE_TOLERANCE_M).astype(float), time_s
        ),
        "rms_tracking_error_m": float(
            np.sqrt(_time_average(tracking_error_m**2, time_s))
        ),
        "final_tracking_error_m": float(tracking_error_m[-1]),
        "minimum_goal_error_after_finish_m": minimum_goal_error_m,
        "completion_status": (
            "completed_and_landed_normally"
            if course_completed and landed_after_course and disarmed_after_course
            else "incomplete"
        ),
    }

    return {
        "path": path,
        "timestamp_us": timestamp_us,
        "time_s": time_s,
        "x_m": x_m,
        "y_m": y_m,
        "x_ref_m": x_ref_m,
        "y_ref_m": y_ref_m,
        "violation_m": violation_m,
        "tracking_error_m": tracking_error_m,
        "origin_x_m": origin_x,
        "origin_y_m": origin_y,
        "metrics": metrics,
    }


def add_direction_arrow(axis, run: dict, color: str, time_s: float) -> None:
    time = run["time_s"]
    start_index = int(np.searchsorted(time, time_s))
    end_index = int(np.searchsorted(time, time_s + 0.18))
    start_index = min(start_index, len(time) - 2)
    end_index = max(start_index + 1, min(end_index, len(time) - 1))
    axis.annotate(
        "",
        xy=(run["x_m"][end_index], run["y_m"][end_index]),
        xytext=(run["x_m"][start_index], run["y_m"][start_index]),
        arrowprops=dict(arrowstyle="-|>", color=color, lw=1.8, mutation_scale=13),
        zorder=7,
    )


def save_figure(fig, output_dir: Path, stem: str) -> None:
    fig.savefig(output_dir / f"{stem}.png", bbox_inches="tight", dpi=300)


def plot_top_down(tiny: dict, px4: dict, output_dir: Path) -> None:
    fig, axis = plt.subplots(figsize=(7.4, 4.2), constrained_layout=True)
    axis.add_patch(
        Polygon(
            CORRIDOR_POLYGON,
            closed=True,
            facecolor=CORRIDOR_COLOR,
            edgecolor=CORRIDOR_EDGE,
            linewidth=1.3,
            label="_nolegend_",
            zorder=0,
        )
    )
    axis.plot(
        REFERENCE_WAYPOINTS[:, 0],
        REFERENCE_WAYPOINTS[:, 1],
        linestyle="--",
        color=REFERENCE_COLOR,
        linewidth=1.6,
        label="_nolegend_",
        zorder=2,
    )
    axis.plot(
        tiny["x_m"], tiny["y_m"], color=TINY_COLOR, linewidth=2.2,
        label="TinyMPC–PX4", zorder=4,
    )
    axis.plot(
        px4["x_m"], px4["y_m"], color=PX4_COLOR, linewidth=2.0,
        label="PX4 cascaded control", zorder=3,
    )
    axis.scatter(
        [0.0], [0.0], marker="o", s=58, facecolor="white", edgecolor="#1B7837",
        linewidth=1.8, zorder=8,
    )
    axis.scatter(
        [3.0], [1.0], marker="*", s=135, facecolor="#F0C419", edgecolor="#6B5500",
        linewidth=1.0, zorder=8,
    )
    axis.annotate(
        "Start", xy=(0.0, 0.0), xytext=(0, -18), textcoords="offset points",
        ha="center", va="top", fontsize=9.5, fontweight="semibold", zorder=9,
    )
    axis.annotate(
        "End", xy=(3.0, 1.0), xytext=(0, -18), textcoords="offset points",
        ha="center", va="top", fontsize=9.5, fontweight="semibold", zorder=9,
    )

    add_direction_arrow(axis, tiny, TINY_COLOR, 3.45)
    add_direction_arrow(axis, px4, PX4_COLOR, 3.45)

    axis.set_xlim(-0.40, 3.40)
    axis.set_ylim(-0.42, 1.42)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xticks([])
    axis.set_yticks([])
    save_figure(fig, output_dir, "chicane_top_down")
    plt.close(fig)


def plot_violation(tiny: dict, px4: dict, output_dir: Path) -> None:
    fig, axis = plt.subplots(figsize=(7.4, 3.75), constrained_layout=True)
    axis.plot(
        tiny["time_s"], tiny["violation_m"], color=TINY_COLOR, linewidth=2.2,
        label="TinyMPC–PX4", zorder=4,
    )
    axis.plot(
        px4["time_s"], px4["violation_m"], color=PX4_COLOR, linewidth=2.0,
        label="PX4 cascaded control", zorder=3,
    )
    axis.fill_between(
        px4["time_s"], 0.0, px4["violation_m"], color=PX4_COLOR, alpha=0.12, zorder=1,
    )
    for corner_time in CORNER_TIMES:
        axis.axvline(corner_time, color="#777777", linestyle=":", linewidth=1.0, zorder=0)

    axis.set_xlim(0.0, DEFAULT_DURATION)
    axis.set_ylim(-0.005, 0.28)
    axis.set_xlabel("Time [s]")
    axis.set_ylabel("Outside-corridor distance [m]")
    axis.grid(True, color="#B8B8B8", linewidth=0.6, alpha=0.45)
    axis.legend(loc="upper right", fontsize=9.0, framealpha=0.96)
    save_figure(fig, output_dir, "chicane_violation_time")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tinympc-log", type=Path, required=True)
    parser.add_argument("--px4-log", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, default=Path("figures"))
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION)
    args = parser.parse_args()

    if not np.isclose(args.duration, DEFAULT_DURATION):
        raise ValueError(
            f"This paper figure is fixed to the {DEFAULT_DURATION:.1f} s evaluation window"
        )

    args.output_dir.mkdir(parents=True, exist_ok=True)
    tiny = load_run(args.tinympc_log, args.duration)
    px4 = load_run(args.px4_log, args.duration)
    plot_top_down(tiny, px4, args.output_dir)
    plot_violation(tiny, px4, args.output_dir)

    print(f"Wrote PNG figures to {args.output_dir}")
    for label, run in (("TinyMPC", tiny), ("Stock PX4", px4)):
        m = run["metrics"]
        print(
            f"{label}: max={m['maximum_outside_corridor_m']:.6f} m, "
            f"mean={m['mean_outside_corridor_m']:.6f} m, "
            f"outside={100.0 * m['fraction_of_trajectory_outside']:.2f}%, "
            f"RMS tracking={m['rms_tracking_error_m']:.6f} m, "
            f"final={m['final_tracking_error_m']:.6f} m, "
            f"status={m['completion_status']}"
        )


if __name__ == "__main__":
    main()
