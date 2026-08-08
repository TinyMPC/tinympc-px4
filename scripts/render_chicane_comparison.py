#!/usr/bin/env python3
"""Render a quantitative replay of the matched PX4 SITL chicane runs."""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter, FuncAnimation
from matplotlib.patches import Polygon
import numpy as np
from pyulog import ULog


CORRIDOR_RECTS = (
    (-0.30, 1.68, -0.18, 0.18),
    (1.32, 1.68, -0.18, 1.18),
    (1.32, 3.30, 0.82, 1.18),
)
CORRIDOR_POLYGON = np.array([
    [-0.30, -0.18], [1.68, -0.18], [1.68, 0.82], [3.30, 0.82],
    [3.30, 1.18], [1.32, 1.18], [1.32, 0.18], [-0.30, 0.18],
])
REFERENCE = np.array([[0.0, 0.0], [1.5, 0.0], [1.5, 1.0], [3.0, 1.0]])


def corridor_violation(x, y):
    distances = []
    for xmin, xmax, ymin, ymax in CORRIDOR_RECTS:
        dx = np.maximum.reduce((xmin - x, np.zeros_like(x), x - xmax))
        dy = np.maximum.reduce((ymin - y, np.zeros_like(y), y - ymax))
        distances.append(np.hypot(dx, dy))
    return np.min(np.vstack(distances), axis=0)


def load_offboard_path(path, duration=6.0):
    ulog = ULog(str(path))
    status = ulog.get_dataset("vehicle_status").data
    offboard = np.flatnonzero(status["nav_state"] == 14)
    if offboard.size == 0:
        raise RuntimeError(f"{path}: no Offboard interval")
    start = status["timestamp"][offboard[0]]

    local = ulog.get_dataset("vehicle_local_position").data
    timestamps = local["timestamp"]
    origin_index = int(np.searchsorted(timestamps, start))
    selected = (timestamps >= start) & (timestamps <= start + duration * 1.0e6)
    time = (timestamps[selected] - start) * 1.0e-6
    x = local["x"][selected] - local["x"][origin_index]
    y = local["y"][selected] - local["y"][origin_index]
    violation = corridor_violation(x, y)
    return time, x, y, violation


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mpc-log", type=Path, required=True)
    parser.add_argument("--pid-log", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    paths = [load_offboard_path(args.mpc_log), load_offboard_path(args.pid_log)]
    labels = ["TinyMPC → PX4 inner loops", "Stock PX4 cascaded position/velocity"]
    colors = ["#35e07a", "#ff5252"]
    results = [float(path[3].max()) for path in paths]

    plt.style.use("dark_background")
    fig, axes = plt.subplots(1, 2, figsize=(16, 6), dpi=120)
    fig.patch.set_facecolor("#11151b")
    fig.suptitle(
        "No-wind chicane • identical X500 • identical 15° tilt limit • actual PX4 SITL logs",
        fontsize=17, fontweight="bold", color="white")

    trails = []
    vehicles = []
    metrics = []
    for axis, label, color in zip(axes, labels, colors):
        axis.set_facecolor("#b9bcc0")
        axis.add_patch(Polygon(CORRIDOR_POLYGON, closed=True,
                               facecolor="#176c8d", edgecolor="#ff6b20",
                               linewidth=3.0, zorder=1))
        axis.plot(REFERENCE[:, 0], REFERENCE[:, 1], "--", color="#ffe65a",
                  linewidth=1.8, alpha=0.9, label="common reference", zorder=2)
        axis.scatter([1.5, 1.5], [0.0, 1.0], marker="D", s=80,
                     facecolor="#ffe65a", edgecolor="#263238", zorder=3)
        trail, = axis.plot([], [], color=color, linewidth=4.0, zorder=4)
        vehicle = axis.scatter([], [], marker="P", s=260, facecolor=color,
                               edgecolor="white", linewidth=1.4, zorder=5)
        metric = axis.text(0.03, 0.94, "", transform=axis.transAxes, va="top",
                           fontsize=13, fontweight="bold", color="#11151b",
                           bbox=dict(boxstyle="round,pad=0.35", facecolor="white",
                                     edgecolor=color, alpha=0.92))
        axis.set_title(label, fontsize=14, fontweight="bold", color="white", pad=10)
        axis.set_xlim(-0.42, 3.42)
        axis.set_ylim(-0.42, 1.42)
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlabel("PX4 local north x [m]")
        axis.set_ylabel("PX4 local east y [m]")
        axis.grid(color="white", alpha=0.16, linewidth=0.8)
        trails.append(trail)
        vehicles.append(vehicle)
        metrics.append(metric)

    replay_times = np.linspace(0.0, 6.0, 240)

    def update(frame):
        replay_time = replay_times[frame]
        for index, (time, x, y, violation) in enumerate(paths):
            upto = max(1, int(np.searchsorted(time, replay_time, side="right")))
            trails[index].set_data(x[:upto], y[:upto])
            vehicles[index].set_offsets(np.array([[x[upto - 1], y[upto - 1]]]))
            live_max = float(violation[:upto].max())
            verdict = "INSIDE" if live_max <= 1.0e-3 else "CUTS CORNER"
            metrics[index].set_text(
                f"t = {replay_time:4.1f} s\nmax outside = {live_max * 100:4.1f} cm\n{verdict}")
        return trails + vehicles + metrics

    animation = FuncAnimation(fig, update, frames=len(replay_times), interval=1000 / 30,
                              blit=False)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    writer = FFMpegWriter(fps=30, codec="libx264", bitrate=2600,
                          extra_args=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])
    animation.save(str(args.output), writer=writer)
    plt.close(fig)
    print(f"wrote {args.output}")
    print(f"TinyMPC max outside: {results[0]:.6f} m")
    print(f"PX4 cascaded max outside: {results[1]:.6f} m")


if __name__ == "__main__":
    main()
