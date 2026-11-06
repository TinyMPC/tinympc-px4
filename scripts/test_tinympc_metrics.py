#!/usr/bin/env python3
"""Regression checks for restart, failure, dropout and idle-sample accounting."""

import unittest
from pathlib import Path

import numpy as np

from summarize_tinympc_log import summarize


def fixture(n=4):
    data = {}
    msg = Path(__file__).resolve().parents[1] / "px4_external/msg/TinympcStatus.msg"
    for line in msg.read_text().splitlines():
        fields = line.split("#", 1)[0].split()
        if len(fields) != 2:
            continue
        typ, name = fields
        if "[" in typ:
            for i in range(int(typ.split("[")[1].rstrip("]"))):
                data[f"{name}[{i}]"] = np.zeros(n, dtype=float if typ.startswith("float") else int)
        else:
            data[name] = np.zeros(n, dtype=float if typ.startswith("float") else int)
    data["schema_version"][:] = 1
    data["run_id"][:] = 1000000
    data["sequence"] = np.arange(1, n + 1)
    data["timestamp"] = np.arange(n) * 20000 + 1000000
    data["running"][:] = 1
    data["config[3]"][:] = 0.02
    data["heap_min_free_bytes"][:] = 2 ** 32 - 1
    data["stack_free_bytes"][:] = 2 ** 32 - 1
    data["init_heap_delta_bytes"][:] = -(2 ** 31)
    return data


class MetricTests(unittest.TestCase):
    def test_idle_and_stopped_samples_are_not_solves(self):
        d = fixture()
        d["solve_called"][:] = [0, 1, 1, 0]
        d["running"][-1] = 0
        d["solve_time_us"][:] = [0, 100, 300, 0]
        d["solve_count"][:] = [0, 1, 2, 2]
        report = summarize(d)[0]
        self.assertEqual(report["solve_us"]["p50"], 200)
        self.assertEqual(report["solve_us"]["max"], 300)
        self.assertEqual(report["solve_capture_fraction"], 1)
        self.assertIsNone(report["minimum_sampled_heap_free_bytes"])

    def test_failed_solve_included(self):
        d = fixture()
        d["solve_called"][:] = [1, 1, 0, 0]
        d["solve_count"][:] = [1, 2, 2, 2]
        d["solve_time_us"][:] = [100, 19000, 0, 0]
        d["failure_reason"][:] = [0, 6, 6, 6]
        d["solve_deadline_misses"][:] = [0, 1, 1, 1]
        report = summarize(d)[0]
        self.assertEqual(report["solve_us"]["max"], 19000)
        self.assertEqual(len(report["failures"]), 1)
        self.assertTrue(report["failures"][0]["solve_called"])
        self.assertEqual(report["counts_at_last_sample"]["solve_deadline_misses"], 1)

    def test_restart_keeps_counters_separate(self):
        d = fixture()
        d["sequence"][:] = [1, 2, 1, 2]
        d["solve_count"][:] = [1, 2, 1, 2]
        d["run_id"][:] = [1000000, 1000000, 2000000, 2000000]
        self.assertEqual(len(summarize(d)), 2)

    def test_failed_init_is_separate_from_restart(self):
        d = fixture()
        d["sequence"][:] = [1, 1, 2, 3]
        d["run_id"][:] = [1000000, 2000000, 2000000, 2000000]
        d["running"][0] = 0
        d["failure_reason"][0] = 1
        reports = summarize(d)
        self.assertEqual(len(reports), 2)
        self.assertEqual(reports[0]["failures"][0]["reason"], 1)
        self.assertEqual(reports[1]["failures"], [])

    def test_dropout_does_not_inflate_outside_time(self):
        d = fixture()
        d["sequence"][:] = [1, 2, 12, 13]
        d["timestamp"][:] = [1000000, 1020000, 1220000, 1240000]
        d["tracking_valid"][:] = 1
        d["hold_position_margin"][:] = -0.1
        report = summarize(d)[0]
        self.assertEqual(report["missing_cycle_samples"], 9)
        self.assertAlmostEqual(report["observed_time_outside_position_bounds_s"], 0.06)

    def test_unknown_schema_refused(self):
        d = fixture()
        d["schema_version"][:] = 2
        with self.assertRaises(ValueError):
            summarize(d)


if __name__ == "__main__":
    unittest.main()
