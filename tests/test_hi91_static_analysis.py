#!/usr/bin/env python3

import csv
import importlib.util
import json
import math
import pathlib
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from io import StringIO


PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[1]
MODULE_PATH = PROJECT_ROOT / "tools" / "hi91_static_analysis.py"
SPEC = importlib.util.spec_from_file_location("hi91_static_analysis", MODULE_PATH)
analysis = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(analysis)


def make_row(
    frame_index,
    system_time_ms,
    host_elapsed_s=None,
    status=0x0807,
    temperature_c=25,
    accel=(0.1, 0.2, 1.0),
    gyro=(1.0, 2.0, 3.0),
    euler=(10.0, 20.0, 30.0),
    quaternion=(1.0, 0.0, 0.0, 0.0),
):
    if host_elapsed_s is None:
        host_elapsed_s = frame_index / 1000.0
    return {
        "host_elapsed_s": host_elapsed_s,
        "frame_index": frame_index,
        "system_time_ms": system_time_ms,
        "status": status,
        "status_hex": "0x{:04X}".format(status),
        "temperature_c": temperature_c,
        "accel_x_g": accel[0],
        "accel_y_g": accel[1],
        "accel_z_g": accel[2],
        "gyro_x_deg_s": gyro[0],
        "gyro_y_deg_s": gyro[1],
        "gyro_z_deg_s": gyro[2],
        "roll_deg": euler[0],
        "pitch_deg": euler[1],
        "yaw_deg": euler[2],
        "quat_w": quaternion[0],
        "quat_x": quaternion[1],
        "quat_y": quaternion[2],
        "quat_z": quaternion[3],
    }


class TemporaryCsv:
    def __init__(self, rows, fieldnames=analysis.REQUIRED_COLUMNS):
        self._temporary = tempfile.TemporaryDirectory()
        self.path = pathlib.Path(self._temporary.name) / "capture.csv"
        with self.path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)

    def close(self):
        self._temporary.cleanup()


class StreamingAnalysisTests(unittest.TestCase):
    def analyze(self, rows, **kwargs):
        capture = TemporaryCsv(rows)
        self.addCleanup(capture.close)
        return analysis.analyze_csv(str(capture.path), **kwargs)

    def test_device_uint32_wrap_duplicate_regression_and_frame_gap(self):
        rows = [
            make_row(1, 0xFFFFFFFE, 0.000),
            make_row(2, 0xFFFFFFFF, 0.001),
            make_row(4, 0, 0.002),
            make_row(5, 0, 0.002),
            make_row(6, 0xFFFFFFFF, 0.001),
            make_row(7, 1, 0.003),
        ]
        result = self.analyze(rows)

        self.assertEqual(result["rows"], 6)
        self.assertEqual(result["frame_index"]["gap_events"], 1)
        self.assertEqual(result["frame_index"]["missing_frames"], 1)
        self.assertFalse(result["frame_index"]["sequence_contiguous"])
        self.assertEqual(result["device_time"]["wraps"], 1)
        self.assertEqual(result["device_time"]["uint32_wraps"], 1)
        self.assertEqual(result["device_time"]["day_wraps"], 0)
        self.assertEqual(result["device_time"]["duplicate_frames"], 1)
        self.assertEqual(result["device_time"]["regressions"], 1)
        self.assertEqual(result["device_time"]["unwrapped_elapsed_ms"], 3)
        self.assertEqual(result["host_time"]["duplicate_frames"], 1)
        self.assertEqual(result["host_time"]["regressions"], 1)
        self.assertAlmostEqual(result["average_frame_rate_hz"], 2000.0)

    def test_device_daily_wrap_is_forward_time(self):
        day = analysis.DAY_MILLISECONDS
        rows = [
            make_row(1, day - 2, 0.001),
            make_row(2, day - 1, 0.002),
            make_row(3, 0, 0.003),
            make_row(4, 1, 0.004),
        ]
        result = self.analyze(rows)

        self.assertEqual(result["device_time"]["unwrapped_elapsed_ms"], 3)
        self.assertEqual(result["device_time"]["wraps"], 1)
        self.assertEqual(result["device_time"]["day_wraps"], 1)
        self.assertEqual(result["device_time"]["uint32_wraps"], 0)
        self.assertEqual(result["device_time"]["regressions"], 0)

    def test_euler_unwrap_windows_burn_in_and_gyro_moments(self):
        rows = [
            make_row(1, 0, 0.0, euler=(0.0, 0.0, 179.0), gyro=(1, 2, 3)),
            make_row(2, 1000, 1.0, euler=(1.0, 2.0, -179.0), gyro=(2, 3, 4)),
            make_row(3, 2000, 2.0, euler=(2.0, 4.0, -177.0), gyro=(3, 4, 5)),
            make_row(4, 3000, 3.0, euler=(3.0, 6.0, -175.0), gyro=(4, 5, 6)),
        ]
        result = self.analyze(
            rows, window_seconds=1.0, burn_in_seconds=2.0
        )

        yaw = result["euler_deg"]["axes"]["yaw"]
        self.assertAlmostEqual(
            yaw["all_span"]["linear_trend_deg_per_hour"], 7200.0
        )
        self.assertAlmostEqual(
            yaw["post_burn_in"]["linear_trend_deg_per_hour"], 7200.0
        )
        self.assertEqual(yaw["window"]["first_samples"], 2)
        self.assertEqual(yaw["window"]["last_samples"], 2)
        self.assertAlmostEqual(
            yaw["window"]["first_mean_unwrapped_deg"], 180.0
        )
        self.assertAlmostEqual(
            yaw["window"]["last_mean_unwrapped_deg"], 184.0
        )
        self.assertAlmostEqual(
            yaw["window"]["last_minus_first_mean_deg"], 4.0
        )
        self.assertAlmostEqual(yaw["all_span"]["mean_unwrapped_deg"], 182.0)
        self.assertAlmostEqual(
            yaw["all_span"]["standard_deviation_deg"], math.sqrt(5.0)
        )
        self.assertEqual(yaw["all_span"]["minimum_unwrapped_deg"], 179.0)
        self.assertEqual(yaw["all_span"]["maximum_unwrapped_deg"], 185.0)
        self.assertEqual(yaw["all_span"]["peak_to_peak_deg"], 6.0)
        self.assertAlmostEqual(
            yaw["all_span"]["linear_fit_residual_rms_deg"], 0.0
        )
        self.assertAlmostEqual(
            yaw["post_burn_in"]["standard_deviation_deg"], 1.0
        )
        self.assertEqual(yaw["post_burn_in"]["peak_to_peak_deg"], 2.0)
        self.assertAlmostEqual(
            yaw["post_burn_in"]["linear_fit_residual_rms_deg"], 0.0
        )

        gyro_x = result["gyro_deg_s"]["axes"]["x"]
        self.assertAlmostEqual(gyro_x["all_span"]["mean"], 2.5)
        self.assertAlmostEqual(
            gyro_x["all_span"]["rms"], math.sqrt(7.5)
        )
        self.assertAlmostEqual(
            gyro_x["all_span"]["standard_deviation"], math.sqrt(1.25)
        )
        self.assertEqual(gyro_x["post_burn_in"]["samples"], 2)
        self.assertAlmostEqual(gyro_x["post_burn_in"]["mean"], 3.5)

    def test_euler_linear_fit_residual_rms_detects_nonlinear_motion(self):
        rows = [
            make_row(1, 0, 0.0, euler=(0.0, 0.0, 0.0)),
            make_row(2, 1000, 1.0, euler=(0.0, 0.0, 1.0)),
            make_row(3, 2000, 2.0, euler=(0.0, 0.0, 0.0)),
        ]
        result = self.analyze(rows)
        yaw = result["euler_deg"]["axes"]["yaw"]["all_span"]

        self.assertAlmostEqual(yaw["mean_unwrapped_deg"], 1.0 / 3.0)
        self.assertAlmostEqual(
            yaw["standard_deviation_deg"], math.sqrt(2.0) / 3.0
        )
        self.assertEqual(yaw["minimum_unwrapped_deg"], 0.0)
        self.assertEqual(yaw["maximum_unwrapped_deg"], 1.0)
        self.assertEqual(yaw["peak_to_peak_deg"], 1.0)
        self.assertAlmostEqual(
            yaw["linear_fit_residual_rms_deg"], math.sqrt(2.0) / 3.0
        )

    def test_post_burn_in_window_has_its_own_steady_first_window(self):
        rows = [
            make_row(
                index + 1,
                index * 1000,
                float(index),
                euler=(float(index * 10), 0.0, 0.0),
            )
            for index in range(7)
        ]
        result = self.analyze(
            rows, window_seconds=1.0, burn_in_seconds=2.0
        )
        roll = result["euler_deg"]["axes"]["roll"]

        self.assertEqual(roll["window"]["first_samples"], 2)
        self.assertAlmostEqual(
            roll["window"]["first_mean_unwrapped_deg"], 5.0
        )
        self.assertAlmostEqual(
            roll["window"]["last_mean_unwrapped_deg"], 55.0
        )
        self.assertAlmostEqual(
            roll["window"]["last_minus_first_mean_deg"], 50.0
        )

        steady = roll["post_burn_in_window"]
        self.assertEqual(steady["first_samples"], 2)
        self.assertEqual(steady["last_samples"], 2)
        self.assertAlmostEqual(steady["first_mean_unwrapped_deg"], 25.0)
        self.assertAlmostEqual(steady["last_mean_unwrapped_deg"], 55.0)
        self.assertAlmostEqual(steady["last_minus_first_mean_deg"], 30.0)

    def test_tail_window_aggregates_duplicate_device_milliseconds(self):
        tail = analysis._TailWindow(300.0)
        for _ in range(10000):
            tail.append(0.0, (1.0, 2.0, math.nan))

        self.assertEqual(len(tail.blocks), 1)
        self.assertEqual(
            len(tail.blocks[0].values), tail._VALUES_PER_RECORD
        )
        self.assertEqual(tail.sums.counts, [10000, 10000, 0])
        self.assertEqual(tail.sums.mean(0), 1.0)
        self.assertEqual(tail.sums.mean(1), 2.0)
        self.assertIsNone(tail.sums.mean(2))

        tail.append(301.0, (3.0, 4.0, 5.0))
        self.assertEqual(tail.sums.counts, [1, 1, 1])
        self.assertEqual(tail.sums.mean(0), 3.0)

    def test_static_and_nonstatic_runs_expose_single_frame_glitches(self):
        static = 0x0807 | (1 << 9)
        dynamic = 0x0807
        states = (static, dynamic, static, static, dynamic, dynamic)
        rows = [
            make_row(index, (index - 1) * 10, status=status)
            for index, status in enumerate(states, start=1)
        ]
        result = self.analyze(rows)

        self.assertEqual(result["static_runs"]["count"], 2)
        self.assertEqual(result["static_runs"]["total_frames"], 3)
        self.assertEqual(result["static_runs"]["frame_fraction"], 0.5)
        self.assertEqual(result["static_runs"]["shortest_frames"], 1)
        self.assertEqual(result["static_runs"]["longest_frames"], 2)
        self.assertEqual(result["static_runs"]["single_frame_runs"], 1)
        self.assertEqual(result["nonstatic_runs"]["count"], 2)
        self.assertEqual(result["nonstatic_runs"]["single_frame_runs"], 1)
        self.assertEqual(result["static_state_transitions"], 3)

    def test_nonfinite_valid_zero_status_quaternion_and_temperature(self):
        special_status = 0x0807 | (1 << 5) | (1 << 6) | (1 << 13) | (1 << 15)
        rows = [
            make_row(
                1,
                0,
                status=special_status,
                temperature_c=-12,
                accel=(0.0, -0.0, 0.0),
                gyro=(0.0, 0.0, 0.0),
                euler=(0.0, 0.0, 0.0),
                quaternion=(1.0, 0.0, 0.0, 0.0),
            ),
            make_row(
                2,
                1,
                status=0x0800,
                temperature_c=37,
                accel=(math.nan, 0.0, 0.0),
                gyro=(math.inf, 0.0, 0.0),
                euler=(math.nan, 0.0, 0.0),
                quaternion=(math.nan, 0.0, 0.0, 0.0),
            ),
            make_row(3, 2, quaternion=(2.0, 0.0, 0.0, 0.0)),
        ]
        result = self.analyze(rows)

        self.assertEqual(result["valid_zero_triplets"], {
            "accel": 1,
            "gyro": 1,
            "euler": 1,
        })
        self.assertEqual(result["nonfinite"]["frames"], 1)
        self.assertEqual(result["nonfinite"]["values"], 4)
        self.assertEqual(result["quaternion_norm"]["samples"], 2)
        self.assertEqual(result["quaternion_norm"]["nonfinite_samples"], 1)
        self.assertEqual(result["quaternion_norm"]["max_abs_error"], 1.0)
        self.assertAlmostEqual(
            result["quaternion_norm"]["rms_error"], math.sqrt(0.5)
        )
        self.assertEqual(result["temperature_c"], {
            "minimum": -12,
            "maximum": 37,
        })
        self.assertEqual(result["fault_counts"]["any_fault_frames"], 1)
        self.assertEqual(result["saturation_counts"]["acc_frames"], 1)
        self.assertEqual(result["saturation_counts"]["gyro_frames"], 1)
        self.assertEqual(result["stream_drop_frames"], 1)
        self.assertEqual(
            result["status_distribution"][
                "0x{:04X}".format(special_status)
            ],
            1,
        )
        self.assertEqual(result["status_bit_counts"]["bmi_fault"], 1)
        self.assertEqual(
            result["euler_deg"]["axes"]["roll"]["invalid_samples"], 0
        )
        self.assertEqual(
            result["euler_deg"]["axes"]["pitch"]["invalid_samples"], 1
        )

    def test_missing_bad_and_ragged_columns_are_rejected(self):
        missing_fields = [
            name for name in analysis.REQUIRED_COLUMNS if name != "yaw_deg"
        ]
        missing = TemporaryCsv([], missing_fields)
        self.addCleanup(missing.close)
        with self.assertRaisesRegex(analysis.AnalysisError, "missing.*yaw_deg"):
            analysis.analyze_csv(str(missing.path))

        bad_row = make_row(1, 0)
        bad_row["quat_w"] = "not-a-number"
        bad = TemporaryCsv([bad_row])
        self.addCleanup(bad.close)
        with self.assertRaisesRegex(analysis.AnalysisError, "row 2.*quat_w"):
            analysis.analyze_csv(str(bad.path))

        mismatch_row = make_row(1, 0)
        mismatch_row["status_hex"] = "0x0000"
        mismatch = TemporaryCsv([mismatch_row])
        self.addCleanup(mismatch.close)
        with self.assertRaisesRegex(analysis.AnalysisError, "status/status_hex"):
            analysis.analyze_csv(str(mismatch.path))

        ragged = TemporaryCsv([])
        self.addCleanup(ragged.close)
        with ragged.path.open("a", encoding="utf-8") as csv_file:
            csv_file.write("1,2,3\n")
        with self.assertRaisesRegex(analysis.AnalysisError, "row 2 has 3 columns"):
            analysis.analyze_csv(str(ragged.path))

    def test_cli_emits_strict_json_and_reports_schema_errors(self):
        capture = TemporaryCsv([make_row(1, 0)])
        self.addCleanup(capture.close)
        output = StringIO()
        with redirect_stdout(output):
            self.assertEqual(
                analysis.main(
                    [
                        str(capture.path),
                        "--window-seconds",
                        "1",
                        "--burn-in-seconds",
                        "0",
                    ]
                ),
                0,
            )
        parsed = json.loads(output.getvalue())
        self.assertEqual(parsed["rows"], 1)

        capture.path.write_text("bad,header\n", encoding="utf-8")
        errors = StringIO()
        with redirect_stderr(errors):
            self.assertEqual(analysis.main([str(capture.path)]), 1)
        self.assertIn("missing required columns", errors.getvalue())


if __name__ == "__main__":
    unittest.main(verbosity=2)
