#!/usr/bin/env python3

import importlib.util
import json
import math
import pathlib
import struct
import sys
import tempfile
import types
import unittest
from contextlib import redirect_stderr, redirect_stdout
from io import StringIO
from unittest import mock


PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[1]
MODULE_PATH = PROJECT_ROOT / "tools" / "hi91_capture.py"
SPEC = importlib.util.spec_from_file_location("hi91_capture", MODULE_PATH)
hi91 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(hi91)


def make_frame(
    status=0x0807,
    timestamp=1234,
    temperature=-12,
    accel=(1.0, 2.0, 3.0),
    gyro=(4.0, 5.0, 6.0),
    euler=(7.0, 8.0, 9.0),
    quaternion=(1.0, 0.0, 0.0, 0.0),
):
    frame = bytearray(hi91.FRAME_SIZE)
    frame[0:2] = hi91.FRAME_HEADER
    struct.pack_into("<H", frame, 2, hi91.PAYLOAD_LENGTH)
    frame[6] = hi91.FRAME_TAG
    struct.pack_into("<H", frame, 7, status)
    struct.pack_into("<b", frame, 9, temperature)
    struct.pack_into("<f", frame, 10, 101325.0)
    struct.pack_into("<I", frame, 14, timestamp)
    struct.pack_into("<3f", frame, 18, *accel)
    struct.pack_into("<3f", frame, 30, *gyro)
    struct.pack_into("<3f", frame, 42, 10.0, 20.0, 30.0)
    struct.pack_into("<3f", frame, 54, *euler)
    struct.pack_into("<4f", frame, 66, *quaternion)
    struct.pack_into("<H", frame, 4, hi91.frame_crc(frame))
    return bytes(frame)


class FrameDecodeTests(unittest.TestCase):
    def test_crc_known_vector_and_all_decoded_fields(self):
        self.assertEqual(hi91.crc16_ccitt(b"123456789"), 0x31C3)
        decoded = hi91.decode_frame(make_frame())
        self.assertEqual(decoded.status, 0x0807)
        self.assertEqual(decoded.temperature_c, -12)
        self.assertEqual(decoded.system_time_ms, 1234)
        self.assertEqual(decoded.acceleration_g, (1.0, 2.0, 3.0))
        self.assertEqual(decoded.angular_rate_deg_s, (4.0, 5.0, 6.0))
        self.assertEqual(decoded.euler_deg, (7.0, 8.0, 9.0))
        self.assertEqual(decoded.quaternion, (1.0, 0.0, 0.0, 0.0))

    def test_decode_rejects_each_structural_error(self):
        for offset in (0, 2, 6, 20):
            damaged = bytearray(make_frame())
            damaged[offset] ^= 0x01
            with self.subTest(offset=offset):
                with self.assertRaises(ValueError):
                    hi91.decode_frame(bytes(damaged))


class StreamParserTests(unittest.TestCase):
    def test_fragmented_frames_and_noise_resynchronize(self):
        first = make_frame(timestamp=10)
        second = make_frame(timestamp=11, temperature=35)
        stream = b"\x00\xffnoise" + first + second
        parser = hi91.HI91StreamParser()
        decoded = []
        sizes = (1, 2, 7, 3, 19, 5, 41, 4, 64, 13)
        offset = 0
        size_index = 0
        while offset < len(stream):
            size = sizes[size_index % len(sizes)]
            decoded.extend(parser.feed(stream[offset : offset + size]))
            offset += size
            size_index += 1
        self.assertEqual([frame.system_time_ms for frame in decoded], [10, 11])
        self.assertEqual(parser.resync_bytes, len(b"\x00\xffnoise"))
        self.assertEqual(parser.crc_errors, 0)
        self.assertEqual(parser.buffered_bytes, 0)

    def test_bad_crc_is_counted_and_next_frame_is_recovered(self):
        damaged = bytearray(make_frame(timestamp=20))
        damaged[30] ^= 0x80
        parser = hi91.HI91StreamParser()
        decoded = parser.feed(bytes(damaged) + make_frame(timestamp=21))
        self.assertEqual([frame.system_time_ms for frame in decoded], [21])
        self.assertEqual(parser.crc_errors, 1)
        self.assertEqual(parser.resync_bytes, hi91.FRAME_SIZE)

    def test_bad_length_and_tag_are_counted(self):
        bad_length = bytearray(make_frame())
        bad_length[2] = 0
        bad_tag = bytearray(make_frame())
        bad_tag[6] = 0
        parser = hi91.HI91StreamParser()
        decoded = parser.feed(
            bytes(bad_length) + bytes(bad_tag) + make_frame(timestamp=99)
        )
        self.assertEqual([frame.system_time_ms for frame in decoded], [99])
        self.assertEqual(parser.length_errors, 1)
        self.assertEqual(parser.tag_errors, 1)
        self.assertEqual(parser.resync_bytes, 2 * hi91.FRAME_SIZE)

    def test_incomplete_frame_becomes_trailing_bytes(self):
        parser = hi91.HI91StreamParser()
        parser.feed(make_frame()[:37])
        self.assertEqual(parser.finish(), 37)
        self.assertEqual(parser.trailing_bytes, 37)
        self.assertEqual(parser.buffered_bytes, 0)


class CaptureStatsTests(unittest.TestCase):
    @staticmethod
    def decoded(**kwargs):
        return hi91.decode_frame(make_frame(**kwargs))

    def test_shared_milliseconds_are_not_regressions_and_dropout_recovers(self):
        stats = hi91.HI91CaptureStats()
        stats.observe(self.decoded(timestamp=100))
        stats.observe(self.decoded(timestamp=100))
        stats.observe(self.decoded(status=0x0800, timestamp=101))
        stats.observe(self.decoded(status=0x0800, timestamp=101))
        stats.observe(self.decoded(timestamp=102))
        summary = stats.summary(0.5)

        self.assertEqual(summary["timestamp_duplicate_frames"], 2)
        self.assertEqual(summary["timestamp_regressions"], 0)
        self.assertEqual(summary["frame_rate_hz"], 10.0)
        for name in ("attitude", "accel", "gyro"):
            self.assertEqual(summary["valid_counts"][name], {
                "valid": 3,
                "invalid": 2,
            })
            self.assertEqual(summary["valid_dropouts"][name]["events"], 1)
            self.assertEqual(
                summary["valid_dropouts"][name][
                    "longest_consecutive_frames"
                ],
                2,
            )
            self.assertEqual(
                summary["valid_dropouts"][name]["longest_recovery_ms"], 1
            )

    def test_uint32_wrap_is_forward_time_but_real_regression_is_counted(self):
        stats = hi91.HI91CaptureStats()
        stats.observe(self.decoded(timestamp=0xFFFFFFFE))
        stats.observe(self.decoded(status=0x0800, timestamp=0xFFFFFFFF))
        stats.observe(self.decoded(status=0x0800, timestamp=0))
        stats.observe(self.decoded(timestamp=1))
        stats.observe(self.decoded(timestamp=0))
        summary = stats.summary(1.0)

        self.assertEqual(summary["timestamp_wraps"], 1)
        self.assertEqual(summary["timestamp_regressions"], 1)
        self.assertEqual(
            summary["valid_dropouts"]["attitude"]["longest_recovery_ms"],
            2,
        )

    def test_zero_nonfinite_status_fault_and_distribution_counts(self):
        stats = hi91.HI91CaptureStats()
        fault_status = (0x0807 | (1 << 5) | (1 << 6) | (1 << 8) |
                        (1 << 13) | (1 << 15))
        stats.observe(
            self.decoded(
                status=fault_status,
                timestamp=1,
                accel=(0.0, -0.0, 0.0),
                gyro=(0.0, 0.0, 0.0),
                euler=(0.0, 0.0, 0.0),
            )
        )
        stats.observe(
            self.decoded(
                status=0x0800,
                timestamp=2,
                accel=(0.0, 0.0, 0.0),
                gyro=(math.nan, 0.0, 0.0),
                euler=(0.0, 0.0, 0.0),
            )
        )
        summary = stats.summary(1.0)

        self.assertEqual(summary["zero_triplets"]["accel"], {
            "valid": 1,
            "invalid": 1,
        })
        self.assertEqual(summary["zero_triplets"]["gyro"], {
            "valid": 1,
            "invalid": 0,
        })
        self.assertEqual(summary["zero_triplets"]["euler"], {
            "valid": 1,
            "invalid": 1,
        })
        self.assertEqual(summary["nonfinite"]["frames"], 1)
        self.assertEqual(summary["nonfinite"]["by_field"]["gyro"], 1)
        self.assertEqual(summary["fault_counts"]["bmi_fault_frames"], 1)
        self.assertEqual(summary["fault_counts"]["any_fault_frames"], 1)
        self.assertEqual(summary["acc_saturation_frames"], 1)
        self.assertEqual(summary["gyro_saturation_frames"], 1)
        self.assertEqual(
            summary["status_counts"]["heading_continuity_lost"], 1
        )
        self.assertEqual(summary["sout_pulse_frames"], 0)
        self.assertEqual(summary["stream_drop_frames"], 1)
        self.assertEqual(summary["status_distribution"]["0x0800"], 1)

    def test_first_valid_and_latest_inverse_convergence_clear_times(self):
        stats = hi91.HI91CaptureStats()
        base = 1 << 11
        stats.observe(
            self.decoded(
                status=base | (1 << 3) | (1 << 7),
                timestamp=0xFFFFFFFE,
            )
        )
        stats.observe(
            self.decoded(
                status=base | (1 << 1) | (1 << 3) | (1 << 7),
                timestamp=0xFFFFFFFF,
            )
        )
        stats.observe(
            self.decoded(
                status=base | (1 << 0) | (1 << 1) | (1 << 7),
                timestamp=0,
            )
        )
        stats.observe(
            self.decoded(status=base | 0x07 | (1 << 3), timestamp=1)
        )
        stats.observe(self.decoded(status=base | 0x07, timestamp=2))
        summary = stats.summary(1.0)

        first_valid = summary["first_valid_device_times"]
        self.assertEqual(
            first_valid["accel"]["device_time"],
            {
                "raw_system_time_ms": 0xFFFFFFFF,
                "unwrapped_system_time_ms": 0xFFFFFFFF,
                "elapsed_since_first_frame_ms": 1,
            },
        )
        self.assertEqual(
            first_valid["attitude"]["device_time"],
            {
                "raw_system_time_ms": 0,
                "unwrapped_system_time_ms": 0x100000000,
                "elapsed_since_first_frame_ms": 2,
            },
        )
        self.assertEqual(
            first_valid["gyro"]["device_time"][
                "elapsed_since_first_frame_ms"
            ],
            3,
        )

        convergence = summary["inverse_convergence_clearance"]
        wb_conv = convergence["wb_conv_bit3"]
        self.assertTrue(wb_conv["ever_asserted"])
        self.assertFalse(wb_conv["asserted_in_last_frame"])
        self.assertTrue(wb_conv["last_assertion_cleared"])
        self.assertEqual(
            wb_conv["last_assertion_start_device_time"][
                "raw_system_time_ms"
            ],
            1,
        )
        self.assertEqual(
            wb_conv["last_assertion_clear_device_time"],
            {
                "raw_system_time_ms": 2,
                "unwrapped_system_time_ms": 0x100000002,
                "elapsed_since_first_frame_ms": 4,
            },
        )
        att_conv = convergence["att_conv_bit7"]
        self.assertTrue(att_conv["last_assertion_cleared"])
        self.assertEqual(
            att_conv["last_assertion_clear_device_time"][
                "elapsed_since_first_frame_ms"
            ],
            3,
        )

    def test_inverse_convergence_never_asserted_and_never_cleared(self):
        stats = hi91.HI91CaptureStats()
        stats.observe(self.decoded(status=0x0803, timestamp=10))
        stats.observe(self.decoded(status=0x0883, timestamp=11))
        summary = stats.summary(1.0)

        self.assertFalse(summary["first_valid_device_times"]["gyro"]["observed"])
        self.assertIsNone(
            summary["first_valid_device_times"]["gyro"]["device_time"]
        )

        wb_conv = summary["inverse_convergence_clearance"]["wb_conv_bit3"]
        self.assertFalse(wb_conv["ever_asserted"])
        self.assertFalse(wb_conv["last_assertion_cleared"])
        self.assertIsNone(wb_conv["last_assertion_start_device_time"])
        self.assertIsNone(wb_conv["last_assertion_clear_device_time"])

        att_conv = summary["inverse_convergence_clearance"][
            "att_conv_bit7"
        ]
        self.assertTrue(att_conv["ever_asserted"])
        self.assertTrue(att_conv["asserted_in_last_frame"])
        self.assertFalse(att_conv["last_assertion_cleared"])
        self.assertIsNone(att_conv["last_assertion_clear_device_time"])


class SummaryPersistenceTests(unittest.TestCase):
    def test_cli_atomically_persists_the_same_completed_summary(self):
        expected = {"capture_duration_s": 1.0, "frames": 1600}
        with tempfile.TemporaryDirectory() as temporary_directory:
            summary_path = pathlib.Path(temporary_directory) / "summary.json"
            summary_path.write_text("stale", encoding="utf-8")
            output = StringIO()
            with mock.patch.object(hi91, "capture", return_value=expected):
                with redirect_stdout(output):
                    result = hi91.main(
                        [
                            "--port",
                            "/dev/fake",
                            "--seconds",
                            "1",
                            "--summary-json",
                            str(summary_path),
                        ]
                    )

            self.assertEqual(result, 0)
            self.assertEqual(json.loads(output.getvalue()), expected)
            self.assertEqual(
                json.loads(summary_path.read_text(encoding="utf-8")), expected
            )
            self.assertEqual(
                summary_path.read_text(encoding="utf-8"), output.getvalue()
            )
            self.assertEqual(
                list(pathlib.Path(temporary_directory).glob(".summary.json.*")),
                [],
            )

    def test_cli_error_removes_stale_summary_and_publishes_nothing(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            summary_path = pathlib.Path(temporary_directory) / "summary.json"
            summary_path.write_text("stale", encoding="utf-8")
            output = StringIO()
            errors = StringIO()
            with mock.patch.object(
                hi91, "capture", side_effect=RuntimeError("capture failed")
            ):
                with redirect_stdout(output), redirect_stderr(errors):
                    result = hi91.main(
                        [
                            "--port",
                            "/dev/fake",
                            "--summary-json",
                            str(summary_path),
                        ]
                    )

            self.assertEqual(result, 1)
            self.assertEqual(output.getvalue(), "")
            self.assertIn("capture failed", errors.getvalue())
            self.assertFalse(summary_path.exists())

    def test_cli_atomic_publish_failure_removes_temporary_output(self):
        expected = {"capture_duration_s": 1.0, "frames": 1600}
        with tempfile.TemporaryDirectory() as temporary_directory:
            directory = pathlib.Path(temporary_directory)
            summary_path = directory / "summary.json"
            output = StringIO()
            errors = StringIO()
            with mock.patch.object(hi91, "capture", return_value=expected):
                with mock.patch.object(
                    hi91.os, "replace", side_effect=OSError("replace failed")
                ):
                    with redirect_stdout(output), redirect_stderr(errors):
                        result = hi91.main(
                            [
                                "--port",
                                "/dev/fake",
                                "--summary-json",
                                str(summary_path),
                            ]
                        )

            self.assertEqual(result, 1)
            self.assertEqual(output.getvalue(), "")
            self.assertIn("replace failed", errors.getvalue())
            self.assertFalse(summary_path.exists())
            self.assertEqual(list(directory.glob(".summary.json.*")), [])

    def test_cli_interrupt_removes_stale_summary_and_publishes_nothing(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            summary_path = pathlib.Path(temporary_directory) / "summary.json"
            summary_path.write_text("stale", encoding="utf-8")
            with mock.patch.object(
                hi91, "capture", side_effect=KeyboardInterrupt
            ):
                with self.assertRaises(KeyboardInterrupt):
                    hi91.main(
                        [
                            "--port",
                            "/dev/fake",
                            "--summary-json",
                            str(summary_path),
                        ]
                    )

            self.assertFalse(summary_path.exists())


class SerialSetupTests(unittest.TestCase):
    def test_raw_dtr_and_flush_use_the_one_open_serial_fd(self):
        events = []

        class FakeSerial:
            def __init__(self):
                self.is_open = False
                self._dtr = None

            @property
            def dtr(self):
                return self._dtr

            @dtr.setter
            def dtr(self, value):
                self._dtr = value
                events.append(("dtr", value, self.is_open))

            def open(self):
                self.is_open = True
                events.append(("open",))

            def fileno(self):
                events.append(("fileno",))
                return 73

            def reset_input_buffer(self):
                events.append(("flush_input",))

            def close(self):
                self.is_open = False
                events.append(("close",))

        device = FakeSerial()
        serial_module = types.SimpleNamespace(Serial=lambda: device)
        with mock.patch.dict(sys.modules, {"serial": serial_module}):
            with mock.patch.object(hi91.os, "isatty", return_value=True):
                with mock.patch.object(hi91.tty, "setraw") as setraw:
                    result = hi91.open_capture_serial("/dev/fake", 115200)

        self.assertIs(result, device)
        setraw.assert_called_once_with(73, when=hi91.termios.TCSANOW)
        self.assertEqual(
            events,
            [
                ("dtr", False, False),
                ("open",),
                ("fileno",),
                ("dtr", True, True),
                ("flush_input",),
            ],
        )
        self.assertFalse(hasattr(device, "write"))
        device.close()


if __name__ == "__main__":
    unittest.main(verbosity=2)
