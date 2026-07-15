#!/usr/bin/env python3
"""Read-only HI91 stream capture and integrity diagnostics."""

import argparse
import csv
import json
import math
import os
import pathlib
import struct
import sys
import termios
import tempfile
import time
import tty
from collections import Counter
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


FRAME_HEADER = b"\x5a\xa5"
PAYLOAD_LENGTH = 76
FRAME_SIZE = 82
FRAME_TAG = 0x91
UINT32_MASK = 0xFFFFFFFF
UINT32_HALF_RANGE = 0x80000000

STATUS_BITS = (
    ("attitude_valid", 1 << 0),
    ("accel_valid", 1 << 1),
    ("gyro_valid", 1 << 2),
    ("bias_not_converged", 1 << 3),
    ("mag_disturbed", 1 << 4),
    ("acc_saturation_recent", 1 << 5),
    ("gyro_saturation_recent", 1 << 6),
    ("attitude_not_converged", 1 << 7),
    ("heading_continuity_lost", 1 << 8),
    ("stationary", 1 << 9),
    ("mag_aiding", 1 << 10),
    ("utc_unsynced", 1 << 11),
    ("sout_pulse", 1 << 12),
    ("bmi_fault", 1 << 13),
    ("icm_fault", 1 << 14),
    ("stream_drop", 1 << 15),
)

VALID_BITS = {
    "attitude": 1 << 0,
    "accel": 1 << 1,
    "gyro": 1 << 2,
}


def crc16_ccitt(data: Iterable[int], initial: int = 0) -> int:
    """CRC-16/CCITT, polynomial 0x1021, non-reflected."""
    crc = initial & 0xFFFF
    for value in data:
        folded = (value & 0xFF) ^ (crc >> 8)
        folded ^= folded >> 4
        crc = (
            (crc << 8)
            ^ (folded << 12)
            ^ (folded << 5)
            ^ folded
        ) & 0xFFFF
    return crc


def frame_crc(frame: bytes) -> int:
    crc = crc16_ccitt(frame[0:4])
    return crc16_ccitt(frame[6:FRAME_SIZE], crc)


@dataclass(frozen=True)
class HI91Frame:
    status: int
    temperature_c: int
    air_pressure_pa: float
    system_time_ms: int
    acceleration_g: Tuple[float, float, float]
    angular_rate_deg_s: Tuple[float, float, float]
    magnetic_field_ut: Tuple[float, float, float]
    euler_deg: Tuple[float, float, float]
    quaternion: Tuple[float, float, float, float]


def _decode_validated_frame(frame: bytes) -> HI91Frame:
    return HI91Frame(
        status=struct.unpack_from("<H", frame, 7)[0],
        temperature_c=struct.unpack_from("<b", frame, 9)[0],
        air_pressure_pa=struct.unpack_from("<f", frame, 10)[0],
        system_time_ms=struct.unpack_from("<I", frame, 14)[0],
        acceleration_g=struct.unpack_from("<3f", frame, 18),
        angular_rate_deg_s=struct.unpack_from("<3f", frame, 30),
        magnetic_field_ut=struct.unpack_from("<3f", frame, 42),
        euler_deg=struct.unpack_from("<3f", frame, 54),
        quaternion=struct.unpack_from("<4f", frame, 66),
    )


def decode_frame(frame: bytes) -> HI91Frame:
    """Validate and decode one complete fixed-size frame."""
    if len(frame) != FRAME_SIZE:
        raise ValueError("HI91 frame must be exactly 82 bytes")
    if frame[0:2] != FRAME_HEADER:
        raise ValueError("invalid HI91 header")
    if struct.unpack_from("<H", frame, 2)[0] != PAYLOAD_LENGTH:
        raise ValueError("invalid HI91 payload length")
    if frame[6] != FRAME_TAG:
        raise ValueError("invalid HI91 tag")
    if struct.unpack_from("<H", frame, 4)[0] != frame_crc(frame):
        raise ValueError("invalid HI91 CRC")
    return _decode_validated_frame(frame)


class HI91StreamParser:
    """Incremental parser that discards bytes until a valid frame is found."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.crc_errors = 0
        self.length_errors = 0
        self.tag_errors = 0
        self.resync_bytes = 0
        self.trailing_bytes = 0

    @property
    def buffered_bytes(self) -> int:
        return len(self._buffer)

    def feed(self, data: bytes) -> List[HI91Frame]:
        if data:
            self._buffer.extend(data)

        frames = []
        while True:
            header_offset = self._buffer.find(FRAME_HEADER)
            if header_offset < 0:
                keep = 1 if self._buffer[-1:] == FRAME_HEADER[:1] else 0
                discard = len(self._buffer) - keep
                if discard:
                    del self._buffer[:discard]
                    self.resync_bytes += discard
                break

            if header_offset:
                del self._buffer[:header_offset]
                self.resync_bytes += header_offset

            if len(self._buffer) < 7:
                break

            if struct.unpack_from("<H", self._buffer, 2)[0] != PAYLOAD_LENGTH:
                self.length_errors += 1
                del self._buffer[0]
                self.resync_bytes += 1
                continue

            if self._buffer[6] != FRAME_TAG:
                self.tag_errors += 1
                del self._buffer[0]
                self.resync_bytes += 1
                continue

            if len(self._buffer) < FRAME_SIZE:
                break

            candidate = bytes(self._buffer[:FRAME_SIZE])
            expected_crc = struct.unpack_from("<H", candidate, 4)[0]
            if frame_crc(candidate) != expected_crc:
                self.crc_errors += 1
                del self._buffer[0]
                self.resync_bytes += 1
                continue

            frames.append(_decode_validated_frame(candidate))
            del self._buffer[:FRAME_SIZE]

        return frames

    def finish(self) -> int:
        """Record and discard an incomplete frame at the end of capture."""
        trailing = len(self._buffer)
        self.trailing_bytes += trailing
        self._buffer.clear()
        return trailing


@dataclass
class _DropoutRun:
    seen_valid: bool = False
    active: bool = False
    events: int = 0
    recovered_events: int = 0
    current_frames: int = 0
    longest_frames: int = 0
    start_timestamp_ms: int = 0
    longest_recovery_ms: int = 0

    def observe(self, valid: bool, timestamp_ms: int) -> None:
        if valid:
            self.seen_valid = True
            if self.active:
                recovery_ms = max(0, timestamp_ms - self.start_timestamp_ms)
                self.longest_recovery_ms = max(
                    self.longest_recovery_ms, recovery_ms
                )
                self.recovered_events += 1
                self.active = False
                self.current_frames = 0
            return

        if not self.seen_valid:
            return
        if not self.active:
            self.active = True
            self.events += 1
            self.current_frames = 0
            self.start_timestamp_ms = timestamp_ms
        self.current_frames += 1
        self.longest_frames = max(self.longest_frames, self.current_frames)

    def as_dict(self) -> Dict[str, object]:
        return {
            "events": self.events,
            "recovered_events": self.recovered_events,
            "longest_consecutive_frames": self.longest_frames,
            "longest_recovery_ms": self.longest_recovery_ms,
            "currently_invalid": self.active,
            "current_invalid_frames": self.current_frames,
        }


@dataclass(frozen=True)
class _DeviceEventTime:
    raw_system_time_ms: int
    unwrapped_system_time_ms: int

    def as_dict(self, first_unwrapped_time_ms: Optional[int]) -> Dict[str, int]:
        elapsed_ms = 0
        if first_unwrapped_time_ms is not None:
            elapsed_ms = max(
                0, self.unwrapped_system_time_ms - first_unwrapped_time_ms
            )
        return {
            "raw_system_time_ms": self.raw_system_time_ms,
            "unwrapped_system_time_ms": self.unwrapped_system_time_ms,
            "elapsed_since_first_frame_ms": elapsed_ms,
        }


@dataclass
class _InverseConvergenceRun:
    active_meaning: str
    ever_asserted: bool = False
    _last_observed_asserted: Optional[bool] = None
    _last_assertion_start: Optional[_DeviceEventTime] = None
    _last_assertion_clear: Optional[_DeviceEventTime] = None

    def observe(self, asserted: bool, event_time: _DeviceEventTime) -> None:
        if asserted and self._last_observed_asserted is not True:
            self.ever_asserted = True
            self._last_assertion_start = event_time
            self._last_assertion_clear = None
        elif not asserted and self._last_observed_asserted is True:
            self._last_assertion_clear = event_time
        self._last_observed_asserted = asserted

    def as_dict(self, first_unwrapped_time_ms: Optional[int]) -> Dict[str, object]:
        asserted_at_end = self._last_observed_asserted
        last_assertion_cleared = (
            self.ever_asserted
            and asserted_at_end is False
            and self._last_assertion_clear is not None
        )
        return {
            "active_meaning": self.active_meaning,
            "ever_asserted": self.ever_asserted,
            "asserted_in_last_frame": asserted_at_end,
            "last_assertion_cleared": last_assertion_cleared,
            "last_assertion_start_device_time": (
                self._last_assertion_start.as_dict(first_unwrapped_time_ms)
                if self._last_assertion_start is not None
                else None
            ),
            "last_assertion_clear_device_time": (
                self._last_assertion_clear.as_dict(first_unwrapped_time_ms)
                if last_assertion_cleared
                else None
            ),
        }


class HI91CaptureStats:
    """Aggregate frame semantics independently from serial transport."""

    def __init__(self) -> None:
        self.frame_count = 0
        self.status_counts = {name: 0 for name, _ in STATUS_BITS}
        self.status_distribution: Counter = Counter()
        self.valid_counts = {
            name: {"valid": 0, "invalid": 0} for name in VALID_BITS
        }
        self.zero_triplets = {
            name: {"valid": 0, "invalid": 0}
            for name in ("accel", "gyro", "euler")
        }
        self.nonfinite_frames = 0
        self.nonfinite_values = 0
        self.nonfinite_by_field = {
            name: 0
            for name in (
                "air_pressure",
                "accel",
                "gyro",
                "magnetic_field",
                "euler",
                "quaternion",
            )
        }
        self.timestamp_regressions = 0
        self.timestamp_wraps = 0
        self.timestamp_duplicate_frames = 0
        self._previous_timestamp: Optional[int] = None
        self._extended_timestamp = 0
        self._first_extended_timestamp: Optional[int] = None
        self._first_valid_times: Dict[str, Optional[_DeviceEventTime]] = {
            name: None for name in VALID_BITS
        }
        self._inverse_convergence = {
            "wb_conv_bit3": _InverseConvergenceRun(
                "gyro bias is not converged while this bit is asserted"
            ),
            "att_conv_bit7": _InverseConvergenceRun(
                "attitude estimate is not converged while this bit is asserted"
            ),
        }
        self._dropouts = {name: _DropoutRun() for name in VALID_BITS}

    def _update_timestamp(self, raw_timestamp: int) -> int:
        if self._previous_timestamp is None:
            self._extended_timestamp = raw_timestamp
            self._previous_timestamp = raw_timestamp
            return self._extended_timestamp

        delta = (raw_timestamp - self._previous_timestamp) & UINT32_MASK
        if delta == 0:
            self.timestamp_duplicate_frames += 1
        elif delta < UINT32_HALF_RANGE:
            if raw_timestamp < self._previous_timestamp:
                self.timestamp_wraps += 1
            self._extended_timestamp += delta
        else:
            self.timestamp_regressions += 1
        self._previous_timestamp = raw_timestamp
        return self._extended_timestamp

    def _record_nonfinite(self, frame: HI91Frame) -> None:
        values = {
            "air_pressure": (frame.air_pressure_pa,),
            "accel": frame.acceleration_g,
            "gyro": frame.angular_rate_deg_s,
            "magnetic_field": frame.magnetic_field_ut,
            "euler": frame.euler_deg,
            "quaternion": frame.quaternion,
        }
        frame_has_nonfinite = False
        for name, field_values in values.items():
            count = sum(not math.isfinite(value) for value in field_values)
            self.nonfinite_by_field[name] += count
            self.nonfinite_values += count
            frame_has_nonfinite = frame_has_nonfinite or count != 0
        if frame_has_nonfinite:
            self.nonfinite_frames += 1

    def observe(self, frame: HI91Frame) -> None:
        self.frame_count += 1
        timestamp_ms = self._update_timestamp(frame.system_time_ms)
        if self._first_extended_timestamp is None:
            self._first_extended_timestamp = timestamp_ms
        event_time = _DeviceEventTime(frame.system_time_ms, timestamp_ms)
        self.status_distribution[frame.status] += 1

        for name, bit in STATUS_BITS:
            if frame.status & bit:
                self.status_counts[name] += 1

        valid = {}
        for name, bit in VALID_BITS.items():
            is_valid = bool(frame.status & bit)
            valid[name] = is_valid
            key = "valid" if is_valid else "invalid"
            self.valid_counts[name][key] += 1
            self._dropouts[name].observe(is_valid, timestamp_ms)
            if is_valid and self._first_valid_times[name] is None:
                self._first_valid_times[name] = event_time

        self._inverse_convergence["wb_conv_bit3"].observe(
            bool(frame.status & (1 << 3)), event_time
        )
        self._inverse_convergence["att_conv_bit7"].observe(
            bool(frame.status & (1 << 7)), event_time
        )

        vectors = {
            "accel": (frame.acceleration_g, valid["accel"]),
            "gyro": (frame.angular_rate_deg_s, valid["gyro"]),
            "euler": (frame.euler_deg, valid["attitude"]),
        }
        for name, (values, is_valid) in vectors.items():
            if all(value == 0.0 for value in values):
                key = "valid" if is_valid else "invalid"
                self.zero_triplets[name][key] += 1

        self._record_nonfinite(frame)

    def summary(
        self,
        capture_duration_s: float,
        parser: Optional[HI91StreamParser] = None,
    ) -> Dict[str, object]:
        parser = parser or HI91StreamParser()
        frame_rate = (
            self.frame_count / capture_duration_s
            if capture_duration_s > 0.0
            else 0.0
        )
        bmi_fault = self.status_counts["bmi_fault"]
        icm_fault = self.status_counts["icm_fault"]
        fault_any = sum(
            count
            for status, count in self.status_distribution.items()
            if status & ((1 << 13) | (1 << 14))
        )
        distribution = {
            "0x{:04X}".format(status): self.status_distribution[status]
            for status in sorted(self.status_distribution)
        }
        return {
            "capture_duration_s": capture_duration_s,
            "frames": self.frame_count,
            "frame_rate_hz": frame_rate,
            "crc_errors": parser.crc_errors,
            "length_errors": parser.length_errors,
            "tag_errors": parser.tag_errors,
            "resync_bytes": parser.resync_bytes,
            "trailing_bytes": parser.trailing_bytes + parser.buffered_bytes,
            "timestamp_regressions": self.timestamp_regressions,
            "timestamp_wraps": self.timestamp_wraps,
            "timestamp_duplicate_frames": self.timestamp_duplicate_frames,
            "device_event_time_semantics": {
                "observation_scope": "successfully decoded frames in this capture",
                "raw_system_time_ms": "uint32 system_time_ms from the event frame",
                "unwrapped_system_time_ms": (
                    "capture-local uint32-unwrapped device time"
                ),
                "elapsed_since_first_frame_ms": (
                    "unwrapped device time relative to the first decoded frame"
                ),
                "inverse_convergence_runs": (
                    "the last asserted run observed in this capture; a run "
                    "already asserted in the first frame is left-censored"
                ),
            },
            "first_valid_device_times": {
                name: {
                    "observed": event_time is not None,
                    "device_time": (
                        event_time.as_dict(self._first_extended_timestamp)
                        if event_time is not None
                        else None
                    ),
                }
                for name, event_time in self._first_valid_times.items()
            },
            "inverse_convergence_clearance": {
                name: run.as_dict(self._first_extended_timestamp)
                for name, run in self._inverse_convergence.items()
            },
            "valid_counts": self.valid_counts,
            "status_counts": self.status_counts,
            "fault_counts": {
                "bmi_fault_frames": bmi_fault,
                "icm_fault_frames": icm_fault,
                "any_fault_frames": fault_any,
            },
            "acc_saturation_frames": self.status_counts[
                "acc_saturation_recent"
            ],
            "gyro_saturation_frames": self.status_counts[
                "gyro_saturation_recent"
            ],
            "sout_pulse_frames": self.status_counts["sout_pulse"],
            "stream_drop_frames": self.status_counts["stream_drop"],
            "zero_triplets": self.zero_triplets,
            "nonfinite": {
                "frames": self.nonfinite_frames,
                "values": self.nonfinite_values,
                "by_field": self.nonfinite_by_field,
            },
            "status_distribution": distribution,
            "valid_dropouts": {
                name: dropout.as_dict()
                for name, dropout in self._dropouts.items()
            },
        }


CSV_FIELDS = (
    "host_elapsed_s",
    "frame_index",
    "system_time_ms",
    "status",
    "status_hex",
    "temperature_c",
    "accel_x_g",
    "accel_y_g",
    "accel_z_g",
    "gyro_x_deg_s",
    "gyro_y_deg_s",
    "gyro_z_deg_s",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "quat_w",
    "quat_x",
    "quat_y",
    "quat_z",
)


def frame_to_csv_row(
    frame: HI91Frame, frame_index: int, host_elapsed_s: float
) -> Dict[str, object]:
    return {
        "host_elapsed_s": "{:.9f}".format(host_elapsed_s),
        "frame_index": frame_index,
        "system_time_ms": frame.system_time_ms,
        "status": frame.status,
        "status_hex": "0x{:04X}".format(frame.status),
        "temperature_c": frame.temperature_c,
        "accel_x_g": frame.acceleration_g[0],
        "accel_y_g": frame.acceleration_g[1],
        "accel_z_g": frame.acceleration_g[2],
        "gyro_x_deg_s": frame.angular_rate_deg_s[0],
        "gyro_y_deg_s": frame.angular_rate_deg_s[1],
        "gyro_z_deg_s": frame.angular_rate_deg_s[2],
        "roll_deg": frame.euler_deg[0],
        "pitch_deg": frame.euler_deg[1],
        "yaw_deg": frame.euler_deg[2],
        "quat_w": frame.quaternion[0],
        "quat_x": frame.quaternion[1],
        "quat_y": frame.quaternion[2],
        "quat_z": frame.quaternion[3],
    }


def open_capture_serial(port: str, baud: int):
    """Open and configure one pyserial fd without transmitting data."""
    try:
        import serial
    except ImportError as error:
        raise RuntimeError(
            "pyserial is required; install it with 'python3 -m pip install pyserial'"
        ) from error

    device = serial.Serial()
    device.port = port
    device.baudrate = baud
    device.timeout = 0.05
    device.write_timeout = 0
    device.dtr = False
    device.open()
    try:
        fd = device.fileno()
        if not os.isatty(fd):
            raise RuntimeError("capture port is not a TTY")
        tty.setraw(fd, when=termios.TCSANOW)
        device.dtr = True
        device.reset_input_buffer()
    except Exception:
        device.close()
        raise
    return device


def capture(
    port: str,
    seconds: float,
    baud: int,
    csv_path: Optional[str] = None,
) -> Dict[str, object]:
    parser = HI91StreamParser()
    stats = HI91CaptureStats()
    device = open_capture_serial(port, baud)
    csv_file = None
    try:
        writer = None
        if csv_path is not None:
            csv_file = open(csv_path, "w", newline="", encoding="utf-8")
            writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
            writer.writeheader()

        start = time.monotonic()
        deadline = start + seconds
        while time.monotonic() < deadline:
            chunk = device.read(4096)
            observed_at = time.monotonic()
            for frame in parser.feed(chunk):
                stats.observe(frame)
                if writer is not None:
                    writer.writerow(
                        frame_to_csv_row(
                            frame, stats.frame_count, observed_at - start
                        )
                    )
        elapsed = time.monotonic() - start
        parser.finish()
        return stats.summary(elapsed, parser)
    finally:
        if csv_file is not None:
            csv_file.close()
        try:
            device.dtr = False
        except Exception:
            pass
        device.close()


def _resolved_path(path: str) -> pathlib.Path:
    return pathlib.Path(path).expanduser().resolve()


def _prepare_summary_target(path: str) -> pathlib.Path:
    """Remove a stale result and verify its directory before capture starts."""
    target = _resolved_path(path)
    target.unlink(missing_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        dir=str(target.parent),
        prefix=".{}.probe-".format(target.name),
        delete=True,
    ):
        pass
    return target


def _atomic_write_summary(target: pathlib.Path, rendered_json: str) -> None:
    temporary_path: Optional[pathlib.Path] = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            newline="\n",
            dir=str(target.parent),
            prefix=".{}.tmp-".format(target.name),
            delete=False,
        ) as temporary:
            temporary_path = pathlib.Path(temporary.name)
            temporary.write(rendered_json)
            temporary.write("\n")
            temporary.flush()
            os.fsync(temporary.fileno())
        os.replace(str(temporary_path), str(target))
        temporary_path = None

        directory_flags = os.O_RDONLY
        if hasattr(os, "O_DIRECTORY"):
            directory_flags |= os.O_DIRECTORY
        try:
            directory_fd = os.open(str(target.parent), directory_flags)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        except OSError:
            pass
    finally:
        if temporary_path is not None:
            try:
                temporary_path.unlink(missing_ok=True)
            except OSError:
                pass


def _positive_float(value: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise argparse.ArgumentTypeError("must be a finite value greater than zero")
    return result


def _positive_int(value: str) -> int:
    result = int(value, 10)
    if result <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return result


def build_argument_parser() -> argparse.ArgumentParser:
    argument_parser = argparse.ArgumentParser(
        description="Capture and validate the fixed 82-byte HI91 USB stream."
    )
    argument_parser.add_argument("--port", required=True, help="CDC TTY path")
    argument_parser.add_argument(
        "--seconds",
        type=_positive_float,
        default=10.0,
        help="capture duration (default: 10)",
    )
    argument_parser.add_argument(
        "--baud",
        type=_positive_int,
        default=115200,
        help="TTY line-coding placeholder; USB CDC payload rate is unaffected",
    )
    argument_parser.add_argument(
        "--csv", metavar="PATH", help="optionally write every decoded frame"
    )
    argument_parser.add_argument(
        "--summary-json",
        metavar="PATH",
        help=(
            "atomically write the completed capture summary as JSON; stale "
            "output is removed before capture"
        ),
    )
    return argument_parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_argument_parser().parse_args(argv)
    try:
        if (
            args.summary_json is not None
            and args.csv is not None
            and _resolved_path(args.summary_json) == _resolved_path(args.csv)
        ):
            raise ValueError("--summary-json and --csv must use different paths")
        summary_target = (
            _prepare_summary_target(args.summary_json)
            if args.summary_json is not None
            else None
        )
        summary = capture(args.port, args.seconds, args.baud, args.csv)
        rendered_summary = json.dumps(
            summary, indent=2, sort_keys=True, allow_nan=False
        )
        if summary_target is not None:
            _atomic_write_summary(summary_target, rendered_summary)
    except (OSError, RuntimeError, ValueError) as error:
        print("hi91_capture: {}".format(error), file=sys.stderr)
        return 1
    print(rendered_summary)
    return 0


if __name__ == "__main__":
    sys.exit(main())
