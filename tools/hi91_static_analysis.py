#!/usr/bin/env python3
"""Streaming offline analysis for CSV files produced by hi91_capture.py."""

import argparse
import csv
import json
import math
import pathlib
import sys
from array import array
from collections import Counter, deque
from dataclasses import dataclass
from typing import Dict, Optional, Sequence


UINT32_MASK = 0xFFFFFFFF
UINT32_HALF_RANGE = 0x80000000
DAY_MILLISECONDS = 24 * 60 * 60 * 1000

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

REQUIRED_COLUMNS = (
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

EULER_AXES = ("roll", "pitch", "yaw")
GYRO_AXES = ("x", "y", "z")


class AnalysisError(ValueError):
    """Raised when a capture CSV does not satisfy the expected schema."""


class _FrameIndexStats:
    def __init__(self) -> None:
        self.first: Optional[int] = None
        self.last: Optional[int] = None
        self.gap_events = 0
        self.missing_frames = 0
        self.duplicate_indices = 0
        self.regressions = 0

    def observe(self, value: int) -> None:
        if self.first is None:
            self.first = value
            self.last = value
            return

        assert self.last is not None
        delta = value - self.last
        if delta == 0:
            self.duplicate_indices += 1
        elif delta < 0:
            self.regressions += 1
        elif delta > 1:
            self.gap_events += 1
            self.missing_frames += delta - 1
        self.last = value

    def summary(self) -> Dict[str, object]:
        sequence_contiguous = (
            self.gap_events == 0
            and self.duplicate_indices == 0
            and self.regressions == 0
        )
        return {
            "first": self.first,
            "last": self.last,
            "starts_at_one": self.first == 1,
            "sequence_contiguous": sequence_contiguous,
            "capture_contiguous_from_one": sequence_contiguous
            and self.first == 1,
            "gap_events": self.gap_events,
            "missing_frames": self.missing_frames,
            "duplicate_indices": self.duplicate_indices,
            "regressions": self.regressions,
        }


class _HostTimeStats:
    def __init__(self) -> None:
        self.first: Optional[float] = None
        self.last: Optional[float] = None
        self.maximum: Optional[float] = None
        self.duplicates = 0
        self.regressions = 0

    def observe(self, value: float) -> None:
        if self.first is None:
            self.first = value
            self.last = value
            self.maximum = value
            return

        assert self.last is not None
        assert self.maximum is not None
        if value == self.last:
            self.duplicates += 1
        elif value < self.last:
            self.regressions += 1
        self.last = value
        self.maximum = max(self.maximum, value)

    def summary(self, rows: int) -> Dict[str, object]:
        span_s = 0.0
        if self.first is not None and self.maximum is not None:
            span_s = max(0.0, self.maximum - self.first)
        capture_elapsed_s = self.maximum if self.maximum is not None else 0.0
        frame_rate_hz = (
            rows / capture_elapsed_s
            if rows > 0 and capture_elapsed_s > 0.0
            else None
        )
        observed_interval_frame_rate_hz = (
            (rows - 1) / span_s
            if rows > 1 and span_s > 0.0
            else None
        )
        return {
            "first_s": self.first,
            "last_s": self.last,
            "maximum_s": self.maximum,
            "capture_elapsed_s": capture_elapsed_s,
            "span_s": span_s,
            "duplicate_frames": self.duplicates,
            "regressions": self.regressions,
            "average_frame_rate_hz": frame_rate_hz,
            "observed_interval_frame_rate_hz": (
                observed_interval_frame_rate_hz
            ),
        }


class _DeviceTimeStats:
    """Maintain a trusted daily/uint32 timeline without advancing on regressions."""

    def __init__(self) -> None:
        self.first_raw: Optional[int] = None
        self.last_raw: Optional[int] = None
        self._trusted_raw: Optional[int] = None
        self.elapsed_ms = 0
        self.duplicates = 0
        self.regressions = 0
        self.wraps = 0
        self.day_wraps = 0
        self.uint32_wraps = 0
        self.frames_behind_trusted_anchor = 0

    @staticmethod
    def _forward_delta(previous: int, current: int):
        if current == previous:
            return 0, None
        if previous < DAY_MILLISECONDS and current < DAY_MILLISECONDS:
            if current > previous:
                return current - previous, None
            delta = current + DAY_MILLISECONDS - previous
            if delta < DAY_MILLISECONDS // 2:
                return delta, "day"
            return None, None

        delta = (current - previous) & UINT32_MASK
        if delta < UINT32_HALF_RANGE:
            return delta, "uint32" if current < previous else None
        return None, None

    def observe(self, raw: int) -> int:
        if self.first_raw is None:
            self.first_raw = raw
            self.last_raw = raw
            self._trusted_raw = raw
            return 0

        assert self.last_raw is not None
        assert self._trusted_raw is not None
        consecutive_delta, _ = self._forward_delta(self.last_raw, raw)
        if consecutive_delta == 0:
            self.duplicates += 1
        elif consecutive_delta is None:
            self.regressions += 1
        self.last_raw = raw

        trusted_delta, wrap_kind = self._forward_delta(self._trusted_raw, raw)
        if trusted_delta == 0:
            return self.elapsed_ms
        if trusted_delta is None:
            self.frames_behind_trusted_anchor += 1
            return self.elapsed_ms

        if wrap_kind is not None:
            self.wraps += 1
            if wrap_kind == "day":
                self.day_wraps += 1
            else:
                self.uint32_wraps += 1
        self.elapsed_ms += trusted_delta
        self._trusted_raw = raw
        return self.elapsed_ms

    def summary(self, rows: int) -> Dict[str, object]:
        duration_s = self.elapsed_ms / 1000.0
        frame_rate_hz = (
            (rows - 1) / duration_s
            if rows > 1 and duration_s > 0.0
            else None
        )
        return {
            "first_raw_ms": self.first_raw,
            "last_raw_ms": self.last_raw,
            "unwrapped_elapsed_ms": self.elapsed_ms,
            "duration_s": duration_s,
            "duplicate_frames": self.duplicates,
            "regressions": self.regressions,
            "wraps": self.wraps,
            "day_wraps": self.day_wraps,
            "uint32_wraps": self.uint32_wraps,
            "frames_behind_trusted_anchor": self.frames_behind_trusted_anchor,
            "average_frame_rate_hz": frame_rate_hz,
        }


class _OnlineRegression:
    def __init__(self) -> None:
        self.samples = 0
        self.mean_x = 0.0
        self.mean_y = 0.0
        self.sxx = 0.0
        self.sxy = 0.0
        self.syy = 0.0
        self.first_x: Optional[float] = None
        self.last_x: Optional[float] = None
        self.minimum_y: Optional[float] = None
        self.maximum_y: Optional[float] = None

    def observe(self, x: float, y: float) -> None:
        self.samples += 1
        if self.first_x is None:
            self.first_x = x
        self.last_x = x
        delta_x = x - self.mean_x
        delta_y = y - self.mean_y
        self.mean_x += delta_x / self.samples
        self.mean_y += delta_y / self.samples
        self.sxx += delta_x * (x - self.mean_x)
        self.sxy += delta_x * (y - self.mean_y)
        self.syy += delta_y * (y - self.mean_y)
        self.minimum_y = y if self.minimum_y is None else min(self.minimum_y, y)
        self.maximum_y = y if self.maximum_y is None else max(self.maximum_y, y)

    def summary(self) -> Dict[str, object]:
        slope_deg_per_hour = None
        if self.samples >= 2 and self.sxx > 0.0:
            slope_deg_per_hour = (self.sxy / self.sxx) * 3600.0
        standard_deviation_deg = None
        peak_to_peak_deg = None
        residual_rms_deg = None
        if self.samples:
            standard_deviation_deg = math.sqrt(
                max(0.0, self.syy / self.samples)
            )
            assert self.minimum_y is not None
            assert self.maximum_y is not None
            peak_to_peak_deg = self.maximum_y - self.minimum_y
            if self.samples == 1:
                residual_rms_deg = 0.0
            elif self.sxx > 0.0:
                residual_sum_squares = max(
                    0.0,
                    self.syy - self.sxy * (self.sxy / self.sxx),
                )
                residual_rms_deg = math.sqrt(
                    residual_sum_squares / self.samples
                )
        duration_s = 0.0
        if self.first_x is not None and self.last_x is not None:
            duration_s = max(0.0, self.last_x - self.first_x)
        return {
            "samples": self.samples,
            "duration_s": duration_s,
            "mean_unwrapped_deg": self.mean_y if self.samples else None,
            "standard_deviation_deg": standard_deviation_deg,
            "minimum_unwrapped_deg": self.minimum_y,
            "maximum_unwrapped_deg": self.maximum_y,
            "peak_to_peak_deg": peak_to_peak_deg,
            "linear_trend_deg_per_hour": slope_deg_per_hour,
            "linear_fit_residual_rms_deg": residual_rms_deg,
        }


class _EulerAxisStats:
    def __init__(self) -> None:
        self._previous_raw: Optional[float] = None
        self._unwrapped = 0.0
        self.full = _OnlineRegression()
        self.post_burn_in = _OnlineRegression()
        self.invalid_samples = 0
        self.nonfinite_samples = 0

    def observe(
        self,
        raw_deg: float,
        elapsed_s: float,
        burn_in_s: float,
        attitude_valid: bool,
    ) -> float:
        if not math.isfinite(raw_deg):
            self.nonfinite_samples += 1
            return math.nan
        if not attitude_valid:
            self.invalid_samples += 1
            return math.nan

        if self._previous_raw is None:
            self._unwrapped = raw_deg
        else:
            delta = math.remainder(raw_deg - self._previous_raw, 360.0)
            self._unwrapped += delta
        self._previous_raw = raw_deg

        self.full.observe(elapsed_s, self._unwrapped)
        if elapsed_s >= burn_in_s:
            self.post_burn_in.observe(elapsed_s, self._unwrapped)
        return self._unwrapped


class _OnlineMoments:
    def __init__(self) -> None:
        self.samples = 0
        self.mean = 0.0
        self.m2 = 0.0

    def observe(self, value: float) -> None:
        self.samples += 1
        delta = value - self.mean
        self.mean += delta / self.samples
        self.m2 += delta * (value - self.mean)

    def summary(self) -> Dict[str, object]:
        if self.samples == 0:
            return {
                "samples": 0,
                "mean": None,
                "rms": None,
                "standard_deviation": None,
            }
        variance = max(0.0, self.m2 / self.samples)
        standard_deviation = math.sqrt(variance)
        return {
            "samples": self.samples,
            "mean": self.mean,
            "rms": math.hypot(self.mean, standard_deviation),
            "standard_deviation": standard_deviation,
        }


class _WindowSums:
    def __init__(self, axes: int) -> None:
        self.sums = [0.0] * axes
        self.counts = [0] * axes

    def add(self, values: Sequence[float]) -> None:
        for axis, value in enumerate(values):
            if math.isfinite(value):
                self.sums[axis] += value
                self.counts[axis] += 1

    def mean(self, axis: int) -> Optional[float]:
        if self.counts[axis] == 0:
            return None
        return self.sums[axis] / self.counts[axis]


@dataclass
class _WindowBlock:
    values: array
    first_record: int = 0


class _TailWindow:
    """A bounded window using one aggregate per retained device millisecond."""

    _AXES = 3
    _SUM_OFFSET = 1
    _COUNT_OFFSET = _SUM_OFFSET + _AXES
    _VALUES_PER_RECORD = 1 + 2 * _AXES
    _BLOCK_RECORDS = 2048

    def __init__(self, width_s: float) -> None:
        self.width_s = width_s
        self.blocks = deque()  # type: ignore[var-annotated]
        self.sums = _WindowSums(self._AXES)

    def append(self, elapsed_s: float, values: Sequence[float]) -> None:
        if self.blocks:
            block = self.blocks[-1]
            last_offset = len(block.values) - self._VALUES_PER_RECORD
            if block.values[last_offset] == elapsed_s:
                self._add_to_record(block.values, last_offset, values)
                self.sums.add(values)
                self._trim(elapsed_s - self.width_s)
                return

        if (
            not self.blocks
            or len(self.blocks[-1].values)
            >= self._BLOCK_RECORDS * self._VALUES_PER_RECORD
        ):
            self.blocks.append(_WindowBlock(array("d")))
        block = self.blocks[-1]
        block.values.append(elapsed_s)
        block.values.extend((0.0,) * (2 * self._AXES))
        offset = len(block.values) - self._VALUES_PER_RECORD
        self._add_to_record(block.values, offset, values)
        self.sums.add(values)
        self._trim(elapsed_s - self.width_s)

    def _add_to_record(
        self, record: array, offset: int, values: Sequence[float]
    ) -> None:
        for axis, value in enumerate(values):
            if not math.isfinite(value):
                continue
            record[offset + self._SUM_OFFSET + axis] += value
            record[offset + self._COUNT_OFFSET + axis] += 1.0

    def _trim(self, cutoff_s: float) -> None:
        while self.blocks:
            block = self.blocks[0]
            record_count = len(block.values) // self._VALUES_PER_RECORD
            while block.first_record < record_count:
                offset = block.first_record * self._VALUES_PER_RECORD
                if block.values[offset] >= cutoff_s:
                    return
                for axis in range(self._AXES):
                    self.sums.sums[axis] -= block.values[
                        offset + self._SUM_OFFSET + axis
                    ]
                    self.sums.counts[axis] -= int(
                        block.values[offset + self._COUNT_OFFSET + axis]
                    )
                block.first_record += 1
            self.blocks.popleft()

    def sums_at_or_after(self, cutoff_s: float) -> _WindowSums:
        """Aggregate the retained tail without allocating another tail window."""
        result = _WindowSums(self._AXES)
        for block in self.blocks:
            record_count = len(block.values) // self._VALUES_PER_RECORD
            for record_index in range(block.first_record, record_count):
                offset = record_index * self._VALUES_PER_RECORD
                if block.values[offset] < cutoff_s:
                    continue
                for axis in range(self._AXES):
                    result.sums[axis] += block.values[
                        offset + self._SUM_OFFSET + axis
                    ]
                    result.counts[axis] += int(
                        block.values[offset + self._COUNT_OFFSET + axis]
                    )
        return result


@dataclass
class _RunAggregate:
    count: int = 0
    total_frames: int = 0
    total_endpoint_span_s: float = 0.0
    single_frame_runs: int = 0
    shortest_frames: Optional[int] = None
    longest_frames: int = 0
    shortest_endpoint_span_s: Optional[float] = None
    longest_endpoint_span_s: float = 0.0

    def add(self, frames: int, start_s: float, end_s: float) -> None:
        duration_s = max(0.0, end_s - start_s)
        self.count += 1
        self.total_frames += frames
        self.total_endpoint_span_s += duration_s
        if frames == 1:
            self.single_frame_runs += 1
        if self.shortest_frames is None or frames < self.shortest_frames:
            self.shortest_frames = frames
        self.longest_frames = max(self.longest_frames, frames)
        if (
            self.shortest_endpoint_span_s is None
            or duration_s < self.shortest_endpoint_span_s
        ):
            self.shortest_endpoint_span_s = duration_s
        self.longest_endpoint_span_s = max(
            self.longest_endpoint_span_s, duration_s
        )

    def summary(self, total_rows: int) -> Dict[str, object]:
        return {
            "count": self.count,
            "total_frames": self.total_frames,
            "frame_fraction": (
                self.total_frames / total_rows if total_rows else 0.0
            ),
            "single_frame_runs": self.single_frame_runs,
            "shortest_frames": self.shortest_frames,
            "longest_frames": self.longest_frames if self.count else None,
            "total_endpoint_span_s": self.total_endpoint_span_s,
            "shortest_endpoint_span_s": self.shortest_endpoint_span_s,
            "longest_endpoint_span_s": (
                self.longest_endpoint_span_s if self.count else None
            ),
        }


class _BinaryRunStats:
    def __init__(self) -> None:
        self.current: Optional[bool] = None
        self.current_frames = 0
        self.start_s = 0.0
        self.last_s = 0.0
        self.transitions = 0
        self.aggregates = {False: _RunAggregate(), True: _RunAggregate()}

    def observe(self, state: bool, elapsed_s: float) -> None:
        if self.current is None:
            self._start(state, elapsed_s)
            return
        if state == self.current:
            self.current_frames += 1
            self.last_s = elapsed_s
            return
        self._finish_current()
        self.transitions += 1
        self._start(state, elapsed_s)

    def _start(self, state: bool, elapsed_s: float) -> None:
        self.current = state
        self.current_frames = 1
        self.start_s = elapsed_s
        self.last_s = elapsed_s

    def _finish_current(self) -> None:
        if self.current is None:
            return
        self.aggregates[self.current].add(
            self.current_frames, self.start_s, self.last_s
        )

    def finish(self) -> None:
        self._finish_current()
        self.current = None


class _QuaternionNormStats:
    def __init__(self) -> None:
        self.samples = 0
        self.nonfinite_samples = 0
        self.max_abs_error = 0.0
        self.sum_squared_error = 0.0

    def observe(self, values: Sequence[float]) -> None:
        if not all(math.isfinite(value) for value in values):
            self.nonfinite_samples += 1
            return
        norm = math.hypot(*values)
        if not math.isfinite(norm):
            self.nonfinite_samples += 1
            return
        error = norm - 1.0
        self.samples += 1
        self.max_abs_error = max(self.max_abs_error, abs(error))
        self.sum_squared_error += error * error

    def summary(self) -> Dict[str, object]:
        return {
            "samples": self.samples,
            "nonfinite_samples": self.nonfinite_samples,
            "max_abs_error": self.max_abs_error if self.samples else None,
            "rms_error": (
                math.sqrt(self.sum_squared_error / self.samples)
                if self.samples
                else None
            ),
        }


def _parse_int(text: str, column: str, row_number: int) -> int:
    try:
        return int(text.strip(), 10)
    except ValueError as error:
        raise AnalysisError(
            "row {} column {!r} is not a decimal integer: {!r}".format(
                row_number, column, text
            )
        ) from error


def _parse_float(text: str, column: str, row_number: int) -> float:
    try:
        return float(text.strip())
    except ValueError as error:
        raise AnalysisError(
            "row {} column {!r} is not a number: {!r}".format(
                row_number, column, text
            )
        ) from error


def _finite_nonnegative_float(value: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result < 0.0:
        raise argparse.ArgumentTypeError("must be finite and non-negative")
    return result


def _finite_positive_float(value: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise argparse.ArgumentTypeError("must be finite and greater than zero")
    return result


def _window_summary(
    axis: int, first: _WindowSums, last: _WindowSums
) -> Dict[str, object]:
    first_mean = first.mean(axis)
    last_mean = last.mean(axis)
    difference = None
    if first_mean is not None and last_mean is not None:
        difference = last_mean - first_mean
    return {
        "first_samples": first.counts[axis],
        "first_mean_unwrapped_deg": first_mean,
        "last_samples": last.counts[axis],
        "last_mean_unwrapped_deg": last_mean,
        "last_minus_first_mean_deg": difference,
    }


def analyze_csv(
    csv_path: str, window_seconds: float = 300.0, burn_in_seconds: float = 0.0
) -> Dict[str, object]:
    """Analyze a capture CSV in one pass with memory bounded by the tail window."""
    if not math.isfinite(window_seconds) or window_seconds <= 0.0:
        raise ValueError("window_seconds must be finite and greater than zero")
    if not math.isfinite(burn_in_seconds) or burn_in_seconds < 0.0:
        raise ValueError("burn_in_seconds must be finite and non-negative")

    frame_indices = _FrameIndexStats()
    host_time = _HostTimeStats()
    device_time = _DeviceTimeStats()
    status_distribution: Counter = Counter()
    zero_triplets = {
        name: {"valid": 0, "invalid": 0}
        for name in ("accel", "gyro", "euler")
    }
    nonfinite_by_field = {
        name: 0 for name in ("accel", "gyro", "euler", "quaternion")
    }
    nonfinite_frames = 0
    nonfinite_values = 0
    quaternion_norm = _QuaternionNormStats()
    euler_axes = [_EulerAxisStats() for _ in EULER_AXES]
    gyro_full = [_OnlineMoments() for _ in GYRO_AXES]
    gyro_post_burn_in = [_OnlineMoments() for _ in GYRO_AXES]
    first_window = _WindowSums(len(EULER_AXES))
    post_burn_in_first_window = _WindowSums(len(EULER_AXES))
    tail_window = _TailWindow(window_seconds)
    static_runs = _BinaryRunStats()
    temperature_min: Optional[int] = None
    temperature_max: Optional[int] = None
    rows = 0

    path = pathlib.Path(csv_path)
    with path.open("r", newline="", encoding="utf-8-sig") as csv_file:
        reader = csv.reader(csv_file, strict=True)
        try:
            header = next(reader)
        except StopIteration as error:
            raise AnalysisError("CSV is empty and has no header") from error

        duplicate_columns = sorted(
            name for name, count in Counter(header).items() if count > 1
        )
        if duplicate_columns:
            raise AnalysisError(
                "CSV header contains duplicate columns: {}".format(
                    ", ".join(duplicate_columns)
                )
            )
        missing_columns = [
            column for column in REQUIRED_COLUMNS if column not in header
        ]
        if missing_columns:
            raise AnalysisError(
                "CSV header is missing required columns: {}".format(
                    ", ".join(missing_columns)
                )
            )
        column = {name: header.index(name) for name in REQUIRED_COLUMNS}

        for row_number, row in enumerate(reader, start=2):
            if len(row) != len(header):
                raise AnalysisError(
                    "row {} has {} columns; header has {}".format(
                        row_number, len(row), len(header)
                    )
                )

            host_elapsed_s = _parse_float(
                row[column["host_elapsed_s"]], "host_elapsed_s", row_number
            )
            if not math.isfinite(host_elapsed_s) or host_elapsed_s < 0.0:
                raise AnalysisError(
                    "row {} host_elapsed_s must be finite and non-negative".format(
                        row_number
                    )
                )
            frame_index = _parse_int(
                row[column["frame_index"]], "frame_index", row_number
            )
            if frame_index < 1:
                raise AnalysisError(
                    "row {} frame_index must be positive".format(row_number)
                )
            system_time_ms = _parse_int(
                row[column["system_time_ms"]], "system_time_ms", row_number
            )
            if not 0 <= system_time_ms <= UINT32_MASK:
                raise AnalysisError(
                    "row {} system_time_ms is outside uint32 range".format(
                        row_number
                    )
                )
            status = _parse_int(row[column["status"]], "status", row_number)
            if not 0 <= status <= 0xFFFF:
                raise AnalysisError(
                    "row {} status is outside uint16 range".format(row_number)
                )
            try:
                status_hex = int(row[column["status_hex"]].strip(), 0)
            except ValueError as error:
                raise AnalysisError(
                    "row {} column 'status_hex' is not an integer: {!r}".format(
                        row_number, row[column["status_hex"]]
                    )
                ) from error
            if status_hex != status:
                raise AnalysisError(
                    "row {} status/status_hex mismatch: {} vs {!r}".format(
                        row_number, status, row[column["status_hex"]]
                    )
                )
            temperature = _parse_int(
                row[column["temperature_c"]], "temperature_c", row_number
            )
            if not -128 <= temperature <= 127:
                raise AnalysisError(
                    "row {} temperature_c is outside int8 range".format(
                        row_number
                    )
                )

            accel = tuple(
                _parse_float(row[column[name]], name, row_number)
                for name in ("accel_x_g", "accel_y_g", "accel_z_g")
            )
            gyro = tuple(
                _parse_float(row[column[name]], name, row_number)
                for name in (
                    "gyro_x_deg_s",
                    "gyro_y_deg_s",
                    "gyro_z_deg_s",
                )
            )
            euler = tuple(
                _parse_float(row[column[name]], name, row_number)
                for name in ("roll_deg", "pitch_deg", "yaw_deg")
            )
            quaternion = tuple(
                _parse_float(row[column[name]], name, row_number)
                for name in ("quat_w", "quat_x", "quat_y", "quat_z")
            )

            rows += 1
            frame_indices.observe(frame_index)
            host_time.observe(host_elapsed_s)
            elapsed_s = device_time.observe(system_time_ms) / 1000.0
            status_distribution[status] += 1
            temperature_min = (
                temperature
                if temperature_min is None
                else min(temperature_min, temperature)
            )
            temperature_max = (
                temperature
                if temperature_max is None
                else max(temperature_max, temperature)
            )

            valid = {
                "accel": bool(status & (1 << 1)),
                "gyro": bool(status & (1 << 2)),
                "euler": bool(status & (1 << 0)),
            }
            vectors = {
                "accel": accel,
                "gyro": gyro,
                "euler": euler,
                "quaternion": quaternion,
            }
            frame_has_nonfinite = False
            for name, values in vectors.items():
                count = sum(not math.isfinite(value) for value in values)
                nonfinite_by_field[name] += count
                nonfinite_values += count
                frame_has_nonfinite = frame_has_nonfinite or count != 0
            if frame_has_nonfinite:
                nonfinite_frames += 1

            for name in ("accel", "gyro", "euler"):
                if all(value == 0.0 for value in vectors[name]):
                    key = "valid" if valid[name] else "invalid"
                    zero_triplets[name][key] += 1

            quaternion_norm.observe(quaternion)
            unwrapped_euler = tuple(
                euler_axes[axis].observe(
                    euler[axis],
                    elapsed_s,
                    burn_in_seconds,
                    valid["euler"],
                )
                for axis in range(len(EULER_AXES))
            )
            if elapsed_s <= window_seconds:
                first_window.add(unwrapped_euler)
            if (
                elapsed_s >= burn_in_seconds
                and elapsed_s <= burn_in_seconds + window_seconds
            ):
                post_burn_in_first_window.add(unwrapped_euler)
            tail_window.append(elapsed_s, unwrapped_euler)

            if valid["gyro"]:
                for axis, value in enumerate(gyro):
                    if not math.isfinite(value):
                        continue
                    gyro_full[axis].observe(value)
                    if elapsed_s >= burn_in_seconds:
                        gyro_post_burn_in[axis].observe(value)

            static_runs.observe(bool(status & (1 << 9)), elapsed_s)

    static_runs.finish()

    status_bit_counts = {name: 0 for name, _ in STATUS_BITS}
    for status, count in status_distribution.items():
        for name, bit in STATUS_BITS:
            if status & bit:
                status_bit_counts[name] += count
    valid_counts = {
        name: {
            "valid": status_bit_counts[bit_name],
            "invalid": rows - status_bit_counts[bit_name],
        }
        for name, bit_name in (
            ("attitude", "attitude_valid"),
            ("accel", "accel_valid"),
            ("gyro", "gyro_valid"),
        )
    }
    fault_any = sum(
        count
        for status, count in status_distribution.items()
        if status & ((1 << 13) | (1 << 14))
    )
    host_summary = host_time.summary(rows)
    device_summary = device_time.summary(rows)
    post_burn_in_last_window = tail_window.sums_at_or_after(burn_in_seconds)

    euler_summary = {}
    for axis, name in enumerate(EULER_AXES):
        euler_summary[name] = {
            "invalid_samples": euler_axes[axis].invalid_samples,
            "nonfinite_samples": euler_axes[axis].nonfinite_samples,
            "all_span": euler_axes[axis].full.summary(),
            "post_burn_in": euler_axes[axis].post_burn_in.summary(),
            "window": _window_summary(
                axis, first_window, tail_window.sums
            ),
            "post_burn_in_window": _window_summary(
                axis,
                post_burn_in_first_window,
                post_burn_in_last_window,
            ),
        }
    gyro_summary = {}
    for axis, name in enumerate(GYRO_AXES):
        gyro_summary[name] = {
            "all_span": gyro_full[axis].summary(),
            "post_burn_in": gyro_post_burn_in[axis].summary(),
        }

    return {
        "source": str(path),
        "configuration": {
            "window_seconds": window_seconds,
            "burn_in_seconds": burn_in_seconds,
            "analysis_time_base": "trusted_unwrapped_system_time_ms",
        },
        "rows": rows,
        "frame_index": frame_indices.summary(),
        "host_time": host_summary,
        "device_time": device_summary,
        "average_frame_rate_hz": host_summary["average_frame_rate_hz"],
        "status_distribution": {
            "0x{:04X}".format(status): status_distribution[status]
            for status in sorted(status_distribution)
        },
        "status_bit_counts": status_bit_counts,
        "valid_counts": valid_counts,
        "fault_counts": {
            "bmi_fault_frames": status_bit_counts["bmi_fault"],
            "icm_fault_frames": status_bit_counts["icm_fault"],
            "any_fault_frames": fault_any,
        },
        "saturation_counts": {
            "acc_frames": status_bit_counts["acc_saturation_recent"],
            "gyro_frames": status_bit_counts["gyro_saturation_recent"],
        },
        "stream_drop_frames": status_bit_counts["stream_drop"],
        "zero_triplets": zero_triplets,
        "valid_zero_triplets": {
            name: zero_triplets[name]["valid"]
            for name in ("accel", "gyro", "euler")
        },
        "nonfinite": {
            "frames": nonfinite_frames,
            "values": nonfinite_values,
            "by_field": nonfinite_by_field,
        },
        "quaternion_norm": quaternion_norm.summary(),
        "temperature_c": {
            "minimum": temperature_min,
            "maximum": temperature_max,
        },
        "static_runs": static_runs.aggregates[True].summary(rows),
        "nonstatic_runs": static_runs.aggregates[False].summary(rows),
        "static_state_transitions": static_runs.transitions,
        "euler_deg": {
            "sample_policy": "attitude_valid status bit set and finite axis value",
            "unwrap_period_deg": 360.0,
            "window_seconds": window_seconds,
            "burn_in_seconds": burn_in_seconds,
            "window_semantics": {
                "window": (
                    "legacy comparison of capture [0, window] with the final "
                    "window; it includes burn-in"
                ),
                "post_burn_in_window": (
                    "comparison of [burn_in, burn_in + window] with the final "
                    "window restricted to samples at or after burn-in"
                ),
            },
            "axes": euler_summary,
        },
        "gyro_deg_s": {
            "sample_policy": "gyro_valid status bit set and finite axis value",
            "burn_in_seconds": burn_in_seconds,
            "axes": gyro_summary,
        },
    }


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Stream-analyze a hi91_capture.py CSV without loading it into memory."
        )
    )
    parser.add_argument("csv", metavar="CSV", help="capture CSV to analyze")
    parser.add_argument(
        "--window-seconds",
        type=_finite_positive_float,
        default=300.0,
        help="first/last Euler mean window (default: 300)",
    )
    parser.add_argument(
        "--burn-in-seconds",
        type=_finite_nonnegative_float,
        default=0.0,
        help="also report statistics after this device elapsed time (default: 0)",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_argument_parser().parse_args(argv)
    try:
        summary = analyze_csv(
            args.csv,
            window_seconds=args.window_seconds,
            burn_in_seconds=args.burn_in_seconds,
        )
    except (AnalysisError, OSError, UnicodeError, csv.Error) as error:
        print("hi91_static_analysis: {}".format(error), file=sys.stderr)
        return 1
    print(json.dumps(summary, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
