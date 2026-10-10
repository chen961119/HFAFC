"""Flight-log loading and analysis, independent of the graphical interface."""
from __future__ import annotations

import csv
from dataclasses import dataclass
import io
from pathlib import Path
import re

import numpy as np
import pandas as pd


class LogError(ValueError):
    """A log cannot be interpreted without an explicit user decision."""


@dataclass(frozen=True)
class Segment:
    start: float
    stop: float
    label: str


@dataclass(frozen=True)
class FlightLog:
    path: Path
    columns: tuple[str, ...]
    data: np.ndarray
    time: np.ndarray
    sample_rate: float
    time_source: str
    time_unit: str
    warnings: tuple[str, ...]
    in_air: tuple[Segment, ...]
    in_air_source: str
    modes: tuple[Segment, ...]

    def column(self, name: str) -> np.ndarray:
        return self.data[:, self.columns.index(name)]

    def summary(self) -> dict:
        return dict(file=str(self.path), samples=len(self.time), columns=list(self.columns),
                    duration_seconds=float(self.time[-1]), sample_rate_hz=self.sample_rate,
                    time_source=self.time_source, time_unit=self.time_unit,
                    memory_bytes=self.data.nbytes + self.time.nbytes,
                    warnings=list(self.warnings), in_air_source=self.in_air_source,
                    in_air=[vars(s) for s in self.in_air], modes=[vars(s) for s in self.modes])


def _headers(raw: list[str]) -> tuple[str, ...]:
    names, used = [], set()
    for i, name in enumerate(raw):
        base = name.strip() or f"Column_{i + 1}"
        name, suffix = base, 2
        while name in used:
            name = f"{base}__{suffix}"
            suffix += 1
        names.append(name)
        used.add(name)
    return tuple(names)


class _ValidatedReader(io.TextIOBase):
    """Validate row width during the same read used by pandas' C parser.

    Its chunk reader can otherwise discard an extra field at a chunk boundary.
    Numeric flight logs use one physical line per record.
    """
    def __init__(self, stream, delimiter, columns):
        self.stream, self.delimiter, self.columns = stream, delimiter, columns
        self.pending = ""
        self.line = 1

    def readable(self):
        return True

    def read(self, size=-1):
        block = self.stream.read(size)
        pieces = (self.pending + block).split("\n")
        self.pending = pieces.pop() if block else ""
        for line in pieces:
            self.line += 1
            if not line.strip():
                continue
            count = len(next(csv.reader([line], delimiter=self.delimiter))) if '"' in line else line.count(self.delimiter) + 1
            if count != self.columns:
                raise LogError(f"物理行 {self.line} 有 {count} 列，表头有 {self.columns} 列")
        return block


def _read_table(path: Path, delimiter: str | None, encoding: str,
                skip_bad_rows: bool, chunk_rows: int) -> tuple[tuple[str, ...], np.ndarray, list[str]]:
    warnings = []
    with path.open(encoding=encoding, newline="") as stream:
        first = stream.readline()
        if not first.strip():
            raise LogError("文件为空或首行没有表头。")
        if delimiter is None:
            delimiter = max((",", "\t", ";"), key=first.count)
        if len(delimiter) != 1:
            raise LogError("分隔符必须是单个字符。")
        columns = _headers(next(csv.reader([first], delimiter=delimiter)))
        body_position = stream.tell()
        # No supplied names/index: extra fields must raise instead of being
        # silently treated as an index or truncated to the header width.
        try:
            chunks = []
            validated = _ValidatedReader(stream, delimiter, len(columns))
            for frame in pd.read_csv(validated, sep=delimiter, header=None, dtype=np.float64,
                                     chunksize=chunk_rows, skip_blank_lines=True):
                if frame.shape[1] != len(columns):
                    raise LogError(f"表头有 {len(columns)} 列，数据有 {frame.shape[1]} 列；请核对日志版本。")
                chunks.append(frame.to_numpy(copy=False))
            if not chunks:
                raise LogError("表头后没有数据。")
            data = chunks[0] if len(chunks) == 1 else np.concatenate(chunks)
        except (pd.errors.ParserError, ValueError) as exc:
            if not skip_bad_rows:
                raise LogError(f"数据格式不一致：{exc}。可使用 --skip-bad-rows 显式跳过坏行。") from exc
            stream.seek(body_position)
            rows, skipped = [], []
            for number, row in enumerate(csv.reader(stream, delimiter=delimiter), start=2):
                if not row or not any(cell.strip() for cell in row):
                    continue
                try:
                    if len(row) != len(columns):
                        raise ValueError("column count")
                    values = [float(cell) if cell.strip() else np.nan for cell in row]
                except ValueError:
                    skipped.append(number)
                    continue
                rows.append(values)
            if not rows:
                raise LogError("没有有效数据行。")
            data = np.asarray(rows, dtype=np.float64)
            warnings.append(f"跳过 {len(skipped)} 个坏行；物理行号示例：{skipped[:10]}。")
    invalid = int(np.count_nonzero(~np.isfinite(data)))
    if invalid:
        warnings.append(f"存在 {invalid} 个空白/非有限值；绘图保留缺口，FFT 单独检查。")
    return columns, data, warnings


def _time_base(columns, data, requested_column, requested_unit, fallback_hz, warnings):
    if not np.isfinite(fallback_hz) or fallback_hz <= 0:
        raise LogError("默认采样率必须为正数。")
    if requested_unit not in ("auto", "s", "ms", "us"):
        raise LogError("时间单位只能为 auto、s、ms 或 us。")
    if requested_column is not None:
        if requested_column not in columns:
            raise LogError(f"找不到时间列：{requested_column}")
        index = columns.index(requested_column)
    else:
        index = next((i for i, name in enumerate(columns)
                      if re.match(r"^(timestamp|time|stamp)(?:\b|_|\()", name, re.I)), None)
    if index is None:
        warnings.append(f"没有时间列，使用样本序号与显式默认采样率 {fallback_hz:g} Hz。")
        return data, np.arange(len(data), dtype=float) / fallback_hz, fallback_hz, "Sample index", "s"
    name = columns[index]
    raw = data[:, index]
    finite = np.isfinite(raw)
    if not finite.all():
        warnings.append(f"删除 {int((~finite).sum())} 个时间戳无效的数据行。")
        data, raw = data[finite], raw[finite]
    if len(raw) < 2:
        raise LogError("时间列至少需要两个有效样本。")
    label = name.lower().replace("µ", "u").replace("μ", "u")
    unit = requested_unit
    if unit == "auto":
        token = re.search(r"(?:\(|_|\b)(us|ms|sec|seconds|s)(?:\)|\b|$)", label)
        if token:
            unit = {"sec": "s", "seconds": "s"}.get(token[1], token[1])
        else:
            steps = np.diff(raw)
            positive = steps[steps > 0]
            if not len(positive):
                raise LogError("时间戳没有正向步进。")
            step = float(np.median(positive))
            unit = "us" if step > 1000 else "ms" if step > 1 else "s"
            warnings.append(f"时间列未注明单位，按增量推断为 {unit}；可用 --time-unit 覆盖。")
    # Teensy micros() is uint32. Unwrap only a high-to-low boundary crossing,
    # never silently sort or bridge a reboot/reset in the middle of a file.
    delta = np.diff(raw)
    backwards = delta < 0
    wraps = backwards & (raw[:-1] > 0.75 * 2**32) & (raw[1:] < 0.25 * 2**32) if unit == "us" else np.zeros_like(backwards)
    if np.any(backwards & ~wraps):
        bad = int(np.flatnonzero(backwards & ~wraps)[0])
        raise LogError(f"时间戳在数据行 {bad + 2} 附近倒退/复位；请按启动段拆分文件，不能排序后当作连续飞行。")
    if wraps.any():
        raw = raw + np.r_[0, np.cumsum(wraps)] * 2**32
        warnings.append(f"修复 {int(wraps.sum())} 次 uint32 微秒计数器回绕。")
    keep = np.r_[True, np.diff(raw) > 0]
    if not keep.all():
        warnings.append(f"删除 {int((~keep).sum())} 个重复时间戳，保留第一次记录。")
        raw, data = raw[keep], data[keep]
    if len(raw) < 2:
        raise LogError("去重后时间列不足两个样本。")
    time = (raw - raw[0]) * {"s": 1, "ms": 1e-3, "us": 1e-6}[unit]
    dt = np.diff(time)
    rate = 1 / float(np.median(dt))
    jitter = float(np.std(dt) / np.median(dt))
    if jitter > 0.01:
        warnings.append(f"采样间隔抖动 {100 * jitter:.2f}%；FFT 根据所选区段检查并重采样。")
    if rate < 1 or rate > 5000:
        warnings.append(f"估计采样率 {rate:g} Hz 超出常见范围；请检查时间单位。")
    return data, time, rate, name, unit


def _runs(mask: np.ndarray):
    edges = np.diff(np.r_[False, mask, False].astype(np.int8))
    return zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1) - 1)


def _smooth_mask(mask, time, close_seconds, min_seconds):
    mask = mask.copy()
    dt = float(np.median(np.diff(time)))
    for start, stop in _runs(~mask):
        if start > 0 and stop + 1 < len(mask) and time[stop + 1] - time[start - 1] - dt <= close_seconds:
            mask[start:stop + 1] = True
    for start, stop in _runs(mask):
        if time[stop] - time[start] + dt < min_seconds:
            mask[start:stop + 1] = False
    return mask


def _in_air(columns, data, time):
    candidates = ["airspeed_A", "A_thro_PWM", "thro_PWM", "CH3_PWM",
                  *[f"{node}thro_PWM" for node in "BCDEFG"]]
    for name in candidates:
        if name not in columns:
            continue
        series = data[:, columns.index(name)]
        good = series[np.isfinite(series)]
        if not len(good):
            continue
        if name == "airspeed_A":
            threshold, gap, minimum = max(8, 0.25 * np.quantile(good, 0.99)), 2, 1
        else:
            threshold = max(1150, min(1400, np.min(good) + 0.25 * np.ptp(good)))
            gap, minimum = 3, 2
        mask = _smooth_mask(np.isfinite(series) & (series > threshold), time, gap, minimum)
        # Do not paint a flight interval across a long missing-data gap.
        segments = []
        for start, stop in _runs(mask):
            splits = np.flatnonzero(np.diff(time[start:stop + 1]) > gap) + start + 1
            bounds = np.r_[start, splits, stop + 1]
            for left, right in zip(bounds[:-1], bounds[1:]):
                if time[right - 1] - time[left] >= minimum:
                    segments.append(Segment(float(time[left]), float(time[right - 1]), "In air (estimate)"))
        if segments:
            return tuple(segments), name
    return (), "none"


def _modes(columns, data, time):
    if "CH5_PWM" not in columns:
        return ()
    values = data[:, columns.index("CH5_PWM")]
    codes = np.where(~np.isfinite(values), 0, np.where(values > 1600, 1, np.where(values < 1400, 3, 2)))
    boundaries = np.r_[0, np.flatnonzero(np.diff(codes)) + 1, len(time)]
    names = {0: "Unknown", 1: "Manual", 2: "Stab1", 3: "Stab2"}
    return tuple(Segment(float(time[a]), float(time[b - 1]), names[int(codes[a])])
                 for a, b in zip(boundaries[:-1], boundaries[1:]))


def load_log(file, *, delimiter=None, encoding="utf-8-sig", time_column=None,
             time_unit="auto", fallback_hz=50.0, skip_bad_rows=False, chunk_rows=32768) -> FlightLog:
    """Read a header + numeric table. Preserve units and never truncate columns."""
    if chunk_rows < 1:
        raise LogError("分块行数必须大于零。")
    path = Path(file).expanduser().resolve()
    columns, data, warnings = _read_table(path, delimiter, encoding, skip_bad_rows, chunk_rows)
    if not len(data):
        raise LogError("文件没有数据样本。")
    data, time, rate, source, unit = _time_base(columns, data, time_column, time_unit, fallback_hz, warnings)
    in_air, in_air_source = _in_air(columns, data, time) if len(time) > 1 else ((), "none")
    modes = _modes(columns, data, time)
    data.setflags(write=False)
    time.setflags(write=False)
    return FlightLog(path, columns, data, time, rate, source, unit, tuple(warnings), in_air, in_air_source, modes)


@dataclass(frozen=True)
class Spectrum:
    frequency: np.ndarray
    amplitude: np.ndarray
    peak_frequency: float
    peak_amplitude: float
    sample_rate: float
    samples: int
    resampled: bool


def compute_spectrum(time, values, bounds=None) -> Spectrum:
    """Hann-windowed one-sided amplitude; reject gaps instead of fabricating FFT."""
    time, values = np.asarray(time), np.asarray(values)
    if time.ndim != 1 or time.shape != values.shape:
        raise LogError("时间与数值必须是长度相同的一维数组。")
    if bounds is not None:
        if len(bounds) != 2 or np.isnan(bounds).any() or bounds[0] >= bounds[1]:
            raise LogError("FFT 区段必须满足起点小于终点。")
        left = np.searchsorted(time, bounds[0], side="left")
        right = np.searchsorted(time, bounds[1], side="right")
        time, values = time[left:right], values[left:right]
    good = np.isfinite(time) & np.isfinite(values)
    time, values = time[good], values[good]
    if len(time) < 8:
        raise LogError("FFT 区段至少需要 8 个有效样本。")
    dt = np.diff(time)
    if np.any(dt <= 0):
        raise LogError("FFT 时间轴必须严格递增。")
    median = float(np.median(dt))
    if np.max(dt) > 5 * median:
        raise LogError("所选区段存在超过 5 个采样周期的缺口，请缩小 FFT 时间范围。")
    resampled = bool(np.max(np.abs(dt - median)) > 0.01 * median or not good.all())
    rate = 1 / median
    if resampled:
        count = round((time[-1] - time[0]) / median) + 1
        uniform = np.linspace(time[0], time[-1], count)
        values = np.interp(uniform, time, values)
        rate = (count - 1) / (time[-1] - time[0])
    count = len(values)
    window = np.hanning(count)
    amplitude = np.abs(np.fft.rfft((values - values.mean()) * window)) * 2 / window.sum()
    amplitude[0] /= 2
    if count % 2 == 0:
        amplitude[-1] /= 2
    frequency = np.fft.rfftfreq(count, 1 / rate)
    peak = 1 + int(np.argmax(amplitude[1:]))
    return Spectrum(frequency, amplitude, float(frequency[peak]), float(amplitude[peak]), rate, count, resampled)


def display_series(time, values, *, max_points=6000, smooth_samples=1):
    """Reduce display cost, preserving chronological local minima and maxima."""
    time, values = np.asarray(time), np.asarray(values)
    if max_points < 4:
        raise ValueError("max_points must be >= 4")
    if not len(time):
        return time, values
    if smooth_samples > 1:
        width = min(int(smooth_samples), len(values))
        good = np.isfinite(values)
        kernel = np.ones(width)
        total = np.convolve(np.where(good, values, 0), kernel, "same")
        weights = np.convolve(good.astype(float), kernel, "same")
        values = np.divide(total, weights, out=np.full_like(total, np.nan), where=weights > 0)
        values[~good] = np.nan
    if len(time) <= max_points:
        return time, values
    count = max_points // 3
    width = int(np.ceil(len(time) / count))
    selected = {0, len(time) - 1}
    for start in range(0, len(time), width):
        chunk = values[start:start + width]
        finite = np.isfinite(chunk)
        if finite.any():
            selected.add(start + int(np.argmin(np.where(finite, chunk, np.inf))))
            selected.add(start + int(np.argmax(np.where(finite, chunk, -np.inf))))
        if not finite.all():
            selected.add(start + int(np.flatnonzero(~finite)[0]))
    selected = np.array(sorted(selected))
    return time[selected], values[selected]


def describe_variable(name):
    lower = name.lower()
    if "_control_us" in lower:
        return "机间有符号控制偏移（μs），尚未应用本机中位、rev 与 trim；不是物理 PWM。"
    if "timestamp" in lower or lower.startswith("time"):
        return "原始记录时间戳；图中横轴转换为秒，并从本文件第一个有效样本开始。"
    for token, text in (("mt6701", "MT6701 夹角传感器"), ("airspeed", "空速测量"),
                        ("strain", "应变传感器"), ("indi", "俯仰 INDI 控制状态"),
                        ("gyro", "陀螺仪角速度"), ("acc", "加速度计测量"),
                        ("roll", "滚转状态/目标/控制量"), ("pitch", "俯仰状态/目标/控制量"),
                        ("yaw", "偏航状态/目标/控制量"), ("phi", "相邻机体构型角/目标/控制量"),
                        ("pwm", "遥控输入或执行器 PWM；具体物理含义以日志版本为准")):
        if token in lower:
            return text + f"；字段：{name}。"
    return f"日志字段：{name}；保留原表头单位。"
