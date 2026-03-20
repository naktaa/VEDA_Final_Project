#!/usr/bin/env python3
"""
Live LK/gyro compare visualizer with gyro-axis and drive-state panels.

This tool now supports two live-debug use cases in one window:
1) Existing LK frame-to-frame rotation vs IMU yaw-delta compare
2) Gyro axis rate / integrated angle / applied correction / tank-drive state

Accepted input line formats:
1) CSV (recommended for compare-only mode): t_ms,lk_da,gyro_dyaw
2) Key-value compare text:
   - "... t_ms=123.4 ... lk_da=0.12 ... gyro_dyaw=0.10 ..."
3) Existing C++ runtime logs:
   - [ALIGN] ... frame=...
   - [CMP] lk_da=...deg gyro_dyaw=...deg diff=... corr=...
   - [GYRO] ... rate(rad/s)=... ... phys(deg)=... ... corr(deg)=...
   - [HIGHPASS]/[DELTA] ... req_crop=...%
   - [TANK] L=... R=... PWM=...
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
import threading
import time
from collections import deque
from queue import Empty, SimpleQueue
from typing import Any, Dict, List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation


FLOAT_RE = r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?"

TS_PAT = re.compile(rf"(?:t_ms|ts_ms|frame_time_ms)\s*=\s*({FLOAT_RE})")
FRAME_PAT = re.compile(rf"\bframe\s*=\s*({FLOAT_RE})")

LK_PAT = re.compile(rf"lk_da\s*=\s*({FLOAT_RE})")
GYRO_DYAW_PAT = re.compile(rf"gyro_dyaw\s*=\s*({FLOAT_RE})")

GYRO_LINE_PAT = re.compile(
    rf"\[GYRO\].*?"
    rf"rate\(rad/s\)=\s*({FLOAT_RE})\s+({FLOAT_RE})\s+({FLOAT_RE}).*?"
    rf"phys\(deg\)=\s*({FLOAT_RE})\s+({FLOAT_RE})\s+({FLOAT_RE}).*?"
    rf"corr\(deg\)=\s*({FLOAT_RE})\s+({FLOAT_RE})\s+({FLOAT_RE}).*?"
    rf"hz=\s*({FLOAT_RE})"
)

WARP_LINE_PAT = re.compile(
    rf"\[(HIGHPASS|DELTA)\].*?"
    rf"corr\(deg\)=\s*({FLOAT_RE})\s+({FLOAT_RE})\s+({FLOAT_RE}).*?"
    rf"pix=\s*({FLOAT_RE})\s+req_scale=\s*({FLOAT_RE})\s+req_crop=\s*({FLOAT_RE})%"
)

TANK_STATE_PAT = re.compile(r"\[TANK\]\s+L=([-+]?\d+)\s+R=([-+]?\d+)\s+PWM=(\d+)")
TANK_READY_PAT = re.compile(r"\[TANK\].*PWM=(\d+)")


def corrcoef_safe(a: np.ndarray, b: np.ndarray) -> float:
    if a.size < 3 or b.size < 3:
        return float("nan")
    aa = a - np.mean(a)
    bb = b - np.mean(b)
    denom = np.linalg.norm(aa) * np.linalg.norm(bb)
    if denom < 1e-12:
        return float("nan")
    return float(np.dot(aa, bb) / denom)


def rad_to_deg(x: float) -> float:
    return float(x) * 180.0 / np.pi


def dominant_axis_name(x: float, y: float, z: float) -> str:
    vals = [abs(x), abs(y), abs(z)]
    idx = int(np.argmax(vals))
    return "xyz"[idx]


class StreamParser:
    def __init__(self, fallback_fps: float):
        self.fallback_dt_ms = 1000.0 / max(1e-6, fallback_fps)
        self.last_fallback_t_ms: Optional[float] = None
        self.last_context_t_ms: Optional[float] = None
        self.last_tank_pwm: int = 0

    def _next_fallback_t_ms(self) -> float:
        if self.last_fallback_t_ms is None:
            self.last_fallback_t_ms = 0.0
        else:
            self.last_fallback_t_ms += self.fallback_dt_ms
        return self.last_fallback_t_ms

    def _event_t_ms(self, explicit_t_ms: Optional[float]) -> float:
        if explicit_t_ms is not None:
            self.last_context_t_ms = explicit_t_ms
            return explicit_t_ms
        if self.last_context_t_ms is not None:
            return self.last_context_t_ms
        return self._next_fallback_t_ms()

    def parse_line(self, line: str) -> List[Dict[str, Any]]:
        line = line.strip()
        if not line:
            return []

        explicit_t_ms = None
        ts_m = TS_PAT.search(line)
        if ts_m:
            explicit_t_ms = float(ts_m.group(1))
            self.last_context_t_ms = explicit_t_ms
        else:
            frame_m = FRAME_PAT.search(line)
            if frame_m:
                explicit_t_ms = float(frame_m.group(1))
                self.last_context_t_ms = explicit_t_ms

        events: List[Dict[str, Any]] = []

        if "," in line:
            parts = [p.strip() for p in line.split(",")]
            if len(parts) >= 3:
                try:
                    t_ms = float(parts[0])
                    lk_da = float(parts[1])
                    gyro_dyaw = float(parts[2])
                    self.last_context_t_ms = t_ms
                    events.append(
                        {
                            "kind": "cmp",
                            "t_ms": t_ms,
                            "lk_da": lk_da,
                            "gyro_dyaw": gyro_dyaw,
                        }
                    )
                    return events
                except ValueError:
                    pass

        gyro_m = GYRO_LINE_PAT.search(line)
        if gyro_m:
            t_ms = self._event_t_ms(explicit_t_ms)
            gx_deg = rad_to_deg(float(gyro_m.group(1)))
            gy_deg = rad_to_deg(float(gyro_m.group(2)))
            gz_deg = rad_to_deg(float(gyro_m.group(3)))
            events.append(
                {
                    "kind": "gyro",
                    "t_ms": t_ms,
                    "rate_deg": (gx_deg, gy_deg, gz_deg),
                    "phys_deg": (
                        float(gyro_m.group(4)),
                        float(gyro_m.group(5)),
                        float(gyro_m.group(6)),
                    ),
                    "corr_deg": (
                        float(gyro_m.group(7)),
                        float(gyro_m.group(8)),
                        float(gyro_m.group(9)),
                    ),
                    "hz": float(gyro_m.group(10)),
                }
            )
            return events

        warp_m = WARP_LINE_PAT.search(line)
        if warp_m:
            t_ms = self._event_t_ms(explicit_t_ms)
            events.append(
                {
                    "kind": "warp",
                    "t_ms": t_ms,
                    "mode": warp_m.group(1).lower(),
                    "corr_deg": (
                        float(warp_m.group(2)),
                        float(warp_m.group(3)),
                        float(warp_m.group(4)),
                    ),
                    "pixel_disp": float(warp_m.group(5)),
                    "req_scale": float(warp_m.group(6)),
                    "req_crop": float(warp_m.group(7)),
                }
            )
            return events

        tank_m = TANK_STATE_PAT.search(line)
        if tank_m:
            left = int(tank_m.group(1))
            right = int(tank_m.group(2))
            pwm = int(tank_m.group(3))
            self.last_tank_pwm = pwm
            events.append(
                {
                    "kind": "tank",
                    "t_ms": self._event_t_ms(explicit_t_ms),
                    "left": left,
                    "right": right,
                    "pwm": pwm,
                }
            )
            return events

        if "[TANK]" in line and "auto-stop" in line:
            events.append(
                {
                    "kind": "tank",
                    "t_ms": self._event_t_ms(explicit_t_ms),
                    "left": 0,
                    "right": 0,
                    "pwm": self.last_tank_pwm,
                }
            )
            return events

        ready_m = TANK_READY_PAT.search(line)
        if ready_m and "[TANK]" in line and "L=" not in line and "R=" not in line:
            self.last_tank_pwm = int(ready_m.group(1))

        lk_m = LK_PAT.search(line)
        gy_m = GYRO_DYAW_PAT.search(line)
        if lk_m and gy_m:
            events.append(
                {
                    "kind": "cmp",
                    "t_ms": self._event_t_ms(explicit_t_ms),
                    "lk_da": float(lk_m.group(1)),
                    "gyro_dyaw": float(gy_m.group(1)),
                }
            )

        return events


class StreamReader(threading.Thread):
    def __init__(
        self,
        queue: SimpleQueue,
        parser: StreamParser,
        input_path: Optional[str],
        from_start: bool,
    ):
        super().__init__(daemon=True)
        self.queue = queue
        self.parser = parser
        self.input_path = input_path
        self.from_start = from_start
        self.stop_event = threading.Event()

    def stop(self) -> None:
        self.stop_event.set()

    def run(self) -> None:
        if self.input_path:
            self._run_file_tail(self.input_path)
        else:
            self._run_stdin()

    def _emit_line(self, line: str) -> None:
        for event in self.parser.parse_line(line):
            self.queue.put(event)

    def _run_stdin(self) -> None:
        while not self.stop_event.is_set():
            line = sys.stdin.readline()
            if not line:
                time.sleep(0.01)
                continue
            self._emit_line(line)

    def _run_file_tail(self, path: str) -> None:
        with open(path, "r", encoding="utf-8", errors="ignore") as f:
            if not self.from_start:
                f.seek(0, 2)
            while not self.stop_event.is_set():
                line = f.readline()
                if not line:
                    time.sleep(0.01)
                    continue
                self._emit_line(line)


class CmdReader(threading.Thread):
    """Run external command and parse its stdout/stderr stream in real time."""

    def __init__(self, queue: SimpleQueue, parser: StreamParser, cmd: str):
        super().__init__(daemon=True)
        self.queue = queue
        self.parser = parser
        self.cmd = cmd
        self.stop_event = threading.Event()
        self.proc: Optional[subprocess.Popen] = None

    def stop(self) -> None:
        self.stop_event.set()
        if self.proc is not None and self.proc.poll() is None:
            try:
                self.proc.terminate()
            except Exception:
                pass

    def run(self) -> None:
        self.proc = subprocess.Popen(
            self.cmd,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        if self.proc.stdout is None:
            return

        while not self.stop_event.is_set():
            line = self.proc.stdout.readline()
            if not line:
                if self.proc.poll() is not None:
                    break
                time.sleep(0.01)
                continue
            for event in self.parser.parse_line(line):
                self.queue.put(event)


class DemoGenerator(threading.Thread):
    """Generate synthetic LK/Gyro compare stream for quick visualization checks."""

    def __init__(self, queue: SimpleQueue, fps: float, lag_ms: float, invert: bool):
        super().__init__(daemon=True)
        self.queue = queue
        self.dt = 1.0 / max(1e-6, fps)
        self.lag_ms = lag_ms
        self.sign = -1.0 if invert else 1.0
        self.stop_event = threading.Event()

    def stop(self) -> None:
        self.stop_event.set()

    def run(self) -> None:
        t0 = time.monotonic()
        while not self.stop_event.is_set():
            t_sec = time.monotonic() - t0
            t_ms = t_sec * 1000.0
            lk = (
                3.5 * np.sin(2.0 * np.pi * 0.65 * t_sec)
                + 1.8 * np.sin(2.0 * np.pi * 1.3 * t_sec + 0.5)
                + 0.6 * np.sin(2.0 * np.pi * 2.1 * t_sec + 1.2)
            )
            tg = t_sec - self.lag_ms / 1000.0
            gyro = self.sign * (
                3.5 * np.sin(2.0 * np.pi * 0.65 * tg)
                + 1.8 * np.sin(2.0 * np.pi * 1.3 * tg + 0.5)
                + 0.6 * np.sin(2.0 * np.pi * 2.1 * tg + 1.2)
            )
            gyro += float(np.random.normal(0.0, 0.08))
            lk += float(np.random.normal(0.0, 0.05))
            self.queue.put({"kind": "cmp", "t_ms": t_ms, "lk_da": lk, "gyro_dyaw": gyro})
            time.sleep(self.dt)


def update_axis_ylim(ax, arrays, default=(-1.0, 1.0), fixed_abs: Optional[float] = None):
    if fixed_abs is not None and fixed_abs > 0:
        ax.set_ylim(-fixed_abs, fixed_abs)
        return

    ys = []
    for arr in arrays:
        if arr is None:
            continue
        arr = np.asarray(arr, dtype=float)
        finite = arr[np.isfinite(arr)]
        if finite.size:
            ys.append(finite)
    if not ys:
        ax.set_ylim(default[0], default[1])
        return

    y = np.concatenate(ys)
    y_min = float(np.min(y))
    y_max = float(np.max(y))
    if abs(y_max - y_min) < 1e-9:
        span = max(1e-3, abs(y_max) * 0.1 + 1e-3)
        ax.set_ylim(y_min - span, y_max + span)
    else:
        margin = 0.15 * (y_max - y_min)
        ax.set_ylim(y_min - margin, y_max + margin)


def estimate_best_lag(
    t_ms: np.ndarray,
    lk_da: np.ndarray,
    gyro_dyaw: np.ndarray,
    analysis_sec: float,
    lag_max_ms: float,
    lag_step_ms: float,
    sign_mode: str,
    min_points: int,
):
    if t_ms.size < min_points:
        return None

    end_ms = t_ms[-1]
    start_ms = end_ms - analysis_sec * 1000.0
    sel = t_ms >= start_ms
    t_ref = t_ms[sel]
    lk_ref = lk_da[sel]
    if t_ref.size < min_points:
        return None

    lags = np.arange(-lag_max_ms, lag_max_ms + 0.5 * lag_step_ms, lag_step_ms)
    if sign_mode == "auto":
        sign_candidates = [("normal", 1.0), ("invert", -1.0)]
    else:
        sign_candidates = [(sign_mode, 1.0 if sign_mode == "normal" else -1.0)]

    best = None
    curves = {}

    for sign_name, sign_mul in sign_candidates:
        corr_curve = np.full_like(lags, np.nan, dtype=float)
        for i, lag in enumerate(lags):
            t_query = t_ref - lag
            gy_interp = np.interp(t_query, t_ms, gyro_dyaw, left=np.nan, right=np.nan)
            valid = np.isfinite(gy_interp) & np.isfinite(lk_ref)
            if int(np.count_nonzero(valid)) < min_points:
                continue
            corr_curve[i] = corrcoef_safe(lk_ref[valid], sign_mul * gy_interp[valid])

        curves[sign_name] = corr_curve
        if np.all(np.isnan(corr_curve)):
            continue
        idx = int(np.nanargmax(corr_curve))
        corr = float(corr_curve[idx])
        lag = float(lags[idx])
        candidate = (corr, lag, sign_name, sign_mul, corr_curve)
        if best is None or candidate[0] > best[0]:
            best = candidate

    if best is None:
        return None

    best_corr, best_lag, best_sign_name, best_sign_mul, best_curve = best
    return {
        "lags": lags,
        "best_corr": best_corr,
        "best_lag": best_lag,
        "best_sign_name": best_sign_name,
        "best_sign_mul": best_sign_mul,
        "best_curve": best_curve,
        "curves": curves,
    }


def select_window(t_ms: np.ndarray, start_ms: float) -> Tuple[np.ndarray, np.ndarray]:
    sel = t_ms >= start_ms
    return sel, (t_ms[sel] - start_ms) / 1000.0


def latest_time_ms(*buffers: deque) -> Optional[float]:
    vals = [buf[-1] for buf in buffers if len(buf) > 0]
    if not vals:
        return None
    return float(max(vals))


def main():
    ap = argparse.ArgumentParser(description="Live LK vs gyro compare visualizer")
    ap.add_argument(
        "--input",
        type=str,
        default=None,
        help="append-only input file path (tail -f style). If omitted, read from stdin.",
    )
    ap.add_argument(
        "--cmd",
        type=str,
        default=None,
        help="run command and parse realtime stdout/stderr (e.g., your C++ app)",
    )
    ap.add_argument(
        "--from-start",
        action="store_true",
        help="for --input mode, read from beginning instead of seeking to end",
    )
    ap.add_argument("--demo", action="store_true", help="run synthetic realtime compare generator")
    ap.add_argument("--demo-fps", type=float, default=30.0, help="synthetic generator FPS")
    ap.add_argument("--demo-lag-ms", type=float, default=12.0, help="synthetic gyro lag for demo")
    ap.add_argument("--demo-invert", action="store_true", help="invert synthetic gyro sign for demo")
    ap.add_argument("--display-sec", type=float, default=10.0, help="visible rolling time window in seconds")
    ap.add_argument("--analysis-sec", type=float, default=2.5, help="recent window for lag estimation in seconds")
    ap.add_argument("--lag-max-ms", type=float, default=80.0, help="lag sweep range [-max, +max] ms")
    ap.add_argument("--lag-step-ms", type=float, default=2.0, help="lag sweep step in ms")
    ap.add_argument("--sign", choices=["auto", "normal", "invert"], default="auto", help="gyro sign mode")
    ap.add_argument("--fps-fallback", type=float, default=30.0, help="pseudo-time FPS when input line has no timestamp")
    ap.add_argument("--max-buffer", type=int, default=6000, help="raw sample buffer size")
    ap.add_argument("--interval-ms", type=int, default=50, help="plot refresh interval")
    ap.add_argument("--rate-ylim", type=float, default=60.0, help="fixed +/- range for gyro-rate panel; <=0 means auto")
    ap.add_argument("--angle-ylim", type=float, default=20.0, help="fixed +/- range for phys-angle panel; <=0 means auto")
    ap.add_argument("--corr-ylim", type=float, default=12.0, help="fixed +/- range for corr-angle panel; <=0 means auto")
    ap.add_argument("--drive-ylim", type=float, default=255.0, help="fixed +/- range for drive-effort panel; <=0 means auto")
    args = ap.parse_args()

    data_queue: SimpleQueue = SimpleQueue()
    parser = StreamParser(fallback_fps=args.fps_fallback)

    bg_worker = None
    if args.demo:
        bg_worker = DemoGenerator(
            data_queue,
            fps=args.demo_fps,
            lag_ms=args.demo_lag_ms,
            invert=args.demo_invert,
        )
        bg_worker.start()
    elif args.cmd:
        bg_worker = CmdReader(data_queue, parser, cmd=args.cmd)
        bg_worker.start()
    else:
        bg_worker = StreamReader(data_queue, parser, input_path=args.input, from_start=args.from_start)
        bg_worker.start()

    cmp_t_buf = deque(maxlen=args.max_buffer)
    lk_buf = deque(maxlen=args.max_buffer)
    cmp_gyro_buf = deque(maxlen=args.max_buffer)

    gyro_t_buf = deque(maxlen=args.max_buffer)
    gx_buf = deque(maxlen=args.max_buffer)
    gy_buf = deque(maxlen=args.max_buffer)
    gz_buf = deque(maxlen=args.max_buffer)
    phys_x_buf = deque(maxlen=args.max_buffer)
    phys_y_buf = deque(maxlen=args.max_buffer)
    phys_z_buf = deque(maxlen=args.max_buffer)
    corr_x_buf = deque(maxlen=args.max_buffer)
    corr_y_buf = deque(maxlen=args.max_buffer)
    corr_z_buf = deque(maxlen=args.max_buffer)

    tank_t_buf = deque(maxlen=args.max_buffer)
    tank_left_buf = deque(maxlen=args.max_buffer)
    tank_right_buf = deque(maxlen=args.max_buffer)

    latest_hz = float("nan")
    latest_req_crop = float("nan")
    latest_req_scale = float("nan")
    latest_pix = float("nan")
    latest_warp_mode = "n/a"
    latest_tank = {"left": 0, "right": 0, "pwm": 0}

    fig, axes = plt.subplots(
        6,
        1,
        figsize=(13, 15),
        gridspec_kw={"height_ratios": [2.0, 2.0, 1.3, 1.8, 2.0, 1.1]},
    )
    ax1, ax2, ax3, ax4, ax5, ax6 = axes
    fig.suptitle("LK vs Gyro Live Compare + Axis/Drive Monitor", fontsize=13)

    line_lk_1, = ax1.plot([], [], label="lk_da", lw=1.8)
    line_gy_1, = ax1.plot([], [], label="gyro_dyaw", lw=1.5, alpha=0.9)
    ax1.set_ylabel("deg")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="upper left")
    ax1.set_title("Raw overlay")

    line_lk_2, = ax2.plot([], [], label="lk_da", lw=1.8)
    line_gy_shift, = ax2.plot([], [], label="shifted gyro", lw=1.6)
    line_diff, = ax2.plot([], [], label="diff(lk-shifted)", lw=1.0, linestyle="--", alpha=0.8)
    ax2.set_ylabel("deg")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="upper left")
    ax2.set_title("Best-lag aligned compare")

    line_corr_norm, = ax3.plot([], [], label="corr(normal)", lw=1.4)
    line_corr_inv, = ax3.plot([], [], label="corr(invert)", lw=1.4, linestyle="--")
    line_best_v = ax3.axvline(0.0, color="r", lw=1.2, linestyle=":")
    ax3.set_xlabel("lag (ms)")
    ax3.set_ylabel("corr")
    ax3.grid(True, alpha=0.3)
    ax3.set_ylim(-1.05, 1.05)
    ax3.set_xlim(-args.lag_max_ms, args.lag_max_ms)
    ax3.legend(loc="upper left")
    ax3.set_title("Lag sweep (recent window)")

    line_gx, = ax4.plot([], [], label="gx (deg/s)")
    line_gy, = ax4.plot([], [], label="gy (deg/s)")
    line_gz, = ax4.plot([], [], label="gz (deg/s)")
    ax4.set_ylabel("deg/s")
    ax4.grid(True, alpha=0.3)
    ax4.legend(loc="upper left", ncol=3)
    ax4.set_title("Gyro rates")

    line_phys_x, = ax5.plot([], [], label="phys x", lw=1.7)
    line_phys_y, = ax5.plot([], [], label="phys y", lw=1.7)
    line_phys_z, = ax5.plot([], [], label="phys z", lw=1.7)
    line_corr_x, = ax5.plot([], [], label="corr x", lw=1.1, linestyle="--")
    line_corr_y, = ax5.plot([], [], label="corr y", lw=1.1, linestyle="--")
    line_corr_z, = ax5.plot([], [], label="corr z", lw=1.1, linestyle="--")
    ax5.set_ylabel("deg")
    ax5.grid(True, alpha=0.3)
    ax5.legend(loc="upper left", ncol=3)
    ax5.set_title("Physical angle vs applied correction")

    line_tank_left, = ax6.plot([], [], label="left effort", lw=1.8, drawstyle="steps-post")
    line_tank_right, = ax6.plot([], [], label="right effort", lw=1.8, drawstyle="steps-post")
    ax6.axhline(0.0, color="k", lw=0.8, alpha=0.35)
    ax6.set_xlabel("time in window (s)")
    ax6.set_ylabel("PWM")
    ax6.grid(True, alpha=0.3)
    ax6.legend(loc="upper left", ncol=2)
    ax6.set_title("Tank drive command")

    text_status = ax3.text(
        0.99,
        0.05,
        "waiting for compare data...",
        transform=ax3.transAxes,
        ha="right",
        va="bottom",
        fontsize=10,
        bbox=dict(facecolor="white", alpha=0.65, edgecolor="none"),
    )
    text_rate = ax4.text(
        0.99,
        0.96,
        "waiting for [GYRO] logs...",
        transform=ax4.transAxes,
        ha="right",
        va="top",
        fontsize=9,
        bbox=dict(facecolor="white", alpha=0.7, edgecolor="none"),
    )
    text_policy = ax5.text(
        0.99,
        0.96,
        "waiting for policy data...",
        transform=ax5.transAxes,
        ha="right",
        va="top",
        fontsize=9,
        bbox=dict(facecolor="white", alpha=0.7, edgecolor="none"),
    )
    text_drive = ax6.text(
        0.99,
        0.96,
        "waiting for [TANK] logs...",
        transform=ax6.transAxes,
        ha="right",
        va="top",
        fontsize=9,
        bbox=dict(facecolor="white", alpha=0.7, edgecolor="none"),
    )

    def animate(_):
        nonlocal latest_hz, latest_req_crop, latest_req_scale, latest_pix, latest_warp_mode, latest_tank

        while True:
            try:
                event = data_queue.get_nowait()
            except Empty:
                break

            kind = event.get("kind")
            if kind == "cmp":
                cmp_t_buf.append(float(event["t_ms"]))
                lk_buf.append(float(event["lk_da"]))
                cmp_gyro_buf.append(float(event["gyro_dyaw"]))
            elif kind == "gyro":
                gyro_t_buf.append(float(event["t_ms"]))
                gx, gyv, gz = event["rate_deg"]
                px, py, pz = event["phys_deg"]
                cx, cy, cz = event["corr_deg"]
                gx_buf.append(float(gx))
                gy_buf.append(float(gyv))
                gz_buf.append(float(gz))
                phys_x_buf.append(float(px))
                phys_y_buf.append(float(py))
                phys_z_buf.append(float(pz))
                corr_x_buf.append(float(cx))
                corr_y_buf.append(float(cy))
                corr_z_buf.append(float(cz))
                latest_hz = float(event["hz"])
            elif kind == "warp":
                latest_warp_mode = str(event["mode"])
                latest_pix = float(event["pixel_disp"])
                latest_req_scale = float(event["req_scale"])
                latest_req_crop = float(event["req_crop"])
            elif kind == "tank":
                t_ms = float(event["t_ms"])
                left = int(event["left"])
                right = int(event["right"])
                pwm = int(event["pwm"])
                tank_t_buf.append(t_ms)
                tank_left_buf.append(left * pwm)
                tank_right_buf.append(right * pwm)
                latest_tank = {"left": left, "right": right, "pwm": pwm}

        end_ms = latest_time_ms(cmp_t_buf, gyro_t_buf, tank_t_buf)
        if end_ms is None:
            return (
                line_lk_1,
                line_gy_1,
                line_lk_2,
                line_gy_shift,
                line_diff,
                line_corr_norm,
                line_corr_inv,
                line_best_v,
                line_gx,
                line_gy,
                line_gz,
                line_phys_x,
                line_phys_y,
                line_phys_z,
                line_corr_x,
                line_corr_y,
                line_corr_z,
                line_tank_left,
                line_tank_right,
                text_status,
                text_rate,
                text_policy,
                text_drive,
            )

        disp_start_ms = end_ms - args.display_sec * 1000.0

        ax1.set_xlim(0.0, args.display_sec)
        ax2.set_xlim(0.0, args.display_sec)
        ax4.set_xlim(0.0, args.display_sec)
        ax5.set_xlim(0.0, args.display_sec)
        ax6.set_xlim(0.0, args.display_sec)

        if len(cmp_t_buf) >= 2:
            t = np.asarray(cmp_t_buf, dtype=float)
            lk = np.asarray(lk_buf, dtype=float)
            gy = np.asarray(cmp_gyro_buf, dtype=float)
            cmp_sel, cmp_x = select_window(t, disp_start_ms)
            lk_d = lk[cmp_sel]
            gy_d = gy[cmp_sel]

            if cmp_x.size >= 1:
                line_lk_1.set_data(cmp_x, lk_d)
                line_gy_1.set_data(cmp_x, gy_d)
                update_axis_ylim(ax1, [lk_d, gy_d], default=(-1.0, 1.0))
            else:
                line_lk_1.set_data([], [])
                line_gy_1.set_data([], [])

            est = estimate_best_lag(
                t_ms=t,
                lk_da=lk,
                gyro_dyaw=gy,
                analysis_sec=args.analysis_sec,
                lag_max_ms=args.lag_max_ms,
                lag_step_ms=args.lag_step_ms,
                sign_mode=args.sign,
                min_points=12,
            )

            if est is None or cmp_x.size < 1:
                line_lk_2.set_data(cmp_x, lk_d)
                line_gy_shift.set_data([], [])
                line_diff.set_data([], [])
                line_corr_norm.set_data([], [])
                line_corr_inv.set_data([], [])
                line_best_v.set_xdata([0.0, 0.0])
                text_status.set_text("insufficient data for lag estimate")
                ax2.set_title("Best-lag aligned compare")
            else:
                best_lag = est["best_lag"]
                best_corr = est["best_corr"]
                best_sign_name = est["best_sign_name"]
                best_sign_mul = est["best_sign_mul"]
                shifted = best_sign_mul * np.interp(
                    t[cmp_sel] - best_lag,
                    t,
                    gy,
                    left=np.nan,
                    right=np.nan,
                )
                diff = lk_d - shifted

                line_lk_2.set_data(cmp_x, lk_d)
                line_gy_shift.set_data(cmp_x, shifted)
                line_diff.set_data(cmp_x, diff)
                update_axis_ylim(ax2, [lk_d, shifted, diff], default=(-1.0, 1.0))

                lags = est["lags"]
                curves = est["curves"]
                line_corr_norm.set_data(lags, curves.get("normal", np.array([])))
                line_corr_inv.set_data(lags, curves.get("invert", np.array([])))
                line_best_v.set_xdata([best_lag, best_lag])
                text_status.set_text(
                    f"best lag={best_lag:+.1f} ms, sign={best_sign_name}, corr={best_corr:.3f}"
                )
                ax2.set_title(
                    f"Best-lag aligned compare | lag={best_lag:+.1f} ms, "
                    f"sign={best_sign_name}, corr={best_corr:.3f}"
                )
        else:
            line_lk_1.set_data([], [])
            line_gy_1.set_data([], [])
            line_lk_2.set_data([], [])
            line_gy_shift.set_data([], [])
            line_diff.set_data([], [])
            line_corr_norm.set_data([], [])
            line_corr_inv.set_data([], [])
            line_best_v.set_xdata([0.0, 0.0])
            text_status.set_text("waiting for [CMP] or CSV compare data...")

        if len(gyro_t_buf) >= 1:
            gt = np.asarray(gyro_t_buf, dtype=float)
            gyro_sel, gyro_x = select_window(gt, disp_start_ms)

            gx = np.asarray(gx_buf, dtype=float)[gyro_sel]
            gyv = np.asarray(gy_buf, dtype=float)[gyro_sel]
            gz = np.asarray(gz_buf, dtype=float)[gyro_sel]
            px = np.asarray(phys_x_buf, dtype=float)[gyro_sel]
            py = np.asarray(phys_y_buf, dtype=float)[gyro_sel]
            pz = np.asarray(phys_z_buf, dtype=float)[gyro_sel]
            cx = np.asarray(corr_x_buf, dtype=float)[gyro_sel]
            cy = np.asarray(corr_y_buf, dtype=float)[gyro_sel]
            cz = np.asarray(corr_z_buf, dtype=float)[gyro_sel]

            line_gx.set_data(gyro_x, gx)
            line_gy.set_data(gyro_x, gyv)
            line_gz.set_data(gyro_x, gz)
            update_axis_ylim(ax4, [gx, gyv, gz], default=(-30.0, 30.0), fixed_abs=args.rate_ylim)

            line_phys_x.set_data(gyro_x, px)
            line_phys_y.set_data(gyro_x, py)
            line_phys_z.set_data(gyro_x, pz)
            line_corr_x.set_data(gyro_x, cx)
            line_corr_y.set_data(gyro_x, cy)
            line_corr_z.set_data(gyro_x, cz)

            phys_abs = args.angle_ylim if args.angle_ylim > 0 else None
            corr_abs = args.corr_ylim if args.corr_ylim > 0 else None
            fixed_abs = None
            if phys_abs is not None or corr_abs is not None:
                fixed_abs = max(v for v in [phys_abs, corr_abs] if v is not None)
            update_axis_ylim(
                ax5,
                [px, py, pz, cx, cy, cz],
                default=(-12.0, 12.0),
                fixed_abs=fixed_abs,
            )

            gx_now = float(gx[-1])
            gy_now = float(gyv[-1])
            gz_now = float(gz[-1])
            px_now = float(px[-1])
            py_now = float(py[-1])
            pz_now = float(pz[-1])
            cx_now = float(cx[-1])
            cy_now = float(cy[-1])
            cz_now = float(cz[-1])
            dom_axis = dominant_axis_name(gx_now, gy_now, gz_now)
            corr_mag = float(np.linalg.norm([cx_now, cy_now, cz_now]))

            ax4.set_title(f"Gyro rates (dominant axis: {dom_axis})")
            text_rate.set_text(
                f"gx={gx_now:+6.2f}  gy={gy_now:+6.2f}  gz={gz_now:+6.2f} deg/s\n"
                f"hz={latest_hz:5.1f}"
            )
            text_policy.set_text(
                f"phys=({px_now:+5.2f}, {py_now:+5.2f}, {pz_now:+5.2f}) deg\n"
                f"corr=({cx_now:+5.2f}, {cy_now:+5.2f}, {cz_now:+5.2f}) deg  |corr|={corr_mag:.2f}\n"
                f"mode={latest_warp_mode} req_crop={latest_req_crop:.1f}% "
                f"req_scale={latest_req_scale:.2f} pix={latest_pix:.1f}"
            )
        else:
            line_gx.set_data([], [])
            line_gy.set_data([], [])
            line_gz.set_data([], [])
            line_phys_x.set_data([], [])
            line_phys_y.set_data([], [])
            line_phys_z.set_data([], [])
            line_corr_x.set_data([], [])
            line_corr_y.set_data([], [])
            line_corr_z.set_data([], [])
            text_rate.set_text("waiting for [GYRO] logs...")
            text_policy.set_text("waiting for [GYRO]/[HIGHPASS]/[DELTA] logs...")

        if len(tank_t_buf) >= 1:
            tt = np.asarray(tank_t_buf, dtype=float)
            tank_sel, tank_x = select_window(tt, disp_start_ms)
            left_eff = np.asarray(tank_left_buf, dtype=float)[tank_sel]
            right_eff = np.asarray(tank_right_buf, dtype=float)[tank_sel]
            line_tank_left.set_data(tank_x, left_eff)
            line_tank_right.set_data(tank_x, right_eff)
            update_axis_ylim(ax6, [left_eff, right_eff], default=(-255.0, 255.0), fixed_abs=args.drive_ylim)
            text_drive.set_text(
                f"L={latest_tank['left']}  R={latest_tank['right']}  PWM={latest_tank['pwm']}\n"
                f"left_eff={latest_tank['left'] * latest_tank['pwm']:+d}  "
                f"right_eff={latest_tank['right'] * latest_tank['pwm']:+d}"
            )
        else:
            line_tank_left.set_data([], [])
            line_tank_right.set_data([], [])
            text_drive.set_text("waiting for [TANK] logs...")

        return (
            line_lk_1,
            line_gy_1,
            line_lk_2,
            line_gy_shift,
            line_diff,
            line_corr_norm,
            line_corr_inv,
            line_best_v,
            line_gx,
            line_gy,
            line_gz,
            line_phys_x,
            line_phys_y,
            line_phys_z,
            line_corr_x,
            line_corr_y,
            line_corr_z,
            line_tank_left,
            line_tank_right,
            text_status,
            text_rate,
            text_policy,
            text_drive,
        )

    ani = FuncAnimation(fig, animate, interval=args.interval_ms, blit=False)
    print("Live LK/Gyro compare + axis/drive monitor started. Close the window to exit.", flush=True)
    try:
        plt.tight_layout()
        plt.show()
    finally:
        if bg_worker is not None:
            bg_worker.stop()
        _ = ani


if __name__ == "__main__":
    main()
