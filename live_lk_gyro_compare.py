#!/usr/bin/env python3
"""
Live LK vs gyro compare plotter.

This tool visualizes offset-calibration compare signals in real time:
  - LK frame-to-frame rotation (lk_da)
  - IMU integrated yaw delta (gyro_dyaw)

Accepted input line formats:
1) CSV (recommended): t_ms,lk_da,gyro_dyaw
   - Header line is allowed.
2) Key-value text:
   - "... t_ms=123.4 ... lk_da=0.12 ... gyro_dyaw=0.10 ..."
3) Existing [CMP] line in eis_capture.cpp:
   - "[CMP] lk_da=...deg gyro_dyaw=...deg diff=... corr=..."
   - When timestamp is missing, frame-index based pseudo-time is generated.
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
from typing import Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation


TS_PAT = re.compile(r"(?:t_ms|ts_ms|frame_time_ms)\s*=\s*([-+]?\d*\.?\d+)")
LK_PAT = re.compile(r"lk_da\s*=\s*([-+]?\d*\.?\d+)")
GYRO_PAT = re.compile(r"gyro_dyaw\s*=\s*([-+]?\d*\.?\d+)")


def corrcoef_safe(a: np.ndarray, b: np.ndarray) -> float:
    if a.size < 3 or b.size < 3:
        return float("nan")
    aa = a - np.mean(a)
    bb = b - np.mean(b)
    denom = np.linalg.norm(aa) * np.linalg.norm(bb)
    if denom < 1e-12:
        return float("nan")
    return float(np.dot(aa, bb) / denom)


class StreamParser:
    def __init__(self, fallback_fps: float):
        self.fallback_dt_ms = 1000.0 / max(1e-6, fallback_fps)
        self.last_t_ms: Optional[float] = None

    def _next_fallback_t(self) -> float:
        if self.last_t_ms is None:
            self.last_t_ms = 0.0
        else:
            self.last_t_ms += self.fallback_dt_ms
        return self.last_t_ms

    def parse_line(self, line: str) -> Optional[Tuple[float, float, float]]:
        line = line.strip()
        if not line:
            return None

        # CSV: t_ms,lk_da,gyro_dyaw (header allowed)
        if "," in line:
            parts = [p.strip() for p in line.split(",")]
            if len(parts) >= 3:
                try:
                    t_ms = float(parts[0])
                    lk_da = float(parts[1])
                    gyro_dyaw = float(parts[2])
                    self.last_t_ms = t_ms
                    return t_ms, lk_da, gyro_dyaw
                except ValueError:
                    pass

        # Key-value or [CMP] line
        lk_m = LK_PAT.search(line)
        gy_m = GYRO_PAT.search(line)
        if lk_m and gy_m:
            lk_da = float(lk_m.group(1))
            gyro_dyaw = float(gy_m.group(1))
            ts_m = TS_PAT.search(line)
            if ts_m:
                t_ms = float(ts_m.group(1))
                self.last_t_ms = t_ms
            else:
                t_ms = self._next_fallback_t()
            return t_ms, lk_da, gyro_dyaw

        return None


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

    def _run_stdin(self) -> None:
        while not self.stop_event.is_set():
            line = sys.stdin.readline()
            if not line:
                time.sleep(0.01)
                continue
            sample = self.parser.parse_line(line)
            if sample is not None:
                self.queue.put(sample)

    def _run_file_tail(self, path: str) -> None:
        with open(path, "r", encoding="utf-8", errors="ignore") as f:
            if not self.from_start:
                f.seek(0, 2)
            while not self.stop_event.is_set():
                line = f.readline()
                if not line:
                    time.sleep(0.01)
                    continue
                sample = self.parser.parse_line(line)
                if sample is not None:
                    self.queue.put(sample)


class CmdReader(threading.Thread):
    """Run external command and parse its stdout/stderr stream in real-time."""

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
            bufsize=1,  # line buffered
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
            sample = self.parser.parse_line(line)
            if sample is not None:
                self.queue.put(sample)


class DemoGenerator(threading.Thread):
    """Generate synthetic LK/Gyro stream for quick visualization checks."""

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
            # Composite motion-like waveform.
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
            # Small noise to look more realistic.
            gyro += float(np.random.normal(0.0, 0.08))
            lk += float(np.random.normal(0.0, 0.05))

            self.queue.put((t_ms, lk, gyro))
            time.sleep(self.dt)


def update_axis_ylim(ax, arrays, default=(-1.0, 1.0)):
    ys = []
    for arr in arrays:
        if arr is None:
            continue
        arr = np.asarray(arr)
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

    sign_candidates = [("normal", 1.0), ("invert", -1.0)] if sign_mode == "auto" else [(sign_mode, 1.0 if sign_mode == "normal" else -1.0)]

    best = None
    curves = {}

    for sign_name, sign_mul in sign_candidates:
        corr_curve = np.full_like(lags, np.nan, dtype=float)
        for i, lag in enumerate(lags):
            # Compare lk(t) vs sign * gyro(t - lag)
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


def main():
    ap = argparse.ArgumentParser(description="Live LK vs gyro compare visualizer")
    ap.add_argument("--input", type=str, default=None, help="append-only input file path (tail -f style). If omitted, read from stdin.")
    ap.add_argument("--cmd", type=str, default=None, help="run command and parse realtime stdout/stderr (e.g., your C++ app)")
    ap.add_argument("--from-start", action="store_true", help="for --input mode, read from beginning instead of seeking to end")
    ap.add_argument("--demo", action="store_true", help="run synthetic realtime data generator (no external input needed)")
    ap.add_argument("--demo-fps", type=float, default=30.0, help="synthetic generator FPS")
    ap.add_argument("--demo-lag-ms", type=float, default=12.0, help="synthetic gyro lag for demo")
    ap.add_argument("--demo-invert", action="store_true", help="invert synthetic gyro sign for demo")
    ap.add_argument("--display-sec", type=float, default=8.0, help="visible rolling time window in seconds")
    ap.add_argument("--analysis-sec", type=float, default=2.5, help="recent window for lag estimation in seconds")
    ap.add_argument("--lag-max-ms", type=float, default=80.0, help="lag sweep range [-max, +max] ms")
    ap.add_argument("--lag-step-ms", type=float, default=2.0, help="lag sweep step in ms")
    ap.add_argument("--sign", choices=["auto", "normal", "invert"], default="auto", help="gyro sign mode")
    ap.add_argument("--fps-fallback", type=float, default=30.0, help="pseudo-time FPS when input line has no timestamp")
    ap.add_argument("--max-buffer", type=int, default=6000, help="raw sample buffer size")
    ap.add_argument("--interval-ms", type=int, default=50, help="plot refresh interval")
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

    t_buf = deque(maxlen=args.max_buffer)
    lk_buf = deque(maxlen=args.max_buffer)
    gyro_buf = deque(maxlen=args.max_buffer)

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 8), gridspec_kw={"height_ratios": [2.2, 2.2, 1.4]})
    fig.suptitle("LK vs Gyro Live Compare", fontsize=12)

    line_lk_1, = ax1.plot([], [], label="lk_da", lw=1.8)
    line_gy_1, = ax1.plot([], [], label="gyro_dyaw", lw=1.5, alpha=0.9)
    ax1.set_ylabel("deg")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="upper left")
    ax1.set_title("Raw overlay")

    line_lk_2, = ax2.plot([], [], label="lk_da", lw=1.8)
    line_gy_shift, = ax2.plot([], [], label="shifted gyro", lw=1.6)
    line_diff, = ax2.plot([], [], label="diff(lk-shifted)", lw=1.0, linestyle="--", alpha=0.8)
    ax2.set_xlabel("time in window (s)")
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

    text_status = ax3.text(
        0.99,
        0.05,
        "waiting for data...",
        transform=ax3.transAxes,
        ha="right",
        va="bottom",
        fontsize=10,
        bbox=dict(facecolor="white", alpha=0.65, edgecolor="none"),
    )

    def animate(_):
        while True:
            try:
                t_ms, lk, gy = data_queue.get_nowait()
            except Empty:
                break
            t_buf.append(t_ms)
            lk_buf.append(lk)
            gyro_buf.append(gy)

        if len(t_buf) < 2:
            return (
                line_lk_1,
                line_gy_1,
                line_lk_2,
                line_gy_shift,
                line_diff,
                line_corr_norm,
                line_corr_inv,
                line_best_v,
                text_status,
            )

        t = np.asarray(t_buf, dtype=float)
        lk = np.asarray(lk_buf, dtype=float)
        gy = np.asarray(gyro_buf, dtype=float)

        disp_start_ms = t[-1] - args.display_sec * 1000.0
        disp_sel = t >= disp_start_ms
        td = t[disp_sel]
        lk_d = lk[disp_sel]
        gy_d = gy[disp_sel]
        if td.size < 2:
            return (
                line_lk_1,
                line_gy_1,
                line_lk_2,
                line_gy_shift,
                line_diff,
                line_corr_norm,
                line_corr_inv,
                line_best_v,
                text_status,
            )

        x = (td - td[0]) / 1000.0
        line_lk_1.set_data(x, lk_d)
        line_gy_1.set_data(x, gy_d)
        ax1.set_xlim(x[0], x[-1] if x[-1] > x[0] else x[0] + 1.0)
        update_axis_ylim(ax1, [lk_d, gy_d], default=(-1.0, 1.0))

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

        if est is None:
            line_lk_2.set_data(x, lk_d)
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
            shifted = best_sign_mul * np.interp(td - best_lag, t, gy, left=np.nan, right=np.nan)
            diff = lk_d - shifted

            line_lk_2.set_data(x, lk_d)
            line_gy_shift.set_data(x, shifted)
            line_diff.set_data(x, diff)
            ax2.set_xlim(x[0], x[-1] if x[-1] > x[0] else x[0] + 1.0)
            update_axis_ylim(ax2, [lk_d, shifted, diff], default=(-1.0, 1.0))

            lags = est["lags"]
            curves = est["curves"]
            if "normal" in curves:
                line_corr_norm.set_data(lags, curves["normal"])
            else:
                line_corr_norm.set_data([], [])
            if "invert" in curves:
                line_corr_inv.set_data(lags, curves["invert"])
            else:
                line_corr_inv.set_data([], [])
            line_best_v.set_xdata([best_lag, best_lag])
            text_status.set_text(f"best lag={best_lag:+.1f} ms, sign={best_sign_name}, corr={best_corr:.3f}")
            ax2.set_title(f"Best-lag aligned compare | lag={best_lag:+.1f} ms, sign={best_sign_name}, corr={best_corr:.3f}")

        return (
            line_lk_1,
            line_gy_1,
            line_lk_2,
            line_gy_shift,
            line_diff,
            line_corr_norm,
            line_corr_inv,
            line_best_v,
            text_status,
        )

    ani = FuncAnimation(fig, animate, interval=args.interval_ms, blit=False)
    print("Live LK/Gyro compare started. Close the window to exit.", flush=True)
    try:
        plt.tight_layout()
        plt.show()
    finally:
        if bg_worker is not None:
            bg_worker.stop()
        _ = ani


if __name__ == "__main__":
    main()
