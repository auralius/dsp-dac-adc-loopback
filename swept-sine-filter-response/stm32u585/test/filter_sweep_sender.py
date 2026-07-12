#!/usr/bin/env python3
"""
PC-side frequency sweep controller for STM32 DAC-ADC loopback filter test.

Features:
- PC controls the sweep by sending: RUN <freq_hz> <N> <settle_N>\n
- STM32 returns:
    OK,<freq_hz>,<N>,<settle_N>
    BEGIN,<freq_hz>,<actual_fs_hz>,<N>[,extra...]
    DATA,<n>,<x>,<y>
    ...
    END

- Computes magnitude/phase using synchronous detection
- Saves raw CSV blocks
- Saves Bode plots
- Saves time-domain waveform PNG for each frequency (input vs output)
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import serial
import matplotlib.pyplot as plt


@dataclass
class SweepResult:
    f_hz: float
    fs_hz: float
    n_samples: int
    gain: float
    gain_db: float
    phase_deg: float              # wrapped phase in [-180, 180)
    phase_unwrapped_deg: float    # filled after the sweep is complete
    group_delay_s: float          # filled after the sweep is complete
    amp_in: float
    amp_out: float
    skipped_lines: int



def wrap_phase_deg(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0



def compute_phase_unwrap_and_group_delay(results: list[SweepResult]) -> None:
    """Fill phase_unwrapped_deg and group_delay_s in-place."""
    if not results:
        return

    idx = np.argsort(np.asarray([r.f_hz for r in results], dtype=np.float64))
    f = np.asarray([results[i].f_hz for i in idx], dtype=np.float64)
    phase_wrapped_deg = np.asarray([results[i].phase_deg for i in idx], dtype=np.float64)

    phase_unwrapped_rad = np.unwrap(np.deg2rad(phase_wrapped_deg))
    phase_unwrapped_deg = np.rad2deg(phase_unwrapped_rad)

    omega = 2.0 * np.pi * f
    if len(results) >= 2:
        # tau_g = -d(phi)/d(omega), with phi in radians.
        group_delay_s = -np.gradient(phase_unwrapped_rad, omega)
    else:
        group_delay_s = np.asarray([float("nan")], dtype=np.float64)

    for j, i in enumerate(idx):
        results[i].phase_unwrapped_deg = float(phase_unwrapped_deg[j])
        results[i].group_delay_s = float(group_delay_s[j])



def wait_for_begin(ser: serial.Serial, timeout_s: float = 180.0) -> tuple[float, float, int]:
    t_start = time.monotonic()

    while True:
        if time.monotonic() - t_start > timeout_s:
            raise TimeoutError("Timed out waiting for BEGIN from STM32")

        raw = ser.readline()
        if not raw:
            continue

        line = raw.decode(errors="replace").strip()
        if not line:
            continue

        print(line)

        if line.startswith("ERR"):
            raise RuntimeError(f"STM32 returned error: {line}")

        if line.startswith("BEGIN"):
            parts = line.split(",")
            if len(parts) < 4:
                raise ValueError(f"Malformed BEGIN line: {line}")
            f_hz = float(parts[1])
            fs_hz = float(parts[2])
            n_samples = int(parts[3])
            return f_hz, fs_hz, n_samples



def parse_sample_line(line: str) -> tuple[float, float] | None:
    if line.startswith("DATA,"):
        parts = line.split(",")
        if len(parts) != 4:
            return None
        return float(parts[2]), float(parts[3])

    parts = line.split(",")
    if len(parts) == 3:
        try:
            int(parts[0])
            return float(parts[1]), float(parts[2])
        except ValueError:
            return None

    return None



def read_block(ser: serial.Serial, expected_n: int, timeout_s: float = 180.0) -> tuple[np.ndarray, np.ndarray, int]:
    xs: list[float] = []
    ys: list[float] = []
    skipped = 0
    t_start = time.monotonic()

    while True:
        if time.monotonic() - t_start > timeout_s:
            raise TimeoutError(
                f"Timed out reading block: got {len(xs)} / expected {expected_n} samples"
            )

        raw = ser.readline()
        if not raw:
            continue

        line = raw.decode(errors="replace").strip()
        if not line:
            continue

        if line == "END":
            break

        parsed = parse_sample_line(line)
        if parsed is None:
            skipped += 1
            if skipped <= 10:
                print(f"skip malformed: {line}")
            continue

        x, y = parsed
        xs.append(x)
        ys.append(y)

    if len(xs) == 0:
        raise RuntimeError("No valid sample lines received")

    if len(xs) != expected_n:
        print(f"WARNING: received {len(xs)} samples, STM32 reported {expected_n}")

    return np.asarray(xs, dtype=np.float64), np.asarray(ys, dtype=np.float64), skipped



def run_one_frequency(
    ser: serial.Serial,
    f_hz: float,
    n_samples: int,
    settle_n: int,
) -> tuple[float, float, np.ndarray, np.ndarray, int]:
    ser.reset_input_buffer()
    time.sleep(0.05)

    cmd = f"RUN {f_hz:.6f} {n_samples:d} {settle_n:d}\n"
    print(f"--> {cmd.strip()}")
    ser.write(cmd.encode("ascii"))
    ser.flush()

    f_reported, fs_reported, n_reported = wait_for_begin(ser)
    x, y, skipped = read_block(ser, n_reported)

    return f_reported, fs_reported, x, y, skipped



def estimate_gain_phase(
    x: np.ndarray,
    y: np.ndarray,
    f_hz: float,
    fs_hz: float,
    discard_start: int = 0,
) -> tuple[float, float, float, float, float]:
    if discard_start > 0:
        x = x[discard_start:]
        y = y[discard_start:]

    n = np.arange(len(x), dtype=np.float64)
    omega = 2.0 * np.pi * f_hz / fs_hz

    x = x - np.mean(x)
    y = y - np.mean(y)

    s = np.sin(omega * n)
    c = np.cos(omega * n)

    xs = np.sum(x * s)
    xc = np.sum(x * c)
    ys = np.sum(y * s)
    yc = np.sum(y * c)

    amp_x = (2.0 / len(x)) * math.sqrt(xs * xs + xc * xc)
    amp_y = (2.0 / len(y)) * math.sqrt(ys * ys + yc * yc)

    if amp_x <= 1e-12:
        raise RuntimeError("Input amplitude is too small; check wiring or DAC output")

    gain = amp_y / amp_x
    gain_db = 20.0 * math.log10(gain) if gain > 0 else -math.inf

    phase_x = math.atan2(xc, xs)
    phase_y = math.atan2(yc, ys)
    phase_deg = wrap_phase_deg(math.degrees(phase_y - phase_x))

    return gain, gain_db, phase_deg, amp_x, amp_y



def save_raw_block(out_dir: Path, f_hz: float, fs_hz: float, x: np.ndarray, y: np.ndarray) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    safe_f = f"{f_hz:g}".replace(".", "p")
    filename = out_dir / f"raw_{safe_f}Hz.csv"

    with filename.open("w", newline="") as fp:
        writer = csv.writer(fp)
        writer.writerow(["n", "t_s", "x", "y"])
        for n, (xi, yi) in enumerate(zip(x, y)):
            writer.writerow([n, n / fs_hz, xi, yi])



def save_summary(out_dir: Path, results: list[SweepResult]) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / "sweep_summary.csv"

    with path.open("w", newline="") as fp:
        writer = csv.writer(fp)
        writer.writerow([
            "f_hz",
            "fs_hz",
            "n_samples",
            "gain",
            "gain_db",
            "phase_deg_wrapped",
            "phase_deg_unwrapped",
            "group_delay_s",
            "group_delay_ms",
            "amp_in",
            "amp_out",
            "skipped_lines",
        ])
        for r in results:
            writer.writerow([
                r.f_hz,
                r.fs_hz,
                r.n_samples,
                r.gain,
                r.gain_db,
                r.phase_deg,
                r.phase_unwrapped_deg,
                r.group_delay_s,
                1000.0 * r.group_delay_s if math.isfinite(r.group_delay_s) else float("nan"),
                r.amp_in,
                r.amp_out,
                r.skipped_lines,
            ])

    return path



def plot_results(results: list[SweepResult], out_dir: Path | None = None, show: bool = True) -> None:
    f = np.asarray([r.f_hz for r in results], dtype=np.float64)
    gain_db = np.asarray([r.gain_db for r in results], dtype=np.float64)
    phase_wrapped_deg = np.asarray([r.phase_deg for r in results], dtype=np.float64)
    phase_unwrapped_deg = np.asarray([r.phase_unwrapped_deg for r in results], dtype=np.float64)
    group_delay_ms = 1000.0 * np.asarray([r.group_delay_s for r in results], dtype=np.float64)

    idx = np.argsort(f)
    f = f[idx]
    gain_db = gain_db[idx]
    phase_wrapped_deg = phase_wrapped_deg[idx]
    phase_unwrapped_deg = phase_unwrapped_deg[idx]
    group_delay_ms = group_delay_ms[idx]

    plt.figure()
    plt.semilogx(f, gain_db, "o-")
    plt.grid(True, which="both")
    plt.xlabel("Frequency [Hz]")
    plt.ylabel("Gain [dB]")
    plt.title("Measured Filter Magnitude Response")
    if out_dir is not None:
        plt.savefig(out_dir / "magnitude_response.png", dpi=160, bbox_inches="tight")

    plt.figure()
    plt.semilogx(f, phase_wrapped_deg, "o-")
    plt.grid(True, which="both")
    plt.xlabel("Frequency [Hz]")
    plt.ylabel("Wrapped Phase [deg]")
    plt.title("Measured Filter Phase Response (Wrapped)")
    if out_dir is not None:
        plt.savefig(out_dir / "phase_response_wrapped.png", dpi=160, bbox_inches="tight")

    plt.figure()
    plt.semilogx(f, phase_unwrapped_deg, "o-")
    plt.grid(True, which="both")
    plt.xlabel("Frequency [Hz]")
    plt.ylabel("Unwrapped Phase [deg]")
    plt.title("Measured Filter Phase Response (Unwrapped)")
    if out_dir is not None:
        plt.savefig(out_dir / "phase_response_unwrapped.png", dpi=160, bbox_inches="tight")

    plt.figure()
    plt.semilogx(f, group_delay_ms, "o-")
    plt.grid(True, which="both")
    plt.xlabel("Frequency [Hz]")
    plt.ylabel("Group Delay [ms]")
    plt.title("Estimated Group Delay")
    if out_dir is not None:
        plt.savefig(out_dir / "group_delay.png", dpi=160, bbox_inches="tight")

    if show:
        plt.show()
    else:
        plt.close("all")



def save_waveform_plot(
    out_dir: Path,
    f_hz: float,
    fs_hz: float,
    x: np.ndarray,
    y: np.ndarray,
    gain_db: float,
    phase_deg: float,
    amp_in: float,
    amp_out: float,
    cycles: float = 4.0,
) -> Path:
    """Save one time-domain waveform image (input vs output) for a frequency.

    If the requested number of cycles exceeds the available capture length,
    it falls back to the full captured block.
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    safe_f = f"{f_hz:g}".replace(".", "p")
    path = out_dir / f"waveform_{safe_f}Hz.png"

    n_total = len(x)
    if f_hz > 0.0:
        samples_per_cycle = fs_hz / f_hz
        n_show = int(round(cycles * samples_per_cycle))
    else:
        n_show = n_total

    if n_show <= 0 or n_show > n_total:
        n_show = n_total

    x_show = x[:n_show]
    y_show = y[:n_show]
    t_show = np.arange(n_show, dtype=np.float64) / fs_hz

    plt.figure(figsize=(8, 4.5))
    plt.plot(t_show, x_show, label="Input x[n]")
    plt.plot(t_show, y_show, label="Output y[n]")
    plt.grid(True)
    plt.xlabel("Time [s]")
    plt.ylabel("Amplitude")
    plt.title(f"Input vs Output at {f_hz:g} Hz")
    plt.legend()

    info = (
        f"Fs = {fs_hz:.3f} Hz\n"
        f"Ain = {amp_in:.3f}\n"
        f"Aout = {amp_out:.3f}\n"
        f"Gain = {gain_db:.3f} dB\n"
        f"Phase(wrapped) = {phase_deg:.3f} deg"
    )
    plt.gca().text(
        0.98, 0.98, info,
        transform=plt.gca().transAxes,
        ha="right", va="top",
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.85)
    )

    plt.tight_layout()
    plt.savefig(path, dpi=160, bbox_inches="tight")
    plt.close()
    return path



def parse_freqs(text: str) -> list[float]:
    return [float(item.strip()) for item in text.split(",") if item.strip()]



def make_linear_freqs(fmin: float, fmax: float, fstep: float) -> list[float]:
    if fmin <= 0.0:
        raise ValueError("--fmin must be > 0")
    if fmax <= 0.0:
        raise ValueError("--fmax must be > 0")
    if fstep <= 0.0:
        raise ValueError("--fstep must be > 0")
    if fmax < fmin:
        raise ValueError("--fmax must be >= --fmin")

    freqs: list[float] = []
    f = fmin
    eps = abs(fstep) * 1e-9
    while f <= fmax + eps:
        freqs.append(round(f, 12))
        f += fstep
    return freqs



def main() -> int:
    parser = argparse.ArgumentParser(
        description="PC-controlled STM32 filter frequency sweep with waveform image export"
    )
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--n", type=int, default=4096, help="Samples captured per frequency")
    parser.add_argument("--settle", type=int, default=1000, help="Settling samples before capture")
    parser.add_argument(
        "--freqs",
        default="2,5,10,15,20,25,30,35,40,45,50,100,200,250",
        help=(
            "Comma-separated frequency list in Hz. "
            "Ignored when --fmin, --fmax, and --fstep are provided."
        ),
    )
    parser.add_argument("--fmin", type=float, default=None, help="Minimum frequency in Hz for linear sweep")
    parser.add_argument("--fmax", type=float, default=None, help="Maximum frequency in Hz for linear sweep")
    parser.add_argument("--fstep", type=float, default=None, help="Frequency step in Hz for linear sweep")
    parser.add_argument("--out", default="sweep_output", help="Output folder for CSV and plots")
    parser.add_argument("--no-show", action="store_true", help="Save plots but do not display Bode windows")
    parser.add_argument("--no-raw", action="store_true", help="Do not save raw sample CSV files")
    parser.add_argument("--no-waveforms", action="store_true", help="Do not save per-frequency waveform PNG images")
    parser.add_argument(
        "--wave-cycles",
        type=float,
        default=4.0,
        help="Number of waveform cycles to show per-frequency PNG image (default: 4)",
    )
    args = parser.parse_args()

    range_args = [args.fmin, args.fmax, args.fstep]
    if any(v is not None for v in range_args):
        if not all(v is not None for v in range_args):
            parser.error("Use --fmin, --fmax, and --fstep together.")
        freqs = make_linear_freqs(args.fmin, args.fmax, args.fstep)
    else:
        freqs = parse_freqs(args.freqs)

    if not freqs:
        parser.error("No frequencies selected.")

    print("Frequencies [Hz]:", ", ".join(f"{f:g}" for f in freqs))

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    waveform_dir = out_dir / "waveforms"

    results: list[SweepResult] = []

    print(f"Opening {args.port} at {args.baud} baud...")
    with serial.Serial(args.port, args.baud, timeout=5) as ser:
        time.sleep(2.0)
        ser.reset_input_buffer()

        for f_cmd in freqs:
            print(f"\n=== Measuring {f_cmd:g} Hz ===")
            f_hz, fs_hz, x, y, skipped = run_one_frequency(
                ser=ser,
                f_hz=f_cmd,
                n_samples=args.n,
                settle_n=args.settle,
            )

            if f_hz <= 0.0:
                print(
                    "WARNING: STM32 reported f_hz <= 0. "
                    "Your firmware may still use sscanf float parsing. "
                    "Fix handle_command() to use String.toFloat()."
                )
                f_hz = f_cmd

            gain, gain_db, phase_deg, amp_in, amp_out = estimate_gain_phase(
                x=x,
                y=y,
                f_hz=f_hz,
                fs_hz=fs_hz,
            )

            result = SweepResult(
                f_hz=f_hz,
                fs_hz=fs_hz,
                n_samples=len(x),
                gain=gain,
                gain_db=gain_db,
                phase_deg=phase_deg,
                phase_unwrapped_deg=phase_deg,
                group_delay_s=float("nan"),
                amp_in=amp_in,
                amp_out=amp_out,
                skipped_lines=skipped,
            )
            results.append(result)

            print(
                f"RESULT: f={result.f_hz:.6g} Hz, "
                f"Fs={result.fs_hz:.3f} Hz, "
                f"N={result.n_samples}, "
                f"Ain={result.amp_in:.6g}, "
                f"Aout={result.amp_out:.6g}, "
                f"gain={result.gain:.6g}, "
                f"gain_db={result.gain_db:.3f} dB, "
                f"phase_wrapped={result.phase_deg:.3f} deg, "
                f"skipped={result.skipped_lines}"
            )

            if not args.no_raw:
                save_raw_block(out_dir, result.f_hz, result.fs_hz, x, y)

            if not args.no_waveforms:
                wave_path = save_waveform_plot(
                    waveform_dir,
                    f_hz=result.f_hz,
                    fs_hz=result.fs_hz,
                    x=x,
                    y=y,
                    gain_db=result.gain_db,
                    phase_deg=result.phase_deg,
                    amp_in=result.amp_in,
                    amp_out=result.amp_out,
                    cycles=args.wave_cycles,
                )
                print(f"Saved waveform image: {wave_path}")

    compute_phase_unwrap_and_group_delay(results)

    print("\n=== Phase and group-delay summary ===")
    for r in sorted(results, key=lambda item: item.f_hz):
        gd_ms = 1000.0 * r.group_delay_s if math.isfinite(r.group_delay_s) else float("nan")
        print(
            f"GD: f={r.f_hz:.6g} Hz, "
            f"phase_wrapped={r.phase_deg:.3f} deg, "
            f"phase_unwrapped={r.phase_unwrapped_deg:.3f} deg, "
            f"group_delay={gd_ms:.3f} ms"
        )

    summary_path = save_summary(out_dir, results)
    print(f"\nSaved summary: {summary_path}")

    plot_results(results, out_dir=out_dir, show=not args.no_show)
    print(f"Saved plots in: {out_dir}")
    if not args.no_waveforms:
        print(f"Saved waveform images in: {waveform_dir}")

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nInterrupted.", file=sys.stderr)
        raise SystemExit(130)
