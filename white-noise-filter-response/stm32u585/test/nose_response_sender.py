#!/usr/bin/env python3
"""
PC-side white-noise frequency-response estimator for STM32 DAC-ADC loopback.

Expected STM32 command:
    NOISE <N> <settle_N> [seed]

Expected STM32 response:
    OK,NOISE,<N>,<settle_N>,<seed>
    BEGIN,NOISE,0.000000,<actual_fs_hz>,<N>,<missed_deadlines>
    DATA,<n>,<x>,<y>
    DATA,<n>,<x>,<y>
    ...
    END

The input x[n] is the measured ADC loopback input.
The output y[n] is the embedded filter output.

This script estimates the frequency response using the H1 estimator:

    H1(f) = Sxy(f) / Sxx(f)

where:
    Sxy(f) = average(conj(X(f)) * Y(f))
    Sxx(f) = average(conj(X(f)) * X(f))

This is better than simple Y/X for white-noise excitation because it supports
block averaging and gives a smoother response.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from pathlib import Path

import numpy as np
import serial
import matplotlib.pyplot as plt


# ============================================================
# Serial receive helpers
# ============================================================

def wait_for_begin(ser: serial.Serial, timeout_s: float = 180.0) -> tuple[str, float, float, int, int]:
    """Wait for BEGIN line.

    Supports the new format:
        BEGIN,NOISE,0.000000,actual_fs,N,missed

    Also tolerates old format:
        BEGIN,f,actual_fs,N,missed
    """
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

        if not line.startswith("BEGIN"):
            continue

        parts = line.split(",")

        # New format:
        # BEGIN,NOISE,effective_f,actual_fs,N,missed
        if len(parts) >= 6 and parts[1].upper() in ("NOISE", "SINE"):
            mode = parts[1].upper()
            effective_f_hz = float(parts[2])
            fs_hz = float(parts[3])
            n_samples = int(parts[4])
            missed = int(parts[5])
            return mode, effective_f_hz, fs_hz, n_samples, missed

        # Old format:
        # BEGIN,effective_f,actual_fs,N[,missed]
        if len(parts) >= 4:
            mode = "UNKNOWN"
            effective_f_hz = float(parts[1])
            fs_hz = float(parts[2])
            n_samples = int(parts[3])
            missed = int(parts[4]) if len(parts) >= 5 else 0
            return mode, effective_f_hz, fs_hz, n_samples, missed

        raise ValueError(f"Malformed BEGIN line: {line}")


def parse_sample_line(line: str) -> tuple[float, float] | None:
    """Parse DATA,n,x,y or old n,x,y format."""
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


def read_block(ser: serial.Serial, expected_n: int, timeout_s: float = 300.0) -> tuple[np.ndarray, np.ndarray, int]:
    """Read DATA lines until END."""
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


def run_noise_capture(
    ser: serial.Serial,
    n_samples: int,
    settle_n: int,
    seed: int,
) -> tuple[str, float, int, int, np.ndarray, np.ndarray, int]:
    """Send NOISE command and return mode, fs, N, missed, x, y, skipped."""
    ser.reset_input_buffer()
    time.sleep(0.05)

    cmd = f"NOISE {n_samples:d} {settle_n:d} {seed:d}\n"
    print(f"--> {cmd.strip()}")
    ser.write(cmd.encode("ascii"))
    ser.flush()

    mode, _effective_f, fs_hz, n_reported, missed = wait_for_begin(ser)
    x, y, skipped = read_block(ser, n_reported)

    return mode, fs_hz, n_reported, missed, x, y, skipped


# ============================================================
# Saving raw data
# ============================================================

def save_raw_block(out_dir: Path, fs_hz: float, x: np.ndarray, y: np.ndarray) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / "noise_raw.csv"

    with path.open("w", newline="") as fp:
        writer = csv.writer(fp)
        writer.writerow(["n", "t_s", "x", "y"])
        for n, (xi, yi) in enumerate(zip(x, y)):
            writer.writerow([n, n / fs_hz, xi, yi])

    return path


# ============================================================
# Frequency response estimation
# ============================================================

def choose_block_size(n_total: int, requested: int) -> int:
    """Choose a valid block size <= n_total."""
    if requested <= 0:
        raise ValueError("block size must be positive")
    if requested > n_total:
        # Use largest power of two not exceeding n_total.
        return 1 << int(np.floor(np.log2(n_total)))
    return requested


def estimate_h1_white_noise(
    x: np.ndarray,
    y: np.ndarray,
    fs_hz: float,
    block_size: int = 4096,
    overlap: float = 0.5,
    fmin: float | None = None,
    fmax: float | None = None,
) -> dict[str, np.ndarray | float | int]:
    """Estimate H1(f)=Sxy/Sxx using Welch-style block averaging."""
    if len(x) != len(y):
        raise ValueError("x and y must have the same length")

    n_total = len(x)
    block_size = choose_block_size(n_total, block_size)

    if not (0.0 <= overlap < 1.0):
        raise ValueError("overlap must be in [0, 1)")

    hop = int(round(block_size * (1.0 - overlap)))
    if hop <= 0:
        hop = 1

    window = np.hanning(block_size)
    win_power = np.sum(window * window)

    sxy = None
    sxx = None
    syy = None
    n_blocks = 0

    for start in range(0, n_total - block_size + 1, hop):
        xb = x[start:start + block_size].astype(np.float64)
        yb = y[start:start + block_size].astype(np.float64)

        # Remove block-local DC.
        xb = xb - np.mean(xb)
        yb = yb - np.mean(yb)

        X = np.fft.rfft(xb * window)
        Y = np.fft.rfft(yb * window)

        if sxy is None:
            sxy = np.zeros_like(X, dtype=np.complex128)
            sxx = np.zeros_like(X, dtype=np.float64)
            syy = np.zeros_like(X, dtype=np.float64)

        sxy += np.conj(X) * Y
        sxx += np.real(np.conj(X) * X)
        syy += np.real(np.conj(Y) * Y)
        n_blocks += 1

    if n_blocks == 0:
        raise RuntimeError("No FFT blocks were produced. Increase N or reduce --block-size.")

    sxy = sxy / n_blocks
    sxx = sxx / n_blocks
    syy = syy / n_blocks

    # Protect against division by zero.
    eps = 1e-30
    H = sxy / np.maximum(sxx, eps)

    # Coherence is useful for checking quality of the estimate.
    coherence = (np.abs(sxy) ** 2) / np.maximum(sxx * syy, eps)

    freqs = np.fft.rfftfreq(block_size, d=1.0 / fs_hz)

    mag = np.abs(H)
    mag_db = 20.0 * np.log10(np.maximum(mag, eps))
    phase_wrapped_rad = np.angle(H)
    phase_wrapped_deg = np.rad2deg(phase_wrapped_rad)
    phase_unwrapped_rad = np.unwrap(phase_wrapped_rad)
    phase_unwrapped_deg = np.rad2deg(phase_unwrapped_rad)

    omega = 2.0 * np.pi * freqs
    if len(freqs) >= 2:
        group_delay_s = -np.gradient(phase_unwrapped_rad, omega)
    else:
        group_delay_s = np.full_like(freqs, np.nan, dtype=np.float64)

    mask = np.ones_like(freqs, dtype=bool)
    if fmin is not None:
        mask &= freqs >= fmin
    if fmax is not None:
        mask &= freqs <= fmax

    # Remove DC by default from plotted/saved response unless user explicitly sets fmin=0.
    if fmin is None:
        mask &= freqs > 0.0

    return {
        "freqs": freqs[mask],
        "H": H[mask],
        "mag_db": mag_db[mask],
        "phase_wrapped_deg": phase_wrapped_deg[mask],
        "phase_unwrapped_deg": phase_unwrapped_deg[mask],
        "group_delay_s": group_delay_s[mask],
        "coherence": coherence[mask],
        "n_blocks": n_blocks,
        "block_size": block_size,
        "hop": hop,
        "df_hz": fs_hz / block_size,
        "win_power": win_power,
    }


def save_response_csv(out_dir: Path, response: dict[str, np.ndarray | float | int]) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / "noise_response_h1.csv"

    freqs = response["freqs"]
    mag_db = response["mag_db"]
    phase_wrapped_deg = response["phase_wrapped_deg"]
    phase_unwrapped_deg = response["phase_unwrapped_deg"]
    group_delay_s = response["group_delay_s"]
    coherence = response["coherence"]

    with path.open("w", newline="") as fp:
        writer = csv.writer(fp)
        writer.writerow([
            "f_hz",
            "gain_db",
            "phase_deg_wrapped",
            "phase_deg_unwrapped",
            "group_delay_s",
            "group_delay_ms",
            "coherence",
        ])

        for f, m, pw, pu, gd, coh in zip(
            freqs, mag_db, phase_wrapped_deg, phase_unwrapped_deg, group_delay_s, coherence
        ):
            writer.writerow([
                f,
                m,
                pw,
                pu,
                gd,
                1000.0 * gd if math.isfinite(gd) else float("nan"),
                coh,
            ])

    return path


# ============================================================
# Plotting
# ============================================================

def save_time_plot(out_dir: Path, fs_hz: float, x: np.ndarray, y: np.ndarray, seconds: float = 2.0) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / "noise_time_input_output.png"

    n_show = int(round(seconds * fs_hz))
    n_show = max(1, min(n_show, len(x)))

    t = np.arange(n_show, dtype=np.float64) / fs_hz

    plt.figure(figsize=(9, 4.5))
    plt.plot(t, x[:n_show], label="Input x[n]")
    plt.plot(t, y[:n_show], label="Output y[n]")
    plt.grid(True)
    plt.xlabel("Time [s]")
    plt.ylabel("Amplitude")
    plt.title("White-Noise Input vs Filter Output")
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=160, bbox_inches="tight")
    plt.close()

    return path


def plot_response(
    out_dir: Path,
    response: dict[str, np.ndarray | float | int],
    show: bool = True,
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)

    f = response["freqs"]
    mag_db = response["mag_db"]
    phase_wrapped_deg = response["phase_wrapped_deg"]
    phase_unwrapped_deg = response["phase_unwrapped_deg"]
    group_delay_ms = 1000.0 * response["group_delay_s"]
    coherence = response["coherence"]

    plt.figure()
    plt.semilogx(f, mag_db)
    plt.grid(True, which="both")
    plt.xlabel("Frequency [Hz]")
    plt.ylabel("Gain [dB]")
    plt.title("White-Noise H1 Magnitude Estimate")
    plt.savefig(out_dir / "noise_magnitude_h1.png", dpi=160, bbox_inches="tight")

    plt.figure()
    plt.semilogx(f, phase_wrapped_deg)
    plt.grid(True, which="both")
    plt.xlabel("Frequency [Hz]")
    plt.ylabel("Wrapped Phase [deg]")
    plt.title("White-Noise H1 Phase Estimate (Wrapped)")
    plt.savefig(out_dir / "noise_phase_wrapped_h1.png", dpi=160, bbox_inches="tight")

    plt.figure()
    plt.semilogx(f, phase_unwrapped_deg)
    plt.grid(True, which="both")
    plt.xlabel("Frequency [Hz]")
    plt.ylabel("Unwrapped Phase [deg]")
    plt.title("White-Noise H1 Phase Estimate (Unwrapped)")
    plt.savefig(out_dir / "noise_phase_unwrapped_h1.png", dpi=160, bbox_inches="tight")

    plt.figure()
    plt.semilogx(f, group_delay_ms)
    plt.grid(True, which="both")
    plt.xlabel("Frequency [Hz]")
    plt.ylabel("Group Delay [ms]")
    plt.title("White-Noise H1 Group Delay Estimate")
    plt.savefig(out_dir / "noise_group_delay_h1.png", dpi=160, bbox_inches="tight")

    plt.figure()
    plt.semilogx(f, coherence)
    plt.grid(True, which="both")
    plt.xlabel("Frequency [Hz]")
    plt.ylabel("Coherence")
    plt.ylim([-0.05, 1.05])
    plt.title("Input-Output Coherence")
    plt.savefig(out_dir / "noise_coherence.png", dpi=160, bbox_inches="tight")

    if show:
        plt.show()
    else:
        plt.close("all")


# ============================================================
# Main
# ============================================================

def main() -> int:
    parser = argparse.ArgumentParser(
        description="White-noise frequency-response estimation for STM32 DAC-ADC loopback"
    )

    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--n", type=int, default=65536, help="Captured samples")
    parser.add_argument("--settle", type=int, default=5000, help="Settling samples before capture")
    parser.add_argument("--seed", type=int, default=12345, help="Noise RNG seed sent to STM32")
    parser.add_argument("--out", default="noise_output", help="Output folder")
    parser.add_argument("--block-size", type=int, default=4096, help="FFT block size")
    parser.add_argument("--overlap", type=float, default=0.5, help="Welch overlap fraction, e.g. 0.5")
    parser.add_argument("--fmin", type=float, default=0.1, help="Minimum plotted/saved frequency")
    parser.add_argument("--fmax", type=float, default=300.0, help="Maximum plotted/saved frequency")
    parser.add_argument("--time-seconds", type=float, default=2.0, help="Seconds shown in time-domain plot")
    parser.add_argument("--no-show", action="store_true", help="Save plots but do not display windows")
    parser.add_argument("--no-raw", action="store_true", help="Do not save raw x/y CSV")

    args = parser.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"Opening {args.port} at {args.baud} baud...")
    with serial.Serial(args.port, args.baud, timeout=5) as ser:
        # Many Arduino-compatible STM32 boards reset when serial opens.
        time.sleep(2.0)
        ser.reset_input_buffer()

        mode, fs_hz, n_reported, missed, x, y, skipped = run_noise_capture(
            ser=ser,
            n_samples=args.n,
            settle_n=args.settle,
            seed=args.seed,
        )

    print()
    print("=== Capture summary ===")
    print(f"Mode              : {mode}")
    print(f"Fs reported        : {fs_hz:.6f} Hz")
    print(f"N reported         : {n_reported}")
    print(f"Received samples   : {len(x)}")
    print(f"Missed deadlines   : {missed}")
    print(f"Skipped text lines  : {skipped}")
    print(f"Input RMS          : {np.sqrt(np.mean((x - np.mean(x))**2)):.6g}")
    print(f"Output RMS         : {np.sqrt(np.mean((y - np.mean(y))**2)):.6g}")

    if not args.no_raw:
        raw_path = save_raw_block(out_dir, fs_hz, x, y)
        print(f"Saved raw data     : {raw_path}")

    time_path = save_time_plot(out_dir, fs_hz, x, y, seconds=args.time_seconds)
    print(f"Saved time plot    : {time_path}")

    response = estimate_h1_white_noise(
        x=x,
        y=y,
        fs_hz=fs_hz,
        block_size=args.block_size,
        overlap=args.overlap,
        fmin=args.fmin,
        fmax=args.fmax,
    )

    print()
    print("=== H1 estimation summary ===")
    print(f"FFT block size     : {response['block_size']}")
    print(f"FFT df             : {response['df_hz']:.6f} Hz")
    print(f"Hop size           : {response['hop']}")
    print(f"Averaged blocks    : {response['n_blocks']}")

    csv_path = save_response_csv(out_dir, response)
    print(f"Saved H1 CSV       : {csv_path}")

    plot_response(out_dir, response, show=not args.no_show)
    print(f"Saved plots in     : {out_dir}")

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nInterrupted.", file=sys.stderr)
        raise SystemExit(130)
