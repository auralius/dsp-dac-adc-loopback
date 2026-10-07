#!/usr/bin/env python3
"""STM32U585 RLC FRF GUI: stepped sine + PRBS, stackable comparison plots."""

from __future__ import annotations

import csv
import math
import struct
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import numpy as np
import serial
from serial.tools import list_ports
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

BAUD = 115200
ADC_FS = 4095.0
ADC_VREF = 3.3


def read_exact(port: serial.Serial, size: int) -> bytes:
    data = bytearray()
    while len(data) < size:
        part = port.read(size - len(data))
        if not part:
            raise TimeoutError(f"Timed out waiting for {size - len(data)} serial bytes")
        data.extend(part)
    return bytes(data)


def find_tag(port: serial.Serial, valid_tags=(b"FRF1", b"PRB1")) -> bytes:
    window = bytearray()
    while True:
        window.extend(read_exact(port, 1))
        if len(window) > 4:
            del window[:-4]
        b = bytes(window)
        if b in valid_tags:
            return b


def decode_pairs(port: serial.Serial, count: int):
    raw = read_exact(port, count * 4)
    words = np.frombuffer(raw, dtype="<u2").reshape(-1, 2)
    vin = words[:, 0].astype(np.float64) * ADC_VREF / ADC_FS
    vc = words[:, 1].astype(np.float64) * ADC_VREF / ADC_FS
    return vin, vc


def read_record(port: serial.Serial):
    tag = find_tag(port)
    if tag == b"FRF1":
        rate, count, freq_millihz = struct.unpack("<III", read_exact(port, 12))
        if not (1000 <= rate <= 200000 and 16 <= count <= 25000):
            raise ValueError(f"Bad FRF frame: rate={rate}, count={count}")
        vin, vc = decode_pairs(port, count)
        return {
            "kind": "sine",
            "rate": rate,
            "count": count,
            "frequency_hz": freq_millihz / 1000.0,
            "vin": vin,
            "vc": vc,
        }

    rate, count, order, periods, amplitude_mv = struct.unpack(
        "<IIIII", read_exact(port, 20)
    )
    if not (1000 <= rate <= 200000 and 16 <= count <= 25000):
        raise ValueError(f"Bad PRBS frame: rate={rate}, count={count}")
    vin, vc = decode_pairs(port, count)
    return {
        "kind": "prbs",
        "rate": rate,
        "count": count,
        "order": order,
        "periods": periods,
        "amplitude_mv": amplitude_mv,
        "vin": vin,
        "vc": vc,
    }


def single_frequency_frf(vin: np.ndarray, vc: np.ndarray, fs: float, freq: float):
    """Windowed synchronous DFT at the exact generated sine frequency."""
    n = np.arange(vin.size, dtype=np.float64)
    t = n / fs
    x = vin - np.mean(vin)
    y = vc - np.mean(vc)
    w = np.hanning(vin.size)
    kernel = w * np.exp(-2j * np.pi * freq * t)
    X = np.sum(x * kernel)
    Y = np.sum(y * kernel)
    if abs(X) < 1e-12:
        raise ValueError("Input tone is too small for FRF estimation")
    H = Y / X
    mag_db = 20.0 * math.log10(abs(H))
    phase_deg = math.degrees(math.atan2(H.imag, H.real))
    coherent_gain = np.sum(w) / 2.0
    vin_amp = abs(X) / coherent_gain
    vc_amp = abs(Y) / coherent_gain
    return H, mag_db, phase_deg, vin_amp, vc_amp


def prbs_periodic_frf(vin: np.ndarray, vc: np.ndarray, fs: float,
                      order: int, periods: int):
    """FRF from period-aligned PRBS repetitions using averaged spectra.

    Each captured PRBS period is transformed separately. Cross- and auto-spectra
    are averaged across periods, giving H1 = S_yx / S_xx and coherence.
    """
    period_len = (1 << order) - 1
    usable = min(vin.size, vc.size, period_len * periods)
    usable_periods = usable // period_len
    if usable_periods < 1:
        raise ValueError("Not enough samples for one complete PRBS period")

    Sxx = None
    Syy = None
    Syx = None

    for p in range(usable_periods):
        sl = slice(p * period_len, (p + 1) * period_len)
        x = vin[sl] - np.mean(vin[sl])
        y = vc[sl] - np.mean(vc[sl])

        # Exact PRBS-period FFT: no window is used because each block is exactly
        # one repeated deterministic period. Windowing would smear the discrete
        # PRBS spectral lines unnecessarily.
        X = np.fft.rfft(x)
        Y = np.fft.rfft(y)

        pxx = X * np.conj(X)
        pyy = Y * np.conj(Y)
        pyx = Y * np.conj(X)

        if Sxx is None:
            Sxx = pxx
            Syy = pyy
            Syx = pyx
        else:
            Sxx += pxx
            Syy += pyy
            Syx += pyx

    Sxx /= usable_periods
    Syy /= usable_periods
    Syx /= usable_periods

    eps = 1e-30
    H = Syx / np.maximum(Sxx.real, eps)
    coherence = (np.abs(Syx) ** 2) / np.maximum(Sxx.real * Syy.real, eps)
    coherence = np.clip(coherence.real, 0.0, 1.0)

    freqs = np.fft.rfftfreq(period_len, d=1.0 / fs)
    mag_db = 20.0 * np.log10(np.maximum(np.abs(H), 1e-15))
    phase_deg = np.rad2deg(np.unwrap(np.angle(H)))

    # DC is not useful. Keep the same practical band as the stepped-sine GUI.
    keep = (freqs >= 5.0) & (freqs <= 5000.0)
    return freqs[keep], H[keep], mag_db[keep], phase_deg[keep], coherence[keep]


class FrfApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        root.title("STM32U585 RLC Frequency Response")
        root.geometry("1120x800")

        self.runs = []
        self.current_rows = []
        self.current_method = None
        self.stop_requested = False
        self.running = False

        top = ttk.Frame(root, padding=8)
        top.pack(fill="x")

        ttk.Label(top, text="Serial port:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(top, textvariable=self.port_var,
                                     width=20, state="readonly")
        self.port_box.pack(side="left", padx=(5, 5))
        ttk.Button(top, text="Refresh", command=self.refresh_ports).pack(side="left")

        ttk.Label(top, text="Method:").pack(side="left", padx=(16, 4))
        self.method_var = tk.StringVar(value="Stepped Sine")
        self.method_box = ttk.Combobox(
            top, textvariable=self.method_var,
            values=["Stepped Sine", "PRBS"], width=15, state="readonly"
        )
        self.method_box.pack(side="left")
        self.method_box.bind("<<ComboboxSelected>>", lambda _e: self.update_method_controls())

        self.run_button = ttk.Button(top, text="Run", command=self.start_run)
        self.run_button.pack(side="left", padx=(12, 4))
        self.stop_button = ttk.Button(top, text="Stop", command=self.request_stop, state="disabled")
        self.stop_button.pack(side="left", padx=4)
        self.save_button = ttk.Button(top, text="Save all CSV", command=self.save_csv, state="disabled")
        self.save_button.pack(side="left", padx=4)
        self.clear_button = ttk.Button(top, text="Clear plots", command=self.clear_plots)
        self.clear_button.pack(side="left", padx=4)

        self.sine_frame = ttk.Frame(root, padding=(8, 0, 8, 4))
        self.sine_frame.pack(fill="x")
        ttk.Label(self.sine_frame, text="Sine sweep — Start Hz:").pack(side="left")
        self.start_var = tk.StringVar(value="30")
        ttk.Entry(self.sine_frame, textvariable=self.start_var, width=7).pack(side="left", padx=(3, 8))
        ttk.Label(self.sine_frame, text="Stop Hz:").pack(side="left")
        self.stop_var = tk.StringVar(value="3000")
        ttk.Entry(self.sine_frame, textvariable=self.stop_var, width=7).pack(side="left", padx=(3, 8))
        ttk.Label(self.sine_frame, text="Points:").pack(side="left")
        self.points_var = tk.StringVar(value="61")
        ttk.Entry(self.sine_frame, textvariable=self.points_var, width=5).pack(side="left", padx=(3, 8))
        self.log_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(self.sine_frame, text="Log spacing", variable=self.log_var).pack(side="left", padx=4)
        ttk.Button(self.sine_frame, text="Coarse 30–3000 Hz", command=self.preset_coarse).pack(side="left", padx=(10, 4))
        ttk.Button(self.sine_frame, text="Fine 250–800 Hz", command=self.preset_fine).pack(side="left", padx=4)

        self.prbs_frame = ttk.Frame(root, padding=(8, 0, 8, 4))
        ttk.Label(self.prbs_frame, text="PRBS — Order:").pack(side="left")
        self.prbs_order_var = tk.StringVar(value="13")
        ttk.Entry(self.prbs_frame, textvariable=self.prbs_order_var, width=5, state="readonly").pack(side="left", padx=(3, 8))
        ttk.Label(self.prbs_frame, text="Periods:").pack(side="left")
        self.prbs_periods_var = tk.StringVar(value="3")
        ttk.Combobox(self.prbs_frame, textvariable=self.prbs_periods_var,
                     values=["1", "2", "3"], width=4, state="readonly").pack(side="left", padx=(3, 8))
        ttk.Label(self.prbs_frame, text="Amplitude ±mV:").pack(side="left")
        self.prbs_amp_var = tk.StringVar(value="600")
        ttk.Entry(self.prbs_frame, textvariable=self.prbs_amp_var, width=6).pack(side="left", padx=(3, 8))
        ttk.Label(self.prbs_frame, text="(PRBS13 = 8191 samples/period; one settling period is automatic)").pack(side="left", padx=8)

        self.status = tk.StringVar(value="Ready. Curves from each completed run are retained for comparison.")
        ttk.Label(root, textvariable=self.status, padding=(10, 2)).pack(anchor="w")
        self.progress = ttk.Progressbar(root, mode="determinate")
        self.progress.pack(fill="x", padx=10, pady=(0, 4))

        self.figure = Figure(figsize=(11.2, 6.8), dpi=100)
        gs = self.figure.add_gridspec(
            2, 2,
            width_ratios=(1.45, 1.0),
            height_ratios=(1.0, 1.0),
            wspace=0.30,
            hspace=0.16,
        )
        self.ax_mag = self.figure.add_subplot(gs[0, 0])
        self.ax_phase = self.figure.add_subplot(gs[1, 0], sharex=self.ax_mag)
        self.ax_nyquist = self.figure.add_subplot(gs[:, 1])

        plot_frame = ttk.Frame(root)
        plot_frame.pack(fill="both", expand=True, padx=10, pady=5)
        self.canvas = FigureCanvasTkAgg(self.figure, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self.toolbar = NavigationToolbar2Tk(self.canvas, plot_frame, pack_toolbar=False)
        self.toolbar.update()
        self.toolbar.pack(fill="x")

        self.refresh_ports()
        self.update_method_controls()
        self.redraw()

    def refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.port_box["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        if not ports:
            self.port_var.set("")

    def update_method_controls(self):
        if self.method_var.get() == "PRBS":
            self.sine_frame.pack_forget()
            self.prbs_frame.pack(fill="x", after=self.root.winfo_children()[0])
        else:
            self.prbs_frame.pack_forget()
            self.sine_frame.pack(fill="x", after=self.root.winfo_children()[0])

    def preset_coarse(self):
        self.start_var.set("30")
        self.stop_var.set("3000")
        self.points_var.set("61")
        self.log_var.set(True)

    def preset_fine(self):
        self.start_var.set("250")
        self.stop_var.set("800")
        self.points_var.set("111")
        self.log_var.set(False)

    def request_stop(self):
        self.stop_requested = True
        self.status.set("Stopping after the current operation…")

    @staticmethod
    def timing_for_frequency(freq: float):
        settle_ms = max(40, int(math.ceil(6.0 / freq * 1000.0)))
        capture_ms = max(80, int(math.ceil(8.0 / freq * 1000.0)))
        return min(settle_ms, 1000), min(capture_ms, 500)

    def start_run(self):
        if self.running:
            return
        port_name = self.port_var.get()
        if not port_name:
            messagebox.showerror("No port", "Select the STM32U585 serial port first.")
            return

        method = self.method_var.get()
        try:
            if method == "Stepped Sine":
                f0 = float(self.start_var.get())
                f1 = float(self.stop_var.get())
                points = int(self.points_var.get())
                if not (5 <= f0 < f1 <= 5000 and 2 <= points <= 300):
                    raise ValueError("Use 5–5000 Hz and 2–300 points.")
                freqs = np.geomspace(f0, f1, points) if self.log_var.get() else np.linspace(f0, f1, points)
                args = (port_name, freqs)
                target = self.sine_worker
                self.progress["maximum"] = len(freqs)
            else:
                order = int(self.prbs_order_var.get())
                periods = int(self.prbs_periods_var.get())
                amp_mv = int(self.prbs_amp_var.get())
                if order != 13 or periods not in (1, 2, 3) or not (50 <= amp_mv <= 1400):
                    raise ValueError("PRBS currently uses order 13, 1–3 periods, and 50–1400 mV amplitude.")
                args = (port_name, order, periods, amp_mv)
                target = self.prbs_worker
                self.progress["maximum"] = 1
        except ValueError as exc:
            messagebox.showerror("Invalid settings", str(exc))
            return

        self.current_rows = []
        self.current_method = method
        self.stop_requested = False
        self.running = True
        self.progress["value"] = 0
        self.run_button.configure(state="disabled")
        self.stop_button.configure(state="normal")
        threading.Thread(target=target, args=args, daemon=True).start()

    def sine_worker(self, port_name: str, freqs: np.ndarray):
        try:
            with serial.Serial(port_name, BAUD, timeout=12) as port:
                port.reset_input_buffer()
                for idx, requested_freq in enumerate(freqs):
                    if self.stop_requested:
                        break
                    settle_ms, capture_ms = self.timing_for_frequency(float(requested_freq))
                    freq_millihz = int(round(float(requested_freq) * 1000.0))
                    port.write(f"SINE {freq_millihz} {settle_ms} {capture_ms}\n".encode("ascii"))
                    port.flush()
                    rec = read_record(port)
                    if rec["kind"] != "sine":
                        raise ValueError("Expected sine frame from MCU")
                    H, mag_db, phase_deg, vin_amp, vc_amp = single_frequency_frf(
                        rec["vin"], rec["vc"], rec["rate"], rec["frequency_hz"]
                    )
                    self.current_rows.append({
                        "frequency_hz": rec["frequency_hz"],
                        "magnitude_db": mag_db,
                        "phase_deg": phase_deg,
                        "h_real": float(H.real),
                        "h_imag": float(H.imag),
                        "coherence": float("nan"),
                        "sample_rate_hz": rec["rate"],
                        "vin_amp_v": vin_amp,
                        "vc_amp_v": vc_amp,
                    })
                    self.root.after(0, self.update_current_sine, idx + 1, len(freqs))
            self.root.after(0, self.finish_sine_run)
        except Exception as exc:
            self.root.after(0, self.run_failed, str(exc))

    def update_current_sine(self, done: int, total: int):
        self.progress["value"] = done
        if self.current_rows:
            r = self.current_rows[-1]
            self.status.set(
                f"Stepped sine {done}/{total}: {r['frequency_hz']:.1f} Hz, "
                f"{r['magnitude_db']:.2f} dB, phase {r['phase_deg']:.1f}°"
            )
        self.redraw(include_current=True)

    def finish_sine_run(self):
        if self.current_rows:
            freqs = np.array([r["frequency_hz"] for r in self.current_rows])
            mags = np.array([r["magnitude_db"] for r in self.current_rows])
            phases = np.rad2deg(np.unwrap(np.deg2rad([r["phase_deg"] for r in self.current_rows])))
            for r, p in zip(self.current_rows, phases):
                r["phase_deg"] = float(p)
            k = int(np.argmax(mags))
            label = f"Stepped Sine #{len(self.runs) + 1}"
            self.runs.append({"method": "Stepped Sine", "label": label, "rows": self.current_rows.copy()})
            self.status.set(
                f"{label} complete: peak {mags[k]:.2f} dB at {freqs[k]:.2f} Hz. "
                "Previous curves retained."
            )
        else:
            self.status.set("Stepped-sine run stopped before any point was captured.")
        self.current_rows = []
        self.finish_common()

    def prbs_worker(self, port_name: str, order: int, periods: int, amp_mv: int):
        try:
            # 1 settling period + up to 3 captured periods is under 0.7 s, but
            # binary USB transfer can add time; use a generous timeout.
            with serial.Serial(port_name, BAUD, timeout=8) as port:
                port.reset_input_buffer()
                port.write(f"PRBS {order} {periods} {amp_mv}\n".encode("ascii"))
                port.flush()
                rec = read_record(port)
                if rec["kind"] != "prbs":
                    raise ValueError("Expected PRBS frame from MCU")
                freqs, H, mags, phases, coh = prbs_periodic_frf(
                    rec["vin"], rec["vc"], rec["rate"], rec["order"], rec["periods"]
                )
                self.current_rows = [
                    {
                        "frequency_hz": float(f),
                        "magnitude_db": float(m),
                        "phase_deg": float(p),
                        "h_real": float(h.real),
                        "h_imag": float(h.imag),
                        "coherence": float(c),
                        "sample_rate_hz": rec["rate"],
                        "vin_amp_v": float("nan"),
                        "vc_amp_v": float("nan"),
                    }
                    for f, h, m, p, c in zip(freqs, H, mags, phases, coh)
                ]
                self.root.after(0, self.finish_prbs_run, order, periods, amp_mv)
        except Exception as exc:
            self.root.after(0, self.run_failed, str(exc))

    def finish_prbs_run(self, order: int, periods: int, amp_mv: int):
        self.progress["value"] = 1
        if self.current_rows:
            freqs = np.array([r["frequency_hz"] for r in self.current_rows])
            mags = np.array([r["magnitude_db"] for r in self.current_rows])
            coh = np.array([r["coherence"] for r in self.current_rows])

            # Report the strongest peak only where coherence is reasonably high.
            valid = np.isfinite(mags) & (coh >= 0.8) & (freqs <= 3000.0)
            if np.any(valid):
                idxs = np.flatnonzero(valid)
                k = idxs[int(np.argmax(mags[valid]))]
                peak_text = f"peak {mags[k]:.2f} dB at {freqs[k]:.2f} Hz"
            else:
                peak_text = "no high-coherence peak identified"

            label = f"PRBS13 #{len(self.runs) + 1}"
            self.runs.append({
                "method": "PRBS",
                "label": label,
                "rows": self.current_rows.copy(),
                "meta": {"order": order, "periods": periods, "amplitude_mv": amp_mv},
            })
            self.status.set(
                f"{label} complete ({periods} periods, ±{amp_mv} mV): {peak_text}. "
                "Previous curves retained."
            )
        self.current_rows = []
        self.finish_common()

    def finish_common(self):
        self.running = False
        self.run_button.configure(state="normal")
        self.stop_button.configure(state="disabled")
        self.save_button.configure(state="normal" if self.runs else "disabled")
        self.redraw()

    def run_failed(self, msg: str):
        self.running = False
        self.run_button.configure(state="normal")
        self.stop_button.configure(state="disabled")
        self.status.set("Run failed.")
        messagebox.showerror("Run failed", msg)

    def clear_plots(self):
        if self.running:
            messagebox.showinfo("Busy", "Stop the current run before clearing plots.")
            return
        self.runs.clear()
        self.current_rows = []
        self.save_button.configure(state="disabled")
        self.status.set("Plots cleared.")
        self.redraw()

    def redraw(self, include_current=False):
        self.ax_mag.clear()
        self.ax_phase.clear()
        self.ax_nyquist.clear()

        all_runs = list(self.runs)
        if include_current and self.current_rows:
            all_runs.append({
                "method": self.current_method or "Current",
                "label": f"{self.current_method or 'Current'} (running)",
                "rows": self.current_rows,
            })

        for run in all_runs:
            rows = run["rows"]
            if not rows:
                continue

            freqs = np.array([r["frequency_hz"] for r in rows], dtype=float)
            mags = np.array([r["magnitude_db"] for r in rows], dtype=float)
            phases = np.array([r["phase_deg"] for r in rows], dtype=float)

            h_real = np.array([
                r.get(
                    "h_real",
                    10.0 ** (r["magnitude_db"] / 20.0)
                    * math.cos(math.radians(r["phase_deg"]))
                )
                for r in rows
            ], dtype=float)

            h_imag = np.array([
                r.get(
                    "h_imag",
                    10.0 ** (r["magnitude_db"] / 20.0)
                    * math.sin(math.radians(r["phase_deg"]))
                )
                for r in rows
            ], dtype=float)

            order = np.argsort(freqs)
            freqs = freqs[order]
            mags = mags[order]
            phases = phases[order]
            h_real = h_real[order]
            h_imag = h_imag[order]

            if run.get("method") == "Stepped Sine":
                line, = self.ax_mag.semilogx(
                    freqs, mags, marker=".", linewidth=1.2, label=run["label"]
                )
                run_color = line.get_color()
                self.ax_phase.semilogx(
                    freqs, phases, marker=".", linewidth=1.2,
                    color=run_color, label=run["label"]
                )
                self.ax_nyquist.plot(
                    h_real, h_imag, marker=".", linewidth=1.2,
                    color=run_color, label=run["label"]
                )
            else:
                line, = self.ax_mag.semilogx(
                    freqs, mags, linewidth=1.1, label=run["label"]
                )
                run_color = line.get_color()
                self.ax_phase.semilogx(
                    freqs, phases, linewidth=1.1,
                    color=run_color, label=run["label"]
                )
                self.ax_nyquist.plot(
                    h_real, h_imag, linewidth=1.1,
                    color=run_color, label=run["label"]
                )

        self.ax_mag.set_ylabel("Magnitude (dB)")
        self.ax_phase.set_ylabel("Phase (deg)")
        self.ax_phase.set_xlabel("Frequency (Hz)")
        self.ax_mag.grid(True, which="both", alpha=0.3)
        self.ax_phase.grid(True, which="both", alpha=0.3)
        self.ax_mag.set_title("RLC frequency response — stacked runs")

        self.ax_nyquist.set_xlabel("Re{H}")
        self.ax_nyquist.set_ylabel("Im{H}")
        self.ax_nyquist.set_title("Nyquist plot")
        self.ax_nyquist.grid(True, alpha=0.3)
        self.ax_nyquist.axhline(0.0, linewidth=0.8, alpha=0.5)
        self.ax_nyquist.axvline(0.0, linewidth=0.8, alpha=0.5)
        self.ax_nyquist.set_aspect("equal", adjustable="datalim")

        if all_runs:
            self.ax_mag.legend(loc="best")
            self.ax_phase.legend(loc="best")
            self.ax_nyquist.legend(loc="best")

        self.canvas.draw_idle()

    def save_csv(self):
        if not self.runs:
            return
        path = filedialog.asksaveasfilename(
            title="Save all frequency-response runs",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*")],
            initialfile="stm32u585_rlc_frf_comparison.csv",
        )
        if not path:
            return

        fields = [
            "run", "method", "label", "frequency_hz", "magnitude_db",
            "phase_deg", "h_real", "h_imag", "coherence",
            "sample_rate_hz", "vin_amp_v", "vc_amp_v"
        ]
        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            for run_index, run in enumerate(self.runs, start=1):
                for row in run["rows"]:
                    writer.writerow({
                        "run": run_index,
                        "method": run["method"],
                        "label": run["label"],
                        **{k: row.get(k, "") for k in fields if k not in ("run", "method", "label")},
                    })
        self.status.set(f"Saved {len(self.runs)} stacked run(s): {path}")


def main():
    root = tk.Tk()
    FrfApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
