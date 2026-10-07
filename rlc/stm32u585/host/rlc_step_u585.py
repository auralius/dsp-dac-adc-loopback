#!/usr/bin/env python3
"""GUI for STM32U585 two-channel RLC step-response capture (RLC2 protocol)."""

from __future__ import annotations

import csv
import struct
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import serial
from serial.tools import list_ports
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

BAUD = 115200
ADC_FULL_SCALE = 4095.0
ADC_REFERENCE_V = 3.3
DEFAULT_VIEW_MS = 20.0


def read_exact(port: serial.Serial, size: int) -> bytes:
    data = bytearray()
    while len(data) < size:
        part = port.read(size - len(data))
        if not part:
            raise TimeoutError(f"Timed out waiting for {size - len(data)} serial bytes")
        data.extend(part)
    return bytes(data)


def read_record(port: serial.Serial) -> tuple[int, list[int], list[int]]:
    """Find one RLC2 frame and return rate, Vin ADC codes, and Vc ADC codes."""
    window = bytearray()
    while True:
        window.extend(read_exact(port, 1))
        if len(window) > 4:
            del window[:-4]
        if window == b"RLC2":
            break

    sample_rate, count = struct.unpack("<II", read_exact(port, 8))
    if not (100 <= sample_rate <= 1_000_000 and 1 <= count <= 500_000):
        raise ValueError(f"Unexpected record dimensions: {sample_rate} Hz, {count} samples")

    raw = read_exact(port, count * 4)
    words = struct.unpack("<" + "H" * (count * 2), raw)

    vin = list(words[0::2])
    vc = list(words[1::2])

    if any(v > 4095 for v in vin) or any(v > 4095 for v in vc):
        raise ValueError("ADC record contains a value outside the 12-bit range")

    return sample_rate, vin, vc


class CaptureApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        root.title("STM32U585 RLC Step Response")
        root.geometry("980x650")

        self.last_rate: int | None = None
        self.last_vin: list[int] | None = None
        self.last_vc: list[int] | None = None

        controls = ttk.Frame(root, padding=10)
        controls.pack(fill="x")

        ttk.Label(controls, text="Serial port:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(
            controls, textvariable=self.port_var, width=28, state="readonly"
        )
        self.port_box.pack(side="left", padx=(6, 8))

        ttk.Button(controls, text="Refresh", command=self.refresh_ports).pack(side="left")

        self.capture_button = ttk.Button(
            controls, text="Capture 100 ms (target 50 kS/s)", command=self.start_capture
        )
        self.capture_button.pack(side="left", padx=(12, 6))

        self.save_button = ttk.Button(
            controls, text="Save CSV", command=self.save_csv, state="disabled"
        )
        self.save_button.pack(side="left", padx=6)

        self.status = tk.StringVar(value="Connect the STM32U585, then capture.")
        ttk.Label(root, textvariable=self.status, padding=(12, 0)).pack(anchor="w")

        figure = Figure(figsize=(9.2, 5.1), dpi=100)
        self.axes = figure.add_subplot(111)
        self.axes.set_title("RLC step response")
        self.axes.set_xlabel("Time from step (ms)")
        self.axes.set_ylabel("Voltage (V)")
        self.axes.grid(True, alpha=0.3)

        plot_frame = ttk.Frame(root)
        plot_frame.pack(fill="both", expand=True, padx=10, pady=10)

        self.canvas = FigureCanvasTkAgg(figure, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        # Matplotlib navigation toolbar: Home, Back/Forward, Pan and rectangle Zoom.
        self.toolbar = NavigationToolbar2Tk(self.canvas, plot_frame, pack_toolbar=False)
        self.toolbar.update()
        self.toolbar.pack(fill="x")

        # Mouse-wheel zoom centred on the cursor, useful for quickly inspecting
        # the first few hundred microseconds of the RLC transient.
        self.canvas.mpl_connect("scroll_event", self.on_scroll_zoom)

        self.refresh_ports()
        root.protocol("WM_DELETE_WINDOW", root.destroy)


    def on_scroll_zoom(self, event) -> None:
        """Zoom both axes around the mouse pointer with the scroll wheel."""
        if event.inaxes is not self.axes or event.xdata is None or event.ydata is None:
            return

        # Scroll up = zoom in, scroll down = zoom out.
        scale = 0.80 if event.button == "up" else 1.25

        x0, x1 = self.axes.get_xlim()
        y0, y1 = self.axes.get_ylim()
        x = event.xdata
        y = event.ydata

        new_x0 = x - (x - x0) * scale
        new_x1 = x + (x1 - x) * scale
        new_y0 = y - (y - y0) * scale
        new_y1 = y + (y1 - y) * scale

        self.axes.set_xlim(new_x0, new_x1)
        self.axes.set_ylim(new_y0, new_y1)
        self.canvas.draw_idle()

    def refresh_ports(self) -> None:
        ports = [p.device for p in list_ports.comports()]
        self.port_box["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        if not ports:
            self.port_var.set("")
            self.status.set("No serial ports found. Connect the board and click Refresh.")

    def start_capture(self) -> None:
        port_name = self.port_var.get()
        if not port_name:
            messagebox.showerror("No port selected", "Select the STM32U585 serial port first.")
            return

        self.capture_button.configure(state="disabled")
        self.save_button.configure(state="disabled")
        self.status.set("Applying 2 V step and recording 100 ms (target 50 kS/s)…")
        threading.Thread(target=self.capture_worker, args=(port_name,), daemon=True).start()

    def capture_worker(self, port_name: str) -> None:
        try:
            with serial.Serial(port_name, BAUD, timeout=8) as port:
                port.reset_input_buffer()
                port.write(b"CAPTURE\n")
                port.flush()
                rate, vin, vc = read_record(port)
            self.root.after(0, self.show_record, rate, vin, vc)
        except Exception as exc:
            self.root.after(0, self.show_error, str(exc))

    @staticmethod
    def adc_to_volts(samples: list[int]) -> list[float]:
        return [raw * ADC_REFERENCE_V / ADC_FULL_SCALE for raw in samples]

    def show_record(self, rate: int, vin: list[int], vc: list[int]) -> None:
        self.last_rate = rate
        self.last_vin = vin
        self.last_vc = vc

        times_ms = [1000.0 * i / rate for i in range(len(vin))]
        vin_v = self.adc_to_volts(vin)
        vc_v = self.adc_to_volts(vc)

        self.axes.clear()
        self.axes.plot(times_ms, vin_v, linewidth=1.2, label="Vin (PA1)")
        self.axes.plot(times_ms, vc_v, linewidth=1.2, label="Vc (PA0)")
        self.axes.set_title(
            f"STM32U585 RLC step response · {len(vin) / rate:.2f} s at {rate:,} samples/s"
        )
        self.axes.set_xlabel("Time from step (ms)")
        self.axes.set_ylabel("Voltage (V), nominal 3.3 V ADC reference")
        self.axes.set_xlim(0, min(DEFAULT_VIEW_MS, times_ms[-1] if times_ms else DEFAULT_VIEW_MS))
        self.axes.grid(True, alpha=0.3)
        self.axes.legend()
        self.canvas.draw_idle()

        self.status.set(
            f"Received {len(vin):,} two-channel samples. Plot shows the first {DEFAULT_VIEW_MS:.0f} ms."
        )
        self.capture_button.configure(state="normal")
        self.save_button.configure(state="normal")

    def save_csv(self) -> None:
        if self.last_rate is None or self.last_vin is None or self.last_vc is None:
            return

        path = filedialog.asksaveasfilename(
            title="Save RLC capture",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*")],
            initialfile="stm32u585_rlc_step.csv",
        )
        if not path:
            return

        vin_v = self.adc_to_volts(self.last_vin)
        vc_v = self.adc_to_volts(self.last_vc)

        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["sample", "time_s", "vin_adc", "vc_adc", "vin_V", "vc_V"])
            for i, (vin_raw, vc_raw, vin_volt, vc_volt) in enumerate(
                zip(self.last_vin, self.last_vc, vin_v, vc_v)
            ):
                writer.writerow(
                    [i, f"{i / self.last_rate:.9f}", vin_raw, vc_raw,
                     f"{vin_volt:.6f}", f"{vc_volt:.6f}"]
                )

        self.status.set(f"Saved CSV: {path}")

    def show_error(self, message: str) -> None:
        self.status.set("Capture failed.")
        self.capture_button.configure(state="normal")
        messagebox.showerror("Capture failed", message)


def main() -> None:
    root = tk.Tk()
    CaptureApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
