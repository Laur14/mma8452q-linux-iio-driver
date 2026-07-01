#!/usr/bin/env python3
"""
mma8452q_gui_v3.py

Aplicatie grafica user-space pentru driverul Linux IIO MMA8452Q.

Functii:
  - detectare automata /sys/bus/iio/devices/iio:deviceX dupa name == mma8452q
  - afisare raw X/Y/Z
  - conversie raw -> g folosind in_accel_scale
  - calcul modul vector acceleratie |a|
  - setare range: 2g / 4g / 8g
  - setare sampling frequency: 1.56, 6.25, 12.5, 50, 100, 200, 400, 800 Hz
  - afisare live in direct mode
  - calibrare offset din N esantioane
  - logging CSV in direct mode
  - citire buffer IIO din /dev/iio:deviceX
  - masurare rata reala in buffer mode
  - cleanup buffer

Rulare:
  python3 mma8452q_user.pydevice_pathbus
  sudo python3 mma8452q_user.py    
  sudo -E python3 mma8452q_user.py
"""

from __future__ import annotations

import csv
import math
import os
import queue
import struct
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Optional, TextIO

IIO_ROOT = Path("/sys/bus/iio/devices")
DEFAULT_DEVICE_NAME = "mma8452q"

RANGE_TO_SCALE = {
    "2g": "0.000976562",
    "4g": "0.001953125",
    "8g": "0.003906250",
}

VALID_FREQ_STRINGS = {
    "1.56": "1.560000",
    "6.25": "6.250000",
    "12.5": "12.500000",
    "50": "50",
    "100": "100",
    "200": "200",
    "400": "400",
    "800": "800",
}

FREQ_DISPLAY = ["1.56", "6.25", "12.5", "50", "100", "200", "400", "800"]

DEFAULT_RANGE = "2g"
DEFAULT_FREQ = "100"
BUFFER_UI_PERIOD_S = 0.08 
BUFFER_RATE_PERIOD_S = 1.00


class IIOError(RuntimeError):
    pass


@dataclass
class Offsets:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Sample:
    timestamp_user_ns: int
    x_raw: int
    y_raw: int
    z_raw: int
    x_g: float
    y_g: float
    z_g: float
    magnitude_g: float
    scale: float
    sampling_frequency: float
    timestamp_iio_ns: Optional[int] = None


class MMA8452QDevice:
    def __init__(self, device_name: str = DEFAULT_DEVICE_NAME, device_path: Optional[Path] = None):
        self.device_name = device_name
        self.dev_path = device_path if device_path else self.find_iio_device(device_name)
        self.dev_num = self.dev_path.name.replace("iio:device", "")
        self.chardev_path = Path(f"/dev/iio:device{self.dev_num}")
        self.offsets = Offsets()

    @staticmethod
    def find_iio_device(device_name: str = DEFAULT_DEVICE_NAME) -> Path:
        if not IIO_ROOT.exists():
            raise IIOError(f"Nu exista {IIO_ROOT}. Driverul IIO nu pare incarcat.")

        for dev in sorted(IIO_ROOT.glob("iio:device*")):
            name_file = dev / "name"
            if name_file.exists():
                try:
                    if name_file.read_text().strip() == device_name:
                        return dev
                except PermissionError:
                    continue

        found = []
        for dev in sorted(IIO_ROOT.glob("iio:device*")):
            name_file = dev / "name"
            if name_file.exists():
                try:
                    found.append(f"{dev.name}: {name_file.read_text().strip()}")
                except Exception:
                    found.append(f"{dev.name}: <nu pot citi>")
        raise IIOError(f"Nu am gasit IIO device cu name == {device_name}. Gasite: {', '.join(found) or 'nimic'}")

    def p(self, relative: str) -> Path:
        return self.dev_path / relative

    @staticmethod
    def read_text(path: Path) -> str:
        try:
            return path.read_text().strip()
        except FileNotFoundError as exc:
            raise IIOError(f"Fisier lipsa: {path}") from exc
        except PermissionError as exc:
            raise IIOError(f"Nu ai permisiune de citire: {path}") from exc

    @staticmethod
    def write_text(path: Path, value: str) -> None:
        try:
            path.write_text(str(value))
        except FileNotFoundError as exc:
            raise IIOError(f"Fisier lipsa: {path}") from exc
        except PermissionError as exc:
            raise IIOError(f"Nu ai permisiune de scriere: {path}. Ruleaza cu sudo sau schimba permisiunile.") from exc
        except OSError as exc:
            raise IIOError(f"Nu pot scrie '{value}' in {path}: {exc}") from exc

    def read_raw_xyz(self) -> tuple[int, int, int]:
        return (
            int(self.read_text(self.p("in_accel_x_raw"))),
            int(self.read_text(self.p("in_accel_y_raw"))),
            int(self.read_text(self.p("in_accel_z_raw"))),
        )

    def read_scale(self) -> float:
        return float(self.read_text(self.p("in_accel_scale")))

    def read_scale_available(self) -> str:
        return self.read_text(self.p("in_accel_scale_available"))

    def read_sampling_frequency(self) -> float:
        return float(self.read_text(self.p("in_accel_sampling_frequency")))

    def read_sampling_frequency_available(self) -> str:
        return self.read_text(self.p("in_accel_sampling_frequency_available"))

    def set_range(self, range_name: str) -> None:
        if range_name not in RANGE_TO_SCALE:
            raise IIOError("Range invalid. Alege 2g, 4g sau 8g.")
        self.write_text(self.p("in_accel_scale"), RANGE_TO_SCALE[range_name])

    def set_sampling_frequency(self, freq: str) -> None:
        if freq not in VALID_FREQ_STRINGS:
            raise IIOError("Frecventa invalida.")
        self.write_text(self.p("in_accel_sampling_frequency"), VALID_FREQ_STRINGS[freq])

    def read_sample(self) -> Sample:
        scale = self.read_scale()
        freq = self.read_sampling_frequency()
        x_raw, y_raw, z_raw = self.read_raw_xyz()

        x_cal = x_raw - self.offsets.x
        y_cal = y_raw - self.offsets.y
        z_cal = z_raw - self.offsets.z

        x_g = x_cal * scale
        y_g = y_cal * scale
        z_g = z_cal * scale
        mag = math.sqrt(x_g * x_g + y_g * y_g + z_g * z_g)

        return Sample(time.time_ns(), x_raw, y_raw, z_raw, x_g, y_g, z_g, mag, scale, freq)

    def calibrate(self, n: int, delay_s: float = 0.02, stop_event: Optional[threading.Event] = None) -> Offsets:
        if n <= 0:
            raise IIOError("Numarul de esantioane trebuie sa fie pozitiv.")
        sx = sy = sz = 0.0
        count = 0
        for _ in range(n):
            if stop_event and stop_event.is_set():
                break
            x, y, z = self.read_raw_xyz()
            sx += x
            sy += y
            sz += z
            count += 1
            time.sleep(delay_s)
        if count == 0:
            raise IIOError("Calibrare oprita inainte de primul esantion.")
        self.offsets = Offsets(sx / count, sy / count, sz / count)
        return self.offsets

    @staticmethod
    def csv_header() -> list[str]:
        return [
            "timestamp_user_ns", "timestamp_iio_ns",
            "x_raw", "y_raw", "z_raw",
            "x_g", "y_g", "z_g", "magnitude_g",
            "scale", "sampling_frequency",
        ]

    @staticmethod
    def csv_row(sample: Sample) -> list[object]:
        return [
            sample.timestamp_user_ns,
            sample.timestamp_iio_ns if sample.timestamp_iio_ns is not None else "",
            sample.x_raw, sample.y_raw, sample.z_raw,
            f"{sample.x_g:.9f}", f"{sample.y_g:.9f}", f"{sample.z_g:.9f}",
            f"{sample.magnitude_g:.9f}",
            f"{sample.scale:.9f}", f"{sample.sampling_frequency:.6f}",
        ]

    def cleanup_buffer(self) -> None:
        for rel, value in [
            ("buffer/enable", "0"),
            ("trigger/current_trigger", ""),
            ("scan_elements/in_accel_x_en", "0"),
            ("scan_elements/in_accel_y_en", "0"),
            ("scan_elements/in_accel_z_en", "0"),
            ("scan_elements/in_timestamp_en", "0"),
        ]:
            path = self.p(rel)
            if path.exists():
                try:
                    self.write_text(path, value)
                except IIOError:
                    pass

    def find_trigger_name(self) -> str:
        own = f"{self.device_name}-trigger"
        names = []
        for trig in sorted(IIO_ROOT.glob("trigger*")):
            name_file = trig / "name"
            if name_file.exists():
                name = self.read_text(name_file)
                names.append(name)
                if name == own:
                    return name
        if names:
            return names[0]
        raise IIOError("Nu exista trigger IIO disponibil.")

    def setup_buffer(self, length: int, trigger_name: Optional[str] = None) -> None:
        enable = self.p("buffer/enable")
        if not enable.exists():
            raise IIOError("Driverul nu expune buffer/enable.")
        if self.read_text(enable) == "1":
            raise IIOError("Bufferul este deja activ. Apasa Cleanup Buffer.")

        trigger_name = trigger_name or self.find_trigger_name()

        for channel in ["in_accel_x_en", "in_accel_y_en", "in_accel_z_en", "in_timestamp_en"]:
            path = self.p(f"scan_elements/{channel}")
            if path.exists():
                self.write_text(path, "1")

        self.write_text(self.p("buffer/length"), str(length))
        self.write_text(self.p("trigger/current_trigger"), trigger_name)
        self.write_text(enable, "1")

    @staticmethod
    def sign_extend(value: int, bits: int) -> int:
        sign_bit = 1 << (bits - 1)
        return (value ^ sign_bit) - sign_bit

    @classmethod
    def decode_buffer_sample(cls, data: bytes) -> tuple[int, int, int, int]:
        if len(data) < 16:
            raise IIOError("Sample buffer incomplet.")
        x16, y16, z16 = struct.unpack_from("<hhh", data, 0)
        timestamp_ns = struct.unpack_from("<q", data, 8)[0]

        def norm(v: int) -> int:
            return cls.sign_extend(v & 0x0FFF, 12)

        return norm(x16), norm(y16), norm(z16), timestamp_ns

    def make_sample_from_raw(self, x_raw: int, y_raw: int, z_raw: int, timestamp_iio_ns: Optional[int] = None, scale: Optional[float] = None, freq: Optional[float] = None) -> Sample:
        scale = self.read_scale() if scale is None else scale
        freq = self.read_sampling_frequency() if freq is None else freq

        x_cal = x_raw - self.offsets.x
        y_cal = y_raw - self.offsets.y
        z_cal = z_raw - self.offsets.z

        x_g = x_cal * scale
        y_g = y_cal * scale
        z_g = z_cal * scale
        mag = math.sqrt(x_g * x_g + y_g * y_g + z_g * z_g)
        return Sample(time.time_ns(), x_raw, y_raw, z_raw, x_g, y_g, z_g, mag, scale, freq, timestamp_iio_ns)

    def read_one_buffer_sample(self) -> Sample:
        if not self.chardev_path.exists():
            raise IIOError(f"Nu exista {self.chardev_path}")
        scale = self.read_scale()
        freq = self.read_sampling_frequency()
        with self.chardev_path.open("rb", buffering=0) as f:
            raw = f.read(16)
        x_raw, y_raw, z_raw, ts_iio = self.decode_buffer_sample(raw)
        return self.make_sample_from_raw(x_raw, y_raw, z_raw, ts_iio, scale, freq)


class MMA8452QGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("MMA8452Q IIO Monitor")
        self.geometry("1250x850")
        self.minsize(1150, 780)

        self.dev: Optional[MMA8452QDevice] = None
        self.worker_stop = threading.Event()
        self.worker_thread: Optional[threading.Thread] = None
        self.ui_queue: queue.Queue = queue.Queue()

        self.csv_file: Optional[TextIO] = None
        self.csv_writer: Optional[csv.writer] = None
        self.log_path: Optional[Path] = None
        self.sample_history: list[Sample] = []
        self.max_history = 80
        self.live_running = False
        self.buffer_running = False
        self.buffer_start_time = 0.0
        self.buffer_count = 0
        self.buffer_last_ui_time = 0.0
        self.buffer_ui_period_s = BUFFER_UI_PERIOD_S   

        self._build_ui()
        self.after(33, self.process_queue)
        self.connect_device(show_errors=False)
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self) -> None:
        self.columnconfigure(0, weight=1)
        self.rowconfigure(2, weight=1)

        top = ttk.LabelFrame(self, text="Device")
        top.grid(row=0, column=0, sticky="ew", padx=10, pady=8)
        top.columnconfigure(1, weight=1)

        self.device_var = tk.StringVar(value="-")
        self.chardev_var = tk.StringVar(value="-")
        self.status_var = tk.StringVar(value="Neconectat")

        ttk.Label(top, text="IIO:").grid(row=0, column=0, sticky="w", padx=6, pady=3)
        ttk.Label(top, textvariable=self.device_var).grid(row=0, column=1, sticky="ew", padx=6, pady=3)
        ttk.Label(top, text="Char:").grid(row=1, column=0, sticky="w", padx=6, pady=3)
        ttk.Label(top, textvariable=self.chardev_var).grid(row=1, column=1, sticky="ew", padx=6, pady=3)
        ttk.Label(top, text="Status:").grid(row=2, column=0, sticky="w", padx=6, pady=3)
        ttk.Label(top, textvariable=self.status_var).grid(row=2, column=1, sticky="ew", padx=6, pady=3)
        ttk.Button(top, text="Detecteaza", command=self.connect_device).grid(row=0, column=2, rowspan=2, padx=6, pady=3, sticky="ns")
        ttk.Button(top, text="Cleanup Buffer", command=self.cleanup_buffer).grid(row=2, column=2, padx=6, pady=3, sticky="ew")

        controls = ttk.LabelFrame(self, text="Configurare")
        controls.grid(row=1, column=0, sticky="ew", padx=10, pady=4)
        for i in range(8):
            controls.columnconfigure(i, weight=1)

        self.range_var = tk.StringVar(value="2g")
        self.freq_var = tk.StringVar(value="100")
        self.period_var = tk.StringVar(value="0.2")
        self.calib_var = tk.StringVar(value="100")
        self.buffer_len_var = tk.StringVar(value="64")

        ttk.Label(controls, text="Range").grid(row=0, column=0, padx=5, pady=4)
        ttk.Combobox(controls, textvariable=self.range_var, values=["2g", "4g", "8g"], width=8, state="readonly").grid(row=1, column=0, padx=5, pady=4)
        ttk.Button(controls, text="Set range", command=self.set_range).grid(row=2, column=0, padx=5, pady=4)

        ttk.Label(controls, text="Frecventa [Hz]").grid(row=0, column=1, padx=5, pady=4)
        ttk.Combobox(controls, textvariable=self.freq_var, values=FREQ_DISPLAY, width=8, state="readonly").grid(row=1, column=1, padx=5, pady=4)
        ttk.Button(controls, text="Set freq", command=self.set_frequency).grid(row=2, column=1, padx=5, pady=4)

        ttk.Label(controls, text="Perioada live [s]").grid(row=0, column=2, padx=5, pady=4)
        ttk.Entry(controls, textvariable=self.period_var, width=8).grid(row=1, column=2, padx=5, pady=4)
        ttk.Button(controls, text="Start Live", command=self.start_live).grid(row=2, column=2, padx=5, pady=4)

        ttk.Label(controls, text="Calibrare N").grid(row=0, column=3, padx=5, pady=4)
        ttk.Entry(controls, textvariable=self.calib_var, width=8).grid(row=1, column=3, padx=5, pady=4)
        ttk.Button(controls, text="Calibreaza", command=self.start_calibration).grid(row=2, column=3, padx=5, pady=4)

        ttk.Label(controls, text="Buffer length").grid(row=0, column=4, padx=5, pady=4)
        ttk.Entry(controls, textvariable=self.buffer_len_var, width=8).grid(row=1, column=4, padx=5, pady=4)
        ttk.Button(controls, text="Start Buffer", command=self.start_buffer).grid(row=2, column=4, padx=5, pady=4)

        ttk.Button(controls, text="Citeste o data", command=self.read_once).grid(row=1, column=5, padx=5, pady=4, sticky="ew")
        ttk.Button(controls, text="Stop", command=self.stop_worker).grid(row=2, column=5, padx=5, pady=4, sticky="ew")
        ttk.Button(controls, text="Alege CSV", command=self.choose_csv).grid(row=1, column=6, padx=5, pady=4, sticky="ew")
        ttk.Button(controls, text="Opreste CSV", command=self.close_csv).grid(row=2, column=6, padx=5, pady=4, sticky="ew")

        self.logging_var = tk.StringVar(value="CSV: oprit")
        ttk.Label(controls, textvariable=self.logging_var).grid(row=1, column=7, rowspan=2, padx=5, pady=4, sticky="w")

        main = ttk.Frame(self)
        main.grid(row=2, column=0, sticky="nsew", padx=10, pady=8)
        main.columnconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(0, weight=1)

        values = ttk.LabelFrame(main, text="Valori curente")
        values.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        for i in range(4):
            values.columnconfigure(i, weight=1)

        self.raw_vars = {axis: tk.StringVar(value="0") for axis in "XYZ"}
        self.g_vars = {axis: tk.StringVar(value="0.00000") for axis in "XYZ"}
        self.mag_var = tk.StringVar(value="0.00000")
        self.scale_var = tk.StringVar(value="-")
        self.sampling_var = tk.StringVar(value="-")
        self.offset_var = tk.StringVar(value="X=0, Y=0, Z=0")
        self.rate_var = tk.StringVar(value="Rata buffer: -")

        ttk.Label(values, text="Axa", font=("TkDefaultFont", 10, "bold")).grid(row=0, column=0, pady=4)
        ttk.Label(values, text="Raw", font=("TkDefaultFont", 10, "bold")).grid(row=0, column=1, pady=4)
        ttk.Label(values, text="g", font=("TkDefaultFont", 10, "bold")).grid(row=0, column=2, pady=4)

        for idx, axis in enumerate("XYZ", start=1):
            ttk.Label(values, text=axis, font=("TkDefaultFont", 13, "bold")).grid(row=idx, column=0, pady=6)
            ttk.Label(values, textvariable=self.raw_vars[axis], font=("TkDefaultFont", 13)).grid(row=idx, column=1, pady=6)
            ttk.Label(values, textvariable=self.g_vars[axis], font=("TkDefaultFont", 13)).grid(row=idx, column=2, pady=6)

        ttk.Separator(values).grid(row=4, column=0, columnspan=3, sticky="ew", pady=8)
        ttk.Label(values, text="|a| [g]").grid(row=5, column=0, sticky="e", padx=6, pady=4)
        ttk.Label(values, textvariable=self.mag_var, font=("TkDefaultFont", 14, "bold")).grid(row=5, column=1, columnspan=2, sticky="w", padx=6, pady=4)
        ttk.Label(values, text="Scale").grid(row=6, column=0, sticky="e", padx=6, pady=4)
        ttk.Label(values, textvariable=self.scale_var).grid(row=6, column=1, columnspan=2, sticky="w", padx=6, pady=4)
        ttk.Label(values, text="Sampling").grid(row=7, column=0, sticky="e", padx=6, pady=4)
        ttk.Label(values, textvariable=self.sampling_var).grid(row=7, column=1, columnspan=2, sticky="w", padx=6, pady=4)
        ttk.Label(values, text="Offset raw").grid(row=8, column=0, sticky="e", padx=6, pady=4)
        ttk.Label(values, textvariable=self.offset_var).grid(row=8, column=1, columnspan=2, sticky="w", padx=6, pady=4)
        ttk.Label(values, textvariable=self.rate_var).grid(row=9, column=0, columnspan=3, sticky="w", padx=6, pady=4)

        graph_box = ttk.LabelFrame(main, text="Grafic simplu |a| [g]")
        graph_box.grid(row=0, column=1, sticky="nsew", padx=(6, 0))
        graph_box.rowconfigure(0, weight=1)
        graph_box.columnconfigure(0, weight=1)
        self.canvas = tk.Canvas(graph_box, background="white", height=260)
        self.canvas.grid(row=0, column=0, sticky="nsew", padx=8, pady=8)

        bottom = ttk.LabelFrame(self, text="Log evenimente")
        bottom.grid(row=3, column=0, sticky="nsew", padx=10, pady=(0, 10))
        bottom.columnconfigure(0, weight=1)
        bottom.rowconfigure(0, weight=1)
        self.log_text = tk.Text(bottom, height=8)
        self.log_text.grid(row=0, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(bottom, orient="vertical", command=self.log_text.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        self.log_text.configure(yscrollcommand=scroll.set)

    def log(self, msg: str) -> None:
        ts = time.strftime("%H:%M:%S")
        self.log_text.insert("end", f"[{ts}] {msg}\n")
        self.log_text.see("end")

    def connect_device(self, show_errors: bool = True) -> None:
        try:
            self.dev = MMA8452QDevice()
            self.device_var.set(str(self.dev.dev_path))
            self.chardev_var.set(str(self.dev.chardev_path))
            self.status_var.set("Conectat")
            self.log("Device detectat corect.")
            self.apply_defaults_on_startup()
            self.refresh_static_info()
        except Exception as exc:
            self.dev = None
            self.status_var.set("Eroare detectare")
            if show_errors:
                messagebox.showerror("Eroare", str(exc))
            self.log(str(exc))

    def apply_defaults_on_startup(self) -> None:
        """
        La pornirea aplicatiei scriem explicit configuratia implicita in driver.
        Altfel, sysfs pastreaza starea ramasa din testul anterior, de exemplu 800 Hz.
        """
        try:
            dev = self.require_dev()
           
            dev.cleanup_buffer()
            dev.set_range(DEFAULT_RANGE)
            dev.set_sampling_frequency(DEFAULT_FREQ)
            self.range_var.set(DEFAULT_RANGE)
            self.freq_var.set(DEFAULT_FREQ)
            self.log(f"Default aplicat la pornire: range={DEFAULT_RANGE}, freq={DEFAULT_FREQ} Hz.")
        except Exception as exc:
            self.log(f"Nu am putut aplica default la pornire: {exc}")

    def require_dev(self) -> MMA8452QDevice:
        if self.dev is None:
            raise IIOError("Device-ul nu este detectat.")
        return self.dev

    def refresh_static_info(self) -> None:
        dev = self.require_dev()
        self.scale_var.set(f"{dev.read_scale():.9f}")
        self.sampling_var.set(f"{dev.read_sampling_frequency():g} Hz")
        self.offset_var.set(f"X={dev.offsets.x:.2f}, Y={dev.offsets.y:.2f}, Z={dev.offsets.z:.2f}")

    def set_range(self) -> None:
        try:
            dev = self.require_dev()
            dev.set_range(self.range_var.get())
            self.refresh_static_info()
            self.log(f"Range setat la {self.range_var.get()}.")
        except Exception as exc:
            messagebox.showerror("Eroare", str(exc))
            self.log(str(exc))

    def set_frequency(self) -> None:
        try:
            dev = self.require_dev()
            dev.set_sampling_frequency(self.freq_var.get())
            self.refresh_static_info()
            self.log(f"Frecventa ceruta: {self.freq_var.get()} Hz; frecventa citita din driver: {dev.read_sampling_frequency():g} Hz.")
        except Exception as exc:
            messagebox.showerror("Eroare", str(exc))
            self.log(str(exc))

    def update_sample_ui(self, sample: Sample) -> None:
        self.raw_vars["X"].set(str(sample.x_raw))
        self.raw_vars["Y"].set(str(sample.y_raw))
        self.raw_vars["Z"].set(str(sample.z_raw))
        self.g_vars["X"].set(f"{sample.x_g:+.5f}")
        self.g_vars["Y"].set(f"{sample.y_g:+.5f}")
        self.g_vars["Z"].set(f"{sample.z_g:+.5f}")
        self.mag_var.set(f"{sample.magnitude_g:.5f}")
        self.scale_var.set(f"{sample.scale:.9f}")
        self.sampling_var.set(f"{sample.sampling_frequency:g} Hz")

        self.sample_history.append(sample)
        if len(self.sample_history) > self.max_history:
            self.sample_history = self.sample_history[-self.max_history:]
        self.draw_graph()
        self.write_csv_sample(sample)

    def draw_graph(self) -> None:
        self.canvas.delete("all")
        w = max(self.canvas.winfo_width(), 10)
        h = max(self.canvas.winfo_height(), 10)
        pad = 28

        range_name = self.range_var.get()
        graph_max_g = {
            "2g": 2.0,
            "4g": 4.0,
            "8g": 8.0,
        }.get(range_name, 8.0)

        mid_g = graph_max_g / 2.0

        self.canvas.create_line(pad, h - pad, w - pad, h - pad)
        self.canvas.create_line(pad, pad, pad, h - pad)

        self.canvas.create_text(8, pad, anchor="w", text=f"{graph_max_g:g}g")
        self.canvas.create_text(8, h // 2, anchor="w", text=f"{mid_g:g}g")
        self.canvas.create_text(8, h - pad, anchor="w", text="0g")

        if len(self.sample_history) < 2:
            return

        mags = [min(max(s.magnitude_g, 0.0), graph_max_g) for s in self.sample_history]
        window = 4
        if len(mags) >= window:
            mags = [
                sum(mags[max(0, i - window + 1):i + 1]) / len(mags[max(0, i - window + 1):i + 1])
                for i in range(len(mags))
            ]
        step = (w - 2 * pad) / max(len(mags) - 1, 1)

        pts = []
        for i, mag in enumerate(mags):
            x = pad + i * step
            y = h - pad - (mag / graph_max_g) * (h - 2 * pad)
            pts.extend([x, y])

        self.canvas.create_line(*pts, width=2)

      
        if graph_max_g >= 1.0:
            y_1g = h - pad - (1.0 / graph_max_g) * (h - 2 * pad)
            self.canvas.create_line(pad, y_1g, w - pad, y_1g, dash=(4, 3))
            self.canvas.create_text(w - pad - 20, y_1g - 8, text="1g")

    def read_once(self) -> None:
        try:
            sample = self.require_dev().read_sample()
            self.update_sample_ui(sample)
            self.log("Citire directa executata.")
        except Exception as exc:
            messagebox.showerror("Eroare", str(exc))
            self.log(str(exc))

    def choose_csv(self) -> None:
        path = filedialog.asksaveasfilename(
            title="Alege fisier CSV",
            defaultextension=".csv",
            filetypes=[("CSV", "*.csv"), ("All files", "*.*")],
        )
        if not path:
            return
        try:
            self.close_csv()
            self.log_path = Path(path)
            self.csv_file = self.log_path.open("w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(MMA8452QDevice.csv_header())
            self.logging_var.set(f"CSV: {self.log_path.name}")
            self.log(f"Logging CSV pornit: {self.log_path}")
        except Exception as exc:
            messagebox.showerror("Eroare CSV", str(exc))
            self.log(str(exc))

    def close_csv(self) -> None:
        if self.csv_file:
            self.csv_file.close()
        self.csv_file = None
        self.csv_writer = None
        self.log_path = None
        self.logging_var.set("CSV: oprit")

    def write_csv_sample(self, sample: Sample) -> None:
        if self.csv_writer and self.csv_file:
            self.csv_writer.writerow(MMA8452QDevice.csv_row(sample))
            self.csv_file.flush()

    def stop_worker(self) -> None:
        self.worker_stop.set()
        self.live_running = False
        self.buffer_running = False
        self.status_var.set("Oprire ceruta")
        self.log("Oprire ceruta.")

    def start_live(self) -> None:
        if self.worker_thread and self.worker_thread.is_alive():
            messagebox.showwarning("Atentie", "Exista deja o operatie pornita.")
            return
        try:
            period = float(self.period_var.get())
            if period <= 0:
                raise ValueError
        except ValueError:
            messagebox.showerror("Eroare", "Perioada trebuie sa fie numar pozitiv.")
            return

        self.worker_stop.clear()
        self.live_running = True
        self.status_var.set("Live direct mode")
        self.worker_thread = threading.Thread(target=self.live_worker, args=(period,), daemon=True)
        self.worker_thread.start()
        self.log("Live direct mode pornit.")

    def live_worker(self, period: float) -> None:
        try:
            dev = self.require_dev()
            while not self.worker_stop.is_set():
                sample = dev.read_sample()
                self.ui_queue.put(("sample", sample))
                time.sleep(period)
        except Exception as exc:
            self.ui_queue.put(("error", str(exc)))
        finally:
            self.ui_queue.put(("status", "Live oprit"))

    def start_calibration(self) -> None:
        if self.worker_thread and self.worker_thread.is_alive():
            messagebox.showwarning("Atentie", "Opreste live/buffer inainte de calibrare.")
            return
        try:
            n = int(self.calib_var.get())
            if n <= 0:
                raise ValueError
        except ValueError:
            messagebox.showerror("Eroare", "N trebuie sa fie intreg pozitiv.")
            return
        self.worker_stop.clear()
        self.status_var.set("Calibrare...")
        self.worker_thread = threading.Thread(target=self.calibration_worker, args=(n,), daemon=True)
        self.worker_thread.start()
        self.log("Calibrare pornita. Tine senzorul nemiscat.")

    def calibration_worker(self, n: int) -> None:
        try:
            offsets = self.require_dev().calibrate(n, stop_event=self.worker_stop)
            self.ui_queue.put(("calibrated", offsets))
        except Exception as exc:
            self.ui_queue.put(("error", str(exc)))

    def start_buffer(self) -> None:
        if self.worker_thread and self.worker_thread.is_alive():
            messagebox.showwarning("Atentie", "Exista deja o operatie pornita.")
            return
        try:
            length = int(self.buffer_len_var.get())
            if length <= 0:
                raise ValueError
        except ValueError:
            messagebox.showerror("Eroare", "Buffer length trebuie sa fie intreg pozitiv.")
            return


        if self.csv_file:
            path = self.log_path
            self.close_csv()
            self.log_path = path
            if path:
                self.logging_var.set(f"CSV buffer: {path.name}")

        self.worker_stop.clear()
        self.buffer_running = True
        self.buffer_count = 0
        self.buffer_start_time = time.monotonic()
        self.buffer_last_ui_time = self.buffer_start_time
        self.status_var.set("Buffer mode")
        self.worker_thread = threading.Thread(target=self.buffer_worker, args=(length,), daemon=True)
        self.worker_thread.start()
        self.log("Buffer mode pornit.")

    def buffer_worker(self, length: int) -> None:
        dev: Optional[MMA8452QDevice] = None
        csv_file: Optional[TextIO] = None
        csv_writer: Optional[csv.writer] = None
        try:
            dev = self.require_dev()
            dev.setup_buffer(length=length)
            self.ui_queue.put(("buffer_started", None))

            if self.log_path:
              
                csv_file = self.log_path.open("a", newline="")
                csv_writer = csv.writer(csv_file)

            sample_size = 16
            scale = dev.read_scale()
            freq = dev.read_sampling_frequency()
            local_count = 0
            last_ui = 0.0
            last_rate = time.monotonic()
            last_rate_count = 0

            if not dev.chardev_path.exists():
                raise IIOError(f"Nu exista {dev.chardev_path}")

            with dev.chardev_path.open("rb", buffering=0) as f:
                while not self.worker_stop.is_set():
                    raw = f.read(sample_size)
                    if len(raw) != sample_size:
                        continue

                    x_raw, y_raw, z_raw, ts_iio = dev.decode_buffer_sample(raw)
                    sample = dev.make_sample_from_raw(x_raw, y_raw, z_raw, ts_iio, scale, freq)
                    local_count += 1

                    if csv_writer and csv_file:
                        csv_writer.writerow(MMA8452QDevice.csv_row(sample))
                 
                        if local_count % 64 == 0:
                            csv_file.flush()

                    now = time.monotonic()
                    if now - last_ui >= self.buffer_ui_period_s:
                        self.ui_queue.put(("buffer_sample", (sample, local_count, now)))
                        last_ui = now

                    if now - last_rate >= BUFFER_RATE_PERIOD_S:
                        real_rate = (local_count - last_rate_count) / max(now - last_rate, 1e-9)
                        self.ui_queue.put(("buffer_rate", real_rate))
                        last_rate = now
                        last_rate_count = local_count

        except PermissionError as exc:
            self.ui_queue.put(("error", f"Nu ai permisiune pentru {dev.chardev_path if dev else '/dev/iio:deviceX'}. Ruleaza cu sudo."))
        except Exception as exc:
            self.ui_queue.put(("error", str(exc)))
        finally:
            if csv_file:
                csv_file.flush()
                csv_file.close()
            if dev:
                dev.cleanup_buffer()
            self.ui_queue.put(("status", "Buffer oprit"))

    def cleanup_buffer(self) -> None:
        try:
            self.require_dev().cleanup_buffer()
            self.log("Buffer dezactivat si trigger curatat.")
        except Exception as exc:
            messagebox.showerror("Eroare", str(exc))
            self.log(str(exc))

    def process_queue(self) -> None:
      
        max_events_per_tick = 25
        processed = 0
        try:
            while processed < max_events_per_tick:
                kind, payload = self.ui_queue.get_nowait()
                processed += 1

                if kind == "sample":
                    self.update_sample_ui(payload)
                elif kind == "buffer_started":
                    self.buffer_start_time = time.monotonic()
                    self.buffer_count = 0
                    self.log("Buffer activat. Rata se masoara din numarul real de sample-uri citite.")
                elif kind == "buffer_sample":
                    sample, local_count, now = payload
                    self.buffer_count = local_count
                    elapsed = max(now - self.buffer_start_time, 1e-9)
                    self.rate_var.set(f"Rata buffer medie: {self.buffer_count / elapsed:.2f} samples/s")
                    self.update_sample_ui(sample)
                elif kind == "buffer_rate":
                    self.rate_var.set(f"Rata buffer reala: {payload:.2f} samples/s")
                elif kind == "error":
                    self.status_var.set("Eroare")
                    self.log(payload)
                    messagebox.showerror("Eroare", payload)
                    self.worker_stop.set()
                elif kind == "status":
                    self.status_var.set(payload)
                    self.log(payload)
                elif kind == "calibrated":
                    off: Offsets = payload
                    self.offset_var.set(f"X={off.x:.2f}, Y={off.y:.2f}, Z={off.z:.2f}")
                    self.status_var.set("Calibrare finalizata")
                    self.log(f"Offset calculat: X={off.x:.2f}, Y={off.y:.2f}, Z={off.z:.2f}")
        except queue.Empty:
            pass
        self.after(100, self.process_queue)

    def on_close(self) -> None:
        self.stop_worker()
        try:
            if self.dev:
                self.dev.cleanup_buffer()
        finally:
            self.close_csv()
            self.destroy()


if __name__ == "__main__":
    app = MMA8452QGUI()
    app.mainloop()