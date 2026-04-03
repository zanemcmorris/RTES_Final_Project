import argparse
import collections
import math
import sys
import threading
import time

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import serial


class IMUSerialReader:
    """Read lines of IMU data from a serial port.

    Expected line format:
        IMU,ax,ay,az,gx,gy,gz

    Example:
        IMU,0.01,-0.02,1.00,3.2,-1.1,0.4

    Units are up to the sender, but the companion C helper below assumes:
        accel in g
        gyro in dps
    """

    def __init__(self, port: str, baud: int, maxlen: int = 500):
        self.port = port
        self.baud = baud
        self.maxlen = maxlen
        self.ser = None
        self.thread = None
        self.stop_event = threading.Event()
        self.lock = threading.Lock()

        self.t = collections.deque(maxlen=maxlen)
        self.ax = collections.deque(maxlen=maxlen)
        self.ay = collections.deque(maxlen=maxlen)
        self.az = collections.deque(maxlen=maxlen)
        self.gx = collections.deque(maxlen=maxlen)
        self.gy = collections.deque(maxlen=maxlen)
        self.gz = collections.deque(maxlen=maxlen)
        self.start_time = time.monotonic()
        self.bad_lines = 0

    def open(self) -> None:
        self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
        time.sleep(2.0)  # allow common dev boards to reset on port open

    def close(self) -> None:
        self.stop_event.set()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            self.ser.close()

    def start(self) -> None:
        if self.ser is None:
            self.open()
        self.thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.thread.start()

    def _reader_loop(self) -> None:
        assert self.ser is not None
        while not self.stop_event.is_set():
            try:
                raw = self.ser.readline()
            except serial.SerialException as exc:
                print(f"Serial error: {exc}", file=sys.stderr)
                break

            if not raw:
                continue

            try:
                line = raw.decode("utf-8", errors="replace").strip()
                parsed = self._parse_line(line)
                if parsed is None:
                    self.bad_lines += 1
                    continue
                ax, ay, az, gx, gy, gz = parsed
                now = time.monotonic() - self.start_time
                with self.lock:
                    self.t.append(now)
                    self.ax.append(ax)
                    self.ay.append(ay)
                    self.az.append(az)
                    self.gx.append(gx)
                    self.gy.append(gy)
                    self.gz.append(gz)
            except Exception:
                self.bad_lines += 1

    @staticmethod
    def _parse_line(line: str):
        parts = line.split(",")
        if len(parts) != 7:
            return None
        if parts[0] != "IMU":
            return None
        try:
            vals = [float(x) for x in parts[1:]]
        except ValueError:
            return None
        return vals

    def snapshot(self):
        with self.lock:
            return {
                "t": list(self.t),
                "ax": list(self.ax),
                "ay": list(self.ay),
                "az": list(self.az),
                "gx": list(self.gx),
                "gy": list(self.gy),
                "gz": list(self.gz),
                "bad_lines": self.bad_lines,
            }


def build_plot(reader: IMUSerialReader, window_seconds: float):
    fig, (ax_acc, ax_gyro) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    fig.suptitle("Live IMU UART Plot")

    acc_x_line, = ax_acc.plot([], [], label="ax (g)")
    acc_y_line, = ax_acc.plot([], [], label="ay (g)")
    acc_z_line, = ax_acc.plot([], [], label="az (g)")
    gyro_x_line, = ax_gyro.plot([], [], label="gx (dps)")
    gyro_y_line, = ax_gyro.plot([], [], label="gy (dps)")
    gyro_z_line, = ax_gyro.plot([], [], label="gz (dps)")

    ax_acc.set_ylabel("Acceleration")
    ax_gyro.set_ylabel("Angular Rate")
    ax_gyro.set_xlabel("Time (s)")
    ax_acc.legend(loc="upper left")
    ax_gyro.legend(loc="upper left")
    ax_acc.grid(True)
    ax_gyro.grid(True)

    status_text = fig.text(0.01, 0.01, "", fontsize=9)

    def update(_frame):
        data = reader.snapshot()
        t = data["t"]
        if not t:
            return (
                acc_x_line, acc_y_line, acc_z_line,
                gyro_x_line, gyro_y_line, gyro_z_line,
                status_text,
            )

        t_end = t[-1]
        t_start = max(0.0, t_end - window_seconds)

        i0 = 0
        for i, val in enumerate(t):
            if val >= t_start:
                i0 = i
                break

        ts = t[i0:]
        ax_vals = data["ax"][i0:]
        ay_vals = data["ay"][i0:]
        az_vals = data["az"][i0:]
        gx_vals = data["gx"][i0:]
        gy_vals = data["gy"][i0:]
        gz_vals = data["gz"][i0:]

        acc_x_line.set_data(ts, ax_vals)
        acc_y_line.set_data(ts, ay_vals)
        acc_z_line.set_data(ts, az_vals)
        gyro_x_line.set_data(ts, gx_vals)
        gyro_y_line.set_data(ts, gy_vals)
        gyro_z_line.set_data(ts, gz_vals)

        ax_acc.set_xlim(t_start, max(t_start + 0.1, t_end))
        ax_gyro.set_xlim(t_start, max(t_start + 0.1, t_end))

        def set_y_limits(axis, series_list):
            vals = [v for series in series_list for v in series]
            if not vals:
                return
            vmin = min(vals)
            vmax = max(vals)
            if math.isclose(vmin, vmax, rel_tol=1e-6, abs_tol=1e-6):
                pad = 1.0 if abs(vmin) < 1.0 else abs(vmin) * 0.1
            else:
                pad = (vmax - vmin) * 0.1
            axis.set_ylim(vmin - pad, vmax + pad)

        set_y_limits(ax_acc, [ax_vals, ay_vals, az_vals])
        set_y_limits(ax_gyro, [gx_vals, gy_vals, gz_vals])

        status_text.set_text(
            f"Samples: {len(t)}    Bad lines: {data['bad_lines']}    Latest t: {t_end:.2f}s"
        )

        return (
            acc_x_line, acc_y_line, acc_z_line,
            gyro_x_line, gyro_y_line, gyro_z_line,
            status_text,
        )

    anim = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    return fig, anim


def parse_args():
    parser = argparse.ArgumentParser(description="Live IMU serial plotter")
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM5 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--window", type=float, default=10.0, help="Visible time window in seconds")
    parser.add_argument("--history", type=int, default=5000, help="Maximum samples stored")
    return parser.parse_args()


def main():
    args = parse_args()
    reader = IMUSerialReader(args.port, args.baud, maxlen=args.history)
    try:
        reader.start()
        fig, _anim = build_plot(reader, args.window)
        plt.show()
    finally:
        reader.close()


if __name__ == "__main__":
    main()
