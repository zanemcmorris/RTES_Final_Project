import argparse
import collections
import math
import sys
import threading
import time

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import serial


def pressure_hpa_to_altitude_feet(pressure_hpa: float, sea_level_hpa: float = 1013.25) -> float:
    """Convert pressure in hPa to altitude above sea level in feet using the
    standard atmosphere barometric formula.
    """
    if pressure_hpa <= 0.0 or sea_level_hpa <= 0.0:
        return float("nan")
    altitude_m = 44330.0 * (1.0 - (pressure_hpa / sea_level_hpa) ** 0.19029495718363465)
    return altitude_m * 3.280839895013123


class IMUSerialReader:
    """Read IMU and barometer lines from a serial port.

    Expected line formats:
        IMU,ax,ay,az,gx,gy,gz
        BARO,pressure_hpa,temp_c

    Example:
        IMU,0.01,-0.02,1.00,3.2,-1.1,0.4
        BARO,835.42,21.75

    Units expected:
        accel in g
        gyro in dps
        pressure in hPa
        temperature in C
    """

    def __init__(
        self,
        port: str,
        baud: int,
        maxlen: int = 500,
        sea_level_hpa: float = 1013.25,
        min_altitude_ft: float = 4500.0,
        max_altitude_ft: float = 6000.0,
    ):
        self.port = port
        self.baud = baud
        self.maxlen = maxlen
        self.sea_level_hpa = sea_level_hpa
        self.min_altitude_ft = min_altitude_ft
        self.max_altitude_ft = max_altitude_ft
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

        self.t_baro = collections.deque(maxlen=maxlen)
        self.pressure_hpa = collections.deque(maxlen=maxlen)
        self.temperature_c = collections.deque(maxlen=maxlen)
        self.altitude_ft = collections.deque(maxlen=maxlen)

        self.start_time = time.monotonic()
        self.bad_lines = 0
        self.filtered_baro_lines = 0
        self.imu_samples = 0
        self.baro_samples = 0

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


    def _baro_sample_is_valid(self, pressure_hpa: float, altitude_ft: float) -> bool:
        if pressure_hpa <= 0.0 or pressure_hpa > 2000.0:
            return False
        if math.isnan(altitude_ft) or math.isinf(altitude_ft):
            return False
        if altitude_ft < self.min_altitude_ft or altitude_ft > self.max_altitude_ft:
            return False
        return True

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

                now = time.monotonic() - self.start_time
                with self.lock:
                    kind = parsed[0]
                    if kind == "IMU":
                        _, ax, ay, az, gx, gy, gz = parsed
                        self.t.append(now)
                        self.ax.append(ax)
                        self.ay.append(ay)
                        self.az.append(az)
                        self.gx.append(gx)
                        self.gy.append(gy)
                        self.gz.append(gz)
                        self.imu_samples += 1
                    elif kind == "BARO":
                        _, pressure_hpa, temperature_c = parsed
                        altitude_ft = pressure_hpa_to_altitude_feet(
                            pressure_hpa, self.sea_level_hpa
                        )

                        if not self._baro_sample_is_valid(pressure_hpa, altitude_ft):
                            self.filtered_baro_lines += 1
                            continue

                        self.t_baro.append(now)
                        self.pressure_hpa.append(pressure_hpa)
                        self.temperature_c.append(temperature_c)
                        self.altitude_ft.append(altitude_ft)
                        self.baro_samples += 1
            except Exception:
                self.bad_lines += 1

    @staticmethod
    def _parse_line(line: str):
        parts = line.split(",")
        if not parts:
            return None

        if parts[0] == "IMU":
            if len(parts) != 7:
                return None
            try:
                vals = [float(x) for x in parts[1:]]
            except ValueError:
                return None
            return ("IMU", *vals)

        if parts[0] == "BARO":
            if len(parts) != 3:
                return None
            try:
                pressure_hpa = float(parts[1])
                temperature_c = float(parts[2])
            except ValueError:
                return None
            return ("BARO", pressure_hpa, temperature_c)

        return None

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
                "t_baro": list(self.t_baro),
                "pressure_hpa": list(self.pressure_hpa),
                "temperature_c": list(self.temperature_c),
                "altitude_ft": list(self.altitude_ft),
                "bad_lines": self.bad_lines,
                "filtered_baro_lines": self.filtered_baro_lines,
                "imu_samples": self.imu_samples,
                "baro_samples": self.baro_samples,
            }


def set_y_limits(axis, series_list):
    vals = [v for series in series_list for v in series if not math.isnan(v)]
    if not vals:
        return
    vmin = min(vals)
    vmax = max(vals)
    if math.isclose(vmin, vmax, rel_tol=1e-6, abs_tol=1e-6):
        pad = 1.0 if abs(vmin) < 1.0 else max(1.0, abs(vmin) * 0.1)
    else:
        pad = (vmax - vmin) * 0.1
    axis.set_ylim(vmin - pad, vmax + pad)


def find_window_start_index(t_values, t_start):
    for i, val in enumerate(t_values):
        if val >= t_start:
            return i
    return 0


def build_plot(reader: IMUSerialReader, window_seconds: float):
    fig, (ax_acc, ax_gyro, ax_alt) = plt.subplots(3, 1, figsize=(11, 9), sharex=False)
    fig.suptitle("Live IMU + Barometer UART Plot")

    acc_x_line, = ax_acc.plot([], [], label="ax (g)")
    acc_y_line, = ax_acc.plot([], [], label="ay (g)")
    acc_z_line, = ax_acc.plot([], [], label="az (g)")

    gyro_x_line, = ax_gyro.plot([], [], label="gx (dps)")
    gyro_y_line, = ax_gyro.plot([], [], label="gy (dps)")
    gyro_z_line, = ax_gyro.plot([], [], label="gz (dps)")

    alt_line, = ax_alt.plot([], [], label="altitude (ft MSL)")
    pressure_text = ax_alt.text(0.01, 0.96, "", transform=ax_alt.transAxes, va="top")

    ax_acc.set_ylabel("Acceleration")
    ax_gyro.set_ylabel("Angular Rate")
    ax_alt.set_ylabel("Altitude")
    ax_alt.set_xlabel("Time (s)")

    ax_acc.legend(loc="upper left")
    ax_gyro.legend(loc="upper left")
    ax_alt.legend(loc="upper left")

    ax_acc.grid(True)
    ax_gyro.grid(True)
    ax_alt.grid(True)

    status_text = fig.text(0.01, 0.01, "", fontsize=9)

    def update(_frame):
        data = reader.snapshot()

        # IMU data
        t = data["t"]
        if t:
            t_end = t[-1]
            t_start = max(0.0, t_end - window_seconds)
            i0 = find_window_start_index(t, t_start)

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
            set_y_limits(ax_acc, [ax_vals, ay_vals, az_vals])
            set_y_limits(ax_gyro, [gx_vals, gy_vals, gz_vals])

        # Baro data
        t_baro = data["t_baro"]
        if t_baro:
            t_end_baro = t_baro[-1]
            t_start_baro = max(0.0, t_end_baro - window_seconds)
            i1 = find_window_start_index(t_baro, t_start_baro)

            ts_baro = t_baro[i1:]
            alt_vals = data["altitude_ft"][i1:]
            pressure_vals = data["pressure_hpa"][i1:]
            temp_vals = data["temperature_c"][i1:]

            alt_line.set_data(ts_baro, alt_vals)
            ax_alt.set_xlim(t_start_baro, max(t_start_baro + 0.1, t_end_baro))
            set_y_limits(ax_alt, [alt_vals])

            if pressure_vals and temp_vals:
                pressure_text.set_text(
                    f"Latest pressure: {pressure_vals[-1]:.2f} hPa    "
                    f"Temp: {temp_vals[-1]:.2f} C"
                )
        else:
            pressure_text.set_text("")

        latest_t = 0.0
        if t and t_baro:
            latest_t = max(t[-1], t_baro[-1])
        elif t:
            latest_t = t[-1]
        elif t_baro:
            latest_t = t_baro[-1]

        status_text.set_text(
            f"IMU samples: {data['imu_samples']}    "
            f"BARO samples: {data['baro_samples']}    "
            f"Filtered BARO: {data['filtered_baro_lines']}    "
            f"Bad lines: {data['bad_lines']}    "
            f"Latest t: {latest_t:.2f}s"
        )

        return (
            acc_x_line, acc_y_line, acc_z_line,
            gyro_x_line, gyro_y_line, gyro_z_line,
            alt_line, pressure_text, status_text,
        )

    anim = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    fig.tight_layout(rect=(0, 0.03, 1, 0.97))
    return fig, anim


def parse_args():
    parser = argparse.ArgumentParser(description="Live IMU + barometer serial plotter")
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM5 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--window", type=float, default=10.0, help="Visible time window in seconds")
    parser.add_argument("--history", type=int, default=5000, help="Maximum samples stored")
    parser.add_argument(
        "--sea-level-hpa",
        type=float,
        default=1013.25,
        help="Reference sea-level pressure in hPa for altitude conversion",
    )
    parser.add_argument(
        "--min-altitude-ft",
        type=float,
        default=4500.0,
        help="Minimum valid altitude in feet above sea level",
    )
    parser.add_argument(
        "--max-altitude-ft",
        type=float,
        default=6000.0,
        help="Maximum valid altitude in feet above sea level",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    reader = IMUSerialReader(
        args.port,
        args.baud,
        maxlen=args.history,
        sea_level_hpa=args.sea_level_hpa,
        min_altitude_ft=args.min_altitude_ft,
        max_altitude_ft=args.max_altitude_ft,
    )
    try:
        reader.start()
        fig, _anim = build_plot(reader, args.window)
        plt.show()
    finally:
        reader.close()


if __name__ == "__main__":
    main()
