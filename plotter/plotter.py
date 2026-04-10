import argparse
import collections
import math
import sys
import threading
import time

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import serial


G_TO_MPS2 = 9.80665
GYRO_SUM_SCALE = 1000.0


def pressure_hpa_to_altitude_feet(pressure_hpa: float, sea_level_hpa: float = 1013.25) -> float:
    if pressure_hpa <= 0.0 or sea_level_hpa <= 0.0:
        return float("nan")
    altitude_m = 44330.0 * (1.0 - (pressure_hpa / sea_level_hpa) ** 0.19029495718363465)
    return altitude_m * 3.280839895013123


class IMUSerialReader:
    """Read raw IMU, integrated values, displacement, and barometer lines.

    Supported line formats:
        IMU,ax,ay,az,gx,gy,gz
        INT,avx,avy,avz,gax,gay,gaz
        DISP,px,py,pz
        BARO,pressure_hpa,temp_c
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

        self.t_int = collections.deque(maxlen=maxlen)
        self.avx = collections.deque(maxlen=maxlen)
        self.avy = collections.deque(maxlen=maxlen)
        self.avz = collections.deque(maxlen=maxlen)
        self.gax = collections.deque(maxlen=maxlen)
        self.gay = collections.deque(maxlen=maxlen)
        self.gaz = collections.deque(maxlen=maxlen)

        self.t_disp = collections.deque(maxlen=maxlen)
        self.px = collections.deque(maxlen=maxlen)
        self.py = collections.deque(maxlen=maxlen)
        self.pz = collections.deque(maxlen=maxlen)

        self.t_baro = collections.deque(maxlen=maxlen)
        self.pressure_hpa = collections.deque(maxlen=maxlen)
        self.temperature_c = collections.deque(maxlen=maxlen)
        self.altitude_ft = collections.deque(maxlen=maxlen)

        self.start_time = time.monotonic()
        self.bad_lines = 0
        self.filtered_baro_lines = 0
        self.imu_samples = 0
        self.int_samples = 0
        self.disp_samples = 0
        self.baro_samples = 0

    def open(self) -> None:
        self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
        time.sleep(2.0)

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
                        self.ax.append(ax * G_TO_MPS2)
                        self.ay.append(ay * G_TO_MPS2)
                        self.az.append(az * G_TO_MPS2)
                        self.gx.append(gx)
                        self.gy.append(gy)
                        self.gz.append(gz)
                        self.imu_samples += 1
                    elif kind == "INT":
                        _, avx, avy, avz, gax, gay, gaz = parsed
                        self.t_int.append(now)
                        self.avx.append(avx)
                        self.avy.append(avy)
                        self.avz.append(avz)
                        self.gax.append(gax / GYRO_SUM_SCALE)
                        self.gay.append(gay / GYRO_SUM_SCALE)
                        self.gaz.append(gaz / GYRO_SUM_SCALE)
                        self.int_samples += 1
                    elif kind == "DISP":
                        _, px, py, pz = parsed
                        self.t_disp.append(now)
                        self.px.append(px)
                        self.py.append(py)
                        self.pz.append(pz)
                        self.disp_samples += 1
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

        if parts[0] in ("IMU", "INT"):
            if len(parts) != 7:
                return None
            try:
                vals = [float(x) for x in parts[1:]]
            except ValueError:
                return None
            return (parts[0], *vals)

        if parts[0] == "DISP":
            if len(parts) != 4:
                return None
            try:
                vals = [float(x) for x in parts[1:]]
            except ValueError:
                return None
            return ("DISP", *vals)

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
                "t_int": list(self.t_int),
                "avx": list(self.avx),
                "avy": list(self.avy),
                "avz": list(self.avz),
                "gax": list(self.gax),
                "gay": list(self.gay),
                "gaz": list(self.gaz),
                "t_disp": list(self.t_disp),
                "px": list(self.px),
                "py": list(self.py),
                "pz": list(self.pz),
                "t_baro": list(self.t_baro),
                "pressure_hpa": list(self.pressure_hpa),
                "temperature_c": list(self.temperature_c),
                "altitude_ft": list(self.altitude_ft),
                "bad_lines": self.bad_lines,
                "filtered_baro_lines": self.filtered_baro_lines,
                "imu_samples": self.imu_samples,
                "int_samples": self.int_samples,
                "disp_samples": self.disp_samples,
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
    fig = plt.figure(figsize=(14, 12))
    gs = fig.add_gridspec(4, 2, width_ratios=[1, 1])

    ax_acc = fig.add_subplot(gs[0, 0])
    ax_vel = fig.add_subplot(gs[1, 0])
    ax_pos = fig.add_subplot(gs[2, 0])
    ax_baro = fig.add_subplot(gs[3, 0])

    ax_gyro = fig.add_subplot(gs[0, 1])
    ax_gsum = fig.add_subplot(gs[1, 1])
    ax_blank_top = fig.add_subplot(gs[2, 1])
    ax_blank_bottom = fig.add_subplot(gs[3, 1])
    ax_blank_top.axis("off")
    ax_blank_bottom.axis("off")

    fig.suptitle("Live IMU + Linear Motion + Summed Gyro + Barometer UART Plot")

    acc_x_line, = ax_acc.plot([], [], label="ax (m/s²)")
    acc_y_line, = ax_acc.plot([], [], label="ay (m/s²)")
    acc_z_line, = ax_acc.plot([], [], label="az (m/s²)")

    vel_x_line, = ax_vel.plot([], [], label="vx")
    vel_y_line, = ax_vel.plot([], [], label="vy")
    vel_z_line, = ax_vel.plot([], [], label="vz")

    pos_x_line, = ax_pos.plot([], [], label="px")
    pos_y_line, = ax_pos.plot([], [], label="py")
    pos_z_line, = ax_pos.plot([], [], label="pz")

    gyro_x_line, = ax_gyro.plot([], [], label="roll rate")
    gyro_y_line, = ax_gyro.plot([], [], label="pitch rate")
    gyro_z_line, = ax_gyro.plot([], [], label="yaw rate")

    gsum_x_line, = ax_gsum.plot([], [], label="roll (deg)")
    gsum_y_line, = ax_gsum.plot([], [], label="pitch (deg)")
    gsum_z_line, = ax_gsum.plot([], [], label="yaw (deg)")

    alt_line, = ax_baro.plot([], [], label="altitude (ft MSL)")
    pressure_text = ax_baro.text(0.01, 0.96, "", transform=ax_baro.transAxes, va="top")

    ax_acc.set_ylabel("Acceleration (m/s²)")
    ax_vel.set_ylabel("Velocity")
    ax_pos.set_ylabel("Displacement")
    ax_baro.set_ylabel("Altitude")
    ax_baro.set_xlabel("Time (s)")

    ax_gyro.set_ylabel("Gyro Rate (roll/pitch/yaw rates)")
    ax_gsum.set_ylabel("Summed Gyro Angle (roll/pitch/yaw)")

    for axis in (ax_acc, ax_vel, ax_pos, ax_baro, ax_gyro, ax_gsum):
        axis.grid(True)
        axis.legend(loc="upper left")

    status_text = fig.text(0.01, 0.01, "", fontsize=9)

    def update(_frame):
        data = reader.snapshot()

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

        t_int = data["t_int"]
        if t_int:
            t_end_int = t_int[-1]
            t_start_int = max(0.0, t_end_int - window_seconds)
            i_int = find_window_start_index(t_int, t_start_int)

            ts_int = t_int[i_int:]
            vx_vals = data["avx"][i_int:]
            vy_vals = data["avy"][i_int:]
            vz_vals = data["avz"][i_int:]
            gax_vals = data["gax"][i_int:]
            gay_vals = data["gay"][i_int:]
            gaz_vals = data["gaz"][i_int:]

            vel_x_line.set_data(ts_int, vx_vals)
            vel_y_line.set_data(ts_int, vy_vals)
            vel_z_line.set_data(ts_int, vz_vals)

            gsum_x_line.set_data(ts_int, gax_vals)
            gsum_y_line.set_data(ts_int, gay_vals)
            gsum_z_line.set_data(ts_int, gaz_vals)

            ax_vel.set_xlim(t_start_int, max(t_start_int + 0.1, t_end_int))
            ax_gsum.set_xlim(t_start_int, max(t_start_int + 0.1, t_end_int))

            set_y_limits(ax_vel, [vx_vals, vy_vals, vz_vals])
            set_y_limits(ax_gsum, [gax_vals, gay_vals, gaz_vals])

        t_disp = data["t_disp"]
        if t_disp:
            t_end_disp = t_disp[-1]
            t_start_disp = max(0.0, t_end_disp - window_seconds)
            i_disp = find_window_start_index(t_disp, t_start_disp)

            ts_disp = t_disp[i_disp:]
            px_vals = data["px"][i_disp:]
            py_vals = data["py"][i_disp:]
            pz_vals = data["pz"][i_disp:]

            pos_x_line.set_data(ts_disp, px_vals)
            pos_y_line.set_data(ts_disp, py_vals)
            pos_z_line.set_data(ts_disp, pz_vals)

            ax_pos.set_xlim(t_start_disp, max(t_start_disp + 0.1, t_end_disp))
            set_y_limits(ax_pos, [px_vals, py_vals, pz_vals])

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
            ax_baro.set_xlim(t_start_baro, max(t_start_baro + 0.1, t_end_baro))
            set_y_limits(ax_baro, [alt_vals])

            if pressure_vals and temp_vals:
                pressure_text.set_text(
                    f"Latest pressure: {pressure_vals[-1]:.2f} hPa    "
                    f"Temp: {temp_vals[-1]:.2f} C"
                )
        else:
            pressure_text.set_text("")

        latest_t = 0.0
        for arr in (t, t_int, t_disp, t_baro):
            if arr:
                latest_t = max(latest_t, arr[-1])

        status_text.set_text(
            f"IMU samples: {data['imu_samples']}    "
            f"INT samples: {data['int_samples']}    "
            f"DISP samples: {data['disp_samples']}    "
            f"BARO samples: {data['baro_samples']}    "
            f"Filtered BARO: {data['filtered_baro_lines']}    "
            f"Bad lines: {data['bad_lines']}    "
            f"Latest t: {latest_t:.2f}s"
        )

        return (
            acc_x_line, acc_y_line, acc_z_line,
            vel_x_line, vel_y_line, vel_z_line,
            pos_x_line, pos_y_line, pos_z_line,
            gyro_x_line, gyro_y_line, gyro_z_line,
            gsum_x_line, gsum_y_line, gsum_z_line,
            alt_line, pressure_text, status_text,
        )

    anim = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    fig.tight_layout(rect=(0, 0.03, 1, 0.97))
    return fig, anim


def parse_args():
    parser = argparse.ArgumentParser(description="Live IMU + linear motion + summed gyro + barometer serial plotter")
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
