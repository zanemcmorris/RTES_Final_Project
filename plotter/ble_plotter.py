import asyncio
import struct
import collections
import math
import threading
import time

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from bleak import BleakClient


# ================= BLE CONFIG =================
ADDRESS     = "02:80:E1:00:00:AA"
NOTIFY_UUID = "d973f2e1-b19e-11e2-9e96-0800200c9a66"

FMT_A = '<BHffff'   # accel + pressure
FMT_B = '<BHffff'   # gyro


# ================= CONVERSIONS =================
def pressure_hpa_to_altitude_feet(pressure_hpa, sea_level_hpa=1013.25):
    if pressure_hpa <= 0.0:
        return float("nan")
    altitude_m = 44330.0 * (1.0 - (pressure_hpa / sea_level_hpa) ** 0.19029495718363465)
    return altitude_m * 3.28084


# ================= DATA STORE =================
class IMUBLEReader:
    def __init__(self, maxlen=5000, sea_level_hpa=1013.25):
        self.lock = threading.Lock()

        # accel
        self.t = collections.deque(maxlen=maxlen)
        self.ax = collections.deque(maxlen=maxlen)
        self.ay = collections.deque(maxlen=maxlen)
        self.az = collections.deque(maxlen=maxlen)

        # gyro (FIXED: separate timestamps)
        self.t_gyro = collections.deque(maxlen=maxlen)
        self.gx = collections.deque(maxlen=maxlen)
        self.gy = collections.deque(maxlen=maxlen)
        self.gz = collections.deque(maxlen=maxlen)

        # baro
        self.t_baro = collections.deque(maxlen=maxlen)
        self.pressure_hpa = collections.deque(maxlen=maxlen)
        self.temperature_c = collections.deque(maxlen=maxlen)
        self.altitude_ft = collections.deque(maxlen=maxlen)

        self.start_time = time.monotonic()

        self.imu_samples = 0
        self.baro_samples = 0
        self.sea_level_hpa = sea_level_hpa

    def handle_packet(self, data: bytearray):
        if len(data) < 1:
            return

        pkt_type = data[0]
        now = time.monotonic() - self.start_time

        # ===== PACKET A (ACCEL + BARO) =====
        if pkt_type == 0x41 and len(data) >= 19:
            _, ts, ax, ay, az, pressure = struct.unpack(FMT_A, data[:19])

            with self.lock:
                self.t.append(now)
                self.ax.append(ax)
                self.ay.append(ay)
                self.az.append(az)

                altitude = pressure_hpa_to_altitude_feet(pressure, self.sea_level_hpa)

                self.t_baro.append(now)
                self.pressure_hpa.append(pressure)
                self.temperature_c.append(0.0)
                self.altitude_ft.append(altitude)

                self.imu_samples += 1
                self.baro_samples += 1   # FIXED

        # ===== PACKET B (GYRO) =====
        elif pkt_type == 0x42 and len(data) >= 19:
            _, ts, gx, gy, gz, _ = struct.unpack(FMT_B, data[:19])

            with self.lock:
                self.t_gyro.append(now)
                self.gx.append(gx)
                self.gy.append(gy)
                self.gz.append(gz)

    def snapshot(self):
        with self.lock:
            return {
                "t": list(self.t),
                "ax": list(self.ax),
                "ay": list(self.ay),
                "az": list(self.az),

                "t_gyro": list(self.t_gyro),
                "gx": list(self.gx),
                "gy": list(self.gy),
                "gz": list(self.gz),

                "t_baro": list(self.t_baro),
                "pressure_hpa": list(self.pressure_hpa),
                "temperature_c": list(self.temperature_c),
                "altitude_ft": list(self.altitude_ft),

                "imu_samples": self.imu_samples,
                "baro_samples": self.baro_samples,
            }


reader = IMUBLEReader()


# ================= BLE =================
def notification_handler(sender, data):
    reader.handle_packet(data)


async def ble_loop():
    try:
        async with BleakClient(ADDRESS) as client:
            print("Connected to BLE")
            await client.start_notify(NOTIFY_UUID, notification_handler)
            print("Streaming started")

            while True:
                await asyncio.sleep(1)

    except Exception as e:
        print("BLE ERROR:", e)


def start_ble():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(ble_loop())


# ================= HELPERS =================
def set_y_limits(axis, series_list):
    vals = [v for s in series_list for v in s if not math.isnan(v)]
    if not vals:
        return
    vmin, vmax = min(vals), max(vals)
    pad = (vmax - vmin) * 0.1 if vmax != vmin else 0.1
    axis.set_ylim(vmin - pad, vmax + pad)


def find_window_start_index(t_values, t_start):
    for i, v in enumerate(t_values):
        if v >= t_start:
            return i
    return 0


# ================= PLOT =================
def build_plot(window_seconds=10.0):
    fig, (ax_acc, ax_gyro, ax_alt) = plt.subplots(3, 1, figsize=(11, 9))
    fig.suptitle("Live IMU + Barometer BLE Plot")

    # accel
    acc_x_line, = ax_acc.plot([], [], label="ax")
    acc_y_line, = ax_acc.plot([], [], label="ay")
    acc_z_line, = ax_acc.plot([], [], label="az")

    # gyro
    gyro_x_line, = ax_gyro.plot([], [], label="gx")
    gyro_y_line, = ax_gyro.plot([], [], label="gy")
    gyro_z_line, = ax_gyro.plot([], [], label="gz")

    # baro
    alt_line, = ax_alt.plot([], [], label="altitude")
    pressure_text = ax_alt.text(0.01, 0.95, "", transform=ax_alt.transAxes)

    for ax in (ax_acc, ax_gyro, ax_alt):
        ax.legend()
        ax.grid(True)

    status_text = fig.text(0.01, 0.01, "", fontsize=9)

    def update(_):
        data = reader.snapshot()

        # ================= ACCEL =================
        t = data["t"]
        if t:
            t_end = t[-1]
            t_start = max(0, t_end - window_seconds)
            i0 = find_window_start_index(t, t_start)
            ts = t[i0:]

            ax_acc.set_xlim(t_start, t_end)

            acc_x_line.set_data(ts, data["ax"][i0:])
            acc_y_line.set_data(ts, data["ay"][i0:])
            acc_z_line.set_data(ts, data["az"][i0:])

            set_y_limits(ax_acc, [
                data["ax"][i0:],
                data["ay"][i0:],
                data["az"][i0:]
            ])

        # ================= GYRO (FIXED) =================
        tg = data["t_gyro"]
        if tg:
            t_end = tg[-1]
            t_start = max(0, t_end - window_seconds)
            i0 = find_window_start_index(tg, t_start)
            ts = tg[i0:]

            ax_gyro.set_xlim(t_start, t_end)

            gx = data["gx"][i0:]
            gy = data["gy"][i0:]
            gz = data["gz"][i0:]

            gyro_x_line.set_data(ts, gx)
            gyro_y_line.set_data(ts, gy)
            gyro_z_line.set_data(ts, gz)

            set_y_limits(ax_gyro, [gx, gy, gz])

        # ================= BARO =================
        tb = data["t_baro"]
        if tb:
            t_end = tb[-1]
            t_start = max(0, t_end - window_seconds)
            i0 = find_window_start_index(tb, t_start)
            tsb = tb[i0:]

            alt = data["altitude_ft"][i0:]
            pres = data["pressure_hpa"][i0:]

            ax_alt.set_xlim(t_start, t_end)
            alt_line.set_data(tsb, alt)

            set_y_limits(ax_alt, [alt])

            if pres:
                pressure_text.set_text(f"P: {pres[-1]:.2f} hPa")

        status_text.set_text(
            f"IMU: {data['imu_samples']}  BARO: {data['baro_samples']}"
        )

        return (
            acc_x_line, acc_y_line, acc_z_line,
            gyro_x_line, gyro_y_line, gyro_z_line,
            alt_line, pressure_text, status_text
        )

    anim = FuncAnimation(fig, update, interval=50, blit=False)
    return fig, anim


# ================= MAIN =================
if __name__ == "__main__":
    threading.Thread(target=start_ble, daemon=True).start()
    fig, anim = build_plot()
    plt.show()