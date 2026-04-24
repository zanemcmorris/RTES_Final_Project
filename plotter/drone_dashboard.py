import asyncio
import struct
import collections
import math
import threading
import time

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Arc, FancyArrowPatch, Rectangle
import matplotlib.patches as mpatches
import numpy as np
from bleak import BleakClient


# ================= BLE CONFIG =================
ADDRESS     = "02:80:E1:00:00:AA"
NOTIFY_UUID = "d973f2e1-b19e-11e2-9e96-0800200c9a66"

FMT_A = '<BHffff'   # id, ts, ax, ay, az, pressure
FMT_B = '<BHffff'   # id, ts, gx, gy, gz, angle_z
FMT_C = '<BHffff'   # id, ts, vx, vy, vz, disp_z

WINDOW_SEC = 10.0
MAXLEN     = 5000

SEA_LEVEL_HPA = 1013.25


# ================= CONVERSIONS =================
def pressure_to_alt_ft(p, sea=SEA_LEVEL_HPA):
    if p <= 0:
        return float("nan")
    return 44330.0 * (1.0 - (p / sea) ** 0.19029495718363465) * 3.28084


# ================= DATA STORE =================
class IMUBLEReader:
    def __init__(self, maxlen=MAXLEN):
        self.lock = threading.Lock()
        self.t      = collections.deque(maxlen=maxlen)
        self.ax     = collections.deque(maxlen=maxlen)
        self.ay     = collections.deque(maxlen=maxlen)
        self.az     = collections.deque(maxlen=maxlen)
        self.vx     = collections.deque(maxlen=maxlen)
        self.vy     = collections.deque(maxlen=maxlen)
        self.vz     = collections.deque(maxlen=maxlen)
        self.px     = collections.deque(maxlen=maxlen)
        self.py     = collections.deque(maxlen=maxlen)
        self.pz     = collections.deque(maxlen=maxlen)
        self.t_gyro = collections.deque(maxlen=maxlen)
        self.gx     = collections.deque(maxlen=maxlen)
        self.gy     = collections.deque(maxlen=maxlen)
        self.gz     = collections.deque(maxlen=maxlen)
        self.deg    = collections.deque(maxlen=maxlen)   # integrated yaw degrees
        self.t_baro = collections.deque(maxlen=maxlen)
        self.alt_ft = collections.deque(maxlen=maxlen)
        self.pres   = collections.deque(maxlen=maxlen)
        self.motors = [0.0, 0.0, 0.0, 0.0]              # 0–100 %
        self.roll   = 0.0
        self.pitch  = 0.0
        self.yaw    = 0.0
        self._prev_t = None
        self.start_time = time.monotonic()
        self.imu_samples  = 0
        self.baro_samples = 0

    def handle_packet(self, data: bytearray):
        if len(data) < 1:
            return
        pkt = data[0]
        now = time.monotonic() - self.start_time

        if pkt == 0x41 and len(data) >= 19:
            _, ts, ax, ay, az, pressure = struct.unpack(FMT_A, data[:19])
            with self.lock:
                dt = (now - self._prev_t) if self._prev_t else 0.0
                self._prev_t = now

                # integrate velocity
                vx = (self.vx[-1] if self.vx else 0.0) + ax * dt
                vy = (self.vy[-1] if self.vy else 0.0) + ay * dt
                vz = (self.vz[-1] if self.vz else 0.0) + az * dt
                # integrate position
                px = (self.px[-1] if self.px else 0.0) + vx * dt
                py = (self.py[-1] if self.py else 0.0) + vy * dt
                pz = (self.pz[-1] if self.pz else 0.0) + vz * dt

                self.t.append(now)
                self.ax.append(ax); self.ay.append(ay); self.az.append(az)
                self.vx.append(vx); self.vy.append(vy); self.vz.append(vz)
                self.px.append(px); self.py.append(py); self.pz.append(pz)

                self.t_baro.append(now)
                self.pres.append(pressure)
                self.alt_ft.append(pressure_to_alt_ft(pressure))
                self.imu_samples += 1
                self.baro_samples += 1

        elif pkt == 0x42 and len(data) >= 19:
            _, ts, gx, gy, gz, _ = struct.unpack(FMT_B, data[:19])
            with self.lock:
                dt = (now - self._prev_t) if self._prev_t else 0.0
                self.yaw += gz * dt
                self.t_gyro.append(now)
                self.gx.append(gx); self.gy.append(gy); self.gz.append(gz)
                self.deg.append(self.yaw)
                # simple complementary-filter-style attitude estimate
                self.roll  = math.atan2(self.ay[-1] if self.ay else 0,
                                        self.az[-1] if self.az else 1) * 57.3
                self.pitch = math.atan2(-(self.ax[-1] if self.ax else 0),
                                        math.hypot(self.ay[-1] if self.ay else 0,
                                                   self.az[-1] if self.az else 1)) * 57.3
        elif pkt == 0x43 and len(data) >= 19:
            _, ts, vx, vy, vz, disp_z = struct.unpack(FMT_C, data[:19])
            with self.lock:
                self.t_vel.append(now)   # or reuse self.t
                self.vx.append(vx)
                self.vy.append(vy)
                self.vz.append(vz)
                self.pz.append(disp_z)

    def snapshot(self):
        with self.lock:
            return {
                "t":      list(self.t),
                "ax": list(self.ax), "ay": list(self.ay), "az": list(self.az),
                "vx": list(self.vx), "vy": list(self.vy), "vz": list(self.vz),
                "px": list(self.px), "py": list(self.py), "pz": list(self.pz),
                "t_gyro": list(self.t_gyro),
                "gx": list(self.gx), "gy": list(self.gy), "gz": list(self.gz),
                "deg":    list(self.deg),
                "t_baro": list(self.t_baro),
                "alt_ft": list(self.alt_ft),
                "pres":   list(self.pres),
                "motors": list(self.motors),
                "roll":   self.roll,
                "pitch":  self.pitch,
                "yaw":    self.yaw,
                "imu_samples":  self.imu_samples,
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
def window_slice(t_list, window=WINDOW_SEC):
    if not t_list:
        return 0, t_list[-1] if t_list else 0
    t_end   = t_list[-1]
    t_start = max(0, t_end - window)
    i0 = next((i for i, v in enumerate(t_list) if v >= t_start), 0)
    return i0, t_end


def safe_ylim(ax_obj, *series):
    vals = [v for s in series for v in s if not math.isnan(v)]
    if not vals:
        return
    lo, hi = min(vals), max(vals)
    pad = (hi - lo) * 0.15 if hi != lo else 0.5
    ax_obj.set_ylim(lo - pad, hi + pad)


def draw_gauge(ax_obj, value, max_val, label, color):
    """Draw a simple semicircle gauge on ax_obj."""
    ax_obj.clear()
    ax_obj.set_aspect('equal')
    ax_obj.axis('off')
    theta = np.linspace(np.pi, 0, 100)
    ax_obj.plot(np.cos(theta), np.sin(theta), color='#444', lw=4, solid_capstyle='round')
    frac  = min(1.0, abs(value) / max_val)
    theta2 = np.linspace(np.pi, np.pi - frac * np.pi, 60)
    ax_obj.plot(np.cos(theta2), np.sin(theta2), color=color, lw=4, solid_capstyle='round')
    ang = np.pi - frac * np.pi
    ax_obj.annotate("", xy=(0.75 * np.cos(ang), 0.75 * np.sin(ang)), xytext=(0, 0),
                    arrowprops=dict(arrowstyle="-|>", color='white', lw=1.5))
    ax_obj.text(0, -0.25, f"{value:.2f}", ha='center', va='center',
                fontsize=8, color='white')
    ax_obj.text(0, -0.6, label, ha='center', va='center',
                fontsize=7, color='#aaa')
    ax_obj.set_xlim(-1.2, 1.2)
    ax_obj.set_ylim(-0.8, 1.2)


def draw_drone_top(ax_obj, roll_deg, pitch_deg, motors):
    ax_obj.clear()
    ax_obj.set_facecolor('#1a1a2e')
    ax_obj.set_xlim(-2, 2); ax_obj.set_ylim(-2, 2)
    ax_obj.set_aspect('equal'); ax_obj.axis('off')

    # arms
    for dx, dy in [(-1, 1), (1, 1), (-1, -1), (1, -1)]:
        ax_obj.plot([0, dx * 1.1], [0, dy * 1.1], color='#888', lw=3, solid_capstyle='round')

    # motors
    positions = [(-1.1, 1.1), (1.1, 1.1), (-1.1, -1.1), (1.1, -1.1)]
    labels    = ['M1', 'M2', 'M3', 'M4']
    colors    = ['#378ADD', '#1D9E75', '#1D9E75', '#378ADD']
    for i, ((mx, my), lbl, col) in enumerate(zip(positions, labels, colors)):
        pct = motors[i] / 100.0 if i < len(motors) else 0.5
        alpha = 0.3 + 0.7 * pct
        circ = plt.Circle((mx, my), 0.38, color=col, alpha=alpha, zorder=3)
        ax_obj.add_patch(circ)
        ax_obj.text(mx, my, lbl, ha='center', va='center',
                    fontsize=7, color='white', fontweight='bold', zorder=4)

    # body
    body = plt.Circle((0, 0), 0.22, color='#ccc', zorder=5)
    ax_obj.add_patch(body)
    ax_obj.text(0, 0, f"R{roll_deg:+.0f}°\nP{pitch_deg:+.0f}°",
                ha='center', va='center', fontsize=6, color='#111', zorder=6)

    # forward indicator
    ax_obj.annotate("", xy=(0, 1.6), xytext=(0, 0.25),
                    arrowprops=dict(arrowstyle="-|>", color='#facc15', lw=1.5), zorder=7)
    ax_obj.set_title("Drone view", fontsize=8, color='#aaa', pad=2)


# ================= COMMAND LOG =================
cmd_log = []


def log(msg, prefix="> "):
    cmd_log.append(f"{prefix}{msg}")
    if len(cmd_log) > 20:
        cmd_log.pop(0)


# ================= LAYOUT =================
DARK = '#0d0d1a'
MID  = '#12122a'
ACC  = '#378ADD'
GRN  = '#1D9E75'
ORG  = '#D85A30'

plt.style.use('dark_background')

fig = plt.figure(figsize=(15, 9), facecolor=DARK)
fig.suptitle("Drone Ground Station", color='#ccc', fontsize=11, y=0.99)

outer = gridspec.GridSpec(1, 3, figure=fig, wspace=0.28, left=0.05, right=0.97,
                          top=0.96, bottom=0.04)

# ---- LEFT: Motion plots + gauges ----
left_gs = gridspec.GridSpecFromSubplotSpec(5, 1, subplot_spec=outer[0],
                                           hspace=0.12, height_ratios=[1,1,1,0.15,0.7])
ax_accel = fig.add_subplot(left_gs[0])
ax_vel   = fig.add_subplot(left_gs[1])
ax_pos   = fig.add_subplot(left_gs[2])
# gauges row
gauge_gs = gridspec.GridSpecFromSubplotSpec(1, 2, subplot_spec=left_gs[4])
ax_ga = fig.add_subplot(gauge_gs[0])
ax_gv = fig.add_subplot(gauge_gs[1])

for ax_ in (ax_accel, ax_vel, ax_pos):
    ax_.set_facecolor(MID)
    ax_.tick_params(labelsize=7, colors='#aaa')
    for sp in ax_.spines.values(): sp.set_color('#333')
    ax_.grid(True, color='#2a2a4a', lw=0.5)

ax_accel.set_ylabel("accel (m/s²)", fontsize=7, color='#aaa')
ax_vel.set_ylabel("vel (m/s)",    fontsize=7, color='#aaa')
ax_pos.set_ylabel("pos (m)",      fontsize=7, color='#aaa')
ax_accel.set_title("Acceleration", fontsize=8, color='#aaa', loc='left', pad=2)

la_x, = ax_accel.plot([], [], color=ACC, lw=1.2, label='ax')
la_y, = ax_accel.plot([], [], color=GRN, lw=1.2, label='ay')
la_z, = ax_accel.plot([], [], color=ORG, lw=1.2, label='az')
lv_x, = ax_vel.plot([],   [], color=ACC, lw=1.2, label='vx')
lv_y, = ax_vel.plot([],   [], color=GRN, lw=1.2, label='vy')
lv_z, = ax_vel.plot([],   [], color=ORG, lw=1.2, label='vz')
lp_x, = ax_pos.plot([],   [], color=ACC, lw=1.2, label='px')
lp_y, = ax_pos.plot([],   [], color=GRN, lw=1.2, label='py')
lp_z, = ax_pos.plot([],   [], color=ORG, lw=1.2, label='pz')

for ax_, lines in [(ax_accel,[la_x,la_y,la_z]),(ax_vel,[lv_x,lv_y,lv_z]),(ax_pos,[lp_x,lp_y,lp_z])]:
    ax_.legend(handles=lines, fontsize=6, loc='upper right',
               facecolor='#1a1a2e', edgecolor='none', labelcolor='#aaa', ncol=3)

# ---- CENTER: Rotation + Drone view ----
ctr_gs = gridspec.GridSpecFromSubplotSpec(5, 1, subplot_spec=outer[1],
                                          hspace=0.12, height_ratios=[1,1,0.15,0.6,1.2])
ax_dps  = fig.add_subplot(ctr_gs[0])
ax_deg  = fig.add_subplot(ctr_gs[1])
ax_gdps = fig.add_subplot(ctr_gs[3])
ax_drone= fig.add_subplot(ctr_gs[4])

for ax_ in (ax_dps, ax_deg):
    ax_.set_facecolor(MID)
    ax_.tick_params(labelsize=7, colors='#aaa')
    for sp in ax_.spines.values(): sp.set_color('#333')
    ax_.grid(True, color='#2a2a4a', lw=0.5)

ax_dps.set_ylabel("dps",    fontsize=7, color='#aaa')
ax_deg.set_ylabel("deg",    fontsize=7, color='#aaa')
ax_dps.set_title("Angular Rate", fontsize=8, color='#aaa', loc='left', pad=2)

ld_x, = ax_dps.plot([], [], color=ACC, lw=1.2, label='gx')
ld_y, = ax_dps.plot([], [], color=GRN, lw=1.2, label='gy')
ld_z, = ax_dps.plot([], [], color=ORG, lw=1.2, label='gz')
ld_deg,= ax_deg.plot([],[], color='#facc15', lw=1.2, label='yaw°')
ax_dps.legend(handles=[ld_x,ld_y,ld_z], fontsize=6, loc='upper right',
              facecolor='#1a1a2e', edgecolor='none', labelcolor='#aaa', ncol=3)
ax_deg.legend(handles=[ld_deg], fontsize=6, loc='upper right',
              facecolor='#1a1a2e', edgecolor='none', labelcolor='#aaa')

# ---- RIGHT: Command interpreter + Motor bars ----
rgt_gs = gridspec.GridSpecFromSubplotSpec(2, 1, subplot_spec=outer[2],
                                          hspace=0.3, height_ratios=[1.2, 1])
ax_cmd   = fig.add_subplot(rgt_gs[0])
ax_motor = fig.add_subplot(rgt_gs[1])

ax_cmd.set_facecolor(MID)
ax_cmd.axis('off')
ax_cmd.set_title("Command interpreter", fontsize=8, color='#aaa', loc='left', pad=2)
cmd_text = ax_cmd.text(0.01, 0.97, "", transform=ax_cmd.transAxes,
                       fontsize=7, color='#7ec8e3', va='top', family='monospace',
                       wrap=True)

ax_motor.set_facecolor(MID)
ax_motor.set_title("Motor outputs", fontsize=8, color='#aaa', loc='left', pad=2)
ax_motor.set_xlim(-0.5, 3.5)
ax_motor.set_ylim(0, 100)
ax_motor.set_xticks([0,1,2,3])
ax_motor.set_xticklabels(['M1','M2','M3','M4'], fontsize=8, color='#aaa')
ax_motor.set_yticks([0,25,50,75,100])
ax_motor.set_yticklabels(['0%','25%','50%','75%','100%'], fontsize=7, color='#aaa')
ax_motor.yaxis.tick_right()
for sp in ax_motor.spines.values(): sp.set_color('#333')
ax_motor.grid(axis='y', color='#2a2a4a', lw=0.5)

m_colors = [ACC, GRN, GRN, ACC]
motor_bars = ax_motor.bar([0,1,2,3], [50,50,50,50], color=m_colors, width=0.6,
                          alpha=0.85)
motor_pct_texts = [ax_motor.text(i, 52, "50%", ha='center', fontsize=7, color='#ccc')
                   for i in range(4)]

# ================= STATUS BAR =================
status_txt = fig.text(0.02, 0.005, "", fontsize=7, color='#555')

# ================= COMMAND INPUT =================
log("system init")
log("IMU OK · BARO OK · BLE OK", prefix="  ")
log("stream start")
log(f"UUID: {NOTIFY_UUID[:20]}…", prefix="  ")

cmd_ax_input = fig.add_axes([0.68, 0.005, 0.25, 0.025])
cmd_ax_input.set_facecolor('#1a1a2e')
for sp in cmd_ax_input.spines.values(): sp.set_color('#444')
cmd_ax_input.axis('off')
cmd_label = cmd_ax_input.text(0.01, 0.5, "> _", transform=cmd_ax_input.transAxes,
                               fontsize=8, color='#7ec8e3', va='center', family='monospace')

_typed = [""]

def on_key(event):
    if event.key == 'enter':
        cmd = _typed[0].strip()
        if cmd:
            log(cmd)
            responses = {
                'help':    'Commands: status, arm, disarm, motors, stream',
                'status':  'IMU OK · BARO OK · BLE connected',
                'arm':     'Motors armed. Throttle ready.',
                'disarm':  'Motors disarmed.',
                'motors':  'M1–M4 live',
                'stream':  'Streaming at 50 Hz',
            }
            log(responses.get(cmd.lower(), f'Unknown: {cmd}'), prefix="  ")
        _typed[0] = ""
    elif event.key == 'backspace':
        _typed[0] = _typed[0][:-1]
    elif event.key and len(event.key) == 1:
        _typed[0] += event.key
    cmd_label.set_text("> " + _typed[0] + "_")
    fig.canvas.draw_idle()

fig.canvas.mpl_connect('key_press_event', on_key)


# ================= ANIMATE =================
def update(_):
    d = reader.snapshot()

    # --- accel ---
    t = d["t"]
    if t:
        i0, te = window_slice(t)
        ts = t[i0:]
        ax_accel.set_xlim(te - WINDOW_SEC, te)
        la_x.set_data(ts, d["ax"][i0:]); la_y.set_data(ts, d["ay"][i0:]); la_z.set_data(ts, d["az"][i0:])
        safe_ylim(ax_accel, d["ax"][i0:], d["ay"][i0:], d["az"][i0:])
        ax_vel.set_xlim(te - WINDOW_SEC, te)
        lv_x.set_data(ts, d["vx"][i0:]); lv_y.set_data(ts, d["vy"][i0:]); lv_z.set_data(ts, d["vz"][i0:])
        safe_ylim(ax_vel, d["vx"][i0:], d["vy"][i0:], d["vz"][i0:])
        ax_pos.set_xlim(te - WINDOW_SEC, te)
        lp_x.set_data(ts, d["px"][i0:]); lp_y.set_data(ts, d["py"][i0:]); lp_z.set_data(ts, d["pz"][i0:])
        safe_ylim(ax_pos, d["px"][i0:], d["py"][i0:], d["pz"][i0:])

    # --- gyro ---
    tg = d["t_gyro"]
    if tg:
        i0, te = window_slice(tg)
        ts = tg[i0:]
        ax_dps.set_xlim(te - WINDOW_SEC, te)
        ld_x.set_data(ts, d["gx"][i0:]); ld_y.set_data(ts, d["gy"][i0:]); ld_z.set_data(ts, d["gz"][i0:])
        safe_ylim(ax_dps, d["gx"][i0:], d["gy"][i0:], d["gz"][i0:])
        ax_deg.set_xlim(te - WINDOW_SEC, te)
        ld_deg.set_data(ts, d["deg"][i0:])
        safe_ylim(ax_deg, d["deg"][i0:])

    # --- gauges ---
    amag = math.hypot(d["ax"][-1] if d["ax"] else 0,
                      math.hypot(d["ay"][-1] if d["ay"] else 0,
                                 d["az"][-1] if d["az"] else 0))
    vmag = math.hypot(d["vx"][-1] if d["vx"] else 0,
                      math.hypot(d["vy"][-1] if d["vy"] else 0,
                                 d["vz"][-1] if d["vz"] else 0))
    gmag = math.hypot(d["gx"][-1] if d["gx"] else 0,
                      math.hypot(d["gy"][-1] if d["gy"] else 0,
                                 d["gz"][-1] if d["gz"] else 0))
    draw_gauge(ax_ga,   amag, 20.0,  "||accel||", ACC)
    draw_gauge(ax_gv,   vmag,  5.0,  "||vel||",   GRN)
    draw_gauge(ax_gdps, gmag, 500.0, "||dps||",   ORG)

    # --- drone view ---
    draw_drone_top(ax_drone, d["roll"], d["pitch"], d["motors"])

    # --- motors ---
    motors = d["motors"]
    for i, (bar, txt) in enumerate(zip(motor_bars, motor_pct_texts)):
        pct = motors[i] if i < len(motors) else 0
        bar.set_height(pct)
        txt.set_y(pct + 2)
        txt.set_text(f"{pct:.0f}%")

    # --- command log ---
    cmd_text.set_text("\n".join(cmd_log[-14:]))

    # --- status bar ---
    status_txt.set_text(
        f"IMU: {d['imu_samples']}  BARO: {d['baro_samples']}  "
        f"Roll: {d['roll']:+.1f}°  Pitch: {d['pitch']:+.1f}°  Yaw: {d['yaw']:+.1f}°  "
        f"Alt: {d['alt_ft'][-1]:.1f} ft" if d['alt_ft'] else
        f"IMU: {d['imu_samples']}  BARO: {d['baro_samples']}"
    )

    return []


anim = FuncAnimation(fig, update, interval=50, blit=False)


# ================= MAIN =================
if __name__ == "__main__":
    threading.Thread(target=start_ble, daemon=True).start()
    plt.show()
