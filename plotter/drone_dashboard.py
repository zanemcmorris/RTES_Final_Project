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
import numpy as np
from bleak import BleakClient


# ================= BLE CONFIG =================
ADDRESS     = "02:80:E1:00:00:AA"
NOTIFY_UUID = "d973f2e1-b19e-11e2-9e96-0800200c9a66"

# ── Write characteristic for sending commands TO the drone ────────────────────
# Replace with your actual write UUID from your FC firmware
WRITE_UUID  = "d973f2e2-b19e-11e2-9e96-0800200c9a66"

# All packets: id(1B) + ts(2B) + f0(4B) + f1(4B) + f2(4B) + f3(4B) = 19 bytes
FMT = '<BHffff'

WINDOW_SEC    = 10.0
MAXLEN        = 5000
SEA_LEVEL_HPA = 1013.25

# ── Motor scaling ──────────────────────────────────────────────────────────────
# FIX: Auto-detect or set explicitly.
# Set True  if FC sends 0.0 – 1.0   (will be multiplied ×100 for display)
# Set False if FC sends 0.0 – 100.0 (used as-is)
MOTOR_IS_NORMALIZED = True   # <── change to False if you see 100x values


# ================= FCU COMMAND PROTOCOL =================
# Outgoing packet format to FC: cmd_id(1B) + param(4B float) = 5 bytes
# Adapt these IDs and encoding to match your FC firmware
FCU_CMD = {
    'START':      (0x01, 0.0),   # Arm + spin up
    'STOP':       (0x02, 0.0),   # Disarm immediately
    'HOVER':      (0x03, 0.0),   # Hold current altitude
    'LAND':       (0x04, 0.0),   # Gentle descent and disarm
    'TAKEOFF':    (0x05, 1.0),   # Take off to 1.0 m
    'SET_ALT':    (0x06, None),  # param = target metres, supply as arg
    'YAW_LEFT':   (0x10, 30.0), # 30 deg/s
    'YAW_RIGHT':  (0x11, 30.0),
    'MOVE_FWD':   (0x20, 0.5),  # 0.5 m/s
    'MOVE_BACK':  (0x21, 0.5),
    'MOVE_LEFT':  (0x22, 0.5),
    'MOVE_RIGHT': (0x23, 0.5),
    'EMERGENCY':  (0xFF, 0.0),  # Kill all motors NOW
}

def build_fcu_packet(cmd_id: int, param: float) -> bytes:
    """Pack a 5-byte command: cmd_id(u8) + param(f32le)."""
    return struct.pack('<Bf', cmd_id, param)


# ================= CONVERSIONS =================
def pressure_to_alt_m(p, sea=SEA_LEVEL_HPA):
    if p <= 0:
        return float("nan")
    return 44330.0 * (1.0 - (p / sea) ** 0.19029495718363465)


def rad_to_deg(r):
    return r * 57.29577951308232


def clamp_motor(raw_pct: float) -> float:
     return max(0.0, min(100.0, raw_pct))


# ================= DATA STORE =================
class IMUBLEReader:
    def __init__(self, maxlen=MAXLEN):
        self.lock = threading.Lock()

        # BLE client reference — needed to send write commands
        self._ble_client = None

        # ── Packet A  0x41 : accel + baro ─────────────────────────────────────
        self.t      = collections.deque(maxlen=maxlen)
        self.ax     = collections.deque(maxlen=maxlen)
        self.ay     = collections.deque(maxlen=maxlen)
        self.az     = collections.deque(maxlen=maxlen)
        self.t_baro = collections.deque(maxlen=maxlen)
        self.pres   = collections.deque(maxlen=maxlen)
        self.alt_m  = collections.deque(maxlen=maxlen)

        # ── Packet B  0x42 : gyro + integrated yaw ────────────────────────────
        self.t_gyro  = collections.deque(maxlen=maxlen)
        self.gx      = collections.deque(maxlen=maxlen)
        self.gy      = collections.deque(maxlen=maxlen)
        self.gz      = collections.deque(maxlen=maxlen)
        self.yaw_deg = collections.deque(maxlen=maxlen)

        # ── Packet C  0x43 : FC-processed velocity + pos_z ───────────────────
        self.t_vel = collections.deque(maxlen=maxlen)
        self.vx    = collections.deque(maxlen=maxlen)
        self.vy    = collections.deque(maxlen=maxlen)
        self.vz    = collections.deque(maxlen=maxlen)
        self.pos_z = collections.deque(maxlen=maxlen)

        # ── Packet D  0x44 : motor setpoints ─────────────────────────────────
        # FIX: use a dedicated timestamp so we can detect stale data
        self.t_motor = collections.deque(maxlen=maxlen)
        self.motors  = [0.0, 0.0, 0.0, 0.0]   # [FL, FR, BL, BR]  0–100 %
        self._last_motor_update = 0.0

        # ── Attitude scalars ──────────────────────────────────────────────────
        self.roll  = 0.0
        self.pitch = 0.0
        self.yaw   = 0.0

        # ── Attitude histories (for dedicated RPY graph) ──────────────────────
        self.t_att   = collections.deque(maxlen=maxlen)
        self.roll_h  = collections.deque(maxlen=maxlen)
        self.pitch_h = collections.deque(maxlen=maxlen)
        self.yaw_h   = collections.deque(maxlen=maxlen)

        self.start_time    = time.monotonic()
        self.imu_samples   = 0
        self.baro_samples  = 0
        self.ble_connected = False

        # ── BLE event loop reference (set by ble_loop) ────────────────────────
        self._ble_loop = None

    # ──────────────────────────────────────────────────────────────────────────
    def handle_packet(self, data: bytearray):
        if len(data) < 19:
            return
        pkt = data[0]
        now = time.monotonic() - self.start_time

        if pkt == 0x41:
            _, ts, ax, ay, az, pressure = struct.unpack(FMT, data[:19])
            with self.lock:
                self.t.append(now)
                self.ax.append(ax)
                self.ay.append(ay)
                self.az.append(az)

                self.t_baro.append(now)
                self.pres.append(pressure)
                self.alt_m.append(pressure_to_alt_m(pressure))
                self.imu_samples  += 1
                self.baro_samples += 1

        elif pkt == 0x42:
            _, ts, gx, gy, gz, yaw_rad = struct.unpack(FMT, data[:19])
            with self.lock:
                self.t_gyro.append(now)
                self.gx.append(gx)
                self.gy.append(gy)
                self.gz.append(gz)
                yaw_d = rad_to_deg(yaw_rad)
                self.yaw_deg.append(yaw_d)
                self.yaw = yaw_d

        elif pkt == 0x43:
            _, ts, vx, vy, vz, pos_z = struct.unpack(FMT, data[:19])
            with self.lock:
                self.t_vel.append(now)
                self.vx.append(vx)
                self.vy.append(vy)
                self.vz.append(vz)
                self.pos_z.append(pos_z)

        elif pkt == 0x44:
            # FIX: was id, ts, fr, fl, br, bl — now also record update time
            _, ts, fr, fl, br, bl = struct.unpack(FMT, data[:19])
            with self.lock:
                scale = 100.0 if MOTOR_IS_NORMALIZED else 1.0
                self.motors = [round(clamp_motor(v * scale), 1) 
                            for v in [fl, fr, bl, br]]
                self._last_motor_update = now
                
        elif pkt == 0x45:
            _, ts, roll_rad, pitch_rad, yaw_rad, _ = struct.unpack(FMT, data[:19])
            with self.lock:
                self.roll  = math.degrees(roll_rad)
                self.pitch = math.degrees(pitch_rad)
                self.yaw   = math.degrees(yaw_rad)
                self.t_att.append(now)
                self.roll_h.append(self.roll)
                self.pitch_h.append(self.pitch)
                self.yaw_h.append(self.yaw)
    # ──────────────────────────────────────────────────────────────────────────
    def send_command(self, cmd_name: str, param: float = None):
        """Send a command to the FC over BLE write characteristic."""
        entry = FCU_CMD.get(cmd_name.upper())
        if entry is None:
            return False, f"unknown FC command: {cmd_name}"
        cmd_id, default_param = entry
        p = param if param is not None else (default_param if default_param is not None else 0.0)
        packet = build_fcu_packet(cmd_id, p)

        client = self._ble_client
        loop   = self._ble_loop
        if client is None or not self.ble_connected:
            return False, "BLE not connected"
        if loop is None:
            return False, "BLE loop not ready"

        async def _write():
            try:
                await client.write_gatt_char(WRITE_UUID, packet)
            except Exception as e:
                log(f"Write error: {e}", prefix="! ")

        asyncio.run_coroutine_threadsafe(_write(), loop)
        return True, f"→ FC  {cmd_name}  param={p:.2f}  bytes={packet.hex()}"

    # ──────────────────────────────────────────────────────────────────────────
    def snapshot(self):
        with self.lock:
            now = time.monotonic() - self.start_time
            motor_stale = (now - self._last_motor_update) > 1.5   # >1.5s = stale
            return {
                "t":    list(self.t),
                "ax":   list(self.ax),
                "ay":   list(self.ay),
                "az":   list(self.az),
                "t_baro": list(self.t_baro),
                "pres":   list(self.pres),
                "alt_m":  list(self.alt_m),
                "t_gyro":  list(self.t_gyro),
                "gx":      list(self.gx),
                "gy":      list(self.gy),
                "gz":      list(self.gz),
                "yaw_deg": list(self.yaw_deg),
                "t_vel":  list(self.t_vel),
                "vx":     list(self.vx),
                "vy":     list(self.vy),
                "vz":     list(self.vz),
                "pos_z":  list(self.pos_z),
                # Attitude histories
                "t_att":   list(self.t_att),
                "roll_h":  list(self.roll_h),
                "pitch_h": list(self.pitch_h),
                "yaw_h":   list(self.yaw_h),
                # Motors
                "motors":       list(self.motors),
                "motor_stale":  motor_stale,   # FIX: expose staleness flag
                # Scalars
                "roll":          self.roll,
                "pitch":         self.pitch,
                "yaw":           self.yaw,
                "imu_samples":   self.imu_samples,
                "baro_samples":  self.baro_samples,
                "ble_connected": self.ble_connected,
            }


reader = IMUBLEReader()


# ================= BLE (auto-reconnect) =================
def notification_handler(sender, data):
    reader.handle_packet(bytearray(data))


async def ble_loop():
    reader._ble_loop = asyncio.get_event_loop()
    while True:
        try:
            log("BLE connecting…", prefix="  ")
            async with BleakClient(ADDRESS, timeout=5.0) as client:
                reader._ble_client = client
                with reader.lock:
                    reader.ble_connected = True
                log(f"Connected  {ADDRESS}", prefix="  ")
                await client.start_notify(NOTIFY_UUID, notification_handler)
                log("Streaming …", prefix="  ")
                while client.is_connected:
                    await asyncio.sleep(0.5)
                log("BLE connection lost", prefix="! ")
        except Exception as e:
            log(f"BLE error: {e}", prefix="! ")
        finally:
            reader._ble_client = None
            with reader.lock:
                reader.ble_connected = False
        await asyncio.sleep(1.0)


def start_ble():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(ble_loop())


# ================= HELPERS =================
def window_slice(t_list, window=WINDOW_SEC):
    if not t_list:
        return 0, 0.0
    t_end   = t_list[-1]
    t_start = max(0.0, t_end - window)
    i0 = next((i for i, v in enumerate(t_list) if v >= t_start), 0)
    return i0, t_end


def safe_ylim(ax_obj, *series):
    vals = [v for s in series for v in s if math.isfinite(v)]
    if not vals:
        return
    lo, hi = min(vals), max(vals)
    pad = (hi - lo) * 0.15 if hi != lo else 0.5
    ax_obj.set_ylim(lo - pad, hi + pad)


# ================= DRAW HELPERS =================
def draw_gauge(ax_obj, value, max_val, label, color):
    ax_obj.clear()
    ax_obj.set_aspect('equal')
    ax_obj.axis('off')
    theta = np.linspace(np.pi, 0, 100)
    ax_obj.plot(np.cos(theta), np.sin(theta), color='#444', lw=4, solid_capstyle='round')
    frac   = min(1.0, abs(value) / max_val) if max_val else 0.0
    theta2 = np.linspace(np.pi, np.pi - frac * np.pi, 60)
    ax_obj.plot(np.cos(theta2), np.sin(theta2), color=color, lw=4, solid_capstyle='round')
    ang = np.pi - frac * np.pi
    ax_obj.annotate("", xy=(0.75 * np.cos(ang), 0.75 * np.sin(ang)), xytext=(0, 0),
                    arrowprops=dict(arrowstyle="-|>", color='white', lw=1.5))
    ax_obj.text(0, -0.25, f"{value:.3f}", ha='center', va='center', fontsize=8, color='white')
    ax_obj.text(0, -0.60, label,          ha='center', va='center', fontsize=7, color='#aaa')
    ax_obj.set_xlim(-1.2, 1.2)
    ax_obj.set_ylim(-0.8, 1.2)


def draw_drone_top(ax_obj, roll_deg, pitch_deg, motors, stale=False):
    ax_obj.clear()
    ax_obj.set_facecolor('#1a1a2e')
    ax_obj.set_xlim(-2, 2)
    ax_obj.set_ylim(-2, 2)
    ax_obj.set_aspect('equal')
    ax_obj.axis('off')

    for dx, dy in [(-1, 1), (1, 1), (-1, -1), (1, -1)]:
        ax_obj.plot([0, dx * 1.1], [0, dy * 1.1], color='#888', lw=3, solid_capstyle='round')

    positions = [(-1.1, 1.1), (1.1, 1.1), (-1.1, -1.1), (1.1, -1.1)]
    labels    = ['FL', 'FR', 'BL', 'BR']
    colors    = ['#378ADD', '#1D9E75', '#378ADD', '#1D9E75']
    for i, ((mx, my), lbl, col) in enumerate(zip(positions, labels, colors)):
        pct   = max(0.0, min(100.0, motors[i] if i < len(motors) else 0.0))
        alpha = 0.25 + 0.75 * (pct / 100.0)
        circ  = plt.Circle((mx, my), 0.38, color=col, alpha=alpha, zorder=3)
        ax_obj.add_patch(circ)
        ax_obj.text(mx, my, f"{lbl}\n{pct:.0f}%", ha='center', va='center',
                    fontsize=6, color='white', fontweight='bold', zorder=4)

    body = plt.Circle((0, 0), 0.22, color='#ccc', zorder=5)
    ax_obj.add_patch(body)
    ax_obj.text(0, 0, f"R{roll_deg:+.0f}°\nP{pitch_deg:+.0f}°",
                ha='center', va='center', fontsize=6, color='#111', zorder=6)

    ax_obj.annotate("", xy=(0, 1.6), xytext=(0, 0.25),
                    arrowprops=dict(arrowstyle="-|>", color='#facc15', lw=1.5), zorder=7)

    # FIX: show stale indicator on diagram
    stale_str = "  [STALE]" if stale else ""
    ax_obj.set_title(f"Drone view  (↑ = fwd){stale_str}", fontsize=7,
                     color='#e74c3c' if stale else '#aaa', pad=2)


# ================= COMMAND LOG =================
cmd_log = []

def log(msg, prefix="> "):
    cmd_log.append(f"{prefix}{msg}")
    if len(cmd_log) > 40:
        cmd_log.pop(0)


# ================= THEME =================
DARK = '#0d0d1a'
MID  = '#12122a'
ACC  = '#378ADD'
GRN  = '#1D9E75'
ORG  = '#D85A30'
YLW  = '#facc15'
PNK  = '#e879f9'

plt.style.use('dark_background')
# ── Disable matplotlib single-key shortcuts so they don't hijack the
# command input (e.g. 's' was opening the Save dialog, 'q' quit, etc.)
_KEYS_TO_CLEAR = [
    'keymap.save', 'keymap.quit', 'keymap.fullscreen',
    'keymap.grid',  'keymap.grid_minor', 'keymap.pan',
    'keymap.zoom',  'keymap.home', 'keymap.back',
    'keymap.forward', 'keymap.yscale', 'keymap.xscale',
    'keymap.copy',
]
for _k in _KEYS_TO_CLEAR:
    matplotlib.rcParams[_k] = []

fig = plt.figure(figsize=(18, 10), facecolor=DARK)
fig.suptitle("Drone Ground Station  v5", color='#ccc', fontsize=11, y=0.99)

outer = gridspec.GridSpec(1, 3, figure=fig, wspace=0.30,
                          left=0.04, right=0.97, top=0.96, bottom=0.06)


# ══════════════════════════════════════════════════════════════════════════════
# LEFT COLUMN  —  accel | velocity | pos_z | RPY | gauges
# ══════════════════════════════════════════════════════════════════════════════
left_gs = gridspec.GridSpecFromSubplotSpec(
    6, 1, subplot_spec=outer[0], hspace=0.22,
    height_ratios=[1, 1, 0.8, 1, 0.08, 0.65])

ax_accel = fig.add_subplot(left_gs[0])
ax_vel   = fig.add_subplot(left_gs[1])
ax_pos   = fig.add_subplot(left_gs[2])
ax_rpy   = fig.add_subplot(left_gs[3])   # ← NEW: Roll/Pitch/Yaw

gauge_gs = gridspec.GridSpecFromSubplotSpec(1, 2, subplot_spec=left_gs[5])
ax_ga    = fig.add_subplot(gauge_gs[0])
ax_gv    = fig.add_subplot(gauge_gs[1])

for ax_ in (ax_accel, ax_vel, ax_pos, ax_rpy):
    ax_.set_facecolor(MID)
    ax_.tick_params(labelsize=7, colors='#aaa')
    for sp in ax_.spines.values():
        sp.set_color('#333')
    ax_.grid(True, color='#2a2a4a', lw=0.5)

ax_accel.set_ylabel("accel (g)",   fontsize=7, color='#aaa')
ax_vel.set_ylabel("vel (m/s)",     fontsize=7, color='#aaa')
ax_pos.set_ylabel("pos_z (m)",     fontsize=7, color='#aaa')
ax_rpy.set_ylabel("degrees (°)",   fontsize=7, color='#aaa')
ax_accel.set_title("Acceleration (packet A)", fontsize=8, color='#aaa', loc='left', pad=2)
ax_vel.set_title("Velocity (packet C)",       fontsize=8, color='#aaa', loc='left', pad=2)
ax_pos.set_title("Position Z (packet C)",     fontsize=8, color='#aaa', loc='left', pad=2)
ax_rpy.set_title("Attitude — Roll / Pitch / Yaw", fontsize=8, color='#aaa', loc='left', pad=2)

la_x, = ax_accel.plot([], [], color=ACC, lw=1.2, label='ax')
la_y, = ax_accel.plot([], [], color=GRN, lw=1.2, label='ay')
la_z, = ax_accel.plot([], [], color=ORG, lw=1.2, label='az')
lv_x, = ax_vel.plot([],   [], color=ACC, lw=1.2, label='vx')
lv_y, = ax_vel.plot([],   [], color=GRN, lw=1.2, label='vy')
lv_z, = ax_vel.plot([],   [], color=ORG, lw=1.2, label='vz')
lp_z, = ax_pos.plot([],   [], color=YLW, lw=1.2, label='pos_z')

# ── NEW: RPY lines ────────────────────────────────────────────────────────────
lr_r, = ax_rpy.plot([], [], color=ACC, lw=1.3, label='Roll')
lr_p, = ax_rpy.plot([], [], color=GRN, lw=1.3, label='Pitch')
lr_y, = ax_rpy.plot([], [], color=PNK, lw=1.3, label='Yaw', linestyle='--')
ax_rpy.axhline(0, color='#444', lw=0.6, linestyle=':')

for ax_, lines in [(ax_accel, [la_x, la_y, la_z]),
                   (ax_vel,   [lv_x, lv_y, lv_z]),
                   (ax_pos,   [lp_z]),
                   (ax_rpy,   [lr_r, lr_p, lr_y])]:
    ax_.legend(handles=lines, fontsize=6, loc='upper right',
               facecolor='#1a1a2e', edgecolor='none', labelcolor='#aaa', ncol=3)
# ══════════════════════════════════════════════════════════════════════════════
# CENTRE COLUMN  —  angular rate | dps gauge | drone view
# ══════════════════════════════════════════════════════════════════════════════
ctr_gs = gridspec.GridSpecFromSubplotSpec(
    4, 1, subplot_spec=outer[1], hspace=0.18,
    height_ratios=[1, 0.08, 0.55, 1.25])

ax_dps   = fig.add_subplot(ctr_gs[0])
ax_gdps  = fig.add_subplot(ctr_gs[2])
ax_drone = fig.add_subplot(ctr_gs[3])

ax_dps.set_facecolor(MID)
ax_dps.tick_params(labelsize=7, colors='#aaa')
for sp in ax_dps.spines.values():
    sp.set_color('#333')
ax_dps.grid(True, color='#2a2a4a', lw=0.5)

ax_dps.set_ylabel("dps", fontsize=7, color='#aaa')
ax_dps.set_title("Angular Rate (packet B)", fontsize=8, color='#aaa', loc='left', pad=2)

ld_x, = ax_dps.plot([], [], color=ACC, lw=1.2, label='gx')
ld_y, = ax_dps.plot([], [], color=GRN, lw=1.2, label='gy')
ld_z, = ax_dps.plot([], [], color=ORG, lw=1.2, label='gz')

ax_dps.legend(handles=[ld_x, ld_y, ld_z], fontsize=6, loc='upper right',
              facecolor='#1a1a2e', edgecolor='none', labelcolor='#aaa', ncol=3)

# ══════════════════════════════════════════════════════════════════════════════
# RIGHT COLUMN  —  command log | motor bars
# ══════════════════════════════════════════════════════════════════════════════
rgt_gs = gridspec.GridSpecFromSubplotSpec(
    2, 1, subplot_spec=outer[2], hspace=0.30,
    height_ratios=[1.2, 1])

ax_cmd   = fig.add_subplot(rgt_gs[0])
ax_motor = fig.add_subplot(rgt_gs[1])

ax_cmd.set_facecolor(MID)
ax_cmd.axis('off')
ax_cmd.set_title("Command interpreter", fontsize=8, color='#aaa', loc='left', pad=2)
cmd_text = ax_cmd.text(0.01, 0.97, "", transform=ax_cmd.transAxes,
                       fontsize=7, color='#7ec8e3', va='top', family='monospace')

ax_motor.set_facecolor(MID)
ax_motor.set_title("Motor outputs  [FL  FR  BL  BR]",
                   fontsize=8, color='#aaa', loc='left', pad=2)
ax_motor.set_xlim(-0.5, 3.5)
ax_motor.set_ylim(0, 110)
ax_motor.set_xticks([0, 1, 2, 3])
ax_motor.set_xticklabels(['FL', 'FR', 'BL', 'BR'], fontsize=8, color='#aaa')
ax_motor.set_yticks([0, 25, 50, 75, 100])
ax_motor.set_yticklabels(['0%', '25%', '50%', '75%', '100%'], fontsize=7, color='#aaa')
ax_motor.yaxis.tick_right()
for sp in ax_motor.spines.values():
    sp.set_color('#333')
ax_motor.grid(axis='y', color='#2a2a4a', lw=0.5)

m_colors       = [ACC, GRN, ACC, GRN]
motor_bars     = ax_motor.bar([0, 1, 2, 3], [0, 0, 0, 0],
                              color=m_colors, width=0.6, alpha=0.85)
motor_pct_txts = [ax_motor.text(i, 2, "0%", ha='center', fontsize=7, color='#ccc')
                  for i in range(4)]

# ── FIX: stale indicator text on motor panel ──────────────────────────────────
motor_stale_txt = ax_motor.text(1.5, 105, "", ha='center', fontsize=7,
                                color='#e74c3c', style='italic')


# ================= STATUS BAR + BLE INDICATOR =================
status_txt = fig.text(0.02, 0.008, "", fontsize=7, color='#7ec8e3')
ble_dot    = fig.text(0.97, 0.008, "●", fontsize=10, color='#e74c3c',
                      ha='right', va='bottom')
ble_lbl    = fig.text(0.963, 0.008, "BLE disconnected",
                      fontsize=7, color='#e74c3c', ha='right', va='bottom')


# ================= COMMAND INPUT =================
cmd_ax_input = fig.add_axes([0.68, 0.012, 0.20, 0.022])
cmd_ax_input.set_facecolor('#1a1a2e')
for sp in cmd_ax_input.spines.values():
    sp.set_color('#444')
cmd_ax_input.axis('off')
cmd_label = cmd_ax_input.text(0.01, 0.5, "> _",
                               transform=cmd_ax_input.transAxes,
                               fontsize=8, color='#7ec8e3',
                               va='center', family='monospace')

_typed = [""]

# ── Extended command set with FCU send support ────────────────────────────────
BUILTIN_CMDS = {
    'help': (
        'cmds: status arm disarm motors stream clear\n'
        '  FC cmds: start stop hover land takeoff emergency\n'
        '  FC move:  fwd back left right yaw_left yaw_right\n'
        '  set_alt <m>  e.g.  set_alt 1.5'
    ),
    'status':  lambda d: (
        f"IMU {d['imu_samples']}  BARO {d['baro_samples']}  "
        f"BLE {'OK' if d['ble_connected'] else 'DISCONNECTED'}"
    ),
    'arm':     'Motors armed — throttle ready',
    'disarm':  'Motors disarmed',
    'motors':  lambda d: (
        f"FL={d['motors'][0]:.1f}%  FR={d['motors'][1]:.1f}%  "
        f"BL={d['motors'][2]:.1f}%  BR={d['motors'][3]:.1f}%"
    ),
    'stream':  f"Window={WINDOW_SEC}s  maxlen={MAXLEN}",
    'clear':   '__clear__',
}

# Commands that map directly to FCU_CMD keys
FC_ALIASES = {
    'start':     'START',
    'stop':      'STOP',
    'hover':     'HOVER',
    'land':      'LAND',
    'takeoff':   'TAKEOFF',
    'emergency': 'EMERGENCY',
    'fwd':       'MOVE_FWD',
    'back':      'MOVE_BACK',
    'left':      'MOVE_LEFT',
    'right':     'MOVE_RIGHT',
    'yaw_left':  'YAW_LEFT',
    'yaw_right': 'YAW_RIGHT',
}


def on_key(event):
    if event.key == 'enter':
        raw_cmd = _typed[0].strip()
        cmd     = raw_cmd.lower()

        if cmd:
            log(raw_cmd)

            # ── built-in display commands ─────────────────────────────────────
            if cmd in BUILTIN_CMDS:
                resp = BUILTIN_CMDS[cmd]
                if resp == '__clear__':
                    cmd_log.clear()
                elif callable(resp):
                    log(resp(reader.snapshot()), prefix="  ")
                else:
                    for line in resp.split('\n'):
                        log(line, prefix="  ")

            # ── set_alt <metres> ──────────────────────────────────────────────
            elif cmd.startswith('set_alt'):
                parts = cmd.split()
                try:
                    alt = float(parts[1])
                    ok, msg = reader.send_command('SET_ALT', param=alt)
                    log(msg, prefix="  " if ok else "! ")
                except (IndexError, ValueError):
                    log("usage: set_alt <metres>  e.g.  set_alt 1.5", prefix="! ")

            # ── FC alias commands (hover, land, start …) ──────────────────────
            elif cmd in FC_ALIASES:
                fc_key = FC_ALIASES[cmd]
                ok, msg = reader.send_command(fc_key)
                log(msg, prefix="  " if ok else "! ")

            else:
                log(f"unknown: {cmd}", prefix="! ")

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

    # ── accel ─────────────────────────────────────────────────────────────────
    t = d["t"]
    if t:
        i0, te = window_slice(t)
        ts = t[i0:]
        ax_accel.set_xlim(te - WINDOW_SEC, te)
        la_x.set_data(ts, d["ax"][i0:])
        la_y.set_data(ts, d["ay"][i0:])
        la_z.set_data(ts, d["az"][i0:])
        safe_ylim(ax_accel, d["ax"][i0:], d["ay"][i0:], d["az"][i0:])

    # ── FC velocity + pos_z ───────────────────────────────────────────────────
    tv = d["t_vel"]
    if tv:
        i0, te = window_slice(tv)
        ts = tv[i0:]
        ax_vel.set_xlim(te - WINDOW_SEC, te)
        lv_x.set_data(ts, d["vx"][i0:])
        lv_y.set_data(ts, d["vy"][i0:])
        lv_z.set_data(ts, d["vz"][i0:])
        safe_ylim(ax_vel, d["vx"][i0:], d["vy"][i0:], d["vz"][i0:])

        ax_pos.set_xlim(te - WINDOW_SEC, te)
        lp_z.set_data(ts, d["pos_z"][i0:])
        safe_ylim(ax_pos, d["pos_z"][i0:])

    # ── RPY attitude ──────────────────────────────────────────────────────────
    ta = d["t_att"]
    if ta:
        i0, te = window_slice(ta)
        ts = ta[i0:]
        ax_rpy.set_xlim(te - WINDOW_SEC, te)
        lr_r.set_data(ts, d["roll_h"][i0:])
        lr_p.set_data(ts, d["pitch_h"][i0:])
        lr_y.set_data(ts, d["yaw_h"][i0:])
        safe_ylim(ax_rpy, d["roll_h"][i0:], d["pitch_h"][i0:], d["yaw_h"][i0:])

    # ── gyro ──────────────────────────────────────────────────────────────────
    tg = d["t_gyro"]
    if tg:
        i0, te = window_slice(tg)
        ts = tg[i0:]
        ax_dps.set_xlim(te - WINDOW_SEC, te)
        ld_x.set_data(ts, d["gx"][i0:])
        ld_y.set_data(ts, d["gy"][i0:])
        ld_z.set_data(ts, d["gz"][i0:])
        safe_ylim(ax_dps, d["gx"][i0:], d["gy"][i0:], d["gz"][i0:])

    # ── gauges ────────────────────────────────────────────────────────────────
    amag = math.hypot(d["ax"][-1] if d["ax"] else 0.0,
                      math.hypot(d["ay"][-1] if d["ay"] else 0.0,
                                 d["az"][-1] if d["az"] else 0.0))
    vmag = math.hypot(d["vx"][-1] if d["vx"] else 0.0,
                      math.hypot(d["vy"][-1] if d["vy"] else 0.0,
                                 d["vz"][-1] if d["vz"] else 0.0))
    gmag = math.hypot(d["gx"][-1] if d["gx"] else 0.0,
                      math.hypot(d["gy"][-1] if d["gy"] else 0.0,
                                 d["gz"][-1] if d["gz"] else 0.0))
    draw_gauge(ax_ga,   amag,  2.0,   "||accel|| (g)",  ACC)
    draw_gauge(ax_gv,   vmag,  5.0,   "||vel|| (m/s)",  GRN)
    draw_gauge(ax_gdps, gmag, 500.0,  "||dps||",        ORG)

    # ── drone top-down view ───────────────────────────────────────────────────
    draw_drone_top(ax_drone, d["roll"], d["pitch"], d["motors"],
                   stale=d["motor_stale"])

    # ── motor bars ── FIX: force redraw each frame ────────────────────────────
    stale = d["motor_stale"]
    motor_stale_txt.set_text("NO PACKET" if stale else "")
    ax_motor.set_title(
        "Motor outputs  [FL  FR  BL  BR]" + ("  ⚠ STALE" if stale else ""),
        fontsize=8, color='#e74c3c' if stale else '#aaa', loc='left', pad=2
    )
    for i, (bar, txt) in enumerate(zip(motor_bars, motor_pct_txts)):
        pct = max(0.0, min(100.0,
                           d["motors"][i] if i < len(d["motors"]) else 0.0))
        bar.set_height(pct)                 # FIX: this IS correct for bar charts
        txt.set_position((i, pct + 1.5))   # FIX: use set_position not set_y
        txt.set_text(f"{pct:.0f}%")
        # FIX: dim bars when stale so operator knows data is old
        bar.set_alpha(0.35 if stale else 0.85)

    # ── command log ───────────────────────────────────────────────────────────
    cmd_text.set_text("\n".join(cmd_log[-18:]))

    # ── BLE indicator ─────────────────────────────────────────────────────────
    if d["ble_connected"]:
        ble_dot.set_color('#2ecc71')
        ble_lbl.set_text("BLE connected")
        ble_lbl.set_color('#2ecc71')
    else:
        ble_dot.set_color('#e74c3c')
        ble_lbl.set_text("BLE disconnected")
        ble_lbl.set_color('#e74c3c')

    # ── status bar ───────────────────────────────────────────────────────────
    alt_str = f"{d['alt_m'][-1]:.2f} m" if d["alt_m"] else "--"
    pz_str  = f"{d['pos_z'][-1]:.2f} m" if d["pos_z"] else "--"
    status_txt.set_text(
        f"IMU: {d['imu_samples']}  BARO: {d['baro_samples']}  "
        f"Roll: {d['roll']:+.1f}°  Pitch: {d['pitch']:+.1f}°  "
        f"Yaw: {d['yaw']:+.1f}°  BaroAlt: {alt_str}  FC pos_z: {pz_str}"
    )

    return []


anim = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)


# ================= STARTUP LOG =================
log("Drone Ground Station  v5")
log(f"addr : {ADDRESS}",             prefix="  ")
log(f"UUID : {NOTIFY_UUID[:24]}…",  prefix="  ")
log(f"write: {WRITE_UUID[:24]}…",   prefix="  ")
log("type 'help' for commands",      prefix="  ")


# ================= MAIN =================
if __name__ == "__main__":
    threading.Thread(target=start_ble, daemon=True).start()
    plt.show()