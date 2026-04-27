import asyncio
import bisect
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
from bleak import BleakClient, BleakScanner


# ================= BLE CONFIG =================
ADDRESS     = "02:80:E1:00:00:AA"
NOTIFY_UUID = "d973f2e1-b19e-11e2-9e96-0800200c9a66"
WRITE_UUID  = "d973f2e2-b19e-11e2-9e96-0800200c9a66"

FMT = '<BHffff'

WINDOW_SEC    = 10.0
MAXLEN        = 5000
SEA_LEVEL_HPA = 1013.25
MOTOR_IS_NORMALIZED = True   # True = FC sends 0.0–1.0, False = 0–100


# ================= FCU COMMAND PROTOCOL =================
# Matches Attribute_Modified_CB switch in sample_service.c exactly
FCU_CMD = {
    'START':  (0x01, 0.0),
    'STOP':   (0x02, 0.0),
    'HOVER':  (0x03, 0.0),
    'LAND':   (0x04, 0.0),
    # PID tuning — param is the new gain value sent from command line
    'RP':     (0x05, None),   # Roll  P
    'RI':     (0x06, None),   # Roll  I
    'RD':     (0x07, None),   # Roll  D
    'PP':     (0x08, None),   # Pitch P
    'PI':     (0x09, None),   # Pitch I
    'PD':     (0x0A, None),   # Pitch D
    'YP':     (0x0B, None),   # Yaw   P
    'YI':     (0x0C, None),   # Yaw   I
    'YD':     (0x0D, None),   # Yaw   D
    'AP':     (0x0E, None),   # Alt   P
    'AI':     (0x0F, None),   # Alt   I
    'AD':     (0x10, None),   # Alt   D
}

# PID commands that REQUIRE a float parameter from the user
PID_CMDS = {'RP','RI','RD','PP','PI','PD','YP','YI','YD','AP','AI','AD'}

# Simple alias commands that need no parameter
FC_ALIASES = {
    'start': 'START',
    'stop':  'STOP',
    'hover': 'HOVER',
    'land':  'LAND',
}

def build_fcu_packet(cmd_id: int, param: float) -> bytes:
    """5-byte packet: cmd_id(u8) + param(f32le)  — matches sample_service.c"""
    return struct.pack('<Bf', cmd_id, param)


# ================= CONVERSIONS =================
def pressure_to_alt_m(p, sea=SEA_LEVEL_HPA):
    if p <= 0:
        return float("nan")
    return 44330.0 * (1.0 - (p / sea) ** 0.19029495718363465)

def clamp_motor(v: float) -> float:
    return max(0.0, min(100.0, v))

def vec3_mag(a, b, c):
    av = a[-1] if a else 0.0
    bv = b[-1] if b else 0.0
    cv = c[-1] if c else 0.0
    return math.sqrt(av*av + bv*bv + cv*cv)


# ================= PID SHADOW TABLE =================
# Mirrors PID.c static params so the GCS shows current values.
# Updated whenever user sends a tuning command.
# These match the initial values in PID.c:
#   rollPIDParams   = { .70, 0.33, 0.08, 0 }
#   pitchPIDParams  = { .70, 0.33, 0.08, 0 }
#   yawPIDParams    = { .1,  .05,  .01,  0 }
#   altitudePIDParams = { .1, 0, 0, 0 }
_pid_lock = threading.Lock()
_pid_shadow = {
    'roll':  {'p': 0.70, 'i': 0.33, 'd': 0.08},
    'pitch': {'p': 0.70, 'i': 0.33, 'd': 0.08},
    'yaw':   {'p': 0.10, 'i': 0.05, 'd': 0.01},
    'alt':   {'p': 0.10, 'i': 0.00, 'd': 0.00},
}

_CMD_TO_PID = {
    'RP': ('roll',  'p'), 'RI': ('roll',  'i'), 'RD': ('roll',  'd'),
    'PP': ('pitch', 'p'), 'PI': ('pitch', 'i'), 'PD': ('pitch', 'd'),
    'YP': ('yaw',   'p'), 'YI': ('yaw',   'i'), 'YD': ('yaw',   'd'),
    'AP': ('alt',   'p'), 'AI': ('alt',   'i'), 'AD': ('alt',   'd'),
}

def pid_shadow_update(cmd_upper: str, value: float):
    key = _CMD_TO_PID.get(cmd_upper)
    if key:
        axis, term = key
        with _pid_lock:
            _pid_shadow[axis][term] = value

def pid_shadow_snapshot():
    with _pid_lock:
        return {k: dict(v) for k, v in _pid_shadow.items()}


# ================= DATA STORE =================
class IMUBLEReader:
    def __init__(self, maxlen=MAXLEN):
        self.lock        = threading.Lock()
        self._ble_client = None
        self._ble_loop   = None

        # Packet A  0x41 : accel (g) + baro (hPa)
        self.t      = collections.deque(maxlen=maxlen)
        self.ax     = collections.deque(maxlen=maxlen)
        self.ay     = collections.deque(maxlen=maxlen)
        self.az     = collections.deque(maxlen=maxlen)
        self.t_baro = collections.deque(maxlen=maxlen)
        self.pres   = collections.deque(maxlen=maxlen)
        self.alt_m  = collections.deque(maxlen=maxlen)

        # Packet B  0x42 : gyro (dps)
        self.t_gyro = collections.deque(maxlen=maxlen)
        self.gx     = collections.deque(maxlen=maxlen)
        self.gy     = collections.deque(maxlen=maxlen)
        self.gz     = collections.deque(maxlen=maxlen)

        # Packet C  0x43 : velocity (m/s) + pos_z (m)
        self.t_vel = collections.deque(maxlen=maxlen)
        self.vx    = collections.deque(maxlen=maxlen)
        self.vy    = collections.deque(maxlen=maxlen)
        self.vz    = collections.deque(maxlen=maxlen)
        self.pos_z = collections.deque(maxlen=maxlen)

        # Packet D  0x44 : motor outputs
        self.motors             = [0.0, 0.0, 0.0, 0.0]  # [FL, FR, BL, BR] 0–100%
        self._last_motor_update = 0.0

        # Packet E  0x45 : attitude roll/pitch/yaw (rad) — AUTHORITATIVE
        self.t_att   = collections.deque(maxlen=maxlen)
        self.roll_h  = collections.deque(maxlen=maxlen)
        self.pitch_h = collections.deque(maxlen=maxlen)
        self.yaw_h   = collections.deque(maxlen=maxlen)
        self.roll    = 0.0
        self.pitch   = 0.0
        self.yaw     = 0.0

        self.start_time    = time.monotonic()
        self.imu_samples   = 0
        self.baro_samples  = 0
        self.ble_connected = False

    # ──────────────────────────────────────────────────────────────────────────
    def handle_packet(self, data: bytearray):
        if len(data) < 19:
            return
        pkt = data[0]
        now = time.monotonic() - self.start_time

        if pkt == 0x41:
            _, ts, ax, ay, az, pressure = struct.unpack(FMT, data[:19])
            with self.lock:
                self.t.append(now);  self.ax.append(ax)
                self.ay.append(ay);  self.az.append(az)
                self.t_baro.append(now)
                self.pres.append(pressure)
                self.alt_m.append(pressure_to_alt_m(pressure))
                self.imu_samples  += 1
                self.baro_samples += 1

        elif pkt == 0x42:
            # Store gyro only; yaw scalar comes exclusively from 0x45
            _, ts, gx, gy, gz, _ = struct.unpack(FMT, data[:19])
            with self.lock:
                self.t_gyro.append(now)
                self.gx.append(gx); self.gy.append(gy); self.gz.append(gz)

        elif pkt == 0x43:
            _, ts, vx, vy, vz, pos_z = struct.unpack(FMT, data[:19])
            with self.lock:
                self.t_vel.append(now)
                self.vx.append(vx); self.vy.append(vy)
                self.vz.append(vz); self.pos_z.append(pos_z)

        elif pkt == 0x44:
            _, ts, fr, fl, br, bl = struct.unpack(FMT, data[:19])
            with self.lock:
                scale = 100.0 if MOTOR_IS_NORMALIZED else 1.0
                self.motors = [round(clamp_motor(v * scale), 1)
                               for v in (fl, fr, bl, br)]
                self._last_motor_update = now

        elif pkt == 0x45:
            # Authoritative attitude — roll, pitch, yaw all from here only
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
    def send_command(self, cmd_name: str, param: float) -> tuple[bool, str]:
        """
        Send a 5-byte command to the FC.
        param is always required — caller must supply it.
        """
        entry = FCU_CMD.get(cmd_name.upper())
        if entry is None:
            return False, f"unknown command: {cmd_name}"

        cmd_id = entry[0]
        packet = build_fcu_packet(cmd_id, param)

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
        return True, f"→ FC  {cmd_name}  param={param:.4f}  raw={packet.hex()}"

    # ──────────────────────────────────────────────────────────────────────────
    def snapshot(self):
        """
        Return windowed data only. Uses bisect for O(log n) slicing.
        Lock is held only during the copy — minimises contention with BLE thread.
        """
        with self.lock:
            now        = time.monotonic() - self.start_time
            t_cutoff   = now - WINDOW_SEC
            motor_stale = (now - self._last_motor_update) > 1.5

            def _slice(t_dq, *val_dqs):
                tl = list(t_dq)
                if not tl:
                    return [[] for _ in range(1 + len(val_dqs))]
                i0 = bisect.bisect_left(tl, t_cutoff)
                return [tl[i0:]] + [list(vd)[i0:] for vd in val_dqs]

            t,  ax,  ay,  az         = _slice(self.t,      self.ax, self.ay, self.az)
            tb, pres, alt            = _slice(self.t_baro,  self.pres, self.alt_m)
            tg, gx,  gy,  gz         = _slice(self.t_gyro,  self.gx, self.gy, self.gz)
            tv, vx,  vy,  vz, pos_z  = _slice(self.t_vel,   self.vx, self.vy, self.vz, self.pos_z)
            ta, rh,  ph,  yh         = _slice(self.t_att,   self.roll_h, self.pitch_h, self.yaw_h)

            return {
                "t": t, "ax": ax, "ay": ay, "az": az,
                "t_baro": tb, "pres": pres, "alt_m": alt,
                "t_gyro": tg, "gx": gx, "gy": gy, "gz": gz,
                "t_vel": tv, "vx": vx, "vy": vy, "vz": vz, "pos_z": pos_z,
                "t_att": ta, "roll_h": rh, "pitch_h": ph, "yaw_h": yh,
                "motors":      list(self.motors),
                "motor_stale": motor_stale,
                "roll":  self.roll, "pitch": self.pitch, "yaw": self.yaw,
                "imu_samples":   self.imu_samples,
                "baro_samples":  self.baro_samples,
                "ble_connected": self.ble_connected,
            }


reader = IMUBLEReader()


# ================= BLE (scan-first, auto-reconnect) =================
def notification_handler(sender, data):
    reader.handle_packet(bytearray(data))


async def ble_loop():
    # FIX: get_running_loop() — get_event_loop() deprecated in Python 3.10+
    reader._ble_loop = asyncio.get_running_loop()
    while True:
        try:
            t0 = time.monotonic()
            log("Scanning for drone...", prefix="  ")
            device = await BleakScanner.find_device_by_address(ADDRESS, timeout=5.0)
            if device is None:
                log(f"Not found ({time.monotonic()-t0:.1f}s) — retrying...", prefix="! ")
                await asyncio.sleep(1.0)
                continue

            log(f"Found in {time.monotonic()-t0:.1f}s, connecting...", prefix="  ")
            t1 = time.monotonic()

            async with BleakClient(device, timeout=5.0) as client:
                reader._ble_client = client
                with reader.lock:
                    reader.ble_connected = True
                log(f"Connected in {time.monotonic()-t1:.1f}s", prefix="  ")
                await client.start_notify(NOTIFY_UUID, notification_handler)
                log("Streaming...", prefix="  ")
                while client.is_connected:
                    await asyncio.sleep(0.5)
                log("Connection lost", prefix="! ")

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
def safe_ylim(ax_obj, *series):
    vals = [v for s in series for v in s if math.isfinite(v)]
    if not vals:
        return
    lo, hi = min(vals), max(vals)
    pad = (hi - lo) * 0.15 if hi != lo else 0.5
    ax_obj.set_ylim(lo - pad, hi + pad)

def set_xlim_from(t_list, ax_obj):
    if t_list:
        te = t_list[-1]
        ax_obj.set_xlim(te - WINDOW_SEC, te)


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
    ax_obj.annotate("", xy=(0.75*np.cos(ang), 0.75*np.sin(ang)), xytext=(0,0),
                    arrowprops=dict(arrowstyle="-|>", color='white', lw=1.5))
    ax_obj.text(0, -0.25, f"{value:.3f}", ha='center', va='center', fontsize=8, color='white')
    ax_obj.text(0, -0.60, label,          ha='center', va='center', fontsize=7, color='#aaa')
    ax_obj.set_xlim(-1.2, 1.2)
    ax_obj.set_ylim(-0.8, 1.2)


def draw_drone_top(ax_obj, roll_deg, pitch_deg, motors, stale=False):
    ax_obj.clear()
    ax_obj.set_facecolor('#1a1a2e')
    ax_obj.set_xlim(-2, 2); ax_obj.set_ylim(-2, 2)
    ax_obj.set_aspect('equal'); ax_obj.axis('off')
    for dx, dy in [(-1,1),(1,1),(-1,-1),(1,-1)]:
        ax_obj.plot([0, dx*1.1],[0, dy*1.1], color='#888', lw=3, solid_capstyle='round')
    positions = [(-1.1,1.1),(1.1,1.1),(-1.1,-1.1),(1.1,-1.1)]
    labels    = ['FL','FR','BL','BR']
    colors    = ['#378ADD','#1D9E75','#378ADD','#1D9E75']
    m = motors if len(motors)==4 else [0.0,0.0,0.0,0.0]
    for i, ((mx,my), lbl, col) in enumerate(zip(positions, labels, colors)):
        pct   = max(0.0, min(100.0, m[i]))
        alpha = 0.25 + 0.75*(pct/100.0)
        ax_obj.add_patch(plt.Circle((mx,my), 0.38, color=col, alpha=alpha, zorder=3))
        ax_obj.text(mx, my, f"{lbl}\n{pct:.0f}%", ha='center', va='center',
                    fontsize=6, color='white', fontweight='bold', zorder=4)
    ax_obj.add_patch(plt.Circle((0,0), 0.22, color='#ccc', zorder=5))
    ax_obj.text(0, 0, f"R{roll_deg:+.0f}°\nP{pitch_deg:+.0f}°",
                ha='center', va='center', fontsize=6, color='#111', zorder=6)
    ax_obj.annotate("", xy=(0,1.6), xytext=(0,0.25),
                    arrowprops=dict(arrowstyle="-|>", color='#facc15', lw=1.5), zorder=7)
    stale_str = "  [STALE]" if stale else ""
    ax_obj.set_title(f"Drone view  (↑=fwd){stale_str}", fontsize=7,
                     color='#e74c3c' if stale else '#aaa', pad=2)


def draw_pid_table(ax_obj, pid):
    """Render live PID gains as a mini table in the given axes."""
    ax_obj.clear()
    ax_obj.set_facecolor('#12122a')
    ax_obj.axis('off')
    ax_obj.set_title("PID gains (live)", fontsize=8, color='#aaa', loc='left', pad=2)
    headers = ["      ", "  Kp  ", "  Ki  ", "  Kd  "]
    rows = [
        ("Roll",  pid['roll']['p'],  pid['roll']['i'],  pid['roll']['d']),
        ("Pitch", pid['pitch']['p'], pid['pitch']['i'], pid['pitch']['d']),
        ("Yaw",   pid['yaw']['p'],   pid['yaw']['i'],   pid['yaw']['d']),
        ("Alt",   pid['alt']['p'],   pid['alt']['i'],   pid['alt']['d']),
    ]
    col_colors = [['#1a1a2e']*4]*5
    cell_text  = [[f"{v:.4f}" if isinstance(v, float) else v for v in row] for row in rows]
    tbl = ax_obj.table(
        cellText=cell_text,
        colLabels=headers,
        loc='center',
        cellLoc='center',
    )
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(7)
    tbl.scale(1, 1.4)
    for (r, c), cell in tbl.get_celld().items():
        cell.set_facecolor('#1a1a2e' if r > 0 else '#0d0d1a')
        cell.set_edgecolor('#333')
        cell.set_text_props(color='#7ec8e3' if r == 0 else '#ccc')


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
_KEYS_TO_CLEAR = [
    'keymap.save','keymap.quit','keymap.fullscreen','keymap.grid',
    'keymap.grid_minor','keymap.pan','keymap.zoom','keymap.home',
    'keymap.back','keymap.forward','keymap.yscale','keymap.xscale','keymap.copy',
]
for _k in _KEYS_TO_CLEAR:
    matplotlib.rcParams[_k] = []

fig = plt.figure(figsize=(20, 11), facecolor=DARK)
fig.suptitle("Drone Ground Station  v7", color='#ccc', fontsize=11, y=0.99)

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
ax_rpy   = fig.add_subplot(left_gs[3])

gauge_gs = gridspec.GridSpecFromSubplotSpec(1, 2, subplot_spec=left_gs[5])
ax_ga    = fig.add_subplot(gauge_gs[0])
ax_gv    = fig.add_subplot(gauge_gs[1])

for ax_ in (ax_accel, ax_vel, ax_pos, ax_rpy):
    ax_.set_facecolor(MID)
    ax_.tick_params(labelsize=7, colors='#aaa')
    for sp in ax_.spines.values():
        sp.set_color('#333')
    ax_.grid(True, color='#2a2a4a', lw=0.5)

ax_accel.set_ylabel("accel (g)",  fontsize=7, color='#aaa')
ax_vel.set_ylabel("vel (m/s)",    fontsize=7, color='#aaa')
ax_pos.set_ylabel("pos_z (m)",    fontsize=7, color='#aaa')
ax_rpy.set_ylabel("degrees (°)",  fontsize=7, color='#aaa')
ax_accel.set_title("Acceleration (packet A)",     fontsize=8, color='#aaa', loc='left', pad=2)
ax_vel.set_title("Velocity (packet C)",           fontsize=8, color='#aaa', loc='left', pad=2)
ax_pos.set_title("Position Z (packet C)",         fontsize=8, color='#aaa', loc='left', pad=2)
ax_rpy.set_title("Attitude — Roll/Pitch/Yaw",     fontsize=8, color='#aaa', loc='left', pad=2)

la_x, = ax_accel.plot([], [], color=ACC, lw=1.2, label='ax')
la_y, = ax_accel.plot([], [], color=GRN, lw=1.2, label='ay')
la_z, = ax_accel.plot([], [], color=ORG, lw=1.2, label='az')
lv_x, = ax_vel.plot([],   [], color=ACC, lw=1.2, label='vx')
lv_y, = ax_vel.plot([],   [], color=GRN, lw=1.2, label='vy')
lv_z, = ax_vel.plot([],   [], color=ORG, lw=1.2, label='vz')
lp_z, = ax_pos.plot([],   [], color=YLW, lw=1.2, label='pos_z')
lr_r, = ax_rpy.plot([],   [], color=ACC, lw=1.3, label='Roll')
lr_p, = ax_rpy.plot([],   [], color=GRN, lw=1.3, label='Pitch')
lr_y, = ax_rpy.plot([],   [], color=PNK, lw=1.3, label='Yaw', linestyle='--')
ax_rpy.axhline(0, color='#444', lw=0.6, linestyle=':')

for ax_, lines in [(ax_accel, [la_x,la_y,la_z]),
                   (ax_vel,   [lv_x,lv_y,lv_z]),
                   (ax_pos,   [lp_z]),
                   (ax_rpy,   [lr_r,lr_p,lr_y])]:
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
ax_dps.legend(handles=[ld_x,ld_y,ld_z], fontsize=6, loc='upper right',
              facecolor='#1a1a2e', edgecolor='none', labelcolor='#aaa', ncol=3)


# ══════════════════════════════════════════════════════════════════════════════
# RIGHT COLUMN  —  command log | motor bars | PID table
# ══════════════════════════════════════════════════════════════════════════════
rgt_gs = gridspec.GridSpecFromSubplotSpec(
    3, 1, subplot_spec=outer[2], hspace=0.35,
    height_ratios=[1.1, 0.9, 0.85])

ax_cmd   = fig.add_subplot(rgt_gs[0])
ax_motor = fig.add_subplot(rgt_gs[1])
ax_pid   = fig.add_subplot(rgt_gs[2])   # ← NEW: live PID table

ax_cmd.set_facecolor(MID); ax_cmd.axis('off')
ax_cmd.set_title("Command interpreter", fontsize=8, color='#aaa', loc='left', pad=2)
cmd_text = ax_cmd.text(0.01, 0.97, "", transform=ax_cmd.transAxes,
                       fontsize=7, color='#7ec8e3', va='top', family='monospace')

ax_motor.set_facecolor(MID)
ax_motor.set_title("Motor outputs  [FL  FR  BL  BR]",
                   fontsize=8, color='#aaa', loc='left', pad=2)
ax_motor.set_xlim(-0.5, 3.5); ax_motor.set_ylim(0, 110)
ax_motor.set_xticks([0,1,2,3])
ax_motor.set_xticklabels(['FL','FR','BL','BR'], fontsize=8, color='#aaa')
ax_motor.set_yticks([0,25,50,75,100])
ax_motor.set_yticklabels(['0%','25%','50%','75%','100%'], fontsize=7, color='#aaa')
ax_motor.yaxis.tick_right()
for sp in ax_motor.spines.values():
    sp.set_color('#333')
ax_motor.grid(axis='y', color='#2a2a4a', lw=0.5)

m_colors       = [ACC, GRN, ACC, GRN]
motor_bars     = ax_motor.bar([0,1,2,3],[0,0,0,0], color=m_colors, width=0.6, alpha=0.85)
motor_pct_txts = [ax_motor.text(i, 2, "0%", ha='center', fontsize=7, color='#ccc')
                  for i in range(4)]
motor_stale_txt = ax_motor.text(1.5, 105, "", ha='center', fontsize=7,
                                color='#e74c3c', style='italic')


# ================= STATUS BAR + BLE INDICATOR =================
status_txt = fig.text(0.02, 0.008, "", fontsize=7, color='#7ec8e3')
ble_dot    = fig.text(0.97, 0.008, "●", fontsize=10, color='#e74c3c', ha='right', va='bottom')
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


# ================= BUILT-IN COMMANDS =================
BUILTIN_CMDS = {
    'help': (
        'cmds: status motors stream clear\n'
        '  FC: start stop hover land\n'
        '  PID tune (need value):\n'
        '    rp ri rd  pp pi pd  yp yi yd  ap ai ad\n'
        '  e.g.  rp 0.8   pd 0.05   ap 0.2'
    ),
    'status': lambda d: (
        f"IMU {d['imu_samples']}  BARO {d['baro_samples']}  "
        f"BLE {'OK' if d['ble_connected'] else 'DISCONNECTED'}"
    ),
    'motors': lambda d: (
        f"FL={d['motors'][0]:.1f}%  FR={d['motors'][1]:.1f}%  "
        f"BL={d['motors'][2]:.1f}%  BR={d['motors'][3]:.1f}%"
        if len(d['motors']) == 4 else "No motor data yet"
    ),
    'stream': f"Window={WINDOW_SEC}s  maxlen={MAXLEN}",
    'clear':  '__clear__',
}


def on_key(event):
    if event.key == 'enter':
        raw = _typed[0].strip()
        cmd = raw.lower()

        if cmd:
            log(raw)

            # ── built-ins ──────────────────────────────────────────────────
            if cmd in BUILTIN_CMDS:
                resp = BUILTIN_CMDS[cmd]
                if resp == '__clear__':
                    cmd_log.clear()
                elif callable(resp):
                    log(resp(reader.snapshot()), prefix="  ")
                else:
                    for line in resp.split('\n'):
                        log(line, prefix="  ")

            # ── FC alias (start / stop / hover / land) — no param needed ──
            elif cmd in FC_ALIASES:
                fc_key = FC_ALIASES[cmd]
                ok, msg = reader.send_command(fc_key, 0.0)
                log(msg, prefix="  " if ok else "! ")

            # ── PID tuning: "rp 0.8"  "pd 0.05"  etc. ────────────────────
            else:
                parts = cmd.split()
                cmd_upper = parts[0].upper()

                if cmd_upper in PID_CMDS:
                    if len(parts) < 2:
                        log(f"usage: {parts[0].lower()} <value>  e.g.  {parts[0].lower()} 0.50",
                            prefix="! ")
                    else:
                        try:
                            val = float(parts[1])
                            ok, msg = reader.send_command(cmd_upper, val)
                            if ok:
                                # Update local shadow so PID table refreshes immediately
                                pid_shadow_update(cmd_upper, val)
                            log(msg, prefix="  " if ok else "! ")
                        except ValueError:
                            log(f"bad value: '{parts[1]}' — need a float", prefix="! ")
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
    d   = reader.snapshot()
    pid = pid_shadow_snapshot()

    # ── accel ─────────────────────────────────────────────────────────────────
    t = d["t"]
    if t:
        set_xlim_from(t, ax_accel)
        la_x.set_data(t, d["ax"]); la_y.set_data(t, d["ay"]); la_z.set_data(t, d["az"])
        safe_ylim(ax_accel, d["ax"], d["ay"], d["az"])

    # ── velocity + pos_z ──────────────────────────────────────────────────────
    tv = d["t_vel"]
    if tv:
        set_xlim_from(tv, ax_vel)
        lv_x.set_data(tv, d["vx"]); lv_y.set_data(tv, d["vy"]); lv_z.set_data(tv, d["vz"])
        safe_ylim(ax_vel, d["vx"], d["vy"], d["vz"])
        set_xlim_from(tv, ax_pos)
        lp_z.set_data(tv, d["pos_z"])
        safe_ylim(ax_pos, d["pos_z"])

    # ── RPY ───────────────────────────────────────────────────────────────────
    ta = d["t_att"]
    if ta:
        set_xlim_from(ta, ax_rpy)
        lr_r.set_data(ta, d["roll_h"])
        lr_p.set_data(ta, d["pitch_h"])
        lr_y.set_data(ta, d["yaw_h"])
        safe_ylim(ax_rpy, d["roll_h"], d["pitch_h"], d["yaw_h"])

    # ── gyro ──────────────────────────────────────────────────────────────────
    tg = d["t_gyro"]
    if tg:
        set_xlim_from(tg, ax_dps)
        ld_x.set_data(tg, d["gx"]); ld_y.set_data(tg, d["gy"]); ld_z.set_data(tg, d["gz"])
        safe_ylim(ax_dps, d["gx"], d["gy"], d["gz"])

    # ── gauges ────────────────────────────────────────────────────────────────
    draw_gauge(ax_ga,   vec3_mag(d["ax"],d["ay"],d["az"]), 2.0,   "||accel|| (g)", ACC)
    draw_gauge(ax_gv,   vec3_mag(d["vx"],d["vy"],d["vz"]), 5.0,   "||vel|| (m/s)", GRN)
    draw_gauge(ax_gdps, vec3_mag(d["gx"],d["gy"],d["gz"]), 500.0, "||dps||",       ORG)

    # ── drone view ────────────────────────────────────────────────────────────
    draw_drone_top(ax_drone, d["roll"], d["pitch"], d["motors"],
                   stale=d["motor_stale"])

    # ── motor bars ────────────────────────────────────────────────────────────
    stale = d["motor_stale"]
    motor_stale_txt.set_text("NO PACKET" if stale else "")
    ax_motor.set_title(
        "Motor outputs  [FL  FR  BL  BR]" + ("  ⚠ STALE" if stale else ""),
        fontsize=8, color='#e74c3c' if stale else '#aaa', loc='left', pad=2)
    m = d["motors"] if len(d["motors"]) == 4 else [0.0]*4
    for i, (bar, txt) in enumerate(zip(motor_bars, motor_pct_txts)):
        pct = max(0.0, min(100.0, m[i]))
        bar.set_height(pct)
        txt.set_position((i, pct + 1.5))
        txt.set_text(f"{pct:.0f}%")
        bar.set_alpha(0.35 if stale else 0.85)

    # ── PID table ─────────────────────────────────────────────────────────────
    draw_pid_table(ax_pid, pid)

    # ── command log ───────────────────────────────────────────────────────────
    cmd_text.set_text("\n".join(cmd_log[-18:]))

    # ── BLE indicator ─────────────────────────────────────────────────────────
    if d["ble_connected"]:
        ble_dot.set_color('#2ecc71'); ble_lbl.set_text("BLE connected");  ble_lbl.set_color('#2ecc71')
    else:
        ble_dot.set_color('#e74c3c'); ble_lbl.set_text("BLE disconnected"); ble_lbl.set_color('#e74c3c')

    # ── status bar ────────────────────────────────────────────────────────────
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
log("Drone Ground Station  v7")
log(f"addr : {ADDRESS}",            prefix="  ")
log(f"UUID : {NOTIFY_UUID[:24]}…", prefix="  ")
log(f"write: {WRITE_UUID[:24]}…",  prefix="  ")
log("type 'help' for commands",     prefix="  ")


# ================= MAIN =================
if __name__ == "__main__":
    threading.Thread(target=start_ble, daemon=True).start()
    plt.show()