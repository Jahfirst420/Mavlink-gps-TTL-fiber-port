"""
MAVLink v2 Reader + External GPS Injector (MAVLink GPS_INPUT) + GUI + HUD กราฟิกแบบ Ardupilot
มี 2 แท็บ:
    1) "GPS Spoofer"    - อ่าน MAVLink ผ่าน telemetry แล้วยิงตำแหน่งเข้า FC ด้วย MAVLink GPS_INPUT (ลิงก์เดียว)
  ส่ง MAVLink GPS_INPUT เข้า FC ผ่าน telemetry link ออกพอร์ตแยก"""

import logging
import math
import threading
import time
import tkinter as tk
from tkinter import ttk
from serial.tools import list_ports
from pymavlink import mavutil

# ---------------- ค่าเริ่มต้น (แก้ในหน้า GUI ได้ภายหลัง ไม่ต้องแก้โค้ด) ----------------
DEFAULT_PORT = "COM20"           # พอร์ต MAVLink (แท็บ GPS Spoofer)
DEFAULT_BAUD = "921600"          # ต้องตรงกับ SERIALx_BAUD ของ FC
DEFAULT_USB_BAUD = "115200"      # baud เริ่มต้นสำหรับเชื่อมต่อ USB โดยตรง (USB CDC ปกติไม่สนใจค่านี้มาก)
BAUD_OPTIONS = [
    "4800", "9600", "19200", "38400", "57600",
    "115200", "230400", "460800", "500000", "576000", "921600",
]

LOG_FILE = "mavlink_log.txt"
DEBUG_RAW = False     # True = log ทุก message type ที่เข้ามาดิบๆ เพื่อ debug
LOG_INTERVAL = 2.0    # ส่ง log สรุปทุกกี่วินาที

DEFAULT_LAT = 14.0562008   # ค่าเริ่มต้นในช่องกรอก (แก้ในหน้า GUI ได้ภายหลัง)
DEFAULT_LON = 100.6164846
DEFAULT_SATELLITES = 20    # จำนวนดาวเทียมปลอมเริ่มต้น
FAKE_HDOP = 0.8
GROUND_ALT_MSL = 15.0   # ****สำคัญ ****ความสูง MSL ของจุดปล่อย/ground station (เมตร) ใส่ค่าจริงของพื้นที่
GPS_INPUT_RATE_HZ = 5.0    # อัตราส่ง GPS_INPUT (Hz) แนะนำ 5
ALT_SMOOTH_ALPHA = 0.3     # ค่ากรองสัญญาณ alt (0-1) ยิ่งน้อยยิ่งสมูธแต่หน่วงมากขึ้น เหมาะกับลงจอดนุ่มนวล

# ---------------- ตั้งค่า Logging ----------------
logging.basicConfig(
    level=logging.DEBUG if DEBUG_RAW else logging.INFO,
    format="%(asctime)s - %(message)s",
    handlers=[
        logging.FileHandler(LOG_FILE, encoding="utf-8"),
        logging.StreamHandler(),
    ],
)
log = logging.getLogger(__name__)


# ==================== ที่เก็บ state ของแต่ละการเชื่อมต่อ ====================
# แยก instance กันคนละแท็บ เพื่อไม่ให้ค่าจากพอร์ตหนึ่งไปทับอีกพอร์ตหนึ่ง

class TelemetryStore:
    def __init__(self, lat=DEFAULT_LAT, lon=DEFAULT_LON, sats=DEFAULT_SATELLITES):
        self.lock = threading.Lock()
        self.data = {
            "lat": lat,
            "lon": lon,
            "sats": sats,
            "vfr_alt": None,       # alt ดิบจาก VFR_HUD (baro)
            "rel_alt": None,       # alt ดิบจาก GLOBAL_POSITION_INT (EKF fused, เทียบ home)
            "computed_alt": None,  # alt ที่คำนวณ+กรองสัญญาณแล้ว (>= 0 เสมอ)
            "armed": False,
            "status_text": "",
            "gps_fix_type": None,
            "gps_sats_fc": None,
            "battery_v": None,
            "battery_pct": None,
            "roll": 0.0,
            "pitch": 0.0,
            "heading": 0.0,
            "airspeed": 0.0,
            "groundspeed": 0.0,
        }

    def get(self):
        with self.lock:
            return dict(self.data)

    def update(self, **kwargs):
        with self.lock:
            self.data.update(kwargs)

    def reset_telemetry(self):
        """เคลียร์เฉพาะค่าที่มาจาก MAVLink ตอน disconnect (ไม่แตะ lat/lon/sats ที่ผู้ใช้กรอกเอง)"""
        with self.lock:
            self.data.update({
                "vfr_alt": None, "rel_alt": None, "computed_alt": None,
                "armed": False, "status_text": "",
                "gps_fix_type": None, "gps_sats_fc": None,
                "battery_v": None, "battery_pct": None,
                "roll": 0.0, "pitch": 0.0, "heading": 0.0,
                "airspeed": 0.0, "groundspeed": 0.0,
            })

    def recompute_alt(self):
        """
        รวมค่า alt จากทุกแหล่งข้อมูล MAVLink ที่มี (VFR_HUD.alt + GLOBAL_POSITION_INT.relative_alt)
        เฉลี่ยกันแล้วกรองสัญญาณแบบ exponential moving average เพื่อให้ค่าไม่กระโดด (ลงจอดนุ่มนวล)
        และ floor ที่ 0 เสมอ ไม่ให้ติดลบ
        """
        with self.lock:
            candidates = [v for v in (self.data.get("vfr_alt"), self.data.get("rel_alt")) if v is not None]
            if not candidates:
                return
            raw = sum(candidates) / len(candidates)

            prev = self.data.get("computed_alt")
            if prev is None:
                smoothed = raw
            else:
                smoothed = ALT_SMOOTH_ALPHA * raw + (1 - ALT_SMOOTH_ALPHA) * prev

            self.data["computed_alt"] = max(0.0, smoothed)


# ==================== ส่วนอ่าน MAVLink (ใช้ร่วมกันทั้ง 2 แท็บ) ====================

def connect(port: str, baud: int):
    log.info(f"กำลังเชื่อมต่อ MAVLink ที่ {port} @ {baud} baud ...")
    conn = mavutil.mavlink_connection(port, baud=baud)
    conn.wait_heartbeat(timeout=10)
    log.info(
        f"เชื่อมต่อสำเร็จ (system {conn.target_system}, component {conn.target_component})"
    )

    if conn.target_system == 0:
        log.warning(
            "target_system = 0 ผิดปกติ! มักเกิดจากบอดเรทไม่ตรงกับที่ตั้งใน FC "
            "(ลองเปลี่ยน baud ให้ตรงกับพารามิเตอร์ SERIALx_BAUD ของบอร์ด)"
        )

    try:
        conn.mav.request_data_stream_send(
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            4,
            1,
        )
        log.info("ส่งคำขอ REQUEST_DATA_STREAM (ALL @ 4Hz) แล้ว")
    except Exception as e:
        log.warning(f"ส่ง request_data_stream ไม่สำเร็จ: {e}")

    for msg_id in (
        mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
        mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,
        mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
        mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS,
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
    ):
        try:
            conn.mav.command_long_send(
                conn.target_system,
                conn.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                msg_id,
                100000,  # 100ms = 10Hz (roll/pitch ต้องอัปเดตไวหน่อยให้ HUD ลื่น)
                0, 0, 0, 0, 0,
            )
        except Exception as e:
            log.warning(f"SET_MESSAGE_INTERVAL สำหรับ msg_id {msg_id} ไม่สำเร็จ: {e}")

    return conn


def mavlink_loop(conn, stop_event: threading.Event, store: TelemetryStore):
    log.info("เริ่มอ่านข้อมูล MAVLink v2 ...")
    idle_count = 0
    last_log_time = 0.0

    while not stop_event.is_set():
        msg = conn.recv_match(blocking=True, timeout=5)
        if msg is None:
            idle_count += 1
            log.warning(f"ไม่มีข้อมูลเข้ามาใน 5 วินาที (ครั้งที่ {idle_count})")
            continue
        idle_count = 0

        msg_type = msg.get_type()

        if DEBUG_RAW and msg_type != "BAD_DATA":
            log.debug(f"ได้รับ message: {msg_type}")

        if msg_type == "HEARTBEAT":
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            store.update(armed=armed)

        elif msg_type == "STATUSTEXT":
            text = msg.text
            if isinstance(text, bytes):
                text = text.decode(errors="ignore")
            text = text.rstrip("\x00").strip()
            if text:
                store.update(status_text=text)
                log.info(f"STATUSTEXT: {text}")

        elif msg_type == "GPS_RAW_INT":
            store.update(gps_fix_type=msg.fix_type, gps_sats_fc=msg.satellites_visible)

        elif msg_type == "GLOBAL_POSITION_INT":
            store.update(rel_alt=msg.relative_alt / 1000.0)  # mm -> m
            store.recompute_alt()

        elif msg_type == "VFR_HUD":
            store.update(
                vfr_alt=msg.alt,  # alt จาก barometer (เมตร)
                heading=float(msg.heading % 360),
                airspeed=msg.airspeed,
                groundspeed=msg.groundspeed,
            )
            store.recompute_alt()

        elif msg_type == "ATTITUDE":
            store.update(roll=math.degrees(msg.roll), pitch=math.degrees(msg.pitch))

        elif msg_type == "SYS_STATUS":
            voltage = msg.voltage_battery / 1000.0
            remaining = msg.battery_remaining
            store.update(battery_v=voltage, battery_pct=remaining)

        elif msg_type == "BATTERY_STATUS":
            voltage = msg.voltages[0] / 1000.0 if msg.voltages[0] != 65535 else None
            if voltage is not None:
                store.update(battery_v=voltage)

        elif msg_type == "BAD_DATA":
            log.warning("ได้รับ BAD_DATA - ตรวจสอบว่า baud ตรงกับที่ FC ตั้งไว้หรือไม่")

        now = time.monotonic()
        if (now - last_log_time) >= LOG_INTERVAL:
            last_log_time = now
            s = store.get()
            log.info(
                "ARMED={armed} | COMPUTED_ALT: {alt} m (vfr={vfr}, rel={rel}) | "
                "ROLL={roll:.1f} PITCH={pitch:.1f} HDG={hdg:.0f} GS={gs:.1f} | "
                "BAT: {bv}V ({bp}%) | GPS_FIX: {fix} SATS: {sats}".format(
                    armed="ARMED" if s["armed"] else "DISARMED",
                    alt=s["computed_alt"],
                    vfr=s["vfr_alt"],
                    rel=s["rel_alt"],
                    roll=s["roll"],
                    pitch=s["pitch"],
                    hdg=s["heading"],
                    gs=s["groundspeed"],
                    bv=s["battery_v"],
                    bp=s["battery_pct"],
                    fix=s["gps_fix_type"],
                    sats=s["gps_sats_fc"],
                )
            )


# ==================== ส่วนส่ง GPS_INPUT ผ่าน MAVLink  ====================

def gps_input_loop(stop_event: threading.Event, conn, store: TelemetryStore):
    """ส่ง MAVLink GPS_INPUT เข้า FC ผ่าน telemetry link ออกพอร์ตแยก"""
    log.info("เริ่มส่ง GPS_INPUT ผ่าน MAVLink ...")
    interval = 1.0 / GPS_INPUT_RATE_HZ

    # เรดาร์ให้แค่ตำแหน่ง ไม่ให้ความเร็ว -> ต้อง ignore ช่อง velocity + accuracy
    # ไม่งั้น EKF จะรับ velocity = 0 แล้วตำแหน่งเพี้ยน
    ignore_flags = (
        mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
        mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
        mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
        mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
        mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY
    )
    last_hb = 0.0
    while not stop_event.is_set():
        s = store.get()
        rel = s["computed_alt"] if s["computed_alt"] is not None else 0.0
        alt_out = rel + GROUND_ALT_MSL   # แปลง relative -> MSL
        now = time.time()
        if now - last_hb >= 1.0:
            conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
            last_hb = now
        try:
            conn.mav.gps_input_send(
                int(time.time() * 1e6),   # time_usec
                0,                        # gps_id
                ignore_flags,
                0,                        # time_week_ms
                0,                        # time_week
                3,                        # fix_type = 3 (3D) ให้ EKF ยอมรับ
                int(s["lat"] * 1e7),      # lat  = degrees * 1e7 (int)
                int(s["lon"] * 1e7),      # lon  = degrees * 1e7 (int)
                float(alt_out),           # alt  = เมตร MSL (ดูหมายเหตุ *)
                float(FAKE_HDOP),         # hdop
                1.0,                      # vdop
                0.0, 0.0, 0.0,            # vn, ve, vd  (ถูก ignore)
                0.0, 0.0, 0.0,            # speed/horiz/vert accuracy (ถูก ignore)
                int(s["sats"]),           # satellites_visible
            )
        except Exception as e:
            log.error(f"ส่ง GPS_INPUT ไม่สำเร็จ: {e}")
        stop_event.wait(interval)

    log.info("หยุดส่ง GPS_INPUT แล้ว")


# ==================== HUD กราฟิก (ใช้ร่วมกันทั้ง 2 แท็บ) ====================

FIX_TYPE_NAMES = {
    0: "No GPS",
    1: "No Fix",
    2: "2D Fix",
    3: "3D Fix",
    4: "DGPS",
    5: "RTK Float",
    6: "RTK Fixed",
}

SKY_COLOR = "#3a6ea5"
GROUND_COLOR = "#6b4a2f"
HUD_LINE = "#ffffff"
HUD_ACCENT = "#ffcc00"


class HUDCanvas(tk.Canvas):
    """
    วาด artificial horizon + heading tape + speed/alt tape คล้ายหน้า HUD ของ Mission Planner / Ardupilot
    """

    def __init__(self, parent, width=480, height=340):
        super().__init__(parent, width=width, height=height, bg="black",
                          highlightthickness=1, highlightbackground="gray")
        self.width = width
        self.height = height

    def draw(self, s: dict):
        self.delete("all")

        cx = self.width / 2
        cy = self.height / 2 + 15  # เผื่อที่ด้านบนให้ heading tape

        roll = s.get("roll") or 0.0
        pitch = s.get("pitch") or 0.0
        heading = s.get("heading") or 0.0
        groundspeed = s.get("groundspeed") or 0.0
        alt = s.get("computed_alt") or 0.0
        armed = s.get("armed", False)
        status_text = s.get("status_text") or ""
        battery_v = s.get("battery_v")
        battery_pct = s.get("battery_pct")
        gps_fix = s.get("gps_fix_type")
        gps_sats = s.get("gps_sats_fc")

        self._draw_horizon(cx, cy, roll, pitch)
        self._draw_roll_scale(cx, cy)
        self._draw_center_symbol(cx, cy)
        self._draw_heading_tape(cx, 18, self.width - 40, 32, heading)
        self._draw_vtape(48, cy, 56, 190, groundspeed, tick_step=5, tick_spacing_px=22,
                          decimals=0, unit="", side="left")
        self._draw_vtape(self.width - 48, cy, 56, 190, alt, tick_step=5, tick_spacing_px=22,
                          decimals=0, unit="m", side="right")

        # ---- overlay ข้อความสถานะ (คล้าย HUD จริง) ----
        if armed:
            self.create_text(cx, cy - 55, text="ARMED", fill="#33ff33", font=("Arial", 18, "bold"))
        else:
            self.create_text(cx, cy - 55, text="DISARMED", fill="#ff3333", font=("Arial", 18, "bold"))

        if status_text:
            self.create_text(cx, cy - 30, text=status_text, fill="#ff5555",
                              font=("Arial", 10, "bold"), width=self.width - 140)

        bat_txt = "Bat -"
        if battery_v is not None:
            pct_txt = f"{battery_pct}%" if battery_pct not in (None, -1) else "-"
            bat_txt = f"Bat {battery_v:.2f}V {pct_txt}"
        self.create_text(60, self.height - 12, text=bat_txt, fill="white",
                          font=("Consolas", 9), anchor="w")

        fix_name = FIX_TYPE_NAMES.get(gps_fix, "-")
        sats_txt = f" ({gps_sats} sats)" if gps_sats is not None else ""
        self.create_text(self.width - 60, self.height - 12, text=f"GPS: {fix_name}{sats_txt}",
                          fill="white", font=("Consolas", 9), anchor="e")

    # ---------------- ส่วนวาดย่อย ----------------

    def _rotate(self, x, y, cx, cy, angle_deg):
        a = math.radians(angle_deg)
        rx = x * math.cos(a) - y * math.sin(a)
        ry = x * math.sin(a) + y * math.cos(a)
        return cx + rx, cy + ry

    def _draw_horizon(self, cx, cy, roll, pitch):
        R = max(self.width, self.height) * 1.5
        px_per_deg = 4.0
        pitch_offset = max(-R + 10, min(R - 10, pitch * px_per_deg))
        angle = -roll  # roll ขวา (บวก) -> ขอบฟ้าเอียงซ้ายเมื่อมองจากมุมมองเครื่องบิน

        def rot(x, y):
            return self._rotate(x, y, cx, cy, angle)

        sky = [rot(-R, -R), rot(R, -R), rot(R, pitch_offset), rot(-R, pitch_offset)]
        ground = [rot(-R, pitch_offset), rot(R, pitch_offset), rot(R, R), rot(-R, R)]

        self.create_polygon([c for p in sky for c in p], fill=SKY_COLOR, outline="")
        self.create_polygon([c for p in ground for c in p], fill=GROUND_COLOR, outline="")

        p1 = rot(-R, pitch_offset)
        p2 = rot(R, pitch_offset)
        self.create_line(*p1, *p2, fill=HUD_LINE, width=2)

        # pitch ladder
        for a in (-30, -20, -10, 10, 20, 30):
            y0 = (pitch - a) * px_per_deg
            half = 26 if a % 20 == 0 else 16
            lp1 = rot(-half, y0)
            lp2 = rot(half, y0)
            self.create_line(*lp1, *lp2, fill=HUD_LINE)
            lbl = rot(half + 14, y0)
            self.create_text(*lbl, text=str(abs(a)), fill=HUD_LINE, font=("Consolas", 7))

    def _draw_roll_scale(self, cx, cy):
        radius = min(cx, cy) - 15
        for ang in (-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60):
            rad = math.radians(ang)
            x1 = cx + radius * math.sin(rad)
            y1 = cy - radius * math.cos(rad)
            tick_len = 10 if ang % 30 == 0 else 6
            x2 = cx + (radius - tick_len) * math.sin(rad)
            y2 = cy - (radius - tick_len) * math.cos(rad)
            self.create_line(x1, y1, x2, y2, fill=HUD_LINE)

        # ตัวชี้อ้างอิงคงที่ (สามเหลี่ยมชี้ลง) ที่ตำแหน่ง 0 องศา
        self.create_polygon(
            cx - 6, cy - radius - 2, cx + 6, cy - radius - 2, cx, cy - radius + 8,
            fill=HUD_LINE,
        )

    def _draw_center_symbol(self, cx, cy):
        wing = 42
        self.create_line(cx - wing, cy, cx - 10, cy, fill=HUD_ACCENT, width=3)
        self.create_line(cx + 10, cy, cx + wing, cy, fill=HUD_ACCENT, width=3)
        self.create_line(cx, cy, cx, cy + 10, fill=HUD_ACCENT, width=3)
        self.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill=HUD_ACCENT, outline="")

    def _draw_vtape(self, x, cy, width, height, value, tick_step, tick_spacing_px, decimals, unit, side):
        self.create_rectangle(x - width / 2, cy - height / 2, x + width / 2, cy + height / 2,
                               fill="black", outline=HUD_LINE)
        top = cy - height / 2
        bottom = cy + height / 2
        base_tick = math.floor(value / tick_step) * tick_step
        frac = (value - base_tick) / tick_step
        n = int(height / (2 * tick_spacing_px)) + 2

        for i in range(-n, n + 1):
            tick_val = base_tick + i * tick_step
            y = cy - (i - frac) * tick_spacing_px
            if top <= y <= bottom:
                if side == "left":
                    self.create_line(x + width / 2 - 8, y, x + width / 2, y, fill=HUD_LINE)
                    self.create_text(x - width / 2 + 4, y, text=f"{tick_val:.{decimals}f}",
                                      fill=HUD_LINE, font=("Consolas", 8), anchor="w")
                else:
                    self.create_line(x - width / 2, y, x - width / 2 + 8, y, fill=HUD_LINE)
                    self.create_text(x + width / 2 - 4, y, text=f"{tick_val:.{decimals}f}",
                                      fill=HUD_LINE, font=("Consolas", 8), anchor="e")

        self.create_rectangle(x - width / 2 - 4, cy - 10, x + width / 2 + 4, cy + 10,
                               outline=HUD_ACCENT, width=2, fill="#222222")
        self.create_text(x, cy, text=f"{value:.{decimals}f}{unit}", fill=HUD_ACCENT,
                          font=("Consolas", 9, "bold"))

    def _draw_heading_tape(self, cx, cy, width, height, heading):
        self.create_rectangle(cx - width / 2, cy - height / 2, cx + width / 2, cy + height / 2,
                               fill="black", outline=HUD_LINE)
        tick_step = 10
        tick_spacing_px = 18
        base = math.floor(heading / tick_step) * tick_step
        frac = (heading - base) / tick_step
        n = int(width / (2 * tick_spacing_px)) + 3
        names = {0: "N", 90: "E", 180: "S", 270: "W", 360: "N"}

        for i in range(-n, n + 1):
            val = (base + i * tick_step) % 360
            x = cx + (i - frac) * tick_spacing_px
            if cx - width / 2 <= x <= cx + width / 2:
                self.create_line(x, cy - height / 2, x, cy - height / 2 + 7, fill=HUD_LINE)
                lbl = names.get(val, str(val))
                self.create_text(x, cy - height / 2 + 17, text=lbl, fill=HUD_LINE, font=("Consolas", 8))

        self.create_polygon(
            cx - 5, cy - height / 2 - 2, cx + 5, cy - height / 2 - 2, cx, cy - height / 2 + 6,
            fill=HUD_ACCENT,
        )
        self.create_text(cx, cy + height / 2 + 8, text=f"{heading:.0f}°", fill=HUD_ACCENT,
                          font=("Consolas", 9, "bold"))


# ==================== แท็บ 1: GPS Spoofer (telemetry + ยิง GPS_INPUT ลิงก์เดียว) ====================

class GpsSpoofTab(ttk.Frame):
    def __init__(self, parent):
        super().__init__(parent, padding=12)
        self.store = TelemetryStore(lat=DEFAULT_LAT, lon=DEFAULT_LON, sats=DEFAULT_SATELLITES)
        self.session_stop_event = None  # สร้างใหม่ทุกครั้งที่ Connect เพื่อให้ Disconnect แล้ว Connect ใหม่ได้
        self.conn = None
        self.mav_thread = None
        self.gps_thread = None
        self.connected = False

        available_ports = [p.device for p in list_ports.comports()]

        # ---- ตั้งค่าการเชื่อมต่อ ----
        conn_frame = ttk.LabelFrame(self, text="การเชื่อมต่อพอร์ต", padding=8)
        conn_frame.grid(row=0, column=0, sticky="new", pady=(0, 8))

        ttk.Label(conn_frame, text="MAVLink Port:").grid(row=0, column=0, sticky="e")
        self.mav_port_var = tk.StringVar(value=DEFAULT_PORT)
        self.mav_port_combo = ttk.Combobox(
            conn_frame, textvariable=self.mav_port_var, values=available_ports, width=14
        )
        self.mav_port_combo.grid(row=0, column=1, padx=4, pady=2)

        ttk.Label(conn_frame, text="Baud:").grid(row=0, column=2, sticky="e")
        self.mav_baud_var = tk.StringVar(value=DEFAULT_BAUD)
        ttk.Combobox(
            conn_frame, textvariable=self.mav_baud_var, values=BAUD_OPTIONS, width=10
        ).grid(row=0, column=3, padx=4, pady=2)


        ttk.Button(conn_frame, text="🔄 Refresh", command=self.refresh_ports).grid(
            row=2, column=0, pady=(6, 0)
        )
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.on_connect_click)
        self.connect_btn.grid(row=2, column=1, pady=(6, 0))

        self.disconnect_btn = ttk.Button(
            conn_frame, text="Disconnect", command=self.disconnect, state="disabled"
        )
        self.disconnect_btn.grid(row=2, column=2, pady=(6, 0), padx=(4, 0))

        self.conn_status_label = ttk.Label(conn_frame, text="ยังไม่เชื่อมต่อ", foreground="gray")
        self.conn_status_label.grid(row=3, column=0, columnspan=4, sticky="w", pady=(4, 0))

        # ---- ช่องกรอกค่า GPS ที่จะยิงออก ----
        gps_frame = ttk.LabelFrame(self, text="ค่า GPS ที่จะยิงออก (MAVLink)", padding=8)
        gps_frame.grid(row=1, column=0, sticky="new", pady=(0, 8))

        ttk.Label(gps_frame, text="Latitude:").grid(row=0, column=0, sticky="e")
        self.lat_var = tk.StringVar(value=str(DEFAULT_LAT))
        ttk.Entry(gps_frame, textvariable=self.lat_var, width=16).grid(row=0, column=1, padx=4, pady=2)

        ttk.Label(gps_frame, text="Longitude:").grid(row=1, column=0, sticky="e")
        self.lon_var = tk.StringVar(value=str(DEFAULT_LON))
        ttk.Entry(gps_frame, textvariable=self.lon_var, width=16).grid(row=1, column=1, padx=4, pady=2)

        ttk.Label(gps_frame, text="Satellites:").grid(row=2, column=0, sticky="e")
        self.sat_var = tk.StringVar(value=str(DEFAULT_SATELLITES))
        ttk.Entry(gps_frame, textvariable=self.sat_var, width=16).grid(row=2, column=1, padx=4, pady=2)

        ttk.Button(gps_frame, text="Apply / ส่งค่า", command=self.apply).grid(
            row=3, column=0, columnspan=2, pady=(4, 2)
        )

        self.error_label = ttk.Label(gps_frame, text="", foreground="red")
        self.error_label.grid(row=4, column=0, columnspan=2)

        self.sending_label = ttk.Label(gps_frame, text="กำลังส่งออก: -", foreground="blue")
        self.sending_label.grid(row=5, column=0, columnspan=2, sticky="w", pady=(4, 0))

        # ---- HUD กราฟิก ----
        hud_container = ttk.LabelFrame(self, text="HUD", padding=6)
        hud_container.grid(row=0, column=1, rowspan=2, padx=(12, 0))
        self.hud = HUDCanvas(hud_container)
        self.hud.grid(row=0, column=0)

        self._tick()

    def refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.mav_port_combo["values"] = ports
        log.info(f"พบพอร์ต: {ports}")

    def on_connect_click(self):
        try:
            mav_port = self.mav_port_var.get().strip()
            mav_baud = int(self.mav_baud_var.get().strip())
        except ValueError:
            self.conn_status_label.config(text="baud rate ต้องเป็นตัวเลข", foreground="red")
            return

        if not mav_port:
            self.conn_status_label.config(text="กรุณาเลือกพอร์ตให้ครบ", foreground="red")
            return

        self.connect_btn.config(state="disabled")
        self.disconnect_btn.config(state="disabled")
        self.conn_status_label.config(text="กำลังเชื่อมต่อ...", foreground="orange")

        self.session_stop_event = threading.Event()

        threading.Thread(
            target=self._connect_worker,
            args=(mav_port, mav_baud, self.session_stop_event),
            daemon=True,
        ).start()

    def _connect_worker(self, mav_port, mav_baud, stop_event):
        try:
            conn = connect(mav_port, mav_baud)
        except Exception as e:
            # ห้ามอ้างอิง e ตรงๆ ใน lambda ที่มาทำงานทีหลังผ่าน .after() เพราะ Python จะลบตัวแปร
            # จาก "except ... as e" ทิ้งทันทีที่ออกจาก except block ต้องแปลงเป็น string เก็บไว้ก่อน
            err_msg = f"{type(e).__name__}: {e}"
            log.exception(f"เชื่อมต่อ MAVLink ไม่สำเร็จ: {err_msg}")
            self.after(0, lambda msg=err_msg: self._connect_failed(msg))
            return

        if stop_event.is_set():
            # ผู้ใช้กด Disconnect ระหว่างที่กำลังเชื่อมต่ออยู่พอดี
            conn.close()
            return

        self.conn = conn
        self.mav_thread = threading.Thread(
            target=mavlink_loop, args=(conn, stop_event, self.store), daemon=True
        )
        self.mav_thread.start()

        self.gps_thread = threading.Thread(
            target=gps_input_loop, args=(stop_event, self.conn, self.store), daemon=True
        )
        self.gps_thread.start()
        self.after(0, self._connect_success)

    def _connect_failed(self, err_text):
        self.conn_status_label.config(text=f"เชื่อมต่อไม่สำเร็จ: {err_text}", foreground="red")
        self.connect_btn.config(state="normal")
        self.disconnect_btn.config(state="disabled")

    def _connect_success(self):
        self.connected = True
        self.conn_status_label.config(text="เชื่อมต่อสำเร็จ", foreground="green")
        self.disconnect_btn.config(state="normal")

    def disconnect(self):
        if not self.connected:
            return

        self.disconnect_btn.config(state="disabled")
        self.conn_status_label.config(text="กำลังตัดการเชื่อมต่อ...", foreground="orange")

        # ปิดใน background thread กัน GUI ค้าง (mavlink_loop อาจติด blocking recv อยู่ได้ถึง 5 วิ)
        threading.Thread(target=self._disconnect_worker, daemon=True).start()

    def _disconnect_worker(self):
        if self.session_stop_event is not None:
            self.session_stop_event.set()
        if self.mav_thread is not None:
            self.mav_thread.join(timeout=6)
        if self.gps_thread is not None:
            self.gps_thread.join(timeout=3)
        if self.conn is not None:
            try:
                self.conn.close()
            except Exception:
                pass

        self.conn = None
        self.mav_thread = None
        self.gps_thread = None
        self.connected = False

        # เคลียร์ค่าจาก MAVLink ให้ HUD กลับไปแสดง "-" แทนค่าค้างจากการเชื่อมต่อครั้งก่อน
        self.store.reset_telemetry()

        log.info("ตัดการเชื่อมต่อ MAVLink แล้ว")
        self.after(0, self._disconnect_done)

    def _disconnect_done(self):
        self.connect_btn.config(state="normal")
        self.disconnect_btn.config(state="disabled")
        self.conn_status_label.config(text="ตัดการเชื่อมต่อแล้ว", foreground="gray")

    def apply(self):
        try:
            lat = float(self.lat_var.get())
            lon = float(self.lon_var.get())
            sats = int(self.sat_var.get())
        except ValueError:
            self.error_label.config(text="กรอกตัวเลขให้ถูกต้อง (lat/lon เป็นทศนิยม, sats เป็นจำนวนเต็ม)")
            return

        self.store.update(lat=lat, lon=lon, sats=sats)
        self.error_label.config(text="")
        log.info(f"ผู้ใช้ตั้งค่าใหม่: LAT={lat} LON={lon} SATS={sats}")

    def _tick(self):
        s = self.store.get()
        self.sending_label.config(text=f"กำลังส่งออก: LAT {s['lat']} / LON {s['lon']} / SAT {s['sats']}")
        self.hud.draw(s)
        self.after(150, self._tick)

    def shutdown(self):
        """เรียกตอนปิดโปรแกรม ไม่ต้องรอ join เพราะ thread เป็น daemon อยู่แล้ว"""
        if self.session_stop_event is not None:
            self.session_stop_event.set()
        if self.conn is not None:
            try:
                self.conn.close()
            except Exception:
                pass


# ==================== แท็บ 2: เชื่อมต่อ USB Type-C โดยตรง ====================

class UsbDirectTab(ttk.Frame):
    """
    เชื่อมต่อ Flight Controller ผ่าน USB-TTL adapter (เช่น CH340) แบบ serial (อ่านอย่างเดียว)
    """

    def __init__(self, parent):
        super().__init__(parent, padding=12)
        self.store = TelemetryStore()
        self.session_stop_event = None
        self.conn = None
        self.mav_thread = None
        self.connected = False

        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=1)

        available_ports = [p.device for p in list_ports.comports()]

        # ---- หน้าตั้งค่าการเชื่อมต่อ (แสดงตอนยังไม่เชื่อมต่อ) ----
        self.setup_frame = ttk.Frame(self)
        self.setup_frame.grid(row=0, column=0)

        ttk.Label(
            self.setup_frame,
            text="เชื่อมต่อ Flight Controller ผ่านสาย USB Type-C โดยตรง",
            font=("Arial", 12, "bold"),
        ).grid(row=0, column=0, columnspan=2, pady=(0, 14))

        ttk.Label(self.setup_frame, text="USB Port:").grid(row=1, column=0, sticky="e", pady=4)
        self.usb_port_var = tk.StringVar(value=available_ports[0] if available_ports else "")
        self.usb_port_combo = ttk.Combobox(
            self.setup_frame, textvariable=self.usb_port_var, values=available_ports, width=18
        )
        self.usb_port_combo.grid(row=1, column=1, padx=6, pady=4)

        ttk.Label(self.setup_frame, text="Baud:").grid(row=2, column=0, sticky="e", pady=4)
        self.usb_baud_var = tk.StringVar(value=DEFAULT_USB_BAUD)
        ttk.Combobox(
            self.setup_frame, textvariable=self.usb_baud_var, values=BAUD_OPTIONS, width=18
        ).grid(row=2, column=1, padx=6, pady=4)

        ttk.Button(self.setup_frame, text="🔄 Refresh พอร์ต", command=self.refresh_ports).grid(
            row=3, column=0, pady=(10, 0)
        )
        self.connect_btn = ttk.Button(self.setup_frame, text="เชื่อมต่อ (Connect)", command=self.on_connect_click)
        self.connect_btn.grid(row=3, column=1, pady=(10, 0))

        self.status_label = ttk.Label(self.setup_frame, text="ยังไม่เชื่อมต่อ", foreground="gray")
        self.status_label.grid(row=4, column=0, columnspan=2, pady=(8, 0))

        ttk.Label(
            self.setup_frame,
            text="เสียบสาย USB Type-C เข้ากับ Flight Controller โดยตรง\n"
                 "แล้วเลือกพอร์ตที่ปรากฏขึ้นมา",
            foreground="gray", justify="center",
        ).grid(row=5, column=0, columnspan=2, pady=(10, 0))

        # ---- หน้า HUD (แสดงหลังเชื่อมต่อสำเร็จ - เหมือนสลับไปอีกหน้าหนึ่ง) ----
        self.hud_frame = ttk.Frame(self)

        ttk.Label(self.hud_frame, text="เชื่อมต่อ USB โดยตรง", font=("Arial", 11, "bold")).pack(pady=(0, 6))
        self.hud = HUDCanvas(self.hud_frame, width=560, height=380)
        self.hud.pack()
        self.disconnect_btn = ttk.Button(
            self.hud_frame, text="Disconnect (กลับไปหน้าตั้งค่า)", command=self.disconnect
        )
        self.disconnect_btn.pack(pady=(8, 0))

        self._tick()

    def refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.usb_port_combo["values"] = ports
        log.info(f"พบพอร์ต USB: {ports}")

    def on_connect_click(self):
        port = self.usb_port_var.get().strip()
        try:
            baud = int(self.usb_baud_var.get().strip())
        except ValueError:
            self.status_label.config(text="baud rate ต้องเป็นตัวเลข", foreground="red")
            return

        if not port:
            self.status_label.config(text="กรุณาเลือกพอร์ต USB", foreground="red")
            return

        self.connect_btn.config(state="disabled")
        self.status_label.config(text="กำลังเชื่อมต่อ...", foreground="orange")

        self.session_stop_event = threading.Event()

        threading.Thread(
            target=self._connect_worker,
            args=(port, baud, self.session_stop_event),
            daemon=True,
        ).start()

    def _connect_worker(self, port, baud, stop_event):
        try:
            conn = connect(port, baud)
        except Exception as e:
            err_msg = f"{type(e).__name__}: {e}"
            log.exception(f"เชื่อมต่อ USB ไม่สำเร็จ: {err_msg}")
            self.after(0, lambda msg=err_msg: self._connect_failed(msg))
            return

        if stop_event.is_set():
            conn.close()
            return

        self.conn = conn
        self.mav_thread = threading.Thread(
            target=mavlink_loop, args=(conn, stop_event, self.store), daemon=True
        )
        self.mav_thread.start()

        self.after(0, self._connect_success)

    def _connect_failed(self, err_text):
        self.status_label.config(text=f"เชื่อมต่อไม่สำเร็จ: {err_text}", foreground="red")
        self.connect_btn.config(state="normal")

    def _connect_success(self):
        self.connected = True
        log.info("เชื่อมต่อ USB โดยตรงสำเร็จ - สลับไปหน้า HUD")
        # สลับหน้า: ซ่อนหน้าตั้งค่า แสดงหน้า HUD แทน (เหมือนเปลี่ยนไปอีกหน้าหนึ่ง)
        self.setup_frame.grid_remove()
        self.disconnect_btn.config(state="normal")
        self.hud_frame.grid(row=0, column=0)

    def disconnect(self):
        if not self.connected:
            return

        self.disconnect_btn.config(state="disabled")

        threading.Thread(target=self._disconnect_worker, daemon=True).start()

    def _disconnect_worker(self):
        if self.session_stop_event is not None:
            self.session_stop_event.set()
        if self.mav_thread is not None:
            self.mav_thread.join(timeout=6)
        if self.conn is not None:
            try:
                self.conn.close()
            except Exception:
                pass

        self.conn = None
        self.mav_thread = None
        self.connected = False
        self.store.reset_telemetry()

        log.info("ตัดการเชื่อมต่อ USB แล้ว - กลับไปหน้าตั้งค่า")
        self.after(0, self._disconnect_done)

    def _disconnect_done(self):
        # กลับไปหน้าตั้งค่า
        self.hud_frame.grid_remove()
        self.setup_frame.grid(row=0, column=0)
        self.connect_btn.config(state="normal")
        self.status_label.config(text="ตัดการเชื่อมต่อแล้ว", foreground="gray")

    def _tick(self):
        if self.connected:
            self.hud.draw(self.store.get())
        self.after(150, self._tick)

    def shutdown(self):
        if self.session_stop_event is not None:
            self.session_stop_event.set()
        if self.conn is not None:
            try:
                self.conn.close()
            except Exception:
                pass


# ==================== main ====================

def main():
    root = tk.Tk()
    root.title("MAVLink GPS Spoofer + HUD")

    notebook = ttk.Notebook(root)
    notebook.pack(fill="both", expand=True)

    tab_spoof = GpsSpoofTab(notebook)
    notebook.add(tab_spoof, text="GPS Spoofer")

    tab_usb = UsbDirectTab(notebook)
    notebook.add(tab_usb, text="เชื่อมต่อ USB โดยตรง")

    def on_close():
        tab_spoof.shutdown()
        tab_usb.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()
    log.info("ปิดโปรแกรมเรียบร้อย")


if __name__ == "__main__":
    main()
