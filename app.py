"""
app.py - single entry point:  python app.py

  Camera 1 (cam1, index 2) -> BASE stepper  (firmware X axis, limit switch, homed)   error_y vs the drawn line
  Camera 2 (cam2, index 1) -> Y stepper     (firmware Y axis, no switch)            error_x vs the centre line

The two axes are completely independent: own camera, own state machine, own thread.
Each axis: no sun -> SEARCH (sweeps its own range in ONE direction, flips only at the range end)
           sun seen -> TRACK (proportional move, min/max step size, hysteresis so it never jitters)
           inside tolerance -> LOCKED (no motion, keeps watching and re-corrects if the sun drifts)

controller.py is not modified (only draw_overlay_cam2 is imported from it).
Requires the updated firmware in firmware/firmware.ino.
"""
import atexit
import json
import os
import sys
import threading
import time

import cv2
import numpy as np
import serial
from flask import Flask, Response, jsonify, render_template, request

from controller import draw_overlay_cam2
from sun_tracker import SunTracker

# =====================================================================
# CONFIGURATION
# =====================================================================
SERIAL_PORT = "/dev/ttyUSB0"
HOST, PORT = "127.0.0.1", 5000          # no auth on the API: only use 0.0.0.0 on a trusted network

CAMERA_INDEXES = {"cam1": 2, "cam2": 1}
CAM1_ROI = (306, 138, 383, 291)         # same values as before (green rectangle)
CAM1_TARGET = ((306 + 383) // 2, 230)   # the drawn line for camera 1

# --- Motor calibration ---
# Homing moves toward the limit switch and then zeroes the position. After that, the mirror
# must move away from the home switch by 90° in the opposite direction before scanning.
STEPS_FOR_90_DEG = 1600
MECHANICAL_DEG = 90.0
DEG_PER_STEP = MECHANICAL_DEG / STEPS_FOR_90_DEG
REFERENCE_STEPS = 1600                  # 90 deg reference after homing
BASE_SCAN_MIN = 1600                    # search window starts at 90° after home
BASE_SCAN_MAX = 2667                    # approx. 150° end of the active search range
MAX_STEPS = 3200

# --- Sun angle (sextant rule: reflected ray moves 2*theta) ---
REFERENCE_SUN_ANGLE = 0.0
SUN_ANGLE_SIGN = 1

# --- Control timing ---
SETTLE_S = 0.20                         # wait after a correction move before measuring (camera lag)
SEARCH_SETTLE_S = 0.10                  # shorter wait between search chunks (faster sweep)
SAMPLES = 2                             # fresh frames averaged per decision
LOST_S = 1.5                            # no sun for this long -> go back to SEARCH
MOVE_TIMEOUT_S = 2.5                    # give up waiting for a move to report finished

# --- Camera robustness ---
FORCE_MJPG = True
FRAME_W, FRAME_H = 640, 480
WARMUP_FRAMES = 10
BLACK_MAX = 2                           # a frame whose brightest pixel is <= this is "no signal"
BLACK_SECONDS = 3.0                     # ... for this long -> close + reopen the camera
CAM_STAGGER_S = 1.0                     # open cam2 this long after cam1 (USB bandwidth / init race)

# --- Tunables (editable live on the web page, saved to tune.json) ---
# key, label, default, min, max, step
PARAMS = [
    ("base_speed_us",     "Base speed (us/step, lower = faster)", 800, 300, 2000, 50),
    ("base_gain",         "Base steps per pixel",                 0.5, 0.05, 3.0, 0.05),
    ("base_sign",         "Base direction (+1 or -1)",            1, -1, 1, 2),
    ("base_tol_in",       "Base lock tolerance (px)",             5, 1, 40, 1),
    ("base_tol_out",      "Base re-correct above (px)",           12, 2, 80, 1),
    ("base_min_step",     "Base min steps per move",              6, 1, 30, 1),
    ("base_max_step",     "Base max steps per move",              150, 10, 600, 10),
    ("base_search_chunk", "Base search chunk (steps)",            80, 10, 300, 10),
    ("y_speed_us",        "Y speed (us/step, lower = faster)",    1000, 300, 2000, 50),
    ("y_gain",            "Y steps per pixel",                    0.45, 0.05, 3.0, 0.05),
    ("y_sign",            "Y direction (+1 or -1)",               1, -1, 1, 2),
    ("y_tol_in",          "Y lock tolerance (px)",                5, 1, 40, 1),
    ("y_tol_out",         "Y re-correct above (px)",              12, 2, 80, 1),
    ("y_min_step",        "Y min steps per move",                 6, 1, 30, 1),
    ("y_max_step",        "Y max steps per move",                 80, 10, 600, 10),
    ("y_search_chunk",    "Y search chunk (steps)",               40, 10, 300, 10),
    ("y_range",           "Y search range +/- (steps)",           1200, 100, 6000, 100),
]
PARAM_META = {p[0]: p for p in PARAMS}
TUNE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tune.json")
TUNE = {p[0]: p[2] for p in PARAMS}
tune_lock = threading.Lock()


def T(key):
    return TUNE[key]


def load_tune():
    try:
        with open(TUNE_FILE) as f:
            saved = json.load(f)
        for k, v in saved.items():
            if k in PARAM_META:
                TUNE[k] = _clean(k, v)
    except FileNotFoundError:
        pass
    except Exception as e:
        print(f"[WARN] could not read tune.json: {e}")


def _clean(key, value):
    _, _, default, lo, hi, _ = PARAM_META[key]
    v = float(value)
    if key.endswith("_sign"):
        return -1 if v < 0 else 1
    v = max(lo, min(hi, v))
    return int(round(v)) if isinstance(default, int) else round(v, 3)


def save_tune():
    try:
        with open(TUNE_FILE, "w") as f:
            json.dump(TUNE, f, indent=1)
    except Exception as e:
        print(f"[WARN] could not save tune.json: {e}")


# =====================================================================
# Arduino link - never blocks, reconnects, queues HOME until the board is ready
# =====================================================================
class Mount:
    def __init__(self, port):
        self.port = port
        self.ser = None
        self.connected = False
        self.opened_at = 0.0
        self.steps = {"x": None, "y": None}
        self.busy = {"x": False, "y": False}
        self.limit = False
        self.homing = False
        self.homed = False
        self.fault = None
        self.last_rx = 0.0
        self.status_ts = 0.0
        self.cmd_ts = {"x": 0.0, "y": 0.0}
        self.cmd_target = {"x": None, "y": None}
        self.y_origin = None
        self.auto_home = True
        self.home_pending = False
        self.home_sent = 0.0
        self.home_tries = 0
        self._guard_until = 0.0
        self._last_status_req = 0.0
        self.speed_dirty = True
        self._tx = threading.Lock()
        self._stop = threading.Event()
        threading.Thread(target=self._run, daemon=True).start()

    # ---- state helpers ----
    @property
    def link_ok(self):
        return self.connected and (time.time() - self.last_rx) < 1.5

    def pos(self, axis):
        return self.steps[axis]

    def move_done(self, axis):
        """True when the last commanded move of this axis has finished (needs a status newer than the command)."""
        if self.status_ts <= self.cmd_ts[axis] or self.busy[axis]:
            return False
        if self.steps[axis] == self.cmd_target[axis]:
            return True
        return (time.time() - self.cmd_ts[axis]) > MOVE_TIMEOUT_S

    # ---- commands (all thread safe, none blocks on the Arduino) ----
    def request_home(self):
        self.fault = None
        self.auto_home = True
        self.home_pending = True
        self.home_tries = 0
        self.home_sent = 0.0

    def halt(self):
        self.auto_home = False
        self.home_pending = False
        self._send("STOP")

    def send_move(self, axis, target):
        target = int(target)
        if axis == "x":
            target = max(0, min(MAX_STEPS, target))
        self.cmd_ts[axis] = time.time()
        self.cmd_target[axis] = target
        return self._send(f"G {target}" if axis == "x" else f"GY {target}")

    def apply_speed(self):
        self.speed_dirty = True

    # ---- serial plumbing ----
    def _open(self):
        try:
            self.ser = serial.Serial(self.port, 115200, timeout=0.05, write_timeout=1)
        except Exception as e:
            self.ser = None
            self.connected = False
            self.fault = f"serial open failed: {e}" if self.fault is None else self.fault
            return False
        print(f"[mount] serial open on {self.port}")
        self.opened_at = time.time()          # opening the port resets an Uno; give it time to boot
        self.connected = True
        self.last_rx = 0.0
        self.status_ts = 0.0
        self.homed = self.homing = False
        self.busy = {"x": False, "y": False}
        self.y_origin = None
        self.fault = None
        self.speed_dirty = True
        return True

    def _lost(self, err):
        print(f"[mount] serial lost: {err}")
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.connected = False
        self.homed = self.homing = False

    def _send(self, text):
        ser = self.ser
        if ser is None:
            return False
        try:
            with self._tx:
                ser.write((text + "\n").encode("ascii"))
            return True
        except Exception as e:
            self._lost(e)
            return False

    def _run(self):
        buf = b""
        while not self._stop.is_set():
            if self.ser is None:
                if not self._open():
                    time.sleep(2.0)
                    continue
                buf = b""
            try:
                data = self.ser.read(256)
            except Exception as e:
                self._lost(e)
                continue
            if data:
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    self._parse(line.decode("ascii", "ignore").strip())
            try:
                self._housekeeping()
            except Exception as e:
                print(f"[mount] housekeeping error: {e}")

    def _housekeeping(self):
        now = time.time()
        if now - self.opened_at < 1.5:
            return
        if now - self.last_rx > 1.0 and now - self._last_status_req > 0.5:
            self._send("STATUS 1")
            self._last_status_req = now
        if not self.link_ok:
            return
        if self.speed_dirty:
            self.speed_dirty = False
            self._send(f"SPD {int(T('base_speed_us'))} {int(T('y_speed_us'))}")
        if (self.auto_home and not self.homed and not self.homing
                and not self.home_pending and self.fault is None):
            self.request_home()
        if self.home_pending and now - self.home_sent >= 1.0:
            if self.home_tries >= 5:
                self.home_pending = False
                return
            if self._send("HOME"):
                self.home_sent = now
                self.home_tries += 1
                self._guard_until = now + 0.12      # ignore stale status that was already in flight
                self.homed = False
                self.homing = True

    def _parse(self, line):
        p = line.split()
        if len(p) < 6 or p[0] != "S":
            return                                   # READY, partial lines, boot noise ...
        try:
            x, y = int(p[1]), int(p[2])
            limit = p[3] == "1"
            state = int(p[4])
            homed = p[5] == "1"
            bx = by = False
            if len(p) >= 8:
                bx, by = p[6] == "1", p[7] == "1"
        except ValueError:
            return
        now = time.time()
        self.steps["x"], self.steps["y"] = x, y
        self.busy["x"], self.busy["y"] = bx, by
        self.limit = limit
        self.last_rx = self.status_ts = now
        if self.y_origin is None:
            self.y_origin = y
        if now >= self._guard_until:
            self.homing = (state == 2)
            self.homed = homed
            if self.homing or self.homed:
                self.home_pending = False            # HOME confirmed by a fresh status
            if self.homed:
                self.fault = None


# =====================================================================
# Cameras - exactly ONE owner per camera device
# =====================================================================
def _placeholder(text):
    img = np.zeros((FRAME_H, FRAME_W, 3), dtype=np.uint8)
    cv2.putText(img, text, (30, FRAME_H // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 255), 2, cv2.LINE_AA)
    ok, jpg = cv2.imencode(".jpg", img)
    return jpg.tobytes()


class CameraWorker(threading.Thread):
    """Owns one VideoCapture. Reads, runs the tracker, draws the overlay and stores the latest result.
    The web stream and the motor controllers only READ what this thread publishes, so the device is
    never opened twice (that was the cause of the black / missing Camera 2)."""

    def __init__(self, name, index, make_tracker, overlay, start_delay=0.0):
        super().__init__(daemon=True)
        self.name_ = name
        self.index = index
        self.make_tracker = make_tracker
        self.overlay = overlay
        self.start_delay = start_delay
        self.lock = threading.Lock()
        self.result = None
        self.result_ts = 0.0
        self.jpeg = None
        self.status = "starting"
        self._reset = False
        self._reopen = False
        self._ph = {}

    # ---- API used by other threads ----
    def latest(self):
        with self.lock:
            return self.result, self.result_ts

    def healthy(self):
        return self.status == "ok" and (time.time() - self.result_ts) < 1.5

    def request_reset(self):
        self._reset = True

    def request_reopen(self):
        self._reopen = True

    def frame_jpeg(self):
        with self.lock:
            if self.jpeg:
                return self.jpeg
        label = f"{self.name_.upper()} {self.status.upper()}..."
        if label not in self._ph:
            self._ph[label] = _placeholder(label)
        return self._ph[label]

    # ---- thread ----
    def _open(self):
        cap = cv2.VideoCapture(self.index, cv2.CAP_V4L2) if sys.platform.startswith("linux") \
            else cv2.VideoCapture(self.index)
        if not cap.isOpened():
            cap.release()
            return None
        if FORCE_MJPG:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        for _ in range(WARMUP_FRAMES):               # first frames are often black / half exposed
            cap.read()
        return cap

    def run(self):
        time.sleep(self.start_delay)
        while True:
            self.status = "opening"
            cap = self._open()
            if cap is None:
                self.status = "offline"
                self.jpeg = None
                time.sleep(2.0)
                continue
            try:
                self._loop(cap, self.make_tracker())
            except Exception as e:
                print(f"[{self.name_}] camera loop error: {e}")
            finally:
                cap.release()
            self.status = "restarting"
            self.jpeg = None
            time.sleep(1.0)

    def _loop(self, cap, tracker):
        bad = 0
        black_since = None
        while True:
            if self._reopen:
                self._reopen = False
                print(f"[{self.name_}] reopen requested")
                return
            t0 = time.time()
            ok, frame = cap.read()
            if not ok or frame is None:
                bad += 1
                if bad >= 30:
                    print(f"[{self.name_}] no frames, reopening")
                    return
                time.sleep(0.05)
                continue
            bad = 0
            now = time.time()
            if int(frame.max()) <= BLACK_MAX:
                black_since = black_since or now
                if now - black_since > BLACK_SECONDS:
                    print(f"[{self.name_}] black frames for {BLACK_SECONDS}s, reopening")
                    return
            else:
                black_since = None
            if self._reset:
                tracker.reset()
                self._reset = False
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            result = dict(tracker.update(frame))
            result["fresh"] = tracker.frames_since_detection == 0      # a real measurement, not a Kalman guess
            annotated = self.overlay(frame, result, tracker)
            ok, jpg = cv2.imencode(".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            with self.lock:
                self.result, self.result_ts = result, now
                if ok:
                    self.jpeg = jpg.tobytes()
            self.status = "ok"
            time.sleep(max(0.0, 0.04 - (time.time() - t0)))            # ~25 fps max


def _make_tracker1():
    return SunTracker(roi=CAM1_ROI, target_point=CAM1_TARGET, debug=False)


def _make_tracker2():
    return SunTracker(roi=None, debug=False)


def _overlay1(frame, result, tracker):
    return tracker.draw_overlay(frame, result)


def _overlay2(frame, result, tracker):
    return draw_overlay_cam2(frame, result, tracker)


# =====================================================================
# One controller per axis - completely independent
# =====================================================================
class AxisController(threading.Thread):
    def __init__(self, label, axis, cam, err_key, prefix, bounds):
        super().__init__(daemon=True)
        self.label, self.axis, self.cam = label, axis, cam
        self.err_key, self.p = err_key, prefix
        self.bounds = bounds                  # callable -> (lo, hi) in steps, or None if not known yet
        self.enabled = True
        self.restart_flag = False
        self.state = "STARTING"
        self.err = None
        self.sun = False
        self.phase = "GO_REF"
        self._reset_internal()

    def _reset_internal(self):
        self.moving = False
        self.measure_after = 0.0
        self.last_ts = 0.0
        self.samples = []
        self.last_seen = 0.0
        self.locked = False
        self.dir = 1
        self.scale = 1.0
        self.last_sign = 0
        self.phase = "GO_REF" if self.axis == "x" else "RUN"
        self.state = "GOING TO 90" if self.axis == "x" else "STARTING"

    def restart(self):
        self.restart_flag = True

    def t(self, name):
        return T(f"{self.p}_{name}")

    def run(self):
        while True:
            time.sleep(0.03)
            try:
                self.step()
            except Exception as e:
                self.state = f"ERROR {e}"

    def _move(self, target):
        mount.send_move(self.axis, target)
        self.moving = True

    def step(self):
        m = mount
        if not self.enabled:
            self.state = "STOPPED"
            return
        if self.restart_flag:
            self.restart_flag = False
            self._reset_internal()
        if not m.link_ok or m.pos(self.axis) is None:
            self.state = "NO LINK"
            return
        if self.axis == "x" and not m.homed:
            self.state = "HOMING" if (m.homing or m.home_pending) else "NOT HOMED"
            self._reset_internal()
            return
        if self.axis == "y" and getattr(base_ctl, "state", "SEARCHING") not in ("SEARCHING", "TRACKING", "LOCKED", "SCAN_LIMIT", "AT LIMIT"):
            self.state = "WAITING FOR BASE"
            self.moving = False
            return
        if self.axis == "x" and m.homed and self.phase not in ("GO_REF", "RUN", "SEARCHING", "TRACKING", "LOCKED"):
            self.phase = "GO_REF"
            self.state = "GOING TO 90"
        b = self.bounds()
        if b is None:
            self.state = "WAIT"
            return
        lo, hi = b
        now = time.time()

        if self.moving:                       # let the last move finish before looking again
            if not m.move_done(self.axis):
                return
            self.moving = False
            self.measure_after = now + (SEARCH_SETTLE_S if self.state == "SEARCHING" else SETTLE_S)
            self.cam.request_reset()          # drop the Kalman lag: next detection is a clean measurement
            self.samples.clear()

        if self.phase == "GO_REF":            # base only: home -> move to 90° reference -> then scan 90°..150°
            pos0 = m.pos(self.axis)
            if abs(pos0 - REFERENCE_STEPS) <= 3:
                self.phase = "RUN"
                self.dir = 1                  # after the 90° reference, continue in the same + direction to 150°
                self.measure_after = now + SETTLE_S
                self.cam.request_reset()
                self.state = "SEARCHING"
            else:
                self.state = "GOING TO 90"
                self._move(REFERENCE_STEPS)
            return

        if not self.cam.healthy():
            self.sun = False
            if m.link_ok and m.homed and self.phase == "RUN":
                try:
                    lo, hi = self.bounds()
                except Exception:
                    lo = hi = None
                if lo is not None and hi is not None:
                    self.state = "SEARCHING"
                    self.err = None
                    self._search(m.pos(self.axis), lo, hi)
                    return
            self.state = "CAMERA OFFLINE"
            return
        res, ts = self.cam.latest()
        if res is None or ts <= self.last_ts or ts < self.measure_after:
            # no fresh data yet: keep scanning if the axis is in run mode and has no sun lock
            if m.link_ok and m.homed and self.phase == "RUN" and now - self.last_seen > LOST_S:
                try:
                    lo, hi = self.bounds()
                except Exception:
                    lo = hi = None
                if lo is not None and hi is not None:
                    self.state = "SEARCHING"
                    self._search(m.pos(self.axis), lo, hi)
            return
        self.last_ts = ts

        fresh = bool(res.get("detected")) and bool(res.get("fresh"))
        self.sun = fresh
        if fresh:
            self.last_seen = now
            self.samples.append(float(res[self.err_key]))
            self.samples = self.samples[-SAMPLES:]
        else:
            self.samples.clear()

        pos = m.pos(self.axis)

        if len(self.samples) >= SAMPLES:
            self._track(pos, lo, hi, sum(self.samples) / len(self.samples))
        elif now - self.last_seen > LOST_S:
            self._search(pos, lo, hi)
        # else: sun seen a moment ago, wait for fresh frames

    def _track(self, pos, lo, hi, err):
        self.err = err
        a = abs(err)
        if self.locked:
            if a <= self.t("tol_out"):
                self.state = "LOCKED"
                self.samples.clear()
                return
            self.locked = False
        elif a <= self.t("tol_in"):
            self.locked = True
            self.scale, self.last_sign = 1.0, 0
            self.state = "LOCKED"
            self.samples.clear()
            return

        sign = 1 if err > 0 else -1
        if self.last_sign and sign != self.last_sign:
            self.scale = max(0.25, self.scale * 0.5)      # crossed the line: shrink moves, no ping-pong
        self.last_sign = sign

        step = int(round(err * self.t("gain") * self.scale * self.t("sign")))
        mag = max(1, int(round(self.t("min_step") * self.scale)))
        mag = max(mag, abs(step))
        mag = min(mag, int(self.t("max_step")))
        direction = (1 if err > 0 else -1) * self.t("sign")
        target = max(lo, min(hi, pos + direction * mag))
        if target == pos:
            self.state = "AT LIMIT"
            self.samples.clear()
            return
        self.state = "TRACKING"
        self.samples.clear()
        self._move(target)

    def _search(self, pos, lo, hi):
        self.err = None
        self.locked = False
        self.scale, self.last_sign = 1.0, 0

        # Fixed scan rule for this rig: after homing and moving to +90°, keep moving in the
        # same + direction until the +150° end of the active window. Do not reverse mid-scan.
        if self.dir > 0 and pos >= hi:
            self.state = "SCAN_LIMIT"
            return
        if self.dir < 0 and pos <= lo:
            self.state = "SCAN_LIMIT"
            return

        target = max(lo, min(hi, pos + self.dir * int(self.t("search_chunk"))))
        if target == pos:
            self.state = "SCAN_LIMIT"
            return
        self.state = "SEARCHING"
        self._move(target)


# =====================================================================
# Shared objects
# =====================================================================
mount = Mount(SERIAL_PORT)
cams = {
    "cam1": CameraWorker("cam1", CAMERA_INDEXES["cam1"], _make_tracker1, _overlay1, 0.0),
    "cam2": CameraWorker("cam2", CAMERA_INDEXES["cam2"], _make_tracker2, _overlay2, CAM_STAGGER_S),
}
base_ctl = AxisController("BASE", "x", cams["cam1"], "error_y", "base",
                          lambda: (BASE_SCAN_MIN, BASE_SCAN_MAX))
y_ctl = AxisController("Y", "y", cams["cam2"], "error_x", "y",
                       lambda: None if mount.y_origin is None
                       else (mount.y_origin - int(T("y_range")), mount.y_origin + int(T("y_range"))))
controllers = (base_ctl, y_ctl)


def set_tracking(on):
    for c in controllers:
        if on and not c.enabled:
            c.restart()                       # START after STOP: redo go-to-90 + scan
        c.enabled = on


# =====================================================================
# Flask
# =====================================================================
app = Flask(__name__)


def _mjpeg(name):
    cam = cams[name]
    last = None
    while True:
        jpg = cam.frame_jpeg()
        if jpg is not last:
            last = jpg
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
        time.sleep(0.04)


def build_status():
    m = mount
    steps = m.steps["x"]
    if not m.connected:
        status = "NO CONTROLLER (reconnecting...)"
    elif not m.link_ok:
        status = "WAITING FOR ARDUINO"
    elif m.fault:
        status = m.fault
    elif m.homing:
        status = "HOMING"
    elif m.home_pending:
        status = "HOME QUEUED"
    elif not m.homed:
        status = "NOT HOMED"
    elif base_ctl.enabled:
        status = f"TRACKING (base {base_ctl.state}, Y {y_ctl.state})"
    else:
        status = "IDLE"

    mirror_angle = sun_angle = None
    if steps is not None:
        mirror_angle = (steps - REFERENCE_STEPS) * DEG_PER_STEP
        sun_angle = REFERENCE_SUN_ANGLE + SUN_ANGLE_SIGN * 2.0 * mirror_angle

    def ax(c):
        return {"state": c.state, "phase": c.phase, "sun": bool(c.sun), "err": None if c.err is None else round(c.err, 1),
                "cam": c.cam.status}

    return {
        "steps": steps,
        "y_steps": m.steps["y"],
        "mirror_angle": mirror_angle,
        "sun_angle": sun_angle,
        "tracking": base_ctl.enabled,
        "limit_switch": bool(m.limit) if m.link_ok else None,
        "status": status,
        "base": ax(base_ctl),
        "yaxis": ax(y_ctl),
    }


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/status")
def api_status():
    return jsonify(build_status())


@app.route("/video_feed/<source_name>")
def video_feed(source_name):
    if source_name not in cams:
        return "Unknown camera", 404
    return Response(_mjpeg(source_name), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/api/home", methods=["POST"])
def api_home():
    # Never fails: if the Arduino is still booting the request is queued and runs as soon as it is ready.
    set_tracking(True)                       # home, then go to 90 deg and scan automatically
    for c in controllers:
        c.restart()
    mount.request_home()
    return jsonify(ok=True, queued=not mount.link_ok)


@app.route("/api/start", methods=["POST"])
def api_start():
    set_tracking(True)
    mount.fault = None
    mount.auto_home = True
    if mount.homed:
        for c in controllers:
            c.restart()                       # already homed: go to 90 deg and scan 90 -> 150
    else:
        mount.request_home()                  # not homed: home first, then 90 deg + scan automatically
    return jsonify(ok=True)


@app.route("/api/stop", methods=["POST"])
def api_stop():
    set_tracking(False)
    mount.halt()
    return jsonify(ok=True)


@app.route("/api/camera/<name>/restart", methods=["POST"])
def api_camera_restart(name):
    if name not in cams:
        return jsonify(ok=False, error="unknown camera"), 404
    cams[name].request_reopen()
    return jsonify(ok=True)


@app.route("/api/tune", methods=["GET", "POST"])
def api_tune():
    if request.method == "POST":
        data = request.get_json(silent=True) or {}
        with tune_lock:
            for k, v in data.items():
                if k in PARAM_META:
                    try:
                        TUNE[k] = _clean(k, v)
                    except (TypeError, ValueError):
                        pass
            save_tune()
        mount.apply_speed()
    return jsonify(ok=True, params=[
        {"key": k, "label": lab, "value": TUNE[k], "min": lo, "max": hi, "step": st}
        for k, lab, _d, lo, hi, st in PARAMS])


def _shutdown():
    set_tracking(False)
    try:
        mount.halt()
    except Exception:
        pass


if __name__ == "__main__":
    load_tune()
    atexit.register(_shutdown)
    for c in cams.values():
        c.start()
    for c in controllers:
        c.start()
    app.run(host=HOST, port=PORT, threaded=True, use_reloader=False)