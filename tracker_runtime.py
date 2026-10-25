"""
tracker_runtime.py  -  the real tracking loop (cameras + steppers + IMU) running in a background thread
=======================================================================================================
This is app.py without the OpenCV windows. It reuses app.py's settings and helpers, so there is ONE place
to tune gains, camera indexes, the rectangle and the line. The Flask dashboard only reads the newest
snapshot and sends commands; the serial port is touched by exactly one thread (the one in here).

    runtime = TrackerRuntime()
    runtime.start()
    runtime.snapshot()                 # dict with all live data (motors, switches, IMU, cameras, sextant ...)
    runtime.submit("home")             # commands from the web page are queued and run inside the loop
    runtime.mjpeg_frames("cam1")       # generator of JPEG frames for <img src="/video/cam1">
"""

import math
import queue
import threading
import time
from collections import deque

import cv2
import numpy as np

import app as tracker_app                       # existing tracking program: settings + helpers are reused
import sextant
from stepper_link import (
    DEGREES_PER_STEP, X_MAX_DEG, X_MAX_STEPS, X_MIN_STEPS, X_SWITCH_DEG,
    Y_MAX_DEG, Y_MIN_STEPS, Y_SWITCH_DEG,
    StepperError, StepperLink, StepperReset,
)
from sun_controller import AxisController, LimitAwareController, XScanController, NO_SUN
from suntracker import SunDetector, draw_overlay, roi_center

# ----------------------------------------------------------------------
# CONFIG
# ----------------------------------------------------------------------
LOOP_PERIOD_S = 0.04              # tracking loop ~25 Hz
SWITCH_POLL_S = 1.0               # ask the Arduino for the limit switch states this often
GIMBAL_STATUS_POLL_S = 2.0        # ask for the full gimbal status line (includes I2C fault count)
JPEG_QUALITY = 70
JPEG_PERIOD_S = 0.066             # video frame rate ~15 fps
IMU_STALE_S = 1.0                 # IMU sample older than this = "no data"
GIMBAL_DEADBAND_DEG = 2.0         # same as ROLL_DEADBAND / PITCH_DEADBAND in the Arduino sketch
MAX_NUDGE_STEPS = 400

# Camera 1: how many pixels the sun image moves per motor step. None = derive it from the controller gain
# (app.py's calibration key 'c' sets X_GAIN = 0.8 / |px_per_step| and X_SIGN = -sign(px_per_step)).
X_PX_PER_STEP = None

# ---- X-axis sweep scan (when the sun is not detected) ----
X_SCAN_START_DEG    = 15.0   # sweep start angle (degrees) — sun altitude 0..90 deg needs X >= 15 deg
X_SCAN_END_DEG      = 60.0   # sweep end angle   (degrees) — 60° = parallel to horizon mirror
X_SCAN_STEP_DEG     = 10.0   # step size between scan positions (10° per step)
X_SCAN_DWELL_FRAMES = 4      # frames to observe at each position before moving on
X_SCAN_NO_SUN_S     = 3.0    # seconds without detecting the sun before a sweep starts
X_SCAN_SETTLE_S     = 0.5    # seconds to wait for the motor to settle at each scan position
# Minimum steps allowed during TRACKING = 15° (X_SCAN_START_DEG).
# stepper_link soft limit allows down to step 10 (-58.875°), but tracking must stay ≥ 15°.
# Uses the already-imported X_SWITCH_DEG and DEGREES_PER_STEP from stepper_link.
X_TRACK_MIN_STEPS = int(round((X_SCAN_START_DEG - X_SWITCH_DEG) / DEGREES_PER_STEP))  # = 667 steps = 15°
# ----------------------------------------------------------------------


def x_px_per_step():
    if X_PX_PER_STEP is not None:
        return X_PX_PER_STEP
    gain = tracker_app.X_GAIN_STEPS_PER_PX
    return -tracker_app.X_SIGN * 0.8 / gain if gain else None


class DashboardLink(StepperLink):
    """StepperLink that also understands the gimbal / IMU lines the Arduino prints.

    All serial/homing output goes directly to the terminal via StepperLink._log (print).
    No dashboard event routing here — the dashboard reads self._events which the
    TrackerRuntime populates independently.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.gimbal = {"active": None, "imu_ok": None, "roll": None,
                       "roll_cmd": None, "i2c_faults": None, "pitch": None, "yaw": None, "updated": None}
        self.switches = {"x_home": None, "y_home": None, "y_far": None, "updated": None}

    def _note_line(self, line):
        if line.startswith("IMU,"):
            parts = line.split(",")
            if len(parts)==4:
                try:
                    roll, pitch, yaw = map(float, parts[1:])
                except ValueError:
                    return
                if all(math.isfinite(v) for v in (roll,pitch,yaw)):
                    self.gimbal.update(roll=roll, pitch=pitch, yaw=yaw,
                                      imu_ok=True, active=True, updated=time.time())
            elif line == "IMU,ERR":
                self.gimbal.update(imu_ok=False, updated=None)
            return
        if line.startswith("GIMBAL"):
            self._parse_gimbal(line)
            return
        super()._note_line(line)

    def _parse_gimbal(self, line):
        # "GIMBAL 0|1"                                   reply to G0 / G1
        # "GIMBAL 1 1 roll rateA"                        stream line (GP1)  -- Motor A / roll only
        # "GIMBAL on imuOk roll rateA [faults]"          status line (GS)
        parts = line.split()
        if parts[0] != "GIMBAL":                          # GIMBAL_READY and similar
            return
        g = self.gimbal
        if len(parts) == 2 and parts[1] in ("0", "1"):
            g["active"] = parts[1] == "1"
            return
        if len(parts) >= 5:
            try:
                vals = [float(p) for p in parts[1:]]
            except ValueError:
                return
            if not all(math.isfinite(v) for v in vals):
                return
            g.update(active=bool(vals[0]), imu_ok=bool(vals[1]), roll=vals[2],
                     roll_cmd=vals[3], updated=time.time())
            if len(vals) >= 5:
                g["i2c_faults"] = int(vals[4])

    def set_gimbal_stream(self, on):
        self._send("GP1" if on else "GP0")

    def set_gimbal(self, on):
        self._send("G1" if on else "G0")

    def request_gimbal_status(self):
        self._send("GS")

    def request_imu(self):
        self._send("I")


class TrackerRuntime:
    def __init__(self, serial_port=None, enable_steppers=None, enable_camera2=None, verbose_serial=True):
        cfg = tracker_app
        self.serial_port = serial_port or cfg.SERIAL_PORT
        self.enable_steppers = cfg.ENABLE_STEPPERS if enable_steppers is None else enable_steppers
        self.enable_camera2 = cfg.ENABLE_CAMERA2 if enable_camera2 is None else enable_camera2
        self.verbose_serial = verbose_serial

        self._stop = threading.Event()
        self._thread = None
        self._cmds = queue.Queue()
        self._events = deque(maxlen=80)
        self._snapshot = {"ready": False, "timestamp": time.time()}
        self._jpeg_cond = threading.Condition()
        self._jpeg = {"cam1": (0, None), "cam2": (0, None)}
        self._clients = 0
        self._clients_lock = threading.Lock()

        self.cam1 = self.cam2 = None
        self.camera_errors = {"cam1": None, "cam2": None}
        self.link = None                  # DashboardLink, only set once it is connected AND homed
        self._link_obj = None             # the same object while it is still connecting
        self._link_thread = None
        self.link_state = "DISABLED" if not self.enable_steppers else "CONNECTING"
        self.link_error = None
        self.fatal = None

        self.x_tracking = cfg.START_TRACKING
        self.y_tracking = cfg.START_Y_TRACKING
        self.x_note = ""
        self.y_note = ""
        self._last_notes = {"x": "", "y": ""}
        self.xctrl = self.yctrl = None
        self.det1 = self.det2 = None
        self.target = None
        self.line = None
        self.c1 = {}                      # newest Camera 1 measurements
        self.c2 = {}
        self.loop_hz = 0.0
        self.t_start = time.time()

    # ------------------------------------------------------------------
    # public API
    # ------------------------------------------------------------------
    def start(self):
        self.t_start = time.time()
        self._thread = threading.Thread(target=self._run, name="tracker", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        with self._jpeg_cond:
            self._jpeg_cond.notify_all()
        if self._thread:
            self._thread.join(timeout=3.0)

    def snapshot(self):
        return self._snapshot

    def submit(self, name, args=None):
        """Validate a command from the web page and queue it. Returns (ok, message)."""
        args = args or {}
        needs_link = {"home", "nudge_x", "nudge_y", "goto_x", "gimbal"}
        if name == "gimbal":
            return False, "Gimbal is autonomous on Arduino Uno"
        if name in ("x_tracking", "y_tracking"):
            if not isinstance(args.get("enabled"), bool):
                return False, "'enabled' (true/false) is required"
        elif name in ("nudge_x", "nudge_y"):
            steps = args.get("steps")
            if isinstance(steps, bool) or not isinstance(steps, int) or steps == 0 or abs(steps) > MAX_NUDGE_STEPS:
                return False, f"'steps' must be a non-zero integer, at most {MAX_NUDGE_STEPS}"
        elif name == "goto_x":
            deg = args.get("deg")
            if isinstance(deg, bool) or not isinstance(deg, (int, float)) or not math.isfinite(deg):
                return False, "'deg' must be a number"
        elif name == "home":
            pass
        elif name == "reconnect":
            if self.link is not None or (self._link_thread and self._link_thread.is_alive()):
                return False, "Arduino is already connected or connecting"
            if not self.enable_steppers:
                return False, "steppers are disabled (--no-steppers)"
        else:
            return False, f"unknown command '{name}'"
        if name in needs_link and self.link is None:
            return False, "Arduino is not connected / not homed yet"
        self._cmds.put((name, args))
        return True, "queued"

    def mjpeg_frames(self, name):
        """Generator of JPEG frames (bytes) for one camera ('cam1' / 'cam2')."""
        with self._clients_lock:
            self._clients += 1
        seq = 0
        try:
            while not self._stop.is_set():
                with self._jpeg_cond:
                    self._jpeg_cond.wait_for(lambda: self._jpeg[name][0] != seq or self._stop.is_set(), timeout=1.0)
                    s, data = self._jpeg[name]
                if data is None or s == seq:
                    continue
                seq = s
                yield data
        finally:
            with self._clients_lock:
                self._clients -= 1

    # ------------------------------------------------------------------
    # events
    # ------------------------------------------------------------------
    def _event(self, text, level="info"):
        self._events.append({"t": time.time(), "level": level, "text": str(text)})

    def _note_change(self, axis, text, level="warn"):
        if text != self._last_notes[axis]:
            self._last_notes[axis] = text
            if text:
                self._event(text, level)

    # ------------------------------------------------------------------
    # thread body
    # ------------------------------------------------------------------
    def _run(self):
        try:
            self._setup()
            self._loop()
        except Exception as e:                       # keep the web page alive and show what happened
            self.fatal = repr(e)
            self._event(f"Tracking loop crashed: {e!r}", "error")
            try:
                self._snapshot = self._build_snapshot()
            except Exception:
                self._snapshot = {"ready": False, "timestamp": time.time(), "error": self.fatal}
        finally:
            self._cleanup()

    def _open_camera(self, name, index, rotate, resize):
        reader = None
        try:
            reader = tracker_app.CameraReader(index, rotate, name, resize=resize)
            tracker_app.wait_for_frame(reader)
            self.camera_errors[name] = None
            self._event(f"{name} opened (index {index})")
            return reader
        except Exception as e:
            if reader:
                reader.release()
            self.camera_errors[name] = str(e)
            self._event(f"{name}: {e}", "error")
            return None

    def _setup(self):
        cfg = tracker_app
        self.cam1 = self._open_camera("cam1", cfg.CAMERA1_INDEX, cfg.ROTATE1_180, None)
        if self.enable_camera2:
            self.cam2 = self._open_camera("cam2", cfg.CAMERA2_INDEX, cfg.ROTATE2_180, cfg.CAMERA2_SIZE)
        self.det1 = SunDetector(roi=cfg.TARGET_ROI)
        self.target = cfg.TARGET_POINT if cfg.TARGET_POINT else roi_center(cfg.TARGET_ROI)
        self.det2 = SunDetector(roi=None)
        self.line = (cfg.LINE_START, cfg.LINE_END)
        _xaxis = AxisController(gain=cfg.X_GAIN_STEPS_PER_PX, sign=cfg.X_SIGN,
                                deadband_px=cfg.ALLOWED_ERROR_PX,
                                max_step=cfg.X_MAX_STEP, settle_s=cfg.X_SETTLE_S,
                                confirm_s=cfg.X_CONFIRM_S)
        self.xctrl = XScanController(
            _xaxis,
            scan_start_deg=X_SCAN_START_DEG,
            scan_end_deg=X_SCAN_END_DEG,
            scan_step_deg=X_SCAN_STEP_DEG,
            dwell_frames=X_SCAN_DWELL_FRAMES,
            no_sun_timeout_s=X_SCAN_NO_SUN_S,
            move_settle_s=X_SCAN_SETTLE_S,
        )
        self.yctrl = LimitAwareController(
            gain=cfg.Y_GAIN_STEPS_PER_PX, sign=cfg.Y_SIGN, deadband_px=cfg.ALLOWED_LINE_ERROR_PX,
            max_step=cfg.Y_MAX_STEP, settle_s=cfg.Y_SETTLE_S, confirm_s=cfg.Y_CONFIRM_S,
            search_step=cfg.Y_SEARCH_STEP, search_interval_s=cfg.Y_SEARCH_INTERVAL_S,
            max_search_reversals=cfg.Y_MAX_SEARCH_REVERSALS, max_reversals=cfg.Y_MAX_REVERSALS,
            single_limit_mode=True)   # Y has only ONE limit switch and circles 360°
        if self.enable_steppers:
            self._start_link_worker()

    def _cleanup(self):
        for cam in (self.cam1, self.cam2):
            if cam:
                cam.release()
        link = self.link or self._link_obj
        if link:
            link.close()

    # ------------------------------------------------------------------
    # Arduino connection (own thread: homing takes a while and the cameras should keep running)
    # ------------------------------------------------------------------
    def _start_link_worker(self):
        self._link_thread = threading.Thread(target=self._link_worker, name="arduino-connect", daemon=True)
        self._link_thread.start()

    def _link_worker(self):
        cfg = tracker_app
        self.link_error = None
        self.link_state = "HOMING"
        link = DashboardLink(port=self.serial_port, verbose=self.verbose_serial)
        self._link_obj = link
        self._event(f"Connecting to the Arduino on {self.serial_port} - it homes first, this takes a while")
        print(f"[tracker] Connecting to Arduino on {self.serial_port} — homing in progress, watch terminal for details")
        try:
            link.connect()
            link.goto_x_deg(cfg.X_START_DEG)
            link.set_gimbal_stream(True)
            link.request_gimbal_status()
        except (StepperError, OSError, AttributeError) as e:     # AttributeError: port closed while connecting (shutdown)
            self.link_state = "OFFLINE"
            self.link_error = str(e)
            self._event(f"Arduino connection failed: {e}", "error")
            link.close()
            return
        self.link = link
        self.link_state = "READY"
        # Update the controller with the actual measured travel (y_range_steps)
        if link.y_range_steps:
            self.yctrl.full_rotation_steps = link.y_range_steps
        self._event(f"Arduino ready. {link.status_text()}")

    def _recover(self, why):
        """The Arduino restarted: wait until it has homed again, go back to X_START_DEG, restart the IMU stream."""
        link = self.link
        self.link_state = "HOMING"
        self._event(f"Arduino restarted ({why}) - waiting for it to home again", "warn")
        ok = False
        try:
            link.wait_ready()
            link.goto_x_deg(tracker_app.X_START_DEG)
            link.set_gimbal_stream(True)
            link.request_gimbal_status()
            ok = True
        except StepperError as e:
            self.link_error = str(e)
            self._event(f"Recovery failed: {e}", "error")
        self.link_state = "READY" if ok else "ERROR"
        self.xctrl.reset()
        self.yctrl.reset()
        self.x_tracking = self.x_tracking and ok
        self.y_tracking = self.y_tracking and ok
        if ok:
            self._event(f"Recovered after restart #{link.reset_count}. {link.status_text()}")
        return ok

    def _hw(self, fn, *args, critical=True):
        """Run a serial call. Handles Arduino restarts and errors; returns None if it failed."""
        try:
            return fn(*args)
        except StepperReset:
            self._recover("during a command")
        except StepperError as e:
            self._event(f"Stepper error: {e}", "error")
            if critical:
                self.x_tracking = self.y_tracking = False
        return None

    # ------------------------------------------------------------------
    # commands from the web page
    # ------------------------------------------------------------------
    def _drain_commands(self):
        while True:
            try:
                name, args = self._cmds.get_nowait()
            except queue.Empty:
                return
            try:
                self._execute(name, args)
            except Exception as e:
                self._event(f"Command '{name}' failed: {e!r}", "error")

    def _execute(self, name, args):
        link = self.link
        if name == "x_tracking":
            self.x_tracking = args["enabled"]
            self.xctrl.reset()
            self._event(f"X tracking {'ON' if self.x_tracking else 'OFF'}")
        elif name == "y_tracking":
            self.y_tracking = args["enabled"]
            self.yctrl.reset()
            self._event(f"Y tracking {'ON' if self.y_tracking else 'OFF'}")
        elif name == "gimbal" and link:
            self._hw(link.set_gimbal, args["enabled"], critical=False)
            self._hw(link.request_gimbal_status, critical=False)
            self._event(f"Gimbal {'ON' if args['enabled'] else 'OFF'} requested")
        elif name == "home" and link:
            self.link_state = "HOMING"
            self._event("Homing both motors ...")
            if self._hw(link.home) is not None or link.homed:
                self._hw(link.goto_x_deg, tracker_app.X_START_DEG)
                self._hw(link.set_gimbal_stream, True, critical=False)
            self.xctrl.reset()
            self.yctrl.reset()
            self.link_state = "READY" if link.homed else "ERROR"
            self._event(f"Homed. {link.status_text()}")
        elif name == "nudge_x" and link:
            self._hw(link.move_steps, args["steps"], 0)
            self.xctrl.reset()
            self._event(f"Nudged X {args['steps']:+d} steps -> {link.status_text()}")
        elif name == "nudge_y" and link:
            self._hw(link.move_steps, 0, args["steps"])
            self.yctrl.reset()
            self._event(f"Nudged Y {args['steps']:+d} steps -> {link.status_text()}")
        elif name == "goto_x" and link:
            self._hw(link.goto_x_deg, float(args["deg"]))
            self.xctrl.reset()
            self._event(f"X moved to {link.x_deg:+.2f} deg (asked {float(args['deg']):+.2f})")
        elif name == "reconnect":
            if self.link is None and not (self._link_thread and self._link_thread.is_alive()):
                self._start_link_worker()

    # ------------------------------------------------------------------
    # main loop
    # ------------------------------------------------------------------
    def _read(self, attr, name):
        reader = getattr(self, attr)
        if reader is None:
            return None
        if not reader.alive:
            self.camera_errors[name] = "camera stopped delivering frames"
            self._event(f"{name} stopped delivering frames", "error")
            reader.release()
            setattr(self, attr, None)
            return None
        return reader.read()

    def _loop(self):
        last_switch = last_gs = last_jpeg = 0.0
        fps_t, fps_n = time.time(), 0
        while not self._stop.is_set():
            t_loop = time.time()
            self._drain_commands()
            link = self.link
            frame1 = self._read("cam1", "cam1")
            frame2 = self._read("cam2", "cam2") if self.enable_camera2 else None

            # ---- Arduino restart check ----
            if link:
                try:
                    if link.poll():
                        self._recover("detected while idle")
                except StepperError as e:
                    self._event(f"Stepper error: {e}", "error")
                    self.x_tracking = self.y_tracking = False

            # ---- Camera 1 + stepper X ----
            res1 = ex = ey = None
            centred = False
            if frame1 is not None:
                res1, ex, ey, centred = tracker_app.process_cam1(frame1, self.det1, self.target)
            detected1 = bool(res1 and res1["detected"])
            conf1 = float(res1["confidence"]) if detected1 else 0.0
            cur_x_deg = link.x_deg if link and link.homed else 0.0
            if link and self.x_tracking:
                action, value = self.xctrl.update(
                    ex, detected1, conf1, cur_x_deg)
                if action == "ctrl" and value:
                    # --- Tracking floor: never let X go below 0° (X_TRACK_MIN_STEPS) ---
                    if link.pos_x < X_TRACK_MIN_STEPS:
                        # Motor is already below the working range — drive back to X_START_DEG
                        self._event(
                            f"X below working range ({link.x_deg:.1f}°) — recovering to {tracker_app.X_START_DEG:.1f}°",
                            "warn")
                        self._hw(link.goto_x_deg, tracker_app.X_START_DEG)
                        self.xctrl.reset()
                        self.x_note = "X BELOW RANGE — returned to 0°"
                        self._note_change("x", self.x_note)
                    else:
                        # Clip a negative move that would cross the 0° floor
                        if value < 0 and (link.pos_x + value) < X_TRACK_MIN_STEPS:
                            value = X_TRACK_MIN_STEPS - link.pos_x   # move only to the floor (0 or small negative = stop)
                        if value:   # after clipping value may be 0 — skip the move
                            moved = self._hw(link.move_steps, value, 0)
                            if moved is not None:
                                note = "X AT SOFT LIMIT" if moved[0] == 0 else ""
                                if link.limit_hits:
                                    note = "X HIT LIMIT SWITCH — returning to 0°"
                                    self._event(f"X hit limit switch — recovering to {tracker_app.X_START_DEG:.1f}°", "warn")
                                    self._hw(link.goto_x_deg, tracker_app.X_START_DEG)
                                    self.xctrl.reset()
                                self.x_note = note
                                self._note_change("x", note)
                elif action == "goto":
                    # scan repositioning — blocking goto inside the loop
                    self._hw(link.goto_x_deg, float(value))
                    if self.xctrl.scan_state == "SCAN_SWEEP":
                        self.x_note = self.xctrl.state
                    elif self.xctrl.scan_state == "SCAN_RETURN":
                        self.x_note = self.xctrl.state
                        self._event(
                            f"X scan done — best position {self.xctrl.scan_best_deg:.1f}deg "
                            f"(conf {self.xctrl.scan_best_conf:.2f})", "info")
                    self._note_change("x", self.x_note)
            else:
                self.xctrl.reset()
                if not self.x_tracking:
                    self.x_note = ""

            # ---- Camera 2 + stepper Y ----
            res2 = dist = None
            on_line = False
            if frame2 is not None:
                res2, dist, on_line = tracker_app.process_cam2(frame2, self.det2, self.line)
            detected2 = bool(res2 and res2["detected"])
            if link and self.y_tracking:
                # -- handle pending full-rotation from limit-switch wrap-around --
                rot = self.yctrl.pending_rotation_steps
                if rot:
                    self.yctrl.pending_rotation_steps = 0
                    self._event(
                        f"Y limit hit \u2013 doing full forward {abs(rot)}-step rotation "
                        f"(360\u00b0 wrap) to reach sun from other side", "warn"
                    )
                    moved_rot = self._hw(link.move_steps, 0, rot)
                    # After the rotation the controller is in post-ROTATING state;
                    # transition it to NO_SUN so it waits to re-acquire the sun.
                    self.yctrl.state = NO_SUN
                    self.yctrl._first_seen = None
                    self.y_note = self.yctrl.last_event
                    self._note_change("y", self.y_note)
                else:
                    steps_y = self.yctrl.update(dist, detected2)
                    if steps_y:
                        moved_y = self._hw(link.move_steps, 0, steps_y)
                        if moved_y is not None:
                            self.yctrl.notify_move_result(steps_y, moved_y[1], bool(link.limit_hits))
                            if link.limit_hits:
                                self._event(f"Y hit a limit switch {link.limit_hits}; controller: {self.yctrl.last_event}", "warn")
                            elif moved_y[1] == 0:
                                self._event(f"Y at its soft limit; controller: {self.yctrl.last_event}", "warn")
                    self.y_note = self.yctrl.last_event
                    self._note_change("y", self.y_note)
            else:
                self.yctrl.reset()
                self.y_note = ""

            self.c1 = {"res": res1, "ex": ex, "ey": ey, "centred": centred, "frame": frame1 is not None}
            self.c2 = {"res": res2, "dist": dist, "on_line": on_line, "frame": frame2 is not None}

            # ---- periodic polls (switches, gimbal status) ----
            now = time.time()
            link = self.link
            if link and link.homed:
                if now - last_switch >= SWITCH_POLL_S:
                    last_switch = now
                    sw = self._hw(link.limit_switches, critical=False)
                    if sw:
                        link.switches.update(x_home=sw[0], y_home=sw[1], y_far=sw[2], updated=time.time())
                if now - last_gs >= GIMBAL_STATUS_POLL_S:
                    last_gs = now
                    self._hw(link.request_gimbal_status, critical=False)

            # ---- video for the browser (only while somebody watches) ----
            if self._clients > 0 and now - last_jpeg >= JPEG_PERIOD_S:
                last_jpeg = now
                self._encode("cam1", self._annotate1(frame1) if frame1 is not None else None, self.camera_errors["cam1"])
                if self.enable_camera2:
                    self._encode("cam2", self._annotate2(frame2) if frame2 is not None else None, self.camera_errors["cam2"])

            fps_n += 1
            if now - fps_t >= 1.0:
                self.loop_hz = fps_n / (now - fps_t)
                fps_t, fps_n = now, 0
            self._snapshot = self._build_snapshot()

            time.sleep(max(0.002, LOOP_PERIOD_S - (time.time() - t_loop)))

    # ------------------------------------------------------------------
    # video
    # ------------------------------------------------------------------
    def _annotate1(self, frame):
        res = self.c1["res"]
        out = draw_overlay(frame, res, roi=self.det1.roi, target=self.target)
        cv2.line(out, (self.det1.roi[0], self.target[1]), (self.det1.roi[2], self.target[1]), (0, 0, 255), 1)
        h = out.shape[0]
        if res["detected"]:
            ex, ey = self.c1["ex"], self.c1["ey"]
            text, col = ("CENTRED", (0, 255, 0)) if self.c1["centred"] else (f"err x={ex:+d} y={ey:+d}", (0, 255, 255))
        else:
            text, col = "NO SUN IN RECTANGLE", (0, 0, 255)
        cv2.putText(out, text, (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2, cv2.LINE_AA)
        return out

    def _annotate2(self, frame):
        res = self.c2["res"]
        out = draw_overlay(frame, res, line=self.line)
        cv2.drawMarker(out, self.line[0], (0, 0, 255), cv2.MARKER_CROSS, 14, 2)
        h = out.shape[0]
        if res["detected"]:
            text, col = ("ON LINE", (0, 255, 0)) if self.c2["on_line"] else (f"dist to line {self.c2['dist']:+.0f}px", (0, 255, 255))
        else:
            text, col = "SUN NOT DETECTED", (0, 0, 255)
        cv2.putText(out, text, (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2, cv2.LINE_AA)
        return out

    def _encode(self, name, img, error):
        if img is None:
            img = np.zeros((480, 640, 3), np.uint8)
            cv2.putText(img, "CAMERA OFFLINE", (170, 230), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (80, 80, 255), 2, cv2.LINE_AA)
            if error:
                cv2.putText(img, str(error)[:60], (20, 270), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (160, 160, 160), 1, cv2.LINE_AA)
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ok:
            return
        with self._jpeg_cond:
            self._jpeg[name] = (self._jpeg[name][0] + 1, buf.tobytes())
            self._jpeg_cond.notify_all()

    # ------------------------------------------------------------------
    # snapshot (everything the dashboard shows)
    # ------------------------------------------------------------------
    @staticmethod
    def _pt(p):
        return None if p is None else [int(p[0]), int(p[1])]

    def _build_snapshot(self):
        cfg = tracker_app
        now = time.time()
        link = self.link
        lobj = link or self._link_obj
        homed = bool(link and link.homed)

        # ---- motors ----
        motor_x = {"homed": homed,
                   "steps": link.pos_x if homed else None,
                   "angle_deg": link.x_deg if homed else None,
                   "switch_deg": X_SWITCH_DEG, "max_deg": X_MAX_DEG,
                   "min_steps": X_MIN_STEPS, "max_steps": X_MAX_STEPS, "deg_per_step": DEGREES_PER_STEP}
        y_max_steps = lobj.y_max_steps if lobj else None
        motor_y = {"homed": homed,
                   "steps": link.pos_y if homed else None,
                   "angle_deg": link.y_deg if homed else None,
                   "switch_deg": Y_SWITCH_DEG, "max_deg": Y_MAX_DEG,
                   "min_steps": Y_MIN_STEPS, "max_steps": y_max_steps,
                   "travel_steps": lobj.y_range_steps if lobj else None,
                   "travel_deg": (lobj.y_range_steps * DEGREES_PER_STEP) if lobj and lobj.y_range_steps else None}

        # ---- limit switches ----
        sw = dict(lobj.switches) if lobj else {"x_home": None, "y_home": None, "y_far": None, "updated": None}
        sw["age_s"] = None if sw["updated"] is None else now - sw["updated"]

        # ---- IMU / gimbal ----
        g = dict(lobj.gimbal) if lobj else {}
        upd = g.get("updated")
        age = None if upd is None else now - upd
        fresh = age is not None and age <= IMU_STALE_S
        level = None
        if fresh and g.get("roll") is not None:
            level = abs(g["roll"]) <= GIMBAL_DEADBAND_DEG  # pitch (Stepper Z) removed; level based on roll only
        gimbal = {"active": g.get("active"), "imu_ok": g.get("imu_ok"), "fresh": fresh, "age_s": age,
                  "sample_time": upd, "roll_deg": g.get("roll") if fresh else None,
                  "pitch_deg": g.get("pitch") if fresh else None,
                  "yaw_deg": g.get("yaw") if fresh else None,
                  "roll_rate_cmd": None,
                  "i2c_faults": g.get("i2c_faults"), "deadband_deg": GIMBAL_DEADBAND_DEG, "level": level}

        # ---- cameras ----
        r1, r2 = self.c1.get("res"), self.c2.get("res")
        d1 = bool(r1 and r1["detected"])
        d2 = bool(r2 and r2["detected"])
        roi = self.det1.roi if self.det1 else None
        cam1 = {"online": self.cam1 is not None, "error": self.camera_errors["cam1"], "detected": d1,
                "center": self._pt(r1["center"]) if d1 else None,
                "radius_px": r1["radius"] if d1 else None,
                "confidence": r1["confidence"] if d1 else None,
                "brightness": r1["brightness"] if d1 else None,
                "error_x_px": self.c1.get("ex") if d1 else None,
                "error_y_px": self.c1.get("ey") if d1 else None,
                "centred": bool(self.c1.get("centred")) if d1 else False,
                "roi": list(roi) if roi else None, "target": self._pt(self.target) if self.target else None,
                "allowed_error_px": cfg.ALLOWED_ERROR_PX}
        cam2 = {"online": self.cam2 is not None, "enabled": self.enable_camera2, "error": self.camera_errors["cam2"],
                "detected": d2,
                "center": self._pt(r2["center"]) if d2 else None,
                "radius_px": r2["radius"] if d2 else None,
                "confidence": r2["confidence"] if d2 else None,
                "brightness": r2["brightness"] if d2 else None,
                "distance_px": self.c2.get("dist") if d2 else None,
                "on_line": bool(self.c2.get("on_line")) if d2 else False,
                "line": [self._pt(self.line[0]), self._pt(self.line[1])] if self.line else None,
                "allowed_error_px": cfg.ALLOWED_LINE_ERROR_PX}

        # ---- tracking state ----
        xs = self.xctrl.state if self.xctrl else None
        xsc = self.xctrl if self.xctrl else None
        # build scan position table (deg + best confidence recorded) for display
        scan_positions = None
        if xsc and xsc._scan_positions:
            scan_positions = [{"deg": round(p[0], 1), "conf": round(p[1], 3)} for p in xsc._scan_positions]
        tracking = {
            "x": {"enabled": self.x_tracking, "state": xs, "note": self.x_note,
                  "gain": cfg.X_GAIN_STEPS_PER_PX, "sign": cfg.X_SIGN, "px_per_step": x_px_per_step(),
                  "scan_state": xsc.scan_state if xsc else None,
                  "scan_best_deg": xsc.scan_best_deg if xsc else None,
                  "scan_best_conf": round(xsc.scan_best_conf, 3) if xsc else None,
                  "scan_positions": scan_positions,
                  "scan_idx": xsc._scan_idx if xsc and xsc._scan_positions else None},
            "y": {"enabled": self.y_tracking, "state": self.yctrl.state if self.yctrl else None,
                  "note": self.y_note, "gain": cfg.Y_GAIN_STEPS_PER_PX, "sign": cfg.Y_SIGN,
                  "reversals": self.yctrl.reversals if self.yctrl else None,
                  "max_reversals": cfg.Y_MAX_REVERSALS,
                  "search_reversals": self.yctrl.search_reversals if self.yctrl else None,
                  "max_search_reversals": cfg.Y_MAX_SEARCH_REVERSALS},
        }

        # ---- sextant (from motor X angle) ----
        sext = None
        if homed:
            sext = sextant.compute(
                link.x_deg, err_px=self.c1.get("ex") if d1 else None, px_per_step=x_px_per_step(),
                deg_per_step=DEGREES_PER_STEP, sun_detected=d1, locked=(xs == "LOCKED"))

        return {
            "ready": True,
            "timestamp": now,
            "uptime_s": now - self.t_start,
            "loop_hz": self.loop_hz,
            "link": {"state": self.link_state, "port": self.serial_port, "error": self.fatal or self.link_error,
                     "enabled": self.enable_steppers, "homed": homed,
                     "restarts": lobj.reset_count if lobj else 0},
            "motor_x": motor_x, "motor_y": motor_y, "switches": sw, "gimbal": gimbal,
            "sextant": sext, "tracking": tracking, "camera1": cam1, "camera2": cam2,
            "events": list(self._events)[-40:],
        }
