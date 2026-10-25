"""
app.py  -  Two cameras + stepper X (Camera 1) and stepper Y (Camera 2) tracking
====================================================================================
Camera 1 (X axis): sun inside the small rectangle -> stepper X turns until the sun is on the red target point
Camera 2 (Y axis): sun anywhere in the frame -> stepper Y turns until the sun is on the red line.
                   No sun: Y sweeps between its two limit switches (limited number of passes).
                   Limit hit / wrong direction: Y reverses (limited number of times), then STOPS - no endless back-and-forth.

Needs in the same folder: suntracker.py, stepper_link.py, sun_controller.py
Arduino: sun_tracker_arduino.ino uploaded; it homes both motors when this program connects.

Keys (click on a video window first):
    q  quit
    t  X tracking ON / OFF          u  Y tracking ON / OFF   (OFF also clears a STUCK / NOT_FOUND state)
    a / d  nudge X -/+ NUDGE_STEPS  (use these to bring the sun into the rectangle)
    j / l  nudge Y -/+ NUDGE_Y_STEPS
    c  calibrate X: moves a few steps, measures how far the sun moves, prints X_SIGN and X_GAIN to use
    v  calibrate Y the same way (sun must be visible in Camera 2)
    h  home both motors again, then go to X_START_DEG
    r  redraw Camera 1's rectangle with the mouse
    s  save the current frames
"""

import threading
import time

import cv2

from stepper_link import StepperError, StepperLink, StepperReset
from sun_controller import AxisController, LimitAwareController
from suntracker import (
    SunDetector,
    draw_overlay,
    error_to_point,
    roi_center,
    signed_distance_to_line,
)

# ----------------------------------------------------------------------
# CONFIG  (edit these)
# ----------------------------------------------------------------------
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
PRINT_EVERY_S = 0.5

# ---- Steppers / Arduino ----
ENABLE_STEPPERS = True              # False = cameras only (no serial port needed)
SERIAL_PORT = "/dev/ttyUSB0"        # e.g. "/dev/ttyUSB0", "/dev/ttyACM0", "COM3"
START_TRACKING = True               # start with X tracking ON
X_START_DEG = 15.0                  # after homing, X moves here

# ---- X axis control (Camera 1) ----
X_GAIN_STEPS_PER_PX = 0.15          # steps per pixel of error. Start gentle, then use key 'c'
X_SIGN = 1                          # +1 or -1. If X corrects the wrong way, flip it (key 'c' tells you)
X_MAX_STEP = 40                     # largest single correction in steps
X_SETTLE_S = 0.5                    # wait after each move so the image catches up
X_CONFIRM_S = 0.15                  # sun must be steady this long before the first move
NUDGE_STEPS = 50                    # keys a / d
CAL_STEPS = 20                      # key c: test move size (keep small so the sun stays in the rectangle)
CAL_SETTLE_S = 1.0

# ---- Y axis control (Camera 2) ----
START_Y_TRACKING = True             # start with Y tracking ON
Y_GAIN_STEPS_PER_PX = 0.20          # steps per pixel of error (vertical or horizontal). Raise to go faster
Y_SIGN = 1                          # +1 or -1. Y also learns the right direction by itself, but set it right if you can
Y_MAX_STEP = 80                     # largest single correction in steps (bigger = faster when far away)
Y_SETTLE_S = 0.15                   # wait after each move so the image catches up (was 0.5 - reduced for speed)
Y_CONFIRM_S = 0.05                  # sun must be steady this long before the first move (was 0.15)
Y_SEARCH_STEP = 40                  # steps per search move while no sun is visible
Y_SEARCH_INTERVAL_S = 0.2           # time between search moves
Y_MAX_SEARCH_REVERSALS = 4          # end-to-end turn-arounds while searching, then it gives up (NOT_FOUND)
Y_MAX_REVERSALS = 8                 # direction reversals while chasing the sun, then it stops (STUCK)
NUDGE_Y_STEPS = 50                  # keys j / l
CAL_Y_STEPS = 20                    # key v: test move size

# ---- Camera 1 : X axis ----
CAMERA1_INDEX = 1
ROTATE1_180 = True                  # old project rotated the image 180 deg
TARGET_ROI = (306, 138, 383, 291)   # same rectangle as the old project (640x480)
TARGET_POINT = (344, 230)           # red cross / line; None = exact centre of the rectangle
ALLOWED_ERROR_PX = 6                # |x error| within this = centred (also the dead band of X)

# ---- Camera 2 : Y axis ----
ENABLE_CAMERA2 = True               # False = run Camera 1 only
CAMERA2_INDEX = 0
ROTATE2_180 = True
CAMERA2_SIZE = (640, 640)           # (width, height) Camera 2 frames are resized to this; None = keep original
LINE_START = (320, 320)             # red reference line in the RESIZED Camera 2 frame
LINE_END = (640, 320)
ALLOWED_LINE_ERROR_PX = 6           # within this many px of the line = on the line
# ----------------------------------------------------------------------


class CameraReader:
    """Reads a camera in a background thread and always keeps the newest frame."""

    def __init__(self, index, rotate_180=False, name="camera", resize=None):
        self.name = name
        self.rotate_180 = rotate_180
        self.resize = resize            # (width, height) or None
        self.cap = cv2.VideoCapture(index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open {name} (index {index}). Try another index.")
        if isinstance(index, int):
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        self._lock = threading.Lock()
        self._frame = None
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while self._running:
            ok, frame = self.cap.read()
            if not ok:
                self._running = False
                break
            if self.rotate_180:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            if self.resize and (frame.shape[1], frame.shape[0]) != tuple(self.resize):
                frame = cv2.resize(frame, tuple(self.resize), interpolation=cv2.INTER_LINEAR)
            with self._lock:
                self._frame = frame

    def read(self):
        """Newest frame (or None if nothing yet). The frame may repeat if the camera is slower than the loop."""
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    @property
    def alive(self):
        return self._running

    def release(self):
        self._running = False
        self._thread.join(timeout=1.0)
        self.cap.release()


def wait_for_frame(reader, timeout=5.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        f = reader.read()
        if f is not None:
            return f
        time.sleep(0.02)
    raise RuntimeError(f"{reader.name} gave no frames.")


def process_cam1(frame, detector, target):
    """Returns (result, err_x, err_y, centred). Error = sun - target (+x = sun right of target).
    'centred' only looks at x, because stepper X can only fix the x error."""
    result = detector.detect(frame)
    err_x = err_y = None
    centred = False
    if result["detected"]:
        err_x, err_y = error_to_point(result["center"], target)
        centred = abs(err_x) <= ALLOWED_ERROR_PX
    return result, err_x, err_y, centred


def process_cam2(frame, detector, line):
    """Returns (result, dist, on_line). dist = the signed error driving the Y motor.

    dist is computed in three regions:
      * Sun LEFT  of line segment (sun_x < x_lo): horizontal distance to x_lo
                  dist = sun_x - x_lo  (negative → large → motor drives toward line)
      * Sun RIGHT of line segment (sun_x > x_hi): horizontal distance to x_hi
                  dist = sun_x - x_hi  (positive → large → motor drives toward line)
      * Sun WITHIN x-span [x_lo..x_hi]: perpendicular (vertical) distance to the line
                  dist = signed_distance_to_line()  (fine positioning)

    on_line: True only when within x-span AND within ALLOWED_LINE_ERROR_PX vertically.

    This gives the motor a real proportional signal even when the sun is on the opposite
    side of the image from the red line (the old vertical-only metric was ~0 there).
    """
    result = detector.detect(frame)
    dist = None
    on_line = False
    if result["detected"]:
        sun_x, sun_y = result["center"]
        x_lo = min(line[0][0], line[1][0])   # 320
        x_hi = max(line[0][0], line[1][0])   # 640
        if sun_x < x_lo:
            # Sun is LEFT of the line: horizontal gap to the left endpoint
            dist = float(sun_x - x_lo)           # negative (e.g. 122-320 = -198)
        elif sun_x > x_hi:
            # Sun is RIGHT of the line: horizontal gap to the right endpoint
            dist = float(sun_x - x_hi)           # positive (e.g. 700-640 = 60)
        else:
            # Sun is within the x-span: use vertical distance for fine locking
            dist = signed_distance_to_line(result["center"], line[0], line[1])
            on_line = abs(dist) <= ALLOWED_LINE_ERROR_PX
    return result, dist, on_line


def put_status(img, text, colour):
    h = img.shape[0]
    cv2.putText(img, text, (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2, cv2.LINE_AA)


def mean_measure(reader, detector, measure, n=8, delay=0.05):
    """Average measure(result) over a few frames (None if the sun is not seen in at least half of them)."""
    vals = []
    for _ in range(n):
        f = reader.read()
        if f is not None:
            r = detector.detect(f)
            if r["detected"]:
                vals.append(measure(r))
        time.sleep(delay)
    return sum(vals) / len(vals) if len(vals) >= n // 2 else None


def mean_sun_x(reader, detector, n=8, delay=0.05):
    """Average sun x over a few frames."""
    return mean_measure(reader, detector, lambda r: r["center"][0], n, delay)


def calibrate_x(link, cam1, detector):
    """Move X a few steps, see how far the sun moves in the image, print the settings to use."""
    x0 = mean_sun_x(cam1, detector)
    if x0 is None:
        print("[calibrate] Sun not seen in the rectangle. Bring it in with keys a / d first.")
        return
    moved = link.move_steps(CAL_STEPS, 0)[0]
    if moved == 0:
        print("[calibrate] X could not move (soft limit). Nudge X away from the limit with key a first.")
        return
    time.sleep(CAL_SETTLE_S)
    x1 = mean_sun_x(cam1, detector)
    link.move_steps(-moved, 0)                      # go back
    if x1 is None:
        print(f"[calibrate] The sun left the rectangle after {moved} steps. Lower CAL_STEPS and try again.")
        return
    px_per_step = (x1 - x0) / moved
    print(f"[calibrate] +{moved} steps moved the sun {x1 - x0:+.1f} px  ({px_per_step:+.3f} px per step)")
    if abs(px_per_step) < 0.05:
        print("[calibrate] Almost no movement - raise CAL_STEPS or check that the mirror/camera really moves.")
        return
    sign = -1 if px_per_step > 0 else 1
    gain = 1.0 / abs(px_per_step)
    print(f"[calibrate] Set  X_SIGN = {sign}   and  X_GAIN_STEPS_PER_PX = {gain * 0.8:.3f}  "
          f"(full correction would be {gain:.3f}; 80% is smoother)")


def calibrate_y(link, cam2, detector, line):
    """Move Y a few steps, see how the distance to the line changes, print the settings to use."""
    measure = lambda r: signed_distance_to_line(r["center"], line[0], line[1])
    d0 = mean_measure(cam2, detector, measure)
    if d0 is None:
        print("[calibrate Y] Sun not seen in Camera 2. Bring it into view with keys j / l first.")
        return
    moved = link.move_steps(0, CAL_Y_STEPS)[1]
    if moved == 0:
        print("[calibrate Y] Y could not move (limit). Nudge Y away from the end with key j first.")
        return
    time.sleep(CAL_SETTLE_S)
    d1 = mean_measure(cam2, detector, measure)
    link.move_steps(0, -moved)                      # go back
    if d1 is None:
        print(f"[calibrate Y] The sun left Camera 2's view after {moved} steps. Lower CAL_Y_STEPS and try again.")
        return
    per_step = (d1 - d0) / moved
    print(f"[calibrate Y] +{moved} steps changed the distance to the line by {d1 - d0:+.1f} px  ({per_step:+.3f} px per step)")
    if abs(per_step) < 0.05:
        print("[calibrate Y] Almost no movement - raise CAL_Y_STEPS or check that Y really moves the image.")
        return
    sign = -1 if per_step > 0 else 1
    gain = 1.0 / abs(per_step)
    print(f"[calibrate Y] Set  Y_SIGN = {sign}   and  Y_GAIN_STEPS_PER_PX = {gain * 0.8:.3f}  "
          f"(full correction would be {gain:.3f}; 80% is smoother)")


def recover_after_reset(link):
    """The Arduino restarted (power dip / reset): wait until it has homed again, then go back to X_START_DEG.
    Returns True if it recovered."""
    print("!! The Arduino restarted - waiting for it to finish homing ...")
    try:
        link.wait_ready()
        link.goto_x_deg(X_START_DEG)
    except StepperError as e:
        print("Could not recover:", e)
        return False
    print(f"Recovered after restart #{link.reset_count}. {link.status_text()}")
    return True


def main():
    cam1 = CameraReader(CAMERA1_INDEX, ROTATE1_180, "Camera 1")
    cam2 = None
    link = None
    try:
        if ENABLE_CAMERA2:
            cam2 = CameraReader(CAMERA2_INDEX, ROTATE2_180, "Camera 2", resize=CAMERA2_SIZE)

        f1 = wait_for_frame(cam1)
        print(f"Camera 1 opened (index {CAMERA1_INDEX}): {f1.shape[1]}x{f1.shape[0]}")
        det1 = SunDetector(roi=TARGET_ROI)
        target = TARGET_POINT if TARGET_POINT else roi_center(TARGET_ROI)
        print(f"Camera 1 rectangle {TARGET_ROI}, target point {target}")

        det2 = line = None
        if cam2:
            f2 = wait_for_frame(cam2)
            print(f"Camera 2 opened (index {CAMERA2_INDEX}): {f2.shape[1]}x{f2.shape[0]}")
            det2 = SunDetector(roi=None)                       # whole frame
            line = (LINE_START, LINE_END)
            print(f"Camera 2 line {line[0]} -> {line[1]}")

        xctrl = AxisController(gain=X_GAIN_STEPS_PER_PX, sign=X_SIGN, deadband_px=ALLOWED_ERROR_PX,
                               max_step=X_MAX_STEP, settle_s=X_SETTLE_S, confirm_s=X_CONFIRM_S)
        tracking = START_TRACKING
        yctrl = LimitAwareController(
            gain=Y_GAIN_STEPS_PER_PX, sign=Y_SIGN, deadband_px=ALLOWED_LINE_ERROR_PX, max_step=Y_MAX_STEP,
            settle_s=Y_SETTLE_S, confirm_s=Y_CONFIRM_S, search_step=Y_SEARCH_STEP,
            search_interval_s=Y_SEARCH_INTERVAL_S, max_search_reversals=Y_MAX_SEARCH_REVERSALS,
            max_reversals=Y_MAX_REVERSALS)
        ytracking = START_Y_TRACKING

        if ENABLE_STEPPERS:
            print(f"Connecting to the Arduino on {SERIAL_PORT} (it homes first, this takes a few seconds)...")
            link = StepperLink(port=SERIAL_PORT).connect()
            link.goto_x_deg(X_START_DEG)
            print(f"Ready. {link.status_text()}")

        last_print = 0.0
        saved = 0
        x_note = ""
        y_note = ""

        while cam1.alive and (cam2 is None or cam2.alive):
            frame1 = cam1.read()
            frame2 = cam2.read() if cam2 else None
            if frame1 is None or (cam2 and frame2 is None):
                time.sleep(0.005)
                continue

            # ---------------- Arduino restart check ----------------
            if link:
                try:
                    if link.poll():
                        ok = recover_after_reset(link)
                        xctrl.reset()
                        yctrl.reset()
                        tracking = tracking and ok
                        ytracking = ytracking and ok
                        x_note = "ARDUINO RESTARTED - re-homed" if ok else "ARDUINO RESTARTED - recovery failed"
                except StepperError as e:
                    print("Stepper error:", e)
                    tracking = ytracking = False

            # ---------------- Camera 1 + stepper X ----------------
            res1, ex, ey, centred = process_cam1(frame1, det1, target)

            steps = xctrl.update(ex, res1["detected"]) if tracking else 0
            if steps and link:
                try:
                    moved = link.move_steps(steps, 0)[0]
                    x_note = "X AT SOFT LIMIT" if moved == 0 else ""
                    if link.limit_hits:
                        x_note = "X HIT LIMIT SWITCH"
                except StepperReset:
                    ok = recover_after_reset(link)
                    xctrl.reset()
                    yctrl.reset()
                    tracking = tracking and ok
                    ytracking = ytracking and ok
                    x_note = "ARDUINO RESTARTED - re-homed" if ok else "ARDUINO RESTARTED - recovery failed"
                except StepperError as e:
                    print("Stepper error:", e)
                    tracking = False
                    x_note = "STEPPER ERROR - tracking off"
            elif not tracking:
                xctrl.reset()

            shown1 = draw_overlay(frame1, res1, roi=det1.roi, target=target)
            cv2.line(shown1, (det1.roi[0], target[1]), (det1.roi[2], target[1]), (0, 0, 255), 1)
            if res1["detected"]:
                put_status(shown1, "CENTRED" if centred else f"err x={ex:+d} y={ey:+d}",
                           (0, 255, 0) if centred else (0, 255, 255))
            else:
                put_status(shown1, "NO SUN IN RECTANGLE", (0, 0, 255))
            info = f"track {'ON' if tracking else 'OFF'}  {xctrl.state}"
            if link:
                info += f"  X {link.pos_x} st {link.x_deg:+.1f} deg"
                if link.reset_count:
                    info += f"  restarts {link.reset_count}"
            cv2.putText(shown1, info, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            if x_note:
                cv2.putText(shown1, x_note, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
            cv2.imshow("Camera 1 - X axis", shown1)

            # ---------------- Camera 2 (display only) ----------------
            res2 = dist = None
            on_line = False
            if cam2:
                res2, dist, on_line = process_cam2(frame2, det2, line)

                if link and ytracking:
                    steps_y = yctrl.update(dist, res2["detected"])
                    if steps_y:
                        try:
                            moved_y = link.move_steps(0, steps_y)[1]
                            yctrl.notify_move_result(steps_y, moved_y, bool(link.limit_hits))
                            if link.limit_hits:
                                print(f"Y hit a limit switch ({link.limit_hits}); controller: {yctrl.last_event}")
                            elif moved_y == 0:
                                print(f"Y at its soft limit; controller: {yctrl.last_event}")
                        except StepperReset:
                            ok = recover_after_reset(link)
                            xctrl.reset()
                            yctrl.reset()
                            tracking = tracking and ok
                            ytracking = ytracking and ok
                        except StepperError as e:
                            print("Stepper error:", e)
                            ytracking = False
                    y_note = yctrl.last_event
                elif not ytracking:
                    yctrl.reset()
                    y_note = ""

                shown2 = draw_overlay(frame2, res2, line=line)
                cv2.drawMarker(shown2, line[0], (0, 0, 255), cv2.MARKER_CROSS, 14, 2)
                if res2["detected"]:
                    put_status(shown2, "ON LINE" if on_line else f"dist to line {dist:+.0f}px",
                               (0, 255, 0) if on_line else (0, 255, 255))
                else:
                    put_status(shown2, "SUN NOT DETECTED", (0, 0, 255))
                yinfo = f"Ytrack {'ON' if ytracking else 'OFF'}  {yctrl.state}"
                if link:
                    yinfo += f"  Y {link.pos_y} st {link.y_deg:+.1f} deg"
                cv2.putText(shown2, yinfo, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                if y_note:
                    cv2.putText(shown2, y_note, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                cv2.imshow("Camera 2 - Y axis", shown2)

            # ---------------- terminal output ----------------
            now = time.time()
            if now - last_print >= PRINT_EVERY_S:
                last_print = now
                s1 = (f"cam1 sun={res1['center']} err=({ex:+d},{ey:+d}) {xctrl.state}"
                      if res1["detected"] else "cam1 no sun")
                if link:
                    s1 += f" | X={link.pos_x}st {link.x_deg:+.1f}deg"
                if cam2:
                    s2 = (f"cam2 sun={res2['center']} dist={dist:+.0f}px{' ON LINE' if on_line else ''}"
                          if res2["detected"] else "cam2 no sun")
                    s2 += f" {yctrl.state}"
                    if link:
                        s2 += f" Y={link.pos_y}st {link.y_deg:+.1f}deg"
                    print(f"{s1}   |   {s2}")
                else:
                    print(s1)

            # ---------------- keys ----------------
            key = cv2.waitKey(1) & 0xFF
            try:
                if key == ord("q"):
                    break
                elif key == ord("t"):
                    tracking = not tracking
                    xctrl.reset()
                    print(f"Tracking {'ON' if tracking else 'OFF'}")
                elif key in (ord("a"), ord("d")) and link:
                    d = -NUDGE_STEPS if key == ord("a") else NUDGE_STEPS
                    link.move_steps(d, 0)
                    xctrl.reset()
                    print(f"Nudged X {d:+d} steps -> {link.status_text()}")
                elif key == ord("c") and link:
                    was = tracking
                    tracking = False
                    calibrate_x(link, cam1, det1)
                    xctrl.reset()
                    tracking = was
                elif key == ord("u"):
                    ytracking = not ytracking
                    yctrl.reset()
                    print(f"Y tracking {'ON' if ytracking else 'OFF'}")
                elif key in (ord("j"), ord("l")) and link:
                    d = -NUDGE_Y_STEPS if key == ord("j") else NUDGE_Y_STEPS
                    link.move_steps(0, d)
                    yctrl.reset()
                    print(f"Nudged Y {d:+d} steps -> {link.status_text()}")
                elif key == ord("v") and link and cam2:
                    was = ytracking
                    ytracking = False
                    calibrate_y(link, cam2, det2, line)
                    yctrl.reset()
                    ytracking = was
                elif key == ord("h") and link:
                    link.home()
                    link.goto_x_deg(X_START_DEG)
                    xctrl.reset()
                    yctrl.reset()
                    print(f"Homed. {link.status_text()}")
                elif key == ord("s"):
                    cv2.imwrite(f"cam1_{saved:04d}.png", frame1)
                    if cam2:
                        cv2.imwrite(f"cam2_{saved:04d}.png", frame2)
                    print(f"Saved frame set {saved:04d}")
                    saved += 1
                elif key == ord("r"):
                    x, y, bw, bh = cv2.selectROI("Camera 1 - X axis", frame1, showCrosshair=False)
                    if bw > 0 and bh > 0:
                        det1.roi = (int(x), int(y), int(x + bw), int(y + bh))
                        target = roi_center(det1.roi)
                        print(f"New TARGET_ROI = {det1.roi}, target point = {target} (paste into the config to keep it)")
            except StepperReset:
                ok = recover_after_reset(link)
                xctrl.reset()
                yctrl.reset()
                tracking = tracking and ok
                ytracking = ytracking and ok
            except StepperError as e:
                print("Stepper error:", e)
                tracking = False
    finally:
        cam1.release()
        if cam2:
            cam2.release()
        if link:
            link.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()