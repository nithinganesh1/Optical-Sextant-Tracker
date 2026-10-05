import time
import math
import cv2
import queue
import threading
import numpy as np
import serial

from sun_tracker import SunTracker


class DualStepperController:
    """
    Controls two stepper motors over a single Arduino serial port.
    Motor 1 (Camera 1 / Y-axis): CNC Shield X driver slot
    Motor 2 (Camera 2 / X-axis): CNC Shield Y driver slot
    """
    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baud: int = 115200,
        steps_per_pixel1: float = 0.30,
        steps_per_pixel2: float = 0.30,
        max_steps_per_frame: int = 80,
        deadband_px: int = 6,
        invert1: bool = False,
        invert2: bool = False,
        microsteps: int = 16,
        gear_ratio: float = 2.0,
        y_smooth_chunk: int = 50,
        y_smooth_delay: float = 0.02,
    ):
        self.steps_per_pixel1 = steps_per_pixel1
        self.steps_per_pixel2 = steps_per_pixel2
        self.max_steps_per_frame = max_steps_per_frame
        self.deadband_px = deadband_px
        self.invert1 = invert1
        self.invert2 = invert2

        self.degrees_per_step = (1.8 / microsteps) / gear_ratio

        self.absolute_steps1 = 0
        self.absolute_steps2 = 0

        self._cmd_queue: "queue.Queue[tuple]" = queue.Queue(maxsize=1)
        self._stop_event = threading.Event()

        try:
            self._ser = serial.Serial(port, baud, timeout=1.0)
            print(f"Waiting for Arduino to finish homing on {port}...")

            while True:
                line = self._ser.readline().decode('ascii', errors='ignore').strip()
                if line == "READY":
                    print("Arduino is homed and READY!")
                    break

            self._ser.timeout = 0
            self._ser.write_timeout = 0
            self._ser.reset_input_buffer()
            self.serial_connected = True
        except Exception as e:
            print(f"Warning: Could not connect to Arduino on {port}. Running in simulation mode. ({e})")
            self.serial_connected = False

        self._thread = threading.Thread(target=self._writer_loop, daemon=True)
        self._thread.start()

        # smoothing params for Y axis (motor 1)
        self.y_smooth_chunk = y_smooth_chunk
        self.y_smooth_delay = y_smooth_delay

    def update(self, error1: int, error2: int) -> dict:
        """Call once per frame from the capture loop."""
        steps1 = 0
        if abs(error1) >= self.deadband_px:
            steps1 = int(round(error1 * self.steps_per_pixel1))
            steps1 = max(-self.max_steps_per_frame, min(self.max_steps_per_frame, steps1))
            if self.invert1:
                steps1 = -steps1

        steps2 = 0
        if abs(error2) >= self.deadband_px:
            steps2 = int(round(error2 * self.steps_per_pixel2))
            steps2 = max(-self.max_steps_per_frame, min(self.max_steps_per_frame, steps2))
            if self.invert2:
                steps2 = -steps2

        return self.update_raw_steps(steps1, steps2)

    def update_raw_steps(self, steps1: int, steps2: int) -> dict:
        """Allows direct sending of step commands for both motors."""
        self.absolute_steps1 += steps1
        self.absolute_steps2 += steps2

        degrees1 = steps1 * self.degrees_per_step
        degrees2 = steps2 * self.degrees_per_step

        abs_deg1 = self.absolute_steps1 * self.degrees_per_step
        abs_deg2 = self.absolute_steps2 * self.degrees_per_step

        if steps1 != 0 or steps2 != 0:
            try:
                self._cmd_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._cmd_queue.put_nowait((steps1, steps2))
            except queue.Full:
                pass

        return {
            "steps1": steps1,
            "steps2": steps2,
            "degrees1": degrees1,
            "degrees2": degrees2,
            "absolute_degrees1": abs_deg1,
            "absolute_degrees2": abs_deg2,
        }

    def _writer_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                steps1, steps2 = self._cmd_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            if self.serial_connected:
                try:
                    # Smooth large Y-axis moves by sending them in smaller chunks
                    if steps2 == 0 and abs(steps1) > self.y_smooth_chunk:
                        sign = 1 if steps1 > 0 else -1
                        remaining = abs(steps1)
                        while remaining > 0 and not self._stop_event.is_set():
                            part = min(self.y_smooth_chunk, remaining)
                            try:
                                self._ser.write(f"{sign * part} 0\n".encode("ascii"))
                            except serial.SerialException as e:
                                print(f"\n[ERROR] Serial connection lost! Motors will not move. Exception: {e}")
                                self.serial_connected = False
                                break
                            remaining -= part
                            time.sleep(self.y_smooth_delay)
                    else:
                        self._ser.write(f"{steps1} {steps2}\n".encode("ascii"))
                except serial.SerialException as e:
                    print(f"\n[ERROR] Serial connection lost! Motors will not move. Exception: {e}")
                    self.serial_connected = False

    def stop(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=1.0)
        if self.serial_connected:
            try:
                self._ser.close()
            except Exception:
                pass


# Backwards compatibility wrapper for single-stepper use cases
class StepperYController:
    def __init__(self, port: str = "/dev/ttyUSB0", **kwargs):
        self._dual = DualStepperController(port=port, **kwargs)
        self.degrees_per_step = self._dual.degrees_per_step
        self.max_steps_per_frame = self._dual.max_steps_per_frame

    @property
    def absolute_steps(self):
        return self._dual.absolute_steps1

    @absolute_steps.setter
    def absolute_steps(self, val):
        self._dual.absolute_steps1 = val

    @property
    def serial_connected(self):
        return self._dual.serial_connected

    def update(self, error_y: int) -> dict:
        res = self._dual.update(error_y, 0)
        return {
            "steps": res["steps1"],
            "degrees": res["degrees1"],
            "absolute_degrees": res["absolute_degrees1"],
        }

    def update_raw_steps(self, steps: int) -> dict:
        res = self._dual.update_raw_steps(steps, 0)
        return {
            "steps": res["steps1"],
            "degrees": res["degrees1"],
            "absolute_degrees": res["absolute_degrees1"],
        }

    def stop(self) -> None:
        self._dual.stop()


class SyntheticCamera:
    """Mock video source if physical webcams are unavailable."""
    def __init__(self, width=640, height=480, sun_offset_x=0, sun_offset_y=0):
        self.w = width
        self.h = height
        self.t = 0
        self.sun_offset_x = sun_offset_x
        self.sun_offset_y = sun_offset_y

    def isOpened(self):
        return True

    def read(self):
        self.t += 0.05
        frame = np.full((self.h, self.w, 3), 30, dtype=np.uint8)

        # Draw a simulated sun disc
        cx = int(self.w / 2 + math.sin(self.t) * 150 + self.sun_offset_x)
        cy = int(self.h / 2 + math.cos(self.t * 0.7) * 80 + self.sun_offset_y)
        cv2.circle(frame, (cx, cy), 25, (255, 255, 255), -1)

        # Apply mild blur to simulate camera view
        frame = cv2.GaussianBlur(frame, (9, 9), 0)
        return True, frame

    def release(self):
        pass


class TrackingUnit:
    """State machine tracker for a single camera / stepper pair."""
    def __init__(self, name: str, SWEEP_SPEED_STEPS: int = 10, MAX_SWEEP_DEGREES: float = 300.0, MAX_VERIFY_STEPS: int = 250, deadband_px: int = 8, steps_per_pixel: float = 0.25):
        self.name = name
        self.state = "SWEEPING"
        self.sweep_speed = SWEEP_SPEED_STEPS
        self.max_sweep_degrees = MAX_SWEEP_DEGREES
        self.max_verify_steps = MAX_VERIFY_STEPS
        self.verify_steps_taken = 0
        self.best_candidate_pos = 0
        self.best_candidate_brightness = 0.0
        self.deadband_px = deadband_px
        self.steps_per_pixel = steps_per_pixel

    def compute_step(self, detected: bool, brightness: float, current_pos: int, max_steps_per_frame: int, degrees_per_step: float, error_val: int) -> int:
        steps_to_move = 0
        current_degrees = current_pos * degrees_per_step

        if not detected:
            # Search continuously: if the sun is not found, keep sweeping Y until a candidate appears.
            if current_degrees < self.max_sweep_degrees:
                steps_to_move = self.sweep_speed
            else:
                print(f"[{self.name}] Reached max sweep angle ({self.max_sweep_degrees}°). Reversing search.")
                steps_to_move = -self.sweep_speed
            return steps_to_move

        # Sun detected: do not send a sweep while already centered; use fine correction instead.
        if abs(error_val) < self.deadband_px:
            return 0

        steps = int(round(error_val * self.steps_per_pixel))
        steps_to_move = max(-max_steps_per_frame, min(max_steps_per_frame, steps))

        # Keep the state machine consistent with real tracking without blocking the sweep logic.
        if self.state == "SWEEPING":
            self.state = "TRACKING"
        elif self.state == "VERIFYING":
            self.state = "TRACKING"

        return steps_to_move


def draw_overlay_cam2(frame: np.ndarray, result: dict, tracker: SunTracker) -> np.ndarray:
    """Draws vertical reference line from image center to bottom for Camera 2."""
    if result is None:
        return frame

    out = frame.copy()
    fh, fw = out.shape[:2]

    # Full frame green boundary
    cv2.rectangle(out, (2, 2), (fw - 3, fh - 3), (0, 255, 0), 1)

    # Vertical reference line from image center down to bottom
    center_x = fw // 2
    center_y = fh // 2

    cv2.line(out, (center_x, center_y), (center_x, fh), (0, 0, 255), 2)
    cv2.drawMarker(out, (center_x, center_y), (0, 0, 255), cv2.MARKER_CROSS, 14, 2)

    if result["detected"]:
        cx, cy = result["center"]
        r = int(round(result["radius"]))
        
        # Calculate error_x relative to vertical center line
        error_x = cx - center_x
        dist_to_line = abs(error_x)
        on_target = (cy >= center_y - r) and (dist_to_line <= max(r, tracker.on_target_tolerance_px))
        
        color = (0, 255, 0) if on_target else \
                ((0, 255, 255) if result["confidence"] > 0.5 else (0, 165, 255))

        cv2.circle(out, (cx, cy), r, color, 2)
        cv2.circle(out, (cx, cy), 3, (0, 0, 255), -1)
        
        # Draw line connecting sun center to vertical reference line
        cv2.line(out, (center_x, cy), (cx, cy), (200, 200, 200), 1)

        label = "ON TARGET" if on_target else f"err_x={error_x:+d}px"
        cv2.putText(out, label, (cx + r + 8, cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    else:
        cv2.putText(out, "SUN NOT DETECTED", (fw // 2 - 90, fh // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)

    cv2.putText(out, f"FPS: {tracker.fps:.1f}", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(out, f"Confidence: {result['confidence']:.2f}", (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

    return out


if __name__ == "__main__":
    import sys

    # Default camera sources: edit these values here to match your USB webcams.
    # For a setup using camera index 1 and camera index 2, keep: (1, 2)
    DEFAULT_CAMERA_SOURCES = (1, 2)
    src1, src2 = DEFAULT_CAMERA_SOURCES

    # Check for --swap argument
    swap_cameras = "--swap" in sys.argv
    if swap_cameras:
        sys.argv.remove("--swap")

    # Parse positional CLI args
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if len(args) > 0:
        src1 = int(args[0]) if args[0].isdigit() else args[0]
    if len(args) > 1:
        src2 = int(args[1]) if args[1].isdigit() else args[1]

    if swap_cameras:
        print("Swapping camera sources as requested (--swap)...")
        src1, src2 = src2, src1

    print(f"Opening Camera 1 on source: {src1}")
    print(f"Opening Camera 2 on source: {src2}")

    # Initialize Camera 1
    cap1 = cv2.VideoCapture(src1)
    if not cap1.isOpened():
        print(f"Notice: Camera 1 (source {src1}) could not be opened. Using synthetic camera 1.")
        cap1 = SyntheticCamera(640, 480, sun_offset_x=10, sun_offset_y=-20)

    # Initialize Camera 2
    cap2 = cv2.VideoCapture(src2)
    if not cap2.isOpened():
        print(f"Notice: Camera 2 (source {src2}) could not be opened. Using synthetic camera 2.")
        cap2 = SyntheticCamera(640, 480, sun_offset_x=-40, sun_offset_y=30)

    # Camera 1 setup: Green ROI rectangle (right side increased by 10px: 306 to 383) & horizontal reference line
    ROI_CAM1 = (306, 138, 383, 291)
    TARGET_CAM1 = ((306 + 383) // 2, 230)
    tracker1 = SunTracker(roi=ROI_CAM1, target_point=TARGET_CAM1, debug=False)

    # Camera 2 setup: Exact original SunTracker on full image (`roi=None`)
    tracker2 = SunTracker(roi=None, debug=False)

    # Dual Stepper Controller over single Arduino connection
    controller = DualStepperController(port="/dev/ttyUSB0", microsteps=16)

    unit1 = TrackingUnit(name="Cam1-Stepper1 (Y)")
    unit2 = TrackingUnit(name="Cam2-Stepper2 (X)")

    print("Starting Dual Camera Sun Tracker System. Press 'q' to quit.")

    try:
        # ---- Pre-alignment: align Camera1 (Y axis) to its reference line ----
        print("Pre-aligning Camera 1 to reference line...")
        aligned_stable = 0
        aligned_required = 5
        while aligned_stable < aligned_required:
            ok1, frame1 = cap1.read()
            ok2, frame2 = cap2.read()
            if not ok1 or not ok2:
                print("Failed to read frame from cameras during pre-align.")
                break

            if not isinstance(cap1, SyntheticCamera):
                frame1 = cv2.rotate(frame1, cv2.ROTATE_180)
            if not isinstance(cap2, SyntheticCamera):
                frame2 = cv2.rotate(frame2, cv2.ROTATE_180)

            # keep frames same size
            h1, w1 = frame1.shape[:2]
            if frame2.shape[:2] != (h1, w1):
                frame2 = cv2.resize(frame2, (w1, h1), interpolation=cv2.INTER_LINEAR)

            result1 = tracker1.update(frame1)
            result2 = tracker2.update(frame2)

            # If camera1 reports on_target, increment stability counter
            if result1.get("on_target"):
                aligned_stable += 1
            else:
                aligned_stable = 0
                err_y = result1["error_y"]
                step_y = unit1.compute_step(
                    detected=result1["detected"],
                    brightness=result1["brightness"],
                    current_pos=controller.absolute_steps1,
                    max_steps_per_frame=controller.max_steps_per_frame,
                    degrees_per_step=controller.degrees_per_step,
                    error_val=err_y,
                )
                controller.update_raw_steps(step_y, 0)

            # Quick visual feedback
            annotated1 = tracker1.draw_overlay(frame1, result1)
            annotated2 = draw_overlay_cam2(frame2, result2, tracker2)
            cv2.imshow("Sun Tracker - Camera 1", annotated1)
            cv2.imshow("Sun Tracker - Camera 2", annotated2)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        print("Camera 1 aligned. Beginning X-axis scans from 90° → 150° to find the sun on Camera 2...")

        # Compute step limits for 90°..150° using the controller's degrees_per_step
        steps_for_90 = int(round(90.0 / controller.degrees_per_step))
        steps_for_150 = int(round(150.0 / controller.degrees_per_step))
        if steps_for_150 <= steps_for_90:
            steps_for_150 = steps_for_90 + 1
        range_steps = steps_for_150 - steps_for_90

        # Move to the 90° reference position first
        delta_to_90 = steps_for_90 - controller.absolute_steps2
        if delta_to_90 != 0:
            controller.update_raw_steps(0, delta_to_90)

        # ---- Two-pass X-axis scan constrained to 90°..150°: forward then backward ----
        for scan_idx in range(2):
            # forward scan: positive moves, backward scan: negative moves
            forward = (scan_idx == 0)
            moved = 0
            chunk = 100
            found = False

            while moved < range_steps:
                # move in small increments so we can check camera2 between moves
                move = min(chunk, range_steps - moved)
                step_move = move if forward else -move
                # ensure move will not exceed clamp bounds
                next_abs = controller.absolute_steps2 + step_move
                if next_abs < steps_for_90:
                    step_move = steps_for_90 - controller.absolute_steps2
                elif next_abs > steps_for_150:
                    step_move = steps_for_150 - controller.absolute_steps2
                if step_move == 0:
                    break
                controller.update_raw_steps(0, step_move)
                moved += abs(step_move)
                ok1, frame1 = cap1.read()
                ok2, frame2 = cap2.read()
                if not ok1 or not ok2:
                    break
                if not isinstance(cap1, SyntheticCamera):
                    frame1 = cv2.rotate(frame1, cv2.ROTATE_180)
                if not isinstance(cap2, SyntheticCamera):
                    frame2 = cv2.rotate(frame2, cv2.ROTATE_180)
                if frame2.shape[:2] != frame1.shape[:2]:
                    frame2 = cv2.resize(frame2, (frame1.shape[1], frame1.shape[0]), interpolation=cv2.INTER_LINEAR)

                result2 = tracker2.update(frame2)

                # If Camera2 sees the sun, try to center it using unit2 tracking
                if result2.get("detected"):
                    print(f"Camera 2 detected sun during scan {scan_idx + 1}; centering...")
                    # attempt to center for up to N iterations
                    for _ in range(40):
                        if result2.get("detected"):
                            err_x = result2["center"][0] - (frame1.shape[1] // 2)
                        else:
                            err_x = result2.get("error_x", 0)
                        step_x = unit2.compute_step(
                            detected=result2.get("detected", False),
                            brightness=result2.get("brightness", 0.0),
                            current_pos=controller.absolute_steps2,
                            max_steps_per_frame=controller.max_steps_per_frame,
                            degrees_per_step=controller.degrees_per_step,
                            error_val=err_x,
                        )
                        if step_x == 0:
                            found = True
                            break
                        controller.update_raw_steps(0, step_x)
                        ok2b, frame2 = cap2.read()
                        if not ok2b:
                            break
                        if not isinstance(cap2, SyntheticCamera):
                            frame2 = cv2.rotate(frame2, cv2.ROTATE_180)
                        result2 = tracker2.update(frame2)
                    if found:
                        print(f"Found and centered on Camera 2 during scan {scan_idx + 1}.")
                        break

                # show overlays while scanning
                result1 = tracker1.update(frame1)
                annotated1 = tracker1.draw_overlay(frame1, result1)
                annotated2 = draw_overlay_cam2(frame2, result2, tracker2)
                cv2.imshow("Sun Tracker - Camera 1", annotated1)
                cv2.imshow("Sun Tracker - Camera 2", annotated2)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            if not found:
                print(f"Scan {scan_idx + 1} complete: sun not found during this sweep (90°→150°).")
            else:
                print(f"Scan {scan_idx + 1} complete: sun found and centered.")

        print("Scans complete — entering continuous tracking mode.")

        # ---- Continuous tracking loop (original behavior) ----
        while True:
            ok1, frame1 = cap1.read()
            ok2, frame2 = cap2.read()

            if not ok1 or not ok2:
                print("Failed to read frame from cameras.")
                break

            if not isinstance(cap1, SyntheticCamera):
                frame1 = cv2.rotate(frame1, cv2.ROTATE_180)
            if not isinstance(cap2, SyntheticCamera):
                frame2 = cv2.rotate(frame2, cv2.ROTATE_180)

            # Resize Camera 2 frame to match Camera 1 frame dimensions exactly
            h1, w1 = frame1.shape[:2]
            h2, w2 = frame2.shape[:2]
            if (h2, w2) != (h1, w1):
                frame2 = cv2.resize(frame2, (w1, h1), interpolation=cv2.INTER_LINEAR)

            # Update SunTrackers
            result1 = tracker1.update(frame1)
            result2 = tracker2.update(frame2)

            # Error for Camera 1 (Y error relative to horizontal target line)
            error_y1 = result1["error_y"]

            # Error for Camera 2 (X error relative to vertical center-down line at w1//2)
            if result2["detected"]:
                error_x2 = result2["center"][0] - (w1 // 2)
            else:
                error_x2 = result2["error_x"]

            # Calculate motor steps for Unit 1 (Cam 1 / Stepper 1 - Y axis)
            step1 = unit1.compute_step(
                detected=result1["detected"],
                brightness=result1["brightness"],
                current_pos=controller.absolute_steps1,
                max_steps_per_frame=controller.max_steps_per_frame,
                degrees_per_step=controller.degrees_per_step,
                error_val=error_y1
            )

            # Calculate motor steps for Unit 2 (Cam 2 / Stepper 2 - X axis)
            step2 = unit2.compute_step(
                detected=result2["detected"],
                brightness=result2["brightness"],
                current_pos=controller.absolute_steps2,
                max_steps_per_frame=controller.max_steps_per_frame,
                degrees_per_step=controller.degrees_per_step,
                error_val=error_x2
            )

            # Send movement updates to Arduino
            movement = controller.update_raw_steps(step1, step2)

            # Draw overlays
            annotated1 = tracker1.draw_overlay(frame1, result1)
            annotated2 = draw_overlay_cam2(frame2, result2, tracker2)

            # Add title labels
            cv2.putText(annotated1, "Camera 1 (Partial ROI / Stepper Y)", (10, h1 - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(annotated2, "Camera 2 (Full Image / Stepper X)", (10, h1 - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2, cv2.LINE_AA)

            # Display in SEPARATE WINDOWS with exact matching sizes
            cv2.imshow("Sun Tracker - Camera 1", annotated1)
            cv2.imshow("Sun Tracker - Camera 2", annotated2)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        controller.stop()
        cap1.release()
        cap2.release()
        cv2.destroyAllWindows()