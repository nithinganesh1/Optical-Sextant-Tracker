Suntracker_Dash
===============

Overview
--------
This project implements a camera-guided sun tracker using two cameras and two stepper motors.
It simulates an optical sextant (double-reflection) and can control a physical mount (Arduino) for real-world tracking.

Key components
--------------
- `app.py` — Flask dashboard and `Mount` wrapper (homes, reports status, calculates mirror/sun angle).
- `controller.py` — DualStepperController (serial motor interface), TrackingUnit (scan/track state machine), camera demo runner.
- `sun_tracker.py` — image processing to detect the sun, outputting `detected`, `center`, `radius`, `confidence`, and pixel errors.
- `templates/ref.html` — interactive simulator (now used as `index.html`).

Image processing & ROI
----------------------
- Camera 1 (primary): configured ROI focuses on a narrow strip (right side) to find horizon and anchor the altitude reference. In `controller.py`:
  - `ROI_CAM1 = (306, 138, 383, 291)` — (x, y, width, height).
  - `TARGET_CAM1` is the horizontal reference point used to compute vertical error (`error_y`) for Y-axis corrections.
- Camera 2 (secondary): full-frame view used for azimuth/centering (X-axis). It provides `error_x` — horizontal pixel offset from the vertical center line.

Sun detection (in `sun_tracker.py`)
----------------------------------
- Uses basic image operations (blurring, color thresholds) and contour detection to find the brightest circular region (sun disk).
- Output fields used by the controller:
  - `detected` (bool) — whether a sun candidate was found.
  - `center` (x,y) — pixel coordinates of the sun center.
  - `radius` — estimated radius in pixels.
  - `confidence` — heuristic score of detection quality.
  - `error_x`, `error_y` — pixel offsets relative to camera references.

Motor control & mapping
-----------------------
- `DualStepperController` maps pixel errors to motor steps:
  - `steps = int(round(error_px * steps_per_pixel))` with `steps_per_pixel` configured per motor.
  - Moves are rate-limited using `max_steps_per_frame` and deadband (`deadband_px`) to reduce jitter.
- Degrees per motor step are computed from hardware constants in `app.py`:
  - `STEPS_FOR_90_DEG = 1600` (motor steps corresponding to 90° mechanical rotation)
  - `DEG_PER_STEP = MECHANICAL_DEG / STEPS_FOR_90_DEG` (mirror degrees per step)

Angle calculation (sextant rule)
--------------------------------
- For a mirror rotated by θ, the reflected beam moves by 2θ. Therefore:
  - mirror_angle_deg = (steps - REFERENCE_STEPS) * DEG_PER_STEP
  - sun_angle = REFERENCE_SUN_ANGLE + SUN_ANGLE_SIGN * 2.0 * mirror_angle_deg
- `REFERENCE_STEPS` is the motor step count corresponding to a chosen zero reference for the mirror. In `app.py` it's 1600.

Operational limits
------------------
- The X-axis scan and tracking have been constrained to an operating window of mirror angles 90° → 150° (60° span). This maps to a step-range computed from `degrees_per_step` so the system never commands the mirror outside that window.

How the scan/lock routine works
-------------------------------
1. Home + move to reference (90°) after homing.
2. Pre-align Y (Camera 1) until on-target stable.
3. Perform two-pass X scan across 90°→150° (forward then reverse) in small chunks; during each step the code checks Camera 2 for the sun.
4. If Camera 2 detects the sun, the controller centers the sun by switching to `TRACKING` state for that camera and stopping the scan.
5. After lock, continuous tracking runs, mapping pixel errors to stepper moves and keeping motion bounded within 90°→150°.

Running
-------
Activate virtualenv then run:

```bash
source ~/main/bin/activate
python app.py
```

Open http://127.0.0.1:5000/ to view the simulator and live telemetry. For hardware use, ensure Arduino is connected on `SERIAL_PORT` (default `/dev/ttyUSB0`) and the Arduino sketch in `firmware/` is loaded.

Tuning
------
- `steps_per_pixel1/2` in `DualStepperController` — maps pixel error to step commands.
- `max_steps_per_frame` — limits per-frame motor update to avoid missed steps.
- `deadband_px` — ignore small pixel jitter inside this threshold.
- `y_smooth_chunk` / `y_smooth_delay` — smooth large Y moves into chunks to avoid stepper stalls.

Notes
-----
- This README is a concise reference. For implementation details, see `controller.py`, `app.py`, and `sun_tracker.py`.
- Hardware testing is required to validate step/degree mapping and tune chunk sizes and delays for your specific motors and drivers.
# Optical-Sextant-Tracker
