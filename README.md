# ESP32 Sun Tracker + Arduino Uno IMU bridge

ESP32 handles X/Y and Python USB serial. Arduino Uno independently stabilizes gimbal and sends `IMU,roll,pitch,yaw` every 100 ms from hardware D1 TX. ESP32 UART2 GPIO16 receives telemetry through a 1k/2k resistor divider, and Python requests status via `GS` (legacy roll status + new IMU data) or `I` (IMU only). No ESP32 gimbal motor control.

## Wiring
ESP32 X STEP25 DIR26, Y STEP27 DIR14, CNC ENABLE13; X limit34, Y home35, Y far39 (each requires 4.7k pull-up to 3.3 V). Uno D1 TX -> 1k -> ESP32 GPIO16; ESP32 GPIO16 -> 2k -> GND. Common Uno/ESP32 GND. ESP32 USB serial for Python (`/dev/ttyUSB0` is example). Keep Uno IMU and motor pins on Uno only. Do not connect Uno 5 V UART directly to ESP32.

## Safety / homing
`ALLOW_XY_HOMING=false` intentionally prevents uncontrolled homing. Test switch `L` responses first (unpressed `LIM 0 0 0` for normally-open GND switches; pressed gives 1), then validate direction & backlash with mechanism disengaged. Only then enable homing. `MEASURE_Y_RANGE=false` by default; enable only after validating Y far switch and safe travel. `H` homes, `P` reads coordinates, `L` reads switches, `<dx> <dy>` moves, `I` requests IMU. `GS` sends legacy GIMBAL line and IMU. Other tracker settings remain inherited from previous firmware. X/Y mechanical orientation and limit count MUST be checked before motion.

## Python
Files are based on uploaded originals with existing camera tracking retained; `tracker_runtime.py` parses `IMU,roll,pitch,yaw` and adds pitch_deg/yaw_deg in the gimbal JSON object. Existing dashboard JS may need UI tweaks to display pitch/yaw. The gimbal button is no longer valid; ESP32 deliberately rejects `G0/G1`.

## Upload / run
Arduino IDE: open esp32_xy_imu_bridge.ino with ESP32 Dev Module selected, upload; no 7Semi library needed on ESP32. The separate Uno needs its working stabilization and telemetry firmware. Python: pip install flask pyserial numpy opencv-python; run `python dashboard_server.py --port-serial /dev/ttyUSB0`. Check camera indices in app.py. Test direct serial command `I` and confirm `IMU,...` before launching dashboard.

## Notes
No hardware validation or ESP32 compile has been performed here. ESP32 pulse timing during X/Y moves is software-based; verify at operating speeds. The Uno's yaw is gyroscope-integrated and drifts. Intercontroller connection does not affect Uno stabilization, barring shared electrical faults. For safe operation, implement a real emergency stop and motion travel protection.
# Optical-Sextant-Tracker
