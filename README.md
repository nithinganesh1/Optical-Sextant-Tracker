# Suntracker Dashboard

A simple project for a sun-tracking system with a Python dashboard, OpenCV-based tracking, and Arduino/ESP32 hardware control.

## Project layout

- `app.py` — camera tracking and control logic
- `dashboard_server.py` — Flask dashboard server
- `serial_check.py` — serial port debugging helper
- `suntracker.py` — sun detection logic
- `sun_controller.py` — control logic for motors/axes
- `stepper_link.py` — serial stepper communication
- `tracker_runtime.py` — tracker runtime logic
- `static/` — frontend CSS/JS assets
- `templates/` — dashboard HTML templates
- `esp32_bridge/` — ESP32 firmware
- `gimbol/` — gimbal/IMU firmware
- `gimbol_x_z/` — alternate gimbal firmware

## Quick start

1. Create and activate a virtual environment:
   ```bash
   python -m venv .venv
   source .venv/bin/activate
   ```

2. Install Python dependencies:
   ```bash
   python -m pip install flask pyserial numpy opencv-python
   ```

3. Run the dashboard:
   ```bash
   python dashboard_server.py
   ```

4. Or run the local tracking app:
   ```bash
   python app.py
   ```

## Notes

- Serial device names may vary, for example `/dev/ttyUSB0` or `/dev/ttyACM0`.
- Hardware wiring and pin configuration must be checked before running the motors.
- The Arduino sketches in the `gimbol/` and `esp32_bridge/` folders are part of the control setup and should be uploaded using the Arduino IDE or platform tools.

## License

This project is provided as-is for personal and experimental use.
