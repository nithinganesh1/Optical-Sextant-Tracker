# Suntracker Dashboard

This project combines a Python-based tracking dashboard with hardware control for a sun-tracking system using cameras, serial communication, and embedded controllers.

## Project layout

- `app.py` — camera tracking and motion logic
- `dashboard_server.py` — Flask dashboard server
- `serial_check.py` — serial port validation helper
- `suntracker.py` — sun detection and image processing
- `sun_controller.py` — tracking and axis controller logic
- `stepper_link.py` — stepper serial communication wrapper
- `tracker_runtime.py` — runtime tracker integration
- `static/` — CSS and JavaScript for the dashboard UI
- `templates/` — HTML dashboard templates
- `esp32_bridge/` — ESP32 firmware files
- `gimbol/` — IMU/gimbal stabilization firmware
- `gimbol_x_z/` — alternate gimbal firmware

## Quick start

### 1) Create a virtual environment

```bash
python -m venv .venv
source .venv/bin/activate
```

### 2) Install dependencies

```bash
pip install -r requirements.txt
```

### 3) Run the dashboard

```bash
python dashboard_server.py
```

### 4) Run the local tracker app

```bash
python app.py
```

## Hardware notes

- Serial ports may appear as `/dev/ttyUSB0`, `/dev/ttyACM0`, or similar.
- Check camera index values in `app.py` before running live tracking.
- Ensure the stepper and embedded firmware are wired and tested before enabling motion.
- The Arduino sketches in `gimbol/` and `esp32_bridge/` are intended for hardware setup and control.

## Typical usage

- Start the Flask dashboard.
- Connect the serial device for the tracking hardware.
- Verify the IMU or motor controller output before motion control.
- Use the dashboard UI to monitor tracker state and live camera feed.

## Notes

This is a research / hardware-control project and may require calibration and adjustments depending on your camera, motors, and enclosure setup.

## License

This project is provided as-is for personal, experimental, and educational use.
