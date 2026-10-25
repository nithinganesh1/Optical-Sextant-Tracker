"""
dashboard_server.py  -  live Flask dashboard for the sun tracker
=================================================================
Starts the real tracking loop (cameras + steppers + IMU, see tracker_runtime.py) and serves a web page that
shows everything live. It REPLACES app.py while it runs (both need the cameras and the serial port), so do not
start app.py at the same time.

    pip install flask pyserial opencv-python numpy
    python dashboard_server.py                       # http://127.0.0.1:5000
    python dashboard_server.py --host 0.0.0.0        # reachable from other devices on the network
    python dashboard_server.py --port-serial COM3 --no-camera2

Needs in the same folder: app.py, sun_controller.py, suntracker.py, stepper_link.py, sextant.py, tracker_runtime.py,
templates/dashboard.html, static/dashboard.css, static/optics.js, static/dashboard.js

Routes:
    /                 the dashboard
    /api/stream       Server-Sent Events, one JSON snapshot every 100 ms
    /api/state        the newest snapshot once (handy for debugging)
    /api/command      POST {"cmd": "...", ...}  -> home, x_tracking, y_tracking, gimbal, nudge_x, nudge_y, goto_x, reconnect
    /video/cam1|cam2  MJPEG streams with the detection overlay
"""

import argparse
import json
import time

from flask import Flask, Response, jsonify, render_template, request, stream_with_context

from tracker_runtime import TrackerRuntime

flask_app = Flask(__name__, static_folder="static", template_folder="templates")
runtime = None          # set in main() (or by a test)

STREAM_PERIOD_S = 0.1


def _json_default(o):
    return str(o)


@flask_app.route("/")
def index():
    return render_template("dashboard.html")


@flask_app.get("/api/state")
def api_state():
    return Response(json.dumps(runtime.snapshot(), default=_json_default), mimetype="application/json",
                    headers={"Cache-Control": "no-store"})


@flask_app.get("/api/stream")
def api_stream():
    @stream_with_context
    def generate():
        yield "retry: 1000\n\n"
        last_ts = None
        while True:
            snap = runtime.snapshot()
            ts = snap.get("timestamp")
            if ts != last_ts:                     # only send when the tracker has produced something new
                last_ts = ts
                yield f"data: {json.dumps(snap, default=_json_default)}\n\n"
            else:
                yield ": keep-alive\n\n"
            time.sleep(STREAM_PERIOD_S)

    return Response(generate(), mimetype="text/event-stream",
                    headers={"Cache-Control": "no-store", "X-Accel-Buffering": "no"})


@flask_app.post("/api/command")
def api_command():
    body = request.get_json(silent=True) or {}
    name = body.pop("cmd", None)
    if not isinstance(name, str):
        return jsonify(ok=False, message="'cmd' is required"), 400
    ok, message = runtime.submit(name, body)
    return jsonify(ok=ok, message=message), (200 if ok else 400)


@flask_app.get("/video/<cam>")
def video(cam):
    if cam not in ("cam1", "cam2"):
        return jsonify(ok=False, message="unknown camera"), 404

    @stream_with_context
    def generate():
        for jpeg in runtime.mjpeg_frames(cam):
            yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(jpeg)).encode() +
                   b"\r\n\r\n" + jpeg + b"\r\n")

    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame",
                    headers={"Cache-Control": "no-store"})


def main():
    global runtime
    p = argparse.ArgumentParser(description="Sun tracker live dashboard")
    p.add_argument("--host", default="127.0.0.1", help="use 0.0.0.0 to open it to the network (the page can move the motors!)")
    p.add_argument("--port", type=int, default=5000, help="web server port")
    p.add_argument("--port-serial", default=None, help="Arduino serial port (default: SERIAL_PORT from app.py)")
    p.add_argument("--no-steppers", action="store_true", help="cameras only, no Arduino")
    p.add_argument("--no-camera2", action="store_true", help="Camera 1 only")
    p.add_argument("--no-verbose", action="store_true", help="suppress raw serial line printing (verbose is ON by default)")
    a = p.parse_args()

    runtime = TrackerRuntime(serial_port=a.port_serial,
                             enable_steppers=False if a.no_steppers else None,
                             enable_camera2=False if a.no_camera2 else None,
                             verbose_serial=not a.no_verbose)
    runtime.start()
    print(f"Dashboard: http://{'localhost' if a.host in ('0.0.0.0', '127.0.0.1') else a.host}:{a.port}")
    try:
        flask_app.run(host=a.host, port=a.port, threaded=True, debug=False, use_reloader=False)
    finally:
        runtime.stop()


if __name__ == "__main__":
    main()
