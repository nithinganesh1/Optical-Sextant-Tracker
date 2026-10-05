import time
from types import SimpleNamespace

import app
from app import Mount


def test_parse_accepts_legacy_status_with_homed_flag_only():
    m = Mount.__new__(Mount)
    m.steps = {"x": None, "y": None}
    m.busy = {"x": False, "y": False}
    m.limit = False
    m.homing = False
    m.homed = False
    m.last_rx = 0.0
    m.status_ts = 0.0
    m.y_origin = None
    m._guard_until = 0.0
    m.fault = None
    m.home_pending = False

    m._parse("S 1200 300 0 0 1")

    assert m.steps == {"x": 1200, "y": 300}
    assert m.homed is True
    assert m.last_rx > 0
    assert m.status_ts > 0

    m2 = Mount.__new__(Mount)
    m2.steps = {"x": None, "y": None}
    m2.busy = {"x": False, "y": False}
    m2.limit = False
    m2.homing = False
    m2.homed = False
    m2.last_rx = 0.0
    m2.status_ts = 0.0
    m2.y_origin = None
    m2._guard_until = 0.0
    m2.fault = None
    m2.home_pending = False

    m2._parse("S 1200 300 0 0 1 1 0")

    assert m2.steps == {"x": 1200, "y": 300}
    assert m2.busy == {"x": True, "y": False}
    assert m2.homed is True


def test_build_status_reports_raw_step_angle_before_reference():
    original_mount = app.mount
    original_base_ctl = app.base_ctl
    app.mount = SimpleNamespace(connected=True, link_ok=True, fault=None, homing=False,
                               home_pending=False, homed=True, steps={"x": 487, "y": 0},
                               limit=False, busy={"x": False, "y": False})
    app.base_ctl = SimpleNamespace(enabled=True, state="GOING TO 90", phase="GO_REF", sun=False, err=None, cam=SimpleNamespace(status="ok"))
    app.y_ctl = SimpleNamespace(state="WAITING FOR BASE", phase="RUN", sun=False, err=None, cam=SimpleNamespace(status="ok"))
    try:
        status = app.build_status()
        assert status["mirror_angle"] < 0.0
        assert status["sun_angle"] < 0.0
        assert abs(status["mirror_angle"] + 62.61) < 1.0
        assert abs(status["sun_angle"] + 125.21) < 1.0
    finally:
        app.mount = original_mount
        app.base_ctl = original_base_ctl


def test_y_axis_waits_for_base_to_reach_reference_before_searching():
    fake_mount = SimpleNamespace(
        link_ok=True,
        homed=True,
        homing=False,
        home_pending=False,
        pos=lambda axis: 100 if axis == "x" else 0,
        move_done=lambda axis: True,
        send_move=lambda axis, target: None,
        busy={"x": False, "y": False},
        fault=None,
    )
    original_mount = app.mount
    app.mount = fake_mount
    try:
        fake_cam = SimpleNamespace(
            healthy=lambda: True,
            latest=lambda: ({"detected": False, "fresh": True, "error_x": 0.0}, time.time()),
            request_reset=lambda: None,
        )
        y = app.AxisController("Y", "y", fake_cam, "error_x", "y", lambda: (0, 200))
        app.base_ctl = SimpleNamespace(state="GOING TO 90")
        y.state = "STARTING"
        y.phase = "RUN"
        y.last_seen = 0.0

        y.step()

        assert y.state == "WAITING FOR BASE"
    finally:
        app.mount = original_mount
