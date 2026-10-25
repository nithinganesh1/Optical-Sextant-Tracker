"""
sextant.py  -  sextant maths for the sun tracker (pure functions: no hardware, no OpenCV, no Flask)
===================================================================================================
The optics are a double-reflection sextant:

    sun -> index mirror B (turned by motor X) -> fixed horizon mirror A -> telescope C

Motor X angle (logical degrees, as reported by stepper_link):
    X = 60 deg  -> the optics look at the surface level (horizon)   -> sextant reading   0 deg
    X =  0 deg  -> start of the scan                                 -> sextant reading 120 deg

    index mirror angle  theta = SURFACE_LEVEL_X_DEG - X          (how far B is turned from parallel to A)
    sextant reading     R     = 2 * theta                        (a mirror turn of theta moves the image by 2*theta)

So the scan range X = 60 ... 0 deg covers the whole sextant limb 0 ... 120 deg.
Sun altitude equals the reading when the two images coincide (the tracker is locked on the sun).
While the image is not yet centred, the camera error is converted into a small correction:

    altitude = R + 2 * DEGREES_PER_STEP * error_px / px_per_step

(px_per_step = how many pixels the sun image moves on Camera 1 per motor step, see tracker_runtime.py).
"""

SURFACE_LEVEL_X_DEG = 60.0     # X angle that corresponds to the surface level (reading 0)
LIMB_MAX_READING_DEG = 120.0   # end of the sextant arc (0 ... 120 deg)
MIRROR_A_DEG = 60.0            # fixed horizon mirror A (angle to the telescope axis)
ALTITUDE_MAX_DEG = 90.0        # the sun cannot be higher than the zenith
EPS = 1e-6


def index_mirror_deg(x_deg):
    """theta: turn of the index mirror B away from the position parallel to mirror A."""
    return SURFACE_LEVEL_X_DEG - x_deg


def reading_deg(x_deg):
    """Sextant reading for a motor X angle."""
    return 2.0 * index_mirror_deg(x_deg)


def x_deg_for_reading(reading):
    """Motor X angle that gives a wanted sextant reading."""
    return SURFACE_LEVEL_X_DEG - reading / 2.0


def residual_deg(err_px, px_per_step, deg_per_step):
    """
    How far (in degrees of altitude) the sun still is from the current reading, from Camera 1's pixel error.
    err_px = sun - target (+ = sun right of target). None if it cannot be calculated.
    """
    if err_px is None or not px_per_step:
        return None
    return 2.0 * deg_per_step * err_px / px_per_step


def compute(x_deg, err_px=None, px_per_step=None, deg_per_step=0.1125, sun_detected=False, locked=False):
    """
    All sextant values for one motor X angle (+ optional camera information). Returns a plain dict:
        x_deg, theta_deg, reading_deg, arc_fraction            geometry of the instrument
        in_scan_range    X is inside the 0 ... 60 deg scan range (reading 0 ... 120)
        on_limb          reading is inside the printed arc 0 ... 120
        residual_deg     remaining sun-vs-reading difference from the camera (None = unknown)
        sun_altitude_deg altitude of the sun (None = sun not seen, or outside 0 ... 90)
        zenith_distance_deg
        coincident       the tracker reports the sun centred (images coincide)
    """
    theta = index_mirror_deg(x_deg)
    reading = 2.0 * theta
    res = residual_deg(err_px, px_per_step, deg_per_step) if sun_detected else None
    altitude = None
    if sun_detected:
        altitude = reading + (res if res is not None else 0.0)
        if not (-EPS <= altitude <= ALTITUDE_MAX_DEG + EPS):
            altitude = None
    return {
        "x_deg": x_deg,
        "theta_deg": theta,
        "reading_deg": reading,
        "arc_fraction": reading / LIMB_MAX_READING_DEG,
        "in_scan_range": -EPS <= x_deg <= SURFACE_LEVEL_X_DEG + EPS,  # 0°..60° = sextant measurement range
        "in_motor_range": -60.0 - EPS <= x_deg <= SURFACE_LEVEL_X_DEG + EPS,  # -60°..60° = full motor range
        "on_limb": -EPS <= reading <= LIMB_MAX_READING_DEG + EPS,
        "mirror_a_deg": MIRROR_A_DEG,
        "residual_deg": res,
        "sun_altitude_deg": altitude,
        "zenith_distance_deg": None if altitude is None else ALTITUDE_MAX_DEG - altitude,
        "coincident": bool(sun_detected and locked),
        "surface_level_x_deg": SURFACE_LEVEL_X_DEG,
    }
