"""
suntracker.py  -  Step 1: reusable sun detection module
=======================================================
Takes ONE image (BGR numpy array, as given by cv2.VideoCapture / cv2.imread)
and returns where the sun is. It does NOT open cameras, talk to the Arduino
or move motors; app.py does that.

Both cameras use the same detector; only the way the result is used differs:
  Camera 1 (X axis): detect inside a small ROI, error = distance to ROI centre
                     -> use  error_to_point()
  Camera 2 (Y axis): detect in the full frame, error = distance to a diagonal
                     line   -> use  diagonal_line() + signed_distance_to_line()

Quick test (no camera needed):
    python suntracker.py                      # builds a synthetic test image
    python suntracker.py my_sun.jpg           # your own image, full frame
    python suntracker.py my_sun.jpg 306 138 383 291    # with ROI x0 y0 x1 y1
An annotated copy is saved as suntracker_result.png
"""

import math
import sys

import cv2
import numpy as np

# ----------------------------------------------------------------------
# Configurable defaults (override via SunDetector(...) arguments)
# ----------------------------------------------------------------------
THRESHOLD_PERCENTILE = 99.5   # sun = brightest pixels in the image/ROI
THRESHOLD_FLOOR = 140         # never threshold below this (0-255)
THRESHOLD_CEILING = 253       # never threshold above this
MIN_RADIUS_PX = 4             # ignore blobs smaller than this
MAX_RADIUS_RATIO = 0.5        # blob radius must be < ratio * shorter side
MIN_AREA_PX = 20              # ignore tiny specks
MIN_CONFIDENCE = 0.35         # below this the result is "not detected"


class SunDetector:
    def __init__(
        self,
        roi=None,                       # (x0, y0, x1, y1) in full-frame pixels, None = whole frame
        threshold_percentile=THRESHOLD_PERCENTILE,
        threshold_floor=THRESHOLD_FLOOR,
        threshold_ceiling=THRESHOLD_CEILING,
        min_radius_px=MIN_RADIUS_PX,
        max_radius_ratio=MAX_RADIUS_RATIO,
        min_area_px=MIN_AREA_PX,
        min_confidence=MIN_CONFIDENCE,
        debug=False,
    ):
        self.roi = roi
        self.threshold_percentile = threshold_percentile
        self.threshold_floor = threshold_floor
        self.threshold_ceiling = threshold_ceiling
        self.min_radius_px = min_radius_px
        self.max_radius_ratio = max_radius_ratio
        self.min_area_px = min_area_px
        self.min_confidence = min_confidence
        self.debug = debug
        self.last_mask = None   # ROI-local threshold mask, handy for imshow while tuning

    # ------------------------------------------------------------------
    def _resolve_roi(self, shape):
        h, w = shape[:2]
        if self.roi is None:
            return 0, 0, w, h
        x0, y0, x1, y1 = self.roi
        x0 = max(0, min(int(x0), w - 1))
        y0 = max(0, min(int(y0), h - 1))
        x1 = max(x0 + 1, min(int(x1), w))
        y1 = max(y0 + 1, min(int(y1), h))
        return x0, y0, x1, y1

    def _threshold(self, smooth, percentile, floor):
        otsu, _ = cv2.threshold(smooth, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        pct = float(np.percentile(smooth, percentile))
        t = float(np.clip(max(otsu, pct), floor, self.threshold_ceiling))
        _, mask = cv2.threshold(smooth, t, 255, cv2.THRESH_BINARY)
        k_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k_open)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k_close, iterations=2)
        return mask

    def _candidates(self, mask, gray):
        h, w = mask.shape[:2]
        max_r = self.max_radius_ratio * min(h, w)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        out = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area_px:
                continue
            (ex, ey), enc_r = cv2.minEnclosingCircle(cnt)
            area_r = math.sqrt(area / math.pi)
            # accept if either radius estimate is plausible (partial discs)
            if not (self.min_radius_px <= enc_r <= max_r or self.min_radius_px <= area_r <= max_r):
                continue
            per = cv2.arcLength(cnt, True)
            if per <= 0:
                continue
            circularity = min(4 * math.pi * area / per ** 2, 1.0)
            hull_area = cv2.contourArea(cv2.convexHull(cnt))
            solidity = min(area / hull_area, 1.0) if hull_area > 0 else 0.0
            blob = np.zeros(mask.shape, np.uint8)
            cv2.drawContours(blob, [cnt], -1, 255, -1)
            brightness = cv2.mean(gray, mask=blob)[0]
            m = cv2.moments(cnt)
            if m["m00"] == 0:
                continue
            cx, cy = m["m10"] / m["m00"], m["m01"] / m["m00"]   # centroid
            score = 0.35 * circularity + 0.30 * solidity + 0.35 * min(brightness / 255.0, 1.0)
            if self.debug:
                print(f"[SunDetector] area={area:.0f} r={area_r:.1f} circ={circularity:.2f} "
                      f"solid={solidity:.2f} bright={brightness:.0f} score={score:.2f}")
            out.append((score, cx, cy, area_r, brightness))
        return out

    # ------------------------------------------------------------------
    def detect(self, frame):
        """
        frame: BGR (or grayscale) image.
        Returns a dict (coordinates are always FULL-FRAME pixels):
            detected   : bool
            center     : (x, y) ints, or None when not detected
            radius     : float px (0 when not detected)
            confidence : 0..1
            brightness : mean gray level of the sun blob (0..255)
            roi        : (x0, y0, x1, y1) actually used
        """
        if frame is None or frame.size == 0:
            raise ValueError("detect() got an empty frame")

        x0, y0, x1, y1 = self._resolve_roi(frame.shape)
        crop = frame[y0:y1, x0:x1]
        gray = crop if crop.ndim == 2 else cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        smooth = cv2.medianBlur(cv2.bilateralFilter(gray, 7, 50, 50), 5)

        mask = self._threshold(smooth, self.threshold_percentile, self.threshold_floor)
        cands = self._candidates(mask, gray)

        if not cands:  # retry once, more permissive (dim / partly hidden sun)
            relaxed = self._threshold(
                smooth,
                max(85.0, self.threshold_percentile - 15.0),
                max(60, self.threshold_floor - 40),
            )
            cands = self._candidates(relaxed, gray)
            if cands:
                mask = relaxed
        self.last_mask = mask

        result = {"detected": False, "center": None, "radius": 0.0,
                  "confidence": 0.0, "brightness": 0.0, "roi": (x0, y0, x1, y1)}
        if not cands:
            return result

        score, cx, cy, r, bright = max(cands, key=lambda c: c[0])
        if score < self.min_confidence:
            return result

        result.update(
            detected=True,
            center=(int(round(cx)) + x0, int(round(cy)) + y0),
            radius=float(r),
            confidence=float(np.clip(score, 0.0, 1.0)),
            brightness=float(bright),
        )
        return result


# ----------------------------------------------------------------------
# Helpers for the two different tracking styles
# ----------------------------------------------------------------------
def roi_center(roi):
    """Centre point (x, y) of an ROI box (x0, y0, x1, y1)."""
    x0, y0, x1, y1 = roi
    return ((x0 + x1) // 2, (y0 + y1) // 2)


def error_to_point(center, target):
    """Camera 1: (error_x, error_y) = sun - target. Positive x = sun is right of target."""
    return center[0] - target[0], center[1] - target[1]


def diagonal_line(frame_shape, end_fraction=(0.75, 0.75)):
    """
    Camera 2: reference line from the frame centre to a point in the
    bottom-right area. Default end = (75% width, 75% height), i.e. the centre
    of the bottom-right quarter. Change end_fraction if you meant another point.
    Returns ((x0, y0), (x1, y1)).
    """
    h, w = frame_shape[:2]
    return (w // 2, h // 2), (int(w * end_fraction[0]), int(h * end_fraction[1]))


def signed_distance_to_line(point, p0, p1):
    """
    Camera 2: perpendicular distance (px) from point to the infinite line p0->p1.
    Sign tells which side of the line the sun is on. 0 = on the line.
    """
    (x, y), (ax, ay), (bx, by) = point, p0, p1
    dx, dy = bx - ax, by - ay
    length = math.hypot(dx, dy)
    if length == 0:
        raise ValueError("line points are identical")
    return ((x - ax) * dy - (y - ay) * dx) / length


def draw_overlay(frame, result, roi=None, target=None, line=None):
    """Return a copy of frame with the ROI, target, line and detected sun drawn."""
    out = frame.copy()
    if roi is not None:
        cv2.rectangle(out, (roi[0], roi[1]), (roi[2], roi[3]), (0, 255, 0), 2)
    if target is not None:
        cv2.drawMarker(out, tuple(target), (0, 0, 255), cv2.MARKER_CROSS, 14, 2)
    if line is not None:
        cv2.line(out, line[0], line[1], (0, 0, 255), 2)
    if result["detected"]:
        cx, cy = result["center"]
        r = int(round(result["radius"]))
        cv2.circle(out, (cx, cy), r, (0, 255, 255), 2)
        cv2.circle(out, (cx, cy), 3, (0, 0, 255), -1)
        cv2.putText(out, f"({cx},{cy}) conf={result['confidence']:.2f}", (cx + r + 6, cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
    else:
        cv2.putText(out, "SUN NOT DETECTED", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
    return out


# ----------------------------------------------------------------------
# Self-test: python suntracker.py [image] [x0 y0 x1 y1]
# ----------------------------------------------------------------------
def _synthetic_image(w=640, h=480, sun=(344, 200), r=22):
    rng = np.random.default_rng(1)
    img = np.full((h, w, 3), 40, np.uint8)
    img = np.clip(img + rng.normal(0, 6, img.shape), 0, 255).astype(np.uint8)
    cv2.circle(img, (90, 400), 6, (255, 255, 255), -1)      # small bright distractor
    cv2.circle(img, sun, r, (255, 255, 255), -1)            # the "sun"
    return cv2.GaussianBlur(img, (9, 9), 0)


if __name__ == "__main__":
    args = sys.argv[1:]
    if args:
        frame = cv2.imread(args[0])
        if frame is None:
            sys.exit(f"Could not read image: {args[0]}")
        roi = tuple(int(v) for v in args[1:5]) if len(args) >= 5 else None
    else:
        print("No image given -> using a synthetic test image (sun at x=344, y=200).")
        frame = _synthetic_image()
        roi = (306, 138, 383, 291)

    detector = SunDetector(roi=roi, debug=True)
    res = detector.detect(frame)
    print(res)

    target = roi_center(roi) if roi else None
    if res["detected"] and target:
        print("error to ROI centre (x, y):", error_to_point(res["center"], target))
    if res["detected"] and roi is None:
        p0, p1 = diagonal_line(frame.shape)
        print("distance to diagonal line:", round(signed_distance_to_line(res["center"], p0, p1), 1))

    shown = draw_overlay(frame, res, roi=roi, target=target)
    cv2.imwrite("suntracker_result.png", shown)
    print("Saved suntracker_result.png")