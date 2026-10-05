"""
sun_tracker.py
==================================================================
Reusable, servo-ready Sun detector for a 2-axis gimbal tracking rig
(SmartCam S600 + ND filter). Classical OpenCV only — no AI models.

SEXTANT MODE
------------
This build is restricted to a fixed ROI (the green rectangle) and
reports error relative to a fixed reference line (the red line) —
NOT the full frame and NOT the frame center. Everything outside the
green rectangle is ignored entirely (never searched, never scored).
The sun is considered "centered" when it sits on the red line inside
that rectangle.

Pipeline
--------
1. Crop to ROI: all processing below runs only inside the green
   rectangle. Nothing outside it is touched.
2. Pre-process: bilateral filter (edge-preserving) + median blur
   (kills speckle/sensor noise while keeping the sun's disc edge sharp).
3. Dynamic threshold: max(Otsu, high percentile of intensities),
   clamped to a sane floor/ceiling so it survives everything from a
   near-uniform overexposed frame (ND filter, midday) to a dimmer
   sun near dawn/dusk.
4. Morphological open (remove specks) + close (fill sunspot / lens
   flare gaps, bridge cloud-edge notches in the disc).
5. External contours -> candidate blobs (in ROI-local coordinates).
6. Candidate filtering/scoring on: brightness, area (vs. expected
   sun size for the ROI), circularity, solidity, aspect ratio.
7. Hough-circle cross-check in a tight ROI around the best contour
   candidate to refine center/radius to sub-pixel-ish precision.
8. Coordinates are translated back to full-frame space (add ROI
   offset) before temporal filtering / output, so center/error are
   directly usable for drawing on the original frame.
9. Temporal stability:
     - Kalman filter (constant-velocity model) smooths (cx, cy) and
       predicts through brief dropouts (cloud, glare flicker, bird).
     - Outlier gating: a fresh measurement that jumps too far from
       the Kalman prediction is provisionally rejected; only after
       it repeats for N consecutive frames (a real sun/gimbal move,
       not noise) does the filter re-anchor to it.
     - EMA smooths radius and confidence (scalar quantities).
10. Confidence blends candidate shape score, Kalman innovation
    (residual) size, and how many consecutive frames we've held lock.
11. error_x / error_y are computed against the RED LINE reference
    point (target_point), not the frame center.

Output (per frame, from `SunTracker.update`)
---------------------------------------------------------------
{
    "detected": bool,
    "center": (cx, cy)   # smoothed, servo-ready, ints, full-frame coords
    "radius": float,
    "confidence": float,   # 0..1
    "error_x": int,        # cx - target_x   (red line reference)
    "error_y": int,        # cy - target_y   (red line reference)
    "on_target": bool,     # True once the sun disc covers the target point
}
"""

import time
import math
from collections import deque

import cv2
import numpy as np


class SunTracker:
    def __init__(
        self,
        roi: tuple = None,               # (x0, y0, x1, y1) — green rectangle, full-frame coords. None = whole frame.
        target_point: tuple = None,      # (tx, ty) — red line reference point, full-frame coords. None = ROI center (or frame center if roi is None).
        on_target_tolerance_px: int = 6, # how close (cx,cy) must be to target_point to count as "on_target" beyond disc coverage
        min_radius_px: int = 4,
        max_radius_ratio: float = 0.5,    # sun disc shouldn't exceed this fraction of the ROI's shorter side
        min_area_px: int = 20,            # px^2, filters pure noise specks
        threshold_percentile: float = 99.5,
        threshold_floor: int = 140,
        threshold_ceiling: int = 253,
        ema_alpha_radius: float = 0.35,
        ema_alpha_confidence: float = 0.4,
        kalman_process_noise: float = 5e-3,
        kalman_measurement_noise: float = 3e-2,
        max_jump_px: int = 90,            # gate: measurement vs prediction
        outlier_persist_frames: int = 4,  # frames needed to re-anchor
        lost_track_timeout_s: float = 1.0,
        fps_window: int = 30,
        debug: bool = False,   # print why candidates are accepted/rejected each frame
    ):
        self.roi = roi  # (x0, y0, x1, y1) or None
        self._explicit_target_point = target_point
        self.debug = debug

        self.on_target_tolerance_px = on_target_tolerance_px
        self.min_radius_px = min_radius_px
        self.max_radius_ratio = max_radius_ratio
        self.min_area_px = min_area_px
        self.threshold_percentile = threshold_percentile
        self.threshold_floor = threshold_floor
        self.threshold_ceiling = threshold_ceiling

        self.ema_alpha_radius = ema_alpha_radius
        self.ema_alpha_confidence = ema_alpha_confidence

        self.kalman_process_noise = kalman_process_noise
        self.kalman_measurement_noise = kalman_measurement_noise
        self.max_jump_px = max_jump_px
        self.outlier_persist_frames = outlier_persist_frames
        self.lost_track_timeout_s = lost_track_timeout_s

        # --- state ---
        self.kalman = None
        self.kalman_initialized = False
        self.smoothed_radius = None
        self.smoothed_brightness = 0.0
        self.smoothed_confidence = 0.0
        self.consecutive_hits = 0
        self.frames_since_detection = 0
        self.last_detection_time = None

        # outlier gating state
        self._pending_center = None
        self._pending_count = 0

        # FPS bookkeeping
        self._frame_times = deque(maxlen=fps_window)
        self.fps = 0.0

        self._last_result = None  # for draw_overlay convenience
        self._last_mask = None    # ROI-local threshold mask, for debug view

    # ------------------------------------------------------------------
    # ROI / target-point resolution
    # ------------------------------------------------------------------
    def _resolve_roi(self, frame_shape):
        h, w = frame_shape[:2]
        if self.roi is None:
            return 0, 0, w, h
        x0, y0, x1, y1 = self.roi
        x0 = max(0, min(x0, w - 1))
        y0 = max(0, min(y0, h - 1))
        x1 = max(x0 + 1, min(x1, w))
        y1 = max(y0 + 1, min(y1, h))
        return x0, y0, x1, y1

    def _resolve_target_point(self, roi_box):
        if self._explicit_target_point is not None:
            return self._explicit_target_point
        x0, y0, x1, y1 = roi_box
        # Default: middle of the ROI (i.e. where the red line would sit
        # if it runs horizontally through the rectangle's center).
        return ((x0 + x1) // 2, (y0 + y1) // 2)

    # ------------------------------------------------------------------
    # Kalman filter (constant velocity, 4-state / 2-measurement)
    # ------------------------------------------------------------------
    def _init_kalman(self, x: float, y: float):
        kf = cv2.KalmanFilter(4, 2)
        kf.transitionMatrix = np.array(
            [[1, 0, 1, 0],
             [0, 1, 0, 1],
             [0, 0, 1, 0],
             [0, 0, 0, 1]], dtype=np.float32,
        )
        kf.measurementMatrix = np.array(
            [[1, 0, 0, 0],
             [0, 1, 0, 0]], dtype=np.float32,
        )
        kf.processNoiseCov = np.eye(4, dtype=np.float32) * self.kalman_process_noise
        kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * self.kalman_measurement_noise
        kf.errorCovPost = np.eye(4, dtype=np.float32)
        kf.statePost = np.array([[x], [y], [0], [0]], dtype=np.float32)
        self.kalman = kf
        self.kalman_initialized = True
        self.consecutive_hits = 1
        self._pending_center = None
        self._pending_count = 0

    def _kalman_predict(self):
        pred = self.kalman.predict()
        return float(pred[0, 0]), float(pred[1, 0])

    def _kalman_correct(self, x: float, y: float):
        measurement = np.array([[np.float32(x)], [np.float32(y)]])
        corrected = self.kalman.correct(measurement)
        return float(corrected[0, 0]), float(corrected[1, 0])

    # ------------------------------------------------------------------
    # Pre-processing + thresholding (operates on the ROI crop only)
    # ------------------------------------------------------------------
    def _preprocess(self, roi_frame: np.ndarray):
        gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)
        # Edge-preserving smoothing kills sensor speckle without
        # blurring away the sun's disc boundary.
        smooth = cv2.bilateralFilter(gray, d=7, sigmaColor=50, sigmaSpace=50)
        smooth = cv2.medianBlur(smooth, 5)
        return gray, smooth

    def _dynamic_threshold(self, smooth: np.ndarray, percentile: float = None, floor: int = None):
        # Otsu gives a data-driven split; the high percentile guards
        # against Otsu collapsing when the ROI is nearly uniform
        # (e.g. sun fills most of the ND-filtered frame at midday).
        percentile = self.threshold_percentile if percentile is None else percentile
        floor = self.threshold_floor if floor is None else floor

        otsu_val, _ = cv2.threshold(smooth, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        percentile_val = float(np.percentile(smooth, percentile))
        thresh_val = max(otsu_val, percentile_val)
        thresh_val = float(np.clip(thresh_val, floor, self.threshold_ceiling))

        _, mask = cv2.threshold(smooth, thresh_val, 255, cv2.THRESH_BINARY)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close, iterations=2)
        return mask, thresh_val

    # ------------------------------------------------------------------
    # Candidate extraction + scoring (ROI-local coordinates in, ROI-local out)
    # ------------------------------------------------------------------
    def _find_candidates(self, mask: np.ndarray, gray: np.ndarray, roi_shape):
        h, w = roi_shape[:2]
        max_radius_px = self.max_radius_ratio * min(h, w)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []

        if self.debug:
            print(f"[SunTracker] roi_shape={w}x{h}  radius_bounds=[{self.min_radius_px}, {max_radius_px:.1f}]  "
                  f"min_area_px={self.min_area_px}  contours_found={len(contours)}")

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area_px:
                if self.debug:
                    print(f"[SunTracker]   reject: area={area:.1f} < min_area_px={self.min_area_px}")
                continue

            (cx, cy), radius = cv2.minEnclosingCircle(cnt)

            # A partial disc -- e.g. a mirror clipping the sun down to a
            # half-circle or crescent -- can make minEnclosingCircle read
            # a radius that doesn't match the true sun size. Fall back to
            # an area-equivalent radius (sqrt(area/pi)) and accept the
            # candidate if EITHER estimate is plausible, instead of
            # rejecting outright on the enclosing-circle radius alone.
            area_equiv_radius = math.sqrt(area / math.pi) if area > 0 else 0.0
            enc_ok = self.min_radius_px <= radius <= max_radius_px
            area_ok = self.min_radius_px <= area_equiv_radius <= max_radius_px
            if not enc_ok and not area_ok:
                if self.debug:
                    print(f"[SunTracker]   reject: enc_radius={radius:.1f} area_equiv_radius={area_equiv_radius:.1f} "
                          f"both outside bounds [{self.min_radius_px}, {max_radius_px:.1f}]")
                continue

            perimeter = cv2.arcLength(cnt, True)
            if perimeter <= 0:
                continue
            circularity = 4 * math.pi * area / (perimeter ** 2)  # 1.0 = perfect circle

            hull = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)
            solidity = area / hull_area if hull_area > 0 else 0

            x, y, bw, bh = cv2.boundingRect(cnt)
            aspect_ratio = min(bw, bh) / max(bw, bh) if max(bw, bh) > 0 else 0

            blob_mask = np.zeros(mask.shape, dtype=np.uint8)
            cv2.drawContours(blob_mask, [cnt], -1, 255, -1)
            mean_brightness = cv2.mean(gray, mask=blob_mask)[0]

            if self.debug:
                print(f"[SunTracker]   accept: area={area:.1f} enc_radius={radius:.1f} "
                      f"area_equiv_radius={area_equiv_radius:.1f} circularity={circularity:.2f} "
                      f"solidity={solidity:.2f} aspect={aspect_ratio:.2f} brightness={mean_brightness:.0f}")

            candidates.append({
                "contour": cnt,
                "center": (cx, cy),        # ROI-local
                "radius": radius,
                "area": area,
                "circularity": circularity,
                "solidity": solidity,
                "aspect_ratio": aspect_ratio,
                "brightness": mean_brightness,
            })

        return candidates

    def _score_candidate(self, cand: dict, roi_shape) -> float:
        h, w = roi_shape[:2]
        expected_r = self.max_radius_ratio * min(h, w) * 0.5  # rough mid-range prior

        circularity_score = min(cand["circularity"], 1.0)
        solidity_score = min(cand["solidity"], 1.0)
        aspect_score = cand["aspect_ratio"]
        brightness_score = min(cand["brightness"] / 255.0, 1.0)
        size_score = 1.0 - min(abs(cand["radius"] - expected_r) / (expected_r + 1e-6), 1.0)

        # A half-disc (mirror clipping the sun) is still convex (high
        # solidity) and fairly circular (~0.75 vs. 1.0 for a full disc),
        # but its bounding-box aspect ratio is naturally far from square
        # (~0.5, not ~1.0). Weighting aspect_ratio heavily used to punish
        # a perfectly valid partial-disc detection just for being clipped.
        # Solidity + brightness are the reliable signals for a partial
        # disc, so they carry more weight; aspect_ratio carries less.
        score = (
            0.25 * circularity_score +
            0.25 * solidity_score +
            0.05 * aspect_score +
            0.30 * brightness_score +
            0.15 * size_score
        )
        return float(np.clip(score, 0.0, 1.0))

    def _refine_with_hough(self, gray: np.ndarray, cand: dict):
        """Cross-check the best contour candidate with HoughCircles in a
        tight ROI-local window to reduce edge-pixelation jitter on the
        radius/center. Operates entirely in ROI-local coordinates."""
        cx, cy = cand["center"]
        r = cand["radius"]
        pad = int(r * 1.6) + 10
        h, w = gray.shape[:2]

        x0, x1 = max(0, int(cx - pad)), min(w, int(cx + pad))
        y0, y1 = max(0, int(cy - pad)), min(h, int(cy + pad))
        sub = gray[y0:y1, x0:x1]
        if sub.size == 0 or sub.shape[0] < 10 or sub.shape[1] < 10:
            return cand["center"], cand["radius"]

        sub_blur = cv2.GaussianBlur(sub, (5, 5), 0)
        circles = cv2.HoughCircles(
            sub_blur, cv2.HOUGH_GRADIENT, dp=1.2,
            minDist=max(sub.shape),
            param1=80, param2=25,
            minRadius=max(1, int(r * 0.6)),
            maxRadius=int(r * 1.4) + 2,
        )
        if circles is None:
            return cand["center"], cand["radius"]

        hx, hy, hr = circles[0][0]
        refined_cx = x0 + hx
        refined_cy = y0 + hy
        # Blend contour-based and Hough-based estimates rather than
        # fully trusting either — Hough can lock onto glare rings.
        blended_cx = 0.5 * cx + 0.5 * refined_cx
        blended_cy = 0.5 * cy + 0.5 * refined_cy
        blended_r = 0.5 * r + 0.5 * hr
        return (blended_cx, blended_cy), blended_r

    # ------------------------------------------------------------------
    # Main entry point
    # ------------------------------------------------------------------
    def update(self, frame: np.ndarray) -> dict:
        now = time.time()
        self._frame_times.append(now)
        if len(self._frame_times) >= 2:
            dt = self._frame_times[-1] - self._frame_times[0]
            self.fps = (len(self._frame_times) - 1) / dt if dt > 0 else 0.0

        roi_x0, roi_y0, roi_x1, roi_y1 = self._resolve_roi(frame.shape)
        target_point = self._resolve_target_point((roi_x0, roi_y0, roi_x1, roi_y1))

        # --- Only the green rectangle is ever looked at. ---
        roi_frame = frame[roi_y0:roi_y1, roi_x0:roi_x1]

        gray, smooth = self._preprocess(roi_frame)
        mask, _thresh_val = self._dynamic_threshold(smooth)
        candidates = self._find_candidates(mask, gray, roi_frame.shape)

        if not candidates:
            # Nothing crossed the normal threshold. A mirror showing only
            # half (or a crescent) of the sun reads dimmer than a full
            # disc and can miss it entirely -- retry once with a more
            # permissive threshold before giving up on this frame.
            relaxed_percentile = max(85.0, self.threshold_percentile - 15.0)
            relaxed_floor = max(60, self.threshold_floor - 40)
            relaxed_mask, _ = self._dynamic_threshold(
                smooth, percentile=relaxed_percentile, floor=relaxed_floor
            )
            relaxed_candidates = self._find_candidates(relaxed_mask, gray, roi_frame.shape)
            if relaxed_candidates:
                mask = relaxed_mask
                candidates = relaxed_candidates

        self._last_mask = mask  # ROI-local, exposed for debug/testing imshow

        best = None
        best_score = -1.0
        for cand in candidates:
            score = self._score_candidate(cand, roi_frame.shape)
            if score > best_score:
                best_score = score
                best = cand

        measurement = None
        current_brightness = 0.0
        if best is not None:
            refined_center, refined_radius = self._refine_with_hough(gray, best)
            # Translate ROI-local -> full-frame coordinates.
            full_cx = refined_center[0] + roi_x0
            full_cy = refined_center[1] + roi_y0
            measurement = (full_cx, full_cy, refined_radius, best_score)
            current_brightness = best["brightness"]

        roi_box = (roi_x0, roi_y0, roi_x1, roi_y1)
        result = self._apply_temporal_stability(measurement, current_brightness, target_point, roi_box)
        self._last_result = result
        self._last_roi_box = roi_box
        self._last_target_point = target_point
        return result

    def _apply_temporal_stability(self, measurement, current_brightness, target_point, roi_box):
        tx, ty = target_point

        if not self.kalman_initialized:
            if measurement is not None:
                mx, my, mr, mscore = measurement
                self._init_kalman(mx, my)
                self.smoothed_radius = mr
                self.smoothed_brightness = current_brightness
                self.smoothed_confidence = mscore
                self.frames_since_detection = 0
                self.last_detection_time = time.time()
                cx, cy = self._kalman_predict()
                return self._pack_result(True, cx, cy, self.smoothed_radius,
                                          self.smoothed_confidence, self.smoothed_brightness, tx, ty, roi_box)
            return self._pack_result(False, tx, ty, 0.0, 0.0, 0.0, tx, ty, roi_box)

        # Predict first (constant-velocity motion model).
        pred_x, pred_y = self._kalman_predict()

        if measurement is None:
            self.frames_since_detection += 1
            self.consecutive_hits = 0
            self._pending_center = None
            self._pending_count = 0
            decay = max(0.0, 1.0 - self.frames_since_detection * 0.25)
            self.smoothed_confidence *= decay

            timed_out = (
                self.last_detection_time is not None and
                (time.time() - self.last_detection_time) > self.lost_track_timeout_s
            )

            if timed_out:
                self.reset()
                return self._pack_result(False, tx, ty, 0.0, 0.0, 0.0, tx, ty, roi_box)

            detected = self.smoothed_confidence > 0.05
            return self._pack_result(detected, pred_x, pred_y,
                                      self.smoothed_radius or 0.0,
                                      self.smoothed_confidence,
                                      self.smoothed_brightness, tx, ty, roi_box)

        mx, my, mr, mscore = measurement
        dist = math.hypot(mx - pred_x, my - pred_y)

        if dist <= self.max_jump_px:
            # Consistent with the motion model -> trust it.
            cx, cy = self._kalman_correct(mx, my)
            self.consecutive_hits += 1
            self._pending_center = None
            self._pending_count = 0
        else:
            # Possible outlier (noise/glare) OR a genuine sun/gimbal
            # jump. Only re-anchor after it repeats for N frames.
            if self._pending_center is not None and \
               math.hypot(mx - self._pending_center[0], my - self._pending_center[1]) <= self.max_jump_px:
                self._pending_count += 1
            else:
                self._pending_count = 1
            self._pending_center = (mx, my)

            if self._pending_count >= self.outlier_persist_frames:
                self._init_kalman(mx, my)
                cx, cy = mx, my
            else:
                # Ignore this measurement for now; keep the prediction.
                cx, cy = pred_x, pred_y
                mscore *= 0.5  # penalize confidence while unresolved

        # EMA smoothing for scalar quantities.
        if self.smoothed_radius is None:
            self.smoothed_radius = mr
            self.smoothed_brightness = current_brightness
        else:
            self.smoothed_radius = (
                self.ema_alpha_radius * mr +
                (1 - self.ema_alpha_radius) * self.smoothed_radius
            )
            self.smoothed_brightness = (
                self.ema_alpha_radius * current_brightness +
                (1 - self.ema_alpha_radius) * self.smoothed_brightness
            )

        residual = min(dist / self.max_jump_px, 1.0)
        consistency_score = 1.0 - residual
        streak_score = min(self.consecutive_hits / 10.0, 1.0)
        instant_confidence = 0.5 * mscore + 0.3 * consistency_score + 0.2 * streak_score

        self.smoothed_confidence = (
            self.ema_alpha_confidence * instant_confidence +
            (1 - self.ema_alpha_confidence) * self.smoothed_confidence
        )

        self.frames_since_detection = 0
        self.last_detection_time = time.time()

        return self._pack_result(True, cx, cy, self.smoothed_radius,
                                  self.smoothed_confidence, self.smoothed_brightness, tx, ty, roi_box)

    def _pack_result(self, detected, cx, cy, radius, confidence, brightness, tx, ty, roi_box):
        cx_i, cy_i = int(round(cx)), int(round(cy))
        
        # If the coordinates are outside the ROI, force detected to False
        if roi_box is not None and detected:
            x0, y0, x1, y1 = roi_box
            if cx_i < x0 or cx_i > x1 or cy_i < y0 or cy_i > y1:
                detected = False

        error_x = cx_i - tx
        error_y = cy_i - ty
        dist_to_target = math.hypot(error_x, error_y)
        on_target = detected and (
            dist_to_target <= max(radius, self.on_target_tolerance_px)
        )
        return {
            "detected": bool(detected),
            "center": (cx_i, cy_i),
            "radius": float(radius),
            "brightness": float(brightness),
            "confidence": float(np.clip(confidence, 0.0, 1.0)),
            "error_x": error_x,
            "error_y": error_y,
            "on_target": bool(on_target),
        }

    def reset(self):
        self.kalman = None
        self.kalman_initialized = False
        self.smoothed_radius = None
        self.smoothed_brightness = 0.0
        self.smoothed_confidence = 0.0
        self.consecutive_hits = 0
        self.frames_since_detection = 0
        self.last_detection_time = None
        self._pending_center = None
        self._pending_count = 0

    # ------------------------------------------------------------------
    # Visualization
    # ------------------------------------------------------------------
    def draw_overlay(self, frame: np.ndarray, result: dict = None) -> np.ndarray:
        result = result or self._last_result
        if result is None:
            return frame

        out = frame.copy()

        roi_box = getattr(self, "_last_roi_box", None)
        target_point = getattr(self, "_last_target_point", None)

        # Green ROI rectangle (the only area that gets checked).
        if roi_box is not None:
            rx0, ry0, rx1, ry1 = roi_box
            cv2.rectangle(out, (rx0, ry0), (rx1, ry1), (0, 255, 0), 2)

        # Red reference line / target point.
        if target_point is not None:
            tx, ty = target_point
            if roi_box is not None:
                rx0, _, rx1, _ = roi_box
                cv2.line(out, (rx0, ty), (rx1, ty), (0, 0, 255), 2)
            cv2.drawMarker(out, (tx, ty), (0, 0, 255), cv2.MARKER_CROSS, 14, 2)

        if result["detected"]:
            cx, cy = result["center"]
            r = int(round(result["radius"]))
            color = (0, 255, 0) if result.get("on_target") else \
                    ((0, 255, 255) if result["confidence"] > 0.5 else (0, 165, 255))

            cv2.circle(out, (cx, cy), r, color, 2)          # sun circle
            cv2.circle(out, (cx, cy), 3, (0, 0, 255), -1)   # sun center dot
            if target_point is not None:
                cv2.line(out, target_point, (cx, cy), (200, 200, 200), 1)

            label = "ON TARGET" if result.get("on_target") else \
                    f"err=({result['error_x']:+d},{result['error_y']:+d})"
            cv2.putText(out, label, (cx + r + 8, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
        else:
            fh, fw = out.shape[:2]
            cv2.putText(out, "SUN NOT DETECTED", (fw // 2 - 90, fh // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)

        cv2.putText(out, f"FPS: {self.fps:.1f}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(out, f"Confidence: {result['confidence']:.2f}", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        return out


# ======================================================================
# Demo / manual test harness (camera control comes later — this only
# proves out detection + smoothing on a live or file-based feed).
# ======================================================================
if __name__ == "__main__":
    import sys

    source = 1
    if len(sys.argv) > 1:
        source = sys.argv[1]
        if source.isdigit():
            source = int(source)

    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        print(f"Could not open video source: {source}")
        sys.exit(1)

    # Green rectangle from the original overlay, and the red line running
    # through its vertical center — this is the only area SunTracker will
    # ever look at, and the point the sun needs to sit on.
    ROI = (306, 138, 383, 291)
    TARGET_POINT = ((306 + 383) // 2, 230)

    tracker = SunTracker(roi=ROI, target_point=TARGET_POINT, debug=True)

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frame = cv2.rotate(frame, cv2.ROTATE_180)

        result = tracker.update(frame)
        annotated = tracker.draw_overlay(frame, result)

        print(result)
        cv2.imshow("Sun Tracker", annotated)
        cv2.imshow("Processed (ROI threshold mask)", tracker._last_mask)  # debug view
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()