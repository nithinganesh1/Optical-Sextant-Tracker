"""
sun_controller.py  -  tracking logic (no camera, no serial in here)
===================================================================
AxisController turns a pixel error into a number of steps for ONE axis.
The same class is used for X (Camera 1) now and for Y (Camera 2) later,
only with different settings.

    ctrl = AxisController(gain=0.15, sign=+1, deadband_px=6)
    steps = ctrl.update(error_px, detected)      # call this every frame
    if steps: link.move_steps(steps, 0)

Rules it follows (so the motor does not jump or hunt):
  * no sun detected        -> do nothing                         (state NO_SUN)
  * inside the dead band   -> do nothing, we are on target       (state LOCKED)
  * sun must be seen steadily for confirm_s before the first move (state SETTLING)
  * after each move wait settle_s so the camera image catches up  (state SETTLING)
  * otherwise move  steps = sign * gain * error_px, limited to +-max_step (state MOVING)

error_px = sun position - target position, so with sign=+1 a positive error gives positive steps.
If the motor corrects the wrong way, flip sign. app.py has a calibration key that finds it for you.

LimitAwareController (used for Y) adds, on top of that:
  * search sweep while no sun is seen, reversing at each end (switch / soft limit), a limited number of times
  * if a limit is hit while chasing the sun, or the error keeps growing after moves,
    it reverses direction - but only a limited number of times, then it STOPS instead of
    going back and forth forever (state STUCK / NOT_FOUND)
  * it stops (LOCKED) as soon as the error is inside the dead band
"""

import time

NO_SUN = "NO_SUN"
SETTLING = "SETTLING"
MOVING = "MOVING"
LOCKED = "LOCKED"
SEARCHING = "SEARCHING"
NOT_FOUND = "NOT_FOUND"      # searched the whole range the allowed number of times, no sun
STUCK = "STUCK"              # sun seen but could not be reached (too many reversals)
ROTATING = "ROTATING"        # doing a full 360° wrap-around after the limit switch was hit


class AxisController:
    def __init__(self, gain, sign=1, deadband_px=6, max_step=40, min_step=1,
                 settle_s=0.5, confirm_s=0.15):
        self.gain = gain              # steps per pixel of error
        self.sign = 1 if sign >= 0 else -1
        self.deadband_px = deadband_px
        self.max_step = max_step      # largest single correction (steps)
        self.min_step = min_step      # smallest correction outside the dead band
        self.settle_s = settle_s      # wait after a move before judging again
        self.confirm_s = confirm_s    # sun must be seen this long before the first move
        self.state = NO_SUN
        self._first_seen = None
        self._last_move = 0.0

    def reset(self):
        self.state = NO_SUN
        self._first_seen = None
        self._last_move = 0.0

    def update(self, error_px, detected, on_target=None, now=None):
        """Returns the number of steps to move now (0 = do nothing).

        on_target: optional bool.  None = use normal deadband.  False = don't go LOCKED even
        when error is within the deadband (motor stays in SETTLING).  True = same as None.
        """
        now = time.time() if now is None else now

        if not detected or error_px is None:
            self._first_seen = None
            self.state = NO_SUN
            return 0

        if self._first_seen is None:
            self._first_seen = now

        if abs(error_px) <= self.deadband_px:
            if on_target is not False:
                self.state = LOCKED
                return 0
            else:
                # Within deadband but not on the target segment: wait in SETTLING, don't move
                self.state = SETTLING
                return 0

        if now - self._first_seen < self.confirm_s or now - self._last_move < self.settle_s:
            self.state = SETTLING
            return 0

        steps = int(round(self.sign * self.gain * error_px))
        if steps == 0:
            steps = self.min_step if (self.sign * error_px) > 0 else -self.min_step
        steps = max(-self.max_step, min(self.max_step, steps))

        self._last_move = now
        self._first_seen = None          # must be seen steadily again after the move
        self.state = MOVING
        return steps


class LimitAwareController(AxisController):
    """
    AxisController + search sweep + direction learning + bounded reversals.
    Call update() every frame, and notify_move_result() after every move you actually made.

    single_limit_mode=True: the Y axis has only ONE limit switch and circles 360°.
    When the limit is hit while tracking, instead of reversing and getting stuck,
    the controller immediately reverses the motor direction and issues a full 360°
    rotation so the motor comes back around to the sun from the other side.
    full_rotation_steps is set by the caller (tracker_runtime) from y_range_steps
    (measured travel between the home switch and itself going the long way around).
    """

    def __init__(self, gain, sign=1, deadband_px=6, max_step=40, min_step=1,
                 settle_s=0.5, confirm_s=0.15,
                 search_step=40, search_interval_s=0.2, max_search_reversals=2,
                 max_reversals=3, grow_px=3.0, grow_count=2, lost_window_s=2.0,
                 single_limit_mode=False, full_rotation_steps=3200):
        super().__init__(gain, sign, deadband_px, max_step, min_step, settle_s, confirm_s)
        self.search_step = search_step                  # steps per search move
        self.search_interval_s = search_interval_s      # time between search moves
        self.max_search_reversals = max_search_reversals  # end-to-end turn-arounds allowed while searching
        self.max_reversals = max_reversals              # direction reversals allowed while chasing the sun
        self.grow_px = grow_px                          # error must grow by more than this to count as "wrong way"
        self.grow_count = grow_count                    # ... this many moves in a row
        self.lost_window_s = lost_window_s              # sun vanishing within this time after a move = wrong way
        self.single_limit_mode = single_limit_mode      # True = only one limit switch, 360° wrap
        self.full_rotation_steps = full_rotation_steps  # steps for a full 360° rotation (set from y_range_steps)
        self.last_event = ""
        # pending full-rotation command (set by notify_move_result, consumed by tracker_runtime)
        self.pending_rotation_steps = 0   # non-zero = tracker_runtime should issue this many steps on Y
        self._reset_attempt()

    def _reset_attempt(self):
        self.dir_flip = 1            # -1 = we have reversed the calibrated direction
        self.reversals = 0
        self.search_dir = 1
        self.search_reversals = 0
        self._grow = 0
        self._err_before = None
        self._last_chase_dir = 0     # sign of the last move made while chasing the sun
        self._last_chase_t = 0.0

    def reset(self):
        super().reset()
        self.last_event = ""
        self.pending_rotation_steps = 0
        self._reset_attempt()

    def _flip(self, why):
        self.dir_flip *= -1
        self.reversals += 1
        self._grow = 0
        self.last_event = f"reversed ({why}) {self.reversals}/{self.max_reversals}"
        if self.reversals > self.max_reversals:
            self.state = STUCK
            self.last_event = f"STUCK after {self.max_reversals} reversals ({why})"

    def update(self, error_px, detected, on_target=None, now=None):
        now = time.time() if now is None else now

        # ---------- full-rotation in progress: wait for it to complete ----------
        if self.state == ROTATING:
            # tracker_runtime is executing the big rotation; don't touch the motor.
            # Once pending_rotation_steps is consumed (set to 0) the runtime calls
            # reset() or we transition back to NO_SUN/SETTLING automatically below.
            return 0

        # ---------- no sun: bounded search sweep ----------
        if not detected or error_px is None:
            self._first_seen = None
            self._err_before = None
            self._grow = 0
            if self._last_chase_dir != 0:
                # the sun vanished right after we moved toward it: we most likely went the wrong way
                back_dir = -self._last_chase_dir
                lost_soon = (now - self._last_chase_t) <= self.lost_window_s
                self._last_chase_dir = 0
                if lost_soon and self.state not in (STUCK, ROTATING):
                    self._flip("sun lost after move")
                    if self.state != STUCK:
                        self.search_dir = back_dir        # go back to where it was last seen
            if self.state in (NOT_FOUND, STUCK):
                return 0
            self.state = SEARCHING
            if now - self._last_move < self.search_interval_s:
                return 0
            self._last_move = now
            return self.search_dir * self.search_step

        # ---------- sun visible ----------
        if self._first_seen is None:
            self._first_seen = now

        if abs(error_px) <= self.deadband_px:
            if on_target is not False:
                self.state = LOCKED
                self._reset_attempt()
                return 0
            else:
                # Within deadband but sun is not on the line segment yet:
                # don't declare LOCKED, just settle and wait (don't move the motor).
                self.state = SETTLING
                return 0

        if self.state == STUCK:
            # Auto-clear STUCK when the sun is clearly far away from the target.
            # A large error means the sun moved significantly (e.g. gimbal rotated, sun angle changed)
            # rather than the motor hunting near the target — safe to retry tracking.
            far_threshold = max(self.deadband_px * 5, 30.0)
            if abs(error_px) > far_threshold:
                self._reset_attempt()
                self.state = SETTLING
                self.last_event = f"STUCK cleared — error {abs(error_px):.0f}px far from target, retrying"
            else:
                return 0
        if self.state in (SEARCHING, NOT_FOUND):
            self.state = SETTLING
            self.search_reversals = 0

        if now - self._first_seen < self.confirm_s or now - self._last_move < self.settle_s:
            self.state = SETTLING
            return 0

        # did the last move make things worse?
        if self._err_before is not None:
            if abs(error_px) > self._err_before + self.grow_px:
                self._grow += 1
            else:
                self._grow = 0
            if self._grow >= self.grow_count:
                self._flip("error growing")
                if self.state == STUCK:
                    return 0
        self._err_before = abs(error_px)

        steps = int(round(self.sign * self.dir_flip * self.gain * error_px))
        if steps == 0:
            steps = self.min_step if (self.sign * self.dir_flip * error_px) > 0 else -self.min_step
        steps = max(-self.max_step, min(self.max_step, steps))

        self._last_move = now
        self._first_seen = None
        self._last_chase_dir = 1 if steps > 0 else -1
        self._last_chase_t = now
        self.state = MOVING
        return steps

    def notify_move_result(self, requested, moved, hit_limit=False):
        """Tell the controller what really happened.
        A blocked move (limit switch or soft limit) reverses direction.
        In single_limit_mode: when limit is hit while chasing the sun,
        schedule a full 360° rotation the other way instead of reversing.
        """
        if requested == 0:
            return
        if moved == 0 or hit_limit:
            if self.state == SEARCHING:
                self.search_dir *= -1
                self.search_reversals += 1
                self.last_event = f"search reversed {self.search_reversals}/{self.max_search_reversals}"
                if self.search_reversals > self.max_search_reversals:
                    self.state = NOT_FOUND
                    self.last_event = "sun not found - search stopped"
            elif self.state != STUCK:
                if self.single_limit_mode and hit_limit:
                    # Single limit switch, 360° wrap: the switch is at the HOME position (pos_y≈0).
                    # The motor only hits it when going BACKWARD (negative steps, approaching 0).
                    # Recovery = rotate FORWARD (+) a full revolution to come around from the other side.
                    # We always go forward regardless of which direction caused the hit.
                    self.pending_rotation_steps = +self.full_rotation_steps
                    self.state = ROTATING
                    self.last_event = (
                        f"limit hit \u2013 rotating forward {self.full_rotation_steps} steps "
                        f"(full 360\u00b0 wrap) to reach sun from other side"
                    )
                    # Reset tracking state so we re-acquire the sun after the rotation
                    self._reset_attempt()
                    self._first_seen = None
                else:
                    self._flip("limit hit")


SCAN_IDLE    = "SCAN_IDLE"
SCAN_SWEEP   = "SCAN_SWEEP"
SCAN_RETURN  = "SCAN_RETURN"


class XScanController:
    """
    Wraps AxisController and adds a full-range sweep scan for the X axis when
    the sun is not detected for longer than 'no_sun_timeout_s'.

    Scan sequence
    -------------
    1. Sun vanishes for no_sun_timeout_s → start sweep from scan_start_deg.
    2. Move to each position (scan_step_deg apart), wait dwell_frames frames
       at each and record the peak detection confidence seen there.
    3. After the full sweep: move to the position with the highest confidence
       (scan_best_deg) and resume normal AxisController tracking.
    4. If nothing was detected at all, return to scan_start_deg and wait.
    """

    def __init__(self, axis_ctrl,
                 scan_start_deg=0.0,
                 scan_end_deg=60.0,
                 scan_step_deg=2.0,
                 dwell_frames=4,
                 no_sun_timeout_s=3.0,
                 move_settle_s=0.4):
        self.ctrl             = axis_ctrl
        self.scan_start_deg   = scan_start_deg
        self.scan_end_deg     = scan_end_deg
        self.scan_step_deg    = scan_step_deg
        self.dwell_frames     = dwell_frames
        self.no_sun_timeout_s = no_sun_timeout_s
        self.move_settle_s    = move_settle_s

        # public (dashboard-readable)
        self.scan_state     = SCAN_IDLE
        self.scan_best_deg  = None
        self.scan_best_conf = 0.0

        self._lost_t        = None
        self._scan_positions = []   # [[deg, best_conf], ...]
        self._scan_idx      = 0
        self._dwell_count   = 0
        self._at_scan_pos   = False
        self._move_done_t   = None

    # ------------------------------------------------------------------
    def reset(self):
        self.ctrl.reset()
        self.scan_state   = SCAN_IDLE
        self._lost_t      = None
        self._scan_positions = []
        self._scan_idx    = 0
        self._dwell_count = 0
        self._at_scan_pos = False
        self._move_done_t = None

    @property
    def state(self):
        """Unified state string shown on the dashboard."""
        if self.scan_state == SCAN_IDLE:
            return self.ctrl.state
        if self.scan_state == SCAN_SWEEP:
            deg = self._scan_positions[self._scan_idx][0] if self._scan_positions else "?"
            return f"SCANNING {deg:.0f}deg"
        if self.scan_state == SCAN_RETURN:
            best = f"{self.scan_best_deg:.0f}" if self.scan_best_deg is not None else "?"
            return f"SCAN_RETURN to {best}deg"
        return self.ctrl.state

    # ------------------------------------------------------------------
    def _build_scan_list(self):
        positions = []
        deg = self.scan_start_deg
        while deg <= self.scan_end_deg + 1e-6:
            positions.append([deg, 0.0])
            deg = round(deg + self.scan_step_deg, 6)
        return positions

    # ------------------------------------------------------------------
    def update(self, error_px, detected, confidence, current_deg, now=None):
        """
        Call every frame. Returns (action, value):

            ("ctrl", steps)  → link.move_steps(steps, 0)   normal tracking
            ("goto", deg)    → link.goto_x_deg(deg)         scan repositioning
            ("idle", 0)      → do nothing this frame

        Parameters
        ----------
        error_px    : int/float pixel error (sun minus target_x); None if not detected
        detected    : bool
        confidence  : float 0-1 from SunDetector.detect()["confidence"]
        current_deg : float  link.x_deg  (current motor angle)
        now         : float timestamp (uses time.time() if None)
        """
        now = time.time() if now is None else now

        # ---------------------------------------------------------------
        # SCAN_IDLE – normal tracking
        # ---------------------------------------------------------------
        if self.scan_state == SCAN_IDLE:
            if detected:
                self._lost_t = None
                steps = self.ctrl.update(error_px, True, now)
                return ("ctrl", steps)
            # Sun not detected
            if self._lost_t is None:
                self._lost_t = now
            self.ctrl.update(None, False, now)          # keep ctrl state = NO_SUN
            if now - self._lost_t >= self.no_sun_timeout_s:
                self._start_sweep(now)
            return ("idle", 0)

        # ---------------------------------------------------------------
        # SCAN_SWEEP – sweeping across the range
        # ---------------------------------------------------------------
        if self.scan_state == SCAN_SWEEP:
            target_deg = self._scan_positions[self._scan_idx][0]

            # Issue the goto for this position (once)
            if not self._at_scan_pos:
                self._at_scan_pos = True
                self._move_done_t = now
                self._dwell_count = 0
                return ("goto", target_deg)

            # Wait for motor to settle
            if now - self._move_done_t < self.move_settle_s:
                return ("idle", 0)

            # Record confidence during dwell
            if detected:
                conf = confidence or 0.0
                self._scan_positions[self._scan_idx][1] = max(
                    self._scan_positions[self._scan_idx][1], conf)

            self._dwell_count += 1
            if self._dwell_count < self.dwell_frames:
                return ("idle", 0)

            # Done dwelling – advance
            self._scan_idx += 1
            if self._scan_idx >= len(self._scan_positions):
                # Sweep complete
                self._finish_sweep(now)
                dest = self.scan_best_deg if self.scan_best_deg is not None else self.scan_start_deg
                return ("goto", dest)
            else:
                self._at_scan_pos = False
                return ("idle", 0)

        # ---------------------------------------------------------------
        # SCAN_RETURN – waiting to arrive at the best position
        # ---------------------------------------------------------------
        if self.scan_state == SCAN_RETURN:
            if self._move_done_t is None:
                self._move_done_t = now
            if now - self._move_done_t < self.move_settle_s * 4:
                return ("idle", 0)
            # Back to normal tracking
            self.scan_state = SCAN_IDLE
            self._lost_t    = None
            self.ctrl.reset()
            if detected:
                steps = self.ctrl.update(error_px, True, now)
                return ("ctrl", steps)
            return ("idle", 0)

        return ("idle", 0)

    # ------------------------------------------------------------------
    def _start_sweep(self, now):
        self._scan_positions = self._build_scan_list()
        self._scan_idx    = 0
        self._dwell_count = 0
        self._at_scan_pos = False
        self._move_done_t = now
        self.scan_state   = SCAN_SWEEP
        self.scan_best_deg  = None
        self.scan_best_conf = 0.0

    def _finish_sweep(self, now):
        best = max(self._scan_positions, key=lambda p: p[1])
        if best[1] > 0.0:
            self.scan_best_deg  = best[0]
            self.scan_best_conf = best[1]
        else:
            self.scan_best_deg  = None
            self.scan_best_conf = 0.0
        self.scan_state   = SCAN_RETURN
        self._move_done_t = now


if __name__ == "__main__":
    # tiny simulation: the sun moves +0.5 px per +step; the controller should settle inside the dead band
    px_per_step = 0.5
    sun_x, target = 400.0, 344.0
    ctrl = AxisController(gain=1 / abs(px_per_step) * 0.8, sign=-1, settle_s=0.2, confirm_s=0.05)
    t = 0.0
    for _ in range(400):
        steps = ctrl.update(sun_x - target, True, now=t)
        sun_x += steps * px_per_step
        t += 0.02
    print(f"final error {sun_x - target:+.1f} px, state {ctrl.state}")