"""
stepper_link.py  -  Python <-> Arduino link for the two steppers (no cameras needed)
===================================================================================
Talks to esp32_sun_tracker.ino over serial and adds the safety limits:

    X : switch = -60 deg, soft limit +60 deg   (60 deg = 1067 steps from the switch, 0 deg = 533 steps from the switch)
    Y : home switch = 0 deg, soft limit 360 deg, plus a switch at the far end of its travel (can be the same switch)
        (the Arduino measures the travel after homing and the soft limit never goes beyond it)

The Arduino only knows "steps from the limit switch". This file converts
degrees <-> steps and refuses to send moves beyond the soft limits.

Use it from other files:
    from stepper_link import StepperLink
    with StepperLink() as link:          # opens the port and waits for homing to finish
        link.goto_x_deg(0)               # move X to 0 deg
        link.move_steps(-12, 0)          # relative correction in steps
        print(link.x_deg, link.y_deg)

Test it by itself (needs only the Arduino, run:  python stepper_link.py  [port]):
    home | pos | lim | x <deg> | y <deg> | m <dx> <dy> | test | quit
"""

import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial is missing. Install it with:  pip install pyserial")

# ----------------------------------------------------------------------
# CONFIG  (edit these)
# ----------------------------------------------------------------------
SERIAL_PORT = "/dev/ttyUSB0"      # e.g. "/dev/ttyUSB0", "/dev/ttyACM0" or "COM3"
BAUD_RATE = 115200

# New driver setup: 800 steps = 90 deg  ->  90/800 = 0.1125 deg per step
# This is assumed to be the same for X and Y. Change if Y has different gearing.
DEGREES_PER_STEP = 90.0 / 800

X_SWITCH_DEG = -60.0              # logical angle of X at its limit switch
X_MAX_DEG = 60.0                  # X soft limit
Y_SWITCH_DEG = 0.0                # logical angle of Y at its limit switch
Y_MAX_DEG = 360.0                 # Y soft limit (lowered automatically if the measured travel is shorter)
SWITCH_MARGIN_STEPS = 10          # stay this many steps away from the switch end (~1.1 deg)

BOOT_RESET_WAIT_S = 2.0           # opening the port resets the Uno; wait for the bootloader to start printing
BOOT_QUIET_S = 6.0                # Phase-1 window to catch the BOOT line (must be > BOOT_RESET_WAIT_S + margin)
HOME_TIMEOUT_S = 120.0            # maximum time to wait for homing to finish (slower homing takes longer)
# ----------------------------------------------------------------------

X_MIN_STEPS = SWITCH_MARGIN_STEPS
X_MAX_STEPS = int(round((X_MAX_DEG - X_SWITCH_DEG) / DEGREES_PER_STEP))   # 1067  (= (60 - -60) / 0.1125)
Y_MIN_STEPS = SWITCH_MARGIN_STEPS
Y_MAX_STEPS = int(round((Y_MAX_DEG - Y_SWITCH_DEG) / DEGREES_PER_STEP))   # 3200


class StepperError(Exception):
    pass


class StepperReset(StepperError):
    """The Arduino restarted by itself (power dip, reset, watchdog). It homes again on its own."""


class StepperLink:
    def __init__(self, port=SERIAL_PORT, baud=BAUD_RATE, verbose=True):
        self.port = port
        self.baud = baud
        self.verbose = verbose
        self.ser = None
        self.pos_x = 0            # steps from the limit switch
        self.pos_y = 0
        self.homed = False
        self.limit_hits = []      # "LIMIT X" / "LIMIT Y" / "LIMIT YMAX" lines seen during the last move
        self.y_max_steps = Y_MAX_STEPS   # soft limit actually used (lowered if the measured travel is shorter)
        self.y_range_steps = None        # measured travel between the two Y switches
        self.reset_count = 0             # how many times an Arduino restart was detected
        self._ready_after_reset = False  # poll() already saw READY after a restart

    # ------------------------------------------------------------------
    # connection
    # ------------------------------------------------------------------
    def connect(self):
        """Open the port and wait for the Arduino to home itself, then accept commands.

        Two-phase approach:
          Phase 1 (BOOT_QUIET_S deadline): look for the BOOT line.
            - BOOT found  → move to Phase 2 and wait for homing to finish.
            - READY found → Arduino was already running; re-home for a known state.
            - No BOOT within the deadline (even if other text arrives) → send H ourselves.
          Phase 2 (HOME_TIMEOUT_S deadline): wait for READY after BOOT.

        This guarantees connect() never hangs silently even if the Arduino is
        streaming GIMBAL status lines or was already running when we opened the port.
        """
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        except serial.SerialException as e:
            raise StepperError(f"Cannot open {self.port}: {e}") from e

        # t0 is captured BEFORE the sleep so that the Phase-1 deadline
        # (BOOT_QUIET_S) starts from the moment we opened the port, not from
        # after the sleep.  The Arduino prints BOOT immediately on power-up /
        # reset; it may arrive while we are still sleeping, and we pick it up
        # from the receive buffer once the sleep ends.  BOOT_QUIET_S must be
        # larger than BOOT_RESET_WAIT_S so that Phase 1 still has time left
        # after the sleep.
        t0 = time.time()
        time.sleep(BOOT_RESET_WAIT_S)

        # ------------------------------------------------------------------
        # Phase 1: look for BOOT within a hard wall-clock deadline.
        # We exit this phase as soon as we see BOOT, READY, or the deadline.
        # ------------------------------------------------------------------
        saw_boot = False
        phase1_deadline = t0 + BOOT_QUIET_S
        while time.time() < phase1_deadline:
            line = self._readline()
            if not line:
                continue
            self._log(f"<- {line}")
            self._note_line(line)

            if line == "BOOT":
                saw_boot = True
                self._log("BOOT seen — Arduino is resetting and will home itself.")
                # Flush any stale bytes (GIMBAL lines printed before BOOT) so
                # Phase 2 only reads what arrives AFTER the reset.
                self.ser.reset_input_buffer()
                break                               # move to Phase 2

            if line == "READY":
                # Arduino was already running (homed). Re-home for a known position.
                self._log("READY seen (no BOOT) — Arduino was already running. Forcing a fresh home.")
                self.ser.reset_input_buffer()
                self.home()
                return self

            if line.startswith("READY"):            # "READY (not homed)"
                self._log("Arduino did not home on boot; sending H")
                self.ser.reset_input_buffer()
                self.home()
                return self

            if line == "HOMING_FAILED" or line.startswith("ERR"):
                raise StepperError(f"Arduino reported: {line}")

        if not saw_boot:
            # Phase 1 expired without a BOOT line.
            # The Arduino did not reset (or BOOT was missed before we started reading).
            # Send H to put it in a known state.
            self._log(f"No BOOT seen within {BOOT_QUIET_S:.1f}s — sending H to home.")
            self.ser.reset_input_buffer()
            self.home()
            return self

        # ------------------------------------------------------------------
        # Phase 2: BOOT was seen — wait for homing to complete (READY).
        # ------------------------------------------------------------------
        phase2_deadline = t0 + HOME_TIMEOUT_S
        last_progress = time.time()
        progress_interval = 10.0     # emit a "still homing" log every 10 s
        while time.time() < phase2_deadline:
            line = self._readline()
            if not line:
                now2 = time.time()
                if now2 - last_progress >= progress_interval:
                    elapsed = int(now2 - t0)
                    remaining = int(phase2_deadline - now2)
                    self._log(f"homing in progress … {elapsed}s elapsed, "
                              f"up to {remaining}s remaining")
                    last_progress = now2
                continue
            self._log(f"<- {line}")
            self._note_line(line)

            if line == "READY":
                self.ser.reset_input_buffer()
                self.pos_x = 0
                self.pos_y = 0
                self.homed = True
                self._log("Arduino homed itself on boot. Ready.")
                return self

            if line.startswith("READY"):            # "READY (not homed)"
                self.ser.reset_input_buffer()
                self.home()
                return self

            if line == "BOOT":
                # Unexpected second reset while waiting: extend the deadline.
                self._log("!! Second BOOT during connect — Arduino reset again. Extending wait.")
                phase2_deadline = time.time() + HOME_TIMEOUT_S
                continue

            if line == "HOMING_FAILED" or line.startswith("ERR"):
                raise StepperError(f"Arduino reported: {line}")

        raise StepperError("Timed out waiting for READY from the Arduino.")

    def close(self):
        if self.ser:
            self.ser.close()
            self.ser = None

    def __enter__(self):
        return self.connect()

    def __exit__(self, *exc):
        self.close()

    # ------------------------------------------------------------------
    # low level
    # ------------------------------------------------------------------
    def _log(self, msg):
        if self.verbose:
            print(f"[stepper] {msg}")

    def _readline(self):
        raw = self.ser.readline()
        return raw.decode("ascii", errors="ignore").strip() if raw else ""

    def _send(self, text):
        self._log(f"-> {text}")
        self.ser.write((text + "\n").encode("ascii"))

    def _wait_for(self, prefixes, timeout):
        """Read lines until one starts with a wanted prefix. LIMIT lines are recorded; ERR lines raise."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            line = self._readline()
            if not line:
                continue
            self._log(f"<- {line}")
            self._note_line(line)
            if line == "BOOT":
                self._on_reset()
                raise StepperReset("The Arduino restarted (power dip or reset) and is homing again.")
            if line.startswith("LIMIT"):
                self.limit_hits.append(line)
                continue
            if line.startswith("ERR") or line == "HOMING_FAILED":
                raise StepperError(f"Arduino reported: {line}")
            if any(line.startswith(p) for p in prefixes):
                return line
        raise StepperError(f"Timed out waiting for {prefixes} from the Arduino.")

    def _note_line(self, line):
        """Pick up information lines the Arduino prints on its own."""
        if line.startswith("YRANGE"):
            try:
                self.y_range_steps = int(line.split()[1])
            except (IndexError, ValueError):
                return
            self.y_max_steps = min(Y_MAX_STEPS, self.y_range_steps - SWITCH_MARGIN_STEPS)
            self._log(f"Y travel between the two switches: {self.y_range_steps} steps "
                      f"({self.y_range_steps * DEGREES_PER_STEP:.1f} deg); Y soft limit = {self.y_max_steps} steps")
        elif line.startswith("WARN"):
            self._log(line)

    @staticmethod
    def _two_ints(line):
        parts = line.split()
        return int(parts[1]), int(parts[2])

    def _require_homed(self):
        if not self.homed:
            raise StepperError("Not homed yet. Call home() first.")

    # ------------------------------------------------------------------
    # commands
    # ------------------------------------------------------------------
    def home(self):
        """Home both motors at the limit switches (takes a few seconds)."""
        if self.ser:
            try:
                self.ser.reset_input_buffer()
            except AttributeError:
                pass
        self._send("H")
        self.wait_ready()
        self._log("Homed.")

    def wait_ready(self, timeout=HOME_TIMEOUT_S):
        """Wait until the Arduino prints READY (homing finished). BOOT lines are fine here:
        it means the Arduino restarted and is homing by itself."""
        if self._ready_after_reset:          # poll() already read the READY line
            self._ready_after_reset = False
            self.pos_x = 0
            self.pos_y = 0
            self.homed = True
            return
        deadline = time.time() + timeout
        while time.time() < deadline:
            line = self._readline()
            if not line:
                continue
            self._log(f"<- {line}")
            self._note_line(line)
            if line.startswith("ERR") or line == "HOMING_FAILED":
                raise StepperError(f"Arduino reported: {line}")
            if line == "READY":
                self.pos_x = 0
                self.pos_y = 0
                self.homed = True
                return
        raise StepperError("Timed out waiting for READY from the Arduino.")

    def _on_reset(self):
        self.homed = False
        self._ready_after_reset = False
        self.pos_x = 0
        self.pos_y = 0
        self.reset_count += 1
        self._log(f"!! Arduino restart detected (#{self.reset_count}). Positions are unknown until it has homed again.")

    def poll(self):
        """Call regularly while idle. Reads any unsolicited lines from the Arduino.
        Returns True if it has restarted (then call wait_ready() to let it finish homing)."""
        if not self.ser:
            return False
        restarted = False
        while self.ser.in_waiting:
            line = self._readline()
            if not line:
                break
            self._log(f"<- {line}")
            self._note_line(line)
            if line == "BOOT":
                self._on_reset()             # also forgets any READY seen before this restart
                restarted = True
            elif line == "READY" and restarted:
                self._ready_after_reset = True
        return restarted

    def sync_position(self):
        """Ask the Arduino for its step counters."""
        self._send("P")
        line = self._wait_for(("POS",), 3.0)
        self.pos_x, self.pos_y = self._two_ints(line)
        return self.pos_x, self.pos_y

    def limit_switches(self):
        """(x_home, y_home, y_far) pressed states - handy to test the wiring."""
        self._send("L")
        line = self._wait_for(("LIM",), 3.0)
        parts = [int(p) for p in line.split()[1:4]]
        return tuple(bool(p) for p in parts)

    def move_steps(self, dx, dy):
        """
        Relative move in steps (+ = away from the limit switch), limited by the soft limits.
        Waits until the Arduino has finished. Returns (moved_x, moved_y) actually done.
        """
        self._require_homed()
        # Never move an axis the caller did not request to move.
        # Previous logic clamped both axes even for dx=0/dy=0, producing
        # unexpected 10-step Y movements immediately after homing.
        dx, dy = int(dx), int(dy)
        tx = self.pos_x if dx == 0 else min(max(self.pos_x + dx, X_MIN_STEPS), X_MAX_STEPS)
        ty = self.pos_y if dy == 0 else min(max(self.pos_y + dy, Y_MIN_STEPS), self.y_max_steps)
        mdx, mdy = tx - self.pos_x, ty - self.pos_y
        if (mdx, mdy) != (int(dx), int(dy)):
            self._log(f"soft limit: asked ({int(dx)},{int(dy)}) -> sending ({mdx},{mdy})")
        if mdx == 0 and mdy == 0:
            return 0, 0

        self.limit_hits = []
        self._send(f"{mdx} {mdy}")
        line = self._wait_for(("OK",), 5.0 + max(abs(mdx), abs(mdy)) * 0.006)
        nx, ny = self._two_ints(line)
        moved = (nx - self.pos_x, ny - self.pos_y)
        self.pos_x, self.pos_y = nx, ny
        return moved

    # ------------------------------------------------------------------
    # angles
    # ------------------------------------------------------------------
    @property
    def x_deg(self):
        return X_SWITCH_DEG + self.pos_x * DEGREES_PER_STEP

    @property
    def y_deg(self):
        return Y_SWITCH_DEG + self.pos_y * DEGREES_PER_STEP

    def goto_x_deg(self, deg):
        """Move X to an absolute angle (clamped to the soft limits)."""
        target = int(round((deg - X_SWITCH_DEG) / DEGREES_PER_STEP))
        return self.move_steps(target - self.pos_x, 0)

    def goto_y_deg(self, deg):
        """Move Y to an absolute angle (clamped to the soft limits)."""
        target = int(round((deg - Y_SWITCH_DEG) / DEGREES_PER_STEP))
        return self.move_steps(0, target - self.pos_y)

    def status_text(self):
        return (f"X: {self.pos_x} steps = {self.x_deg:+.2f} deg   "
                f"Y: {self.pos_y} steps = {self.y_deg:+.2f} deg")


# ----------------------------------------------------------------------
# Camera-free test shell:  python stepper_link.py [port]
# ----------------------------------------------------------------------
def selftest(link):
    print("Self-test: X to 0 deg, +10 deg, back; Y +20 deg, back. Watch the motors.")
    steps = [("X to 0 deg", lambda: link.goto_x_deg(0)),
             ("X to +10 deg", lambda: link.goto_x_deg(10)),
             ("X back to 0 deg", lambda: link.goto_x_deg(0)),
             ("Y to +20 deg", lambda: link.goto_y_deg(20)),
             ("Y back to 0 deg (just above the switch)", lambda: link.goto_y_deg(0)),
             ("X and Y together: +100 / +100 steps", lambda: link.move_steps(100, 100)),
             ("X and Y together: -100 / -100 steps", lambda: link.move_steps(-100, -100))]
    for name, fn in steps:
        print(f"\n>> {name}")
        fn()
        print("   " + link.status_text())
        time.sleep(0.5)
    print("\nSelf-test finished.")


HELP = """commands:
  home            home both motors again
  pos             show positions (steps and degrees)
  lim             show limit switch states
  x <deg>         move X to an absolute angle   (e.g. x 0, x 30)
  y <deg>         move Y to an absolute angle   (e.g. y 45)
  m <dx> <dy>     relative move in steps        (e.g. m 100 0, m 0 -50)
  test            run a short automatic test
  quit"""


def shell(link):
    print(HELP)
    print(link.status_text())
    while True:
        try:
            parts = input("> ").split()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not parts:
            continue
        cmd, args = parts[0].lower(), parts[1:]
        try:
            if cmd in ("q", "quit", "exit"):
                break
            elif cmd == "home":
                link.home()
            elif cmd == "pos":
                link.sync_position()
            elif cmd == "lim":
                print("limit switches pressed (X home, Y home, Y far):", link.limit_switches())
                continue
            elif cmd == "x" and len(args) == 1:
                link.goto_x_deg(float(args[0]))
            elif cmd == "y" and len(args) == 1:
                link.goto_y_deg(float(args[0]))
            elif cmd == "m" and len(args) == 2:
                link.move_steps(int(args[0]), int(args[1]))
            elif cmd == "test":
                selftest(link)
            else:
                print(HELP)
                continue
            if link.limit_hits:
                print("   limit switch hit during the move:", link.limit_hits)
            print(link.status_text())
        except (StepperError, ValueError) as e:
            print("ERROR:", e)


if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else SERIAL_PORT
    print(f"Connecting to {port} (the Arduino resets and homes - this takes a few seconds)...")
    try:
        with StepperLink(port=port) as link:
            shell(link)
    except StepperError as e:
        sys.exit(f"ERROR: {e}")