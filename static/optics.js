/*
 * optics.js - ray geometry of the double-reflection sextant (pure maths, no DOM; also loadable in node for tests)
 *
 *   sun -> index mirror B (turned by theta) -> horizon mirror A (fixed) -> telescope C
 *
 * Screen coordinates (SVG): x right, y DOWN, angles in degrees measured clockwise on screen.
 * The telescope looks along the horizon line y = A.y; the horizon beam travels along it from the left.
 * Mirror A is a fixed line at MIRROR_A_DEG (svg angle 60 deg).  Mirror B is the same line turned by theta:
 *     B line angle = MIRROR_A_DEG + theta
 * A ray with direction d reflected by a mirror line with direction u:   d' = 2 (d.u) u - d
 * The sun ray travels right/down with direction (cos(alt), sin(alt)) in svg coordinates.
 * With theta = alt / 2 the ray leaves A exactly along the horizon beam (images coincide).
 * With any other theta the exit ray is tilted by (2*theta - alt): that tilt is what the split view shows.
 */
(function (root) {
  "use strict";

  const RAD = Math.PI / 180;

  const GEOM = {
    A: { x: 250, y: 390 },            // horizon mirror pivot (on the telescope axis)
    BA_DISTANCE: 215,                 // distance B -> A along the coincident ray
    MIRROR_A_DEG: 60,                 // svg angle of mirror A's line (same as B at theta = 0)
    // theta = 0 : B line = 60 deg (parallel to A); theta = 60 : B line = 120 deg.
    // 60 gives the Z shape: B up-right of A, ray B->A runs down-left, exit runs right.
    // B sits at svg angle 2*MIRROR_A_DEG from A, so the B->A ray always hits A
    // and the exit ray is horizontal whenever theta = altitude / 2.
    AXIS_END_X: 700,                  // where the rays stop (telescope end)
    SUN_DISTANCE: 190,                // drawing distance sun -> B
    MIRROR_HALF: 34,                  // half length of the drawn mirrors
    A_HALF: 30,                       // half length of the drawn mirror A (rays > 2x this far from A count as missed)
    LIMB_RADIUS: 290,
    THETA_DRAW_MIN: -4,               // drawing limits for theta (outside the 0..60 scan range)
    THETA_DRAW_MAX: 64,
    ALT_DRAW_MAX: 120,                // reading at X = 0 is 120 deg; the drawing may show a 'ghost' ray up to that (a real sun is only 0..90)
  };
  // B sits on the line through A along the coincident B->A direction (svg angle = 2*MIRROR_A_DEG)
  GEOM.B = {
    x: GEOM.A.x - GEOM.BA_DISTANCE * Math.cos(2 * GEOM.MIRROR_A_DEG * RAD),
    y: GEOM.A.y - GEOM.BA_DISTANCE * Math.sin(2 * GEOM.MIRROR_A_DEG * RAD),
  };

  const unit = (deg) => ({ x: Math.cos(deg * RAD), y: Math.sin(deg * RAD) });
  const dot = (a, b) => a.x * b.x + a.y * b.y;
  const cross = (a, b) => a.x * b.y - a.y * b.x;
  const sub = (a, b) => ({ x: a.x - b.x, y: a.y - b.y });
  const add = (a, s, d) => ({ x: a.x + s * d.x, y: a.y + s * d.y });
  const reflect = (d, u) => ({ x: 2 * dot(d, u) * u.x - d.x, y: 2 * dot(d, u) * u.y - d.y });
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

  /**
   * trace(thetaDeg, altitudeDeg) -> everything needed to draw the optics
   *   sun           position of the sun symbol
   *   B, A          pivots
   *   hit           where the ray from B meets mirror A's line (null if it never does)
   *   missed        the ray misses the silvered part of A (not visible in the telescope)
   *   exitEnd       end point of the ray after A (at the telescope end)
   *   exitTiltDeg   tilt of the exit ray, + = rising (image appears BELOW the horizon line); equals 2*theta - altitude
   *   mirrorB, mirrorA   end points of the drawn mirror lines
   */
  function trace(thetaDeg, altitudeDeg) {
    const th = clamp(thetaDeg, GEOM.THETA_DRAW_MIN, GEOM.THETA_DRAW_MAX);
    const alt = clamp(altitudeDeg, 0, GEOM.ALT_DRAW_MAX);
    const { A, B } = GEOM;
    const uA = unit(GEOM.MIRROR_A_DEG);
    const uB = unit(GEOM.MIRROR_A_DEG + th);
    const dIn = unit(alt);
    const sun = { x: B.x - GEOM.SUN_DISTANCE * dIn.x, y: B.y - GEOM.SUN_DISTANCE * dIn.y };

    const d1 = reflect(dIn, uB);                         // after mirror B
    const denom = cross(d1, uA);
    let hit = null, t1 = null;
    if (Math.abs(denom) > 1e-9) {
      t1 = cross(sub(A, B), uA) / denom;
      if (t1 > 0) hit = add(B, t1, d1);
    }
    let missed = true, d2 = null, exitEnd = null, exitTilt = null;
    if (hit) {
      missed = Math.hypot(hit.x - A.x, hit.y - A.y) > 2 * GEOM.A_HALF;   // far off mirror A (schematic mirror is short)
      d2 = reflect(d1, uA);                              // after mirror A
      if (d2.x > 1e-6) {
        const t2 = (GEOM.AXIS_END_X - hit.x) / d2.x;
        exitEnd = add(hit, t2, d2);
      }
      exitTilt = -Math.atan2(d2.y, d2.x) / RAD;          // + = travelling upward on screen
    }
    return {
      theta: th, altitude: alt, sun, A, B, hit, missed, d1, d2, exitEnd, exitTiltDeg: exitTilt,
      mirrorB: [add(B, -GEOM.MIRROR_HALF, uB), add(B, GEOM.MIRROR_HALF, uB)],
      mirrorA: [add(A, -GEOM.A_HALF, uA), add(A, GEOM.A_HALF, uA)],
      uA, uB,
    };
  }

  /** Point on the limb for a given index-mirror angle theta (the limb is centred on B). */
  function limbPoint(thetaDeg, extra) {
    const a = unit(GEOM.MIRROR_A_DEG + thetaDeg);
    const r = GEOM.LIMB_RADIUS + (extra || 0);
    return { x: GEOM.B.x + r * a.x, y: GEOM.B.y + r * a.y };
  }

  const Optics = { GEOM, trace, limbPoint, unit, clamp };
  if (typeof module !== "undefined" && module.exports) module.exports = Optics;
  else root.Optics = Optics;
})(typeof window !== "undefined" ? window : globalThis);