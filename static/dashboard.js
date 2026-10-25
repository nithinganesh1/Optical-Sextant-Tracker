/*
 * dashboard.js - live dashboard: binds the server snapshot to the page, drives the sextant animation
 * Every number shown comes from /api/stream (the real tracker); nothing here is a sample value.
 */
(() => {
  "use strict";

  const G = Optics.GEOM;
  const NS = "http://www.w3.org/2000/svg";
  const $ = (id) => document.getElementById(id);
  const num = (v) => typeof v === "number" && Number.isFinite(v);
  const get = (o, path) => path.split(".").reduce((a, k) => (a == null ? undefined : a[k]), o);
  const set = (el, attrs) => { for (const k in attrs) el.setAttribute(k, attrs[k]); };
  const f = (n, d) => n.toFixed(d);
  const signed = (n, d) => (n > 0 ? "+" : n < 0 ? "−" : "") + Math.abs(n).toFixed(d);

  // ------------------------------------------------------------------ formatting
  const FMT = {
    int: (v) => f(Math.round(v), 0),
    f1: (v) => f(v, 1), f2: (v) => f(v, 2), f3: (v) => f(v, 3),
    deg1: (v) => f(v, 1) + "°",
    deg2: (v) => f(v, 2) + "°",
    sdeg2: (v) => signed(v, 2) + "°",
    px: (v) => signed(Math.round(v), 0) + " px",
    pt: (v) => (Array.isArray(v) ? `${v[0]}, ${v[1]}` : "—"),
    list: (v) => (Array.isArray(v) ? (Array.isArray(v[0]) ? v.map((p) => `(${p.join(", ")})`).join(" → ") : v.join(", ")) : "—"),
    pressed: (v) => (v ? "PRESSED" : "released"),
    yesno: (v) => (v ? "YES" : "NO"),
    onoff: (v) => (v ? "ON" : "OFF"),
    sun: (v) => (v ? "SUN" : "NO SUN"),
    online: (v) => (v ? "ONLINE" : "OFFLINE"),
    level: (v) => (v ? "LEVEL" : "TILTED"),
    imustatus: (v) => ({ OK: "OK", UNO_SILENT: "NO DATA FROM UNO", NO_REPLY: "ESP32 NOT ANSWERING", WAITING: "WAITING" }[v] || String(v)),
    sign: (v) => (v >= 0 ? "+1" : "−1"),
    pps: (v) => signed(v, 2) + " px/step",
    rate: (v) => signed(v, 0) + " st/s",
    age: (v) => f(v, 1) + " s",
    hz: (v) => f(v, 0) + " Hz",
    uptime: (v) => { const s = Math.floor(v); return `${Math.floor(s / 3600)}:${String(Math.floor(s / 60) % 60).padStart(2, "0")}:${String(s % 60).padStart(2, "0")}`; },
    text: (v) => String(v),
  };

  const STATE_TONE = {
    LOCKED: "good", READY: "good", MOVING: "info", SETTLING: "warn", SEARCHING: "warn", HOMING: "warn",
    CONNECTING: "warn", RESETTING: "warn", BOOTING: "warn", NO_SUN: "dim", DISABLED: "dim", NOT_FOUND: "bad", STUCK: "bad", OFFLINE: "bad", ERROR: "bad"
  };
  function toneOf(kind, v) {
    if (v == null) return "dim";
    if (kind === "state") return STATE_TONE[v] || "dim";
    if (kind === "bool-good") return v ? "good" : "dim";
    if (kind === "pressed") return v ? "warn" : "dim";
    if (kind === "imu") return v === "OK" ? "good" : v === "WAITING" ? "dim" : "bad";
    return "";
  }
  function setTone(el, tone) {
    el.classList.remove("tone-good", "tone-warn", "tone-bad", "tone-info", "tone-dim");
    if (tone) el.classList.add("tone-" + tone);
  }

  const bound = Array.from(document.querySelectorAll("[data-k]"));
  function bindText(s) {
    for (const el of bound) {
      const v = get(s, el.dataset.k);
      let txt = "—";
      if (v !== null && v !== undefined && v !== "") {
        const fn = FMT[el.dataset.f];
        txt = fn ? (num(v) || typeof v !== "number" ? fn(v) : "—") : String(v);
      }
      if (el.textContent !== txt) el.textContent = txt;
      if (el.dataset.tone) setTone(el, toneOf(el.dataset.tone, v));
    }
  }

  // ------------------------------------------------------------------ gauges
  const gauges = Array.from(document.querySelectorAll("[data-gauge]"));
  function bindGauges(s) {
    for (const g of gauges) {
      const v = get(s, g.dataset.gauge);
      const marker = g.querySelector(".marker");
      let max = parseFloat(g.dataset.max);
      if (g.dataset.maxK) { const m = get(s, g.dataset.maxK); if (num(m) && m > 0) max = m; }
      let min = parseFloat(g.dataset.min);
      if (g.dataset.minK) { const m = get(s, g.dataset.minK); if (num(m)) min = m; }
      const band = g.querySelector(".band");
      if (band && g.dataset.band) {
        const d = get(s, g.dataset.band);
        if (num(d)) { const span = max - min; band.style.left = (100 * (-d - min) / span) + "%"; band.style.width = (100 * 2 * d / span) + "%"; }
      }
      if (!num(v)) { g.classList.remove("live"); continue; }
      g.classList.add("live");
      marker.style.left = Math.max(0, Math.min(100, 100 * (v - min) / (max - min))) + "%";
    }
  }

  // ------------------------------------------------------------------ sextant drawing
  const E = (id) => $(id);
  const eyeX = 810, eyeY = G.A.y, EYE_R = 56, EYE_SUN_R = 18, EYE_PX_PER_DEG = 8;

  function buildStatic() {
    const limb = E("limb");
    const arcD = (r) => { const a = Optics.limbPoint(0, r), b = Optics.limbPoint(60, r); return `M ${a.x} ${a.y} A ${G.LIMB_RADIUS + r} ${G.LIMB_RADIUS + r} 0 0 1 ${b.x} ${b.y}`; };
    for (const [w, c] of [[22, "#2a2318"], [1.5, "#714e1a"]]) {
      const p = document.createElementNS(NS, "path");
      set(p, { d: arcD(0), fill: "none", stroke: c, "stroke-width": w, "stroke-linecap": "round" });
      limb.appendChild(p);
    }
    for (let r = 0; r <= 120; r += 5) {                  // reading r  <->  index mirror theta = r / 2
      const major = r % 10 === 0, th = r / 2;
      const a = Optics.limbPoint(th, -4), b = Optics.limbPoint(th, major ? 8 : 4);
      const l = document.createElementNS(NS, "line");
      set(l, { x1: a.x, y1: a.y, x2: b.x, y2: b.y, stroke: major ? "#a17028" : "#5a3d16", "stroke-width": major ? 1 : .75 });
      limb.appendChild(l);
      if (major) {
        const p = Optics.limbPoint(th, 20), t = document.createElementNS(NS, "text");
        set(t, { x: p.x, y: p.y + 3, "text-anchor": "middle", fill: "#a17028", "font-size": 9, "font-family": "monospace" });
        t.textContent = r;
        limb.appendChild(t);
      }
    }
    const cap = document.createElementNS(NS, "text");
    const cp = Optics.limbPoint(30, 70);
    set(cap, { x: cp.x, y: cp.y, "text-anchor": "middle", fill: "#92621c", "font-size": 9.5, "letter-spacing": 1, "font-family": "monospace" });
    cap.textContent = "SEXTANT LIMB (READING 0° – 120°)";
    limb.appendChild(cap);

    set(E("horizonLabel"), { y: G.A.y - 8 });
    set(E("horizonBeam"), { y1: G.A.y + 2.5, y2: G.A.y + 2.5, x2: G.AXIS_END_X });   // 2.5 px lower so it stays visible under the coincident red ray
    set(E("scope"), { x: G.AXIS_END_X, y: G.A.y - 14, width: 58, height: 28 });
    set(E("scopeLabel"), { x: G.AXIS_END_X - 6, y: G.A.y + 34 });
    set(E("eyepiece"), { transform: `translate(${eyeX},${eyeY})` });
    set(E("pivotB"), { cx: G.B.x, cy: G.B.y });
    set(E("pivotA"), { cx: G.A.x, cy: G.A.y });
    set(E("labelB"), { x: G.B.x + 20, y: G.B.y - 22 }); set(E("labelBsub"), { x: G.B.x + 20, y: G.B.y - 10 });
    set(E("labelA"), { x: G.A.x - 34, y: G.A.y + 28 }); set(E("labelAsub"), { x: G.A.x - 50, y: G.A.y + 42 });
    const u = Optics.unit(G.MIRROR_A_DEG);
    set(E("parallelRef"), { x1: G.B.x - 90 * u.x, y1: G.B.y - 90 * u.y, x2: G.B.x + 90 * u.x, y2: G.B.y + 90 * u.y });
    // horizon arrow, mid-way along the horizon beam
    set(E("arrHor"), { points: arrowPts({ x: 40, y: G.A.y }, { x: G.AXIS_END_X, y: G.A.y }, 0.62) });
  }

  function arrowPts(p0, p1, frac) {
    const x = p0.x + (p1.x - p0.x) * frac, y = p0.y + (p1.y - p0.y) * frac;
    const a = Math.atan2(p1.y - p0.y, p1.x - p0.x), L = 9, W = 5;
    const pt = (d, o) => `${x + d * Math.cos(a) - o * Math.sin(a)},${y + d * Math.sin(a) + o * Math.cos(a)}`;
    return `${pt(L, 0)} ${pt(-L / 2, W)} ${pt(-L / 2, -W)}`;
  }

  function drawOptics(s) {
    const sx = s.sextant, badge = E("coinBadge");
    const homed = !!sx;
    // what to draw: theta from the real motor X angle; sun altitude from the camera-corrected sextant value
    const theta = homed ? sx.theta_deg : 0;
    const reading = homed ? sx.reading_deg : 0;
    const sunKnown = homed && num(sx.sun_altitude_deg);
    const alt = sunKnown ? sx.sun_altitude_deg : Optics.clamp(reading, 0, G.ALT_DRAW_MAX);   // ghost sun at the reading (= coincident altitude) when nothing is seen
    const t = Optics.trace(theta, alt);
    const live = homed;
    for (const id of ["optics"]) E(id).style.opacity = live ? 1 : 0.45;

    // sun
    set(E("sunGlowC"), { cx: t.sun.x, cy: t.sun.y }); set(E("sunCore"), { cx: t.sun.x, cy: t.sun.y });
    set(E("sunLabel"), { x: t.sun.x, y: t.sun.y - 26 });
    E("sunLabel").textContent = sunKnown ? `SUN  α ${f(alt, 1)}°` : (homed ? (reading > 90 ? `NO SUN (reading ${f(reading, 0)}° > 90°)` : "NO SUN") : "—");
    E("sunGroup").classList.toggle("ghost", !sunKnown);

    // mirrors, index arm, limb pointer
    set(E("mirrorB"), { x1: t.mirrorB[0].x, y1: t.mirrorB[0].y, x2: t.mirrorB[1].x, y2: t.mirrorB[1].y });
    set(E("mirrorBhi"), { x1: t.mirrorB[0].x, y1: t.mirrorB[0].y, x2: t.mirrorB[1].x, y2: t.mirrorB[1].y });
    set(E("mirrorA"), { x1: t.mirrorA[0].x, y1: t.mirrorA[0].y, x2: G.A.x, y2: G.A.y });
    set(E("mirrorAclear"), { x1: G.A.x, y1: G.A.y, x2: t.mirrorA[1].x, y2: t.mirrorA[1].y });
    const armTheta = Optics.clamp(theta, 0, 60), ap = Optics.limbPoint(armTheta);
    for (const id of ["indexArmBar", "indexArmHi"]) set(E(id), { x1: G.B.x, y1: G.B.y, x2: ap.x, y2: ap.y });
    set(E("armDot"), { cx: ap.x, cy: ap.y });
    const lp = Optics.limbPoint(armTheta, 34);
    set(E("armLabel"), { x: lp.x, y: lp.y + 4 });
    E("armLabel").textContent = homed ? f(reading, 1) + "°" : "";

    // theta arc at B
    const u0 = Optics.unit(G.MIRROR_A_DEG), u1 = Optics.unit(G.MIRROR_A_DEG + t.theta), R = 62;
    if (Math.abs(t.theta) > 0.05) {
      set(E("thetaArc"), { d: `M ${G.B.x + R * u0.x} ${G.B.y + R * u0.y} A ${R} ${R} 0 0 ${t.theta > 0 ? 1 : 0} ${G.B.x + R * u1.x} ${G.B.y + R * u1.y}` });
    } else set(E("thetaArc"), { d: "" });
    const um = Optics.unit(G.MIRROR_A_DEG + t.theta / 2);
    set(E("thetaLabel"), { x: G.B.x + (R + 8) * um.x + 6, y: G.B.y + (R + 8) * um.y + 4 });
    E("thetaLabel").textContent = homed ? `θ ${f(theta, 1)}°` : "θ";

    // rays
    const hit = t.hit || { x: G.B.x + 120 * t.d1.x, y: G.B.y + 120 * t.d1.y };
    set(E("raySun"), { x1: t.sun.x, y1: t.sun.y, x2: G.B.x, y2: G.B.y });
    set(E("rayBA"), { x1: G.B.x, y1: G.B.y, x2: hit.x, y2: hit.y });
    set(E("arrSun"), { points: arrowPts(t.sun, G.B, 0.5) });
    set(E("arrBA"), { points: arrowPts(G.B, hit, 0.5) });
    if (t.exitEnd) {
      set(E("rayOut"), { x1: hit.x, y1: hit.y, x2: t.exitEnd.x, y2: t.exitEnd.y });
      set(E("arrOut"), { points: arrowPts(hit, t.exitEnd, 0.62) });
    } else { set(E("rayOut"), { x1: 0, y1: 0, x2: 0, y2: 0 }); set(E("arrOut"), { points: "" }); }
    for (const id of ["raySun", "rayBA", "rayOut", "arrSun", "arrBA", "arrOut"]) E(id).classList.toggle("dim", !sunKnown);

    // eyepiece: image of the sun displaced by the exit tilt (= 2*theta - altitude)
    const eyeSun = E("eyeSun"), note = E("eyeNote");
    let msg = "";
    if (!homed) { eyeSun.style.display = "none"; msg = "NO MOTOR DATA"; }
    else if (!sunKnown) { eyeSun.style.display = "none"; msg = sx.sun_altitude_deg == null && s.camera1.detected ? "SUN OUTSIDE 0–90°" : "NO SUN"; }
    else if (t.exitTiltDeg == null) { eyeSun.style.display = "none"; msg = "NO IMAGE"; }
    else {
      const off = EYE_PX_PER_DEG * t.exitTiltDeg;        // + = image below the horizon line
      if (Math.abs(off) > EYE_R + EYE_SUN_R) { eyeSun.style.display = "none"; msg = `OUT OF FIELD (${signed(t.exitTiltDeg, 1)}°)`; }
      else { eyeSun.style.display = ""; set(eyeSun, { cy: off }); msg = Math.abs(t.exitTiltDeg) < 0.05 ? "" : `offset ${signed(t.exitTiltDeg, 2)}°`; }
    }
    note.textContent = msg;

    // badge
    let txt = "NO MOTOR DATA", tone = "dim";
    if (homed) {
      if (!sx.in_scan_range) { txt = "OUTSIDE X SCAN RANGE 0–60°"; tone = "bad"; }
      else if (sunKnown && t.exitTiltDeg != null && Math.abs(t.exitTiltDeg) < 0.05) { txt = "COINCIDENT"; tone = "good"; }   // from the drawn geometry, so badge and picture always agree
      else if (sunKnown) { txt = `TRACKING  Δ ${signed(t.exitTiltDeg == null ? 0 : -t.exitTiltDeg, 2)}°`; tone = "warn"; }
      else { txt = "NO SUN"; tone = "dim"; }
    }
    badge.textContent = txt; setTone(badge, tone);
  }

  // ------------------------------------------------------------------ IMU chart (last 30 s of real samples)
  const imuCanvas = $("imuChart"), samples = [];
  let lastSample = null;
  const WINDOW_S = 30;
  function pushSample(s) {
    const g = s.imu;
    if (g.sample_time && g.sample_time !== lastSample && num(g.roll_deg) && num(g.pitch_deg)) {
      lastSample = g.sample_time;
      samples.push({ t: g.sample_time, roll: g.roll_deg, pitch: g.pitch_deg });
    }
    while (samples.length && s.timestamp - samples[0].t > WINDOW_S) samples.shift();
  }
  function drawChart(s) {
    const dpr = window.devicePixelRatio || 1, w = imuCanvas.clientWidth, h = imuCanvas.clientHeight;
    if (imuCanvas.width !== Math.round(w * dpr)) { imuCanvas.width = Math.round(w * dpr); imuCanvas.height = Math.round(h * dpr); }
    const c = imuCanvas.getContext("2d");
    c.setTransform(dpr, 0, 0, dpr, 0, 0);
    c.clearRect(0, 0, w, h);
    let m = 5;
    for (const p of samples) m = Math.max(m, Math.abs(p.roll), Math.abs(p.pitch));
    m = Math.ceil(m / 5) * 5;
    const y = (v) => h / 2 - (v / m) * (h / 2 - 8), x = (t) => w - ((s.timestamp - t) / WINDOW_S) * w;
    c.strokeStyle = "#1c2230"; c.fillStyle = "#64748b"; c.font = "10px monospace"; c.lineWidth = 1;
    c.beginPath(); c.moveTo(0, y(0)); c.lineTo(w, y(0)); c.stroke();
    c.fillText(`+${m}°`, 4, 11); c.fillText(`−${m}°`, 4, h - 4);
    const db = s.imu.deadband_deg;
    if (num(db)) {
      c.setLineDash([3, 3]); c.strokeStyle = "#34d39966";
      for (const v of [db, -db]) { c.beginPath(); c.moveTo(0, y(v)); c.lineTo(w, y(v)); c.stroke(); }
      c.setLineDash([]);
    }
    for (const [key, col] of [["roll", "#38bdf8"], ["pitch", "#f59e0b"]]) {
      if (samples.length < 2) break;
      c.strokeStyle = col; c.lineWidth = 1.5; c.beginPath();
      samples.forEach((p, i) => (i ? c.lineTo(x(p.t), y(p[key])) : c.moveTo(x(p.t), y(p[key]))));
      c.stroke();
    }
    if (samples.length < 2) { c.fillStyle = "#475569"; c.fillText("waiting for IMU data from the Uno …", 40, h / 2 - 8); }
  }

  // ------------------------------------------------------------------ controls
  const cmdMsg = $("cmdMsg");
  async function send(body) {
    try {
      const r = await fetch("/api/command", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
      const d = await r.json();
      cmdMsg.textContent = d.ok ? `${body.cmd}: ${d.message}` : `${body.cmd}: ${d.message}`;
      cmdMsg.style.color = d.ok ? "" : "var(--bad)";
    } catch (e) { cmdMsg.textContent = "server not reachable"; cmdMsg.style.color = "var(--bad)"; }
  }
  let last = null;
  document.querySelectorAll("button[data-cmd]").forEach((b) => b.addEventListener("click", () => {
    if (b.dataset.confirm && !window.confirm(b.dataset.confirm)) return;
    const body = { cmd: b.dataset.cmd };
    if (b.dataset.steps) body.steps = parseInt(b.dataset.steps, 10);
    send(body);
  }));
  document.querySelectorAll("button[data-toggle]").forEach((b) => b.addEventListener("click", () => {
    const cur = last ? get(last, b.dataset.path) : false;
    send({ cmd: b.dataset.toggle, enabled: !cur });
  }));
  $("gotoForm").addEventListener("submit", (e) => {
    e.preventDefault();
    const deg = parseFloat($("gotoDeg").value);
    if (Number.isFinite(deg)) send({ cmd: "goto_x", deg });
  });
  function bindControls(s) {
    const homed = !!s.link.homed;
    document.querySelectorAll("button[data-cmd]").forEach((b) => { if (b.dataset.cmd !== "reconnect") b.disabled = !homed; });
    document.querySelectorAll("button[data-toggle]").forEach((b) => {
      const v = get(s, b.dataset.path);
      b.classList.toggle("on", v === true); b.classList.toggle("off", v === false);
      b.textContent = `${b.dataset.label}: ${v == null ? "—" : v ? "ON" : "OFF"}`;
    });
    $("gotoForm").querySelector("button").disabled = !homed;
    $("reconnectBtn").hidden = s.link.state !== "OFFLINE";
  }

  // ------------------------------------------------------------------ event log
  const log = $("eventLog");
  let logKey = "";
  function bindEvents(s) {
    const ev = s.events || [], key = ev.length + ":" + (ev.length ? ev[ev.length - 1].t : 0);
    if (key === logKey) return;
    logKey = key;
    log.textContent = "";
    for (let i = ev.length - 1; i >= 0; i--) {
      const li = document.createElement("li"), t = document.createElement("span");
      t.className = "t"; t.textContent = new Date(ev[i].t * 1000).toLocaleTimeString([], { hour12: false });
      li.className = ev[i].level; li.append(t, document.createTextNode(ev[i].text));
      log.appendChild(li);
    }
  }

  // ------------------------------------------------------------------ stream
  const streamDot = $("streamDot"), streamState = $("streamState");
  let lastMsg = 0;
  function render(s) {
    last = s;
    bindText(s); bindGauges(s); bindControls(s); bindEvents(s);
    drawOptics(s); pushSample(s); drawChart(s);
  }
  function setStream(txt, cls) {
    streamState.textContent = txt;
    streamDot.className = "dot " + cls;
  }
  function connect() {
    const es = new EventSource("/api/stream");
    es.onmessage = (e) => {
      lastMsg = performance.now();
      let s;
      try { s = JSON.parse(e.data); } catch (_) { return; }
      if (!s.ready) return;
      setStream("LIVE", "live");
      render(s);
    };
    es.onerror = () => setStream("RECONNECTING", "stale");     // EventSource retries by itself
  }
  setInterval(() => { if (lastMsg && performance.now() - lastMsg > 2500) setStream("NO DATA", "stale"); }, 1000);

  buildStatic();
  connect();
})();