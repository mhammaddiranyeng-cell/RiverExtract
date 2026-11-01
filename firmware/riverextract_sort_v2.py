#!/usr/bin/env python3
"""
RiverExtract sorting firmware v2  --  event-driven controller (matches the IEEE paper)

Hardware (unchanged): Raspberry Pi 4, Logitech C270 (USB), A4988 in full step
(MS1-MS3 floating), STEP = GPIO27, DIR = GPIO17, A4988 VDD from the Pi 5 V.
Gate: printed gate v2 (hub on the top edge), J = 3.31e-4 kg m^2.

What changed from riverextract_lite.py
  * One registration per item: a tracker follows each item across frames and
    commits it once (the old loop registered every frame -> false "bin full").
  * The gate acts on the item's predicted POSITION, computed from the
    time-stamped centroid and the belt speed (the old loop acted at detection
    time, or on the falling edge + 0.3 s).
  * Three routes with one stepper (paper Table I):
        OTHER    rest pose +45 deg  -> rear-left bin  (no motion)
        ORGANIC  move to -45 deg before the item arrives, hold, return
        PLASTIC  strike +45 -> -45 deg while the item is in the strike sector
  * Ramped (constant-acceleration) step timing in a separate process with a
    busy-wait clock; the old constant 250/500 step/s starts need more torque
    than a NEMA 17 has with the printed gate (paper Table II).
  * Broccoli and carrot are back in the whitelist (they were filtered out).
  * The dashboard shows the measured inference time and loop period.

Run
  sudo systemctl stop riverextract.service ; sudo pkill -9 -f riverextract_lite
  python3 riverextract_sort_v2.py --jog     # once: put the gate at the rest pose
  python3 riverextract_sort_v2.py --test    # dry run of the three gate actions
  python3 riverextract_sort_v2.py           # normal run, dashboard on :5000
"""
import argparse
import math
import multiprocessing as mp
import os
import sys
import threading
import time

import numpy as np

# =============================================================================
# CONFIG
# =============================================================================
# ---- camera / detector ------------------------------------------------------
YOLO_MODEL = "yolov8n.pt"
CONF_THRESH = 0.55
YOLO_IMGSZ = 320
FRAME_W, FRAME_H = 480, 360
CAM_INDEX = 0
JPEG_QUALITY = 60
PACE_S = 0.6          # sleep per detection cycle (thermal). Smaller -> faster belt allowed
FRAME_LAG_S = 0.017   # mean camera latency subtracted from the read() time stamp

# ---- classes -> routes (the whitelist is exactly these keys) ---------------
ROUTE = {
    "bottle": "PLASTIC", "cup": "PLASTIC", "wine glass": "PLASTIC", "bowl": "PLASTIC",
    "banana": "ORGANIC", "apple": "ORGANIC", "orange": "ORGANIC",
    "broccoli": "ORGANIC", "carrot": "ORGANIC",
    "book": "OTHER", "cell phone": "OTHER",
}

# ---- stepper / gate -----------------------------------------------------------
DIR_PIN, STEP_PIN = 17, 27
DIR_CCW_LEVEL = 1      # DIR level that turns the gate counter-clockwise seen from
                       # ABOVE (towards the left side bin). Flip to 0 if reversed.
STEP_RAD = math.radians(1.8)
J_GATE = 3.31e-4       # kg m^2, printed gate v2 (solid-PLA bound)
TAU_STRIKE = 0.100     # N m  -> 90 deg in 0.144 s
TAU_ORG = 0.050        # N m  -> 90 deg in 0.204 s
TAU_RET = 0.025        # N m  acceleration limit of the gentle return
OMEGA_RET = 3.0        # rad/s cap of the gentle return
DWELL_S = 0.05         # pause at -45 deg after a strike
POSE_REST = +25        # steps from the along-belt direction: +45 deg
POSE_ORG = -25         # -45 deg

# ---- belt / camera geometry (calibrate once, see README) --------------------
BELT_SPEED = 200.0     # mm/s nominal; refined online from the tracks
GAP_G = 200.0          # mm: downstream edge of the detection window -> pivot
ALONG_AXIS = "v"       # image axis that runs along the belt: "v" rows, "u" columns
ALONG_SIGN = +1        # +1 if items move toward larger pixel index on that axis
MM_PER_PX = 300.0 / 360.0   # belt mm per pixel (measure with a ruler on the belt)
PX_WINDOW_END = 350    # pixel (along axis) of the window's downstream edge
PX_CENTRE = 240        # pixel (across axis) of the belt centre line
LEFT_SIGN = -1         # +1 if the belt's LEFT side has the larger across-pixel index
BAND_MM = 70.7         # capture band half-width (lane guides)

# ---- controller ---------------------------------------------------------------
COMMIT_M = 1           # detections needed to commit a track
ASSOC_X_MM = 60.0      # association gate along the belt
ASSOC_Y_MM = 50.0      # association gate across the belt
ORG_MARGIN_MM = 40.0   # organic move: start this far before the latest start
RET_DELAY_S = 0.10     # organic: return this long after the predicted contact
STRIKE_LATE_MM = 10.0  # strike still allowed this far inside the window end
DISPATCH_LEAD_S = 0.06 # actions are handed to the gate process this early
ITEM_W_MM = 65.0       # reference item diameter used for contact prediction

# ---- bins ---------------------------------------------------------------------
BIN_FULL_COUNT = 3
FULL_RESET_SECONDS = 5

# ---- capture windows from the planar contact simulation (paper Fig. 4) ------
# centroid x (mm, pivot = 0) at action start, item diameter 65 mm, vs belt
# speed (rows, mm/s) and lateral position (columns, mm).
CW_V = [50, 75, 100, 125, 150, 175, 200, 225, 250, 275, 300, 350, 400]
CW_Y = [-38.2, -22.9, -7.6, 7.6, 22.9, 38.2]
CW_LO = [[-126, -130, -132, -132, -130, -128], [-126, -130, -132, -132, -132, -130], [-128, -132, -134, -134, -134, -132], [-128, -134, -136, -136, -136, -134], [-130, -134, -138, -138, -138, -136], [-132, -136, -140, -140, -140, -138], [-132, -138, -140, -142, -142, -140], [-134, -140, -142, -144, -144, -142], [-134, -140, -144, -146, -146, -144], [-136, -142, -146, -148, -148, -146], [-138, -144, -148, -148, -150, -148], [-140, -146, -150, -152, -154, -152], [-142, -150, -154, -156, -156, -156]]
CW_HI = [[-60, -60, -62, -64, -60, -58], [-62, -62, -62, -64, -62, -60], [-62, -62, -64, -66, -64, -62], [-62, -62, -64, -68, -66, -64], [-62, -62, -66, -70, -68, -68], [-62, -62, -66, -70, -70, -70], [-62, -62, -66, -72, -72, -72], [-62, -62, -68, -74, -72, -74], [-62, -62, -68, -74, -74, -74], [-62, -62, -70, -76, -76, -76], [-62, -62, -70, -78, -78, -78], [-62, -62, -72, -80, -82, -82], [-64, -64, -74, -82, -84, -86]]
CW_OMAX = [[-134, -138, -140, -142, -140, -136], [-136, -140, -144, -144, -142, -140], [-138, -144, -146, -146, -146, -142], [-140, -146, -148, -150, -148, -146], [-142, -148, -150, -152, -152, -148], [-144, -150, -154, -154, -154, -152], [-146, -152, -156, -156, -156, -156], [-148, -154, -158, -160, -160, -158], [-150, -156, -160, -162, -162, -162], [-152, -158, -162, -164, -166, -164], [-154, -162, -166, -168, -168, -168], [-160, -166, -170, -172, -174, -174], [-164, -170, -176, -178, -180, -180]]


def _interp2(table, v, y):
    v = min(max(v, CW_V[0]), CW_V[-1])
    y = min(max(y, CW_Y[0]), CW_Y[-1])
    rows = [np.interp(y, CW_Y, r) for r in table]
    return float(np.interp(v, CW_V, rows))


def strike_window(v, y):
    lo, hi = _interp2(CW_LO, v, y), _interp2(CW_HI, v, y)
    return lo, hi, 0.5 * (lo + hi)


def organic_latest_start(v, y):
    return _interp2(CW_OMAX, v, y)


def contact_x_org(y, w=ITEM_W_MM):
    """Centroid x at which a disc first touches the -45 deg plate."""
    return -y - (w / 2 + 2.5) * math.sqrt(2)


# =============================================================================
# STEP TIMING (exact constant-acceleration instants, AVR446 form)
# =============================================================================
def pulse_times(nsteps, kind, tau, wmax=None):
    """Instants (s, from move start) of the nsteps STEP pulses.  Pulse k is
    issued when the profile reaches (k-1) steps, so the commanded position
    leads the ramp by one step."""
    if nsteps <= 0:
        return []
    al = tau / J_GATE
    D = nsteps * STEP_RAD
    if kind == "trap" and wmax is not None:
        ta = wmax / al
        if D > wmax * ta:                      # reaches the cap
            da = 0.5 * al * ta ** 2
            tc = (D - 2 * da) / wmax
            tm = 2 * ta + tc
            out = []
            for k in range(nsteps):
                th = k * STEP_RAD
                if th <= da:
                    out.append(math.sqrt(2 * th / al))
                elif th <= D - da:
                    out.append(ta + (th - da) / wmax)
                else:
                    out.append(tm - math.sqrt(max(0.0, 2 * (D - th) / al)))
            return out
    tm = 2 * math.sqrt(D / al)
    out = []
    for k in range(nsteps):
        th = k * STEP_RAD
        if th <= D / 2:
            out.append(math.sqrt(2 * th / al))
        else:
            out.append(tm - math.sqrt(max(0.0, 2 * (D - th) / al)))
    return out


def move_duration(nsteps, kind, tau, wmax=None):
    al = tau / J_GATE
    D = abs(nsteps) * STEP_RAD
    if kind == "trap" and wmax is not None and D > wmax * (wmax / al):
        return D / wmax + wmax / al
    return 2 * math.sqrt(D / al)


T_STRIKE = move_duration(50, "tri", TAU_STRIKE)
T_ORGMOVE = move_duration(50, "tri", TAU_ORG)
T_RET = move_duration(50, "trap", TAU_RET, OMEGA_RET)


# =============================================================================
# GATE PROCESS: owns the GPIO, runs moves at absolute (monotonic) times
# =============================================================================
def gate_process(cmd_q, pos_val, busy_val, dry, log_q=None):
    try:
        os.sched_setaffinity(0, {3})           # keep a core for step timing
    except Exception:
        pass
    try:
        os.nice(-10)
    except Exception:
        pass
    if dry:
        GPIO = None
    else:
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(DIR_PIN, GPIO.OUT)
        GPIO.setup(STEP_PIN, GPIO.OUT, initial=GPIO.LOW)
    pos = pos_val.value

    def wait_until(t):
        while True:
            d = t - time.monotonic()
            if d <= 0:
                return
            if d > 0.003:
                time.sleep(d - 0.002)

    while True:
        cmd = cmd_q.get()
        if cmd is None:
            break
        t0, target, kind, tau, wmax = cmd
        n = target - pos
        if n == 0:
            continue
        busy_val.value = 1
        ccw = n > 0
        wait_until(t0 - 0.002)
        if GPIO:
            GPIO.output(DIR_PIN, DIR_CCW_LEVEL if ccw else 1 - DIR_CCW_LEVEL)
        ts = pulse_times(abs(n), kind, tau, wmax)
        t0 = max(t0, time.monotonic() + 0.001)    # late command: start now
        actual = []
        for tk in ts:
            tt = t0 + tk
            while time.monotonic() < tt:
                pass
            if GPIO:
                GPIO.output(STEP_PIN, 1)
                t_hi = time.monotonic() + 3e-6
                while time.monotonic() < t_hi:
                    pass
                GPIO.output(STEP_PIN, 0)
            if log_q is not None:
                actual.append(time.monotonic() - t0)
            pos += 1 if ccw else -1
            pos_val.value = pos
        if log_q is not None:
            log_q.put((t0, target, kind, ts, actual))
        busy_val.value = 0
    if GPIO:
        GPIO.cleanup()


class Gate:
    """Front end used by the scheduler: plans are dispatched just in time."""

    def __init__(self, dry=False, log_q=None):
        self.q = mp.Queue()
        self.pos = mp.Value("i", POSE_REST)
        self.busy = mp.Value("i", 0)
        self.p = mp.Process(target=gate_process,
                            args=(self.q, self.pos, self.busy, dry, log_q), daemon=True)
        self.p.start()

    def move(self, t0, target, kind, tau, wmax=None):
        self.q.put((t0, int(target), kind, float(tau), wmax))

    def strike(self, t0):
        self.move(t0, POSE_ORG, "tri", TAU_STRIKE)
        self.move(t0 + T_STRIKE + DWELL_S, POSE_REST, "trap", TAU_RET, OMEGA_RET)
        return t0 + T_STRIKE + DWELL_S + T_RET

    def organic_move(self, t0):
        self.move(t0, POSE_ORG, "tri", TAU_ORG)
        return t0 + T_ORGMOVE

    def ret(self, t0):
        self.move(t0, POSE_REST, "trap", TAU_RET, OMEGA_RET)
        return t0 + T_RET

    def close(self):
        self.q.put(None)


# =============================================================================
# BINS
# =============================================================================
bins = {k: {"count": 0, "full": False, "last": 0.0} for k in ("PLASTIC", "ORGANIC", "OTHER")}
bin_lock = threading.Lock()
notifications = []


def register(route):
    with bin_lock:
        b = bins[route]
        b["count"] += 1
        b["last"] = time.time()
        if b["count"] >= BIN_FULL_COUNT and not b["full"]:
            b["full"] = True
            msg = f"Bin '{route}' is FULL ({b['count']} items)"
            print("[NOTIFY]", msg, flush=True)
            notifications.append({"t": time.time(), "msg": msg})
            del notifications[:-6]


def bin_reset_loop():
    while True:
        now = time.time()
        with bin_lock:
            for name, b in bins.items():
                if b["full"] and now - b["last"] >= FULL_RESET_SECONDS:
                    b["count"], b["full"] = 0, False
                    print(f"[NOTIFY] Bin '{name}' reset", flush=True)
        time.sleep(1)


# =============================================================================
# TRACKER + SCHEDULER
# =============================================================================
class Track:
    _next = 1

    def __init__(self, det):
        self.id = Track._next
        Track._next += 1
        self.dets = [det]           # (t_cap, x, y, cls, conf)
        self.votes = {}
        self.vote(det)
        self.committed = False
        self.route = None
        self.plan = None            # dict(kind, t0, t_ret)
        self.dispatched = False
        self.registered = False

    def vote(self, det):
        r = ROUTE[det[3]]
        self.votes[r] = self.votes.get(r, 0.0) + det[4]

    def best_route(self):
        return max(self.votes, key=self.votes.get)

    def last(self):
        return self.dets[-1]

    def y_mean(self):
        return float(np.mean([d[2] for d in self.dets]))


class Controller:
    def __init__(self, gate):
        self.gate = gate
        self.tracks = []
        self.lock = threading.Lock()
        self.v_hat = BELT_SPEED
        self.v_samples = []
        self.busy_until = 0.0        # end of the last dispatched/planned gate action
        self.log = []                # last decisions for the dashboard

    # ---- belt speed from track motion ------------------------------------
    def _update_speed(self, tr):
        if len(tr.dets) >= 2:
            (t1, x1, *_), (t2, x2, *_) = tr.dets[-2], tr.dets[-1]
            if t2 - t1 > 0.2:
                v = (x2 - x1) / (t2 - t1)
                if 0.7 * BELT_SPEED <= v <= 1.3 * BELT_SPEED:
                    self.v_samples = (self.v_samples + [v])[-15:]
                    self.v_hat = float(np.median(self.v_samples))

    def predict_x(self, tr, t):
        tc, xc = tr.last()[0], tr.last()[1]
        return xc + self.v_hat * (t - tc)

    # ---- detections from one frame -----------------------------------------
    def on_frame(self, t_cap, dets):
        with self.lock:
            free = list(range(len(dets)))
            for tr in self.tracks:
                xp = self.predict_x(tr, t_cap)
                best, bd = None, 1e9
                for i in free:
                    d = dets[i]
                    dx, dy = abs(d[1] - xp), abs(d[2] - tr.y_mean())
                    if dx < ASSOC_X_MM and dy < ASSOC_Y_MM and dx < bd:
                        best, bd = i, dx
                if best is not None:
                    tr.dets.append((t_cap,) + tuple(dets[best][1:]))
                    tr.vote(dets[best])
                    free.remove(best)
                    self._update_speed(tr)
            for i in free:
                self.tracks.append(Track((t_cap,) + tuple(dets[i][1:])))
            now = time.monotonic()
            for tr in self.tracks:
                if not tr.committed and len(tr.dets) >= COMMIT_M:
                    tr.committed = True
                    tr.route = tr.best_route()
                if tr.committed and not tr.dispatched:
                    self._plan(tr, now)

    # ---- planning (re-run at every detection until dispatch) ---------------
    def _plan(self, tr, now):
        v = self.v_hat
        tc, xc = tr.last()[0], tr.last()[1]
        y = tr.y_mean()
        t_dec = now + 0.005
        if tr.route == "PLASTIC":
            lo, hi, aim = strike_window(v, y)
            t0 = tc + (aim - xc) / v
            if t0 < t_dec:
                if xc + v * (t_dec - tc) <= hi - STRIKE_LATE_MM:
                    t0 = t_dec
                else:
                    tr.plan = None
                    return
            tr.plan = dict(kind="strike", t0=t0, t_end=t0 + T_STRIKE + DWELL_S + T_RET)
        elif tr.route == "ORGANIC":
            om = organic_latest_start(v, y)
            t0 = tc + (om - ORG_MARGIN_MM - xc) / v
            if t0 < t_dec:
                if xc + v * (t_dec - tc) <= om - 5.0:
                    t0 = t_dec
                else:
                    tr.plan = None
                    return
            t_ret = tc + (contact_x_org(y) - xc) / v + RET_DELAY_S
            tr.plan = dict(kind="organic", t0=t0, t_ret=max(t_ret, t0 + T_ORGMOVE),
                           t_end=max(t_ret, t0 + T_ORGMOVE) + T_RET)
        else:
            t_pass = tc + (0.0 - xc) / v
            tr.plan = dict(kind="none", t0=t_pass, t_end=t_pass)

    # ---- dispatcher: hands actions to the gate just in time -----------------
    def dispatch_loop(self):
        while True:
            now = time.monotonic()
            with self.lock:
                # pending return of the organic pose
                if self.pending_return is not None and now >= self.pending_return - DISPATCH_LEAD_S:
                    self.busy_until = self.gate.ret(self.pending_return)
                    self.pending_return = None
                for tr in sorted(self.tracks, key=lambda t: t.plan["t0"] if t.plan else 1e18):
                    if not tr.committed or tr.dispatched:
                        continue
                    if tr.plan is None:
                        # deadline missed: the item takes the rest route (OTHER)
                        if self.predict_x(tr, now) > 0:
                            tr.dispatched = True
                            self._register(tr, "OTHER", "late -> OTHER")
                        continue
                    p = tr.plan
                    if now < p["t0"] - DISPATCH_LEAD_S:
                        continue
                    tr.dispatched = True
                    if p["kind"] == "none":
                        self._register(tr, "OTHER", "rest")
                        continue
                    if p["kind"] == "organic" and self.pending_return is not None:
                        # gate already at -45 deg: extend the hold
                        self.pending_return = max(self.pending_return, p["t_ret"])
                        self.busy_until = self.pending_return + T_RET
                        self._register(tr, "ORGANIC", "hold extended")
                        continue
                    if p["t0"] < self.busy_until or self.pending_return is not None:
                        # gate occupied by the previous item: this item takes the
                        # pose the gate holds when it arrives
                        held = "ORGANIC" if self.pending_return is not None else "OTHER"
                        self._register(tr, held, "gate busy")
                        continue
                    if p["kind"] == "strike":
                        self.busy_until = self.gate.strike(p["t0"])
                        self._register(tr, "PLASTIC", "strike")
                    else:
                        self.gate.organic_move(p["t0"])
                        self.pending_return = p["t_ret"]
                        self.busy_until = p["t_ret"] + T_RET
                        self._register(tr, "ORGANIC", "organic")
                # forget old tracks
                self.tracks = [t for t in self.tracks
                               if not (t.dispatched and now - t.last()[0] > 10.0)
                               and not (not t.dispatched and now - t.last()[0] > 30.0)]
            time.sleep(0.005)

    pending_return = None

    def _register(self, tr, route, why):
        if tr.registered:
            return
        tr.registered = True
        register(route)
        self.log = (self.log + [f"#{tr.id} {tr.best_route()} -> {route} ({why})"])[-6:]
        print(f"[SORT] track {tr.id} {tr.best_route()} -> {route} ({why})", flush=True)


# =============================================================================
# CAMERA + DETECTION
# =============================================================================
def px_to_belt(u, v):
    p_along, p_across = (v, u) if ALONG_AXIS == "v" else (u, v)
    x = -GAP_G - ALONG_SIGN * (PX_WINDOW_END - p_along) * MM_PER_PX
    y = LEFT_SIGN * (p_across - PX_CENTRE) * MM_PER_PX
    return x, y


def belt_to_px(x, y):
    p_along = PX_WINDOW_END - ALONG_SIGN * (-GAP_G - x) / MM_PER_PX
    p_across = PX_CENTRE + y / (LEFT_SIGN * MM_PER_PX)
    return (p_across, p_along) if ALONG_AXIS == "v" else (p_along, p_across)


class Vision:
    def __init__(self, ctrl):
        import cv2
        import torch
        from ultralytics import YOLO
        self.cv2 = cv2
        torch.set_num_threads(2)
        print("Loading YOLOv8...", flush=True)
        self.model = YOLO(YOLO_MODEL)
        self.cap = cv2.VideoCapture(CAM_INDEX)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.ctrl = ctrl
        self.lock = threading.Lock()
        self.frame, self.t_cap = None, 0.0
        self.jpeg = None
        self.boxes = []
        self.t_inf = 0.0
        self.T_loop = 0.0
        threading.Thread(target=self._cam, daemon=True).start()
        threading.Thread(target=self._det, daemon=True).start()

    def _cam(self):
        cv2 = self.cv2
        while True:
            ok, fr = self.cap.read()
            t = time.monotonic() - FRAME_LAG_S
            if not ok or fr is None:
                time.sleep(0.05)
                continue
            fr = cv2.resize(fr, (FRAME_W, FRAME_H))
            with self.lock:
                self.frame, self.t_cap = fr, t
                boxes = list(self.boxes)
            vis = fr.copy()
            # calibration overlay: window end and capture band
            a = belt_to_px(-GAP_G, -BAND_MM)
            b = belt_to_px(-GAP_G, BAND_MM)
            cv2.line(vis, tuple(map(int, a)), tuple(map(int, b)), (0, 0, 255), 1)
            for yb in (-BAND_MM, BAND_MM):
                p = belt_to_px(-GAP_G - 300, yb)
                q = belt_to_px(-GAP_G, yb)
                cv2.line(vis, tuple(map(int, p)), tuple(map(int, q)), (0, 200, 255), 1)
            for (lab, conf, x1, y1, x2, y2) in boxes:
                cv2.rectangle(vis, (x1, y1), (x2, y2), (136, 148, 13), 2)
                cv2.putText(vis, f"{lab} {conf * 100:.0f}%", (x1 + 3, max(12, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            ok2, j = cv2.imencode(".jpg", vis, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            if ok2:
                with self.lock:
                    self.jpeg = j.tobytes()

    def _det(self):
        t_prev = None
        while True:
            with self.lock:
                fr, t_cap = self.frame, self.t_cap
            if fr is None:
                time.sleep(0.1)
                continue
            t_a = time.monotonic()
            res = self.model(fr, conf=CONF_THRESH, imgsz=YOLO_IMGSZ, verbose=False)
            self.t_inf = time.monotonic() - t_a
            dets, boxes = [], []
            for r in res:
                for box in r.boxes:
                    cls = self.model.names[int(box.cls[0])]
                    if cls not in ROUTE:
                        continue
                    conf = float(box.conf[0])
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    x, y = px_to_belt(0.5 * (x1 + x2), 0.5 * (y1 + y2))
                    dets.append((t_cap, x, y, cls, conf))
                    boxes.append((cls, conf, x1, y1, x2, y2))
            with self.lock:
                self.boxes = boxes
            self.ctrl.on_frame(t_cap, dets)
            now = time.monotonic()
            if t_prev is not None:
                self.T_loop = now - t_prev
            t_prev = now
            time.sleep(PACE_S)


# =============================================================================
# DASHBOARD
# =============================================================================
DASH = """<!DOCTYPE html><html><head><meta charset='utf-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'><title>RiverExtract</title>
<style>*{margin:0;padding:0;box-sizing:border-box}body{font-family:system-ui,Arial;background:#0b1416;color:#e6eef0;padding:16px}
.top{display:flex;gap:16px;justify-content:center;margin-bottom:6px}.top img{height:56px}
h1{color:#2f9d8f;font-size:24px;text-align:center}.sub{text-align:center;color:#7c8a8d;font-size:12px;margin-bottom:14px}
.wrap{max-width:640px;margin:0 auto}.panel{background:#0f1e21;border:1px solid #1d3438;border-radius:14px;padding:10px;margin-bottom:12px}
.panel img{width:100%;border-radius:8px;display:block}.bins{display:flex;gap:12px;margin-bottom:12px}
.bin{flex:1;background:#0f1e21;border:1px solid #1d3438;border-radius:14px;padding:12px;text-align:center}
.bin .n{font-size:12px;color:#7c8a8d;text-transform:uppercase}.bin .c{font-size:30px;font-weight:700;margin:6px 0}
.bin.full{border-color:#e8552d;background:#2a1512}.bin .s{font-size:11px;color:#4a5a5d}.bin.full .s{color:#ff7a52;font-weight:700}
.info{font-size:12px;color:#9fb3b6;line-height:1.6}.notif div{color:#ff9d7a}</style></head><body>
<div class='top'><img src='/logo/aust' onerror="this.style.display='none'"><img src='/logo/foe' onerror="this.style.display='none'"></div>
<h1>RiverExtract</h1><div class='sub'>Event-driven vision sorting</div><div class='wrap'>
<div class='panel'><img src='/video'></div>
<div class='bins'>
<div class='bin' id='bin-PLASTIC'><div class='n'>Plastic (side)</div><div class='c' id='c-PLASTIC'>0</div><div class='s' id='s-PLASTIC'>ok</div></div>
<div class='bin' id='bin-ORGANIC'><div class='n'>Organic (rear-right)</div><div class='c' id='c-ORGANIC'>0</div><div class='s' id='s-ORGANIC'>ok</div></div>
<div class='bin' id='bin-OTHER'><div class='n'>Other (rear-left)</div><div class='c' id='c-OTHER'>0</div><div class='s' id='s-OTHER'>ok</div></div></div>
<div class='panel info' id='info'></div><div class='panel notif' id='notif'></div></div>
<script>async function poll(){try{let s=await(await fetch('/api/status')).json();
for(const k of ['PLASTIC','ORGANIC','OTHER']){document.getElementById('c-'+k).textContent=s.bins[k].count;
let b=document.getElementById('bin-'+k),t=document.getElementById('s-'+k);if(s.bins[k].full){b.classList.add('full');t.textContent='FULL'}else{b.classList.remove('full');t.textContent='ok'}}
document.getElementById('info').innerHTML='inference '+s.t_inf_ms+' ms &middot; loop period T '+s.T_ms+' ms &middot; belt speed '+s.v_hat+' mm/s<br>'+(s.log||[]).join('<br>');
document.getElementById('notif').innerHTML=(s.notifications||[]).map(m=>'<div>&#9888; '+m+'</div>').join('')||'<span style=color:#4a5a5d>No alerts</span>';}catch(e){}}
setInterval(poll,700);poll();</script></body></html>"""


def run_server(vision, ctrl):
    from flask import Flask, Response, jsonify, send_file
    app = Flask(__name__)

    def gen():
        while True:
            with vision.lock:
                j = vision.jpeg
            if j:
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + j + b"\r\n"
            time.sleep(0.05)

    @app.route("/video")
    def video():
        return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.route("/logo/<which>")
    def logo(which):
        fn = "aust_logo.png" if which == "aust" else "faculty_logo.png"
        p = os.path.join(os.path.expanduser("~"), fn)
        return send_file(p) if os.path.exists(p) else ("", 404)

    @app.route("/api/status")
    def status():
        with bin_lock:
            b = {k: {"count": v["count"], "full": v["full"]} for k, v in bins.items()}
            n = [x["msg"] for x in notifications[-4:]]
        return jsonify(bins=b, notifications=n, log=ctrl.log,
                       t_inf_ms=int(vision.t_inf * 1000), T_ms=int(vision.T_loop * 1000),
                       v_hat=int(ctrl.v_hat))

    @app.route("/")
    def index():
        return DASH

    print("RiverExtract sorting v2 on :5000", flush=True)
    app.run(host="0.0.0.0", port=5000, threaded=True)


# =============================================================================
# TOOLS: jog and dry test
# =============================================================================
def jog():
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(DIR_PIN, GPIO.OUT)
    GPIO.setup(STEP_PIN, GPIO.OUT, initial=GPIO.LOW)
    print("Jog: a = one step CCW, d = one step CW, A/D = 5 steps, q = done.")
    print("Finish with the gate at the REST pose: 45 deg, downstream end toward the LEFT edge.")
    while True:
        k = input("> ").strip()
        if k == "q":
            break
        n = {"a": 1, "d": -1, "A": 5, "D": -5}.get(k, 0)
        GPIO.output(DIR_PIN, DIR_CCW_LEVEL if n > 0 else 1 - DIR_CCW_LEVEL)
        time.sleep(0.002)
        for _ in range(abs(n)):
            GPIO.output(STEP_PIN, 1)
            time.sleep(0.002)
            GPIO.output(STEP_PIN, 0)
            time.sleep(0.03)
    GPIO.cleanup()


def dry_test(dry):
    gate = Gate(dry=dry)
    t = time.monotonic() + 0.5
    print(f"strike: {T_STRIKE:.3f} s, organic move: {T_ORGMOVE:.3f} s, return: {T_RET:.3f} s")
    end = gate.strike(t)
    print("strike scheduled; organic move 1.5 s after it, hold 1 s, return")
    t1 = gate.organic_move(end + 1.5)
    end = gate.ret(t1 + 1.0)
    while time.monotonic() < end + 0.3:
        time.sleep(0.05)
    print("final position (steps):", gate.pos.value, "(rest =", POSE_REST, ")")
    gate.close()


# =============================================================================
if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--jog", action="store_true")
    ap.add_argument("--test", action="store_true")
    ap.add_argument("--dry", action="store_true", help="no GPIO (desk test)")
    a = ap.parse_args()
    if a.jog:
        jog()
        sys.exit(0)
    if a.test:
        dry_test(a.dry)
        sys.exit(0)
    threading.Thread(target=bin_reset_loop, daemon=True).start()
    gate = Gate(dry=a.dry)
    ctrl = Controller(gate)
    threading.Thread(target=ctrl.dispatch_loop, daemon=True).start()
    vision = Vision(ctrl)
    try:
        run_server(vision, ctrl)
    finally:
        gate.close()
