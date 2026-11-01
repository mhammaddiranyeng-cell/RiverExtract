#!/usr/bin/env python3
"""
RiverExtract sorting firmware v2.1  --  v2 event-driven controller + measurement tools

v2.1 keeps the v2 gate control unchanged and adds what the experiments need:
  * CSV logs of every frame, decision, gate move, Pi temperature/throttling, and
    the ground truth typed on the dashboard after each pass
    (~/riverextract_logs/<date_time>/)
  * --pace S        detection pacing (v2 default 0.6 s; 0 = back-to-back)
  * --legacy        the first firmware's trigger logic on the same gate (baseline):
                    one registration per cycle, organic swing per cycle,
                    plastic strike 0.3 s after the falling edge
  * --offsets LIST  strike aim offsets (mm) cycled over plastic items, to measure
                    the real capture window
  * --strike-test N repeated strikes without items, to check for lost steps
  * --save-frames K save every K-th camera frame for labelling a new detector
  * --belt-speed V  starting belt-speed estimate (mm/s); the estimate now follows
                    speed changes instead of staying within +/-30% of the nominal
  * calibration (camera geometry, DIR level, belt speed, model) is read from your
    existing riverextract_sort_v2.py, so nothing has to be re-entered

Original v2 notes follow.

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

v2.1 runs (same flags as above, plus):
  python3 riverextract_sort_v2_1.py                       # v2 behaviour + logs
  python3 riverextract_sort_v2_1.py --pace 0              # no pacing sleep
  python3 riverextract_sort_v2_1.py --legacy              # first-firmware baseline
  python3 riverextract_sort_v2_1.py --offsets=-60,-40,-20,0,20,40,60
  python3 riverextract_sort_v2_1.py --strike-test 20 --tau 0.15
  python3 riverextract_sort_v2_1.py --save-frames 15      # 2 frame/s to disk
  python3 riverextract_sort_v2_1.py --model riverextract_debris.pt   # new detector
"""
import argparse
import ast
import csv
import json
import math
import subprocess
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

# ---- v2.1 experiment options (set from the command line) ---------------------
LOG_ROOT = os.path.expanduser("~/riverextract_logs")
LEGACY = False             # first-firmware trigger logic (baseline)
LEGACY_HOLD_S = 0.25       # first firmware: hold at the organic pose
LEGACY_DELAY_S = 0.30      # first firmware: plastic strike after the falling edge
STRIKE_OFFSETS = [0.0]     # mm added to the strike aim, cycled over plastic items
SAVE_EVERY = 0             # save every K-th camera frame (0 = off)
SAVE_MAX = 6000            # stop saving after this many frames
CALIB_NAMES = ("YOLO_MODEL", "CONF_THRESH", "YOLO_IMGSZ", "FRAME_W", "FRAME_H",
               "CAM_INDEX", "PACE_S", "FRAME_LAG_S", "DIR_CCW_LEVEL", "BELT_SPEED",
               "GAP_G", "ALONG_AXIS", "ALONG_SIGN", "MM_PER_PX", "PX_WINDOW_END",
               "PX_CENTRE", "LEFT_SIGN", "BAND_MM", "BIN_FULL_COUNT",
               "FULL_RESET_SECONDS", "POSE_REST", "POSE_ORG")

# ---- capture windows from the planar contact simulation (paper Fig. 4) ------
# centroid x (mm, pivot = 0) at action start, item diameter 65 mm, vs belt
# speed (rows, mm/s) and lateral position (columns, mm).
CW_V = [50, 75, 100, 125, 150, 175, 200, 225, 250, 275, 300, 350, 400]
CW_Y = [-38.2, -22.9, -7.6, 7.6, 22.9, 38.2]
CW_LO = [[-126, -130, -132, -132, -130, -128], [-126, -130, -132, -132, -132, -130], [-128, -132, -134, -134, -134, -132], [-128, -134, -136, -136, -136, -134], [-130, -134, -138, -138, -138, -136], [-132, -136, -140, -140, -140, -138], [-132, -138, -140, -142, -142, -140], [-134, -140, -142, -144, -144, -142], [-134, -140, -144, -146, -146, -144], [-136, -142, -146, -148, -148, -146], [-138, -144, -148, -148, -150, -148], [-140, -146, -150, -152, -154, -152], [-142, -150, -154, -156, -156, -156]]
CW_HI = [[-60, -60, -62, -64, -60, -58], [-62, -62, -62, -64, -62, -60], [-62, -62, -64, -66, -64, -62], [-62, -62, -64, -68, -66, -64], [-62, -62, -66, -70, -68, -68], [-62, -62, -66, -70, -70, -70], [-62, -62, -66, -72, -72, -72], [-62, -62, -68, -74, -72, -74], [-62, -62, -68, -74, -74, -74], [-62, -62, -70, -76, -76, -76], [-62, -62, -70, -78, -78, -78], [-62, -62, -72, -80, -82, -82], [-64, -64, -74, -82, -84, -86]]
CW_OMAX = [[-134, -138, -140, -142, -140, -136], [-136, -140, -144, -144, -142, -140], [-138, -144, -146, -146, -146, -142], [-140, -146, -148, -150, -148, -146], [-142, -148, -150, -152, -152, -148], [-144, -150, -154, -154, -154, -152], [-146, -152, -156, -156, -156, -156], [-148, -154, -158, -160, -160, -158], [-150, -156, -160, -162, -162, -162], [-152, -158, -162, -164, -166, -164], [-154, -162, -166, -168, -168, -168], [-160, -166, -170, -172, -174, -174], [-164, -170, -176, -178, -180, -180]]


def _num(node):
    """Value of a constant or a small arithmetic expression of constants."""
    if isinstance(node, ast.Constant) and isinstance(node.value, (int, float, str)):
        return node.value
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.USub, ast.UAdd)):
        v = _num(node.operand)
        return -v if isinstance(node.op, ast.USub) else v
    if isinstance(node, ast.BinOp) and isinstance(node.op, (ast.Add, ast.Sub, ast.Mult, ast.Div)):
        a, b = _num(node.left), _num(node.right)
        if isinstance(node.op, ast.Add):
            return a + b
        if isinstance(node.op, ast.Sub):
            return a - b
        if isinstance(node.op, ast.Mult):
            return a * b
        return a / b
    raise ValueError("not a constant")


def import_calibration(path):
    """Copy the calibrated CONFIG values from the v2 file the prototype runs."""
    if not path or not os.path.exists(path):
        print(f"[CAL] {path} not found: using the values in this file", flush=True)
        return {}
    tree = ast.parse(open(path).read())
    got = {}
    for node in tree.body:
        if not isinstance(node, ast.Assign) or len(node.targets) != 1:
            continue
        tgt = node.targets[0]
        pairs = []
        if isinstance(tgt, ast.Name):
            pairs = [(tgt.id, node.value)]
        elif isinstance(tgt, ast.Tuple) and isinstance(node.value, ast.Tuple):
            pairs = [(t.id, v) for t, v in zip(tgt.elts, node.value.elts)
                     if isinstance(t, ast.Name)]
        for name, val in pairs:
            if name in CALIB_NAMES:
                try:
                    got[name] = _num(val)
                except ValueError:
                    pass
    globals().update(got)
    print(f"[CAL] {len(got)} calibration values read from {path}", flush=True)
    return got


class ExpLog:
    """Append-only CSV files for one run, flushed after every row."""

    def __init__(self, root, info):
        self.dir = os.path.join(root, time.strftime("%Y%m%d_%H%M%S"))
        os.makedirs(self.dir, exist_ok=True)
        self.lock = threading.Lock()
        self.files = {}
        self.t0_wall, self.t0_mono = time.time(), time.monotonic()
        with open(os.path.join(self.dir, "run_info.json"), "w") as f:
            json.dump(info, f, indent=1, default=str)
        print(f"[LOG] writing to {self.dir}", flush=True)

    def wall(self, t_mono):
        return round(self.t0_wall + (t_mono - self.t0_mono), 4)

    def row(self, name, header, values):
        with self.lock:
            if name not in self.files:
                f = open(os.path.join(self.dir, name + ".csv"), "w", newline="")
                w = csv.writer(f)
                w.writerow(header)
                self.files[name] = (f, w)
            f, w = self.files[name]
            w.writerow(values)
            f.flush()


LOG = None


def log_row(name, header, values):
    if LOG is not None:
        LOG.row(name, header, values)


DEC_HDR = ["t_wall", "mode", "track", "route_detected", "route_sent", "reason",
           "n_dets", "t_first", "t_last", "x_last_mm", "y_mean_mm", "v_hat_mm_s",
           "plan", "plan_t0", "offset_mm", "votes"]


def cpu_temp():
    try:
        return int(open("/sys/class/thermal/thermal_zone0/temp").read()) / 1000.0
    except Exception:
        return None


def throttled():
    try:
        out = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True,
                             text=True, timeout=2).stdout.strip()
        return out.split("=")[-1]
    except Exception:
        return ""


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

    def strike(self, t0, tau=None):
        tau = TAU_STRIKE if tau is None else tau
        t_s = move_duration(50, "tri", tau)
        self.move(t0, POSE_ORG, "tri", tau)
        self.move(t0 + t_s + DWELL_S, POSE_REST, "trap", TAU_RET, OMEGA_RET)
        return t0 + t_s + DWELL_S + T_RET

    def organic_move(self, t0):
        self.move(t0, POSE_ORG, "tri", TAU_ORG)
        return t0 + T_ORGMOVE

    def ret(self, t0):
        self.move(t0, POSE_REST, "trap", TAU_RET, OMEGA_RET)
        return t0 + T_RET

    def close(self):
        self.q.put(None)


def drain_moves(log_q):
    """Gate-process move records -> moves.csv (planned vs. issued step times)."""
    while True:
        rec = log_q.get()
        if rec is None:
            return
        t0, target, kind, ts, actual = rec
        err = max((abs(a - b) for a, b in zip(actual, ts)), default=0.0) * 1e6
        log_row("moves", ["t_wall", "target_steps", "kind", "n_steps",
                          "last_pulse_planned_s", "max_step_error_us"],
                [LOG.wall(t0) if LOG else t0, target, kind, len(ts),
                 round(ts[-1], 4) if ts else 0, round(err)])


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
        self.offset = 0.0           # strike aim offset (capture-window test)

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
        self.k_off = 0               # next strike offset
        self.plastic_seen = False    # legacy mode: plastic in the previous cycle

    # ---- belt speed from track motion ------------------------------------
    def _update_speed(self, tr):
        """Belt speed from the first and last detection of a track.  v2 only
        accepted samples within +/-30 % of the nominal BELT_SPEED, so the
        estimate could not follow a slower or faster belt; here a sample is
        accepted within 0.4-2.5x of the current estimate (and 20-1000 mm/s), and
        the median of the last 7 samples is used."""
        if len(tr.dets) >= 2:
            (t1, x1, *_), (t2, x2, *_) = tr.dets[0], tr.dets[-1]
            if t2 - t1 > 0.2:
                v = (x2 - x1) / (t2 - t1)
                if 20.0 <= v <= 1000.0 and 0.4 * self.v_hat <= v <= 2.5 * self.v_hat:
                    self.v_samples = (self.v_samples + [v])[-7:]
                    self.v_hat = float(np.median(self.v_samples))

    def predict_x(self, tr, t):
        tc, xc = tr.last()[0], tr.last()[1]
        return xc + self.v_hat * (t - tc)

    # ---- detections from one frame -----------------------------------------
    def on_frame(self, t_cap, dets):
        if LEGACY:
            self.on_frame_legacy(t_cap, dets)
            return
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
                    if tr.route == "PLASTIC":
                        tr.offset = STRIKE_OFFSETS[self.k_off % len(STRIKE_OFFSETS)]
                        self.k_off += 1
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
            aim += tr.offset
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
        p = tr.plan or {}
        log_row("decisions", DEC_HDR,
                [LOG.wall(time.monotonic()) if LOG else time.time(), "event", tr.id,
                 tr.best_route(), route, why, len(tr.dets),
                 LOG.wall(tr.dets[0][0]) if LOG else tr.dets[0][0],
                 LOG.wall(tr.last()[0]) if LOG else tr.last()[0],
                 round(tr.last()[1], 1), round(tr.y_mean(), 1), round(self.v_hat, 1),
                 p.get("kind", ""), LOG.wall(p["t0"]) if (LOG and "t0" in p) else "",
                 tr.offset, json.dumps({k: round(v, 2) for k, v in tr.votes.items()})])

    # ---- first-firmware trigger logic on the same gate (baseline) ------------
    def on_frame_legacy(self, t_cap, dets):
        """riverextract_lite.py logic: every cycle that sees an item registers it
        (best non-OTHER class by confidence, else OTHER); an organic cycle swings
        the gate to -45 deg, holds 0.25 s and returns; a plastic item is struck
        0.3 s after the first cycle that no longer sees plastic.  The gate moves
        with the same ramped profiles as v2, so only the trigger logic differs."""
        now = time.monotonic()
        with self.lock:
            bucket = None
            if dets:
                best = None
                for d in dets:
                    r = ROUTE[d[3]]
                    if r != "OTHER" and (best is None or d[4] > best[1]):
                        best = (r, d[4])
                bucket = best[0] if best else "OTHER"
            acts = []
            if bucket:
                register(bucket)
                acts.append(("register", bucket))
            if bucket == "ORGANIC":
                if now >= self.busy_until:
                    t0 = now + 0.005
                    self.gate.organic_move(t0)
                    t_r = t0 + T_ORGMOVE + LEGACY_HOLD_S
                    self.busy_until = self.gate.ret(t_r)
                    acts.append(("organic swing", "ORGANIC"))
                else:
                    acts.append(("organic swing skipped: gate busy", "ORGANIC"))
            if bucket == "PLASTIC":
                self.plastic_seen = True
            elif self.plastic_seen:
                self.plastic_seen = False
                t0 = now + LEGACY_DELAY_S
                if t0 >= self.busy_until:
                    self.busy_until = self.gate.strike(t0)
                    acts.append(("strike after falling edge", "PLASTIC"))
                else:
                    acts.append(("strike skipped: gate busy", "PLASTIC"))
            for why, route in acts:
                self.log = (self.log + [f"legacy: {why} ({route})"])[-6:]
                log_row("decisions", DEC_HDR,
                        [LOG.wall(now) if LOG else time.time(), "legacy", "", bucket or "",
                         route, why, len(dets), LOG.wall(t_cap) if LOG else t_cap, "",
                         "", "", round(self.v_hat, 1), "", "", "",
                         json.dumps([(d[3], round(d[4], 2)) for d in dets])])


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
        self.n_saved = 0
        self.k_frame = 0
        self.save_dir = os.path.join(LOG.dir, "frames") if (LOG and SAVE_EVERY) else None
        if self.save_dir:
            os.makedirs(self.save_dir, exist_ok=True)
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
            self.k_frame += 1
            if self.save_dir and self.k_frame % SAVE_EVERY == 0 and self.n_saved < SAVE_MAX:
                fname = f"f{self.k_frame:07d}.jpg"
                cv2.imwrite(os.path.join(self.save_dir, fname), fr,
                            [cv2.IMWRITE_JPEG_QUALITY, 92])
                self.n_saved += 1
                log_row("saved_frames", ["t_wall", "file"], [round(time.time(), 3), fname])
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
            log_row("frames", ["t_wall", "t_cap", "t_inf_s", "T_loop_s", "pace_s",
                               "n_dets", "dets(class:conf:x_mm:y_mm)"],
                    [LOG.wall(now) if LOG else now, LOG.wall(t_cap) if LOG else t_cap,
                     round(self.t_inf, 4), round(self.T_loop, 4), PACE_S, len(dets),
                     ";".join(f"{d[3]}:{d[4]:.2f}:{d[1]:.0f}:{d[2]:.0f}" for d in dets)])
            if PACE_S > 0:
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
.info{font-size:12px;color:#9fb3b6;line-height:1.6}.notif div{color:#ff9d7a}
.gt .h{font-size:13px;color:#2f9d8f;margin-bottom:8px}.gt .row{display:flex;flex-wrap:wrap;gap:6px;align-items:center;margin-bottom:8px}
.gt input{background:#0b1416;border:1px solid #1d3438;color:#e6eef0;border-radius:8px;padding:8px;font-size:14px;flex:1;min-width:60px}
.gt span{font-size:12px;color:#7c8a8d;width:80px}.gt b{border:1px solid #2f9d8f;border-radius:8px;padding:8px 10px;font-weight:400;font-size:13px;cursor:pointer}
.gt b.on{background:#2f9d8f;color:#0b1416}.gt button{width:100%;padding:12px;border-radius:10px;border:0;background:#2f9d8f;color:#0b1416;font-size:15px;font-weight:700}</style></head><body>
<div class='top'><img src='/logo/aust' onerror="this.style.display='none'"><img src='/logo/foe' onerror="this.style.display='none'"></div>
<h1>RiverExtract</h1><div class='sub'>Event-driven vision sorting</div><div class='wrap'>
<div class='panel'><img src='/video'></div>
<div class='bins'>
<div class='bin' id='bin-PLASTIC'><div class='n'>Plastic (side)</div><div class='c' id='c-PLASTIC'>0</div><div class='s' id='s-PLASTIC'>ok</div></div>
<div class='bin' id='bin-ORGANIC'><div class='n'>Organic (rear-right)</div><div class='c' id='c-ORGANIC'>0</div><div class='s' id='s-ORGANIC'>ok</div></div>
<div class='bin' id='bin-OTHER'><div class='n'>Other (rear-left)</div><div class='c' id='c-OTHER'>0</div><div class='s' id='s-OTHER'>ok</div></div></div>
<div class='panel info' id='info'></div><div class='panel notif' id='notif'></div>
<div class='panel gt'><div class='h'>Ground truth: log each pass after the item lands</div>
<div class='row'><input id='item' placeholder='item id (P07)'><input id='nitems' value='1' title='items in this pass'><input id='note' placeholder='note'></div>
<div class='row' id='tc'><span>true class</span><b data-v='PLASTIC'>plastic</b><b data-v='ORGANIC'>organic</b><b data-v='OTHER'>other</b></div>
<div class='row' id='bn'><span>bin reached</span><b data-v='SIDE'>side</b><b data-v='REAR_RIGHT'>rear-right</b><b data-v='REAR_LEFT'>rear-left</b><b data-v='ELSEWHERE'>elsewhere</b><b data-v='STUCK'>stuck</b></div>
<button id='go'>Log pass</button><div id='tlog' class='info'></div></div></div>
<script>async function poll(){try{let s=await(await fetch('/api/status')).json();
for(const k of ['PLASTIC','ORGANIC','OTHER']){document.getElementById('c-'+k).textContent=s.bins[k].count;
let b=document.getElementById('bin-'+k),t=document.getElementById('s-'+k);if(s.bins[k].full){b.classList.add('full');t.textContent='FULL'}else{b.classList.remove('full');t.textContent='ok'}}
document.getElementById('info').innerHTML='mode '+s.mode+' &middot; pace '+s.pace_s+' s &middot; CPU '+(s.cpu_c==null?'-':Math.round(s.cpu_c))+' &deg;C<br>inference '+s.t_inf_ms+' ms &middot; loop period T '+s.T_ms+' ms &middot; belt speed '+s.v_hat+' mm/s<br>'+(s.log||[]).join('<br>');
document.getElementById('notif').innerHTML=(s.notifications||[]).map(m=>'<div>&#9888; '+m+'</div>').join('')||'<span style=color:#4a5a5d>No alerts</span>';}catch(e){}}
setInterval(poll,700);poll();
let sel={tc:null,bn:null};for(const g of ['tc','bn']){document.querySelectorAll('#'+g+' b').forEach(e=>e.onclick=()=>{
document.querySelectorAll('#'+g+' b').forEach(x=>x.classList.remove('on'));e.classList.add('on');sel[g]=e.dataset.v;});}
document.getElementById('go').onclick=async()=>{if(!sel.tc||!sel.bn){alert('pick true class and bin');return}
let r=await(await fetch('/api/truth',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({item:document.getElementById('item').value,
true_class:sel.tc,bin:sel.bn,n_items:document.getElementById('nitems').value,note:document.getElementById('note').value})})).json();
document.getElementById('tlog').innerHTML=r.last.map(x=>'#'+x.join(' &middot; ')).join('<br>');
document.querySelectorAll('#bn b').forEach(x=>x.classList.remove('on'));sel.bn=null;document.getElementById('note').value='';};</script></body></html>"""


def run_server(vision, ctrl):
    from flask import Flask, Response, jsonify, request, send_file
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
                       v_hat=int(ctrl.v_hat), mode="legacy" if LEGACY else "v2",
                       pace_s=PACE_S, cpu_c=cpu_temp(), log_dir=LOG.dir if LOG else "")

    truth = []

    @app.route("/api/truth", methods=["POST"])
    def add_truth():
        d = request.get_json(force=True) or {}
        n = len(truth) + 1
        row = [n, str(d.get("item", "")).strip(), d.get("true_class", ""), d.get("bin", ""),
               str(d.get("n_items", "1")).strip(), str(d.get("note", "")).strip()]
        truth.append(row)
        log_row("truth", ["t_wall", "pass", "item_id", "true_class", "bin_reached",
                          "n_items", "note"], [round(time.time(), 3)] + row)
        return jsonify(ok=True, last=truth[-5:][::-1])

    @app.route("/")
    def index():
        return DASH

    print("RiverExtract sorting v2.1 on :5000", flush=True)
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


def strike_test(n, tau, dry, log_q):
    """n strike sequences (strike, dwell, return) without items.  Mark the rest
    pose before the test; if the gate is not back on the mark afterwards, the
    motor lost steps at this torque budget."""
    gate = Gate(dry=dry, log_q=log_q)
    t_s = move_duration(50, "tri", tau)
    print(f"strike test: {n} strikes, tau = {tau:.3f} N m -> 90 deg in {t_s:.3f} s, "
          f"peak {2 * math.pi / 2 / t_s:.1f} rad/s", flush=True)
    t = time.monotonic() + 1.0
    for i in range(n):
        end = gate.strike(t, tau)
        while time.monotonic() < end + 0.05:
            time.sleep(0.01)
        print(f"  strike {i + 1}/{n} done", flush=True)
        t = end + 0.5
    time.sleep(0.5)
    print("Check the gate against the rest mark now. On the mark: no lost steps.", flush=True)
    log_row("strike_test", ["t_wall", "n_strikes", "tau_Nm", "t_strike_s"],
            [round(time.time(), 3), n, tau, round(t_s, 4)])
    gate.close()


def system_loop(vision, ctrl):
    while True:
        log_row("system", ["t_wall", "cpu_temp_c", "throttled", "v_hat_mm_s", "t_inf_s",
                           "T_loop_s", "pace_s", "mode"],
                [round(time.time(), 3), cpu_temp(), throttled(), round(ctrl.v_hat, 1),
                 round(vision.t_inf, 4), round(vision.T_loop, 4), PACE_S,
                 "legacy" if LEGACY else "v2"])
        time.sleep(5.0)


# =============================================================================
if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--jog", action="store_true")
    ap.add_argument("--test", action="store_true")
    ap.add_argument("--dry", action="store_true", help="no GPIO (desk test)")
    ap.add_argument("--config-from", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "riverextract_sort_v2.py"),
        help="file whose calibrated CONFIG values are used (default: v2 next to this)")
    ap.add_argument("--pace", type=float, default=None, help="pacing sleep per cycle, s")
    ap.add_argument("--legacy", action="store_true", help="first-firmware trigger logic")
    ap.add_argument("--offsets", default=None, help="strike aim offsets, mm, e.g. -40,0,40")
    ap.add_argument("--strike-test", type=int, default=0, metavar="N")
    ap.add_argument("--tau", type=float, default=None, help="strike torque budget, N m")
    ap.add_argument("--save-frames", type=int, default=0, metavar="K")
    ap.add_argument("--belt-speed", type=float, default=None, help="initial estimate, mm/s")
    ap.add_argument("--model", default=None,
                    help="detector weights (.pt); classes plastic/organic/other are routed")
    ap.add_argument("--conf", type=float, default=None, help="detector confidence threshold")
    ap.add_argument("--logdir", default=LOG_ROOT)
    ap.add_argument("--no-log", action="store_true")
    a = ap.parse_args()
    if a.jog:
        jog()
        sys.exit(0)
    calib = import_calibration(a.config_from)
    if a.model:
        if not os.path.exists(a.model):
            sys.exit(f"--model {a.model}: file not found")
        YOLO_MODEL = a.model
        # river-debris detector trained on the prototype's own frames
        ROUTE.update({"plastic": "PLASTIC", "organic": "ORGANIC", "other": "OTHER"})
    if a.conf is not None:
        CONF_THRESH = a.conf
    if a.pace is not None:
        PACE_S = max(0.0, a.pace)
    if a.belt_speed is not None:
        BELT_SPEED = a.belt_speed
    if a.tau is not None:
        TAU_STRIKE = a.tau
        T_STRIKE = move_duration(50, "tri", TAU_STRIKE)
    LEGACY = a.legacy
    if a.offsets:
        STRIKE_OFFSETS = [float(x) for x in a.offsets.split(",") if x.strip()]
    SAVE_EVERY = max(0, a.save_frames)
    if a.legacy and a.offsets:
        sys.exit("--offsets applies to the v2 controller, not to --legacy")
    if not a.no_log:
        LOG = ExpLog(a.logdir, dict(argv=sys.argv, calibration=calib, pace_s=PACE_S,
                                    legacy=LEGACY, offsets=STRIKE_OFFSETS,
                                    belt_speed=BELT_SPEED, tau_strike=TAU_STRIKE,
                                    tau_org=TAU_ORG, conf=CONF_THRESH, imgsz=YOLO_IMGSZ,
                                    model=YOLO_MODEL, bin_full=BIN_FULL_COUNT))
    log_q = mp.Queue() if LOG else None
    if log_q is not None:
        threading.Thread(target=drain_moves, args=(log_q,), daemon=True).start()
    if a.test:
        dry_test(a.dry)
        sys.exit(0)
    if a.strike_test:
        strike_test(a.strike_test, TAU_STRIKE, a.dry, log_q)
        time.sleep(0.5)
        sys.exit(0)
    threading.Thread(target=bin_reset_loop, daemon=True).start()
    gate = Gate(dry=a.dry, log_q=log_q)
    ctrl = Controller(gate)
    threading.Thread(target=ctrl.dispatch_loop, daemon=True).start()
    vision = Vision(ctrl)
    threading.Thread(target=system_loop, args=(vision, ctrl), daemon=True).start()
    try:
        run_server(vision, ctrl)
    finally:
        gate.close()
