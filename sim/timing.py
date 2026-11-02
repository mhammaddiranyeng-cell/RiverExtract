"""
Vision-sampling timing model and controller Monte Carlo.

Geometry along the belt (x, mm, origin at the gate pivot): the camera's
effective detection window spans centroid positions [-g-Lv, -g].  The
detection loop samples every T seconds (T = inference latency tau_i + the
0.6 s pacing sleep); each sample uses the newest camera frame, whose age is
uniform on [0, 1/F_CAM).  An item in the window is detected in a sample with
probability 1 - p_m.  Results become available tau_i after the sample.

Controllers
  baseline  (as-built logic, riverextract_lite.py):
     ORGANIC: one swing + one registration per detection cycle
     PLASTIC: one strike + one registration per falling edge, issued
              t_d after the edge result (t_d = 0.3 s as built)
  proposed  (event-driven tracking):
     a track starts at the first detection, coasts through misses and is
     committed at its m-th detection; the action time is computed from the
     time-stamped centroid and the belt speed; one registration per track.
Both controllers drive the same gate with the same motion profiles, so the
comparison isolates the trigger logic.
"""
import math
import numpy as np
from params import *
from kin import (Segments, simulate, J_GATE, seq_strike, seq_organic)
from dynamics import tri_time, trap_time

T_STRIKE = tri_time(math.pi / 2, J_GATE, TAU_STRIKE)
T_ORGMOVE = tri_time(math.pi / 2, J_GATE, TAU_ORG)
T_RET = trap_time(math.pi / 2, J_GATE, TAU_RET, OMEGA_RET)
A_MAX = 1.0 / F_CAM


# ---------------------------------------------------------------------------
# aim points from the kinematic model
# ---------------------------------------------------------------------------
def capture_windows(v, w=65.0, ny=6, dx=2.0, mu=0.5):
    """Centroid position at action start that yields the intended route.
    Returns dict with strike window [lo, hi] and organic latest start."""
    ymax = BAND - w / 2
    ys = np.linspace(-ymax, ymax, ny)
    x0 = np.arange(-420, 60, dx)
    n = x0.size
    res = dict(y=ys, s_lo=[], s_hi=[], o_max=[])
    h = w / 2 + PLATE_T / 2
    for y0v in ys:
        y0 = np.full(n, y0v)
        seg = Segments(n, 3)
        seq_strike(seg, np.arange(n), 0.0)
        o = simulate(seg, x0, y0, w, v, mu=mu)['out']
        P = np.nonzero(o == 'P')[0]
        # longest contiguous run of 'P'
        if P.size:
            br = np.split(P, np.nonzero(np.diff(P) > 1)[0] + 1)
            best = max(br, key=len)
            res['s_lo'].append(x0[best[0]])
            res['s_hi'].append(x0[best[-1]])
        else:
            res['s_lo'].append(np.nan)
            res['s_hi'].append(np.nan)
        # organic: hold until predicted contact + 0.1 s (exact here)
        xc = -y0v - h * math.sqrt(2)
        seg = Segments(n, 3)
        t_ret = np.maximum(T_ORGMOVE, (xc - x0) / v + 0.10)
        seq_organic(seg, np.arange(n), 0.0, t_ret)
        o = simulate(seg, x0, y0, w, v, mu=mu)['out']
        ok = o == 'O'
        # latest start for which every earlier start also succeeds
        bad = np.nonzero(~ok)[0]
        res['o_max'].append(x0[bad[0] - 1] if bad.size and bad[0] > 0 else np.nan)
    for k in ('s_lo', 's_hi', 'o_max'):
        res[k] = np.array(res[k])
    return res


# ---------------------------------------------------------------------------
# sampling
# ---------------------------------------------------------------------------
def sample_items(n, v, T, Lv, g, pm, rng):
    """Sample instants, capture instants and detections for n items.
    Time origin: the item's centroid enters the window (x = -g-Lv)."""
    xw0 = -g - Lv
    tau_in = Lv / v                               # dwell in the window
    J = int(math.ceil(tau_in / T)) + 2
    phi = rng.uniform(0, T, n)
    tj = phi[:, None] + T * np.arange(-1, J)[None, :]    # sample instants
    age = rng.uniform(0, A_MAX, tj.shape)
    tc = tj - age                                        # capture instants
    inwin = (tc >= 0) & (tc <= tau_in)
    det = inwin & (rng.uniform(size=tj.shape) >= pm)
    return dict(tj=tj, tc=tc, det=det, inwin=inwin, xw0=xw0)


def nth_detection(det, m):
    """Index of the m-th detection per item (-1 if fewer)."""
    c = np.cumsum(det, axis=1)
    hit = c >= m
    idx = np.where(hit.any(axis=1), hit.argmax(axis=1), -1)
    return idx


# ---------------------------------------------------------------------------
# controllers
# ---------------------------------------------------------------------------
def proposed(route, n, v, T, Lv, g, pm, rng, w=65.0, m=1, sig_x=5.0,
             eps_v=0.05, margin=40.0, cw=None, mu=0.5, tau_i=None, with_decided=False,
             replan=True):
    """Simulate the proposed controller for n items of one route class.
    Returns outcome codes and per-item registration counts.
    tau_i: inference latency (default T - T_PACE, the paper's v1 model).
    replan=False: every action is timed once from the committing detection
    (fixed delay from the first detection; with eps_v=0 this is the baseline
    with an encoder-measured belt speed)."""
    if tau_i is None:
        tau_i = T - T_PACE
    S = sample_items(n, v, T, Lv, g, pm, rng)
    k = nth_detection(S['det'], m)
    decided = k >= 0
    out = np.full(n, 'X', dtype='<U1')          # undecided -> rest pose
    regs = decided.astype(int)                  # one registration per track
    if route == 'OTHER' or not decided.any():
        return out, regs, decided
    ii = np.nonzero(decided)[0]
    xw0 = S['xw0']
    nd = ii.size
    y0 = rng.uniform(-(BAND - w / 2), BAND - w / 2, nd)
    # per-detection measurements (all samples; only detected ones are used)
    tjA = S['tj'][ii]
    tcA = S['tc'][ii]
    detA = S['det'][ii]
    xA = xw0 + v * tcA + rng.normal(0, sig_x, tcA.shape)
    yA = y0[:, None] + rng.normal(0, sig_x, tcA.shape)
    thA = tcA + rng.uniform(0, A_MAX, tcA.shape) - A_MAX / 2       # stamp jitter
    vh = v * (1 + rng.uniform(-eps_v, eps_v, nd))                  # belt speed
    resA = tjA + tau_i + 0.005                                      # result instant
    kk = k[ii]
    t_dec = resA[np.arange(nd), kk]
    if cw is None:
        cw = capture_windows(v, w)

    def latest_before(t_lim):
        """Index of the latest detection whose result is available before t_lim
        (never earlier than the committing detection)."""
        ok = detA & (resA <= t_lim[:, None])
        idx = np.where(ok.any(axis=1), ok.shape[1] - 1 - np.argmax(ok[:, ::-1], axis=1), kk)
        return np.maximum(idx, kk)

    def predict(idx):
        r = np.arange(nd)
        # lateral position: running mean of the detections up to idx
        m = detA & (np.arange(detA.shape[1])[None, :] <= idx[:, None])
        yh = (yA * m).sum(axis=1) / np.maximum(m.sum(axis=1), 1)
        return xA[r, idx], yh, thA[r, idx]

    if route == 'PLASTIC':
        xh, yh, th = predict(kk)
        aim = np.interp(yh, cw['y'], 0.5 * (cw['s_lo'] + cw['s_hi']))
        t_act = th + (aim - xh) / vh
        # re-plan once with the newest detection available 10 ms before the stroke
        idx = latest_before(t_act - 0.010) if replan else kk
        xh, yh, th = predict(idx)
        aim = np.interp(yh, cw['y'], 0.5 * (cw['s_lo'] + cw['s_hi']))
        hi = np.interp(yh, cw['y'], cw['s_hi'])
        t_act = th + (aim - xh) / vh
        x_pred_dec = xh + vh * (t_dec - th)
        late = t_act < t_dec
        feas = (~late | (x_pred_dec <= hi - 10.0)) & np.isfinite(t_act) & np.isfinite(hi)
        t_act = np.where(late, t_dec, t_act)
        jj = ii[feas]
        x0 = xw0 + v * t_act[feas]
        seg = Segments(jj.size, 3)
        seq_strike(seg, np.arange(jj.size), 0.0)
        r = simulate(seg, x0, y0[feas], w, v, mu=mu)
        out[jj] = np.where(r['out'] == 'N', 'X', r['out'])
        return out, regs, decided
    if route == 'ORGANIC':
        h = w / 2 + PLATE_T / 2
        xh, yh, th = predict(kk)
        omax = np.interp(yh, cw['y'], cw['o_max'])
        t_act = th + (omax - margin - xh) / vh
        idx = latest_before(t_act - 0.010) if replan else kk
        xh, yh, th = predict(idx)
        omax = np.interp(yh, cw['y'], cw['o_max'])
        t_act = th + (omax - margin - xh) / vh
        x_pred_dec = xh + vh * (t_dec - th)
        late = t_act < t_dec
        feas = (~late | (x_pred_dec <= omax - 5.0)) & np.isfinite(t_act) & np.isfinite(omax)
        t_act = np.where(late, t_dec, t_act)
        # release instant re-planned from the newest detection of the item
        idx2 = latest_before(np.full(nd, np.inf)) if replan else kk
        xh2, yh2, th2 = predict(idx2)
        xc_pred = -yh2 - h * math.sqrt(2)
        t_ret = th2 + (xc_pred - xh2) / vh + 0.10
        jj = ii[feas]
        x0 = xw0 + v * t_act[feas]
        seg = Segments(jj.size, 3)
        seq_organic(seg, np.arange(jj.size), 0.0,
                    np.maximum(t_ret[feas] - t_act[feas], T_ORGMOVE))
        r = simulate(seg, x0, y0[feas], w, v, mu=mu)
        out[jj] = np.where(r['out'] == 'N', 'X', r['out'])
        return out, regs, decided
    raise ValueError(route)


def baseline(route, n, v, T, Lv, g, pm, rng, w=65.0, t_d=PLASTIC_SWING_DELAY,
             mu=0.5, kinematics=True, tau_i=None):
    """As-built trigger logic on the same gate and profiles."""
    if tau_i is None:
        tau_i = T - T_PACE
    S = sample_items(n, v, T, Lv, g, pm, rng)
    det, tj = S['det'], S['tj']
    xw0 = S['xw0']
    out = np.full(n, 'X', dtype='<U1')
    y0 = rng.uniform(-(BAND - w / 2), BAND - w / 2, n)
    if route == 'OTHER':
        return out, det.sum(axis=1), det.any(axis=1)
    if route == 'ORGANIC':
        regs = det.sum(axis=1)               # one per detection cycle
        req = np.where(det, tj + tau_i, np.inf)
        dur = T_ORGMOVE + HOLD_ASBUILT + T_RET
    else:  # PLASTIC: falling edges
        prev = np.concatenate([np.zeros((n, 1), bool), det[:, :-1]], axis=1)
        edge = prev & ~det
        regs = edge.sum(axis=1)
        req = np.where(edge, tj + tau_i + t_d, np.inf)
        dur = T_STRIKE + DWELL + T_RET
    if not kinematics:
        return out, regs, det.any(axis=1)
    # serialise on the gate lock: start_k = max(request_k, end_{k-1})
    req = np.sort(req, axis=1)
    start = np.full(req.shape, np.inf)
    last_end = np.full(n, -np.inf)
    for c in range(req.shape[1]):
        r = req[:, c]
        s = np.where(np.isfinite(r), np.maximum(r, last_end), np.inf)
        start[:, c] = s
        last_end = np.where(np.isfinite(s), s + dur, last_end)
    # keep only actions that can touch the item: the item must not have
    # passed the gate before the action, and must reach the swept sector
    # before the action ends
    x_s = xw0 + v * start                 # item centroid at action start
    x_e = xw0 + v * (start + dur)         # ... at action end
    reach = R_SWING + w / 2 + 5.0
    relevant = np.isfinite(start) & (x_e >= -reach) & (x_s <= reach)
    has = relevant.any(axis=1)
    ii = np.nonzero(has)[0]
    if ii.size == 0:
        return out, regs, det.any(axis=1)
    first = np.where(relevant[ii].any(axis=1), relevant[ii].argmax(axis=1), 0)
    t_origin = start[ii, first]
    nact = relevant[ii].sum(axis=1)
    seg = Segments(ii.size, 3 * int(nact.max()) + 1)
    for c in range(start.shape[1]):
        sel = relevant[ii, c]
        if not sel.any():
            continue
        loc = np.nonzero(sel)[0]
        t0 = start[ii[loc], c] - t_origin[loc]
        if route == 'ORGANIC':
            t1 = seg.add(loc, t0, 'tri', TH_REST, TH_ORG - TH_REST, tau=TAU_ORG)
            seg.add(loc, t1 + HOLD_ASBUILT, 'trap', TH_ORG, TH_REST - TH_ORG,
                    tau=TAU_RET, wmax=OMEGA_RET)
        else:
            t1 = seg.add(loc, t0, 'tri', TH_REST, TH_ORG - TH_REST, tau=TAU_STRIKE)
            seg.add(loc, t1 + DWELL, 'trap', TH_ORG, TH_REST - TH_ORG,
                    tau=TAU_RET, wmax=OMEGA_RET)
    x0 = xw0 + v * t_origin
    r = simulate(seg, x0, y0[ii], w, v, mu=mu)
    out[ii] = np.where(r['out'] == 'N', 'X', r['out'])
    return out, regs, det.any(axis=1)


# ---------------------------------------------------------------------------
# closed forms
# ---------------------------------------------------------------------------
def p_det_closed(v, T, Lv, pm=0.0):
    """Probability of at least one detection (uniform sampling phase)."""
    tau = Lv / v
    N0 = np.floor(tau / T)
    frac = tau / T - N0            # P(N = N0 + 1) = frac, P(N = N0) = 1-frac
    return 1 - ((1 - frac) * pm ** N0 + frac * pm ** (N0 + 1))


def regs_level_closed(v, T, Lv, pm=0.0):
    """Expected registrations per item, level-triggered (one per detection)."""
    return (1 - pm) * Lv / (v * T)


def regs_edge_closed(v, T, Lv, pm=0.0):
    """Expected falling edges per item (edge-triggered, no debounce)."""
    tau = Lv / v
    N0 = np.floor(tau / T)
    frac = tau / T - N0
    q = 1 - pm
    e = lambda N: np.where(N > 0, q * (1 + (N - 1) * pm), 0.0)
    return (1 - frac) * e(N0) + frac * e(N0 + 1)
