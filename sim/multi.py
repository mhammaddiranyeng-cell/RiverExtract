"""
Several items on the belt at once (v3, Dr. Saide point 5).

Each stream is a sequence of items (discs, w = 65 mm) at given belt
positions.  The camera samples every T s; one frame per sample shows every
item that is in the window, each detected with probability 1 - p_m and
optionally misclassified.  The event-driven controller of Section IV-C runs
on the detections:

  tracker   a detection updates the active track whose prediction lies
            within the gating distance G (greedy, nearest first) or opens a
            new track; a track is committed at its m-th detection, which
            fixes its class
  planner   strike / organic move timed from the newest usable detection,
            exactly as in timing.proposed()
  gate      one action at a time; an action waits until the gate is back at
            rest and is dropped if it can no longer meet its deadline;
            consecutive organic items extend the hold (paper, Sec. IV-C)

The planar contact model (kin.py) then runs every item against the whole
gate trajectory, so a strike or hold issued for one item acts on every
other item that is near the gate at that moment.

Outcomes per item: correct bin, reject (rear-left / OTHER bin for a plastic
or organic item), wrong (side or rear-right bin of another class), lost
(post or right edge).
"""
import math
import numpy as np
from params import *
from kin import Segments, simulate, J_GATE
from dynamics import tri_time, trap_time

T_STRIKE = tri_time(math.pi / 2, J_GATE, TAU_STRIKE)
T_ORGMOVE = tri_time(math.pi / 2, J_GATE, TAU_ORG)
T_RET = trap_time(math.pi / 2, J_GATE, TAU_RET, OMEGA_RET)
A_MAX = 1.0 / F_CAM
CLS = np.array(['P', 'O', 'X'])        # plastic, organic, other
H_ORG = 65.0 / 2 + PLATE_T / 2
X_START = -160.0                       # contact model starts each item here (gate cannot reach it)
ZONE = R_SWING + 65.0 / 2 + 10.0       # |x| within which the gate can act on an item (mm)


def cw_interp(cw, y):
    c = 0.5 * (cw['s_lo'] + cw['s_hi'])
    return (np.interp(y, cw['y'], c), np.interp(y, cw['y'], cw['s_hi']),
            np.interp(y, cw['y'], cw['o_max']))


def run_stream(xs, ys, cls, v, rng, cw, T=0.4, tau_i=0.4, Lv=300.0, g=200.0, pm=0.0,
               sig=5.0, eps=0.05, G=50.0, m=1, margin=40.0, conf=None, policy='fcfs'):
    """Controller + scheduler for one stream.
    xs: item centroid positions at t = 0 (mm, upstream negative); ys: lateral;
    cls: true class index (0 P, 1 O, 2 X); conf: 3x3 row-stochastic matrix
    P(reported class | true class) applied to every detection.
    policy: 'fcfs'  -- one action at a time, first come first served (paper)
            'safe'  -- additionally drop an action if another committed track
                       of a different class is predicted inside the gate's
                       reach while the gate is away from rest
    Returns (gate sequences, per-item diagnostics)."""
    n = len(xs)
    vh = v * (1 + rng.uniform(-eps, eps))          # one belt-speed estimate per stream
    x_in, x_out = -g - Lv, -g
    t_first = (x_in - xs.max()) / v - T
    t_last = (x_out - xs.min()) / v + T
    phi = rng.uniform(0, T)
    tj = np.arange(t_first + phi, t_last, T)
    tracks = []                                     # dict per track
    det_track = [[] for _ in range(n)]              # track ids each item's detections went to
    for t in tj:
        age = rng.uniform(0, A_MAX)
        tc = t - age
        th = tc + rng.uniform(-A_MAX / 2, A_MAX / 2)   # time stamp of the frame
        xt = xs + v * tc
        inw = (xt >= x_in) & (xt <= x_out)
        det = inw & (rng.uniform(size=n) >= pm)
        idx = np.nonzero(det)[0]
        if idx.size == 0:
            continue
        res_t = t + tau_i + 0.005
        xm = xt[idx] + rng.normal(0, sig, idx.size)
        ym = ys[idx] + rng.normal(0, sig, idx.size)
        if conf is None:
            cm = cls[idx]
        else:
            cm = np.array([rng.choice(3, p=conf[c]) for c in cls[idx]])
        # association with tracks that are still near the window
        act = [k for k, tr in enumerate(tracks)
               if tr['x'][-1] + vh * (th - tr['t'][-1]) <= x_out + 3 * G]
        pairs = []
        for a, i in enumerate(idx):
            for k in act:
                tr = tracks[k]
                xp = tr['x'][-1] + vh * (th - tr['t'][-1])
                yp = float(np.mean(tr['y']))
                d = math.hypot(xm[a] - xp, ym[a] - yp)
                if d <= G:
                    pairs.append((d, a, k))
        pairs.sort()
        used_a, used_k = set(), set()
        assign = {}
        for d, a, k in pairs:
            if a in used_a or k in used_k:
                continue
            used_a.add(a)
            used_k.add(k)
            assign[a] = k
        for a, i in enumerate(idx):
            if a in assign:
                k = assign[a]
            else:
                k = len(tracks)
                tracks.append(dict(t=[], x=[], y=[], r=[], c=[], item=[]))
            tr = tracks[k]
            tr['t'].append(th)
            tr['x'].append(xm[a])
            tr['y'].append(ym[a])
            tr['r'].append(res_t)
            tr['c'].append(int(cm[a]))
            tr['item'].append(int(i))
            det_track[i].append(k)
    # ------------------------------------------------------------ planning
    reqs = []
    for k, tr in enumerate(tracks):
        if len(tr['t']) < m:
            continue
        tr['cls'] = tr['c'][m - 1]
        t_dec = tr['r'][m - 1]
        tr['committed'] = True
        if tr['cls'] == 2:
            continue
        r = np.array(tr['r'])

        def pred(j):
            return tr['x'][j], float(np.mean(tr['y'][:j + 1])), tr['t'][j]

        def newest_before(t_lim):
            ok = np.nonzero(r <= t_lim)[0]
            return max(int(ok[-1]) if ok.size else m - 1, m - 1)

        if tr['cls'] == 0:
            xh, yh, th = pred(m - 1)
            aim, hi, _ = cw_interp(cw, yh)
            t_act = th + (aim - xh) / vh
            xh, yh, th = pred(newest_before(t_act - 0.010))
            aim, hi, _ = cw_interp(cw, yh)
            t_act = th + (aim - xh) / vh
            t_dl = th + (hi - 10.0 - xh) / vh
            if not (np.isfinite(t_act) and np.isfinite(t_dl)):
                continue
            reqs.append(dict(kind='S', t_req=max(t_act, t_dec), t_dl=t_dl, trk=k))
        else:
            xh, yh, th = pred(m - 1)
            _, _, om = cw_interp(cw, yh)
            t_act = th + (om - margin - xh) / vh
            xh, yh, th = pred(newest_before(t_act - 0.010))
            _, _, om = cw_interp(cw, yh)
            t_act = th + (om - margin - xh) / vh
            t_dl = th + (om - 5.0 - xh) / vh
            x2, y2, t2 = pred(len(tr['t']) - 1)
            xc = -y2 - H_ORG * math.sqrt(2)
            t_ret = t2 + (xc - x2) / vh + 0.10
            if not (np.isfinite(t_act) and np.isfinite(t_dl)):
                continue
            reqs.append(dict(kind='O', t_req=max(t_act, t_dec), t_dl=t_dl, t_ret=t_ret, trk=k))
    # predicted exposure of every committed track to the gate's reach
    expo = {}
    for k, tr in enumerate(tracks):
        if tr.get('committed'):
            xl, tl = tr['x'][-1], tr['t'][-1]
            expo[k] = (tr['cls'], tl + (-ZONE - xl) / vh, tl + (ZONE - xl) / vh)

    def conflict(k, kind, t0, t1):
        ck = 0 if kind == 'S' else 1
        for j, (cj, a, b) in expo.items():
            if j != k and cj != ck and a < t1 and b > t0:
                return True
        return False

    # ----------------------------------------------------------- scheduling
    reqs.sort(key=lambda q: q['t_req'])
    seqs = []                   # dict(kind, t0, t_ret, t_end)
    free_at = -np.inf
    n_drop = n_merge = n_conf = 0
    for q in reqs:
        last = seqs[-1] if seqs else None
        if (q['kind'] == 'O' and last is not None and last['kind'] == 'O'
                and q['t_req'] <= last['t_ret']):
            if policy == 'safe' and conflict(q['trk'], 'O', last['t0'],
                                             max(last['t_ret'], q['t_ret']) + T_RET):
                n_conf += 1
                continue
            last['t_ret'] = max(last['t_ret'], q['t_ret'])
            last['t_end'] = last['t_ret'] + T_RET
            free_at = last['t_end']
            n_merge += 1
            continue
        start = max(q['t_req'], free_at)
        if start > q['t_dl']:
            n_drop += 1
            continue
        if q['kind'] == 'S':
            t_ret = start + T_STRIKE + DWELL
        else:
            t_ret = max(q['t_ret'], start + T_ORGMOVE)
        if policy == 'safe' and conflict(q['trk'], q['kind'], start, t_ret + T_RET):
            n_conf += 1
            continue
        seqs.append(dict(kind=q['kind'], t0=start, t_ret=t_ret, t_end=t_ret + T_RET))
        free_at = t_ret + T_RET
    # ---------------------------------------------------------- diagnostics
    n_tracks_item = np.array([len(set(d)) for d in det_track])
    mixed = np.zeros(n, dtype=bool)          # item's detections share a track with another item
    for tr in tracks:
        its = set(tr['item'])
        if len(its) > 1:
            for i in its:
                mixed[i] = True
    regs = sum(1 for tr in tracks if tr.get('committed'))
    return seqs, dict(n_tracks=n_tracks_item, mixed=mixed, regs=regs, n_drop=n_drop,
                      n_merge=n_merge, n_conf=n_conf, n_req=len(reqs))


def contact(items, v, mu=0.5, w=65.0, batch=4000):
    """items: list of (x_start, y, seqs) per item.  Runs the planar model for
    every item against the gate sequences that can act on it."""
    out = np.empty(len(items), dtype='<U1')
    for b0 in range(0, len(items), batch):
        chunk = items[b0:b0 + batch]
        smax = 2 * max(1, max(len(s) for _, _, s in chunk))
        seg = Segments(len(chunk), smax)
        y0 = np.empty(len(chunk))
        for i, (xs0, y, seqs) in enumerate(chunk):
            y0[i] = y
            t0 = (X_START - xs0) / v                  # item at X_START
            for s in seqs:
                tau = TAU_STRIKE if s['kind'] == 'S' else TAU_ORG
                seg.add(np.array([i]), s['t0'] - t0, 'tri', TH_REST, TH_ORG - TH_REST, tau=tau)
                seg.add(np.array([i]), s['t_ret'] - t0, 'trap', TH_ORG, TH_REST - TH_ORG,
                        tau=TAU_RET, wmax=OMEGA_RET)
        r = simulate(seg, np.full(len(chunk), X_START), y0, w, float(v), mu=mu,
                     t_extra=0.6, t_stop_min=(2 * abs(X_START) + w) / v + 0.3)
        o = r['out']
        out[b0:b0 + len(chunk)] = np.where(o == 'N', 'X', o)
    return out


def relevant(seqs, xs0, v, w=65.0):
    """Gate sequences that can act on an item that starts at xs0 (t = 0)."""
    t0 = (X_START - xs0) / v
    t1 = (-X_START + w - xs0) / v
    return [s for s in seqs if s['t_end'] >= t0 and s['t0'] <= t1]


def outcomes(cls, out):
    """Four outcome classes per item."""
    tgt = CLS[cls]
    correct = out == tgt
    lost = (out == 'B') | (out == 'R')
    reject = (~correct) & (~lost) & (out == 'X')
    wrong = (~correct) & (~lost) & (~reject)
    return dict(correct=correct, reject=reject, wrong=wrong, lost=lost)
