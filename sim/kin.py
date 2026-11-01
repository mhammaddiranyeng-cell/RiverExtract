"""
Planar kinematic simulation of the centre-pivot gate and one item.

Top view, belt frame: x downstream, y to the left, origin at the pivot.
The gate is a 200 x 5 mm plate rotating about z.  An item is a disc of
diameter w that rides on the belt at speed v.  Contact rules:

  FREE    item moves with the belt, (v, 0)
  FACE    item rests against a plate face that is static or moving away;
          its x advances at v (driven by the trailing cleat) and y follows
          the face
  ATT     the plate face moves into the item: the item is carried at a fixed
          contact radius and released when |omega| starts to fall; release
          velocity = normal velocity of the plate at the contact radius
          (frictionless plate face, i.e. no tangential flick)
  DONE    released item slides on the belt with Coulomb friction mu; it
          leaves over the left edge (y = +135 mm) unless it meets the post,
          which occupies |x| <= 16 mm beyond the edge

Outcome codes: 'P' side bin (left edge, clear of the post), 'B' blocked by
the post, 'X' rear-left (OTHER), 'O' rear-right (ORGANIC), 'R' over the
right edge, 'N' not touched by this action sequence.
"""
import math
import numpy as np
from params import *
from dynamics import gate_inertia

J_GATE = gate_inertia()['J']
KIND = {'none': 0, 'tri': 1, 'trap': 2, 'const': 3}


class Segments:
    """Per-item chronological motion segments (N items x S segments)."""

    def __init__(self, n, smax):
        self.n, self.smax = n, smax
        self.ts = np.full((n, smax), np.inf)
        self.te = np.full((n, smax), np.inf)
        self.kind = np.zeros((n, smax), dtype=int)
        self.th0 = np.zeros((n, smax))
        self.dth = np.zeros((n, smax))
        self.tau = np.ones((n, smax))
        self.wmax = np.ones((n, smax))
        self.rate = np.ones((n, smax))
        self.cnt = np.zeros(n, dtype=int)

    def add(self, idx, t0, kind, th0, dth, tau=None, wmax=None, rate=None):
        """Append a segment to items idx (bool mask or index array)."""
        idx = np.atleast_1d(np.nonzero(idx)[0] if np.asarray(idx).dtype == bool
                            else idx)
        if idx.size == 0:
            return np.zeros(0)
        k = self.cnt[idx]
        assert np.all(k < self.smax)
        t0 = np.broadcast_to(np.asarray(t0, float), idx.shape)
        th0 = np.broadcast_to(np.asarray(th0, float), idx.shape)
        dth = np.broadcast_to(np.asarray(dth, float), idx.shape)
        D = np.abs(dth)
        if kind == 'tri':
            dur = 2 * np.sqrt(D * J_GATE / tau)
        elif kind == 'trap':
            al = tau / J_GATE
            ta = wmax / al
            dur = np.where(D <= wmax * ta, 2 * np.sqrt(D / al), D / wmax + ta)
        elif kind == 'const':
            dur = D / (rate * STEP)
        else:
            raise ValueError(kind)
        self.ts[idx, k] = t0
        self.te[idx, k] = t0 + dur
        self.kind[idx, k] = KIND[kind]
        self.th0[idx, k] = th0
        self.dth[idx, k] = dth
        if tau is not None:
            self.tau[idx, k] = tau
        if wmax is not None:
            self.wmax[idx, k] = wmax
        if rate is not None:
            self.rate[idx, k] = rate
        self.cnt[idx] += 1
        return t0 + dur

    def eval(self, t, th_rest):
        th = np.full(self.n, th_rest, dtype=float)
        om = np.zeros(self.n)
        for s in range(self.smax):
            kd = self.kind[:, s]
            if not np.any(kd):
                continue
            done = (kd > 0) & (t >= self.te[:, s])
            th = np.where(done, self.th0[:, s] + self.dth[:, s], th)
            act = (kd > 0) & (t >= self.ts[:, s]) & (t < self.te[:, s])
            if not np.any(act):
                continue
            tt = t - self.ts[act, s]
            D = np.abs(self.dth[act, s])
            sg = np.sign(self.dth[act, s])
            k = kd[act]
            al = self.tau[act, s] / J_GATE
            wm = self.wmax[act, s]
            rt = self.rate[act, s] * STEP
            # triangular
            tm_tri = 2 * np.sqrt(D / al)
            th_tri = np.where(tt < tm_tri / 2, 0.5 * al * tt ** 2,
                              D - 0.5 * al * np.clip(tm_tri - tt, 0, None) ** 2)
            om_tri = np.where(tt < tm_tri / 2, al * tt,
                              al * np.clip(tm_tri - tt, 0, None))
            # trapezoidal
            ta = wm / al
            tri_like = D <= wm * ta
            tc = np.clip(D / wm - ta, 0, None)
            tm_tr = 2 * ta + tc
            th_tr = np.where(tt < ta, 0.5 * al * tt ** 2,
                             np.where(tt < ta + tc, 0.5 * al * ta ** 2 + wm * (tt - ta),
                                      D - 0.5 * al * np.clip(tm_tr - tt, 0, None) ** 2))
            om_tr = np.where(tt < ta, al * tt,
                             np.where(tt < ta + tc, wm, al * np.clip(tm_tr - tt, 0, None)))
            th_tr = np.where(tri_like, th_tri, th_tr)
            om_tr = np.where(tri_like, om_tri, om_tr)
            # constant rate
            th_c = np.clip(rt * tt, 0, D)
            om_c = rt
            thv = np.where(k == 1, th_tri, np.where(k == 2, th_tr, th_c))
            omv = np.where(k == 1, om_tri, np.where(k == 2, om_tr, om_c))
            th[act] = self.th0[act, s] + sg * thv
            om[act] = sg * omv
        return th, om

    def t_end(self):
        te = np.where(np.isfinite(self.te), self.te, -np.inf)
        return te.max(axis=1)


FREE, FACE, ATT, DONE = 0, 1, 2, 3


def flight(cx, cy, vx, vy, v, w, mu):
    """Coulomb slide of a released item on a belt moving at v (+x).
    Returns outcome code and final lateral position."""
    out = np.empty(cx.shape, dtype='<U1')
    rx, ry = vx - v, vy                       # velocity relative to belt
    sr = np.hypot(rx, ry)
    a = mu * G_MM
    with np.errstate(divide='ignore', invalid='ignore'):
        ux, uy = rx / sr, ry / sr
        reach = sr ** 2 / (2 * a)                 # slide length (belt frame)
        yf = cy + uy * reach
    yf = np.where(sr < 1e-9, cy, yf)
    ux = np.nan_to_num(ux)
    uy = np.nan_to_num(uy)

    def cross_time(target):
        # solve uy*(sr t - a t^2/2) = target - cy  (first root)
        dy = target - cy
        with np.errstate(divide='ignore', invalid='ignore'):
            q = dy / uy
            disc = sr ** 2 - 2 * a * q
            t = (sr - np.sqrt(np.clip(disc, 0, None))) / a
        ok = (uy != 0) & (q >= 0) & (disc >= 0)
        return np.where(ok, t, np.nan)

    def x_at(t):
        return cx + v * t + ux * (sr * t - 0.5 * a * t ** 2)

    hw = w / 2
    # left side
    t_edge_l = cross_time(POST_INNER - hw)     # item edge reaches post plane
    t_out_l = cross_time(POST_INNER)           # centroid passes the edge
    blocked = np.isfinite(t_edge_l) & (np.abs(x_at(np.nan_to_num(t_edge_l))) <= POST_X / 2 + hw)
    left_out = np.isfinite(t_out_l) & ~blocked
    t_out_r = cross_time(-POST_INNER)
    right_out = np.isfinite(t_out_r)
    out[:] = np.where(yf >= 0, 'X', 'O')
    out = np.where(right_out, 'R', out)
    out = np.where(blocked, 'B', out)
    out = np.where(left_out, 'P', out)
    return out, yf


def rest_state(x0, y0, h, th=TH_REST):
    """Exact state at t = 0 of items that have been riding towards the gate
    held at the rest pose: free, sliding along the upstream face, or past
    the downstream tip."""
    st, sn, cs = np.sin(th), np.sin(th), np.cos(th)
    x = x0.copy()
    y = y0.copy()
    state = np.full(x0.shape, FREE)
    sig = np.ones(x0.shape)            # all approach the +n face at +45 deg
    touched = np.zeros(x0.shape, dtype=bool)
    xc = (y0 * cs - h) / st            # face contact abscissa
    sc = xc * cs + y0 * sn
    onface = (sc >= -R_HALF) & (sc <= R_HALF) & (x0 >= xc)
    s_now = (x0 + h * st) / cs
    y_face = s_now * sn + h * cs
    past = onface & (s_now > R_HALF)
    face = onface & ~past
    state[face] = FACE
    y[face] = y_face[face]
    touched[onface] = True
    y[past] = R_HALF * sn + h[past] * cs
    return x, y, state, sig, touched


def simulate(seg, x0, y0, w, v, mu=0.5, dt=4e-4, th_rest=TH_REST,
             t_extra=0.6, record=False, t_stop_min=0.0):
    """Run items (arrays x0, y0 = centroid at t = 0) against per-item
    segments `seg`.  Returns dict with outcome code per item."""
    n = np.asarray(x0).size
    if n == 0:
        e = np.zeros(0)
        return dict(out=np.zeros(0, dtype='<U1'), vrel=e, rc=e, t_contact=e, vx=e, vy=e,
                    x=e, y=e)
    x0 = np.asarray(x0, float)
    y0 = np.broadcast_to(np.asarray(y0, float), x0.shape).copy()
    w = np.broadcast_to(np.asarray(w, float), x0.shape).copy()
    h = w / 2 + PLATE_T / 2
    x, y, state, sig, touched = rest_state(x0, y0, h, th_rest)
    s_att = np.zeros(n)          # plate-frame coordinates of attached item
    d_att = np.zeros(n)
    s_cp = np.zeros(n)           # contact point on the plate centre line
    out = np.full(n, 'N', dtype='<U1')
    vrel = np.full(n, np.nan)
    vxo = np.full(n, np.nan)
    vyo = np.full(n, np.nan)
    rc = np.full(n, np.nan)
    t_contact = np.full(n, np.nan)
    t_stop = max(np.nanmax(np.where(np.isfinite(seg.t_end()), seg.t_end(), 0.0)) + t_extra,
                 t_stop_min)
    t = 0.0
    th_prev, om_prev = seg.eval(-dt, th_rest)
    hist = []
    while t <= t_stop:
        th, om = seg.eval(t, th_rest)
        c, s_ = np.cos(th), np.sin(th)
        ux, uy, nx, ny = c, s_, -s_, c
        # ---------------- ATT: carried by the plate ---------------------
        m = state == ATT
        if np.any(m):
            x[m] = s_att[m] * ux[m] + d_att[m] * nx[m]
            y[m] = s_att[m] * uy[m] + d_att[m] * ny[m]
            rel = m & ((np.abs(om) < np.abs(om_prev) - 1e-12) | (om == 0))
            if np.any(rel):
                px, py = s_cp[rel] * ux[rel], s_cp[rel] * uy[rel]
                cx_, cy_ = x[rel] - px, y[rel] - py
                nn = np.hypot(cx_, cy_)
                ncx, ncy = cx_ / nn, cy_ / nn                 # contact normal
                vpx = om_prev[rel] * s_cp[rel] * nx[rel]
                vpy = om_prev[rel] * s_cp[rel] * ny[rel]
                vn = vpx * ncx + vpy * ncy                    # normal speed
                vn = np.clip(vn, 0, None)
                o, _ = flight(x[rel], y[rel], vn * ncx, vn * ncy, v, w[rel], mu)
                out[rel] = o
                vrel[rel] = vn
                vxo[rel] = vn * ncx
                vyo[rel] = vn * ncy
                rc[rel] = np.abs(s_cp[rel])
                state[rel] = DONE
        # ---------------- FACE: sliding along a plate face --------------
        m = state == FACE
        if np.any(m):
            xn = x[m] + v * dt
            # plate point closest to the free-moving position
            sp = np.clip(xn * ux[m] + y[m] * uy[m], -R_HALF, R_HALF)
            dvx, dvy = xn - sp * ux[m], y[m] - sp * uy[m]
            dist = dvx * sig[m] * nx[m] + dvy * sig[m] * ny[m]
            vpn = om[m] * sp                             # plate normal speed
            pushing = (vpn * sig[m] > 1e-9) & (np.abs(sp) > HUB_D / 2)
            sep = (dist >= h[m]) & ~pushing
            # constrained motion along the face
            snew = (xn - sig[m] * h[m] * nx[m]) / np.where(np.abs(ux[m]) < 1e-6, 1e-6, ux[m])
            ynew = snew * uy[m] + sig[m] * h[m] * ny[m]
            off = snew > R_HALF                          # slid past the tip
            idx = np.nonzero(m)[0]
            # pushed by the moving face -> attach
            ia = idx[pushing]
            state[ia] = ATT
            s_cp[ia] = sp[pushing]
            s_att[ia] = x[ia] * ux[ia] + y[ia] * uy[ia]
            d_att[ia] = x[ia] * nx[ia] + y[ia] * ny[ia]
            # separation / leaving the tip -> free
            ifree = idx[(sep | off) & ~pushing]
            x[ifree] = xn[(sep | off) & ~pushing]
            state[ifree] = FREE
            # stay on face
            ist = idx[~sep & ~off & ~pushing]
            x[ist] = xn[~sep & ~off & ~pushing]
            y[ist] = ynew[~sep & ~off & ~pushing]
        # ---------------- FREE: riding the belt --------------------------
        m = state == FREE
        if np.any(m):
            x[m] += v * dt
            sp = np.clip(x[m] * ux[m] + y[m] * uy[m], -R_HALF, R_HALF)
            dvx, dvy = x[m] - sp * ux[m], y[m] - sp * uy[m]
            d = np.hypot(dvx, dvy)
            hit = d <= h[m]
            if np.any(hit):
                idx = np.nonzero(m)[0][hit]
                sp, dvx, dvy = sp[hit], dvx[hit], dvy[hit]
                side = np.sign(dvx * nx[idx] + dvy * ny[idx])
                side = np.where(side == 0, 1.0, side)
                vpx, vpy = om[idx] * sp * nx[idx], om[idx] * sp * ny[idx]
                approach = (dvx * (v - vpx) + dvy * (0 - vpy)) < 0
                leading = (dvx * vpx + dvy * vpy) > 1e-9
                ia = idx[approach & leading & (np.abs(sp) > HUB_D / 2)]
                state[ia] = ATT
                s_cp[ia] = sp[approach & leading & (np.abs(sp) > HUB_D / 2)]
                s_att[ia] = x[ia] * ux[ia] + y[ia] * uy[ia]
                d_att[ia] = x[ia] * nx[ia] + y[ia] * ny[ia]
                sig[ia] = side[approach & leading & (np.abs(sp) > HUB_D / 2)]
                t_contact[ia] = t
                touched[ia] = True
                ifc = idx[approach & ~(leading & (np.abs(sp) > HUB_D / 2))]
                state[ifc] = FACE
                sig[ifc] = side[approach & ~(leading & (np.abs(sp) > HUB_D / 2))]
                touched[ifc] = True
                t_contact[ifc] = np.where(np.isnan(t_contact[ifc]), t, t_contact[ifc])
        if record:
            hist.append((t, th.copy(), x.copy(), y.copy(), state.copy()))
        th_prev, om_prev = th, om
        t += dt
    # ------------- final classification of unresolved items --------------
    th_end, _ = seg.eval(t, th_rest)
    m = state == FACE
    out[m] = np.where(np.sin(th_end[m]) > 0, 'X', 'O')    # slide off that tip
    m = (state == FREE) & touched
    out[m] = np.where(y[m] >= 0, 'X', 'O')
    # untouched items: either they will still meet the rest plate (-> rear-left)
    # or they have already passed it on the right side (-> by lateral position)
    m = (state == FREE) & ~touched
    if np.any(m):
        st_, ct_ = math.sin(th_rest), math.cos(th_rest)
        xc = (y[m] * ct_ - h[m]) / st_
        sc = xc * ct_ + y[m] * st_
        will_hit = (x[m] < xc) & (sc >= -R_HALF) & (sc <= R_HALF)
        out[m] = np.where(will_hit, 'N', np.where(y[m] >= 0, 'X', 'O'))
    m = state == ATT                                      # still carried
    out[m] = np.where(y[m] >= 0, 'X', 'O')
    res = dict(out=out, vrel=vrel, rc=rc, t_contact=t_contact, vx=vxo, vy=vyo,
               x=x, y=y)
    if record:
        res['hist'] = hist
    return res


# ---------------------------------------------------------------------------
# Standard action sequences
# ---------------------------------------------------------------------------
def seq_strike(seg, idx, t0):
    """Proposed plastic action: ramped CW strike +45 -> -45, dwell, gentle
    CCW return.  Returns end time."""
    t1 = seg.add(idx, t0, 'tri', TH_REST, TH_ORG - TH_REST, tau=TAU_STRIKE)
    t2 = seg.add(idx, t1 + DWELL, 'trap', TH_ORG, TH_REST - TH_ORG,
                 tau=TAU_RET, wmax=OMEGA_RET)
    return t2


def seq_organic(seg, idx, t0, t_ret):
    """Proposed organic action: ramped CW move to -45, hold until t_ret,
    gentle CCW return."""
    t1 = seg.add(idx, t0, 'tri', TH_REST, TH_ORG - TH_REST, tau=TAU_ORG)
    t_ret = np.maximum(t_ret, t1)
    t2 = seg.add(idx, t_ret, 'trap', TH_ORG, TH_REST - TH_ORG,
                 tau=TAU_RET, wmax=OMEGA_RET)
    return t2


def seq_asbuilt(seg, idx, t0, steps, rate, hold=HOLD_ASBUILT):
    """As-built swing_to_and_back(): constant-rate move by `steps` from the
    rest pose, hold, constant-rate return."""
    d = steps * STEP
    t1 = seg.add(idx, t0, 'const', TH_REST, d, rate=rate)
    t2 = seg.add(idx, t1 + hold, 'const', TH_REST + d, -d, rate=rate)
    return t2
