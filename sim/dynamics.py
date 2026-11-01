"""Gate mass properties and stepper motion profiles."""
import math
import numpy as np
from params import *


def gate_inertia():
    """Mass and moment of inertia of the gate about its vertical axis.
    Plate + hub + 45-deg flare, fully dense PLA (upper bound)."""
    rho = RHO_PLA
    # plate (L along x, t along y, H along z) about z through its centre
    Lm, Tm, Hm = PLATE_L / 1e3, PLATE_T / 1e3, PLATE_H / 1e3
    m_p = rho * Lm * Tm * Hm
    J_p = m_p * (Lm ** 2 + Tm ** 2) / 12.0
    # hub: hollow cylinder on the axis
    ro, ri, hh = HUB_D / 2e3, BORE_D / 2e3, HUB_H / 1e3
    m_h = rho * math.pi * (ro ** 2 - ri ** 2) * hh
    J_h = 0.5 * m_h * (ro ** 2 + ri ** 2)
    # flare: hull of an 18 x 5 mm rectangle and an 18 mm disc, 6.5 mm apart.
    # cross-section at fraction s is (1-s)A + sB (Minkowski combination);
    # integrate mass and polar moment numerically on a fine grid.
    a, b, r = HUB_D / 1e3, PLATE_T / 1e3, HUB_D / 2e3
    hf = FLARE_H / 1e3
    n = 400
    xs = np.linspace(-a / 2, a / 2, n)
    ys = np.linspace(-r, r, n)
    X, Y = np.meshgrid(xs, ys)
    dA = (xs[1] - xs[0]) * (ys[1] - ys[0])
    m_f = 0.0
    J_f = 0.0
    for s in np.linspace(0, 1, 41)[:-1] + 1 / 80:
        # support-function test for the Minkowski sum (1-s)A + sB:
        # point p is inside iff max over directions of (p.d - hA(d)(1-s) - hB(d)s) <= 0
        ang = np.linspace(0, 2 * np.pi, 180, endpoint=False)
        dx, dy = np.cos(ang), np.sin(ang)
        hA = (a / 2) * np.abs(dx) + (b / 2) * np.abs(dy)
        hB = r * np.ones_like(ang)
        h = (1 - s) * hA + s * hB
        inside = np.ones_like(X, dtype=bool)
        for k in range(len(ang)):
            inside &= (X * dx[k] + Y * dy[k]) <= h[k]
        dm = rho * inside.sum() * dA * (hf / 40)
        m_f += dm
        J_f += rho * ((X ** 2 + Y ** 2) * inside).sum() * dA * (hf / 40)
    m = m_p + m_h + m_f
    J = J_p + J_h + J_f
    return dict(m_plate=m_p, J_plate=J_p, m_hub=m_h, J_hub=J_h,
                m_flare=m_f, J_flare=J_f, m=m, J=J)


def tri_time(dtheta, J, tau):
    """Triangular (constant |alpha|) move time."""
    return 2.0 * math.sqrt(abs(dtheta) * J / tau)


def trap_time(dtheta, J, tau, wmax):
    al = tau / J
    ta = wmax / al
    if abs(dtheta) <= wmax * ta:          # never reaches wmax
        return 2.0 * math.sqrt(abs(dtheta) / al)
    return abs(dtheta) / wmax + ta


def step_schedule_tri(nsteps, J, tau):
    """Exact step instants of a triangular profile: the k-th step boundary
    is reached when the accumulated angle equals k*STEP (AVR446 style)."""
    al = tau / J
    D = nsteps * STEP
    tm = 2 * math.sqrt(D / al)
    t = []
    for k in range(1, nsteps + 1):
        th = k * STEP
        if th <= D / 2:
            t.append(math.sqrt(2 * th / al))
        else:
            t.append(tm - math.sqrt(2 * (D - th) / al))
    return np.array(t), tm


def start_torque(J, f):
    """Mean inertial torque needed to reach the commanded full-step rate f
    within the first step period, i.e. alpha = STEP*f^2."""
    return J * STEP * f ** 2


def profile(kind, t, dtheta, J=None, tau=None, wmax=None, rate=None):
    """theta(t), omega(t) for 0<=t<=T of one motion segment (theta(0)=0).
    kind: 'tri' | 'trap' | 'const'."""
    sgn = 1.0 if dtheta >= 0 else -1.0
    D = abs(dtheta)
    t = np.asarray(t, dtype=float)
    if kind == 'tri':
        al = tau / J
        tm = 2 * math.sqrt(D / al)
        th = np.where(t < tm / 2, 0.5 * al * t ** 2,
                      D - 0.5 * al * np.clip(tm - t, 0, None) ** 2)
        om = np.where(t < tm / 2, al * t, al * np.clip(tm - t, 0, None))
        th = np.where(t >= tm, D, th)
        om = np.where(t >= tm, 0.0, om)
        return sgn * th, sgn * om, tm
    if kind == 'trap':
        al = tau / J
        ta = wmax / al
        if D <= wmax * ta:
            return profile('tri', t, dtheta, J=J, tau=tau)
        tc = D / wmax - ta
        tm = 2 * ta + tc
        th = np.where(t < ta, 0.5 * al * t ** 2,
             np.where(t < ta + tc, 0.5 * al * ta ** 2 + wmax * (t - ta),
                      D - 0.5 * al * np.clip(tm - t, 0, None) ** 2))
        om = np.where(t < ta, al * t, np.where(t < ta + tc, wmax,
                                                 al * np.clip(tm - t, 0, None)))
        th = np.where(t >= tm, D, th)
        om = np.where(t >= tm, 0.0, om)
        return sgn * th, sgn * om, tm
    if kind == 'const':
        w = rate * STEP
        tm = D / w
        th = np.clip(w * t, 0, D)
        om = np.where(t < tm, w, 0.0)
        return sgn * th, sgn * om, tm
    raise ValueError(kind)


def seg_time(kind, dtheta, J, tau=None, wmax=None, rate=None):
    if kind == 'tri':
        return tri_time(dtheta, J, tau)
    if kind == 'trap':
        return trap_time(dtheta, J, tau, wmax)
    if kind == 'const':
        return abs(dtheta) / (rate * STEP)
    raise ValueError(kind)


if __name__ == '__main__':
    g = gate_inertia()
    for k, v in g.items():
        print(f'{k:8s} {v:.4e}')
    J = g['J']
    print('strike 90deg @100mNm', tri_time(math.pi / 2, J, TAU_STRIKE))
    print('organic 90deg @50mNm', tri_time(math.pi / 2, J, TAU_ORG))
    print('return trap', trap_time(math.pi / 2, J, TAU_RET, OMEGA_RET))
    print('start torque 250', start_torque(J, F_SLOW), '500', start_torque(J, F_FAST))
