"""
Point 7: load of a struck item on the stepper.

The planar model carries a struck item at its contact radius r_c until the
plate starts to decelerate, so during the acceleration half of the strike the
motor drives J_gate + J_rotor + m r_c^2 at the strike's angular acceleration
alpha = tau_b / J_gate, plus the belt friction mu m g r_c.  This script finds
the contact radius at the strike aim point (planar model) and the item mass
at which the load torque reaches a given motor torque.  The holding torque
bounds the available torque from above; the pull-out torque at the strike's
peak rate (693 full steps/s) is lower and has to be measured.
-> results/stepper_load.json
"""
import json, pickle
import numpy as np
from params import *
from kin import Segments, simulate, seq_strike, J_GATE

ALPHA = TAU_STRIKE / J_GATE           # rad/s^2, designed strike
MU = 0.5                              # belt friction (planar model)
MOTORS = {                            # holding torque (N m), rotor inertia (kg m^2), typical values
    'NEMA 17, 48 mm': (0.59, 82e-7),
    'NEMA 17, 48 mm, 60% current': (0.6 * 0.59, 82e-7),
    'NEMA 23, 56 mm': (1.26, 280e-7),
    'NEMA 23, 76 mm': (1.90, 480e-7),
    'NEMA 23, 76 mm, TB6600 at 2.2 A': (1.90 * 2.2 / 2.8, 480e-7),   # 23HS30-2804S as fitted (24 Oct)
}


def contact_radius(v=200, w=65.0):
    cw = pickle.load(open('results/cw_v3.pkl', 'rb'))[v]
    aim = 0.5 * (cw['s_lo'] + cw['s_hi'])
    ok = np.isfinite(aim)
    seg = Segments(int(ok.sum()), 3)
    seq_strike(seg, np.arange(int(ok.sum())), 0.0)
    r = simulate(seg, aim[ok], cw['y'][ok], w, float(v))
    return r['rc'], r['t_contact'], r['out']


def load_torque(m, rc, J_rot):
    return (J_GATE + J_rot + m * rc ** 2) * ALPHA + MU * m * 9.81 * rc


def m_max(T, rc, J_rot):
    return (T - (J_GATE + J_rot) * ALPHA) / (rc ** 2 * ALPHA + MU * 9.81 * rc)


if __name__ == '__main__':
    rc, tc, o = contact_radius()
    print('contact radius at the aim point (mm):', np.round(rc, 1), 'contact t (s):', np.round(tc, 3), o)
    res = dict(alpha=ALPHA, rc=rc.tolist(), t_contact=tc.tolist(), motors={})
    for name, (T, Jr) in MOTORS.items():
        res['motors'][name] = {f'{int(r)}': m_max(T, r / 1000, Jr) for r in (51, 75, 90, 100)}
        print(f'{name:30s}', {k: f'{1000 * x:.0f} g' for k, x in res['motors'][name].items()})
    for m in (0.02, 0.05, 0.1, 0.25, 0.5):
        print(f'item {1000 * m:.0f} g at 90 mm: load torque {load_torque(m, 0.09, 82e-7):.2f} N m')
    json.dump(res, open('results/stepper_load.json', 'w'), indent=1)
