import json, math, numpy as np
from kin import Segments, simulate, seq_strike, seq_organic, J_GATE
from params import *
from dynamics import tri_time
T_ORGMOVE = tri_time(math.pi/2, J_GATE, TAU_ORG)

def windows(v, w=65.0, mu=0.5, ny=9, dx=0.5):
    ymax = BAND - w/2
    ys = np.linspace(-ymax, ymax, ny)
    x0 = np.arange(-260, 0, dx)
    X0, Y0 = np.meshgrid(x0, ys)
    seg = Segments(X0.size, 3); seq_strike(seg, np.arange(X0.size), 0.0)
    r = simulate(seg, X0.ravel(), Y0.ravel(), w, v, mu=mu)
    P = (r['out'] == 'P').reshape(X0.shape)
    L = P.sum(axis=1) * dx
    lo = [x0[np.nonzero(p)[0][0]] if p.any() else np.nan for p in P]
    hi = [x0[np.nonzero(p)[0][-1]] if p.any() else np.nan for p in P]
    vr = r['vrel'][(r['out'] == 'P')]; rc = r['rc'][(r['out'] == 'P')]
    h = w/2 + PLATE_T/2
    xc = -Y0.ravel() - h*math.sqrt(2)
    seg = Segments(X0.size, 3)
    seq_organic(seg, np.arange(X0.size), 0.0, np.maximum(T_ORGMOVE, (xc - X0.ravel())/v + 0.10))
    o = simulate(seg, X0.ravel(), Y0.ravel(), w, v, mu=mu)['out'].reshape(X0.shape)
    omax = []
    for row in o:
        bad = np.nonzero(row != 'O')[0]
        omax.append(x0[bad[0]-1] if bad.size and bad[0] > 0 else np.nan)
    return dict(Lmin=float(L.min()), Lmax=float(L.max()), lo=float(np.nanmin(lo)), hi=float(np.nanmax(hi)),
                vmin=float(vr.min()/1000) if vr.size else None, vmax=float(vr.max()/1000) if vr.size else None,
                rcmin=float(rc.min()) if rc.size else None,
                omax_near=float(-np.nanmax(omax)), omax_far=float(-np.nanmin(omax)))
out = {}
out['design'] = windows(200.0)
print('design', out['design'], flush=True)
for mu in (0.3, 0.7, 1.3):
    out[f'mu{mu}'] = windows(200.0, mu=mu, ny=5)
    print('mu', mu, out[f'mu{mu}'], flush=True)
for w in (55.0, 90.0):
    out[f'w{w}'] = windows(200.0, w=w, ny=5)
    print('w', w, out[f'w{w}'], flush=True)
json.dump(out, open('results/fine_windows.json', 'w'), indent=1)
