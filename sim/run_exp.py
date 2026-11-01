"""Run every simulation experiment used in the paper; save to results/."""
import json, math, os, sys, time, pickle
import numpy as np
from params import *
from dynamics import gate_inertia, tri_time, trap_time, start_torque, step_schedule_tri
from kin import Segments, simulate, seq_strike, seq_organic, J_GATE
import timing as tm

OUT = os.path.join(os.path.dirname(__file__), 'results')
os.makedirs(OUT, exist_ok=True)
rng = np.random.default_rng(20261012)
R = {}
t_all = time.time()


def log(*a):
    print(f'[{time.time() - t_all:7.1f}s]', *a, flush=True)


# ------------------------------------------------------------ dynamics ---
g = gate_inertia()
R['inertia'] = g
J = g['J']
R['moves'] = dict(
    strike=dict(t=tri_time(math.pi / 2, J, TAU_STRIKE), tau=TAU_STRIKE),
    organic=dict(t=tri_time(math.pi / 2, J, TAU_ORG), tau=TAU_ORG),
    ret=dict(t=trap_time(math.pi / 2, J, TAU_RET, OMEGA_RET), tau=TAU_RET,
             wmax=OMEGA_RET),
    start_250=start_torque(J, F_SLOW), start_500=start_torque(J, F_FAST),
    start_250_noload=None)
for k in ('strike', 'organic'):
    mv = R['moves'][k]
    mv['wpk'] = 2 * (math.pi / 2) / mv['t']
    mv['tip'] = mv['wpk'] * R_HALF / 1e3
    mv['steps_pk'] = mv['wpk'] / STEP
log('dynamics', R['moves'])

# ------------------------------------------------ capture windows vs v ---
VS = [50, 75, 100, 125, 150, 175, 200, 225, 250, 275, 300, 350, 400]
CW = {}
for v in VS:
    CW[v] = tm.capture_windows(float(v))
    log('cw', v, np.round(CW[v]['s_lo']), np.round(CW[v]['s_hi']),
        np.round(CW[v]['o_max']))
pickle.dump(CW, open(os.path.join(OUT, 'cw.pkl'), 'wb'))
R['cw'] = {v: dict(y=CW[v]['y'].tolist(), s_lo=CW[v]['s_lo'].tolist(),
                   s_hi=CW[v]['s_hi'].tolist(), o_max=CW[v]['o_max'].tolist())
           for v in VS}

# --------------------------------------------- item size sensitivity ---
R['cw_w'] = {}
for w in (55.0, 65.0, 90.0):
    c = tm.capture_windows(200.0, w=w)
    R['cw_w'][w] = dict(len_min=float(np.nanmin(c['s_hi'] - c['s_lo'])),
                        len_max=float(np.nanmax(c['s_hi'] - c['s_lo'])),
                        o_max_max=float(np.nanmax(c['o_max'])),
                        o_max_min=float(np.nanmin(c['o_max'])))
    log('w', w, R['cw_w'][w])

# --------------------------------------------- friction sensitivity ---
R['mu'] = {}
for mu in (0.3, 0.5, 0.7, 0.9, 1.1, 1.3):
    c = tm.capture_windows(200.0, mu=mu, ny=3)
    R['mu'][mu] = dict(len=(c['s_hi'] - c['s_lo']).tolist())
    log('mu', mu, R['mu'][mu])

# release speed / contact radius statistics inside the window (v=0.2 m/s)
x0 = np.arange(-160, -50, 1.0)
y0g = np.linspace(-(BAND - 32.5), BAND - 32.5, 9)
X0, Y0 = np.meshgrid(x0, y0g)
seg = Segments(X0.size, 3)
seq_strike(seg, np.arange(X0.size), 0.0)
r = simulate(seg, X0.ravel(), Y0.ravel(), 65.0, 200.0)
P = r['out'] == 'P'
R['release'] = dict(vmin=float(np.nanmin(r['vrel'][P])), vmax=float(np.nanmax(r['vrel'][P])),
                    rcmin=float(np.nanmin(r['rc'][P])), rcmax=float(np.nanmax(r['rc'][P])))
log('release', R['release'])

# ------------------------------------------------ outcome maps (v=0.2) ---
xm = np.arange(-300, 61, 2.0)
ym = np.linspace(-(BAND - 32.5), BAND - 32.5, 25)
XM, YM = np.meshgrid(xm, ym)
seg = Segments(XM.size, 3)
seq_strike(seg, np.arange(XM.size), 0.0)
o_s = simulate(seg, XM.ravel(), YM.ravel(), 65.0, 200.0)['out'].reshape(XM.shape)
h = 65.0 / 2 + PLATE_T / 2
xc = -YM.ravel() - h * math.sqrt(2)
seg = Segments(XM.size, 3)
seq_organic(seg, np.arange(XM.size), 0.0,
            np.maximum(tm.T_ORGMOVE, (xc - XM.ravel()) / 200.0 + 0.10))
o_o = simulate(seg, XM.ravel(), YM.ravel(), 65.0, 200.0)['out'].reshape(XM.shape)
np.savez(os.path.join(OUT, 'maps.npz'), xm=xm, ym=ym, o_s=o_s, o_o=o_o)
log('maps done')

# ----------------------------------------------- trajectories for a figure
ex = dict()
for name, xx, yy in (('hit', -104.0, 0.0), ('blocked', -60.0, 0.0),
                     ('early', -190.0, 0.0), ('late', -20.0, 0.0)):
    seg = Segments(1, 3)
    seq_strike(seg, np.arange(1), 0.0)
    rr = simulate(seg, np.array([xx]), np.array([yy]), 65.0, 200.0, record=True)
    H = rr['hist']
    ex[name] = dict(out=str(rr['out'][0]),
                    t=[q[0] for q in H[::5]], th=[float(q[1][0]) for q in H[::5]],
                    x=[float(q[2][0]) for q in H[::5]], y=[float(q[3][0]) for q in H[::5]],
                    st=[int(q[4][0]) for q in H[::5]])
pickle.dump(ex, open(os.path.join(OUT, 'traj.pkl'), 'wb'))
log('traj', {k: v['out'] for k, v in ex.items()})

# ------------------------------------------------------ Monte Carlo ---
T0, LV, GAP = 1.0, 300.0, 200.0
VS_MC = [50, 100, 150, 200, 250, 300, 350, 400]
N = int(sys.argv[1]) if len(sys.argv) > 1 else 3000
mc = {}
for pm in (0.0, 0.1):
    for v in VS_MC:
        cw = CW[v]
        for route in ('PLASTIC', 'ORGANIC'):
            o, rg, d = tm.proposed(route, N, float(v), T0, LV, GAP, pm, rng, cw=cw)
            ob, rb, db = tm.baseline(route, N, float(v), T0, LV, GAP, pm, rng)
            key = f'{route}|{pm}|{v}'
            tgt = 'P' if route == 'PLASTIC' else 'O'
            mc[key] = dict(
                prop=dict(ok=float(np.mean(o == tgt)), regs=float(rg.mean()),
                          det=float(d.mean()),
                          dist={c: float(np.mean(o == c)) for c in 'PBXOR'}),
                base=dict(ok=float(np.mean(ob == tgt)), regs=float(rb.mean()),
                          det=float(db.mean()),
                          dist={c: float(np.mean(ob == c)) for c in 'PBXOR'}))
            log('mc', key, 'prop', mc[key]['prop']['ok'], 'base', mc[key]['base']['ok'],
                'regs', mc[key]['prop']['regs'], mc[key]['base']['regs'])
R['mc'] = mc
R['mc_cfg'] = dict(T=T0, Lv=LV, g=GAP, N=N, w=65.0, sig_x=5.0, eps_v=0.05, m=1)
json.dump(R, open(os.path.join(OUT, 'results.json'), 'w'), indent=1, default=float)

# ---------------------------------- baseline: best fixed delay (plastic)
best = {}
TDS = np.round(np.arange(0.0, 1.31, 0.1), 2)
for v in VS_MC:
    accs = []
    for td in TDS:
        ob, rb, db = tm.baseline('PLASTIC', 1500, float(v), T0, LV, GAP, 0.0, rng, t_d=float(td))
        accs.append(float(np.mean(ob == 'P')))
    k = int(np.argmax(accs))
    best[v] = dict(td=float(TDS[k]), ok=accs[k], all=accs)
    log('best delay', v, best[v]['td'], best[v]['ok'])
R['base_best'] = best
json.dump(R, open(os.path.join(OUT, 'results.json'), 'w'), indent=1, default=float)

# ----------------------------------------- loop-period sensitivity ---
env = {}
for T in (0.8, 1.2):
    for v in VS_MC:
        cw = CW[v]
        for route in ('PLASTIC', 'ORGANIC'):
            o, rg, d = tm.proposed(route, 1500, float(v), T, LV, GAP, 0.0, rng, cw=cw)
            tgt = 'P' if route == 'PLASTIC' else 'O'
            env[f'{route}|{T}|{v}'] = float(np.mean(o == tgt))
            log('env', T, v, route, env[f'{route}|{T}|{v}'])
# confirmation m = 2 at T = 1.0
for v in VS_MC:
    cw = CW[v]
    for route in ('PLASTIC', 'ORGANIC'):
        o, rg, d = tm.proposed(route, 1500, float(v), T0, LV, GAP, 0.0, rng, cw=cw, m=2)
        tgt = 'P' if route == 'PLASTIC' else 'O'
        env[f'{route}|m2|{v}'] = float(np.mean(o == tgt))
        log('env m2', v, route, env[f'{route}|m2|{v}'])
R['env'] = env
json.dump(R, open(os.path.join(OUT, 'results.json'), 'w'), indent=1, default=float)
log('ALL DONE')
