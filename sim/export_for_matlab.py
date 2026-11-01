"""Export the data behind every plotted figure to MAT files for the MATLAB
figure scripts (../matlab/data/*.mat).  Geometry is drawn from dimensions in
the MATLAB script itself and needs no data file."""
import json, math, os, pickle
import numpy as np
from scipy.io import savemat
from params import *
from dynamics import gate_inertia, profile, step_schedule_tri
from kin import Segments, simulate, seq_strike

OUT = os.path.join('..', 'matlab', 'data')
os.makedirs(OUT, exist_ok=True)
R = json.load(open('results/v3.json'))
MUL = json.load(open('results/v3_multi.json'))
NUM = json.load(open('results/numbers_v3.json'))
CW = pickle.load(open('results/cw_v3.pkl', 'rb'))
VS = sorted(CW)
get = lambda ctl, cfg, pm, route, v: R[f'{ctl}|{cfg}|{pm}|{route}|{v}']

# ---------------------------------------------------------------- fig_mc ---
VB = sorted({int(k.split('|')[4]) for k in R if k.startswith('base|D|0.0|PLASTIC|')})
best_v, best_ok = [], []
for v in (50, 100, 150, 200, 250, 300, 400):
    accs = [R[f'best|D|0.0|PLASTIC|{v}|{td}']['ok']
            for td in np.round(np.arange(0.0, 1.31, 0.1), 2) if f'best|D|0.0|PLASTIC|{v}|{td}' in R]
    best_v.append(v / 1000)
    best_ok.append(max(accs))
mc = dict(
    v_ed=np.array(VS) / 1000,
    P_ed_pm0=[get('prop', 'D', 0.0, 'PLASTIC', v)['ok'] for v in VS],
    O_ed_pm0=[get('prop', 'D', 0.0, 'ORGANIC', v)['ok'] for v in VS],
    P_ed_pm01=[get('prop', 'D', 0.1, 'PLASTIC', v)['ok'] for v in VS],
    O_ed_pm01=[get('prop', 'D', 0.1, 'ORGANIC', v)['ok'] for v in VS],
    v_ff=np.array(VB) / 1000,
    P_ff_pm0=[get('base', 'D', 0.0, 'PLASTIC', v)['ok'] for v in VB],
    O_ff_pm0=[get('base', 'D', 0.0, 'ORGANIC', v)['ok'] for v in VB],
    P_ff_pm01=[get('base', 'D', 0.1, 'PLASTIC', v)['ok'] for v in VB],
    O_ff_pm01=[get('base', 'D', 0.1, 'ORGANIC', v)['ok'] for v in VB],
    contam_ff=[get('base', 'D', 0.0, 'ORGANIC', v)['dist']['P'] for v in VB],
    v_best=best_v, P_best=best_ok,
    bound_P=NUM['tab']['D']['b4P'] / 1000, bound_O=NUM['tab']['D']['b4O'] / 1000)
savemat(os.path.join(OUT, 'fig_mc.mat'), {k: np.asarray(v, float) for k, v in mc.items()})

# ------------------------------------------------------------- fig_multi ---
SP = [0, 100, 150, 200, 300, 450, 600]
RT = [10, 20, 30, 45, 60, 90]
pair = {}
for pol in ('fcfs', 'safe'):
    pair[pol] = np.array([[np.mean([MUL[f'pair|{v}|{dx}|{a}{b}|0.0|{pol}']['both_ok']
                                    for a in range(3) for b in range(3)]) for dx in SP]
                          for v in (100, 200, 300)])
stream = {}
for pol in ('fcfs', 'safe'):
    stream[pol] = np.array([[MUL[f'stream|200|{r}|0.0|{pol}']['all'][o] for r in RT]
                            for o in ('correct', 'reject', 'wrong')])
savemat(os.path.join(OUT, 'fig_multi.mat'), dict(
    spacing=np.array(SP, float), speeds=np.array([0.1, 0.2, 0.3]),
    pair_fcfs=pair['fcfs'], pair_safe=pair['safe'], rates=np.array(RT, float),
    stream_fcfs=stream['fcfs'], stream_safe=stream['safe']))

# ------------------------------------------------------------ fig_motion ---
J = gate_inertia()['J']
t = np.linspace(0, 0.62, 3000)
D = math.pi / 2
cases = [dict(kind='tri', tau=TAU_STRIKE), dict(kind='tri', tau=TAU_ORG),
         dict(kind='trap', tau=TAU_RET, wmax=OMEGA_RET), dict(kind='const', rate=F_FAST),
         dict(kind='const', rate=F_SLOW)]
TH, OM, TM = [], [], []
for kw in cases:
    kw = dict(kw)
    kind = kw.pop('kind')
    th, om, tm = profile(kind, t, D, J=J, **kw)
    TH.append(np.degrees(th))
    OM.append(om)
    TM.append(tm)
ts, _ = step_schedule_tri(STEPS_90, J, TAU_STRIKE)
savemat(os.path.join(OUT, 'fig_motion.mat'), dict(
    t=t, theta_deg=np.array(TH), omega=np.array(OM), t_end=np.array(TM),
    tick_t=ts[4::5], tick_deg=np.degrees(np.arange(5, 51, 5) * STEP)))

# -------------------------------------------------------------- fig_maps ---
d = np.load('results/maps.npz')
codes = ['P', 'B', 'X', 'O', 'R', 'N']


def to_idx(o):
    m = np.zeros(o.shape)
    for i, c in enumerate(codes):
        m[o == c] = i
    m[o == 'N'] = codes.index('X')       # untouched items keep the rest route
    return m


ex = {}
for name, xx in (('hit', -104.0), ('blocked', -60.0), ('early', -190.0), ('late', -20.0)):
    seg = Segments(1, 3)
    seq_strike(seg, np.arange(1), 0.0)
    rr = simulate(seg, np.array([xx]), np.array([0.0]), 65.0, 200.0, record=True)
    H = rr['hist']
    xs = [q[2][0] for q in H]
    ys = [q[3][0] for q in H]
    st = [q[4][0] for q in H]
    if 3 in st:                          # cut at release, append the Coulomb slide
        k = st.index(3)
        xs, ys = xs[:k + 1], ys[:k + 1]
        cx, cy = xs[-1], ys[-1]
        rx, ry = rr['vx'][0] - 200.0, rr['vy'][0]
        sr = math.hypot(rx, ry)
        a = 0.5 * G_MM
        for tt in np.linspace(0, sr / a, 80):
            s = sr * tt - 0.5 * a * tt ** 2
            px = cx + 200.0 * tt + rx / sr * s
            py = cy + ry / sr * s
            xs.append(px)
            ys.append(py)
            if py > POST_INNER + 10 or (abs(px) <= POST_X / 2 + 32.5 and py >= POST_INNER - 32.5):
                break
    ex[name] = (np.array(xs), np.array(ys))
maps = dict(xm=d['xm'], ym=d['ym'], map_strike=to_idx(d['o_s']), map_organic=to_idx(d['o_o']))
for k, (x, y) in ex.items():
    maps[f'path_{k}_x'] = x
    maps[f'path_{k}_y'] = y
savemat(os.path.join(OUT, 'fig_maps.mat'), maps)
print('exported', sorted(os.listdir(OUT)))
