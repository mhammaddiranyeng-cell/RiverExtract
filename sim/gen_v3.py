"""v3: every number, table row and figure of the revised paper from the v3
results.  -> results/numbers_v3.json, ../numbers_v3.tex, ../figs/fig_mc.pdf,
../figs/fig_reg.pdf, ../figs/fig_multi.pdf"""
import json, math, os, pickle
import numpy as np
from params import *
from figstyle import plt, C, COL
import timing as tm
from kin import J_GATE

R = json.load(open('results/v3.json'))
CW = pickle.load(open('results/cw_v3.pkl', 'rb'))
VS = sorted(CW)
MUL = json.load(open('results/v3_multi.json')) if os.path.exists('results/v3_multi.json') and not os.environ.get('SKIP_MULTI') else {}
LV, GAP = 300.0, 200.0
CFG = {
    'D':     dict(tau_i=0.40, tp=0.0),
    'P3':    dict(tau_i=0.40, tp=0.3),
    'P6':    dict(tau_i=0.40, tp=0.6),
    'I25':   dict(tau_i=0.25, tp=0.0),
    'Lv400': dict(tau_i=0.40, tp=0.0, Lv=400.0),
    'm2':    dict(tau_i=0.40, tp=0.0, m=2),
    'sig10': dict(tau_i=0.40, tp=0.0, sig_x=10.0, eps_v=0.10),
}
Z99, Z99_1 = 2.5758, 2.3263
out = {}


def wilson_lb(k, n, z=1.96):
    if n == 0:
        return 0.0
    p = k / n
    d = 1 + z * z / n
    c = p + z * z / (2 * n)
    s = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return (c - s) / d


def get(ctl, cfg, pm, route, v):
    return R.get(f'{ctl}|{cfg}|{pm}|{route}|{v}')


def v99(cfg, pm=0.0, thr=0.99):
    """Highest speed of the 10 mm/s grid up to which both routes pass the
    criterion at every grid speed (95% Wilson lower bound >= thr)."""
    best = None
    for v in VS:
        a, b = get('prop', cfg, pm, 'PLASTIC', v), get('prop', cfg, pm, 'ORGANIC', v)
        if a is None or b is None:
            break
        if min(wilson_lb(a['k_ok'], a['n']), wilson_lb(b['k_ok'], b['n'])) >= thr:
            best = v
        else:
            break
    return best


def v99_route(cfg, route, pm=0.0, thr=0.99):
    best = None
    for v in VS:
        a = get('prop', cfg, pm, route, v)
        if a is None:
            break
        if wilson_lb(a['k_ok'], a['n']) >= thr:
            best = v
        else:
            break
    return best


# ---------------------------------------------------------- bounds -----
def cw_arr(key):
    return np.array([CW[v][key] for v in VS], dtype=float)


S_LO, S_HI, O_MAX = cw_arr('s_lo'), cw_arr('s_hi'), cw_arr('o_max')
YS = CW[VS[0]]['y']


def xstar(v, route):
    if route == 'P':
        vals = np.nanmax(-(S_HI - 10.0), axis=1)
    else:
        vals = np.nanmax(-(O_MAX - 5.0), axis=1)
    return float(np.interp(v, VS, vals))


def bound4(route, tau_i, tp, Lv=LV, g=GAP, m=1, **_):
    """Feasibility bound (4): first (m-th) detection in time for the action."""
    T = tau_i + tp
    v = 200.0
    for _ in range(300):
        v_new = (g + Lv - xstar(v, route)) / (m * T + tau_i + 1.0 / F_CAM)
        v = 0.5 * v + 0.5 * v_new
    return min(v, Lv / (m * T))


def bound_noise(route, tau_i, tp, Lv=LV, g=GAP, m=1, sig_x=5.0, eps_v=0.05, margin=40.0,
                first=False, **_):
    """Noise-aware bound, eq. (5): the action starts with a position error of
    variance eps^2/3 E[d^2] + sigma^2 + v^2/(12 f_c^2), where d, the distance
    from the newest usable detection to the action point, is uniform on
    [a, a + min(vT, Lv)], a = max(x_a + g, v (tau_i + 1/f_c)).  Strike: fails
    if the error leaves the capture window (two-sided, half-width L_c(y)/2);
    organic move: fails if it starts more than margin - 5 mm late (one-sided).
    Gaussian approximation; failure probability averaged over the lateral
    positions of the planar model; bound = highest speed with every lower
    speed at <= 1% failure.  first=True: the action is timed once from the
    first detection (fixed-delay baseline), so d is measured from within
    min(vT, Lv) of the window's upstream end: a = x_a + g + Lv - min(vT, Lv)."""
    from math import erf, sqrt
    Phi = lambda z: 0.5 * (1 + erf(z / sqrt(2)))
    T = tau_i + tp
    vs = np.arange(20.0, 800.0, 1.0)
    ok = np.zeros(vs.size, dtype=bool)
    for i, v in enumerate(vs):
        fails = []
        for j in range(len(YS)):
            if route == 'P':
                lo, hi = np.interp(v, VS, S_LO[:, j]), np.interp(v, VS, S_HI[:, j])
                if not (np.isfinite(lo) and np.isfinite(hi)):
                    fails.append(1.0)
                    continue
                aim, tol = 0.5 * (lo + hi), (hi - lo) / 2
            else:
                om = np.interp(v, VS, O_MAX[:, j])
                if not np.isfinite(om):
                    fails.append(1.0)
                    continue
                aim, tol = om - margin, margin - 5.0
            a = max(aim + g, v * (tau_i + 1.0 / F_CAM + 0.015))
            W = min(v * T, Lv)
            if first:
                a = aim + g + Lv - W
            Ed2 = a * a + a * W + W * W / 3
            se = sqrt(eps_v ** 2 / 3 * Ed2 + sig_x ** 2 + (v / F_CAM) ** 2 / 12)
            fails.append(2 * (1 - Phi(tol / se)) if route == 'P' else 1 - Phi(tol / se))
        ok[i] = np.mean(fails) <= 0.01
    bad = np.nonzero(~ok)[0]
    if bad.size == 0:
        return float(vs[-1])
    return float(vs[bad[0] - 1]) if bad[0] > 0 else 0.0


def bounds(cfg):
    kw = CFG[cfg]
    b4 = min(bound4('P', **kw), bound4('O', **kw))
    bn = min(bound_noise('P', **kw), bound_noise('O', **kw))
    return b4, bn, min(b4, bn)


f2 = lambda v: f'{v / 1000:.2f}'
TAB = {}
for c in CFG:
    vs_ = v99(c)
    b4, bn, bb = bounds(c)
    TAB[c] = dict(T=CFG[c]['tau_i'] + CFG[c]['tp'], sim=vs_, b4=b4, bn=bn, b=bb,
                  simP=v99_route(c, 'PLASTIC'), simO=v99_route(c, 'ORGANIC'),
                  b4P=bound4('P', **CFG[c]), b4O=bound4('O', **CFG[c]))
    print(f'{c:6s} T={TAB[c]["T"]:.2f} sim={vs_} (P {TAB[c]["simP"]}, O {TAB[c]["simO"]})  '
          f'b4={b4:.0f} (P {TAB[c]["b4P"]:.0f}, O {TAB[c]["b4O"]:.0f})  noise={bn:.0f}  '
          f'bound={bb:.0f}')
for c, name in (('D', 'D'), ('P3', 'Pthree'), ('P6', 'Psix'), ('I25', 'Ifast'),
                ('Lv400', 'Lv'), ('m2', 'Mtwo'), ('sig10', 'Sig')):
    t = TAB[c]
    out[f'Env{name}'] = f2(t['sim']) if t['sim'] else '<0.05'
    out[f'Bnd{name}'] = f2(t['b4'])
    out[f'Bnn{name}'] = f2(t['b'])
out['VcfP'] = f2(TAB['D']['b4P'])
out['VcfO'] = f2(TAB['D']['b4O'])
out['VmaxBoth'] = f2(TAB['D']['sim'])
out['VmaxP'] = f2(TAB['D']['simP'])
out['VmaxO'] = f2(TAB['D']['simO'])
vpm = v99('D', pm=0.1)
out['VmaxPm'] = f2(vpm) if vpm else '<0.05'
out['Ntrials'] = str(R[f'prop|D|0.0|PLASTIC|{VS[0]}']['n'])
out['Tdesign'] = '0.40'

# speed at which the old design point (T = 1.0 s) passes -> gain from removing the pacing
out['GainPace'] = f"{TAB['D']['sim'] / TAB['P6']['sim']:.1f}"

# x*(v) is close to linear on the grid, |x*| = x0 + v t_lead, which turns (4) into
# v <= (g + Lv - x0) / (T + tau_i + 1/f_c + t_lead)
VV = np.array(VS, dtype=float) / 1000.0
for route, name in (('P', 'P'), ('O', 'O')):
    xs_ = np.array([xstar(v, route) for v in VS])
    ok_ = np.isfinite(xs_)
    lead, x0 = np.polyfit(VV[ok_], xs_[ok_], 1)
    res = np.abs(xs_[ok_] - (x0 + lead * VV[ok_])).max()
    vcf = (GAP + LV - x0) / (CFG['D']['tau_i'] + CFG['D']['tp'] + CFG['D']['tau_i'] + 1.0 / F_CAM + lead / 1000.0)
    out[f'Xs{name}zero'] = f'{x0:.0f}'
    out[f'Xs{name}lead'] = f'{lead:.0f}'
    out[f'Xs{name}res'] = f'{res:.1f}'
    out[f'Vcf{name}lin'] = f2(min(vcf, LV / (CFG['D']['tau_i'] + CFG['D']['tp'])))
    print(f'x* {route}: {x0:.1f} mm + v * {lead:.1f} ms (max residual {res:.1f} mm), '
          f'closed-form (4) at D: {vcf:.0f} mm/s')

# ---------------------------------------------- wrong-bin / four outcomes ---
def four(v, pm, conf):
    """Four outcomes per true class at speed v (design point) for confusion
    matrix conf (rows: true P, O, X; cols: committed P, O, X)."""
    dP = get('prop', 'D', pm, 'PLASTIC', v)
    dO = get('prop', 'D', pm, 'ORGANIC', v)
    det = dP['det']
    dist_dec = {0: dP['dist_dec'], 1: dO['dist_dec'], 2: dict(P=0, B=0, X=1, O=0, R=0)}
    res = {}
    for i, name in enumerate('POX'):
        p = {x: 0.0 for x in 'PBXOR'}
        p['X'] += 1 - det
        for j in range(3):
            for x in 'PBXOR':
                p[x] += det * conf[i][j] * dist_dec[j][x]
        tgt = name
        correct = p[tgt]
        lost = p['B'] + p['R']
        reject = p['X'] if tgt != 'X' else 0.0
        wrong = 1 - correct - lost - reject
        res[name] = dict(correct=correct, reject=reject, wrong=max(wrong, 0.0), lost=lost)
    return res


CONF = {
    'perfect': [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
    'c10': [[0.9, 0.05, 0.05], [0.05, 0.9, 0.05], [0.05, 0.05, 0.9]],
    # bench test (Table VII): detected passes only; OTHER never detected -> identity row
    'bench': [[27 / 33, 6 / 33, 0.0], [0.0, 12 / 14, 2 / 14], [0, 0, 1]],
}
FOUR = {}
for k, cf in CONF.items():
    for pm in (0.0, 0.1):
        FOUR[f'{k}|{pm}'] = four(200, pm, cf)
for k, r_ in FOUR.items():
    print('four', k, {c: {o: round(100 * x, 1) for o, x in d.items()} for c, d in r_.items()})
mean_wrong = lambda r_: np.mean([r_[c]['wrong'] for c in 'POX'])
out['WrongCten'] = f"{100 * mean_wrong(FOUR['c10|0.0']):.1f}"
out['WrongBench'] = f"{100 * mean_wrong(FOUR['bench|0.0']):.1f}"
# largest share of items that left a sorting route without misclassification
out['LostMaxD'] = f"{100 * max(get('prop', 'D', 0.0, r, v)['dist']['B'] + get('prop', 'D', 0.0, r, v)['dist']['R'] for r in ('PLASTIC', 'ORGANIC') for v in VS):.2f}"
out['LostMaxDlim'] = f"{100 * max(get('prop', 'D', 0.0, r, v)['dist']['B'] + get('prop', 'D', 0.0, r, v)['dist']['R'] for r in ('PLASTIC', 'ORGANIC') for v in VS if v <= TAB['D']['sim']):.2f}"
# wrong bins from timing alone (no misclassification) at the design point
out['WrongTimingMax'] = f"{100 * max(max(get('prop', 'D', 0.0, 'PLASTIC', v)['dist']['O'], get('prop', 'D', 0.0, 'ORGANIC', v)['dist']['P']) for v in VS):.2f}"

# ---------------------------------------------------- first firmware at D ---
VB = sorted({int(k.split('|')[4]) for k in R if k.startswith('base|D|0.0|PLASTIC|')})
bp = lambda v, pm=0.0: get('base', 'D', pm, 'PLASTIC', v)
bo = lambda v, pm=0.0: get('base', 'D', pm, 'ORGANIC', v)
out['BaseP'] = f"{100 * bp(100)['ok']:.1f}"
out['BasePtwo'] = f"{100 * bp(200)['ok']:.0f}"
out['BasePonefifty'] = f"{100 * bp(150)['ok']:.0f}"
out['BaseOmax'] = f"{100 * max(bo(v)['ok'] for v in VB if v <= 300):.0f}"
out['BaseOmin'] = f"{100 * min(bo(v)['ok'] for v in VB if v <= 300):.0f}"
out['BaseContam'] = f"{100 * max(bo(v)['dist']['P'] for v in VB):.0f}"
BEST = {}
for v in (50, 100, 150, 200, 250, 300, 400):
    accs = [(td, R[f'best|D|0.0|PLASTIC|{v}|{td}']['ok'])
            for td in np.round(np.arange(0.0, 1.31, 0.1), 2) if f'best|D|0.0|PLASTIC|{v}|{td}' in R]
    if accs:
        BEST[v] = max(accs, key=lambda a: a[1])
out['BaseBestP'] = f"{100 * BEST[100][1]:.0f}"
out['BaseBestPtwo'] = f"{100 * BEST[200][1]:.0f}"
out['BaseBestPonefifty'] = f"{100 * BEST[150][1]:.0f}"
out['BaseBestPthree'] = f"{100 * BEST[300][1]:.0f}"
print('first firmware D', {v: (round(bp(v)['ok'], 3), round(bo(v)['ok'], 3)) for v in VB})
print('best td', BEST)
# registrations, level-triggered, at the new loop period
out['RegsTen'] = f"{tm.regs_level_closed(100.0, 0.4, 300.0, 0.0):.1f}"
out['RegsTenOld'] = f"{tm.regs_level_closed(100.0, 1.0, 300.0, 0.0):.1f}"

# ------------------------------------------- stronger baselines (fixed delay) ---
# run_v3_baseline2.py: every action timed once from the first detection, with the
# belt speed from an encoder (Enc) or from the tracks (Trk), and the event-driven
# controller with an encoder (EncNew); prop|D and prop|sig10 are the event-driven
# controller with the speed from the tracks (EnvD, EnvSig).
B2 = json.load(open('results/v3_baseline2.json')) if os.path.exists('results/v3_baseline2.json') else {}


def v99_b2(var, thr=0.99):
    best = None
    for v in VS:
        rs = [B2.get(f'{var}|{r}|{v}') for r in ('PLASTIC', 'ORGANIC')]
        if None in rs:
            break
        if min(wilson_lb(x['k_ok'], x['n']) for x in rs) >= thr:
            best = v
        else:
            break
    return best


def mean_d(route, v, cfg, first):
    """Mean distance from the timing detection to the action point (mm), over
    the lateral positions, as in bound_noise."""
    kw = CFG[cfg]
    T = kw['tau_i'] + kw['tp']
    ds = []
    for j in range(len(YS)):
        if route == 'P':
            lo, hi = np.interp(v, VS, S_LO[:, j]), np.interp(v, VS, S_HI[:, j])
            aim = 0.5 * (lo + hi)
        else:
            aim = np.interp(v, VS, O_MAX[:, j]) - 40.0
        if not np.isfinite(aim):
            continue
        W = min(v * T, LV)
        a = aim + GAP + LV - W if first else max(aim + GAP, v * (kw['tau_i'] + 1.0 / F_CAM + 0.015))
        ds.append(a + W / 2)
    return float(np.mean(ds))


if B2:
    for var, name in (('enc1', 'EncFirst'), ('vis1', 'TrkFirst'), ('encR', 'EncNew'),
                      ('enc1s10', 'EncFirstSig'), ('vis1s10', 'TrkFirstSig'), ('encRs10', 'EncNewSig')):
        vb = v99_b2(var)
        out[f'Bl{name}'] = f2(vb) if vb else 'none'
        print(f'baseline {var:8s} v99 = {vb}')
    pl = [B2[f'vis1s10|PLASTIC|{v}']['ok'] for v in VS if v <= TAB['sig10']['b4']]
    out['BlTrkFirstSigPmin'] = f'{100 * min(pl):.0f}'
    out['BlTrkFirstSigPmax'] = f'{100 * max(pl):.0f}'
    for c, name in (('D', 'D'), ('sig10', 'Sig')):
        bf = min(TAB[c]['b4'], bound_noise('P', first=True, **CFG[c]), bound_noise('O', first=True, **CFG[c]))
        out[f'BnnFirst{name}'] = f2(bf) if bf >= 50 else 'none'
        print(f'eq. (5) timed from the first detection, {c}: {bf:.0f} mm/s')
    # at low speed the first detection is several times farther from the action
    # point than the newest one; near bound (4) only one detection arrives in time
    out['DnewPten'] = f'{mean_d("P", 100, "D", False):.0f}'
    out['DfirstPten'] = f'{mean_d("P", 100, "D", True):.0f}'
    out['DnewOten'] = f'{mean_d("O", 100, "D", False):.0f}'
    out['DfirstOten'] = f'{mean_d("O", 100, "D", True):.0f}'
    for v in (100, 200, 300, 370):
        print(f'mean d at {v} mm/s: strike {mean_d("P", v, "D", False):.0f} (newest) vs '
              f'{mean_d("P", v, "D", True):.0f} (first); organic {mean_d("O", v, "D", False):.0f} vs '
              f'{mean_d("O", v, "D", True):.0f} mm')

# ---------------------------------------------------------------- figure 5 ---
fig, axs = plt.subplots(2, 1, figsize=(COL, 3.2), sharex=True,
                        gridspec_kw=dict(hspace=0.16, top=0.80))
vv = np.array(VS) / 1000
for ax, route, ttl, vb in ((axs[0], 'PLASTIC', '(a) plastic: side bin', TAB['D']['b4P']),
                           (axs[1], 'ORGANIC', '(b) organic: rear-right bin', TAB['D']['b4O'])):
    for pm, ls, mfc in ((0.0, '-', None), (0.1, '--', 'white')):
        p = [get('prop', 'D', pm, route, v)['ok'] for v in VS]
        ax.plot(vv, p, ls=ls, color=C['blue'], lw=1.1, label=f'event-driven, $p_m={pm:g}$')
        b = [get('base', 'D', pm, route, v)['ok'] for v in VB]
        ax.plot(np.array(VB) / 1000, b, ls='none', marker='o', ms=3.0, color=C['red'],
                mfc=mfc if mfc else C['red'], mew=0.7, label=f'first firmware, $p_m={pm:g}$')
    if route == 'PLASTIC':
        ks = sorted(BEST)
        ax.plot(np.array(ks) / 1000, [BEST[k][1] for k in ks], ls=':', marker='s', ms=2.6,
                color=C['gray'], lw=0.8, label='first firmware, best $t_d$')
    else:
        ax.plot(np.array(VB) / 1000, [bo(v)['dist']['P'] for v in VB], ls='none', marker='^',
                ms=3.0, color=C['orange'], label='first firmware, into plastic bin')
    ax.axvline(vb / 1000, color='k', lw=0.6, ls=(0, (1, 1.5)))
    ax.set_ylim(-0.03, 1.05)
    ax.set_ylabel('fraction of items')
    ax.set_title(ttl, loc='left', fontsize=7, pad=2)
    ax.grid(True, lw=0.3, color='#dddddd')
axs[1].set_xlabel('belt speed $v$ (m/s)')
axs[1].set_xlim(0.04, 0.61)
axs[1].set_xticks([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
h0, l0 = axs[0].get_legend_handles_labels()
h1, l1 = axs[1].get_legend_handles_labels()
order = [0, 2, 1, 3, 4]
fig.legend([h0[i] for i in order] + [h1[-1]], [l0[i] for i in order] + [l1[-1]],
           loc='upper center', bbox_to_anchor=(0.53, 1.0), ncol=2, fontsize=5.8,
           handlelength=1.8, columnspacing=1.0, borderaxespad=0.1)
fig.savefig('../figs/fig_mc.pdf')
fig.savefig('../figs/fig_mc.png', dpi=220)
plt.close(fig)

# ---------------------------------------------------------------- figure 6 ---
rng = np.random.default_rng(7)
fig, ax = plt.subplots(figsize=(COL, 2.05))
vg = np.linspace(0.025, 0.6, 400)
T0 = 0.4
for pm, col in ((0.0, C['red']), (0.1, C['orange']), (0.2, C['purple'])):
    ax.plot(vg, tm.regs_level_closed(vg * 1000, T0, 300.0, pm), color=col, lw=1.0,
            label=f'level, $p_m={pm:g}$')
    if pm > 0:
        ax.plot(vg, tm.regs_edge_closed(vg * 1000, T0, 300.0, pm), color=col, lw=0.9, ls='--',
                label=f'edge, $p_m={pm:g}$')
    ks = np.arange(50, 625, 50)
    lv, ed, pr = [], [], []
    for v in ks:
        _, rb_o, _ = tm.baseline('ORGANIC', 20000, float(v), T0, 300.0, 200.0, pm, rng,
                                 kinematics=False, tau_i=0.4)
        _, rb_p, _ = tm.baseline('PLASTIC', 20000, float(v), T0, 300.0, 200.0, pm, rng,
                                 kinematics=False, tau_i=0.4)
        S = tm.sample_items(20000, float(v), T0, 300.0, 200.0, pm, rng)
        lv.append(rb_o.mean()); ed.append(rb_p.mean()); pr.append(S['det'].any(axis=1).mean())
    ax.plot(ks / 1000, lv, 'o', ms=2.4, color=col, mfc='white', mew=0.6)
    if pm > 0:
        ax.plot(ks / 1000, ed, 's', ms=2.2, color=col, mfc='white', mew=0.6)
    if pm == 0.2:
        out['EdgeMax'] = f'{max(ed):.2f}'
    if pm == 0.1:
        prop_pts = pr
ax.plot(vg, tm.p_det_closed(vg * 1000, T0, 300.0, 0.1), color=C['blue'], lw=1.3,
        label='event-driven, $p_m=0.1$')
ax.plot(np.arange(50, 625, 50) / 1000, prop_pts, 'D', ms=2.2, color=C['blue'], mfc='white', mew=0.6)
ax.axhline(3, color='k', lw=0.6, ls=':')
ax.text(0.33, 3.2, 'bin-full threshold', ha='left', fontsize=6)
ax.set_yscale('log')
ax.set_yticks([0.5, 1, 2, 3, 5, 10, 20])
ax.set_yticklabels(['0.5', '1', '2', '3', '5', '10', '20'])
ax.set_ylim(0.45, 25)
ax.set_xlim(0.02, 0.61)
ax.set_xlabel('belt speed $v$ (m/s)')
ax.set_ylabel('registrations per item')
ax.grid(True, lw=0.3, color='#dddddd', which='both')
ax.legend(loc='upper right', fontsize=5.6, ncol=2, borderaxespad=0.2, handlelength=1.8,
          columnspacing=0.9)
fig.savefig('../figs/fig_reg.pdf')
fig.savefig('../figs/fig_reg.png', dpi=220)
plt.close(fig)

# -------------------------------------------------------- several items ---
if MUL:
    SP = (0, 100, 150, 200, 300, 450, 600)
    fig, axs = plt.subplots(1, 2, figsize=(COL, 1.85), gridspec_kw=dict(wspace=0.32))
    ax = axs[0]
    for pol, ls in (('fcfs', '-'), ('safe', '--')):
        for v, col in ((100, C['green']), (200, C['blue']), (300, C['red'])):
            def pair_stat(dx, stat):
                vals = [MUL[f'pair|{v}|{dx}|{a}{b}|0.0|{pol}'] for a in range(3) for b in range(3)
                        if f'pair|{v}|{dx}|{a}{b}|0.0|{pol}' in MUL]
                if stat == 'both':
                    return np.mean([x['both_ok'] for x in vals]) if vals else np.nan
                return np.mean([0.5 * (x['lead']['wrong'] + x['follow']['wrong']) for x in vals]) \
                    if vals else np.nan
            ax.plot(np.array(SP), [pair_stat(dx, 'both') for dx in SP], ls=ls, color=col, lw=1.0,
                    marker='o' if pol == 'fcfs' else None, ms=2.2,
                    label=f'{v / 1000:.1f} m/s' if pol == 'fcfs' else None)
    ax.set_xlabel('spacing (mm)')
    ax.set_ylabel('pairs fully sorted')
    ax.set_ylim(-0.02, 1.16)
    ax.set_yticks([0, 0.2, 0.4, 0.6, 0.8, 1.0])
    ax.set_title('(a) pairs, all class orders', loc='left', fontsize=6.5, pad=2)
    ax.grid(True, lw=0.3, color='#dddddd')
    ax.legend(fontsize=5.4, loc='lower right', borderaxespad=0.2, handlelength=1.5)
    ax = axs[1]
    RT = (10, 20, 30, 45, 60, 90)
    for pol, ls in (('fcfs', '-'), ('safe', '--')):
        for o, col in (('correct', C['blue']), ('reject', C['gray']), ('wrong', C['red'])):
            ys = [MUL.get(f'stream|200|{r}|0.0|{pol}', {}).get('all', {}).get(o, np.nan) for r in RT]
            ax.plot(RT, ys, ls=ls, color=col, lw=1.0, marker='o' if pol == 'fcfs' else None, ms=2.2,
                    label=o if pol == 'fcfs' else None)
    ax.set_xlabel('items per minute')
    ax.set_ylabel('fraction of items')
    ax.set_ylim(-0.02, 1.16)
    ax.set_yticks([0, 0.2, 0.4, 0.6, 0.8, 1.0])
    ax.set_title('(b) random stream, 0.2 m/s', loc='left', fontsize=6.5, pad=2)
    ax.grid(True, lw=0.3, color='#dddddd')
    ax.legend(fontsize=5.4, loc='upper right', ncol=3, borderaxespad=0.15, handlelength=1.3,
              columnspacing=0.7, handletextpad=0.4)
    fig.savefig('../figs/fig_multi.pdf')
    fig.savefig('../figs/fig_multi.png', dpi=220)
    plt.close(fig)

    def S(v, r, pol, pm=0.0):
        return MUL[f'stream|{v}|{r}|{pm}|{pol}']
    out['MiCorrTwenty'] = f"{100 * S(200, 20, 'fcfs')['all']['correct']:.0f}"
    out['MiWrongTwenty'] = f"{100 * S(200, 20, 'fcfs')['all']['wrong']:.1f}"
    out['MiCorrSixty'] = f"{100 * S(200, 60, 'fcfs')['all']['correct']:.0f}"
    out['MiWrongSixty'] = f"{100 * S(200, 60, 'fcfs')['all']['wrong']:.0f}"
    out['MiSafeWrongSixty'] = f"{100 * S(200, 60, 'safe')['all']['wrong']:.1f}"
    out['MiSafeCorrSixty'] = f"{100 * S(200, 60, 'safe')['all']['correct']:.0f}"
    out['MiSafeRejSixty'] = f"{100 * S(200, 60, 'safe')['all']['reject']:.0f}"
    out['MiMixed'] = f"{100 * max(S(v, r, p)['mixed'] for v in (100, 200, 300) for r in RT for p in ('fcfs', 'safe')):.2f}"
    out['MiRegs'] = f"{max(S(v, r, p)['regs_per_item'] for v in (100, 200, 300) for r in RT for p in ('fcfs', 'safe')):.3f}"
    # spacing needed for 99% of pairs fully sorted (FCFS), per speed
    for v, name in ((100, 'One'), (200, 'Two'), (300, 'Three')):
        need = None
        for dx in SP:
            vals = [MUL[f'pair|{v}|{dx}|{a}{b}|0.0|fcfs'] for a in range(3) for b in range(3)]
            if min(x['both_ok'] for x in vals) >= 0.99 and need is None:
                need = dx
            elif min(x['both_ok'] for x in vals) < 0.99:
                need = None
        out[f'MiSpace{name}'] = str(need) if need is not None else '>600'
    for v, name in ((100, 'One'), (200, 'Two'), (300, 'Three')):
        vals = [MUL[f'pair|{v}|300|{a}{b}|0.0|fcfs']['both_ok'] for a in range(3) for b in range(3)]
        out[f'MiPairTh{name}'] = f'{100 * np.mean(vals):.1f}'
        oo = MUL[f'pair|{v}|300|11|0.0|fcfs']['both_ok']
        out[f'MiPairOO{name}'] = f'{100 * oo:.0f}'
    out['MiWrongSafeMax'] = f"{100 * max(S(200, r, 'safe')['all']['wrong'] for r in RT):.1f}"
    out['MiWrongNinety'] = f"{100 * S(200, 90, 'fcfs')['all']['wrong']:.0f}"
    for k in [k for k in out if k.startswith('Mi')]:
        print(k, out[k])

# ------------------------------------------------------------ stepper load ---
ALPHA = TAU_STRIKE / J_GATE
MU_B = 0.5


def m_max(T_h, rc, J_rot=82e-7):
    """Item mass the strike can carry at contact radius rc (m) before the
    load torque (J + J_rot + m rc^2) alpha + mu m g rc exceeds T_h."""
    return (T_h - (J_GATE + J_rot) * ALPHA) / (rc ** 2 * ALPHA + MU_B * 9.81 * rc)


out['Alpha'] = f'{ALPHA:.0f}'
out['MmaxFull'] = f'{1000 * m_max(0.59, 0.090):.0f}'
out['MmaxSixty'] = f'{1000 * m_max(0.6 * 0.59, 0.090):.0f}'
out['MmaxNtwentythree'] = f'{1000 * m_max(1.26, 0.090, J_rot=280e-7):.0f}'
out['MmaxNtwentythreeL'] = f'{1000 * m_max(1.90, 0.090, J_rot=480e-7):.0f}'
# the same motor on the TB6600 at 2.2 A (holding torque scaled with the current)
out['MmaxNtwentythreeTB'] = f'{1000 * m_max(1.90 * 2.2 / 2.8, 0.090, J_rot=480e-7):.0f}'
SL = json.load(open('results/stepper_load.json')) if os.path.exists('results/stepper_load.json') else None
if SL:
    out['RcMinAim'] = f"{min(SL['rc']):.0f}"
    out['RcMaxAim'] = f"{max(SL['rc']):.0f}"
print('stepper', {k: out[k] for k in out if k.startswith(('Mmax', 'Alpha', 'Rc'))})

json.dump(dict(out=out, tab=TAB, four=FOUR), open('results/numbers_v3.json', 'w'), indent=1,
          default=float)
with open('../paper/numbers_v3.tex', 'w') as f:
    f.write('% generated by sim/gen_v3.py from sim/results/v3*.json\n')
    for k, v in out.items():
        f.write(f'\\providecommand{{\\{k}}}{{}}\\renewcommand{{\\{k}}}{{{v}}}\n')
print('numbers', len(out))
