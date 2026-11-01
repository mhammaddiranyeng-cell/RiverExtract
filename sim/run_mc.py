"""Monte Carlo experiments (re-uses the kinematic capture windows)."""
import json, os, pickle, sys, time
import numpy as np
import timing as tm

OUT = 'results'
R = json.load(open(os.path.join(OUT, 'results.json')))
CW = pickle.load(open(os.path.join(OUT, 'cw.pkl'), 'rb'))
rng = np.random.default_rng(1012)
t0 = time.time()


def log(*a):
    print(f'[{time.time() - t0:7.1f}s]', *a, flush=True)


def save():
    json.dump(R, open(os.path.join(OUT, 'results.json'), 'w'), indent=1, default=float)


T0, LV, GAP = 1.0, 300.0, 200.0
VS = [50, 75, 100, 125, 150, 175, 200, 225, 250, 275, 300, 350, 400]
N = int(sys.argv[1]) if len(sys.argv) > 1 else 3000

# --------------------------------------------------- registrations only ---
reg = {}
for pm in (0.0, 0.1, 0.2):
    for v in np.arange(25, 425, 25):
        _, rb_o, d = tm.baseline('ORGANIC', 20000, float(v), T0, LV, GAP, pm, rng, kinematics=False)
        _, rb_p, _ = tm.baseline('PLASTIC', 20000, float(v), T0, LV, GAP, pm, rng, kinematics=False)
        S = tm.sample_items(20000, float(v), T0, LV, GAP, pm, rng)
        reg[f'{pm}|{int(v)}'] = dict(level=float(rb_o.mean()), edge=float(rb_p.mean()),
                                     prop=float(S['det'].any(axis=1).mean()),
                                     full1=float(np.mean(rb_o >= 3)))
log('registrations done')
R['reg'] = reg
save()

# ------------------------------------------------------- main MC ---
mc = {}
for pm in (0.0, 0.1):
    for v in VS:
        cw = CW[v]
        for route in ('PLASTIC', 'ORGANIC'):
            o, rg, d = tm.proposed(route, N, float(v), T0, LV, GAP, pm, rng, cw=cw)
            ob, rb, db = tm.baseline(route, N, float(v), T0, LV, GAP, pm, rng)
            tgt = 'P' if route == 'PLASTIC' else 'O'
            key = f'{route}|{pm}|{v}'
            mc[key] = dict(
                prop=dict(ok=float(np.mean(o == tgt)), regs=float(rg.mean()), det=float(d.mean()),
                          dist={c: float(np.mean(o == c)) for c in 'PBXOR'}),
                base=dict(ok=float(np.mean(ob == tgt)), regs=float(rb.mean()), det=float(db.mean()),
                          dist={c: float(np.mean(ob == c)) for c in 'PBXOR'}))
            log('mc', key, round(mc[key]['prop']['ok'], 4), round(mc[key]['base']['ok'], 4),
                {c: round(x, 4) for c, x in mc[key]['prop']['dist'].items() if x > 0})
R['mc'] = mc
R['mc_cfg'] = dict(T=T0, Lv=LV, g=GAP, N=N, w=65.0, sig_x=5.0, eps_v=0.05, m=1)
save()

# ------------------------------------------ baseline, best fixed delay ---
best = {}
TDS = np.round(np.arange(0.0, 1.31, 0.1), 2)
for v in [50, 100, 150, 200, 250, 300]:
    accs = []
    for td in TDS:
        ob, rb, db = tm.baseline('PLASTIC', 1500, float(v), T0, LV, GAP, 0.0, rng, t_d=float(td))
        accs.append(float(np.mean(ob == 'P')))
    k = int(np.argmax(accs))
    best[v] = dict(td=float(TDS[k]), ok=accs[k], all=accs)
    log('best delay', v, best[v]['td'], round(best[v]['ok'], 3))
R['base_best'] = best
save()

# ------------------------------------------------ envelope variants ---
env = {}
for tag, kw in (('T0.8', dict(T=0.8)), ('T1.2', dict(T=1.2)), ('m2', dict(T=1.0, m=2)),
                ('Lv400', dict(T=1.0, Lv=400.0)), ('sig10', dict(T=1.0, sig_x=10.0, eps_v=0.10))):
    for v in VS:
        cw = CW[v]
        for route in ('PLASTIC', 'ORGANIC'):
            T = kw.get('T', 1.0)
            Lv = kw.get('Lv', LV)
            extra = {k: kw[k] for k in ('m', 'sig_x', 'eps_v') if k in kw}
            o, rg, d = tm.proposed(route, 1500, float(v), T, Lv, GAP, 0.0, rng, cw=cw, **extra)
            tgt = 'P' if route == 'PLASTIC' else 'O'
            env[f'{route}|{tag}|{v}'] = float(np.mean(o == tgt))
        log('env', tag, v, env[f'PLASTIC|{tag}|{v}'], env[f'ORGANIC|{tag}|{v}'])
    R['env'] = env
    save()
log('ALL DONE')
