"""v3 Monte Carlo: loop-period sweep (pacing removed), Table IV envelope on a
10 mm/s grid, first-firmware comparison at the new design point, outcome
distributions conditional on commitment (for the misclassification model).
-> results/v3.json   (incremental; re-running skips finished keys)"""
import json, os, pickle, sys, time, zlib
from multiprocessing import Pool
import numpy as np
import timing as tm

CW = pickle.load(open('results/cw_v3.pkl', 'rb'))
VS = sorted(CW)
N = int(os.environ.get('NTRIALS', 4000))
LV, GAP = 300.0, 200.0
OUT = 'results/v3.json'

# configurations: loop period T = tau_i + t_p  (t_p = pacing sleep)
CFG = {
    'D':     dict(tau_i=0.40, tp=0.0),                    # design point, pacing removed
    'P3':    dict(tau_i=0.40, tp=0.3),
    'P6':    dict(tau_i=0.40, tp=0.6),                    # pacing of the first firmware
    'I25':   dict(tau_i=0.25, tp=0.0),                    # faster (INT8) inference
    'Lv400': dict(tau_i=0.40, tp=0.0, Lv=400.0),
    'm2':    dict(tau_i=0.40, tp=0.0, m=2),
    'sig10': dict(tau_i=0.40, tp=0.0, sig_x=10.0, eps_v=0.10),
}


def job_list():
    J = []
    for c in CFG:
        for pm in ((0.0, 0.1) if c == 'D' else (0.0,)):
            for route in ('PLASTIC', 'ORGANIC'):
                for v in VS:
                    J.append(('prop', c, pm, route, v, None))
    for pm in (0.0, 0.1):                                   # first firmware at D
        for route in ('PLASTIC', 'ORGANIC'):
            for v in range(50, 610, 25):
                J.append(('base', 'D', pm, route, v, None))
    for v in (50, 100, 150, 200, 250, 300, 400):            # best fixed delay at D
        for td in np.round(np.arange(0.0, 1.31, 0.1), 2):
            J.append(('best', 'D', 0.0, 'PLASTIC', v, float(td)))
    return J


def key(j):
    ctl, c, pm, route, v, td = j
    return f'{ctl}|{c}|{pm}|{route}|{v}' + (f'|{td}' if td is not None else '')


def run(j):
    ctl, c, pm, route, v, td = j
    k = key(j)
    rng = np.random.default_rng(zlib.crc32(k.encode()))
    cf = dict(CFG[c])
    tau_i, tp = cf.pop('tau_i'), cf.pop('tp')
    T = tau_i + tp
    Lv = cf.pop('Lv', LV)
    t0 = time.time()
    if ctl == 'prop':
        o, rg, d = tm.proposed(route, N, float(v), T, Lv, GAP, pm, rng, cw=CW[v],
                               tau_i=tau_i, **cf)
        tgt = 'P' if route == 'PLASTIC' else 'O'
        res = dict(n=N, ok=float(np.mean(o == tgt)), k_ok=int(np.sum(o == tgt)),
                   det=float(d.mean()), regs=float(rg.mean()),
                   dist={x: float(np.mean(o == x)) for x in 'PBXOR'},
                   dist_dec={x: float(np.mean(o[d] == x)) if d.any() else 0.0 for x in 'PBXOR'})
    elif ctl == 'base':
        o, rb, db = tm.baseline(route, 1500, float(v), T, Lv, GAP, pm, rng, tau_i=tau_i)
        tgt = 'P' if route == 'PLASTIC' else 'O'
        res = dict(n=1500, ok=float(np.mean(o == tgt)), det=float(db.mean()),
                   regs=float(rb.mean()), dist={x: float(np.mean(o == x)) for x in 'PBXOR'})
    else:
        o, rb, db = tm.baseline('PLASTIC', 1500, float(v), T, Lv, GAP, 0.0, rng, t_d=td,
                                tau_i=tau_i)
        res = dict(n=1500, ok=float(np.mean(o == 'P')))
    res['sec'] = round(time.time() - t0, 2)
    return k, res


if __name__ == '__main__':
    R = json.load(open(OUT)) if os.path.exists(OUT) else {}
    todo = [j for j in job_list() if key(j) not in R]
    # longest first: low speeds of the first firmware are the slowest jobs
    todo.sort(key=lambda j: (j[0] != 'base', j[4]))
    print('jobs', len(todo), 'N', N, flush=True)
    t0, last = time.time(), time.time()
    with Pool(2) as p:
        for i, (k, res) in enumerate(p.imap_unordered(run, todo, chunksize=1)):
            R[k] = res
            if time.time() - last > 20 or i == len(todo) - 1:
                json.dump(R, open(OUT, 'w'))
                last = time.time()
            print(f'[{time.time() - t0:7.1f}s] {i + 1}/{len(todo)} {k} ok={res["ok"]:.4f} '
                  f'({res["sec"]}s)', flush=True)
    json.dump(R, open(OUT, 'w'))
    print('MC DONE', flush=True)
