"""Stronger baselines: every action timed once from the first detection (a fixed
delay), with the belt speed known exactly (encoder) or estimated from the
tracks, and the event-driven controller, which re-plans from the newest
detection, with an encoder.  Design point: tau_i = T = 0.4 s, L_v = 300 mm,
g = 200 mm, p_m = 0; centroid noise sigma = 5 mm with a +-5 % speed error, and
sigma = 10 mm with +-10 % (the last row of the design-envelope table).
The event-driven controller with the speed from the tracks is prop|D and
prop|sig10 in results/v3.json.  gen_v3.py reads the results.
-> results/v3_baseline2.json"""
import json, os, pickle, time, zlib
from multiprocessing import Pool
import numpy as np
import timing as tm

CW = pickle.load(open('results/cw_v3.pkl', 'rb'))
VS = sorted(CW)
N = int(os.environ.get('NTRIALS', 4000))
OUT = 'results/v3_baseline2.json'
VAR = {
    'enc1': dict(replan=False, eps_v=0.0),    # fixed delay from first detection, encoder speed
    'vis1': dict(replan=False, eps_v=0.05),   # same, speed estimated from the tracks
    'encR': dict(replan=True, eps_v=0.0),     # event-driven controller with encoder speed
    # doubled centroid noise (sigma = 10 mm), as in the last row of the design-envelope table
    'enc1s10': dict(replan=False, eps_v=0.0, sig_x=10.0),
    'vis1s10': dict(replan=False, eps_v=0.10, sig_x=10.0),
    'encRs10': dict(replan=True, eps_v=0.0, sig_x=10.0),
}


def run(j):
    var, route, v = j
    k = f'{var}|{route}|{v}'
    rng = np.random.default_rng(zlib.crc32(k.encode()))
    t0 = time.time()
    o, rg, d = tm.proposed(route, N, float(v), 0.4, 300.0, 200.0, 0.0, rng, cw=CW[v],
                           tau_i=0.4, **VAR[var])
    tgt = 'P' if route == 'PLASTIC' else 'O'
    return k, dict(n=N, ok=float(np.mean(o == tgt)), k_ok=int(np.sum(o == tgt)),
                   dist={x: float(np.mean(o == x)) for x in 'PBXOR'},
                   sec=round(time.time() - t0, 2))


if __name__ == '__main__':
    R = json.load(open(OUT)) if os.path.exists(OUT) else {}
    todo = [(a, r, v) for a in VAR for r in ('PLASTIC', 'ORGANIC') for v in VS
            if f'{a}|{r}|{v}' not in R]
    print('jobs', len(todo), flush=True)
    t0 = time.time()
    with Pool(2) as p:
        for i, (k, res) in enumerate(p.imap_unordered(run, todo, chunksize=2)):
            R[k] = res
            if i % 20 == 0:
                json.dump(R, open(OUT, 'w'))
                print(f'[{time.time() - t0:6.0f}s] {i + 1}/{len(todo)} {k} ok={res["ok"]:.4f}', flush=True)
    json.dump(R, open(OUT, 'w'))
    print('DONE', round(time.time() - t0), 's', flush=True)
