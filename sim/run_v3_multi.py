"""v3: several items on the belt (pairs at set spacings, random streams).
-> results/v3_multi.json"""
import json, os, pickle, sys, time, zlib
from multiprocessing import Pool
import numpy as np
import multi as M

CW = pickle.load(open('results/cw_v3.pkl', 'rb'))
T, TAU = 0.4, 0.4                     # design point: pacing removed
SPEEDS = (100, 200, 300)
SPACINGS = (0, 100, 150, 200, 300, 450, 600)
NPAIR = int(os.environ.get('NPAIR', 200))
RATES = (10, 20, 30, 45, 60, 90)      # items per minute
NSTREAM, LSTREAM = 30, 40
OUT = 'results/v3_multi.json'
YMAX = M.BAND - 65.0 / 2


def pair_job(args):
    v, dx, a, b, pm, pol = args
    key = f'pair|{v}|{dx}|{a}{b}|{pm}|{pol}'
    rng = np.random.default_rng(zlib.crc32(key.encode()))
    cw = CW[v]
    items, cls, diag = [], [], []
    for _ in range(NPAIR):
        if dx == 0:   # side by side in one cleat pocket
            s = rng.choice([-1, 1])
            ys = np.array([s * rng.uniform(32.6, YMAX), -s * rng.uniform(32.6, YMAX)])
        else:
            ys = rng.uniform(-YMAX, YMAX, 2)
        xs = np.array([-700.0, -700.0 - dx])          # item 0 leads
        c = np.array([a, b])
        seqs, d = M.run_stream(xs, ys, c, float(v), rng, cw, T=T, tau_i=TAU, pm=pm, policy=pol)
        for i in range(2):
            items.append((xs[i], ys[i], M.relevant(seqs, xs[i], float(v))))
        cls += [a, b]
        diag.append(d)
    out = M.contact(items, float(v))
    cls = np.array(cls)
    oc = M.outcomes(cls, out)
    res = dict(lead={k: float(np.mean(x[0::2])) for k, x in oc.items()},
               follow={k: float(np.mean(x[1::2])) for k, x in oc.items()},
               both_ok=float(np.mean(oc['correct'][0::2] & oc['correct'][1::2])),
               mixed=float(np.mean([dd['mixed'].any() for dd in diag])),
               frag=float(np.mean([(dd['n_tracks'] > 1).any() for dd in diag])),
               drops=float(np.mean([dd['n_drop'] for dd in diag])),
               merges=float(np.mean([dd['n_merge'] for dd in diag])),
               conf=float(np.mean([dd['n_conf'] for dd in diag])),
               regs=float(np.mean([dd['regs'] for dd in diag])))
    return key, res


def stream_job(args):
    v, rate, pm, pol = args
    key = f'stream|{v}|{rate}|{pm}|{pol}'
    rng = np.random.default_rng(zlib.crc32(key.encode()))
    cw = CW[v]
    mean_gap = v * 60.0 / rate                       # mm between items
    items, cls, diag, span = [], [], [], []
    for _ in range(NSTREAM):
        gaps = 70.0 + rng.exponential(max(mean_gap - 70.0, 1.0), LSTREAM - 1)
        xs = -700.0 - np.concatenate([[0.0], np.cumsum(gaps)])
        ys = rng.uniform(-YMAX, YMAX, LSTREAM)
        c = rng.integers(0, 3, LSTREAM)
        seqs, d = M.run_stream(xs, ys, c, float(v), rng, cw, T=T, tau_i=TAU, pm=pm, policy=pol)
        for i in range(LSTREAM):
            items.append((xs[i], ys[i], M.relevant(seqs, xs[i], float(v))))
        cls.append(c)
        diag.append(d)
        span.append((xs[0] - xs[-1]) / v)               # stream duration (s)
    out = M.contact(items, float(v))
    cls = np.concatenate(cls)
    oc = M.outcomes(cls, out)
    res = dict(all={k: float(np.mean(x)) for k, x in oc.items()},
               by_cls={str(M.CLS[c]): {k: float(np.mean(x[cls == c])) for k, x in oc.items()}
                       for c in range(3)},
               rate_real=float(np.mean([(LSTREAM - 1) / s * 60 for s in span])),
               mixed=float(np.mean(np.concatenate([dd['mixed'] for dd in diag]))),
               frag=float(np.mean(np.concatenate([dd['n_tracks'] > 1 for dd in diag]))),
               regs_per_item=float(np.sum([dd['regs'] for dd in diag]) / len(cls)),
               drop_frac=float(np.sum([dd['n_drop'] for dd in diag]) /
                               max(1, np.sum([dd['n_req'] for dd in diag]))),
               conf_frac=float(np.sum([dd['n_conf'] for dd in diag]) /
                               max(1, np.sum([dd['n_req'] for dd in diag]))))
    return key, res


def dispatch(j):
    return pair_job(j[1]) if j[0] == 'pair' else stream_job(j[1])


if __name__ == '__main__':
    R = json.load(open(OUT)) if os.path.exists(OUT) else {}
    jobs = []
    for v in SPEEDS:
        for dx in SPACINGS:
            for a in range(3):
                for b in range(3):
                    for pol in ('fcfs', 'safe'):
                        jobs.append(('pair', (v, dx, a, b, 0.0, pol)))
        for rate in RATES:
            for pm in (0.0, 0.1):
                for pol in ('fcfs', 'safe'):
                    jobs.append(('stream', (v, rate, pm, pol)))
    keyf = lambda j: (f'pair|{j[1][0]}|{j[1][1]}|{j[1][2]}{j[1][3]}|{j[1][4]}|{j[1][5]}'
                      if j[0] == 'pair' else f'stream|{j[1][0]}|{j[1][1]}|{j[1][2]}|{j[1][3]}')
    todo = [j for j in jobs if keyf(j) not in R]
    print('jobs', len(todo), flush=True)
    t0 = time.time()

    with Pool(2) as p:
        for i, (k, res) in enumerate(p.imap_unordered(dispatch, todo)):
            R[k] = res
            json.dump(R, open(OUT, 'w'))
            print(f'[{time.time() - t0:7.1f}s] {i + 1}/{len(todo)} {k}', flush=True)
    print('MULTI DONE', flush=True)
