"""v3: capture windows on a 10 mm/s speed grid (50..600 mm/s) -> results/cw_v3.pkl"""
import os, pickle, sys, time
from multiprocessing import Pool
import numpy as np
import timing as tm

VS = list(range(50, 610, 10))
OUT = 'results/cw_v3.pkl'


def one(v):
    t = time.time()
    c = tm.capture_windows(float(v))
    print(f'cw {v} {time.time() - t:.1f}s lo={np.round(c["s_lo"])} hi={np.round(c["s_hi"])} '
          f'omax={np.round(c["o_max"])}', flush=True)
    return v, c


if __name__ == '__main__':
    done = pickle.load(open(OUT, 'rb')) if os.path.exists(OUT) else {}
    todo = [v for v in VS if v not in done]
    with Pool(2) as p:
        for v, c in p.imap_unordered(one, todo):
            done[v] = c
            pickle.dump(done, open(OUT, 'wb'))
    print('CW DONE', len(done), flush=True)
