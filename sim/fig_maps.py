"""Kinematic outcome maps and example trajectories at v = 0.2 m/s."""
import math, pickle
import numpy as np
from figstyle import plt, C, COL
from matplotlib.colors import ListedColormap
from matplotlib.patches import Polygon, Rectangle, Circle, Patch
from params import *

d = np.load('results/maps.npz')
xm, ym, o_s, o_o = d['xm'], d['ym'], d['o_s'], d['o_o']
codes = ['P', 'B', 'X', 'O', 'R', 'N']
cols = ['#c0392b', '#222222', '#b5b5b5', '#2e8b57', '#8e44ad', '#ffffff']
cmap = ListedColormap(cols)


def to_idx(o):
    m = np.zeros(o.shape)
    for i, c in enumerate(codes):
        m[o == c] = i
    m[o == 'N'] = codes.index('X')      # untouched items keep the rest route
    return m


fig = plt.figure(figsize=(COL, 3.95))
axs = [fig.add_axes([0.13, 0.790, 0.85, 0.175]),
       fig.add_axes([0.13, 0.555, 0.85, 0.175]),
       fig.add_axes([0.13, 0.055, 0.60, 0.36])]
for ax, o, ttl in ((axs[0], o_s, '(a) plastic strike: centroid at stroke start'),
                   (axs[1], o_o, '(b) organic move: centroid at move start')):
    ax.imshow(to_idx(o), origin='lower', aspect='auto', cmap=cmap, vmin=-0.5, vmax=5.5,
              extent=[xm[0] - 1, xm[-1] + 1, ym[0], ym[-1]], interpolation='nearest')
    ax.set_title(ttl, loc='left', fontsize=7)
    ax.set_ylabel('$y_0$ (mm)')
    ax.set_xlim(-300, 60)
    ax.set_yticks([-35, 0, 35])
axs[1].set_xlabel('$x_0$ (mm)', labelpad=1)
axs[0].set_xticklabels([])
leg = [Patch(fc=cols[i], ec='k', lw=0.3, label=l) for i, l in
       enumerate(['side bin', 'post', 'rear-left', 'rear-right'])]
leg_maps = leg

# trajectories
from kin import Segments, simulate, seq_strike
ex = {}
for name, xx in (('hit', -104.0), ('blocked', -60.0), ('early', -190.0), ('late', -20.0)):
    seg = Segments(1, 3)
    seq_strike(seg, np.arange(1), 0.0)
    rr = simulate(seg, np.array([xx]), np.array([0.0]), 65.0, 200.0, record=True)
    H = rr['hist']
    xs = [q[2][0] for q in H]
    ys = [q[3][0] for q in H]
    st = [q[4][0] for q in H]
    # cut at release and append the Coulomb slide on the belt
    if 3 in st:
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
    ex[name] = dict(out=str(rr['out'][0]), x=xs, y=ys)
ax = axs[2]
ax.add_patch(Rectangle((-300, -POST_INNER), 420, 2 * POST_INNER, fc='#eef4ef', ec='none'))
for s in (1, -1):
    ax.add_patch(Rectangle((-POST_X / 2, POST_INNER if s > 0 else -POST_INNER - POST_X), POST_X,
                           POST_X, fc=C['orange'], ec='k', lw=0.4, zorder=4))
ax.plot([-300, 120], [POST_INNER] * 2, color=C['green'], lw=0.6)
ax.plot([-300, 120], [-POST_INNER] * 2, color=C['green'], lw=0.6)


def plate(th, **kw):
    u = np.array([math.cos(th), math.sin(th)])
    n = np.array([-math.sin(th), math.cos(th)])
    p = [(-R_HALF * u + 2.5 * n), (R_HALF * u + 2.5 * n), (R_HALF * u - 2.5 * n), (-R_HALF * u - 2.5 * n)]
    ax.add_patch(Polygon(p, closed=True, **kw))


for th, a in ((TH_REST, 0.9), (math.radians(22.5), 0.35), (0.0, 0.35), (TH_ORG, 0.35)):
    plate(th, fc=C['blue'], ec='none', alpha=a, zorder=3)
colmap = dict(hit=C['red'], blocked='#222222', early=C['green'], late='#7a7a7a')
lab = dict(hit='hit: side bin', blocked='near hub: post', early='early: rear-right',
           late='late: rear-left')
for k in ('early', 'hit', 'blocked', 'late'):
    e = ex[k]
    x, y = np.array(e['x']), np.array(e['y'])
    ax.plot(x, y, color=colmap[k], lw=1.0, label=lab[k], zorder=5)
    ax.plot(x[0], y[0], 'o', ms=2.5, color=colmap[k], zorder=6)
    ax.plot(x[-1], y[-1], marker='x', ms=3.5, color=colmap[k], zorder=6)
ax.set_xlim(-300, 120)
ax.set_ylim(-150, 150)
ax.set_aspect('equal')
ax.set_xlabel('$x$ (mm)')
ax.set_ylabel('$y$ (mm)')
ax.set_title('(c) item paths during one strike sequence', loc='left', fontsize=7)
l1 = ax.legend(loc='center left', bbox_to_anchor=(1.02, 0.28), fontsize=5.8, ncol=1,
               handlelength=1.4, borderaxespad=0.0, title='(c) paths', title_fontsize=5.8)
ax.add_artist(l1)
ax.legend(handles=leg_maps, loc='center left', bbox_to_anchor=(1.02, 0.80), fontsize=5.8,
          ncol=1, handlelength=1.0, borderaxespad=0.0, title='(a), (b) outcome', title_fontsize=5.8)
fig.savefig('../figs/fig_maps.pdf')
fig.savefig('../figs/fig_maps.png', dpi=220)
print('ok')
