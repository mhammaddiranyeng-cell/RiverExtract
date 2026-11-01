"""Monte Carlo figures: routing success and registrations."""
import json
import numpy as np
from figstyle import plt, C, COL
import timing as tm

R = json.load(open('results/results.json'))
N = json.load(open('results/numbers.json'))
VS = [50, 75, 100, 125, 150, 175, 200, 225, 250, 275, 300, 350, 400]
vv = np.array(VS) / 1000.0

# ------------------------------------------------------------ routing ---
fig, axs = plt.subplots(2, 1, figsize=(COL, 3.05), sharex=True,
                        gridspec_kw=dict(hspace=0.16, top=0.83))
for ax, route, ttl, vb in ((axs[0], 'PLASTIC', '(a) plastic: side bin', float(N['VcfP'])),
                           (axs[1], 'ORGANIC', '(b) organic: rear-right bin', float(N['VcfO']))):
    for pm, ls, mfc in ((0.0, '-', None), (0.1, '--', 'white')):
        p = [R['mc'][f'{route}|{pm}|{v}']['prop']['ok'] for v in VS]
        b = [R['mc'][f'{route}|{pm}|{v}']['base']['ok'] for v in VS]
        ax.plot(vv, p, ls=ls, color=C['blue'], lw=1.1,
                label=f'event-driven, $p_m={pm:g}$')
        ax.plot(vv, b, ls='none', marker='o', ms=3.2, color=C['red'],
                mfc=mfc if mfc else C['red'], mew=0.7,
                label=f'first firmware, $p_m={pm:g}$')
    if route == 'PLASTIC':
        bb = R['base_best']
        vb_ = sorted(int(k) for k in bb)
        ax.plot(np.array(vb_) / 1000, [bb[str(k)]['ok'] for k in vb_], ls=':', marker='s', ms=2.6,
                color=C['gray'], lw=0.8, label='first firmware, best $t_d$')
        th = [min(1.0, N['Lc_design'] / (v * 1.0)) for v in np.linspace(50, 400, 200)]
    if route == 'ORGANIC':
        cont = [R['mc'][f'ORGANIC|0.0|{v}']['base']['dist']['P'] for v in VS]
        ax.plot(vv, cont, ls='none', marker='^', ms=3.0, color=C['orange'],
                label='first firmware, into plastic bin')
    ax.axvline(vb, color='k', lw=0.6, ls=(0, (1, 1.5)))
    ax.set_ylim(-0.03, 1.05)
    ax.set_ylabel('fraction of items')
    ax.set_title(ttl, loc='left', fontsize=7, pad=2)
    ax.grid(True, lw=0.3, color='#dddddd')
axs[1].set_xlabel('belt speed $v$ (m/s)')
axs[1].set_xlim(0.04, 0.41)
axs[1].set_xticks([0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40])
axs[1].tick_params(labelbottom=True)
h0, l0 = axs[0].get_legend_handles_labels()
h1, l1 = axs[1].get_legend_handles_labels()
order = [0, 2, 1, 3, 4]
fig.legend([h0[i] for i in order], [l0[i] for i in order], loc='upper center',
           bbox_to_anchor=(0.53, 1.0), ncol=2, fontsize=5.8, handlelength=1.8,
           columnspacing=1.0, borderaxespad=0.1)
axs[1].legend([h1[-1]], [l1[-1]], loc='upper right', fontsize=5.6, borderaxespad=0.2)
fig.savefig('../figs/fig_mc.pdf')
fig.savefig('../figs/fig_mc.png', dpi=220)

# ------------------------------------------------------- registrations ---
fig, ax = plt.subplots(figsize=(COL, 2.05))
vg = np.linspace(0.025, 0.4, 400)
for pm, col in ((0.0, C['red']), (0.1, C['orange']), (0.2, C['purple'])):
    ax.plot(vg, tm.regs_level_closed(vg * 1000, 1.0, 300.0, pm), color=col, lw=1.0,
            label=f'level, $p_m={pm:g}$')
    if pm > 0:
        ax.plot(vg, tm.regs_edge_closed(vg * 1000, 1.0, 300.0, pm), color=col, lw=0.9, ls='--',
                label=f'edge, $p_m={pm:g}$')
    ks = sorted(int(k.split('|')[1]) for k in R['reg'] if k.startswith(f'{pm}|'))
    ax.plot(np.array(ks) / 1000, [R['reg'][f'{pm}|{k}']['level'] for k in ks], 'o', ms=2.4,
            color=col, mfc='white', mew=0.6)
    if pm > 0:
        ax.plot(np.array(ks) / 1000, [R['reg'][f'{pm}|{k}']['edge'] for k in ks], 's', ms=2.2,
                color=col, mfc='white', mew=0.6)
ax.plot(vg, tm.p_det_closed(vg * 1000, 1.0, 300.0, 0.1), color=C['blue'], lw=1.3,
        label='event-driven, $p_m=0.1$')
ks = sorted(int(k.split('|')[1]) for k in R['reg'] if k.startswith('0.1|'))
ax.plot(np.array(ks) / 1000, [R['reg'][f'0.1|{k}']['prop'] for k in ks], 'D', ms=2.2,
        color=C['blue'], mfc='white', mew=0.6)
ax.axhline(3, color='k', lw=0.6, ls=':')
ax.text(0.155, 3.2, 'bin-full threshold', ha='left', fontsize=6)
ax.set_yscale('log')
ax.set_yticks([0.5, 1, 2, 3, 5, 10])
ax.set_yticklabels(['0.5', '1', '2', '3', '5', '10'])
ax.set_ylim(0.45, 13)
ax.set_xlim(0.02, 0.405)
ax.set_xlabel('belt speed $v$ (m/s)')
ax.set_ylabel('registrations per item')
ax.grid(True, lw=0.3, color='#dddddd', which='both')
ax.legend(loc='upper right', fontsize=5.6, ncol=2, borderaxespad=0.2, handlelength=1.8,
          columnspacing=0.9)
fig.savefig('../figs/fig_reg.pdf')
fig.savefig('../figs/fig_reg.png', dpi=220)
print('ok')
