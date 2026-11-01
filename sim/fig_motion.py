"""Gate motion profiles: proposed ramped moves vs as-built constant rate."""
import math
import numpy as np
from figstyle import plt, C, COL
from params import *
from dynamics import gate_inertia, profile, step_schedule_tri

J = gate_inertia()['J']
t = np.linspace(0, 0.62, 3000)
D = math.pi / 2

fig, (a1, a2) = plt.subplots(2, 1, figsize=(COL, 2.8), sharex=True,
                             gridspec_kw=dict(hspace=0.12, top=0.86))
cases = [
    ('strike, ramped', dict(kind='tri', tau=TAU_STRIKE), C['red'], '-'),
    ('organic, ramped', dict(kind='tri', tau=TAU_ORG), C['blue'], '-'),
    ('return, capped', dict(kind='trap', tau=TAU_RET, wmax=OMEGA_RET), C['green'], '-'),
    ('first firmware, 500 step/s', dict(kind='const', rate=F_FAST), C['red'], ':'),
    ('first firmware, 250 step/s', dict(kind='const', rate=F_SLOW), C['blue'], ':'),
]
for lab, kw, col, ls in cases:
    kind = kw.pop('kind')
    th, om, tm = profile(kind, t, D, J=J, **kw)
    a1.plot(t, np.degrees(th), color=col, ls=ls, label=lab)
    a2.plot(t, om, color=col, ls=ls)
    if kind != 'const':
        a1.plot([tm], [90], marker='o', ms=2.5, color=col)
# step instants of the strike (every 5th step)
ts, tm = step_schedule_tri(STEPS_90, J, TAU_STRIKE)
a1.plot(ts[4::5], np.degrees(np.arange(5, 51, 5) * STEP), ls='none', marker='|', ms=4,
        color=C['red'], mew=0.6)
a1.set_ylabel(r'$\Delta\theta$ (deg)')
a1.set_yticks([0, 30, 60, 90])
a1.set_ylim(0, 98)
h, l = a1.get_legend_handles_labels()
fig.legend(h, l, loc='upper center', bbox_to_anchor=(0.54, 1.0), fontsize=5.9, handlelength=2.0,
           ncol=3, columnspacing=0.9, borderaxespad=0.1)
a2.set_ylabel(r'$\omega$ (rad/s)')
a2.set_xlabel('time (s)')
a2.set_xlim(0, 0.62)
a2.set_ylim(0, 24)
a2.annotate('step to 15.7 rad/s', xy=(0.003, 15.7), xytext=(0.19, 17.0), fontsize=6,
            arrowprops=dict(arrowstyle='->', lw=0.5))
a2.annotate('step to 7.9 rad/s', xy=(0.003, 7.85), xytext=(0.215, 9.6), fontsize=6,
            arrowprops=dict(arrowstyle='->', lw=0.5))
for a in (a1, a2):
    a.grid(True, lw=0.3, color='#dddddd')
fig.savefig('../figs/fig_motion.pdf')
fig.savefig('../figs/fig_motion.png', dpi=200)
print('ok', tm)
