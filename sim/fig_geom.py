"""Plan view and section of the sorting stage, to scale (mm)."""
import math
import numpy as np
from figstyle import plt, C, COL, DCOL
from matplotlib.patches import Rectangle, Wedge, Polygon, FancyArrowPatch, Circle
from params import *

LV, GAP = 300.0, 200.0


def plate(ax, th, **kw):
    u = np.array([math.cos(th), math.sin(th)])
    n = np.array([-math.sin(th), math.cos(th)])
    p = [(-R_HALF * u + 2.5 * n), (R_HALF * u + 2.5 * n), (R_HALF * u - 2.5 * n), (-R_HALF * u - 2.5 * n)]
    ax.add_patch(Polygon(p, closed=True, **kw))


def plan(ax):
    x0, x1 = -560, 300
    # belt band between post faces
    ax.add_patch(Rectangle((x0, -POST_INNER), x1 - x0, 2 * POST_INNER,
                           fc='#e7f1ea', ec='none', zorder=0))
    ax.plot([x0, x1], [POST_INNER] * 2, color=C['green'], lw=0.8)
    ax.plot([x0, x1], [-POST_INNER] * 2, color=C['green'], lw=0.8)
    # camera window
    ax.add_patch(Rectangle((-GAP - LV, -POST_INNER), LV, 2 * POST_INNER,
                           fc='#dfe7f7', ec=C['blue'], lw=0.6, ls='--', zorder=1))
    ax.text(-GAP - LV / 2, -POST_INNER + 14, 'detection window $L_v$',
            ha='center', va='bottom', fontsize=6.5, color=C['blue'])
    # lane guides
    for s in (1, -1):
        ax.plot([x0 + 10, -GAP - LV - 20, -GAP - 40], [s * 128, s * 75, s * 75],
                color='#555555', lw=1.6, solid_capstyle='butt', zorder=3)
    ax.text(-400, 82, 'lane guides', fontsize=6, color='#444444', va='bottom', ha='center')
    # capture band
    for s in (1, -1):
        ax.plot([x0, x1], [s * BAND] * 2, color=C['gray'], lw=0.5, ls=(0, (4, 2, 1, 2)))
    # posts
    for s in (1, -1):
        ax.add_patch(Rectangle((-POST_X / 2, POST_INNER if s > 0 else -POST_INNER - POST_X),
                               POST_X, POST_X, fc=C['orange'], ec='k', lw=0.4, zorder=4))
    # swing circle and strike sector
    ax.add_patch(Circle((0, 0), R_SWING, fc='none', ec=C['gray'], lw=0.5, ls=':', zorder=2))
    ax.add_patch(Wedge((0, 0), R_HALF, 135, 225, fc='#f6d7d7', ec=C['red'], lw=0.5, zorder=2))
    ax.text(-62, 0, 'strike\nsector', ha='center', va='center', fontsize=6, color=C['red'], zorder=6)
    # gate poses
    plate(ax, TH_ORG, fc='none', ec=C['blue'], lw=0.7, ls='--', zorder=5)
    plate(ax, TH_REST, fc=C['blue'], ec=C['blue'], lw=0.5, zorder=5)
    ax.add_patch(Circle((0, 0), HUB_D / 2, fc='white', ec='k', lw=0.5, zorder=6))
    ax.text(80, 95, r'rest $+45^\circ$', fontsize=6.5, color=C['blue'], ha='center')
    ax.text(84, -97, r'organic $-45^\circ$', fontsize=6.5, color=C['blue'], ha='center')
    # outlets
    ax.add_patch(FancyArrowPatch((-95, 112), (-95, 168), arrowstyle='-|>', mutation_scale=7,
                                 color=C['red'], lw=0.9, zorder=7))
    ax.text(-95, 172, 'side bin (PLASTIC)', ha='center', va='bottom', fontsize=6.5, color=C['red'])
    ax.plot([215, x1], [0, 0], color='k', lw=0.8)
    ax.text(x1 - 4, 40, 'OTHER\n(rear-left)', ha='right', va='center', fontsize=6.5)
    ax.text(x1 - 4, -40, 'ORGANIC\n(rear-right)', ha='right', va='center', fontsize=6.5)
    ax.add_patch(FancyArrowPatch((-160, 112), (-60 - 40, 112), arrowstyle='-|>',
                                 mutation_scale=7, color='k', lw=0.7))
    ax.text(-170, 112, r'belt $v$', ha='right', va='center', fontsize=6.5)
    # dimensions
    def dim(xa, ya, xb, yb, txt, off=(0, 0), **kw):
        ax.add_patch(FancyArrowPatch((xa, ya), (xb, yb), arrowstyle='<|-|>', mutation_scale=5,
                                     lw=0.5, color='k', zorder=8))
        ax.text((xa + xb) / 2 + off[0], (ya + yb) / 2 + off[1], txt, fontsize=6, ha='center',
                va='center', zorder=8, **kw)
    dim(-GAP, -150, 0, -150, '$g$', off=(0, -9))
    dim(-GAP - LV, -150, -GAP, -150, '$L_v$', off=(0, -9))
    ax.text(-8, -117, '$r=100$', fontsize=6, color=C['gray'], ha='center', va='center')
    dim(150, BAND, 150, -BAND, '141', off=(9, -40), rotation=90)
    ax.set_xlim(x0, x1)
    ax.set_ylim(-175, 195)
    ax.set_aspect('equal')
    ax.set_xlabel('$x$ (mm), downstream')
    ax.set_ylabel('$y$ (mm), left')
    ax.set_xticks([-500, -400, -300, -200, -100, 0, 100, 200, 300])
    ax.set_yticks([-135, 0, 135])


def section(ax):
    # transverse section through the pivot (y horizontal, z up), belt surface z=0
    ax.add_patch(Rectangle((-150, -6), 300, 6, fc=C['green'], ec='none'))
    ax.add_patch(Rectangle((-125, 0), 250, CLEAT_H, fc='#bcd9c6', ec=C['green'], lw=0.5))
    ax.text(0, 18, 'cleat (behind), 50 mm', ha='center', fontsize=6, color=C['green'])
    off = 10.0
    for s in (1, -1):
        ax.add_patch(Rectangle((s * POST_INNER if s > 0 else -POST_INNER - 40, 0), 40, off,
                               fc=C['lgray'], ec='k', lw=0.3))
        ax.add_patch(Rectangle((POST_INNER + 4 if s > 0 else -POST_INNER - 36, off), 32, 140,
                               fc=C['orange'], ec='k', lw=0.4))
    zb = off + 140
    ax.add_patch(Rectangle((-161, zb), 150, 6, fc=C['orange'], ec='k', lw=0.4))
    ax.add_patch(Rectangle((11, zb), 150, 6, fc=C['orange'], ec='k', lw=0.4))
    ax.add_patch(Rectangle((-31, zb + 6), 62, 6, fc=C['orange'], ec='k', lw=0.4))
    ax.add_patch(Rectangle((-21.15, zb + 12), 42.3, 40, fc='#3a3a3a', ec='k', lw=0.4))
    ax.text(0, zb + 32, 'motor', color='w', ha='center', va='center', fontsize=5)
    hub_top = zb + 6 - 1
    ax.add_patch(Rectangle((-HUB_D / 2, hub_top - HUB_H), HUB_D, HUB_H, fc=C['blue'], ec='k', lw=0.4))
    # gate seen at 0 deg would be edge-on; draw it at 90 deg (full face) for clarity
    ax.add_patch(Rectangle((-R_HALF, GATE_LOW), 2 * R_HALF, PLATE_H, fc='#9fb7e3', ec=C['blue'], lw=0.6))
    ax.text(0, GATE_LOW + PLATE_H / 2, 'gate 200 x 80 x 5', ha='center', va='center', fontsize=6)
    ax.plot([-130, 130], [GATE_LOW] * 2, color=C['red'], lw=0.5, ls='--')
    ax.text(-127, GATE_LOW + 2, '55', fontsize=6, color=C['red'], ha='left', va='bottom')
    ax.plot([-130, 130], [GATE_HIGH] * 2, color=C['red'], lw=0.5, ls='--')
    ax.text(-127, GATE_HIGH + 2, '135', fontsize=6, color=C['red'], ha='left', va='bottom')
    ax.set_xlim(-180, 180)
    ax.set_ylim(-10, 215)
    ax.set_aspect('equal')
    ax.set_xlabel('$y$ (mm)')
    ax.yaxis.tick_right()
    ax.yaxis.set_label_position('right')
    ax.set_ylabel('$z$ (mm)')
    ax.set_yticks([0, 50, 100, 150, 200])


if __name__ == '__main__':
    fig = plt.figure(figsize=(DCOL, 2.25))
    gs = fig.add_gridspec(1, 2, width_ratios=[2.45, 1.0], wspace=0.12)
    a1 = fig.add_subplot(gs[0])
    a2 = fig.add_subplot(gs[1])
    plan(a1)
    section(a2)
    a1.set_title('(a) plan view at the design point', loc='left', fontsize=7.5)
    a2.set_title('(b) section through the pivot', loc='left', fontsize=7.5)
    fig.savefig('../figs/fig_geometry.pdf')
    fig.savefig('../figs/fig_geometry.png', dpi=200)
    print('ok')
