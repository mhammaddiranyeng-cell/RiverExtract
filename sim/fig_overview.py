"""Fig. 1: prototype photograph and system architecture (two columns)."""
from figstyle import plt, C, DCOL
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
import matplotlib.image as mpimg

H = 2.45
fig = plt.figure(figsize=(DCOL, H))
img = mpimg.imread('../figs/photo_crop.jpg')
ar = img.shape[1] / img.shape[0]
wp = H * ar                                   # photo width in inches
a0 = fig.add_axes([0, 0, wp / DCOL, 1])
a0.imshow(img)
a0.axis('off')
a0.text(0.04, 0.975, '(a)', transform=a0.transAxes, color='w', fontsize=8, va='top',
        fontweight='bold', bbox=dict(boxstyle='round,pad=0.15', fc='black', ec='none', alpha=0.55))
a0.annotate('gate', xy=(0.47, 0.705), xycoords='axes fraction', xytext=(0.80, 0.83),
            textcoords='axes fraction', fontsize=6.5, ha='center', va='center',
            bbox=dict(boxstyle='round,pad=0.2', fc='white', ec='none', alpha=0.9),
            arrowprops=dict(arrowstyle='-|>', lw=0.7, color='k', shrinkA=0, shrinkB=1,
                            mutation_scale=6))
gap = 0.10
ax = fig.add_axes([(wp + gap) / DCOL, 0, 1 - (wp + gap) / DCOL, 1])
ax.set_xlim(0, 100)
ax.set_ylim(0, 41)
ax.axis('off')
ax.text(0.2, 40.2, '(b)', fontsize=8, va='top', fontweight='bold')


def box(x, y, w, h, txt, fc='#ffffff', fs=6.4, lw=0.6):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.2,rounding_size=0.8',
                                fc=fc, ec='k', lw=lw))
    ax.text(x + w / 2, y + h / 2, txt, ha='center', va='center', fontsize=fs, linespacing=1.2)


def arrow(p, q, txt=None, off=(0, 0.4), fs=5.8, ha='center', va='bottom', style='-|>'):
    ax.add_patch(FancyArrowPatch(p, q, arrowstyle=style, mutation_scale=7, lw=0.6, color='k',
                                 shrinkA=0, shrinkB=0))
    if txt:
        ax.text((p[0] + q[0]) / 2 + off[0], (p[1] + q[1]) / 2 + off[1], txt, fontsize=fs,
                ha=ha, va=va)


# lanes
ax.add_patch(FancyBboxPatch((3.5, 17.6), 96, 18.4, boxstyle='round,pad=0,rounding_size=1',
                            fc='#f2f5fb', ec='#c7d2e8', lw=0.5))
ax.text(4.5, 34.8, 'machine layer', fontsize=6.4, color=C['blue'], va='center')
ax.add_patch(FancyBboxPatch((3.5, 0.4), 96, 15.6, boxstyle='round,pad=0,rounding_size=1',
                            fc='#fbf3ec', ec='#ead3c0', lw=0.5))
ax.text(4.5, 14.8, 'vision and sorting layer', fontsize=6.4, color=C['orange'], va='center')
# power strip
ax.text(51.5, 38.4, 'PV panel charges a 24 V battery: 24 V for the PLC and the belt motor; '
        'step-down converters 12 V for the stepper driver and 5 V for logic', fontsize=6.0, ha='center', va='center',
        color='#333333')

# machine layer
box(5.5, 21.0, 19, 11.6, 'intake sensor  X0\nIR jam beam  X2\nE-stop (NC)  X3\n\nfrom Pi relays:\npause  X4\nbins full  X5–X7', fs=5.8)
box(32, 20.5, 22, 12.0, 'PLC\nDelta DVP-14SS2 + DVP-08SP\nrun latch, jam reverse,\nPWM speed, bin logic', fc='#e1e8f6',
    fs=6.0)
box(61, 27.6, 15, 5.2, 'BTS7960\nY0 PWM / Y1 rev.', fs=5.8)
box(61, 20.0, 15, 5.2, 'beacon  Y4\nrun lamp  Y5', fs=5.8)
box(82, 27.6, 16, 5.2, '24 V worm-gear\nmotor (belt)', fs=5.8)
arrow((24.9, 26.8), (32, 26.8))
arrow((54.3, 30.2), (61, 30.2))
arrow((54.3, 22.6), (61, 22.6))
arrow((76.3, 30.2), (82, 30.2))

# vision layer
box(5.5, 7.4, 14, 5.2, 'Logitech C270\nUSB camera', fs=5.8)
box(5.5, 1.0, 14, 5.2, 'phone / laptop\n(Wi-Fi)', fs=5.8)
box(26, 1.6, 30, 11.4, 'Raspberry Pi 4\nYOLOv8n (COCO), 320 px, conf 0.55\ntracker, scheduler, bin counters\nbin-level ultrasonics, web dashboard',
    fc='#f7e5d6', fs=5.8)
box(63, 4.2, 12, 6.0, 'A4988\nfull step', fs=5.8)
box(82, 2.6, 16, 9.2, 'NEMA 17\n+ printed gate\n(3 routes)', fc='#dbe4f5', fs=5.8)
arrow((19.8, 10.0), (26, 10.0), 'USB', off=(0, 0.35), fs=5.4)
arrow((19.8, 3.6), (26, 3.6), style='<|-|>')
ax.text(22.9, 4.0, 'HTTP', fontsize=5.4, ha='center', va='bottom')
arrow((56.3, 7.2), (63, 7.2), 'STEP 27\nDIR 17', off=(0, 0.35), fs=5.4)
arrow((75.3, 7.2), (82, 7.2))
# the only link between the layers: relay contacts into PLC inputs
arrow((30.0, 13.3), (24.9, 21.6))
ax.text(28.6, 17.0, 'relays K1–K4\n(isolated contacts)', fontsize=5.4, ha='left', va='center',
        color='#7a3d00')
fig.savefig('../figs/fig_overview.pdf')
fig.savefig('../figs/fig_overview.png', dpi=220)
print('ok', wp)
