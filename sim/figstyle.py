import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
import glob

for f in glob.glob('/usr/share/fonts/truetype/liberation/LiberationSerif-*.ttf'):
    fm.fontManager.addfont(f)

plt.rcParams.update({
    'font.family': 'Liberation Serif',
    'mathtext.fontset': 'stix',
    'font.size': 8,
    'axes.labelsize': 8,
    'axes.titlesize': 8,
    'legend.fontsize': 7,
    'xtick.labelsize': 7,
    'ytick.labelsize': 7,
    'axes.linewidth': 0.6,
    'xtick.major.width': 0.5,
    'ytick.major.width': 0.5,
    'xtick.major.size': 2.5,
    'ytick.major.size': 2.5,
    'lines.linewidth': 1.0,
    'legend.frameon': False,
    'savefig.dpi': 600,
    'savefig.bbox': 'tight',
    'savefig.pad_inches': 0.02,
    'pdf.fonttype': 42,
})
COL = 3.5           # IEEE column width, inches
DCOL = 7.16         # IEEE text width
C = dict(blue='#1f4e9c', orange='#d9822b', green='#2e7d4f', red='#b23a3a',
         gray='#7a7a7a', lgray='#d9d9d9', purple='#6a3d9a', teal='#1b7f86')
