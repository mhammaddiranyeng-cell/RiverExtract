# RiverExtract

Code, CAD files and controller programs behind the paper **"RiverExtract: A Vision-Guided
Three-Way Sorting Gate with Sampling-Aware Control for River-Debris Collection"** by
M. A. Dirany, M. Shadad, A. Farhat, A. Youssef, G. Abou Haidar (American University of
Science and Technology, Zahle, Lebanon) and C. Saide (Lebanese University, Zahle, Lebanon).

RiverExtract is a solar-assisted cleated-belt conveyor that lifts floating debris out of a
river. A Delta PLC runs the belt. A Raspberry Pi 4 runs YOLOv8n on a USB camera and drives a
3D-printed gate on a stepper motor, which sends each item to a plastic, an organic, or an
unsorted bin. Because the detector samples the belt only a few times per second, the timing
of the gate decides where an item lands; the simulation in this repository models that
timing and the gate's contact with the items.

## Contents

| Folder | What it holds |
|---|---|
| [`sim/`](sim) | Python simulation: planar contact model of the printed gate, vision sampling model, first-firmware and event-driven controllers, several-item scheduler, motor load. The scripts produce every number, table and result figure in the paper. |
| [`figs/`](figs) | The figures as used in the paper, and the photo of the prototype. |
| [`paper/`](paper) | The numbers quoted in the paper, as LaTeX macros written by the simulation. |
| [`matlab/`](matlab) | MATLAB scripts that redraw the figures from data exported by the simulation. |
| [`cad/`](cad) | OpenSCAD sources and STL files of the gate, bridge, lane guides and sensor mount. |
| [`plc/`](plc) | The PLC program (Delta DVP-14SS2, WPLSoft instruction list) with its I/O map. |
| [`firmware/`](firmware) | Sorting firmware and dashboard for the Raspberry Pi. |

## Reproducing the results

Python 3.10 or later with numpy, scipy and matplotlib:

```
pip install numpy scipy matplotlib
cd sim
python3 run_v3_cw.py       # capture windows of the gate, 0.05-0.60 m/s
python3 run_v3_mc.py       # Monte Carlo: loop periods, design envelope, first firmware, misclassification
python3 run_v3_multi.py    # pairs and random streams of several items
python3 stepper_load.py    # contact radius and item-mass limit of the strike
python3 gen_v3.py          # numbers (paper/numbers_v3.tex) and result figures (figs/)
```

The full run takes about an hour on two cores. Its results are kept in `sim/results/`, so
`python3 gen_v3.py` on its own regenerates the numbers and figures in seconds. The geometry,
motion and contact-map figures come from `fig_geom.py`, `fig_motion.py` and `fig_maps.py`,
and Fig. 1 from `fig_overview.py`. For the MATLAB versions of the figures, run
`python3 export_for_matlab.py`, then `make_all_figures.m` in `matlab/`.

## Prototype

| Part | As built |
|---|---|
| Belt | Cleated belt (50 mm flights) in a Z profile, 24 V worm-gear motor (Gimson GR-WM4 V3) through a BTS7960 H-bridge |
| Machine layer | Delta DVP-14SS2 PLC with a DVP-08SP expansion: intake start, PWM speed, slowdown and stop on full bins, jam reverse with retry limit, latched fault |
| Vision layer | Raspberry Pi 4, Logitech C270 camera, YOLOv8n (COCO weights) at 320 px, web dashboard |
| Link | Four relay contacts from the Pi to PLC inputs (pause and three bin-full signals); the two layers share no wire |
| Gate | 200 x 80 x 5 mm PLA plate on a center pivot: rest pose, organic hold, plastic strike. NEMA 17 on an A4988 in the bench test; now a NEMA 23 (23HS30-2804S) on a TB6600 |
| Power | Photovoltaic panel charging a 24 V battery |
| Cost | About USD 900 in parts |

## License

MIT; see [LICENSE](LICENSE).

## Citation

If you use this work, please cite the paper. The full reference will be added here once it
is published.
