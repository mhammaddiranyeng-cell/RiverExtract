# Simulation

Python 3 with numpy, scipy and matplotlib. Run every script from this folder; figures go to
`../figs/` and the paper's numbers to `../paper/`.

## Model

| File | Contents |
|---|---|
| `params.py` | Geometry from the printed parts, firmware timing constants |
| `dynamics.py` | Gate mass properties; ramped and constant-rate motion profiles |
| `kin.py` | Planar contact simulation: gate plate, disc-shaped items, posts, belt friction |
| `timing.py` | Vision sampling model; first-firmware and event-driven controllers, with the inference latency and the pacing interval as separate parameters; `replan=False` times every action once from the first detection (the fixed-delay baseline) |
| `multi.py` | Several items on the belt: tracker, scheduler (first come first served or conflict-aware), every gate action applied to every item near the gate |
| `stepper_load.py` | Contact radius at the strike's aim point, and the item mass at which the strike's load torque reaches a motor's holding torque |

## Experiments

| Script | Output |
|---|---|
| `run_v3_cw.py` | Capture windows on a 10 mm/s grid, 0.05-0.60 m/s -> `results/cw_v3.pkl` |
| `run_v3_mc.py` | Loop-period sweep, design-envelope table, first firmware at the design point, outcome distributions of committed items (misclassification) -> `results/v3.json` |
| `run_v3_multi.py` | Pairs 0-600 mm apart and random streams of 10-90 items/min -> `results/v3_multi.json` |
| `run_v3_baseline2.py` | Stronger baselines: every action timed once from the first detection (fixed delay), with the belt speed from an encoder or from the tracks, and the event-driven controller with an encoder; centroid noise 5 and 10 mm -> `results/v3_baseline2.json` |
| `stepper_load.py` | -> `results/stepper_load.json` |
| `gen_v3.py` | Every number of the current paper, both speed bounds of the design-envelope table (feasibility and noise-aware, also with the timing distance measured from the first detection), the linear fit of the action point x*(v) and the closed form of the feasibility bound, the stronger baselines, result figures -> `../paper/numbers_v3.tex`, `../figs/` |
| `export_for_matlab.py` | Plotted data for the MATLAB figure scripts -> `../matlab/data/` |

Order (about 75 min on two cores):

```
python3 run_v3_cw.py ; python3 run_v3_mc.py ; python3 run_v3_multi.py
python3 run_v3_baseline2.py ; python3 stepper_load.py ; python3 gen_v3.py
```

## First version

`run_exp.py`, `run_mc.py`, `fine_windows.py` and `gen_numbers.py` are the first version of the
study, with a 1.0 s loop period. They produce `results/results.json`, `results/maps.npz` and
`../paper/numbers.tex`, which still supply the contact-simulation numbers and the geometry,
motion and contact-map figures (`fig_geom.py`, `fig_motion.py`, `fig_maps.py`).
`fig_results.py` is the first version's result figure; `gen_v3.py` replaces it.
