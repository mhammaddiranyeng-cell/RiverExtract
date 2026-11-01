# Sorting firmware (Raspberry Pi)

Python firmware for the sorting stage: camera, YOLOv8n detector, tracker, gate scheduler,
ramped stepper control, logging and web dashboard.

| File | Contents |
|---|---|
| `riverextract_sort_v2.py` | Event-driven controller: each item is tracked and committed once, and the gate acts on the item's predicted position from time-stamped centroids and the belt speed. Holds the calibration block (camera geometry, belt speed, gate direction). |
| `riverextract_sort_v2_1.py` | Same controller, plus the measurement tools: CSV logs of every frame, decision, gate move and processor temperature; the first firmware's trigger logic as a baseline (`--legacy`); pacing control (`--pace`); strike-offset and lost-step tests. It reads its calibration from `riverextract_sort_v2.py` in the same folder. |

The ultrasonic bin-level sensors and the relay contacts to the PLC are not part of this code.

## Requirements

Raspberry Pi 4, Logitech C270 (USB), Python 3 with `ultralytics`, `opencv-python`, `flask`
and `RPi.GPIO`.

## Wiring

STEP on GPIO 27, DIR on GPIO 17, driver in full-step mode.

- **A4988** (bench test, NEMA 17): VDD from the Pi's 5 V, ENABLE to GND, SLEEP jumpered to
  RESET, MS1-MS3 floating (full step); current limit Vref = 8 x I x Rs.
- **TB6600** (current prototype, NEMA 23): PUL+ to GPIO 27, DIR+ to GPIO 17, PUL- and DIR-
  to the Pi's ground, ENA unconnected; microstep switches on full step, so the step timing
  is unchanged.

## Calibration (once)

Edit the CONFIG block at the top of `riverextract_sort_v2.py`:

- `MM_PER_PX`: belt millimetres per pixel (lay a ruler along the belt in view).
- `PX_WINDOW_END`: pixel row or column where the detection window ends.
- `GAP_G`: distance on the belt from that line to the gate pivot (200 mm at the design point).
- `PX_CENTRE`: pixel column of the belt center line.
- `ALONG_AXIS`, `ALONG_SIGN`, `LEFT_SIGN`: set so the red line in the video sits on the window
  end and the yellow lines on the lane guides.
- `BELT_SPEED`: starting estimate; the firmware refines it from the tracks.

Then put the gate at the rest pose (+45 deg, downstream end of the plate towards the left
edge) with `python3 riverextract_sort_v2.py --jog`; key `a` steps counter-clockwise seen from
above. If it turns the other way, set `DIR_CCW_LEVEL = 0`.

## Run

```
python3 riverextract_sort_v2_1.py --test             # the three gate actions, no items
python3 riverextract_sort_v2_1.py --pace 0           # event-driven controller, no pacing
python3 riverextract_sort_v2_1.py --pace 0 --legacy  # first-firmware trigger logic (baseline)
python3 riverextract_sort_v2_1.py --strike-test 20   # repeated strikes, to check for lost steps
python3 riverextract_sort_v2_1.py --save-frames 15   # save frames for labelling a detector
python3 riverextract_sort_v2_1.py --model best.pt    # detector with plastic/organic/other classes
```

The dashboard runs on port 5000: annotated video, bin counters, bin-full notifications, the
measured inference time and loop period (the paper's tau_i and T), and a form for entering
the bin each item reached. Logs are written to `~/riverextract_logs/<date_time>/`.

## Routes

| Detected class | Gate action | Bin |
|---|---|---|
| bottle, cup, wine glass, bowl | 90 deg strike while the item is in the strike sector | side (left) |
| banana, apple, orange, broccoli, carrot | move to -45 deg before the item arrives, hold, return | rear-right |
| book, cell phone, anything not detected | none (rest pose +45 deg) | rear-left |
