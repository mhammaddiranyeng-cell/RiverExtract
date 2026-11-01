# CAD: sorting gate and bridge

OpenSCAD sources (`RiverExtract_Sorter_v2.scad`, `assembly.scad`) and printable STL files.

| File | Part |
|---|---|
| `stl/RiverExtract_v2_Gate.stl` | Gate: 200 x 80 x 5 mm plate with an 18 mm hub on its top edge, on the rotation axis. Print standing (long edge on the bed) with a 5 mm brim, no supports. |
| `stl/RiverExtract_v2_PostL/R.stl`, `BeamL/R.stl`, `Center.stl` | Two-post bridge that carries the motor over the belt (270 mm clear opening) |
| `stl/RiverExtract_v2_SpacerL/R.stl` | 10 mm foot spacers that set the gate's height |
| `stl/RiverExtract_v2_GuideL/R.stl` | Lane guides that keep items inside the gate's 141 mm capture band |
| `stl/RiverExtract_v2_Sensor.stl` | Sensor mount |
| `stl/RiverExtract_v2_ALL_bed_layout.stl` | All parts on one 300 x 300 mm bed |

PLA, 0.28 mm layers, 2 walls, 3 top and bottom layers, 10 % gyroid infill.

## Assembly

1. Set the height so that the gate's lower edge runs 5 mm above the cleat tips (55 mm above
   the belt). The 10 mm spacers give this height when the feet sit at belt level.
2. Place the lane guides upstream of the camera window, with their inner faces 71 mm either
   side of the belt center line.
3. Bins: the plastic bin on the left side, just upstream of the left post; two bins under the
   head, split at the belt center line (rear-left OTHER, rear-right ORGANIC).
4. Rest pose: +45 deg, with the downstream end of the plate pointing towards the left edge.

## Prototype notes

- On the prototype, a cantilever arm on the panel mast carries the motor instead of the bridge.
- The hub's bore fits the 5 mm shaft of the NEMA 17 used in the bench test. The prototype now
  runs a NEMA 23 (23HS30-2804S, 6.35 mm shaft) on a metal-reinforced arm; that motor needs a
  matching hub, preferably a metal flange coupling bolted to the plate.
