// Assembled view of the v2 sorting bridge on the belt (visual only).
use <RiverExtract_Sorter_v2.scad>
OFF = 10;                // spacer under each foot
BELT_HALF = 135;
// coordinates: x along belt (downstream), y left, z up, z = 0 belt surface
module beltpiece() {
  color([0.12,0.45,0.30]) translate([-420, -BELT_HALF, -6]) cube([700, 2*BELT_HALF, 6]);
  for (xc = [-380, -180, 20, 220])
    color([0.10,0.38,0.25]) translate([xc, -BELT_HALF + 10, 0]) cube([6, 2*BELT_HALF - 20, 50]);
}
beltpiece();
// posts: foot centre at y = +-(135 + 16), column inner face at +-135
for (s = [-1, 1]) {
  color([0.85,0.85,0.88]) translate([-30, s > 0 ? BELT_HALF - 4 : -BELT_HALF - 36, 0]) spacer();
  color([0.95,0.55,0.15]) translate([-30, s > 0 ? BELT_HALF - 4 : -BELT_HALF - 36, OFF]) post();
}
// beams (x across the gap is y here): rotate the 135-long beams to run along y
color([0.95,0.55,0.15]) translate([15, -BELT_HALF - 20, OFF + 140]) rotate([0,0,90]) beamHalf();
color([0.95,0.55,0.15]) translate([15, 11 + 4, OFF + 140]) rotate([0,0,90]) beamHalf();
// centre mount, centred on the axis
color([0.95,0.55,0.15]) translate([35, -45, OFF + 146]) rotate([0,0,90]) centerMount();
// NEMA 17 (reference box)
color([0.25,0.25,0.28]) translate([-21.15, -21.15, OFF + 152]) cube([42.3, 42.3, 40]);
// gate at the rest pose (+45 deg); hub top 1 mm below the centre mount
color([0.15,0.45,0.85])
  translate([0, 0, OFF + 146 - 1 - 100]) rotate([0,0,45]) translate([-100, -2.5, 0]) gate();
// lane guides upstream of the gate, at y = +-75 (inner faces at +-71)
for (s = [-1, 1]) color([0.75,0.75,0.78,0.9])
  translate([-480, s > 0 ? 71 : -75, 55]) cube([240, 4, 60]);
