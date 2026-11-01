// =============================================================================
//  RiverExtract -- PRINTABLE three-way sorting gate (NEMA 17), version 2
//  Matches the IEEE paper.  OpenSCAD (free): https://openscad.org
//
//  WHAT CHANGED FROM VERSION 1
//   * Gate: the hub now sits on the plate's TOP EDGE, on the rotation axis
//     (it was on the plate face).  Hub d = 18 mm so it clears both beam ends
//     by 2 mm.  A 40 mm gusset joins hub and plate.  The gate prints
//     STANDING (no supports), bore vertical.
//   * Spacer: optional 10 mm foot spacers.  With them, and the hub 1 mm below
//     the centre mount, the gate's lower edge runs 55 mm above the belt
//     surface = 5 mm above the 50 mm cleat tips (feet on the belt plane).
//   * LaneGuide: guide plate that narrows the stream to the gate's capture
//     band (|y| <= 70 mm).  Bolt two of them, mirrored, upstream of the camera.
//   Posts, beam halves, centre mount and sensor mount are unchanged.
//
//  HOW TO USE
//   F6 (Render) -> File -> Export -> STL.  To export one part, set PRINT_ONE
//   to its name, e.g. PRINT_ONE = "Gate";
//
//  PRINT SETTINGS (Ender-3 V3 Plus class printer, PLA)
//   Layer 0.28 mm, 2 walls, 3 top/bottom layers, 10 % gyroid infill.
//   Gate: print standing as laid out (long edge on the bed), 5 mm brim.
// =============================================================================

PRINT_ONE = "";   // "" = all parts, or one of:
                  // "PostL" "PostR" "BeamL" "BeamR" "Center" "Gate" "Sensor"
                  // "SpacerL" "SpacerR" "GuideL" "GuideR"

$fn = 64;

// ------------------------- FASTENERS (mm) -----------------------------------
M3_CLR = 3.4;  M3_TAP = 2.8;  M4_CLR = 4.5;  JOINT_SP = 7.0;  M12_CLR = 12.2;

// ------------------------- POST / BEAM / CENTRE (unchanged) -----------------
FOOT_W = 60; FOOT_D = 40; FOOT_T = 8;
POST_W = 32; POST_D = 32; POST_TOTAL_H = 140;
BEAM_LEN = 135; BEAM_W = 30; BEAM_T = 6;
CTR_W = 90; CTR_D = 70; CTR_T = 6;
NEMA17_SPACING = 15.5; NEMA17_BOSS_CLR = 22.5;
SENSOR_TAB_W = 30; SENSOR_TAB_D = 20;
SENS_W = 50; SENS_D = 25; SENS_T = 5;

// ------------------------- GATE (version 2) ---------------------------------
GATE_L = 200;       // along the belt at 0 deg
GATE_H = 80;        // plate height
GATE_T = 5;         // plate thickness
HUB_D  = 18;        // clears the beam ends (22 mm gap) by 2 mm per side
HUB_H  = 20;
GUSSET_L = 40;      // gusset footprint on the plate's top edge
GUSSET_H = 10;      // gusset height (inside the hub's lower half)
BORE_D = 5.4;       // 5 mm NEMA 17 shaft
BORE_DEPTH = 16;    // stops where the gusset is wider than the bore + walls
SETSCREW_Z = 8;     // M3 set screw, 8 mm below the hub top

// ------------------------- SPACER / GUIDE -----------------------------------
SPACER_T = 10;
GUIDE_L = 240; GUIDE_H = 60; GUIDE_T = 4; GUIDE_TAB = 30;

module holeThruZ(cx, cy, z0, t, d) { translate([cx, cy, z0 - 1.5]) cylinder(h = t + 3, d = d); }
module holeDownFromTop(cx, cy, ztop, depth, d) { translate([cx, cy, ztop - depth - 1.5]) cylinder(h = depth + 3, d = d); }
module jointHoles2Y(cx, cy, z0, t, d) { holeThruZ(cx, cy - JOINT_SP, z0, t, d); holeThruZ(cx, cy + JOINT_SP, z0, t, d); }

module post() {
    difference() {
        union() {
            cube([FOOT_W, FOOT_D, FOOT_T]);
            translate([(FOOT_W - POST_W)/2, (FOOT_D - POST_D)/2, FOOT_T])
                cube([POST_W, POST_D, POST_TOTAL_H - FOOT_T]);
        }
        holeThruZ(FOOT_W/2 - 17.5, FOOT_D/2, 0, FOOT_T, M4_CLR);
        holeThruZ(FOOT_W/2 + 17.5, FOOT_D/2, 0, FOOT_T, M4_CLR);
        holeDownFromTop(FOOT_W/2 - JOINT_SP, FOOT_D/2, POST_TOTAL_H, POST_TOTAL_H - FOOT_T, M4_CLR);
        holeDownFromTop(FOOT_W/2 + JOINT_SP, FOOT_D/2, POST_TOTAL_H, POST_TOTAL_H - FOOT_T, M4_CLR);
    }
}

module beamHalf() {
    difference() {
        cube([BEAM_LEN, BEAM_W, BEAM_T]);
        jointHoles2Y(10, BEAM_W/2, 0, BEAM_T, M4_CLR);
        jointHoles2Y(BEAM_LEN - 10, BEAM_W/2, 0, BEAM_T, M4_CLR);
    }
}

module centerMount() {
    ccx = CTR_W/2; ccy = CTR_D/2;
    difference() {
        union() {
            cube([CTR_W, CTR_D, CTR_T]);
            translate([ccx - SENSOR_TAB_W/2, CTR_D, 0]) cube([SENSOR_TAB_W, SENSOR_TAB_D, CTR_T]);
        }
        for (sx = [-NEMA17_SPACING, NEMA17_SPACING]) for (sy = [-NEMA17_SPACING, NEMA17_SPACING])
            holeThruZ(ccx + sx, ccy + sy, 0, CTR_T, M3_CLR);
        holeThruZ(ccx, ccy, 0, CTR_T, NEMA17_BOSS_CLR);
        jointHoles2Y(10, ccy, 0, CTR_T, M4_CLR);
        jointHoles2Y(CTR_W - 10, ccy, 0, CTR_T, M4_CLR);
        holeThruZ(ccx, CTR_D + SENSOR_TAB_D - 10, 0, CTR_T, M4_CLR);
    }
}

// Gate printed standing: x = along the plate, y = thickness, z = up.
// The rotation axis is the vertical line x = GATE_L/2, y = GATE_T/2.
module gate() {
    ax = GATE_L/2; ay = GATE_T/2;
    difference() {
        union() {
            cube([GATE_L, GATE_T, GATE_H]);
            // gusset: hull of a 40 x 5 footprint and the hub circle
            hull() {
                translate([ax - GUSSET_L/2, 0, GATE_H - 0.01]) cube([GUSSET_L, GATE_T, 0.01]);
                translate([ax, ay, GATE_H + GUSSET_H - 0.01]) cylinder(h = 0.01, d = HUB_D);
            }
            translate([ax, ay, GATE_H]) cylinder(h = HUB_H, d = HUB_D);
        }
        // shaft bore from the hub top
        translate([ax, ay, GATE_H + HUB_H - BORE_DEPTH]) cylinder(h = BORE_DEPTH + 1, d = BORE_D);
        // M3 radial set screw (tapped), perpendicular to the plate
        translate([ax, ay, GATE_H + HUB_H - SETSCREW_Z]) rotate([90, 0, 0])
            cylinder(h = HUB_D + 2, d = M3_TAP, center = true);
    }
}

module sensorMount() {
    difference() {
        cube([SENS_W, SENS_D, SENS_T]);
        holeThruZ(12, SENS_D/2, 0, SENS_T, M12_CLR);
        jointHoles2Y(SENS_W - 10, SENS_D/2, 0, SENS_T, M4_CLR);
    }
}

module spacer() {
    difference() {
        cube([FOOT_W, FOOT_D, SPACER_T]);
        holeThruZ(FOOT_W/2 - 17.5, FOOT_D/2, 0, SPACER_T, M4_CLR);
        holeThruZ(FOOT_W/2 + 17.5, FOOT_D/2, 0, SPACER_T, M4_CLR);
    }
}

// Lane guide: a flat plate with a 90-deg mounting tab carrying two M4 slots.
module laneGuide() {
    difference() {
        union() {
            cube([GUIDE_L, GUIDE_T, GUIDE_H]);
            translate([0, 0, 0]) cube([GUIDE_TAB, 25, GUIDE_T]);
            translate([GUIDE_L - GUIDE_TAB, 0, 0]) cube([GUIDE_TAB, 25, GUIDE_T]);
        }
        for (x0 = [GUIDE_TAB/2, GUIDE_L - GUIDE_TAB/2])
            hull() {
                translate([x0 - 5, 16, -1]) cylinder(h = GUIDE_T + 2, d = M4_CLR);
                translate([x0 + 5, 16, -1]) cylinder(h = GUIDE_T + 2, d = M4_CLR);
            }
        // rounded leading edge relief
        translate([-1, -1, GUIDE_H - 6]) rotate([0, 90, 0]) cylinder(h = 2, r = 0.1);
    }
}

// ------------------------------------------------------------------ LAYOUT --
// Fits a 300 x 300 mm bed.  The gate stands on its long edge (prints 100 mm
// tall); the guides stand on their long edges too.
module layout() {
    G = 10;
    if (PRINT_ONE == "" || PRINT_ONE == "Gate")    translate([0, 8, 0]) gate();
    if (PRINT_ONE == "" || PRINT_ONE == "GuideL")  translate([0, 25, 0]) laneGuide();
    if (PRINT_ONE == "" || PRINT_ONE == "GuideR")  translate([0, 58, 0]) laneGuide();
    row2 = 95;
    if (PRINT_ONE == "" || PRINT_ONE == "Center")  translate([0, row2, 0]) centerMount();
    if (PRINT_ONE == "" || PRINT_ONE == "BeamL")   translate([CTR_W + G, row2, 0]) beamHalf();
    if (PRINT_ONE == "" || PRINT_ONE == "BeamR")   translate([CTR_W + G, row2 + BEAM_W + G, 0]) beamHalf();
    row3 = row2 + CTR_D + SENSOR_TAB_D + G;          // 195
    if (PRINT_ONE == "" || PRINT_ONE == "PostL")   translate([0, row3, 0]) post();
    if (PRINT_ONE == "" || PRINT_ONE == "PostR")   translate([FOOT_W + G, row3, 0]) post();
    if (PRINT_ONE == "" || PRINT_ONE == "SpacerL") translate([2 * (FOOT_W + G), row3, 0]) spacer();
    if (PRINT_ONE == "" || PRINT_ONE == "SpacerR") translate([3 * (FOOT_W + G), row3, 0]) spacer();
    if (PRINT_ONE == "" || PRINT_ONE == "Sensor")  translate([2 * (FOOT_W + G), row3 + FOOT_D + G, 0]) sensorMount();
}

layout();
