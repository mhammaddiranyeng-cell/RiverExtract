RiverExtract figures in MATLAB
==============================

Run make_all_figures.m from this folder (MATLAB R2018b or later, no toolboxes
needed; the scripts also run in GNU Octave 8.4). It writes figs/<name>.pdf
(vector) and figs/<name>.png (preview) for:

  fig_geometry  sorting stage to scale: plan view and section
  fig_mc        single items reaching their bin vs belt speed
  fig_multi     several items on the belt: pairs and streams
  fig_motion    gate stroke profiles
  fig_maps      planar contact simulation: outcome maps and item paths

The file names match the figures in ../figs/.

Data: data/*.mat holds the plotted values, exported from the simulation
results by ../sim/export_for_matlab.py. fig_geometry needs no data file; it is
drawn from the part dimensions.

Helper functions:
  re_setup   fonts, line widths, colour palette
  re_save    PDF/PNG export at the paper's figure size
  re_legend, re_item   compact legends that print the same in MATLAB and Octave
  re_cells   categorical maps drawn as vector rectangles
  re_plate   the 200 x 5 mm gate plate at a given angle
  re_arrow   arrows in data units

The architecture diagram (Fig. 1) and the PLC state diagram are drawings, not
plots; Fig. 1 comes from ../sim/fig_overview.py.
