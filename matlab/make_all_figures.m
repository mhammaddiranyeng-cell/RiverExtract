% MAKE_ALL_FIGURES  Regenerate every plotted figure of the RiverExtract papers.
% Run from this folder in MATLAB (or GNU Octave). Figures are written to
% ./figs as PDF (for the papers) and PNG (preview), with the same file names
% the LaTeX sources use, so they can be copied straight into figs/ of the
% Overleaf projects. The data come from ./data/*.mat, exported from the
% Python simulation (sim/export_for_matlab.py).
here = fileparts(mfilename('fullpath'));
addpath(here);
scripts = {'fig_geometry', 'fig_motion', 'fig_maps', 'fig_mc', 'fig_multi'};
for i = 1:numel(scripts)
    fprintf('%s ... ', scripts{i});
    run(fullfile(here, [scripts{i} '.m']));
    fprintf('done\n');
end
