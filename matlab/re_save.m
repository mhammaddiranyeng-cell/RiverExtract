function re_save(f, name, w, h)
%RE_SAVE Save figure F as figs/NAME.pdf (vector) and figs/NAME.png at W x H inches.
set(f, 'PaperUnits', 'inches', 'PaperSize', [w h], 'PaperPosition', [0 0 w h]);
if ~exist('OCTAVE_VERSION', 'builtin')
    set(f, 'Renderer', 'painters');          % vector output in MATLAB
end
outdir = fullfile(fileparts(mfilename('fullpath')), 'figs');
if ~exist(outdir, 'dir')
    mkdir(outdir);
end
print(f, fullfile(outdir, [name '.pdf']), '-dpdf');
print(f, fullfile(outdir, [name '.png']), '-dpng', '-r220');
end
