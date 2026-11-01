function C = re_setup()
%RE_SETUP Common settings for the RiverExtract paper figures.
%   C = RE_SETUP() sets default fonts and line widths and returns the
%   colour palette used in every figure. Works in MATLAB (R2018b or later)
%   and GNU Octave (7 or later, Qt graphics toolkit).

if exist('OCTAVE_VERSION', 'builtin')
    graphics_toolkit('qt');
end
set(0, 'DefaultAxesFontName', 'Helvetica', 'DefaultTextFontName', 'Helvetica', ...
       'DefaultAxesFontSize', 7, 'DefaultTextFontSize', 7, ...
       'DefaultLineLineWidth', 1.0, 'DefaultAxesLineWidth', 0.5);

C.blue   = [31 78 156] / 255;
C.orange = [217 130 43] / 255;
C.green  = [46 125 79] / 255;
C.red    = [178 58 58] / 255;
C.gray   = [122 122 122] / 255;
C.lgray  = [217 217 217] / 255;
C.purple = [106 61 154] / 255;
C.dark   = [34 34 34] / 255;
end
