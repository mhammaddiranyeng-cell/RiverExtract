function ax = re_legend(f, pos, items, ncol, fs)
%RE_LEGEND Compact legend drawn with plain graphics objects, so that it looks
%   the same in MATLAB and GNU Octave and prints cleanly to PDF.
%   POS   [x y w h] in normalized figure units
%   ITEMS cell array of structs with fields
%         label  text
%         color  RGB triplet
%         ls     line style ('-', '--', ':', '-.' or 'none')
%         marker marker symbol or 'none'
%         mfc    marker face colour (RGB, 'none' or 'w')
%         patch  true for a filled square (colour swatch)
%   NCOL  number of columns (filled row by row); FS font size in points.
if nargin < 4, ncol = 1; end
if nargin < 5, fs = 6; end
ax = axes('Parent', f, 'Position', pos, 'Visible', 'off', 'XLim', [0 1], 'YLim', [0 1]);
hold(ax, 'on');
n = numel(items);
nrow = ceil(n / ncol);
cw = 1 / ncol;  rh = 1 / nrow;
L = min(0.16 * cw / 0.5, 0.16);           % sample length
for i = 1:n
    it = items{i};
    r = floor((i - 1) / ncol);  c = mod(i - 1, ncol);
    x0 = c * cw + 0.01;  y = 1 - (r + 0.5) * rh;
    if isfield(it, 'patch') && it.patch
        hh = 0.32 * rh;
        patch(ax, x0 + [0 0.45 0.45 0] * L, y + [-1 -1 1 1] * hh, it.color, ...
              'EdgeColor', 'k', 'LineWidth', 0.3);
        xt = x0 + 0.45 * L + 0.02;
    else
        if ~strcmp(it.ls, 'none')
            plot(ax, x0 + [0 L], [y y], it.ls, 'Color', it.color, 'LineWidth', 1.0);
        end
        if ~strcmp(it.marker, 'none')
            plot(ax, x0 + L / 2, y, it.marker, 'Color', it.color, 'MarkerFaceColor', it.mfc, ...
                 'MarkerSize', 3);
        end
        xt = x0 + L + 0.02;
    end
    text(ax, xt, y, it.label, 'FontSize', fs, 'VerticalAlignment', 'middle');
end
end
