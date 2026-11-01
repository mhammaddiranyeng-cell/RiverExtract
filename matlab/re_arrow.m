function re_arrow(ax, p, q, col, lw, heads, hl)
%RE_ARROW Arrow from P to Q in data units on axes with equal aspect ratio.
%   HEADS = 1: head at Q; HEADS = 2: heads at both ends. HL: head length.
if nargin < 6, heads = 1; end
if nargin < 7, hl = 9; end
hold(ax, 'on');
d = (q - p) / norm(q - p);
n = [-d(2) d(1)];
hw = 0.38 * hl;
tipQ = q;  baseQ = q - hl * d;
if heads == 2
    tipP = p;  baseP = p + hl * d;
    plot(ax, [baseP(1) baseQ(1)], [baseP(2) baseQ(2)], '-', 'Color', col, 'LineWidth', lw);
    patch(ax, [tipP(1) baseP(1) + hw * n(1) baseP(1) - hw * n(1)], ...
              [tipP(2) baseP(2) + hw * n(2) baseP(2) - hw * n(2)], col, 'EdgeColor', 'none');
else
    plot(ax, [p(1) baseQ(1)], [p(2) baseQ(2)], '-', 'Color', col, 'LineWidth', lw);
end
patch(ax, [tipQ(1) baseQ(1) + hw * n(1) baseQ(1) - hw * n(1)], ...
          [tipQ(2) baseQ(2) + hw * n(2) baseQ(2) - hw * n(2)], col, 'EdgeColor', 'none');
end
