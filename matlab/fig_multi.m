% FIG_MULTI  Several items on the belt at the design point (T = 0.4 s).
% (a) pairs fully sorted versus spacing; (b) outcomes in random streams.
C = re_setup();
here = fileparts(mfilename('fullpath'));
S = load(fullfile(here, 'data', 'fig_multi.mat'));

W = 3.5;  H = 1.9;
f = figure('Units', 'inches', 'Position', [1 1 W H], 'Color', 'w', 'Visible', 'off');

% (a) pairs
ax = axes('Position', [0.115 0.20 0.36 0.66]);
hold(ax, 'on');  box(ax, 'on');  grid(ax, 'on');
vc = {C.green, C.blue, C.red};
for i = 1:3
    plot(ax, S.spacing, S.pair_safe(i, :), '--', 'Color', vc{i}, 'LineWidth', 1.0);
    hs(i) = plot(ax, S.spacing, S.pair_fcfs(i, :), '-o', 'Color', vc{i}, 'LineWidth', 1.0, ...
                 'MarkerSize', 2.5, 'MarkerFaceColor', vc{i});
end
set(ax, 'XLim', [-20 620], 'YLim', [-0.02 1.16], 'YTick', 0:0.2:1, 'XTick', 0:200:600);
xlabel(ax, 'spacing (mm)');  ylabel(ax, 'pairs fully sorted');
text(0, 1.02, '(a) pairs, all class orders', 'Units', 'normalized', ...
     'VerticalAlignment', 'bottom', 'FontSize', 6.5);
re_legend(f, [0.31 0.24 0.16 0.24], {re_item('0.1 m/s', C.green, '-', 'o', C.green), ...
    re_item('0.2 m/s', C.blue, '-', 'o', C.blue), re_item('0.3 m/s', C.red, '-', 'o', C.red)}, 1, 5.5);

% (b) random streams at 0.2 m/s
ax = axes('Position', [0.615 0.20 0.36 0.66]);
hold(ax, 'on');  box(ax, 'on');  grid(ax, 'on');
oc = {C.blue, C.gray, C.red};
for i = 1:3
    plot(ax, S.rates, S.stream_safe(i, :), '--', 'Color', oc{i}, 'LineWidth', 1.0);
    ho(i) = plot(ax, S.rates, S.stream_fcfs(i, :), '-o', 'Color', oc{i}, 'LineWidth', 1.0, ...
                 'MarkerSize', 2.5, 'MarkerFaceColor', oc{i});
end
set(ax, 'XLim', [5 95], 'YLim', [-0.02 1.16], 'YTick', 0:0.2:1, 'XTick', [25 50 75]);
xlabel(ax, 'items per minute');  ylabel(ax, 'fraction of items');
text(0, 1.02, '(b) random stream, 0.2 m/s', 'Units', 'normalized', ...
     'VerticalAlignment', 'bottom', 'FontSize', 6.5);
re_legend(f, [0.62 0.775 0.35 0.07], {re_item('correct', C.blue, '-', 'o', C.blue), ...
    re_item('reject', C.gray, '-', 'o', C.gray), re_item('wrong', C.red, '-', 'o', C.red)}, 3, 5.5);
re_save(f, 'fig_multi', W, H);
close(f);
