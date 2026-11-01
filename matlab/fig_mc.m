% FIG_MC  Fraction of single items that reach their bin versus belt speed
% at the design point (T = 0.4 s, Lv = 300 mm, g = 200 mm).
C = re_setup();
here = fileparts(mfilename('fullpath'));
S = load(fullfile(here, 'data', 'fig_mc.mat'));

W = 3.5;  H = 3.2;
f = figure('Units', 'inches', 'Position', [1 1 W H], 'Color', 'w', 'Visible', 'off');
pos = {[0.135 0.465 0.835 0.285], [0.135 0.105 0.835 0.285]};
ttl = {'(a) plastic: side bin', '(b) organic: rear-right bin'};
for k = 1:2
    ax = axes('Position', pos{k});
    hold(ax, 'on');  box(ax, 'on');  grid(ax, 'on');
    if k == 1
        e0 = S.P_ed_pm0; e1 = S.P_ed_pm01; f0 = S.P_ff_pm0; f1 = S.P_ff_pm01; b = S.bound_P;
    else
        e0 = S.O_ed_pm0; e1 = S.O_ed_pm01; f0 = S.O_ff_pm0; f1 = S.O_ff_pm01; b = S.bound_O;
    end
    plot(ax, [b b], [-0.03 1.05], ':', 'Color', 'k', 'LineWidth', 0.6);
    h(1) = plot(ax, S.v_ed, e0, '-', 'Color', C.blue, 'LineWidth', 1.1);
    h(2) = plot(ax, S.v_ed, e1, '--', 'Color', C.blue, 'LineWidth', 1.1);
    h(3) = plot(ax, S.v_ff, f0, 'o', 'Color', C.red, 'MarkerFaceColor', C.red, 'MarkerSize', 3);
    h(4) = plot(ax, S.v_ff, f1, 'o', 'Color', C.red, 'MarkerFaceColor', 'w', 'MarkerSize', 3);
    if k == 1
        h(5) = plot(ax, S.v_best, S.P_best, ':s', 'Color', C.gray, 'MarkerFaceColor', C.gray, ...
                    'MarkerSize', 3, 'LineWidth', 0.8);
        ax1 = ax;  hp = h;
    else
        h(6) = plot(ax, S.v_ff, S.contam_ff, '^', 'Color', C.orange, 'MarkerFaceColor', C.orange, ...
                    'MarkerSize', 3);
        ho = h(6);
    end
    set(ax, 'XLim', [0.04 0.61], 'YLim', [-0.03 1.05], 'XTick', 0.1:0.1:0.6, ...
            'YTick', 0:0.25:1, 'TickDir', 'in');
    ylabel(ax, 'fraction of items');
    text(0, 1.02, ttl{k}, 'Units', 'normalized', 'VerticalAlignment', 'bottom', 'FontSize', 7);
    if k == 1
        set(ax, 'XTickLabel', []);
    else
        xlabel(ax, 'belt speed v (m/s)');
    end
end
items = {re_item('event-driven, p_m = 0', C.blue, '-'), ...
         re_item('first firmware, p_m = 0', C.red, 'none', 'o', C.red), ...
         re_item('event-driven, p_m = 0.1', C.blue, '--'), ...
         re_item('first firmware, p_m = 0.1', C.red, 'none', 'o', 'w'), ...
         re_item('first firmware, best t_d', C.gray, ':', 's', C.gray), ...
         re_item('first firmware, into plastic bin', C.orange, 'none', '^', C.orange)};
re_legend(f, [0.06 0.835 0.93 0.155], items, 2, 6);
re_save(f, 'fig_mc', W, H);
close(f);
