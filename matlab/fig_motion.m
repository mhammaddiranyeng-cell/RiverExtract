% FIG_MOTION  Gate stroke of 90 deg: ramped profiles and the constant-rate
% moves of the first firmware.
C = re_setup();
here = fileparts(mfilename('fullpath'));
S = load(fullfile(here, 'data', 'fig_motion.mat'));

W = 3.5;  H = 2.8;
f = figure('Units', 'inches', 'Position', [1 1 W H], 'Color', 'w', 'Visible', 'off');
col = {C.red, C.blue, C.green, C.red, C.blue};
sty = {'-', '-', '-', ':', ':'};
lab = {'strike, ramped', 'organic, ramped', 'return, capped', ...
       'first firmware, 500 step/s', 'first firmware, 250 step/s'};

a1 = axes('Position', [0.13 0.465 0.84 0.30]);
hold(a1, 'on');  box(a1, 'on');  grid(a1, 'on');
for k = 1:5
    h(k) = plot(a1, S.t, S.theta_deg(k, :), sty{k}, 'Color', col{k}, 'LineWidth', 1.0);
end
for k = 1:3
    plot(a1, S.t_end(k), 90, 'o', 'Color', col{k}, 'MarkerFaceColor', col{k}, 'MarkerSize', 2.5);
end
plot(a1, S.tick_t, S.tick_deg, '|', 'Color', C.red, 'MarkerSize', 4);
set(a1, 'XLim', [0 0.62], 'YLim', [0 98], 'YTick', [0 30 60 90], 'XTickLabel', []);
ylabel(a1, '\Delta\theta (deg)');

a2 = axes('Position', [0.13 0.115 0.84 0.30]);
hold(a2, 'on');  box(a2, 'on');  grid(a2, 'on');
for k = 1:5
    plot(a2, S.t, S.omega(k, :), sty{k}, 'Color', col{k}, 'LineWidth', 1.0);
end
set(a2, 'XLim', [0 0.62], 'YLim', [0 24]);
ylabel(a2, '\omega (rad/s)');  xlabel(a2, 'time (s)');
text(a2, 0.115, 17.4, '\leftarrow step to 15.7 rad/s', 'FontSize', 6);
text(a2, 0.215, 9.4, '\leftarrow step to 7.9 rad/s', 'FontSize', 6);

items = cell(1, 5);
for k = 1:5
    items{k} = re_item(lab{k}, col{k}, sty{k});
end
re_legend(f, [0.06 0.80 0.93 0.19], items, 2, 5.8);
re_save(f, 'fig_motion', W, H);
close(f);
