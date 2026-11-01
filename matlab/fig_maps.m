% FIG_MAPS  Planar contact simulation at v = 0.2 m/s, w = 65 mm, mu = 0.5.
% (a) bin reached after one strike sequence versus the item centroid at
% stroke start; (b) the same for the organic sequence; (c) item paths for
% four strike timings with y0 = 0.
C = re_setup();
here = fileparts(mfilename('fullpath'));
S = load(fullfile(here, 'data', 'fig_maps.mat'));

W = 3.5;  H = 3.95;
f = figure('Units', 'inches', 'Position', [1 1 W H], 'Color', 'w', 'Visible', 'off');
% outcome codes 0..5: side bin, post, rear-left, rear-right, right edge, untouched
cmap = [C.red; C.dark; 0.71 0.71 0.71; 0.18 0.545 0.341; 0.557 0.267 0.678; 1 1 1];
ttl = {'(a) plastic strike: centroid at stroke start', '(b) organic move: centroid at move start'};
M = {S.map_strike, S.map_organic};
pos = {[0.13 0.790 0.85 0.175], [0.13 0.555 0.85 0.175]};
dx = S.xm(2) - S.xm(1);  dy = S.ym(2) - S.ym(1);
for k = 1:2
    ax = axes('Position', pos{k});
    hold(ax, 'on');  box(ax, 'on');
    re_cells(ax, S.xm, S.ym, M{k}, cmap);
    set(ax, 'XLim', [S.xm(1) - dx/2, S.xm(end) + dx/2], 'YLim', [S.ym(1) - dy/2, S.ym(end) + dy/2], ...
            'XTick', -300:100:0, 'YTick', [-35 0 35], ...
            'Layer', 'top');
    ylabel(ax, 'y_0 (mm)');
    text(0, 1.03, ttl{k}, 'Units', 'normalized', 'VerticalAlignment', 'bottom', 'FontSize', 7);
    if k == 1
        set(ax, 'XTickLabel', []);
    else
        xlabel(ax, 'x_0 (mm)');
    end
end

% (c) item paths
ax = axes('Position', [0.13 0.055 0.60 0.36]);
hold(ax, 'on');  box(ax, 'on');
P_IN = 135;  PX = 32;  bg = [0.933 0.957 0.937];
patch(ax, [-300 120 120 -300], [-P_IN -P_IN P_IN P_IN], bg, 'EdgeColor', 'none');
plot(ax, [-300 120], [P_IN P_IN], '-', 'Color', C.green, 'LineWidth', 0.6);
plot(ax, [-300 120], [-P_IN -P_IN], '-', 'Color', C.green, 'LineWidth', 0.6);
% posts, drawn only inside the plotted area (|y| <= 150 mm)
patch(ax, [-PX/2 PX/2 PX/2 -PX/2], [P_IN P_IN 150 150], C.orange, 'EdgeColor', 'k', 'LineWidth', 0.4);
patch(ax, [-PX/2 PX/2 PX/2 -PX/2], [-150 -150 -P_IN -P_IN], C.orange, 'EdgeColor', 'k', 'LineWidth', 0.4);
% gate at +45 (rest), 22.5, 0 and -45 deg; lighter shades instead of transparency
ang = [deg2rad(22.5) 0 -pi/4 pi/4];  alp = [0.35 0.35 0.35 0.9];
for i = 1:4
    re_plate(ax, ang(i), 'FaceColor', alp(i) * C.blue + (1 - alp(i)) * bg, 'EdgeColor', 'none');
end
names = {'early', 'hit', 'blocked', 'late'};
cols  = {C.green, C.red, C.dark, C.gray};
labs  = {'early: rear-right', 'hit: side bin', 'near hub: post', 'late: rear-left'};
for i = 1:4
    x = S.(['path_' names{i} '_x']);  y = S.(['path_' names{i} '_y']);
    plot(ax, x, y, '-', 'Color', cols{i}, 'LineWidth', 1.0);
    plot(ax, x(1), y(1), 'o', 'Color', cols{i}, 'MarkerFaceColor', cols{i}, 'MarkerSize', 2.5);
    plot(ax, x(end), y(end), 'x', 'Color', cols{i}, 'MarkerSize', 4);
end
set(ax, 'XLim', [-300 120], 'YLim', [-150 150], 'DataAspectRatio', [1 1 1]);
xlabel(ax, 'x (mm)');  ylabel(ax, 'y (mm)');
text(0, 1.03, '(c) item paths during one strike sequence', 'Units', 'normalized', ...
     'VerticalAlignment', 'bottom', 'FontSize', 7);

% legends: outcome colours of (a), (b) and the paths of (c)
oc = {'side bin', 'post', 'rear-left', 'rear-right'};
items = cell(1, 4);
for i = 1:4
    items{i} = re_item(oc{i}, cmap(i, :), 'none', 'none', 'none', true);
end
re_legend(f, [0.755 0.255 0.24 0.15], items, 1, 5.6);
text(0.755, 0.415, '(a), (b) outcome', 'Units', 'normalized', 'FontSize', 5.6, ...
     'Parent', axes('Position', [0 0 1 1], 'Visible', 'off'));
items = cell(1, 4);
for i = 1:4
    items{i} = re_item(labs{i}, cols{i}, '-');
end
re_legend(f, [0.755 0.065 0.24 0.15], items, 1, 5.6);
text(0.755, 0.225, '(c) paths', 'Units', 'normalized', 'FontSize', 5.6, ...
     'Parent', axes('Position', [0 0 1 1], 'Visible', 'off'));
re_save(f, 'fig_maps', W, H);
close(f);
