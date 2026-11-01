% FIG_GEOMETRY  Sorting stage to scale (mm). (a) Plan view at the design point
% Lv = 300 mm, g = 200 mm; (b) section through the pivot.
C = re_setup();
LV = 300;  GAP = 200;  P_IN = 135;  PX = 32;  R = 100;  RS = 100.03;  BAND = 70.71;
HUB = 18;  HUB_H = 20;  CLEAT = 50;  G_LOW = 55;  G_HIGH = 135;  PH = 80;

W = 7.16;  H = 2.25;
f = figure('Units', 'inches', 'Position', [1 1 W H], 'Color', 'w', 'Visible', 'off');

% ------------------------------------------------------------ (a) plan view
ax = axes('Position', [0.05 0.17 0.57 0.72]);
hold(ax, 'on');  box(ax, 'on');
x0 = -560;  x1 = 300;
patch(ax, [x0 x1 x1 x0], [-P_IN -P_IN P_IN P_IN], [0.906 0.945 0.918], 'EdgeColor', 'none');
plot(ax, [x0 x1], [P_IN P_IN], '-', 'Color', C.green, 'LineWidth', 0.8);
plot(ax, [x0 x1], [-P_IN -P_IN], '-', 'Color', C.green, 'LineWidth', 0.8);
rectangle(ax, 'Position', [-GAP-LV -P_IN LV 2*P_IN], 'FaceColor', [0.875 0.906 0.969], ...
          'EdgeColor', C.blue, 'LineStyle', '--', 'LineWidth', 0.6);
text(ax, -GAP - LV/2, -P_IN + 14, 'detection window L_v', 'HorizontalAlignment', 'center', ...
     'VerticalAlignment', 'bottom', 'FontSize', 6.5, 'Color', C.blue);
for s = [1 -1]
    plot(ax, [x0+10 -GAP-LV-20 -GAP-40], s * [128 75 75], '-', 'Color', [0.33 0.33 0.33], 'LineWidth', 1.6);
    plot(ax, [x0 x1], s * [BAND BAND], '-.', 'Color', C.gray, 'LineWidth', 0.5);
end
text(ax, -400, 82, 'lane guides', 'FontSize', 6, 'Color', [0.27 0.27 0.27], ...
     'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
rectangle(ax, 'Position', [-PX/2 P_IN PX PX], 'FaceColor', C.orange, 'EdgeColor', 'k', 'LineWidth', 0.4);
rectangle(ax, 'Position', [-PX/2 -P_IN-PX PX PX], 'FaceColor', C.orange, 'EdgeColor', 'k', 'LineWidth', 0.4);
a = linspace(0, 2*pi, 200);
plot(ax, RS * cos(a), RS * sin(a), ':', 'Color', C.gray, 'LineWidth', 0.5);
a = linspace(deg2rad(135), deg2rad(225), 60);
patch(ax, [0 R*cos(a) 0], [0 R*sin(a) 0], [0.965 0.843 0.843], 'EdgeColor', C.red, 'LineWidth', 0.5);
text(ax, -62, 0, {'strike', 'sector'}, 'HorizontalAlignment', 'center', 'FontSize', 6, 'Color', C.red);
re_plate(ax, -pi/4, 'FaceColor', 'none', 'EdgeColor', C.blue, 'LineStyle', '--', 'LineWidth', 0.7);
re_plate(ax, pi/4, 'FaceColor', C.blue, 'EdgeColor', C.blue, 'LineWidth', 0.5);
a = linspace(0, 2*pi, 60);
patch(ax, HUB/2 * cos(a), HUB/2 * sin(a), [1 1 1], 'EdgeColor', 'k', 'LineWidth', 0.5);
text(ax, 80, 102, 'rest +45^\circ', 'FontSize', 6.5, 'Color', C.blue, 'HorizontalAlignment', 'center');
text(ax, 84, -97, 'organic -45^\circ', 'FontSize', 6.5, 'Color', C.blue, 'HorizontalAlignment', 'center');
re_arrow(ax, [-95 112], [-95 168], C.red, 0.9, 1, 10);
text(ax, -95, 172, 'side bin (PLASTIC)', 'HorizontalAlignment', 'center', ...
     'VerticalAlignment', 'bottom', 'FontSize', 6.5, 'Color', C.red);
plot(ax, [215 x1], [0 0], '-k', 'LineWidth', 0.8);
text(ax, x1 - 4, 40, {'OTHER', '(rear-left)'}, 'HorizontalAlignment', 'right', 'FontSize', 6.5);
text(ax, x1 - 4, -40, {'ORGANIC', '(rear-right)'}, 'HorizontalAlignment', 'right', 'FontSize', 6.5);
re_arrow(ax, [-160 112], [-100 112], 'k', 0.7, 1, 10);
text(ax, -170, 112, 'belt v', 'HorizontalAlignment', 'right', 'FontSize', 6.5);
re_arrow(ax, [-GAP -150], [0 -150], 'k', 0.5, 2, 7);
text(ax, -GAP/2, -159, 'g', 'HorizontalAlignment', 'center', 'FontSize', 6);
re_arrow(ax, [-GAP-LV -150], [-GAP -150], 'k', 0.5, 2, 7);
text(ax, -GAP - LV/2, -159, 'L_v', 'HorizontalAlignment', 'center', 'FontSize', 6);
text(ax, -8, -117, 'r = 100', 'FontSize', 6, 'Color', C.gray, 'HorizontalAlignment', 'center');
re_arrow(ax, [150 BAND], [150 -BAND], 'k', 0.5, 2, 7);
text(ax, 159, -40, '141', 'Rotation', 90, 'HorizontalAlignment', 'center', 'FontSize', 6);
set(ax, 'XLim', [x0 x1], 'YLim', [-175 195], 'DataAspectRatio', [1 1 1], ...
        'XTick', -500:100:300, 'YTick', [-135 0 135]);
xlabel(ax, 'x (mm), downstream');  ylabel(ax, 'y (mm), left');
text(0, 1.02, '(a) plan view at the design point', 'Units', 'normalized', ...
     'VerticalAlignment', 'bottom', 'FontSize', 7.5);

% ---------------------------------------------------------- (b) section
ax = axes('Position', [0.645 0.17 0.315 0.72]);
hold(ax, 'on');  box(ax, 'on');
rectangle(ax, 'Position', [-150 -6 300 6], 'FaceColor', C.green, 'EdgeColor', 'none');
rectangle(ax, 'Position', [-125 0 250 CLEAT], 'FaceColor', [0.737 0.851 0.776], ...
          'EdgeColor', C.green, 'LineWidth', 0.5);
text(ax, 0, 18, 'cleat (behind), 50 mm', 'HorizontalAlignment', 'center', 'FontSize', 6, 'Color', C.green);
off = 10;
rectangle(ax, 'Position', [P_IN 0 40 off], 'FaceColor', C.lgray, 'EdgeColor', 'k', 'LineWidth', 0.3);
rectangle(ax, 'Position', [-P_IN-40 0 40 off], 'FaceColor', C.lgray, 'EdgeColor', 'k', 'LineWidth', 0.3);
rectangle(ax, 'Position', [P_IN+4 off 32 140], 'FaceColor', C.orange, 'EdgeColor', 'k', 'LineWidth', 0.4);
rectangle(ax, 'Position', [-P_IN-36 off 32 140], 'FaceColor', C.orange, 'EdgeColor', 'k', 'LineWidth', 0.4);
zb = off + 140;
rectangle(ax, 'Position', [-161 zb 150 6], 'FaceColor', C.orange, 'EdgeColor', 'k', 'LineWidth', 0.4);
rectangle(ax, 'Position', [11 zb 150 6], 'FaceColor', C.orange, 'EdgeColor', 'k', 'LineWidth', 0.4);
rectangle(ax, 'Position', [-31 zb+6 62 6], 'FaceColor', C.orange, 'EdgeColor', 'k', 'LineWidth', 0.4);
rectangle(ax, 'Position', [-21.15 zb+12 42.3 40], 'FaceColor', [0.227 0.227 0.227], 'EdgeColor', 'k', 'LineWidth', 0.4);
text(ax, 0, zb + 32, 'motor', 'Color', 'w', 'HorizontalAlignment', 'center', 'FontSize', 5);
hub_top = zb + 5;
rectangle(ax, 'Position', [-HUB/2 hub_top-HUB_H HUB HUB_H], 'FaceColor', C.blue, 'EdgeColor', 'k', 'LineWidth', 0.4);
rectangle(ax, 'Position', [-R G_LOW 2*R PH], 'FaceColor', [0.624 0.718 0.890], 'EdgeColor', C.blue, 'LineWidth', 0.6);
text(ax, 0, G_LOW + PH/2, 'gate 200 x 80 x 5', 'HorizontalAlignment', 'center', 'FontSize', 6);
plot(ax, [-130 130], [G_LOW G_LOW], '--', 'Color', C.red, 'LineWidth', 0.5);
text(ax, -127, G_LOW + 2, '55', 'FontSize', 6, 'Color', C.red, 'VerticalAlignment', 'bottom');
plot(ax, [-130 130], [G_HIGH G_HIGH], '--', 'Color', C.red, 'LineWidth', 0.5);
text(ax, -127, G_HIGH + 2, '135', 'FontSize', 6, 'Color', C.red, 'VerticalAlignment', 'bottom');
set(ax, 'XLim', [-180 180], 'YLim', [-10 215], 'DataAspectRatio', [1 1 1], ...
        'YAxisLocation', 'right', 'YTick', 0:50:200, 'XTick', [-100 0 100]);
xlabel(ax, 'y (mm)');  ylabel(ax, 'z (mm)');
text(0, 1.02, '(b) section through the pivot', 'Units', 'normalized', ...
     'VerticalAlignment', 'bottom', 'FontSize', 7.5);
re_save(f, 'fig_geometry', W, H);
close(f);
