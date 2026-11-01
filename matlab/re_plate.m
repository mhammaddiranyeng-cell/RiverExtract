function h = re_plate(ax, th, varargin)
%RE_PLATE Draw the 200 x 5 mm gate plate at angle TH (rad) about the pivot.
R = 100;  t2 = 2.5;
u = [cos(th) sin(th)];
n = [-sin(th) cos(th)];
P = [-R * u + t2 * n; R * u + t2 * n; R * u - t2 * n; -R * u - t2 * n];
h = patch(ax, P(:, 1), P(:, 2), [1 1 1], varargin{:});
end
