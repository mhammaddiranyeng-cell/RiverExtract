function re_cells(ax, x, y, M, cmap)
%RE_CELLS Draw a categorical map as vector rectangles (one patch, runs of
%   equal value merged along x), so that it prints as vector graphics.
%   M(j, i) is the 0-based category at x(i), y(j); CMAP holds one RGB per row.
dx = x(2) - x(1);  dy = y(2) - y(1);
V = zeros(0, 2);  F = zeros(0, 4);  Cd = zeros(0, 3);
for j = 1:numel(y)
    row = M(j, :);
    i0 = 1;
    for i = 2:numel(x) + 1
        if i > numel(x) || row(i) ~= row(i0)
            xa = x(i0) - dx/2;  xb = x(i - 1) + dx/2;
            if i <= numel(x), xb = xb + 0.2 * dx; end              % overlap inside, no seams
            ya = y(j) - dy/2;   yb = y(j) + dy/2;
            if j < numel(y), yb = yb + 0.2 * dy; end
            k = size(V, 1);
            V = [V; xa ya; xb ya; xb yb; xa yb];                     %#ok<AGROW>
            F = [F; k + (1:4)];                                      %#ok<AGROW>
            Cd = [Cd; cmap(row(i0) + 1, :)];                         %#ok<AGROW>
            i0 = i;
        end
    end
end
patch(ax, 'Vertices', V, 'Faces', F, 'FaceVertexCData', Cd, 'FaceColor', 'flat', ...
      'EdgeColor', 'none');
end
