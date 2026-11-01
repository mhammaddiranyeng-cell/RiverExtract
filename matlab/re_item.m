function it = re_item(label, color, ls, marker, mfc, ispatch)
%RE_ITEM Legend entry for RE_LEGEND.
if nargin < 3, ls = '-'; end
if nargin < 4, marker = 'none'; end
if nargin < 5, mfc = 'none'; end
if nargin < 6, ispatch = false; end
it = struct('label', label, 'color', color, 'ls', ls, 'marker', marker, 'mfc', mfc, ...
            'patch', ispatch);
end
