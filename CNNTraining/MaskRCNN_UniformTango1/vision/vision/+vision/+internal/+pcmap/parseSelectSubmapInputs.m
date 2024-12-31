%vision.internal.pcmap.parseSelectSubmapInputs Parse inputs for
%the selectSubmap method of the map representations.

% Copyright 2020 The MathWorks, Inc.

function roi = parseSelectSubmapInputs(xlims, ylims, zlims, varargin)
%parseSelectSubmapInputs Get and validate the ROI for selectSubmap
%   For narargin == 4, the submap is provided as an ROI and for 
%   narargin == 5, the submap is provided as a center and a size
%   which is converted to an ROI.

%#codegen

narginchk(4,5);

if nargin==4
    roi = varargin{1};
    validateattributes(roi, {'single', 'double'}, ...
        {'numel', 6, 'vector', 'real', 'nonsparse', 'nonnan'}, ...
        'selectSubmap', 'roi');
    
    if (roi(1) >= roi(2) || roi(3) >= roi(4) || roi(5) >= roi(6))
        coder.internal.error('vision:pointcloud:invalidROI');
    end
elseif nargin==5
    center = varargin{1};
    sz     = varargin{2};
    validateattributes(center, {'single', 'double'}, ...
        {'numel', 3, 'real', 'nonsparse', 'finite'}, ...
        'selectSubmap', 'center');
    validateattributes(sz, {'single', 'double'}, ...
        {'numel', 3, 'real', 'nonsparse', 'finite'}, ...
        'selectSubmap', 'sz');
    
    roi = centerSizeToROI(center, sz);
end

roi = clampSubmapToMapLimits(double(roi), xlims, ylims, zlims);

end

function roi = centerSizeToROI(center, sz)
% Finds the ROI that corresponds to a center and size
halfSize = sz/2;

roi = [center(1)-halfSize(1), center(1) + halfSize(1), ...
       center(2)-halfSize(2), center(2) + halfSize(2), ...
       center(3)-halfSize(3), center(3) + halfSize(3)];
end

function roi = clampSubmapToMapLimits(roi, xlims, ylims, zlims)

roi(1) = max(roi(1), xlims(1));
roi(2) = min(roi(2), xlims(2));

roi(3) = max(roi(3), ylims(1));
roi(4) = min(roi(4), ylims(2));

roi(5) = max(roi(5), zlims(1));
roi(6) = min(roi(6), zlims(2));

if ~isrow(roi)
    roi = reshape(roi, 1, 6);
end
end