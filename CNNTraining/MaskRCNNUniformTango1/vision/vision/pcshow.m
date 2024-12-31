function ax = pcshow(varargin)

% Copyright 2013-2023 The MathWorks, Inc.

[X, Y, Z, C, map, ptCloud, params] = validateAndParseInputs(varargin{:});
                                
currentAxes = params.Parent;
[currentAxes, hFigure] = pointclouds.internal.pcui.setupAxesAndFigure(currentAxes);

% Set the colormap
if ~isempty(map)
    colormap(hFigure, map);
end

% Set the colordata for storing
if isempty(C)
    C = Z;
    colorData = [];
else
    if isempty(ptCloud.Color) && isempty(ptCloud.Intensity)
        colorData = C;
    else
        % This means point cloud object holds the color information
        colorData = []; 
    end
end

markerSize = params.MarkerSize;
scatterObj = scatter3(currentAxes, X, Y, Z, markerSize, C, '.', 'Tag', 'pcviewer');

if ischar(C)
    % This is done to use a numeric value for color data
    colorData = scatterObj.CData;
end

pointclouds.internal.pcui.utils.setAppData(scatterObj, 'PointCloud', ptCloud);
pointclouds.internal.pcui.utils.setAppData(scatterObj, 'ColorData', colorData);

% Lower and upper limit of auto downsampling.
ptCloudThreshold = [1920*1080, 1e8]; 
params.PtCloudThreshold = ptCloudThreshold;

% Equal axis is required for cameratoolbar
axis(currentAxes, 'equal');

% Initialize point cloud viewer controls.
pointclouds.internal.pcui.initializePCSceneControl(hFigure, currentAxes,...
    scatterObj, params);

if nargout > 0
    ax = currentAxes;
    
    % Disable default interactions
    disableDefaultInteractivity(ax);
end

end

%-------------------------------------------------------------------------- 
function [X, Y, Z, C, map, ptCloud, params] = ...
    validateAndParseInputs(varargin)
% Validate and parse inputs
narginchk(1, 20);

% the 2nd argument is C only if the number of arguments is even and the
% first argument is not a pointCloud object
if  ~bitget(nargin, 1) && ~isa(varargin{1}, 'pointCloud')
    [X, Y, Z, C, map, ptCloud] = pointclouds.internal.pcui.validateAndParseInputsXYZC(mfilename, varargin{1:2});
    pvpairs = varargin(3:end);
else
    [X, Y, Z, C, map, ptCloud] = pointclouds.internal.pcui.validateAndParseInputsXYZC(mfilename, varargin{1});
    pvpairs = varargin(2:end);
end

parser = pointclouds.internal.pcui.getSharedParamParser(mfilename);

uiaxesSupported = true;
parser.addParameter('Parent', [], ...
    @(p)vision.internal.inputValidation.validateAxesHandle(p, uiaxesSupported));

parser.parse(pvpairs{:});
    
params = parser.Results;

colorSource = params.ColorSource;
pointclouds.internal.pcui.validateColorSourceWithPC(colorSource, ptCloud);

end
