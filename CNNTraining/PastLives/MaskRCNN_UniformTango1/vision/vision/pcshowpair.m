function ax = pcshowpair(ptCloudA, ptCloudB, varargin) 

% Copyright 2015-2024 The MathWorks, Inc.

params = validateAndParseInputs(ptCloudA, ptCloudB, varargin{:});

currentAxes = params.Parent;
[currentAxes, hFigure] = pointclouds.internal.pcui.setupAxesAndFigure(currentAxes);

markerSize  = params.MarkerSize;
blendFactor = params.BlendFactor;

scatterObj = plotPointCloud(currentAxes, ptCloudA, markerSize, blendFactor, 'first');
tf = ishold(currentAxes);
if ~tf
    hold(currentAxes, 'on');
    plotPointCloud(currentAxes, ptCloudB, markerSize, blendFactor, 'second');
    hold(currentAxes, 'off');
else
    plotPointCloud(currentAxes, ptCloudB, markerSize, blendFactor, 'second');
end

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
end
end

%-------------------------------------------------------------------------- 
function hObj = plotPointCloud(currentAxes, ptCloud, markerSize, blendFactor, colorAssignment)
C = getColorValues(ptCloud, blendFactor, colorAssignment);

count = ptCloud.Count;
X = ptCloud.Location(1:count);
Y = ptCloud.Location(count+1:count*2);
Z = ptCloud.Location(count*2+1:end);
hObj = scatter3(currentAxes, X, Y, Z, markerSize, C, '.', 'Tag', 'pcviewer');

pointclouds.internal.pcui.utils.setAppData(hObj, 'PointCloud', ptCloud);
pointclouds.internal.pcui.utils.setAppData(hObj, 'MagGrColor', C);
end

%-------------------------------------------------------------------------- 
function C = getColorValues(ptCloud, blendFactor, colorAssignment)

    if isempty(ptCloud.Color)
        if strcmpi(colorAssignment, 'first')
            C = [blendFactor, 0, blendFactor];
        else
            C = [0, blendFactor, 0];
        end
    else
        C = im2double(ptCloud.Color);
        if ~ismatrix(C)
            C = reshape(C, [], 3);
        end
        if strcmpi(colorAssignment, 'first')
            C(:, [1,3]) = C(:, [1,3]) * (1 - blendFactor) + blendFactor;
        else
            C(:, 2) = C(:, 2) * (1 - blendFactor) + blendFactor;
        end
    end
    
end

%--------------------------------------------------------------------------
function params = validateAndParseInputs(ptCloudA, ptCloudB, varargin)

validateattributes(ptCloudA, {'pointCloud'}, {'scalar'}, 'pcshowpair', 'ptCloudA');
validateattributes(ptCloudB, {'pointCloud'}, {'scalar'}, 'pcshowpair', 'ptCloudB');

params = validateAndParseOptInputs(varargin{:});

colorSource = params.ColorSource;
pointclouds.internal.pcui.validateColorSourceWithPC(colorSource, ptCloudA);
pointclouds.internal.pcui.validateColorSourceWithPC(colorSource, ptCloudB);
end

%-------------------------------------------------------------------------- 
function params = validateAndParseOptInputs(varargin)

parser = pointclouds.internal.pcui.getSharedParamParser(mfilename);

parser.addParameter('BlendFactor', 0.7, ...
            @(x)validateattributes(x, {'single', 'double'}, ...
                {'real', 'scalar', '>=', 0, '<=', 1}, mfilename, 'BlendFactor'));

uiaxesSupported = true;
parser.addParameter('Parent', [], @(p)vision.internal.inputValidation.validateAxesHandle(p, uiaxesSupported));

parser.parse(varargin{:});
    
params = parser.Results;

end