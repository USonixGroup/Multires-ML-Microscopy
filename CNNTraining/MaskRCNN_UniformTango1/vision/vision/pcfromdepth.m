function ptCloud = pcfromdepth(varargin)

% Copyright 2022-2023 The MathWorks, Inc.

%#codegen
% For GPU codegen support
coder.gpu.kernelfun;

% Validate and parse PV pairs
if isSimMode()
    [depthImage, depthScaleFactor, intrinsics, ...
     imagePoints, colorImage, depthRange, roi] = ...
     validateAndParseInputs(varargin{:});
else
    [depthImage, depthScaleFactor, intrinsics, ...
     imagePoints, colorImage, depthRange, roi] = ...
     validateAndParseInputsCodegen(varargin{:});
end

imageSize = size(depthImage);
collectColor = ~isempty(colorImage);
if isSimMode()
    isOrganizedPtCould = isempty(imagePoints);
else
    if size(imagePoints, 2) == 0
        isOrganizedPtCould = coder.const(true);
    else
        isOrganizedPtCould = coder.const(false);
    end
end

if isOrganizedPtCould % Fetch all pixel coordinates in image
    [row, col] = meshgrid(1:imageSize(1),1:imageSize(2));
    points1 = [col(:), row(:)];
    points = cast(points1, class(imagePoints));
else
    points = imagePoints;
end

% First filter out points based on ROI
points = points(points(:,2) <= (roi(2) + roi(4) - 1) & points(:,2) >= roi(2) & points(:,1) <= (roi(1) + roi(3) - 1) & points(:,1) >= roi(1),:);

if isOrganizedPtCould
    % By default all values are nan
    xyzPoints   = nan(imageSize(1), imageSize(2), 3, 'single');
else
    if collectColor
        colorComponent = zeros(size(points, 1), 3, 'uint8');
    end
    
    % By default all values are nan
    xyzPoints   = nan(size(points, 1), 3, 'single');
end

for i = 1:size(points, 1)
    r = floor(points(i, 2));
    c = floor(points(i, 1));

    % Compute X, Y, Z
    Z  = double(depthImage(r, c)) / depthScaleFactor;
    XY = ((points(i, :) - intrinsics.PrincipalPoint) ./ intrinsics.FocalLength) * Z;
    
    % Filter valid points using depth range
    if Z >= depthRange(1) && Z <= depthRange(2)
        if isOrganizedPtCould
            xyzPoints(r,c, :)= [XY, Z];
        else
            xyzPoints(i, :)= [XY, Z];
            if collectColor
                colorComponent(i, :) = colorImage(r,c,:);
            end
        end
    end
end

% Create point cloud
if collectColor
    if isOrganizedPtCould
        ptCloud = pointCloud(xyzPoints, Color=colorImage);
    else
        ptCloud = pointCloud(xyzPoints, Color=colorComponent);
    end
else
    ptCloud = pointCloud(xyzPoints);
end

%==========================================================================
function [depthImage, depthScaleFactor, intrinsics, ...
          imagePoints, colorImage, depthRange, roi] = ...
          validateAndParseInputs(depthImage, depthScaleFactor, intrinsics, varargin)

narginchk(3,11);

% Validate positional args
checkDepthImage(depthImage);
checkScaleFactor(depthScaleFactor);
checkIntrinsics(intrinsics);

depthImageSize = size(depthImage);

defaults = getDefaultParametersVal(depthImageSize);

% Parse the PV pairs
parser = inputParser;
parser.addParameter('ImagePoints', defaults.ImagePoints, @checkImagePoints);
parser.addParameter('ColorImage',  defaults.ColorImage,  @checkColorImage);
parser.addParameter('DepthRange',  defaults.DepthRange,  @checkDepthRange);
parser.addParameter('ROI',  defaults.ROI,  @(x)vision.internal.detector.checkROI(x,depthImageSize));

% Parse input
parser.parse(varargin{:});

% Populate the parameters
imagePoints = parser.Results.ImagePoints;
colorImage  = parser.Results.ColorImage;
depthRange  = parser.Results.DepthRange;
roi   = parser.Results.ROI;

if ~isnumeric(imagePoints)
    imagePoints  = imagePoints.Location;
end

if ~isempty(colorImage)
    % Validate the resolution of the input
    % The size of two input images must be the same
    if any(depthImageSize ~= size(colorImage,[1 2]))
        error(message('vision:pointcloud:mismatchDepthToColor'));
    end
end

%==========================================================================
function [depthImage, depthScaleFactor, intrinsics, ...
          imagePoints, colorImage, depthRange, roi] = ...
          validateAndParseInputsCodegen(depthImage, depthScaleFactor, intrinsics, varargin)

narginchk(3,11);

% Validate positional args
checkDepthImage(depthImage);
checkScaleFactor(depthScaleFactor);
checkIntrinsics(intrinsics);

depthImageSize = size(depthImage);

defaultsVal   = getDefaultParametersVal(depthImageSize);
defaultsNoVal = getDefaultParametersNoVal();
properties    = getEmlParserProperties();

% Parse the PV pairs
optarg = coder.internal.parseParameterInputs(defaultsNoVal, ...
                                             properties, varargin{:});

% Assign and validate PV values
imagePoints1 = coder.internal.getParameterValue( ...
    optarg.ImagePoints, defaultsVal.ImagePoints, varargin{:});
checkImagePoints(imagePoints1);
if ~isnumeric(imagePoints1)
    imagePoints  = imagePoints1.Location;
else
    imagePoints  = imagePoints1;
end

colorImage = coder.internal.getParameterValue( ...
    optarg.ColorImage, defaultsVal.ColorImage, varargin{:});
checkColorImage(colorImage);

depthRange = coder.internal.getParameterValue( ...
    optarg.DepthRange, defaultsVal.DepthRange, varargin{:});
checkDepthRange(depthRange);

roi  = coder.internal.getParameterValue(optarg.ROI, ...
                                        defaultsVal.ROI, varargin{:});
vision.internal.detector.checkROI(roi,depthImageSize);

if ~isempty(colorImage)
    % Validate the resolution of the input
    % The size of two input images must be the same
    coder.internal.errorIf(any(depthImageSize ~= size(colorImage,[1 2])), ...
                           'vision:pointcloud:mismatchDepthToColor');
end

%==========================================================================
function defaultsVal = getDefaultParametersVal(imageSize)

defaultsVal = struct(...
    'ImagePoints', [], ...
    'ColorImage', [], ...
    'DepthRange', [0 inf],...
    'ROI', [1 1 imageSize(2) imageSize(1)]);

%==========================================================================
function defaultsNoVal = getDefaultParametersNoVal()

defaultsNoVal = struct(...
    'ImagePoints', uint32(0), ...
    'ColorImage',  uint32(0), ...
    'DepthRange',  uint32(0), ...
    'ROI',  uint32(0));

%==========================================================================
function properties = getEmlParserProperties()

properties = struct( ...
    'CaseSensitivity', false, ...
    'StructExpand',    true, ...
    'PartialMatching', false);

%==========================================================================
function checkDepthImage(val)
vision.internal.inputValidation.validateImage(val, 'depthImage', 'grayscale');

%==========================================================================
function checkScaleFactor(val)
validateattributes(val, {'numeric'}, {'scalar', ...
                                      'finite', 'nonsparse', 'real', 'positive'}, 'pcfromdepth', 'depthScaleFactor');

%==========================================================================
function checkIntrinsics(val)
if isa(val, 'cameraIntrinsics')
    validateattributes(val, {'cameraIntrinsics'}, ...
                       {'scalar'}, 'pcfromdepth', 'intrinsics');
else
    validateattributes(val, {'struct'}, ...
                       {'scalar'}, 'pcfromdepth', 'intrinsics');
    coder.internal.errorIf(~isfield(val,'FocalLength'), ...
                           'vision:pointcloud:missingFocalLength');
    coder.internal.errorIf(~isfield(val,'PrincipalPoint'), ...
                           'vision:pointcloud:missingPrincipalPoint');
end

%==========================================================================
function checkImagePoints(val)
if isnumeric(val)
    if ~isempty(val)
        validateattributes(val, {'numeric'}, {'real','nonnegative','nonsparse','finite',...
                                          'size',[NaN,2]}, 'pcfromdepth', 'imagePoints');
    end
else
    validateattributes(val, {'vision.internal.FeaturePoints'}, {}, 'pcfromdepth', 'imagePoints');
end

%==========================================================================
function checkColorImage(val)
if ~isempty(val)
    vision.internal.inputValidation.validateImage(val, 'colorImage', 'rgb');
end

%==========================================================================
function checkDepthRange(val)
validateattributes(val, {'numeric'}, {'real','nonnegative','nonsparse',...
                                      'nonempty','size',[1,2], 'increasing'}, 'pcfromdepth', 'depthRange');

%==========================================================================
function tf = isSimMode()
tf = isempty(coder.target);
