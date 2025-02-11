function [orientation, location, inlierIdx, status, statusCode] = estWorldPoseImpl(imagePoints, worldPoints, cameraParams, isCameraParamsSupported, filename, varargin)
% estWorldPoseImpl Function to estimate the orientation and location of a
%   calibrated camera in the world coordinate system in which worldPoints
%   are defined.

% Copyright 2022 The MathWorks, Inc.

%#codegen

[params, outputClass, imagePts, worldPts] = parseInputs(imagePoints, worldPoints, ...
    cameraParams, isCameraParamsSupported, filename, varargin{:});

% List of status codes
statusCode = struct(...
    'NoError',           int32(0),...
    'NotEnoughPts',      int32(1),...
    'NotEnoughInliers',  int32(2));

% Additional RANSAC parameters
params.sampleSize = 4;
params.recomputeModelFromInliers = false;
params.defaultModel.R = nan(3);
params.defaultModel.t = nan(1, 3);

% RANSAC function handles
funcs.fitFunc = @solveCameraPose;
funcs.evalFunc = @evalCameraPose;
funcs.checkFunc = @check;

numPoints = size(worldPts, 1);
if numPoints < params.sampleSize
   status = statusCode.NotEnoughPts;
   [orientation, location, inlierIdx] = badPose(numPoints);
else
    % Compute the pose using RANSAC
    points = pack(imagePts, worldPts);
    [isFound, pose, inlierIdx] = vision.internal.ransac.msac(...
        points, params, funcs, cameraParams.K', outputClass);
    
    if isFound
        % Convert from extrinsics to orientation and location
        orientation =  pose.R';
        location    = -pose.t * pose.R';
        status = statusCode.NoError;
    else
        % Could not compute the pose
        status = statusCode.NotEnoughInliers;
        [orientation, location, inlierIdx] = badPose(numPoints);
    end
end

end

%--------------------------------------------------------------------------
function [orientation, location, inlierIdx] = badPose(numPoints)
orientation = nan(3);
location = nan(1, 3);
inlierIdx = false(numPoints, 1);
end

%--------------------------------------------------------------------------
function pose = solveCameraPose(points, varargin)
[worldPoints, imagePoints] = unpack(points);

intrinsicMatrix = varargin{1};

% Get up to 4 solutions for the pose using 3 points
[Rs, Ts] = vision.internal.calibration.solveP3P(...
    imagePoints, worldPoints, intrinsicMatrix);

% Choose the best solution using the 4th point
p = [worldPoints(4, :), 1]; % homogeneous coordinates
q = imagePoints(4, :);

pose.R = nan(3);
pose.t = nan(1, 3);
if ~isempty(Rs)
    pose = chooseBestSolution(p, q, Rs, Ts, intrinsicMatrix);
end
end

%--------------------------------------------------------------------------
% Find the solution that results in smallest squared reprojection error for
% the 4th point.
% worldPoint must be in homogeneous coordinates (1-by-4)
function pose = chooseBestSolution(worldPoint, imagePoint, Rs, Ts, intrinsicMatrix)

pose.R = zeros(3);
pose.t = zeros(1, 3);

numSolutions = size(Ts, 1);
errors = zeros(numSolutions, 1, 'like', worldPoint);

for i = 1:numSolutions    
    cameraMatrix = [Rs(:,:,i); Ts(i,:)] * intrinsicMatrix;
    projectedPoint = worldPoint * cameraMatrix;
    projectedPoint = projectedPoint(1:2) ./ projectedPoint(3);
    d = imagePoint - projectedPoint;
    errors(i) = d * d';
end

[~, idx] = min(errors);
idx = idx(1); % in case we have two identical errors
pose.t = Ts(idx, :);
pose.R = Rs(:,:,idx);
end
    
%--------------------------------------------------------------------------
% Compute reprojection errors
function dis = evalCameraPose(pose, points, varargin)
[worldPoints, imagePoints] = unpack(points);

intrinsicMatrix = varargin{1};
cameraMatrix = [pose.R; pose.t] * intrinsicMatrix;

% Project world points into the image
numPoints = size(worldPoints, 1);
worldPointsHomog = [worldPoints, ones(numPoints, 1, 'like', worldPoints)];
projectedPointsHomog = worldPointsHomog * cameraMatrix;
projectedPoints  = bsxfun(@rdivide, projectedPointsHomog(:, 1:2), ...
    projectedPointsHomog(:, 3));

% Compute reprojection errors
diffs = imagePoints - projectedPoints;
dis = sum(diffs.^2, 2);
end

%--------------------------------------------------------------------------
% Pack points into a single entity for RANSAC
function points = pack(imagePoints, worldPoints)
points = [imagePoints, worldPoints];
end

%--------------------------------------------------------------------------
% Unpack the points
function [worldPoints, imagePoints] = unpack(points)
imagePoints = points(:, 1:2);
worldPoints = points(:, 3:end);
end

%--------------------------------------------------------------------------
function r = check(pose, varargin)
r = ~isempty(pose) && ~isempty(pose.R) && ~isempty(pose.t);
end

%--------------------------------------------------------------------------
function [ransacParams, outputClass, imagePts, worldPts] = ...
    parseInputs(imagePoints, worldPoints, cameraParams, isCameraParamsSupported, filename, varargin)

validatePoints(imagePoints, worldPoints, filename);
imagePts = double(imagePoints);
worldPts = double(worldPoints);
outputClass = class(imagePts);

if isCameraParamsSupported
    validateattributes(cameraParams, {'cameraParameters','cameraIntrinsics'}, ...
        {'scalar'}, filename, 'cameraParams');
else
    validateattributes(cameraParams, {'cameraIntrinsics'}, {'scalar'}, ...
        filename, 'cameraParams');
end

defaults = struct('MaxNumTrials',1000, 'Confidence',99, 'MaxDistance', 1);

if isempty(coder.target)
    ransacParams = parseRANSACParamsMatlab(defaults, ...
        isCameraParamsSupported, filename, varargin{:});
else
    ransacParams = parseRANSACParamsCodegen(defaults, ...
        isCameraParamsSupported, varargin{:});
end
end

%--------------------------------------------------------------------------
function ransacParams = parseRANSACParamsMatlab(defaults, isCameraParamsSupported, filename, varargin)
parser = inputParser;
parser.FunctionName = filename;
if isCameraParamsSupported
    parser.addParameter('MaxNumTrials', defaults.MaxNumTrials, @checkMaxNumTrialsWCP);
    parser.addParameter('Confidence', defaults.Confidence, @checkConfidenceWCP);
    parser.addParameter('MaxReprojectionError', defaults.MaxDistance, @checkMaxDistanceWCP);
else
    parser.addParameter('MaxNumTrials', defaults.MaxNumTrials, @checkMaxNumTrials);
    parser.addParameter('Confidence', defaults.Confidence, @checkConfidence);
    parser.addParameter('MaxReprojectionError', defaults.MaxDistance, @checkMaxDistance);
end

parser.parse(varargin{:});
ransacParams.confidence = parser.Results.Confidence;
ransacParams.maxDistance = parser.Results.MaxReprojectionError^2;
ransacParams.maxNumTrials = parser.Results.MaxNumTrials;
ransacParams.verbose = false;
end

%--------------------------------------------------------------------------
function ransacParams = parseRANSACParamsCodegen(defaults, isCameraParamsSupported, varargin)
% Instantiate an input parser
parms = struct( ...
    'MaxNumTrials',       uint32(0), ...
    'Confidence',         uint32(0), ...
    'MaxReprojectionError',        uint32(0));

popt = struct( ...
    'CaseSensitivity', false, ...
    'StructExpand',    true, ...
    'PartialMatching', false);

% Specify the optional parameters
optarg = eml_parse_parameter_inputs(parms, popt, varargin{:});
ransacParams.maxNumTrials = eml_get_parameter_value(optarg.MaxNumTrials,...
    defaults.MaxNumTrials, varargin{:});
ransacParams.confidence   = eml_get_parameter_value(optarg.Confidence,...
    defaults.Confidence, varargin{:});
ransacParams.maxDistance  = eml_get_parameter_value(...
    optarg.MaxReprojectionError, defaults.MaxDistance, varargin{:});
ransacParams.verbose  = false;

if isCameraParamsSupported
    checkMaxNumTrialsWCP(ransacParams.maxNumTrials);
    checkConfidenceWCP(ransacParams.confidence);
    checkMaxDistanceWCP(ransacParams.maxDistance);
else
    checkMaxNumTrials(ransacParams.maxNumTrials);
    checkConfidence(ransacParams.confidence);
    checkMaxDistance(ransacParams.maxDistance);
end

ransacParams.maxDistance = ransacParams.maxDistance^2;
end

%--------------------------------------------------------------------------
function validatePoints(imagePoints, worldPoints, filename)
validateattributes(imagePoints, {'double', 'single'}, ...
    {'real', 'nonsparse', 'nonempty', '2d', 'ncols', 2}, ...
    filename, 'imagePoints');

validateattributes(worldPoints, {'double', 'single'}, ...
    {'real', 'nonsparse', 'nonempty', '2d', 'ncols', 3}, ...
    filename, 'worldPoints');

coder.internal.errorIf(~isa(imagePoints, class(worldPoints)), ...
    'vision:points:ptsClassMismatch', 'imagePoints', 'worldPoints');
coder.internal.errorIf(size(imagePoints, 1) ~= size(worldPoints, 1), ...
    'vision:points:numPtsMismatch', 'imagePoints', 'worldPoints');
end

%--------------------------------------------------------------------------
% Two sets of validation functions to call original function name on
% invalid value. Validation functions must only have one input, so we can
% not pass in the original filename.
%--------------------------------------------------------------------------
function tf = checkMaxNumTrialsWCP(value)
validateattributes(value, {'numeric'}, ...
    {'scalar', 'nonsparse', 'real', 'integer', 'positive'}, ...
    'estimateWorldCameraPose', 'MaxNumTrials');
tf = true;
end

%--------------------------------------------------------------------------
function tf = checkConfidenceWCP(value)
validateattributes(value, {'numeric'}, ...
    {'scalar', 'nonsparse', 'real', 'positive', '<', 100}, ...
    'estimateWorldCameraPose', 'Confidence');
tf = true;
end

%--------------------------------------------------------------------------
function tf = checkMaxDistanceWCP(value)
validateattributes(value,{'single','double'}, ...
    {'real', 'nonsparse', 'scalar','nonnegative','finite'}, ...
    'estimateWorldCameraPose', 'MaxDistance');
tf = true;
end

%--------------------------------------------------------------------------
function tf = checkMaxNumTrials(value)
validateattributes(value, {'numeric'}, ...
    {'scalar', 'nonsparse', 'real', 'integer', 'positive'}, ...
    'estworldpose', 'MaxNumTrials');
tf = true;
end

%--------------------------------------------------------------------------
function tf = checkConfidence(value)
validateattributes(value, {'numeric'}, ...
    {'scalar', 'nonsparse', 'real', 'positive', '<', 100}, ...
    'estworldpose', 'Confidence');
tf = true;
end

%--------------------------------------------------------------------------
function tf = checkMaxDistance(value)
validateattributes(value,{'single','double'}, ...
    {'real', 'nonsparse', 'scalar','nonnegative','finite'}, ...
    'estworldpose', 'MaxDistance');
tf = true;
end