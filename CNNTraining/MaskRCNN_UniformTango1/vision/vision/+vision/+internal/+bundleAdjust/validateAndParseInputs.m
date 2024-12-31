function [xyzPoints, measurements, visibility, intrinsics, ...
    maxIterations, absTol, relTol, isUndistorted, ...
    verbose, returnType, cameraPoses, fixedCameraIndex, solver, varargout] = ...
    validateAndParseInputs(optimType, funcName, varargin)
%validateAndParseInputs Parameter validation and parsing for bundle
%   adjustment functions.

%#codegen
% Copyright 2019-2023 The MathWorks, Inc.

% Required inputs
if isempty(coder.target)
isClassSyntax = isa(varargin{1}, 'worldpointset');

isFull      = strncmpi(optimType, 'full', 1);
isMotion    = strncmpi(optimType, 'motion', 1);
isStructure = strncmpi(optimType, 'structure', 1);

varargout = cell(1,3);
if isClassSyntax
    % Only full and structure-only modes support this syntax
    wpSet          = varargin{1};
    vSet           = varargin{2};
    refinedViewIds = varargin{3};
    intrinsics     = varargin{4};
    
    % wpSet
    returnType = vision.internal.bundleAdjust.bundleAdjustmentValidation.validateMapPointSet(wpSet, funcName);

    % vSet
    vision.internal.bundleAdjust.bundleAdjustmentValidation.validateViewSet(vSet, funcName);

    % viewIds
    viewIds = vision.internal.bundleAdjust.bundleAdjustmentValidation.validateViewIds(refinedViewIds, funcName);
    vision.internal.bundleAdjust.bundleAdjustmentValidation.checkIfViewIdsMissing(viewIds, wpSet, vSet);

    % intrinsics
    validateCameraIntrinsics(intrinsics, viewIds, false, funcName);
        
    % Indices of map points observed by local key frames
    [~, featureIdxAll, mapPointIdx, visibility]  = findVisibilityOfView(wpSet, refinedViewIds);

    % Find image points across the optimized views
    measurements = zeros(2, nnz(visibility));

    refinedViewTable = findView(vSet, refinedViewIds);
    featurePointsAll =  refinedViewTable.Points;
    index = 1;
    for k = 1: numel(refinedViewIds)
        
        featureIdxOfView    = featureIdxAll{k};
        featurePointsOfView = featurePointsAll{k};
        numObservation      = numel(featureIdxOfView); 

        coder.internal.errorIf(size(featurePointsOfView, 1) < max(featureIdxOfView), ...
            'vision:sfm:notEnoughFeaturePoints', refinedViewIds(k), max(featureIdxOfView));
        
        if ~isnumeric(featurePointsOfView)
            featurePointsOfView = featurePointsOfView.Location; % [x, y] location
        end

        measurements(:, index: numObservation+index-1) = featurePointsOfView(featureIdxOfView,:)';
        index = numObservation+index;
    end

    xyzPoints    = double(wpSet.WorldPoints(mapPointIdx,:)');
    cameraPoses  = poses(vSet, refinedViewIds);
    measurements = double(measurements);

    varargout{1} = wpSet;
    varargout{2} = vSet;
    varargout{3} = mapPointIdx;
else
    [xyzPoints, returnType] = validateXYZPoints(varargin{1}, funcName);
    pointTracks = varargin{2};
    cameraPoses = varargin{3};
    intrinsics  = varargin{4};

    if isMotion
        % imagePoints
        pointTracks = validateImagePoints(pointTracks, xyzPoints, funcName);

        % absolutePose
        vision.internal.bundleAdjust.bundleAdjustmentValidation.validateSinglePose(cameraPoses, funcName);

        % intrinsics
        validateCameraIntrinsics(intrinsics, cameraPoses, true, funcName);

        fixedCameraIndex = 0;
    elseif isFull
        % pointTracks
        validatePointTracks(pointTracks, xyzPoints, funcName);

        % cameraPoses
        validatePoseTableFull(cameraPoses, funcName);

        % intrinsics
        validateCameraIntrinsics(intrinsics, cameraPoses, false, funcName);
    else % structure
        % pointTracks
        validatePointTracks(pointTracks, xyzPoints, funcName);

        % cameraPoses
        validatePoseTableStructure(cameraPoses, funcName);

        % intrinsics
        validateCameraIntrinsics(intrinsics, cameraPoses, false, funcName);
    end

    if isMotion
        measurements = double(pointTracks)';
        visibility   = sparse(ones(size(pointTracks, 1), 1));
    else
        [measurements, visibility] = convertToMeasurementsAndVisibility(cameraPoses, pointTracks);
    end
end

% Set input parser
defaults = struct(...
    'MaxIterations', 50,...
    'AbsoluteTolerance', 1,...
    'RelativeTolerance', 1e-5,...
    'PointsUndistorted', false, ...
    'FixedViewIDs', [], ...
    'Verbose', false, ...
    'Solver', 'sparse-linear-algebra');

parser = inputParser;
parser.CaseSensitive = false;
parser.FunctionName = funcName;

if isFull
    % Parameter specific to 'full' mode
    parser.addParameter('FixedViewIDs', defaults.FixedViewIDs, ...
        @(x)vision.internal.bundleAdjust.bundleAdjustmentValidation.validateFixedViewIDs(x, funcName));

    parser.addParameter('Solver', defaults.Solver, ...
        @(x)vision.internal.bundleAdjust.bundleAdjustmentValidation.validateSolver(x, funcName, 'Solver'));
end

% Common parameters
parser.addParameter('MaxIterations', defaults.MaxIterations, ...
    @(x)vision.internal.bundleAdjust.bundleAdjustmentValidation.validateMaxIterations(x, funcName));
parser.addParameter('AbsoluteTolerance', defaults.AbsoluteTolerance, ...
    @(x)vision.internal.bundleAdjust.bundleAdjustmentValidation.validateTolerance(x, funcName, 'AbsoluteTolerance'));
parser.addParameter('RelativeTolerance', defaults.RelativeTolerance, ...
    @(x)vision.internal.bundleAdjust.bundleAdjustmentValidation.validateTolerance(x, funcName, 'RelativeTolerance'));
parser.addParameter('PointsUndistorted', defaults.PointsUndistorted, ...
    @(x)vision.internal.inputValidation.validateLogical(x, 'PointsUndistorted'));
parser.addParameter('Verbose', defaults.Verbose, ...
    @(x)vision.internal.inputValidation.validateLogical(x, 'Verbose'));

parser.parse(varargin{5:end});

maxIterations = double(parser.Results.MaxIterations);
absTol        = double(parser.Results.AbsoluteTolerance);
relTol        = double(parser.Results.RelativeTolerance);
isUndistorted = parser.Results.PointsUndistorted;
verbose       = parser.Results.Verbose;

% Convert FixedViewIDs to indices in the camera poses table
if isFull
    fixedViewIDs  = uint32(parser.Results.FixedViewIDs);

    % Check the fixed view ID
    if ~isempty(fixedViewIDs)
        fixedCameraIndex = convertFixedViewIDsToIndices(cameraPoses.ViewId, fixedViewIDs);
    else
        fixedCameraIndex = 0;
    end

elseif isStructure
    % All cameras are fixed
    fixedCameraIndex = 1:height(cameraPoses);
end

% Determine solver type
if isFull
    solver  = lower(parser.Results.Solver);
else
    solver  = defaults.Solver;
end
else
    [xyzPoints, measurements, visibility, intrinsics, ...
    maxIterations, absTol, relTol, isUndistorted, ...
    verbose, returnType, cameraPoses, fixedCameraIndex, solver, varargout{:}] = vision.internal.codegen.bundleAdjust.validateAndParseInputs(optimType, funcName, varargin{:});
end
end

%--------------------------------------------------------------------------
function fixedCameraIndex = convertFixedViewIDsToIndices(viewIds, fixedViewIDs)
if ~isempty(fixedViewIDs)
    % Check if fixedViewIDs are valid
    missingViewIdx = ~ismember(fixedViewIDs, viewIds);
    if any(missingViewIdx)
        missingViewIds = fixedViewIDs(missingViewIdx);
        error(message('vision:viewSet:missingViewId', ...
            missingViewIds(1)));
    end
    [~, fixedCameraIndex] = intersect(viewIds, fixedViewIDs, 'stable');
else
    fixedCameraIndex = 0;
end
end

%--------------------------------------------------------------------------
function [measurements, visibility] = convertToMeasurementsAndVisibility(cameraPoses, pointTracks)
numViews  = height(cameraPoses);
numPoints = numel(pointTracks);

% visibility(i,j): true if point i is visible in view j
visibility = zeros(numPoints, numViews);
viewIds = cameraPoses.ViewId;
x = zeros(numPoints, numViews);
y = zeros(numPoints, numViews);
for m = 1:numPoints
    trackViewIds = pointTracks(m).ViewIds;
    imgPoints    = pointTracks(m).Points;
    for n = 1:length(trackViewIds)
        viewIndex = find(viewIds == trackViewIds(n), 1, 'first');
        if isempty(viewIndex)
            error(message('vision:absolutePoses:missingViewId', trackViewIds(n)));
        end
        visibility(m, viewIndex) = 1;
        x(m, viewIndex) = imgPoints(n, 1);
        y(m, viewIndex) = imgPoints(n, 2);
    end
end

isVisible = find(visibility);
x = x(isVisible);
y = y(isVisible);

visibility = sparse(visibility);

% Measurements stores 2-D points in 1st view first, then 2nd view, ...
measurements = double([x, y])';
end

%==========================================================================
% Validate Required Inputs
%==========================================================================
function [xyzPoints, returnType] = validateXYZPoints(xyzPoints, funcName)
validateattributes(xyzPoints, {'single', 'double'}, ...
    {'finite', 'nonempty', 'nonsparse', '2d', 'ncols', 3}, funcName, 'xyzPoints');
returnType = class(xyzPoints);
xyzPoints  = double(xyzPoints');
end
%--------------------------------------------------------------------------
function validatePointTracks(points, xyzPoints, funcName)

validateattributes(points, {'pointTrack'}, {'nonempty','vector'}, funcName, 'pointTracks');

% Check the size of input
if numel(points) ~= size(xyzPoints, 2)
    error(message('vision:sfm:unmatchedXYZTrack'));
end
end

%--------------------------------------------------------------------------
function points = validateImagePoints(points, xyzPoints, funcName)
points = vision.internal.inputValidation.checkAndConvertPoints(points, funcName, 'imagePoints');

% Check the size of input
if size(points, 1) ~= size(xyzPoints, 2)
    error(message('vision:sfm:unmatchedXYZImagePoints'));
end
end

%--------------------------------------------------------------------------
function validatePoseTableFull(poses, funcName)

% Validate the table
validateattributes(poses, {'table'},{'nonempty'}, funcName, 'cameraPoses');

% Check columns. In 'full' mode, the camera poses table can contain
% 'AbsolutePose' or 'Location' and 'Orientation'
if ismember('AbsolutePose', poses.Properties.VariableNames)
    vision.internal.inputValidation.validatePoseTableRigid3d(poses, funcName, 'cameraPoses');
else
    vision.internal.inputValidation.checkAbsolutePoses(poses, funcName, 'cameraPoses');
end
end

%--------------------------------------------------------------------------
function validatePoseTableStructure(poses, funcName)
vision.internal.inputValidation.validatePoseTableRigid3d(poses, funcName, 'cameraPoses');
end

%--------------------------------------------------------------------------
function validateCameraIntrinsics(intrinsics, cameraPosesOrViewIds, isScalar, funcName)
% cameraParameters is supported but not recommended or documented and it
% can only be a scalar due to restriction in its constructor.
vision.internal.inputValidation.checkIntrinsicsAndParameters( ...
    intrinsics, isScalar, funcName);

% Check the camera intrinsics array
% ViewIDs array does not match the size of intrinsics array
viewIDsNotMatch = ~istable(cameraPosesOrViewIds) && numel(intrinsics) ~= numel(cameraPosesOrViewIds);
% CameraPose table does not match the size of intrinsics array
poseTableNotMatch = istable(cameraPosesOrViewIds) && numel(intrinsics) ~= height(cameraPosesOrViewIds);
if ~isscalar(intrinsics) && (viewIDsNotMatch || poseTableNotMatch)
    error(message('vision:sfm:unmatchedParamsPoses'));
end
end