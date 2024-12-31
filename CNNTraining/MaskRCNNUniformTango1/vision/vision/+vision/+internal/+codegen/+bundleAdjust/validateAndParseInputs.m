%#codegen

%   Copyright 2022-2023 The MathWorks, Inc.
function [xyzPoints, measurements, visibility, intrinsics, ...
    maxIterations, absTol, relTol, isUndistorted, ...
    verbose, returnType, poseStruct, fixedCameraIndex, solver, varargout] = validateAndParseInputs(optimType, funcName, varargin)

    % Required inputs
    isClassSyntax = isa(varargin{1}, 'worldpointset');
    
    isFull      = strncmpi(optimType, 'full', 1);
    isMotion    = strncmpi(optimType, 'motion', 1);
    isStructure = strncmpi(optimType, 'structure', 1);

     if isClassSyntax
         % Only full and structure-only modes support this syntax
        wpSet          = varargin{1};
        vSet           = varargin{2};
        refinedViewIds = varargin{3};
        intrinsics     = varargin{4};

        intrinsicsObj = convertIntrinsicsStruct(intrinsics);
        
        % wpSet
        returnType = vision.internal.bundleAdjust.bundleAdjustmentValidation.validateMapPointSet(wpSet, funcName);
    
        % vSet
        vision.internal.bundleAdjust.bundleAdjustmentValidation.validateViewSet(vSet, funcName);
    
        % viewIds
        viewIds = vision.internal.bundleAdjust.bundleAdjustmentValidation.validateViewIds(refinedViewIds, funcName);
        vision.internal.bundleAdjust.bundleAdjustmentValidation.checkIfViewIdsMissing(viewIds, wpSet, vSet);
    
        % intrinsics
        validateCameraIntrinsics(intrinsicsObj, viewIds, false, funcName);
        
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
            
            coder.internal.errorIf(size(featurePointsOfView.Location, 1) < max(featureIdxOfView), ...
                'vision:sfm:notEnoughFeaturePoints', refinedViewIds(k), max(featureIdxOfView));
            
            if ~isnumeric(featurePointsOfView)
                featurePointsOfViewLoc = featurePointsOfView.Location; % [x, y] location
            else
                featurePointsOfViewLoc = featurePointsOfView;
            end
            measurements(:, index: numObservation+index-1) = featurePointsOfViewLoc(featureIdxOfView,:)';
            index = numObservation+index;
        end
        
        xyzPoints    = double(wpSet.WorldPoints(mapPointIdx,:)');
        cameraPoses  = poses(vSet, refinedViewIds);
        poseStruct = cameraPoses;
        measurements = double(measurements);
    
        varargout{1} = wpSet;
        varargout{2} = vSet;
        varargout{3} = mapPointIdx;
        
    else
        varargout = {[],[],[]};
        [xyzPoints, returnType] = validateXYZPoints(varargin{1}, funcName);
        pointTracks = varargin{2};
        cameraPoses = varargin{3};
        intrinsics  = varargin{4};
        intrinsicsObj = convertIntrinsicsStruct(intrinsics);

        if isMotion
            % imagePoints
            pointTracks = validateImagePoints(pointTracks, xyzPoints, funcName);
    
            % absolutePose
            vision.internal.bundleAdjust.bundleAdjustmentValidation.validateSinglePose(cameraPoses, funcName);
            poseStruct = cameraPoses;
            % intrinsics
            validateCameraIntrinsics(intrinsicsObj, cameraPoses, true, funcName);
    
            fixedCameraIndex = 0;
        elseif isFull
            % pointTracks
            validatePointTracks(pointTracks, xyzPoints, funcName);
            
            % cameraPoses
            poseStruct = validatePoseTableFull(cameraPoses, funcName);
    
            % intrinsics
            validateCameraIntrinsics(intrinsicsObj, cameraPoses, false, funcName);
            
        else % structure
            % pointTracks
            validatePointTracks(pointTracks, xyzPoints, funcName);
    
            % cameraPoses
            poseStruct = validatePoseTableStructure(cameraPoses, funcName);
    
            % intrinsics
            validateCameraIntrinsics(intrinsicsObj, cameraPoses, false, funcName);
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
    
    nvPairNames = {'MaxIterations', 'AbsoluteTolerance', 'RelativeTolerance', ...
        'PointsUndistorted', 'FixedViewIDs', 'Verbose', 'Solver'};

    poptions = struct('CaseSensitivity', false);
    pstruct = coder.internal.parseInputs({}, nvPairNames, poptions, varargin{5:end});

    if isFull
        fixedViewIds  = coder.internal.getParameterValue(pstruct.FixedViewIDs, ...
            defaults.FixedViewIDs, varargin{5:end});
        solver = coder.internal.getParameterValue(pstruct.Solver, ...
            defaults.Solver, varargin{5:end});
        vision.internal.bundleAdjust.bundleAdjustmentValidation.validateFixedViewIDs(fixedViewIds, funcName);
        vision.internal.bundleAdjust.bundleAdjustmentValidation.validateSolver(solver, funcName, 'Solver');
    end
    maxIterations = coder.internal.getParameterValue(pstruct.MaxIterations, ...
            defaults.MaxIterations, varargin{5:end});
    absTol = coder.internal.getParameterValue(pstruct.AbsoluteTolerance, ...
            defaults.AbsoluteTolerance, varargin{5:end});
    relTol = coder.internal.getParameterValue(pstruct.RelativeTolerance, ...
            defaults.RelativeTolerance, varargin{5:end});
    isUndistorted = coder.internal.getParameterValue(pstruct.PointsUndistorted, ...
            defaults.PointsUndistorted, varargin{5:end});
    verbose = coder.internal.getParameterValue(pstruct.Verbose, ...
            defaults.Verbose, varargin{5:end});

    vision.internal.bundleAdjust.bundleAdjustmentValidation.validateMaxIterations(maxIterations, funcName);
    vision.internal.bundleAdjust.bundleAdjustmentValidation.validateTolerance(absTol, funcName, 'AbsoluteTolerance');
    vision.internal.bundleAdjust.bundleAdjustmentValidation.validateTolerance(relTol, funcName, 'RelativeTolerance');
    vision.internal.inputValidation.validateLogical(isUndistorted, 'PointsUndistorted');
    vision.internal.inputValidation.validateLogical(verbose, 'Verbose');

    % Convert FixedViewIDs to indices in the camera poses table
    if isFull
        fixedViewIDs  = uint32(fixedViewIds);
    
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
        solver  = lower(solver);
    else
        solver  = defaults.Solver;
    end

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
coder.internal.errorIf((~isscalar(intrinsics) && (viewIDsNotMatch || poseTableNotMatch)), ...
    'vision:sfm:unmatchedParamsPoses');
end

%--------------------------------------------------------------------------
function [xyzPoints, returnType] = validateXYZPoints(xyzPoints, funcName)
validateattributes(xyzPoints, {'single', 'double'}, ...
    {'finite', 'nonempty', 'nonsparse', '2d', 'ncols', 3, 'real'}, funcName, 'xyzPoints');
returnType = class(xyzPoints);
xyzPoints  = double(xyzPoints');
end

%--------------------------------------------------------------------------
function validatePointTracks(points, xyzPoints, funcName)

validateattributes(points, {'pointTrack'}, {'nonempty','vector'}, funcName, 'pointTracks');
% Check the size of input
coder.internal.errorIf((numel(points) ~= size(xyzPoints, 2)), 'vision:sfm:unmatchedXYZTrack');
end

%--------------------------------------------------------------------------
function poseStruct  = validatePoseTableFull(poses, funcName)

% Validate the table
validateattributes(poses, {'table'},{'nonempty'}, funcName, 'cameraPoses');

% Check columns. In 'full' mode, the camera poses table can contain
% 'AbsolutePose' or 'Location' and 'Orientation'
if any(strcmp(poses.Properties.VariableNames, 'AbsolutePose'))  
    poseStruct = checkAndConvertPoseTableRigid3d(poses, funcName, 'cameraPoses');
else
    poseStruct = checkAndConvertAbsolutePoses(poses, funcName, 'cameraPoses');
end
end

function fixedCameraIndex = convertFixedViewIDsToIndices(viewIds, fixedViewIDs)
if ~isempty(fixedViewIDs)
    % Check if fixedViewIDs are valid
    missingViewIdx = ~ismember(fixedViewIDs, viewIds);
    if any(missingViewIdx)
        missingViewIds = fixedViewIDs(missingViewIdx);
        coder.internal.error('vision:viewSet:missingViewId', missingViewIds(1));
    end
    [~, fixedCameraIndex] = intersect(viewIds, fixedViewIDs, 'stable');
else
    fixedCameraIndex = 0;
end
end

function poseStruct = checkAndConvertAbsolutePoses(poses, funcName, argName)

    minRows = 2;
    reqVars = {'ViewId', 'Orientation', 'Location'};
    
    validateattributes(poses, {'table'}, {'nonempty'}, funcName, argName);
    
    coder.internal.errorIf((height(poses)<minRows), 'vision:table:tooFewRows', argName, minRows);
    coder.internal.errorIf(~checkMember(reqVars, poses.Properties.VariableNames), ...
        'vision:table:missingRequiredColumns', argName, strjoin(reqVars, ', '));
    coder.internal.errorIf(~checkMember(poses.Properties.VariableNames, reqVars), 'vision:table:unrecognizedColumns', ...
                    argName, strjoin(reqVars, ', '));

    viewId = poses.ViewId;
    orientation = poses.Orientation;
    location = poses.Location;
    
    checkViewId(viewId, height(poses), funcName);
    checkOrientation(orientation, height(poses), funcName);
    checkLocation(location, height(poses), funcName);

    poseStruct.ViewId = viewId;
    poseStruct.Orientation = orientation;
    poseStruct.Location = location;
    poseStruct.VariableNames = poses.Properties.VariableNames;


end
%--------------------------------------------------------------------------
function poseStruct = checkAndConvertPoseTableRigid3d(poses, funcName, argName)
    
    validateattributes(poses, {'table'},{'size',[NaN 2],'nonempty'}, funcName, argName);
    coder.internal.errorIf(~checkMember({'ViewId', 'AbsolutePose'}, poses.Properties.VariableNames), ...
    'vision:table:missingRequiredColumns', 'cameraPoses', ...
        ['ViewId', ', ', 'AbsolutePose']);

    viewIds = poses.ViewId;

    validateattributes(viewIds, {'uint32'}, {'nonsparse', 'vector', ...
    'integer', 'positive', 'real'}, funcName, 'ViewIds');
    
    absolutePose = poses(1,:).AbsolutePose{1};
    if iscell(poses.AbsolutePose)
        for i = 2:size(poses,1)
            absPose = poses(i,:).AbsolutePose{1};
            validateattributes(absPose, {'rigidtform3d','rigid3d'}, {'scalar'}, funcName, 'AbsolutePose');
            absolutePose(i) = absPose;
        end
    end
    
    poseStruct.ViewId = viewIds;
    poseStruct.AbsolutePose = absolutePose;
    poseStruct.VariableNames = poses.Properties.VariableNames;
    
end

%--------------------------------------------------------------------------
function [measurements, visibilitySp] = convertToMeasurementsAndVisibility(cameraPoses, pointTracks)
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
        indices = (viewIds == trackViewIds(n));
        viewIndices = find(indices);
        coder.internal.errorIf(isempty(viewIndices), 'vision:absolutePoses:missingViewId', trackViewIds(n));
        if ~isscalar(viewIndices)
            viewIndex = viewIndices(1);
        else
            viewIndex = viewIndices;
        end
        visibility(m, viewIndex) = 1;
        x(m, viewIndex) = imgPoints(n, 1);
        y(m, viewIndex) = imgPoints(n, 2);
    end
end

isVisible = find(visibility);
x = x(isVisible);
y = y(isVisible);

visibilitySp = sparse(visibility);

% Measurements stores 2-D points in 1st view first, then 2nd view, ...
measurements = double([x, y])';
end

%==========================================================================
% Validate Optional Parameters
%==========================================================================
function out = checkMember(input, required)

    count = 0;
    for i=1:size(input, 2)
        count = count + any(strcmp(required, input{i}));
    end
    out = (count == size(input, 2));

end

 %----------------------------------------------------------------------
 function checkViewId(viewId, numRows, funcName)
    for i = 1:numRows
        id = viewId(i);
        validateattributes(id, {'uint32'}, {'scalar'}, funcName, 'ViewId');
    end
end

%----------------------------------------------------------------------
function checkOrientation(R, numRows, funcName)
    for i = 1:numRows
        orientation = R{i};
        vision.internal.inputValidation.validateRotationMatrix(...
            orientation, funcName, 'Orientation');
    end
end

%----------------------------------------------------------------------
function checkLocation(T, numRows, funcName)
    for i = 1:numRows
        loc = T{i};
        vision.internal.inputValidation.validateTranslationVector(...
            loc, funcName, 'Location');
    end
end

%--------------------------------------------------------------------------
function points = validateImagePoints(points, xyzPoints, funcName)
points = vision.internal.inputValidation.checkAndConvertPoints(points, funcName, 'imagePoints');

% Check the size of input
coder.internal.errorIf((size(points, 1) ~= size(xyzPoints, 2)), ...
    'vision:sfm:unmatchedXYZImagePoints');
end

%--------------------------------------------------------------------------
function poseStruct = validatePoseTableStructure(poses, funcName)
poseStruct = checkAndConvertPoseTableRigid3d(poses, funcName, 'cameraPoses');
end

function intrinsicsObj = convertIntrinsicsStruct(intrinsics)

    if isstruct(intrinsics) && isfield(intrinsics, 'FocalLength') ...
            && isfield(intrinsics, 'ImageSize') && isfield(intrinsics, 'PrincipalPoint')
        numCameras = numel(intrinsics);
        intrinsicsObj = cameraIntrinsics(ones(1,2),ones(1,2),ones(1,2));
        intrinsicsObj(1) = cameraIntrinsics(intrinsics(1).FocalLength, ...
                intrinsics(1).PrincipalPoint, intrinsics(1).ImageSize);
        for i = 2:numCameras
            intrinsicsObj(i) = cameraIntrinsics(intrinsics(i).FocalLength, ...
                intrinsics(i).PrincipalPoint, intrinsics(i).ImageSize);
        end

    else
        intrinsicsObj = intrinsics;
    end
end
