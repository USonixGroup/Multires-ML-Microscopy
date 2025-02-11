function [tform, rmse, peak] = pcregistercorr(moving, fixed, gridSize,...
                            gridStep, varargin)

% Copyright 2020-2023 The Mathworks, Inc.

%#codegen

% Validate inputs
[zLimits, windowTF] =  parseAndValidateInputs(moving, fixed, gridSize, gridStep, varargin{:});

if ~isGPUCodegen()
    movingValid = removeInvalidPoints(moving);
    fixedValid  = removeInvalidPoints(fixed);

    % Calculate occupancy grid for both point clouds
    occGridMoving = vision.internal.pc.pcOccupancyGrid(movingValid, gridSize,...
        gridStep, zLimits);
    occGridFixed = vision.internal.pc.pcOccupancyGrid(fixedValid, gridSize,...
        gridStep, zLimits);

    % Perform 2d registration of occupancy grids
    [tform2d, peak] = vision.internal.pc.pcregistercorr2d(occGridMoving,...
        occGridFixed, windowTF);

    tform = rigid2dTo3d(tform2d, gridStep);
    
    if nargout >= 2
        % rmse is based on point-to-point correspondence
        rmse = cast(...
            vision.internal.pc.rmse(movingValid, fixedValid, tform), ...
            'like', moving.Location);
    end
else
    % GPU implementation for phase correlation registration   
    movingLocation = moving.Location;
    fixedLocation = fixed.Location;

    validMovingPoints = vision.internal.codegen.gpu.removeInvalidPoints(movingLocation);
    validFixedPoints = vision.internal.codegen.gpu.removeInvalidPoints(fixedLocation);

    % Calculate occupancy grid for both point clouds
    occGridMoving = vision.internal.codegen.gpu.pcregistercorr.pcOccupancyGrid(validMovingPoints, gridSize,...
        gridStep, zLimits);
    occGridFixed = vision.internal.codegen.gpu.pcregistercorr.pcOccupancyGrid(validFixedPoints, gridSize,...
        gridStep, zLimits);

    % Perform 2d registration of occupancy grids
    [tform2d, peak] = vision.internal.pc.pcregistercorr2d(occGridMoving,...
        occGridFixed, windowTF);

    tform = vision.internal.codegen.gpu.pcregistercorr.rigid2dto3d(tform2d, gridStep);

    if nargout >= 2
        % GPU implementation to transform the moving point cloud points
        % based on the tform generated through GPU codegen.
        transMat = tform.Translation;
        rotMat = tform.Rotation;
        transformedPoints = vision.internal.codegen.gpu.pcregisterndt.transformPoints(...
            validMovingPoints, rotMat, transMat);
        rmse = cast(...
            vision.internal.codegen.gpu.pcregistercorr.computeRmse(validFixedPoints, transformedPoints), ...
            'like', moving.Location);
    end
end

peak = cast(peak, 'like', moving.Location);

end

%--------------------------------------------------------------------------
function [zLimits, windowTF] = parseAndValidateInputs(moving, fixed, gridSize,...
    gridStep, varargin)
coder.inline('always');
coder.internal.prefer_const( varargin{:} );

validateattributes(moving, {'pointCloud'}, {'scalar'}, mfilename, 'moving');
validateattributes(fixed, {'pointCloud'}, {'scalar'}, mfilename, 'fixed');
validateattributes(gridSize, {'single', 'double'}, ...
    {'real','scalar', 'nonnan', 'nonsparse','positive', 'finite'}, mfilename, 'gridSize');
validateattributes(gridStep, {'single', 'double'}, ...
    {'real','scalar', 'nonnan', 'nonsparse','positive', 'finite'}, mfilename, 'gridStep');

defaults = struct('ZLimit', [0 3], 'Window', true);

isSimMode = isempty(coder.target);

if isSimMode
    parser = inputParser;
    parser.CaseSensitive = false;

    parser.addParameter('ZLimit', defaults.ZLimit, @(x)validateZLimit(x, moving, fixed));
    parser.addParameter('Window', defaults.Window, @(x)validateWindow(x));
    parser.parse(varargin{:});

    zLimits = parser.Results.ZLimit;
    windowTF = parser.Results.Window;
else
    pvPairs = struct( 'ZLimit', uint32(0), 'Window', uint32(0));
    poptions = struct();
    pstruct = coder.internal.parseParameterInputs(pvPairs, ...
        poptions, varargin{:});

    zLimits = coder.internal.getParameterValue(pstruct.ZLimit,...
        defaults.ZLimit, varargin{:});
    windowTF = coder.internal.getParameterValue(pstruct.Window,...
        defaults.Window, varargin{:});
end

end

%--------------------------------------------------------------------------
function validateZLimit(inputValue, moving, fixed)

% ZLimit values should exist in both point clouds
minLimit = max(moving.ZLimits(1), fixed.ZLimits(1));
maxLimit = min(moving.ZLimits(2), fixed.ZLimits(2));

validateattributes(inputValue, {'single', 'double'},...
    {'real','nonsparse','numel', 2, '>=', minLimit, '<=', maxLimit,...
    'increasing'}, mfilename, 'ZLimit');

end

%--------------------------------------------------------------------------
function validateWindow(window)

validateattributes(window, {'logical', 'numeric'},...
    {'scalar','finite', 'real', 'nonsparse'},...
    mfilename, 'Window');

end

%--------------------------------------------------------------------------
function tform3d = rigid2dTo3d(tform2d, gridStep)

tform3d = rigidtform3d();

rotMatrix = tform3d.Rotation;
rotMatrix(1:2,1:2) = tform2d.T(1:2,1:2);
[U, ~, V] = svd(rotMatrix);
rotMatrix = U * V';

tform3d.Rotation = rotMatrix;

% Multiply with gridStep to convert to world units
tform3d.Translation(1:2) = tform2d.T(3,1:2) .* gridStep;

end

%--------------------------------------------------------------------------
% GPU Codegen flag
function flag = isGPUCodegen()
flag = coder.const(coder.gpu.internal.isGpuEnabled);
end

