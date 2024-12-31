function worldPoints = patternWorldPointsImpl(patternName, patternDims, pointsDistance, varargin)

% Copyright 2024 The MathWorks, Inc.

%#codegen


    patternDims = double(patternDims);
    pointsDistance = double(pointsDistance);

    if nargin > 3
        tagSpacing = double(varargin{1});
    else
        tagSpacing = [];
    end
    
    switch patternName
         case "checkerboard"
            % Number of keypoints in each dimensions.
            gridSize = patternDims-1;

            % First dimension must always be the shortest dimension which
            % corresponds to the y-axis of the pattern coordinate system.
            % detectCheckerboardPoints always returns the smallest
            % dimension as the first dimension.
            worldPoints = generateRegularGridPoints(gridSize, pointsDistance);
        case "circle-grid-asymmetric"
            
            % Error if the board is not 180-degrees rotation invariant.
            vision.internal.errorIf(all(mod(patternDims, 2) == 0),...
                'vision:calibrate:unsupportedPatternDims');

            worldPoints = generateAsymmetricCircleGridPoints(patternDims, pointsDistance);
        case "circle-grid-symmetric"

            worldPoints = generateRegularGridPoints(patternDims, pointsDistance);
        case "charuco-board"
            % Number of keypoints in each dimensions.
            gridSize = patternDims-1;
            
            % First dimension must always be the shortest dimension which
            % corresponds to the y-axis of the pattern coordinate system.
            % Swap the dimensions if the first dimension is not the smallest
            % dimension.
            if gridSize(1) > gridSize(2)
                gridSize = gridSize([2,1]);
            end
            worldPoints = generateRegularGridPoints(gridSize, pointsDistance);
        case "aprilgrid"

            % First dimension must always be the shortest dimension which
            % corresponds to the y-axis of the pattern coordinate system.
            % Swap the dimensions if the first dimension is not the smallest
            % dimension.
            if patternDims(1) > patternDims(2)
                patternDims = patternDims([2,1]);
            end

            worldPoints = generateAprilGridPoints(patternDims, pointsDistance, tagSpacing);
    end
end


%-------------------------------------------------------------------------------
function worldPoints = generateRegularGridPoints(gridSize, pointsDistance)
    
    rows = gridSize(1);
    cols = gridSize(2);
    
    % Create a grid of column and row indices.
    [colGrid, rowGrid] = meshgrid(0:cols-1, 0:rows-1);
    
    % Calculate world points based on the keypoint distances.
    worldPoints = [colGrid(:), rowGrid(:)] * pointsDistance;
end

%-------------------------------------------------------------------------------
function worldPoints = generateAsymmetricCircleGridPoints(dims, centerDistance)
    
    % Calculate column offset for each column.
    halfCenterDistance = double(centerDistance)/2;
    colOffset = zeros(dims(2),1);
    colOffset(2:2:end) = halfCenterDistance;
    
    worldPoints = zeros(dims(1) * dims(2), 2);
    idx = 1;
    for colNo = 1:dims(2)
        for rowNo = 1:dims(1)
            worldPoints(idx,1) = (colNo - 1) * halfCenterDistance;
            worldPoints(idx,2) = (rowNo - 1) * centerDistance + colOffset(colNo);
            idx = idx + 1;
        end
    end
end

%-------------------------------------------------------------------------------
% AprilGrid is made of AprilTags and each AprilTag contributes 4 keypoints
% to the calibration pattern. Thus, for a dim1-by-dim2 grid, there are
% dim1*dim2*4 keypoints. The distance between consecutive keypoints
% alternates between tagSize and tagSpacing.
%-------------------------------------------------------------------------------
function worldPoints = generateAprilGridPoints(dims, tagSize, tagSpacing)
    
    % Initalize world points.
    numPoints = dims(1)*dims(2)*4;
    worldPoints = zeros(numPoints, 2);

    k = 1;
    yTagCounter = 0;
    ySpaceCounter = 0;
    for j = 0:dims(2)*2-1
        xTagCounter = 0;
        xSpaceCounter = 0;
        for i = 0:dims(1)*2-1
            xTagCounter = xTagCounter + mod(i,2);
            xSpaceCounter = xSpaceCounter + mod(i+1,2);
            worldPoints(k,2) = yTagCounter * tagSize + ySpaceCounter * tagSpacing;
            worldPoints(k,1) = xTagCounter * tagSize + xSpaceCounter * tagSpacing;
            k = k + 1;
        end
        yTagCounter = yTagCounter + mod(j+1,2);
        ySpaceCounter = ySpaceCounter + mod(j,2);
    end

    % The above computation returns the keypoint locations starting at
    % (tagSpacing, 0). Normalize the keypoints to start from (0,0).
    worldPoints = worldPoints - [tagSpacing, 0];
end