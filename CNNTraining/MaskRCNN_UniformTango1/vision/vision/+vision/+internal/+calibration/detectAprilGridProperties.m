function [patternDims, tagFamily, minTagID, numBorderBits] = ...
    detectAprilGridProperties(I)
% vision.internal.calibration.detectAprilGridProperties returns AprilGrid
% pattern properties given a perfect image of an AprilGrid pattern. An
% AprilGrid image is perfect if the entire pattern is captured with the
% camera facing directly towards the pattern. This limitation or condition
% is coming from the pattern dimension detection step. This step assumes
% least perspective distortion of the pattern so as to approximate vertex
% angles of the pattern contour.

% Copyright 2024 The MathWorks, Inc.

    % Read all AprilTags in the given image.
    [ids, tagLocs, tagFamilies, numBorderBits] = iReadAprilTag(I);

    % Retrieve tags associated with the predominant detected family.
    [ids, tagLocs, tagFamily, numBorderBits] = iFilterTagsUsingDetectedFamily(ids, tagLocs, tagFamilies, numBorderBits);
    
    % Sort the detected tags in the ascending order of their IDs.
    [ids, tagLocs, minTagID] = iSortTags(ids, tagLocs);

    % Find board contour/perimeter.
    contour = iFindBoardContour(tagLocs);

    % Compute the pattern dimensions using heuristics.
    angularTolerance = 5; % in degrees
    patternDims = iComputePatternDims(contour, ids, angularTolerance);
end

%-----------------------------------------------------------------------------
function [detectedIds, detectedLocs, detectedFamily, detectedNumBorderBits] = iReadAprilTag(I)
    tagFamily =  string(vision.internal.supportedAprilGridFamilies);
    numBorderBits = [1,2];
    numTagFamilies = numel(tagFamily);
    numNumBorderBits = numel(numBorderBits);
    numAttempts = numTagFamilies*numNumBorderBits;

    detectedIds = cell(numAttempts,1);
    detectedLocs = cell(numAttempts,1);
    detectedFamily = cell(numAttempts,1);
    detectedNumBorderBits = cell(numAttempts,1);
    numValidAttempts = 0;
    
    for i = 1:numTagFamilies
        for j = 1:numNumBorderBits
            params.NumBorderBits = numBorderBits(j);
            [ids, locs] = vision.internal.calibration.readAprilTag(I, tagFamily(i), params);
            if ~isempty(ids)
                numValidAttempts = numValidAttempts + 1;
                detectedIds{numValidAttempts} = double(ids);
                detectedLocs{numValidAttempts} = double(locs);
                detectedFamily{numValidAttempts} = tagFamily(i);
                detectedNumBorderBits{numValidAttempts} = numBorderBits(j);
            end
        end
    end
end

%-----------------------------------------------------------------------------
% Utility to retrieve tags associated with the predominant detected family.
%-----------------------------------------------------------------------------
function [ids, locs, tagFamily, numBorderBits] = iFilterTagsUsingDetectedFamily(...
    detectedIds, detectedLocs, detectedFamily, detectedNumBorderBits)

    numDetectedTags = cellfun(@numel, detectedIds);
    vision.internal.errorIf(sum(numDetectedTags)==0, "vision:caltool:NoTagsDetected");
    
    mostFrequentNumTags = numDetectedTags == max(numDetectedTags);
    mostFrequentLocs = detectedLocs(mostFrequentNumTags);
    mostFrequentIds = detectedIds(mostFrequentNumTags);
    mostFrequentFamilies = detectedFamily(mostFrequentNumTags);
    mostFrequentNumBorderBits = detectedNumBorderBits(mostFrequentNumTags);
   
    if isscalar(mostFrequentFamilies)
        ids = mostFrequentIds{1};
        locs = mostFrequentLocs{1};
        tagFamily = mostFrequentFamilies{1};
        numBorderBits = mostFrequentNumBorderBits{1};
    elseif numel(mostFrequentFamilies)==2

        % If there are tags from two different tag families, the
        % calibration pattern is invalid.
        vision.internal.errorIf(mostFrequentFamilies{1} ~= mostFrequentFamilies{2}, ...
            "vision:caltool:TagFamilyNotDetected");

        % Choose numBorderBits = 2 by default.
        idx = cellfun(@(x) x == 2, mostFrequentNumBorderBits);

        ids = mostFrequentIds{idx};
        locs = mostFrequentLocs{idx};
        tagFamily = mostFrequentFamilies{idx};
        numBorderBits = mostFrequentNumBorderBits{idx};
    else
        error(message("vision:caltool:TagFamilyNotDetected"));
    end
end

%-----------------------------------------------------------------------------
% Utility to sort tag detections in the ascending order of their IDs.
%-----------------------------------------------------------------------------
function [ids, tagLocs, minTagID] = iSortTags(ids, tagLocs)

    [ids, idx] = sort(ids);
    tagLocs = tagLocs(:,:,idx);
    minTagID = ids(1);
end

%-----------------------------------------------------------------------------
% Utility to find the AprilGrid contour/perimeter.
%-----------------------------------------------------------------------------
function contour = iFindBoardContour(locs)

    % Use tag centers to find the contour.
    centers = mean(locs);
    centers = permute(centers,[3,2,1]);

    % Find the centers lying on the contour/perimeter.
    contour.VertexInds = convhull(centers, Simplify=true);
    contour.Vertices = centers(contour.VertexInds,:);

    % Compute the angle between origin and every vertex.
    originInd = 1;
    contour.OriginAngles = iComputeAngles(contour.Vertices, originInd);
end

%-----------------------------------------------------------------------------
% Utility to compute pattern dimensions of an AprilGrid.
%-----------------------------------------------------------------------------
function dims = iComputePatternDims(contour, ids, tol)
    
    bounds = @(x,angle) (x > angle-tol & x < angle+tol);
    
    % Find vertices on the contour that have 90 degrees angle with the origin.
    verticesWith90degrees = find(bounds(contour.OriginAngles, 90));

    % Find vertices on the contour that have 0 degrees vertex angle with the origin.
    verticesWith0degrees = find(bounds(contour.OriginAngles, 0) | bounds(contour.OriginAngles, 180));

    % There can be no 90 degrees vertex angles for partial boards.
    vision.internal.errorIf(~any(verticesWith90degrees), ...
        "vision:caltool:DimDetectionFailed");

    % Normalize tag ids to start from zero.
    minTagID = ids(1);
    ids = ids - minTagID;
    
    % Find the extreme vertex in the vertical direction (a.k.a y-direction).
    extremeCornerIndY = iFindFarthestVertex(contour.Vertices, verticesWith90degrees, 1);
    extremeTagIdY = ids(contour.VertexInds(extremeCornerIndY));

    % Find the extreme vertex in the horizontal direction (a.k.a x-direction).
    extremeCornerIndX = iFindFarthestVertex(contour.Vertices, verticesWith0degrees, 1);
    extremeTagIdX = ids(contour.VertexInds(extremeCornerIndX));

    % Tag IDs can increase by 1 along the row or along the column.
    if extremeTagIdX > extremeTagIdY
        [extremeTagIdY, extremeTagIdX] = deal(extremeTagIdX, extremeTagIdY);
        doSwapDims = true;
    end
    
    % dim1 is the dimension along which the tag IDs increase by 1.
    dim1 = extremeTagIdX + 1;

    % dim2 is the dimension along which the tag IDs increase by dim1.
    dim2 = (extremeTagIdY/dim1) + 1;

    if doSwapDims
        dims = [dim2, dim1];
    else
        dims = [dim1, dim2];
    end

    % Get number of detected tags for validation.
    numDetectedTags = numel(ids);
    vision.internal.errorIf(~isValidPatternDims(dims, numDetectedTags), ...
        "vision:caltool:DimDetectionFailed");
end

%-----------------------------------------------------------------------------------
% Helper to find the farthest vertex from a reference vertex amongst the given candidate points.
% vertices - M-by-2 list of all vertices.
% candidateInds - N-by-2 list of indices in corners to be considered for search.
% cornerInd - A scalar index of the farthest corner.
%-----------------------------------------------------------------------------------
function vertexInd = iFindFarthestVertex(vertices, candidateInds, refInd)
    refVertex = vertices(refInd,:);

    % Calculate the differences between each vertex and the origin.
    diffs = vertices(candidateInds,:) - refVertex;
    
    % Compute the Euclidean distances for all vertices.
    distances = sqrt(sum(diffs.^2, 2));
    
    % Find the maximum distance and the index of the farthest vertex.
    [~, idx] = max(distances);
    
    % Get the index of the farthest vertex.
    vertexInd = candidateInds(idx);
end

%-----------------------------------------------------------------------------
% Helper to compute angles of all vertices w.r.t a reference vertex
%-----------------------------------------------------------------------------
function angles = iComputeAngles(vertices, refInd)

    numCorners = size(vertices,1);
    angles = zeros(numCorners,1);
    
    refVertex = vertices(refInd,:);
    v1 = [vertices(refInd+1,:) - refVertex,0];
    for i = 1:numCorners
        v2 = [vertices(i,:) - refVertex,0];
        angles(i) = atan2d(norm(cross(v1, v2)), dot(v1, v2));
    end
end

%-----------------------------------------------------------------------------
% Helper to validate pattern dimensions given the number of tags detected.
%-----------------------------------------------------------------------------
function tf = isValidPatternDims(dims, numDetectedTags)

    numTags = floor(dims(1)*dims(2));

    % Number of tags computed using valid pattern dimensions must be the
    % same as the number of tags detected.
    tf = numTags == numDetectedTags;
end