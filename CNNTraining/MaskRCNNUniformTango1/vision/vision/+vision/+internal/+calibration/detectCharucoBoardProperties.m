function [patternDims, markerFamily, minMarkerID, originCheckerColor] = ...
    detectCharucoBoardProperties(I)
% vision.internal.calibration.detectCharucoBoardProperties returns ChArUco
% board properties given a perfect image of a ChArUco board. A ChArUco
% board image is perfect if the entire board is captured with the camera
% facing directly towards the board. This limitation or condition is coming
% from the pattern dimension & origin checker color detection steps. These
% steps assume least perspective distortion of the board so as to
% approximate vertex angles of the board contour.

% Copyright 2024 The MathWorks, Inc.

    % Read all ArUco markers in the given image.
    numBorderBits = 1;
    markerFamily = vision.internal.supportedCharucoBoardFamilies;
    [ids, markerLocs, detectedFamily] = readArucoMarker(I, markerFamily, NumBorderBits=numBorderBits);

    % Retrieve markers associated with the predominant detected family.
    [ids, markerLocs, markerFamily] = iFilterMarkersUsingDetectedFamily(ids, markerLocs, detectedFamily);

    % Sort the detected markers in the ascending order of their IDs.
    [ids, markerLocs, minMarkerID] = iSortMarkers(ids, markerLocs);

    % Find board contour/perimeter.
    contour = iFindBoardContour(markerLocs);

    % Set the tolerance for determining if an angle is 0 or 90 degrees. That
    % is, if an angle is within angularTolerance of 90 degrees, it is considered 
    % 90 degrees. This angularTolerance also limits the allowable perspective 
    % distortion for successful board property detection in the image. This value 
    % is tuned to permit minimal perspective distortion, as documented.
    angularTolerance = 7; % in degrees

    % Identify the origin checker color using vertex angles.
    originCheckerColor = iIdentifyOriginCheckerColor(contour.OriginAngles, angularTolerance);
    
    % Compute the pattern dimensions using heuristics.
    patternDims = iComputePatternDims(contour, ids, originCheckerColor, angularTolerance);

    % If no valid pattern dimensions were found, loosen the angular
    % tolerance and retry.
    numDetectedMarkers = numel(ids);
    if ~isValidPatternDims(patternDims, numDetectedMarkers)
        angularTolerance = 10; % in degrees
    
        % Identify the origin checker color using vertex angles.
        originCheckerColor = iIdentifyOriginCheckerColor(contour.OriginAngles, angularTolerance);
        
        % Compute the pattern dimensions using heuristics.
        patternDims = iComputePatternDims(contour, ids, originCheckerColor, angularTolerance);
    end

    vision.internal.errorIf(~isValidPatternDims(patternDims, numDetectedMarkers), ...
    "vision:caltool:DimDetectionFailed");
end

%-----------------------------------------------------------------------------
% Utility to retrieve markers associated with the predominant detected family.
%-----------------------------------------------------------------------------
function [ids, locs, markerFamily] = iFilterMarkersUsingDetectedFamily(ids, locs, detectedFamily)

    vision.internal.errorIf(isempty(ids), "vision:caltool:NoMarkersDetected");

    % Find the most commonly detected family.
    family = categorical(detectedFamily);
    mostFrequentFamily = mode(family);
    markerFamily = string(mostFrequentFamily);

    % Discard markers not associated with the most commonly detected family.
    invalidDetections = family ~= mostFrequentFamily;
    ids(invalidDetections) = [];
    locs(:,:,invalidDetections) = [];
end

%-----------------------------------------------------------------------------
% Utility to sort marker detections in the ascending order of their IDs.
%-----------------------------------------------------------------------------
function [ids, markerLocs, minMarkerID] = iSortMarkers(ids, markerLocs)

    [ids, idx] = sort(ids);
    markerLocs = markerLocs(:,:,idx);
    minMarkerID = ids(1);
end

%-----------------------------------------------------------------------------
% Utility to find the ChAruco board contour/perimeter.
%-----------------------------------------------------------------------------
function contour = iFindBoardContour(locs)

    % Use marker centers to find the contour.
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
% Utility to identify the origin checker color using vertex angles.
%-----------------------------------------------------------------------------
function originCheckerColor = iIdentifyOriginCheckerColor(angles, tol)

    originVertexAngle = angles(end-1);
    if originVertexAngle > 90+tol 
        % Latest board contains a black checker in the very first cell on
        % the top-left, for all boards.
        %
        % |--V-----X--|  V - origin marker center
        % |  |  |  |  |  X - marker center next to origin in the X direction
        % Y-----*-----*  Y - marker center next to origin in the Y direction
        % |  |  |  |  |  Angle(YVX) - originVertexAngle
        % |--*-----*--|
        % |  |  |  |  |
        % *-----*-----*

        originCheckerColor = "black";
    else 
        % Legacy board contains a white checker in the very first cell on
        % the top-left for even number of rows.
        %
        % V-----X-----*  V - origin marker center
        % |  |  |  |  |  X - marker center next to origin in the X direction
        % |--*-----*--|  Y - marker center next to origin in the Y direction
        % |  |  |  |  |  Angle(YVX) - originVertexAngle
        % Y-----*-----*
        % |  |  |  |  |
        % |--*-----*--|
        
        originCheckerColor = "white";
    end
end

%-----------------------------------------------------------------------------
% Utility to compute pattern dimensions of a ChArUco board using heuristics.
% The board contour can take the following shapes based on pattern dims and 
% origin checker color.
%
% --------------------------------------------------
% | ID | numRows | numCols | Color | Contour Shape  |
% |-------------------------------------------------|
% | xx |   odd   |    odd  | white | Does not exist |
% | xx |   odd   |   even  | white | Does not exist |
% | 01 |  even   |    odd  | white |  v--------     |
% |    |         |         |       |  |        |    | (Legacy board)
% |    |         |         |       |   \      /     |
% |    |         |         |       |     ----       |
% | 02 |  even   |   even  | white |  v------       |
% |    |         |         |       |  |       \     |
% |    |         |         |       |  |        |    | (Legacy board)
% |    |         |         |       |   \       |    |
% |    |         |         |       |     ------     |
% | 03 |   odd   |    odd  | black |    v-----      |
% |    |         |         |       |   /       \    |
% |    |         |         |       |  |         |   | (Legacy & Latest boards)
% |    |         |         |       |   \       /    |
% |    |         |         |       |     -----      |
% | 04 |   odd   |   even  | black |    v-----      |
% |    |         |         |       |   /      |     | (Legacy & Latest boards)
% |    |         |         |       |  |       |     |
% |    |         |         |       |   \      |     |
% |    |         |         |       |     -----      |
% | 05 |  even   |   odd   | black |    v----       |
% |    |         |         |       |   /     \      | (Latest board)
% |    |         |         |       |  |       |     |
% |    |         |         |       |   -------      |
% | 06 |  even   |  even   | black |    v------     |
% |    |         |         |       |   /       |    |
% |    |         |         |       |  |        |    | (Latest board)
% |    |         |         |       |  |       /     |
% |    |         |         |       |   ------       |
% --------------------------------------------------
%-----------------------------------------------------------------------------
function dims = iComputePatternDims(contour, ids, originCheckerColor, tol)

    % Normalize marker ids to start from zero.
    minMarkerID = ids(1);
    ids = ids - minMarkerID;

    % Get number of detected markers for validation.
    numDetectedMarkers = numel(ids);

    bounds = @(x,angle) (x > angle-tol & x < angle+tol);
    
    % Find vertices on the contour that have 90 degrees angle with the origin.
    verticesWith90degrees = find(bounds(contour.OriginAngles, 90));

    % Find vertices on the contour that have 0 degrees vertex angle with the origin.
    verticesWith0degrees = find(bounds(contour.OriginAngles, 0) | bounds(contour.OriginAngles, 180));

    % If there are any vertices in the contour with 90 degrees (that is
    % directly below the origin marker), then it cannot be Shape-05 or
    % Shape-06. This condition means the first and last row are not the same.
    if any(verticesWith90degrees)

        % Find the extreme vertex in the vertical direction (a.k.a y-direction).
        cornerIndY = iFindFarthestVertex(contour.Vertices, verticesWith90degrees, 1);
        markerIdY = ids(contour.VertexInds(cornerIndY));
        
        % Find the extreme vertex in the horizontal direction (a.k.a x-direction).
        cornerIndX = iFindFarthestVertex(contour.Vertices, verticesWith0degrees, 1);
        markerIdX = ids(contour.VertexInds(cornerIndX));

        if originCheckerColor == "white" % Shape-01 and Shape-02
            dims = iComputeDimsForEvenRows(markerIdX, markerIdY, numDetectedMarkers);
        else % Shape-03 and Shape-04
            dims = iComputeDimsForOddRows(markerIdX, markerIdY, numDetectedMarkers);
        end

    else % Shape-05 and Shape-06
    % There is no vertex at 90 degrees with origin. This can happen when
    % the first and last row of the ChArUco board are not the same. 

        % Find a vertex which has 90 degrees with any other vertex.
        % Note: this logic does not work for boards with 2 rows or 2
        % columns as there will not be any 90 degrees vertex angle.
        % However, CVT does not support such boards in 
        % detectCharucoBoardPoints or generateCharucoBoard.
        vertexInd = iFindVertexClosestTo90Degrees(contour.Vertices);

        % Compute the angles between this vertex and every other vertex.
        angles = iComputeAngles(contour.Vertices, vertexInd);

        % Find the extreme vertex in the vertical direction (a.k.a y-direction).
        verticesWith90degrees = find(bounds(angles, 90));
        
        % There can be no 90 degrees vertex angles for some partial boards.
        vision.internal.errorIf(~any(verticesWith90degrees), ...
            "vision:caltool:DimDetectionFailed");

        cornerIndY = iFindFarthestVertex(contour.Vertices, verticesWith90degrees, vertexInd);
        markerIdY = ids(contour.VertexInds(cornerIndY));

        % Find the extreme vertex in the horizontal direction (a.k.a x-direction).
        verticesWith0degrees = find(bounds(angles, 0) | bounds(angles, 180));
        cornerIndX = iFindFarthestVertex(contour.Vertices, verticesWith0degrees, vertexInd);
        markerIdX = ids(contour.VertexInds(cornerIndX));

        % Normalize the marker ids w.r.t reference vertex marker id.
        refMarkerId = ids(contour.VertexInds(vertexInd));
        markerIdY = abs(markerIdY - refMarkerId);
        markerIdX = abs(markerIdX - refMarkerId);

        if markerIdY < markerIdX
            % MarkerIdY must always be greater than MarkerIdX as markers
            % are arranged in row major in a ChArUco board.
            [markerIdY, markerIdX] = deal(markerIdX, markerIdY);
        end
        
        dims = iComputeDimsForEvenRows(markerIdX, markerIdY, numDetectedMarkers);
    end
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
% Utility to find a vertex with an angle closest to 90 degrees in the contour.
%-----------------------------------------------------------------------------
function vertexInd = iFindVertexClosestTo90Degrees(vertices)
    numCorners = size(vertices,1);
    vertexInds = 1:numCorners;
    
    nextVertexInds = vertexInds+1;
    nextVertexInds(end) = 1;

    prevVertexInds = vertexInds-1;
    prevVertexInds(1) = numCorners;

    angles = zeros(numCorners,1);

    for i = 1:numCorners
        v1 = [vertices(prevVertexInds(i),:) - vertices(vertexInds(i),:),0];
        v2 = [vertices(nextVertexInds(i),:) - vertices(vertexInds(i),:),0];
        angles(i) = atan2d(norm(cross(v1, v2)), dot(v1, v2));
    end

    [~,idx] = min(abs(abs(angles) - 90));
    vertexInd = vertexInds(idx);
end

%-----------------------------------------------------------------------------
% Helper to compute angles of all vertices w.r.t a reference vertex
%-----------------------------------------------------------------------------
function angles = iComputeAngles(vertices, refInd)
    refVertex = vertices(refInd,:);
    numCorners = size(vertices,1);
    angles = zeros(numCorners,1);
    v1 = [vertices(refInd+1,:) - refVertex,0];
    for i = 1:numCorners
        v2 = [vertices(i,:) - refVertex,0];
        angles(i) = atan2d(norm(cross(v1, v2)), dot(v1, v2));
    end
end

%-----------------------------------------------------------------------------
% Helper to compute pattern dimensions for the following contour shapes.
% --------------------------------------------------
% | ID | numRows | numCols | Color | Contour Shape  |
% |-------------------------------------------------|
% | 01 |  even   |   odd   | white |  v--------     |
% |    |         |         |       |  |        |    | (Legacy board)
% |    |         |         |       |   \      /     |
% |    |         |         |       |     ----       |
% | 02 |  even   |  even   | white |  v------       |
% |    |         |         |       |  |       \     |
% |    |         |         |       |  |        |    | (Legacy board)
% |    |         |         |       |   \       |    |
% |    |         |         |       |     ------     |
% | 05 |  even   |   odd   | black |    v----       |
% |    |         |         |       |   /     \      | (Latest board)
% |    |         |         |       |  |       |     |
% |    |         |         |       |   -------      |
% | 06 |  even   |  even   | black |    v------     |
% |    |         |         |       |   /       |    |
% |    |         |         |       |  |        |    | (Latest board)
% |    |         |         |       |  |       /     |
% |    |         |         |       |   ------       |
% --------------------------------------------------
%-----------------------------------------------------------------------------
function dims = iComputeDimsForEvenRows(markerIdX, markerIdY, numDetectedMarkers)
    
    % Try computing the dimensions assuming the number of columns is
    % odd. If it does not yeild a correct result, then retry assuming
    % it is even.
    for isNumColsOdd = [true, false]
        dims = iComputeDimsForEvenRowsImpl(markerIdX, markerIdY, isNumColsOdd);
        if isValidPatternDims(dims, numDetectedMarkers)
            break
        end
    end

    %-------------------------------------------------------------------------
    % Given the ids of the extreme markers in X and Y direction and that
    % the number of rows are even, compute the pattern dimensions. The
    % markerIds are assumed to be normalized to the origin markerId. Here
    % is an example computation:
    %
    %  NumCols: odd                     NumCols: even
    %  MarkerIds: X = 2, Y = 5          markerIds: X = 2, Y = 6
    %   M0   x  M1   x  M2               M0   x  M1   x  M2   x
    %    x  M3   x  M4   x                x  M3   x  M4   x  M5
    %   M5   x  M6   x  M7               M6   x  M7   x  M8   x
    %    x  M8   x  M9  x                x   M9  x   M10  x  M11 
    %  numCols = 2*(2+1) - 1 = 5        numCols = 2*(2+1) = 6
    %  numRows = 2*ceil(6/5) = 4        numRows = 2*ceil(6/6) = 4
    %-------------------------------------------------------------------------
    function dims = iComputeDimsForEvenRowsImpl(markerIdX, markerIdY, isNumColsOdd)
        
        numMarkersAlongCols = markerIdX + 1;
        numCols = 2*numMarkersAlongCols - isNumColsOdd ;
        
        numMarkersAlongRows = ceil((markerIdY + 1)/numCols);
        numRows = 2*numMarkersAlongRows;

        dims = [numRows, numCols];
    end
end

%-----------------------------------------------------------------------------
% Helper to compute pattern dimensions for the following contour shapes.
% --------------------------------------------------
% | ID | numRows | numCols | Color | Contour Shape  |
% |-------------------------------------------------|
% | 03 |   odd   |    odd  | black |    v-----      |
% |    |         |         |       |   /       \    |
% |    |         |         |       |  |         |   | (Legacy & Latest boards)
% |    |         |         |       |   \       /    |
% |    |         |         |       |     -----      |
% | 04 |   odd   |   even  | black |    v-----      |
% |    |         |         |       |   /      |     | (Legacy & Latest boards)
% |    |         |         |       |  |       |     |
% |    |         |         |       |   \      |     |
% |    |         |         |       |     -----      |
% --------------------------------------------------
%-----------------------------------------------------------------------------
function dims = iComputeDimsForOddRows(markerIdX, markerIdY, numDetectedMarkers)
    
    % Try computing the dimensions assuming the number of rows is
    % odd. If it does not yield a correct result, then retry assuming
    % it is even.
    for isNumRowsOdd = [true, false]
        dims = iComputeDimsForOddRowsImpl(markerIdX, markerIdY, isNumRowsOdd);
        if isValidPatternDims(dims, numDetectedMarkers)
            break
        end
    end

    %-------------------------------------------------------------------------
    % Given the ids of the extreme markers in X and Y direction and that
    % the number of columns are odd, compute the pattern dimensions. The
    % markerIds are assumed to be normalized to the origin markerId. Here
    % is an example computation:
    %
    %  NumCols: odd                      NumCols: even
    %  MarkerIds: X = 1, Y = 5           MarkerIds: X = 1, Y = 4
    %    x  M0   x  M1   x                 x  M0   x  M1 
    %   M2   x  M3   x  M4                M2   x  M3   x
    %    x  M5   x  M6   x                 x  M4   x  M5
    %  NumCols = 2*(1+1) + 1 = 5         numCols = 2*(1+1) = 4
    %  NumRows = 2*ceil((5+1)/5)-1 = 3   numRows = ceil(4+1/1+1) = 3
    %-------------------------------------------------------------------------
    function dims = iComputeDimsForOddRowsImpl(markerIdX, markerIdY, isNumRowsOdd)

        if isNumRowsOdd
            numMarkersAlongCols = markerIdX + 1;
            numCols = 2*numMarkersAlongCols + 1;
            numMarkersAlongRows = ceil((markerIdY + 1)/numCols);
            numRows = 2*numMarkersAlongRows - 1;
        else
            numMarkersAlongCols = markerIdX + 1;
            numMarkersAlongRows = markerIdY + 1;
            numRows = ceil(numMarkersAlongRows/numMarkersAlongCols);
            numCols = 2*numMarkersAlongCols;
        end
    
        dims = [numRows, numCols];
    end
end

%-----------------------------------------------------------------------------
% Helper to validate pattern dimensions given the number of markers detected.
%-----------------------------------------------------------------------------
function tf = isValidPatternDims(dims, numDetectedMarkers)

    numMarkers = floor(dims(1)*dims(2)/2);

    % Number of markers computed using valid pattern dimensions must be the
    % same as the number of markers detected.
    tf =  numMarkers == numDetectedMarkers;
end