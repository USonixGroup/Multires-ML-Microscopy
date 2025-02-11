function [labels,numClusters] = visionLabelRangeDataCore(location, range, threshold, angleThreshold, minClusterPoints, maxClusterPoints)
%   vision.internal.codegen.pc.visionLabelRangeDataCorePortable is used in
%   segmentLidarData to labels clusters. This file generates portable code.
%   Shared library code generation version is found in 
%   vision.internal.buildable.visionLabelRangeDataBuildable.
%
%   Inputs:
%   -------
%   location  : M-by-N-by-3 matrix, each entry gives x, y, z coordinates
%               of a point.
%   range     : M-by-N-by-3 matrix, each entry gives
%               range, pitch, yaw coordinates of a point.
%   threshold : distance threshold, two points belong to the same cluster
%               if distance is below the threshold.
%   angleThreshold  : angle threshold, two points belong to the same cluster
%                     if angle between the sensor and neighboring points is 
%                     greater than or equal to the input angleThreshold.
%   minClusterPoints : minimum number of points in a valid segmented cluster,
%                      specified as a uint32 scalar. Points that are not part
%                      of a valid cluster have a 0 label.
%   minClusterPoints : maximum number of points in a valid segmented cluster,
%                      specified as a uint32 scalar. Points that are not part
%                      of a valid cluster have a 0 label.
%
%   Outputs:
%   -------
%   labels	    : M-by-N unit32 matrix, each entry gives an integer label.
%                 Zero is reserved for bad (NaN) points.
%   numClusters : scalar, number of clusters (label 0 is not included).

% Copyright 2021 The MathWorks, Inc.
%#codegen

    nRows = size(location, 1);
    nCols = size(location, 2);

    labels = zeros(nRows, nCols, 'uint32');
    currentLabel = ones(1,'uint32');
    
    % Use a queue to keep track of invalid label indices to remove.
    invalidLabelsIdxQueue = coder.nullcopy(zeros([2 nRows*nCols],'int32'));
    invalidLabelsIdxQueueEndIdx = coder.internal.indexInt(1); % end index of queue
    
    % Group loop-invariant parameters into params struct.
    params.nRows = nRows;
    params.nCols = nCols;
    params.threshold = threshold;
    params.angleThreshold = angleThreshold;
    params.minClusterPoints = minClusterPoints;
    params.maxClusterPoints = maxClusterPoints;
    coder.internal.prefer_const(params);
    
    % For each point, find the cluster it belongs and label the cluster.
    for r = 1:nRows
        for c = 1:nCols
            % Skip if we have already labeled this point or if depth is invalid.
            if labels(r,c) > 0 || ~isfinite(location(r,c))
                continue;
            end
            % Search one component for the current label.
            [isValidCluster,invalidLabelsIdxQueue,invalidLabelsIdxQueueEndIdx,labels] = floodFill(...
                location,range,labels,currentLabel,r,c,invalidLabelsIdxQueue,invalidLabelsIdxQueueEndIdx,params);
            if isValidCluster
                % Get new label only if the previous label was used.
                currentLabel = currentLabel + 1;
            end
        end
    end

    numClusters = cast(currentLabel-1, 'like', location);
    % Remove labels for invalid clusters.
    for i=1:invalidLabelsIdxQueueEndIdx-1
        labels(invalidLabelsIdxQueue(1,i),invalidLabelsIdxQueue(2,i)) = 0;
    end
end

function [isValidCluster,invalidLabelsIdx,invalidLabelsIdxEnd,labels] = floodFill(locations,range,labels,label,r,c,invalidLabelsIdx,invalidLabelsIdxEnd,params)
% Label one connected component from the seed position for code generation.
    coder.inline("always");
    
    nRows = params.nRows;
    nCols = params.nCols;
    threshold = params.threshold;
    angleThreshold = params.angleThreshold;
    minClusterPoints = params.minClusterPoints;
    maxClusterPoints = params.maxClusterPoints;
    
    shiftsR = int32([-1 1 0 0]);
    shiftsC = int32([0 0 -1 1]);
    nPoints = nRows * nCols;

    % Use a queue for breadth first search. 
    % When we traverse each point in order, the worst-case space usage for 
    % queueing the unlabeled neighbors is 2*numPoints. This happens when all 
    % points belong to the same label.
    labelQueue = coder.nullcopy(zeros([2 2*nPoints],'int32'));
    labelQueue(:,1) = [r c];
    labelQueueEndIdx = coder.internal.indexInt(2);
    labelQueueFrontIdx = coder.internal.indexInt(1);

    labeledPoint = coder.nullcopy(zeros([2 nPoints],'int32'));
    labeledPointEnd = coder.internal.indexInt(1);

    while labelQueueFrontIdx ~= labelQueueEndIdx
        currR = labelQueue(1,labelQueueFrontIdx);
        currC = labelQueue(2,labelQueueFrontIdx);
        labelQueueFrontIdx = labelQueueFrontIdx+1;

        if labels(currR,currC) > 0 || ~isfinite(locations(currR,currC))
            continue;
        end
        % Set the label of this point to current label.
        labels(currR,currC) = label;

        labeledPoint(:,labeledPointEnd) = [currR,currC];
        labeledPointEnd = labeledPointEnd + 1;
        
        % Search a point's four neighbors.
        for k=1:4
            r = currR + shiftsR(k);
            if r < 1 || r > nRows
                continue;
            end
            c = currC + shiftsC(k);
            % Wrap around when crossing the column border.
            if c < 1
                c = c + nCols;
            elseif c > nCols
                c = c - nCols;
            end

            if labels(r,c) > 0 || ~isfinite(locations(r,c))
                % Skip if we have already labeled this point or if depth is invalid.
                continue;
            end

            dist = iDistanceBetweenPoints(locations,currR,currC,r,c);
            if dist < threshold
                labelQueue(:,labelQueueEndIdx) = [r,c];
                labelQueueEndIdx = labelQueueEndIdx + 1;
            else
                angleDiff = iAngleBetweenPoints(currR,currC,r,c,range,nRows,nCols);
                if angleDiff > angleThreshold
                    labelQueue(:,labelQueueEndIdx) = [r,c];
                    labelQueueEndIdx = labelQueueEndIdx + 1;
                end
            end
        end
    end

    numPoints = labeledPointEnd - 1;
    % Check if the number of points in the cluster is valid.
    isValidCluster = ~((numPoints < minClusterPoints) || (numPoints > maxClusterPoints));
    if ~isValidCluster
        % Use a list to keep track of invalid label indices to remove.
        for i = 1:numPoints
            invalidLabelsIdx(:,invalidLabelsIdxEnd) = labeledPoint(:,i);
            invalidLabelsIdxEnd = invalidLabelsIdxEnd + 1;
        end
    end
end

function dist = iDistanceBetweenPoints(locations,currR,currC,r,c)
% Compute the L2 distance between two points.
    coder.inline("always");
    dist = (locations(currR,currC,1) - locations(r,c,1)) * (locations(currR,currC,1) - locations(r,c,1));
    dist = dist + (locations(currR,currC,2) - locations(r,c,2)) * (locations(currR,currC,2) - locations(r,c,2));
    dist = dist + (locations(currR,currC,3) - locations(r,c,3)) * (locations(currR,currC,3) - locations(r,c,3));
    dist = sqrt(dist);
end

function angleDiff = iAngleBetweenPoints(r1,c1,r2,c2,range,nRows,nCols)
% Compute angle to determine if two points are from the same object.
    coder.inline("always");
    % depth is the 1st channel in range, i.e. range(r,c,1).
    % pitch is the 2nd channel in range, i.e. range(r,c,2).
    % yaw   is the 3rd channel in range, i.e. range(r,c,3).
    currentDepth  = range(r1,c1,1);
    neighborDepth = range(r2,c2,1);
    
    alpha = cast(0,'like',range);
    if (r1 < r2)
        alpha = range(r2,c2,2) - range(r1,c1,2);
    elseif (r1 > r2 && r1 < nRows)
        alpha = range(r1,c1,2) - range(r2,c2,2);
    elseif (r1 > r2 && r1 == nRows)
        alpha = cast(0,'like',range);
    elseif (c1 == 1 && c2 == nCols)
        alpha = range(r2,c2,3) - range(r1,c1,3);
        if (alpha > cast(pi,'like',range))
            alpha = alpha - 2 * cast(pi,'like',range);
        elseif (alpha < -cast(pi,'like',range))
            alpha = alpha + 2 * cast(pi,'like',range);
        else
            % Keep alpha as is.
        end
    elseif (c2 == 1 && c1 == nCols)
        alpha = range(r2,c2,3) - range(r1,c1,3);
        if (alpha > pi)
            alpha = alpha - 2 * cast(pi,'like',range);
        elseif (alpha < -pi)
            alpha = alpha + 2 * cast(pi,'like',range);
        else
            % Keep alpha as is.
        end
    elseif (c1 < c2)
        alpha = range(r2,c2,3) - range(r1,c1,3);
    elseif (c1 > c2)
        alpha = range(r1,c1,3) - range(r2,c2,3);
    else
        alpha = cast(0,'like',range);
    end

    d1 = max(currentDepth, neighborDepth);
    d2 = min(currentDepth, neighborDepth);
    beta = atan2((d2 * sin(alpha)), (d1 - d2 * cos(alpha)));
    angleDiff = cast(abs(beta),'like',range);
end