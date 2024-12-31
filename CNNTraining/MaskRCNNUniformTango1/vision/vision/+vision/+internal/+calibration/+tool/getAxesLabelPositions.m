function [originLabel, xLabel, yLabel] = getAxesLabelPositions(imagePoints, numRows, numCols, varargin)
% getAxesLabelPositions returns label position and orientation of origin, 
% x-axis and y-axis given the arrangement of detected points in the image.
% These positions and orientations are computed using three reference
% points.
%
% imagePoints - M-by-2 points
% numRows - Number of keypoint rows in the calibration pattern
% numCols - Number of keypoint columns in the calibration pattern
% refPtIdxOffset - (optional) Index offset from first reference point to 
%                  find the second reference point. This is primarily needed 
%                  for asymmetric circle grid patterns.
%
% This function is used to plot the axes labels over the calibration pattern
% detections in vision.calibration.PatternDetector.drawImageAxesLabels method.

% Copyright 2024 The MathWorks, Inc.
    
    if nargin > 3
        refPtIdxOffset = varargin{1};
    else
        refPtIdxOffset = 1;
    end

    patternCoordsX = reshape(imagePoints(:,1), [numRows, numCols]);
    patternCoordsY = reshape(imagePoints(:,2), [numRows, numCols]);
    patternCoords  = cat(3, patternCoordsX, patternCoordsY);
    
    % Origin label
    p1 = patternCoords(1, 1, :);
    p2 = patternCoords(1 + refPtIdxOffset, 1, :);
    p3 = patternCoords(1, 1 + refPtIdxOffset, :);
    originLabel = getAxesLabelPosition(p1, p2, p3, refPtIdxOffset);
    
    % X-axis label
    p1 = patternCoords(1, numCols  , :);
    p3 = patternCoords(1, numCols - refPtIdxOffset, :);
    p2 = patternCoords(1 + refPtIdxOffset, numCols, :);
    xLabel = getAxesLabelPosition(p1, p2, p3, refPtIdxOffset);
    xLabel.Orientation = 180 + xLabel.Orientation;
    
    % Y-axis label
    p1 = patternCoords(numRows  , 1, :);
    p2 = patternCoords(numRows - refPtIdxOffset, 1, :);
    p3 = patternCoords(numRows  , 1 + refPtIdxOffset, :);
    yLabel = getAxesLabelPosition(p1, p2, p3, refPtIdxOffset);
end

%--------------------------------------------------------------
% p1+v
%  \
%   \     v1
%    p1 ------ p3
%    |
% v2 |
%    |
%    p2
function labelPosition = getAxesLabelPosition(p1, p2, p3, offset)
    v1 = p3 - p1;
    theta = -atan2d(v1(2), v1(1));

    v2 = p2 - p1;
    v = -v1 - v2;
    d = hypot(v(1), v(2));
    minDist = 40;
    maxDist = 100;
    if d < minDist
        v = (v / d) * minDist;
    end

    if d > maxDist
        v = (v / d) * maxDist;
    end
    % Half the length for asymmetric circle grid as every other
    % rows/columns are used for computation.
    v = v/offset;
    loc = p1 + v;

    labelPosition.Location = loc;
    labelPosition.Orientation = theta;
end