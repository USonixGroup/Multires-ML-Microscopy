function [points, imageIdx] = separatePoints(points, imageIdx)
% separatePoints Separate detected keypoints in a combined image stack of stereo images.
%     This is a helper function for detectCheckerboardPoints and
%     detectCircleGridPoints to separate the detected keypoints in an image
%     stack that has images from both the cameras of a stereo pair.
%     Inputs: 
%       - points is a numKeypoints-by-2-by-numImages matrix.
%       - imageIdx is a numImages-by-1 logical vector
%     Outputs: 
%       - points is a numKeypoints-by-2-by-numPairs-by-2 matrix.
%       - imageIdx is a numPairs-by-1 logical vector.
%      where numPairs is numImages/2.

% Copyright 2021 MathWorks, Inc.

    numImages = numel(imageIdx);

    % Get the indices corresponding to the two original sets.
    leftRightIdx = ones(numImages, 1);
    leftRightIdx(end/2+1:end) = 2;

    % Find the pairs where the pattern was detected in both images.
    imagesUsedLeft  = imageIdx(1:end/2);
    imagesUsedRight = imageIdx(end/2+1:end);
    commonImagesIdx = imagesUsedLeft & imagesUsedRight;
    commonImagesIdxFull = [commonImagesIdx; commonImagesIdx];

    % Get points on the left camera.
    pointsLeftIdx = commonImagesIdxFull & (leftRightIdx == 1);
    pointsLeftIdx = pointsLeftIdx(imageIdx);
    pointsLeft = points(:, :, pointsLeftIdx);

    % Get points on the right camera.
    pointsRightIdx = commonImagesIdxFull & (leftRightIdx == 2);
    pointsRightIdx = pointsRightIdx(imageIdx);
    pointsRight = points(:, :, pointsRightIdx);

    % Combine the points into a 4D array.
    points = cat(4, pointsLeft, pointsRight);
    imageIdx = commonImagesIdx;
end