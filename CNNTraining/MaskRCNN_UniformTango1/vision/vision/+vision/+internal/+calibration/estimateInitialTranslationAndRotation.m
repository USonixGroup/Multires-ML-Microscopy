% Compute the initial estimate of translation and rotation of camera 2
% relative to camera 1.

% Copyright 2017 MathWorks, Inc.
function [R, t] = estimateInitialTranslationAndRotation(cameraParameters1, ...
    cameraParameters2)
% Now the number of images in both cameras should be the same.
numImages = cameraParameters1.NumPatterns;

rotationVectors = zeros(numImages, 3);
translationVectors = zeros(numImages, 3);

% For each set of extrinsics, compute R (rotation) and T (translation)
% between the two cameras.
for i = 1:numImages
    R = cameraParameters2.RotationMatrices(:, :, i)' * ...
        cameraParameters1.RotationMatrices(:, :, i);
    rotationVectors(i, :) = vision.internal.calibration.rodriguesMatrixToVector(R);
    translationVectors(i, :) = (cameraParameters2.TranslationVectors(i, :)' - ...
        R * cameraParameters1.TranslationVectors(i, :)')';
end

% Take the median rotation and translation as the initial guess.
r = median(rotationVectors, 1);
R = vision.internal.calibration.rodriguesVectorToMatrix(r)';
t = median(translationVectors, 1);