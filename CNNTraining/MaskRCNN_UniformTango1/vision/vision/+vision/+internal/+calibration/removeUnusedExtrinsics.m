% Remove the extrinsics corresponding to the images that were not used by
% the other camera

% Copyright 2017 MathWorks, Inc.
function cameraParams = removeUnusedExtrinsics(cameraParams, pairsUsed, ...
    imagesUsed)
rotationVectors = zeros(numel(pairsUsed), 3);
rotationVectors(imagesUsed, :) = cameraParams.RotationVectors;

translationVectors = zeros(numel(pairsUsed), 3);
translationVectors(imagesUsed, :) = cameraParams.TranslationVectors;

cameraParams.setExtrinsics(...
    rotationVectors(pairsUsed, :), translationVectors(pairsUsed, :));