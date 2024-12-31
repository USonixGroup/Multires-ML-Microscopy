function numLevels = adjustNumScaleLevels(imageSize, numLevels, scaleFactor)
%adjustNumScaleLevels Adjust the number of levels based on the scale factor and 
% the image size during ORB feature detection.

% Copyright 2023 The MathWorks, Inc.

%#codegen
patchSize=31;

% To make sure last level has 31x31 pixel size.
maxNumLevels = floor((log(min(imageSize))-log(double(patchSize)*2+1))/log(double(scaleFactor)))+1;

if numLevels > maxNumLevels
    numLevels = maxNumLevels;
    warning(message('vision:vslam_utils:maxNumLevels', ...
        sprintf('%4.3f',scaleFactor), imageSize(1), imageSize(2), maxNumLevels, numLevels));
end
end