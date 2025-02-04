function [tform1,tform2] = estimateStereoRectification(f,inlierPoints1,inlierPoints2,imageSize)

% Copyright 2022-2023 The MathWorks, Inc.

%#codegen

[tform1, tform2] = vision.internal.calibration.estimateStereoRectificationImpl(f, ...
    inlierPoints1, inlierPoints2, imageSize, mfilename, true);

end