function [camMatrix, reprojectionErrors] = estimateCameraProjection(imagePoints, worldPoints)

% Copyright 2022-2023 The MathWorks, Inc.

%#codegen

[camMatrix, reprojectionErrors] = vision.internal.calibration.estimateCameraMatrixImpl( ...
    imagePoints, worldPoints, mfilename);

end