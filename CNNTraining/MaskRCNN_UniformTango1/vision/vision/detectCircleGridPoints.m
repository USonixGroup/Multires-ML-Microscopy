function [imagePoints, imagesUsed, userCanceled] = detectCircleGridPoints(varargin)
    
% Copyright 2021-2024 The MathWorks, Inc.

    detector = vision.internal.calibration.CircleGridDetector(mfilename, varargin{:});

    [imagePoints, imagesUsed, userCanceled] = detectKeyPoints(detector);
end