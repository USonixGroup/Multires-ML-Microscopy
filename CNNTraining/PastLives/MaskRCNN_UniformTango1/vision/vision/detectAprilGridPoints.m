function [imagePoints, imagesUsed, userCanceled] = detectAprilGridPoints(varargin)

% Copyright 2024 The MathWorks, Inc.
    
    detector = vision.internal.calibration.AprilGridDetector(mfilename, varargin{:});

    [imagePoints, imagesUsed, userCanceled] = detectKeyPoints(detector);
end