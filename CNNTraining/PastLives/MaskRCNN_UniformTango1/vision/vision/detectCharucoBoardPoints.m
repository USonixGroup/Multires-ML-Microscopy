function [imagePoints, imagesUsed, userCanceled] = detectCharucoBoardPoints(varargin)

% Copyright 2024 The MathWorks, Inc.
    
    detector = vision.internal.calibration.CharucoBoardDetector(mfilename, varargin{:});

    [imagePoints, imagesUsed, userCanceled] = detectKeyPoints(detector);
end