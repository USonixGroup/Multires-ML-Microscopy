function nativeDetectorsList = getNativeDetectorsList(cameraType)
% getNativeDetectorsList returns list of all native detectors depending on the
% input.

%   Copyright 2021-2024 The MathWorks, Inc.

    arguments
        cameraType (1,1) string {mustBeMember(cameraType,["stereo","monocular"])}
    end

    packageName = "vision.calibration";

    if cameraType == "stereo"
        nativeDetectorsList = ["CheckerboardDetector",...
                               "CharucoBoardDetector",...
                               "AsymmetricCircleGridDetector",...
                               "AprilGridDetector"];
    else % monocular
        nativeDetectorsList = ["CheckerboardDetector",...
                               "CharucoBoardDetector",...
                               "AsymmetricCircleGridDetector",...
                               "SymmetricCircleGridDetector",...
                               "AprilGridDetector"];
    end
    
    nativeDetectorsList = packageName + "." + cameraType + "." + nativeDetectorsList;
end