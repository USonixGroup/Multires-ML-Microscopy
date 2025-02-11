function [worldPose, inlierIdx, status] = estworldpose(imagePoints, worldPoints, intrinsics, varargin)

% Copyright 2022-2023 The MathWorks, Inc.

%#codegen

% True if cameraParameters is a supported class for intrinsics
isCameraParamsSupported = false;

[orientation, location, inlierIdx, status, statusCode] = vision.internal.calibration.estWorldPoseImpl( ...
    imagePoints, worldPoints, intrinsics, isCameraParamsSupported, mfilename, varargin{:});

if nargout < 3
    checkRuntimeStatus(statusCode, status);
end

% Orientation and location are only valid on a no-error status
if status == 0
    worldPose = rigidtform3d(orientation', location);
else
    worldPose = rigidtform3d();
end

end

%==========================================================================
% Check runtime status and report error if there is one
%==========================================================================
function checkRuntimeStatus(statusCode, status)
coder.internal.errorIf(status==statusCode.NotEnoughPts, ...
    'vision:points:notEnoughMatchedPts', 'imagePoints', 'worldPoints', 4);

coder.internal.errorIf(status==statusCode.NotEnoughInliers, ...
    'vision:points:notEnoughInlierMatches', 'imagePoints', ...
    'worldPoints');
end