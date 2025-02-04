%==========================================================================
% Check runtime status and report error if there is one
%==========================================================================
function checkRansacRuntimeStatus(statusCode, status)

%   Copyright 2015-2020 The MathWorks, Inc.

%#codegen

if status == statusCode.NotEnoughPts
    coder.internal.warning('vision:pointcloud:notEnoughPts');
elseif status == statusCode.NotEnoughInliers
    coder.internal.warning('vision:pointcloud:notEnoughInliers');
end
