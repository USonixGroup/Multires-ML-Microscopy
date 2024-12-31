function [refinedPose, reprojectionErrors] = bundleAdjustmentMotion(varargin)

% Copyright 2019-2023 The MathWorks, Inc.

%#codegen
nargoutchk(0, 2);
narginchk(4, 14);
[refinedPose, reprojectionErrors] = vision.internal.bundleAdjust.sparseBA(...
    'motion', mfilename, varargin{:});
