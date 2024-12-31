function camPose = extr2pose(extrinsics)

% Copyright 2022-2023 The MathWorks, Inc.

%#codegen

% Validate extrinsics input.
validateattributes(extrinsics, {'rigidtform3d'}, {'scalar'}, mfilename, 'extrinsics');

% Get camPose output.
camPose = extrinsics.invert;
end
