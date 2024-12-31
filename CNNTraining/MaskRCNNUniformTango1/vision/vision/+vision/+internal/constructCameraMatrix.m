function cameraMatrix = constructCameraMatrix(R, t, K)
% constructCameraMatrix Function to construct camera matrix from the intrinsics and extrinsics.

% Copyright 2018-2022 The MathWorks, Inc.
%#codegen
    cameraMatrix = K * [R, t'];
end
