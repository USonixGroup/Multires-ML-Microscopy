function checkIntrinsicsAndParameters(intrinsics, scalarFlag, fileName, varargin)
%checkIntrinsicsAndParameters Validate inputs of cameraIntrinsics,
% cameraIntrinsicsKB or cameraParameters type

% Copyright 2019-2023 Mathworks, Inc.

%#codegen

if scalarFlag % Scalar only
    attributes = {'scalar'};
else
    % Allows an array of objects. Note that cameraParameters can only be a
    % scalar due to restriction in its constructor.
    attributes = {'vector'};
end

validIntrinsicsTypes = {'cameraIntrinsics', 'cameraParameters', ...
    'cameraIntrinsicsKB', 'cameraParametersKB'};

validateattributes(intrinsics, validIntrinsicsTypes, attributes, fileName, varargin{:});