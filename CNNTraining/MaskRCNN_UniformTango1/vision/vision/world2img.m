function varargout = world2img(worldPoints, varargin)

% Copyright 2022-2023 The MathWorks, Inc.

%#codegen

narginchk(3,5)

% Validate worldPoints
validateattributes(worldPoints, {'double', 'single'}, ...
    {'real', 'nonsparse', 'nonempty', '2d', 'ncols', 3}, mfilename, 'worldPoints')

% Validate tform and pull the rotation matrix and translation vector from tform
tform = varargin{1};
validateattributes(tform, {'rigidtform3d'}, {'scalar'}, mfilename, 'tform')
rotationMatrix = tform.R;
translationVector = tform.Translation;
idx = 3;

% Validate intrinsics
intrinsics = varargin{2};
validateattributes(intrinsics, {'cameraIntrinsics', 'fisheyeIntrinsics', 'cameraIntrinsicsKB'}, ...
    {'scalar'}, mfilename, 'intrinsics')

% True if cameraParameters is a supported class for intrinsics
isCameraParamsSupported = false;

[varargout{1:nargout}] = vision.internal.calibration.worldToImageTransform( ...
    intrinsics, rotationMatrix, translationVector, idx, worldPoints, ...
    isCameraParamsSupported, varargin{:});

end