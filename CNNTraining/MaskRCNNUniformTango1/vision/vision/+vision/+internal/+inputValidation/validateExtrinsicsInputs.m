function validateExtrinsicsInputs(imagePoints, worldPoints, cameraParams, filename, varargin)
% validateExtrinsicsInputs Validate parameters for computing extrinsics. 
% At least 4 points are needed

%   Copyright 2018-2022 The MathWorks, Inc.

%#codegen

narginchk(4, 5);

% Set isCameraParamsSupported to true by default.
if isempty(varargin)
    isCameraParamsSupported = true;
else
    isCameraParamsSupported = varargin{1};
end

% image pts
validateattributes(imagePoints, {'double', 'single'}, ...
    {'real', 'finite', 'nonsparse','2d', 'ncols', 2}, ...
    filename, 'imagePoints');

% world pts
validateattributes(worldPoints, {'double', 'single'}, ...
    {'real', 'finite', 'nonsparse', '2d', 'ncols', 2}, ...
    filename, 'worldPoints');

% points must be of the same class
coder.internal.errorIf( ~strcmp(class(imagePoints), class(worldPoints)), ...
    'vision:calibrate:pointsClassMismatch');

% same number of points
coder.internal.errorIf( size(imagePoints, 1) ~= size(worldPoints, 1), ...
    'vision:calibrate:numberOfPointsMustMatch');

% min number of points
minNumberOfPoints2D = 4;
coder.internal.errorIf(...
    size(worldPoints, 2) == 2 && size(worldPoints, 1) < minNumberOfPoints2D, ...
    'vision:calibrate:minNumWorldPoints', minNumberOfPoints2D - 1);

% camera parameters
if isCameraParamsSupported
    validateattributes(cameraParams, ...
        {'cameraParameters', 'cameraIntrinsics', 'fisheyeIntrinsics', 'cameraIntrinsicsKB'}, ...
        {'scalar'}, filename, 'cameraParams');
else
    validateattributes(cameraParams, ...
        {'cameraIntrinsics', 'fisheyeIntrinsics', 'cameraIntrinsicsKB'}, {'scalar'}, filename, 'cameraParams');
end
