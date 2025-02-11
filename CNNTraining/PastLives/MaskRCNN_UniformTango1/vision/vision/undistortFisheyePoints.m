function [undistortedPoints, camIntrinsics, reprojectionErrors] = ...
    undistortFisheyePoints(points, intrinsics, varargin)

% Copyright 2017-2024 The MathWorks, Inc.
%#codegen
[pointsDouble, outputClass, scaleFactor] = validateAndParseInputs(points, ...
    intrinsics, varargin{:});

imageSize = intrinsics.ImageSize;
coder.internal.errorIf(isempty(imageSize), 'vision:calibrate:emptyImageSize');

 % Default to the middle of the original image, which is the same as for 
 % undistortFisheyeImage.  That way, they'll match at least for the 'same'
 % output view. It's a more intuitive result.
principalPoint = imageSize([2 1]) / 2 + 0.5 ;
f = min(imageSize) / 2;
focalLength = f .* scaleFactor(:)';
camIntrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);

undistortedPointsDouble = undistortPointsImpl(intrinsics, pointsDouble, camIntrinsics);
undistortedPoints = cast(undistortedPointsDouble, outputClass);

if nargout > 2
    redistortedPoints = distortPoints(intrinsics, undistortedPointsDouble, camIntrinsics);
    errorsDouble = sqrt(sum((pointsDouble - redistortedPoints).^ 2 , 2));
    reprojectionErrors = cast(errorsDouble, outputClass);
end

%--------------------------------------------------------------------------
function [pointsDouble, outputClass, scaleFactor] = ...
    validateAndParseInputs(points, intrinsics, varargin)
validateattributes(points, {'numeric'}, ...
    {'2d', 'nonsparse', 'real', 'size', [NaN, 2]}, mfilename, 'points');

validateattributes(intrinsics, {'fisheyeIntrinsics'}, {}, mfilename, 'intrinsics');

if isa(points, 'double')
    outputClass = 'double';
else
    outputClass = 'single';
end

pointsDouble = double(points);

if isempty(varargin)
    scaleFactor = [1 1];
else
    scaleFactor = varargin{1};
    if isscalar(scaleFactor)
        validateattributes(scaleFactor, {'single', 'double'}, ...
            {'scalar', 'nonsparse', 'real', 'positive'},...
            mfilename, 'scaleFactor');
        scaleFactor = [scaleFactor, scaleFactor];
    else
        validateattributes(scaleFactor, {'single', 'double'}, ...
            {'vector', 'nonsparse', 'real', 'numel', 2, 'positive'},...
            mfilename, 'scaleFactor');
    end
end