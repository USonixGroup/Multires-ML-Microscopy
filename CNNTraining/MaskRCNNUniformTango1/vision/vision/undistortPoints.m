function [undistortedPoints, reprojectionErrors] = undistortPoints(points, intrinsicParams)

%   Copyright 2014-2023 The MathWorks, Inc.
%#codegen

validateInputs(points, intrinsicParams);

if isa(points, 'double')
    outputClass = 'double';
else
    outputClass = 'single';
end

pointsDouble = double(points);

undistortedPointsDouble = undistortPointsImpl(intrinsicParams, pointsDouble);

if isSimMode
    undistortedPoints = cast(undistortedPointsDouble, outputClass);
else
    % reshape to match the vector output from the codegen version of 
    % lsqcurvefit to match the shape of input points
    undistortedPointsVec = undistortedPointsDouble(:);
    undistortedPointsReshaped  = reshape(undistortedPointsVec, [size(undistortedPointsVec,1)/2, 2]);
    undistortedPoints = cast(undistortedPointsReshaped, outputClass);

end

if nargout > 1
    redistortedPoints = distortPoints(intrinsicParams, undistortedPointsDouble);
    errorsDouble = sqrt(sum((pointsDouble - redistortedPoints).^ 2 , 2));
    reprojectionErrors = cast(errorsDouble, outputClass);
end
end

%--------------------------------------------------------------------------
function validateInputs(points, cameraParams)
validateattributes(points, {'numeric'}, ...
    {'2d', 'nonsparse', 'real', 'finite', 'nonnan', 'size', [NaN, 2]}, mfilename, 'points');

vision.internal.inputValidation.checkIntrinsicsAndParameters( ...
    cameraParams, true, mfilename, 'cameraParams');
end

function mode = isSimMode()
mode = isempty(coder.target);
end