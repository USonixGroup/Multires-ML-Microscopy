function worldPoints = mapXYWorldPlane(intrinsics, rotationMatrix, translationVector, imagePoints, isCameraParamsSupported)
% mapXYWorldPlane Function to map to X-Y world plane.

% Copyright 2022-2023 The MathWorks, Inc.

%#codegen

if isa(intrinsics, 'fisheyeIntrinsics')
    [R, t, pts, outputClass] = vision.internal.inputValidation.parseProjectionInputs(...
    intrinsics, rotationMatrix, translationVector, imagePoints, ...
    isCameraParamsSupported, mfilename);

    % Compute the normalized vector on unit sphere, i.e, ray vector
    X = imageToNormalizedVector(intrinsics, pts);
    tform = [R(:, 1), R(:, 2), t'];
elseif isa(intrinsics, 'cameraIntrinsicsKB') 
    [R, t, pts, outputClass, K] = vision.internal.inputValidation.parseProjectionInputs(...
    intrinsics, rotationMatrix, translationVector, imagePoints, ...
    isCameraParamsSupported, mfilename);

    undistortedPts = undistortPoints(pts, intrinsics);
    
    X = [undistortedPts,ones(size(undistortedPts, 1),1,'like',undistortedPts)];
    tform = K * [R(:, 1), R(:, 2), t'];  
else % cameraIntrinsics
    [R, t, pts, outputClass, K] = vision.internal.inputValidation.parseProjectionInputs(...
    intrinsics, rotationMatrix, translationVector, imagePoints, ...
    isCameraParamsSupported, mfilename);

    X = [pts,ones(size(pts, 1),1,'like',pts)];
    tform = K * [R(:, 1), R(:, 2), t'];
end

U = X / tform';

if isempty(U)
    Y = zeros(0,2,'like',U);
else
    U(:, 1) = U(:, 1)./ U(:, 3);
    U(:, 2) = U(:, 2)./ U(:, 3);
    Y = U(:, 1:2);
end

worldPoints = cast(Y, outputClass);

end