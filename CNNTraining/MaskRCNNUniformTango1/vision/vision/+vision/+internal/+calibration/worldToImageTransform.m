function varargout = worldToImageTransform(intrinsics, rotationMatrix, translationVector, idx, worldPoints, isCameraParamsSupported, varargin)
% worldToImageTransform Function to compute imagePoints and validIndex.

% Copyright 2023 The MathWorks, Inc.

%#codegen

if isa(intrinsics, 'fisheyeIntrinsics')
    nargoutchk(0, 1);
    
    [R, t, pts, outputClass] = vision.internal.inputValidation.parseProjectionInputs( ...
        intrinsics, rotationMatrix, translationVector, worldPoints, ...
        isCameraParamsSupported, mfilename);

    varargout{1} = mapImagePlaneFishEyeIntrinsics(R, t, pts, intrinsics, outputClass);

else % cameraIntrinsics or cameraParameters or cameraIntrinsicsKB
    nargoutchk(0, 2);
    
    [R, t, pts, outputClass, K, applyDistortion] = ...
        vision.internal.inputValidation.parseProjectionInputs(intrinsics, ...
        rotationMatrix, translationVector, worldPoints, ...
        isCameraParamsSupported, mfilename, varargin{idx:end});

    cameraMatrix = vision.internal.constructCameraMatrix(R, t, K);

    [varargout{1:nargout}] = mapImagePlaneCameraIntrinsics(pts, ...
        cameraMatrix, intrinsics, K, applyDistortion, outputClass);
end

end

%--------------------------------------------------------------------------
% Function to map to image plane for fisheyeIntrinsics.
%--------------------------------------------------------------------------
function imagePoints = mapImagePlaneFishEyeIntrinsics(R, t, pts, intrinsics, outputClass)
points = pts * R';
points(:, 1) = points(:, 1) + t(1);
points(:, 2) = points(:, 2) + t(2);
points(:, 3) = points(:, 3) + t(3);

% Add the constant zero a1, so the full set is [a0 a1 a2 a3 a4]
mappingCoef = [intrinsics.MappingCoefficients(1), ...
                  zeros(1, 'like', intrinsics.MappingCoefficients), ...
                  intrinsics.MappingCoefficients(2:end)];

imgPoints = vision.internal.calibration.computeImageProjection(...
    points, mappingCoef, intrinsics.StretchMatrix, intrinsics.DistortionCenter);

imagePoints = cast(imgPoints, outputClass);

end

%--------------------------------------------------------------------------
% Function to map to image plane for cameraIntrinsics.
%--------------------------------------------------------------------------
function varargout = mapImagePlaneCameraIntrinsics(pts, cameraMatrix, intrinsics, K, applyDistortion, outputClass)
nargoutchk(0, 2);

projectedPoints = [pts ones(size(pts, 1), 1)] * cameraMatrix';
imagePointsTmp = bsxfun(@rdivide, projectedPoints(:, 1:2), ...
    projectedPoints(:, 3));

if applyDistortion
    % When computing distortion, the skew effect is removed at the 
    % beginning and added back at the end.
    if isempty(coder.target)

        % Use the builtin c++ function to compute imagePointsTmp when in MATLAB.
        intrinsicMatrix = cast(K, 'like', imagePointsTmp);
        
        if ~isa(intrinsics, 'cameraIntrinsicsKB')
            radialDistortion = cast(intrinsics.RadialDistortion, 'like', imagePointsTmp);
            tangentialDistortion = cast(intrinsics.TangentialDistortion, 'like', imagePointsTmp);
            imagePointsTmp = visionDistortPoints(imagePointsTmp, intrinsicMatrix, ...
                radialDistortion, tangentialDistortion);
        else % cameraIntrinsicsKB
            distortionCoeffs = cast(intrinsics.DistortionCoefficients, 'like', imagePointsTmp);
            imagePointsTmp = visionDistortPointsKB(imagePointsTmp, intrinsicMatrix, ...
                distortionCoeffs);
        end
    else
        % Use MATLAB function to compute imagePointsTmp when in codegen.
        imagePointsTmp = vision.internal.calibration.distortPoints(...
            imagePointsTmp, K, intrinsics.RadialDistortion, ...
            intrinsics.TangentialDistortion);
    end
end

varargout{1} = cast(imagePointsTmp, outputClass);

if nargout == 2
    validateattributes(intrinsics.ImageSize, {'double', 'single'}, ...
        {'nonempty'}, mfilename, 'ImageSize')
    isInFrontOfCamera = projectedPoints(:, 3) > 0;
    isInImageBoundary = (imagePointsTmp(:, 1) >= 0)...
        & (imagePointsTmp(:, 1) <= intrinsics.ImageSize(2))...
        & (imagePointsTmp(:, 2) >= 0)...
        & (imagePointsTmp(:, 2) <= intrinsics.ImageSize(1));
    varargout{2} = isInImageBoundary & isInFrontOfCamera;
end

end