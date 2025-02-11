function [rotationMatrix, translationVector] = estimateExtrinsicsImpl(imagePoints, worldPoints, intrinsicParams, filename, isCameraParamsSupported)
% estimateExtrinsicsImpl Function to find the rotation and translation from the
% world coordinate system defined by worldPoints into the camera-based
% coordinate system.

% Copyright 2022-2023 The MathWorks, Inc.

%#codegen
%#ok<*EMCLS>
%#ok<*EMCA>

vision.internal.inputValidation.validateExtrinsicsInputs(imagePoints, ...
    worldPoints, intrinsicParams, filename, isCameraParamsSupported);
    
if isa(intrinsicParams, 'fisheyeIntrinsics')
    
    if ~isempty(coder.target) % codegen is not supported
        coder.internal.error('vision:vision_utils:CodegenUnsupported');
    end
        
    pts = imageToNormalizedVector(intrinsicParams, imagePoints);
    u = pts(:, 1) ./ pts(:, 3);
    v = pts(:, 2) ./ pts(:, 3);
    [rotationMatrix, translationVector] = vision.internal.calibration.extrinsicsPlanar(...
        [u, v], worldPoints, eye(3, 'like', pts));
else
    
    if isa(intrinsicParams, 'cameraIntrinsicsKB')
        imagePoints = undistortPoints(imagePoints, intrinsicParams);
    end
    
    intrinsics = intrinsicParams.K;

    [rotationMatrix, translationVector] = vision.internal.calibration.extrinsicsPlanar(...
        imagePoints, worldPoints, intrinsics);
end

end