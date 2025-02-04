function [intrinsicMatrix, distortionCoefficients] = cameraIntrinsicsToOpenCV(intrinsics)

%   Copyright 2021-2023 The MathWorks, Inc.

%#codegen

    validateInput(intrinsics);
    
    intrinsicMatrix = getOCVIntrinsicMatrix(intrinsics);
    
    distortionCoefficients = getOCVDistortionCoefficients(intrinsics);
end

function distortionCoefficients = getOCVDistortionCoefficients(intrinsics)    
    
    % Distortion Coefficients in OpenCV [k1 k2 p1 p2 k3]
    distortionCoefficients = zeros(1,5);
    if length(intrinsics.RadialDistortion) == 3
        distortionCoefficients([1,2,5]) = intrinsics.RadialDistortion;
    else
        distortionCoefficients([1,2]) = intrinsics.RadialDistortion;
    end
    distortionCoefficients([3,4])   = intrinsics.TangentialDistortion;
end

function intrinsicMatrix = getOCVIntrinsicMatrix(intrinsics)
    % Focal length.
    fx = intrinsics.FocalLength(1);
    fy = intrinsics.FocalLength(2);
    
    % Principal point.
    cx = intrinsics.PrincipalPoint(1) - 1;
    cy = intrinsics.PrincipalPoint(2) - 1;        
    
    % Construct OpenCV's intrinsic matrix.
    intrinsicMatrix = [fx   0  cx;
                        0  fy  cy;
                        0   0  1];
end

function validateInput(intrinsics)

    validateattributes(intrinsics, {'cameraIntrinsics', 'cameraParameters'}, ...
        {'nonempty', 'scalar'}, mfilename, 'intrinsics');
    
    % Skew is not considered in OCV. Verify whether it is zero.
    coder.internal.errorIf(intrinsics.Skew ~= 0, 'vision:calibrate:nonzeroSkewCoefficient');
end