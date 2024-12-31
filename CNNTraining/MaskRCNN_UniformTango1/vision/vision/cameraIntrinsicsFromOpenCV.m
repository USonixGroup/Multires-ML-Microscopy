function intrinsics = cameraIntrinsicsFromOpenCV(intrinsicMatrix, distortionCoefficients, imageSize)

%   Copyright 2021-2024 The MathWorks, Inc.

%#codegen

    validateInputs(intrinsicMatrix, distortionCoefficients, imageSize);

    [focalLength, principalPoint] = ...
            getIntrinsicParamsFromOCV(intrinsicMatrix);

    if numel(distortionCoefficients) == 4 % fisheye intrinsics

        intrinsics = cameraIntrinsicsKB.construct(focalLength, principalPoint, ...
            imageSize, distortionCoefficients);
    else % camera intrinsics
        
        [radialDistortion, tangentialDistortion] = ...
            getDistortionCoefficientsFromOCV(distortionCoefficients);
    
        intrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize, ...
                                      'RadialDistortion', radialDistortion, ...
                                      'TangentialDistortion', tangentialDistortion);
    end
end

function [focalLength, principalPoint] = getIntrinsicParamsFromOCV(intrinsicMatrix)
    % OpenCV's intrinsic matrix
    % -------------------------
    %  -          -            
    % | fx   0   cx|           
    % |  0  fy   cy|           
    % |  0   0    1|           
    %  -          -            
    
    % Focal length.
    fx = intrinsicMatrix(1,1);
    fy = intrinsicMatrix(2,2);
    focalLength = [fx fy];
    
    % Principal point.
    cx = intrinsicMatrix(1,3) + 1;
    cy = intrinsicMatrix(2,3) + 1;
    principalPoint = [cx cy];
end

function [radialDistortion, tangentialDistortion] = getDistortionCoefficientsFromOCV(distCoeff)
    
    % Distortion Coefficients in OpenCV [k1 k2 p1 p2 k3 k4 k5 k6].
    n = numel(distCoeff);
    radialDistortion = distCoeff([1, 2, 5:n]);
    tangentialDistortion = distCoeff([3,4]);
end

function validateInputs(intrinsicMatrix, distortionCoefficients, imageSize)

    validateattributes(intrinsicMatrix, {'double', 'single'}, ...
        {'nonempty', '2d', 'finite', 'real', 'nonsparse', ...
        'size', [3,3]}, 'cameraIntrinsicsFromOpenCV', 'intrinsicMatrix');
    validateattributes(distortionCoefficients, {'double', 'single'}, ...
        {'nonempty', 'vector', 'finite', 'real', 'nonsparse'}, ...
        'cameraIntrinsicsFromOpenCV', 'distortionCoefficients');

    n = numel(distortionCoefficients);
    coder.internal.errorIf(n ~= 4 && n ~= 5 && n ~= 8, ...
        'vision:cameraIntrinsicsFromOpenCV:incorrectNumelCoeffs');

    validateattributes(imageSize, {'double', 'single'}, ...
        {'nonempty', 'vector', 'finite', 'real', 'nonsparse', 'integer', ...
        'positive', 'numel', 2}, 'cameraIntrinsicsFromOpenCV', 'imageSize');
end