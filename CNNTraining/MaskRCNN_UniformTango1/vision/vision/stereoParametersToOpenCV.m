function [intrinsicMatrix1, distortionCoefficients1, intrinsicMatrix2, ...
          distortionCoefficients2, rotationOfCamera2, translationOfCamera2] ...
             = stereoParametersToOpenCV(stereoParams)

%   Copyright 2021-2023 The MathWorks, Inc.

%#codegen

    validateattributes(stereoParams, {'stereoParameters'}, ...
        {'nonempty', 'scalar'}, mfilename, 'stereoParams');
    
    params1 = stereoParams.CameraParameters1;
    params2 = stereoParams.CameraParameters2;
    
    % Skew is not considered in OCV. Verify whether it is zero.
    coder.internal.errorIf(params1.Skew ~= 0 || params2.Skew ~= 0, ...
        'vision:calibrate:nonzeroSkewCoefficient');
    
    [intrinsicMatrix1, distortionCoefficients1] = cameraIntrinsicsToOpenCV(params1);
    [intrinsicMatrix2, distortionCoefficients2] = cameraIntrinsicsToOpenCV(params2);
    
    % CVT's rotation matrix is transpose of OCV's rotation matrix.
    rotationOfCamera2    = stereoParams.RotationOfCamera2';
    
    translationOfCamera2 = stereoParams.TranslationOfCamera2;    
end