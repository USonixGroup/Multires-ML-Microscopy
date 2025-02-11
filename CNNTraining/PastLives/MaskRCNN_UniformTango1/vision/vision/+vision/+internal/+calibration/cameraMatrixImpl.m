function camMatrix = cameraMatrixImpl(intrinsicParams, isCameraParamsSupported, filename, varargin)
% cameraMatrixImpl Function to compute a camera matrix.

% Copyright 2022-2023 The MathWorks, Inc.

%#codegen

narginchk(4,5)

[K, R, t] = parseInputs(intrinsicParams, isCameraParamsSupported, filename, varargin{:});
camMatrix = vision.internal.constructCameraMatrix(R, t, K);

end

%--------------------------------------------------------------------------
function [K, R, t] = parseInputs(intrinsicParams, isCameraParamsSupported, filename, varargin)

% isCameraParamsSupported indicates cameraMatrix function in use, which
% supports cameraParameters and cameraIntrinsics, as well as a rigid3d
% object or a rotation matrix and a translation vector input
if isCameraParamsSupported
    vision.internal.inputValidation.checkIntrinsicsAndParameters( ...
        intrinsicParams, true, filename, 'cameraMatrix');
    
    if nargin == 5
        rotationMatIn = varargin{1};
        translationVector = varargin{2};
        vision.internal.inputValidation.validateRotationMatrix(...
            rotationMatIn, filename, 'rotationMatrix');
        vision.internal.inputValidation.validateTranslationVector(...
            translationVector, filename, 'translationVector');

        % Transpose the rotation matrix for camera matrix construction
        rotationMatrix = rotationMatIn';
        
        coder.internal.errorIf(~isequal(class(rotationMatrix), class(translationVector)),...
            'vision:calibrate:RandTClassMismatch');
    else
        tform = varargin{1};
        validateattributes(tform, {'rigid3d', 'rigidtform3d'}, {'scalar'}, filename, 'tform')
        rotationMatrix = tform.Rotation';
        translationVector = tform.Translation;
    end
else
    % cameraProjection only supports cameraIntrinsics and rigidtform3 object input
    tform = varargin{1};
    validateattributes(intrinsicParams, {'cameraIntrinsics'}, {'scalar'}, filename, 'cameraProjection');
    validateattributes(tform, {'rigidtform3d'}, {'scalar'}, filename, 'tform')
    
    rotationMatrix = tform.R;
    translationVector = tform.Translation;
end

if isa(rotationMatrix, 'double')
    outputClass = 'double';
else
    outputClass = 'single';
end

K = cast(intrinsicParams.K, outputClass);
R = cast(rotationMatrix, outputClass);
if isrow(translationVector)
    t = cast(translationVector, outputClass);
else
    t = cast(translationVector', outputClass);
end
end