function params = stereoParametersFromOpenCV(leftIntrinsicMatrix, ...
          leftDistortionCoefficients, rightIntrinsicMatrix, ...
          rightDistortionCoefficients, rotationOfCamera2, translationOfCamera2, ...
          imageSize, varargin)

%   Copyright 2021-2024 The MathWorks, Inc.

%#codegen

    narginchk(7,9);
    
    % Validate and parse inputs.
    units = validateAndParseInputs(leftIntrinsicMatrix, ...
        leftDistortionCoefficients, rightIntrinsicMatrix, ...
        rightDistortionCoefficients, rotationOfCamera2, translationOfCamera2, ...
        imageSize, varargin{:});
    
    % Convert intrinsics parameters from OpenCV of both cameras.
    intrinsics1 = cameraIntrinsicsFromOpenCV(leftIntrinsicMatrix, ...
        leftDistortionCoefficients, imageSize);
    intrinsics2  = cameraIntrinsicsFromOpenCV(rightIntrinsicMatrix, ...
        rightDistortionCoefficients, imageSize);

    % Construct a cameraParameters object using a cameraIntrinsics object.
    cameraParams1 = intrinsics2params(intrinsics1, units);
    cameraParams2 = intrinsics2params(intrinsics2, units);
    
    % CVT's rotation matrix is transpose of OCV's rotation matrix.
    rotationOfCamera2     = rotationOfCamera2';
    
    % Construct stereoParameters object.
    params = stereoParameters(cameraParams1, cameraParams2, ...
        rotationOfCamera2, translationOfCamera2);
end


%-------------------------------------------------------------------------------
% Helper to construct a cameraParameters object usign a cameraIntrinsics object.
%-------------------------------------------------------------------------------
function cameraParams = intrinsics2params(intrinsics, worldUnits)
    cameraParams = cameraParameters('IntrinsicMatrix', intrinsics.IntrinsicMatrix,...
        'RadialDistortion', intrinsics.RadialDistortion,...
        'TangentialDistortion', intrinsics.TangentialDistortion,...
        'ImageSize', intrinsics.ImageSize,...
        'WorldUnits', worldUnits);
end

%-------------------------------------------------------------------------------
function units = validateAndParseInputs(leftIntrinsicMatrix, ...
            leftDistortionCoefficients, rightIntrinsicMatrix, ...
            rightDistortionCoefficients, rotationOfCamera2, translationOfCamera2, ...
            imageSize, varargin)
    
    % Validate required inputs.
    validateIntrinsicMatrix(leftIntrinsicMatrix);
    validateDistortionCoefficient(leftDistortionCoefficients);
    
    validateIntrinsicMatrix(rightIntrinsicMatrix);
    validateDistortionCoefficient(rightDistortionCoefficients);
    
    validateRotationMatrix(rotationOfCamera2);
    validateTranslationVector(translationOfCamera2);
    
    validateImageSize(imageSize);
    
    % Parse N-V pair.
    if isempty(coder.target)  % Simulation
        units = parseOptionalInputsSimulation(varargin{:});
    else % Code generation
        units = parseOptionalInputsCodegen(varargin{:});
    end
    
    
end

%-------------------------------------------------------------------------------
function validateIntrinsicMatrix(intrinsicMatrix)
    validateattributes(intrinsicMatrix, {'double', 'single'}, ...
        {'nonempty', '2d', 'finite', 'real', 'nonsparse', ...
        'nrows', 3, 'ncols', 3}, 'stereoParametersFromOpenCV', 'intrinsicMatrix');
end

%-------------------------------------------------------------------------------
function validateDistortionCoefficient(distortionCoefficients)
    validateattributes(distortionCoefficients, {'double', 'single'}, ...
        {'nonempty', 'vector', 'finite', 'real', 'nonsparse'}, ...
        'stereoParametersFromOpenCV', 'distortionCoefficients');
    

    n = numel(distortionCoefficients);
    coder.internal.errorIf(n ~= 4 && n ~= 5 && n ~= 8, ...
        'vision:cameraIntrinsicsFromOpenCV:incorrectNumelCoeffs');
end

%-------------------------------------------------------------------------------
function validateRotationMatrix(rotationMatrix)
    validateattributes(rotationMatrix, {'double', 'single'}, ...
        {'nonempty', '2d', 'finite', 'real', 'nonsparse', ...
        'nrows', 3, 'ncols', 3}, 'stereoParametersFromOpenCV', 'rotationOfCamera2'); 
end

%-------------------------------------------------------------------------------
function validateTranslationVector(translationVector)
    validateattributes(translationVector, {'double', 'single'}, ...
        {'nonempty', 'vector', 'finite', 'real', 'nonsparse', ...
        'numel', 3}, 'stereoParametersFromOpenCV', 'translationOfCamera2'); 
end

%-------------------------------------------------------------------------------
function validateImageSize(imageSize)
    validateattributes(imageSize, {'double', 'single'}, ...
        {'nonempty', 'vector', 'finite', 'real', 'nonsparse', 'integer', ...
        'positive', 'numel', 2}, 'stereoParametersFromOpenCV', 'imageSize');
end

%-------------------------------------------------------------------------------
function units = parseOptionalInputsCodegen(varargin)

    % Set parser inputs.
    pvPairs = struct('WorldUnits', 'mm');

    % Specify parse options.
    poptions = struct( ...
        'CaseSensitivity', false, ...
        'StructExpand',    true, ...
        'PartialMatching', true);

    optarg = coder.internal.parseParameterInputs(pvPairs, poptions, varargin{:});
    units = coder.internal.getParameterValue(optarg.WorldUnits, 'mm', varargin{:});
    validateWorldUnits(units);
end

%-------------------------------------------------------------------------------
function units = parseOptionalInputsSimulation(options)
    arguments %#ok
        options.WorldUnits (1,1) string {validateWorldUnits} = "mm"
    end

    units = options.WorldUnits;
end

%-------------------------------------------------------------------------------
function validateWorldUnits(worldUnits)
    if isstring(worldUnits)
        validateattributes(worldUnits, {'string'}, ...
            {'scalar'}, 'stereoParametersFromOpenCV', 'WorldUnits');
    else
        validateattributes(worldUnits, {'char'}, ...
            {'nonempty','vector'}, 'stereoParametersFromOpenCV', 'WorldUnits');
    end
end