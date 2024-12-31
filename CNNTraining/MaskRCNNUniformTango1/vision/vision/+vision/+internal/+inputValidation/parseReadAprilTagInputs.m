function [Iuint8, tagFamily, doEstimatePose, intrinsics, tagSize, detectorParams] = parseReadAprilTagInputs(I, varargin)
% parseReadAprilTagInputs Utility to parse and validate inputs of readAprilTag function.

% Copyright 2024 The MathWorks, Inc.

%#codegen
   
    % Validate input image.
    Iuint8 = validateImage(I);
    
    % Parse inputs for all syntaxes.
    if nargin == 1
        % Syntax: readAprilTag(I)
        tagFamily = 'all';
        doEstimatePose = false;
        nvpInputs = [];
    elseif nargin == 2 
        % Syntax: readAprilTag(I, tagFamily)
        tagFamily = varargin{1};
        doEstimatePose = false;
        nvpInputs = [];
    elseif isa(varargin{1},'cameraIntrinsics') || isa(varargin{1},'cameraParameters') 
        % Syntax: readAprilTag(I, intrinsics, tagSize, NVP)
        tagFamily = 'all';
        doEstimatePose = true;
        intrinsics = varargin{1};
        tagSize = varargin{2};
        if nargin > 3
            nvpInputs = {varargin{3:end}};
        else
            nvpInputs = [];
        end
    elseif isa(varargin{2},'cameraIntrinsics') || isa(varargin{2},'cameraParameters')
        % Syntax: readAprilTag(I, tagFamily, intrinsics, tagSize, NVP)
        tagFamily = varargin{1};
        doEstimatePose = true;
        intrinsics = varargin{2};
        tagSize = varargin{3};
        if nargin > 4
            nvpInputs = {varargin{4:end}};
        else
            nvpInputs = [];
        end
    elseif mod(nargin,2) == 1 
        % Syntax: readAprilTag(I, NVP)
        tagFamily = 'all';
        doEstimatePose = false;
        nvpInputs = varargin;
    elseif ischar(varargin{2}) || isstring(varargin{2})
        % Syntax: readAprilTag(I, tagFamily, NVP)
        tagFamily = varargin{1};
        doEstimatePose = false;
        nvpInputs = {varargin{2:end}};
    else
        % Syntax: readAprilTag(I, tagFamily, invalidIntrinsics, tagSize, NVP)
        tagFamily = varargin{1};
        doEstimatePose = true;
        intrinsics = varargin{2};
        tagSize = varargin{3};
        if nargin > 5
            nvpInputs = {varargin{4:end}};
        else
            nvpInputs = [];
        end
    end
    
    % Validate tag family
    tagFamily = validateTagFamily(tagFamily);
    
    if doEstimatePose
        % Validate intrinsics input.
        intrinsics = validateIntrinsics(intrinsics);

        % Validate tag size input.
        validateTagSize(tagSize);
    else
        % Allocate memory for intrinsics by assigning default values for
        % camera parameters.
        intrinsics = cameraIntrinsics(ones(1,2),ones(1,2),ones(1,2));
        tagSize = 0;
    end
    
    % Parse and validate name-value arguments.
    if isempty(nvpInputs)
        detectorParams = validateAndParseNVPInputs;
    else
        detectorParams = validateAndParseNVPInputs(nvpInputs{:});
    end
end

%--------------------------------------------------------------------------
function intrinsics = validateIntrinsics(intrinsics)
    
    validTypes = {'cameraIntrinsics', 'cameraParameters'};
    validateattributes(intrinsics, validTypes, {'scalar'}, 'readAprilTag', 'intrinsics');

    if isa(intrinsics, 'cameraParameters')
        intrinsics = intrinsics.Intrinsics;
    end
end

%--------------------------------------------------------------------------
function tagFamily = validateTagFamily(tagFamily)
    validateattributes(tagFamily, {'char', 'string', 'cell'}, {'nonempty', 'vector'}, 'readAprilTag', 'tagFamily');
    
    supportedFamilies = vision.internal.supportedAprilTagFamilies();
    
    if ischar(tagFamily) || isStringScalar(tagFamily)
        
        if ~isSimMode()
            n = size(supportedFamilies,2);
            validFamiliesChar = cell(1,n+1);
            validFamiliesChar{1} = 'all';
            for i = 1:n
                validFamiliesChar{i+1} = supportedFamilies{i};
            end
            
            % A tag family is valid if it matches with families present in
            % validFamilies.
            isValidFamily = any(strcmp(validFamiliesChar,tagFamily));
            
            formatMsg = strjoin(supportedFamilies, ', ');
            vision.internal.errorIf(~isValidFamily,'vision:apriltag:unrecognizedStringChoice', formatMsg);
            
        else 
            validFamiliesChar = [{'all'}, supportedFamilies(:)'];
            tagFamily = validatestring(tagFamily, validFamiliesChar, 'readAprilTag', 'tagFamily');
        end
    else
            
        vision.internal.errorIf(~iscellstr(tagFamily) && ...
        ~isstring(tagFamily),'vision:apriltag:invalidStringList');
        
        if isSimMode()
            isValidFamily = all(ismember(tagFamily, supportedFamilies));
        else
            count = 0;
            if iscellstr(tagFamily) %#ok
                for i = 1:size(tagFamily,2)
                    count = count+any(strcmp(supportedFamilies,tagFamily{i}));
                end
                isValidFamily = (count == size(tagFamily,2));
            else
                isValidFamily = any(strcmp(supportedFamilies, tagFamily));
            end
        end
        
        formatMsg = strjoin(supportedFamilies, ', ');
        vision.internal.errorIf(~isValidFamily,'vision:apriltag:unrecognizedStringChoice', formatMsg);
    
        if isSimMode()
            tagFamily = cellstr(tagFamily);
        end
    end
end

%--------------------------------------------------------------------------
function validateTagSize(tagSize)
    validateattributes(tagSize, {'numeric'}, {'finite', 'real', 'nonsparse', ...
        'scalar', 'positive'}, 'readAprilTag', 'tagSize');
end

%--------------------------------------------------------------------------
function Iuint8 = validateImage(I)

    % Validate input image.
    vision.internal.inputValidation.validateImage(I);
    
    % Convert to grayscale.
    if ~ismatrix(I)
        Igray = rgb2gray(I);
    else
        Igray = I;
    end
    
    % Ensure image data is uint8.
    if ~isa(Igray, 'uint8')
        Iuint8 = im2uint8(Igray);
    else
        Iuint8 = Igray;
    end
end

%--------------------------------------------------------------------------
function params = validateAndParseNVPInputs(options)
    arguments
        options.DecimationFactor (1,1) {mustBeNumeric, mustBeFinite, mustBeNonsparse, ...
            mustBePositive, mustBeGreaterThanOrEqual(options.DecimationFactor,1)} = 2
        options.GaussianSigma (1,1) {mustBeNumeric, mustBeFinite, mustBeNonsparse, ...
            mustBeNonnegative} = 0
    end
    
    params = defaultAprilTagDetectorParameters();
    params.quadDecimate = options.DecimationFactor;
    params.quadSigma = options.GaussianSigma;
end

%-------------------------------------------------------------------------------
function params = defaultAprilTagDetectorParameters()
    % Hyperparameter values for april tag detector. 
   
    % Gaussian blur standard deviation for quad detection in pixels. Helps
    % with very noisy images (e.g. 0.8). A quad is an apriltag
    % candidate after the segmentation step during the detection.
    params.quadSigma = 0.0;

    % Decimation factor that controls the scale of image downsampling.
    % Increasing this parameter will increase the speed of the detector at
    % the cost of the detection distance. Decoding the binary payload is
    % still done at full resolution.
    params.quadDecimate = 2.0;

    % Number of threads to be used by the detector for parallel processing.
    % Increasing this parameter can accelerate the detector's performance.
    % However, setting this parameter higher than what the CPU can manage
    % might lead to a failure in detecting anything.
    params.nthreads = 1;

    % Refines edge detection by snapping to strong gradients. Improves quad
    % estimate quality, especially with decimation.
    params.refineEdges = true;

    % Sharpening level for decoded images. Aids in decoding small tags and
    % can help in odd/low lighting conditions.
    params.decodeSharpening = 0.25;
    
    % Maximum hamming distance for decoding that limits all possible codes
    % within a specific bit error of valid codes in a tag family. Increasing
    % this parameter results in better predictions, but comes at a cost of 
    % prohibitively large amounts of memory consumption. Maximum allowed: 3.
    % The value set here is used only when autoBitCorrection is false.
    params.numBitsCorrected = 2;

    % Automatically adjusts numBitsCorrected to optimal values for different
    % tag families. Sets it to 1 for larger families like "tag16h5", "tag25h9"
    % "tag36h11" and "tagCircle21h7" and sets it to 2 for other smaller families.
    params.autoBitCorrection = true;
end

%--------------------------------------------------------------------------
function out = isSimMode()
    % check if simulation mode or codegen mode
    out = isempty(coder.target);
end