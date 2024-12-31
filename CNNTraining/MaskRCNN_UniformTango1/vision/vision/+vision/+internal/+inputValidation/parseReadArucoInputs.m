function [Iuint8, markerFamily, doEstimatePose, intrinsics, markerSize, params] = parseReadArucoInputs(I, varargin)

%   Copyright 2023-2024 The MathWorks, Inc.
%#codegen

coder.inline('always');
coder.internal.prefer_const(I,varargin{:});

% Validate input image.
Iuint8 = checkImage(I);

% Parse inputs for all syntaxes.
[markerFamily, doEstimatePose, intrinsics, markerSize, hasNVPs,...
    nvpStartIdx] = parseInputs(varargin{:});

% Validate marker family input.
markerFamily = validateMarkerFamily(markerFamily);

if doEstimatePose
    % Validate intrinsics input.
    validateIntrinsics(intrinsics);

    % Validate marker size input.
    validateMarkerSize(markerSize);
end

% Parse and validate name-value arguments.
if ~hasNVPs
    params = vision.internal.inputValidation.parseArucoDetectorParameters;
else
    params = vision.internal.inputValidation.parseArucoDetectorParameters(varargin{nvpStartIdx:end});
end

end

%--------------------------------------------------------------------------
function Iuint8 = checkImage(I)

    % Validate input image.
    vision.internal.inputValidation.validateImage(I);
    
    % Ensure image data is uint8
    if ~isa(I, 'uint8')
        Iuint8 = im2uint8(I);
    else
        Iuint8 = I;
    end
end

%--------------------------------------------------------------------------
%  Syntax 1: readArucoMarker(I)
%  Syntax 2: readArucoMarker(I, markerFamily)
%  Syntax 3: readArucoMarker(I, intrinsics, markerSize)
%  Syntax 4: readArucoMarker(I, markerFamily, intrinsics, markerSize)
%  Syntaxes 5-8: NVP + above syntaxes.
%--------------------------------------------------------------------------
function [markerFamily, doEstimatePose, intrinsics, markerSize, hasNVPs, nvpStartIdx] = parseInputs(varargin)
    
    coder.inline('always');
    coder.internal.prefer_const(varargin);
    numInputs = nargin + 1;

    if numInputs == 1 % readArucoMarker(I)
        markerFamily = 'all';
        doEstimatePose = false;
        hasNVPs = false;
        nvpStartIdx = 0;
    elseif numInputs == 2 % readArucoMarker(I, markerFamily)
        markerFamily = varargin{1};
        doEstimatePose = false;
        hasNVPs = false;
        nvpStartIdx = 0;
    elseif isa(varargin{1},'cameraIntrinsics') % readArucoMarker(I, intrinsics, markerSize, NVP)
        markerFamily = 'all';
        doEstimatePose = true;
        intrinsics = varargin{1};
        markerSize = varargin{2};
        if numInputs > 3
            hasNVPs = true;
            nvpStartIdx = 3;
        else
            hasNVPs = false;
            nvpStartIdx = 0;
        end
    elseif isa(varargin{2},'cameraIntrinsics') % readArucoMarker(I, markerFamily, intrinsics, markerSize, NVP)
        markerFamily = varargin{1};
        doEstimatePose = true;
        intrinsics = varargin{2};
        markerSize = varargin{3};
        if numInputs > 4
            hasNVPs = true;
            nvpStartIdx = 4;            
        else
            hasNVPs = false;
            nvpStartIdx = 0;
        end
    elseif mod(numInputs,2) == 1 % readArucoMarker(I, NVP)
        markerFamily = 'all';
        doEstimatePose = false;
        hasNVPs = true;
        nvpStartIdx = 1;         
    else % readArucoMarker(I, markerFamily, NVP)
        markerFamily = varargin{1};
        doEstimatePose = false;
        hasNVPs = true;
        nvpStartIdx = 2;        
    end
    
    if ~doEstimatePose
        intrinsics = [];
        markerSize = [];
    end
end
%--------------------------------------------------------------------------
function markerFamily = validateMarkerFamily(markerFamily)
    validateattributes(markerFamily, {'char', 'string', 'cell'}, {'nonempty', 'vector'}, 'readArucoMarker', 'markerFamily');
    
    supportedFamilies = vision.internal.supportedArucoMarkerFamilies();
    
    if strcmp(markerFamily, 'all')
        markerFamily = {'DICT_4X4_1000'; 'DICT_5X5_1000'; 'DICT_6X6_1000';
                        'DICT_7X7_1000'; 'DICT_ARUCO_ORIGINAL'};
    elseif ischar(markerFamily) || isStringScalar(markerFamily)
        markerFamily = validatestring(markerFamily, supportedFamilies, 'readArucoMarker', 'markerFamily');
        markerFamily = char(markerFamily);
    else
        if isSimMode
            isValidFamily = all(ismember(markerFamily, supportedFamilies));
        else
            if iscellstr(markerFamily)
                numValidFamilies = 0;
                numFamilies = size(markerFamily,2);
                for i = 1:numFamilies
                    numValidFamilies = numValidFamilies+any(strcmp(supportedFamilies,markerFamily{i}));
                end
                isValidFamily = (numValidFamilies == numFamilies);
            else
                isValidFamily = any(strcmp(supportedFamilies, markerFamily));
            end
        end        
        
        formatMsg = strjoin(supportedFamilies, ', ');
        coder.internal.errorIf(~isValidFamily,'vision:apriltag:unrecognizedStringChoice', formatMsg);
    
        markerFamily = cellstr(markerFamily);
    end
end

%--------------------------------------------------------------------------
function validateIntrinsics(intrinsics)
    
    validTypes = {'cameraIntrinsics'};
    validateattributes(intrinsics, validTypes, {'scalar'}, 'readArucoMarker', 'intrinsics');
end

%--------------------------------------------------------------------------
function validateMarkerSize(markerSize)
    validateattributes(markerSize, {'numeric'}, {'finite', 'real', 'nonsparse', ...
        'scalar', 'positive'}, 'readArucoMarker');
end

%--------------------------------------------------------------------------
% isSimMode - check if simulation mode or codegen mode
%--------------------------------------------------------------------------
function out = isSimMode()
    out = isempty(coder.target);
end