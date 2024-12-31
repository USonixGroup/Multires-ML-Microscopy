function [features, valid_points] = extractFeatures(I, points, varargin)

% Copyright 2010-2024 The MathWorks, Inc.

%#codegen
%#ok<*EMCA>

% Parse and check inputs
[blockSize, FeatureSize, descriptor,upright] ...
    = parseInputs(I, points, varargin{:});

% Extract features from image I
if strcmpi(descriptor, 'SIFT')
    
    [features, valid_points] = extractSIFTFeatures(I, points);
    
elseif strcmpi(descriptor, 'SURF')
    
    [features, valid_points] = extractSURFFeatures(I, points, FeatureSize, upright);
    
elseif strcmpi(descriptor, 'FREAK')
    
    [features, valid_points] = extractFreakFeatures(I, points, upright);
    
elseif strcmpi(descriptor, 'BRISK')
    
    [features, valid_points] = extractBRISKFeatures(I, points, upright);
    
elseif strcmpi(descriptor, 'KAZE')
    
    [features, valid_points] = extractKAZEFeatures(I, points, FeatureSize, upright);
    
elseif strcmpi(descriptor, 'ORB')
    
    [features, valid_points] = extractORBFeatures(I, points);
    
else % block
    
    [features, valid_points] = extractBlockFeatures(I, points, ...
        blockSize);
end

%==========================================================================
% Parse and check inputs
%==========================================================================
function [blockSize, FeatureSize, descriptor, upright] = ...
    parseInputs(I, points, varargin)

if isSimMode()
    [params,method] = parseInputsSimulation(varargin{:});
else
    [params,method] = parseInputsCodegen(varargin{:});
end

% Validate user input
checkImage(I);
method = checkMethod(method);
checkBlockSize(params.BlockSize);
checkFeatureSize(params.FeatureSize);
vision.internal.inputValidation.validateLogical(params.Upright,'Upright');

issueWarningIf(params.wasUprightSpecified && strcmpi(method,'block'),...
    'vision:extractFeatures:uprightInvalidForBlock');

% Cast user input to required types.
upright  = logical(params.Upright);
FeatureSize = double(params.FeatureSize);

% clip block size (necessary for codegen to avoid segV for blockSize = inf)
maxBlockSize = double(intmax('uint32'));
blockSizeIn = params.BlockSize;
if blockSizeIn > maxBlockSize
    blockSize = uint32(maxBlockSize);
else
    blockSize = uint32(blockSizeIn);
end

% Map 'Auto' into an actual descriptor choice.
descriptor = methodToDescriptor(method, points);

if strcmpi(descriptor, 'ORB')
    
    issueWarningIf(params.wasUprightSpecified, 'vision:extractFeatures:noUprightForORB');
    issueWarningIf(params.wasFeatureSizeSpecified , 'vision:extractFeatures:featureSizeForORBIsConstant');
    issueWarningIf(params.wasBlockSizeSpecified, 'vision:extractFeatures:blockSizeForORBIsInvalid');
    
end

% Check points
isValidPointObject = vision.internal.inputValidation.isValidPointObj(points);

if ~isValidPointObject
    checkPoints(points);
else
    vision.internal.inputValidation.checkPoints(points,mfilename,'POINTS');
end

%==========================================================================
% Maps 'Auto' to the descriptor choice based on the input POINTS class,
%   i.e. detector used to extract the local features
%==========================================================================
function descriptor = methodToDescriptor(method, points)

if strcmpi(method,'Auto')
    switch class(points)
        case {'SIFTPoints', 'vision.internal.SIFTPoints_cg',}
            descriptor = 'SIFT';
        case {'SURFPoints', 'vision.internal.SURFPoints_cg',}
            descriptor = 'SURF';
        case {'MSERRegions', 'vision.internal.MSERRegions_cg'}
            descriptor = 'SURF';
        case {'cornerPoints', 'vision.internal.cornerPoints_cg'}
            descriptor = 'FREAK';
        case {'BRISKPoints','vision.internal.BRISKPoints_cg'}
            descriptor = 'FREAK';
        case {'KAZEPoints','vision.internal.KAZEPoints_cg'}
            descriptor = 'KAZE';
        case {'ORBPoints','vision.internal.ORBPoints_cg'}
            descriptor = 'ORB';
        otherwise % array of X, Y coordinates
            descriptor = 'Block';
    end
else
    descriptor = method;
end

%==========================================================================
% Parameter defaults
%==========================================================================
function defaults = getParameterDefaults()

defaults.Method    = 'Auto';
defaults.BlockSize = 11;
defaults.SURFSize = 64; %<flag> backward compatibility
defaults.FeatureSize = 64;
defaults.Upright   = false;

%==========================================================================
% Parse input for simulation
%==========================================================================
function [params,method] = parseInputsSimulation(varargin)

defaults = getParameterDefaults();

% Setup parser
parser = inputParser;
parser.FunctionName  = 'extractFeatures';

parser.addParameter('Method',     defaults.Method);
parser.addParameter('BlockSize',  defaults.BlockSize);
parser.addParameter('SURFSize',   defaults.FeatureSize); %only for backward compatibility
parser.addParameter('FeatureSize',defaults.FeatureSize);
parser.addParameter('Upright',    defaults.Upright);

% Parse input
parser.parse(varargin{:});

% Assign outputs
r = parser.Results;

method = r.Method;
params.BlockSize = r.BlockSize;
params.Upright   = r.Upright;
params.FeatureSize = r.FeatureSize;

wasUprightSpecified = ~any(strcmpi('Upright',parser.UsingDefaults));
params.wasUprightSpecified = wasUprightSpecified;
%<flag> backward compatibility gating for SURFSize
wasFeatureSizeSpecified = ~any(strcmpi('FeatureSize',parser.UsingDefaults));
params.wasFeatureSizeSpecified = wasFeatureSizeSpecified;
wasBlockSizeSpecified = ~any(strcmpi('BlockSize',parser.UsingDefaults));
params.wasBlockSizeSpecified = wasBlockSizeSpecified;

wasSURFSizeSpecified = ~any(strcmpi('SURFSize',parser.UsingDefaults));

if wasSURFSizeSpecified
    if ~wasFeatureSizeSpecified
        if ~strcmpi(method, 'KAZE')
            params.FeatureSize = r.SURFSize;
        else
            error(message('vision:extractFeatures:useFeatureSize'));
        end
    else
        error(message('vision:extractFeatures:useEitherSURFOrFeature'));
    end
end

%==========================================================================
% Parse input for codegen
%==========================================================================
function [params,method] = parseInputsCodegen(varargin)

% Setup parser
parms = struct( ...
    'BlockSize',   uint32(0), ...
    'Method',      uint32(0), ...
    'SURFSize',    uint32(0),...
    'FeatureSize', uint32(0),...
    'Upright',     uint32(0));

popt = struct( ...
    'CaseSensitivity', false, ...
    'StructExpand',    true, ...
    'PartialMatching', false);

defaults = getParameterDefaults();

optarg = eml_parse_parameter_inputs(parms, popt, varargin{:});

blockSize = eml_get_parameter_value(optarg.BlockSize, defaults.BlockSize, varargin{:});

method = coder.internal.const(eml_tolower(eml_get_parameter_value(optarg.Method, ...
    defaults.Method, varargin{:})));


SURFSize = eml_get_parameter_value(optarg.SURFSize, defaults.SURFSize, varargin{:});

FeatureSize = eml_get_parameter_value(optarg.FeatureSize, defaults.FeatureSize, varargin{:});

upright = eml_get_parameter_value(optarg.Upright, defaults.Upright, varargin{:});

% method must remain out of param struct to remain const. Otherwise the
% output feature type cannot change from single to binaryFeatures. Pack the
% others into the param struct.

params.BlockSize = blockSize;
params.FeatureSize = FeatureSize;
params.Upright   = upright;

wasUprightSpecified = logical(optarg.Upright);
params.wasUprightSpecified = wasUprightSpecified;

% Backward compatibility gating for SURFSize
wasSURFSizeSpecified = false;
wasFeatureSizeSpecified = false;
for vi = 1:2:numel(varargin)
    varname = varargin{vi};
    if strcmpi(varname, 'SURFSize')
        wasSURFSizeSpecified = true;
    end
    if strcmpi(varname, 'FeatureSize')
        wasFeatureSizeSpecified = true;
    end
end
if wasSURFSizeSpecified
    if ~wasFeatureSizeSpecified
        if ~strcmpi(method, 'KAZE')
            params.FeatureSize = SURFSize;
        end
    end
end
wasFeatureSizeSpecified = logical(optarg.FeatureSize);
params.wasFeatureSizeSpecified = wasFeatureSizeSpecified;

wasBlockSizeSpecified = logical(optarg.BlockSize);
params.wasBlockSizeSpecified = wasBlockSizeSpecified;

%==========================================================================
% Parse SIFT inputs
%==========================================================================
function [Iu8, ptsStruct] = parseSIFTInputs(I, points)

Iu8 = im2uint8(I);

switch class(points)
    case {'SIFTPoints', 'vision.internal.SIFTPoints_cg'}
        ptsObj = points;
    case {'KAZEPoints', 'vision.internal.KAZEPoints_cg', ...
            'SURFPoints', 'vision.internal.SURFPoints_cg'}
        scale    = points.Scale .*(4/3);  % map KAZE/SURF to SIFT scale
        ptsObj = SIFTPoints(points.Location, 'Scale', scale);
        ptsObj = computeSIFTOctavefromSIFTScale(Iu8, ptsObj);
    case {'BRISKPoints', 'vision.internal.BRISKPoints_cg'}
        scale    = points.Scale .*(2/9);  % map BRISK to SIFT scale
        ptsObj = SIFTPoints(points.Location, 'Scale', scale);
        ptsObj = computeSIFTOctavefromSIFTScale(Iu8, ptsObj);
    case {'MSERRegions', 'vision.internal.MSERRegions_cg'}
        scale    = computeMSERScale(points, 0.8);
        ptsObj   = SIFTPoints(points.Location,'Scale',scale);
        ptsObj = computeSIFTOctavefromSIFTScale(Iu8, ptsObj);
    otherwise  % convert raw [X,Y] coordinates to SIFTPoints
        ptsObj = SIFTPoints(points);
end

% convert SIFTPoints object back to structure required by
% the ocvExtractSIFT built-in function
if isSimMode()
    ptsStruct.Location      = ptsObj.Location;
    ptsStruct.Metric        = ptsObj.Metric;
    ptsStruct.Octave        = ptsObj.Octave;
    ptsStruct.Layer         = ptsObj.Layer;
    ptsStruct.Orientation   = ptsObj.Orientation;
    ptsStruct.Scale         = ptsObj.Scale;
else
    coder.varsize('valLocation',        [inf, 2]);
    coder.varsize('valScale',           [inf, 1]);
    coder.varsize('valMetric',          [inf, 1]);
    coder.varsize('valOrientation',     [inf, 1]);
    coder.varsize('valOctave',          [inf, 1]);
    coder.varsize('valLayer',           [inf, 1]);
    
    out_numel          = size(ptsObj.Location,1);
    dtClass            = class(ptsObj.Location);
    valLocation        = coder.nullcopy(zeros(out_numel,2,dtClass));
    valScale           = coder.nullcopy(zeros(out_numel,1,dtClass));
    valMetric          = coder.nullcopy(zeros(out_numel,1,dtClass));
    valOrientation     = coder.nullcopy(zeros(out_numel,1,dtClass));
    valOctave          = coder.nullcopy(zeros(out_numel,1,'int32'));
    valLayer           = coder.nullcopy(zeros(out_numel,1,'int32'));
    
    valLocation(1:out_numel,:)        = ptsObj.Location;
    valScale(1:out_numel,:)           = ptsObj.Scale;
    valMetric(1:out_numel,:)          = ptsObj.Metric;
    valOrientation(1:out_numel,:)     = ptsObj.Orientation;
    valOctave(1:out_numel,:)          = ptsObj.Octave;
    valLayer(1:out_numel,:)           = ptsObj.Layer;
    
    ptsStruct.Location         = valLocation;
    ptsStruct.Scale            = valScale;
    ptsStruct.Metric           = valMetric;
    ptsStruct.Orientation      = valOrientation;
    ptsStruct.Octave           = valOctave;
    ptsStruct.Layer            = valLayer;
end

%==========================================================================
% Parse SURF inputs
%==========================================================================
function [Iu8, ptsStruct] = parseSURFInputs(I, points)

Iu8 = im2uint8(I);

switch class(points)
    case {'MSERRegions', 'vision.internal.MSERRegions_cg'}
        location = points.Location;
        scale    = computeMSERScale(points, 1.6);
        ptsObj   = SURFPoints(location,'Scale',scale);
    case {'cornerPoints', 'vision.internal.cornerPoints_cg'}
        ptsObj = SURFPoints(points.Location, 'Metric', points.Metric);
    case {'KAZEPoints', 'vision.internal.KAZEPoints_cg'}
        ptsObj = SURFPoints(points.Location, 'Scale', points.Scale, ...
            'Metric', points.Metric);
    case {'SIFTPoints', 'vision.internal.SIFTPoints_cg'}
        location = points.Location;
        scale    = points.Scale .* 0.75;  % map SIFT to SURF scale
        scale(scale < 1.6) = 1.6;         % saturate to min SURF scale
        ptsObj   = SURFPoints(location,'Scale',scale);
    case {'SURFPoints', 'vision.internal.SURFPoints_cg'}
        ptsObj = points;
    case {'BRISKPoints', 'vision.internal.BRISKPoints_cg'}
        location = points.Location;
        scale    = points.Scale ./ 6;  % map BRISK to SURF scale
        scale(scale < 1.6) = 1.6;      % saturate to min SURF scale
        ptsObj   = SURFPoints(location,'Scale',scale);
    otherwise  % convert raw [X,Y] coordinates to SURFPoints
        ptsObj = SURFPoints(points);
end

% convert SURFPoints object back to structure required by
% the ocvExtractSurf built-in function
if isSimMode()
    ptsStruct.Location         = ptsObj.Location;
    ptsStruct.Scale            = ptsObj.Scale;
    ptsStruct.Metric           = ptsObj.Metric;
    ptsStruct.SignOfLaplacian  = ptsObj.SignOfLaplacian;
else
    coder.varsize('valLocation',        [inf, 2]);
    coder.varsize('valScale',           [inf, 1]);
    coder.varsize('valMetric',          [inf, 1]);
    coder.varsize('valSignOfLaplacian', [inf, 1]);
    
    out_numel = size(ptsObj.Location,1);
    dtClass =  class(ptsObj.Location);
    valLocation        = coder.nullcopy(zeros(out_numel,2,dtClass));
    valScale           = coder.nullcopy(zeros(out_numel,1,dtClass));
    valMetric          = coder.nullcopy(zeros(out_numel,1,dtClass));
    valSignOfLaplacian = coder.nullcopy(zeros(out_numel,1,'int8'));
    
    valLocation(1:out_numel,:)        = ptsObj.Location;
    valScale(1:out_numel,:)           = ptsObj.Scale;
    valMetric(1:out_numel,:)          = ptsObj.Metric;
    valSignOfLaplacian(1:out_numel,:) = ptsObj.SignOfLaplacian;
    
    ptsStruct.Location         = valLocation;
    ptsStruct.Scale            = valScale;
    ptsStruct.Metric           = valMetric;
    ptsStruct.SignOfLaplacian  = valSignOfLaplacian;
end

%==========================================================================
% Parse KAZE inputs
%==========================================================================
function [Iu8, ptsStruct, threshold, ...
    noctaves, nscalelevels, diffusion] = parseKAZEInputs(I, points)

Iu8 = im2uint8(I);
switch class(points)
    case {'MSERRegions', 'vision.internal.MSERRegions_cg'}
        location = points.Location;
        scale    = computeMSERScale(points, 1.6);
        ptsObj   = KAZEPoints(location,'Scale',scale);
    case {'cornerPoints', 'vision.internal.cornerPoints_cg'}
        ptsObj = KAZEPoints(points.Location, 'Metric', points.Metric);
    case {'KAZEPoints', 'vision.internal.KAZEPoints_cg'}
        ptsObj = points;
    case {'SIFTPoints', 'vision.internal.SIFTPoints_cg'}
        location = points.Location;
        scale    = points.Scale .* 0.75;  % map SIFT to SURF/KAZE scale
        scale(scale < 1.6) = 1.6;         % saturate to min SURF/KAZE scale
        ptsObj   = KAZEPoints(location,'Scale',scale);
    case {'SURFPoints', 'vision.internal.SURFPoints_cg'}
        ptsObj = KAZEPoints(points.Location, 'Scale', points.Scale, ...
            'Metric', points.Metric);
    case {'BRISKPoints', 'vision.internal.BRISKPoints_cg'}
        location = points.Location;
        scale    = points.Scale ./ 6;  % map BRISK to SURF scale
        scale(scale < 1.6) = 1.6;      % saturate to min SURF scale
        ptsObj   = KAZEPoints(location,'Scale',scale);
    otherwise  % convert raw [X,Y] coordinates to SURFPoints
        ptsObj = KAZEPoints(points);
end

%% populate the scale space related attributes
%if ~strcmpi(class(points),'KAZEPoints')
%    ptsObj = populateInternalKAZEPointsAttributes(ptsObj);
%end

% convert KAZEPoints object back to structure required by
% the ocvExtractKAZE built-in function.
% NOTE: the conversion of Scale from radius to diameter is not done at this
% point, but in a later step before calling the ocvExtractKAZE built-in
% function.
if isSimMode()
    ptsStruct.Location         = ptsObj.Location;
    ptsStruct.Scale            = ptsObj.Scale;
    ptsStruct.Metric           = ptsObj.Metric;
    ptsStruct.Orientation      = ptsObj.Orientation;
    ptsStruct.Misc             = ptsObj.getLayerID();
else
    coder.varsize('valLocation',        [inf, 2]);
    coder.varsize('valScale',           [inf, 1]);
    coder.varsize('valMetric',          [inf, 1]);
    coder.varsize('valOrientation',     [inf, 1]);
    coder.varsize('valMisc',            [inf, 1]);
    
    out_numel = size(ptsObj.Location,1);
    dtLoc =  class(ptsObj.Location);
    dtScl =  class(ptsObj.Scale);
    dtMet =  class(ptsObj.Metric);
    dtOri =  class(ptsObj.Orientation);
    valLocation        = coder.nullcopy(zeros(out_numel,2,dtLoc));
    valScale           = coder.nullcopy(zeros(out_numel,1,dtScl));
    valMetric          = coder.nullcopy(zeros(out_numel,1,dtMet));
    valOrientation     = coder.nullcopy(zeros(out_numel,1,dtOri));
    valMisc            = coder.nullcopy(zeros(out_numel,1,'uint8'));
    
    valLocation(1:out_numel,:)        = ptsObj.Location;
    valScale(1:out_numel,:)           = ptsObj.Scale;
    valMetric(1:out_numel,:)          = ptsObj.Metric;
    valOrientation(1:out_numel,:)     = ptsObj.Orientation;
    valMisc(1:out_numel,:)            = ptsObj.getLayerID();
    
    ptsStruct.Location         = valLocation;
    ptsStruct.Scale            = valScale;
    ptsStruct.Metric           = valMetric;
    ptsStruct.Orientation      = valOrientation;
    ptsStruct.Misc             = valMisc;
end
noctaves                   = ptsObj.getNumOctaves();
nscalelevels               = ptsObj.getNumScaleLevels();
diffusion                  = ptsObj.getDiffusion();
threshold                  = single(0); % Threshold is ignored.

%==========================================================================
%function outPts = populateInternalKAZEPointsAttributes(inPts)
%outPts = inPts;
%outPts = setLayerID(outPts, ones(length(inPts), 1));
%outPts = setNumOctaves(outPts, 1);
%outPts = setNumScaleLevels(outPts, 3);
%outPts = setDiffusion(outPts, 'region');

%==========================================================================

%==========================================================================
% Parse ORB inputs
%==========================================================================
function [Iu8, ptsStruct, ScaleFactor, NumLevels, PatchSize] = parseORBInputs(I, points)

Iu8 = im2uint8(I);
switch class(points)
    
    case {'ORBPoints', 'vision.internal.ORBPoints_cg'}
        ptsObj = points;
        
    otherwise
        coder.internal.error('vision:extractFeatures:invalidPoints');
end

NumLevels                      = ptsObj.getNumLevels();
ScaleFactor                    = ptsObj.getScaleFactor();
PatchSize                      = ptsObj.getPatchSize();
% convert ORBPoints object back to structure required by
% the ocvExtractORB built-in function.

if isSimMode()
    ptsStruct.Location         = ptsObj.Location;
    ptsStruct.Scale            = ptsObj.Scale * single(PatchSize);
    ptsStruct.Metric           = ptsObj.Metric;
    ptsStruct.Orientation      = ptsObj.Orientation * (180/pi);
    
else
    coder.varsize('valLocation',        [inf, 2]);
    coder.varsize('valScale',           [inf, 1]);
    coder.varsize('valMetric',          [inf, 1]);
    coder.varsize('valOrientation',     [inf, 1]);
    
    
    out_numel = size(ptsObj.Location,1);
    dtLoc =  class(ptsObj.Location);
    dtScl =  class(ptsObj.Scale);
    dtMet =  class(ptsObj.Metric);
    dtOri =  class(ptsObj.Orientation);
    valLocation        = coder.nullcopy(zeros(out_numel,2,dtLoc));
    valScale           = coder.nullcopy(zeros(out_numel,1,dtScl));
    valMetric          = coder.nullcopy(zeros(out_numel,1,dtMet));
    valOrientation     = coder.nullcopy(zeros(out_numel,1,dtOri));
    
    valLocation(1:out_numel,:)        = ptsObj.Location;
    valScale(1:out_numel,:)           = ptsObj.Scale * single(PatchSize);
    valMetric(1:out_numel,:)          = ptsObj.Metric;
    valOrientation(1:out_numel,:)     = ptsObj.Orientation* (180/pi);
    
    ptsStruct.Location         = valLocation;
    ptsStruct.Scale            = valScale;
    ptsStruct.Metric           = valMetric;
    ptsStruct.Orientation      = valOrientation;
    
end
%==========================================================================
function checkImage(I)
vision.internal.inputValidation.validateImage(I, 'I', 'grayscale');

%==========================================================================
function checkPoints(points)

sz = [NaN 2];

validateattributes(points,{'int16', 'uint16', 'int32', 'uint32', ...
    'single', 'double'}, {'2d', 'nonsparse', 'real', 'size', sz},...
    mfilename, 'POINTS', 2);
% Use standard checkPoints to guard against gpuArrays
if isa(points, 'gpuArray')
    vision.internal.inputValidation.checkPoints(points, mfilename, 'POINTS');
end

%==========================================================================
function str = checkMethod(method)

str = validatestring(method,{'Block','SIFT','SURF','KAZE','FREAK','BRISK','ORB','Auto'},...
    mfilename,'Method');

%==========================================================================
function checkBlockSize(blockSize)

validateattributes(blockSize,{'numeric'}, {'nonempty', ...
    'finite', 'nonsparse', 'real', 'positive', 'integer', 'odd', ...
    'scalar'}, mfilename, 'BlockSize');

%==========================================================================
function checkFeatureSize(FeatureSize)
validateattributes(FeatureSize,{'numeric'}, {'scalar'}, 'extractFeatures',...
    'FEATURE_SIZE');

%<flag> message catalog?
coder.internal.errorIf(FeatureSize ~= 64 && FeatureSize ~= 128, ...
    'vision:extractFeatures:invalidFeatureSize');

%==========================================================================
% Extract block features
%==========================================================================
function [features, valid_points] = extractBlockFeatures(I, points, ...
    blockSize)

if isCornerPointObj(points) || isBRISKPointsObj(points)
    % for cornerPoints and BRISKPoints, output points is same as input
    % points.
    [features, valid_indices] = ...
        extractBlockAlg(I, points.Location, blockSize);
    
    valid_points = extractValidPoints(points, valid_indices);
else
    % For other input, valid_points is the location of points
    if isSIFTPointObj(points) || isSURFPointObj(points) || ...
            isMSERRegionObj(points) || isKAZEPointObj(points)
        pointsTmp = points.Location;
    else
        % X/Y points
        pointsTmp = points;
    end
    
    [features, valid_indices] = extractBlockAlg(I, pointsTmp, blockSize);
    
    valid_points = extractValidPoints(pointsTmp, valid_indices);
end

function [features, valid_indices] = extractBlockAlg(I, points, blockSize)
% Define casting constants
intClass = 'int32';
uintClass = 'uint32';

% Define length of feature vector
lengthFV = cast(blockSize*blockSize, uintClass);

nPoints = size(points,1);
if (islogical(I))
    features = false(nPoints, lengthFV);
else
    features = zeros(nPoints, lengthFV, 'like', I);
end
valid_indices = zeros(nPoints, 1);

%--------------------------------------------------------
% Define working variables needed for feature extraction
%--------------------------------------------------------
% Determine image size
nRows = cast(size(I, 1), intClass);
nCols = cast(size(I, 2), intClass);

% Compute half length of blockSize-by-blockSize neighborhood in units of
% integer pixels
halfSize = cast( (blockSize-mod(blockSize, 2)) / 2, intClass);

%------------------
% Extract features
%------------------
nValidPoints = cast(0, uintClass);
% Iterate over the set of input interest points, extracting features when
% the blockSize-by-blockSize neighborhood centered at the interest point is
% fully contained within the image boundary.
for k = 1:nPoints
    % Convert current interest point coordinates to integer pixels (Note:
    % geometric origin is at (0.5, 0.5)).
    [c, r] = castAndRound(points, k, intClass);
    % c = cast_ef(round_ef(points(k,1)), intClass);
    % r = cast_ef(round_ef(points(k,2)), intClass);
    
    % Check if interest point is within the image boundary
    if (c > halfSize && c <= (nCols - halfSize) && ...
            r > halfSize && r <= (nRows - halfSize))
        % Increment valid interest point count
        nValidPoints = nValidPoints + 1;
        % Reshape raw image data around the interest point into a feature
        % vector
        features(nValidPoints, :) = reshape(I(r-halfSize:r+halfSize, ...
            c-halfSize:c+halfSize), 1, lengthFV);
        % Save associated interest point location
        valid_indices(nValidPoints) = k;
    end
end

% Trim output data
features = features(1:nValidPoints, :);
valid_indices = valid_indices(1:nValidPoints,:);

%==========================================================================
% Extract SIFT features
%==========================================================================
function [features, valid_points] = extractSIFTFeatures(I, points)

[Iu8,ptsStruct] = parseSIFTInputs(I,points);

if isSimMode()
    [vPts, features] = ocvExtractSift(Iu8, ptsStruct);
else
    [features, vPts] = ...
        vision.internal.buildable.extractSIFTBuildable.extractSIFT_uint8(...
        Iu8, ptsStruct);
end

% modify the orientation so that it is measured counter-clockwise from
% horizontal the x-axis
vPts.Orientation = single(2*pi) - vPts.Orientation;

if isBRISKPointsObj(points)
    scale = vPts.Scale * 4.5;
    scale(scale <12) = 12;
    valid_points = BRISKPoints(vPts.Location, 'Metric', vPts.Metric,...
        'Scale', scale, 'Orientation', vPts.Orientation);
elseif isSURFPointObj(points)
    scale = vPts.Scale * 0.75;
    scale(scale <1.6) = 1.6;
    valid_points = SURFPoints(vPts.Location, 'Metric', vPts.Metric,...
        'Scale', scale, 'Orientation', vPts.Orientation);
elseif isKAZEPointObj(points)
    scale = vPts.Scale * 0.75;
    scale(scale <1.6) = 1.6;
    valid_points = KAZEPoints(vPts.Location, 'Metric', vPts.Metric,...
        'Scale', scale, 'Orientation', vPts.Orientation);
else
    valid_points = SIFTPoints(vPts.Location, vPts);
end

%==========================================================================
% Extract SURF features
%==========================================================================
function [features, valid_points] = extractSURFFeatures(I, points, SURFSize,upright)
[Iu8,ptsStruct] = parseSURFInputs(I,points);

params.extended = SURFSize == 128;
params.upright  = upright;

if isSimMode()
    [vPts, features] = ocvExtractSurf(Iu8, ptsStruct, params);
else
    if coder.isColumnMajor
        % column-major (matlab) to row-major (opencv)
        Iu8T = Iu8';
    else
        Iu8T = Iu8;
    end
    
    % SURFSize is the width of the feature
    [outLocation, outScale, outMetric, outSignOfLaplacian, outOrientation, features] = ...
        vision.internal.buildable.extractSurfBuildable.extractSurf_uint8(Iu8T, ...
        ptsStruct.Location, ptsStruct.Scale, ptsStruct.Metric, ...
        ptsStruct.SignOfLaplacian, SURFSize, params.extended, params.upright);
    
    vPts.Location        = outLocation;
    vPts.Scale           = outScale;
    vPts.Metric          = outMetric;
    vPts.SignOfLaplacian = outSignOfLaplacian;
    vPts.Orientation     = outOrientation;
end

% modify the orientation so that it is measured counter-clockwise from
% horizontal the x-axis
vPts.Orientation = single(2*pi) - vPts.Orientation;

if isCornerPointObj(points)
    % For cornerPoints input, valid_points is a cornerPoints object
    valid_points = cornerPoints(vPts.Location, 'Metric', vPts.Metric);
    
elseif isBRISKPointsObj(points)
    valid_points = BRISKPoints(vPts.Location, 'Metric', vPts.Metric,...
        'Scale', 6 * vPts.Scale, 'Orientation', vPts.Orientation);
elseif isSIFTPointObj(points)
    valid_points = SIFTPoints(vPts.Location, 'Metric', vPts.Metric,...
        'Scale', (4/3) * vPts.Scale, 'Orientation', vPts.Orientation);
    valid_points = computeSIFTOctavefromSIFTScale(Iu8, valid_points);
else
    % For other inputs, valid_points is a SURFPoints object
    valid_points = SURFPoints(vPts.Location, vPts);
end

%==========================================================================
% Extract KAZE features
%==========================================================================
function [features, valid_points] = extractKAZEFeatures(I, points, FeatureSize, upright_in)

% retrieve extraction parameters and prepare extraction inputs.
[Iu8, ptsStruct, threshold, noctaves, nscalelevels, diffusion] = parseKAZEInputs(I,points);

ptsStruct.Scale = ptsStruct.Scale*2; % convert radius to diameter

extended = FeatureSize == 128;
upright  = upright_in;

diffusionCode = ...
    vision.internal.detector.convertKAZEDiffusionToOCVCode(diffusion);

if isSimMode()
    [features, vPts] = ocvExtractKAZE(Iu8, ptsStruct, ...
        extended, upright, threshold, ...
        noctaves, nscalelevels, ...
        diffusionCode);
else
    [features, vPts] = ...
        vision.internal.buildable.extractKAZEBuildable.extractKAZE_uint8(...
        Iu8, ptsStruct, ...
        extended, upright, threshold, ...
        noctaves, nscalelevels, ...
        diffusionCode);
end

% modify the orientation so that it is measured counter-clockwise from
% horizontal the x-axis
vPts.Orientation = single(pi/2) - vPts.Orientation;
vPts.Scale = vPts.Scale/2; % convert from diameter to radius

if isCornerPointObj(points)
    % For cornerPoints input, valid_points is a cornerPoints object
    valid_points = cornerPoints(vPts.Location, 'Metric', vPts.Metric);
    
elseif isBRISKPointsObj(points)
    valid_points = BRISKPoints(vPts.Location, 'Metric', vPts.Metric,...
        'Scale', 6 * vPts.Scale, 'Orientation', vPts.Orientation);

elseif isSIFTPointObj(points)
    scale = (4/3) * vPts.Scale;
    valid_points = SIFTPoints(vPts.Location, 'Metric', vPts.Metric,...
        'Scale', scale, 'Orientation', vPts.Orientation);
    valid_points = computeSIFTOctavefromSIFTScale(Iu8, valid_points);
    
else
    % For other inputs, valid_points is a KAZEPoints object
    valid_points = KAZEPoints(vPts.Location, 'Diffusion', diffusion, ...
        'Scale', vPts.Scale, 'Orientation', vPts.Orientation, ...
        'Metric', vPts.Metric, 'NumOctaves', noctaves, ...
        'NumScaleLevels', nscalelevels, 'LayerID', vPts.Misc);
end


%==========================================================================
% Extract ORB features
%==========================================================================
function [features, valid_points] = extractORBFeatures(I, points)

% retrieve extraction parameters and prepare extraction inputs.
[Iu8, ptsStruct, ScaleFactor, NumLevels, PatchSize] = parseORBInputs(I, points);

% Hard coded values that are used in detection
EdgeThreshold  = int32(31);
FirstLevel     = uint8(0);
SamplingPairs  = uint8(2);
FastThreshold  = uint8(20);
ScoreTypeCode  = uint8(0);
NumFeatures    = int32(numel(I));

if isSimMode()
    [features, vPts] = ocvExtractORB(Iu8,ptsStruct, NumFeatures, ScaleFactor,...
        NumLevels, EdgeThreshold, FirstLevel,...
        SamplingPairs, ScoreTypeCode, PatchSize, FastThreshold);
else
    [features, vPts] = ...
        vision.internal.buildable.extractORBBuildable.extractORB_uint8(...
        Iu8,ptsStruct, NumFeatures, ScaleFactor,...
        NumLevels, EdgeThreshold,FirstLevel,...
        SamplingPairs, ScoreTypeCode, PatchSize, FastThreshold);
end
features = binaryFeatures(features);

valid_points = ORBPoints(vPts.Location,'Scale', vPts.Scale / single(PatchSize), 'Orientation', vPts.Orientation, ...
    'Metric', vPts.Metric, 'ScaleFactor', ScaleFactor, 'NumLevels', NumLevels);


%==========================================================================
% Extract freak features
%==========================================================================
function [features, valid_points] = extractFreakFeatures(I, points, upright)

Iu8 = im2uint8(I);

ptsStruct = pointsToFREAKPoints(points);

% configure parameters
params.nbOctave              = 4;
params.orientationNormalized = ~upright;
params.scaleNormalized       = true;
params.patternScale          = 7;

% extract
if isSimMode()
    [validPts, features] = ocvExtractFreak(Iu8, ptsStruct, params);
else
    if coder.isColumnMajor
        % column-major (matlab) to row-major (opencv)
        Iu8T = Iu8';
    else
        Iu8T = Iu8;
    end
    [outLocation, outScale, outMetric, outMisc, outOrientation, features] = ...
        vision.internal.buildable.extractFreakBuildable.extractFreak_uint8(Iu8T, ...
        ptsStruct.Location, ptsStruct.Scale, ptsStruct.Metric, ptsStruct.Misc, ...
        params.nbOctave, params.orientationNormalized, ...
        params.scaleNormalized, params.patternScale);
    validPts.Location        = outLocation;
    validPts.Scale           = outScale;
    validPts.Metric          = outMetric;
    validPts.Misc            = outMisc;
    validPts.Orientation     = outOrientation; % yes for FREAK too
end

% repackage features to the desired final format
features = binaryFeatures(features);

validPts.Orientation = correctOrientation(validPts.Orientation, upright);

valid_points = extractValidPoints(points, validPts.Misc);

if isSIFTPointObj(points) || isSURFPointObj(points) || isBRISKPointsObj(points) || isKAZEPointObj(points)
    % update with orientation estimated during extraction
    valid_points = setOrientation(valid_points, validPts.Orientation);
end

%==========================================================================
% Extract BRISK features
%==========================================================================
function [features, valid_points] = extractBRISKFeatures(I,points,upright)

Iu8 = im2uint8(I);

ptsStruct = pointsToBRISKPoints(points);

params.upright = upright;

if isSimMode()
    [features, validPts] = ocvExtractBRISK(Iu8, ptsStruct, params);
else
    [features, validPts] = ...
        vision.internal.buildable.extractBRISKBuildable.extractBRISKFeatures(Iu8, ptsStruct, params);
end

features = binaryFeatures(features);

validPts.Orientation = correctOrientation(validPts.Orientation, upright);

valid_points = extractValidPoints(points, validPts.Misc);

if isSIFTPointObj(points) || isSURFPointObj(points) || isBRISKPointsObj(points) || isKAZEPointObj(points)
    % update with orientation estimated during extraction
    valid_points = setOrientation(valid_points, validPts.Orientation);
end

%==========================================================================
% Correct the orientation returned from OpenCV such that it is measured
% counter-clockwise from horizontal the x-axis.
%
% For upright features, the convention is that they have orientations of
% pi/2. However, OpenCV's orientation values are zero when orientation is
% not estimated. As a result, the orientation is manually set to pi/2.
%==========================================================================
function orientation = correctOrientation(orientation, upright)
if upright
    % By convention, upright features have orientation of pi/2.
    orientation(:) = single(pi/2);
else
    orientation(:) = single(2*pi) - orientation;
end

%==========================================================================
% Compute the scale for MSER regions
%==========================================================================
function scale = computeMSERScale(points, minScale)
if isempty(points.Axes)
    scale = zeros(0,1,'single');
else
    majorAxes  = points.Axes(:,1);
    minorAxes  = points.Axes(:,2);
    % Make the scale proportional to the ellipse area.
    scale      = 1/8*sqrt(majorAxes.*minorAxes);
    scale((scale < minScale)) = single(minScale);
end

%==========================================================================
% Computes octave and layer values required by SIFT extractor
% Input1 : Image
% Input2 : SIFTPoints with valid scale values
% Output : SIFTPoints with valid octave and layer values
%
% Note: Octave is a collection of layers. So, an octave index is composed 
%  of first 8-bits (L.S.B.) which represent octave number in the pyramid 
%  and next 8-bits (left of L.S.B.) which represent layer index.  
%==========================================================================
function validSIFTPoints = computeSIFTOctavefromSIFTScale(Iu8, siftPoints)

% Default values from SIFT algorithm
sigma = 1.6;
nOctaveLayers = 3;
numOctaves = log(min(size(Iu8)))/log(2);
numOctaves = 0:(numOctaves-1); % 0-index based

scale = siftPoints.Scale;

% Formula computed from the SIFT algorithm
% 'layers' will be of size NxM where M=numOctaves
layers = round(nOctaveLayers*log(scale*(1./(sigma*2.^(numOctaves-1))))./log(2));

% Format the values in which the extractor expects, 
% -1 to compensate 1-based MATLAB indexing and additional -1 
% as the firstOctave has to be -1 in SIFT, which eventually is
% represented as 255
layers(layers<1) = -1;
layers(layers>nOctaveLayers) = -1;
[layers, octaves] = max(layers,[],2);

% Format the values in which the extractor expects
octaves=octaves-2;
octaves(octaves<0)=255;

% Form the index, octaveIndex = octaveNumber + (layer<<8)
octaveIndex = pow2(8).*layers + octaves;

% Remove points with invalid octave values
% Cannot use object array syntax because codegen does not support it
% g2413236
isValidPoints = octaveIndex >= 0;
validSIFTPoints = SIFTPoints(siftPoints.Location(isValidPoints,:), ...
                    'Scale', siftPoints.Scale(isValidPoints), ...
                    'Orientation', siftPoints.Orientation(isValidPoints), ...
                    'Octave', octaves(isValidPoints), ...
                    'Layer', layers(isValidPoints));

%==========================================================================
% Converts any point type to make it compatible with FREAK
%==========================================================================
function ptsStruct = pointsToFREAKPoints(points)

switch class(points)
    case {'MSERRegions','vision.internal.MSERRegions_cg'}
        ptsStruct.Location = points.Location;
        scale = computeMSERScale(points, 1.6);
        ptsStruct.Scale    = round(scale * 7.5);
        ptsStruct.Metric   = zeros(size(scale), 'single');
        ptsStruct.Orientation = zeros(size(scale), 'single');
        len = size(points.Location,1);
    case {'SIFTPoints','vision.internal.SIFTPoints_cg'}
        ptsStruct.Location = points.Location;
        ptsStruct.Scale    = round(points.Scale .* 6.75);
        ptsStruct.Metric   = points.Metric;
        ptsStruct.Orientation = points.Orientation;
        len = size(points.Location,1);
    case {'SURFPoints','vision.internal.SURFPoints_cg'}
        ptsStruct.Location = points.Location;
        ptsStruct.Scale    = round(points.Scale .* 7.5);
        ptsStruct.Metric   = points.Metric;
        ptsStruct.Orientation = points.Orientation;
        len = size(points.Location,1);
    case {'KAZEPoints', 'vision.internal.KAZEPoints_cg'}
        ptsStruct.Location = points.Location;
        ptsStruct.Scale    = round(points.Scale .* 7.5);
        ptsStruct.Metric   = points.Metric;
        ptsStruct.Orientation = points.Orientation;
        len = size(points.Location,1);
    case {'cornerPoints','vision.internal.cornerPoints_cg'}
        ptsStruct.Location = points.Location;
        % The value of 18 below corresponds to the FREAK pattern radius.
        ptsStruct.Scale    = ones(size(points.Metric), 'single').*18;
        ptsStruct.Metric   = points.Metric;
        ptsStruct.Orientation = zeros(size(points.Metric), 'single');
        len = size(points.Location,1);
    case {'BRISKPoints','vision.internal.BRISKPoints_cg'}
        ptsStruct.Location    = points.Location;
        % The value of 12 and 18 below corresponds to the BRISK and FREAK
        % pattern radius, respectively.
        ptsStruct.Scale       = points.Scale .* (18/12);
        ptsStruct.Metric      = points.Metric;
        ptsStruct.Orientation = points.Orientation;
        len = size(points.Location,1);
    otherwise
        ptsStruct.Location = single(points);
        % The value of 18 below corresponds to the FREAK pattern radius.
        ptsStruct.Scale    = ones(1, size(points,1), 'single').*18;
        ptsStruct.Metric   = zeros(1, size(points, 1), 'single');
        ptsStruct.Orientation = zeros(1, size(points,1), 'single');
        len = size(points,1);
end

% encode the point index in the Misc property; we'll use it to determine
% which points were removed during the extraction process
ptsStruct.Misc = int32(1):int32(len);

%==========================================================================
% Maps data from feature points to equivalent BRISK points.
%==========================================================================
function ptsStruct = pointsToBRISKPoints(points)

switch class(points)
    case {'MSERRegions','vision.internal.MSERRegions_cg'}
        ptsStruct.Location = points.Location;
        scale = computeMSERScale(points, 12);
        ptsStruct.Scale    = scale;
        ptsStruct.Metric   = zeros(size(scale), 'single');
        ptsStruct.Orientation = zeros(1, length(scale),'single');
        len = size(points.Location,1);
        
    case {'SIFTPoints','vision.internal.SIFTPoints_cg'}
        ptsStruct.Location = points.Location;
        ptsStruct.Metric   = points.Metric;
        ptsStruct.Scale = max(12, round(points.Scale .* 4.5)); % Saturate to 12
        ptsStruct.Orientation = points.Orientation;
        len = size(points.Location,1);
        
    case {'SURFPoints','vision.internal.SURFPoints_cg'}
        ptsStruct.Location = points.Location;
        ptsStruct.Metric   = points.Metric;
        
        % Set the BRISK scale to be 10*s which covers most of the
        % equivalent SURF extraction region.
        ptsStruct.Scale    = round(points.Scale .* 10);
        ptsStruct.Orientation = points.Orientation;
        len = size(points.Location,1);
        
    case {'KAZEPoints', 'vision.internal.KAZEPoints_cg'}
        ptsStruct.Location = points.Location;
        ptsStruct.Metric   = points.Metric;
        
        % Set the BRISK scale to be 10*s which covers most of the
        % equivalent KAZE extraction region.
        ptsStruct.Scale    = round(points.Scale .* 10);
        ptsStruct.Orientation = points.Orientation;
        len = size(points.Location,1);
        
        
    case {'cornerPoints','vision.internal.cornerPoints_cg'}
        % The value of 12 corresponds to the BRISK pattern size at
        % scale 0. Corner points are treated as single scale detections.
        ptsStruct.Location = points.Location;
        ptsStruct.Metric   = points.Metric;
        ptsStruct.Scale    = ones(size(points.Metric), 'single').* 12;
        ptsStruct.Orientation = zeros(1, length(points.Metric), 'single');
        len = size(points.Location,1);
        
    case {'BRISKPoints','vision.internal.BRISKPoints_cg'}
        ptsStruct.Location    = points.Location;
        ptsStruct.Metric      = points.Metric;
        ptsStruct.Scale       = points.Scale;
        ptsStruct.Orientation = points.Orientation;
        len = size(points.Location,1);
        
    otherwise
        ptsStruct.Location = single(points);
        ptsStruct.Metric   = zeros(1, size(points, 1), 'single');
        ptsStruct.Scale    = ones(1, size(points, 1), 'single').*12;
        ptsStruct.Orientation = zeros(1,size(points,1),'single');
        len = size(points,1);
end

% encode the point index in the Misc property; we'll use it to determine
% which points were removed during the extraction process
ptsStruct.Misc = int32(1):int32(len);

%==========================================================================
function [c, r] = castAndRound(points, k, intClass)

if ~isobject(points)
    c = cast(round(points(k,1)), intClass);
    r = cast(round(points(k,2)), intClass);
else
    c=0;
    r=0;
end

%==========================================================================
function validPoints = extractValidPoints(points, idx)
if isnumeric(points)
    validPoints = points(idx,:);
else
    if isSimMode()
        validPoints = points(idx);
    else
        validPoints = select(points, idx);
    end
end

%==========================================================================
function issueWarningIf(valueIsTrue, id)
coder.extrinsic('warning','message');
if valueIsTrue
    warning(message(id));
end

%==========================================================================
function flag = isCornerPointObj(points)

flag = isa(points, 'cornerPoints') || ...
    isa(points, 'vision.internal.cornerPoints_cg');

%==========================================================================
function flag = isSIFTPointObj(points)

flag = isa(points, 'SIFTPoints') || ...
    isa(points, 'vision.internal.SIFTPoints_cg');

%==========================================================================
function flag = isSURFPointObj(points)

flag = isa(points, 'SURFPoints') || ...
    isa(points, 'vision.internal.SURFPoints_cg');

%==========================================================================
function flag = isKAZEPointObj(points)

flag = isa(points, 'KAZEPoints') || ...
    isa(points, 'vision.internal.KAZEPoints_cg');

%==========================================================================
function flag = isMSERRegionObj(points)

flag = isa(points, 'MSERRegions') || ...
    isa(points, 'vision.internal.MSERRegions_cg');

%==========================================================================
function flag = isBRISKPointsObj(points)

flag = isa(points, 'BRISKPoints') || ...
    isa(points, 'vision.internal.BRISKPoints_cg');

%==========================================================================
function flag = isSimMode()

flag = isempty(coder.target);

