function points = detectORBFeatures(I, varargin)

% Copyright 2019-2023 The MathWorks, Inc.

%#codegen
params = parseInputs(I, varargin{:});
points = detectORB(I, params);
end

% -------------------------------------------------------------------------
% Process image and detect ORB features
% -------------------------------------------------------------------------
function points = detectORB(I, params)

img  = vision.internal.detector.cropImageIfRequested(I, params.ROI, params.UsingROI);
Iu8  = im2uint8(img);



% We hardcode the following parameters, to preserve the ORB algorithm
% as per the reference paper.
defaults = getDefaultParameters(size(Iu8));

EdgeThreshold = defaults.EdgeThreshold;
FirstLevel    = defaults.FirstLevel;
SamplingPairs = defaults.SamplingPairs;
PatchSize     = defaults.PatchSize;
FastThreshold = defaults.FastThreshold;
ScoreType     = defaults.ScoreType;
NumFeatures   = defaults.NumFeatures;


ScaleFactor = single(params.ScaleFactor);
NumLevels = params.NumLevels;


if isSimMode()
    
    rawPts = ocvDetectORB(Iu8, NumFeatures, ScaleFactor,...
        NumLevels, EdgeThreshold, FirstLevel,...
        SamplingPairs, ScoreType, PatchSize, FastThreshold);
    
else
    rawPts = ...
        vision.internal.buildable.detectORBBuildable.detectORB_uint8(...
        Iu8, NumFeatures, ScaleFactor,...
        NumLevels, EdgeThreshold, FirstLevel,...
        SamplingPairs, ScoreType, PatchSize, FastThreshold);
end

% apply offset for ROI if used
rawPts.Location = vision.internal.detector.addOffsetForROI( ...
    rawPts.Location, params.ROI, params.UsingROI);

% construct ORBPoints object based on the raw structure output
points = ORBPoints(rawPts.Location, ...
    'ScaleFactor', ScaleFactor, ...
    'NumLevels',   NumLevels, ...
    'Scale',       rawPts.Scale / single(PatchSize), ...
    'Metric',      rawPts.Metric, ...
    'Orientation', rawPts.Orientation, ...
    'PatchSize',   PatchSize);
end

% -------------------------------------------------------------------------
% Limit number of levels based on image size and scale factor.
% -------------------------------------------------------------------------
function NumLevels = adjustNumLevels(sz, n, sf, p, defaultNumLevelsFlag)
coder.internal.prefer_const(sz);
coder.internal.prefer_const(n);
coder.internal.prefer_const(sf);
coder.internal.prefer_const(p);

% To make sure last level has 31x31 pixel size.
MaxNumLevels = uint8(floor((log(min(sz))-log(double(p)*2+1))/log(double(sf)))+1);
coder.internal.prefer_const(MaxNumLevels);

if n > MaxNumLevels
    NumLevels = MaxNumLevels;
    if defaultNumLevelsFlag
        issueWarningDefault('vision:detectORBFeatures:maxNumLevelsDefault', sf, sz, MaxNumLevels);
    else
        issueWarning('vision:detectORBFeatures:maxNumLevels', sf, sz, MaxNumLevels, n);
    end
else
    NumLevels = uint8(n);
end
coder.internal.prefer_const(NumLevels);
end

% -------------------------------------------------------------------------
% Issuing warning for MaxNumLevels
% -------------------------------------------------------------------------
function issueWarning(id, sf, sz, MaxNumLevels, n)
coder.extrinsic('warning', 'message');
warning(message(id, sprintf('%4.3f',sf), sz(1), sz(2), MaxNumLevels, n));
end

function issueWarningDefault(id, sf, sz, MaxNumLevels)
coder.extrinsic('warning', 'message');
warning(message(id, sprintf('%4.3f',sf), sz(1), sz(2), MaxNumLevels));
end

% -------------------------------------------------------------------------
% Default parameter values
% -------------------------------------------------------------------------
function defaults = getDefaultParameters(imgSize)

defaults = struct( 'NumLevels', uint8(8), ...
    'ScaleFactor', single(1.2), ...
    'ROI', int32([1 1 imgSize([2 1])]), ...
    'EdgeThreshold',  int32(31), ...
    'FirstLevel',     uint8(0), ...
    'SamplingPairs',  uint8(2), ...
    'PatchSize',      int32(31), ...
    'FastThreshold',  uint8(20), ...
    'ScoreType',      uint8(0), ...
    'NumFeatures',    int32(imgSize(1)*imgSize(2)) ...
    );
end

function flag = isSimMode()
flag = isempty(coder.target);
end

% -------------------------------------------------------------------------
% Parse inputs
% -------------------------------------------------------------------------
function params = parseInputs(I,varargin)

defaults = getDefaultParameters(size(I));

% parse in sim mode with inputParser
if isSimMode()
    
    parser = inputParser;
    addParameter(parser, 'NumLevels', defaults.NumLevels);
    addParameter(parser, 'ScaleFactor', defaults.ScaleFactor);
    addParameter(parser, 'ROI', defaults.ROI);
    
    parse(parser,varargin{:});
    userInput = parser.Results;
    userInput.UsingROI = isempty(...
        regexp([parser.UsingDefaults{:} ''], 'ROI', 'once'));
    
    % parse in code gen mode with eml_* parameter parsing functions
else
    %'pvPairs' is storing the initialized indices of each of
    % the valid parameter names. The value of each field only
    % needs to be a scalar uint32 so 'pvPairs' satisfies the
    % requirement to be the first argument in
    % eml_parse_parameter_inputs.
    pvPairs = struct('NumLevels', uint32(0),...
        'ScaleFactor', uint32(0),...
        'ROI', uint32(0));
    
    properties =  struct( ...
        'CaseSensitivity', false, ...
        'StructExpand',    true, ...
        'PartialMatching', false);
    
    optarg = eml_parse_parameter_inputs(pvPairs, properties, varargin{:});
    
    pnames = fieldnames(pvPairs);
    for idx = 1:numel(pnames) % read all values
        pname = pnames{idx};
        userInput.(pname) = eml_get_parameter_value(...
            optarg.(pname), defaults.(pname), varargin{:});
    end
    
    if optarg.ROI == uint32(0)
        userInput.UsingROI = false;
    else
        userInput.UsingROI = true;
    end
end

% Validate user input
vision.internal.inputValidation.validateImage(I, 'I', 'grayscale');

if min(size(I)) < 63
    coder.internal.error('vision:detectORBFeatures:invalidImageSize');
end

if userInput.UsingROI
    vision.internal.detector.ORBcheckROI(userInput.ROI, size(I));
end

checkScaleFactor(userInput.ScaleFactor);
checkNumLevels(userInput.NumLevels);

defaultNumLevelsFlag = ~any(strcmp( {varargin{:}},'NumLevels'));

NumLevels = adjustNumLevels(size(I), userInput.NumLevels, ...
    userInput.ScaleFactor, defaults.PatchSize, defaultNumLevelsFlag);
params = setParams(userInput, NumLevels);
end

% -------------------------------------------------------------------------
% Set parameters based on user input
% -------------------------------------------------------------------------
function params = setParams(userInput, NumLevels)

params.ScaleFactor    = single(userInput.ScaleFactor);
params.NumLevels      = NumLevels;
params.UsingROI       = logical(userInput.UsingROI);
params.ROI            = userInput.ROI;
end

% -------------------------------------------------------------------------
% Check Scale Factor
% -------------------------------------------------------------------------
function checkScaleFactor(n)

vision.internal.errorIfNotFixedSize(n, 'ScaleFactor');
validateattributes(n,{'numeric'}, ...
    {'real', 'nonsparse', 'scalar', 'finite', '>', 1}, ...
    mfilename, 'ScaleFactor');
end

% -------------------------------------------------------------------------
% Check number of Pyramidal Levels
% -------------------------------------------------------------------------
function checkNumLevels(n)

vision.internal.errorIfNotFixedSize(n,'NumLevels');
validateattributes(n,{'numeric'}, ...
    {'integer', 'real', 'nonsparse', 'scalar', 'finite', '>=', 1}, ...
     mfilename, 'NumLevels');
end
