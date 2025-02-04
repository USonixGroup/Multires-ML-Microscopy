function points = detectBRISKFeatures(I, varargin)

% Copyright 2013-2024 The MathWorks, Inc.

%#codegen

params = parseInputs(I, varargin{:});

points = detectBRISK(I, params);

points = selectPoints(points, params.MinQuality);

% -------------------------------------------------------------------------
% Process image and detect BRISK features
% -------------------------------------------------------------------------
function points = detectBRISK(I, params)

img  = vision.internal.detector.cropImageIfRequested(I, params.ROI, params.UsingROI);

Iu8  = im2uint8(img);

numOctaves = adjustNumOctaves(size(Iu8),params.NumOctaves);

if isempty(coder.target)
    rawPts = ocvDetectBRISK(Iu8, params.Threshold, numOctaves);
    
else
    rawPts = vision.internal.buildable.detectBRISKBuildable.detectBRISK(...
        Iu8, params.Threshold, numOctaves);
end

rawPts.Location = vision.internal.detector.addOffsetForROI(rawPts.Location, params.ROI, params.UsingROI);

points = BRISKPoints(rawPts.Location, 'Metric', rawPts.Metric, ...
    'Scale', rawPts.Scale, 'Orientation', rawPts.Orientation);


% -------------------------------------------------------------------------
% Limit number of octaves based on image size.
% -------------------------------------------------------------------------
function numOctaves = adjustNumOctaves(sz, n)
coder.internal.prefer_const(sz);
coder.internal.prefer_const(n);

maxNumOctaves = uint8(floor(log2(min(sz))));
coder.internal.prefer_const(maxNumOctaves);

if n > maxNumOctaves
    numOctaves = maxNumOctaves;
else
    numOctaves = n;
end
coder.internal.prefer_const(numOctaves);

% -------------------------------------------------------------------------
% Select points based on minimum quality
% -------------------------------------------------------------------------
function selectedPoints = selectPoints(points, minQuality)
if isempty(points)
    selectedPoints = points;
else
    maxMetric = max(points.Metric);
    minMetric = minQuality * maxMetric;
    
    idx = points.Metric >= minMetric;
    if isempty(coder.target)
        selectedPoints = points(idx);
    else
        selectedPoints = points.select(idx);
    end
end

% -------------------------------------------------------------------------
% Default parameter values
% -------------------------------------------------------------------------
function defaults = getDefaultParameters(imgSize)

defaults = struct(...
    'MinContrast', single(0.2),...
    'NumOctaves' , uint8(4),...
    'MinQuality' , single(0.1), ...
    'ROI', int32([1 1 imgSize([2 1])]));

% -------------------------------------------------------------------------
% Parse inputs
% -------------------------------------------------------------------------
function params = parseInputs(I,varargin)

defaults = getDefaultParameters(size(I));

if isempty(coder.target)
    parser = inputParser;
    
    addParameter(parser, 'MinContrast', defaults.MinContrast);
    addParameter(parser, 'MinQuality',  defaults.MinQuality);
    addParameter(parser, 'NumOctaves',  defaults.NumOctaves);
    addParameter(parser, 'ROI',         defaults.ROI);
    
    parse(parser,varargin{:});
    
    userInput = parser.Results;
    
    userInput.UsingROI = isempty(regexp([parser.UsingDefaults{:} ''],...
        'ROI','once'));
else
    
    pvPairs = struct( ...
        'MinContrast', uint32(0), ...
        'NumOctaves',  uint32(0), ...
        'MinQuality',  uint32(0),...
        'ROI',         uint32(0));
    
    popt = struct( ...
        'CaseSensitivity', false, ...
        'StructExpand'   , true, ...
        'PartialMatching', false);
    
    
    optarg = eml_parse_parameter_inputs(pvPairs, popt, varargin{:});
    
    userInput.MinContrast  = eml_get_parameter_value(optarg.MinContrast, ...
        defaults.MinContrast, varargin{:});
    
    userInput.NumOctaves = eml_get_parameter_value(optarg.NumOctaves, ...
        defaults.NumOctaves, varargin{:});
    
    userInput.MinQuality = eml_get_parameter_value(optarg.MinQuality, ...
        defaults.MinQuality, varargin{:});
    
    if optarg.ROI==uint32(0)
        userInput.UsingROI = false;
    else
        userInput.UsingROI = true;
    end
    
    userInput.ROI = eml_get_parameter_value(optarg.ROI,defaults.ROI, varargin{:});
end

% Validate user input

vision.internal.inputValidation.validateImage(I, 'I', 'grayscale');

vision.internal.detector.checkMinQuality(userInput.MinQuality);

vision.internal.detector.checkMinContrast(userInput.MinContrast);

if userInput.UsingROI
    vision.internal.detector.checkROI(userInput.ROI,size(I));
end

checkNumOctaves(userInput.NumOctaves);

params = setParams(userInput);

% -------------------------------------------------------------------------
% Set parameters based on user input
% -------------------------------------------------------------------------
function params = setParams(userInput)
minContrast = single(userInput.MinContrast);

params.Threshold  = uint8(minContrast * single(255.0));
params.NumOctaves = uint8(userInput.NumOctaves);
params.MinQuality = single(userInput.MinQuality);
params.UsingROI   = logical(userInput.UsingROI);
params.ROI        = userInput.ROI;

% -------------------------------------------------------------------------
% Check number of octaves
% -------------------------------------------------------------------------
function checkNumOctaves(n)

vision.internal.errorIfNotFixedSize(n,'NumOctaves');

validateattributes(n,{'numeric'},...
    {'real', 'nonsparse', 'scalar', 'nonnan', 'finite', 'integer' ,...
    '>=', 0}, 'detectBRISKFeatures','NumOctaves');

