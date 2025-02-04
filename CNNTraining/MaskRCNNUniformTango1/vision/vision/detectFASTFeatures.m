function pts = detectFASTFeatures(I, varargin)

% Copyright 2012-2023 The MathWorks, Inc.

%#codegen
%#ok<*EMCA>

% Check the input image and convert it to the range of uint8.

params = parseInputs(I, varargin{:});

I_u8 = im2uint8(I);

[I_u8c, expandedROI] = vision.internal.detector.fast.cropImage(I_u8, params);

% Convert the minContrast property to the range of unit8.
minContrast = im2uint8(params.MinContrast);

% Find corner locations by using OpenCV.
if isSimMode()
    rawPts = ocvDetectFAST(I_u8c, minContrast);
else
    [rawPts_loc,  rawPts_metric] = vision.internal.buildable.detectFASTBuildable.detectFAST_uint8(I_u8c, minContrast);
    rawPts.Location = rawPts_loc;
    rawPts.Metric = rawPts_metric;
end

[locations, metricValues] = vision.internal.detector.applyMinQuality(rawPts, params);

if params.usingROI
    % Because the ROI was expanded earlier, we need to exclude corners
    % which are outside the original ROI.
    [locations, metricValues] ...
        = vision.internal.detector.excludePointsOutsideROI(...
        params.ROI, expandedROI, locations, metricValues);
end

% Pack the output into a cornerPoints object.
pts = cornerPoints(locations, 'Metric', metricValues);

%==========================================================================
function params = parseInputs(I,varargin)
if isSimMode()    
    params = vision.internal.detector.fast.parseInputs(I, varargin{:});
else
    params = parseInputs_cg(I,varargin{:});
end

%==========================================================================
function params = parseInputs_cg(I,varargin)

vision.internal.inputValidation.validateImage(I, 'I', 'grayscale');

imageSize = size(I);

% Optional Name-Value pair: 3 pairs (see help section)
defaults = vision.internal.detector.fast.getDefaultParameters(imageSize);
defaultsNoVal = getDefaultParametersNoVal();
properties    = getEmlParserProperties();

optarg = eml_parse_parameter_inputs(defaultsNoVal, properties, varargin{:});
params.MinQuality = (eml_get_parameter_value( ...
        optarg.MinQuality, defaults.MinQuality, varargin{:}));
params.MinContrast = (eml_get_parameter_value( ...
    optarg.MinContrast, defaults.MinContrast, varargin{:}));
ROI = (eml_get_parameter_value( ...
    optarg.ROI, defaults.ROI, varargin{:}));

usingROI = ~(optarg.ROI==uint32(0));

if usingROI
    params.usingROI = true;
    vision.internal.detector.checkROI(ROI, imageSize);       
else
    params.usingROI = false;
end

params.ROI = vision.internal.detector.roundAndCastToInt32(ROI);

vision.internal.detector.checkMinQuality(params.MinQuality);
vision.internal.detector.checkMinContrast(params.MinContrast);

%==========================================================================
function defaultsNoVal = getDefaultParametersNoVal()

defaultsNoVal = struct(...
    'MinQuality', uint32(0), ... 
    'MinContrast', uint32(0), ... 
    'ROI', uint32(0));

%==========================================================================
function properties = getEmlParserProperties()

properties = struct( ...
    'CaseSensitivity', false, ...
    'StructExpand',    true, ...
    'PartialMatching', false);

%==========================================================================
function flag = isSimMode()

flag = isempty(coder.target);   


