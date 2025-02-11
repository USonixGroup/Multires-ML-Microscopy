function Pts=detectSIFTFeatures(I, varargin)

% Copyright 2020-2024 The MathWorks, Inc.

%#codegen

checkImage(I);

Iu8 = im2uint8(I);

if isSimMode()
    params = parseInputs(varargin{:});
    PtsStruct=ocvDetectSIFT(Iu8, params);
else
    params = parseInputs_cg(varargin{:});
    
    PtsStruct = vision.internal.buildable.detectSIFTBuildable.detectSIFT(...
        Iu8, params.ContrastThreshold, params.EdgeThreshold, ...
        params.NumLayersInOctave, params.Sigma);
end

Pts = SIFTPoints(PtsStruct.Location, PtsStruct);

%========================================================================== 
function checkImage(I)

vision.internal.inputValidation.validateImage(I, 'I', 'grayscale');
               
%========================================================================== 
function flag = isSimMode()

flag = isempty(coder.target);

%==========================================================================
% Parse and check inputs - simulation
%==========================================================================
function params = parseInputs(varargin)

defaults = getDefaultParametersVal();

% Parse the PV pairs
parser = inputParser;
parser.addParameter('ContrastThreshold', defaults.ContrastThreshold, @checkContrastThreshold);
parser.addParameter('EdgeThreshold',      defaults.EdgeThreshold,      @checkEdgeThreshold);
parser.addParameter('NumLayersInOctave',  defaults.NumLayersInOctave,  @checkNumLayersInOctave);
parser.addParameter('Sigma',  defaults.Sigma,  @checkSigma);

% Parse input
parser.parse(varargin{:});

% Populate the parameters to pass into OpenCV's SIFT
params.EdgeThreshold       = single(parser.Results.EdgeThreshold);
params.NumLayersInOctave   = int32(parser.Results.NumLayersInOctave);
params.Sigma               = single(parser.Results.Sigma);
params.ContrastThreshold   = single(parser.Results.ContrastThreshold);

% Note: The contrast threshold will be divided by NumLayersInOctave 
% when the filtering is applied in the SIFT detector. Hence, multiplying
% in advance
params.ContrastThreshold   = params.ContrastThreshold * single(params.NumLayersInOctave);

%==========================================================================
% Parse and check inputs - code-generation
%==========================================================================
function params = parseInputs_cg(varargin)

% varargin must be non-empty
defaultsVal       = getDefaultParametersVal();
defaultsNoVal     = getDefaultParametersNoVal();
properties        = getEmlParserProperties();
optarg            = eml_parse_parameter_inputs(defaultsNoVal, properties, varargin{:});
contrastThreshold = eml_get_parameter_value(optarg.ContrastThreshold, ...
                        defaultsVal.ContrastThreshold, varargin{:});
edgeThreshold     = eml_get_parameter_value(optarg.EdgeThreshold, ...
                        defaultsVal.EdgeThreshold, varargin{:});
numLayersInOctave = eml_get_parameter_value(optarg.NumLayersInOctave, ...
                        defaultsVal.NumLayersInOctave, varargin{:});
sigma             = eml_get_parameter_value(optarg.Sigma, ...
                        defaultsVal.Sigma, varargin{:});        

vision.internal.errorIfNotFixedSize(contrastThreshold,'ContrastThreshold');
checkContrastThreshold(contrastThreshold);

vision.internal.errorIfNotFixedSize(edgeThreshold,'EdgeThreshold');
checkEdgeThreshold(edgeThreshold);

vision.internal.errorIfNotFixedSize(numLayersInOctave,'NumLayersInOctave');
checkNumLayersInOctave(numLayersInOctave);

vision.internal.errorIfNotFixedSize(sigma,'Sigma');
checkSigma(sigma);

params.EdgeThreshold     = single(edgeThreshold);
params.NumLayersInOctave = int32(numLayersInOctave);
params.Sigma             = single(sigma);

% Note: The contrast threshold will be divided by NumLayersInOctave 
% when the filtering is applied in the SIFT detector. Hence, multiplying
% in advance
params.ContrastThreshold   = single(contrastThreshold) * single(numLayersInOctave);

%==========================================================================
function defaultsVal = getDefaultParametersVal()

defaultsVal = struct(...
    'ContrastThreshold', single(0.0133), ...
    'EdgeThreshold', single(10.0), ...
    'NumLayersInOctave', int32(3),...
    'Sigma', single(1.6));

%==========================================================================
function defaultsNoVal = getDefaultParametersNoVal()

defaultsNoVal = struct(...
    'ContrastThreshold', uint32(0), ...
    'EdgeThreshold', uint32(0), ...
    'NumLayersInOctave', uint32(0),...
    'Sigma', uint32(0));

%==========================================================================
function properties = getEmlParserProperties()

properties = struct( ...
    'CaseSensitivity', false, ...
    'StructExpand',    true, ...
    'PartialMatching', false);

%==========================================================================
function tf = checkContrastThreshold(threshold)
validateattributes(threshold, {'numeric'}, {'scalar', ...
    'finite', 'nonsparse', 'real', 'nonnegative', 'nonnan', '<=', 1}, 'detectSIFTFeatures', 'ContrastThreshold');
tf = true;

%==========================================================================
function tf = checkEdgeThreshold(threshold)
validateattributes(threshold, {'numeric'}, {'scalar', ...
    'finite', 'nonsparse', 'real', 'nonnan', '>=', 1}, 'detectSIFTFeatures', 'EdgeThreshold');
tf = true;

%==========================================================================
function tf = checkNumLayersInOctave(numLayersOctaves)
validateattributes(numLayersOctaves, {'numeric'}, {'real', 'nonsparse', ...
    'scalar', 'nonnan', 'finite', 'integer', '>=', 1}, 'detectSIFTFeatures', 'NumLayersInOctave');
tf = true;

%==========================================================================
function tf = checkSigma(sigmaVal)
validateattributes(sigmaVal, {'numeric'}, {'scalar', ...
    'finite', 'nonsparse', 'real', 'nonnan', 'positive'}, 'detectSIFTFeatures', 'Sigma');
tf = true;
