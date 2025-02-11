function [imageIDs, scores, varargout] = retrieveImages(queryImage, imageIndex, varargin)

% Copyright 2017-2023 The MathWorks, Inc.

%#codegen

nargoutchk(0,3);
if isSimMode()
    if nargin > 2
        [varargin{:}] = convertStringsToChars(varargin{:});
    end
    params = parseInputs(queryImage, imageIndex, varargin{:});
else
    params = parseInputsCodegen(queryImage, imageIndex, varargin{:});
end

queryImage = vision.internal.detector.cropImageIfRequested(queryImage, params.ROI, params.UsingROI);

[imageIDs, scores, queryWords] = search(imageIndex, queryImage, ...
    'NumResults', params.NumResults, 'Metric', params.Metric);

if nargout==3
    if params.UsingROI
        queryWords = addROIOffset(queryWords,params.ROI);
    end
    varargout{1} = queryWords;
end
end

% -------------------------------------------------------------------------
function params = parseInputs(queryImage, imageIndex, varargin)
vision.internal.inputValidation.validateImage(queryImage,'queryImage');

validateattributes(imageIndex, {'invertedImageIndex'}, {}, mfilename, 'imageIndex',2);

% parse optional parameters
defaults = getParameterDefaults();
parser   = inputParser;

parser.addParameter('NumResults',  defaults.NumResults,  @checkNumberOfResults);
parser.addParameter('ROI',         defaults.ROI,         @(x)vision.internal.detector.checkROI(x,size(queryImage)));
parser.addParameter('Metric',      defaults.Metric);

parse(parser, varargin{:});

params.NumResults  = double(parser.Results.NumResults);
params.ROI         = parser.Results.ROI;
params.UsingROI    = ~ismember('ROI', parser.UsingDefaults);
params.Metric      = validatestring(parser.Results.Metric,{'cosine','L1'},mfilename, 'Metric');
end

% -------------------------------------------------------------------------
function params = parseInputsCodegen(queryImage, imageIndex, varargin)
coder.inline('always')
coder.internal.prefer_const(varargin{:});

vision.internal.inputValidation.validateImage(queryImage,'queryImage');

validateattributes(imageIndex, {'invertedImageIndex'}, {}, mfilename, 'imageIndex',2);

% get default parameters values
defaultParamValues = getParameterDefaults();
defaults = struct( ...
    'NumResults', defaultParamValues.NumResults, ...
    'ROI', defaultParamValues.ROI, ...
    'Metric', defaultParamValues.Metric);
% Define parser mapping struct
pvPairs = struct('NumResults', uint32(0), ...
    'ROI', uint32(0), ...
    'Metric', uint32(0));
% Specify parser options
poptions = struct( ...
    'CaseSensitivity', false, ...
    'StructExpand',    true, ...
    'PartialMatching', true);

% Parse PV pairs
pstruct = coder.internal.parseParameterInputs(pvPairs, poptions, varargin{:});
numResults = coder.internal.getParameterValue(pstruct.NumResults, ...
    defaults.NumResults, varargin{:});
roi = coder.internal.getParameterValue(pstruct.ROI, ...
    defaults.ROI, varargin{:});
metric = coder.internal.getParameterValue(pstruct.Metric, ...
    defaults.Metric, varargin{:});

checkNumberOfResults(numResults);
params.NumResults = numResults;
vision.internal.detector.checkROI(roi, size(queryImage));
params.ROI = roi;
params.UsingROI    = ~isempty(roi);
params.Metric = validatestring(metric,{'cosine','L1'},mfilename, 'Metric');
end

% -------------------------------------------------------------------------
function defaults = getParameterDefaults()

defaults.NumResults = 20;
defaults.ROI        = [];
defaults.Metric     = 'cosine';
end

% -------------------------------------------------------------------------
function checkNumberOfResults(n)
attrib = {'scalar','real','nonsparse','nonnan','positive'};
if isSimMode()
    if isfinite(n)
        % add integer for non-inf
        attrib =  [attrib 'integer'];
    end
    validateattributes(n, {'numeric'}, attrib, mfilename, 'NumResults');
else
    if isfinite(n)
        attrib = {'scalar','real','nonsparse','nonnan','positive','integer'};
        validateattributes(n, {'numeric'}, attrib, mfilename, 'NumResults');
    else
        attrib = {'scalar','real','nonsparse','nonnan','positive'};
        validateattributes(n, {'numeric'}, attrib, mfilename, 'NumResults');
    end
end
end

% -------------------------------------------------------------------------
function out = isSimMode()
% Check if in simulation mode
out = coder.target('MATLAB');
end
