function [avgPrecision, actualIDs, scores] = evaluateImageRetrieval(queryImage, imageIndex, expectedIDs, varargin)

% Copyright 2017-2023 The MathWorks, Inc. 

%#codegen

if isSimMode() && nargin > 3
    [varargin{:}] = convertStringsToChars(varargin{:});
end

[params, wasROISpecified] = parseInputs(queryImage, imageIndex, expectedIDs, varargin{:});

if wasROISpecified
    [actualIDs, scores] = retrieveImages(queryImage, imageIndex, params);
else
    [actualIDs, scores] = retrieveImages(queryImage, imageIndex, 'NumResults', params.NumResults);
end

if isfinite(params.NumResults)
    % average-precision-at-N
    avgPrecision = vision.internal.averagePrecision(actualIDs(:), expectedIDs(:), params.NumResults);
else
    % average precision over all expectedIDs
    avgPrecision = vision.internal.averagePrecision(actualIDs(:), expectedIDs(:));
end

% -------------------------------------------------------------------------
function [params, wasROISpecified] = parseInputs(queryImage,imageIndex,expectedIDs, varargin)
coder.inline('always');
coder.internal.prefer_const(queryImage,imageIndex,expectedIDs, varargin{:});

vision.internal.inputValidation.validateImage(queryImage,'queryImage');

% Validate the imageIndex input
validateattributes(imageIndex, {'invertedImageIndex'},{}, mfilename, 'imageIndex',2);

validateattributes(expectedIDs, {'numeric'}, ...
    {'vector','integer','positive','real','nonsparse','finite'},...
    mfilename,'expectedIDs')

% parse optional parameters
defaults  = getParameterDefaults();

if isSimMode()
    parser    = inputParser;

    parser.addParameter('NumResults', defaults.NumResults, @checkNumberOfResults);
    parser.addParameter('ROI',        defaults.ROI,        @(x)vision.internal.detector.checkROI(x,size(queryImage)));

    parse(parser, varargin{:});

    params.NumResults  = double(parser.Results.NumResults);

    wasROISpecified = ~ismember('ROI', parser.UsingDefaults);

    if wasROISpecified
        params.ROI = double(round(parser.Results.ROI));
    end
else
    defaults = struct( ...
        'NumResults', defaults.NumResults, ...
        'ROI', defaults.ROI);
    % Define parser mapping struct
    pvPairs = struct('NumResults', uint32(0), ...
        'ROI', uint32(0));
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

    % Validate the value of numResults
    checkNumberOfResults(numResults);
    params.NumResults = numResults;
    % Validate the value of ROI
    vision.internal.detector.checkROI(roi, size(queryImage));
    % Check if ROI is specified by user
    if isempty(roi)
        wasROISpecified    = false;
    else
        wasROISpecified    = true;
    end
    params.ROI = double(round(roi));
end

% -------------------------------------------------------------------------
function defaults = getParameterDefaults()

defaults.NumResults = inf; % by default, measure all results
defaults.ROI        = [];

% -------------------------------------------------------------------------
function checkNumberOfResults(n)
if isfinite(n)
    attrib = {'scalar','real','nonsparse','nonnan','positive', 'integer'};
    validateattributes(n, {'numeric'}, attrib, mfilename, 'NumResults');
else
    attrib = {'scalar','real','nonsparse','nonnan','positive'};
    validateattributes(n, {'numeric'}, attrib, mfilename, 'NumResults');
end

% -------------------------------------------------------------------------
function out = isSimMode()
% Check if in simulation mode
out = coder.target('MATLAB');