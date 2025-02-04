function [indexPairs, matchMetric] = matchFeatures(varargin)

%  Copyright 2010-2024 The MathWorks, Inc.

%#codegen
%#ok<*EMCLS>
%#ok<*EMCA>


% Parse and check inputs
if isempty(coder.target)
    [features1, features2, metric, matchThreshold, method, maxRatioThreshold, ...
        isPrenormalized, uniqueMatches, isLegacyMethod] = parseInputs(varargin{:});
else
    [features1, features2, metric, matchThreshold, method, maxRatioThreshold, ...
        isPrenormalized, uniqueMatches, isLegacyMethod] = parseInputsCodegen(varargin{:});
end

vision.internal.matchFeatures.checkFeatureConsistency(features1, features2);

% If GPU is enabled, generate GPU code for matchFeatures
if coder.gpu.internal.isGpuEnabled && strcmpi(method, 'Exhaustive')
    [matchMetric, indexPairs] = ...
        vision.internal.codegen.gpu.matchFeaturesGPUImpl(features1, features2, metric, matchThreshold, maxRatioThreshold, uniqueMatches);
else
    % If 'approximate' method is called with GPU enabled, display warning and generate C code.
    if coder.gpu.internal.isGpuEnabled && strcmpi(method, 'Approximate')
        coder.internal.compileWarning('gpucoder:common:MatchFeatureUnsupportedSearchMethod');
    end
    
    % Determine output class
    if (isa(features1, 'double'))
        outputClass = 'double';
    else
        outputClass = 'single';
    end
    
    % Convert match threshold percent to a numeric threshold
    matchThreshold = vision.internal.matchFeatures.percentToLevel( ...
        matchThreshold, size(features1, 2), metric, outputClass);
    
    [indexPairs, matchMetric] = ...
        vision.internal.matchFeatures.cvalgMatchFeatures(...
        features1, features2, metric, matchThreshold, method, ...
        maxRatioThreshold, isPrenormalized, uniqueMatches, isLegacyMethod, ...
        outputClass);
end

%==========================================================================
% Parse and check inputs for code generation
%==========================================================================
function [features1, features2, metric, match_thresh, method, ...
    maxRatioThreshold, isPrenormalized, uniqueMatches, isLegacyMethod] = parseInputsCodegen(varargin)

fileName = 'matchFeatures';

eml_lib_assert(nargin == 2 || nargin > 3, ...
    'vision:matchFeatures:NotEnoughArgs', ...
    'Not enough input arguments.');

f1 = varargin{1};
f2 = varargin{2};

vision.internal.inputValidation.checkFeatures(f1, 'matchFeatures', 'features1');
vision.internal.inputValidation.checkFeatures(f2, 'matchFeatures', 'features2');

isBinaryFeature = isa(f1, 'binaryFeatures');
isRadiusSearch  = false;
defaults = vision.internal.matchFeatures.getDefaultParameters(isBinaryFeature, isRadiusSearch);

% Set parser inputs
params = struct( ...
    'Metric',                uint32(0), ...
    'MatchThreshold',        uint32(0), ...
    'Method',                uint32(0), ...
    'MaxRatio',              uint32(0), ...
    'Prenormalized',         uint32(0), ...
    'Unique',         uint32(0));

popt = struct( ...
    'CaseSensitivity', false, ...
    'StructExpand',    true, ...
    'PartialMatching', true);

if (nargin > 3)
    % Parse parameter/value pairs
    optarg = eml_parse_parameter_inputs(params, popt, varargin{3:end});
    
    metricString = eml_get_parameter_value(optarg.Metric, ...
        defaults.Metric, varargin{3:end});
    match_thresh  = eml_get_parameter_value(optarg.MatchThreshold, ...
        defaults.MatchThreshold, varargin{3:end});
    methodString = eml_get_parameter_value(optarg.Method, ...
        defaults.Method, varargin{3:end});
    maxRatioThreshold = eml_get_parameter_value(optarg.MaxRatio, ...
        defaults.MaxRatio, varargin{3:end});
    isPrenormalizedFromUser = eml_get_parameter_value(optarg.Prenormalized, ...
        defaults.Prenormalized, varargin{3:end});
    uniqueMatchesFromUser = eml_get_parameter_value(optarg.Unique, ...
        defaults.Unique, varargin{3:end});
    
    % Check parameters
    
    metric          = vision.internal.matchFeatures.checkMetric(metricString, fileName);
    vision.internal.matchFeatures.checkMatchThreshold(match_thresh, fileName);
    method          = checkMatchMethod(methodString);
    vision.internal.matchFeatures.checkMaxRatioThreshold(maxRatioThreshold, fileName);
    checkPrenormalized(isPrenormalizedFromUser);
    vision.internal.matchFeatures.checkUniqueMatches(uniqueMatchesFromUser, fileName);
    
    isPrenormalized = logical(isPrenormalizedFromUser);
    uniqueMatches   = logical(uniqueMatchesFromUser);
    
    isMethodSetByUser        = logical(optarg.Method);
    isPrenormalizedSetByUser = logical(optarg.Prenormalized);
    
else
    metric             = defaults.Metric;
    match_thresh       = defaults.MatchThreshold;
    method             = defaults.Method;
    maxRatioThreshold  = defaults.MaxRatio;
    
    isPrenormalized    = logical(defaults.Prenormalized);
    uniqueMatches      = logical(defaults.Unique);
    
    isMethodSetByUser        = false;
    isPrenormalizedSetByUser = false;
    
end

crossCheckMetricAndMethod(isMethodSetByUser, metric, method);

crossCheckPrenormalizedAndMethod(isPrenormalizedSetByUser, ...
    isMethodSetByUser, method);

isLegacyMethod = isLegacy(method);

[features1, features2, metric] = assignFeaturesAndMetric(f1, f2, metric);

%==========================================================================
% Parse and check inputs
%==========================================================================
function [features1, features2, metric, match_thresh, method,...
    maxRatioThreshold, isPrenormalized, uniqueMatches, isLegacyMethod] ...
    = parseInputs(varargin)

fileName = 'matchFeatures';

if nargin >= 1
    isBinaryFeature = isa(varargin{1}, 'binaryFeatures');
else
    isBinaryFeature = false;
end

isRadiusSearch = false;
defaults = vision.internal.matchFeatures.getDefaultParameters(isBinaryFeature, isRadiusSearch);

% Setup parser
parser = inputParser;
parser.addRequired('features1', ...
    @(x)vision.internal.inputValidation.checkFeatures(x, fileName, 'features1'));
parser.addRequired('features2', ...
    @(x)vision.internal.inputValidation.checkFeatures(x, fileName, 'features2'));
parser.addParameter('MatchThreshold', defaults.MatchThreshold, ...
    @(x)vision.internal.matchFeatures.checkMatchThreshold(x, fileName));
parser.addParameter('Method', defaults.Method);
parser.addParameter('MaxRatio', defaults.MaxRatio, ...
    @(x)vision.internal.matchFeatures.checkMaxRatioThreshold(x, fileName));
parser.addParameter('Metric', defaults.Metric);
parser.addParameter('Prenormalized', defaults.Prenormalized, ...
    @checkPrenormalized);
parser.addParameter('Unique', defaults.Unique, ...
    @(x)vision.internal.matchFeatures.checkUniqueMatches(x, fileName));

% Parse input
parser.parse(varargin{:});

method            = checkMatchMethod(parser.Results.Method);
metric            = vision.internal.matchFeatures.checkMetric(parser.Results.Metric, fileName);
match_thresh      = parser.Results.MatchThreshold;
maxRatioThreshold = parser.Results.MaxRatio;
isPrenormalized   = logical(parser.Results.Prenormalized);
uniqueMatches     = logical(parser.Results.Unique);

isLegacyMethod = isLegacy(method);

% Issue warnings or errors if using new parameters with legacy method
% values.
if isLegacyMethod
    
    if strcmpi(method, 'nearestneighbor_old')
        % issue deprecation warning
        warning(message('vision:matchFeatures:deprecatedNNOld'));
        
    end
    
    % Unique cannot be used with old Method values
    if isParameterSetByCaller('Unique', parser)
        error(message('vision:matchFeatures:invalidMethodForUnique'));
    end
    
    %'MaxRatio' only has meaning if the 'Method' is 'NearestNeighborRatio'
    %(for backward compatibility), Exhaustive, or Approximate.
    if ~strcmpi(method, 'nearestneighborratio') ...
            && isParameterSetByCaller('MaxRatio', parser)
        warning(message('vision:matchFeatures:maxRatioUnused'));
    end
    
end

isMethodSetByUser        = isParameterSetByCaller('Method', parser);
isPrenormalizedSetByUser = isParameterSetByCaller('Prenormalized', parser);

crossCheckMetricAndMethod(isMethodSetByUser, metric, method);

crossCheckPrenormalizedAndMethod(isPrenormalizedSetByUser, ...
    isMethodSetByUser, method);

if isBinaryFeature
    if isPrenormalizedSetByUser
        warning(message('vision:matchFeatures:binParamUnused', 'Prenormalized'));
    end
    
    if isParameterSetByCaller('Metric', parser)
        warning(message('vision:matchFeatures:binParamUnused', 'Metric'));
    end
end

f1 = parser.Results.features1;
f2 = parser.Results.features2;

[features1, features2, metric] = assignFeaturesAndMetric(f1, f2, metric);

%==========================================================================
function [features1, features2, metric] = assignFeaturesAndMetric(f1, f2,...
    temp_metric)

% Handle the case when features are of class binaryFeatures

coder.internal.errorIf(~isequal(class(f1), class(f2)),...
    'vision:matchFeatures:featuresNotSameClass');

% Assign outputs
if isa(f1, 'binaryFeatures')
    features1 = f1.Features;
    features2 = f2.Features;
    metric = 'hamming'; % assume default metric for the binary features
else
    features1 = f1;
    features2 = f2;
    metric = lower(temp_metric);
end

%==========================================================================
function matchedValue = checkMatchMethod(value)
list = {'nearestneighborratio', 'threshold', 'nearestneighborsymmetric',...
    'nearestneighbor_old','approximate','exhaustive'};
validateattributes(value, {'char','string'}, {'nonempty'}, 'matchFeatures', ...
    'Method');
matchedValue = validatestring(value, list, 'matchFeatures', 'Method');

%==========================================================================
function checkPrenormalized(isPrenormalized)
validateattributes(isPrenormalized, {'logical','numeric'}, ...
    {'nonempty', 'scalar', 'real', 'nonnan', 'nonsparse'}, ...
    'matchFeatures', 'Prenormalized');

%==========================================================================
function crossCheckMetricAndMethod(isMethodSetByUser, metric, method)

if isMethodSetByUser
    % only check if Method is user specified. Otherwise an error can break
    % backward compatibility.
    invalidUseOfNormxcorr = ...
        (strcmpi(method, 'approximate') || strcmpi(method, 'exhaustive')) ...
        && strcmpi(metric, 'normxcorr');
    
    coder.internal.errorIf(invalidUseOfNormxcorr, ...
        'vision:matchFeatures:invalidMethodForNormxcorr');
end

%==========================================================================
function crossCheckPrenormalizedAndMethod(userDefinedPrenormalized, ...
    userDefinedMethod, method)

% only check if the user specifies the Method. This preserves backward
% compatibility when the user only specified the Prenormalized parameter.
if userDefinedMethod
    invalidMethodForPrenormalized = userDefinedPrenormalized ...
        && (strcmpi(method, 'approximate') || strcmpi(method, 'exhaustive'));
    
    coder.internal.errorIf(invalidMethodForPrenormalized, ...
        'vision:matchFeatures:invalidMethodForPrenormalized');
end

%==========================================================================
% Return true if using one of the legacy Method values
%==========================================================================
function tf = isLegacy(method)

tf = strcmpi(method,'threshold') ...
    || strcmpi(method,'nearestneighborratio')...
    || strcmpi(method,'nearestneighborsymmetric')...
    || strcmpi(method,'nearestneighbor_old');

%==========================================================================
function tf = isParameterSetByCaller(param, parser)
tf = strcmp(param, parser.UsingDefaults);
tf = ~any(tf);