%==========================================================================
% Validate and parse inputs for model fitting functions
%==========================================================================

% Copyright 2018-2024 The MathWorks, Inc.
function [ptCloud, ransacParams, sampleIndices, ...
    referenceVector, maxAngularDistance] = ...
    validateAndParseRansacInputs(filename, optionalConstraint, varargin)

%#codegen
coder.inline('always');
coder.internal.prefer_const(varargin{:});
if(isSimMode())
    % Simulation
    params = parseAndValidateArgsSimulation(filename, ...
        optionalConstraint, varargin{:});
else
    % Codegen
    % check if GPU is enabled and if it is pcfitplane function.
    if isGPUTarget() && strcmp(filename, 'pcfitplane')
        % Find the Name-Value pair: 'Confidence' and display a warning for
        % the same.
        for iter = 1:numel(varargin)
            if strcmp(varargin{iter}, 'Confidence')
                coder.internal.warning('vision:pointcloud:confNotSupportedForGPUCodegen');
            end
        end
    end
    params = parseAndValidateArgsCodegen(filename, ...
        optionalConstraint, varargin{:});
end

ptCloud                      = params.ptCloud;
ransacParams.maxDistance     = params.maxDistance;
ransacParams.maxNumTrials    = params.MaxNumTrials;
ransacParams.confidence      = params.Confidence;
sampleIndices                = params.SampleIndices;
if optionalConstraint
    referenceVector    = params.referenceVector;
    maxAngularDistance = params.maxAngularDistance;
    % Convert to radians
    maxAngularDistance = maxAngularDistance*pi/180;
else
    referenceVector    = [];
    maxAngularDistance = [];
end

%==========================================================================
function params = ...
    parseAndValidateArgsSimulation(filename, optionalConstraint, varargin)

parser                = inputParser;
parser.CaseSensitive  = false;
parser.FunctionName   = filename;
defaultParams         = getDefaultParams();
defaultOptionalInputs = getDefaultOptionalInputs();
% Add required arguments
parser.addRequired('ptCloud', @validatePointCloudInput);
parser.addRequired('maxDistance', @(x)checkMaxDistance(x, filename));
% Add optional arguments for orientation constraints
if optionalConstraint
    parser.addOptional('referenceVector', ...
        defaultOptionalInputs.referenceVector, ...
        @(x)checkReferenceVector(x, filename));
    
    parser.addOptional('maxAngularDistance', ...
        defaultOptionalInputs.maxAngularDistance, ...
        @(x)checkMaxAngularDistance(x, filename));
end
% Add PV-pair arguments
parser.addParameter('MaxNumTrials', defaultParams.MaxNumTrials, ...
    @(x)checkMaxNumTrials(x, filename));
parser.addParameter('Confidence', defaultParams.Confidence, ...
    @(x)checkConfidence(x, filename));
parser.addParameter('SampleIndices', defaultParams.SampleIndices, ...
    @(x)checkSampleIndices(x, filename));

parser.parse(varargin{:});

params = parser.Results;

%==========================================================================
function params = ...
    parseAndValidateArgsCodegen(filename, optionalConstraint, varargin)
coder.inline('always');
coder.internal.prefer_const(varargin{:});
validatePointCloudInput(varargin{1});
params.ptCloud     = varargin{1};
checkMaxDistance(varargin{2}, filename);
params.maxDistance = varargin{2};

defaultParams         = getDefaultParams();
defaultOptionalInputs = getDefaultOptionalInputs();

params.MaxNumTrials   = defaultParams.MaxNumTrials;
params.Confidence     = defaultParams.Confidence;
params.SampleIndices  = defaultParams.SampleIndices;

if(optionalConstraint)
    params.referenceVector    = defaultOptionalInputs.referenceVector;
    params.maxAngularDistance = defaultOptionalInputs.maxAngularDistance;
end

if(length(varargin) > 2)
    paramIdx = coder.internal.indexInt(0);
    if(optionalConstraint)
        % Find the first param input
        numOptInputs = 0;
        for n = 3 : length(varargin)
            if ischar(varargin{n}) || isstring(varargin{n})
                paramIdx = coder.internal.indexInt(n);
                break;
            end
            numOptInputs = numOptInputs + 1;
        end
        switch (numOptInputs)
            case 0
                params.referenceVector    = defaultOptionalInputs.referenceVector;
                params.maxAngularDistance = defaultOptionalInputs.maxAngularDistance;
            case 1
                checkReferenceVector(varargin{3}, filename);
                params.referenceVector    = cast(varargin{3}, 'like', ...
                    defaultOptionalInputs.referenceVector);
                params.maxAngularDistance = defaultOptionalInputs.maxAngularDistance;
            case 2
                checkReferenceVector(varargin{3}, filename);
                checkMaxAngularDistance(varargin{4}, filename);
                params.referenceVector    = cast(varargin{3}, 'like', ...
                    defaultOptionalInputs.referenceVector);
                params.maxAngularDistance = cast(varargin{4}, 'like', ...
                    defaultOptionalInputs.maxAngularDistance);
            otherwise
                disp('Number of optional inputs should be <=2. Ignoring extra optional inputs.');
                checkReferenceVector(varargin{3}, filename);
                checkMaxAngularDistance(varargin{4}, filename);
                params.referenceVector    = cast(varargin{3}, 'like', ...
                    defaultOptionalInputs.referenceVector);
                params.maxAngularDistance = cast(varargin{4}, 'like', ...
                    defaultOptionalInputs.maxAngularDistance);
        end
    else
        paramIdx = coder.internal.indexInt(3);
    end
    if(0 ~= paramIdx)
        pvPairs = struct(...
            'MaxNumTrials', uint32(0),...
            'Confidence', uint32(0),...
            'SampleIndices', uint32(0));
        
        properties =  struct( ...
            'CaseSensitivity', false, ...
            'StructExpand',    true, ...
            'PartialMatching', false);
        
        optarg = eml_parse_parameter_inputs(pvPairs, properties, ...
            varargin{paramIdx:end});
        
        params.MaxNumTrials  = cast(eml_get_parameter_value(optarg.MaxNumTrials, ...
            defaultParams.MaxNumTrials, varargin{paramIdx:end}), ...
            'like', defaultParams.MaxNumTrials);
        
        params.Confidence    = cast(eml_get_parameter_value(optarg.Confidence, ...
            defaultParams.Confidence, varargin{paramIdx:end}), ...
            'like', defaultParams.Confidence);
        
        params.SampleIndices= cast(eml_get_parameter_value(optarg.SampleIndices, ...
            defaultParams.SampleIndices, varargin{paramIdx:end}), ...
            'like', defaultParams.SampleIndices);
        
        checkMaxNumTrials(params.MaxNumTrials, filename);
        checkConfidence(params.Confidence, filename);
        checkSampleIndices(params.SampleIndices, filename);
    end
end

%==========================================================================
function defaultParams = getDefaultParams()
defaultParams = struct('MaxNumTrials', 1000, 'Confidence', 99, ...
    'SampleIndices', []);

%==========================================================================
function defaultOptionalInputs = getDefaultOptionalInputs()
defaultOptionalInputs = struct('referenceVector', [], ...
    'maxAngularDistance', 5);

%==========================================================================
function tf = checkMaxDistance(value, filename)
validateattributes(value,{'single', 'double'}, ...
    {'real', 'nonsparse', 'scalar', 'nonnan', 'finite', 'nonnegative'}, filename, 'maxDistance');
tf = true;

%==========================================================================
function tf = checkReferenceVector(value, filename)
validateattributes(value,{'single', 'double'}, ...
    {'real', 'nonsparse', 'vector', 'nonnan', 'finite', 'numel', 3}, filename, 'referenceVector');
validateattributes(any(value), {'logical'}, ...
    {'nonzero'}, filename, 'referenceVector');
tf = true;

%==========================================================================
function tf = checkMaxAngularDistance(value, filename)
validateattributes(value,{'single','double'}, ...
    {'real', 'nonsparse', 'scalar', 'nonnan', 'finite', 'nonnegative'}, filename, 'maxAngularDistance');
tf = true;

%==========================================================================
function tf = checkMaxNumTrials(value, filename)
validateattributes(value, {'numeric'}, ...
    {'real', 'nonsparse', 'scalar', 'nonnan', 'finite', 'integer', 'positive'}, filename, 'MaxNumTrials');
tf = true;

%==========================================================================
function tf = checkConfidence(value, filename)
validateattributes(value, {'numeric'}, ...
    {'real', 'nonsparse', 'scalar', 'nonnan', 'finite', 'positive', '<', 100}, filename, 'Confidence');
tf = true;

%==========================================================================
function tf = checkSampleIndices(value, filename)
if ~isempty(value)
    validateattributes(value, {'numeric'}, ...
        {'real', 'nonsparse', 'vector', 'nonnan', 'finite', 'integer', 'positive' },...
        filename, 'SampleIndices');
else
    validateattributes(value, {'numeric'}, {'real'},...
        filename, 'SampleIndices');
end
tf = true;

%==========================================================================
function tf = validatePointCloudInput(value)
validateattributes(value, {'pointCloud'}, {'scalar'}, '', 'ptCloud');

tf = true;

function flag = isSimMode()
flag = isempty(coder.target);

% ==========================================================================
% GPU codegen support flag
% ==========================================================================
function flag = isGPUTarget()
flag = coder.gpu.internal.isGpuEnabled;
