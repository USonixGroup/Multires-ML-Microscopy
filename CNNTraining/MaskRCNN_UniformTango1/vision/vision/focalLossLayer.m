function layer = focalLossLayer(varargin)

% Copyright 2019-2023 The MathWorks, Inc.

vision.internal.requiresNeuralToolbox(mfilename);

params = parseInputs(varargin{:});

% Create an internal representation of the layer.
internalLayer = nnet.internal.cnn.layer.FocalLossLayer(params.Name, ...
    params.Classes, ...
    params.Alpha, ...
    params.Gamma);

% Pass the internal layer to a function to construct a user visible layer.
layer = nnet.cnn.layer.FocalLossLayer(internalLayer);
end

%--------------------------------------------------------------------------
function params = parseInputs(varargin)

    p = inputParser;
    isVersionOne = false;
    
    if nargin >= 2 && isnumeric(varargin{1}) % handle inputs in v1.
        isVersionOne = true;
        inputIndex = 1;
        
        validateArgOne = @(x) validateattributes(x, {'numeric'}, {'finite', 'scalar', 'real', 'nonnegative', 'nonsparse'}, ...
            mfilename, 'Gamma', inputIndex);
        p.addRequired('ArgOne', validateArgOne);
        
        inputIndex = inputIndex + 1;
        validateArgTwo = @(x)validateattributes(x, {'numeric'}, {'finite', 'scalar', 'real', 'positive', 'nonsparse'}, ...
            mfilename, 'Alpha', inputIndex);
        p.addRequired('ArgTwo', validateArgTwo);
        
    else
        
        validateGamma = @(x)validateattributes(x, {'numeric'}, {'finite', 'scalar', 'real', 'nonnegative', 'nonsparse'}, ...
            mfilename, 'Gamma');
        p.addParameter('Gamma', 2, validateGamma);
        
        validateAlpha = @(x)validateattributes(x, {'numeric'}, {'finite', 'scalar', 'real', 'positive', 'nonsparse'}, ...
            mfilename, 'Alpha');
        p.addParameter('Alpha', 0.25, validateAlpha);
    end
    
    p.addParameter('Classes', 'auto', @iAssertValidClasses);
    
    p.addParameter('Name','',...
        @nnet.internal.cnn.layer.paramvalidation.validateLayerName);
    
    p.parse(varargin{:});
    
    if isVersionOne
        params.Gamma   = p.Results.ArgOne;
        params.Alpha   = p.Results.ArgTwo;
    else
        params.Gamma   = p.Results.Gamma;
        params.Alpha   = p.Results.Alpha;
    end
    
    params.Name    = p.Results.Name;
    params.Classes = p.Results.Classes;
    params.Name    = char(params.Name);
    
    params.Classes = iConvertClassesToCanonicalForm(params.Classes);
end

%--------------------------------------------------------------------------
function iAssertValidClasses(value)
iEvalAndThrow(@()...
    nnet.internal.cnn.layer.paramvalidation.validateClasses(value));
if ~iIsAuto(value)
    names = string(value);
    % There should be 2 or more classes
    if numel(names)<2
        error(message('vision:semanticseg:MultiClass'));
    end
end    
end

%--------------------------------------------------------------------------
function tf = iIsAuto(val)
tf = isequal(string(val), "auto");
end


%--------------------------------------------------------------------------
function classes = iConvertClassesToCanonicalForm(classes)
if iIsAuto(classes)
    classes = categorical();
else
    classes = ...
        nnet.internal.cnn.layer.paramvalidation...
            .convertClassesToCanonicalForm(classes);
end
end

%--------------------------------------------------------------------------
function iEvalAndThrow(func)
% Omit the stack containing internal functions by throwing as caller
try
    func();
catch exception
    throwAsCaller(exception)
end
end
