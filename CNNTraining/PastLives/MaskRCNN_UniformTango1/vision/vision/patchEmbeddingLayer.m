function layer = patchEmbeddingLayer(patchSize, outputSize, nameValueArgs)
%

%   Copyright 2023 The MathWorks, Inc.

arguments
    patchSize        {mustBeNumeric, mustBeInteger, mustBeNonempty, mustBePositive, mustBeRowVector}
    outputSize (1,1) {mustBeNumeric, mustBeInteger, mustBePositive}
    nameValueArgs.Name                  {mustBeTextScalar, mustBeNonmissing} = ''
    nameValueArgs.SpatialFlattenMode    {mustBeTextScalar, mustBeNonmissing, iAssertValidSpatialFlattenMode} = 'column-major'
    nameValueArgs.Weights               = []
    nameValueArgs.Bias                  = []
    nameValueArgs.WeightLearnRateFactor {iAssertValidFactor} = 1
    nameValueArgs.BiasLearnRateFactor   {iAssertValidFactor} = 1
    nameValueArgs.WeightL2Factor        {iAssertValidFactor} = 1
    nameValueArgs.BiasL2Factor          {iAssertValidFactor} = 1
    nameValueArgs.WeightsInitializer    {iAssertValidWeightsInitializer} = 'glorot'
    nameValueArgs.BiasInitializer       {iAssertValidBiasInitializer} = 'zeros'
end

vision.internal.requiresNeuralToolbox(mfilename);

args = nameValueArgs;
args.PatchSize = patchSize;
args.OutputSize = outputSize;

% Gather arguments to CPU and convert them to canonical form
args = nnet.internal.cnn.layer.util.gatherParametersToCPU(args);
args = iConvertToCanonicalForm(args);

% Create the layer
layer = nnet.cnn.layer.PatchEmbeddingLayer(args.Name, args.PatchSize, args.OutputSize, args.SpatialFlattenMode);

layer.Weights = args.Weights;
layer.Bias = args.Bias;
layer.WeightsInitializer = args.WeightsInitializer;
layer.BiasInitializer = args.BiasInitializer;
layer.WeightLearnRateFactor = args.WeightLearnRateFactor;
layer.BiasLearnRateFactor = args.BiasLearnRateFactor;
layer.WeightL2Factor = args.WeightL2Factor;
layer.BiasL2Factor = args.BiasL2Factor;

end

function mustBeRowVector(value)
if ~isrow(value)
    error(message('vision:patchembed:invalidPatchSize'));
end
end

function iAssertValidSpatialFlattenMode(value)
try
    iValidateSpatialFlattenMode(value);
catch exception
    throwAsCaller(exception);
end
end

function iAssertValidFactor(value)
validateattributes(value, {'numeric'}, {'scalar','real','finite','nonnegative'});
end

function iAssertValidWeightsInitializer(value)
validValues = {'narrow-normal','glorot','he','orthogonal','zeros','ones'};
try
    iAssertValidInitializer(value, validValues, 'WeightsInitializer');
catch exception
    throwAsCaller(exception)
end
end

function iAssertValidBiasInitializer(value)
validValues = {'narrow-normal','zeros','ones'};
try
    iAssertValidInitializer(value, validValues, 'BiasInitializer');
catch exception
    throwAsCaller(exception)
end
end

function iAssertValidInitializer(value, validValues, name)
validateattributes(value, {'function_handle','char','string'}, {}, '', name);
if(ischar(value) || isstring(value))
    validatestring(value, validValues, '', name);
end
end

function inputArguments = iConvertToCanonicalForm(params)
% Make sure integer values are converted to double and strings to char
% vectors
inputArguments = struct;
inputArguments.PatchSize = iConvertToDouble(params.PatchSize);
inputArguments.OutputSize = iConvertToDouble(params.OutputSize);
inputArguments.SpatialFlattenMode = iValidateSpatialFlattenMode(params.SpatialFlattenMode);
inputArguments.WeightLearnRateFactor = iConvertToDouble(params.WeightLearnRateFactor);
inputArguments.BiasLearnRateFactor = iConvertToDouble(params.BiasLearnRateFactor);
inputArguments.WeightL2Factor = iConvertToDouble(params.WeightL2Factor);
inputArguments.BiasL2Factor = iConvertToDouble(params.BiasL2Factor);
inputArguments.WeightsInitializer = params.WeightsInitializer;
inputArguments.BiasInitializer = params.BiasInitializer;
inputArguments.Name = char(params.Name);
inputArguments.Weights = params.Weights;
inputArguments.Bias = params.Bias;
end

function value = iConvertToDouble(value)
value = double(value);
if isdlarray(value)
    value = extractdata(value);
end
end

function value = iValidateSpatialFlattenMode(value)
validValues = {'column-major', 'row-major'};
value = validatestring(value, validValues, '', 'SpatialFlattenMode');
end
