classdef PatchEmbeddingLayer < nnet.layer.Layer ...
        & nnet.layer.Formattable & nnet.layer.Acceleratable
    % PatchEmbeddingLayer   Patch embedding layer
    %
    %   To create a patch embedding layer, use patchEmbeddingLayer.
    %
    %   PatchEmbeddingLayer properties:
    %       PatchSize              - Patch size
    %       InputSize              - Number of channels in the layer input
    %       OutputSize             - Number of channels in the layer output
    %       SpatialFlattenMode     - Spatial flatten mode
    %       Name                   - Name for the layer
    %       NumInputs              - Number of inputs of the layer
    %       InputNames             - Names of the inputs of the layer
    %       NumOutputs             - Number of outputs of the layer
    %       OutputNames            - Names of the outputs of the layer
    %
    % Properties for learnable parameters:
    %       Weights                - Weights
    %       Bias                   - Bias
    %       WeightsInitializer     - Function for initializing the weights
    %       BiasInitializer        - Function for initializing the bias
    %       WeightLearnRateFactor  - Multiplier for the learning rate of
    %                                the weights
    %       BiasLearnRateFactor    - Multiplier for the learning rate of
    %                                the bias
    %       WeightL2Factor         - Multiplier for the L2 regularizer for
    %                                the weights
    %       BiasL2Factor           - Multiplier for the L2 regularizer for
    %                                the bias
    %
    %   Example:
    %       % Create a patch embedding layer with a patch size of 16 and an
    %       % output size of 256.
    %
    %       patchSize = 16;
    %       outputSize = 256;
    %       layer = patchEmbeddingLayer(patchSize, outputSize);
    %
    %   See also patchEmbeddingLayer
    
    %   Copyright 2023 The MathWorks, Inc.

    properties(SetAccess=private)
        PatchSize
    end

    properties(SetAccess=private, Dependent)
        InputSize
    end

    properties(SetAccess=private)
        OutputSize
        SpatialFlattenMode
    end

    properties(Learnable, Dependent)
        Weights
        Bias
    end

    properties(Dependent)
        WeightsInitializer
        BiasInitializer
        WeightLearnRateFactor
        BiasLearnRateFactor
        WeightL2Factor
        BiasL2Factor
    end
    
    properties(Access=private, Hidden)
        NumSpatialDims
        PrivateInputSize
        PrivateWeights
        PrivateBias
        PrivateWeightsInitializer
        PrivateBiasInitializer
    end

    methods
        function layer = PatchEmbeddingLayer(name, patchSize, outputSize, flattenMode)
            arguments
                name
                patchSize
                outputSize
                flattenMode = 'column-major'
            end
            layer.PatchSize = patchSize;
            layer.OutputSize = outputSize;
            layer.SpatialFlattenMode = flattenMode;
            layer.Name = name;
            [layer.Description, layer.Type] = layer.getOneLineDisplay;
        end

        function X = predict(layer, X)
            numSpatialDims = numel(finddim(X,'S'));
            numChannels = size(X,finddim(X,'C'));
            iAssertValidNumSpatialDimensions(numSpatialDims, layer.NumSpatialDims);
            iAssertValidNumChannels(layer.InputSize, numChannels);
            dimLabel = dims(X);

            % Extract patches and project each patch to output size
            X = dlconv(X, layer.Weights, layer.Bias, Stride=layer.PatchSize);
            
            % Flatten the spatial dimensions
            if strcmp(layer.SpatialFlattenMode, 'column-major')
                X = reshape(X, [prod(size(X, 1:numSpatialDims)), size(X, numSpatialDims+1:ndims(X))]);
            else
                % Row-major spatial flattening
                spatialIndices = 1:numSpatialDims;
                otherIndices = numSpatialDims+1:ndims(X);
                X = permute(X, [flip(spatialIndices), otherIndices]);
                X = reshape(X, [prod(size(X, spatialIndices)), size(X, otherIndices)]);
            end

            % Return dimension labels with one 'S' label
            X = dlarray(X, dimLabel(numSpatialDims:end));
        end

        function layer = initialize(layer, X)
            [numSpatialDims, numChannels] = validateNumSpatialAndChannelDims(layer, X);

            patchSize = layer.PatchSize;
            if isscalar(patchSize)
                layer.PatchSize = repmat(patchSize, 1, numSpatialDims);
            else
                numelPatchSize = numel(patchSize);
                iAssertValidNumSpatialDimensions(numSpatialDims, numelPatchSize);
            end
            layer.NumSpatialDims = numSpatialDims;

            if isempty(layer.PrivateInputSize)
                layer.PrivateInputSize = numChannels;
            else
                iAssertValidNumChannels(layer.InputSize, numChannels)
            end

            if isempty(layer.Weights)
                layer = setPrivateWeightsInitializer(layer, layer.WeightsInitializer);
                weightSize = [layer.PatchSize, layer.InputSize, layer.OutputSize];
                layer.Weights = layer.PrivateWeightsInitializer.initialize(weightSize, 'Weights');
            end

            if isempty(layer.Bias)
                biasSize = [layer.OutputSize, 1];
                layer.Bias = layer.PrivateBiasInitializer.initialize(biasSize, 'Bias');
            end
        end

        function out = saveobj(layer)
            out.Version = 2.0;
            out.Name = layer.Name;
            out.PatchSize = layer.PatchSize;
            out.OutputSize = layer.OutputSize;
            out.SpatialFlattenMode = layer.SpatialFlattenMode;
            out.Weights = gather(layer.Weights);
            out.Bias = gather(layer.Bias);
            out.WeightsInitializer = layer.WeightsInitializer;
            out.BiasInitializer = layer.BiasInitializer;
            out.WeightLearnRateFactor = layer.WeightLearnRateFactor;
            out.BiasLearnRateFactor = layer.BiasLearnRateFactor;
            out.WeightL2Factor = layer.WeightL2Factor;
            out.BiasL2Factor = layer.BiasL2Factor;
            out.PrivateInputSize = layer.PrivateInputSize;
            % Not saving NumSpatialDims as it will be inferred without
            % error during initialize.
        end

        function value = get.Weights(layer)
            value = layer.PrivateWeights;
            if isdlarray(value)
                value = extractdata(value);
            end
        end

        function value = get.Bias(layer)
            value = layer.PrivateBias;
            if isdlarray(value)
                value = extractdata(value);
            end
        end

        function value = get.InputSize(layer)
            value = layer.PrivateInputSize;
            if isempty(value)
                value = 'auto';
            end
        end

        function value = get.WeightsInitializer(layer)
            if iIsCustomInitializer(layer.PrivateWeightsInitializer)
                value = layer.PrivateWeightsInitializer.Fcn;
            else
                value = layer.PrivateWeightsInitializer.Name;
            end
        end

        function value = get.BiasInitializer(layer)
            if iIsCustomInitializer(layer.PrivateBiasInitializer)
                value = layer.PrivateBiasInitializer.Fcn;
            else
                value = layer.PrivateBiasInitializer.Name;
            end
        end

        function value = get.WeightLearnRateFactor(layer)
            value = getLearnRateFactor(layer,"Weights");
        end

        function value = get.BiasLearnRateFactor(layer)
            value = getLearnRateFactor(layer,"Bias");
        end

        function value = get.WeightL2Factor(layer)
            value = getL2Factor(layer,"Weights");
        end

        function value = get.BiasL2Factor(layer)
            value = getL2Factor(layer,"Bias");
        end

        function layer = set.Weights(layer, value)
            if ~isempty(value)
                patchSize = layer.PatchSize;
                numDims = ndims(value);
                if isscalar(patchSize)
                    layer.PatchSize = repmat(patchSize, 1, numDims-2);
                end
                numChannels = layer.PrivateInputSize;
                if isempty(numChannels)
                    numChannels = NaN;
                end
                expWeightSize = [layer.PatchSize, numChannels, layer.OutputSize];
                value = iValidateLearnable(value, expWeightSize);
                layer.NumSpatialDims = numel(layer.PatchSize);
                layer.PrivateInputSize = size(value, numDims-1);
            else
                layer.NumSpatialDims = [];
                layer.PrivateInputSize = [];
            end
            layer.PrivateWeights = value;
        end

        function layer = set.Bias(layer, value)
            if ~isempty(value)
                expSize = [layer.OutputSize, 1];
                value = iValidateLearnable(value, expSize);
            end
            layer.PrivateBias = value;
        end

        function layer = set.WeightsInitializer(layer, value)
            validValues = {'narrow-normal','glorot','he','orthogonal','zeros','ones'};
            value = iAssertValidInitializer(value, validValues, 'WeightsInitializer');
            if isempty(layer.NumSpatialDims)
                % The number of spatial dimensions is required to create
                % the private weights initializer. Create a dummy version
                % for now.
                layer.PrivateWeightsInitializer = struct('Name', value);
            else
                layer = setPrivateWeightsInitializer(layer, value);
            end
        end

        function layer = set.BiasInitializer(layer, value)
            validValues = {'narrow-normal','zeros','ones'};
            value = iAssertValidInitializer(value, validValues, 'BiasInitializer');
            layer.PrivateBiasInitializer = iInitializerFactory(value);
        end

        function layer = set.WeightLearnRateFactor(layer,value)
            iAssertValidFactor(value,'WeightLearnRateFactor')
            layer = setLearnRateFactor(layer,"Weights",value);
        end

        function layer = set.BiasLearnRateFactor(layer,value)
            iAssertValidFactor(value,'BiasLearnRateFactor')
            layer = setLearnRateFactor(layer,"Bias",value);
        end

        function layer = set.WeightL2Factor(layer,value)
            iAssertValidFactor(value,'WeightL2Factor')
            layer = setL2Factor(layer,"Weights",value);
        end

        function layer = set.BiasL2Factor(layer,value)
            iAssertValidFactor(value,'BiasL2Factor')
            layer = setL2Factor(layer,"Bias",value);
        end
    end

    methods(Access=protected)
        function [description, type] = getOneLineDisplay(layer)
            outputSizeStr = int2str(layer.OutputSize);
            patchSize = layer.PatchSize;
            if isscalar(patchSize)
                patchSizeStr = int2str(patchSize);
            else
                patchSizeStr = "["+num2str(patchSize, "  %g")+"]";
            end
            description = iGetMessageString('vision:patchembed:oneLineDisplay', ...
                patchSizeStr, outputSizeStr);
            type = iGetMessageString('vision:patchembed:Type');
        end
    end

    methods(Access=private)
        function [numSpatialDims, numChannels] = validateNumSpatialAndChannelDims(~, X)
            dimS = finddim(X,'S');
            numChannels = X.Size(finddim(X,'C'));
            if isempty(dimS) || any(X.Size(dimS)==0)
                throwAsCaller(MException(message('vision:patchembed:invalidSpatialDimension')));
            end
            if isempty(numChannels) || isequal(numChannels, 0)
                throwAsCaller(MException(message('vision:patchembed:invalidChannelDimension')));
            end
            numSpatialDims = numel(dimS);
        end

        function layer = setPrivateWeightsInitializer(layer, value)
            % Create the initializer with in and out indices of the
            % weights only if the number of spatial dimensions is
            % known. For example, for 2D patches:
            % weights =
            % PatchSize(1)-by-PatchSize(2)-by-InputSize-by-OutputSize;
            % inIdx = [1 2 3]; 
            % outIdx = [1 2 4];
            numSpatialDims = layer.NumSpatialDims;
            inIdx = 1:numSpatialDims+1;
            outIndx = [1:numSpatialDims, numSpatialDims+2];
            layer.PrivateWeightsInitializer = iInitializerFactory(...
            value, inIdx, outIndx);
        end
    end

    methods(Hidden, Static)
        function layer = loadobj(in)
            if in.Version <= 1
                in = iUpgradeVersionOneToVersionTwo(in);
            end
            layer = nnet.cnn.layer.PatchEmbeddingLayer(in.Name, in.PatchSize, in.OutputSize, in.SpatialFlattenMode);

            layer.Weights = in.Weights;
            layer.Bias = in.Bias;
            layer.WeightsInitializer = in.WeightsInitializer;
            layer.BiasInitializer = in.BiasInitializer;
            layer.WeightLearnRateFactor = in.WeightLearnRateFactor;
            layer.BiasLearnRateFactor = in.BiasLearnRateFactor;
            layer.WeightL2Factor = in.WeightL2Factor;
            layer.BiasL2Factor = in.BiasL2Factor;
            layer.PrivateInputSize = in.PrivateInputSize;
        end

        function n = matlabCodegenRedirect(~)
            n = 'nnet.cnn.layer.coder.PatchEmbeddingLayer';
        end
    end
end

function messageString = iGetMessageString(varargin)
messageString = getString(message(varargin{:}));
end

function value = iValidateLearnable(value, expectedSize)
try
    validateattributes(value, {'single', 'double'}, {'size', expectedSize, 'real', 'nonsparse'});
catch exception
    throwAsCaller(exception)
end
end

function iAssertValidFactor(value,factorName)
try
    validateattributes(value, {'numeric'}, {'scalar','real','finite','nonnegative'},...
        '', factorName);
catch exception
    throwAsCaller(exception)
end
end

function value = iAssertValidInitializer(value, validValues, name)
try
    validateattributes(value, {'function_handle','char','string'}, {}, '', name);
    if(ischar(value) || isstring(value))
        value = validatestring(value, validValues, '', name);
    end
catch exception
    throwAsCaller(exception)
end
end

function initializer = iInitializerFactory(varargin)
initializer = nnet.internal.cnn.layer.learnable.initializer.initializerFactory(varargin{:});
end

function tf = iIsCustomInitializer(init)
tf = nnet.internal.cnn.layer.learnable.initializer.util.isCustomInitializer(init);
end

function iAssertValidNumSpatialDimensions(numSpatialDims, numelPatchSize)
if ~isequal(numSpatialDims, numelPatchSize)
    throwAsCaller(MException(message('vision:patchembed:numSpatialDimsMismatch', numSpatialDims, numelPatchSize)));
end
end

function iAssertValidNumChannels(inputSize, numChannels)
if ~isequal(inputSize, numChannels)
    if isempty(numChannels)
        numChannels = 0;
    end
    throwAsCaller(MException(message('vision:patchembed:invalidNumChannels', numChannels, inputSize)));
end
end

function S = iUpgradeVersionOneToVersionTwo(S)
S.Version = 2;
% Add SpatialFlattenMode and set it to the default value
S.SpatialFlattenMode = 'column-major';
end
