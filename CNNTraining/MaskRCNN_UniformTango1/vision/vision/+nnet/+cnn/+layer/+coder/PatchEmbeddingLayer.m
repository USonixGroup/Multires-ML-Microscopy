classdef PatchEmbeddingLayer < nnet.layer.Layer  & nnet.layer.Formattable
    % PatchEmbeddingLayer   Patch embedding for codegen

    %   Copyright 2023 The MathWorks, Inc.

    %#codegen

    properties (SetAccess=private)
        PatchSize
        OutputSize
        SpatialFlattenMode
    end
    
    properties
        Weights
        Bias
    end

    methods
        function this = PatchEmbeddingLayer(name, patchSize, outputSize, flattenMode, weights, bias)
            this.Name = name;
            this.PatchSize = patchSize;
            this.OutputSize = outputSize;
            this.SpatialFlattenMode = flattenMode;
            this.Weights = weights;
            this.Bias = bias;
        end

        function X = predict(this, X)
            coder.allowpcode('plain');

            numSpatialDims = numel(finddim(X,'S'));
            numDims = coder.internal.ndims(X);
            dimLabels = dims(X);

            % Extract patches and project each patch to output size using
            % dlconv. However code generation for dlconv errors for
            % spatial-temporal data with variable size time dimension.
            % Throw a more suitable error for now
            coder.internal.assert(coder.internal.isConst(size(X, finddim(X,'T'))), 'vision:patchembed:UnsupportedPatchEmbedTimeDimension');

            X = dlconv(X, this.Weights, this.Bias, Stride = this.PatchSize);
            
            % Flatten the spatial dimensions
            if coder.const(strcmp(this.SpatialFlattenMode, 'column-major'))
                X = reshape(X, [prod(size(X, 1:numSpatialDims)), size(X, numSpatialDims+1:numDims)]);
            else
                % Row-major spatial flattening
                spatialIndices = 1:numSpatialDims;
                otherIndices = numSpatialDims+1:numDims;
                X = permute(X, [flip(spatialIndices), otherIndices]);
                X = reshape(X, [prod(size(X, spatialIndices)), size(X, otherIndices)]);
            end

            % Return dimension labels with one 'S' label
            X = dlarray(X, dimLabels(numSpatialDims:end));
        end

    end

    methods (Static)
        function cgObj = matlabCodegenToRedirected(mlObj)
            cgObj = nnet.cnn.layer.coder.PatchEmbeddingLayer( ...
                mlObj.Name, ...
                mlObj.PatchSize, ...
                mlObj.OutputSize, ...
                mlObj.SpatialFlattenMode, ...
                mlObj.Weights, ...
                mlObj.Bias ...
                );
        end

        function mlObj = matlabCodegenFromRedirected(cgObj)
            mlObj = patchEmbeddingLayer(cgObj.PatchSize, cgObj.OutputSize, ...
                SpatialFlattenMode = cgObj.SpatialFlattenMode, ...
                Name = cgObj.Name, ...
                Weights = cgObj.Weights, ...
                Bias = cgObj.Bias ...
                );
        end
    end

    methods(Static, Hidden)
        function n = matlabCodegenNontunableProperties(~)
            n = {'PatchSize', 'OutputSize', 'SpatialFlattenMode'};

            if ~dlcoderfeature('RuntimeLoad')
                % Learnables are non-tunable when RuntimeLoad is not
                % enabled
                n = [n {'Weights', 'Bias'}];
            end
        end

        function matlabCodegenValidate(layer, validator)
            % Code generation for dlconv (used in predict) only supports up
            % to two spatial dimensions
            supportedNumSpatialDims = 0:2;
            dltargets.internal.utils.checkSpatialDims(layer, validator, supportedNumSpatialDims);
        end

    end
end
