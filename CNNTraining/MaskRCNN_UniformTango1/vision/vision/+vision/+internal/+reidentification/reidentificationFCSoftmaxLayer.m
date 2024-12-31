classdef reidentificationFCSoftmaxLayer < nnet.layer.Layer
%

%   Copyright 2023 The MathWorks, Inc.

    properties
        NormalizedWeights % Normalized weights matrix
        LossFunction      % Used to handle data, weights, and logits before softmax
        InFormat          % Character array indicating the input format
        CBIdx             % Input indices corresponding to channel and batch
    end

    properties(Learnable)
        Weights           % Weights matrix
        K                 % Cosine softmax scaling parameter
    end

    methods
        function layer = reidentificationFCSoftmaxLayer(name, weights, lossFunction, inFormat)
            narginchk(3,4)
            layer.Name = name;
            layer.Description = "Custom layer that normalizes and takes the dot product of weights and feature vector.";
            layer.Weights = weights;
            layer.K = 1;
            layer.LossFunction = lossFunction;

            if ~strcmp(inFormat,"CB")
                [~,channelDim] = find(inFormat == 'C');
                [~,batchDim] = find(inFormat == 'B');
                dimSize = 1:length(size(inFormat));
                [~, otherIndices] = find(dimSize ~= channelDim & dimSize ~= batchDim);
                layer.CBIdx = [channelDim batchDim otherIndices];
            else
                layer.CBIdx = [1 2];
            end
            layer.InFormat = inFormat;
        end

        function layer = normalizeWeights(layer)
            % Normalize weights to unit-length.
            layer.Weights(layer.Weights==0) = eps;
            layer.NormalizedWeights = layer.Weights ./ vecnorm(layer.Weights,2,2);
        end

        function Z = predict(layer, X)
            X(X==0) = eps;

            % Rearrange the input to be C-by-B.
            X = permute(X,layer.CBIdx);
            X = squeeze(X);

            if ~strcmp(layer.LossFunction, "cross-entropy")
                % Normalize weights.
                layer = normalizeWeights(layer);
                X = X ./ vecnorm(X,2,1);
            else
                % For normal softmax, do not normalize the weights or
                % feature vector.
                layer.Weights(layer.Weights==0) = eps;
                layer.NormalizedWeights = layer.Weights;
            end

            % Obtain logits and subtract largest value for stability.
            logits = layer.NormalizedWeights * X;
            logits = logits - max(logits,[],1);

            switch layer.LossFunction
                case "cross-entropy"
                    Z = exp(logits) ./ sum(exp(logits),1);
                case "cosine-softmax"
                    % Loss defined in:
                    % Wojke, N. and Bewley, A., "Deep Cosine Metric Learning for Person Re-Identification”

                    % Scale logits to better define and separate the hypersphere of classes.
                    cosSim = layer.K .* logits;
                    Z = exp(cosSim) ./ sum(exp(cosSim),1);
                case "additive-margin-softmax"
                    % Loss define in:
                    % Wang, F., Cheng, J., Liu, W., and Liu, H., “Additive Margin Softmax for Face Verification”

                    % Send out purely normalized logits. Handle the target
                    % logits margin subtraction in the loss function.
                    Z = logits;
                otherwise
                    Z = exp(logits) ./ sum(exp(logits),1);
            end

        end
    end
end
