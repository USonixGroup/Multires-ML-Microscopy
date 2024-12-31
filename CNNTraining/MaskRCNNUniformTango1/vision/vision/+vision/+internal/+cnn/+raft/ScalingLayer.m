classdef ScalingLayer < nnet.layer.Layer ...
        & nnet.layer.Acceleratable
    % Internal CVT layer for custom scaling operations in the RAFT
    % dlnetworks imported from PyTorch

    properties
        Scale = 1
        Bias = 0
    end

    methods
        function layer = ScalingLayer(options)
            arguments
                options.Name (1,1) string = "scaling"
                options.Scale (1,1) {mustBeNumeric} = 1
                options.Bias (1,1) {mustBeNumeric} = 0
            end
            layer.Name = options.Name;
            layer.Description = layer.Name;
            layer.Scale = options.Scale;
            layer.Bias = options.Bias;
            layer.Type = "vision.internal.cnn.raft.ScalingLayer";
        end

        function [Z] = predict(layer,X)
            % Forward input data through the layer at prediction time and
            % output the result
            scale = cast(layer.Scale, 'like', X);
            bias = cast(layer.Bias, 'like', X);
            Z = X.*scale + bias;
        end

    end
end