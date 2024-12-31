classdef ROIMaxPooling2DHostStrategy < nnet.internal.cnn.layer.util.ExecutionStrategy
    % ROIMaxPooling2DHostStrategy  Execution strategy for running roi max
    % pooling on the Host.
    
    %   Copyright 2016-2018 The MathWorks, Inc.
    
    methods
        function [Z, memory] = forward(~, X, roi, gridSize)
            % Host processing requires data on the CPU.
            X = gather(X);
            roi = gather(roi);
            
            Z = visionROIMaxPoolingForward(single(X),single(roi'),gridSize(1),gridSize(2));
            memory.roi = roi;
        end
        
        function [dX, dW] = backward(~, X, Z, dZ, memory, ~)   
            % Host processing requires data on the CPU.
            X = gather(X);
            
            roi = memory.roi;
            dX = visionROIMaxPoolingBackward(single(X),single(roi'),single(Z),single(dZ));
            dW = []; % no learnable parameters.
        end
    end
end