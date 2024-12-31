classdef ROIAveragePooling2DHostStrategy < nnet.internal.cnn.layer.util.ExecutionStrategy
    % ROIMaxPooling2DHostStrategy  Execution strategy for running roi max
    % pooling on the Host.
    
    %   Copyright 2016-2017 The MathWorks, Inc.
    
    methods
        function [Z, memory] = forward(~, X, roi, gridSize)
            Z = visionROIAvgPoolingForward(single(X),single(roi'),gridSize(1),gridSize(2));           
            memory.roi = roi;
            
        end
        
        function [dX, dW] = backward(~, X, Z, dZ, memory, ~)                             
            roi = memory.roi;
            dX = visionROIAvgPoolingBackward(single(X),single(roi'),single(Z),single(dZ));
            dW = []; % no learnable params.
        end
    end
end
