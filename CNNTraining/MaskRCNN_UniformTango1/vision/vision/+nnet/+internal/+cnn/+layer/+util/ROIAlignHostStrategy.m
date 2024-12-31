classdef ROIAlignHostStrategy < nnet.internal.cnn.layer.util.ExecutionStrategy
    % ROIAlignHostStrategy  Execution strategy for running roi align
    % on the Host.
    
    %   Copyright 2020-2021 The MathWorks, Inc.
    
    methods
        function [Z, memory] = forward(~, X, roi, gridSize, samplingRatio)
            % Host processing requires data on the CPU.
            X = gather(X);
            roi = gather(roi);
            
            Z = visionROIAlignForward(single(X),single(roi),...
                                    gridSize(1),gridSize(2), single(samplingRatio));
            memory.roi = roi;
            memory.inputSize = size(X);
        end
        
        function [dX, dW] = backward(~, dZ, samplingRatio, memory, ~)   
            roi = memory.roi;
            inputSize = memory.inputSize;
            dX = visionROIAlignBackward(single(inputSize),single(roi),single(dZ),single(samplingRatio));
            dW = []; % no learnable parameters.
        end
    end
end