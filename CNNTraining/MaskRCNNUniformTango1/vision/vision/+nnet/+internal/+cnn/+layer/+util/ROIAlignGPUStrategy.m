classdef ROIAlignGPUStrategy < nnet.internal.cnn.layer.util.ExecutionStrategy
    % ROIAlignHostStrategy  Execution strategy for running roi align
    % on the GPU.
    
    %   Copyright 2020-2021 The MathWorks, Inc.
    
    methods
        function [Z, memory] = forward(~, X, roi, gridSize, samplingRatio)
            X = gather(X);
            roi = gather(roi);
            gridSize = gather(gridSize);
            samplingRatio = gather(samplingRatio);
            
            Z = visiongpuROIAlignForward(gpuArray(single(X)),double(roi),...
                                    double(gridSize(1)),double(gridSize(2)), single(samplingRatio));
            memory.roi = roi;
            memory.inputSize = size(X);
        end
        
        function [dX, dW] = backward(~, dZ, samplingRatio, memory, ~)   
            samplingRatio = gather(samplingRatio);
            roi = memory.roi;
            inputSize = memory.inputSize;
            dX = visiongpuROIAlignBackward(single(inputSize),double(roi),gpuArray(single(dZ)), single(samplingRatio));
            dW = []; % no learnable parameters.
        end
    end
end