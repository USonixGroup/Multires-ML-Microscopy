classdef ROIAveragePooling2DGPUStrategy < nnet.internal.cnn.layer.util.ExecutionStrategy
    % ROIMaxPooling2DGPUStrategy  Execution strategy for running roi max pooling on the GPU
    
    %   Copyright 2016-2017 The MathWorks, Inc.
    
    methods
        function [Z, memory] = forward(~, X, roi, gridSize)
            % On the GPU the number of channels are limited to the CUDA max
            % grid dim size of 65535.
            if size(X,3) > 65535
                error(message('vision:rcnn:unsupportedNumChannelsForROIPooling'));
            end
            
            roiTransposed = roi'; % use 4xM to allow contigous access.
            
            Z = visiongpuROIAvgPoolingForward(X, double(roiTransposed), gridSize(1), gridSize(2));

            % Cache ROI input for use in backward function.  
            memory.roi = roi;
        end             

        function [dX, dW] = backward(~, X, Z, dZ, memory, gridSize)                                    
            % Input X should be the whole image
            %
            % Input dZ is the gradient of next layer, hence it's 4 dim is the
            % number of ROIs.
            %
            % Input Z is layer output, unused, here to be compatible with 
            % the call signature backward(X, Z, dZ, roi, gridSize)
                           
            % On the GPU the number of channels are limited to the CUDA max
            % grid dim size of 65535.
            if size(X,3) > 65535
                error(message('vision:rcnn:unsupportedNumChannelsForROIPooling'));
            end
            roi = memory.roi;
            roiTransposed = roi'; % use 4xM to allow contigous access.
            X = gpuArray(X);
            dX = visiongpuROIAvgPoolingBackward(X, Z, dZ, double(roiTransposed), gridSize(1), gridSize(2));
            
            dW = []; % no learnable params.
        end
    end
end
