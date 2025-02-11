classdef ROIMaxPooling2DGPUStrategy < nnet.internal.cnn.layer.util.ExecutionStrategy
    % ROIMaxPooling2DGPUStrategy  Execution strategy for running roi max pooling on the GPU
    
    %   Copyright 2016-2018 The MathWorks, Inc.
    
    methods
        function [Z, memory] = forward(~, X, roi, gridSize)
            % On the GPU the number of channels are limited to the CUDA max
            % grid dim size of 65535.
            if size(X,3) > 65535
                error(message('vision:rcnn:unsupportedNumChannelsForROIPooling'));
            end
            
            % ROI data must be on the Host even for GPU processing.
            roi = gather(roi);
            
            % Transpose into 5xM format to allow contigous access.
            roiTransposed = roi'; 
            
            % GPU processing requires data on the GPU. Datatype must be
            % single.
            X = gpuArray(single(X));
                                    
            Z = visiongpuROIMaxPoolingForward(X, double(roiTransposed), gridSize(1), gridSize(2));

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
            
            % ROI data in memory is already on the Host.
            roi = memory.roi;
            roiTransposed = roi'; % use 5xM to allow contigous access.
            
            % GPU processing requires data on the GPU. Datatype must be
            % single.
            X  = gpuArray(single(X));
            Z  = gpuArray(single(Z));
            dZ = gpuArray(single(dZ));
            
            dX = visiongpuROIMaxPoolingBackward(X, Z, dZ, double(roiTransposed), gridSize(1), gridSize(2));
            
            dW = []; % no learnable params.
        end

    end
end
