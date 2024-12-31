classdef spaceToDepthGPUStrategy < nnet.internal.cnn.layer.util.ExecutionStrategy
    % spaceToDepthHostStrategy  Execution strategy for running space to 
    % depth permute algorithm. According to space to depth algorithm, input 
    % activation is reordered according to block size. Elements in input 
    % activations are reordered according to values of block size and 
    % additional elements are stored in new set of channels. The 
    % height and width of input activations are divided by value of block size 
    % and channels size is multiplied by block size value i.e output activation
    % size would be [floor(height/blockSize(1)), floor(width/blockSize(2)), 
    % channels*blockSize(1)*blockSize(2)] where [height, width, channels] 
    % correspond to size of input activations.
    
    %   Copyright 2020 The MathWorks, Inc.

    methods
        function Z = forward(~, X, blockSize)
            % In forward pass activations are resized as per the block size.
            % Input width and height are divided by block size and channels are
            % multiplied by block size.
            X = gpuArray(X);
            Z = visiongpuSpaceToDepthLayerForward(X, blockSize(2), blockSize(1));
        end

        function [dX, dW] = backward(~, X, Z, dZ, blockSize)
            % In backward pass feature maps are reordered back to their
            % original size. X is required to know the original size.
            [outH, outW, ~, ~] = size(X);
            dZ = gpuArray(dZ);
            dX = visiongpuSpaceToDepthLayerBackward(dZ, outH, outW, blockSize(2), blockSize(1));

            dW = []; % no learnable parameters.
        end
    end
end
