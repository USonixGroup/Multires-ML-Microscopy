classdef spaceToDepthHostStrategy < nnet.internal.cnn.layer.util.ExecutionStrategy
    % spaceToDepthHostStrategy  Execution strategy for running space to 
    % depth permute algorithm. According to space to depth algorithm, input 
    % activation is reordered according to block size. Elements in input 
    % activations are reordered according to values of block size and 
    % additional elements are stored in new set of channels. Hence the 
    % height and width of input activations are divided by value of BlockSize 
    % and channels size is multiplied by BlockSize value i.e output activation
    % size would be [floor(height/BlockSize(1)), floor(width/BlockSize(2)), 
    % channels*BlockSize(1)*BlockSize(2)] where [height, width, channels] 
    % correspond to size of input activations.
    
    %   Copyright 2020 The MathWorks, Inc.
    
    methods
        function Z = forward(~, X, blockSize)
            % Crop features from activations depending on block size, as
            % they cannot be retrieved during back propagation.
            [tmpInputHeight,tmpInputWidth,~,~] = size(X);
            remInputHeight = mod(tmpInputHeight,blockSize(1));
            remInputWidth = mod(tmpInputWidth,blockSize(2));
            X = X(1:(tmpInputHeight-remInputHeight),1:(tmpInputWidth-remInputWidth),:,:);
            
            % Forward input data through the layer and output the result.
            [inputHeight,inputWidth,inputChannel,batchSize] = size(X);
            outputHeight = floor(inputHeight/blockSize(1));
            outputWidth = floor(inputWidth/blockSize(2));
            outputChannel = inputChannel*(blockSize(1)*blockSize(2));
            
            %  Output feature maps are reordered according to size of input
            %  feature map and block size.
            Z = zeros([outputHeight,outputWidth,outputChannel,batchSize],'like',X);

            for idxBlockSizeX = 1:blockSize(1)
                for idxBlockSizeY = 1:blockSize(2)
                    idx = (idxBlockSizeX-1)*blockSize(2) + idxBlockSizeY;
                    val = inputChannel*(idx-1)+1:inputChannel*idx;
                    Z(:,:,val,:) = X(idxBlockSizeX:blockSize(1):size(X,1), idxBlockSizeY:blockSize(2):size(X,2),:,:);
                end
            end
        end
        
        function [dX, dW] = backward(~, X, Z, dZ, blockSize)
            % In backward pass feature maps are reordered back to their
            % original size.
            [inputHeight,inputWidth,inputChannel,~] = size(Z);
            outputHeight = (inputHeight*blockSize(1));
            outputWidth = (inputWidth*blockSize(2));
            outputChannel = inputChannel/(blockSize(1)*blockSize(2));
            dX = zeros(size(X),'like',X);
            
            for idxBlockSizeX=1:blockSize(1)
                for idxBlockSizeY=1:blockSize(2)
                    idx = (idxBlockSizeX-1)*blockSize(2) + idxBlockSizeY;
                    val = outputChannel*(idx-1)+1:outputChannel*idx;
                    dX(idxBlockSizeX:blockSize(1):outputHeight,idxBlockSizeY:blockSize(2):outputWidth,:,:)= dZ(:,:,val,:);
                end
            end
            dW = []; % no learnable parameters.
        end
    end
end
