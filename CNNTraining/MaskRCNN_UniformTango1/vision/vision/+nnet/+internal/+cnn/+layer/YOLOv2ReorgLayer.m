classdef YOLOv2ReorgLayer < nnet.internal.cnn.layer.Layer
    % Internal layer for YOLOv2ReorgLayer.
    
    %   Copyright 2020-2023 The MathWorks, Inc.
    
    properties
        % LearnableParameters   Learnable parameters for the layer.
        %   This layer has no learnable parameters.
        LearnableParameters = nnet.internal.cnn.layer.learnable.PredictionLearnableParameter.empty();
        
        % Name (char array)   A name for the layer.
        Name
    end
    
    properties (Constant)
        % DefaultName   Default layer's name.
        DefaultName = 'spaceToDepth'
    end
    
    properties (SetAccess = private)
        % InputNames   This layer has one input.
        InputNames = {'in'}
        
        % OutputNames   This layer has one output.
        OutputNames = {'out'}
        
        % HasSizeDetermined   True for layers with BlockSize determined.
        HasSizeDetermined = true;
        
        % BlockSize    Reorder input activations to required output.
        BlockSize
    end
    
    properties(Access = protected)
        ExecutionStrategy
    end
    
    properties
        % Learnables   Empty
        Learnables
    end
    
    properties (SetAccess = protected)
        % IsInFunctionalMode   Returns true if layer is currently being
        % used in "functional" mode (i.e. in dlnetwork). Required by
        % FunctionalLayer interface. On construction, all layers are set up
        % for usage in DAGNetwork.
        IsInFunctionalMode = false
    end
    
    properties(SetAccess=protected)
        % LearnablesName   Empty
        LearnablesNames
    end
    
    methods
        function this = YOLOv2ReorgLayer(name, blockSize)
            this.Name = name;
            this.BlockSize = blockSize;
            this.ExecutionStrategy = nnet.internal.cnn.layer.util.spaceToDepthHostStrategy();
        end
        
        function Z = predict(this, X)
            Z = this.ExecutionStrategy.forward(X, this.BlockSize);
        end
        
        function [dX, dW] = backward(this, X, Z, dZ, ~)
            [dX, dW] = this.ExecutionStrategy.backward(X, Z, dZ, this.BlockSize);
        end

        function outputSize = forwardPropagateSize(this, inputSize)
            height = floor(inputSize(1)/this.BlockSize(1));
            width = floor(inputSize(2)/this.BlockSize(2));
            numChannels = inputSize(3)*this.BlockSize(1)*this.BlockSize(2);
            outputSize = [height,width,numChannels];
        end
        
        function this = inferSize(this, ~)
            % no-op
        end
        
        function tf = isValidInputSize(this, inSize)
            % isValidInputSize Check if the layer can accept an input of
            % a certain size.
            % Input must be at least the size of BlockSize and should have
            % three elements.
            tf = (numel(inSize)== 3) && all(inSize(1:2) >= this.BlockSize);
        end
        
        function this = initializeLearnableParameters(this, ~)
            % no-op
        end
        
        function this = prepareForTraining(this)
            this.LearnableParameters = nnet.internal.cnn.layer.learnable.TrainingLearnableParameter.empty();
        end
        
        function this = prepareForPrediction(this)
            this.LearnableParameters = nnet.internal.cnn.layer.learnable.PredictionLearnableParameter.empty();
        end
        
        function this = setupForHostPrediction(this)
            this.ExecutionStrategy = nnet.internal.cnn.layer.util.spaceToDepthHostStrategy();
        end
        
        function this = setupForGPUPrediction(this)
            this.ExecutionStrategy = nnet.internal.cnn.layer.util.spaceToDepthGPUStrategy();
        end
        
        function this = setupForHostTraining(this)
            this.ExecutionStrategy = nnet.internal.cnn.layer.util.spaceToDepthHostStrategy();
        end
        
        function this = setupForGPUTraining(this)
            this.ExecutionStrategy = nnet.internal.cnn.layer.util.spaceToDepthGPUStrategy();
        end
    end  
    
end
