classdef RPNSoftmaxLayer < nnet.internal.cnn.layer.Layer
    % RPNSoftmaxLayer   Implementation of the RPN softmax layer
    
    %   Copyright 2018 The MathWorks, Inc.
    
    properties
        % LearnableParameters   Learnable parameters for the layer
        %   This layer has no learnable parameters.
        LearnableParameters = nnet.internal.cnn.layer.learnable.PredictionLearnableParameter.empty();

        % Name (char array)   A name for the layer
        Name               
    end
    
    properties (Constant)
        % DefaultName   Default layer's name.
        DefaultName = 'rpnsoftmax'
    end
            
    properties(SetAccess = private)
        % InputNames   This layer has a single input
        InputNames = {'in'}
        
        % OutputNames   This layer has a single output
        OutputNames = {'out'}
        
        % HasSizeDetermined   Specifies if all size parameters are set
        HasSizeDetermined = true
        
        % ExecutionStrategy   The execution strategy for this layer
        ExecutionStrategy
    end
    
    methods
        function this = RPNSoftmaxLayer(name)
            this.Name = name;
            
            this.ExecutionStrategy = nnet.internal.cnn.layer.util.SoftmaxHostDAGStrategy();
        end
        
        function Z = predict(this, X)   
            
            % Reshape X from [H W 2*numAnchors] to [H*W numAnchors 2]. In
            % RPN, 2 represents to 2 classes: object and background. The
            % following layer, RPNClassificationLayer will apply the loss
            % over H, W, and numAnchors.
            [H, W, numAnchorsTimesTwo, numObservations] = size(X);            
            X = reshape(X, H*W, numAnchorsTimesTwo/2, 2, numObservations );
            Z = this.ExecutionStrategy.forward(X, 3);
            
            % Reshape X from [H W 2*numAnchors] to [H W 2 numAnchors]. In
            % RPN, 2 represents to 2 classes: object and background. The
            % following layer, RPNClassificationLayer will apply the loss
            % over H, W, and numAnchors.
            %sz = size(X);            
            %X = reshape(X, sz(1),sz(2), 2, []);
            %Z = this.ExecutionStrategy.forward(X);
        end
        
        function [dX,dW] = backward(this, X, Z, dZ, ~)            
            [dX,dW] = this.ExecutionStrategy.backward(Z, dZ, 3);            
            [H, W, numAnchorsTimesTwo, numObservations] = size(X);
            dX = reshape(dX, H, W, numAnchorsTimesTwo, numObservations);
        end
        
        function outputSize = forwardPropagateSize(~, inputSize)            
            outputSize = [inputSize(1)*inputSize(2) inputSize(3)/2 2];
        end
        
        function this = inferSize(this, ~)           
            
        end
        
        function tf = isValidInputSize(~, inputSize)
            % isValidInputSize True if valid size is [H W C], where C is
            % even.            
            tf = iNonEmptyMatrix(inputSize(1:2)) ...
                && numel(inputSize)== 3 ...
                && mod(inputSize(3),2) == 0;
        end
        
        function this = initializeLearnableParameters(this, ~)
        end
        
        function this = prepareForTraining(this)
            this.LearnableParameters = nnet.internal.cnn.layer.learnable.TrainingLearnableParameter.empty();
        end
        
        function this = prepareForPrediction(this)
            this.LearnableParameters = nnet.internal.cnn.layer.learnable.PredictionLearnableParameter.empty();
        end
        
        function this = setupForHostPrediction(this)
            this.ExecutionStrategy = nnet.internal.cnn.layer.util.SoftmaxHostDAGStrategy;
        end
        
        function this = setupForGPUPrediction(this)
            this.ExecutionStrategy = nnet.internal.cnn.layer.util.SoftmaxGPUDAGStrategy();
        end
        
        function this = setupForHostTraining(this)
            this.ExecutionStrategy = nnet.internal.cnn.layer.util.SoftmaxHostDAGStrategy;
        end
        
        function this = setupForGPUTraining(this)
            this.ExecutionStrategy = nnet.internal.cnn.layer.util.SoftmaxGPUDAGStrategy();
        end
        
    end    
end

function tf = iNonEmptyMatrix(inputSize)
tf = all(inputSize > 0);
end
