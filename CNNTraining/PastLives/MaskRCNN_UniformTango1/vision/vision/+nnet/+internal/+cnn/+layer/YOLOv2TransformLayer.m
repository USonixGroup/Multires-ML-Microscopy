classdef YOLOv2TransformLayer < nnet.internal.cnn.layer.Layer
    %   YOLOv2TransformLayer Internal representation of the YOLO v2 Transform layer.

    %   Copyright 2018-2020 The MathWorks, Inc.
    
    properties
        % LearnableParameters   Learnable parameters for the layer
        %   This layer has no learnable parameters.
        LearnableParameters = nnet.internal.cnn.layer.learnable.PredictionLearnableParameter.empty();
        
        % Name (char array)   A name for the layer.
        Name         
    end
    
    properties (Constant)
        % DefaultName   Default layer's name.
        DefaultName = 'yolov2Transform'
    end
    
    properties (SetAccess = private)
        % InputNames   This layer has one input.
        InputNames = {'in'}
        OutputNames = {'out'}
        % HasSizeDetermined     True for layer if number of anchor boxes are
        %                       determined.
        HasSizeDetermined = true;
        % NumAnchorBoxes    Number of anchor boxes to transform input
        %                   activation.
        NumAnchorBoxes
    end
    
    properties(Access = protected)
        ExecutionStrategy        
    end              
    
    methods
        function this = YOLOv2TransformLayer(name,numAnchorBoxes)
            this.Name = name;
            this.NumAnchorBoxes = numAnchorBoxes;                                 
            this.ExecutionStrategy = nnet.internal.cnn.layer.util.YOLOv2TransformHostStrategy();
        end
        
        function Z = predict(this, X)             
                Z = this.ExecutionStrategy.forward(X ,this.NumAnchorBoxes);
        end
        
        function [dX, dW] = backward(this,X, Z, dZ,~)
                [dX, dW] = this.ExecutionStrategy.backward( X, Z, dZ,this);                
        end

        function outputsize = forwardPropagateSize(~,inputsize)
            outputsize = inputsize;
        end
        
        function this = inferSize(this, ~)
            % no-op
        end
        
        function tf = isValidInputSize(~, inputSize)
            tf = iNonEmptyMatrix(inputSize(1:2)) ...
                && numel(inputSize)== 3;
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
            this.ExecutionStrategy = nnet.internal.cnn.layer.util.YOLOv2TransformHostStrategy();
        end
        
        function this = setupForGPUPrediction(this)
            this.ExecutionStrategy = nnet.internal.cnn.layer.util.YOLOv2TransformHostStrategy();
        end
        
        function this = setupForHostTraining(this)
            this.ExecutionStrategy = nnet.internal.cnn.layer.util.YOLOv2TransformHostStrategy();
        end
        
        function this = setupForGPUTraining(this)
            this.ExecutionStrategy = nnet.internal.cnn.layer.util.YOLOv2TransformHostStrategy();
        end
    end
    
end

function tf = iNonEmptyMatrix(inputSize)
tf = all(inputSize > 0);
end

