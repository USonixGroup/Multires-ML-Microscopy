classdef FrozenBatchNormalization < nnet.internal.cnn.layer.Layer
    % This layer is used as a replacement for a batch normalization layer
    % when we wish to freeze a batch normalization layer. Freezing the BN
    % layer is best practice when the mini-batch size is low. For small
    % batch sizes, the mean and variance estimate are not reliable.
    
    % Copyright 2019 The MathWorks, Inc.
    
    properties
        % NormalizationFactor Precomputed normalization factor.
        NormalizationFactor
        
        % BatchNormalizationLayer Original batch normalization layer.
        BatchNormalizationLayer
        
        % LearnableParameters   Learnable parameters for the layer
        % (Vector of nnet.internal.cnn.layer.LearnableParameter)
        LearnableParameters = nnet.internal.cnn.layer.learnable.PredictionLearnableParameter.empty();
        
        % Name (char array)
        Name
    end
    
     properties (SetAccess = private)
        % InputNames   The names of the inputs for this layer
        %   This should be defined as a row cell array of char arrays. If
        %   there is only one input, then it should still be in a cell.
        %   Note that the ordering of the names determines the order of the
        %   inputs.
        InputNames = {'in'};
        
        % OutputNames   The names of the outputs for this layer
        %   This should be defined as a row cell array of char arrays. If
        %   there is only one output, then it should still be in a cell.
        %   Note that the ordering of the names determines the order of the
        %   outputs.
        OutputNames = {'out'};
        
        % HasSizeDetermined   True for layers with size determined.
        HasSizeDetermined = true
    end
    
    properties (Constant)
        % DefaultName   Default layer's name. This will be assigned in case
        % the user leaves an empty name.
        DefaultName = 'frozenBN';
    end
    
    properties        
        % Alpha term in X.*Alpha + Beta
        Alpha
        
        % Beta term in X.*Alpha + Beta
        Beta
        
        % Strategy to use for moving data to appropriate device. 
        DeviceStrategy
    end
   
    
    methods
        function this = FrozenBatchNormalization(bnLayer)
            this.BatchNormalizationLayer = bnLayer;
            this.NormalizationFactor = sqrt(bnLayer.TrainedVariance + bnLayer.Epsilon);
            
            m = this.BatchNormalizationLayer.TrainedMean;
            n = this.NormalizationFactor;
            
            s = this.BatchNormalizationLayer.Scale;
            b = this.BatchNormalizationLayer.Offset;
            
            % Batch norm output is computed as:
            %   Z = (X-m)./n
            %   Z = Z .* s + b;
            %
            % Roll up terms and use Z = X.*A + B form for efficiency.
            this.Alpha = nnet.internal.cnn.layer.util.CachedParameter(s./n);
            this.Beta =  nnet.internal.cnn.layer.util.CachedParameter(-m.*this.Alpha.Value + b);
            
            % Default CachedParameter UseGPU flag to false (host).
            this = this.updateUseGPUFlag(false);
            
            this.DeviceStrategy = vision.internal.cnn.layer.util.ToHostStrategy();
            
            this.NeedsXForBackward = false;
            this.NeedsZForBackward = false;
        end
        
        function Z = predict(this, X)       
            X = this.DeviceStrategy.toDevice(single(X));
            Z = X .* this.Alpha.Value + this.Beta.Value;          
        end
        
        function dLdX = backward(this,~,~,dLdZ,~)
            dLdZ = this.DeviceStrategy.toDevice(single(dLdZ));
            dLdX = this.Alpha.Value .* dLdZ;
        end
                
        % forwardPropagateSize    The size of the output from the layer for
        % a given size of input
        function outputSize = forwardPropagateSize(~, inputSize)
            outputSize = inputSize;
        end
        
        % inferSize    Infer the size of the learnable parameters based
        % on the input size
        function this = inferSize(this, ~)
            % no-op
        end
        
        % isValidInputSize   Check if the layer can accept an input of a
        % certain size
        function tf = isValidInputSize(~,~)
            tf = true;
        end
        
        % initializeLearnableParameters    Initialize learnable parameters
        % using their initializer
        function this = initializeLearnableParameters(this, ~)
            % no-op
        end
        
        % prepareForTraining   Prepare the layer for training
        function this = prepareForTraining(this)
            this.LearnableParameters = nnet.internal.cnn.layer.learnable.TrainingLearnableParameter.empty();
        end
        
        % prepareForPrediction   Prepare the layer for prediction
        function this = prepareForPrediction(this)
            this.LearnableParameters = nnet.internal.cnn.layer.learnable.PredictionLearnableParameter.empty();
        end
        
        % setupForHostPrediction   Prepare this layer for host prediction
        function this = setupForHostPrediction(this)
            this.DeviceStrategy = vision.internal.cnn.layer.util.ToHostStrategy();
            this = this.updateUseGPUFlag(false);
        end
        
        % setupForGPUPrediction   Prepare this layer for GPU prediction
        function this = setupForGPUPrediction(this)
            this.DeviceStrategy = vision.internal.cnn.layer.util.ToGPUStrategy();
            this = this.updateUseGPUFlag(true);
        end
        
        % setupForHostTraining   Prepare this layer for host training
        function this = setupForHostTraining(this)
            this.DeviceStrategy = vision.internal.cnn.layer.util.ToHostStrategy();
            this = this.updateUseGPUFlag(false);
        end
        
        % setupForGPUTraining   Prepare this layer for GPU training
        function this = setupForGPUTraining(this)
            this.DeviceStrategy = vision.internal.cnn.layer.util.ToGPUStrategy();
            this = this.updateUseGPUFlag(true);
        end
        
    end
    
    methods(Access = private)
        
        % Set UseGPU flag on CachedParameters
        function this = updateUseGPUFlag(this,tf)
            this.Alpha.UseGPU = tf;
            this.Beta.UseGPU = tf;
        end
    end
       
end