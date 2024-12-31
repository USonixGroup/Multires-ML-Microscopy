classdef SmoothL1Loss < nnet.internal.cnn.layer.RegressionLayer
    % Implements the Smooth L1 Regression loss defined in:
    %
    % Girshick, Ross. "Fast r-cnn." Proceedings of the IEEE International
    % Conference on Computer Vision. 2015.
    
    % Copyright 2016-2021 The MathWorks, Inc.
    
    properties(SetAccess = private)
        % InputNames   SmoothL1Loss layer has one input
        InputNames = {'in'}
        
        % OutputNames   SmoothL1Loss layer has no outputs
        OutputNames = {}
    end
    
    properties
        % LearnableParameters   Learnable parameters for the layer
        %   This layer has no learnable parameters.
        LearnableParameters = nnet.internal.cnn.layer.learnable.PredictionLearnableParameter.empty();
        
        % Name (char array)   A name for the layer
        Name
        
        % NumResponses
        NumResponses
        
        % ResponseNames Not used, but here so that an odd error isn't
        % thrown by trainNetwork.
        ResponseNames
    end
    
    properties (Constant)
        % DefaultName   Default layer's name.
        DefaultName = 'boxRegression'
    end
    
    properties (SetAccess = private)
        % HasSizeDetermined   True for layers with size determined.
        HasSizeDetermined = true;
    end
    
    properties (Access = private)
        % DeviceStrategy either moves data to the GPU or to the Host.         
        DeviceStrategy
    end
    
    methods
        
        function this = SmoothL1Loss(name, numResponses)
            this.Name = name;
            this.NumResponses = numResponses;
            if isempty(numResponses)
                this.HasSizeDetermined = false;
            else
                this.HasSizeDetermined = true;
            end
            
            this.DeviceStrategy = vision.internal.cnn.layer.util.ToHostStrategy();
            
        end
        
        function this = inferSize(this, inputSize)
            % inferSize    Infer the number of classes based on the input
            
            if ~isValidInputSize(this, inputSize)
                error(message('vision:rcnn:invalidSizeSmoothL1'));
            end
            
            this.NumResponses = inputSize(3);
            this.HasSizeDetermined = true;
        end
        
        function tf = isValidInputSize(~, inputSize)
            % isValidInputSize   Check if the layer can accept an input of
            % a certain size.
            if numel(inputSize) < 3
                tf = false;
            else
                v = inputSize(3);
                
                if v >= 4 || mod(v, 4) == 0
                    tf = true;
                else
                    tf = false;
                    % inputSize must be divisible by 4. num responses is 4 *
                    % numClasses.
                end
            end
        end
        
        function this = initializeLearnableParameters(this, ~)
            % initializeLearnableParameters     no-op since there are no
            % learnable parameters
        end
        
        function this = prepareForTraining(this)
            this.LearnableParameters = nnet.internal.cnn.layer.learnable.TrainingLearnableParameter.empty();
        end
        
        function this = prepareForPrediction(this)
            this.LearnableParameters = nnet.internal.cnn.layer.learnable.PredictionLearnableParameter.empty();
        end
        
        function this = setupForHostPrediction(this)
            this.DeviceStrategy = vision.internal.cnn.layer.util.ToHostStrategy();
        end
        
        function this = setupForGPUPrediction(this)
            this.DeviceStrategy = vision.internal.cnn.layer.util.ToGPUStrategy();
        end
        
        function this = setupForHostTraining(this)
            this.DeviceStrategy = vision.internal.cnn.layer.util.ToHostStrategy();
        end
        
        function this = setupForGPUTraining(this)
            this.DeviceStrategy = vision.internal.cnn.layer.util.ToGPUStrategy();
        end
        
        function outputSize = forwardPropagateSize(~, inputSize)
            % forwardPropagateSize  Output the size of the layer based on
            % the input size
            outputSize = inputSize;
        end
        
        % applyToDeviceFcn  Apply DeviceStrategy to Y and T.
        function [Y, T] = applyToDeviceStrategy(this, Y, T)
            Y = this.DeviceStrategy.toDevice(single(Y));
            T = cellfun(@(t)this.DeviceStrategy.toDevice(single(t)),T,'UniformOutput',0);
        end
        
        % forwardLoss    Return the loss between the output obtained from
        % the network and the expected output
        %
        % Inputs
        %   Y - the output from forward propagation thru the layer
        %   T - is a 1x2 cell array. T{1} are the target responses. T{2}
        %       are the intance weights, W.
        %
        %   Y and T are the same size.
        %
        % Outputs
        %   loss - the loss between Y and T. Set instance weights correctly
        %   to achieve average loss. For example, W = 1/N, from most cases
        %   where N is the number of observations.
        function loss = forwardLoss(this, Y, T)
            assert(~isempty(this.DeviceStrategy))
            iCheckInputs(T);
            
            [Y, T] = applyToDeviceStrategy(this, Y, T);
            
            loss = forwardLossPerSample(this, Y, T{1});
            
            loss = T{2} .* loss;
            
            loss = sum(loss(:));
        end
        
        
        % backwardLoss    Back propagate the derivative of the loss function
        %
        % Inputs
        %   anOutputLayer - the output layer to backprop the loss thru
        %   Y - the output from forward propagation thru the layer
        %   T - is a 1x2 cell array. T{1} are the target responses. T{2}
        %       are the intance weights, W.
        %
        %   Y and T are the same size.
        %
        % Outputs
        %   dX - the derivative of the loss function with respect to Y
        function dX = backwardLoss(this, Y, T )
            assert(~isempty(this.DeviceStrategy))
            iCheckInputs(T);
            
            [Y, T] = applyToDeviceStrategy(this, Y, T);
            
            dX = zeros(size(Y),'like',Y);
            
            X = Y - T{1};
            
            one = ones(1.0,'like',X);
            
            % abs(x) < 1
            idx = (X > -one) & (X < one);
            dX(idx) = X(idx);
            
            % x >= 1 || x <= 1
            dX(X >= one) = one;
            dX(X <= -one) = -one;
            
            dX = T{2} .* dX ;
            
        end
        
        function loss = forwardLossPerSample(~, Y, T)
            % Return the loss for each sample.
            X = Y - T;
            
            loss = zeros(size(X), 'like', X);
            
            one     = ones(1.0,'like',X);
            onehalf = cast(0.5,'like',X);
            
            % abs(x) < 1
            idx = (X > -one) & (X < one);
            loss(idx) = 0.5 * X(idx).^2;
            
            % x >= 1 || x <= 1
            idx = ~idx;
            loss(idx) = abs(X(idx)) - onehalf;
        end
    end
end

%--------------------------------------------------------------------------
function iCheckInputs(T)
if ~iscell(T)
    error(message('vision:rcnn:useTrainFastOrFaster','rcnnBoxRegressionLayer'));
end
end
