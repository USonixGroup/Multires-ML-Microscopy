classdef RPNCrossEntropy < nnet.internal.cnn.layer.ClassificationLayer
    % RPNCrossEntropy   RPN Cross entropy loss output layer
    
    %   Copyright 2015-2021 The MathWorks, Inc.
    
    properties(SetAccess = private)
        % InputNames   RPNCrossEntropy layer has one input
        InputNames = {'in'}
        
        % OutputNames   RPNCrossEntropy layer has no outputs
        OutputNames = {}
    end
    
    properties
        % LearnableParameters   Learnable parameters for the layer
        %   This layer has no learnable parameters.
        LearnableParameters = nnet.internal.cnn.layer.learnable.PredictionLearnableParameter.empty();
        
        % Name (char array)   A name for the layer
        Name
        
        % Categories (column categorical array) The categories of the classes
        % It can store ordinality of the classes as well.
        Categories
    end
    
    properties (Constant)
        % DefaultName   Default layer's name.
        DefaultName = 'classoutput'
    end
    
    properties (SetAccess = private)
        % HasSizeDetermined   True for layers with size determined.
        HasSizeDetermined
        
        % NumClasses (scalar int)   Number of classes
        NumClasses
        
        % ObservationDim (scalar int)   The dimension of the input data
        % along which holds the number of observations within the data.
        ObservationDim
    end
    
    methods
        function this = RPNCrossEntropy(name, numClasses)
            % Output  Constructor for the layer
            % creates an output layer with the following parameters:
            %
            %   name                - Name for the layer
            %   numClasses          - Number of classes. [] if it has to be
            %                       determined later
            this.Name = name;
            if(isempty(numClasses))
                this.NumClasses = [];
            else
                this.NumClasses = numClasses;
            end
            this.Categories = categorical();
            this.HasSizeDetermined = ~isempty( numClasses );
            this.ObservationDim = 4;
        end
        
        function outputSize = forwardPropagateSize(~, inputSize)
            % forwardPropagateSize  Output the size of the layer based on
            % the input size
            outputSize = inputSize;
        end
        
        function this = inferSize(this, inputSize)
            % inferSize    Infer the number of classes and the observation
            % dimension, based on the input dimensions
            newNumClasses = inputSize(end);
            
            % Set NumClasses only if empty.
            if isempty(this.NumClasses)
                this.NumClasses = newNumClasses;
            end
            
            % Allow to update ObservationDim only if newNumClasses equals
            % this.NumClasses. Else, the input size is invalid and do not 
            % change the current observation for the new one. 
            if isequal(this.NumClasses, newNumClasses)
                this.ObservationDim = numel(inputSize) + 1;
            end
            
            this.HasSizeDetermined = true;
         end
        
        function tf = isValidInputSize(~, inputSize)
            % isValidInputSize   Check if the layer can accept an input of
            % a certain size.
            tf = numel(inputSize) == 3;
            tf = tf & inputSize(end) == 2; % only 2 classes.
        end
        
        function this = initializeLearnableParameters(this, ~)
            % initializeLearnableParameters     no-op since there are no
            % learnable parameters
        end
        
        function this = set.Categories( this, val )
            % Set Categories as a column array.
            if isrow(val)
                val = val';
            end
            this.Categories = val;
        end
        
        function this = prepareForTraining(this)
            this.LearnableParameters = nnet.internal.cnn.layer.learnable.TrainingLearnableParameter.empty();
        end
        
        function this = prepareForPrediction(this)
            this.LearnableParameters = nnet.internal.cnn.layer.learnable.PredictionLearnableParameter.empty();
        end
        
        function this = setupForHostPrediction(this)
        end
        
        function this = setupForGPUPrediction(this)
        end
        
        function this = setupForHostTraining(this)
        end
        
        function this = setupForGPUTraining(this)
        end
        
        function loss = forwardLoss( ~, Y, T )
            % forwardLoss    Return the cross entropy loss between estimate
            % and true responses averaged by the number of observations
            %
            % Syntax:
            %   loss = layer.forwardLoss( Y, T );
            %
            % Inputs:
            %   Y   Predictions made by network, M-by-N-by-numClasses-by-numAnchors
            %   T   Targets (actual values), M-by-N-by-numClasses-by-numAnchors    
            
            % Observations are encoded in T as non-zero values. T may
            % contain all zeros. Prevent divsion by zero preventing numObs
            % to be zero.
            numObservations = max(nnz(T),1); 
            
            % sum along numClasses
            loss = sum( T .* log(nnet.internal.cnn.util.boundAwayFromZero(Y)), 3);
            
            % sum all observations and average. Here all the
            % non-observations are also summed, but the loss for those is
            % zero, so it does not contribute.
            loss = -1/numObservations * sum(loss(:));                        
        end
        
        function dX = backwardLoss( ~, Y, T )
            % backwardLoss    Back propagate the derivative of the loss
            % function
            %
            % Syntax:
            %   dX = layer.backwardLoss( Y, T );
            %
            % Image Inputs:
            %   Y   Predictions made by network, 1-by-1-by-numClasses-by-numObs
            %   T   Targets (actual values), 1-by-1-by-numClasses-by-numObs
            %
            % Vector Inputs:
            %   Y   Predictions made by network,  numClasses-by-numObs-by-seqLength
            %   T   Targets (actual values),  numClasses-by-numObs-by-seqLength
            numObservations = max(nnz(T),1);          
            dX = (-T./nnet.internal.cnn.util.boundAwayFromZero(Y))./numObservations;
        end
    end   
end

