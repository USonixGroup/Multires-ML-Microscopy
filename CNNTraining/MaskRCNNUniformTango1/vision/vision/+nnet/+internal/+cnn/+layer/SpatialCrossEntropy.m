classdef SpatialCrossEntropy < nnet.internal.cnn.layer.ClassificationLayer
%

%   Copyright 2017-2021 The MathWorks, Inc.
    
    properties(SetAccess = private)
        % InputNames   SpatialCrossEntropy layer has one input
        InputNames = {'in'}
        
        % OutputNames   SpatialCrossEntropy layer has no outputs
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
        % NumClasses (scalar int)   Number of classes
        NumClasses
    end
    
    
    properties(SetAccess = private, GetAccess = public)
        
        % ClassWeights A vector of weights. Either [] or the same size as
        %              number of class names.
        ClassWeights
        
        % OutputSize A 3 element vector of [H W C] or 4 element vector of
        %            [H W D C] defining the size of the output.
        OutputSize
        
        % NormalizedClassWeights A vector of weights. Either [] or the same
        %                        size as number of class names.
        NormalizedClassWeights
    end
    
    properties
        % ChannelDim  The channel dimension in the input. Leave as [] to
        %             infer later.
        ChannelDim
    end
    
    properties (Dependent, SetAccess = private)
        % HasSizeDetermined   True for layers with size determined. Both
        %                     OutputSize and ChannelDim need to be set for
        %                     this to be true.
        HasSizeDetermined
    end
    
    methods
        function this = SpatialCrossEntropy(name, categories, weights, outputSize)
            
            if numel(categories) == 0
                this.NumClasses = [];
            else
                this.NumClasses = numel(categories);
            end   
            
            this.Categories = categories;
            
            this.ClassWeights = weights;
            
            this.OutputSize = outputSize;
            
            this.Name = name;
            
            % If OutputSize is known use it to set ChannelDim
            if isempty(this.OutputSize)                                
                this.ChannelDim = [];
            else
                this.ChannelDim = numel(this.OutputSize);
                % Store normalized weights
                this.NormalizedClassWeights = iNormalizeWeights(this.ClassWeights, this.ChannelDim);
            end
        end
        
        function tf = get.HasSizeDetermined(this)
            tf = ~isempty(this.OutputSize) && ~isempty(this.ChannelDim);
        end
        
        function outputSize = forwardPropagateSize(~, inputSize)
            % forwardPropagateSize  Output the size of the layer based on
            % the input size
            outputSize = inputSize;
        end
        
        function tf = isValidInputSize(this, inputSize)
            % isValidInputSize   Check if the layer can accept an input of
            % a certain size.
            tf = this.HasSizeDetermined;
            tf = tf && numel(inputSize)==this.ChannelDim && isequal(inputSize(this.ChannelDim), this.OutputSize(this.ChannelDim));
        end
        
        function this = inferSize(this, inputSize)
            % inferSize    Infer the number of classes based on the input
            if isempty(this.ChannelDim)
                this.ChannelDim = numel(inputSize);
                % Store normalized weights
                this.NormalizedClassWeights = iNormalizeWeights(this.ClassWeights, this.ChannelDim);
            end
            
            this.NumClasses = inputSize(this.ChannelDim);
            this.OutputSize = inputSize(1:this.ChannelDim);
        end
        
        function this = initializeLearnableParameters(this, ~)
            % initializeLearnableParameters     no-op since there are no
            % learnable parameters
        end
        
        function this = set.Categories( this, val )
            if isequal(val, 'default')
                this.Categories = iDefaultCategories(this.NumClasses); %#ok<MCSUP>
            elseif iscategorical(val)
                % Set Categories as a column array.
                if isrow(val)
                    val = val';
                end
                this.Categories = val;
            else
                warning('Invalid value in set.Categories.');
            end
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
        
        function loss = forwardLoss(this, Y, T )
            % forwardLoss    Return the cross entropy loss between estimate
            % and true responses averaged by the number of observations.
            % Here the observations include the spatial dimensions.
            %
            % When ClassWeights is not empty, observations are weighted by
            % values specified in ClassWeights. ClassWeights are normalized
            % to sum to 1.
            %
            % Syntax:
            %   loss = layer.forwardLoss( Y, T );
            %
            % Inputs:
            %   Y   Predictions made by network, H-by-W-by-numClasses-by-numObs or H-by-W-by-D-by-numClasses-by-numObs 
            %   T   Targets (actual values), H-by-W-by-numClasses-by-numObs or H-by-W-by-D-by-numClasses-by-numObs 
            
            if this.ChannelDim == 3
                numObservations = size(Y, 4) * size(Y, 1) * size(Y, 2);
                if ~isempty(this.ClassWeights)
                    W = this.classWeights(T);
                    loss_i = W .* log(nnet.internal.cnn.util.boundAwayFromZero(Y));
                    loss = -sum( sum( sum( sum(loss_i, 3), 1), 2));
                else
                    
                    loss_i = T .* log(nnet.internal.cnn.util.boundAwayFromZero(Y));
                    loss = -sum( sum( sum( sum(loss_i, 3).*(1./numObservations), 1), 2));
                end
            else
                numObservations = size(Y, 5) * size(Y, 1) * size(Y, 2)* size(Y, 3);
                if ~isempty(this.ClassWeights)
                    W = this.classWeights(T);
                    loss_i = W .* log(nnet.internal.cnn.util.boundAwayFromZero(Y));
                    loss = -sum( sum( sum( sum( sum(loss_i, 4), 1), 2), 3));
                else
                    
                    loss_i = T .* log(nnet.internal.cnn.util.boundAwayFromZero(Y));
                    loss = -sum( sum( sum( sum( sum(loss_i, 4).*(1./numObservations), 1), 2), 3));
                end
            end
            
        end
        
        function dX = backwardLoss( this, Y, T )
            % backwardLoss    Back propagate the derivative of the loss
            % function
            %
            % Syntax:
            %   dX = layer.backwardLoss( Y, T );
            %
            % Inputs:
            %   Y   Predictions made by network, H-by-W-by-numClasses-by-numObs  or H-by-W-by-D-by-numClasses-by-numObs 
            %   T   Targets (actual values), H-by-W-by-numClasses-by-numObs or H-by-W-by-D-by-numClasses-by-numObs 
            
            numObservations = iNumObservations(Y, this.ChannelDim);
            if ~isempty(this.ClassWeights)
                W = this.classWeights(T);
                dX = (-W./nnet.internal.cnn.util.boundAwayFromZero(Y));
            else
                dX = (-T./nnet.internal.cnn.util.boundAwayFromZero(Y)).*(1./numObservations);
            end         
        end
        
        function W = classWeights(this, T)
            % Assign weight, W(c) to each observation that belongs to
            % class(c). Then normalize all weights to sum to 1.           
            W = T .* this.NormalizedClassWeights;
            W = W ./ (sum(W(:)) + iEpsilon(W));
        end
    end
    
end

%--------------------------------------------------------------------------
function wn = iNormalizeWeights(weights, channelDim)
wn = weights ./  (sum(weights(:)) + iEpsilon(weights));
wn = shiftdim(wn(:)',-(channelDim-2));
end

%--------------------------------------------------------------------------
function e = iEpsilon(x)
e = eps(underlyingType(x));
end

%--------------------------------------------------------------------------
function cats = iDefaultCategories(numClasses)
% Set the default Categories
cats = categorical(1:numClasses)'; 
end

%--------------------------------------------------------------------------
function numObservations = iNumObservations(Y, channelDim)
% numObservations product of all dimensions except channel dimensions

numDims = channelDim+1;
x = cell(1,numDims);
[x{1:numDims}] = size(Y);
x(channelDim) = [];
numObservations = prod([x{:}]);

end
