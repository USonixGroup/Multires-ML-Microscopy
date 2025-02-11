classdef GeneralizedDiceLoss < nnet.internal.cnn.layer.ClassificationLayer
% The Dice loss is based on the Sørensen-Dice similarity coefficient for
% measuring overlap between two segmented images. The generalized dice loss
% uses a class specific weighting factor that controls the contribution
% each class makes to the loss. 
% This factor is typically the inverse area of the expected region. This
% weighting helps counter the influence of larger regions on the Dice score
% making it easier for the network to learn how to segment smaller regions

%   Copyright 2019-2021 The MathWorks, Inc.
    
    properties(SetAccess = private)
        % InputNames   GeneralizedDiceLoss layer has one input
        InputNames = {'in'}
        
        % OutputNames   GeneralizedDiceLoss layer has no outputs
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
        % OutputSize A 3 element vector of [H W C] or 4 element vector of
        %            [H W D C] defining the size of the output.
        OutputSize
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
    
    properties (Access = private)
        % DeviceStrategy either moves data to the GPU or to the Host
        DeviceStrategy
    end
    
    methods
        function this = GeneralizedDiceLoss(name, categories, outputSize, channelDim)
            
            this.Name = name;
            this.OutputSize = outputSize;
            this.Categories = categories;
           
            if numel(categories) == 0
                this.NumClasses = [];
            else
                this.NumClasses = numel(categories);
            end 
            
            if ~isempty(channelDim)
                this.ChannelDim = channelDim;
            else
                % If OutputSize is known use it to set ChannelDim
                if isempty(this.OutputSize)
                    this.ChannelDim = [];
                else
                    this.ChannelDim = numel(this.OutputSize);
                end
            end
            
            this.DeviceStrategy = vision.internal.cnn.layer.util.ToHostStrategy();
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
                assert(false,'Invalid value in set.Categories.');
            end
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
        
        function loss = forwardLoss(this, Y, T )
            % forwardLoss    Return the Generalized Dice loss between
            % estimate (Y) and true responses (T) averaged by the number of
            % observations.
            %
            % Syntax:
            %   loss = layer.forwardLoss( Y, T );
            %
            % Inputs:
            %   Y   Predictions made by network, H-by-W-by-numClasses-by-numObs or H-by-W-by-D-by-numClasses-by-numObs 
            %   T   Targets (actual values), H-by-W-by-numClasses-by-numObs or H-by-W-by-D-by-numClasses-by-numObs 
            
            Y = this.DeviceStrategy.toDevice(single(Y));
            T = this.DeviceStrategy.toDevice(single(T));

            [~,numer,denom] = this.generalizedDiceLossParams( Y, T);
            
            % Compute Dice score.
            dice = numer./denom;
            
            % Return average Dice loss.
            obsDim = this.ChannelDim+1;
            N = iGetNDLastDimSize(Y,obsDim);
            loss = sum((1-dice))/N;      
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
            
            Y = this.DeviceStrategy.toDevice(single(Y));
            T = this.DeviceStrategy.toDevice(single(T));
            
            [W,numer,denom] = this.generalizedDiceLossParams( Y, T);
            
            obsDim = this.ChannelDim+1;
            N = iGetNDLastDimSize(Y,obsDim);
      
            dX = (2*W.*Y.*numer./(denom.^2) - 2*W.*T./denom)./N;                   
        end
        
        function [W,numer,denom] = generalizedDiceLossParams(this, Y, T)
           
            spatialDimVec = 1:this.ChannelDim-1;
            % Weights by inverse of region size.
            RegionSize = sum(T, spatialDimVec);
            W = 1 ./ max(iEpsilon(RegionSize),RegionSize.^2);
            
            intersection = sum(Y.*T, spatialDimVec);
            union = sum(Y.^2 + T.^2, spatialDimVec);
     
            numer = 2*sum(W.*intersection,this.ChannelDim) + iEpsilon(RegionSize);
            denom = sum(W.*union,this.ChannelDim) + iEpsilon(RegionSize);            
        end
    end
    
end

%--------------------------------------------------------------------------
function cats = iDefaultCategories(numClasses)
% Set the default Categories
cats = categorical(1:numClasses)'; 
end

%--------------------------------------------------------------------------
function val = iGetNDLastDimSize(X, obsDim)
sz = ones(1,obsDim);
sz(1:ndims(X)) = size(X);
val = sz(obsDim);
end

%--------------------------------------------------------------------------
function e = iEpsilon(x)
e = eps(underlyingType(x));
end
