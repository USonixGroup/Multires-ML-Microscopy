classdef FastRCNNOutputInternalLayer < nnet.internal.cnn.layer.RegressionLayer & vision.internal.cnn.FasterRCNNEndToEndLossUtilities
%

%   Copyright 2018-2021 The MathWorks, Inc.
    
    properties(SetAccess = private)
        % InputNames   FastRCNNOutputInternalLayer layer has one input
        InputNames = {'in'}
        
        % OutputNames   FastRCNNOutputInternalLayer layer has no outputs
        OutputNames = {}
    end
    
    properties
        % LearnableParameters   Learnable parameters for the layer
        % (Vector of nnet.internal.cnn.layer.LearnableParameter)
        LearnableParameters = nnet.internal.cnn.layer.learnable.PredictionLearnableParameter.empty();
        
        % Name (char array)
        Name
        
        % NumResponses
        NumResponses
        
        % ResponseNames Not used, but here so that an odd error isn't
        % thrown by trainNetwork.
        ResponseNames   
        
        % The fraction of foreground (i.e. positive) observations in 
        ForegroundFraction

    end
    
    properties (SetAccess = private)
        % HasSizeDetermined   True for layers with size determined.
        HasSizeDetermined = true;
    end
    
    properties (Constant)
        % DefaultName   Default layer's name. This will be assigned in case
        % the user leaves an empty name.
        DefaultName = 'fast-rcnn-output'
    end
    
    properties(Access = private)
        
        % SmoothL1Loss Loss layer for computing regression loss. 
        SmoothL1Loss
        
        % CrossEntropy Loss layer for computing classification loss.
        CrossEntropy
        
    end
    
    methods
        
        function this = FastRCNNOutputInternalLayer(name, classNames, boxMatcher, numRegionsToSample, foregroundFraction, isGeneralDatastore)
             this.NumResponses = []; % gets set during training.
             this.Name = char(name);     
             this.ClassNames = classNames;  
                        
             this.PositiveOverlapRange = boxMatcher.PositiveOverlapRange;
             this.NegativeOverlapRange = boxMatcher.NegativeOverlapRange;             
             this.NumRegionsToSample   = numRegionsToSample;             
             this.ForegroundFraction   = foregroundFraction;
             this.IsGeneralDatastore   = isGeneralDatastore;
             this.ClassCategoricals    = categorical(classNames, classNames);
             
             % Reuse existing layers to compute loss.
             this.SmoothL1Loss  = vision.internal.cnn.layer.SmoothL1Loss(name, this.NumClasses * 4);
             this.CrossEntropy  = vision.internal.cnn.layer.RPNCrossEntropy(name,[]);
        end
                
        function L = forwardLoss(this, Y, T)
            % size(Y): [1 1 C+(4*C)+5 N]
            % T contains ground truth boxes for each images in batch and
            % their labels.
            
            [Ycls, Tcls, Yreg, Treg, Wreg, hasResponsesInBatch, numRegionsToSample] = ...
                fastRCNNClassificationRegressionPredictionResponses(this, Y, T);

            if ~hasResponsesInBatch
                L    = NaN;
            else
                Tcls = this.balanceSamplesInBatch(Tcls, numRegionsToSample);

                % Compute total loss.
                Lreg = this.SmoothL1Loss.forwardLoss(Yreg, {Treg, Wreg});
                Lcls = this.CrossEntropy.forwardLoss(Ycls, Tcls);
                L    = Lcls + Lreg;
            end
            
        end
      
        function dX = backwardLoss(this, Y, T)
                    
            [Ycls, Tcls, Yreg, Treg, Wreg, hasResponsesInBatch, numRegionsToSample] = ...
                fastRCNNClassificationRegressionPredictionResponses(this, Y, T);

            if ~hasResponsesInBatch
                dX = zeros(size(Y),'like',Y);
            else                
                Tcls = this.balanceSamplesInBatch(Tcls, numRegionsToSample);

                dXcls = this.CrossEntropy.backwardLoss(Ycls, Tcls);

                dXreg = this.SmoothL1Loss.backwardLoss(Yreg, {Treg,Wreg});

                % grad w.r.t to proposals is zero.
                N = size(Y,4);
                dXproposals = zeros([1,1,5,N],'like',Y);

                % Assemble gradients for each loss component.
                dX = cat(3,dXcls,dXreg,dXproposals);
            end
        end
        
        % balanceSamplesInBatch  Balances samples in batch by randomly
        % sampling observations. Samples are dropped from T by setting
        % their entries to all zeros. This removes them from the loss
        % computation. 
        %
        % A batch is padded with negatives if required. The batch may be
        % smaller if there are not enough negatives to fill the batch. 
        function T = balanceSamplesInBatch(this, T, numROIPerBatch)

            foregroundFraction = this.ForegroundFraction;
            
            % Total number of positive observations in the batch.
            Npos = sum(sum(T(:,:,1:end-1,:)));
            
            % Total number of negative observations in the batch. 
            Nneg = sum(sum(T(:,:,end,:)));   
            
            % Number of positive observations to include in the batch.
            numPos = min(Npos, floor(foregroundFraction * numROIPerBatch));
            
            % Fill remaining space in batch with negative observations.
            numNeg = min(Nneg, numROIPerBatch - numPos); 
                    
            % Indices to positive and negative observations.            
            positiveIndices = any(reshape(T(:,:,1:end-1,:),this.NumClasses,[]),1);
            negativeIndices = logical(reshape(T(:,:,end,:),1,[]));
            
            % Balance the batch by removing observations. 
            posIdx = randperm(Npos, Npos-numPos);
            negIdx = randperm(Nneg, Nneg-numNeg);
            
            orderedIndices = 1:size(T,4);
            orderedIndices = orderedIndices(positiveIndices);
            
            T(:,:,:,orderedIndices(posIdx)) = 0;
            
            orderedIndices = 1:size(T,4);
            orderedIndices = orderedIndices(negativeIndices);
            
            T(:,:,:,orderedIndices(negIdx)) = 0;
        end
        
        % forwardPropagateSize    The size of the output from the layer for
        % a given size of input
        function outputSize = forwardPropagateSize(~, inputSize)
            outputSize = inputSize;
        end
        
        % inferSize    Infer the size of the learnable parameters based
        % on the input size
        function this = inferSize(this, ~)
        end
        
        % isValidInputSize   Check if the layer can accept an input of a
        % certain size
        function tf = isValidInputSize(~, inputSize)
            tf = numel(inputSize) == 3;
        end
        
        %------------------------------------------------------------------
        % initializeLearnableParameters    Initialize learnable parameters
        % using their initializer
        function this = initializeLearnableParameters(this, ~)
            
        end
        
        %------------------------------------------------------------------
        % prepareForTraining   Prepare the layer for training
        function this = prepareForTraining(this)            
            this.SmoothL1Loss = this.SmoothL1Loss.prepareForTraining();
            this.CrossEntropy = this.CrossEntropy.prepareForTraining();
        end
        
        %------------------------------------------------------------------
        % prepareForPrediction   Prepare the layer for prediction
        function this = prepareForPrediction(this)           
            this.SmoothL1Loss = this.SmoothL1Loss.prepareForPrediction();
            this.CrossEntropy = this.CrossEntropy.prepareForPrediction();
        end
        
        %------------------------------------------------------------------
        % setupForHostPrediction   Prepare this layer for host prediction
        function this = setupForHostPrediction(this)           
            this.SmoothL1Loss = this.SmoothL1Loss.setupForHostPrediction();
            this.CrossEntropy = this.CrossEntropy.setupForHostPrediction();
        end
        
        %------------------------------------------------------------------
        % setupForGPUPrediction   Prepare this layer for GPU prediction
        function this = setupForGPUPrediction(this)           
            this.SmoothL1Loss = this.SmoothL1Loss.setupForGPUPrediction();
            this.CrossEntropy = this.CrossEntropy.setupForGPUPrediction();
        end
        
        %------------------------------------------------------------------
        % setupForHostTraining   Prepare this layer for host training
        function this = setupForHostTraining(this)            
            this.SmoothL1Loss = this.SmoothL1Loss.setupForHostTraining();
            this.CrossEntropy = this.CrossEntropy.setupForHostTraining();
        end
        
        %------------------------------------------------------------------
        % setupForGPUTraining   Prepare this layer for GPU training
        function this = setupForGPUTraining(this)           
            this.SmoothL1Loss = this.SmoothL1Loss.setupForGPUTraining();
            this.CrossEntropy = this.CrossEntropy.setupForGPUTraining();
        end
        
    end
end
