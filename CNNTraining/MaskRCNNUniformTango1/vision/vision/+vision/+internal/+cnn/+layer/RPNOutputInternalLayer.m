classdef RPNOutputInternalLayer < nnet.internal.cnn.layer.RegressionLayer & vision.internal.cnn.FasterRCNNEndToEndLossUtilities
% The internal layer that implements the RPN output layer needed
% for datastores inputs in faster R-CNN.
%

% Copyright 2019-2021 The MathWorks, Inc.

    properties(SetAccess = private)
        % InputNames   RPNOutputInternalLayer layer has one input
        InputNames = {'in'}
        
        % OutputNames   RPNOutputInternalLayer layer has no outputs
        OutputNames = {}
    end
    
    properties
        % LearnableParameters   Not a learnable layer
        % (Vector of nnet.internal.cnn.layer.LearnableParameter)
        LearnableParameters = nnet.internal.cnn.layer.learnable.PredictionLearnableParameter.empty();

        % Name (char array)
        Name

        % NumResponses
        NumResponses

        % ResponseNames Not used, but it's here so that an odd error isn't
        % thrown by trainNetwork.
        ResponseNames
    end

    properties (SetAccess = private)
        % HasSizeDetermined   True for layers with size determined.
        HasSizeDetermined = true;
    end

    properties (Constant)
        % DefaultName   Default layer's name. This will be assigned in case
        % the user leaves an empty name.
        DefaultName = 'rpn-output-layer'
    end

    properties(Access = private)

        % RPN Classfication layer
        RPNClassificationLayer

        % RPN Box regression layer
        RPNBoxDeltaSmooth1LossLayer
    end

    methods

        function this = RPNOutputInternalLayer(name, classNames, proposalParams)
             this.NumResponses = []; % gets set during training.
             this.Name = char(name);
             this.ClassNames = classNames;

             % Reuse existing layers to compute loss.
             this.RPNSoftmaxLayer = nnet.internal.cnn.layer.RPNSoftmaxLayer('rpnSoftmax');

             numResponses = [];
             this.RPNBoxDeltaSmooth1LossLayer = vision.internal.cnn.layer.SmoothL1Loss('rpnBoxDeltas', numResponses);

             numClasses = [];
             this.RPNClassificationLayer = vision.internal.cnn.layer.RPNCrossEntropy('rpnClassfication', numClasses);

             this.ProposalParameters = proposalParams;
        end

        function L = forwardLoss(this, Y, T)
            % size(Y): [H W 4*numAnchors+2*numAnchors N]
            % N: Batch size

            [Ycls, Tcls, Yreg, Treg, Wreg, hasResponsesInBatch] = rpnClassificationRegressionPredictionResponses(this, Y, T);
            if ~hasResponsesInBatch
                L = NaN;
            else
                % Convolution class layer -->--forward-to--> softmax -->--forward-to--> rpn classification output layer
                Ycls = this.RPNSoftmaxLayer.forward(Ycls);
                % Forwarding to output layer is the same, so this is not needed:
                % Ycls = this.RPNClassificationLayer.forward(Ycls);

                % Compute total loss.
                % Convolution regression layer -->--forward-to--> rpn box regression output layer
                % Forward to output layer is the same, so this is not needed:
                % Yreg = this.RPNBoxDeltaSmooth1LossLayer.forward(Yreg);
                Lreg = this.RPNBoxDeltaSmooth1LossLayer.forwardLoss(Yreg, {Treg, Wreg});
                Lcls = this.RPNClassificationLayer.forwardLoss(Ycls, Tcls);
                L    = Lcls + Lreg;
            end
        end

        function dX = backwardLoss(this, Y, T)
            % size(Y): [H W 4*numAnchors+2*numAnchors N]
            % N: Batch size

            [Ycls, Tcls, Yreg, Treg, Wreg, hasResponsesInBatch] = rpnClassificationRegressionPredictionResponses(this, Y, T);

            if ~hasResponsesInBatch
                dX = zeros(size(Y),'like',Y);
            else
                % forward the classification to softmax first.
                Zcls = this.RPNSoftmaxLayer.forward(Ycls);

                % Back propagate classification gradient.
                dLdZCls = this.RPNClassificationLayer.backwardLoss(Zcls, Tcls);
                % Softmax --back-to--> Convolution classification layer
                dXCls = this.RPNSoftmaxLayer.backward(Ycls, Zcls, dLdZCls);

                % BoxDelta output layer --back-to--> Convolution regression layer
                dXReg = this.RPNBoxDeltaSmooth1LossLayer.backwardLoss(Yreg, {Treg, Wreg});

                % Assemble gradients for each loss component.
                % For depth concatenation layer:
                %    - input1 is regression.
                %    - input2 is classification.
                dX = cat(3,dXReg,dXCls);
            end
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
            this.RPNSoftmaxLayer = this.RPNSoftmaxLayer.prepareForTraining();

            this.RPNClassificationLayer = this.RPNClassificationLayer.prepareForTraining();

            this.RPNBoxDeltaSmooth1LossLayer = this.RPNBoxDeltaSmooth1LossLayer.prepareForTraining();
        end

        %------------------------------------------------------------------
        % prepareForPrediction   Prepare the layer for prediction
        function this = prepareForPrediction(this)
            this.RPNSoftmaxLayer = this.RPNSoftmaxLayer.prepareForPrediction();

            this.RPNClassificationLayer = this.RPNClassificationLayer.prepareForPrediction();

            this.RPNBoxDeltaSmooth1LossLayer = this.RPNBoxDeltaSmooth1LossLayer.prepareForPrediction();
        end

        %------------------------------------------------------------------
        % setupForHostPrediction   Prepare this layer for host prediction
        function this = setupForHostPrediction(this)
            this.RPNSoftmaxLayer = this.RPNSoftmaxLayer.setupForHostPrediction();

            this.RPNClassificationLayer = this.RPNClassificationLayer.setupForHostPrediction();

            this.RPNBoxDeltaSmooth1LossLayer = this.RPNBoxDeltaSmooth1LossLayer.setupForHostPrediction();
        end

        %------------------------------------------------------------------
        % setupForGPUPrediction   Prepare this layer for GPU prediction
        function this = setupForGPUPrediction(this)
            this.RPNSoftmaxLayer = this.RPNSoftmaxLayer.setupForGPUPrediction();

            this.RPNClassificationLayer = this.RPNClassificationLayer.setupForGPUPrediction();

            this.RPNBoxDeltaSmooth1LossLayer = this.RPNBoxDeltaSmooth1LossLayer.setupForGPUPrediction();
        end

        %------------------------------------------------------------------
        % setupForHostTraining   Prepare this layer for host training
        function this = setupForHostTraining(this)
            this.RPNSoftmaxLayer = this.RPNSoftmaxLayer.setupForHostTraining();

            this.RPNClassificationLayer = this.RPNClassificationLayer.setupForHostTraining();

            this.RPNBoxDeltaSmooth1LossLayer = this.RPNBoxDeltaSmooth1LossLayer.setupForHostTraining();
        end

        %------------------------------------------------------------------
        % setupForGPUTraining   Prepare this layer for GPU training
        function this = setupForGPUTraining(this)
            this.RPNSoftmaxLayer = this.RPNSoftmaxLayer.setupForGPUTraining();

            this.RPNClassificationLayer = this.RPNClassificationLayer.setupForGPUTraining();

            this.RPNBoxDeltaSmooth1LossLayer = this.RPNBoxDeltaSmooth1LossLayer.setupForGPUTraining();
        end

    end
end

