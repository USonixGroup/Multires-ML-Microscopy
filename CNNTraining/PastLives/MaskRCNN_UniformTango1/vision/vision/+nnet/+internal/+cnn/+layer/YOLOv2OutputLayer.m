classdef YOLOv2OutputLayer < nnet.internal.cnn.layer.RegressionLayer
    % YOLOv2OutputLayer       Implementation of YOLO v2 Output Layer
    
    %   Copyright 2018-2021 The MathWorks, Inc.
    
    properties(SetAccess = private)
        % InputNames    YOLOv2OutputLayer layer has one input
        InputNames = {'in'}
        
        % OutputNames   YOLOv2OutputLayer layer has no outputs
        OutputNames = {}
    end
    
    properties
        % LearnableParameters   Learnable parameters for the layer
        %   This layer has no learnable parameters.
        LearnableParameters = nnet.internal.cnn.layer.learnable.PredictionLearnableParameter.empty();
        
        % Name (char array)   A name for the layer.
        Name
        
        % ResponseNames (cellstr)   The names of the responses.
        ResponseNames
        
        % Categories (column categorical array) The categories of the classes
        % It can store ordinality of the classes as well.
        Categories
    end
    
    properties (Constant)
        % DefaultName   Default layer's name.
        DefaultName = 'yolov2Output'
    end
    
    properties (SetAccess = private)
        % HasSizeDetermined   True for layers with size determined.
        HasSizeDetermined
        
        % NumClasses (scalar int)   Number of classes.
        NumClasses
    end
    
    properties (SetAccess = private)
        % AnchorBoxes - An M-by-2 matrix defining the [width height] of M
        %               anchor boxes.
        AnchorBoxes
        
        % LossFactors - A 1-by-4 vector, K, representing the loss
        %               factors for each component of the YOLO v2 loss
        %               function. K(1) is the objectness factor, K(2)
        %               is the non-objectness factor, K(3) is the box
        %               coordinate factor, and K(4) is class factor.
        %               Increase the loss factor to give more weight to
        %               anyone of the loss components.
        LossFactors
    end
    
    properties(SetAccess = private, Hidden)
        % IouThresh - Threshold to discard predictions with less confidence.
        IouThresh = 0.6;
    end
    
    methods
        function this = YOLOv2OutputLayer(name,AnchorBoxes,LossFactors,numClasses)
            % YOLOv2OutputLayer   Constructor for the layer.
            this.Name = name;
            this.HasSizeDetermined = false;
            this.AnchorBoxes = AnchorBoxes;
            this.LossFactors = LossFactors;
            if(isempty(numClasses))
                this.NumClasses = [];
            else
                this.NumClasses = numClasses;
            end
        end
        
        function outputSize = forwardPropagateSize(~, inputSize)
            % ForwardPropagateSize  Output the size of the layer based on
            % the input size.
            outputSize = inputSize;
        end
        
        function this = inferSize(this, inputSize)
            % inferSize    Infer the number of classes based on the input
            %              dimensions.
            inputVectorDim = inputSize(end);
            numAnchors = size(this.AnchorBoxes,1);
            
            % Set NumClasses only if empty.
            if isempty(this.NumClasses)
                % 5 corresponds to predictions per anchor, they are [x, y, w, h, iou].
                this.NumClasses = floor(inputVectorDim/numAnchors) - 5;
            end
            
            this.HasSizeDetermined = true;
        end
        
        function tf = isValidInputSize(this, inputSize)
            % isValidInputSize   Check if the layer can accept an input of
            % a certain size.
            tf1 = iNonEmptyMatrix(inputSize(1:2)) ...
                && numel(inputSize)== 3;
            tf2 = 1;
            if (~isempty(this.Categories))
                numLayerClasses = size(this.Categories,1);
                numAnchors = size(this.AnchorBoxes,1);
                reqFilters = numAnchors*(5 + numLayerClasses);
                if (inputSize(3) ~= reqFilters)
                    tf2 = 0;
                end
            end
            tf = tf1 & tf2;
        end
        
        function this = initializeLearnableParameters(this, ~)
            % no-op since there are no learnable parameters.
        end
        
        function this = set.Categories(this, val)
            if isequal(val, 'default')
                this.Categories = iDefaultCategories(this.NumClasses); %#ok<MCSUP>
            elseif iscategorical(val)
                % Set Categories as a column array.
                if isrow(val)
                    val = val';
                end
                this.Categories = val;
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
        
        function loss = forwardLoss( this, Y, T)
            inSize = size(Y);
            
            Y = permute(Y,[2 1 3 4]);
            Y = reshape(Y,inSize(1)*inSize(2),inSize(3),1,[]);
            Y = reshape(Y,inSize(1)*inSize(2),size(this.AnchorBoxes,1),inSize(3)/size(this.AnchorBoxes,1),[]);
            Y = permute(Y,[2 3 1 4]);
            
            % Bound predicted values within the range of activation size.
            Y(:,4,:,:) = min(Y(:,4,:,:),inSize(2));
            Y(:,5,:,:) = min(Y(:,5,:,:),inSize(1));
            
            response = zeros(size(Y),'like',Y);
            weights = zeros(size(Y),'like',Y);
            
            Y = gather(Y);
            T = iGatherAndConvertBoxesToMinMaxFormat(T);
            
            lossFactors = this.LossFactors;
            anchorBoxes = this.AnchorBoxes;
            iouThresh = this.IouThresh;
            numBoxes = 0;
            for i=1:size(Y,4)
                perImgGT = T{i};
                numBoxes = numBoxes+size(perImgGT,1);
                [response(:,:,:,i), weights(:,:,:,i)] = nnet.internal.cnn.layer.util.yolov2ResponseAndWeights(Y(:,:,:,i),perImgGT,lossFactors,anchorBoxes,iouThresh,inSize);
            end
            squareLoss = ((weights.*Y)-(weights.*response)).^2;
            loss = sum(squareLoss(:))/numBoxes;
        end
        
        function dX = backwardLoss( this, Y, T )
            % backwardLoss    Back propagate the derivative of the loss
            % function.
            %
            % Syntax:
            %   dX = layer.backwardLoss( Y, T );
            %
            % Inputs (image):
            %   Y   Predictions made by network, of size:
            %   height-by-width-by-numResponses-by-numObservations
            %   T   Targets (actual values), of size:
            %   height-by-width-by-numResponses-by-numObservations
            %
            % Inputs (sequence):
            %   Y   Predictions made by network, of size:
            %   numResponses-by-numObservations-by-seqLength
            %   T   Targets (actual values), of size:
            %   numResponses-by-numObservations-by-seqLength
            
            inSize = size(Y);
            
            Y = permute(Y,[2 1 3 4]);
            Y = reshape(Y,inSize(1)*inSize(2),inSize(3),1,[]);
            Y = reshape(Y,inSize(1)*inSize(2),size(this.AnchorBoxes,1),inSize(3)/size(this.AnchorBoxes,1),[]);
            Y = permute(Y,[2 3 1 4]);
            
            % Bound predicted values within the range of activation size.
            Y(:,4,:,:) = min(Y(:,4,:,:),inSize(2));
            Y(:,5,:,:) = min(Y(:,5,:,:),inSize(1));
            
            response = zeros(size(Y),'like',Y);
            weights = zeros(size(Y),'like',Y);
            
            Y = gather(Y);
            T = iGatherAndConvertBoxesToMinMaxFormat(T);
            
            lossFactors = this.LossFactors;
            anchorBoxes = this.AnchorBoxes;
            iouThresh = this.IouThresh;
            numberboxes = 0;
            for i = 1:size(Y,4)
                perImgGT = T{i};
                numberboxes = numberboxes+size(perImgGT,1);
                [response(:,:,:,i), weights(:,:,:,i)] = nnet.internal.cnn.layer.util.yolov2ResponseAndWeights(Y(:,:,:,i),perImgGT,lossFactors,anchorBoxes,iouThresh,inSize);
            end
            dX = 2*((Y - response).*weights.*weights)/numberboxes;
            
            dX = permute(dX,[3,1,2,4]);
            dX = reshape(dX,inSize(1)*inSize(2),inSize(3),1,[]);
            dX = reshape(dX,inSize(1),inSize(2),inSize(3),[]);
            dX = permute(dX,[2 1 3 4]);
        end
    end
    
    methods (Hidden, Static)
        function layer = constructWithClasses(name, anchorBoxes, lossFactors, numClasses, categories)
            % constructWithClasses     Construct a YOLO v2 Output layer
            %                          with the classes defined on
            %                          construction.
            layer = nnet.internal.cnn.layer.YOLOv2OutputLayer(...
                name, anchorBoxes, lossFactors, numClasses);
            layer.Categories = categories;
        end
    end
    
end

function T = iGatherAndConvertBoxesToMinMaxFormat(T)
T = T{1};
for ii = 1:numel(T)
    T{ii} = gather(T{ii});
    T{ii}(:,1:4) = vision.internal.cnn.boxUtils.xywhToX1Y1X2Y2(T{ii}(:,1:4));
end
end

function tf = iNonEmptyMatrix(inputSize)
tf = all(inputSize > 0);
end

function cats = iDefaultCategories(numClasses)
% Set the default Classes.
cats = categorical(1:numClasses)';
end
