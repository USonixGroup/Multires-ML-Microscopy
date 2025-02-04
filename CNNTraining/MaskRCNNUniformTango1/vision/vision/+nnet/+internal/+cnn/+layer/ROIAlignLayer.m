classdef ROIAlignLayer < nnet.internal.cnn.layer.FunctionalLayer
    % Internal layer for ROIAlign operation 

    %   Copyright 2020-2022 The MathWorks, Inc.
    
    properties
        % LearnableParameters   Learnable parameters for the layer
        %   This layer has no learnable parameters.
        LearnableParameters = nnet.internal.cnn.layer.learnable.PredictionLearnableParameter.empty();
        
        % Name (char array)   A name for the layer
        Name
    end
    
    properties
        % Learnables   Empty
        Learnables
    end
    
    properties (SetAccess = protected)
        % IsInFunctionalMode   Returns true if layer is currently being
        % used in "functional" mode (i.e. in dlnetwork). Required by
        % FunctionalLayer interface. On construction, all layers are set up
        % for usage in DAGNetwork.
        IsInFunctionalMode = false
    end
    
    properties(SetAccess=protected)
        % LearnablesName   Empty
        LearnablesNames
    end
    
    properties (Constant)
        % DefaultName   Default layer's name.
        DefaultName = 'ROI Align'
    end
    
    properties (SetAccess = private)
        % InputNames   This layer has two inputs
        InputNames = {'in','roi'}
        
        % OutputNames   This layer has one output
        OutputNames = {'out'}
        
        % HasSizeDetermined   Specifies if all size parameters are set
        HasSizeDetermined = true;
                
        % OutputSize The height and width to divide each ROI. Each pooled cell
        % is then  sampled SamplingRatio times and then average pooled.
        OutputSize 
        
        % ROIScale (double scalar) Used to scale boxes from feature map to
        % image
        ROIScale = 1
        
        % Sampling Ratio (double [1x2] or string)  Number of samples in each pooled bin
        SamplingRatio = 'auto';
    end
    
    properties(Access = protected)
        ExecutionStrategy        
    end              
    
    methods
        function this = ROIAlignLayer(name, varargin)
            narginchk(1,4);
            this.Name = name;                                    
            N = numel(varargin);
            if N >= 1                
                this.OutputSize = varargin{1};
            end
            if N >= 2
                this.ROIScale = varargin{2};          
            end
            if N >= 3
                this.SamplingRatio = varargin{3};          
            end
            
            % roi align layer does not use either X or Z for the backward pass
            this.NeedsXForBackward = false;
            this.NeedsZForBackward = false;
            
            this.ExecutionStrategy = nnet.internal.cnn.layer.util.ROIAlignHostStrategy();
        end
        
        
        function Z = predict(this, X)      
            
            roi = X{2}; % [x1 y1 x2 y2 batch_idx] format.
            
            % Enforce 5xM size for ROIs
            assert(size(roi,1)==5);
            
            if isempty(roi)
                outputSize = [this.OutputSize size(X{1},3) size(X{1},4)];
                if(isdlarray(X{1}))
                    dimsX = dims(X{1});
                    Z = zeros(outputSize,'like',extractdata(X{1}));
                    Z = dlarray(Z, dimsX);
                else
                    Z = zeros(outputSize,'like',X{1});
                end
            else
                
                scaledBBoxes = this.scaleBoxesToFeatureSpace( gather(roi) );
                
                Z = this.ExecutionStrategy.forward(X{1}, scaledBBoxes, this.OutputSize, iConvertSamplingRatioToNum(this.SamplingRatio));
            end
        end
        
        function [Z, memory] = forward(this, X)             
            
            roi = gather(X{2}); % [x1 y1 x2 y2 batch_idx] format.
            
            % Enforce 5xM size for ROIs
            assert(size(roi,1)==5);
            
            if isempty(roi)
                % Zero ROI input. Return NaN feature maps in this case.
                outputSize = [this.OutputSize size(X{1},3) size(X{1},4)];
                if(isdlarray(X{1}))
                    dimsX = dims(X{1});
                    Z = NaN(outputSize,'like',extractdata(X{1}));
                    Z = dlarray(Z, dimsX);
                else
                    Z = NaN(outputSize,'like',X{1});
                end
                memory.roi = roi;
            else
                scaledBBoxes = this.scaleBoxesToFeatureSpace( roi );
                
                [Z, memory] = this.ExecutionStrategy.forward(X{1}, scaledBBoxes, this.OutputSize, iConvertSamplingRatioToNum(this.SamplingRatio));
            end
        end
        
        function [dX, dW] = backward(this, ~, ~, dZ, memory)
            
            if isempty(memory.roi)
                % Backprop zero gradients. No parameter updates ought to be
                % made in this.
                dX{1} = zeros(memory.inputSize,'like',dZ);               
                dW = [];
            else
                memory.roi = iAppendImageIndexIfRequired(memory.roi);
                [dX{1}, dW] = this.ExecutionStrategy.backward(dZ, iConvertSamplingRatioToNum(this.SamplingRatio), memory);                
            end
            dX{2} = 0; % no gradient for ROI input.
        end
        
        function bboxes = scaleBoxesToFeatureSpace(this,bboxes)
            % Scales boxes to feature space. Input bboxes and scaled boxes
            % are in [x1 y1 x2 y2] format.
            if(this.ROIScale == 1)
                return;
            end
            % Translate the bboxes to 0-index coordinate space and scale them.
            % Translate the boxes back to 1-index coordinate space.
            bboxes(1:4,:) = ((bboxes(1:4,:)-1).*this.ROIScale)+1;

        end
        
        function outputSize = forwardPropagateSize(this, inputSize)
            % Output size is the grid size by number-of-maps in input.
            outputMaps = inputSize{1}(3);
            outputSize = [this.OutputSize outputMaps];
        end
        
        function this = inferSize(this, ~)
            % no-op
        end
        
        function tf = isValidInputSize(~, inputSize)
            % isValidInputSize Check if the layer can accept an input of
            % a certain size.
            roiSize = inputSize{2};
                                    
            % Second input is available, check if it is 4/5-by-M. 
            tf = (roiSize(1) == 4 | roiSize(1) == 5);
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
            this.ExecutionStrategy = nnet.internal.cnn.layer.util.ROIAlignHostStrategy();
        end
        
        function this = setupForGPUPrediction(this)
            this.ExecutionStrategy = nnet.internal.cnn.layer.util.ROIAlignGPUStrategy();
        end
        
        function this = setupForHostTraining(this)
            this.ExecutionStrategy = nnet.internal.cnn.layer.util.ROIAlignHostStrategy();
        end
        
        function this = setupForGPUTraining(this)
            this.ExecutionStrategy = nnet.internal.cnn.layer.util.ROIAlignGPUStrategy();
        end
        
        function this = setupForFunctionalStrategy(this)
             this = setFunctionalStrategy(this);
        end
    end
    
    methods(Access=protected)
        function this = setFunctionalStrategy(this)
            this.ExecutionStrategy = ...
                nnet.internal.cnn.layer.util.ROIAlignFunctionalStrategy();
        end
    end
       
end

%--------------------------------------------------------------------------
function roi = iAppendImageIndexIfRequired(roi)
if size(roi,2) == 4
    roi = [roi ones(size(roi,1),1,'like',roi)];
end
end

%--------------------------------------------------------------------------
% Handle default 'auto' for samplingratio
function ratioOut = iConvertSamplingRatioToNum(ratioIn)
   if(ischar(ratioIn))
       ratioOut = [-1, -1];
   else
       ratioOut = ratioIn;
   end
end
