classdef ROIMaxPooling2DLayer < nnet.internal.cnn.layer.Layer
    %

    %   Copyright 2016-2022 The MathWorks, Inc.

    properties
        % LearnableParameters   Learnable parameters for the layer
        %   This layer has no learnable parameters.
        LearnableParameters = nnet.internal.cnn.layer.learnable.PredictionLearnableParameter.empty();

        % Name (char array)   A name for the layer
        Name

        % ScaleFactor ([sx sy]) Used to scale boxes from feature map to
        %             image.
        ScaleFactor = [1 1]
    end

    properties (Constant)
        % DefaultName   Default layer's name.
        DefaultName = 'ROI Pooling'
    end

    properties (SetAccess = private)
        % InputNames   This layer has two inputs
        InputNames = {'in','roi'}

        % OutputNames   This layer has one output
        OutputNames = {'out'}

        % HasSizeDetermined   Specifies if all size parameters are set
        HasSizeDetermined = true;

        % GridSize The height and width to divide each ROI. Each grid cell
        % is then max pooled.
        GridSize
    end

    properties(Access = protected)
        ExecutionStrategy
    end

    methods
        function val = get.ScaleFactor(this)
            val = vision.internal.cnn.layer.util.PropertyCache.fetchValue(this.ScaleFactor);
        end
    end

    methods
        function this = ROIMaxPooling2DLayer(name, varargin)
            narginchk(1,3);
            this.Name = name;
            N = numel(varargin);
            if N >= 1
                this.GridSize = varargin{1};
            end
            if N >= 2
                this.ScaleFactor = varargin{2};
            end

            this.ExecutionStrategy = nnet.internal.cnn.layer.util.ROIMaxPooling2DHostStrategy();
        end

        function Z = predict(this, X)
            roi = iAppendImageIndexIfRequired(X{2}); % [x1 y1 x2 y2] format. In image space.
            if isempty(roi)
                outputSize = [this.GridSize size(X{1},3) size(X{1},4)];
                Z = zeros(outputSize,'like',X{1});
            else
                scaledBBoxes = this.scaleBoxesToFeatureSpace( gather(roi) );
                % TODO update internal ROI pooling to accept gpuArray ROIs
                Z = this.ExecutionStrategy.forward(X{1}, scaledBBoxes, this.GridSize);
            end
        end

        function [Z, memory] = forward(this, X )
            roi = iAppendImageIndexIfRequired(gather(X{2}));
            if isempty(roi)
                % Zero ROI input. Return NaN feature maps in this case.
                outputSize = [this.GridSize size(X{1},3) size(X{1},4)];
                Z = NaN(outputSize,'like',X{1});
                memory.roi = roi;
            else
                scaledBBoxes = this.scaleBoxesToFeatureSpace( roi );
                % TODO update internal ROI pooling to accept gpuArray ROIs
                [Z, memory] = this.ExecutionStrategy.forward(X{1}, scaledBBoxes, this.GridSize);
            end
        end

        function bboxes = scaleBoxesToFeatureSpace(this,bboxes)
            % Scales boxes to feature space. Input bboxes and scaled boxes
            % are in [x1 y1 x2 y2] format.
            sx = this.ScaleFactor(1);
            sy = this.ScaleFactor(2);
            bboxes(:,1:4) = vision.internal.cnn.boxUtils.scaleX1X2Y1Y2(bboxes(:,1:4), sx, sy);
        end

        function [dX, dW] = backward(this, X, Z, dZ, memory)
            X = X{1};
            if isempty(memory.roi)
                % Backprop zero gradients. No parameter updates ought to be
                % made in this.
                dX{1} = zeros(size(X),'like',X);
                dW = [];
            else
                memory.roi = iAppendImageIndexIfRequired(memory.roi);
                [dX{1}, dW] = this.ExecutionStrategy.backward(X, Z, dZ, memory, this.GridSize);
            end
            dX{2} = 0; % no gradient for ROI input.
        end

        function Zs = forwardExampleInputs(this, Zs)

            % Validate the input data
            this.validateInputs(Zs);

            % Output size is the grid size by number-of-maps in input.
            Zs(2) = [];
            Zs{1} = setSizeForDim(Zs{1},'S',this.GridSize);
        end

        function this = configureForInputs(this,Zs)
            this.validateInputs(Zs);
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
            this.ExecutionStrategy = nnet.internal.cnn.layer.util.ROIMaxPooling2DHostStrategy();
        end

        function this = setupForGPUPrediction(this)
            this.ExecutionStrategy = nnet.internal.cnn.layer.util.ROIMaxPooling2DGPUStrategy();
        end

        function this = setupForHostTraining(this)
            this.ExecutionStrategy = nnet.internal.cnn.layer.util.ROIMaxPooling2DHostStrategy();
        end

        function this = setupForGPUTraining(this)
            this.ExecutionStrategy = nnet.internal.cnn.layer.util.ROIMaxPooling2DGPUStrategy();
        end
    end

    methods(Access=private)
        function validateInputs(this, Zs)
            % Validate that the input placeholder arrays Zs are of valid
            % size.
            inSize = size(Zs{1});
            roiSize = size(Zs{2});

            % Input must be at least the size of the grid.
            if ~all(inSize(1:2) >= this.GridSize)
                error(message("vision:rcnn:incorrectInputROIPoolGridSize",iSizeToString(inSize),iSizeToString(this.GridSize)));
            end

            % Second input is available, check if it is M-by-4/5.
            if ~(any(roiSize(2) == [4,5]))
                error(message("vision:rcnn:incorrectInputROIPoolSecondInput",iSizeToString(roiSize)));
            end
        end
    end

end

%--------------------------------------------------------------------------
function roi = iAppendImageIndexIfRequired(roi)
if size(roi,2) == 4
    roi = [roi ones(size(roi,1),1,'like',roi)];
end
end

function str = iSizeToString(sz)
str = join(string(sz), matlab.internal.display.getDimensionSpecifier);
end
