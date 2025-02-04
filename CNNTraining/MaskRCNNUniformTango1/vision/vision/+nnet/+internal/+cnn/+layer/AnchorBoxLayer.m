classdef AnchorBoxLayer < nnet.internal.cnn.layer.Layer
    % AnchorBoxLayer

    %   Copyright 2019 The MathWorks, Inc.

    %----------------------------------------------------------------------
    properties
        % Name (char array)   A name for the layer
        Name

        % LearnableParameters   Learnable parameters for the layer
        %   This layer has no learnable parameters.
        LearnableParameters = nnet.internal.cnn.layer.learnable.PredictionLearnableParameter.empty();
    end

    %----------------------------------------------------------------------
    properties (SetAccess = protected)
        %AnchorBoxes
        AnchorBoxes

        %IsClipped true if boxes are clipped at feature map border.
        %          Set to true by default.
        IsClipped
    end
    %----------------------------------------------------------------------
    properties (SetAccess = protected)
        %NumBoxesPerGrid Scalar integer indicating the number of anchor
        %   boxes per grid cell.
        NumBoxesPerGrid

        %FeatureMapSize Size of the feature map that this layer is attached
        %   to.
        FeatureMapSize
    end

    %----------------------------------------------------------------------
    properties (SetAccess = private)
        % InputNames   The names of the inputs for this layer
        %   This should be defined as a row cell array of char arrays. If
        %   there is only one input, then it should still be in a cell.
        %   Note that the ordering of the names determines the order of the
        %   inputs.
        InputNames = {'in'}

        % OutputNames   The names of the outputs for this layer
        %   This should be defined as a row cell array of char arrays. If
        %   there is only one output, then it should still be in a cell.
        %   Note that the ordering of the names determines the order of the
        %   outputs.
        OutputNames = {'out'}

        %HasSizeDetermined   True for layers with size determined.
        HasSizeDetermined = false
    end

    %----------------------------------------------------------------------
    properties (Constant)
        % DefaultName   Default layer's name.
        DefaultName = 'AnchorBoxLayer'
    end

    % CTOR
    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        function this = AnchorBoxLayer(name, ...
                            anchorBoxes, ...
                            isClipped)
            this.Name              = name;
            this.AnchorBoxes       = anchorBoxes;
            this.NumBoxesPerGrid   = size(anchorBoxes, 1);
            this.IsClipped         = isClipped;
            this.NeedsXForBackward = false; 
            this.NeedsZForBackward = false;
        end
    end

    % Abstract Implementations
    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        function Z = predict(~, X)
            % pass through.
            Z = X;
        end
        %------------------------------------------------------------------
        function dX = backward(~, ~, ~, dZ, ~)
            % pass through.
            dX = dZ;
        end
        %------------------------------------------------------------------
        function outputSize = forwardPropagateSize(~, inputSize)
            % forwardPropagateSize  Output the size of the layer based on
            % the input size
            outputSize = inputSize;
        end
        %------------------------------------------------------------------
        function this = inferSize(this, inputSize)
            % inferSize    Infer the number of classes and the observation
            % dimension, based on the input dimensions
            this.FeatureMapSize = inputSize(1:2);
            this.HasSizeDetermined = true;
        end
        %------------------------------------------------------------------
        function tf = isValidInputSize(~, ~)
            % isValidInputSize   Check if the layer can accept an input of
            % a certain size.
            tf = true;
        end
        %------------------------------------------------------------------
        function this = initializeLearnableParameters(this, ~)
            % initializeLearnableParameters     no-op since there are no
            % learnable parameters
        end
        %------------------------------------------------------------------
        function this = prepareForTraining(this)
            this.LearnableParameters = nnet.internal.cnn.layer.learnable.TrainingLearnableParameter.empty();
        end
        %------------------------------------------------------------------
        function this = prepareForPrediction(this)
            this.LearnableParameters = nnet.internal.cnn.layer.learnable.PredictionLearnableParameter.empty();
        end
        %------------------------------------------------------------------
        function this = setupForHostPrediction(this)
        end
        %------------------------------------------------------------------
        function this = setupForGPUPrediction(this)
        end
        %------------------------------------------------------------------
        function this = setupForHostTraining(this)
        end
        %------------------------------------------------------------------
        function this = setupForGPUTraining(this)
        end
        %------------------------------------------------------------------
        function this = storeResponseMetaData(this, ~)
        end
    end

    % Hidden API
    %----------------------------------------------------------------------
    methods (Hidden)
        %------------------------------------------------------------------
        % anchorBoxes returned by this function are in normalized with
        % respect to input image size and are in [x,y,w,h] format.
        function anchorBoxes = getBoxes(this, inputImageSize)
            layerWidth = this.FeatureMapSize(2);
            layerHeight = this.FeatureMapSize(1);

            imageWidth = inputImageSize(2);
            imageHeight = inputImageSize(1);

            % define prior boxes shapes
            boxWidth  = 0.5 * this.AnchorBoxes(:, 2);
            boxHeight = 0.5 * this.AnchorBoxes(:, 1);

            % define centers of prior boxes
            stepX = imageWidth / layerWidth;
            stepY = imageHeight / layerHeight;
            linx = (0.5 * stepX):stepX:(imageWidth - 0.5 * stepX);
            liny = (0.5 * stepY):stepY:(imageHeight - 0.5 * stepY);
            [centersY, centersX] = meshgrid(linx, liny);
            centersX = centersX(:);
            centersY = centersY(:);

            % define xmin, ymin, xmax, ymax of prior boxes
            numGridCells = numel(centersX);
            numAnchorBoxes = numGridCells * this.NumBoxesPerGrid;
            anchorBoxes = zeros(numAnchorBoxes, 4);

            pbIdx = 1;
            for bIdx = 1:this.NumBoxesPerGrid
                thisBoxWidth = boxWidth(bIdx);
                thisBoxHeight = boxHeight(bIdx);
                for cIdx = 1:numGridCells
                    x1 = (centersX(cIdx) - thisBoxWidth)/imageWidth;
                    y1 = (centersY(cIdx) - thisBoxHeight)/imageHeight;
                    x2 = (centersX(cIdx) + thisBoxWidth)/imageWidth;
                    y2 = (centersY(cIdx) + thisBoxHeight)/imageHeight;
                    anchorBoxes(pbIdx, :) = [x1 y1 x2 y2];
                    pbIdx = pbIdx + 1;
                end
            end
            
            % Axis align coordinates of normalized boxes having negative 
            % and zero coordinate values.
            if this.IsClipped
                anchorBoxes(:,1) = min(max(anchorBoxes(:,1), 1/imageWidth), 1.0);
                anchorBoxes(:,2) = min(max(anchorBoxes(:,2), 1/imageHeight), 1.0);
            end

            % Convert to xywh format.
            anchorBoxes(:, 3) = anchorBoxes(:, 3) - anchorBoxes(:, 1);
            anchorBoxes(:, 4) = anchorBoxes(:, 4) - anchorBoxes(:, 2);
        end
    end
end