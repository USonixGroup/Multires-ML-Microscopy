classdef SSDMergeLayer < nnet.internal.cnn.layer.FunctionalLayer
    % SSDMergeLayer

    %   Copyright 2019-2024 The MathWorks, Inc.

    %----------------------------------------------------------------------
    properties
        % LearnableParameters   Learnable parameters for the layer
        %   This layer has no learnable parameters.
        LearnableParameters = nnet.internal.cnn.layer.learnable.PredictionLearnableParameter.empty();

        % Name (char array)   A name for the layer
        Name

        % NumInputs A scalar positive integer
        %    Specifying the number of inputs.
        NumInputs

        % NumChannels A scalar positive integer
        %    Specifying the number of classes, or the number of regression
        %    parameters at the input.
        NumChannels
    end

    %----------------------------------------------------------------------
    properties (Constant)
        % DefaultName   Default layer's name.
        DefaultName = 'ssdMergeLayer'
    end

    %----------------------------------------------------------------------
    properties
        % Learnables   Empty
        Learnables
    end

    %----------------------------------------------------------------------
    properties(SetAccess=protected)
        % LearnablesNames   Empty
        LearnablesNames
    end

    %----------------------------------------------------------------------
    properties (SetAccess = protected)
        % IsInFunctionalMode   Returns true if layer is currently being
        % used in "functional" mode (i.e. in dlnetwork). Required by
        % FunctionalLayer interface. On construction, all layers are set up
        % for usage in DAGNetwork.
        IsInFunctionalMode = false
    end    

    %----------------------------------------------------------------------
    properties(SetAccess = private)
        % InputNames   This layer has a variable number of inputs.
        InputNames

        % OutputNames   This layer has a single output
        OutputNames = {'out'}

        % HasSizeDetermined   Specifies if all size parameters are set
        HasSizeDetermined = true
    end

    %----------------------------------------------------------------------
    properties(Access = protected)
        ExecutionStrategy
    end    

    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        function this = SSDMergeLayer(name, numChannels, numInputs)

            this.Name        = name;
            this.NumChannels = numChannels;
            this.NumInputs   = numInputs;
            this.InputNames  = cell(1, numInputs);

            % Populate the input names by indexing into number of inputs.
            for idx = 1:numInputs
                this.InputNames{idx} = char("in" + num2str(idx));
            end
            
            this.NeedsXForBackward = true; 
            this.NeedsZForBackward = false;
        end
        %------------------------------------------------------------------
        function Z = predict(this, X)
            % predict   Forward input data through the layer and output the
            %           result
            
            % X is organized as a cell array, each element corresponding to
            % one feature map. The contents of X are in the shape of
            % [M-by-N-by-C-by-B] where M is the width of the feature map, N
            % is the height, C is the depth, and B is the number of
            % batches. The output is of size F-by-1-by-C-by-B, where F is
            % the number of anchor boxes for all the feature maps (in order
            % of their appearance in X).
            sizeOfZ = 0;
            
            if ~iscell(X)
                X = {X};
            end
            
            for idx = 1:this.NumInputs
                [h, w, c, b] = size(X{idx});
                numBoxesPerGrid = c/this.NumChannels;
                sizeOfZ = sizeOfZ + h*w*numBoxesPerGrid;
            end
            if isa(X{1,1},'dlarray')
                fmt = dims(X{1,1});
                Z = zeros(sizeOfZ, 1, this.NumChannels, b, 'like', X{1,1});
                Z = dlarray(Z,fmt);
            else
                Z = zeros(sizeOfZ, 1, this.NumChannels, b, 'like', X{1,1});
            end
            
            idxZ = 1;
            for idx = 1:this.NumInputs
                [h, w, c, b] = size(X{idx});
                
                % Make row order instead of column order.
                thisX = permute(X{idx}, [2 1 3 4]);
                
                numBoxesPerGrid = c/this.NumChannels;
                thisX = reshape(thisX, [h*w*numBoxesPerGrid, 1, this.NumChannels, b]);
                
                % Convert Z to gpuArray to handle cpu and gpuArray data.
                if isgpuarray(thisX)
                    Z = gpuArray(Z);
                end
                
                Z(idxZ:idxZ + h*w*numBoxesPerGrid - 1, :, :, :) = thisX;
                idxZ = idxZ + h*w*numBoxesPerGrid;
            end
        end
        %------------------------------------------------------------------
        function [dX,dW] = backward(this, X, ~, dZ, ~)
            % backward    Back propagate the derivative of the loss
            %             function through one layer
            
            % Need X here only for determining sizes. Reshape dZ correctly,
            % and send it back to it's respective branches. See forward for
            % a description of how X is arranged.
            if ~iscell(X)
                X = {X};
            end

            dX = cell(1, this.NumInputs);
            numBoxesCovered = 1;
            for idx = 1:this.NumInputs
                [h, w, c, b] = size(X{idx});
                numBoxesPerGrid = c/this.NumChannels;
                thisNumBoxes = h*w*numBoxesPerGrid;

                thisdZ = dZ(numBoxesCovered:numBoxesCovered + thisNumBoxes - 1, :, :, :);
                numBoxesCovered = numBoxesCovered + thisNumBoxes;

                % Extract the right contents of dZ.
                thisdZ = reshape(thisdZ, [w, h, numBoxesPerGrid * this.NumChannels, b]);

                % Revert the row-order change made earlier.
                thisdZ = permute(thisdZ, [2 1 3 4]);

                % Push it into the specific layer that it came from.
                dX{idx} = thisdZ;
            end

            % dW is empty for all the inputs.
            dW = repmat(cell(0), 1, this.NumInputs);

            if numel(dX) == 1
                dX = dX{1};
                dW = [];
            end
        end
        %------------------------------------------------------------------
        function outputSize = forwardPropagateSize(this, inputSize)
            % Handle single input case.
            if ~iscell(inputSize)
                inputSize = {inputSize};
            end

            totalBoxes = 0;
            for idx = 1:this.NumInputs
                h = inputSize{idx}(1);
                w = inputSize{idx}(2);
                c = inputSize{idx}(3);
                numBoxesPerCell = c/this.NumChannels;
                numBoxesThisMap = h * w * numBoxesPerCell;
                totalBoxes = totalBoxes + numBoxesThisMap;
            end
            outputSize = [totalBoxes, 1, this.NumChannels];
        end
        %------------------------------------------------------------------
        function this = inferSize(this, ~)
        end
        %------------------------------------------------------------------
        function tf = isValidInputSize(this, inputSize)
            % isValidInputSize True if valid size is [H W C].

            % Handle single input case.
            if ~iscell(inputSize)
                inputSize = {inputSize};
            end

            % Ensure the number of inputs matches what the layer is setup
            % with.
            tf = numel(inputSize) == this.NumInputs;

            if tf
                % Ensure that there is at least three dimensions in the
                % input.
                for idx = 1:this.NumInputs
                    if numel(inputSize{idx}) < 3
                        tf = false;
                        break;
                    end
                end
            end
        end
        %------------------------------------------------------------------
        function this = initializeLearnableParameters(this, ~)
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
    end
    methods(Access=protected)
        function this = setFunctionalStrategy(this)
            this.ExecutionStrategy = nnet.internal.cnn.layer.util.ssdMergeFunctionalStrategy();
        end
    end
end
