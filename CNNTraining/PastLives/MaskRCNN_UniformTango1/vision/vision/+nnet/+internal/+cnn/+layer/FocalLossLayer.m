classdef FocalLossLayer < nnet.internal.cnn.layer.ClassificationLayer
    % FocalLossLayer

    %   Copyright 2019-2022 The MathWorks, Inc.

    %----------------------------------------------------------------------

    properties(SetAccess = private)
        % InputNames   FocalLossLayer layer has one input
        InputNames = {'in'}

        % OutputNames   FocalLossLayer layer has no outputs
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

    %----------------------------------------------------------------------
    properties (Constant)
        % DefaultName   Default layer's name.
        DefaultName = 'focallossoutput'
    end

    %----------------------------------------------------------------------
    properties (SetAccess = private)
        % NumClasses   Number of classes
        NumClasses

        % Gamma (scalar positive real) is the focusing parameter.
        Gamma

        % Alpha (scalar positive real) is the balancing parameter.
        Alpha
    end

    %----------------------------------------------------------------------
    properties(SetAccess = private, GetAccess = public)
        % OutputSize    A 2 element vector of [H C] or 3 element vector of
        %               [H W C] or 4 element vector of [H W D C] defining
        %               the size of the output.
        OutputSize
    end

    %----------------------------------------------------------------------
    properties
        % ChannelDim  The channel dimension in the input. Leave as [] to
        %             infer later.
        ChannelDim
    end

    %----------------------------------------------------------------------
    properties (Dependent, SetAccess = private)
        % HasSizeDetermined   True for layers with size determined. Both
        %                     OutputSize and ChannelDim need to be set for
        %                     this to be true.
        HasSizeDetermined
    end

    %----------------------------------------------------------------------
    properties (Access = private)
        % DeviceStrategy either moves data to the GPU or to the Host
        DeviceStrategy
    end

    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        function this = FocalLossLayer(name, classes, alpha, gamma)
            % Output  Constructor for the layer
            % creates a focal loss layer with the following parameters:
            %
            %   Name                - Name for the layer
            %   Classes             - Categorical array
            %   Alpha               - Numeric scalar for balancing
            %                         parameter
            %   Gamma               - Numeric scalar for focusing
            %                         parameter

            this.Name = name;
            if(isempty(classes))
                this.NumClasses = [];
                this.Categories = categorical();
            else
                this.NumClasses = numel(classes);
                this.Categories = classes;
            end

            this.Alpha = alpha;
            this.Gamma = gamma;
            this.ChannelDim = [];
            this.OutputSize = [];

            this.DeviceStrategy = vision.internal.cnn.layer.util.ToHostStrategy();
        end

        %------------------------------------------------------------------
        function tf = get.HasSizeDetermined(this)
            tf = ~isempty(this.OutputSize) && ~isempty(this.ChannelDim);
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
            if isempty(this.ChannelDim)
                this.ChannelDim = numel(inputSize);
            end

            this.NumClasses = inputSize(this.ChannelDim);
            this.OutputSize = inputSize(1:this.ChannelDim);
        end

        %------------------------------------------------------------------
        function tf = isValidInputSize(this, inputSize)
            % isValidInputSize   Check if the layer can accept an input of
            % a certain size.
            tf = this.HasSizeDetermined;
            tf = tf && numel(inputSize)==this.ChannelDim && ...
                isequal(inputSize(this.ChannelDim), this.OutputSize(this.ChannelDim));
        end

        %------------------------------------------------------------------
        function this = initializeLearnableParameters(this, ~)
            % initializeLearnableParameters     no-op since there are no
            % learnable parameters
        end

        %------------------------------------------------------------------
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
            this.DeviceStrategy = vision.internal.cnn.layer.util.ToHostStrategy();
        end

        %------------------------------------------------------------------
        function this = setupForGPUPrediction(this)
            this.DeviceStrategy = vision.internal.cnn.layer.util.ToGPUStrategy();
        end

        %------------------------------------------------------------------
        function this = setupForHostTraining(this)
            this.DeviceStrategy = vision.internal.cnn.layer.util.ToHostStrategy();
        end

        %------------------------------------------------------------------
        function this = setupForGPUTraining(this)
            this.DeviceStrategy = vision.internal.cnn.layer.util.ToGPUStrategy();
        end

        %------------------------------------------------------------------
        function loss = forwardLoss( this, Y, T )
            % forwardLoss    Return the focal loss between estimate
            % and true responses averaged by the number of observations
            %
            % Syntax:
            %   loss = layer.forwardLoss( Y, T );
            %
            % Inputs:
            %   Y   Predictions made by network H-by-numClasses-by-numObs
            %       or H-by-W-by-numClasses-by-numObs or
            %       H-by-W-by-D-by-numClasses-by-numObs.
            %   T   Targets (actual values), H-by-numClasses-by-numObs
            %       or H-by-W-by-numClasses-by-numObs or
            %       H-by-W-by-D-by-numClasses-by-numObs.

            Y = this.DeviceStrategy.toDevice(Y);
            T = this.DeviceStrategy.toDevice(T);

            % Observations are encoded in T as non-zero values.
            numObservations = nnz(sum(T ~= 0, this.ChannelDim));
            if numObservations > 0
                boundAwayY = nnet.internal.cnn.util.boundAwayFromZero(Y);
                loss = sum( T .* this.Alpha .*((1 - boundAwayY).^this.Gamma).* log(boundAwayY), this.ChannelDim);
                loss = (-1/numObservations) * sum(loss(:));
            else
                loss = 0;
            end
        end

        %------------------------------------------------------------------
        function dX = backwardLoss( this, Y, T )
            % backwardLoss    Back propagate the derivative of the loss
            % function
            %
            % Syntax:
            %   dX = layer.backwardLoss( Y, T );
            %
            % Inputs:
            %   Y   Predictions made by network, H-by-numClasses-by-numObs
            %       or H-by-W-by-numClasses-by-numObs or
            %       H-by-W-by-D-by-numClasses-by-numObs.
            %   T   Targets (actual values), H-by-numClasses-by-numObs
            %       or H-by-W-by-numClasses-by-numObs or
            %       H-by-W-by-D-by-numClasses-by-numObs.

            Y = this.DeviceStrategy.toDevice(Y);
            T = this.DeviceStrategy.toDevice(T);

            numObservations = nnz(sum(T ~= 0, this.ChannelDim));
            if numObservations > 0
                boundAwayY = nnet.internal.cnn.util.boundAwayFromZero(Y);

                % Ensure (1-Y)^Gamma does not explode for Y = 1, Gamma < 1.
                boundAwayOmY = nnet.internal.cnn.util.boundAwayFromZero(1-Y);

                dX = this.Alpha .* T .* ( ...
                     +(this.Gamma .* (boundAwayOmY).^(this.Gamma-1) .* log(boundAwayY)) ...
                     -((boundAwayOmY.^this.Gamma) ./ (boundAwayY)) ...
                     ) ./ numObservations;

            else
                dX = zeros(size(Y), 'like', Y);
            end
        end
    end

end

%--------------------------------------------------------------------------
function cats = iDefaultCategories(numClasses)
% Set the default Categories
cats = categorical(1:numClasses)';
end
