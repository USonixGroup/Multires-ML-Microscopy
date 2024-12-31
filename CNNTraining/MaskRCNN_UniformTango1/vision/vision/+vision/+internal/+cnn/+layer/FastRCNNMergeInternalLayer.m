classdef FastRCNNMergeInternalLayer < nnet.internal.cnn.layer.Layer
%

%   Copyright 2018-2020 The MathWorks, Inc.
    
    properties
        % LearnableParameters   Learnable parameters for the layer
        % (Vector of nnet.internal.cnn.layer.LearnableParameter)
        LearnableParameters
        
        % Name (char array)
        Name
    end
    
    properties (SetAccess = private)
        % InputNames   The names of the inputs for this layer
        %   This should be defined as a row cell array of char arrays. If
        %   there is only one input, then it should still be in a cell.
        %   Note that the ordering of the names determines the order of the
        %   inputs.
        InputNames = {'cls','reg','bboxes'}
        
        % OutputNames   The names of the outputs for this layer
        %   This should be defined as a row cell array of char arrays. If
        %   there is only one output, then it should still be in a cell.
        %   Note that the ordering of the names determines the order of the
        %   outputs.
        OutputNames = {'out'}
        
        % HasSizeDetermined   True for layers with size determined.
        HasSizeDetermined = false;
        
        % NumClasses Number of classes (excludes background). Inferred.
        NumClasses
    end
    
    properties (Constant)
        % DefaultName   Default layer's name. This will be assigned in case
        % the user leaves an empty name.
        DefaultName = 'fast-rcnn-merge'
    end
    
    methods
        
        function this = FastRCNNMergeInternalLayer(name, numClasses)
            this.Name = char(name);
            if nargin == 2
                this.NumClasses = numClasses;
                this.HasSizeDetermined = true;
            end
            this.NeedsXForBackward = false;
            this.NeedsZForBackward = false;
        end
        
        function Z = predict(~, X)
            %Xcls      = X{1}; % [1 1 C       N]
            %Xreg      = X{2}; % [1 1 4*(C-1) N]
            %proposals = X{3}; % [N 5]
            if isempty(X{3})
                % No proposals generated. Just merge first two inputs.
                % Because proposals are empty, the ROI pooling layer is not
                % actually pooling anything and just fowarding dummy data.
                Z = cat(3,X{1},X{2}); % [1 1 C+(4*(C-1)) N]
            else
                X{3} = transpose(X{3});
                X{3} = reshape(X{3},1,1,5,[]);
                Z = cat(3, X{:}); % [1 1 C+(4*(C-1))+5 N]
            end
        end
        
        function [dLdX, dW] = backward(this, ~, ~, dLdZ, ~)
            C = this.NumClasses + 1; % include background
            R = this.NumClasses * 4;
           
            N = size(dLdZ,4);
            dLdX{1} = dLdZ(:,:,1:C,:);     % [1 1 C       N]
            dLdX{2} = dLdZ(:,:,C+1:C+R,:); % [1 1 4*(C-1) N]

            % no gradient for boxes.
            dLdX{3} = zeros([N 5],'like',dLdZ);
            dW = [];
        end
        
        % forwardPropagateSize    The size of the output from the layer for
        % a given size of input
        function outputSize = forwardPropagateSize(~, inputSize)
            C = inputSize{1}(3); % include background
            R = (C-1)*4;         % regression only includes foreground. There is a separate regressor for each class.
            outputSize = [inputSize{1}(1:2) C+R+5];
        end
        
        % inferSize    Infer the size of the learnable parameters based
        % on the input size
        function this = inferSize(this, inputSize)
            sz = inputSize{1};
            this.NumClasses = sz(3) - 1; % exclude background.
            this.HasSizeDetermined = true;
        end
        
        % isValidInputSize   Check if the layer can accept an input of a
        % certain size
        function tf = isValidInputSize(~,inputSize)
            tf = iscell(inputSize) && numel(inputSize)==3;
        end
        
        %------------------------------------------------------------------
        % initializeLearnableParameters    Initialize learnable parameters
        % using their initializer
        function this = initializeLearnableParameters(this, ~)
            % no-op: crop has no learnable parameters.
        end
        
        %------------------------------------------------------------------
        % prepareForTraining   Prepare the layer for training
        function this = prepareForTraining(this)
            % no-op: crop has no learnable parameters.
        end
        
        %------------------------------------------------------------------
        % prepareForPrediction   Prepare the layer for prediction
        function this = prepareForPrediction(this)
            % no-op: crop has no learnable parameters.
        end
        
        %------------------------------------------------------------------
        % setupForHostPrediction   Prepare this layer for host prediction
        function this = setupForHostPrediction(this)
            % empty on purpose.
        end
        
        %------------------------------------------------------------------
        % setupForGPUPrediction   Prepare this layer for GPU prediction
        function this = setupForGPUPrediction(this)
            % empty on purpose.
        end
        
        %------------------------------------------------------------------
        % setupForHostTraining   Prepare this layer for host training
        function this = setupForHostTraining(this)
            % empty on purpose.
        end
        
        %------------------------------------------------------------------
        % setupForGPUTraining   Prepare this layer for GPU training
        function this = setupForGPUTraining(this)
            % empty on purpose.
        end
        
    end
    
end
