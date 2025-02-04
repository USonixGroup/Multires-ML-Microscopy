classdef ROIInputLayer < nnet.internal.cnn.layer.InputLayer
    %

    % Copyright 2018-2023 The MathWorks, Inc.
    
    properties
        % Name (char array)   A name for the layer
        Name
    end

    properties (Constant)
        % DefaultName   Default layer's name. This will be assigned in case
        % the user leaves an empty name.
        DefaultName = 'roiInput'
    end
    
    properties(SetAccess = private)
        % InputSize Input size vector.
        InputSize
        
        % InputData (deep.internal.PlaceholderArray)   Sample input data
        InputData       
        
        % HasSizeDetermined   True for layers with size determined.
        HasSizeDetermined = true;
    end
    
    methods
        
        function this = ROIInputLayer(name)
            this.Name = name;
            this.InputSize = [1 4];
            this.InputData = deep.internal.PlaceholderArray([NaN 4 NaN],'SCB');
        end
        
        function X = predict(layer, X)
            % predict   Forward propagation at test time
            if ~isreal(X)
                error(message("deep:dlarray:ComplexNotSupported"));
            end
            if layer.IsInFunctionalMode && isdlarray(X)
                X = deep.internal.dlarray.errorOnComplexGradientInputs(X,'roiInput');
                X = deep.internal.dlarray.errorOnComplexGradientOutputs(X,'roiInput');
            end
        end
        
        function [dX,dW] = backward( ~, ~, ~, ~, ~ )
            % backward  Return empty value
            dX = [];
            dW = [];
        end
        
        function tf = isValidInputSize(~, ~)
            % isValidInputSize   Overload to allow any input size
            tf = true;
        end

        function fmt = getDefaultInputFormat(~)
            % getDefaultInputFormat   Get default input dlarray format labels.
            fmt = 'SC';
        end
        
        function this = initializeLearnableParameters(this, ~)
            % initializeLearnableParameters     no-op since there are no
            %                                   learnable parameters
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
    end
end
