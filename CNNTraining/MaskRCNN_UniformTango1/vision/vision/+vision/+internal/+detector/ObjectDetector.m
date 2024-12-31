classdef (Abstract) ObjectDetector < vision.internal.EnforceScalarValue 
    % ObjectDetector   Abstract class for object detectors.
    
    % Copyright 2021 The MathWorks, Inc.
    
    properties
        % ModelName (char vector)   A name for the detector.
        ModelName = '';
    end
    
    methods (Abstract)
        % predict   Predicts the model activations using the input data.
        %
        % Syntax
        %   varargout = predict(aDetector, X)
        %
        % Inputs
        %   aDetector - Detector object.
        %   X         - Input to the model.
        %
        % Output
        %   varargout - Activations obtained from the model.
        varargout = predict(aDetector, varargin)

        % preprocess   Preprocess input data to required format.
        %
        % Syntax
        %   varargout = preprocess(aDetector, X)
        %
        % Inputs
        %   aDetector - Detector object.
        %   X         - Input to be preprocessed.
        %
        % Output
        %   varargout - Preprocessed output.        
        varargout = preprocess(aDetector, varargin)

        % postprocess   Postprocess function that converts predicted 
        %               activations to detections.
        %
        % Syntax
        %   varargout = postprocess(aDetector, X)
        %
        % Inputs
        %   aDetector - Detector object.
        %   X         - Predicted activations.
        %
        % Output
        %   varargout - Predicted detections.        
        varargout = postprocess(aDetector, varargin)
    end
    
    
    methods(Hidden, Access = protected)
        
        function varargout = performDetect(this, I, params)
            % performDetect  Defines the steps to compute the detections
            %                using the input data.
            %
            % Syntax
            %   varargout = performDetect(this, I, params)
            %
            % Inputs
            %   this      - Detector object.
            %   I         - Input to the detector.
            %   params    - Structure that captures the data related to 
            %               objectDetector.detect NVP's.
            %
            % Output
            %   varargout - Predicted detections.
            [Ipreprocessed, info]  = this.preprocess(I, params);
            features = this.predict(Ipreprocessed, params);
            [varargout{1:nargout}] = this.postprocess(features, info, params);
        end
    end
    
    methods
        function layer = set.ModelName( layer, val )
            iEvalAndThrow(@()iAssertValidDetectorName( val ));
            layer.ModelName = convertStringsToChars( val );
        end         
    end
    
    methods (Static, Hidden, Access = public)
        function name = matlabCodegenRedirect(~)
            % vision.internal.detector.ObjectDetector is not supported for
            % codegen. vision.internal.detector.coder.ObjectDetector is the
            % implementation of vision.internal.detector.ObjectDetector that
            % supports codegen for inference detect when generating code
            % for any custom layer inheriting from
            % vision.internal.cnn.ObjectDetector class, we will redirect it
            % to the codegenable implementation.
            %
            % Any changes made in vision.vision.Detector that affects the
            % detect call needs to be implemented in
            % vision.internal.cnn.coder.ObjectDetector as well.
            name = 'ObjectCodegenDetector';
        end

        %------------------------------------------------------------------
        % Validate Threshold value.
        %------------------------------------------------------------------
        function checkThreshold(threshold,callerName)
            validateattributes(threshold, {'single', 'double'}, {'nonempty', 'nonnan', ...
                'finite', 'nonsparse', 'real', 'scalar', '>=', 0, '<=', 1}, ...
                callerName, 'Threshold');
        end        
    end
end

function iAssertValidDetectorName( name )
iEvalAndThrow(@()...
    nnet.internal.cnn.layer.paramvalidation.validateLayerName( name ));
end

function varargout = iEvalAndThrow(func)
% Omit the stack containing internal functions by throwing as caller.
try
    [varargout{1:nargout}] = func();
catch exception
    throwAsCaller(exception)
end
end
