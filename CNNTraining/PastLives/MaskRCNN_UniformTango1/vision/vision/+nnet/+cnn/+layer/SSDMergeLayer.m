classdef SSDMergeLayer < nnet.cnn.layer.Layer & nnet.internal.cnn.layer.Externalizable
    % SSDMergeLayer  SSD Merge Layer.
    %
    %   A SSD Merge layer is used to merge the outputs of the feature maps
    %   for subsequent regression and classification loss computation.
    %
    %   SSDMergeLayer properties:
    %       Name          - A name for the layer.
    %       NumChannels   - A scalar integer indicating either the number
    %                       of classes or the number of regression
    %                       parameters to merge.
    %       NumInputs     - A scalar integer indicating the number of
    %                       inputs to this merge layer.
    %
    %
    %   Example: Create a SSD Merge layer for regression.
    %   -------------------------------------------------
    %   % Number of channels for regression is 4.
    %   numChannels = 4;
    %
    %   % Number of inputs.
    %   numInputs = 6;
    %
    %   % Create a SSD Merge layer.
    %   regressMergeLayer = ssdMergeLayer(numChannels, numInputs, 'Name', 'regressionMergeLayer')
    %
    % See also ssdMergeLayer, trainSSDObjectDetector.

    % Copyright 2019 The MathWorks, Inc.

    %----------------------------------------------------------------------
     properties(Dependent)
        % Name   A name for the layer
        %   The name for the layer. If this is set to '', then a name will
        %   be automatically set at training time.
        Name

        % NumChannels A scalar positive integer
        %    Specifying the number of classes, or the number of regression
        %    parameters at the input.
        NumChannels
    end

    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        function this = SSDMergeLayer(privateLayer)
            this.PrivateLayer = privateLayer;
        end
        %------------------------------------------------------------------
        function val = get.Name(this)
            val = this.PrivateLayer.Name;
        end
        %------------------------------------------------------------------
        function val = get.NumChannels(this)
            val = this.PrivateLayer.NumChannels;
        end
        %------------------------------------------------------------------
        function this = set.Name(this, val)
            iAssertValidLayerName(val);
            this.PrivateLayer.Name = char(val);
        end
        %------------------------------------------------------------------
        function s = saveobj(this)
            s.Version     = 1.0;
            s.Name        = this.PrivateLayer.Name;
            s.NumChannels = this.PrivateLayer.NumChannels;
            s.NumInputs   = this.PrivateLayer.NumInputs;
        end
    end
    %----------------------------------------------------------------------
    methods(Hidden,Static)
        %------------------------------------------------------------------
        function obj = loadobj(s)
            internalLayer = nnet.internal.cnn.layer.SSDMergeLayer(...
                                s.Name, ....
                                s.NumChannels, ...
                                s.NumInputs);
            obj = nnet.cnn.layer.SSDMergeLayer(internalLayer);
        end
    end
    %----------------------------------------------------------------------
    methods(Access = protected)
        %------------------------------------------------------------------
        function [description, type] = getOneLineDisplay(~)
            description = getString(message('vision:ssd:ssdMergeDescription'));
            type        = getString(message('vision:ssd:ssdMergeType'));
        end
        %------------------------------------------------------------------
        function groups = getPropertyGroups( this )
            groups = this.propertyGroupGeneral(...
                {'Name', 'NumChannels', 'NumInputs'});
        end           
    end
end

%--------------------------------------------------------------------------
function iAssertValidLayerName(name)
    iEvalAndThrow(@()...
        nnet.internal.cnn.layer.paramvalidation.validateLayerName(name));
end
%--------------------------------------------------------------------------
function iEvalAndThrow(func)
% Omit the stack containing internal functions by throwing as caller
    try
        func();
    catch exception
        throwAsCaller(exception)
    end
end