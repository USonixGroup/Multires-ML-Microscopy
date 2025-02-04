%ROIInputLayer ROI input layer.
%
%   Use roiInputLayer to create this layer. This layer is used in Fast
%   R-CNN object detection networks.
%
%   <a href="matlab:helpview('vision','rcnnConcept')">Learn more about Fast R-CNN.</a>
%
%   ROIInputLayer properties:
%      Name         - The layer name.
%      NumOutputs   - The number of outputs of the layer.
%      OutputNames  - The names of the outputs of the layer.
%
%   Example - Create an ROI input layer
%   -----------------------------------
%   roiInput = roiInputLayer('Name','roiInput')
%
%   See also roiInputLayer, trainFastRCNNObjectDetector.

% Copyright 2018-2023 The MathWorks, Inc.

classdef ROIInputLayer < nnet.cnn.layer.Layer & nnet.internal.cnn.layer.Externalizable & ...
       nnet.internal.cnn.layer.Summarizable
    
    properties(Dependent)
        % Name   A name for the layer
        %   The name for the layer. If this is set to '', then a name will
        %   be automatically set at training time.
        Name
    end
    
    methods
        function this = ROIInputLayer(privateLayer)
            this.PrivateLayer = privateLayer;
        end
        %------------------------------------------------------------------
        function val = get.Name(this)
            val = this.PrivateLayer.Name;
        end
        
        %------------------------------------------------------------------
        function this = set.Name(this, name)
            iEvalAndThrow(@()...
                nnet.internal.cnn.layer.paramvalidation.validateLayerName(name));
            this.PrivateLayer.Name = char(name);
        end
        
        %------------------------------------------------------------------
        function s = saveobj(this)
            s.Version    = 1.0;
            s.Name       = this.PrivateLayer.Name;
        end
    end
    
    methods(Access = protected)
        function [description,type] = getOneLineDisplay(~)
            description = getString(message('vision:rcnn:roiInputDesc'));
            type = getString(message('vision:rcnn:roiInputType'));
        end
        
        function groups = getPropertyGroups( this )
            groups = this.propertyGroupGeneral( {'Name'} );
        end

        function str = getOneLineSummary(~)
            str = getString(message('vision:rcnn:roiInputDesc'));
        end
    end
    
    %----------------------------------------------------------------------
    methods(Hidden,Static)
        function obj = loadobj(s)
            internalLayer = nnet.internal.cnn.layer.ROIInputLayer(s.Name);
            obj = nnet.cnn.layer.ROIInputLayer(internalLayer);
        end
    end
    
end

function iEvalAndThrow(func)
% Omit the stack containing internal functions by throwing as caller
try
    func();
catch exception
    throwAsCaller(exception)
end
end
