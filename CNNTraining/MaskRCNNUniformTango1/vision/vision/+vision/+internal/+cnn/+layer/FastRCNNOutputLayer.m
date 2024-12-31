classdef FastRCNNOutputLayer < nnet.cnn.layer.Layer & nnet.internal.cnn.layer.Externalizable
%

%   Copyright 2018-2020 The MathWorks, Inc.

    properties(Dependent)
        % Name   A name for the layer
        %   The name for the layer. If this is set to '', then a name will
        %   be automatically set at training time.
        Name
    end
    
    methods
        
        function this = FastRCNNOutputLayer(privateLayer)
            this.PrivateLayer = privateLayer;
        end
        
        function val = get.Name(this)
            val = this.PrivateLayer.Name;
        end
        
        function this = set.Name(this, val)
            nnet.internal.cnn.layer.paramvalidation.validateLayerName(val);
            this.PrivateLayer.Name = char(val);
        end
    end
    methods(Hidden, Access = protected)
        function [description, type] = getOneLineDisplay(~)
            description = 'fast-rcnn output';
            type = 'FastRCNNOutput';
        end
    end
end
