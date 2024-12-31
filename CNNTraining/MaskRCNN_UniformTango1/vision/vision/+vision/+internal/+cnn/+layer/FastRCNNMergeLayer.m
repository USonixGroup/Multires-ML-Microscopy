classdef FastRCNNMergeLayer < nnet.cnn.layer.Layer & nnet.internal.cnn.layer.Externalizable
%

%   Copyright 2018-2020 The MathWorks, Inc.

    properties(Dependent)
        % Name   A name for the layer
        %   The name for the layer. If this is set to '', then a name will
        %   be automatically set at training time.
        Name
    end
    
    methods
        
        function this = FastRCNNMergeLayer(privateLayer)
            this.PrivateLayer = privateLayer;
        end
        
        function val = get.Name(this)
            val = this.PrivateLayer.Name;
        end
        
        function this = set.Name(this, val)
            iEvalAndThrow(@()...
                nnet.internal.cnn.layer.paramvalidation.validateLayerName(val));
            this.PrivateLayer.Name = char(val);
        end
    end
    methods(Hidden, Access = protected)
        function [description, type] = getOneLineDisplay(~)
            description = 'fast-rcnn merge';
            type = 'FastRCNNMerge';
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
