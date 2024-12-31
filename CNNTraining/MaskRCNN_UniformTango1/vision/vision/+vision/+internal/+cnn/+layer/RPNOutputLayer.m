classdef RPNOutputLayer < nnet.cnn.layer.Layer & nnet.internal.cnn.layer.Externalizable
% The external layer that contains an internal layer which implements
% the RPN output layer needed for datastores inputs in faster R-CNN.
%

% Copyright 2019 The MathWorks, Inc.
    properties(Dependent)
        % Name   A name for the layer
        %   The name for the layer. If this is set to '', then a name will
        %   be automatically set at training time.
        Name
    end

    methods
        function this = RPNOutputLayer(privateLayer)
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
            description = 'rpn output';
            type = 'RPNOutput';
        end

    end
end
