classdef AnchorBoxLayer < nnet.cnn.layer.Layer & nnet.internal.cnn.layer.Externalizable
    % AnchorBoxLayer  Anchor Box layer.
    %
    %   An anchor box layer is used to store the anchor boxes in an
    %   object detection network. Use anchorBoxLayer to create this layer.
    %
    %   AnchorBoxLayer properties:
    %       Name          - A name for the layer.
    %       AnchorBoxes   - An M-by-2 matrix that specifies the
    %                       [height width] of M anchor boxes.
    %
    %   Example: Create an Anchor Box layer.
    %   ------------------------------------
    %   anchorBoxes = [50 50; 100 100; 50 100; 100 50];
    %   layer = anchorBoxLayer(anchorBoxes, 'Name', 'anchorBoxLayer')
    %
    % See also anchorBoxLayer, trainSSDObjectDetector.

    % Copyright 2019 The MathWorks, Inc.

    %----------------------------------------------------------------------
    properties(Dependent)
        %Name   A name for the layer
        Name

        %AnchorBoxes dimensions of bounding boxes used as anchors
        AnchorBoxes
    end

    %----------------------------------------------------------------------
    properties(Dependent, Hidden)
        %IsClipped logical for if boxes are clipped at feature map border
        IsClipped 
    end
    
    % Get and Set methods
    %----------------------------------------------------------------------
    methods
        function val = get.Name(this)
            val = this.PrivateLayer.Name;
        end
        %------------------------------------------------------------------
        function val = get.AnchorBoxes(this)
            val = this.PrivateLayer.AnchorBoxes;
        end
        %------------------------------------------------------------------
        function val = get.IsClipped(this)
            val = this.PrivateLayer.IsClipped;
        end
        %------------------------------------------------------------------
        function this = set.Name(this, val)
            iAssertValidLayerName(val);
            this.PrivateLayer.Name = char(val);
        end
    end

    % CTOR
    %----------------------------------------------------------------------
    methods(Access = public)
        %------------------------------------------------------------------
        function this = AnchorBoxLayer(privateLayer)
            this.PrivateLayer = privateLayer;
        end
        %------------------------------------------------------------------
        function out = saveobj(this)
            out.Version     = 1.0;
            out.Name        = this.PrivateLayer.Name;
            out.AnchorBoxes = this.PrivateLayer.AnchorBoxes;
            out.IsClipped   = this.PrivateLayer.IsClipped;
        end
    end
    %----------------------------------------------------------------------
    methods (Hidden, Access = public)
        %------------------------------------------------------------------
        function boxes = getBoxes(this, inputImageSize)
            boxes = this.PrivateLayer.getBoxes(inputImageSize);
        end
        % Used only for testing
        %------------------------------------------------------------------
        function this = inferSize(this, featureMapSize)
            this.PrivateLayer = this.PrivateLayer.inferSize(featureMapSize);
        end
    end

    % Load object
    %----------------------------------------------------------------------
    methods(Static)
        %------------------------------------------------------------------
        function this = loadobj(in)
            internalLayer = nnet.internal.cnn.layer.AnchorBoxLayer(in.Name, ...
                                in.AnchorBoxes, ...
                                in.IsClipped);
            this = nnet.cnn.layer.AnchorBoxLayer(internalLayer);
        end
    end

    % Helpers
    %----------------------------------------------------------------------
    methods(Access = protected)
        %------------------------------------------------------------------
        function [description, type] = getOneLineDisplay(~)
            description = getString(message('vision:ssd:anchorBoxDescription'));
            type = getString(message('vision:ssd:anchorBoxType'));
        end
        
        %------------------------------------------------------------------
        function groups = getPropertyGroups( this )
            hyperparameters = {
                'AnchorBoxes'
                };
            groups = [
                this.propertyGroupGeneral({'Name'})
                this.propertyGroupHyperparameters(hyperparameters)
                ];
        end        
    end
end

%--------------------------------------------------------------------------
function iAssertValidLayerName(name)
    nnet.internal.cnn.layer.paramvalidation.validateLayerName(name);
end