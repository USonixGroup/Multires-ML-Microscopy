classdef FocalLossLayer < nnet.cnn.layer.Layer & nnet.internal.cnn.layer.Externalizable
    % FocalLossLayer  Focal Loss Layer.
    %
    %   A focal loss layer is used to predict object classes layer using
    %   the focal loss function. Use focalLossLayer to create this layer.
    %
    %   FocalLossLayer properties:
    %       Name          - A name for the layer.
    %       Classes       - A categorical vector listing the classes
    %                       predicted by this layer.
    %       Alpha         - A scalar positive real value specifying the
    %                       balancing parameter.
    %       Gamma         - A scalar positive real value specifying the
    %                       focusing parameter.
    %       LossFunction  - The loss function used for training the network.
    %
    %   Example: Create a Focal Loss layer.
    %   ------------------------------------
    %   classes = ["Vehicle", "Background"];
    %   layer = focalLossLayer(2, 0.1, classes, 'Name','focalLossLayer')
    %
    % See also focalLossLayer, trainSSDObjectDetector.

    % Copyright 2019-2020 The MathWorks, Inc.

    %----------------------------------------------------------------------
    properties(Dependent)
        % Name   A name for the layer
        Name
    end
    
    %----------------------------------------------------------------------
    properties (SetAccess=private, Dependent)
        % Classes (categorical)  The categories into which the input data
        % is classified.
        %   A categorical column vector whose elements are the distinct
        %   classes to classify the input data to the network. It can be
        %   set passing a string or categorical vector, or a cell vector of
        %   character vectors, or 'auto'. When 'auto' is specified, the
        %   classes are automatically set during training. Default: 'auto'.
        Classes
        
        % Alpha (scalar positive real) is the balancing parameter.
        Alpha

        % Gamma (scalar positive real) is the focusing parameter.
        Gamma
    end
    
    %----------------------------------------------------------------------
    properties(Hidden, SetAccess = private)
        % LossFunction   The loss function for training
        %   The loss function that will be used during training. Possible
        %   values are:
        %     'focalLoss'
        LossFunction = 'focalLoss';
    end
    
    %----------------------------------------------------------------------
    properties(SetAccess = private, Dependent, Hidden)
        % ClassNames   The names of the classes
        %   A cell array containing the names of the classes.
        ClassNames
    end    
    
    %----------------------------------------------------------------------
    methods
        
        %------------------------------------------------------------------
        function val = get.Name(this)
            val = this.PrivateLayer.Name;
        end
        
        %------------------------------------------------------------------
        function this = set.Name(this, name)
            iAssertValidLayerName(name);
            this.PrivateLayer.Name = char(name);
        end
        
        %------------------------------------------------------------------
        function val = get.ClassNames(this)
             val = this.PrivateLayer.ClassNames(:);
        end
        
        %------------------------------------------------------------------
        function val = get.Classes(this)
            if isempty(this.PrivateLayer.Categories)
                val = 'auto';
            else
                val = this.PrivateLayer.Categories(:);
            end
        end
        
        %------------------------------------------------------------------
        function val = get.Alpha(this)
            val = this.PrivateLayer.Alpha;
        end
        
        %------------------------------------------------------------------
        function val = get.Gamma(this)
            val = this.PrivateLayer.Gamma;
        end
        
        %------------------------------------------------------------------
        function out = saveobj(this)
            
            out.Version       = 2.0;
            out.Name          = this.PrivateLayer.Name;
            out.Classes       = this.PrivateLayer.Categories;
            out.Alpha         = this.PrivateLayer.Alpha;
            out.Gamma         = this.PrivateLayer.Gamma;
        end
    end
    %----------------------------------------------------------------------
    methods(Static)
        %------------------------------------------------------------------
        function this = loadobj(in)
            
            if in.Version == 1
                % Update version number.
                in.Version = 2;
            end    
            this = iLoadFocalLossLayerFromCurrentVersion(in);
        end
    end
    %----------------------------------------------------------------------
    methods(Access = public)
        function this = FocalLossLayer(privateLayer)
            this.PrivateLayer = privateLayer;
        end
    end
    %----------------------------------------------------------------------
    methods(Access = protected)
        function [description, type] = getOneLineDisplay(~)
            description = getString(message('vision:ssd:focalLossDescription'));
            type        = getString(message('vision:ssd:focalLossType'));
        end
        %------------------------------------------------------------------
        function groups = getPropertyGroups( this )
            generalParameters = 'Name';
            
            groups = [
                this.propertyGroupGeneral( generalParameters )
                this.propertyGroupHyperparameters( {'Gamma','Alpha','Classes','LossFunction'})
                ];
        end 
    end
end

%--------------------------------------------------------------------------
function iAssertValidLayerName(name)
nnet.internal.cnn.layer.paramvalidation.validateLayerName(name);
end

%--------------------------------------------------------------------------
function layer = iLoadFocalLossLayerFromCurrentVersion(in)
internalLayer = nnet.internal.cnn.layer.FocalLossLayer(...
    in.Name, in.Classes, in.Alpha, in.Gamma);
layer = nnet.cnn.layer.FocalLossLayer(internalLayer);
end