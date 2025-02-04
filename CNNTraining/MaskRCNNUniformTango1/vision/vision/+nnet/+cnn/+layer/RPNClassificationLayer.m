%RPNClassificationLayer
%
%   Use rpnClassificationLayer to create this layer.
%
%   This layer is a 2-class classification layer for the region proposal
%   network (RPN) within a  Faster R-CNN object detection network. The RPN
%   classification layer classifies image regions as either "object" or
%   "background". This layer uses the cross entropy loss function.
%
%   <a href="matlab:helpview('vision','rcnnConcept')">Learn more about Faster R-CNN.</a>
%
%   RPNClassificationLayer properties:
%      Name        - The layer name.
%      NumInputs   - The number of inputs of the layer.
%      InputNames  - The names of the inputs of the layer.
%
%   Example - Create an RPN classification layer
%   --------------------------------------------
%   layer = rpnClassificationLayer('Name','rpnClassification')
%
%   See also rpnClassificationLayer, trainFasterRCNNObjectDetector

% Copyright 2016-2023 The MathWorks, Inc.
classdef RPNClassificationLayer < nnet.cnn.layer.Layer & nnet.internal.cnn.layer.Externalizable
    
    properties(Dependent)
        % Name   A name for the layer
        %   The name for the layer. If this is set to '', then a name will
        %   be automatically set at training time.
        Name
    end
    
    %----------------------------------------------------------------------
    methods
        function this = RPNClassificationLayer(privateLayer)
            this.PrivateLayer = privateLayer;
        end
        
        %------------------------------------------------------------------
        function val = get.Name(this)
            val = this.PrivateLayer.Name;
        end
        
        %------------------------------------------------------------------
        function this = set.Name(this, val)
            iAssertValidLayerName(val);
            this.PrivateLayer.Name = char(val);
        end
        
        %------------------------------------------------------------------
        function s = saveobj(this)
            s.Version    = 1.0;
            s.Name       = this.PrivateLayer.Name;
            s.NumClasses = this.PrivateLayer.NumClasses;
        end
        
    end
    
    %----------------------------------------------------------------------
    methods(Hidden,Static)
        function obj = loadobj(s)
            s = iUpgradeFromPreviousRelease(s);
            internalLayer = vision.internal.cnn.layer.RPNCrossEntropy(...
                s.Name,s.NumClasses);
            
            obj = nnet.cnn.layer.RPNClassificationLayer(internalLayer);
        end
    end
    
    %----------------------------------------------------------------------
    methods(Access = protected)
        function [description, type] = getOneLineDisplay(~)
            
            description = getString(message('vision:rcnn:rpnClsDesc'));
            
            type = getString(message('vision:rcnn:rpnClsType'));
        end
        
        function groups = getPropertyGroups( this )
            groups = this.propertyGroupGeneral( {'Name'} );
        end
    end
end

%--------------------------------------------------------------------------
function out = iUpgradeFromPreviousRelease(s)
if ~isstruct(s)
    % save/load serializes to struct as of 18b.
    out.Version = 1.0;
    out.NumClasses = s.PrivateLayer.NumClasses;
    out.Name = s.Name;
else
    out = s;
end
end

%--------------------------------------------------------------------------
function iAssertValidLayerName(name)
iEvalAndThrow(@()...
    nnet.internal.cnn.layer.paramvalidation.validateLayerName(name));
end

function iEvalAndThrow(func)
% Omit the stack containing internal functions by throwing as caller
try
    func();
catch exception
    throwAsCaller(exception)
end
end