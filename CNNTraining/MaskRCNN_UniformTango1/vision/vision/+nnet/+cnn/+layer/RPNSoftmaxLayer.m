%RPNSoftmaxLayer
%
%   Use rpnSoftmaxLayer to create this layer. This layer is used in Faster
%   R-CNN object detection networks.
%
%   <a href="matlab:helpview('vision','rcnnConcept')">Learn more about Faster R-CNN.</a>
%
%   RPNSoftmaxLayer properties:
%      Name         - The layer name.
%      NumInputs    - The number of inputs of the layer.
%      InputNames   - The names of the inputs of the layer.
%      NumOutputs   - The number of outputs of the layer.
%      OutputNames  - The names of the outputs of the layer.
%
%   Example - Create an RPN softmax layer
%   -------------------------------------
%   layer = rpnSoftmaxLayer('Name','rpnSoftmax')
%
%   See also rpnSoftmaxLayer, trainFasterRCNNObjectDetector.

% Copyright 2018-2023 The MathWorks, Inc.
classdef RPNSoftmaxLayer < nnet.cnn.layer.Layer & nnet.internal.cnn.layer.Externalizable
    properties(Dependent)
        % Name   A name for the layer
        %   The name for the layer. If this is set to '', then a name will
        %   be automatically set at training time.
        Name
    end
    
    %----------------------------------------------------------------------
    methods
        function this = RPNSoftmaxLayer(privateLayer)
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
        end
        
    end
    
    %----------------------------------------------------------------------
    methods(Hidden,Static)
        function obj = loadobj(s)
            internalLayer = nnet.internal.cnn.layer.RPNSoftmaxLayer(...
                s.Name);
            
            obj = nnet.cnn.layer.RPNSoftmaxLayer(internalLayer);
        end
    end
    
    %----------------------------------------------------------------------
    methods(Access = protected)
        function [description, type] = getOneLineDisplay(~)
            
            description = getString(message('vision:rcnn:rpnSoftmaxDesc'));
            
            type = getString(message('vision:rcnn:rpnSoftmaxType'));
        end
        
        function groups = getPropertyGroups( this )
            groups = this.propertyGroupGeneral( 'Name' );
        end
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