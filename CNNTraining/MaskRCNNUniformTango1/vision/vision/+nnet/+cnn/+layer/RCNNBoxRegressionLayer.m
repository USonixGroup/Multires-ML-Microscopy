%RCNNBoxRegressionLayer Box regression layer for Fast and Faster R-CNN.
%
%   Use rcnnBoxRegressionLayer to create this layer. This layer is used in
%   Fast R-CNN and Faster R-CNN object detection networks.
%
%   <a href="matlab:helpview('vision','rcnnConcept')">Learn more about Fast R-CNN and Faster R-CNN.</a>
%
%   RCNNBoxRegressionLayer properties:
%      Name - The layer name.  
%  
%   Example - Create an R-CNN box regression layer
%   ----------------------------------------------
%   boxReg = rcnnBoxRegressionLayer('Name','boxReg')
%
%   See also rcnnBoxRegressionLayer, trainFastRCNNObjectDetector,
%            trainFasterRCNNObjectDetector.

% Copyright 2018-2023 The MathWorks, Inc.
classdef RCNNBoxRegressionLayer < nnet.cnn.layer.Layer & nnet.internal.cnn.layer.Externalizable

    properties(Dependent)
        % Name   A name for the layer
        %   The name for the layer. If this is set to '', then a name will
        %   be automatically set at training time.
        Name       
    end
    
    %----------------------------------------------------------------------
    methods
        function this = RCNNBoxRegressionLayer(privateLayer)          
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
            privateLayer   = this.PrivateLayer;
            s.Version      = 1.0;
            s.Name         = privateLayer.Name;
            s.NumResponses = privateLayer.NumResponses;
        end
    end
    
    %----------------------------------------------------------------------
    methods(Hidden, Access = protected)
        function [desc, type] = getOneLineDisplay(~)
            desc = getString(message('vision:rcnn:rcnnBoxDesc'));
            type = getString(message('vision:rcnn:rcnnBoxType'));
        end
        
        function groups = getPropertyGroups( this )
            generalParameters = {
                'Name' 
                };     
            groups = this.propertyGroupGeneral( generalParameters );
        end
    end
    
    %----------------------------------------------------------------------
    methods(Hidden,Static)
        function obj = loadobj(s)
            s = iUpgradeFromPreviousRelease(s);
            internalLayer = vision.internal.cnn.layer.SmoothL1Loss(...
                s.Name,s.NumResponses);
           
            obj = nnet.cnn.layer.RCNNBoxRegressionLayer(internalLayer);
        end
    end
        
end

%--------------------------------------------------------------------------
function out = iUpgradeFromPreviousRelease(s)
if ~isstruct(s)
    % save/load serializes to struct as of 18b.
    out.Version = 1.0;
    out.NumResponses = s.PrivateLayer.NumResponses;
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

%--------------------------------------------------------------------------
function iEvalAndThrow(func)
% Omit the stack containing internal functions by throwing as caller
try
    func();
catch exception
    throwAsCaller(exception)
end
end
