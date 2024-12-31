%RegionProposalLayer Region proposal layer.
%
%   Use regionProposalLayer to create this layer. 
%
%   The region proposal layer is part of the region proposal network (RPN)
%   within Faster R-CNN. This layer outputs bounding boxes around potential
%   objects in an image. These outputs are further refined by additional
%   layers within Faster R-CNN to produce the final object detection
%   results.
%
%   <a href="matlab:helpview('vision','rcnnConcept')">Learn more about Faster R-CNN.</a>
%
%   RegionProposalLayer properties:
%      Name         - The layer name.
%      AnchorBoxes  - An M-by-2 matrix defining the [height width] of M
%                     anchor boxes.
%      NumInputs    - The number of inputs for the layer.
%      InputNames   - The names of the inputs of the layer.
%      NumOutputs   - The number of outputs of the layer.
%      OutputNames  - The names of the outputs of the layer.
%
%   Example - Create a region proposal layer
%   ----------------------------------------
%   % Define anchor boxes used for region proposal. 
%   anchorBoxes = [16 16; 32 32];
%
%   layer = regionProposalLayer(anchorBoxes, 'Name','regionProposals')
%
%   See also regionProposalLayer, trainFasterRCNNObjectDetector.

% Copyright 2018-2023 The MathWorks, Inc.
classdef RegionProposalLayer < nnet.cnn.layer.Layer & nnet.internal.cnn.layer.Externalizable
    
    properties(Dependent)
        % Name   A name for the layer
        %   The name for the layer. If this is set to '', then a name will
        %   be automatically set at training time.
        Name                      
    end
    
    properties(SetAccess = private, Dependent)
        % AnchorBoxes An M-by-2 matrix defining the [height width] of M
        %             anchor boxes.
        AnchorBoxes
    end
    
    %----------------------------------------------------------------------
    % These properties are accessed/updated during inference.
    %----------------------------------------------------------------------
    properties(SetAccess = private, Hidden, Dependent)
       % MinSize The minimum proposal box size. Boxes smaller than MinSize
        % are discarded.
        MinSize
        
        % MaxSize The maximum proposal box size. Boxes larger than MaxSize
        % are discarded.
        MaxSize
        
        % ScaleFactor ([sx sy]) Used to scale boxes from feature map to
        %             image.
        ScaleFactor
        
        % ImageSize The size of the image for which boxes are proposed.
        ImageSize
        
        % NumStrongestRegions The number of strongest proposal boxes to
        %                     return.
        NumStrongestRegions
    end
    
    %----------------------------------------------------------------------
    methods
        function this = RegionProposalLayer(privateLayer)          
            this.PrivateLayer = privateLayer;
        end       
        
        %------------------------------------------------------------------
        function val = get.Name(this)
            val = this.PrivateLayer.Name;
        end
        
        %------------------------------------------------------------------
        function this = set.Name(this, val)       
            iEvalAndThrow(@()...
            nnet.internal.cnn.layer.paramvalidation.validateLayerName(val));
            this.PrivateLayer.Name = char(val);
        end  
        
        %------------------------------------------------------------------
        function val = get.AnchorBoxes(this)
            % Round anchor boxes in case they are fractional. Fractional
            % anchor boxes are used when loading models trained in previous
            % releases (<= 18a).
            val = round(this.PrivateLayer.AnchorBoxes);
        end
        
        %------------------------------------------------------------------
        function s = saveobj(this)
            privateLayer   = this.PrivateLayer;
            s.Version      = 1.0;
            s.Name         = privateLayer.Name;    
            s.AnchorBoxes  = privateLayer.AnchorBoxes;                      
        end
        
        %------------------------------------------------------------------
        function val = get.MinSize(this)
            val = this.PrivateLayer.MinSize;
        end
        
        %------------------------------------------------------------------
        function val = get.MaxSize(this)
            val = this.PrivateLayer.MaxSize;
        end
        
        %------------------------------------------------------------------
        function val = get.ScaleFactor(this)
            val = this.PrivateLayer.ScaleFactor;
        end
        
        %------------------------------------------------------------------
        function val = get.ImageSize(this)
            val = this.PrivateLayer.ImageSize;            
        end
        
        %------------------------------------------------------------------
        function val = get.NumStrongestRegions(this)
            val = this.PrivateLayer.NumStrongestRegions;
        end
        
    end
    
    methods(Hidden)
        function val = getAnchorBoxes(this)
            % Return (fractional) anchors from private layer. This is
            % required during inference for detectors trained in releases
            % prior to 18b.
            val = this.PrivateLayer.AnchorBoxes;
        end
    end
  
    %----------------------------------------------------------------------
    methods(Hidden, Access = protected)
        function [desc, type] = getOneLineDisplay(this)   
            n = size(this.AnchorBoxes,1);
            if n > 1
                desc = getString(message('vision:rcnn:proposalDesc',n));
            else
                desc =  getString(message('vision:rcnn:proposalDescSingle'));
            end
            type = getString(message('vision:rcnn:proposalType'));
        end
        
        function groups = getPropertyGroups( this )
            generalParameters = {
                'Name' 
                'AnchorBoxes'
                'NumInputs'
                'InputNames'
                };   
            groups = this.propertyGroupGeneral( generalParameters );
        end
    end
    
    %----------------------------------------------------------------------
    methods(Hidden,Static)
        function obj = loadobj(s)            
            internalLayer = nnet.internal.cnn.layer.RegionProposalLayer(...
                s.Name,s.AnchorBoxes);
           
            obj = nnet.cnn.layer.RegionProposalLayer(internalLayer);
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