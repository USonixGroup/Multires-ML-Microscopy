%RegionProposalNetwork Object for storing a region proposal network (RPN).
% 
% This class is not recommended. Use DAGNetwork instead.
% 
% A RegionProposalNetwork object stores the layers that define a Region
% Proposal Network (RPN). A RPN is a convolutional neural network with a
% classification and regression output. The connectivity of a RPN is as
% follows:
%
% Region Proposal Network Connectivity
% ------------------------------------
%
%   [Layers(1) ... Layers(LayerIndex) ... Layers(end)]
%                         \                        
%                          \
%                           [RegressionLayers(1) ... RegressionLayers(end)]
%
%
% RegionProposalNetwork properties:
%   Layers           - Convolutional neural network layers.
%   RegressionLayers - Layers used for regression.
%   LayerIndex       - Index that specifies which element of Layers
%                      connects to the beginning of the RegressionLayers.
%  
% See also fastRCNNObjectDetector, fasterRCNNObjectDetector, 
%          trainFastRCNNObjectDetector, trainFasterRCNNObjectDetector,
%          SeriesNetwork.

% Copyright 2016-2018 The MathWorks, Inc.
classdef RegionProposalNetwork
    
    properties(SetAccess = private)
        %Layers Network layers that make up convolutional neural network
        %       used within the region proposal network (RPN).
        Layers               
    end
        
    properties(GetAccess = public, SetAccess = protected)
       
        %RegressionLayers Network layers that make up the regression branch
        %                 within the region proposal network.
        RegressionLayers
        
        %LayerIndex Index that specifies which Layer is connected
        %           to the regression layers of the region proposal network.
        LayerIndex
    end
    
    properties(Hidden, SetAccess = private)      
        RegLayers
    end
    
    properties(Hidden)        
        BranchLayerIdx
    end
    
    methods(Access = public, Hidden)
        function this = RegionProposalNetwork(layers, regLayers, branchLayerIdx)                  
          this.Layers = layers;
          this.RegLayers = regLayers;
          this.RegressionLayers = regLayers;
          this.BranchLayerIdx = branchLayerIdx;
          this.LayerIndex = branchLayerIdx;
        end
    end
    
    methods(Hidden, Access = public)        
        
        function out = saveobj(this)            
            out.Version = 1.0;
            out.Layers = this.Layers; % User visible layers
            out.RegLayers = this.RegLayers;
            out.BranchLayerIdx = this.BranchLayerIdx;            
        end
    end
    
    methods(Static)
        function this = loadobj(in)
            if in.Version == 1.0
                % replace reshape->softmax layers with RPNSoftmaxLayer
                idx = find(arrayfun(@(x)isa(x,'vision.cnn.layer.RPNReshape'),in.Layers));                
                if ~isempty(idx) && isa(in.Layers(idx+1),'nnet.cnn.layer.SoftmaxLayer')
                    in.Layers(idx) = []; % remove reshape layer
                    in.Layers(idx) = rpnSoftmaxLayer('Name','rpn_softmax');                
                else
                    % Unexpected outcome.
                    assert(false,'Unable to find reshape followed by softmax layer.');
                end
            end
            this = vision.cnn.RegionProposalNetwork( in.Layers, in.RegLayers, in.BranchLayerIdx );
        end
    end
end