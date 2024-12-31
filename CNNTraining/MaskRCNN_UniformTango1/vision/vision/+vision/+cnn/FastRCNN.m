%FastRCNN Object for storing a Fast R-CNN network.
%
% This class is not recommended. Use DAGNetwork instead.
%
% A FastRCNN object stores the layers that define a convolutional neural
% network with a classification and regression output. The connectivity of
% the Fast R-CNN network is as follows:
%
% Fast R-CNN Network Connectivity
% -------------------------------
%
%   [Layers(1) ... Layers(LayerIndex) ... Layers(end)]
%                         \
%                          \
%                           [RegressionLayers(1) ... RegressionLayers(end)]
%
%
% FastRCNN properties:
%   Layers           - Convolutional neural network layers.
%   RegressionLayers - Layers used for regression.
%   LayerIndex       - Index that specifies which element of Layers
%                      connects to the beginning of the RegressionLayers.
%
% See also fastRCNNObjectDetector, fasterRCNNObjectDetector,
%          trainFastRCNNObjectDetector, trainFasterRCNNObjectDetector,
%          SeriesNetwork.

% Copyright 2016-2018 The MathWorks, Inc.
classdef FastRCNN
    
    properties(GetAccess = public, SetAccess = protected)
        %Layers Network layers that make up convolutional neural network
        %       used within the Fast R-CNN network.
        Layers
    end
    
    properties(GetAccess = public, SetAccess = protected)
        %RegressionLayers Network layers that make up the regression branch
        %                 within the Fast R-CNN network.
        RegressionLayers
        
        %LayerIndex Index that specifies which Layer is connected
        %           to the regression layers of the Fast R-CNN Network.
        LayerIndex
        
        BranchLayerIdx;
    end
    
    properties(Hidden, SetAccess = private)
        RegLayers
    end
    
    methods(Access = public, Hidden)
        function this = FastRCNN(layers, regLayers, branchLayerIdx)
            
            this.Layers = layers;
            this.RegLayers = regLayers;
            this.RegressionLayers = regLayers;
            this.LayerIndex = branchLayerIdx;
            this.BranchLayerIdx = branchLayerIdx;
        end
        
        function lgraph = toLayerGraph(this)                        
            
            N1 = numel(this.Layers);            
            allLayers = vertcat(this.Layers,this.RegressionLayers);
            allLayers = deep.internal.sdk.layer.assignUniqueLayerNames(allLayers);
            in.Layers = allLayers(1:N1);
            in.RegLayers = allLayers(N1+1:end);
                      
            lgraph = vision.cnn.FastRCNN.fastRCNNLayerGraph(in, this.LayerIndex, true);
        end                    
        
    end
    
    methods(Hidden, Access = public)
       
        function out = saveobj(this)
            out.Layers         = this.Layers;
            out.RegLayers      = this.RegLayers;
            out.BranchLayerIdx = this.BranchLayerIdx;
            out.Version        = 1.0;
        end
    end
    
    methods(Static)
        function this = loadobj(in)
            this = vision.cnn.FastRCNN( in.Layers, in.RegLayers, in.BranchLayerIdx );
        end        
    end
    
    methods(Hidden, Static)
        function lgraph = fastRCNNLayerGraph(in, layerIndex, includeROIInputLayer)
            
            lgraph = layerGraph(in.Layers);
            
            lgraph = lgraph.addLayers(in.RegLayers);
            src = in.Layers(layerIndex).Name;
            dst = in.RegLayers(1).Name;
            lgraph = lgraph.connectLayers(src,dst);
            
            if includeROIInputLayer
                % Create ROI input layer
                roiInput = nnet.cnn.layer.ROIInputLayer(...
                    nnet.internal.cnn.layer.ROIInputLayer('ROI Input'));
                
                lgraph = lgraph.addLayers(roiInput);
                
                idx = arrayfun(@(x)isa(x,'vision.cnn.layer.ROIAveragePooling2DLayer') || ...
                    isa(x,'nnet.cnn.layer.ROIMaxPooling2DLayer'),...
                    in.Layers);
                idx = find(idx,1);
                
                % roi input -> roi pooling's roi input
                lgraph = lgraph.connectLayers('ROI Input', [in.Layers(idx).Name '/roi']);
            end
        end
    end
end
