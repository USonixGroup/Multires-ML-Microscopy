classdef FrozenBatchNormalizationLayer < nnet.cnn.layer.Layer & nnet.internal.cnn.layer.Externalizable
    % FrozenBatchNormalizationLayer 
    
    % This layer is used as a replacement for a batch normalization layer
    % when we wish to freeze a batch normalization layer. Freezing the BN
    % layer is best practice when the mini-batch size is low. For small
    % batch sizes, the mean and variance estimate are not reliable.
    
    % Copyright 2017-2019 The MathWorks, Inc.

    properties(Dependent)
        Name        
    end
    
    properties(Dependent,SetAccess = private)
        BatchNormalizationLayer
    end
    
    methods
        function this = FrozenBatchNormalizationLayer(privateLayer)                                     
           this.PrivateLayer = privateLayer;
        end

        function s = saveobj(this)
            s.Version = 1;
            s.BatchNormalizationLayer = this.PrivateLayer.BatchNormalizationLayer;
        end
        
        function name = get.Name(this)
            name = this.PrivateLayer.Name;
        end
        
        function this = set.Name(this, val)
            
            this.PrivateLayer.Name = char(val);
        end
        
        function layer = get.BatchNormalizationLayer(this)
            layer = this.PrivateLayer.BatchNormalizationLayer;
        end

    end
    
    methods(Access = protected)
        function [description, type] = getOneLineDisplay(this)
           
            description = 'frozen batch norm';
            
            type = 'Frozen Batch Norm';
        end
        
    end
    
    methods(Static)
        function layer = loadobj(s)
            layer = vision.internal.cnn.FrozenBatchNormalizationLayer.create(s.BatchNormalizationLayer);
        end
        
        function layer = create(bnLayer)
            privateLayer = vision.internal.cnn.layer.FrozenBatchNormalization(bnLayer);            
            layer = vision.internal.cnn.FrozenBatchNormalizationLayer(privateLayer);
            layer.Name = bnLayer.Name;
        end
    end
    
end