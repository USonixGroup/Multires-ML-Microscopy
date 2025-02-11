%   YOLOv2ReorgLayer YOLO v2 Reorg layer.
% 
%   Use yolov2ReorgLayer to create this layer. Use this layer to create
%   YOLO v2 object detection network.
%
%   The YOLO v2 Reorg layer outputs fixed size feature maps for every
%   stride with input feature map. Given an input feature map of size 
%   [H W C], where C is the number of channels, the output feature map 
%   size is [height width channels], where height is [floor(H/stride(1))], 
%   width is [floor(W/stride(2))] and channels dimension is 
%   [C*stride(1)*stride(2)]. This layer is used in YOLOv2 object detection 
%   network.
%
%   <a href="matlab:helpview('vision','yolov2Concept')">Learn more about YOLO v2.</a>
%   
%   YOLOv2ReorgLayer properties (read-only):
%      Name       - The layer name.
%      stride     - The stride to reorder input activation.
%   
%   Example - Create a YOLO v2 ReorgLayer.
%   --------------------------------------
%   % Stride for reshaping input activations.
%   stride = [2 2];
% 
%   % Create YOLO v2 Reorg layer.
%   yolov2Reorg = yolov2ReorgLayer(stride,'Name','reOrg')
%  
%   Example - Create a YOLO v2 network.
%   -----------------------------------
%   % <a href="matlab:helpview('vision','createYolov2Network')">Create a YOLO v2 network.</a>
%
%   See also yolov2ReorgLayer, yolov2Layers, yolov2OutputLayer, 
%            trainYOLOv2ObjectDetector.

% Copyright 2018-2023 The MathWorks, Inc.

classdef YOLOv2ReorgLayer < nnet.cnn.layer.Layer & nnet.internal.cnn.layer.Externalizable      
    
    properties(Dependent)
        % Name A name for the layer.
        %   The name for the layer. If this is set to '', then a name will
        %   be automatically set at training time.
        Name
    end
    
    properties(SetAccess = private, Dependent)
        %   Stride       The stride is used to reorder input size, 
        %   [height width channels] to [outHeight, outWidth, outChannels]
        %   where outHeight is [floor(height/stride(1))], outWidth is
        %   [floor(width/stride(2))] and outChannels dimension is
        %   [channels*stride(1)*stride(2)].        
        Stride
    end
    
    methods
        function this = YOLOv2ReorgLayer(privateLayer)
            this.PrivateLayer = privateLayer;
        end
        
        function sz = get.Stride(this)
            sz = this.PrivateLayer.BlockSize;
        end
        
        function name = get.Name(this)
            name = this.PrivateLayer.Name;
        end
        
        function this = set.Name(this, val)
            iAssertValidLayerName(val);
            this.PrivateLayer.Name = char(val);
        end
        
        
        function s = saveobj(this)
            privateLayer   = this.PrivateLayer;
            s.Version      = 1.0;
            s.Name         = privateLayer.Name;
            s.Stride       = privateLayer.BlockSize;  

        end
    end
    
    methods(Access = protected)
        function [description, type] = getOneLineDisplay(this)
            stringSize = mat2str(this.Stride);
            description = getString(message('vision:yolo:yoloReorgOneLineDisp',stringSize));
            type = getString(message('vision:yolo:yoloReorgType'));
        end
        
        %------------------------------------------------------------------
        function groups = getPropertyGroups( this )
            hyperparameters = {
                'Stride'
                };
            
            groups = [
                this.propertyGroupGeneral( {'Name'} )
                this.propertyGroupHyperparameters( hyperparameters )
                ];
        end
    end
    
    methods(Hidden, Static)
        function validateStride(Stride,name)
            
            validateattributes(Stride, {'numeric'}, ...
                {'row', 'numel', 2, 'finite','real','positive','integer', 'nonsparse'}, ...
                name);
            
        end
        
        %------------------------------------------------------------------
        function obj = loadobj(s)
            warning(message('vision:yolo:yoloReorgToBeRemoved'));
            internalLayer = nnet.internal.cnn.layer.YOLOv2ReorgLayer(...
                s.Name, s.Stride);           
            
            obj = nnet.cnn.layer.YOLOv2ReorgLayer(internalLayer);
        end
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
