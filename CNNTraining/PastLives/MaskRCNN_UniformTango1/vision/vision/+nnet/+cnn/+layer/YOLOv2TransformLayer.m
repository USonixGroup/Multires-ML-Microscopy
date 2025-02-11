%YOLOv2TransformLayer YOLO v2 transform layer.
%
%   Use yolov2TransformLayer to create this layer. Use this layer to create
%   YOLO v2 object detection network.
%
%   This layer is used to extract bounding box predictions, iou
%   predictions, probability predictions from activations of last
%   convolutional layer.
%
%   <a href="matlab:helpview('vision','yolov2Concept')">Learn more about YOLO v2.</a>
%
%   YOLOv2TransformLayer properties (read-only):
%      Name             -  The layer name.
%      numAnchorBoxes   -  Number of anchor boxes.
%
%   Example - Create a YOLO v2 transform layer.
%   ---------------------------------------
%   numAnchorBoxes = 5;
%   layer = yolov2TransformLayer(numAnchorBoxes)
%
%   Example - Create a YOLO v2 network.
%   -----------------------------------
%   % <a href="matlab:helpview('vision','createYolov2Network')">Create a YOLO v2 network.</a>
%
%   See also yolov2TransformLayer, yolov2Layers, yolov2ObjectDetector, 
%            trainYOLOv2ObjectDetector.

%   Copyright 2018-2024 The MathWorks, Inc.

classdef YOLOv2TransformLayer < nnet.layer.Layer & nnet.internal.cnn.layer.Traceable & nnet.layer.Formattable & nnet.layer.Acceleratable    
    
    properties(SetAccess = private)
        % NumAnchorBoxes are the number of anchor boxes per grid according
        % to which input feature map is modified.
        NumAnchorBoxes
    end
    
    methods
        function layer = YOLOv2TransformLayer(name,numAnchors)
            iAssertValidLayerName(name)
            layer.Name = char(name);
            layer.NumAnchorBoxes = numAnchors;
            layer.Description = getString(message('vision:yolo:yolov2TransfromDispAnchors',numAnchors));
            layer.Type = getString(message('vision:yolo:yolov2TransfromType'));
        end

        function Z = predict(layer,X)
            inSize = size(X);
            numAnchorBoxes = layer.NumAnchorBoxes;
            
            % Input X is reshaped to get the predictions along channels.
            reshapedAct = reshape(X,inSize(1)*inSize(2),numAnchorBoxes,inSize(3)/numAnchorBoxes,[]);
            
            % Squash the IoU, x, y cooridnaes to range from 0 to 1.
            iouPred = 1./(1+exp(-1*(reshapedAct(:,:,1,:))));
            xyPred = reshapedAct(:, :,2:3,:);
            sigmaXY = 1./(1+exp(-1*xyPred));
            
            % Get w, h predictions from feature map.
            whPred = reshapedAct(:,:,4:5,:);
            expWH = exp(whPred);
            
            % Compute probabilities of class predictions.
            scorePred = reshapedAct(:,:,6:end,:);
            probPred = nnet.internal.cnnhost.softmaxForward(scorePred, 3);
            
            % Reshaping back to original dimension of X.
            Z = cat(3,iouPred,sigmaXY,expWH,probPred);
            Z = reshape(Z,inSize);
            if isa(X, 'dlarray')
                labels = dims(X);
                isInputDataFormatted = ~isempty(dims(X));
                if isInputDataFormatted
                    Z = dlarray(Z,labels);
                end
            end  
            
        end
                
        function s = saveobj(this)
            s.Name         = this.Name;
            s.NumAnchorBoxes  = this.NumAnchorBoxes;    
        end
    end
    
   
    methods(Hidden, Static)
        function validateParameters(sz, name, varname)
            validateattributes(sz, {'numeric'}, ...
                {'scalar','real', 'finite', 'positive', 'integer','nonsparse'}, ...
                name, varname);
        end  
        %------------------------------------------------------------------
        function obj = loadobj(s)
            obj = nnet.cnn.layer.YOLOv2TransformLayer(s.Name,s.NumAnchorBoxes);           
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
% Omit the stack containing internal functions by throwing as caller.
try
    func();
catch exception
    throwAsCaller(exception)
end
end
