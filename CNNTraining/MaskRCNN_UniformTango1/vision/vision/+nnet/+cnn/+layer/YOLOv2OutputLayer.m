%YOLOv2OutputLayer YOLO v2 Output Layer
%
%   Use yolov2OutputLayer to create this layer. Use this layer to create
%   YOLO v2 object detection network.
%
%   This layer is used to refine bounding box locations of anchor Boxes,
%   update IOU, and calculate class prob. It uses custom weighted L2
%   error function to computing the loss.
%
%   <a href="matlab:helpview('vision','yolov2Concept')">Learn more about YOLO v2.</a>
%
%   YOLOv2OutputLayer properties (read-only):
%      Name        -  The layer name.
%      AnchorBoxes -  An M-by-2 matrix defining the [width height] of
%                     M anchor boxes.
%      LossFactors -  A 1-by-4 vector, K, representing the loss
%                     factors for each component of the YOLO v2 loss
%                     function. K(1) is the objectness factor, K(2)
%                     is the non-objectness factor, K(3) is the box
%                     coordinate factor, and K(4) is class factor.
%                     Increase the loss factor to give more weight to
%                     anyone of the loss components.
%      Classes        Specify the names of object classes that the YOLO v2
%                     object detector is trained to find as a string
%                     vector, a categorical vector, a cell array of
%                     character vectors, or 'auto'. If the value is 'auto',
%                     the classes are automatically set during training.
%
%   Example - Create a YOLO v2 output layer.
%   ---------------------------------------
%   anchors = [32 32;64 64];
%   layer = yolov2OutputLayer(anchors)
%
%   Example - Create a YOLO v2 network.
%   -----------------------------------
%   % <a href="matlab:helpview('vision','createYolov2Network')">Create a YOLO v2 network.</a>
%
%   See also yolov2OutputLayer, spaceToDepthLayer, yolov2Layers,
%            yolov2ObjectDetector, trainYOLOv2ObjectDetector.

%   Copyright 2018-2023 The MathWorks, Inc.

classdef YOLOv2OutputLayer < nnet.cnn.layer.Layer & nnet.internal.cnn.layer.Externalizable
    
    properties(Dependent)
        % Name   A name for the layer
        %   The name for the layer. If this is set to '', then a name will
        %   be automatically set at training time.
        Name
        
        % Classes (categorical)  The categories on which the network is
        % trained.
        %  A categorical column vector of object classes that the YOLO v2
        %  object detector is trained to find. Specify classes as a string
        %  vector, a categorical vector, a cell array of character vectors,
        %  or 'auto'. If the value is 'auto', the classes are automatically
        %  set during training.
        Classes
    end  
    
    properties(SetAccess = private, Dependent)
        % NumClasses   The size of the clases
        %   The number of the classes. This will be determined at training
        %   time. Prior to training, it is set to 'auto'.
        NumClasses
    end
    
    properties(SetAccess = private)
        % LossFunction   The loss function for training
        %   The loss function that will be used during training.
        LossFunction = 'mean-squared-error';
    end
    
    properties(SetAccess = private, Dependent)
        % AnchorBoxes - An M-by-2 matrix defining the [width height] of M
        %               anchor boxes.
        AnchorBoxes
        
        % LossFactors - Scale factors for each component of YOLO v2 loss
        %               function.
        LossFactors
    end
    
    
    methods
        function this = YOLOv2OutputLayer(privateLayer)
            this.PrivateLayer = privateLayer;
        end
        
        function out = saveobj(this)
            privateLayer = this.PrivateLayer;
            out.Version = 2.0;
            out.Name = privateLayer.Name;
            out.AnchorBoxes = privateLayer.AnchorBoxes;
            out.LossFactors = privateLayer.LossFactors;
            out.NumClasses = privateLayer.NumClasses;
            out.Categories = privateLayer.Categories;
        end
        
        function val = get.NumClasses(this)
            if(isempty(this.PrivateLayer.NumClasses))
                val = 'auto';
            else
                val = this.PrivateLayer.NumClasses;
            end
        end
        
        function val = get.Classes(this)
            if isempty(this.PrivateLayer.Categories)
                val = 'auto';
            else
                val = this.PrivateLayer.Categories(:);
            end
        end
        
        function this = set.Classes(this, val)
            iAssertValidClasses(val);
            if iIsAuto(val) && ischar(val)
                this = this.resetClasses();
            else
                classes = iConvertClassesToCanonicalForm(val);
                this = this.updateClasses(classes);
            end
        end
        
        function val = get.AnchorBoxes(this)
            val = this.PrivateLayer.AnchorBoxes;
        end
        
        function this = set.AnchorBoxes(this, val)
            this.PrivateLayer.AnchorBoxes = val;
        end
        
        function val = get.LossFactors(this)
            val = this.PrivateLayer.LossFactors;
        end
        
        function this = set.LossFactors(this, val)
            this.PrivateLayer.LossFactors = val;
        end
        
        function val = get.Name(this)
            val = this.PrivateLayer.Name;
        end
        
        function this = set.Name(this, val)
            iAssertValidLayerName(val);
            this.PrivateLayer.Name = char(val);
        end
    end
    
    methods(Hidden, Static)
        function this = loadobj(in)
            if in.Version == 1
                in = iUpgradeVersionOneToVersionTwo(in);
            end
            internalLayer = iConstructInternalLayerWithClasses( ...
                in.Name,in.AnchorBoxes,in.LossFactors,in.NumClasses,...
                in.Categories);
            this = nnet.cnn.layer.YOLOv2OutputLayer(internalLayer);
        end
    end
    
    methods(Hidden, Access = protected)
        function [description, type] = getOneLineDisplay(this)
            stringSize = mat2str(size(this.AnchorBoxes,1));
            description = iGetMessageString('vision:yolo:yolov2OutputOneLineDisp', ....
                stringSize);
            
            type = iGetMessageString('vision:yolo:yolov2OutputType');
        end
        
        function groups = getPropertyGroups( this )
            generalParameters = 'Name';
            
            groups = [
                this.propertyGroupGeneral( generalParameters )
                this.propertyGroupHyperparameters( {'Classes','LossFunction', 'AnchorBoxes', 'LossFactors'})
                ];
        end
    end
    
    methods(Access = private)
        function this = resetClasses(this)
            name = this.PrivateLayer.Name;
            anchorBoxes = this.AnchorBoxes;
            lossFactors = this.LossFactors;            
            this.PrivateLayer = ...
                    iConstructInternalLayerWithClasses(...
                    name, anchorBoxes, lossFactors,[], categorical());
        end        
        
        function this = updateClasses(this, classes)
            numClasses = numel(classes);
            name = this.PrivateLayer.Name;
            anchorBoxes = this.AnchorBoxes;
            lossFactors = this.LossFactors;
            this.PrivateLayer = ...
                    iConstructInternalLayerWithClasses(...
                    name, anchorBoxes, lossFactors, numClasses, classes);
        end
    end
    
    methods(Hidden, Static)
        function validateAnchors(anchorBoxes, name)
            validateattributes(anchorBoxes, {'numeric'}, ...
                {'2d','ncols',2,'ndims',2,'nonempty','nonsparse',...
                'real','finite','nonnan','positive'},name);
        end
        
        function validateLossFactors(LossFactors,name)
            validateattributes(LossFactors,{'numeric'},...
                {'2d','nrows',1,'ncols',4,'ndims',2,'nonempty','nonsparse',...
                'real','finite','nonnan','positive'},name);
        end
    end
    
end

function messageString = iGetMessageString( varargin )
messageString = getString( message( varargin{:} ) );
end

function iAssertValidClasses(value)
iEvalAndThrow(@()...
    nnet.internal.cnn.layer.paramvalidation.validateClasses(value));
end

function tf = iIsAuto(val)
tf = isequal(string(val), "auto");
end

function S = iUpgradeVersionOneToVersionTwo(S)
% iUpgradeVersionOneToVersionTwo   Upgrade a v1 saved struct to a v2 saved struct
%   This means initializing the numClasses and categories.
S.Version = 2;
S.NumClasses = [];
S.Categories = categorical();
end

function classes = iConvertClassesToCanonicalForm(classes)
classes = ...
    nnet.internal.cnn.layer.paramvalidation.convertClassesToCanonicalForm(classes);
end

function internalLayer = ...
        iConstructInternalLayerWithClasses(name, anchorBoxes, lossFactors, numClasses, classes)
internalLayer = ...
    nnet.internal.cnn.layer.YOLOv2OutputLayer.constructWithClasses(...
    name, anchorBoxes, lossFactors, numClasses, classes);
end

function iAssertValidLayerName(name)
iEvalAndThrow(@()...
    nnet.internal.cnn.layer.paramvalidation.validateLayerName(name));
end

function iEvalAndThrow(func)
% Omit the stack containing internal functions by throwing as caller.
try
    func();
catch exception
    throwAsCaller(exception)
end
end
