%ROIMaxPooling2DLayer ROI max pooling layer.
%
%   Use roiMaxPooling2dLayer to create this layer.
%
%   The ROI max pooling layer outputs fixed size feature maps for every
%   rectangular ROI within the input feature map. Given an input feature
%   map of size [H W C N], where C is the number of channels and N is the
%   number of observations, the output feature map size is [height width C
%   sum(M)], where M is a vector of length N and M(i) is the number of ROIs
%   associated with the i-th input feature map. This layer is used in Fast
%   R-CNN and Faster R-CNN object detection networks.
%
%   <a href="matlab:helpview('vision','rcnnConcept')">Learn more about Fast R-CNN and Faster R-CNN.</a>
%   
%   ROIMaxPooling2DLayer properties:
%      Name         - The layer name.
%      OutputSize   - The pooled output size, [height width].
%      NumInputs    - The number of inputs of the layer.
%      InputNames   - The names of the inputs of the layer.
%      NumOutputs   - The number of outputs of the layer.
%      OutputNames  - The names of the outputs of the layer.
%   
%   Example - Create an ROI max pooling layer
%   -----------------------------------------
%   % Size of the ROI pooled output.
%   outputSize = [7 7];
%
%   % Create ROI max pooling layer.
%   roiPool = roiMaxPooling2dLayer(outputSize,'Name','roiPool')
%
%   See also roiMaxPooling2dLayer, trainFastRCNNObjectDetector, 
%            trainFasterRCNNObjectDetector.

% Copyright 2016-2023 The MathWorks, Inc.

classdef ROIMaxPooling2DLayer < nnet.cnn.layer.Layer & nnet.internal.cnn.layer.Externalizable      
    
    properties(Dependent)
        % Name A name for the layer.
        %   The name for the layer. If this is set to '', then a name will
        %   be automatically set at training time.
        Name
    end
    
    properties(SetAccess = private, Dependent)
        %  OutputSize The pooled output size, [height width].
        %  The height and width by which to partition an ROI for max
        %  pooling. An ROI is divided into OutputSize blocks. The dimensions
        %  of each block is floor( [roiHeight roiWidth] ./ OutputSize] )
        %  pixels. The data within each block is max pooled and returned as
        %  a feature map for each input ROI. Grid based partitioning
        %  enables fixed-sized outputs from arbitrarily sized ROIs.        
        OutputSize
    end
    
    properties(Hidden, SetAccess = private, Dependent)
        % This parameter should no longer be used. Use OutputSize instead.
        GridSize
        
        % ScaleFactor
        ScaleFactor 
    end
    
    methods
        function this = ROIMaxPooling2DLayer(privateLayer)
            this.PrivateLayer = privateLayer;
        end
        
        function this = set.OutputSize(this, sz)
            nnet.cnn.layer.ROIMaxPooling2DLayer.validateOutputSize(sz, mfilename, 'OutputSize');
            this.PrivateLayer.GridSize = sz;
        end
        
        function sz = get.OutputSize(this)
            sz = this.PrivateLayer.GridSize;
        end
        function this = set.GridSize(this, sz)
            this.OutputSize = sz;
        end
        
        function sz = get.GridSize(this)
            sz = this.OutputSize;
        end
        
        function val = get.ScaleFactor(this)
            val = this.PrivateLayer.ScaleFactor;
        end
        
        function name = get.Name(this)
            name = this.PrivateLayer.Name;
        end
        
        function this = set.Name(this, val)
            iAssertValidLayerName(val);
            this.PrivateLayer.Name = char(val);
        end
        
        function val = this.ScaleFactor(this)
            val = this.PrivateLayer.ScaleFactor;
        end
        
        function s = saveobj(this)
            privateLayer   = this.PrivateLayer;
            s.Version      = 2.0;
            s.Name         = privateLayer.Name;
            s.OutputSize   = privateLayer.GridSize;    
            s.ScaleFactor  = privateLayer.ScaleFactor;
        end
    end
    
    methods(Access = protected)
        function [description, type] = getOneLineDisplay(this)
            sizeString  = mat2str(this.OutputSize);
            description = getString(message('vision:roiPooling:oneLineDisp', sizeString));
            
            type = getString(message('vision:roiPooling:Type'));
        end
        
        %------------------------------------------------------------------
        function groups = getPropertyGroups( this )
            hyperparameters = {
                'OutputSize'
                };
            
            groups = [
                this.propertyGroupGeneral( {'Name','NumInputs','InputNames'} )
                this.propertyGroupHyperparameters( hyperparameters )
                ];
        end
    end
    
    methods(Hidden, Static)
        function validateOutputSize(sz, name, varname)
            
            validateattributes(sz, {'numeric'}, ...
                {'row', 'numel', 2, 'positive', 'real', 'nonsparse'}, ...
                name, varname);
            
        end
        
        %------------------------------------------------------------------
        function obj = loadobj(s)
            s = iUpdateFromPreviousVersion(s);
            internalLayer = nnet.internal.cnn.layer.ROIMaxPooling2DLayer(...
                s.Name, s.OutputSize,s.ScaleFactor);           
            
            obj = nnet.cnn.layer.ROIMaxPooling2DLayer(internalLayer);
        end
    end
end

%--------------------------------------------------------------------------
function s = iUpdateFromPreviousVersion(s)
if s.Version < 2
    s.OutputSize = s.GridSize;
    s.ScaleFactor = [];
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