%ROIAlignLayer ROI Align layer.
%
%   Use ROIAlignLayer to create this layer.
%
%   The ROI align layer outputs fixed size feature maps for every
%   rectangular ROI within the input feature map. Given an input feature
%   map of size [H W C N], where C is the number of channels and N is the
%   number of observations, the output feature map size is [height width C
%   sum(M)], where M is a vector of length N and M(i) is the number of ROIs
%   associated with the i-th input feature map. This layer is used in 
%   Mask-RCNN networks.
%
%   
%   ROIAlignLayer properties:
%      Name          - The layer name.
%      OutputSize    - The pooled output size, [height width].
%      ROIScale      - The factor used  to scale input ROIs
%                      to the input feature map size.
%      SamplingRatio - Number of sampling points in each bin.
%      NumInputs     - The number of inputs of the layer.
%      InputNames    - The names of the inputs of the layer.
%      NumOutputs    - The number of outputs of the layer.
%      OutputNames   - The names of the outputs of the layer.
%   
%   Example - Create an ROI align layer
%   -----------------------------------------
%   % Output Grid size
%   outputSize = [7 7];
%
%   % Create ROI align layer.
%   roiAlign = roiAlignLayer(outputSize,'Name','roiAlign')
%
%   See also roiAlignLayer, roiMaxPooling2dLayer

% Copyright 2020 The MathWorks, Inc.

classdef ROIAlignLayer < nnet.cnn.layer.Layer & nnet.internal.cnn.layer.Externalizable      
    
    properties(Dependent)
        % Name A name for the layer.
        %   The name for the layer. If this is set to '', then a name will
        %   be automatically set at training time.
        Name
    end
    
    properties(SetAccess = private, Dependent)
        %  OutputSize The pooled output size, [height width].
        %  The height and width by which to partition an ROI for 
        %  pooling into fixed size bins without quantizing the grid points. 
        %  Each bin is further sampled at SamplingRatio locations. The 
        %  value at each sampled point is inferred using bilinear interpolation. 
        %  The sampled values in each bin is averaged.
        OutputSize
        
        % ROIScale Scale of input feature map to the input image. 
        % This value is a scalar.
        ROIScale
        
        % SamplingRatio Number of samples in each pooled bin along height
        % and width expressed as a 1x2 vector. The interpolated
        % value of these points are used to determine the output
        % value of each pooled bin. The default is an adaptive
        % number of samples calculates as ceil(roiWidth/outputWidth)
        % along the X and similarly calculated for samples along Y.
        SamplingRatio
    end
    
    methods
        function this = ROIAlignLayer(privateLayer)
            this.PrivateLayer = privateLayer;
        end
        
        function this = set.OutputSize(this, sz)
            nnet.cnn.layer.ROIAlignLayer.validateOutputSize(sz, mfilename, 'OutputSize');
            this.PrivateLayer.OutputSize = sz;
        end
        
        function sz = get.OutputSize(this)
            sz = this.PrivateLayer.OutputSize;
        end
        
        function this = set.SamplingRatio(this, ratio)
            nnet.cnn.layer.ROIAlignLayer.validateSamplingRatio(ratio, mfilename, 'SamplingRatio');
            this.PrivateLayer.SamplingRatio = ratio;
        end

        function val = get.SamplingRatio(this)
            val = this.PrivateLayer.SamplingRatio;
        end
        
        function this = set.ROIScale(this, scale)
            nnet.cnn.layer.ROIAlignLayer.validateROIScale(scale, mfilename, 'ROIScale');
            this.PrivateLayer.ROIScale = scale;
        end
        
        function val = get.ROIScale(this)
            val = this.PrivateLayer.ROIScale;
        end
        
        function name = get.Name(this)
            name = this.PrivateLayer.Name;
        end
        
        function this = set.Name(this, val)
            iAssertValidLayerName(val);
            this.PrivateLayer.Name = char(val);
        end
        
 
        
                
        function s = saveobj(this)
            privateLayer    = this.PrivateLayer;
            s.Version       = 1.0;
            s.Name          = privateLayer.Name;
            s.OutputSize    = privateLayer.OutputSize;    
            s.ROIScale      = privateLayer.ROIScale;
            s.SamplingRatio = privateLayer.SamplingRatio;
        end
    end
    
    methods(Access = protected)
        function [description, type] = getOneLineDisplay(this)
            sizeString  = mat2str(this.OutputSize);
            description = getString(message('vision:roiAlign:oneLineDisp', sizeString));
            
            type = getString(message('vision:roiAlign:Type'));
        end
        
        %------------------------------------------------------------------
        function groups = getPropertyGroups( this )
            hyperparameters = {
                'ROIScale',...
                'SamplingRatio'
                };
            
            groups = [
                this.propertyGroupGeneral( {'Name','NumInputs','InputNames','OutputSize'} )
                this.propertyGroupHyperparameters( hyperparameters )
                ];
        end
    end
    
    methods(Hidden, Static)
        function validateOutputSize(sz, name, varname)
            
            validateattributes(sz, {'numeric'}, ...
                {'row', 'nonempty', 'numel', 2, 'positive', 'real', 'nonsparse', 'integer'}, ...
                name, varname);
            
        end
        
        function validateSamplingRatio(ratio, name, varname)
            
            if(isstring(ratio))
                ratio = char(ratio);
            end
            
            if(ischar(ratio))
                validatestring(ratio,{'auto'}, name, varname);
            else
                validateattributes(ratio, {'numeric'}, ...
                    {'real', 'nonempty', 'numel', 2, 'integer', 'positive', 'nonsparse','>=',1}, ...
                    name, varname);
            end
            
        end
        
        function validateROIScale(scale, name, varname)
            
            validateattributes(scale, {'numeric'}, ...
                {'real', 'nonempty','scalar', 'positive', 'nonsparse', '>', 0}, ...
                name, varname);
            
        end
        %------------------------------------------------------------------
        function obj = loadobj(s)
            internalLayer = nnet.internal.cnn.layer.ROIAlignLayer(...
                s.Name, s.OutputSize,s.ROIScale,s.SamplingRatio);           
            
            obj = nnet.cnn.layer.ROIAlignLayer(internalLayer);
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