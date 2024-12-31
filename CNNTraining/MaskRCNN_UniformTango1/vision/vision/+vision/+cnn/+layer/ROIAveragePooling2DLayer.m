classdef ROIAveragePooling2DLayer < nnet.cnn.layer.Layer & nnet.internal.cnn.layer.Externalizable
    % ROIAveragePooling2DLayer ROI average pooling layer.
    %
    %   To create an ROI average pooling layer, use roiAveragePooling2dLayer.
    %
    %   ROIAveragePooling2DLayer properties (read-only):
    %     Name         - A name for the layer.
    %     GridSize     - The height and width used to partition an ROI for 
    %                    average pooling.
    %     NumInputs    - The number of inputs of the layer.
    %     InputNames   - The names of the inputs of the layer.
    %     NumOutputs   - The number of outputs of the layer.
    %     OutputNames  - The names of the outputs of the layer.
    %
    % Example
    % -------
    % layer = roiAveragePooling2dLayer([6 6])
    %
    % See also roiAveragePooling2dLayer.
    
    % Copyright 2016-2018 The MathWorks, Inc.
    
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
        GridSize
        ScaleFactor
    end
    
    
    methods
        function this = ROIAveragePooling2DLayer(privateLayer)
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
        
        function val = get.ScaleFactor(this)
            val = this.PrivateLayer.ScaleFactor;
        end
        
        function sz = get.GridSize(this)
            sz = this.OutputSize;
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
            s.Version      = 2.0;
            s.Name         = privateLayer.Name;
            s.OutputSize   = privateLayer.GridSize;      
            s.ScaleFactor  = privateLayer.ScaleFactor;
        end
    end
    
    methods(Access = protected)
        function [description, type] = getOneLineDisplay(this)
            sizeString  = mat2str(this.GridSize);
            description = getString(message('vision:roiPooling:oneLineDispAvg', sizeString));
            
            type = getString(message('vision:roiPooling:TypeAvg'));
        end
        
        function groups = getPropertyGroups( this )
            hyperparameters = {
                'GridSize'
                };
            
            groups = [
                this.propertyGroupGeneral( {'Name','NumInputs','InputNames'} )
                this.propertyGroupHyperparameters( hyperparameters )
                ];
        end
    end
    
    methods(Hidden, Static)
        
        function obj = loadobj(s)
            s = iUpdateFromPreviousVersion(s);
            
            internalLayer = vision.internal.cnn.layer.ROIAveragePooling2DLayer(...
                s.Name, s.OutputSize, s.ScaleFactor);     
            
            obj = vision.cnn.layer.ROIAveragePooling2DLayer(internalLayer);
        end
    end
end

function s = iUpdateFromPreviousVersion(s)
if s.Version < 2
    s.OutputSize = s.GridSize;
    s.ScaleFactor = [];
end
end

function iAssertValidLayerName(name)
nnet.internal.cnn.layer.paramvalidation.validateLayerName(name);
end
