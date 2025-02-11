classdef bBoxDataAugmenter < imageDataAugmenter
    
 % bBoxDataAugmenter Configure bounding box data augmentation
    %
    %   aug = bBoxDataAugmenter() creates a bBoxDataAugmenter 
    %   object with default property values. The default state of the
    %   bBoxDataAugmenter is the identity transformation.
    %
    %   aug = bBoxDataAugmenter(Name,Value,___) configures a set of 
    %   image augmentation options using Name/Value pairs to set properties.
    %
    %   bBoxDataAugmenter properties:
    %       FillValue           - Value used to define out of bounds points
    %       RandXReflection     - Random X reflection
    %       RandYReflection     - Random Y reflection
    %       RandRotation        - Random rotation
    %       RandXScale          - Random X scale
    %       RandYScale          - Random Y scale
    %       RandXShear          - Random X shear
    %       RandYShear          - Random Y shear
    %       RandXTranslation    - Random X translation
    %       RandYTranslation    - Random Y translation
    %
    %   Example 1
    %   ---------
    %   Train a convolutional neural network on some synthetic images of
    %   handwritten digits. Apply random rotations during training to add
    %   rotation invariance to trained network.
    %
    %   [XTrain, YTrain] = digitTrain4DArrayData;
    %
    %   imageSize = [28 28 1];
    %
    %   layers = [ ...
    %       imageInputLayer(imageSize, 'Normalization', 'none');
    %       convolution2dLayer(5,20);
    %       reluLayer();
    %       maxPooling2dLayer(2,'Stride',2);
    %       fullyConnectedLayer(10);
    %       softmaxLayer();
    %       classificationLayer()];
    %
    %   opts = trainingOptions('sgdm');
    %
    %   imageAugmenter = imageDataAugmenter('RandRotation',[-10 10]);
    %
    %   datasource = augmentedImageDatasource(imageSize,XTrain,YTrain,'DataAugmentation',imageAugmenter);
    %
    %   net = trainNetwork(datasource, layers, opts);
    %
    % See also augmentedImageDatasource, imageInputLayer, trainNetwork

    % Copyright 2017 The MathWorks, Inc.
    
    
    methods
        
        function self = bBoxDataAugmenter(varargin)
            %bBoxDataAugmenter Construct bBoxDataAugmenter object.
            %
            %   augmenter = bBoxDataAugmenter() constructs an
            %   boundingBoxDataAugmenter object with default property settings.
            %
            %   augmenter = bBoxDataAugmenter('Name',Value,___) specifies
            %   parameters that control aspects of data augmentation.
            %   Parameter names can be abbreviated and case does not
            %   matter.
            %
            %   Parameters include:
            %
            %   'FillValue'         A numeric scalar or vector 
            %                       (when augmenting multi-channel images) that
            %                       defines the value used during resampling
            %                       when transformed points fall out of bounds.
            %
            %                       Default: 0
            %
            %   'RandXReflection'   A scalar logical that defines whether
            %                       random left/right reflections are
            %                       applied.
            %
            %                       Default: false
            %
            %   'RandYReflection'   A scalar logical that defines whether
            %                       random up/down reflections are applied.
            %
            %                       Default: false
            %
            %   'RandRotation'      A two element vector that defines the
            %                       uniform range, in units of degrees, of
            %                       rotations that will be applied.
            %
            %                       Default: [0 0]
            %
            %   'RandXScale'        A two element vector that defines the
            %                       uniform range of scale that will
            %                       applied in the X dimension.
            %
            %                       Default: [1 1]
            %
            %   'RandYScale'        A two element vector that defines the
            %                       uniform range of scale that will be
            %                       applied in the Y dimension.
            %
            %                       Default: [1 1]
            %
            %   'RandXShear'        A two element vector that defines the
            %                       uniform range of shear that will be
            %                       applied in the X dimension. Specified
            %                       as a shear angle in units of degrees.
            %
            %                       Default: [0 0]
            %
            %   'RandYShear'        A two element vector that defines the
            %                       uniform range of shear that will be
            %                       applied in the Y dimension. Specified
            %                       as a shear angle in units of degrees.
            %
            %                       Default: [0 0]
            %
            %   'RandXTranslation'  A two element vector that defines the
            %                       uniform range of translation that will
            %                       be applied in the X dimension. Specified
            %                       in units of pixels.
            %
            %                       Default: [0 0]
            %
            %   'RandYTranslation'  A two element vector that defines the
            %                       uniform range of translation that will
            %                       be applied in the Y dimension. Specified
            %                       in units of pixels.
            %
            %                       Default: [0 0]            
            % self.parseInputs(varargin{:});
            self = self@imageDataAugmenter(varargin{:});
        end
        
    end
    
    methods (Hidden)
        
        % @override
        function [A, B] = augmentPair(self, X, Y)
            %augmentPair Augment inputs X and Y.
            %
            %   [A, B] = augmentPair(augmenter, X, Y) performs the same
            %   image augmentation on the input image X and Y. X is a
            %   MxNxC matrix or N element cell arrays containing MxNxC
            %   images. Y is an Mx4 array of object bounding boxes.
                 
            % apply augmentation on the image
            A = self.augment(X);
            tform = self.AffineTransforms;  
            
            % apply that augmentation on the bounding box
            B = self.augmentY(Y, tform);                        
            
        end
        
        
        % @override
        function B = augmentY(~, Y, tform)
            % Augment bounding boxes using known transform `tform`.
            
            if isnumeric(Y) && ismatrix(Y)   
                % ---------------------------------------------------------
                % Y is an Mx4 array of bounding boxes
                %   This is usually obtained from a nextBatch() call on a
                %   boundingBoxImageDatasource object, returning a single
                %   image with possibly multiple bounding boxes.
                B = augmentBboxes(Y);
                
                
            elseif iscell(Y)
                % ---------------------------------------------------------
                % Y is a cell array of bounding boxes
                %   This is usually obtained by the getObservations()
                %   method being called on a boundingBoxImageDatasource 
                %   object. 
                B = cell(size(Y));
                for jj = 1:numel(Y)   
                    B{jj} = augmentBboxes(Y{jj});
                end
            else
                assert(false,'Invalid bounding box input. Y needs to be an Mx4 array of bounding boxes.');
            end
            
            
            function B = augmentBboxes(Y)
                
                % convert bounding boxes to points
                %   bbox2points() returns a 4x2xM array for M bboxes
                pointsOrig = bbox2points(Y);          
                numBoxes = size(pointsOrig, 3);
                
                % reshape points to (4*M)x2 for transformPointsForward()
                % which needs the input to be in the form of an Nx2 array
                pointsOrig = permute(pointsOrig, [1 3 2]);
                pointsOrig = reshape(pointsOrig, [4*numBoxes 2]);
                
                % apply warping to points
                pointsWarped = transformPointsForward(affine2d(tform), pointsOrig);
                pointsWarped = reshape(pointsWarped, [4 numBoxes 2]);  
                
                % convert warped points to rectangle vertices
                %   note that `pointsWarped` is a 4xMx2 array
                pointsWarped = round(pointsWarped);
                x1 = squeeze( min( pointsWarped(:,:,1), [], 1) ) ;
                x2 = squeeze( max( pointsWarped(:,:,1), [], 1) ) ;
                y1 = squeeze( min( pointsWarped(:,:,2), [], 1) ) ;
                y2 = squeeze( max( pointsWarped(:,:,2), [], 1) ) ;
                
                % convert rectangle vertices to bboxes
                w = x2 - x1 + 1;
                h = y2 - y1 + 1;     
                B = [x1; y1; w; h]';    
                
            end
            
           
            function [x1, y1, w, h] = points2bbox(points)
            % convert transformed points back to bounding boxes
            %   - the 4 corner points of a bbox are in a 4x2xM matrix
            %   - approximated to axis-aligned rectangles         
                x1 = round(min(squeeze(points(:, 1))));     
                x2 = round(max(squeeze(points(:, 1)))); 

                y1 = round(min(squeeze(points(:, 2))));     
                y2 = round(max(squeeze(points(:, 2))));

                w = x2 - x1 + 1;
                h = y2 - y1 + 1;
            end
            
            
        end
        
    end
    
    
        methods(Static, Hidden = true)
        function self = loadobj(S)
            % RandScale property was introduced in R2018b
            if isfield(S,'RandScale')
                self = vision.internal.cnn.bBoxDataAugmenter('FillValue',S.FillValue,...
                'RandXReflection',S.RandXReflection,...
                'RandYReflection',S.RandYReflection,...
                'RandRotation',S.RandRotation,...
                'RandScale',S.RandScale,...
                'RandXScale',S.RandXScale,...
                'RandYScale',S.RandYScale,...
                'RandXShear',S.RandXShear,...
                'RandYShear',S.RandYShear,...
                'RandXTranslation',S.RandXTranslation,...
                'RandYTranslation',S.RandYTranslation);
                
            else
                self = vision.internal.cnn.bBoxDataAugmenter('FillValue',S.FillValue,...
                    'RandXReflection',S.RandXReflection,...
                    'RandYReflection',S.RandYReflection,...
                    'RandRotation',S.RandRotation,...
                    'RandXScale',S.RandXScale,...
                    'RandYScale',S.RandYScale,...
                    'RandXShear',S.RandXShear,...
                    'RandYShear',S.RandYShear,...
                    'RandXTranslation',S.RandXTranslation,...
                    'RandYTranslation',S.RandYTranslation);
            end
            
            if isfield(S,'Stream')
                self.Stream = S.Stream; 
            end    
        end
    end
    
    methods (Hidden)
        function S = saveobj(self)
            % Serialize denoisingImageDatasource object
            S = struct('FillValue',self.FillValue,...
                'RandXReflection',self.RandXReflection,...
                'RandYReflection',self.RandYReflection,...
                'RandRotation',self.RandRotation,...
                'RandScale',self.RandScale,...
                'RandXScale',self.RandXScale,...
                'RandYScale',self.RandYScale,...
                'RandXShear',self.RandXShear,...
                'RandYShear',self.RandYShear,...
                'RandXTranslation',self.RandXTranslation,...
                'RandYTranslation',self.RandYTranslation);
           if isprop(self,'Stream')
               S.Stream = self.Stream;
           end
        end
        
    end
       
end
