%bBoxImageDatasource Data source for training object detector networks.
%
%   datasource = bBoxImageDatasource(trainingDataTable) creates a
%   datasource from a table of training annotations. This can handle
%   multi-category labels for the objects within the bounding boxes.
%   The table must be in the format for object detector training data:
%       | imageFileNames | classA | classB | classC |...
%    Each row of the table has the path to an image, stored under the first
%    column. The subsequent columns store Mx4 bounding boxes for M objects
%    of a specific category (e.g. 'classA' or 'classB' etc.) present in
%    that image. If there are no objects of a particular class present in
%    an image, then the value under that class' column in that row is set
%    to be an empty matrix.
%
%   datasource = bBoxImageDatasource(groundTruthObject) creates a
%   datasource from a groundTruth object. This can handle multi-category
%   object labels along with bounding boxes.
%
%   datasource = bBoxImageDatasource(imageFilenames, bboxes) creates
%   a data source for training an object detector network.
%   The inputs are a list of image filenames and a list of corresponding
%   bounding boxes. The output is a bBoxImageDatasource object. This
%   handles only a single foreground-object class.
%
%   datasource = bBoxImageDatasource(imds, bboxes) where the first
%   input is an object of |imageDatastore| class and the second input is a
%   cell array of Mx4 bounding boxes. The output is a
%   bBoxImageDatasource object. This handles only a single
%   foreground-object class.
%
%
%
%   [...] = bBoxImageDatasource(..., Name, Value) specifies
%   additional name-value pair arguments described below:
%
%      'DataAugmentation'      Specify image data augmentation using a
%                              bBoxDataAugmenter object or 'none'.
%                              Training data is augmented in real-time
%                              while training.
%
%                              Default: 'none'
%
%      'BboxOverflowMode'      A scalar string or character vector
%                              specifying the technique used to adjust
%                              bounding boxes when they overflow from the
%                              image margins as a result of augmentation.
%                              Valid values are 'cut', 'clip' and
%                              'centerclip'. The 'cut' option discards all
%                              those boxes that overflow, while 'clip'
%                              resizes the bounding box to fit in  the
%                              image. The 'center-clip' option clips only
%                              if the center of the bounding box lies
%                              outside the image margins.
%
%                              Default: 'cut'
%
%
%      'ColorPreprocessing'    A scalar string or character vector
%                              specifying color channel pre-processing.
%                              This option can be used when you have a
%                              training set that contains both color and
%                              grayscale image data and you need data
%                              created by the datasource to be strictly
%                              color or grayscale. Valid values are
%                              'gray2rgb', 'rgb2gray', and 'none'. For
%                              example, if you need to train a  network
%                              that expects color images but some of the
%                              images in your training set are grayscale,
%                              then specifying the option 'gray2rgb' will
%                              replicate the color channels of the
%                              grayscale images in the input image set to
%                              create M-by-N-by-3 output images.
%
%                              Default: 'none'
%
%      'ResizedShortestSide'   A single scalar specifying how to resize
%                              the input image.
%                              When a single scalar is provided the
%                              smallest side of the image is resized to
%                              this value, maintaing aspect ratio. This can
%                              be combined with the 'OutputSize' and
%                              'OutputSizeMode' options to first resize the
%                              input image, then take crops out of it. E.g.
%                              rescale image such that its smallest side is
%                              600 pixels, then take 512x512 random crops
%                              out of it.
%
%                               Default: []
%
%      'OutputSize'            A two element vector specifying the number
%                              of rows and columns in images produced by
%                              the datasource. When specified, image sizes
%                              are adjusted as necessary to achieve output
%                              of images of the specified size. By default,
%                              'OutputSize' is empty and the output size is
%                              not adjusted.
%
%                              Default: []
%
%      'OutputSizeMode'        A scalar string or character vector
%                              specifying the technique used to adjust
%                              image sizes to the specified 'OutputSize'.
%                              Valid values are 'resize', 'randcrop' and
%                              'centercrop'. This option only applies when
%                              'OutputSize' is not empty.
%
%                              Default: 'resize'
%
%      'BackgroundExecution'   Accelerate image augmentation by
%                              asyncronously reading, augmenting, and
%                              queueing augmented images for use in
%                              training. Requires Parallel Computing
%                              Toolbox.
%
%                              Default: false
%
%   bBoxImageDatasource Properties:
%
%      Images              - A list of image filenames.
%      BoundingBoxData     - A list of bounding boxes.
%      ClassLabelData      - A list of class labels.
%      ClassNames          - A cell array of object class names.
%      DataAugmentation    - Selected data augmentations.
%      ColorPreprocessing  - Selected color channel processing.
%      ResizedShortestSide - Resize shortest side of input image.
%      OutputSize          - Output size of images.
%      OutputSizeMode      - Output size adjustment mode.
%
%   bBoxImageDatasource Methods:
%
%       nextBatch()                  - Returns the next batch of data.
%       getObservations(indices)     - Returns a batch of data specified by
%                                      their indices.
%       encodeBboxRegression(bboxes) -
%       decodeBboxRegression(bboxes) -
%
%
%
%   Example 1
%   ---------
%   % Create a bBoxImageDatasource using single-class vehicle
%   % training data in the form of a table.
%
%   data = load('fasterRCNNVehicleTrainingData.mat');
%   trainingData = data.vehicleTrainingData;
%   trainingData(1:5, :);  % have a look at the table format
%
%   trainingData.imageFilename = fullfile(toolboxdir('vision'),'visiondata', ...
%       trainingData.imageFilename);
%
%   datasource = vision.internal.cnn.bBoxImageDatasource(trainingData) ;
%
%
%   Example 2
%   ---------
%   % Create a bBoxImageDatasource using multi-class training data in
%   % the form of a table.
%
%   load('stopSignsAndCars.mat');
%
%   % have a look at the table with multiple categories
%   stopSignsAndCars(1:5,:)
%
%   % append full paths to the image filenames
%   stopSignsAndCars.imageFilename = fullfile(toolboxdir('vision'),'visiondata', ...
%       stopSignsAndCars.imageFilename);
%
%   % create datasource and call a batch of data
%   datasource = vision.internal.cnn.bBoxImageDatasource(stopSignsAndCars) ;
%   [im, bbox, labels] = datasource.nextBatch();
%
%   % display annotated image
%   im1 = insertObjectAnnotation(im, 'rectangle', bbox, labels);
%   figure;
%   imshow(im1);
%
%
%   Example 3
%   ---------
%   % Create a bBoxImageDatasource with multi-class data and center crops.
%
%   % Load multi-class training data table
%   data = load('stopSignsAndCars.mat');
%   stopSignsAndCars = data.stopSignsAndCars;
%   stopSignsAndCars.imageFilename = ...
%         fullfile(toolboxdir('vision'),'visiondata', ...
%         stopSignsAndCars.imageFilename);
%
%   % Create datasource with multi-class data table  and center cropping
%   datasource = ...
%     vision.internal.cnn.bBoxImageDatasource(stopSignsAndCars, ...
%                                 'OutputSizeMode', 'centercrop', ...
%                                 'OutputSize', [600 600], ...
%                                 'BboxOverflowMode', 'clip') ;
%
%   % get data from the datasource
%   [im, bbox, labels] = datasource.nextBatch();
%
%   % display annotated image
%   im1 = insertObjectAnnotation(im, 'rectangle', bbox, labels);
%   figure;
%   imshow(im1);
%
%
%   Example 4
%   ---------
%   % Create a bBoxImageDatasource using a |groundTruth| object.
%
%   % Load data
%   data = load('stopSignsAndCars.mat');
%   stopSignsAndCars = data.stopSignsAndCars;
%   imageFiles = fullfile(toolboxdir('vision'),'visiondata', ...
%                           stopSignsAndCars.imageFilename);
%
%   % Define data source for ground truth
%   gtSource = groundTruthDataSource(imageFiles);
%
%   % Define labels used to specify the ground truth.
%   names = stopSignsAndCars.Properties.VariableNames(2:end);
%   names = names';
%   types = repmat(labelType('Rectangle'), [numel(names) 1]);
%   labelDefs = table(names, types, 'VariableNames', {'Name','Type'});
%
%   % Define a table of label data.
%   labelData = stopSignsAndCars(:, 2:end);
%
%   % Create a |groundTruth| object
%   gTruth = groundTruth(gtSource,labelDefs,labelData);
%
%   % Create a bounding box image datasource using this ground truth object
%   datasource = vision.internal.cnn.bBoxImageDatasource(gTruth) ;
%
%   % get data from the datasource
%   [im, bbox, labels] = datasource.nextBatch();
%
%   % display annotated image
%   im1 = insertObjectAnnotation(im, 'rectangle', bbox, labels);
%   figure;
%   imshow(im1);
%
%
%
% See also trainNetwork, groundTruth, imageDataAugmenter, imageDatastore.

% Copyright 2017 The MathWorks, Inc.
classdef bBoxImageDatasource < ...
        matlab.io.Datastore &...
        matlab.io.datastore.MiniBatchable &...
        matlab.io.datastore.Shuffleable &...
        matlab.io.datastore.BackgroundDispatchable
    
    properties(Dependent, SetAccess = private)
        %Images Source of ground truth images.
        Images
        
        %BoundingBoxes
        BoundingBoxes
        
        %Labels
        ClassLabels
    end
    
    
    properties(SetAccess = protected)
        CurrentImageIndex
    end
    
    properties(SetAccess = private)
        %DataAugmentation
        %Specify image data augmentation using an bBoxDataAugmenter object
        %or 'none'. Training data is augmented in real-time while training.
        DataAugmentation
        
        %ColorPreprocessing
        %A scalar string or character vector specifying color channel
        %pre-processing. This option can be used when you have a training
        %set that contains both color and grayscale image data and you need
        %data created by the datasource to be strictly color or grayscale.
        %Options are: 'gray2rgb','rgb2gray','none'. For example, if you
        %need to train a  network that expects color images but some of the
        %images in your training set are grayscale, then specifying the
        %option 'gray2rgb' will replicate the color channels of the
        %grayscale images in the input image set to create MxNx3 output
        %images. The default is 'none'.
        ColorPreprocessing
        
        % ResizedShortestSide
        % A single scalar or two-element vector specifying how to resize the
        % input image. When a single scalar is provided, the smallest side
        % of the image is resized to this value, maintaing aspect ratio.
        % This can be cascaded with the 'OutputSize' and'OutputSizeMode'
        % options to first resize the input image, then take crops out of
        % it. E.g. rescale image such that its smallest side is 600 pixels,
        % then take 512x512 random crops out of it.
        ResizedShortestSide
        
        %OutputSize
        %A two element vector specifying the number of rows and columns in
        %images produced by the datasource. When specified, image sizes are
        %adjusted as necessary to achieve output of images of the specified
        %size. By default, 'OutputSize' is empty and the ouput size is not
        %adjusted.
        OutputSize
        
        %OutputSizeMode
        % A scalar string or character vector specifying the technique used
        % to adjust image sizes to the specified 'OutputSize'. Valid values
        % are 'resize', 'centercrop' and 'randcrop'. This option only
        % applies when 'OutputSize' is not empty. The default is 'resize'.
        OutputSizeMode
        
        %BboxOverflowMode
        % A scalar string or character vector specifying the technique used
        % to adjust bounding boxes when they overflow from the image
        % margins as a result of augmentation. Valid values are 'cut',
        % 'clip' and 'center-clip'. The first option discards those boxes
        % that overflow, while 'clip' resizes the bounding box to fit in
        % the image. The 'center-clip' option clips only if the center of
        % the bounding box lies within the image margins.
        BboxOverflowMode
        
        %BackgroundExecution
        % Accelerate image augmentation by asyncronously reading,
        % augmenting, and queueing augmented images for use in training.
        % Requires Parallel Computing Toolbox.
        BackgroundExecution logical
        
        %ReshapeResponse
        % A logical flag that reshapes the matrix holding the encoding of
        % the bounding boxes. This is done for compatibility with the shape
        % of the predicted bounding boxes from a neural network during
        % training. The default is false.
        ReshapeResponse logical
        
    end
    
    properties
        MiniBatchSize
    end
    
    properties(Hidden, SetAccess=protected, Dependent)
        NumObservations
    end
    
    properties(Hidden)
        %ImageDatastore Datastore for images.
        ImageDatastore
        
        %BoundingBoxes Cell array of ground truth bounding boxes.
        BoundingBoxData
        
        %ClassLabelData Cell array of ground truth class labels.
        ClassLabelData
        
        ClassNames
    end
    
    methods (Hidden)
        function frac = progress(this)
            % Stub for now
        end
    end
    
    methods
        
        % Class constructor
        function this = bBoxImageDatasource(varargin)
            narginchk(1,inf);
            
            % Create internal datastores with default values
            opts.IncludeSubfolders = false;
            opts.ReadSize = 1;
            
            
            switch class(varargin{1})
                
                case 'table'
                    % constructor:
                    %   bBoxImageDatasource(trainingDataTable)
                    narginchk(1,inf);
                    [imageFilenames, bboxes, labels, params] ...
                        = iParseTableInput(varargin{:});
                    
                    this.ImageDatastore = imageDatastore(imageFilenames, ...
                        'IncludeSubfolders', opts.IncludeSubfolders, ...
                        'ReadSize', opts.ReadSize);
                    this.BoundingBoxData = bboxes;
                    this.ClassLabelData = labels;
                    this.ClassNames = iGetUniqueClassLabels(labels);
                    
                    
                case 'groundTruth'
                    % constructor:
                    %   bBoxImageDatasource(groundTruth)
                    [imageFilenames, bboxes, labels, params] = ...
                        iParseGroundTruthInput(varargin{:});
                    
                    this.ImageDatastore = imageDatastore(imageFilenames, ...
                        'IncludeSubfolders', opts.IncludeSubfolders, ...
                        'ReadSize', opts.ReadSize);
                    this.BoundingBoxData = bboxes;
                    this.ClassLabelData = labels;
                    this.ClassNames = iGetUniqueClassLabels(labels);
                    
                case 'matlab.io.datastore.ImageDatastore'
                    % constructor:
                    %   bBoxImageDatasource(imageDatastore, bboxes)
                    narginchk(2,inf);
                    [imds, bboxes, params] = iParseImdsBboxesInput(varargin{:});
                    
                    this.ImageDatastore = imds;
                    this.BoundingBoxData = bboxes;
                    this.ClassNames = []; % no class info
                    
                case 'cell'
                    % constructor:
                    %   bBoxImageDatasource(imageFilenames, bboxes)
                    narginchk(2,inf);
                    [imageFilenames, bboxes, params] = iParseImfnBboxesInput(varargin{:});
                    
                    this.ImageDatastore = imageDatastore(imageFilenames, ...
                        'IncludeSubfolders', opts.IncludeSubfolders, ...
                        'ReadSize', opts.ReadSize);
                    this.BoundingBoxData = bboxes;
                    this.ClassNames = []; % no class info
                    
                otherwise
                    error('Invalid call to boundingBoxImageDatasource() constructor.');
            end
            
            this.ColorPreprocessing  = params.ColorPreprocessing;
            this.DataAugmentation    = params.DataAugmentation;
            this.ResizedShortestSide = params.ResizedShortestSide;
            this.OutputSize          = params.OutputSize;
            this.OutputSizeMode      = params.OutputSizeMode;
            this.BackgroundExecution = params.BackgroundExecution;
            this.DispatchInBackground  = this.BackgroundExecution;
            this.BboxOverflowMode    = params.BboxOverflowMode;
            this.CurrentImageIndex   = 1;
            this.ReshapeResponse     = params.ReshapeResponse;
            
            this.reset();
        end
        
        %------------------------------------------------------------------
        function batchSize = get.MiniBatchSize(this)
            batchSize = this.ImageDatastore.ReadSize;
        end
        
        %------------------------------------------------------------------
        function set.MiniBatchSize(this,batchSize)
            this.ImageDatastore.ReadSize = batchSize;
        end
        
        
        %------------------------------------------------------------------
        function val = get.NumObservations(this)
            val = length(this.ImageDatastore.Files);
        end
        
        %------------------------------------------------------------------
        function val = get.ClassLabels(this)
            val = this.ClassLabelData;
        end
        
        %------------------------------------------------------------------
        function val = get.BoundingBoxes(this)
            val = this.BoundingBoxData;
        end
        
        %------------------------------------------------------------------
        function s = saveobj(this)
            s.Version = 1.0;
            s.imds = this.ImageDatastore;
            s.bboxes = this.BoundingBoxData;
            s.labels = this.ClassLabelData;
            s.ColorPreprocessing = this.ColorPreprocessing;
            s.DataAugmentation = this.DataAugmentation;
            s.BboxOverflowMode = this.BboxOverflowMode;
            s.ResizedShortestSide = this.ResizedShortestSide;
            s.OutputSize = this.OutputSize;
            s.OutputSizeMode = this.OutputSizeMode;
            s.BackgroundExecution = this.BackgroundExecution;
            s.DispatchInBackground  = this.BackgroundExecution;
            s.ReshapeResponse     = this.ReshapeResponse; 
        end
        
        %------------------------------------------------------------------
        function [tbl,info] = read(this)
            [images, bboxes, labels, info] = nextBatch(this);
            tbl = iMakeTable(images, bboxes, labels);
        end
        
        %------------------------------------------------------------------
        function [images, bboxes, labels, info] = nextBatch(this)
            % Returns a batch of images and their bounding boxes.
            % Optionally may also return the class labels.
            
            idx = this.CurrentImageIndex;
            
            % Check if end of data source is reached
            if idx > this.NumObservations
                warning('vision:cnn:EndOfDataSource', 'The end of the datasource has been reached.');
                images = [];
                bboxes = [];
                labels = [];
                return;
            end
            
            if this.MiniBatchSize == 1
                % Read and pre-process image and bounding box data.
                [images, info] = readimage(this.ImageDatastore, idx);
                bboxes = this.BoundingBoxData(idx, :);
                bboxes = bboxes{:}; % Mx4 matrix of M bounding boxes
                
                if ~isempty(this.ClassLabelData)
                    % If multi-class data is being used
                    labels = this.ClassLabelData{idx}; % Mx1 cell array of labels
                    [images, bboxes, labels] = preprocess(this, images, bboxes, labels);
                else
                    % No class information is being used
                    [images, bboxes] = preprocess(this, images, bboxes, []);
                    labels = [];
                end
                
                this.CurrentImageIndex = this.CurrentImageIndex + 1 ;
                
            else
                % Read mini-batch of image and bounding box data as cell
                % arrays. Optionally also read class label data.
                [images, info] = read(this.ImageDatastore);
                
                indices = idx:(idx + this.MiniBatchSize - 1) ;
                bboxes = this.BoundingBoxData(indices);
                
                
                % =========================================================
                assert(~isempty(images), 'Empty mini-batch of images.');
                assert(~isempty(bboxes), 'Empty mini-batch of bounding boxes.');
                assert(numel(images) == numel(bboxes));
                % =========================================================
                
                
                % Check if the data is multi-class or not
                if ~isempty(this.ClassLabelData)
                    if numel(indices)==1
                        % for a single sample, a cell array of strings
                        labels = this.ClassLabelData{indices};
                    else
                        % for batchsize > 1, cell array of cell array of
                        % strings
                        labels = this.ClassLabelData(indices);
                    end
                    
                    % =====================================================
                    assert(~isempty(labels), 'Empty mini-batch of class labels.');
                    assert(numel(labels) == numel(bboxes));
                    % =====================================================
                    
                else
                    labels = [];
                end
                
                % loop over the preprocessing for each image-bbox pair
                for ii = 1:numel(images)
                    if ~isempty(labels)
                        % multi-class data -- returns labels C{ii} for each
                        % bbox Y{ii}
                        [images{ii}, bboxes{ii}, labels{ii}] = preprocess(this, images{ii}, bboxes{ii}, labels{ii});
                    else
                        % only image and bbox -- no class label information
                        [images{ii}, bboxes{ii}] = preprocess(this, images{ii}, bboxes{ii}, []);
                    end
                end
                
                % update current index by batchsize
                this.CurrentImageIndex = this.CurrentImageIndex + this.MiniBatchSize ;
                
                % After the last batch is read, CurrentImageIndex will be
                % (NumObservations + 1). Further read attempts will
                % trigger the EOF warning in the beginning of nextBatch().
                
                
                %                 assert(this.NumObservations > this.CurrentImageIndex, ...
                %                   'Image index exceeded the number of observations during nextBatch() call on datasource.');
            end
            
            if this.ReshapeResponse
                bboxes = iReshapeSingleObjectBboxes(bboxes);
            end
            
        end
        
        
        
        %------------------------------------------------------------------
        function val = hasdata(this)
            % Check for end-of-data
            idx = this.CurrentImageIndex ;
            val = (idx <= this.NumObservations) ;
        end
        
    end
    
    methods(Hidden)
        
        %------------------------------------------------------------------
        function [data,info] = readByIndex(this,indices)
            % Create datastore partition via a copy and index. This is
            % faster than constructing a new datastore with the new
            % files.
            subds = copy(this.ImageDatastore);
            subds.Files = this.ImageDatastore.Files(indices);
            
            % Select multiple observations as cell-arrays
            images = readall(subds);
            bboxes = this.BoundingBoxData(indices, :);
            if ~isempty(this.ClassLabelData)
                labels = this.ClassLabelData(indices);
            else
                labels = [];
            end
            
            % loop over the preprocessing for each image-bbox pair
            for ii = 1:numel(images)
                if ~isempty(labels)
                    % multi-class data -- returns labels C{ii} for each
                    % bbox Y{ii}
                    [images{ii}, bboxes{ii}, labels{ii}] = preprocess(this, images{ii}, bboxes{ii}, labels{ii});
                else
                    % only image and bbox -- no class label information
                    [images{ii}, bboxes{ii}] = preprocess(this, images{ii}, bboxes{ii}, []);
                end
            end
            
            if this.ReshapeResponse
                bboxes = iReshapeSingleObjectBboxes(bboxes);
            end
            
            
            data = iMakeTable(images,bboxes,labels);
            
            info.TODO = 'figure out what to put in here';
            
        end
        
        %------------------------------------------------------------------
        function reset(this)
            reset(this.ImageDatastore);
            this.CurrentImageIndex = 1;
        end
        
        %------------------------------------------------------------------
        function dsnew = shuffle(this)
            dsnew = copy(this);
            ord = randperm( numel(this.Images) );
            reorder(dsnew, ord);
        end
        
        %------------------------------------------------------------------
        function reorder(this, indices)
            this.ImageDatastore.Files = this.ImageDatastore.Files(indices);
            this.BoundingBoxData = this.BoundingBoxData(indices);
            this.ClassLabelData = this.ClassLabelData(indices);
        end
        
    end
    
    methods
        %------------------------------------------------------------------
        function src = get.Images(this)
            src = this.ImageDatastore.Files;
        end
    end
    
    methods(Hidden, Static)
        %------------------------------------------------------------------
        function this = loadobj(s)
            this = vision.internal.cnn.bBoxImageDatasource(s.imds, s.bboxes);
            this.ClassLabelData = s.labels;
            this.ColorPreprocessing = s.ColorPreprocessing;
            this.DataAugmentation = s.DataAugmentation;
            this.BboxOverflowMode = s.BboxOverflowMode;
            this.ResizedShortestSide = s.ResizedShortestSide;
            this.OutputSize = s.OutputSize;
            this.OutputSizeMode = s.OutputSizeMode;
            this.BackgroundExecution = s.BackgroundExecution;
            this.DispatchInBackground  = s.BackgroundExecution;
            this.ReshapeResponse     = s.ReshapeResponse;
        end
        
        
        %------------------------------------------------------------------
        function [X,Y,C] = augment(augmenter, X, Y, C, overflowMode)
            % Apply augmentation to the image (X) and bbox (Y) pair. If
            % class labels C is not empty, then any modification of Y is
            % reflected in the corresponding elements of C.
            
            if iscell(Y)
                Y = cellfun(@(y)single(y),Y,'UniformOutput', false);
            else
                Y = single(Y);
            end
            
            % Transform image and bounding boxes pair
            [X,Y] = augmenter.augmentPair(X,Y);
            
            % Validate the transformed bboxes
            if ~iscell(Y)
                % bboxes from a single image
                [Y, removeRows] = iValidateTransformedBoundingBoxes(...
                    X, Y, overflowMode);
                
                if ~isempty(C)
                    % sync class labels with Y
                    C = C(~removeRows);
                end
            else
                % cell array of bboxes from multiple images
                for kk = 1:numel(Y)
                    [Y{kk}, removeRows] =  iValidateTransformedBoundingBoxes( ...
                        X{kk}, Y{kk}, overflowMode);
                    
                    % sync class labels with Y
                    if ~isempty(C)
                        Ck = C{kk};
                        C{kk} = Ck(~removeRows);
                    end
                end
            end
            
        end
        
    end
    
    
    methods(Hidden, Access = private)
        %------------------------------------------------------------------
        function [img, bboxes, labels] = augmentData(this, img, bboxes, labels)
            if ~strcmp(this.DataAugmentation,'none')
                [img,bboxes,labels] = ...
                    vision.internal.cnn.bBoxImageDatasource.augment( ...
                    this.DataAugmentation, img, bboxes, labels, this.BboxOverflowMode);
            end
        end
        
        
        %------------------------------------------------------------------
        function [img, bboxes, labels] = preprocess(this, img, bboxes, labels)
            % Apply the following pre-processing steps on X and Y:
            %
            %   1. Convert between colorspaces.
            %   2. Apply data augmentation depending on the
            %       boundingBoxDataAugmenter object passed as
            %       'DataAugmentation'.
            %   3. Apply output size selection to the image
            
            
            %--------------------------------------------------------------
            % Apply color preprocessing on the image X.
            %--------------------------------------------------------------
            switch this.ColorPreprocessing
                case 'none'
                case 'rgb2gray'
                    img = iConvertAnyRGBToGray(img);
                case 'gray2rgb'
                    img = iConvertAnyGrayToRGB(img);
            end
            
            
            %--------------------------------------------------------------
            % Apply data augmentation.
            %--------------------------------------------------------------
            [img, bboxes, labels] = augmentData(this, img, bboxes, labels);
            % Class labels C are passed along to make sure discarded
            % bounding boxes also have the corresponding label discarded.
            
            
            %--------------------------------------------------------------
            % Apply input image resizing.
            %--------------------------------------------------------------
            if ~isempty(this.ResizedShortestSide)
                
                if numel(this.ResizedShortestSide) > 1
                    % Randomly selects a value from the array of
                    % this.ResizedShortestSide values (useful for
                    % multi-scale training)
                    resizedValue = this.ResizedShortestSide( ...
                        randi(length(this.ResizedShortestSide)) );
                    
                elseif numel(this.ResizedShortestSide) == 1
                    % Rescales smallest side of image to specified value
                    resizedValue = this.ResizedShortestSide ;
                    
                else
                    % =====================================================
                    error('Invalid option for ResizedShortestSide property of boundingBoxImageDatasource.');
                    % =====================================================
                    
                end
                [img,bboxes] = iResizeSmallestSide(img, bboxes, resizedValue);
            end
            
            
            %--------------------------------------------------------------
            % Apply output size selection.
            %--------------------------------------------------------------
            if ~isempty(this.OutputSize)
                imSize = size(img);
                imSize = imSize(1:2);
                
                switch this.OutputSizeMode
                    case 'resize'
                        bboxes = iResizeBoundingBox(bboxes, imSize, this.OutputSize);
                        img = iResize(img, this.OutputSize);
                        
                    case 'randcrop'
                        iCheckCropSize(imSize, this.OutputSize);
                        [img, bboxes, labels] = iRandCrop(img, bboxes, labels, this.OutputSize, this.BboxOverflowMode);
                        
                    case 'centercrop'
                        iCheckCropSize(imSize, this.OutputSize);
                        [img, bboxes, labels] = iCenterCrop(img, bboxes, labels, this.OutputSize, this.BboxOverflowMode);
                end
                
            end
            
            % Leave X as cell array. It will be converted to 4D array by
            % datasource dispatcher.
        end
        
    end
    methods(Access = protected)
        function aCopy = copyElement(this)
            %   COPYELEMENT   Create a deep copy of the datastore
            %   Create a deep copy of the datastore. We need to call
            %   copy on the datastore's property ImageDatastore, because it is
            %   a handle object. Creating a deep copy allows methods
            %   such as readall and preview, that call the copy method,
            %   to remain stateless.
            aCopy = copyElement@matlab.mixin.Copyable(this);
            aCopy.ImageDatastore = copy(this.ImageDatastore);
            aCopy.reset();
        end
    end
end



% =========================================================================
%% Parse constructor arguments
%  ------------------------------------------------------------------------
%   - iParseGroundTruthInput()
%   - iParseTableInput()
%   - iParseImdsBboxesInput()
%   - iParseImfnBboxesInput()
% =========================================================================

%--------------------------------------------------------------------------
function [imageFilenames, bboxes, labels, params] = iParseGroundTruthInput(varargin)
%   Parse varargin for constructor:
%       ___(groundTruth)
p = inputParser;
p.addRequired('gTruth', @iCheckGroundTruth);
p = iAddCommonParameters(p);
p.parse(varargin{:});
userInput = p.Results;

gTruth = userInput.gTruth;

% Convert groundTruth into a table
imageFilenames = gTruth.DataSource.Source;
dataTable = horzcat(table(imageFilenames), gTruth.LabelData);
[imageFilenames, bboxes, labels, params] = iParseTableInput(dataTable);

end

%--------------------------------------------------------------------------
function [imageFilenames, bboxes, labels, params] = iParseTableInput(varargin)
%   Parse varargin for constructor:
%       ___(trainingDataTable)
p = inputParser;
p.addRequired('dataTable', @istable);
p = iAddCommonParameters(p);
p.parse(varargin{:});
userInput = p.Results;

[imageFilenames, bboxes, labels] = iMergeBBoxAnnots(userInput.dataTable);

params = iCheckCommonParameters(userInput);

end


%--------------------------------------------------------------------------
function [imds, bboxes, params] = iParseImdsBboxesInput(varargin)
%   Parse varargin for constructor:
%       ___(imageDatastore, bboxes)

p = inputParser;
p.addRequired('imds');
p.addRequired('bboxes', @iscell);
p = iAddCommonParameters(p);
p.parse(varargin{:});
userInput = p.Results;

% validate the input bounding boxes
keepIndices = true(1, numel(userInput.bboxes));
for ii = 1:numel(userInput.bboxes)
    try
        iCheckBoundingBoxes(userInput.bboxes{ii});
    catch ME
        msgText = getReport(ME);
        % Warning('vision:cnn:bboxInvalid', 'Discarding image %d due to invalid bounding box. \n%s.\n', ii, msgText);
        keepIndices(ii) = false;
    end
end

imds = userInput.imds;
imds.Files(~keepIndices) = []; % prune invalid bbox images
bboxes = userInput.bboxes(keepIndices);

assert(isequal(numel(imds.Files), numel(bboxes)));

validateattributes(imds, {'matlab.io.datastore.ImageDatastore'},...
    {'scalar'}, mfilename, 'imds');

params = iCheckCommonParameters(userInput);

end


%--------------------------------------------------------------------------
function [imageFilenames, bboxes, params] = iParseImfnBboxesInput(varargin)
%   Parse varargin for constructor:
%       ___(imageFilenames, bboxes)
p = inputParser;
p.addRequired('imageFilenames', @iscell);
p.addRequired('bboxes', @iscell);
p = iAddCommonParameters(p);
p.parse(varargin{:});
userInput = p.Results;
assert(isequal(numel(userInput.imageFilenames), numel(userInput.bboxes)), ...
    'Number of imageFilenames and bboxes should match.');


% validate the input bounding boxes
keepIndices = true(1, numel(userInput.bboxes));
for ii = 1:numel(userInput.bboxes)
    try
        iCheckBoundingBoxes(userInput.bboxes{ii});
    catch ME
        msgText = getReport(ME, 'basic');
        % warning('vision:cnn:bboxInvalid', 'Discarding image %d due to invalid bounding box: \n %s', ii, msgText);
        keepIndices(ii) = false;
    end
end

imageFilenames = userInput.imageFilenames(keepIndices);
bboxes = userInput.bboxes(keepIndices);



params = iCheckCommonParameters(userInput);

end


%--------------------------------------------------------------------------
function classNames = iGetUniqueClassLabels(labels)
if ~isempty(labels)
    a = labels{:};
    classNames = unique(a);
else
    classNames = [];
end
end



% =========================================================================
%% Validate name-value pairs
%  ------------------------------------------------------------------------
%   - iAddCommonParameters()
%   - iCheckCommonParameters()
%   - iCheckColorPreprocessing()
%   - iCheckDataAugmentation()
%   - iCheckDataAugmentation()
%   - iCheckOutputSizeMode()
%   - iCheckBboxOverflowMode()
%   - iCheckOutputSize()
%   - iCheckImageResize()
% =========================================================================

%--------------------------------------------------------------------------
function p = iAddCommonParameters(p)
p.addParameter('DataAugmentation', 'none');
p.addParameter('ColorPreprocessing', 'none');
p.addParameter('ResizedShortestSide', []);
p.addParameter('OutputSize', []);
p.addParameter('OutputSizeMode', 'resize');
p.addParameter('BackgroundExecution', false);
p.addParameter('BboxOverflowMode', 'cut');
p.addParameter('ReshapeResponse', false);
end


%--------------------------------------------------------------------------
function params = iCheckCommonParameters(userInput)
dataAugmentation   = iCheckDataAugmentation(userInput.DataAugmentation);
colorPreprocessing = iCheckColorPreprocessing(userInput.ColorPreprocessing);
mode               = iCheckOutputSizeMode(userInput.OutputSizeMode);

vision.internal.inputValidation.validateLogical(...
    userInput.BackgroundExecution, 'BackgroundExecution');
iCheckImageResize(userInput.ResizedShortestSide);
iCheckOutputSize(userInput.OutputSize);

params.DataAugmentation    = dataAugmentation;
params.ColorPreprocessing  = char(colorPreprocessing);
params.OutputSizeMode      = char(mode);

if ~isempty(userInput.ResizedShortestSide)
    params.ResizedShortestSide = double(userInput.ResizedShortestSide);
else
    params.ResizedShortestSide = [];
end

if ~isempty(userInput.OutputSize)
    params.OutputSize = double(userInput.OutputSize(1:2));
else
    params.OutputSize = [];
end

params.BboxOverflowMode = iCheckBboxOverflowMode(userInput.BboxOverflowMode);
params.BackgroundExecution = logical(userInput.BackgroundExecution);
params.ReshapeResponse = logical(userInput.ReshapeResponse);

end


%--------------------------------------------------------------------------
function x = iCheckColorPreprocessing(x)
x = validatestring(x, {'none', 'rgb2gray', 'gray2rgb'}, mfilename, 'ColorPreprocessing');
end

%--------------------------------------------------------------------------
function aug = iCheckDataAugmentation(aug)

validateattributes(aug, {'char', 'string', 'vision.internal.cnn.bBoxDataAugmenter'},{},...
    mfilename, 'DataAugmentation');

if isa(aug, 'vision.internal.cnn.bBoxDataAugmenter')
    validateattributes(aug, {'vision.internal.cnn.bBoxDataAugmenter'},{'scalar'}, mfilename, 'DataAugmentation');
else
    aug = validatestring(aug, {'none'}, mfilename, 'DataAugmentation');
    aug = char(aug);
end
end


%--------------------------------------------------------------------------
function mode = iCheckOutputSizeMode(mode)
mode = validatestring(mode, {'resize', 'randcrop', 'centercrop'}, ...
    mfilename, 'OutputSizeMode');
end


%--------------------------------------------------------------------------
function mode = iCheckBboxOverflowMode(mode)
mode = validatestring(mode, {'cut', 'clip', 'centerclip'}, ...
    mfilename, 'BboxOverflowMode');
end


%--------------------------------------------------------------------------
function iCheckOutputSize(sz)
if ~isempty(sz)
    validateattributes(sz, {'numeric'}, ...
        {'row', 'positive', 'finite'}, mfilename, 'OutputSize');
end
end

%--------------------------------------------------------------------------
function iCheckImageResize(sz)
if ~isempty(sz)
    validateattributes(sz, {'numeric'}, ...
        {'row', 'positive', 'finite'}, mfilename, 'ResizedShortestSide');
end
end

%--------------------------------------------------------------------------
function iCheckCropSize(imageSize, outputSize)
% Check that the size of the crop fits inside the image
%   outputSize is two-element vector of row-size and column-size.
if outputSize(1) <= imageSize(1) && outputSize(2) <= imageSize(2)
    
else
    % ======================================================================
    error('vision:rcnn:OutputCropOverflow', 'OutputSize cannot be larger than the image dimensions for randcrop or centercrop mode.');
    % ======================================================================
end
end



% =========================================================================
%% Handle multi-class bboxes
% -------------------------------------------------------------------------
%   - iMergeBBoxAnnots()
%   - iAggregateBboxesAndLabels()
% =========================================================================

% -------------------------------------------------------------------------
function [imageFilenames, bboxes, labels] = iMergeBBoxAnnots(T)
%MERGEBBOXANNOTS Merge multi-category bounding-box annotations
%
%   The ground-truth labels may contain multi-category bounding boxes: e.g.
%   separate labels for 'vehicleFront', 'vehicleRear', 'vehicleRightSide',
%   'vehicleLeftSide', etc.
%
%   This helper function merges multi-category object labels into a single
%   category.
%
%   Input:
%   ------
%
%   groundTruthTable    Table in format for object detector training data.
%                       The first column corresponds to image filenames.
%                       Subsequent columns each correspond to an object
%                       category that is present in the dataset.
%
%                       | imageFiles | classA | classB | classC |...
%
%                       Each row of the table has the path to an image,
%                       stored under the first column. The subsequent
%                       columns store Mx4 bounding boxes for M objects of a
%                       specific category present in that image.
%
%   Outputs:
%   --------
%   imageFilenames      Cell array of image filenames.
%
%   bboxes              Cell array of bounding boxes, each cell
%                       corresponding to an image in 'imageFilenames'. For
%                       M objects in that image, the bounding box will be
%                       an Mx4 matrix of [x y width height].
%
%   labels              A cell array of cell arrays. Each element of the
%                       nested cell array corresponds to the class label of
%                       a bounding box.


bboxes = cell(height(T), 1);
labels = cell(height(T), 1);

imageFilenames = T{:,1}; % image filenames in first col of table
tableVarNames = T.Properties.VariableNames(2:end); % other cols are classes

for ii = 1:height(T)
    % for every row, aggregate the bboxes and/or labels
    bboxRow = T{ii, 2:end};
    [x, y] = iAggregateBboxesAndLabels(bboxRow, tableVarNames);
    
    
    % sanity checks
    assert(isequal(numel(y), size(x,1)));
    
    bboxes{ii} = x;
    labels{ii} = y;
    
    % bbox = iAggregateBboxes(bboxRow);
    % mergedBbox(ii).vehicle = bbox;
end

assert(~isempty(imageFilenames));
assert(~isempty(bboxes));

end

% -------------------------------------------------------------------------
function x = iAggregateBboxes(labeledBoxes)
%   (Relic from the single foreground class version of the code)
numBoxes = numel(labeledBoxes);
x = [];
for ii = 1:numBoxes
    bbox = labeledBoxes{ii};
    x = vertcat(x, bbox);
end
end

% -------------------------------------------------------------------------
function [x, y] = iAggregateBboxesAndLabels(labeledBoxes, tableVarNames)
%   Aggregate annotations in a single row of a table

% find the empty annotations - that class is not present in image
emptyIdx = cellfun(@isempty, labeledBoxes, 'un', 0);
emptyIdx = cell2mat(emptyIdx);

% merge non-empty bboxes
x = labeledBoxes(~emptyIdx);
x = vertcat(x{:});

boxcounts = cellfun(@(x)(size(x,1)), labeledBoxes, 'un', 0);
boxlabels = cellfun(@(a,b)(repmat({a},[1 b])), tableVarNames, boxcounts, 'un', 0);
y = cat(2, boxlabels{:});

% merge present object categories
% y = tableVarNames(~emptyIdx);
end



% =========================================================================
%% Bbox validation routines
%  ------------------------------------------------------------------------
%   - iValidateTransformedBoundingBoxes()
%   - iCheckBboxesOverflow()
%   - iCheckBoundingBoxes()
%   - iClipBboxesOverflow()
% =========================================================================

%--------------------------------------------------------------------------
function [bbox, removeRows] = iValidateTransformedBoundingBoxes(img, bbox, overflowMode)
%   Data augmentation may change the bounding box such that it is no longer
%   within the image margins. We modify these 'overflowing' bounding boxes,
%   either by removing (default behaviour) them or by clipping their
%   dimensions to fit within the image. Following this tidying up, the
%   pruned bounding box list is again checked for validation using the
%   usual iCheckBoundingBoxes().
%
%   Inputs:  'img' is the image, 'bbox' is an array of M x 4 bounding boxes [x y w h].
%   Output:  Returns a valid set of bounding boxes Y.

bbox = round(bbox);

switch overflowMode
    
    case 'cut'
        % Removes invalid (overflowing) bounding boxes
        %   if an object is partially cropped out, eliminate that bbox
        [ix, iy, iw, ih] = iCheckBboxesOverflow(img, bbox);
        removeRows = ix | iy | iw | ih ;
        
        if sum(removeRows) > 0
            warning('vision:cnn:bboxOverflow', 'Removed %d bounding boxes that were outside the image range.', ...
                sum(removeRows));
            bbox = bbox(~removeRows, :);
        end
        
    case 'clip'
        % Clip the borders of overflowing bounding boxes to image margins
        % TODO - vision.internal.detector.clipBBox
        bbox = vision.internal.detector.clipBBox(bbox, size(img));
        
        % Remove any pathological bboxes after this (if any)
        [ix, iy, iw, ih] = iCheckBboxesOverflow(img, bbox);
        removeRows = ix | iy | iw | ih ;
        bbox = bbox(~removeRows, :);
        
    case 'centerclip'
        % Check if center of bbox lies within the image
        overlapAreas = rectint(bbox, [1 1 size(img,2) size(img,1)]);
        bboxAreas = bbox(:,3) .* bbox(:,4);
        overlapRatio = overlapAreas ./ bboxAreas ;
        
        % remove bboxes whose centers are outside image
        %   i.e. their overlap ratio will be less than 0.5
        removeRows = (overlapRatio < 0.5) ;
        bbox = bbox(~removeRows, :);
        
        % clip the rest to image margins
        bbox = vision.internal.detector.clipBBox(bbox, size(img));
        
        if sum(removeRows) > 0
            warning('vision:cnn:bboxOverflow', 'Removed %d bounding boxes that were outside the image range.', ...
                sum(removeRows));
            bbox = bbox(~removeRows, :);
        end
        
        % Remove any pathological bboxes after this (if any)
        [ix, iy, iw, ih] = iCheckBboxesOverflow(img, bbox);
        removeRows = ix | iy | iw | ih ;
        bbox = bbox(~removeRows, :);
        
    otherwise
        error('Invalid value given for the property BboxOverflowMode of boundingBoxImageDatasource class.');
end


% validate the remaining bounding boxes
if ~isempty(bbox)
    iCheckBoundingBoxes(bbox);
else
    % warning('vision:cnn:noBboxInImage', 'No valid bounding box left after data augmentation.');
end

end


%--------------------------------------------------------------------------
function [ix, iy, iw, ih] = iCheckBboxesOverflow(img, bbox)
ix = (bbox(:,1) < 1) | (bbox(:,1) > size(img,2)) ;
iy = (bbox(:,2) < 1) | (bbox(:,2) > size(img,1)) ;
iw = (bbox(:,3) < 1) | (bbox(:,3) + bbox(:,1) - 1 > size(img,2)) ;
ih = (bbox(:,4) < 1) | (bbox(:,4) + bbox(:,2) - 1 > size(img,1)) ;
end


%------------------------------------------------------------------
function iCheckBoundingBoxes(bboxes)
if ~ismatrix(bboxes) || size(bboxes, 2) ~= 4
    error(message('vision:rcnn:invalidBBoxDim'));
end

if ~isreal(bboxes) || issparse(bboxes) || ~all(isfinite(bboxes(:)))
    error(message('vision:rcnn:invalidBBox'));
end

if any(bboxes(:,3) <= 0) || any(bboxes(:,4) <= 0)
    error(message('vision:rcnn:proposalInvalidWidthHeight'));
end

end


%--------------------------------------------------------------------------
function [X, Y] = iClipBboxesOverflow(X, Y)

% Case 1: Handle "underflow"

% calculate underflow amount
xUnderflow = (Y(:,1) < 1) .* abs(Y(:,1)) ;
yUnderflow = (Y(:,2) < 1) .* abs(Y(:,2)) ;

% clip (x,y) to top-left image borders
Y(Y(:,1) < 1, 1) = 1;
Y(Y(:,2) < 1, 2) = 1;

% trim (w,h) by the (x,y) underflows
Y(:,3) = Y(:,3) - xUnderflow;
Y(:,4) = Y(:,4) - yUnderflow;



% Case 2: Handle "overflow"

% calculate the amount of overflow
%   > 0 , there is an overflow
%   <=0, there is no overflow
wOverflow = Y(:,3)+Y(:,1) - size(X,2) ;
hOverflow = Y(:,4)+Y(:,2) - size(X,1) ;

% For those rows with overflow, subtract the overflowing amount
Y(wOverflow>0, 3) = Y(wOverflow>0, 3) - wOverflow(wOverflow>0);
Y(hOverflow>0, 4) =  Y(hOverflow>0, 4) - hOverflow(hOverflow>0);
end



% =========================================================================
%% Ground-truth validation routines
%  ------------------------------------------------------------------------
%   - [TODO] iCheckGroundTruth()
% =========================================================================

%--------------------------------------------------------------------------
function iCheckGroundTruth(g)
validateattributes(g, {'groundTruth'}, {'vector','nonempty'}, ...
    mfilename, 'gTruth');

% names = cell(numel(g),1);

%% TODO

% for i = 1:numel(g)
%
%     iAssertAllGroundTruthHasPixelLabelType(g(i));
%
%     iAssertAllBoundingBoxDataIsNotMissing(g(i));
%
%     names{i} = iGatherLabelNamesOfPixelLabelType(g(i));
% end

% array of groundTruth should have same pixel label classes. order does not
% matter.
% iAssertAllGroundTruthHasSameLabelNames(names);
end



% =========================================================================
%% Colorspace conversion routines
%  ------------------------------------------------------------------------
%   - iConvertAnyRGBToGray()
%   - iConvertAnyGrayToRGB()
%   - iMakeVertical()
% =========================================================================

%--------------------------------------------------------------------------
function img = iConvertAnyRGBToGray(img)
if iscell(img)
    for i = 1:numel(img)
        if ~ismatrix(img{i})
            img = rgb2gray(img);
        end
    end
    img = cellfun(@(x)rgb2gray(x), img, 'UniformOutput', false);
else
    if ~ismatrix(img)
        img = rgb2gray(img);
    end
end
end

%--------------------------------------------------------------------------
function img = iConvertAnyGrayToRGB(img)
if iscell(img)
    for i = 1:numel(img)
        if ismatrix(img{i})
            img{i} = repelem(img{i},1,1,3);
        end
    end
else
    if ismatrix(img)
        img = repelem(img,1,1,3);
    end
end
end

%--------------------------------------------------------------------------
function vec = iMakeVertical( vec )
vec = reshape( vec, numel( vec ), 1 );
end


% =========================================================================
%% Internal resizing & cropping routines
%  ------------------------------------------------------------------------
%   - iRandCrop()
%   - iCenterCrop()
%   - iCenterCropBboxImage()
%   - iRandCropBboxImage()
%   - iResize()
%   - iResizeBoundingBox()
%   - iResizeSmallestSide()
% =========================================================================

%--------------------------------------------------------------------------
function [img, bbox, labels] = iRandCrop(img, bbox, labels, outputSize, BboxOverflowMode)
if iscell(img)
    [img, bbox, labels] = cellfun(@(x,y,z)iRandCropBboxImage(x, y, z,...
        outputSize, BboxOverflowMode), ...
        img, bbox, labels, 'UniformOutput', false);
else
    [img, bbox, labels] = iRandCropBboxImage(img, bbox, labels, outputSize, BboxOverflowMode);
end
end

%--------------------------------------------------------------------------
function [img, bbox, labels] = iCenterCrop(img, bbox, labels, outputSize, BboxOverflowMode)
if iscell(img)
    [img, bbox, labels] = cellfun(@(x,y)iCenterCropBboxImage(x, y, z,...
        outputSize, BboxOverflowMode), ...
        img, bbox, labels, 'UniformOutput', false);
    
else
    [img, bbox, labels] = iCenterCropBboxImage(img,bbox,labels,outputSize,BboxOverflowMode);
end
end

%--------------------------------------------------------------------------
function [img, bbox, labels] = iCenterCropBboxImage(img, bbox, labels, outputSize, BboxOverflowMode)
% calculate the center crop w.r.t. original image coords
xShift = (size(img,2) - outputSize(2)) / 2;
yShift = (size(img,1) - outputSize(1)) / 2;

% get center cropped image - origin has shifted!
img = augmentedImageDatastore.centerCrop(img,outputSize);

% shift bbox according to new origin
shiftedY = bbox;
shiftedY(:,1) = shiftedY(:,1) - xShift;
shiftedY(:,2) = shiftedY(:,2) - yShift;

[bbox, discardedRows] = iValidateTransformedBoundingBoxes(img, shiftedY, BboxOverflowMode);
if ~isempty(labels)
    labels = labels(~discardedRows);
end
end

%--------------------------------------------------------------------------
function [img, bbox, labels] = iRandCropBboxImage(img, bbox, labels, outputSize, BboxOverflowMode)
% random rectangle crop of image
rect = round(augmentedImageDatastore.randCropRect(img,outputSize)); % randrect has fractional values
img = augmentedImageDatastore.crop(img, rect);

% shift bbox according to new origin
shiftedY = bbox;
shiftedY(:,1:2) = bsxfun(@minus, shiftedY(:,1:2), rect(1:2));

[bbox, discardedRows] = iValidateTransformedBoundingBoxes(img, shiftedY, BboxOverflowMode);
if ~isempty(labels)
    labels = labels(~discardedRows);
end
end

%--------------------------------------------------------------------------
function img = iResize(img, outputSize)
if iscell(img)
    img = cellfun(@(x)augmentedImageDatastore.resizeImage(x, outputSize), img, 'UniformOutput', false);
else
    img = augmentedImageDatastore.resizeImage(img, outputSize);
end
end


%--------------------------------------------------------------------------
function bbox = iResizeBoundingBox(bbox, imageSize, outputSize)
%   outputSize is two-element vector of row-size and column-size.

if isempty(bbox)
    return
end

scaleFactor = outputSize ./ imageSize ;

% scale x and w by column-size
bbox(:,1) = round(bbox(:,1) * scaleFactor(2));
bbox(:,3) = round(bbox(:,3) * scaleFactor(2));

% scale y and h by row-size
bbox(:,2) = round(bbox(:,2) * scaleFactor(1));
bbox(:,4) = round(bbox(:,4) * scaleFactor(1));

end


%--------------------------------------------------------------------------
function [img,bbox] = iResizeSmallestSide(img, bbox, resizedValue)
% iResizeSmallestSide   Resize the image such that its shortest side equals
% the `resizedValue` argument, maintaining aspect ratio. The bboxes are
% also scaled in correspondence.

% get size of the input image
sizeX = size(img);
sizeX = sizeX(1:2);

scaleFactor = resizedValue / min(sizeX) ;
img = imresize(img, scaleFactor);
if ~isempty(bbox)
    bbox = bbox * scaleFactor ;
end
end

%--------------------------------------------------------------------------
function bboxes = iReshapeSingleObjectBboxes( bboxes )
% iReshapeSingleObjectBboxes   Reshape list of single bboxes into a format
% accepted by trainNetwork(), i.e. reshapes [4 1] arrays to [1 1 4] arrays.
%   Note: this works only when there is ONE bbox per image (M=1).
if iscell(bboxes)
    % for a cell array of bboxes (batchSize > 1)
    for ii = 1:length(bboxes)
        assert(isequal(size(bboxes{ii}), [1 4]), ...
            'Reshapes correctly only when there is just one bounding box per image.');
        bboxes{ii} = reshape(bboxes{ii}, [1 1 4]);
    end
else
    % for only one bbox
    assert(isequal(size(bboxes), [1 4]), ...
        'Reshapes correctly only when there is just one bounding box per image.');
    bboxes = reshape(bboxes, [1 1 4]);
end

end

%--------------------------------------------------------------------------
function data = iMakeTable(images,bboxes,labels)

if ~iscell(labels)
    labels = {labels};
end

if ~iscell(bboxes)
    bboxes = {bboxes};
end

if ~iscell(images)
    images = {images};
end

% TODO format table correctly
data = table(images,bboxes,labels);
end
