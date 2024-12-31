%PixelLabelImageDatastore Datastore for semantic segmentation networks. (internal use only)
%
%   The PixelLabelImageDatastore is a superclass of the
%   pixelLabelImageDatastore which instantiates itself by calling the
%   superclass constructor. This is for internal use only.
%
%   ds = PixelLabelImageDatastore(gTruth) creates a datastore
%   for training a semantic segmentation network using deep learning. The
%   input gTruth is an array of groundTruth objects. The output is a
%   PixelLabelImageDatastore object. Use this output with trainNetwork
%   from Deep Learning Toolbox to train convolutional neural networks for
%   semantic segmentation.
%
%   ds = PixelLabelImageDatastore(imds, pxds) creates a data
%   source for training a semantic segmentation network using deep
%   learning. The input imds is an imageDatastore that specifies the ground
%   truth images. The input pxds is a pixelLabelDatastore that specifies
%   the pixel label data. imds represents the training input to the network
%   and pxds represents the desired network output.
%
%   [...] = PixelLabelImageDatastore(..., Name, Value) specifies
%   additional name-value pair arguments described below:
%
%      'DataAugmentation'      Specify image data augmentation using an
%                              imageDataAugmenter object or 'none'.
%                              Training data is augmented in real-time
%                              while training.
%
%                              Default: 'none'
%
%      'ColorPreprocessing'    A scalar string or character vector
%                              specifying color channel pre-processing.
%                              This option can be used when you have a
%                              training set that contains both color and
%                              grayscale image data and you need data
%                              created by the datastore to be strictly
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
%      'OutputSize'            A two element vector specifying the number
%                              of rows and columns in images produced by
%                              the datastore. When specified, image sizes
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
%                              Valid values are 'resize', 'centercrop' and
%                              'randcrop'. This option only applies when
%                              'OutputSize' is not empty.
%
%                              Default: 'resize'
%
%      'DispatchInBackground'  Accelerate image augmentation by
%                              asynchronously reading, augmenting, and
%                              queueing augmented images for use in
%                              training. Requires Parallel Computing
%                              Toolbox.
%
%                              Default: false
%
%   PixelLabelImageDatastore Properties:
%
%      Images               - A list of image filenames.
%      PixelLabelData       - A list of pixel label data filenames.
%      ClassNames           - A cell array of pixel label class names.
%      DataAugmentation     - Selected data augmentations.
%      ColorPreprocessing   - Selected color channel processing.
%      OutputSize           - Output size of output images.
%      OutputSizeMode       - Output size adjustment mode.
%      DispatchInBackground - Whether background dispatch is used.
%      MiniBatchSize        - Number of images returned in each read.
%      NumObservations      - Total number of images in an epoch.
%
%   PixelLabelImageDatastore Methods:
%
%      countEachLabel       - Counts the number of pixel labels for each class.
%      hasdata              - Returns true if there is more data in the datastore
%      partitionByIndex     - Partitions datastore given indices
%      preview              - Reads the first image from the datastore
%      read                 - Reads data from the datastore
%      readall              - Reads all observations from the datastore
%      readByIndex          - Random access read from datastore given indices
%      reset                - Resets datastore to the start of the data
%      shuffle              - Shuffles the observations in the datastore
%

% Copyright 2017-2021 The MathWorks, Inc.

classdef PixelLabelImageDatastore < ...
        matlab.io.Datastore &...
        matlab.io.datastore.MiniBatchable &...
        matlab.io.datastore.Shuffleable &...
        matlab.io.datastore.BackgroundDispatchable &...
        matlab.io.datastore.PartitionableByIndex

    properties(Dependent, SetAccess = private)
        %Images Source of ground truth images.
        Images
        
        %PixelLabelData Source of ground truth label images.
        %Pixel labels are stored as label matrices in uint8 images.
        PixelLabelData
    end
    
    properties(Dependent, SetAccess = private)
        %ClassNames A cell array of class names.
        ClassNames
    end
    
    properties(SetAccess = private)
        %DataAugmentation
        %Specify image data augmentation using an imageDataAugmenter object
        %or 'none'. Training data is augmented in real-time while training.
        DataAugmentation
        
        %ColorPreprocessing
        %A scalar string or character vector specifying color channel
        %pre-processing. This option can be used when you have a training
        %set that contains both color and grayscale image data and you need
        %data created by the datastore to be strictly color or grayscale.
        %Options are: 'gray2rgb','rgb2gray','none'. For example, if you
        %need to train a  network that expects color images but some of the
        %images in your training set are grayscale, then specifying the
        %option 'gray2rgb' will replicate the color channels of the
        %grayscale images in the input image set to create MxNx3 output
        %images. The default is 'none'.
        ColorPreprocessing
        
        %OutputSize
        %A two element vector specifying the number of rows and columns in
        %images produced by the datastore. When specified, image sizes are
        %adjusted as necessary to achieve output of images of the specified
        %size. By default, 'OutputSize' is empty and the output size is not
        %adjusted.
        OutputSize
        
        %OutputSizeMode
        % A scalar string or character vector specifying the technique used
        % to adjust image sizes to the specified 'OutputSize'. Valid values
        % are 'resize', 'centercrop' and 'randcrop'. This option only
        % applies when 'OutputSize' is not empty. The default is 'resize'.
        OutputSizeMode
        
    end
    
    properties(Dependent)
        %MiniBatchSize - MiniBatch Size
        %
        %   The number of observations returned as rows in the table
        %   returned by the read method.
        MiniBatchSize
                
    end
    
    properties(Hidden, Dependent)
        
        %BackgroundExecution
        %  The BackgroundExecution property is not recommended. Use
        %  DispatchInBackground instead.
        BackgroundExecution
        
    end

    properties(Dependent,SetAccess=protected)
        %NumObservations - Number of observations
        %
        %   The number of observations in the datastore. 
        NumObservations
    end
    
    properties(Hidden, SetAccess = private)
        ImageDatastore
        PixelLabelDatastore
    end
    
    methods
        
        function this = PixelLabelImageDatastore(varargin)
            narginchk(1,inf);
            if isstruct(varargin{1}) 
                % Struct input used by copyElement.
                % N.B. copy(ds) should not reset the datastore.
                params = varargin{1};
                this.ImageDatastore         = params.ImageDatastore;
                this.PixelLabelDatastore    = params.PixelLabelDatastore;
                this.DataAugmentation       = params.DataAugmentation;
                this.ColorPreprocessing     = params.ColorPreprocessing;
                this.DataAugmentation       = params.DataAugmentation;
                this.OutputSize             = params.OutputSize;
                this.OutputSizeMode         = params.OutputSizeMode;
                this.DispatchInBackground   = params.DispatchInBackground;
            else 
                if isa(varargin{1}, 'groundTruth')
                    
                    [gTruth, params] = iParseGroundTruthInput(varargin{:});
                    
                    % Create internal datastores
                    opts.IncludeSubfolders = false;
                    opts.ReadSize = 1;
                    opts.FileExtensions = string(iDefaultFileExtensions);
                    opts.ReadFcn = iGetDefaultReadFcn;
                    [this.PixelLabelDatastore, this.ImageDatastore] = ...
                        matlab.io.datastore.PixelLabelDatastore.createFromGroundTruth(gTruth, opts);
                    
                else
                    
                    [imds, pxds, params] = iParseImdsPxdsInput(varargin{:});
                    
                    % Make copy of user datastores.
                    this.ImageDatastore      = copy(imds);
                    this.PixelLabelDatastore = copy(pxds);
                    
                    if numel(this.ImageDatastore.Files) ~= numel(this.PixelLabelDatastore.Files)
                        error(message('vision:semanticseg:unequalNumelFiles'));
                    end
                    
                end
                
                this.ColorPreprocessing     = params.ColorPreprocessing;
                this.DataAugmentation       = params.DataAugmentation;
                this.OutputSize             = params.OutputSize;
                this.OutputSizeMode         = params.OutputSizeMode;
                this.DispatchInBackground   = params.DispatchInBackground;
                
                %Augmentation for 3D inputs is not supported.
                if ~strcmp(this.ColorPreprocessing,'none') || ~strcmp(this.DataAugmentation,'none') || ~isempty(this.OutputSize)
                    out = preview(this.PixelLabelDatastore);
                    if ndims(out) == 3
                        error(message('vision:semanticseg:aug3dNotSupported'));
                    end
                end
                
                this.reset();
            end
        
        end
        
        %------------------------------------------------------------------
        function batchSize = get.MiniBatchSize(this)
            batchSize = this.ImageDatastore.ReadSize;
        end
        
        %------------------------------------------------------------------
        function set.MiniBatchSize(this,batchSize)
            this.ImageDatastore.ReadSize = batchSize;
            this.PixelLabelDatastore.ReadSize = batchSize;
        end
        
        %------------------------------------------------------------------
        function val = get.ClassNames(this)
            val = this.PixelLabelDatastore.ClassNames;
        end
        
        %------------------------------------------------------------------
        function val = get.NumObservations(this)
            val = this.ImageDatastore.numpartitions;
        end
        
        %------------------------------------------------------------------
        function TF = get.BackgroundExecution(this)
           TF = this.DispatchInBackground;
        end
                
        %------------------------------------------------------------------
        function s = saveobj(this)
            s.Version = 2.0;
            s.imds = this.ImageDatastore;
            s.pxds = this.PixelLabelDatastore;
            s.ColorPreprocessing = this.ColorPreprocessing;
            s.DataAugmentation = this.DataAugmentation;
            s.DispatchInBackground = this.DispatchInBackground;
            s.OutputSize = this.OutputSize;
            s.OutputSizeMode = this.OutputSizeMode;
        end
        
    end
    
    methods
        
        %------------------------------------------------------------------
        function [data,info] = readByIndex(this,indices)
            % Create datastore partition via a copy and index. This is
            % faster than constructing a new datastore with the new
            % files.
                
            [inputImage, inputImageFileNames, pixelLabelImage, labeledImageInfo] = readImagesAndNumericLabelsByIndex(this, indices);
            [inputImage, pixelLabelImage] = preprocess(this, inputImage, pixelLabelImage, labeledImageInfo);
            
            % Convert preprocessed pixel label data back to categorical. 
            pixelLabelImage = label2categorical(this.PixelLabelDatastore, pixelLabelImage, labeledImageInfo);
            
            [data, info] = iOutputDataAndInfo(inputImage, inputImageFileNames, pixelLabelImage, labeledImageInfo);
        end
        
        %------------------------------------------------------------------
        function [data,info] = read(this)
            
            if ~hasdata(this)
                error(message('vision:semanticseg:outOfData'));
            end
            
            % Read image and pixel label data. Return pixel data as numeric
            % to avoid double converting from numeric to/from categorical.
            [inputImage,infoX] = read(this.ImageDatastore);           
            [pixelLabelImage, labeledImageInfo] = readNumeric(this.PixelLabelDatastore);
            [inputImage, pixelLabelImage] = preprocess(this, inputImage, pixelLabelImage, labeledImageInfo);
            
            % Convert preprocessed pixel label data back to categorical.
            pixelLabelImage = label2categorical(this.PixelLabelDatastore, pixelLabelImage, labeledImageInfo);

            inputImageFileNames = infoX.Filename;
            [data, info] = iOutputDataAndInfo(inputImage, inputImageFileNames, pixelLabelImage, labeledImageInfo);
        end
        
        %------------------------------------------------------------------
        function reset(this)
            reset(this.ImageDatastore);
            reset(this.PixelLabelDatastore);
        end
        
        %------------------------------------------------------------------
        function newds = shuffle(this)
            ord = randperm( numel(this.Images) );
            newds = this.partitionByIndex(ord);            
        end
        
        function TF = hasdata(this)
            TF = hasdata(this.ImageDatastore);
        end

        function newds = partitionByIndex(this,indices)
            newds = copy(this);
            newds.reset(); % copy does not reset().
            newds.ImageDatastore = subset(this.ImageDatastore,indices);
            newds.PixelLabelDatastore = subset(this.PixelLabelDatastore,indices);            
        end
    end
    
    methods
        %------------------------------------------------------------------
        function src = get.Images(this)
            src = this.ImageDatastore.Files;
        end
        
        %------------------------------------------------------------------
        function src = get.PixelLabelData(this)
            src = this.PixelLabelDatastore.Files;
        end
        
        %------------------------------------------------------------------
        function tbl = countEachLabel(this)
            % tbl = countEachLabel(ds) counts the occurrence of
            % each pixel label for all images represented by the
            % datastore. The output tbl is a table with the following
            % variables names:
            %
            %   Name            - The pixel label class name.
            %
            %   PixelCount      - The number of pixels of a given class.
            %
            %   ImagePixelCount - The total number of pixels in images that
            %                     had an instance of the given class.
            %
            % Class Balancing
            % ---------------
            % The output of countEachLabel can be used to calculate class
            % weights for class balancing, for example:
            %
            %   * Uniform class balancing weights each class such that each
            %     has a uniform prior probability:
            %
            %        numClasses = height(tbl)
            %        prior = 1/numClasses;
            %        classWeights = prior./tbl.PixelCount
            %
            %   * Inverse frequency balancing weights each class such that
            %     underrepresented classes are given higher weight:
            %
            %        totalNumberOfPixels = sum(tbl.PixelCount)
            %        frequency = tbl.PixelCount / totalNumberOfPixels;
            %        classWeights = 1./frequency
            %
            %   * Median frequency balancing weights each class using the
            %     median frequency. The weight for each class c is defined
            %     as median(imageFreq)/imageFreq(c) where imageFreq(c) is
            %     the number of pixels of a given class divided by the
            %     total number of pixels in images that had a instance of
            %     the given class c.
            %
            %        imageFreq = tbl.PixelCount ./ tbl.ImagePixelCount
            %        classWeights = median(imageFreq) ./ imageFreq
            %
            % The calculated class weights can be passed to the
            % pixelClassificationLayer. See example below.
            %
            % Example
            % --------
            % % Setup of data location.
            % dataDir = fullfile(toolboxdir('vision'), 'visiondata');
            % imDir = fullfile(dataDir, 'building');
            % pxDir = fullfile(dataDir, 'buildingPixelLabels');
            %
            % % Create datastore using ground truth and pixel labeled images.
            % imds = imageDatastore(imDir);
            % classNames = ["sky" "grass" "building" "sidewalk"];
            % pixelLabelID = [1 2 3 4];
            % pxds = pixelLabelDatastore(pxDir, classNames, pixelLabelID);
            % src = pixelLabelImageDatastore(imds, pxds);
            %
            % % Tabulate pixel label counts in dataset.
            % tbl = countEachLabel(src)
            %
            % % Class balancing using uniform prior weighting.
            % prior = 1/numel(classNames);
            % uniformClassWeights = prior./tbl.PixelCount
            %
            % % Class balancing using inverse frequency weighting.
            % totalNumberOfPixels = sum(tbl.PixelCount);
            % frequency = tbl.PixelCount / totalNumberOfPixels;
            % invFreqClassWeights = 1./frequency
            %
            % % Class balancing using median frequency weighting.
            % freq = tbl.PixelCount ./ tbl.ImagePixelCount
            % medFreqClassWeights = median(freq) ./ freq
            %
            % % Pass the class weights to the pixel classification layer.
            % layer = pixelClassificationLayer('ClassNames', tbl.Name, ...
            %     'ClassWeights', medFreqClassWeights)
            %
            % See also pixelClassificationLayer, pixelLabelDatastore,
            %          imageDatastore.
            
            tbl = this.PixelLabelDatastore.countEachLabel();
        end
        
    end
        
    methods(Hidden)
     
        %------------------------------------------------------------------
        function frac = progress(this)
            frac = progress(this.ImageDatastore);
        end
        
        %------------------------------------------------------------------
        function [data,info] = readNumericByIndex(this,indices)
            % Create datastore partition via a copy and index. This is
            % faster than constructing a new datastore with the new
            % files.
                 
            [inputImage, inputImageFileNames, pixelLabelImage, labeledImageInfo] = readImagesAndNumericLabelsByIndex(this, indices);
            [inputImage, pixelLabelImage] = preprocess(this, inputImage, pixelLabelImage, labeledImageInfo);
            
            % Set 
            [data, info] = iOutputDataAndInfo(inputImage, inputImageFileNames, pixelLabelImage, labeledImageInfo, true);
        end
        
    end
    
    
    methods(Access = protected)
        %------------------------------------------------------------------
        function aCopy = copyElement(this)
            % Return a deep copy. Copy preserves the state. 
            s = this.copyElementImpl();
            aCopy = vision.internal.PixelLabelImageDatastore(s);
        end
        
        %------------------------------------------------------------------
        function s = copyElementImpl(this)
            % Make deep copy of underlying datastores.
            s.ImageDatastore = copy(this.ImageDatastore);
            s.PixelLabelDatastore = copy(this.PixelLabelDatastore);
            
            if isa(this.DataAugmentation,'mageDataAugmenter')
                % copy augmenter, which is a handle class.
                s.DataAugmentation = copy(this.DataAugmentation);
            else
                s.DataAugmentation = this.DataAugmentation;
            end
            
            s.ColorPreprocessing     = this.ColorPreprocessing;
            s.OutputSize             = this.OutputSize;
            s.OutputSizeMode         = this.OutputSizeMode;
            s.DispatchInBackground   = this.DispatchInBackground;
        end
    end
    
    
    methods(Hidden, Static)
        %------------------------------------------------------------------
        function this = loadobj(s)
            this = pixelLabelImageDatastore(s.imds, s.pxds);
            this.ColorPreprocessing = s.ColorPreprocessing;
            this.DataAugmentation = s.DataAugmentation;
            
            if s.Version > 1.0
                this.OutputSize = s.OutputSize;
                this.OutputSizeMode = s.OutputSizeMode;
            end
            
            if isfield(s,'DispatchInBackground')
                this.DispatchInBackground = s.DispatchInBackground;
            end
        end
        
        %------------------------------------------------------------------
        function A = onehotencode(C)
            
            numCategories = numel(categories(C));
            [H, W, ~, numObservations] = size(C);
            dummifiedSize = [H, W, numCategories, numObservations];
            A = zeros(dummifiedSize, 'single');
            C = iMakeVertical( C );
            
            [X,Y,Z] = meshgrid(1:W, 1:H, 1:numObservations);
            
            X = iMakeVertical(X);
            Y = iMakeVertical(Y);
            Z = iMakeVertical(Z);
            
            % Remove missing labels. These are pixels we should ignore during
            % training. The dummified output is all zeros along the 3rd dims and are
            % ignored during the loss computation.
            [C, removed] = rmmissing(C);
            X(removed) = [];
            Y(removed) = [];
            Z(removed) = [];
            
            idx = sub2ind(dummifiedSize, Y(:), X(:), int32(C), Z(:));
            A(idx) = 1;
        end
        
        %------------------------------------------------------------------
        function [X,Y] = augment(augmenter,X,Y,fillValue)
            % Convert pixel label data to single for augmentation. This
            % allows using NaN fill values which are converted to
            % undefined categoricals and excluded from
            % training/inference.
            if iscell(Y)
                Y = cellfun(@(y)single(y),Y,'UniformOutput', false);
            else
                Y = single(Y);
            end
            % Use nearest interpolation for pixel label data.
            interpY = 'nearest';
            
            % Ensure all fillValues are all the same.
            assert(all(fillValue==fillValue(1)), "fillValue needs to be" + ...
                " a vector of all equal entries.");
            
            % Convert fillValue to scalar as per imageAugmenter 
            % specifications.
            fillValue = fillValue(1);
            [X,Y] = augmenter.augmentPair(X,Y,interpY,fillValue);
       end
    end
    
    methods(Hidden, Access = private)
        %------------------------------------------------------------------
        function [X, Y] = augmentData(this, X, Y, fillval)
            if ~strcmp(this.DataAugmentation,'none')
                [X,Y] = vision.internal.PixelLabelImageDatastore.augment(this.DataAugmentation,X,Y,fillval);
            end
        end
        
        %------------------------------------------------------------------
        function [X, Y] = preprocess(this, X, Y, info)
            % Apply color preprocessing.
            switch this.ColorPreprocessing
                case 'none'
                case 'rgb2gray'
                    X = iConvertAnyRGBToGray(X);
                case 'gray2rgb'
                    X = iConvertAnyGrayToRGB(X);
            end
            
            if isa(this.DataAugmentation,'imageDataAugmenter') || ...
                    (~isempty(this.OutputSize) && string(this.OutputSizeMode).contains('crop'))
                
                % Data augmentation and output cropping require that both X
                % and Y have the same [H W].
                sizesX = iSizesInBatch(X);
                sizesY = iSizesInBatch(Y);
                
                if ~isequal(sizesX, sizesY)
                    error(message('vision:semanticseg:augOutSizeNotSupported'))
                end
            end
            
            % Apply data augmentation. If there is augmentation, output Y is
            % single, else it is categorical.
            [X, Y] = augmentData(this, X, Y, info.FillValue);
            
            % Apply output size selection.
            if ~isempty(this.OutputSize)
                
                switch this.OutputSizeMode
                    case 'resize'
                        Y = iResizeLabelMatrix(Y, this.OutputSize);
                        X = iResize(X, this.OutputSize);
                    case 'randcrop'
                        [X, Y] = iRandCrop(X, Y, this.OutputSize);
                    case 'centercrop'
                        [X, Y] = iCenterCrop(X, Y, this.OutputSize);
                end
                
            end
            
        end
        
        %------------------------------------------------------------------
        function [inputImage, inputImageFileNames, pixelLabelImage, labeledImageInfo] = readImagesAndNumericLabelsByIndex(this, indices)
            % Read the image and pixel-label image as numeric values. Also,
            % return the list of input image filenames and labeled image
            % info struct
            subds = copy(this.ImageDatastore);
            subds.Files = this.ImageDatastore.Files(indices);
            inputImageFileNames = subds.Files;
            inputImage = readall(subds);
            [pixelLabelImage, labeledImageInfo] = readNumeric(this.PixelLabelDatastore, indices);
        end
        
    end
end

%------------------------------------------------------------------

function [data, info] = iOutputDataAndInfo(inputImage, inputImageFileNames, pixelLabelImage, labeledImageInfo, varargin)
% Prepare the image and pixel-label image data along with the corresponding
% info struct

if isempty(varargin)
    isLabeledImageInfoEmitted = false;
else
    isLabeledImageInfoEmitted = varargin{1};
end

if ~iscell(inputImage)
    inputImage = {inputImage};
end

if ~iscell(pixelLabelImage)
    pixelLabelImage = {pixelLabelImage};
end

if ~iscell(labeledImageInfo.Filename)
    labeledImageInfo.Filename = {labeledImageInfo.Filename};
end

data = table(inputImage,pixelLabelImage);

info.ImageFilename = inputImageFileNames;
info.PixelLabelFilename = labeledImageInfo.Filename;

% Output the complete LabeledImageInfo struct only for the hidden method
% readNumericByIndex used by some datastores such as
% randomPatchExtractionDatastore for data augmentation (requires the
% FillValues field). For other methods, this flag should be set to false.
if isLabeledImageInfoEmitted
    info.LabeledImageInfo = labeledImageInfo;
end

end

%--------------------------------------------------------------------------
function sizes = iSizesInBatch(batch)
if iscell(batch)
    sizes = cellfun(@(x)iHeightWidth(x),batch,'UniformOutput',false);
else
    sizes = {iHeightWidth(batch)};
end
end

%--------------------------------------------------------------------------
function hw = iHeightWidth(x)
sz = size(x);
hw = sz(1:2);
end

%--------------------------------------------------------------------------
function x = iCheckColorPreprocessing(x)
x = validatestring(x, {'none', 'rgb2gray', 'gray2rgb'}, mfilename, 'ColorPreprocessing');
end

%--------------------------------------------------------------------------
function aug = iCheckDataAugmentation(aug)
validateattributes(aug, {'char', 'string', 'imageDataAugmenter'},{},...
    mfilename, 'DataAugmentation');

if isa(aug, 'imageDataAugmenter')
    validateattributes(aug, {'imageDataAugmenter'},{'scalar'}, mfilename, 'DataAugmentation');
else
    aug = validatestring(aug, {'none'}, mfilename, 'DataAugmentation');
    aug = char(aug);
end
end

%--------------------------------------------------------------------------
function p = iAddCommonParameters(p)
p.addParameter('DataAugmentation', 'none');
p.addParameter('ColorPreprocessing', 'none');
p.addParameter('OutputSize', []);
p.addParameter('OutputSizeMode', 'resize');
p.addParameter('BackgroundExecution', false);
p.addParameter('DispatchInBackground', false);
end

%--------------------------------------------------------------------------
function params = iCheckCommonParameters(userInput)
dataAugmentation   = iCheckDataAugmentation(userInput.DataAugmentation);
colorPreprocessing = iCheckColorPreprocessing(userInput.ColorPreprocessing);
mode               = iCheckOutputSizeMode(userInput.OutputSizeMode);

vision.internal.inputValidation.validateLogical(...
    userInput.DispatchInBackground, 'DispatchInBackground');

iCheckOutputSize(userInput.OutputSize);

params.DataAugmentation    = dataAugmentation;
params.ColorPreprocessing  = char(colorPreprocessing);
params.OutputSizeMode      = char(mode);
params.DispatchInBackground = userInput.DispatchInBackground;
if ~isempty(userInput.OutputSize)
    params.OutputSize = double(userInput.OutputSize(1:2));
else
    params.OutputSize = [];
end

params.BackgroundExecution = logical(userInput.BackgroundExecution);
end

%--------------------------------------------------------------------------
function mode = iCheckOutputSizeMode(mode)
mode = validatestring(mode, {'resize', 'randcrop', 'centercrop'}, ...
    mfilename, 'OutputSizeMode');
end

%--------------------------------------------------------------------------
function iCheckOutputSize(sz)
if ~isempty(sz)
    validateattributes(sz, {'numeric'}, ...
        {'row', 'positive', 'finite'}, mfilename, 'OutputSize');
end
end

%--------------------------------------------------------------------------
function [gTruth, params] = iParseGroundTruthInput(varargin)

p = inputParser;
p.addRequired('gTruth', @iCheckGroundTruth);
p = iAddCommonParameters(p);

p.parse(varargin{:});

userInput = p.Results;

gTruth = userInput.gTruth;

params = iCheckCommonParameters(userInput);

end

%--------------------------------------------------------------------------
function [imds, pxds, params] = iParseImdsPxdsInput(varargin)

p = inputParser;
p.addRequired('imds');
p.addRequired('pxds');
p = iAddCommonParameters(p);

p.parse(varargin{:});

userInput = iManageDispatchInBackgroundNameValue(p);

imds = userInput.imds;
pxds = userInput.pxds;

validateattributes(imds, {'matlab.io.datastore.ImageDatastore'},...
    {'scalar'}, mfilename, 'imds');

validateattributes(pxds, {'matlab.io.datastore.PixelLabelDatastore'},...
    {'scalar'}, mfilename, 'pxds');


params = iCheckCommonParameters(userInput);

end

%--------------------------------------------------------------------------
function iCheckGroundTruth(g)
validateattributes(g, {'groundTruth'}, {'vector','nonempty'}, ...
    mfilename, 'gTruth');

names = cell(numel(g),1);
for i = 1:numel(g)
    if hasTimeStamps(g(i).DataSource)
        error(message('vision:semanticseg:gTruthWithTimeStamps'));
    end
    
    iAssertAllGroundTruthHasPixelLabelType(g(i));
    
    iAssertAllPixelLabelDataIsNotMissing(g(i));
    
    names{i} = iGatherLabelNamesOfPixelLabelType(g(i));
end

% array of groundTruth should have same pixel label classes. order does not
% matter.
iAssertAllGroundTruthHasSameLabelNames(names);

end

%--------------------------------------------------------------------------
function iAssertAllGroundTruthHasPixelLabelType(gTruth)
if ~any(gTruth.LabelDefinitions.Type == labelType.PixelLabel)
    error(message('vision:semanticseg:missingPixelLabelData'))
end
end

%--------------------------------------------------------------------------
function iAssertAllPixelLabelDataIsNotMissing(gTruth)
if all(strcmp('', gTruth.LabelData.PixelLabelData))
    error(message('vision:semanticseg:missingPixelLabelData'));
end
end

%--------------------------------------------------------------------------
function names = iGatherLabelNamesOfPixelLabelType(gTruth)
defs = gTruth.LabelDefinitions;
names = defs.Name(defs.Type == labelType.PixelLabel);
end

%--------------------------------------------------------------------------
function iAssertAllGroundTruthHasSameLabelNames(names)

n = names{1};
for i = 2:numel(names)
    C = setdiff(n, names{i});
    if ~isempty(C)
        error(message('vision:semanticseg:inconsistentPixelLabelNames'));
    end
end

end

%--------------------------------------------------------------------------
function X = iConvertAnyRGBToGray(X)
if iscell(X)
    for i = 1:numel(X)
        if ~ismatrix(X{i})
            X = rgb2gray(X);
        end
    end
    X = cellfun(@(x)rgb2gray(x), X, 'UniformOutput', false);
else
    if ~ismatrix(X)
        X = rgb2gray(X);
    end
end
end

%--------------------------------------------------------------------------
function X = iConvertAnyGrayToRGB(X)
if iscell(X)
    for i = 1:numel(X)
        if ismatrix(X{i})
            X{i} = repelem(X{i},1,1,3);
        end
    end
else
    if ismatrix(X)
        X = repelem(X,1,1,3);
    end
end
end

%--------------------------------------------------------------------------
function vec = iMakeVertical( vec )
vec = reshape( vec, numel( vec ), 1 );
end

%--------------------------------------------------------------------------
function [X, Y] = iRandCrop(X, Y, outputSize)
if iscell(X)
    [X, Y] = cellfun(@(x,y)iRandCropImage(x,y,outputSize), X, Y, 'UniformOutput', false);
else
    [X, Y] = iRandCropImage(X, Y, outputSize);
end
end

%--------------------------------------------------------------------------
function [X, Y] = iCenterCrop(X, Y, outputSize)
if iscell(X)
    [X, Y] = cellfun(@(x,y)iCenterCropImage(x,y,outputSize), X, Y, 'UniformOutput', false);
    
else
    [X, Y] = iCenterCropImage(X,Y,outputSize);
end
end

%--------------------------------------------------------------------------
function [X, Y] = iCenterCropImage(X, Y, outputSize)
% X and Y MUST have same size for this to make sense.
X = augmentedImageDatastore.centerCrop(X,outputSize);
Y = augmentedImageDatastore.centerCrop(Y,outputSize);
end

%--------------------------------------------------------------------------
function [X, Y] = iRandCropImage(X, Y, outputSize)
rect = augmentedImageDatastore.randCropRect(X,outputSize);
X = augmentedImageDatastore.crop(X, rect);
Y = augmentedImageDatastore.crop(Y, rect);
end

%--------------------------------------------------------------------------
function X = iResize(X, outputSize)
if iscell(X)
    X = cellfun(@(x)augmentedImageDatastore.resizeImage(x, outputSize), X, 'UniformOutput', false);
else
    X = augmentedImageDatastore.resizeImage(X, outputSize);
end
end

%------------------------------------------------------------------
function L = iResizeLabelMatrix(L,outputSize)
if iscell(L)
    L = cellfun(...
        @(x)iResizeLabels(x, outputSize), ...
        L, 'UniformOutput', false);
else
    L = iResizeLabels(L, outputSize);
end
end

%------------------------------------------------------------------
function imOut = iResizeLabels(L,outputSize)
supportedWithCast = isa(L,'int8') || isa(L,'uint16') || isa(L,'int16');
supportedType = isa(L,'uint8') || isa(L,'single');
modeSupported = supportedWithCast || supportedType;

if supportedWithCast
    L = single(L);
end

if modeSupported
    imOut = images.internal.builtins.imageStackResizeNearest(L,outputSize);
else
    imOut = imresize(L,'OutputSize',outputSize,'method','nearest','Antialias',false);
end

end

%------------------------------------------------------------------
function resultsStruct = iManageDispatchInBackgroundNameValue(p)

resultsStruct = p.Results;

DispatchInBackgroundSpecified = ~any(strncmp('DispatchInBackground',p.UsingDefaults,length('DispatchInBackground')));
BackgroundExecutionSpecified = ~any(strncmp('BackgroundExecution',p.UsingDefaults,length('BackgroundExecution')));

% In R2017b, BackgroundExecution was name used to control
% DispatchInBackground. Allow either to be specified.
if BackgroundExecutionSpecified && ~DispatchInBackgroundSpecified
    resultsStruct.DispatchInBackground = resultsStruct.BackgroundExecution;
end

end

%------------------------------------------------------------------
function defaultExtensions = iDefaultFileExtensions()
 i = imformats;
 defaultExtensions = strcat('.', [i.ext]);
end

%------------------------------------------------------------------
function defaultReadFcn = iGetDefaultReadFcn()
% Get the default ReadFcn used by imageDatastore
 temp = imageDatastore({});
 defaultReadFcn = temp.ReadFcn;
end
