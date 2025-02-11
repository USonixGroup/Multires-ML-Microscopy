%pixelLabelImageDatastore Datastore for semantic segmentation networks.
%
%   pixelLabelImageDatastore will be removed in a future release. Use
%   imageDatastore, pixelLabelDatastore, and datastore combine instead:
%
%      imds = imageDatastore(...);
%      pxds = pixelLabelDatastore(...);
%      ds = combine(imds,pxds);
%
%   ds = pixelLabelImageDatastore(gTruth) creates a datastore
%   for training a semantic segmentation network using deep learning. The
%   input gTruth is an array of groundTruth objects. The output is a
%   pixelLabelImageDatastore object. Use this output with trainNetwork
%   from Deep Learning Toolbox to train convolutional neural networks for
%   semantic segmentation.
%
%   ds = pixelLabelImageDatastore(imds, pxds) creates a data
%   source for training a semantic segmentation network using deep
%   learning. The input imds is an imageDatastore that specifies the ground
%   truth images. The input pxds is a pixelLabelDatastore that specifies
%   the pixel label data. imds represents the training input to the network
%   and pxds represents the desired network output.
%
%   [...] = pixelLabelImageDatastore(..., Name, Value) specifies
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
%                              'OutputSize' is empty and the ouput size is
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
%                              queuing augmented images for use in
%                              training. Requires Parallel Computing
%                              Toolbox.
%
%                              Default: false
%
%   pixelLabelImageDatastore Properties:
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
%   pixelLabelImageDatastore Methods:
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
%      transform            - Create an altered form of the current datastore by
%                             specifying a function handle that will execute
%                             after read on the current datastore.
%      combine              - Create a new datastore that horizontally
%                             concatenates the result of read from two or more
%                             input datastores.
%
%   Example 1
%   ---------
%   % Train a semantic segmentation network using data provided by the
%   % pixelLabelImageDatastore.
%
%   % Load training images and pixel labels.
%   dataSetDir = fullfile(toolboxdir('vision'),'visiondata','triangleImages');
%   imageDir = fullfile(dataSetDir, 'trainingImages');
%   labelDir = fullfile(dataSetDir, 'trainingLabels');
%
%   % Create an imageDatastore holding the training images.
%   imds = imageDatastore(imageDir);
%
%   % Define the class names and their associated label IDs.
%   classNames = ["triangle", "background"];
%   labelIDs   = [255 0];
%
%   % Create a pixelLabelDatastore holding the ground truth pixel labels for
%   % the training images.
%   pxds = pixelLabelDatastore(labelDir, classNames, labelIDs);
%
%   % Create network for semantic segmentation.
%   layers = [
%       imageInputLayer([32 32 1])
%       convolution2dLayer(3, 64, 'Padding', 1)
%       reluLayer()
%       maxPooling2dLayer(2, 'Stride', 2)
%       convolution2dLayer(3, 64, 'Padding', 1)
%       reluLayer()
%       transposedConv2dLayer(4, 64, 'Stride', 2, 'Cropping', 1);
%       convolution2dLayer(1, 2);
%       softmaxLayer()
%       pixelClassificationLayer()
%       ]
%
%   % Create datastore for training a semantic segmentation network.
%   ds = pixelLabelImageDatastore(imds,pxds);
%
%   % Setup training options. Note MaxEpochs is set to 5 to reduce example
%   % run-time.
%   options = trainingOptions('sgdm', 'InitialLearnRate', 1e-3, ...
%       'MaxEpochs', 5, 'VerboseFrequency', 10);
%
%   % Train network.
%   net = trainNetwork(ds, layers, options)
%
% See also trainNetwork, groundTruth, imageDataAugmenter, semanticseg,
%          pixelClassificationLayer, pixelLabelDatastore, imageDatastore.

% Copyright 2017-2021 The MathWorks, Inc.
classdef pixelLabelImageDatastore < ...
        vision.internal.PixelLabelImageDatastore &...
        matlab.io.datastore.internal.RandomizedReadable

    methods        
        function this = pixelLabelImageDatastore(varargin)
            this = this@vision.internal.PixelLabelImageDatastore(varargin{:});
        end
        
    end
    
    methods(Access = protected)
        %------------------------------------------------------------------
        function aCopy = copyElement(this)
            s = this.copyElementImpl();
            aCopy = pixelLabelImageDatastore(s);
        end
    end
    
 end
