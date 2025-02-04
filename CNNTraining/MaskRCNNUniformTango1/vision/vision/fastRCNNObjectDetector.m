
% Copyright 2016-2023 The MathWorks, Inc.

classdef fastRCNNObjectDetector < vision.internal.EnforceScalarHandle & matlab.mixin.CustomDisplay
    
    properties(GetAccess = public, SetAccess = public)
        % ModelName Name of the classification model. By default, the name
        %           is set by trainFastRCNNObjectDetector. The name may
        %           be modified after training as desired.
        ModelName char
    end
    
    properties(GetAccess = public, SetAccess = protected)
        % Network An object representing the Fast R-CNN network used within
        %         the Faster R-CNN detector.
        Network
    end
    
    properties(Access = public)
        % RegionProposalFcn A function handle to the region proposal
        %                   method.
        RegionProposalFcn
    end
    
    properties(Dependent)
        % ClassNames A cell array of object class names. These are the
        %            object classes that the R-CNN detector was trained to
        %            find.
        ClassNames
    end
    
    properties (GetAccess = public, SetAccess = protected)
        % MinObjectSize Minimum object size supported by the Fast R-CNN
        %               network. The minimum size depends on the network
        %               architecture.
        MinObjectSize
    end
    
    properties(Hidden, Access = protected)
        % UsingDefaultRegionProposalFcn A logical flag to determine whether
        % or not the user has trained the detector with a custom region
        % proposal function.
        UsingDefaultRegionProposalFcn
        
        % BackgroundLabel Label to use for background class. Default is
        % 'Background'.
        BackgroundLabel
        
        % LayerIndices A struct that caches indices to certain layers used
        %             frequently during detection.
        LayerIndices
        
    end
    
    properties(Hidden)
        % Mean and standard deviation values for box regression.
        BoxMean
        BoxStd        
    end
    
    properties(Access = private)
        % RecomputeScaleFactor Whether or not the ScaleFactor should be
        %                      recomputed during inference. This is true
        %                      for models trained prior to R2018b.
        RecomputeScaleFactor = false
    end
    
    properties(Access = private, Transient)
        ImageSizeCache
        ROIScaleFactorCache
    end

    properties (Access = protected, Transient)
        FilterBboxesFunctor
    end

    
    methods(Static, Access = public, Hidden)
        
        %------------------------------------------------------------------
        % Returns bounding boxes in an M-by-4 array. Each bounding box is a
        % row vector containing [x y width height]. The scores is an M-by-1
        % vector. Higher score values indicate the bounding box is more
        % likely to contain an object.
        %------------------------------------------------------------------
        function [bboxes, scores] = proposeRegions(I)
            alpha = 0.65;
            beta  = 0.75;
            minScore = 0.1;
            
            [bboxes, scores] = vision.internal.rcnn.edgeBoxes(I, alpha, beta, minScore);
            
        end
        
        %------------------------------------------------------------------
        function  I = scaleImage(I, imageLength)
            if iscell(I)
                for ii = 1:numel(I)
                    I{ii} = scaleMe(I{ii}, imageLength);
                end
                return;
            end

            if ischar(I)
                I = imread(I);
            end

            I = scaleMe(I, imageLength);

            function O = scaleMe(I, imageLength)
                % scale the smallest side to value specified
                sz = size(I);
                [~, smallestSide] = min(sz(1:2));
                scale = imageLength / sz(smallestSide);

                O = imresize(I, scale);
            end
        end
        
        %------------------------------------------------------------------
        function [detector, lgraph, info] = train(trainingData, lgraph, opts, executionSettings, params, checkpointSaver)
            assert(isa(lgraph,'nnet.cnn.LayerGraph'));
            
            params.Verbose         = opts.Verbose;
            params.MiniBatchSize   = opts.MiniBatchSize;
            
            % create datastore
            %  dispatcher needs to scale roi depending on input image size for this it uses the network layers.
            if params.TrainingDataWasTable
                [ds, mapping, batchingFcn, params] = fastRCNNObjectDetector.createTrainingDatastoreFromTable(trainingData, params, lgraph);
            else
                [ds, mapping, batchingFcn, params, opts] = fastRCNNObjectDetector.createTrainingDatastore(trainingData, params, lgraph, opts);
            end

            % Update detector properties to ensure checkpoint is up-to-date.
            checkpointSaver.Detector.BoxMean = params.BoxMean;
            checkpointSaver.Detector.BoxStd  = params.BoxStd;
            
            if params.FreezeBatchNormalization
                lgraph = vision.internal.cnn.RCNNLayers.freezeBatchNorm(lgraph);
            end
            isValidationDataSpecified = ~isempty(opts.ValidationData);
            if isValidationDataSpecified 
                infoContent = vision.internal.cnn.FastRCNNAndRPNValidationContent();
                validationPredictStrategyFcn = @vision.internal.cnn.ValidationPredictDatastoreStrategy;
            else
                infoContent = vision.internal.cnn.FastRCNNAndRPNContent();
                validationPredictStrategyFcn = @(varargin)[];
            end
            columnStrategy = vision.internal.cnn.ClassificationRegressionColumns(isValidationDataSpecified);
            axesConfigFactory = vision.internal.cnn.FastRCNNAxesConfig();
            axesConfig = axesConfigFactory.AxesConfiguration;
            
            summaryBuilderFcn = @vision.internal.cnn.FastRCNNSummary.makeSummary;
            summaryMonitorInfo.Monitor = params.Monitor;
            summaryMonitorInfo.SummaryBuilderFcn = summaryBuilderFcn;
            summaryMonitorInfo.MonitorFieldInformation = vision.internal.cnn.FastRCNNAndRPNValidationContent.monitorSummaryFields(isValidationDataSpecified);
            try
                [network, info] = vision.internal.cnn.trainNetwork(...
                    ds, lgraph, opts, executionSettings, mapping, checkpointSaver, ...
                    summaryMonitorInfo, ...
                    infoContent, ...
                    columnStrategy, ...
                    axesConfig, ...
                    batchingFcn, ...
                    validationPredictStrategyFcn, ...
                    iInfoToSummaryMapAndStartValues());
            catch ME
                if strcmp(ME.identifier,'nnet_cnn:internal:cnn:GeneralDatastoreDispatcher:VariableInputSizes')
                    error(message('vision:rcnn:unableToBatchImages'));
                else
                    rethrow(ME);
                end
            end
            
            if params.FreezeBatchNormalization
                lgraph = layerGraph(network);
                lgraph = vision.internal.cnn.RCNNLayers.unfreezeBatchNorm(lgraph);
                network = vision.internal.cnn.createDAGNetwork(lgraph);
            end
            
            detector = fastRCNNObjectDetector.partiallyMakeDetector(params);
            detector.Network = network;
            
            % Return trained network as layer graph for use in 4-step
            % Faster R-CNN training.
            lgraph = layerGraph(network);            
            
        end
        
        %------------------------------------------------------------------
        function [bboxes, scores] = filterSmallBBoxes(bboxes, scores, minSize)
            [bboxes,scores] = vision.internal.cnn.utils.FilterBboxesFunctor.filterSmallBBoxes(minSize,bboxes,scores);
        end
        
        %------------------------------------------------------------------
        function [bboxes, scores] = filterLargeBBoxes(bboxes, scores, maxSize)
            [bboxes,scores] = vision.internal.cnn.utils.FilterBboxesFunctor.filterLargeBBoxes(maxSize,bboxes,scores);
        end
        
        %------------------------------------------------------------------
        % Detector checkpoint function. Assign network checkpoint to
        % detector.
        function detector = detectorCheckpoint(net, detector, freezeBatchNorm)
            if freezeBatchNorm
                % Unfreeze prior to creating detector checkpoint so that
                % internal "frozen" batch norm layer is not serialized.
                lgraph = layerGraph(net);
                lgraph = vision.internal.cnn.RCNNLayers.unfreezeBatchNorm(lgraph);
                net = vision.internal.cnn.createDAGNetwork(lgraph);
            end
            detector.Network = net;
        end       
        
        %------------------------------------------------------------------
        function imageInfo = collectImageInfo(trainingData,params)
            % Collect size information if images need to be resized via the
            % SmallestImageDimension parameter.
            imageInfo = [];
            if params.ScaleImage
              imageInfo = vision.internal.cnn.imageInformation(trainingData, params.InputSize, params.UseParallel);
            end
        end
        
        %------------------------------------------------------------------
        function trainingData = scaleImageData(trainingData, imageInfo, params)
            if params.ScaleImage
                trainingData = iScaleGroundTruth(trainingData, imageInfo, params.ImageScale, params.UseParallel);
            end
        end
        
        %------------------------------------------------------------------
        function executionSettings = setupExecutionEnvironment(opts, useParallel)
            % Set up and validate parallel training. This opens a pool if required.

            % If parallel training is enabled, use useParallel to decide
            % whether to force enable DispatchInBackground. This will lead
            % to configuring the pool as if DispatchInBackground is
            % selected, even if the user did not select that option
            forceDispatchInBackground = iIsExecutionEnvironmentParallel(opts.ExecutionEnvironment) && useParallel;

            userDispatchInBackground = opts.DispatchInBackground;
            % Edit the user option to possibly force enable
            % DispatchInBackground
            opts.DispatchInBackground = opts.DispatchInBackground || forceDispatchInBackground;

            % Set training precision.
            precision = nnet.internal.cnn.util.Precision('single');

            % Setup the environment
            executionSettings = nnet.internal.cnn.assembler.setupExecutionEnvironment( opts, [], precision );

            % Restore the option selected by the user
            executionSettings.backgroundPrefetch = userDispatchInBackground;
        end
        
        %------------------------------------------------------------------
        function detector = partiallyMakeDetector(params)
            % Construct a detector and initialize a subset of the
            % detector's properties. BoxMean and BoxStd might not be
            % initialized. Network is not set and is only set at the end of
            % training.
            %
            % This function is used during training to create detector
            % checkpoints and to build the final detector returned by the
            % training function.
                       
            detector = fastRCNNObjectDetector();
            detector.RegionProposalFcn             = params.RegionProposalFcn;
            detector.UsingDefaultRegionProposalFcn = params.UsingDefaultRegionProposalFcn;
            detector.BackgroundLabel               = params.BackgroundLabel;
            detector.MinObjectSize                 = params.MinObjectSize;
            detector.ModelName                     = params.ModelName;
            
            if isfield(params, 'BoxMean')
                detector.BoxMean = params.BoxMean;
            end
            
            if isfield(params, 'BoxStd')
                detector.BoxStd = params.BoxStd;
            end
        end
    end
    
    %----------------------------------------------------------------------
    methods
        function varargout = detect(this, I, varargin)
            %DETECT Detect objects within the images using fastRCNN object detector.
            %
            %   bboxes = detect(fastRCNN,I) detects objects within the image I.
            %   The location of objects within I are returned in bboxes, an
            %   M-by-4 matrix defining M bounding boxes. Each row of bboxes
            %   contains a four-element vector, [x, y, width, height]. This
            %   vector specifies the upper-left corner and size of a bounding
            %   box in pixels. fastRCNN is a fastRCNNObjectDetector and I is
            %   a truecolor or grayscale image.
            %
            %   bboxes = detect(fastRCNN,IBatch) detects objects within each
            %   image contained in the batch of images IBatch. IBatch is a
            %   numeric array containing images in the format
            %   H-by-W-by-C-by-B, where B is the number of images in the
            %   batch, and C is the channel size. For grayscale images, C must be
            %   1. The size of each image in the batch, H-by-W-by-C, must be
            %   greater than or equal to the network input size. bboxes is a
            %   B-by-1 cell array, containing M-by-4 matrices for each image
            %   in the batch.
            %
            %   [..., scores] = detect(fastRCNN,I) optionally return the detection
            %   scores for each bounding box. The score for each detection is
            %   the output of the softmax classifier used in the
            %   rcnn.Network. The range of the score is [0 1]. Larger score
            %   values indicate higher confidence in the detection. scores is a
            %   B-by-1 cell array, if the input I is a batch of images in the
            %   format H-by-W-by-C-by-B.
            %
            %   [..., labels] = detect(fastRCNN,I) optionally return the labels
            %   assigned to the bounding boxes in an M-by-1 categorical
            %   array. The labels used for object classes is defined during
            %   training using the trainRCNNObjectDetector function. labels is a
            %   B-by-1 cell array, if the input I is a batch of images in the
            %   format H-by-W-by-C-by-B.
            %
            %   detectionResults = detect(fastRCNN,DS) detects objects within the
            %   series of images returned by the read method of datastore, DS. DS, can
            %   be a datastore that returns a table or a cell array with the first
            %   column containing images. detectionResults is a 3-column table with
            %   variable names 'Boxes', 'Scores', and 'Labels' containing bounding
            %   boxes, scores, and the labels. The location of objects within an image,
            %   I are returned in bounding boxes, an M-by-4 matrix defining M bounding
            %   boxes. Each row of boxes contains a four-element vector, [x, y, width,
            %   height]. This vector specifies the upper-left corner and size of a
            %   bounding box in pixels. fastRCNN is a fastRCNNObjectDetector
            %   object.
            %
            %   [...] = detect(..., roi) optionally detects objects within
            %   the rectangular search region specified by roi. roi must be a
            %   4-element vector, [x, y, width, height], that defines a
            %   rectangular region of interest fully contained in I.
            %
            %   [...] = detect(..., Name, Value) specifies additional
            %   name-value pairs described below:
            %
            %    'Threshold'           A scalar between 0 and 1. Detections
            %                          with scores less than the threshold
            %                          value are removed. Increase this value
            %                          to reduce false positives.
            %
            %                          Default: 0.5
            %
            %    'NumStrongestRegions' Specify the maximum number of
            %                          strongest region proposals to process.
            %                          Reduce this value to speed-up
            %                          processing time at the cost of
            %                          detection accuracy. Set this to inf to
            %                          use all region proposals.
            %
            %                          Default: 2000
            %
            %    'SelectStrongest'     A logical scalar. Set this to true to
            %                          eliminate overlapping bounding boxes
            %                          based on their scores. This process is
            %                          often referred to as non-maximum
            %                          suppression. Set this to false if you
            %                          want to perform a custom selection
            %                          operation. When set to false, all the
            %                          detected bounding boxes are returned.
            %
            %                          Default: true
            %
            %    'MinSize'             Specify the size of the smallest
            %                          region containing an object, in
            %                          pixels, as a two-element vector,
            %                          [height width]. When the minimum size
            %                          is known, you can reduce computation
            %                          time by setting this parameter to that
            %                          value. By default, 'MinSize' is the
            %                          smallest object that can be detected
            %                          by the trained network.
            %
            %                          Default: fastRCNN.MinObjectSize
            %
            %    'MaxSize'             Specify the size of the biggest region
            %                          containing an object, in pixels, as a
            %                          two-element vector, [height width].
            %                          When the maximum object size is known,
            %                          you can reduce computation time by
            %                          setting this parameter to that value.
            %                          Otherwise, the maximum size is
            %                          determined based on the width and
            %                          height of I.
            %
            %                          Default: size(I)
            %
            %    'MiniBatchSize'       The mini-batch size used for processing a
            %                          large collection of images. Images are grouped
            %                          into mini-batches and processed as a batch to
            %                          improve computational efficiency. Larger
            %                          mini-batch sizes lead to faster processing, at
            %                          the cost of more memory.
            %
            %                          Default: 128
            %
            %    'ExecutionEnvironment'
            %                          Specify what hardware resources will be used to
            %                          run the Fast R-CNN detector. Valid values for
            %                          resource are:
            %
            %                          'auto' - Use a GPU if it is available, otherwise
            %                                   use the CPU.
            %
            %                          'gpu'  - Use the GPU. To use a GPU, you must have
            %                                   Parallel Computing Toolbox(TM), and a
            %                                   CUDA-enabled NVIDIA GPU. If a suitable
            %                                   GPU is not available, an error message
            %                                   is issued.
            %
            %                          'cpu'  - Use the CPU.
            %
            %                          Default: 'auto'
            %
            %   Notes
            %   -----
            %   - When 'SelectStrongest' is true the selectStrongestBboxMulticlass
            %     function is used to eliminate overlapping boxes. By
            %     default, the function is called as follows:
            %
            %        selectStrongestBboxMulticlass(bbox, scores, labels, ...
            %                  'RatioType', 'Min', 'OverlapThreshold', 0.5);
            %
            %   - The class of the input image I should match class of the
            %     images used for train the detector. Otherwise, the
            %     differences in the range of image pixel values caused by
            %     different class types may cause the detector to perform
            %     poorly.
            %
            %   Class Support
            %   -------------
            %   The input image I can be uint8, uint16, int16, double,
            %   single, or logical, and it must be real and non-sparse.
            %
            %   Example
            %   -------
            %   % Load pre-trained stop sign detector.
            %   data = load('rcnnStopSigns.mat');
            %
            %   imageFilenames = fullfile(toolboxdir('vision'),'visiondata', ...
            %       data.stopSigns.imageFilename);
            %
            %   ds = imageDatastore(imageFilenames);
            %
            %   % Resize images for uniform size.
            %   ds = transform(ds, @(x)imresize(x,[920,968]));
            %
            %   % Run detector.
            %   detector = data.fastRCNN;
            %   detectionResults = detect(detector, ds, 'MiniBatchSize', 16);
            %
            %   % Read the first test image.
            %   I = read(ds);
            %
            %   % Display results.
            %   bboxes = detectionResults.Boxes{1};
            %   labels = detectionResults.Labels{1};
            %   detectedImg = insertObjectAnnotation(I, 'Rectangle', bboxes, labels);
            %   figure
            %   imshow(detectedImg)
            %
            %   See also trainFastRCNNObjectDetector,
            %            fastRCNNObjectDetector/classifyRegions,
            %            selectStrongestBboxMulticlass.

            params = this.parseDetectInputs(I, varargin{:});

            roi       = params.ROI;
            useROI    = params.UseROI;
            inputSize = this.Network.Layers(this.LayerIndices.ImageLayerIdx).InputSize;
            colorPreprocessing = vision.internal.cnn.utils.colorPreprocessingForImageInputSize(inputSize);
            if params.DetectionInputWasDatastore
                nargoutchk(0,1);
                % Copy and reset the given datastore, so external state events are
                % not reflected.
                ds = copy(I);
                reset(ds);

                ds = transform(ds, @(x)iPreProcessForDatastoreRead(x,roi,useROI,colorPreprocessing, ...
                    params.MinSize, params.MaxSize, params.NumStrongestRegions, this.RegionProposalFcn, this.FilterBboxesFunctor));
                params = iUpdateImageSizeIfNeeded(ds,params);
                updateNetworkIfRequired(this,params.ImageSize);
                [dataMap, layerOut] = datastoreInputsForActivationsMIMO(this,inputSize);
                params = updateParamsForPostProcess(this, params);
                varargout{1} = vision.internal.cnn.fastrcnn.detectUsingDatastore(ds, this.Network, dataMap, layerOut, params);
            else
                nargoutchk(0,3);
                if params.DetectionInputWasBatchOfImages
                    [Iroi, bboxes] = iPreprocessBatch(I, roi, useROI, colorPreprocessing, ...
                        params.MinSize, params.MaxSize, params.NumStrongestRegions, this.RegionProposalFcn, this.FilterBboxesFunctor,...
                        params.ImageSize, params.InputSize);
                else
                    [Iroi, bboxes] = iPreprocess(I, roi, useROI, colorPreprocessing, ...
                        params.MinSize, params.MaxSize, params.NumStrongestRegions, this.RegionProposalFcn, this.FilterBboxesFunctor);
                end
                params = iUpdateImageSizeIfNeeded(Iroi,params);
                updateNetworkIfRequired(this,params.ImageSize);

                [ds,dataMap,clsName,regName] = this.inputsForActivationsMIMO(Iroi, bboxes);
                [fmap, reg] = this.Network.activationsMIMO(ds, dataMap, {clsName,regName}, ...
                    'ExecutionEnvironment', params.ExecutionEnvironment,...
                    'MiniBatchesInCell', true);

                postProcessParams = updateParamsForPostProcess(this, params);
                if params.DetectionInputWasBatchOfImages
                    numFiles = params.ImageSize(4);
                    numBatches = 1;
                    [varargout{1:nargout}] = vision.internal.cnn.fastrcnn.postProcessBatchActivations(...
                        fmap,reg,{bboxes},numFiles,numBatches,postProcessParams);
                else
                    [varargout{1:nargout}] = fastRCNNObjectDetector.postProcessActivations(...
                        fmap{1}, reg{1}, bboxes, postProcessParams);
                end
            end

        end

        %------------------------------------------------------------------
        function [labels, scores, allScores] = classifyRegions(this, I, roi, varargin)
            % [labels, scores] = classifyRegions(fastRCNN, I, rois) classifies
            % objects within regions specified in rois, an M-by-4 array
            % defining M rectangular regions. Each row of rois contains a
            % four-element vector [x, y, width, height]. This vector
            % specifies the upper-left corner and size of a region in
            % pixels.
            %
            % The output labels is an M-by-1 categorical array of class
            % names assigned to each region in rois. scores is an M-by-1
            % vector of classification scores. The range of the score is [0
            % 1]. Larger score values indicate higher confidence in the
            % classification.
            %
            % [..., allScores] = classifyRegions(...) optionally return all
            % the classification scores in an M-by-N matrix for M regions.
            % N is the number of classes.
            %
            % [...] = classifyRegions(..., 'ExecutionEnvironment',
            % resource) determines what hardware resources will be used to
            % run Fast R-CNN detector. Valid values for resource
            % are:
            %
            %  'auto' - Use a GPU if it is available, otherwise use the CPU.
            %
            %  'gpu'  - Use the GPU. To use a GPU, you must have Parallel
            %           Computing Toolbox(TM), and a CUDA-enabled NVIDIA
            %           GPU. If a suitable GPU is not available, an error
            %           message is issued.
            %
            %  'cpu'  - Use the CPU.
            %
            % The default is 'auto'.
            %
            % Notes
            % -----
            % - The class of the input image I should match class of the
            %   images used for train the detector. Otherwise, the
            %   differences in the range of image pixel values caused by
            %   different class types may cause the detector to perform
            %   poorly.
            %
            % Class Support
            % -------------
            % The input image I can be uint8, uint16, int16, double,
            % single, or logical, and it must be real and non-sparse.
            %
            % Example: Classify multiple image regions
            % -----------------------------------------
            % % Load a pre-trained detector
            % data = load('rcnnStopSigns.mat', 'fastRCNN');
            % fastRCNN = data.fastRCNN;
            %
            % % Read test image
            % img = imread('stopSignTest.jpg');
            %
            % % Specify multiple regions to classify within test image.
            % rois = [416   143    33    27
            %     347   168    36    54];
            %
            % % Classify regions
            % [labels, scores] = classifyRegions(fastRCNN, img, rois);
            %
            % detectedImg = insertObjectAnnotation(img, 'rectangle', rois, cellstr(labels));
            %
            % figure
            % imshow(detectedImg)
            %
            % See also fastRCNNObjectDetector/detect, trainFastRCNNObjectDetector.
            
            [roi, params] = fastRCNNObjectDetector.parseClassifyInputs(I, roi, varargin{:});
            
            % convert [x y w h] boxes from proposal function into
            % [x1 y1 x2 y2] format expected by ROI pooling layer.
            bboxesX1Y1X2Y2 = vision.internal.cnn.boxUtils.xywhToX1Y1X2Y2(single(roi));
            
            updateNetworkIfRequired(this,size(I));
            
            [ds,dataMap,clsName] = this.inputsForActivationsMIMO(I, bboxesX1Y1X2Y2);
            fmap = this.Network.activationsMIMO(ds, dataMap, {clsName}, ...
                'ExecutionEnvironment', params.ExecutionEnvironment,...
                'MiniBatchesInCell', true);
            fmap = fmap{1};
            
            allScores = squeeze(fmap)';
            classNames = categorical(this.ClassNames, this.ClassNames);
            labels = nnet.internal.cnn.util.undummify( allScores, classNames );
            
            scores = getScoreAssociatedWithLabel(this, labels, allScores);
            
        end
    end

    methods (Static, Hidden)
        function [bboxes, scores, labels] = postProcessActivations(fmap, reg, bboxes, params)

            if isempty(bboxes)

                scores = zeros(0,1,'single');
                labels = categorical(cell(0,1),params.ClassNames);

            else
                % Network outputs boxes in [x1 y1 x2 y2] format in the image
                % space. Convert to [x y w h] for further processing.
                bboxes = vision.internal.cnn.boxUtils.x1y1x2y2ToXYWH(bboxes);

                % Input fmap is [1 1 numClasses numProposals].
                allScores = squeeze(fmap); % -> [numClasses+background numProposals]

                % Remove scores associated with background.
                bgIndex = strcmp(string(params.ClassNames),params.BackgroundLabel);
                allScores(bgIndex,:) = []; % -> [numClasses numProposals]

                % Reshape scores into a column vector so that it groups
                % class scores for each proposal. [c1 c2 c1 c2 ...]'
                scores = reshape(allScores,[],1); % -> [numClasses*numProposals 1]

                numClasses = numel(params.ClassNames)-1; % exclude background

                % replicate each proposal box for each class.
                bboxes = repelem(bboxes, numClasses, 1);
                numObservations = size(reg,4);

                % replicate labels
                classNames = categorical(params.ClassNames, params.ClassNames);
                classNames(classNames==params.BackgroundLabel) = [];
                classNames = removecats(classNames, params.BackgroundLabel);

                ind = (1:numel(classNames))';
                labels = classNames(repmat(ind,numObservations,1));

                % reg is 4D array [1 1 numClasses*4 numObs]. reshape
                % to 4-by-(numClasses * numObs), where each column is
                % the target for a specific classes. Data is arranged
                % as follows:
                %
                %   [c1 c2 c1 c2 c1 c2], where rows stride by
                %   numClasses goes to next observation.
                reg = reshape(reg, 4, numClasses*numObservations)';

                % apply regression to offset boxes.
                reg = (reg .* params.BoxStd) + params.BoxMean;
                bboxes = fastRCNNObjectDetector.applyReg(bboxes, reg, params.MinSize, params.MaxSize);

                [bboxes, scores, labels] = fastRCNNObjectDetector.filterBoxesAfterRegression(bboxes,scores,labels,params.ImageSize);

                switch params.DetectionsOnBorder
                    case 'clip'
                        bboxes = vision.internal.detector.clipBBox(bboxes,params.ImageSize);

                        % Remove boxes that are too small after clipping.
                        tooSmall = any(bboxes(:,3:4) < params.MinSize,2);
                        bboxes(tooSmall,:) = [];
                        scores(tooSmall,:) = [];
                        labels(tooSmall,:) = [];
                    case 'discard'
                        [bboxes, scores, labels] = fastRCNNObjectDetector.discardBBoxOutsideImage(bboxes,scores,labels,params.ImageSize);
                end

                keep = scores >= params.Threshold;
                bboxes = bboxes(keep,1:4);
                scores = scores(keep,:);
                labels = labels(keep,:);

                if params.SelectStrongest
                    [bboxes, scores, labels] = selectStrongestBboxMulticlass(bboxes,scores,labels,...
                        'RatioType', 'Min', 'OverlapThreshold', 0.5);
                end

                % return bboxes in original image space.
                bboxes(:,1:2) = vision.internal.detector.addOffsetForROI(bboxes(:,1:2),params.ROI,params.UseROI);
            end
        end
    end
    
    methods
        
        %------------------------------------------------------------------
        function this = fastRCNNObjectDetector(varargin)
            if nargin == 0
                
                this.UsingDefaultRegionProposalFcn = false;
                this.BackgroundLabel = 'Background';
                
            elseif nargin == 1
                clsname = 'fastRCNNObjectDetector';
                
                validateattributes(varargin{1},...
                    {clsname}, ...
                    {'nonempty','scalar'}, mfilename);
                
                if isequal(clsname, class(varargin{1}))
                    this = setPropertiesOnLoad(this, saveobj(varargin{1}));
                end
            end
            this.FilterBboxesFunctor = vision.internal.cnn.utils.FilterBboxesFunctor;
        end
        
        %------------------------------------------------------------------
        function set.Network(this, network)
            validateattributes(network,{'DAGNetwork'},{'scalar'});
            fastRCNNObjectDetector.analyzeNetwork(network);
            checkLayerForBackgroundLabel(this, network);
            network = this.syncROIPoolCache(network);
            this.Network = network;
            this.setLayerIndices(this.Network);
            imageSize = this.Network.Layers(this.LayerIndices.ImageLayerIdx).InputSize; %#ok<MCSUP>
            this.updateNetworkIfRequired(imageSize);            
        end
        
        %------------------------------------------------------------------
        function set.RegionProposalFcn(this, fcn)
            
            fastRCNNObjectDetector.checkRegionProposalFcn(fcn);
            
            this.RegionProposalFcn = fcn;
        end
        
        %------------------------------------------------------------------
        function cls = get.ClassNames(this)
            if isempty(this.Network)
                cls = [];
            else
                idx = this.LayerIndices.ClassificationLayerIdx;
                classes = this.Network.Layers(idx).Classes;
                names = categories( classes );
                cls = cellstr(names(:));
            end
        end
    end
    
    methods (Access = protected)
        %------------------------------------------------------------------
        function updateNetworkIfRequired(this, imageSize)
            if this.RecomputeScaleFactor && ~isequal(this.ImageSizeCache,imageSize)
                
                % Update ROI scale factor if required.
                % Recompute scale factor to maintain backward
                % compatibility with models train in previous releases
                % (<= 18a).
                if this.RecomputeScaleFactor
                    analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(this.Network);
                    
                    scaleFactor = vision.internal.cnn.RCNNLayers.imageToFeatureScaleGivenImageSize(analysis,imageSize);
                    
                    % update cache value for roi pooling layer.
                    this.ROIScaleFactorCache.Value = scaleFactor;
                end
                
                this.ImageSizeCache = imageSize;
            end
        end
        
        %------------------------------------------------------------------
        function setLayerIndices(this, network)
            
            this.LayerIndices = iPopulateLayerIndices(network.Layers);
            
        end
        
        %------------------------------------------------------------------
        function network = syncROIPoolCache(this,network)
            % Use scale factor cache in roi pooling layer. This allows us to
            % change the value of the cached property and have it
            % automatically update in the layer.
            idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.ROIMaxPooling2DLayer') || isa(x,'vision.cnn.layer.ROIAveragePooling2DLayer') ,...
                network.Layers);
            idx = find(idx,1,'first');
            scaleFactor = network.Layers(idx).ScaleFactor;
            this.ROIScaleFactorCache = vision.internal.cnn.layer.util.PropertyCache(scaleFactor);
            network = vision.internal.cnn.RCNNLayers.updateROIPoolingLayerScaleFactorValue(network, this.ROIScaleFactorCache);
        end
       
        %------------------------------------------------------------------
        function dispatcher = configureRegionDispatcher(~, I, bboxes, miniBatchSize, imageSize)
            endOfEpoch    = 'truncateLast';
            precision     = nnet.internal.cnn.util.Precision('single');
            regionResizer = fastRCNNObjectDetector.createRegionResizer(imageSize);
            dispatcher    = vision.internal.rcnn.ImageRegionDispatcher(...
                I, bboxes, miniBatchSize, endOfEpoch, precision, imageSize, regionResizer);
        end
        
        %------------------------------------------------------------------
        % Returns classification score associated with a label.
        %------------------------------------------------------------------
        function scores = getScoreAssociatedWithLabel(this, labels, allScores)
            N = numel(this.ClassNames);
            M = numel(labels);
            
            ind = sub2ind([M N], 1:M, double(labels)');
            
            scores = allScores(ind)';
        end
        
        %------------------------------------------------------------------
        function checkLayerForBackgroundLabel(this, network)
            % The classification layer must have a "Background" class to
            % support the detect method. Networks trained using
            % trainRCNNObjectDetector will have this type of network.
            
            idx = find(...
                arrayfun(@(x)isa(x,'nnet.cnn.layer.ClassificationOutputLayer'),network.Layers),...
                1,'last');
            
            if ~ismember(this.BackgroundLabel, network.Layers(idx).ClassNames)
                error(message('vision:rcnn:missingBackgroundClass'));
            end
        end
        
        %------------------------------------------------------------------
        function this = setPropertiesOnLoad(this, s)
            try
                s = iUpdatePreviousVersions(s);
                
                vision.internal.requiresNeuralToolbox(mfilename);
                
                this.BackgroundLabel   = s.BackgroundLabel;
                
                if ~isempty(s.RegionProposalFcn)
                    this.RegionProposalFcn = s.RegionProposalFcn;
                end
                
                this.UsingDefaultRegionProposalFcn = s.UsingDefaultRegionProposalFcn;
                this.MinObjectSize                 = s.MinObjectSize;
                this.ModelName                     = s.ModelName;
                this.BoxMean                       = s.BoxMean;
                this.BoxStd                        = s.BoxStd;
               
                % Backward compatibility support.
                this.RecomputeScaleFactor = s.RecomputeScaleFactor;
                
                % Set networks after other properties are set because of
                % the need to sync detector and layer properties.
                if ~isempty(s.Network)
                    this.Network = s.Network;
                end
                
            catch ME
                rethrow(ME);
            end
        end
        
        %------------------------------------------------------------------
        function params = parseDetectInputs(this, I, varargin)
            % image should be bigger network image input layer. This
            % ensures the feature map sizes are large enough to perform RPN
            % processing and ROI Pooling.
            params.DetectionInputWasDatastore = ~isnumeric(I);

            if ~params.DetectionInputWasDatastore
                sampleImage = I;
            else
                sampleImage = vision.internal.cnn.validation.checkDetectionInputDatastore(I, mfilename);
            end

            network = this.Network;

            networkInputSize = network.Layers(this.LayerIndices.ImageLayerIdx).InputSize;

            validateChannelSize = false;
            validateImageSize   = true; % fast R-CNN cannot support images smaller than input size
            [sz,params.DetectionInputWasBatchOfImages] = vision.internal.cnn.validation.checkDetectionInputImage(...
                networkInputSize,sampleImage,validateChannelSize,validateImageSize);

            p = inputParser;
            p.addOptional('roi', zeros(0,4));
            p.addParameter('NumStrongestRegions', 2000);
            p.addParameter('SelectStrongest', true);
            p.addParameter('MiniBatchSize', 128);
            p.addParameter('MinSize', []);
            p.addParameter('MaxSize', sz(1:2));
            p.addParameter('ExecutionEnvironment', 'auto');
            p.addParameter('Threshold', 0.5)
            p.addParameter('DetectionsOnBorder', 'clip');
            parse(p, varargin{:});

            userInput = p.Results;

            useROI = ~ismember('roi', p.UsingDefaults);

            if useROI
                vision.internal.detector.checkROI(userInput.roi, size(I));
            end

            vision.internal.inputValidation.validateLogical(...
                userInput.SelectStrongest, 'SelectStrongest');

            vision.internal.cnn.validation.checkMiniBatchSize(userInput.MiniBatchSize, mfilename);

            fastRCNNObjectDetector.checkStrongestRegions(userInput.NumStrongestRegions);

            wasMinSizeSpecified = ~ismember('MinSize', p.UsingDefaults);
            wasMaxSizeSpecified = ~ismember('MaxSize', p.UsingDefaults);

            if wasMinSizeSpecified
                vision.internal.detector.ValidationUtils.checkMinSize(userInput.MinSize, this.MinObjectSize, mfilename);
            else
                % set min size to model training size if not user specified.
                userInput.MinSize = this.MinObjectSize;
            end

            if wasMaxSizeSpecified
                vision.internal.detector.ValidationUtils.checkMaxSize(userInput.MaxSize, this.MinObjectSize, mfilename);
                % note: default max size set above in inputParser to size(I)
            end

            if wasMaxSizeSpecified && wasMinSizeSpecified
                % cross validate min and max size
                coder.internal.errorIf(any(userInput.MinSize >= userInput.MaxSize) , ...
                    'vision:ObjectDetector:minSizeGTMaxSize');
            end

            if useROI
                if ~isempty(userInput.roi)
                    sz = userInput.roi([4 3]);
                    vision.internal.detector.ValidationUtils.checkImageSizes(sz(1:2), userInput, wasMinSizeSpecified, ...
                        this.MinObjectSize, ...
                        'vision:ObjectDetector:ROILessThanMinSize', ...
                        'vision:ObjectDetector:ROILessThanModelSize');
                end
            else
                vision.internal.detector.ValidationUtils.checkImageSizes(sz(1:2), userInput, wasMinSizeSpecified, ...
                    this.MinObjectSize , ...
                    'vision:ObjectDetector:ImageLessThanMinSize', ...
                    'vision:ObjectDetector:ImageLessThanModelSize');
            end
            exeenv = vision.internal.cnn.validation.checkExecutionEnvironment(...
                userInput.ExecutionEnvironment, mfilename);

            fastRCNNObjectDetector.checkThreshold(userInput.Threshold);

            detOnBorder = fastRCNNObjectDetector.checkDetectionsOnBorder(userInput.DetectionsOnBorder);


            params.ROI                  = double(userInput.roi);
            params.UseROI               = useROI;
            params.NumStrongestRegions  = double(userInput.NumStrongestRegions);
            params.SelectStrongest      = logical(userInput.SelectStrongest);
            params.ImageSize            = sz;
            params.MiniBatchSize        = double(userInput.MiniBatchSize);
            params.MinSize              = double(userInput.MinSize);
            params.MaxSize              = double(userInput.MaxSize);
            params.ExecutionEnvironment = exeenv;
            params.Threshold            = double(userInput.Threshold);
            params.DetectionsOnBorder   = char(detOnBorder);
            params.InputSize            = networkInputSize;
        end

        %------------------------------------------------------------------
        function [dataMap, layerOut] = datastoreInputsForActivationsMIMO(this, inputSize)
            clsName = this.Network.Layers(this.LayerIndices.ClassificationLayerIdx).Name;
            regName = this.Network.Layers(this.LayerIndices.RegressionLayerIdx).Name;
            roiName = this.Network.Layers(this.LayerIndices.ROIInputLayerIdx).Name;

            layerOut = {clsName, regName, roiName};

            % There are two input layers: ImageInputLayer, ROIInputLayer.
            %  - the first column from the read output goes to the image input layer.
            %  - the second column from the read output goes to the ROI input layer.
            inputMapping = {1,2};
            % There are no outputs in the datastore.
            outputMapping = {};

            dataMap.DataTranslator = nnet.internal.cnn.util.DataTranslator( ...
                    inputMapping, outputMapping);
            dataMap.CollateFunctions.InputFunctions = {[], @iCollateROIInputs};
            dataMap.CollateFunctions.OutputFunctions = [];
            dataMap.InputSizes = {inputSize, []};
            dataMap.OutputSizes = {[]};
            dataMap.InputFormats = { deep.internal.PlaceholderArray([inputSize NaN],'SSCB'), ...
                deep.internal.PlaceholderArray([NaN 4 NaN],'SCB'); };
            dataMap.OutputFormats = {};
            dataMap.UnpackCellInputOutput = false;
        end
        
        %------------------------------------------------------------------
        function [ds, dataMap, clsName, regName] = inputsForActivationsMIMO(this,Iroi,scaledBBoxes)
            clsName = this.Network.Layers(this.LayerIndices.ClassificationLayerIdx).Name;
            regName = this.Network.Layers(this.LayerIndices.RegressionLayerIdx).Name;
            imgName = this.Network.Layers(this.LayerIndices.ImageLayerIdx).Name;
            roiName = this.Network.Layers(this.LayerIndices.ROIInputLayerIdx).Name;
            ds = vision.internal.cnn.fastrcnn.imageAndROIDatastore(Iroi,scaledBBoxes);
            dst = {
                imgName
                roiName
                };
            dataMap = table(ds.ColumnVar',dst,'VariableNames',{'Data','Destination'});
        end

        %-----------------------------------------------------------------------
        function params = updateParamsForPostProcess(this, params)
            params.BackgroundLabel     = this.BackgroundLabel;
            params.BoxMean             = this.BoxMean;
            params.BoxStd              = this.BoxStd;
            params.ClassNames          = this.ClassNames;
        end

    end
    
    %======================================================================
    % Save/Load
    %======================================================================
    methods(Hidden)
        function s = saveobj(this)
            s.Network                  = this.Network;
            s.RegionProposalFcn        = this.RegionProposalFcn;
            s.UsingDefaultRegionProposalFcn = this.UsingDefaultRegionProposalFcn;
            s.BackgroundLabel          = this.BackgroundLabel;
            s.MinObjectSize            = this.MinObjectSize;
            s.ModelName                = this.ModelName;
            s.BoxMean                  = this.BoxMean;
            s.BoxStd                   = this.BoxStd;
            s.RecomputeScaleFactor     = this.RecomputeScaleFactor;
            s.Version                  = 4.0;
        end
    end
    
    methods(Static, Hidden)
        function this = loadobj(s)
            this = fastRCNNObjectDetector();
            this = setPropertiesOnLoad(this, s);
        end
    end
    
    %----------------------------------------------------------------------
    % Shared parameter validation routines.
    %----------------------------------------------------------------------
    methods(Hidden, Static)
        
        %------------------------------------------------------------------
        function analysis = analyzeNetwork(network)
            analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(network);
            isClsLayer = [analysis.LayerAnalyzers.IsClassificationLayer];
            numClasses = analysis.LayerAnalyzers(isClsLayer).ExternalLayer.OutputSize-1; % exclude background class
            allowMultiChannel = true;
            constraints = vision.internal.cnn.RCNNLayers.constraints('fast-rcnn', numClasses, allowMultiChannel);
            analysis.applyConstraints(constraints);
            analysis.throwIssuesIfAny()
        end
        
        %------------------------------------------------------------------
        function checkRegionProposalFcn(func)
            
            validateattributes(func, {'function_handle'}, {'scalar'}, ...
                '', 'RegionProposalFcn');
            
            % get num args in/out. This errors out if func does not exist.
            numIn  = nargin(func);
            numOut = nargout(func);
            
            % functions may have varargin/out (i.e. anonymous functions)
            isVarargin  = (numIn  < 0);
            isVarargout = (numOut < 0);
            
            numIn  = abs(numIn);
            numOut = abs(numOut);
            
            % validate this API: [bboxes, scores] = func(I)
            if ~isVarargin && numIn ~= 1
                error(message('vision:rcnn:proposalFcnInvalidNargin'));
            end
            
            if ~isVarargout && numOut ~= 2
                error(message('vision:rcnn:proposalFcnInvalidNargout'));
            end
        end
        
        %------------------------------------------------------------------
        function sz = determineMinBoxSize(lgraph)
            % MinBox size is a function of the network architecture and the roi pooling
            % layer's grid size.
            
            analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(lgraph);
            externalLayers = [analysis.LayerAnalyzers.ExternalLayer];
            
            % figure out min box size that network can handle. get the
            % feature map size that will be input to the ROI pooling layer.
            roiIdx = arrayfun(@(x)isa(x,'nnet.cnn.layer.ROIMaxPooling2DLayer')|| ...
                isa(x,'vision.cnn.layer.ROIAveragePooling2DLayer'),externalLayers);
            featureMapSize = analysis.LayerAnalyzers(roiIdx).Inputs.Size{1};
            
            imageIndex = [analysis.LayerAnalyzers.IsImageInputLayer];
            inputSize = analysis.LayerAnalyzers(imageIndex).Outputs.Size{1};
            
            scaleFactor = inputSize(1:2)./featureMapSize(1:2);
            
            sz = ceil(scaleFactor);
            
        end
        
        %------------------------------------------------------------------
        function I = convertImageToMatchNumberOfNetworkImageChannels(I, imageSize)
            
            isNetImageRGB = numel(imageSize) == 3 && imageSize(end) == 3;
            isImageRGB    = ~ismatrix(I);
            
            if isImageRGB && ~isNetImageRGB
                I = rgb2gray(I);
                
            elseif ~isImageRGB && isNetImageRGB
                I = repmat(I,1,1,3);
            end
        end
        
        %------------------------------------------------------------------
        function bboxes = applyReg(P,reg, minSize, maxSize)
            x = reg(:,1);
            y = reg(:,2);
            w = reg(:,3);
            h = reg(:,4);
            
            % center of proposals
            px = P(:,1) + floor(P(:,3)/2);
            py = P(:,2) + floor(P(:,4)/2);
            
            % compute regression value of ground truth box
            gx = P(:,3).*x + px; % center position
            gy = P(:,4).*y + py;
            
            gw = P(:,3) .* exp(w);
            gh = P(:,4) .* exp(h);
            
            if nargin > 2
                % regression can push boxes outside user defined range. clip the boxes
                % to the min/max range. This is only done after the initial min/max size
                % filtering.
                gw = min(gw, maxSize(2));
                gh = min(gh, maxSize(1));
                
                % expand to min size
                gw = max(gw, minSize(2));
                gh = max(gh, minSize(1));
            end
            
            % convert to [x y w h] format
            bboxes = [ gx - floor(gw/2) gy - floor(gh/2) gw gh];
            
            bboxes = double(round(bboxes));
            
        end
        
        %------------------------------------------------------------------
        function bboxes = applyBoxRegression(P, reg, labels, minSize, maxSize, boxMean, boxStd)
            
            % reg is 4D array [1 1 numClasses*4 numObs]. reshape to
            % 4-by-numClasses-by-numObs
            numObservations = size(reg,4);
            reg = reshape(reg, 4, numel(categories(labels)), numObservations);
            
            idx = int32(labels);
            v = zeros(numObservations, 4, 'like', reg);
            for i = 1:numObservations
                v(i,:) = reg(:, idx(i), i)';
            end
            
            v = (v .* boxStd) + boxMean;
            
            bboxes = fastRCNNObjectDetector.applyReg(P, v, minSize, maxSize);
            
        end
        
        %------------------------------------------------------------------
        function  [bboxes, scores, labels] = removeInvalidBoxesAndScores(bboxes, scores, labels)
            % remove invalid boxes. An invalid box has:  
            %   * non positive width or height
            %   * inf or nan values
            %   * inf/nan scores
            %   
            % NB: boxes whose top-left corner outside the image boundary
            % are not considered invalid as these can be either clipped or
            % discarded. This is done as a separate step. 
            
            % Remove boxes with zero width/height.
            remove = bboxes(:,3) < 1 | bboxes(:,4) < 1;
            
            % Remove boxes with inf/nan.
            remove = remove | ~all(isfinite(bboxes),2);
            
            % Remove boxes with NaN scores.
            remove = remove | ~isfinite(scores);
            
            bboxes(remove,:) = [];
            scores(remove,:)  = [];
                        
            if nargin == 3
                labels(remove) = [];
            end
        end
        
        %------------------------------------------------------------------
        function [bboxes, scores, labels] = filterBoxesAfterRegression(bboxes, scores, labels, imageSize)
            % Remove boxes that don't make sense after regression:
            % * boxes with non-positive width and height.
            % * boxes that have zero overlap with image.
            
            [bboxes, scores, labels] = fastRCNNObjectDetector.removeInvalidBoxesAndScores(bboxes, scores, labels);
            
            % Remove boxes that are completely outside the image.
            x1 = bboxes(:,1);
            y1 = bboxes(:,2);
            x2 = bboxes(:,3) + x1 - 1;
            y2 = bboxes(:,4) + y1 - 1;
            
            boxOverlapsImage = ...
                (x1 < imageSize(2) & x2 > 1) & ...
                (y1 < imageSize(1) & y2 > 1);
            
            bboxes = bboxes(boxOverlapsImage,:);
            scores = scores(boxOverlapsImage,:);
            labels = labels(boxOverlapsImage,:);
            
        end
        
        %------------------------------------------------------------------
        function [bboxes, scores, labels] = discardBBoxOutsideImage(bboxes,scores,labels,imageSize)
            x1 = bboxes(:,1);
            y1 = bboxes(:,2);
            x2 = bboxes(:,3) + x1 - 1;
            y2 = bboxes(:,4) + y1 - 1;
            
            onBorder = (x1 < 1) | (y1 < 1) | (x2 > imageSize(2)) | (y2 > imageSize(1));
            bboxes(onBorder,:) = [];
            scores(onBorder) = [];
            labels(onBorder) = [];
        end
        
        %------------------------------------------------------------------
        function printHeader(printer, classNames)
            printer.print('*******************************************************************\n');
            printer.printMessage('vision:rcnn:fastTrainingHeader');
            printer.linebreak;

            for i = 1:numel(classNames)
                printer.print('* %s\n', classNames{i});
            end
            
            printer.linebreak;
            
        end
    end
    
    %----------------------------------------------------------------------
    methods(Hidden, Static, Access = private)
        %------------------------------------------------------------------
        function [ds, mapping, batchingFcn, params, options] = createTrainingDatastore(datastore, params, lgraph, options)

            params.RandomSelector = vision.internal.rcnn.RandomSelector();
            applyTransformFcn     = @(ds)vision.internal.cnn.fastrcnn.createTransformedImageCentricDatastore(...
                                                 ds, params);
            [ds,batchingFcn]      = applyTransformFcn(datastore);
            options               = vision.internal.cnn.validationReportUtils.updateValidationDataTransform(options,...
                                                 applyTransformFcn);
            [mapping,batchingFcn] = fastRCNNObjectDetector.createMIMODatastoreCellMapping(lgraph,batchingFcn, params.InputSize);
            % add regression mean/std to params struct
            params.BoxMean        = params.ImageInfo.BoxRegressionMean;
            params.BoxStd         = params.ImageInfo.BoxRegressionStd;

            % Iterate over the data until we find at least one non-empty
            % result. This is required for the four-step training in Faster
            % R-CNN where we training data is generated based on previous
            % stages. If we don't check this here, then other parts of the
            % training will break.
            data = cell.empty;
            while isempty(data) && hasdata(ds)
                data = read(ds);
            end
            if isempty(data)
                error(message('vision:rcnn:noTrainingSamples'));
            end
            
            % Reset datastore back to the start.
            reset(ds);

        end

        %------------------------------------------------------------------
        function [ds, mapping, batchingFcn, params] = createTrainingDatastoreFromTable(trainingData, params, lgraph)
            fcnCopy  = params.RegionProposalFcn;
            fcn      = @(x,filename)rcnnObjectDetector.invokeRegionProposalFcn(fcnCopy, x, filename);
            
            if params.ScaleImage
                imds = imageDatastore(trainingData{:,1}, 'ReadFcn', @(x)fastRCNNObjectDetector.scaleImage(x, params.ImageScale));
            else
                imds = imageDatastore(trainingData{:,1});
            end
            
            % Setup region proposal function. For parallel processing using
            % multiple MATLAB workers, copy the function handle before
            % assigning passing it to the extraction routine. This prevents
            % the fastRCNNObjectDetector object from being copied to all the
            % workers.
            [regionProposals, trainingData, validTrainingData] = rcnnObjectDetector.extractRegionProposals(fcn, imds, params, trainingData);
            
            rcnnObjectDetector.issueWarningIfRequired(validTrainingData);
            
            % create datastore
            %  dispatcher needs to scale roi depending on input image size for this it uses the network layers.
            params.RandomSelector = vision.internal.rcnn.RandomSelector();
            ds                    = vision.internal.cnn.fastrcnn.imageCentricRegionDatastore(...
                                        trainingData,regionProposals,params);
            batchingFcn           = ds.BatchingFcn;
            mapping               = fastRCNNObjectDetector.createMIMODatastoreMapping(ds.OutputTableVariableNames,lgraph);

            % add regression mean/std to params struct
            params.BoxMean        = ds.BoxRegressionMean;
            params.BoxStd         = ds.BoxRegressionStd;
        end

        function [roi, params] = parseClassifyInputs(I, roi, varargin)
            p = inputParser;
            p.addParameter('ExecutionEnvironment', 'auto');
            parse(p, varargin{:});
            
            userInput = p.Results;
            
            % grayscale or RGB images allowed
            vision.internal.inputValidation.validateImage(I, 'I');
            
            roi = fastRCNNObjectDetector.checkROIs(roi, size(I));
            
            exeenv = vision.internal.cnn.validation.checkExecutionEnvironment(...
                userInput.ExecutionEnvironment, mfilename);
            
            params.ExecutionEnvironment = exeenv;
        end
        
        %------------------------------------------------------------------
        function checkStrongestRegions(N)
            if isinf(N)
                % OK, use all regions.
            else
                validateattributes(N, ...
                    {'numeric'},...
                    {'scalar', 'real', 'positive', 'integer', 'nonempty', 'finite', 'nonsparse'}, ...
                    mfilename, 'NumStrongestRegions');
            end
        end
        
        %------------------------------------------------------------------
        function checkThreshold(threshold)
            validateattributes(threshold, {'single', 'double'}, {'nonempty', 'nonnan', ...
                'finite', 'nonsparse', 'real', 'scalar', '>=', 0, '<=', 1}, ...
                mfilename, 'Threshold');
        end
        
        %------------------------------------------------------------------
        function str = checkDetectionsOnBorder(str)
            str = validatestring(str,{'clip','discard'},mfilename,'DetectionsOnBorder');
        end
        
        %------------------------------------------------------------------
        function roi = checkROIs(roi, imageSize)
            validateattributes(roi, {'numeric'}, ...
                {'size', [NaN 4], 'real', 'nonsparse', 'finite', 'nonsparse'},...
                mfilename, 'roi');
            
            % rounds floats and casts to int32 to avoid saturation of smaller integer types.
            roi = vision.internal.detector.roundAndCastToInt32(roi);
            
            % width and height must be >= 0
            if any(roi(:,3) < 0) || any(roi(:,4) < 0)
                error(message('vision:validation:invalidROIWidthHeight'));
            end
            
            % roi must be fully contained within I
            if any(roi(:,1) < 1) || any(roi(:,2) < 1)...
                    || any(roi(:,1) + roi(:,3) > imageSize(2)+1) ...
                    || any(roi(:,2) + roi(:,4) > imageSize(1)+1)
                
                error(message('vision:validation:invalidROIValue'));
            end
            
        end
        
        %------------------------------------------------------------------
        function updateMessage(printer, prevMessage, nextMessage)
            backspace = sprintf(repmat('\b',1,numel(prevMessage))); % figure how much to delete
            printer.printDoNotEscapePercent([backspace nextMessage]);
        end
        
        %------------------------------------------------------------------
        function nextMessage = printProgress(printer, prevMessage, k, K)
            nextMessage = sprintf('%.2f%%%%',100*k/K);
            fastRCNNObjectDetector.updateMessage(printer, prevMessage(1:end-1), nextMessage);
        end
        
        %------------------------------------------------------------------
        function [mapping,batchingFcn] = createMIMODatastoreCellMapping(lgraph, batchingFcn, inputLayerSize)

            externalLayers = lgraph.Layers;
            % There 2 input layers: ImageInputLayer, ROIInputLayer.
            %  - the first column from the read output goes to the image input layer.
            %  - the second column from the read output goes to the ROI input layer.
            inputMapping = {1, 2};
            idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.ImageInputLayer'),...
                externalLayers);
            imageLayerIdx = find(idx,1,'first');
            idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.ROIInputLayer'),...
                externalLayers);
            roiInputLayerIdx = find(idx,1,'first');
            [~,inputOrdering]=sort([imageLayerIdx, roiInputLayerIdx]);
            inputMapping = inputMapping(inputOrdering);
            batchingFcn.InputFunctions = batchingFcn.InputFunctions(inputOrdering);

            % There are 2 output layers: ClassificationOutputLayer, RCNNBoxRegressionLayer.
            % RPN output layer needs image sizes and boxes.
            %    - the third column contains classification responses.
            %    - the fourth column contains regression responses.
            outputMapping = {3, 4};
            classificationOutputs = [true, false];

            % Find locations of each layer in Layers array.
            idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.ClassificationOutputLayer'),...
                externalLayers);
            classificationLayerIdx = find(idx,1,'last');

            idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.RCNNBoxRegressionLayer'),...
                externalLayers);
            regressionLayerIdx = find(idx,1,'last');
            [~,outputOrdering] = sort([classificationLayerIdx, regressionLayerIdx]);

            outputMapping = outputMapping(outputOrdering);
            classificationOutputs = classificationOutputs(outputOrdering);
            batchingFcn.OutputFunctions = batchingFcn.OutputFunctions(outputOrdering);

            batchingFcn = iUpdateCatAndSliceFunctions(batchingFcn,inputOrdering,outputOrdering);

            inputSizes = { inputLayerSize, [] };
            % Set outputSizes to placeholder values to enforce 4D format.
            outputSizes = { [1,1,1], [1,1,1] };
            inputFormats = { deep.internal.PlaceholderArray([inputLayerSize NaN],'SSCB'), deep.internal.PlaceholderArray([NaN 4 NaN],'SCB') };
            outputFormats = { deep.internal.PlaceholderArray([1 1 1 NaN],'SSCB'), deep.internal.PlaceholderArray([1 1 1 NaN],'SSCB') };

            mapping = {inputMapping, outputMapping, classificationOutputs, inputSizes, outputSizes, inputFormats, outputFormats};
        end

        %------------------------------------------------------------------
        function mapping = createMIMODatastoreMapping(outputTableVariableNames, lgraph)
            
            externalLayers = lgraph.Layers;
            
            % Find locations of each layer in Layers array.
            idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.ClassificationOutputLayer'),...
                externalLayers);
            layerIndices.ClassificationLayerIdx = find(idx,1,'last');
            
            idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.RCNNBoxRegressionLayer'),...
                externalLayers);
            layerIndices.RegressionLayerIdx = find(idx,1,'last');
            
            idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.ImageInputLayer'),...
                externalLayers);
            layerIndices.ImageLayerIdx = find(idx,1,'first');
            
            idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.ROIInputLayer'),...
                externalLayers);
            layerIndices.ROIInputLayerIdx = find(idx,1,'first');
            
            idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.ROIMaxPooling2DLayer') || isa(x,'vision.cnn.layer.ROIAveragePooling2DLayer') ,...
                externalLayers);
            layerIndices.ROIPoolingLayerIdx = find(idx,1,'first');
            
            % Get layer name based on location in Layers array.
            clsName = externalLayers(layerIndices.ClassificationLayerIdx).Name;
            regName = externalLayers(layerIndices.RegressionLayerIdx).Name;
            imgName = externalLayers(layerIndices.ImageLayerIdx).Name;
            roiName = externalLayers(layerIndices.ROIInputLayerIdx).Name;
            
            dst = {
                imgName
                roiName
                clsName
                regName
                };
            
            mapping = table(outputTableVariableNames',dst,...
                'VariableNames',{'Data','Destination'});
        end
        
    end
    
    methods(Hidden, Access = protected)
        function groups = getPropertyGroups( this )
            propertyList = struct;
            propertyList.ModelName = this.ModelName;
            propertyList.Network = this.Network;
            propertyList.RegionProposalFcn = this.RegionProposalFcn;
            % Transpose class names, which is a column vector
            propertyList.ClassNames = this.ClassNames';
            propertyList.MinObjectSize = this.MinObjectSize;
            
            groups = matlab.mixin.util.PropertyGroup(propertyList);
        end
    end
end

%--------------------------------------------------------------------------
function new = iUpdatePreviousVersions(prev)
new = prev;
if prev.Version < 3
    
    if prev.Version < 2
        % Previous versions did not standardize regression targets.
        new.BoxMean = zeros(1,4,'single');
        new.BoxStd = ones(1,4,'single');
    end
    
    % This prop will be set when Network is set.
    new.LayerIndices = [];
    
    % Detectors trained prior to 18b require computation of scale factor
    % if the image size changes.
    new.RecomputeScaleFactor = true;        
    
    if ~isempty(prev.Network)
        if ~isa(prev.Network,'DAGNetwork')
            frcnn = vision.cnn.FastRCNN.loadobj(prev.Network);
            lgraph = frcnn.toLayerGraph;
            new.Network = vision.internal.cnn.createDAGNetwork(lgraph);
        end
    end
end
end

%--------------------------------------------------------------------------
function layerIndices = iPopulateLayerIndices(layers)

idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.ClassificationOutputLayer'),...
    layers);
layerIndices.ClassificationLayerIdx = find(idx,1,'last');

idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.RCNNBoxRegressionLayer'),...
    layers);
layerIndices.RegressionLayerIdx = find(idx,1,'last');

idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.ImageInputLayer'),...
    layers);
layerIndices.ImageLayerIdx = find(idx,1,'first');

idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.ROIInputLayer'),...
    layers);
layerIndices.ROIInputLayerIdx = find(idx,1,'first');

idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.ROIMaxPooling2DLayer') || isa(x,'vision.cnn.layer.ROIAveragePooling2DLayer') ,...
    layers);
layerIndices.ROIPoolingLayerIdx = find(idx,1,'first');
end

%--------------------------------------------------------------------------
function trainingData = iScaleGroundTruth(trainingData, imageInfo,  imageScale, useParallel)
% Scale groundtruth if required.

% scale ground truth boxes. Images are scaled on-the-fly in the
% data dispatchers.
[trainingData, ~, boxesRemoved] = vision.internal.cnn.scaleGroundTruthBoxes(...
    trainingData, imageInfo.Sizes, imageScale, useParallel);

% Report which images had boxes removed because of scaling
if any(boxesRemoved)
    files = trainingData{boxesRemoved,1};
    files = sprintf('%s\n', files{:});
    vision.internal.cnn.WarningLogger.warning(message('vision:rcnn:boxesRemovedByScaling', files));
end
end

%-----------------------------------------------------------------------
function allBoxes = iCollateROIInputs(bboxes, ~)
    N = size(bboxes,1);
    allBoxes = cell(N,1);
    for i = 1:N
        boxes = bboxes{i};
        % Associate batch indices with each set of boxes.
        numBoxes = size(boxes,1);
        allBoxes{i} = [boxes repelem(i,numBoxes,1)];
    end
    allBoxes = vertcat(allBoxes{:});
end

%-----------------------------------------------------------------------
function out = iPreProcessForDatastoreRead(in,varargin)
    if isnumeric(in)
        % Numeric input
        in = {in};
    end
    if istable(in)
        % Table input
        in = in{:,1};
    else
        % Cell input
        in = in(:,1);
    end
    numItems = numel(in);
    out = cell(numItems, 2);
    for ii = 1:numItems
        [out{ii, 1}, out{ii,2}] = iPreprocess(in{ii},varargin{:});
    end
end

%-----------------------------------------------------------------------
function [Iroi, bboxes] = iPreprocessBatch(I, roi, useROI, colorPreprocessing, minSize, maxSize,...
        numStrongestRegions, regionProposalFcn, filterBboxesFunctor, imageSize, inputSize)
    sz = imageSize;
    sz(3) = inputSize(3);
    Iroi = zeros(sz,'like',I);
    bboxes = cell(imageSize(4),1);
    for ii = 1:imageSize(4)
        [Iroi(:,:,:,ii),bboxes{ii}] = iPreprocess(I(:,:,:,ii), roi, useROI, colorPreprocessing, minSize, maxSize,...
            numStrongestRegions, regionProposalFcn, filterBboxesFunctor);
        % Append image index to the bboxes that can be post
        % processed after getting the activations.
        bboxes{ii}(:,5) = ii;
    end
    bboxes = vertcat(bboxes{:});
end

%-----------------------------------------------------------------------
function [Iroi, bboxes] = iPreprocess(I, roi, useROI, colorPreprocessing, minSize, maxSize,...
        numStrongestRegions, regionProposalFcn, filterBboxesFunctor)
    % Crop image, if needed.
    Iroi = vision.internal.detector.cropImageIfRequested(I, roi, useROI);

    % Convert image from RGB <-> grayscale as required by network.
    Iroi = vision.internal.cnn.utils.convertImageToMatchNumberOfNetworkChannels({Iroi}, colorPreprocessing);
    Iroi = Iroi{1};

    % Run region proposal function.
    [bboxes, boxScores] = regionProposalFcn(Iroi);

    rcnnObjectDetector.checkRegionProposalOutputs(bboxes, boxScores);

    [bboxes, boxScores] = filterBBoxes(filterBboxesFunctor, minSize, maxSize, bboxes, boxScores);

    % Keep the top-N strongest object proposals.
    bboxes = rcnnObjectDetector.selectStrongestRegions(bboxes, boxScores, numStrongestRegions);
    % convert [x y w h] boxes from proposal function into
    % [x1 y1 x2 y2] format expected by ROI pooling layer.
    bboxes = vision.internal.cnn.boxUtils.xywhToX1Y1X2Y2(bboxes);
end

%-----------------------------------------------------------------------
function params = iUpdateImageSizeIfNeeded(inputImageOrDatastore,params)
    if isnumeric(inputImageOrDatastore)
        params.ImageSize = size(inputImageOrDatastore);
        return;
    end
    % We assume every image is of the same size.
    cpy = copy(inputImageOrDatastore);
    reset(cpy);
    data = read(cpy);
    if iscell(data)
        params.ImageSize = size(data{1,1});
    else
        params.ImageSize = size(data);
    end
end

%-----------------------------------------------------------------------
function batchFcn = iUpdateCatAndSliceFunctions(batchFcn,inputOrdering,outputOrdering)
    ordering.InputOrdering = inputOrdering;
    ordering.OutputOrdering = outputOrdering;

    % Pass input layer ordering and output layer ordering to the
    % fastRCNN CatAndSliceStrategy.
    batchFcn.CatAndSliceStrategy.Ordering = ordering;
end

%--------------------------------------------------------------------------
function m = iInfoToSummaryMapAndStartValues()
% Full info content resides in the validation content. Use that to build
% the map from info to summary fields. All fields are initialized to [].
% This is used for OutputFcn support.
infoContent = vision.internal.cnn.FastRCNNAndRPNValidationContent();
initialValue = repelem({[]},numel(infoContent.FieldNames),1);
m = [infoContent.FieldNames' infoContent.SummaryNames' initialValue];
end

function tf = iIsExecutionEnvironmentParallel(executionEnvironment)
tf = ismember(executionEnvironment, {'multi-gpu','parallel'});
end

% LocalWords:  Girshick Zitnick ECCV Ren Shaoqing roi bboxes truecolor grayscale IBatch Bbox
% LocalWords:  Multiclass bbox visiondata rois fmap DLayer RPN func nonsparse groundtruth
