classdef fasterRCNNObjectDetector < vision.internal.EnforceScalarHandle & matlab.mixin.CustomDisplay

% Copyright 2016-2023 The MathWorks, Inc.

    properties(GetAccess = public, SetAccess = public)
        % ModelName Name of the classification model. By default, the name
        %           is set by trainFasterRCNNObjectDetector. The name may
        %           be modified after training as desired.
        ModelName   char
    end
    
    properties(SetAccess = protected)
        % Network A DAGNetwork object representing the Faster R-CNN
        %         network.
        Network                      
    end
    
    properties(Dependent)        
        % AnchorBoxes An M-by-2 matrix defining the [height width] of M
        %             anchor boxes. 
        AnchorBoxes
        
        % ClassNames A cell array of object class names. These are the
        %            object classes that the Faster R-CNN detector was
        %            trained to find.
        ClassNames
    end
    
    properties(Dependent, Transient)
        % MinObjectSize Minimum object size supported by the Faster R-CNN
        %               network. The minimum size depends on the network
        %               architecture.
        MinObjectSize
    end
    
    properties(Hidden, SetAccess = private)                               
        % RegionProposalNetwork is not recommended. Use Network to get the
        % complete Faster R-CNN network.       
        RegionProposalNetwork
    end
    
    properties(Hidden, SetAccess = protected)
        
        % MinBoxSizes is not recommended. Use AnchorBoxes instead.
        %     
        % The minimum anchor box sizes. 
        MinBoxSizes
        
        % BoxPyramidScale is not recommended. Use AnchorBoxes instead.
        %
        % The scale factor used to successively upscale anchor box sizes.  
        BoxPyramidScale
        
        % NumBoxPyramidLevels is not recommended. Use AnchorBoxes instead.
        %
        % The number of levels in an anchor box pyramid. 
        NumBoxPyramidLevels
                
    end
    
    properties (Hidden)
        ModelSize
        TrainingStage
        
        % Mean and standard deviation values for box regression.
        BoxMean = [];
        BoxStd = [];
        RPNBoxMean = [];
        RPNBoxStd = [];       
    end
    
    properties(Access = private)
        % BackgroundLabel Label to use for background class. Default is
        % 'Background'.
        BackgroundLabel
        
        % LayerIndices A struct that caches indices to certain layers used
        %             frequently during detection.
        LayerIndices        
        
        % RecomputeScaleFactor Whether or not the ScaleFactor should be
        %                      recomputed during inference. This is true
        %                      for models trained prior to R2018b.         
        RecomputeScaleFactor = false;

        % SetImageSizeToEmptyFrom19b Whether or not the ImageSize should be
        %                            set to empty during inference. This is true
        %                            for models trained starting R2019b.
        SetImageSizeToEmptyFrom19b = true;

        % Partially trained networks for training checkpoints.
        PartiallyTrainedFastRCNN
        PartiallyTrainedRPN
        
        % struct that holds information from splitting and merging of Faster R-CNN
        % network.
        RCNNInfo
    end
       
    properties(Access = private, Dependent)
        % ScaleFactor ROI scale factor from image to feature space. This
        %             depends on the value stored in the network's ROI
        %             pooling layer.
        ScaleFactor
    end
    
    properties(Access = private, Transient)
        % A collection of layer properties cached by the detector. These
        % property values are shared by the detector and the layers.
        % Updating these properties updates those in the layers without
        % having to recreate the DAGNetwork.
        NumStrongestRegionsCache  
        ImageSizeCache
        ScaleFactorCache
        MinSizeCache
        MaxSizeCache
        ROIScaleFactorCache
    end
    
    methods        
        function sz = get.MinObjectSize(this)
            sz = this.ModelSize;
        end
        
        function val = get.RegionProposalNetwork(this)
            lgraph = layerGraph(this.Network);
            [~,val] = vision.internal.cnn.RCNNLayers.splitFasterIntoFastAndRPN(lgraph);                    
        end
        
        function val = get.ScaleFactor(this)
            assert(~isempty(this.LayerIndices));
            val = this.Network.Layers(this.LayerIndices.ROIPoolingLayerIdx).ScaleFactor;
        end
    end
    
    
    methods(Static, Access = public, Hidden)
        
        %------------------------------------------------------------------
        function [detector, lgraph, info] = trainRPN(ds, lgraph, opts, executionSettings, params, checkpointSaver)
            
            % Use network training options to control verbosity.
            params.Verbose = opts.Verbose;
            
            detector = checkpointSaver.Detector;
            detector.BoxPyramidScale = params.BoxPyramidScale;
            detector.NumBoxPyramidLevels = params.NumBoxPyramidLevels;
            detector.MinBoxSizes = params.MinBoxSizes;
            detector.ModelSize = params.ModelSize;             

            % Create the rpn training dispatcher
            params.Layers = lgraph.Layers;

            % Update box regression mean/std
            detector.RPNBoxMean = params.ImageInfo.BoxRegressionMean;
            detector.RPNBoxStd  =  params.ImageInfo.BoxRegressionStd;

            detector.Network = checkpointSaver.Detector.Network;
            
            % Update version in checkpoint saver.
            checkpointSaver.Detector = detector;
            
            if params.FreezeBatchNormalization
                lgraph = vision.internal.cnn.RCNNLayers.freezeBatchNorm(lgraph);
            end     
            axesConfigFactory = vision.internal.cnn.FasterRCNNAxesConfig();
            axesConfig = axesConfigFactory.AxesConfiguration;
            % Enable general datastore dispatching.
            %   - TransformedDatastore/CombinedDatastore or a custom datastore
            %     that can yield a Mx3 cell or a table with images, boxes and labels.
            try
                
                [rpn, info] = vision.internal.cnn.trainNetwork(...
                    ds, lgraph, opts, executionSettings, params.Mapping, checkpointSaver, ...
                    @(x,y)vision.internal.cnn.FastRCNNSummary.makeSummary(x,y,params.RCNNInfo.RPNProposalParameters),...
                    vision.internal.cnn.FastRCNNAndRPNContent(), ...
                    vision.internal.cnn.ClassificationRegressionColumns(), ...
                    axesConfig, ...
                    params.BatchingFunctions);
            catch ME
                if strcmp(ME.identifier,'nnet_cnn:internal:cnn:GeneralDatastoreDispatcher:VariableInputSizes')
                    error(message('vision:rcnn:unableToBatchImages'));
                else
                    rethrow(ME);
                end
            end
            
            if params.FreezeBatchNormalization
                lgraph = layerGraph(rpn);
                lgraph = vision.internal.cnn.RCNNLayers.unfreezeBatchNorm(lgraph);
                rpn = vision.internal.cnn.createDAGNetwork(lgraph);
            end
            
            % Store partially trained RPN network.
            
            detector.PartiallyTrainedRPN = rpn;
            detector.RCNNInfo = params.RCNNInfo;
            
            % Return trained network as layer graph
            lgraph = layerGraph(rpn);
            
        end

        function detector = detectorCheckpointEndToEnd(net, detector, info, freezeBatchNorm, classNames)
            
            if freezeBatchNorm
                % Unfreeze prior to creating detector checkpoint so that
                % internal "frozen" batch norm layer is not serialized.
                lgraph = layerGraph(net);
                lgraph = vision.internal.cnn.RCNNLayers.unfreezeBatchNorm(lgraph);
                net = vision.internal.cnn.createDAGNetwork(lgraph);
            end
                        
            % Revert the modifications made for end-to-end training.
            net = vision.internal.cnn.RCNNLayers.revertModificationForEndToEndTraining(net, info, classNames);
            
            detector.Network = net;
            
        end
        
        %------------------------------------------------------------------
        % Detector checkpoint function specialized for either RPN or Fast
        % R-CNN networks. This is required for use of alternating training
        % scheme.
        %------------------------------------------------------------------
        function detector = detectorCheckpoint(net, detector, trainingStage, freezeBatchNorm)
            
            detector.TrainingStage = trainingStage;
            
            if freezeBatchNorm
                % Unfreeze prior to creating detector checkpoint so that
                % internal "frozen" batch norm layer is not serialized.
                lgraph = layerGraph(net);
                lgraph = vision.internal.cnn.RCNNLayers.unfreezeBatchNorm(lgraph);
                net = vision.internal.cnn.createDAGNetwork(lgraph);
            end
            
            switch trainingStage
                case {1,3}
                    detector.PartiallyTrainedRPN = net;
                case {2,4}
                    detector.PartiallyTrainedFastRCNN = net;                    
            end
        end                        
    end
    
    %----------------------------------------------------------------------
    methods (Hidden)
        
        function [bboxes, scores] = propose(this, I, minBoxSize, varargin)
            % Used in alternate training scheme to produce region proposals
            % for training Fast R-CNN.

            params = fasterRCNNObjectDetector.parseProposeInputs(varargin{:});
            
            imageSize = size(I);
                                    
            params.RPNBoxStd    = this.RPNBoxStd;
            params.RPNBoxMean   = this.RPNBoxMean;
            params.ImageSize    = imageSize;
            params.MinSize      = minBoxSize;
            params.MaxSize      = [inf inf];
            params.BoxFilterFcn = @(a,b,c,~)fastRCNNObjectDetector.filterSmallBBoxes(a, b, c);                       
                 
            clsName = this.RCNNInfo.ProposalInputLayers.ClassificationLayerName;
            regName = this.RCNNInfo.ProposalInputLayers.RegressionLayerName;
            [cls, reg] = this.PartiallyTrainedRPN.activationsMIMO(I,...
                {clsName,regName}, ...
                'ExecutionEnvironment', params.ExecutionEnvironment,...
                'MiniBatchesInCell', true);
            
            reg = reg{1};
            cls = cls{1};
            
            % Create region proposal layer.
            proposalLayer = nnet.internal.cnn.layer.RegionProposalLayer(...
                '', this.RCNNInfo.AnchorBoxes, params);
            
            % Propose regions.
            bboxes = proposalLayer.predict({cls,reg});
                       
            % Proposal layer returns boxes in [x1 x2 y1 y2 idx] format. The
            % fast r-cnn trainer assumes propose function returns boxes in
            % [x y w h] format. Remove the batch idx.
            bboxes = bboxes(:,1:4);
            
            % Convert proposal layer boxes into [x y w h].            
            bboxes = vision.internal.cnn.boxUtils.x1y1x2y2ToXYWH(bboxes);                      
            
            % Return dummy score for each box. Proposal layer's predict
            % already returns the strongest N proposals. The output scores
            % is required by Fast R-CNN implementation and only required
            % for 4-step alternate training method.
            scores = ones(size(bboxes,1),1,'single');
            
        end
    end
    
    methods
        function varargout = detect(this, I, varargin)
            %DETECT Detect objects within the images using fasterRCNN object detector.
            %
            %   bboxes = detect(fasterRCNN,I) detects objects within the image I.
            %   The location of objects within I are returned in bboxes, an
            %   M-by-4 matrix defining M bounding boxes. Each row of bboxes
            %   contains a four-element vector, [x, y, width, height]. This
            %   vector specifies the upper-left corner and size of a bounding
            %   box in pixels. fasterRCNN is a fasterRCNNObjectDetector object
            %   and I is a truecolor or grayscale image.
            %
            %   bboxes = detect(fasterRCNN,IBatch) detects objects within each
            %   image contained in the batch of images IBatch. IBatch is a
            %   numeric array containing images in the format
            %   H-by-W-by-C-by-B, where B is the number of images in the
            %   batch, and C is the channel size. For grayscale images, C must be
            %   1. The size of each image in the batch, H-by-W-by-C, must be
            %   greater than or equal to the network input size. bboxes is a
            %   B-by-1 cell array, containing M-by-4 matrices for each image
            %   in the batch.
            %
            %   [..., scores] = detect(fasterRCNN,I) optionally return the detection
            %   scores for each bounding box. The score for each detection is
            %   the output of the softmax classifier used in the
            %   rcnn.Network. The range of the score is [0 1]. Larger score
            %   values indicate higher confidence in the detection. scores is a
            %   B-by-1 cell array, if the input I is a batch of images in the
            %   format H-by-W-by-C-by-B.
            %
            %   [..., labels] = detect(fasterRCNN,I) optionally return the labels
            %   assigned to the bounding boxes in an M-by-1 categorical
            %   array. The labels used for object classes is defined during
            %   training using the trainFasterRCNNObjectDetector function.
            %   labels is a B-by-1 cell array, if the input I is a batch of
            %   images in the format H-by-W-by-C-by-B.
            %
            %   detectionResults = detect(fasterRCNN,DS) detects objects within the
            %   series of images returned by the read method of datastore, DS. DS, can
            %   be a datastore that returns a table or a cell array with the first
            %   column containing images. detectionResults is a 3-column table with
            %   variable names 'Boxes', 'Scores', and 'Labels' containing bounding
            %   boxes, scores, and the labels. The location of objects within an image,
            %   I are returned in bounding boxes, an M-by-4 matrix defining M bounding
            %   boxes. Each row of boxes contains a four-element vector, [x, y, width,
            %   height]. This vector specifies the upper-left corner and size of a
            %   bounding box in pixels. fasterRCNN is a fasterRCNNObjectDetector
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
            %                          Default: fasterRCNN.MinObjectSize
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
            %   'MiniBatchSize'        The mini-batch size used for processing a
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
            %                          run the Faster R-CNN detector. Valid values for
            %                          resource are:
            %
            %                          'auto' - Use a GPU if it is available, otherwise
            %                                   use the CPU.
            %
            %                          'gpu'  - Use the GPU. To use a GPU, you must have
            %                                   Parallel Computing Toolbox(TM), and a
            %                                   CUDA-enabled NVIDIA GPU with compute
            %                                   capability 3.0 or higher. If a suitable
            %                                   GPU is not available, an error message
            %                                   is issued.
            %
            %                          'cpu'  - Use the CPU.
            %
            %                          Default: 'auto'
            %
            %   Notes:
            %   -----
            %   - When 'SelectStrongest' is true the selectStrongestBboxMulticlass
            %     function is used to eliminate overlapping boxes. By
            %     default, the function is called as follows:
            %
            %        selectStrongestBboxMulticlass(bbox, scores, labels,...
            %                              'OverlapThreshold', 0.5);
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
            %   % Load pre-trained vehicle detector.
            %   data = load('fasterRCNNVehicleTrainingData.mat');
            %
            %   imageFilenames = fullfile(toolboxdir('vision'),'visiondata', ...
            %       data.vehicleTrainingData.imageFilename);
            %
            %   ds = imageDatastore(imageFilenames);
            %
            %   % Resize images for uniform size.
            %   ds = transform(ds, @(x)imresize(x,[128,128]));
            %
            %   % Run detector.
            %   detector = data.detector;
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
            %   See also trainFasterRCNNObjectDetector,
            %          selectStrongestBboxMulticlass.

            params = this.parseDetectInputs(I, varargin{:});

            roi       = params.ROI;
            useROI    = params.UseROI;
            colorPreprocessing = vision.internal.cnn.utils.colorPreprocessingForImageInputSize(params.InputSize);
            if params.DetectionInputWasDatastore
                nargoutchk(0,1);

                % Copy and reset the given datastore, so external state events are
                % not reflected.
                ds = copy(I);
                reset(ds);

                if params.MiniBatchSize == 1 && ~this.SetImageSizeToEmptyFrom19b
                    % Before R2019b, we set the image size on to the network using the
                    % input image's size. Loop detect one image at a time.

                    varargout{1} = detectOneImageAtATime(this, ds, varargin{:});
                    return;
                end

                fcn = @iPreprocess;
                % We need just the preprocessed image -> num arg out is 1.
                fcnArgOut = 1;
                ds = transform(ds, @(x)iPreProcessForDatastoreRead(x,fcn,fcnArgOut,roi,useROI,colorPreprocessing));
                params = iUpdateImageSizeIfNeeded(ds,params);
                updateNetworkIfRequired(this,params.ImageSize,params);
                [dataMap, layerOut] = datastoreInputsForActivationsMIMO(this,params.InputSize);
                params = updateParamsForPostProcess(this, params);
                varargout{1} = vision.internal.cnn.fastrcnn.detectUsingDatastore(ds, this.Network, dataMap, layerOut, params);
            else
                nargoutchk(0,3);
                if params.DetectionInputWasBatchOfImages
                    Iroi = iPreprocessBatch(I, roi, useROI, colorPreprocessing, params);
                else
                    Iroi = iPreprocess(I, roi, useROI, colorPreprocessing);
                end
                params = iUpdateImageSizeIfNeeded(Iroi,params);
                updateNetworkIfRequired(this,params.ImageSize,params);

                [clsName,regName,proposalLayerName] = this.inputsForActivationsMIMO();
                [fmap, reg, bboxes] = this.Network.activationsMIMO(Iroi,...
                    {clsName,regName,proposalLayerName}, ...
                    'ExecutionEnvironment', params.ExecutionEnvironment,...
                    'MiniBatchesInCell', true);

                postProcessParams = updateParamsForPostProcess(this, params);
                if params.DetectionInputWasBatchOfImages
                    numFiles = params.ImageSize(4);
                    numBatches = 1;
                    [varargout{1:nargout}] = vision.internal.cnn.fastrcnn.postProcessBatchActivations(...
                        fmap,reg,bboxes,numFiles,numBatches,postProcessParams);
                else
                    [varargout{1:nargout}] = fastRCNNObjectDetector.postProcessActivations(...
                        fmap{1}, reg{1}, bboxes{1}, postProcessParams);
                end
            end
        end
    end

    methods (Access = private)
        function tbl = detectOneImageAtATime(this, ds, varargin)
            % Before R2019b, we set the image size on to the network using the
            % input image's size. Loop detect one image at a time.

            fcn = @this.detect;
            % We need bboxes, scores and labels -> num arg out is 3.
            fcnArgOut = 3;

            %TODO: Amortized pre-allocation and use UseParallel options.
            out = {};
            ii  = 1;

            while hasdata(ds)
                data = read(ds);
                out{ii} = iPreProcessForDatastoreRead(data,fcn,fcnArgOut,varargin{:});
                ii = ii + 1;
            end

            out = vertcat(out{:});
            varNames = {'Boxes', 'Scores', 'Labels'};
            tbl = table(out(:,1), out(:,2), out(:,3), 'VariableNames', varNames);
        end
    end

    methods
        
        %------------------------------------------------------------------
        function this = fasterRCNNObjectDetector(varargin)
            if nargin == 0
                this.BackgroundLabel = 'Background';
                this.TrainingStage = 0; % untrained
            elseif nargin == 1
                
                clsname = 'fasterRCNNObjectDetector';
                
                validateattributes(varargin{1},{clsname}, ...
                    {'scalar'}, mfilename);
                
                if isequal(class(varargin{1}), clsname)
                    this = setPropertiesOnLoad(this, saveobj(varargin{1}));
                end
            end
        end
        
        %------------------------------------------------------------------
        function set.Network(this, network)
            if ~isempty(network)
                
                validateattributes(network, ...
                    {'nnet.cnn.LayerGraph',...
                    'SeriesNetwork',...
                    'DAGNetwork'},{'scalar'});
                                                                   
                network = this.syncROIPoolCache(network);
                network = this.syncRegionProposalCache(network);
                                
                % update layer index cache
                this.setLayerIndices(network);                               
                
                this.Network = network;
                
                % update internal network parameters.
                imageSize = this.Network.Layers(this.LayerIndices.ImageLayerIdx).InputSize; %#ok<MCSUP>
                s.MinSize = [1 1];
                s.MaxSize = [inf inf];
                s.NumStrongestRegions = 2000;
                this.updateNetworkIfRequired(imageSize, s);
            else
                this.Network = [];
            end
            
            
        end
        
        %------------------------------------------------------------------
        function cls = get.ClassNames(this)
            if ~isempty(this.Network)                
                cls = this.Network.Layers(this.LayerIndices.ClassificationLayerIdx).ClassNames;                
            else
                cls = {};
            end
        end
        
        %------------------------------------------------------------------
        function val = get.AnchorBoxes(this)
            if ~isempty(this.Network) || ~isempty(this.RCNNInfo)
                if isempty(this.Network)
                    val = this.RCNNInfo.AnchorBoxes;
                else
                    % Get the anchor boxes from the region proposal layer.
                    idx = this.LayerIndices.ProposalLayerIdx;
                    val = this.Network.Layers(idx).AnchorBoxes;
                end
            else
                val = [];
            end
        end
    end
    
    methods (Access = protected)
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
            if ~ismember(this.BackgroundLabel, network.Layers(end).ClassNames)
                error(message('vision:rcnn:missingBackgroundClass'));
            end
        end
        
        %------------------------------------------------------------------
        function [bboxes, scores] = filterBBoxes(~, bboxes, scores, minSize, maxSize)
            [bboxes, scores] = fasterRCNNObjectDetector.filterBBoxesBySize(bboxes, scores, minSize, maxSize);
        end
        
        %------------------------------------------------------------------
        function params = parseDetectInputs(this, I, varargin)
            % image should be bigger network image input layer. This
            % ensures the feature map sizes are large enough to perform RPN
            % processing and ROI Pooling.

            if this.TrainingStage > 0 && this.TrainingStage < 5
                error(message('vision:rcnn:partiallyTrainedDetector'));
            end

            params.DetectionInputWasDatastore = ~isnumeric(I);

            if ~params.DetectionInputWasDatastore
                sampleImage = I;
            else
                sampleImage = vision.internal.cnn.validation.checkDetectionInputDatastore(I, mfilename);
            end

            network = this.Network;

            networkInputSize = network.Layers(this.LayerIndices.ImageLayerIdx).InputSize;

            validateChannelSize = false;
            validateImageSize   = true; % faster R-CNN cannot support images smaller than input size
            [sz,params.DetectionInputWasBatchOfImages] = vision.internal.cnn.validation.checkDetectionInputImage(...
                networkInputSize,sampleImage,validateChannelSize,validateImageSize);

            defaults = iDefaultDetectionParams();

            p = inputParser;
            p.addOptional('roi', defaults.roi);
            p.addParameter('NumStrongestRegions', defaults.NumStrongestRegions);
            p.addParameter('NumStrongestRegionsBeforeProposalNMS', defaults.NumStrongestRegionsBeforeProposalNMS);
            p.addParameter('SelectStrongest', defaults.SelectStrongest);
            p.addParameter('MiniBatchSize', defaults.MiniBatchSize);
            p.addParameter('MinSize', defaults.MinSize);
            p.addParameter('MaxSize', sz(1:2));
            p.addParameter('ExecutionEnvironment', defaults.ExecutionEnvironment);
            p.addParameter('ProposalOverlapThreshold', defaults.ProposalOverlapThreshold);
            p.addParameter('Threshold', defaults.Threshold)
            p.addParameter('DetectionsOnBorder', defaults.DetectionsOnBorder);
            parse(p, varargin{:});

            userInput = p.Results;

            useROI = ~ismember('roi', p.UsingDefaults);

            if useROI
                vision.internal.detector.checkROI(userInput.roi, sz);
            end

            vision.internal.inputValidation.validateLogical(...
                userInput.SelectStrongest, 'SelectStrongest');

            vision.internal.cnn.validation.checkMiniBatchSize(userInput.MiniBatchSize, mfilename);

            wasMinSizeSpecified = ~ismember('MinSize', p.UsingDefaults);
            wasMaxSizeSpecified = ~ismember('MaxSize', p.UsingDefaults);

            if wasMinSizeSpecified
                vision.internal.detector.ValidationUtils.checkMinSize(userInput.MinSize, this.ModelSize, mfilename);
            else
                % set min size to model training size if not user specified.
                userInput.MinSize = this.ModelSize;
            end

            if wasMaxSizeSpecified
                vision.internal.detector.ValidationUtils.checkMaxSize(userInput.MaxSize, this.ModelSize, mfilename);
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
                        this.ModelSize, ...
                        'vision:ObjectDetector:ROILessThanMinSize', ...
                        'vision:ObjectDetector:ROILessThanModelSize');
                end
            else
                vision.internal.detector.ValidationUtils.checkImageSizes(sz(1:2), userInput, wasMinSizeSpecified, ...
                    this.ModelSize , ...
                    'vision:ObjectDetector:ImageLessThanMinSize', ...
                    'vision:ObjectDetector:ImageLessThanModelSize');
            end

            fasterRCNNObjectDetector.checkStrongestRegions(userInput.NumStrongestRegions);

            fasterRCNNObjectDetector.checkStrongestRegions(userInput.NumStrongestRegionsBeforeProposalNMS);

            fasterRCNNObjectDetector.checkThreshold(userInput.Threshold);

            detOnBorder = fasterRCNNObjectDetector.checkDetectionsOnBorder(userInput.DetectionsOnBorder);

            exeenv = vision.internal.cnn.validation.checkExecutionEnvironment(...
                userInput.ExecutionEnvironment, mfilename);

            fasterRCNNObjectDetector.checkOverlapThreshold(userInput.ProposalOverlapThreshold)

            params.ROI                      = double(userInput.roi);
            params.UseROI                   = useROI;
            params.NumStrongestRegions      = double(userInput.NumStrongestRegions);
            params.NumStrongestRegionsBeforeProposalNMS = double(userInput.NumStrongestRegionsBeforeProposalNMS);
            params.SelectStrongest          = logical(userInput.SelectStrongest);
            params.ImageSize                = sz;
            params.MiniBatchSize            = double(userInput.MiniBatchSize);
            params.MinSize                  = double(userInput.MinSize);
            params.MaxSize                  = double(userInput.MaxSize);
            params.ProposalOverlapThreshold = double(userInput.ProposalOverlapThreshold);
            params.ExecutionEnvironment     = exeenv;
            params.Threshold                = double(userInput.Threshold);
            params.DetectionsOnBorder       = char(detOnBorder);
            params.ScaleFactor              = [1 1]; % scale factor for ROIs
            params.InputSize                = networkInputSize;
        end

        %------------------------------------------------------------------
        function updateNetworkIfRequired(this, imageSize, params)
            % User settable parameters during prediction. If any of these
            % change, update cached values.
            paramList = [              
                "NumStrongestRegions"
                "MinSize"
                "MaxSize"
                "ImageSize"
                ];
            params.ImageSize = imageSize;
            layer = this.Network.Layers(this.LayerIndices.ProposalLayerIdx);
            needsUpdate = false;

            for i = 1:numel(paramList)
                if ~isequal(layer.(paramList(i)),params.(paramList(i)))
                    needsUpdate = true;
                    break;
                end
            end
                    
            if needsUpdate                                                   
                isImageSizeDifferent = isempty(this.ImageSizeCache.Value) || ...
                    ~isequal(params.ImageSize(1:2),this.ImageSizeCache.Value(1:2));
                
                if this.RecomputeScaleFactor && isImageSizeDifferent
                    % Recompute scale factor to maintain backward
                    % compatibility with models train in previous releases
                    % (<= 18a).                                
                    analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(this.Network);
                    
                    scaleFactor = vision.internal.cnn.RCNNLayers.imageToFeatureScaleGivenImageSize(analysis,imageSize);     
                    
                    % update cache value for roi pooling layer.
                    this.ROIScaleFactorCache.Value = scaleFactor;
                else
                    scaleFactor = this.ScaleFactor;
                end
                                
                % update cache values. These directly update the values
                % used in the region proposal layer.
                this.NumStrongestRegionsCache.Value = params.NumStrongestRegions;
                this.ScaleFactorCache.Value = 1./scaleFactor;
                this.MinSizeCache.Value = params.MinSize;
                this.MaxSizeCache.Value = params.MaxSize;

                if ~this.SetImageSizeToEmptyFrom19b && ...
                        (~isfield(params, 'DetectionInputWasDatastore') || ~params.DetectionInputWasDatastore)
                    % For backward compatibility, before R2019a we want to set the image size cache
                    % value, only when the input is not a datastore.

                    % Set the image size as the input value
                    this.ImageSizeCache.Value = params.ImageSize;
                else
                    % From R2019b, making the imageSize cache value empty.
                    % This would trigger to use the feature size and the scale factor
                    % in the proposal layer.
                    this.ImageSizeCache.Value = [];
                end
            end
  
        end
        
        %------------------------------------------------------------------
        function setLayerIndices(this, network)
            idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.ClassificationOutputLayer'),...
                network.Layers);
            this.LayerIndices.ClassificationLayerIdx = find(idx,1,'last');
            
            idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.ROIMaxPooling2DLayer') || isa(x,'vision.cnn.layer.ROIAveragePooling2DLayer') ,...
                network.Layers);
            this.LayerIndices.ROIPoolingLayerIdx = find(idx,1,'first');
            
            % need to find regression layer in Fast branch.
            idx = fasterRCNNObjectDetector.findFastBoxRegressionLayer(network,...
                network.Layers(this.LayerIndices.ROIPoolingLayerIdx).Name);
            this.LayerIndices.RegressionLayerIdx = idx;
            
            idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.ImageInputLayer'),...
                network.Layers);
            this.LayerIndices.ImageLayerIdx = find(idx,1,'first');
            
            idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.RegionProposalLayer'),...
                network.Layers);
            this.LayerIndices.ProposalLayerIdx = find(idx,1,'first');
        end               
        
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
        
        function network = syncRegionProposalCache(this, network)            
            
            % Used cached property values in layer. This allows us to
            % change the value of the cached property and have it
            % automatically update in the layer.
            this.ScaleFactorCache         = vision.internal.cnn.layer.util.PropertyCache(); 
            this.ImageSizeCache           = vision.internal.cnn.layer.util.PropertyCache(); 
            this.MinSizeCache             = vision.internal.cnn.layer.util.PropertyCache(); 
            this.MaxSizeCache             = vision.internal.cnn.layer.util.PropertyCache(); 
            this.NumStrongestRegionsCache = vision.internal.cnn.layer.util.PropertyCache(); 
            
            params = iDefaultDetectionParams();
            
            proposalParams.ProposalsOutsideImage = 'clip';
            proposalParams.OverlapThreshold = params.ProposalOverlapThreshold;
            proposalParams.MinScore = 0;          
            proposalParams.NumStrongestRegionsBeforeProposalNMS = params.NumStrongestRegionsBeforeProposalNMS;
            proposalParams.BoxFilterFcn = @this.filterBBoxes;
            proposalParams.RPNBoxStd   = this.RPNBoxStd;
            proposalParams.RPNBoxMean  = this.RPNBoxMean;
            proposalParams.ScaleFactor = this.ScaleFactorCache;
            proposalParams.ImageSize   = this.ImageSizeCache;
            proposalParams.MinSize     = this.MinSizeCache;
            proposalParams.MaxSize     = this.MaxSizeCache;
            proposalParams.NumStrongestRegions = this.NumStrongestRegionsCache;
            
            lgraph = layerGraph(network);            
            
            lgraph = nnet.internal.cnn.layer.RegionProposalLayer.updateParameters(lgraph, proposalParams);
                    
            network = vision.internal.cnn.createDAGNetwork(lgraph);
                        
        end
        
        %------------------------------------------------------------------
        function [dataMap, layerOut] = datastoreInputsForActivationsMIMO(this, inputSize)
            clsName = this.Network.Layers(this.LayerIndices.ClassificationLayerIdx).Name;
            regName = this.Network.Layers(this.LayerIndices.RegressionLayerIdx).Name;
            proposalName = this.Network.Layers(this.LayerIndices.ProposalLayerIdx).Name;

            layerOut = {clsName, regName, proposalName};

            % There is one input layer: ImageInputLayer.
            %  - the first column from the read output goes to the input layer.
            inputMapping = {1};
            % There are no outputs in the datastore.
            outputMapping = {};

            dataMap.DataTranslator = nnet.internal.cnn.util.DataTranslator( ...
                    inputMapping, outputMapping);
            dataMap.CollateFunctions = [];
            dataMap.InputSizes = {inputSize};
            dataMap.OutputSizes = {[]};
            dataMap.InputFormats = { deep.internal.PlaceholderArray([inputSize NaN],'SSCB') };
            dataMap.OutputFormats = { };

            dataMap.UnpackCellInputOutput = false;
        end
        
        %------------------------------------------------------------------
        function [clsName, regName, proposalName] = inputsForActivationsMIMO(this)
            clsName = this.Network.Layers(this.LayerIndices.ClassificationLayerIdx).Name;
            regName = this.Network.Layers(this.LayerIndices.RegressionLayerIdx).Name;
            proposalName = this.Network.Layers(this.LayerIndices.ProposalLayerIdx).Name;
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
            s.BackgroundLabel          = this.BackgroundLabel;
            s.ModelSize                = this.ModelSize;
            s.TrainingStage            = this.TrainingStage;
            s.ModelName                = this.ModelName;
            s.BoxMean                  = this.BoxMean;
            s.BoxStd                   = this.BoxStd;
            s.RPNBoxMean               = this.RPNBoxMean;
            s.RPNBoxStd                = this.RPNBoxStd;
            s.PartiallyTrainedRPN      = this.PartiallyTrainedRPN;
            s.PartiallyTrainedFastRCNN = this.PartiallyTrainedFastRCNN; 
            s.RCNNInfo                 = this.RCNNInfo;            
            s.Version                  = 5.0;

            s.RecomputeScaleFactor       = this.RecomputeScaleFactor;
            s.SetImageSizeToEmptyFrom19b = this.SetImageSizeToEmptyFrom19b;
        end
        
        %------------------------------------------------------------------
        function this = setPropertiesOnLoad(this, s)
            try
                vision.internal.requiresNeuralToolbox(mfilename);
                
                s = iLoadPreviousVersion(s);
                
                this.BackgroundLabel   = s.BackgroundLabel;
                                              
                this.ModelSize             = s.ModelSize;
                this.TrainingStage         = s.TrainingStage;
                this.ModelName             = s.ModelName;
                this.BoxMean               = s.BoxMean;
                this.BoxStd                = s.BoxStd;
                this.RPNBoxMean            = s.RPNBoxMean;
                this.RPNBoxStd             = s.RPNBoxStd;
                this.RCNNInfo              = s.RCNNInfo;               
                
                % Backward compatibility support.
                this.RecomputeScaleFactor       = s.RecomputeScaleFactor;
                this.SetImageSizeToEmptyFrom19b = s.SetImageSizeToEmptyFrom19b;
                
                % Set networks after other properties are set because of
                % the need to sync detector and layer properties.
                if isempty(s.Network)
                    this.PartiallyTrainedRPN = s.PartiallyTrainedRPN;
                    this.PartiallyTrainedFastRCNN = s.PartiallyTrainedFastRCNN;
                else
                    this.Network = s.Network;
                end
   
            catch ME
                rethrow(ME)
            end
        end
        
        %------------------------------------------------------------------
        function [frcnn,rpn,info] = getNetworksForAlternateTraining(this)
            if this.TrainingStage > 0 && this.TrainingStage < 5
                % return partially trained networks as layerGraph objects
                % for training.
                if isa(this.PartiallyTrainedFastRCNN,'DAGNetwork')
                    frcnn = layerGraph(this.PartiallyTrainedFastRCNN);
                else
                    frcnn = this.PartiallyTrainedFastRCNN;
                end
                
                if isa(this.PartiallyTrainedRPN,'DAGNetwork')
                    rpn = layerGraph(this.PartiallyTrainedRPN);
                else
                    rpn = this.PartiallyTrainedRPN;
                end
                
                info = this.RCNNInfo;
            else               
                lgraph = layerGraph(this.Network);
                [frcnn,rpn,info] = vision.internal.cnn.RCNNLayers.splitFasterIntoFastAndRPN(lgraph);               
            end            
        end
    end
    
    methods(Static, Hidden)
        function this = loadobj(s)
            this = fasterRCNNObjectDetector();
            this = setPropertiesOnLoad(this, s);
        end
    end
    
    %----------------------------------------------------------------------
    % Shared parameter validation routines.
    %----------------------------------------------------------------------
    methods(Hidden, Static)

        %------------------------------------------------------------------
        function mapping = createRPNMIMODatastoreCellMapping(inputLayerSize)
            % There is one input layer: ImageInputLayer.
            %  - the first column from the read output goes to the input layer.
            inputMapping = {1};
            % There is one output layer: RPNOutputLayer.
            % RPN output layer needs image sizes and boxes.
            %    - the second column contains image sizes.
            %    - the third column contains ground truth boxes.
            outputMapping = {[2, 3]};
            % RPN Output layer is not a classification layer, but a regression layer.
            classificationOutputs = false;

            inputSizes = {inputLayerSize};
            % Set outputSizes to placeholder values to enforce 4D format.
            outputSizes = {[1,1,1]};

            inputFormats = { deep.internal.PlaceholderArray([inputLayerSize NaN],'SSCB') };
            outputFormats = { deep.internal.PlaceholderArray([1 1 1 NaN],'SSCB') };

            mapping = {inputMapping, outputMapping, classificationOutputs, inputSizes, outputSizes, inputFormats, outputFormats};
        end

        %------------------------------------------------------------------
        function mapping = createMIMODatastoreMapping(outputTableVariableNames, lgraph)
            
            externalLayers = lgraph.Layers;
            
            % Find locations of each layer in Layers array.
            idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.RPNClassificationLayer'),...
                externalLayers);
            layerIndices.ClassificationLayerIdx = find(idx,1,'last');
            
            idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.RCNNBoxRegressionLayer'),...
                externalLayers);
            layerIndices.RegressionLayerIdx = find(idx,1,'last');
            
            idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.ImageInputLayer'),...
                externalLayers);
            layerIndices.ImageLayerIdx = find(idx,1,'first');
            
            % Get layer name based on location in Layers array.
            clsName = externalLayers(layerIndices.ClassificationLayerIdx).Name;
            regName = externalLayers(layerIndices.RegressionLayerIdx).Name;
            imgName = externalLayers(layerIndices.ImageLayerIdx).Name;
                       
            dst = {
                imgName
                clsName
                regName
                };
            
            mapping = table(outputTableVariableNames',dst,...
                'VariableNames',{'Data','Destination'});
        end

        %------------------------------------------------------------------
        function aboxes = initializeAnchorBoxes(minBoxes, numLevels, scale, ~)
            boxScales = cumprod([1 repelem(scale, numLevels-1)]);
            % Generate anchor box sizes.
            aboxes = cell(1,size(minBoxes,1));
            for i = 1:size(minBoxes,1)
                aboxes{i} = bsxfun(@times, minBoxes(i,:), boxScales');
            end
            if nargin == 4   
                % Backward compatibility with models trained in previous
                % releases (<= 18a). Fractional anchor boxes were used
                % during proposal generation. 
                aboxes = vertcat(aboxes{:});            
            else               
                aboxes = round(vertcat(aboxes{:}));                
            end
        end
        
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
        
        function detector = assembleDetectorFromCheckpoint(frcnn, rpn, params)            
            
            detector = iCreateAndPartiallyInitializeDetector(params);
            
            % Set networks after other properties are set because of
            % the need to sync detector and layer properties.
            if params.TrainingStage < 5
                if ~isempty(rpn)
                    if isa(rpn,'DAGNetwork')
                        detector.PartiallyTrainedRPN = rpn;
                    else
                        % Partially trained RPN returned as DAGNetwork for
                        % detector because it is used for proposal
                        % generation.
                        detector.PartiallyTrainedRPN = ...
                            vision.internal.cnn.createDAGNetwork(rpn);
                    end
                end
                
                if ~isempty(frcnn)
                    detector.PartiallyTrainedFastRCNN = frcnn;
                end                               
                
            else                   
                assert(false,'only for creating checkpoint detectors')
            end             
        end   
        
        %------------------------------------------------------------------
        function detector = assembleDetectorForPredictionFourStep(frcnn, rpn, params)
            % Assemble detector for prediction. This is called after
            % training is complete.
            assert(params.TrainingStage == 5, ...
                'Expected training stage to be 5 to indicate training complete');
            
            detector = iCreateAndPartiallyInitializeDetector(params);

            % Merge fast r-cnn and rpn to form faster r-cnn network.
            fastrcnn = vision.internal.cnn.RCNNLayers.mergeFastAndRPNLayerGraphs(...
                frcnn, rpn, params.NumClasses, params.RCNNInfo);
            detector.Network = vision.internal.cnn.createDAGNetwork(fastrcnn);
            
            % Clear hidden properties that may hold data required only for
            % training.
            detector.PartiallyTrainedRPN = [];
            detector.PartiallyTrainedFastRCNN = [];
            detector.RCNNInfo = [];
            
        end
        
        %------------------------------------------------------------------
        function detector = assembleDetectorForPredictionEndToEnd(net, params)
            
            detector = iCreateAndPartiallyInitializeDetector(params);
 
            detector.Network = net;
            
            % Clear hidden properties that may hold data required only for
            % training.
            detector.RCNNInfo = [];
        end
        
        
        %------------------------------------------------------------------
        function lgraph = createFasterRCNNFromFastAndRPN(...
                detectorNetwork,detectorRegionProposalNetwork,LastSharedLayerIndex,anchorBoxes)
            % Return a layerGraph created by merging Fast and RPN networks
            % training in previous releases (<= 18a).
            if ~isempty(detectorNetwork) && ~isempty(detectorRegionProposalNetwork)
                
                % get all layers for fast and rpn networks.
                fast.Layers = detectorNetwork.Layers;
                fast.RegLayers = detectorNetwork.RegLayers;
                
                rpn.Layers = detectorRegionProposalNetwork.Layers(LastSharedLayerIndex+1:end);
                rpn.RegLayers = detectorRegionProposalNetwork.RegLayers;
                
                % Create unique names for layers
                N1 = numel(fast.Layers);
                N2 = numel(fast.RegLayers);
                N3 = numel(rpn.Layers);
                allLayers = vertcat(...
                    fast.Layers, ...
                    fast.RegLayers, ...
                    rpn.Layers, ...
                    rpn.RegLayers);
                allLayers = deep.internal.sdk.layer.assignUniqueLayerNames(allLayers);
                fast.Layers = allLayers(1:N1);
                fast.RegLayers = allLayers(N1+1:N1+N2);
                rpn.Layers = allLayers(N1+N2+1:N1+N2+N3);
                rpn.RegLayers = allLayers(N1+N2+N3+1:end);
                
                % Create Fast R-CNN network.
                includeROIInputLayer = false;
                lgraph = vision.cnn.FastRCNN.fastRCNNLayerGraph(fast, detectorNetwork.LayerIndex, includeROIInputLayer);
                
                % Add RPN layers.
                proposalLayer = nnet.cnn.layer.RegionProposalLayer(...
                    nnet.internal.cnn.layer.RegionProposalLayer(...
                    'proposalLayer', anchorBoxes));
                
                % add RPN to Fast R-CNN layer graph.
                lastSharedLayerIndex = LastSharedLayerIndex;
                lgraph = lgraph.addLayers(rpn.Layers);
                lgraph = lgraph.addLayers(rpn.RegLayers);
                src = rpn.Layers(detectorRegionProposalNetwork.LayerIndex-LastSharedLayerIndex).Name;
                dst = rpn.RegLayers(1).Name;
                lgraph = lgraph.connectLayers(src,dst);
                
                % Connect last shared layer
                lgraph = lgraph.connectLayers(fast.Layers(...
                    lastSharedLayerIndex).Name,...
                    rpn.Layers(1).Name);
                
                % Add region proposal layer
                lgraph = lgraph.addLayers(proposalLayer);
                
                % Find rpn cls layer
                idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.RPNClassificationLayer'),...
                    lgraph.Layers);
                assert(sum(idx)==1)
                clsName = lgraph.Layers(idx).Name;
                
                % Find rpn reg layer
                idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.RCNNBoxRegressionLayer'),...
                    rpn.RegLayers);
                assert(sum(idx)==1)
                regName = rpn.RegLayers(idx).Name;
                
                % Connect RPN classification and regression heads to proposal
                % layer. proposal layer should be connected to conv layer
                % outputs that feed RPN softmax.
                
                % First find RPN reshape.
                idx = string(lgraph.Connections.Destination)==clsName;
                assert(sum(idx)==1);
                pCls = lgraph.Connections.Source{idx};
                
                % Now find layer feeding RPN reshape.
                idx = string(lgraph.Connections.Destination)==pCls;
                assert(sum(idx)==1);
                pCls = lgraph.Connections.Source{idx};
                
                idx = string(lgraph.Connections.Destination)==regName;
                assert(sum(idx)==1);
                pReg = lgraph.Connections.Source{idx};
                
                lgraph = lgraph.connectLayers(pCls, [proposalLayer.Name '/scores']);
                lgraph = lgraph.connectLayers(pReg, [proposalLayer.Name '/boxDeltas']);
                
                % Connect proposal layer to roi pooling layer.
                idx = arrayfun(@(x)...
                    isa(x,'nnet.cnn.layer.ROIMaxPooling2DLayer') || ...
                    isa(x,'vision.cnn.layer.ROIAveragePooling2DLayer'),...
                    lgraph.Layers);
                assert(sum(idx)==1)
                
                roiPool = lgraph.Layers(idx).Name;
                
                lgraph = lgraph.connectLayers(proposalLayer.Name, [roiPool '/roi']);
                
                lgraph = vision.internal.cnn.RCNNLayers.updateROIPoolingLayerScaleFactor(lgraph);
                            
                % Group the filter weights/bias of the convolution layer
                % producing the RPN classification scores such that the
                % foreground scores are group in the first N channels of
                % the conv layer output. Previously, the foreground and
                % background scores were interleaved.
                names = {lgraph.Layers.Name};
                idx = strcmp(pCls,names);
                convLayer = lgraph.Layers(idx);
                W = convLayer.Weights;
                B = convLayer.Bias;
                
                W = cat(4, W(:,:,:,1:2:end),W(:,:,:,2:2:end));
                B = cat(3, B(:,:,1:2:end),B(:,:,2:2:end));
                convLayer.Weights = W;
                convLayer.Bias = B;
                lgraph = lgraph.replaceLayer(convLayer.Name, convLayer);
            else
                lgraph = [];
            end
        end
        
        %------------------------------------------------------------------
        function idx = findFastBoxRegressionLayer(network, roiPoolName)
            % Search for box regression layer attached to sub-graph
            % starting from ROI pooling layer. This finds the box
            % regression layer required for detection (the one attached to
            % the Fast R-CNN sub-network), the other box regression layer
            % is used in the RPN sub-network.
            
            dg = vision.internal.cnn.RCNNLayers.digraph(network);
            
            % Find the ROI pooling layer node.
            roiPoolID = findnode(dg,roiPoolName);
            assert(~isempty(roiPoolID));
            
            % Search for all nodes starting from ROI pooling layer.
            ids = dfsearch(dg,roiPoolID);
            
            % For all nodes after the ROI pooling layer, find the one that
            % is a box regression layer.
            names = dg.Nodes.Name(ids,:);
            allNames = string({network.Layers.Name});
            idx = [];
            for i = 1:numel(names)
                idx = string(names{i}) == allNames;
                if isa(network.Layers(idx), 'nnet.cnn.layer.RCNNBoxRegressionLayer')
                    idx = find(idx);
                    break;
                end
            end
            assert(~isempty(idx));
        end
        
    end
    
    %----------------------------------------------------------------------
    methods(Hidden, Static, Access = public)
        
        function [bboxes, scores] = filterBBoxesBySize(bboxes, scores, minSize, maxSize)
            [bboxes, scores] = fastRCNNObjectDetector.filterSmallBBoxes(bboxes, scores, minSize);
            
            [bboxes, scores] = fastRCNNObjectDetector.filterLargeBBoxes(bboxes, scores, maxSize);
        end
        
        %------------------------------------------------------------------
        function params = parseProposeInputs(varargin)
            
            p = inputParser;
            p.addParameter('NumStrongestRegions', 2000);
            p.addParameter('NumStrongestRegionsBeforeProposalNMS', Inf);
            p.addParameter('MiniBatchSize', 128);
            p.addParameter('MinScore', 0);
            p.addParameter('ExecutionEnvironment', 'auto');
            p.addParameter('ProposalsOutsideImage', 'discard');
            p.addParameter('OverlapThreshold', 0.7);
            p.addParameter('ScaleFactor', [1 1]);        
            parse(p, varargin{:});
            
            userInput = p.Results;
            
            fasterRCNNObjectDetector.checkStrongestRegions(userInput.NumStrongestRegions);
            
            vision.internal.cnn.validation.checkMiniBatchSize(userInput.MiniBatchSize, mfilename);
            
            exeenv = vision.internal.cnn.validation.checkExecutionEnvironment(...
                userInput.ExecutionEnvironment, mfilename);
            
            params.NumStrongestRegions   = double(userInput.NumStrongestRegions);
            params.MiniBatchSize         = double(userInput.MiniBatchSize);
            params.MinScore              = double(userInput.MinScore);
            params.ExecutionEnvironment  = exeenv;
            params.OverlapThreshold      = double(userInput.OverlapThreshold);
            params.ProposalsOutsideImage = char(userInput.ProposalsOutsideImage);
            params.NumStrongestRegionsBeforeProposalNMS = double(userInput.NumStrongestRegionsBeforeProposalNMS);
            params.ScaleFactor           = double(userInput.ScaleFactor);     
        end
        
        %------------------------------------------------------------------
        function checkStrongestRegions(vec)
            for i = 1:numel(vec)
                N = vec(i);
                if isinf(N)
                    % OK, use all regions.
                else
                    validateattributes(N, ...
                        {'numeric'},...
                        {'scalar', 'real', 'positive', 'integer', 'nonempty', 'finite', 'nonsparse'}, ...
                        mfilename, 'NumStrongestRegions');
                end
            end
        end
        
        %------------------------------------------------------------------
        function checkThreshold(threshold)
            validateattributes(threshold, {'single', 'double'}, {'nonempty', 'nonnan', ...
                'finite', 'nonsparse', 'real', 'scalar', '>=', 0, '<=', 1}, ...
                mfilename, 'Threshold');
        end
        
        %------------------------------------------------------------------
        function checkOverlapThreshold(threshold)
            validateattributes(threshold, {'single', 'double'}, {'nonempty', 'nonnan', ...
                'finite', 'nonsparse', 'real', 'scalar', '>=', 0, '<=', 1}, ...
                mfilename, 'ProposalOverlapThreshold');
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
        function checkRegionProposalOutputs(bboxes, scores)
            if ~ismatrix(bboxes) || size(bboxes, 2) ~= 4
                error(message('vision:rcnn:invalidBBoxDim'));
            end
            
            if ~iscolumn(scores)
                error(message('vision:rcnn:invalidScoreDim'));
            end
            
            if ~isreal(bboxes) || issparse(bboxes) || ~all(isfinite(bboxes(:)))
                error(message('vision:rcnn:invalidBBox'));
            end
            
            if ~isreal(scores) || issparse(scores) || ~all(isfinite(scores))
                error(message('vision:rcnn:invalidScores'));
            end
            
            if size(bboxes, 1) ~= size(scores, 1)
                error(message('vision:rcnn:proposalInconsistentNumel'));
            end
            
            if any(bboxes(:,3) <= 0) || any(bboxes(:,4) <= 0)
                error(message('vision:rcnn:proposalInvalidWidthHeight'));
            end
            
        end
        
        %------------------------------------------------------------------
        function printHeader(printer, classNames)
            printer.print('*************************************************************************\n');
            printer.printMessage('vision:rcnn:fasterTrainingBanner');
            printer.linebreak;

            for i = 1:numel(classNames)
                printer.print('* %s\n', classNames{i});
            end
         
            printer.linebreak;
            
        end
        
        %------------------------------------------------------------------
        function printFooter(printer)
            printer.printMessage('vision:rcnn:trainingFooter');
            printer.print('*************************************************************************\n');
            printer.linebreak;
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
    end
    
    methods(Hidden, Access = protected)
        function groups = getPropertyGroups( this )
            propertyList = struct;
            propertyList.ModelName = this.ModelName;
            propertyList.Network = this.Network;
            propertyList.AnchorBoxes = this.AnchorBoxes;
            % Transpose class names, which is a column vector
            propertyList.ClassNames = this.ClassNames';
            propertyList.MinObjectSize = this.MinObjectSize;
            
            groups = matlab.mixin.util.PropertyGroup(propertyList);
        end
    end
end

%--------------------------------------------------------------------------
function s = iLoadPreviousVersion(s)
if ~isfield(s,'Version') || s.Version == 1.0
    % version 1.0 has BoxAspectRatios, BoxPyramidScales,
    
    % MinBoxSizes
    n = numel(s.BoxAspectRatios);
    w = zeros(n,1);
    h = zeros(n,1);
    
    iw = s.BoxAspectRatios < 1; % W < H
    ih = s.BoxAspectRatios >=1; % H < W
    
    w(iw) = s.MinBoxSize(1);
    h(ih) = s.MinBoxSize(2);
    
    h(iw) = w(iw) ./ s.BoxAspectRatios(iw);
    w(ih) = h(ih) .* s.BoxAspectRatios(ih);
    
    s.MinBoxSizes = [h w];
    
    % BoxPyramidScale
    scales = sort(s.BoxScales);
    scale = median( scales(2:end) ./ scales(1:end-1) );
    s.BoxPyramidScale = scale;
    
    % NumBoxPyramidLevels
    s.NumBoxPyramidLevels = numel(s.BoxScales);
    
    % ModelSize
    s.ModelSize = s.MinBoxSize;
    
    s.ModelName = '';
        
end

if s.Version <= 1.5
    % Fix-up min model size for models saved in previous versions.
    if isfield(s,'Network')
        scaleByGridSize = true; % preserve behavior of older networks.
        modelSize = iDetermineMinBoxSize(s.Network, scaleByGridSize);
        
        s.ModelSize = modelSize;
        
        % MinBoxSizes must be >= ModelSize
        s.MinBoxSizes = max(s.MinBoxSizes, s.ModelSize);
    end
end

if s.Version < 2
    s.BoxMean    = zeros(1,4,'single');
    s.BoxStd     = ones(1,4,'single');
    s.RPNBoxMean = zeros(1,4,'single');
    s.RPNBoxStd  = ones(1,4,'single');
end

if s.Version < 3
    
    % Previous releases require recomputing scale factor during inference.
    s.RecomputeScaleFactor = true;
    s.RCNNInfo = [];
    s.PartiallyTrainedRPN = [];
    s.PartiallyTrainedFastRCNN = [];    
    
    if ~isempty(s.Network)
        % Load previous versions of the networks.
        detectorNetwork = vision.cnn.FastRCNN.loadobj(s.Network);
        detectorRPN     = vision.cnn.RegionProposalNetwork.loadobj(s.RegionProposalNetwork);
        
        % Previous releases used fractional anchors during proposal
        % generation. Anchors boxes must be kept fractional in order to
        % produce the same detection results in new releases.
        fractionalAnchors = true;
        s.AnchorBoxes = fasterRCNNObjectDetector.initializeAnchorBoxes(...
            s.MinBoxSizes, s.NumBoxPyramidLevels, s.BoxPyramidScale, fractionalAnchors);              
                
        % Convert to DAGNetwork.
        lgraph = fasterRCNNObjectDetector.createFasterRCNNFromFastAndRPN(...
            detectorNetwork,detectorRPN,s.LastSharedLayerIndex,s.AnchorBoxes);
                        
        s.Network = vision.internal.cnn.createDAGNetwork(lgraph);
    end
end
    
if s.Version < 5
    % Previous releases require setting image size on to the layer, from the input images
    % when MiniBatchSize is 1, during inference.
    s.SetImageSizeToEmptyFrom19b = false;
end
end

%--------------------------------------------------------------------------
function sz = iDetermineMinBoxSize(frcnn,~)
analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(frcnn.Layers);
scaleFactor = vision.internal.cnn.RCNNLayers.featureToImageScale(analysis);
externalLayers = [analysis.LayerAnalyzers.ExternalLayer];
idx = vision.internal.cnn.RCNNLayers.findROIPoolingLayer(externalLayers);

if nargin == 1
    sz = ceil(scaleFactor);
else
    % for backward compatibility loading older versions
    sz = ceil( scaleFactor .*  externalLayers(idx).OutputSize );
end
end

%--------------------------------------------------------------------------
function s = iDefaultDetectionParams()
s.roi = zeros(0,4);
s.NumStrongestRegions = 2000;
s.NumStrongestRegionsBeforeProposalNMS = 6000;
s.SelectStrongest = true;
s.MiniBatchSize = 128;
s.MinSize = [];
s.MaxSize = [];
s.ExecutionEnvironment = 'auto';
s.ProposalOverlapThreshold = 0.7;
s.Threshold = 0.5;
s.DetectionsOnBorder = 'clip';
end

%------------------------------------------------------------------
function detector = iCreateAndPartiallyInitializeDetector(params)
% Creates a detector and partially initializes properties. The Network is
% not set.
detector = fasterRCNNObjectDetector();

detector.ModelSize            = params.ModelSize;
detector.TrainingStage        = params.TrainingStage;
detector.ModelName            = params.ModelName;
detector.BoxMean              = params.BoxMean;
detector.BoxStd               = params.BoxStd;
detector.RPNBoxMean           = params.RPNBoxMean;
detector.RPNBoxStd            = params.RPNBoxStd;
detector.RCNNInfo             = params.RCNNInfo;

detector.BoxPyramidScale      = params.BoxPyramidScale;
detector.MinBoxSizes          = params.MinBoxSizes;
detector.NumBoxPyramidLevels  = params.NumBoxPyramidLevels;
end

%-----------------------------------------------------------------------
function out = iPreProcessForDatastoreRead(in, fcn, numArgOut, varargin)
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
    out = cell(numItems, numArgOut);
    for ii = 1:numel(in)
        [out{ii, 1:numArgOut}] = fcn(in{ii},varargin{:});
    end
end
%-----------------------------------------------------------------------
function Iroi = iPreprocess(I, roi, useROI, colorPreprocessing)
    Iroi = vision.internal.detector.cropImageIfRequested(I, roi, useROI);

    % Convert image from RGB <-> grayscale as required by network.
    Iroi = vision.internal.cnn.utils.convertImageToMatchNumberOfNetworkChannels({Iroi}, colorPreprocessing);
    Iroi = Iroi{1};
end

%-----------------------------------------------------------------------
function Iroi = iPreprocessBatch(I, roi, useROI, colorPreprocessing, params)
    sz = params.ImageSize;
    sz(3) = params.InputSize(3);
    Iroi = zeros(sz,'like',I);
    for ii = 1:params.ImageSize(4)
        Iroi(:,:,:,ii) = iPreprocess(I(:,:,:,ii), roi, useROI, colorPreprocessing);
    end
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
        params.ImageSize = size(data{1});
    else
        params.ImageSize = size(data);
    end
end
