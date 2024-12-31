classdef ssdObjectDetector < vision.internal.detector.ObjectDetector
%

% Copyright 2019-2024 The MathWorks, Inc.

    properties(SetAccess = protected)
        % Network is a dlnetwork object.
        Network

        % AnchorBoxes is a M-by-1 cell array of K-by-2 array, where K is
        % number of anchor boxes, M is either the number of elements in 
        % DetectionNetworkSource or if dlnetwork is complete SSD network
        % then it will be half of the count of outputs of the dlnetwork. K
        % corresponds to array of anchor boxes in [height width] format.
        AnchorBoxes

        % ClassName specifies the names of the object classes that the SSD 
        % object detector is configured to detect.
        ClassNames

        % An M-by-3 matrix defining the [height width channel] of image
        % sizes used to train the detector. If [height width] specified
        % then it will be assumed to be grayscale image. Default will be
        % size of image input layer of the input dlnetwork if it has image
        % input layer otherwise it will be required parameter.
        InputSize
    end

    %----------------------------------------------------------------------
    properties(Access = protected, Hidden)
        % BackgroundLabel Label to use for background class. Default is
        % 'Background'.
        BackgroundLabel

        % LayerIndices A struct that caches indices to certain layers used
        % frequently.
        LayerIndices
        % PredictOutputOrder A two-element vector use to order the output
        %                    of predict as scores and bboxes.
        PredictOutputOrder
    end

    %----------------------------------------------------------------------
    properties (Access = public, Hidden)
        % TiledAnchorBoxes  are the anchor box priors tiled across
        %                   individual feature map.It is an M-by-4 matrix 
        %                   defining the [x y width height] of M anchor
        %                   boxes.
        TiledAnchorBoxes
    end

    %----------------------------------------------------------------------
    properties (Access = protected, Transient)
        FilterBboxesFunctor
    end

    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        function this = ssdObjectDetector(varargin)
            vision.internal.requiresNeuralToolbox(mfilename);
            narginchk(2,9);
            params = ssdObjectDetector.parseDetectorInputs(varargin{:});
            params = vision.internal.cnn.ssdNetworkCreation(params);
            this.ClassNames = params.ClassNames;
            this.InputSize = params.InputSize;
            this.ModelName = params.ModelName;
            this.AnchorBoxes = params.AnchorBoxes;
            % Setting layerIndices along with network. Params has
            % detectionNetworkSource LayerIndices information
            this.Network = params;
            this.FilterBboxesFunctor = vision.internal.cnn.utils.FilterBboxesFunctor;
        end
        %-------------------------------------------------------
        function this = set.Network(this, params)
            if ~isempty(params)
                % It will be used during checkpoint saving
                if isa(params,'dlnetwork')
                    % Validate and load the checkpoint network.
                    network = params;
                    validateattributes(network, ...
                        {'dlnetwork'},{'scalar'});
                    this.Network = network;
                else
                    % Create or configure network
                    network = params.Network;
                    validateattributes(network,{'DAGNetwork','dlnetwork'},{'scalar'});
                    % Update layer index cache
                    this = this.setLayerIndices(params);
                    this = this.setPredictOutputOrder(network);
                    if  ~(params.QuantizedFlag)
                        this.Network = initialize(network);
                    else
                        this.Network = network;
                    end
                    this = setTiledAnchorBoxes(this);
                end
            else
                this.Network = [];
            end
        end
        %----------------------------------------------------------------------
        function varargout = detect(detector, I, varargin)
            % bboxes = detect(ssd,I) detects objects within the image I.
            % The location of objects within I are returned in bboxes, an
            % M-by-4 matrix defining M bounding boxes. Each row of bboxes
            % contains a four-element vector, [x, y, width, height]. This
            % vector specifies the upper-left corner and size of a bounding
            % box in pixels. ssd is a ssdObjectDetector object and I is a
            % truecolor or grayscale image.
            %
            % bboxes = detect(ssd,IBatch) detects objects within each
            % image contained in the batch of images IBatch. IBatch is a
            % numeric array containing images in the format
            % H-by-W-by-C-by-B, where B is the number of images in the
            % batch, and C is the channel size. For grayscale images, C must
            % be 1. The network input channel size of the detector, ssd, must
            % match the channel size of each image in the batch, H-by-W-by-C-by-B.
            % bboxes is a B-by-1 cell array, containing M-by-4 matrices for
            % each image in the batch.
            %
            % [..., scores] = detect(ssd,I) optionally return the class
            % specific confidence scores for each bounding box. The scores
            % for each detection is product of objectness prediction and
            % classification scores. The range of the scores is [0 1].
            % Larger score values indicate higher confidence in the
            % detection. scores is a B-by-1 cell array, if the input I is
            % a batch of images in the format H-by-W-by-C-by-B.
            %
            % [..., labels] = detect(ssd,I) optionally return the labels
            % assigned to the bounding boxes in an M-by-1 categorical
            % array. The labels used for object classes is defined during
            % training using the trainSSDObjectDetector function.
            % labels is a B-by-1 cell array, if the input I is a batch of
            % images in the format H-by-W-by-C-by-B.
            %
            % detectionResults = detect(ssd,DS) detects objects within the
            % series of images returned by the read method of datastore,
            % DS. DS, must be a datastore that returns a table or a cell
            % array with the first column containing images.
            % detectionResults is a 3-column table with variable names
            % 'Boxes', 'Scores', and 'Labels' containing bounding boxes,
            % scores, and the labels. The location of objects within an
            % image, I are returned in bounding boxes, an M-by-4 matrix
            % defining M bounding boxes. Each row of boxes contains a
            % four-element vector, [x, y, width, height]. This vector
            % specifies the upper-left corner and size of a bounding box in
            % pixels. ssd is a ssdObjectDetector object.
            %
            % [...] = detect(..., roi) optionally detects objects within
            % the rectangular search region specified by roi. roi must be a
            % 4-element vector, [x, y, width, height], that defines a
            % rectangular region of interest fully contained in I.
            %
            % [...] = detect(..., Name, Value) specifies additional
            % name-value pairs described below:
            %
            % 'Threshold'              A scalar between 0 and 1. Detections
            %                          with scores less than the threshold
            %                          value are removed. Increase this value
            %                          to reduce false positives.
            %
            %                          Default: 0.5
            %
            % 'SelectStrongest'        A logical scalar. Set this to true to
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
            % 'MinSize'                Specify the size of the smallest
            %                          region containing an object, in
            %                          pixels, as a two-element vector,
            %                          [height width]. When the minimum size
            %                          is known, you can reduce computation
            %                          time by setting this parameter to that
            %                          value. By default, 'MinSize' is the
            %                          smallest object that can be detected
            %                          by the trained network.
            %
            %                          Default: [1,1]
            %
            % 'MaxSize'                Specify the size of the biggest region
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
            % 'MiniBatchSize'          The mini-batch size used for processing a
            %                          large collection of images. Images are grouped
            %                          into mini-batches and processed as a batch to
            %                          improve computational efficiency. Larger
            %                          mini-batch sizes lead to faster processing, at
            %                          the cost of more memory.
            %
            %                          Default: 128
            %
            % 'ExecutionEnvironment'   The hardware resources used to run the
            %                          SSD detector. Valid values are:
            %
            %                          'auto' - Use a GPU if it is available,
            %                                   otherwise use the CPU.
            %
            %                           'gpu' - Use the GPU. To use a GPU,
            %                                   you must have Parallel
            %                                   Computing Toolbox(TM), and
            %                                   a CUDA-enabled NVIDIA GPU.
            %                                   If a suitable GPU is not
            %                                   available, an error message
            %                                   is issued.
            %
            %                           'cpu  - Use the CPU.
            %
            %                          Default : 'auto'
            %
            % 'Acceleration'           Optimizations that can improve
            %                          performance at the expense of some
            %                          overhead on the first call, and possible
            %                          additional memory usage. Valid values
            %                          are:
            %
            %                           'auto'    - Automatically select
            %                                       optimizations suitable
            %                                       for the input network and
            %                                       environment.
            %
            %                           'mex'     - (GPU Only) Generate and
            %                                       execute a MEX function.
            %
            %                           'none'    - Disable all acceleration.
            %
            %                          Default : 'auto'
            %
            %  Notes:
            %  -----
            %  - When 'SelectStrongest' is true the selectStrongestBboxMulticlass
            %    function is used to eliminate overlapping boxes. By
            %    default, the function is called as follows:
            %
            %   selectStrongestBboxMulticlass(bbox, scores, ...
            %                                       'RatioType', 'Union', ...
            %                                       'OverlapThreshold', 0.5);
            %
            %  - When the input image size does not match the network input size, the
            %    detector resizes the input image to the network input size.
            %
            % Class Support
            % -------------
            % The input image I can be uint8, uint16, int16, double,
            % single, or logical, and it must be real and non-sparse.
            %
            % Example
            % -------
            % % Load pre-trained vehicle detector.
            % vehicleDetector = load('ssdVehicleDetector.mat', 'detector');
            % detector = vehicleDetector.detector;
            %
            % % Read test image.
            % I = imread('highway.png');
            %
            % % Run detector.
            % [bboxes, scores, labels] = detect(detector, I);
            %
            % % Display results.
            % detectedImg = insertObjectAnnotation(I, 'Rectangle', bboxes, cellstr(labels));
            % figure
            % imshow(detectedImg)
            %
            % See also trainSSDObjectDetector, selectStrongestBboxMulticlass.
            params = parseDetectInputs(detector,I,varargin{:});
            [varargout{1:nargout}] = performDetect(detector, I, params);
        end
    end
%----------------------------------------------------------------------
methods (Hidden)
    %------------------------------------------------------------------
    function s = saveobj(this)
        s.Network         = this.Network;
        s.BackgroundLabel = this.BackgroundLabel;
        s.AnchorBoxes     = this.AnchorBoxes;
        s.ModelName       = this.ModelName;
        s.ClassNames      = this.ClassNames;
        s.TiledAnchorBoxes = this.TiledAnchorBoxes;
        s.InputSize        = this.InputSize;
        s.Version         = 2.0;
    end
    %------------------------------------------------------------------
    function net = matlabCodegenPrivateNetwork(this)
        net = this.Network;
    end
    %------------------------------------------------------------------
    function varargout = preprocess(~, I, varargin)

        roi    = varargin{1,1}.ROI;
        useROI = varargin{1,1}.UseROI;
        trainedImageSize = varargin{1,1}.TrainedImageSize;

        if ~isnumeric(I) && ~iscell(I) % If input is datastore
            % Copy and reset the given datastore, so external state events are
            % not reflected
            ds = copy(I);
            reset(ds);

            fcn = @iPreprocessForDetect;
            % Obtain preprocessed image -> num arg out is 1
            fcnArgOut = 2;
            varargout{1} = transform(ds, @(x)iPreProcessForDatastoreRead(x, fcn, fcnArgOut, roi, useROI, trainedImageSize,varargin{1,1}));
            % Process datastore with network and output the predictions
            varargout{2} = {};
        else
            [varargout{1:nargout}]  = iPreprocessForDetect(I, roi, useROI, trainedImageSize, varargin{1,1});
        end

    end
    %----------------------------------------------------------------------
    function outputFeatures = predict(detector,dlX, varargin)

        predictParams = varargin{1,1};
        anchorBoxes = detector.TiledAnchorBoxes;

        if (~isnumeric(dlX) && ~ iscell(dlX))

            % Process datastore with network and output the predictions
            loader = iCreateDataLoader(dlX,predictParams.MiniBatchSize,predictParams.NetworkInputSize);

            % Iterate through data and perform prediction on one image
            % at a time as original inference image size (batchInfo)
            % can be different for each images
            k = 1;
            bboxes = cell(predictParams.MiniBatchSize, 1);
            scores = cell(predictParams.MiniBatchSize, 1);
            labels = cell(predictParams.MiniBatchSize, 1);

            while hasdata(loader)
                X = nextBatch(loader);
                imgBatch = X{1};
                batchInfo = X{2};
                numMiniBatch = size(batchInfo,1);
                outFeature = getFeaturesUsingPredict(detector, imgBatch, predictParams);
                bboxesAll  = outFeature{1};
                scoresAll  = outFeature{2};
                for ii = 1:numMiniBatch
                    [bboxes{k}, scores{k}, labels{k}] = ...
                        iPostProcessPredictions(bboxesAll(:, :, :, ii), ...
                        scoresAll(:, :, :, ii), ...
                        batchInfo{ii}, ...
                        anchorBoxes, ...
                        predictParams.TrainedImageSize, predictParams);
                    k = k + 1;
                end
            end
            varNames = {'Boxes', 'Scores', 'Labels'};
            outputFeatures = table(bboxes(1:k-1), scores(1:k-1), labels(1:k-1), 'VariableNames', varNames);
        else
            outputFeatures  = getFeaturesUsingPredict(detector, dlX,predictParams);
        end
    end
    %------------------------------------------------------------------
    function varargout = postprocess(detector,YPredData, info, params)
        if (istable(YPredData))
            [varargout{1:nargout}] = YPredData;
        else
            if params.DetectionInputWasBatchOfImages                
                [varargout{1:nargout}] = iPostProcessBatchPredictions(YPredData{1},YPredData{2},info,detector.TiledAnchorBoxes,params.TrainedImageSize,params);
            else
                [varargout{1:nargout}] = iPostProcessPredictions(YPredData{1},YPredData{2},info,detector.TiledAnchorBoxes,params.TrainedImageSize,params);
            end
        end
    end
end
%----------------------------------------------------------------------
methods (Access = protected)
    %------------------------------------------------------------------
    function this = setTiledAnchorBoxes(this)
        % Table (DetectionNeworkSource Name,index and FeatureMap Size
        % Table)
        if isempty(this.TiledAnchorBoxes)
        dnsLayerFeatureMapSizeTable = this.LayerIndices.DetectionNetworkSourceLayerIndicesFeaturMapSizeTable;
        numDetectionNetworkSource = size(dnsLayerFeatureMapSizeTable,1);
        tiledAnchorBoxes = cell(numDetectionNetworkSource,1);
        for idx = 1:numDetectionNetworkSource
            tiledAnchorBoxes{idx} = iGetTiledAnchorBoxes( ...
                dnsLayerFeatureMapSizeTable.FeatureMapSize{idx}(1:2), ...
                this.AnchorBoxes{idx}, this.InputSize);
        end
        this.TiledAnchorBoxes = cell2mat(tiledAnchorBoxes);
        end
    end
   %------------------------------------------------------------------
    function params = parseDetectInputs(detector, I, varargin)
        % Image should be bigger than network image input layer. This
        % ensures the feature map sizes are large enough to perform RPN
        % processing and ROI Pooling
        params.DetectionInputWasDatastore = ~isnumeric(I);
        if params.DetectionInputWasDatastore
            sampleImage = vision.internal.cnn.validation.checkDetectionInputDatastore(I, mfilename);
        else
            sampleImage = I;
        end

        networkInputSize = detector.InputSize;
        params.ClassNames = detector.ClassNames;
        params.BackgroundLabel = detector.BackgroundLabel;
        params.TrainedImageSize = detector.InputSize(1:2);
        params.FilterBboxesFunctor = detector.FilterBboxesFunctor;

        validateChannelSize = true;  % check if the channel size is equal to that of the network
        validateImageSize   = false; % ssd can support images smaller than input size
        [sz,params.DetectionInputWasBatchOfImages] = vision.internal.cnn.validation.checkDetectionInputImage(...
            networkInputSize,sampleImage,validateChannelSize,validateImageSize);

        defaults = iDefaultDetectionParams();

        p = inputParser;
        p.addOptional('roi', defaults.roi);
        p.addParameter('SelectStrongest', defaults.SelectStrongest);
        p.addParameter('MinSize', defaults.MinSize);
        p.addParameter('MaxSize', sz(1:2));
        p.addParameter('MiniBatchSize', defaults.MiniBatchSize);
        p.addParameter('ExecutionEnvironment', defaults.ExecutionEnvironment);
        p.addParameter('Acceleration', 'auto');
        p.addParameter('Threshold', defaults.Threshold);
        parse(p, varargin{:});
        userInput = p.Results;

        vision.internal.cnn.validation.checkMiniBatchSize(userInput.MiniBatchSize, mfilename);

        vision.internal.inputValidation.validateLogical(...
            userInput.SelectStrongest, 'SelectStrongest');

        % Validate minsize and maxsize
        wasMinSizeSpecified = ~ismember('MinSize', p.UsingDefaults);
        wasMaxSizeSpecified = ~ismember('MaxSize', p.UsingDefaults);

        if wasMinSizeSpecified
            vision.internal.detector.ValidationUtils.checkMinSize(userInput.MinSize, [1,1], mfilename);
        end

        if wasMaxSizeSpecified
            % Default max size set above in inputParser to size(I)
            vision.internal.detector.ValidationUtils.checkSize(userInput.MaxSize, 'MaxSize', mfilename);                
        end

        if wasMinSizeSpecified && wasMaxSizeSpecified
            coder.internal.errorIf(any(userInput.MinSize >= userInput.MaxSize) , ...
                'vision:ObjectDetector:minSizeGTMaxSize');
        end

        % Validate ROI
        useROI = ~ismember('roi', p.UsingDefaults);
        if useROI
            vision.internal.detector.checkROI(userInput.roi, size(I));
            if ~isempty(userInput.roi)
                sz = userInput.roi([4 3]);
                vision.internal.detector.ValidationUtils.checkImageSizes(sz(1:2), userInput, wasMinSizeSpecified, ...
                    userInput.MinSize, ...
                    'vision:ObjectDetector:ROILessThanMinSize', ...
                    'vision:ObjectDetector:ROILessThanMinSize');
            end
        else
            vision.internal.detector.ValidationUtils.checkImageSizes(sz(1:2), userInput, wasMaxSizeSpecified, ...
                userInput.MinSize , ...
                'vision:ObjectDetector:ImageLessThanMinSize', ...
                'vision:ObjectDetector:ImageLessThanMinSize');
        end

        % Validate threshold
        ssdObjectDetector.checkThreshold(userInput.Threshold,mfilename);
        % Validate execution environment
        exeEnv = vision.internal.cnn.validation.checkExecutionEnvironment(...
            userInput.ExecutionEnvironment, mfilename);
        % Validate Acceleration environment
        accel = vision.internal.cnn.validation.checkAcceleration(...
            userInput.Acceleration, mfilename);

        params.ROI                  = double(userInput.roi);
        params.UseROI               = useROI;
        params.SelectStrongest      = logical(userInput.SelectStrongest);
        params.MinSize              = single(userInput.MinSize);
        params.MaxSize              = double(userInput.MaxSize);
        params.Threshold            = double(userInput.Threshold);
        params.MiniBatchSize        = double(userInput.MiniBatchSize);
        params.ExecutionEnvironment = exeEnv;
        params.Acceleration         = accel;
        params.NetworkInputSize     = networkInputSize;
        params.CastToGpuArray       = ~isgpuarray(sampleImage);
    end
    %-----------------------------------------------------------------------
    function outFeatures = getFeaturesUsingPredict(this, imgBatch, params)
        try
            out = cell(1,2);
            [out{:}] = this.Network.predict(imgBatch, ...
                'Acceleration',params.Acceleration);
            outFeatures{1} = out{this.PredictOutputOrder(1)};
            outFeatures{2} = out{this.PredictOutputOrder(2)};
        catch ME
            throwAsCaller(ME);
        end
    end
end
%----------------------------------------------------------------------
methods(Hidden, Access = protected)
    %------------------------------------------------------------------
    function this = setLayerIndices(this, params)
        % Set image input layer Index
        this.LayerIndices.InputLayerIdx = params.ImgInputIdx;

        clIdx = iFindLayer(params.Network.Layers, 'nnet.cnn.layer.SoftmaxLayer');

        this.LayerIndices.ClassificationOutIdx1 = clIdx;

        this.LayerIndices.DetectionNetworkSourceLayerIndicesFeaturMapSizeTable = params.DetectionNetworkSourceTable(:,[2,3]);
    end
    %------------------------------------------------------------------
    function this = setPredictOutputOrder(this, network)
        % Expected order of predict is [bboxes, scores]
        clsName = network.Layers(this.LayerIndices.ClassificationOutIdx1).Name;
        regressOutIdx = ~strcmp(network.OutputNames,clsName);
        regressionOut = network.OutputNames(regressOutIdx);
        regName = regressionOut{1,1};
        this.PredictOutputOrder = [
            find(strcmp(regName,network.OutputNames),1)  % bboxes
            find(strcmp(clsName,network.OutputNames),1)  % scores
            ];
    end
end
%----------------------------------------------------------------------
methods(Static, Access = public, Hidden)
    %------------------------------------------------------------------
    function this = loadobj(s)
        try
            vision.internal.requiresNeuralToolbox(mfilename);
            s = iUpgradeToLatestVersion(s);
            IsQuantizedNet = deep.internal.quantization.isQuantizationEnabled(s.Network);
            if ~IsQuantizedNet
                if isa(s.Network,'DAGNetwork')
                    % Provide backward compatibility by removeing loss and
                    % merge layers from detectors with Network saved as
                    % DAGNetwork
                    [s.Network,s.removedLayers] = ssdObjectDetector.removeLossAndMergeLayersFromLgraph(layerGraph(s.Network));
                    s.Network = dlnetwork(s.Network,Initialize=true);
                    % Create detector
                    this = ssdObjectDetector(s, s.ClassNames,s.AnchorBoxes, ...
                        'InputSize',s.InputSize,'ModelName',s.ModelName);                    
                else
                    s.Network = ssdObjectDetector.removeLossAndMergeLayersFromdlNetwork(s.Network);
                    % Create detector
                    this = ssdObjectDetector(s.Network, s.ClassNames,s.AnchorBoxes, ...
                        'InputSize',s.InputSize,'ModelName',s.ModelName);
                end
            else
                % Create detector
                this = ssdObjectDetector(s.Network, s.ClassNames,s.AnchorBoxes, ...
                    'InputSize',s.InputSize,'ModelName',s.ModelName);
            end

            this.BackgroundLabel = s.BackgroundLabel;
            % Set tiled anchor boxes
            this.TiledAnchorBoxes = s.TiledAnchorBoxes;
        catch ME
            rethrow(ME);
        end
    end
    %------------------------------------------------------------------
    function n = matlabCodegenDlRedirect(~)
        n = 'vision.internal.codegen.ssdObjectDetector';
    end
    %------------------------------------------------------------------
    function detector = assembleDetector(inpDetector, DAGNet)
        % During training input to the function is always DAGNet
        if nargin == 1
            DAGNet = inpDetector.InitializedDAGNet;
            IsQuantizedDAGNet = deep.internal.quantization.isQuantizationEnabled(DAGNet);
            DAGNet = layerGraph(DAGNet);
        else
            IsQuantizedDAGNet = deep.internal.quantization.isQuantizationEnabled(DAGNet);
        end
        if ~IsQuantizedDAGNet
            % Remove loss and merge layer
            [DAGNet,~] = ssdObjectDetector.removeLossAndMergeLayersFromLgraph(DAGNet);

            % Create detector
            detector = ssdObjectDetector(DAGNet,inpDetector.ClassNames, ...
                inpDetector.AnchorBoxes,'InputSize',inpDetector.InputSize,'ModelName',inpDetector.ModelName);
        else
            detector = ssdObjectDetector(DAGNet,inpDetector.ClassNames, ...
                inpDetector.AnchorBoxes,'InputSize',inpDetector.InputSize,'ModelName',inpDetector.ModelName);

        end

        if ~isempty(inpDetector.TiledAnchorBoxes)
            % Update the tiled anchorBoxes if extracted tiled anchorBoxes
            % are from layerGraph
            detector.TiledAnchorBoxes = inpDetector.TiledAnchorBoxes;
        else
            detector = setTiledAnchorBoxes(detector);
        end
    end
    %------------------------------------------------------------------
    function analysis = analyzeNetwork(network)
        analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(network);
        analysis.throwIssuesIfAny()
    end
    %------------------------------------------------------------------
    function [bboxes, scores, labels] = filterSmallBBoxes(bboxes, scores,labels, minSize)
        [bboxes, scores, labels] = vision.internal.cnn.utils.FilterBboxesFunctor.filterSmallBBoxes(minSize, bboxes, scores, labels);
    end
    %------------------------------------------------------------------
    function [bboxes, scores, labels] = filterLargeBBoxes(bboxes, scores, labels, maxSize)
        [bboxes, scores, labels] = vision.internal.cnn.utils.FilterBboxesFunctor.filterLargeBBoxes(maxSize, bboxes, scores, labels);
    end
    %----------------------------------------------------------------------
    function anchorBoxData = extractTiledAnchorBoxesForTrainer(detector)
        % It will be used during training for extracting tiled anchor boxes
        % when detector is passed
        inputImageSize = detector.InputSize;
        scaleFactor = [inputImageSize(1:2) inputImageSize(1:2)];
        detectionNetworkSourceTable = detector.LayerIndices.DetectionNetworkSourceLayerIndicesFeaturMapSizeTable;

        s = struct('AnchorBoxes', []);
        tiledAnchorBoxes = detector.TiledAnchorBoxes;
        s.AnchorBoxes = {};
        idxZ = 1;
        for idx = 1:size(detectionNetworkSourceTable,1)
            featureMapSize = detectionNetworkSourceTable.FeatureMapSize{idx}(1:2);
            sizeOfCurrentTiledAnchorBoxes = featureMapSize(1)*featureMapSize(2)*size(detector.AnchorBoxes{idx},1);
            if ~isempty(tiledAnchorBoxes)
                % Detector with tiledAnchorBoxes
                D = tiledAnchorBoxes(idxZ:idxZ+sizeOfCurrentTiledAnchorBoxes-1,:);
            else
                D = iGetTiledAnchorBoxes(detectionNetworkSourceTable.FeatureMapSize{idx}(1:2), detector.AnchorBoxes{idx}, inputImageSize);
            end
            s.AnchorBoxes = [s.AnchorBoxes, D(:, 1:4).*scaleFactor];
            idxZ = idxZ + sizeOfCurrentTiledAnchorBoxes;
        end
        anchorBoxData = struct2table(s, 'AsArray', true);
    end
    %----------------------------------------------------------------------
    function tiledAnchorBoxes = calculateTiledAnchorBoxesDlnetworkForInference(detector)
        % Extract tiled anchor boxes during inference
        dnsLayerFeatureMapSizeTable = detector.LayerIndices.DetectionNetworkSourceLayerIndicesFeaturMapSizeTable;
        numDetectionNetworkSource = size(dnsLayerFeatureMapSizeTable,1);
        tiledAnchorBoxes = cell(numDetectionNetworkSource,1);
        for idx = 1:numDetectionNetworkSource
            tiledAnchorBoxes{idx} = iGetTiledAnchorBoxes( ...
                dnsLayerFeatureMapSizeTable.FeatureMapSize{idx}(1:2), ...
                detector.AnchorBoxes{idx}, detector.InputSize);
        end
        tiledAnchorBoxes = cell2mat(tiledAnchorBoxes);
    end
    %----------------------------------------------------------------------
    function anchorBoxData = extractTiledAnchorBoxesLayerGraphForTrainer(lgraph)
        % Extract tiled anchor boxes for training when layerGraph is input
        % to trainer    
        imageInputIdx = iFindLayer(lgraph.Layers, 'nnet.cnn.layer.ImageInputLayer');
        inputImageSize = lgraph.Layers(imageInputIdx).InputSize;
        scaleFactor = [inputImageSize(1:2) inputImageSize(1:2)];
        pbLayerIdx = iFindLayer(lgraph.Layers, 'nnet.cnn.layer.AnchorBoxLayer');

        s = struct('AnchorBoxes', []);
        s.AnchorBoxes = {};
        for idx = 1:numel(pbLayerIdx)
            pbLayer = lgraph.Layers(pbLayerIdx(idx));
            D = pbLayer.getBoxes(inputImageSize);
            s.AnchorBoxes = [s.AnchorBoxes, D(:, 1:4).*scaleFactor];
        end
        anchorBoxData = struct2table(s, 'AsArray', true);
    end
    %----------------------------------------------------------------------
    function tiledAnchorBoxes = calculateTiledAnchorBoxesLgraphForInference(lgraph)
        % Calculate tiled anchor boxes for inference from layerGraph with
        % anchorBoxLayer 
        imageInputLayerLayerClassName = 'nnet.cnn.layer.ImageInputLayer';
        imageInputIdx = find(arrayfun(@(x)isa(x,imageInputLayerLayerClassName),lgraph.Layers));
        inputImageSize = lgraph.Layers(imageInputIdx).InputSize;
        anchorBoxLayerClassName = 'nnet.cnn.layer.AnchorBoxLayer';
        anchorBoxLayerIdx = find(arrayfun(@(x)isa(x,anchorBoxLayerClassName),lgraph.Layers));
        pbLayer = lgraph.Layers(anchorBoxLayerIdx);
        numSetOfTiledAnchorBoxes = numel(pbLayer);
        tiledAnchorBoxes = cell(numSetOfTiledAnchorBoxes,1);
        for idx = 1:numSetOfTiledAnchorBoxes
            tiledAnchorBoxes{idx} = pbLayer(idx).getBoxes(inputImageSize);
        end
        tiledAnchorBoxes = cell2mat(tiledAnchorBoxes);
    end
    %----------------------------------------------------------------------
    function anchorBoxes = baseAnchorBoxes(lgraph)
        anchorBoxLayerClassName = 'nnet.cnn.layer.AnchorBoxLayer';
        anchorBoxLayerIdx = find(arrayfun(@(x)isa(x,anchorBoxLayerClassName),lgraph.Layers));
        pbLayer = lgraph.Layers(anchorBoxLayerIdx);
        anchorBoxes = cell(numel(numel(pbLayer)),1);
        for idx = 1:numel(pbLayer)
            theseAnchorBoxes = pbLayer(idx).AnchorBoxes;
            anchorBoxes{idx,1} = theseAnchorBoxes; 
        end
    end

    %----------------------------------------------------------------------
    function lgraph = removeAnchorBoxLayer(lgraph,~)
        % Remove anchorBoxLayer from the layerGraph
        anchorBoxLayerClassName = 'nnet.cnn.layer.AnchorBoxLayer';
        anchorBoxLayerIdx = find(arrayfun(@(x)isa(x,anchorBoxLayerClassName),lgraph.Layers));
        inputLayerGraph = lgraph;
        T = lgraph.Connections;
        if ~isempty(anchorBoxLayerIdx)
            for idparent = 1:numel(anchorBoxLayerIdx)
                eachAnchorBoxLayeridx = anchorBoxLayerIdx(idparent);
                parentLayerIndex= find(strcmp(T.Destination, inputLayerGraph.Layers(eachAnchorBoxLayeridx).Name) == 1);
                childLayerIndex = find(strcmp(T.Source, inputLayerGraph.Layers(eachAnchorBoxLayeridx).Name) == 1);
                lgraph = removeLayers(lgraph,inputLayerGraph.Layers(eachAnchorBoxLayeridx).Name);

                for idchild = 1:numel(childLayerIndex)
                    eachChildLayer = childLayerIndex(idchild);
                    lgraph = connectLayers(lgraph, T.Source{parentLayerIndex}, ...
                        T.Destination{eachChildLayer});
                end
            end
        end
    end
    %----------------------------------------------------------------------
    function [lgraph,removedLayers] = removeLossAndMergeLayersFromLgraph(lgraph)
        % Remove anchorBoxLayer, ssdMergeLayer and loss layers from
        % layerGraph
        rcnnBoxRegressionLayerIdx = iFindLayer(lgraph.Layers, 'nnet.cnn.layer.RCNNBoxRegressionLayer');
        softmaxLayerLayerIdx = iFindLayer(lgraph.Layers, 'nnet.cnn.layer.SoftmaxLayer');
        classificationLayerIdx = iFindLayer(lgraph.Layers,'nnet.cnn.layer.FocalLossLayer');
        ssdMergeLayerLayerIdx= iFindLayer(lgraph.Layers, 'nnet.cnn.layer.SSDMergeLayer');
        if isempty(classificationLayerIdx)
            classificationLayerIdx = iFindLayer(lgraph.Layers,'vision.internal.cnn.SSDHardNegativeMiningLossLayer');
        end
        removeLayerIds = [classificationLayerIdx;ssdMergeLayerLayerIdx;rcnnBoxRegressionLayerIdx;softmaxLayerLayerIdx];
        removedLayers = lgraph.Layers(removeLayerIds);
        lgraph = removeLayers(lgraph,{lgraph.Layers(removeLayerIds).Name});
        if isa(lgraph,'dlnetwork')
            lgraph.OutputNames = [];
        end
    end

    %----------------------------------------------------------------------
    function dlNet= removeLossAndMergeLayersFromdlNetwork(dlNet)
        % Remove anchorBoxLayer, ssdMergeLayer afrom dlNetwork
        softmaxLayerLayerIdx = iFindLayer(dlNet.Layers, 'nnet.cnn.layer.SoftmaxLayer');
        ssdMergeLayerLayerIdx= iFindLayer(dlNet.Layers, 'nnet.cnn.layer.SSDMergeLayer');
        removeLayerIds = [ssdMergeLayerLayerIdx;softmaxLayerLayerIdx];
        dlNet = removeLayers(dlNet,{dlNet.Layers(removeLayerIds).Name});
    end    
    %----------------------------------------------------------------------
    function lgraph = updateImageInputLayer(lgraph)
        % Update normalization of image input layer if input layer
        % properties are empty
        imgIdx = arrayfun(@(x)isa(x,'nnet.cnn.layer.ImageInputLayer'),...
            lgraph.Layers);
        imageInputIdx = find(imgIdx,1,'first');
        inputLayer = lgraph.Layers(imageInputIdx,1);
        imageSize = inputLayer.InputSize;
        idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.Convolution2DLayer'),...
            lgraph.Layers);
        convIdx = find(idx,1,'first');
        numChannels = lgraph.Layers(convIdx,1).NumChannels;
        if (~strcmp(numChannels,'auto'))
            if numChannels ~= imageSize
                error(message('vision:ssd:InvalidLayerSize'));
            end
        end
        if ~isempty(inputLayer.Mean) || ~(isempty(inputLayer.Min)&&~isempty(inputLayer.Max))
            % Set normalization to none only if input layer stats are empty
            % as it will throw error while converting to dlnetwork
            imageInput = imageInputLayer(imageSize,'Name',inputLayer.Name,'Normalization','none');
            lgraph = replaceLayer(lgraph,lgraph.Layers(imageInputIdx).Name,...
                imageInput);
        end
    end
    %---------------------------------------------------------------------------------------------------
    function ssdDAGNetwork = convertToDAGNetwork(params, dlnet,removedLayers)
        % Convert detector to layerGraph by adding ssd merge to
        % SSD network
        numClasses = numel(params.ClassNames);
        numClassesPlusBackground = numClasses+1;
        ssdLayerGraph = layerGraph(dlnet);
        outputNames = dlnet.OutputNames;
        numFeaturePredictors = size(outputNames,2)/2;
        if nargin == 2
            % Create classification SSD merge layer
            mConfLayer = ssdMergeLayer(numClassesPlusBackground, numFeaturePredictors, 'Name', 'confmerge');
            % Create regression SSD merge layer
            mLocLayer = ssdMergeLayer(4, numFeaturePredictors, 'Name', 'locmerge');
            softmaxLayerName = "anchorBoxSoftmax";
            abSoftmaxLayer = softmaxLayer('Name', softmaxLayerName);
            regressLayerName = "anchorBoxRegression";
            regressionLayer = rcnnBoxRegressionLayer('Name',regressLayerName);
            classificationLayerName = "focalLoss";
            classificationOutputLayer = focalLossLayer(2, 0.25,'Classes',[params.ClassNames(:)' 'Background'],'Name', classificationLayerName);
        else
            mConfLayer = removedLayers(2);
            mLocLayer = removedLayers(3);
            abSoftmaxLayer = removedLayers(5);
            regressionLayer = removedLayers(4);
            classificationOutputLayer = removedLayers(1);
        end


        ssdLayerGraph = addLayers(ssdLayerGraph, mConfLayer);
        ssdLayerGraph = addLayers(ssdLayerGraph, mLocLayer);
        conf = 0;
        for i = 1:numFeaturePredictors
            idx = conf+i;
            ssdLayerGraph = connectLayers(ssdLayerGraph,outputNames{idx}, mConfLayer.Name + "/in" + num2str(i));
            ssdLayerGraph = connectLayers(ssdLayerGraph, outputNames{idx+1}, mLocLayer.Name + "/in" + num2str(i));
            conf = i;
        end

        ssdLayerGraph = addLayers(ssdLayerGraph, abSoftmaxLayer);
        ssdLayerGraph = connectLayers(ssdLayerGraph, mConfLayer.Name, abSoftmaxLayer.Name);
        ssdLayerGraph = addLayers(ssdLayerGraph,classificationOutputLayer);
        ssdLayerGraph = connectLayers(ssdLayerGraph, abSoftmaxLayer.Name, classificationOutputLayer.Name);
        ssdLayerGraph = addLayers(ssdLayerGraph,regressionLayer);
        ssdLayerGraph = connectLayers(ssdLayerGraph, mLocLayer.Name, regressionLayer.Name);
        ssdDAGNetwork = assembleNetwork(ssdLayerGraph);
    end

    %---------------------------------------------------------------------------------------------------
    function dlnet = convertToDLNetwork(params, dlnet, removedLayers)
        % Convert input dlnet to complete SSD dlNetwork by adding ssd
        % merge layers, softmax layer to SSD network
        numClasses = numel(params.ClassNames);
        numClassesPlusBackground = numClasses+1;
        outputNames = dlnet.OutputNames;
        numFeaturePredictors = size(outputNames,2)/2;
        if isempty(removedLayers)
            % Create classification SSD merge layer and softmax layer
            mConfLayer = ssdMergeLayer(numClassesPlusBackground, numFeaturePredictors, 'Name', 'confmerge');
            softmaxLayerName = "anchorBoxSoftmax";
            abSoftmaxLayer = softmaxLayer('Name', softmaxLayerName);
            % Create regression SSD merge layer
            mLocLayer = ssdMergeLayer(4, numFeaturePredictors, 'Name', 'locmerge');
        else
            mConfLayer = removedLayers(2);
            mLocLayer = removedLayers(3);
            abSoftmaxLayer = removedLayers(5);
        end

        dlnet = addLayers(dlnet, mConfLayer);
        dlnet = addLayers(dlnet, mLocLayer);
        conf = 0;
        for i = 1:numFeaturePredictors
            idx = conf+i;
            dlnet = connectLayers(dlnet,outputNames{idx}, mConfLayer.Name + "/in" + num2str(i));
            dlnet = connectLayers(dlnet, outputNames{idx+1}, mLocLayer.Name + "/in" + num2str(i));
            conf = i;
        end

        dlnet = addLayers(dlnet, abSoftmaxLayer);
        dlnet = connectLayers(dlnet, mConfLayer.Name, abSoftmaxLayer.Name);
    end        
    %----------------------------------------------------------------------
    function detectionNetworkSource = extractDetectionNetworkSource(network,quantizedFlag)
        % Extract detection network source of complete network
        if ~(quantizedFlag)
            if ~isa(network,'dlnetwork')
            network = dlnetwork(network);
            end
            outputs = network.OutputNames;
            detectionNetworkLayers = [];
            for i = 1:numel(outputs)
                detectionNetworkSourceCurrent = network.Connections.Source(find(strcmp(network.Connections.Destination, outputs(i)) == 1));
                currentDNSLayersCount = numel(union(detectionNetworkLayers,detectionNetworkSourceCurrent));
                if currentDNSLayersCount > numel(detectionNetworkLayers)
                    detectionNetworkLayers = [detectionNetworkLayers;detectionNetworkSourceCurrent];
                end
            end
            if numel(outputs) == 2*numel(detectionNetworkLayers)
                detectionNetworkSource = {detectionNetworkLayers{:}};
            else
                error(message('vision:ssd:invalidCompleteSSDNetwork'));
            end
        else
            % Extract detection network source during quantization
            detectionNetworkSource = iExtractDetectionNetworkSourceDAGNetwork(network);
        end
    end
    %--------------------------------------------------------------------------
    function lgraph = iRevertClassificationLayer(lgraph,params)
        % Replace the classification layer with
        % SSDHardNegativeMiningLossLayer to incorporate Hard Negative
        % Mining
        if (isfield(params,'ClassificationLayer'))
            classificationOutputLayer = focalLossLayer('Gamma',params.ClassificationLayer.Gamma, ...
                'Alpha',params.ClassificationLayer.Alpha,'Classes', [params.ClassNames(:)' 'Background'],'Name', params.ClassificationLayer.Name);
            lgraph = replaceLayer(lgraph,params.ClassificationLayer.Name,...
                classificationOutputLayer);
        else
            classificationLayerClassName = 'vision.internal.cnn.SSDHardNegativeMiningLossLayer';
            classificationLayerIdx = find(arrayfun(@(x)isa(x,classificationLayerClassName),lgraph.Layers));
            classificationOutputLayer = focalLossLayer('Gamma',2, 'Alpha',0.25,'Classes', [params.ClassNames(:)' 'Background'],'Name', lgraph.Layers(classificationLayerIdx).Name);
            lgraph = replaceLayer(lgraph,lgraph.Layers(classificationLayerIdx).Name,...
                classificationOutputLayer);
        end
    end
    %------------------------------------------------------------------
    function detector = detectorCheckpoint(net, detector)
        % Save checkpoint
        lgraph = layerGraph(net);
        [ssdLayerGraph,removedLayers] = ssdObjectDetector.removeLossAndMergeLayersFromLgraph(lgraph);
        ssdDlnetwork = dlnetwork(ssdLayerGraph,'Initialize',true);
        detector.Network = ssdObjectDetector.convertToDLNetwork(detector,ssdDlnetwork,removedLayers);
    end
    %------------------------------------------------------------------
    function ds = createTrainingDatastore(trainingData, anchorBoxes, params)

        vision.internal.inputValidation.checkGroundTruthDatastore(trainingData);
        ds = transform(trainingData, @(tbl) augmentationFcn(tbl, @ssdObjectDetector.resizeImageWithBbox, params.InputSize));

        targetGenParams.AnchorBoxes          = anchorBoxes;
        targetGenParams.ClassNames           = params.ClassNames;
        targetGenParams.BackgroundLabel      = params.BackgroundLabel;
        targetGenParams.PositiveOverlapRange = params.PositiveOverlapRange;
        targetGenParams.NegativeOverlapRange = params.NegativeOverlapRange;
        ds = transform(ds, @(tbl) iTargetFcn(tbl, @ssdObjectDetector.generateTargets, targetGenParams));
        ds.reset();
    end
    %------------------------------------------------------------------
    function mapping = createMIMODatastoreCellMapping(inputLayerSize)
        % There is one input layer: ImageInputLayer
        %  - the first column from the read output goes to the input layer
        inputMapping = {1};
        % There are two output layers: FocalLossLayer, and
        % BoxRegressionLayer
        %    - the second column contains Classification targets
        %    - the third column contains Regression targets
        outputMapping = {2, 3};
        % FocalLoss is a classification layer, BoxRegression is not
        classificationOutputs = [true, false];
        inputSizes = {inputLayerSize};
        % Empty the outputLayerSize to make the dispatcher consider the
        % output observation dimension as 1.
        outputSizes = {[], []};

        inputFormats = { deep.internal.PlaceholderArray([inputLayerSize NaN],'SSCB') };
        outputFormats = { deep.internal.PlaceholderArray([NaN NaN NaN NaN],'SSCB'), deep.internal.PlaceholderArray([NaN NaN NaN NaN],'SSCB') };

        mapping = {inputMapping, outputMapping, classificationOutputs, inputSizes, outputSizes, inputFormats, outputFormats};
    end
    %------------------------------------------------------------------
    function printHeader(printer, classNames)
        printer.print('*************************************************************************\n');
        printer.printMessage('vision:ssd:trainingBanner');
        printer.linebreak;
        for i = 1:numel(classNames)
            printer.print('* %s\n', classNames{i});
        end
        printer.linebreak;
    end
    %------------------------------------------------------------------
    function printFooter(printer)
        printer.printMessage('vision:ssd:trainingFooter');
        printer.print('*************************************************************************\n');
        printer.linebreak;
    end
end
%----------------------------------------------------------------------
methods(Static, Access = private, Hidden)
    %------------------------------------------------------------------
    function params = parseDetectorInputs(varargin)

        p = inputParser;
        p.addRequired('Network');
        p.addRequired('ClassNames');
        p.addRequired('AnchorBoxes');
        params.RemovedLayers = [];
        % Extract removedLayers information if input to the detector is a
        % DAGNetwork
        if isa(varargin{1,1},'struct')
            params.RemovedLayers = varargin{1,1}.removedLayers;
            varargin{1,1} = varargin{1,1}.Network;
        end
        imgInputIdx = iFindLayer(varargin{1,1}.Layers, 'nnet.cnn.layer.ImageInputLayer');
        if ~isempty(imgInputIdx)
            networkInputSize = varargin{1,1}.Layers(imgInputIdx,1).InputSize;
        else
            error(message('vision:ssd:imageInputLayerRequired'));
        end

        p.addParameter('InputSize', networkInputSize);
        p.addParameter('DetectionNetworkSource',{});
        p.addParameter('ModelName', '',@iAssertValidLayerName);
        parse(p, varargin{:});
        if ~isa(p.Results.Network,'nnet.cnn.LayerGraph')
            params.QuantizedFlag = deep.internal.quantization.isQuantizationEnabled(p.Results.Network);
        else
            params.QuantizedFlag = false;
        end

        if isempty(p.Results.DetectionNetworkSource) && ~isa(p.Results.Network,'nnet.cnn.LayerGraph') && params.QuantizedFlag
            validateattributes(varargin{1,1}, { 'DAGNetwork','dlnetwork'}, ...
                {'scalar'}, mfilename, ...
                'varargin{1,1}', 1);
        else
            validateattributes(varargin{1,1}, {'nnet.cnn.LayerGraph','dlnetwork'}, ...
                {'scalar'}, mfilename, ...
                'varargin{1,1}', 1);
        end
        
        params.Network = p.Results.Network;
        params.ClassNames = p.Results.ClassNames(:);
        params.AnchorBoxes = p.Results.AnchorBoxes(:);
        params.InputSize = p.Results.InputSize;
        params.DetectionNetworkSource = p.Results.DetectionNetworkSource;
        params.ModelName = char(p.Results.ModelName);
        params.InputSize = iCheckInputSize(params.InputSize);
        params.ImgInputIdx = imgInputIdx;
        iValidateClassNames(params.ClassNames);

        if isempty(params.DetectionNetworkSource)
            if isa(p.Results.Network,'nnet.cnn.LayerGraph')
                iValidateSSDNetwork(p.Results.Network)
            end
            params.ExtractedDetectionNetworkSource = ssdObjectDetector.extractDetectionNetworkSource(params.Network, params.QuantizedFlag);
            numDetectionNetworkSource = numel(params.ExtractedDetectionNetworkSource);
        else
            numDetectionNetworkSource = numel(params.DetectionNetworkSource);
        end
        
        iValidateDetectionNetworkSource(params.DetectionNetworkSource);
        iValidateAnchorBoxes(params.AnchorBoxes,numDetectionNetworkSource);
        params.AnchorBoxes = iSortAnchorBoxes(params.AnchorBoxes);
    end

    %------------------------------------------------------------------
    function [bboxes, scores, labels] = filterBoxesAfterRegression(bboxes, scores, labels, imageSize)
        % Remove boxes that don't make sense after regression:
        % * boxes with non-positive width and height.
        % * boxes that have zero overlap with image.

        [bboxes, scores, labels] = ssdObjectDetector.removeBoxesWithNonPositiveWidthHeight(bboxes, scores, labels);

        % Remove boxes that are completely outside the image
        x1 = bboxes(:,1);
        y1 = bboxes(:,2);
        x2 = bboxes(:,3) + x1 - 1;
        y2 = bboxes(:,4) + y1 - 1;

        boxOverlapsImage = ...
            (x1 < imageSize(2) & x2 > 1) & ...
            (y1 < imageSize(1) & y2 > 1);

        bboxes = bboxes(boxOverlapsImage,:);
        scores = scores(boxOverlapsImage);
        labels = labels(boxOverlapsImage);
    end
    %------------------------------------------------------------------
    function [bboxes, scores, labels] = removeBoxesWithNonPositiveWidthHeight(bboxes, scores, labels)

        % Remove any boxes that have non positive width and height
        remove = bboxes(:,3) < 1 | bboxes(:,4) < 1;

        bboxes(remove, :) = [];
        scores(remove, :) = [];

        if nargin == 3
            labels(remove) = [];
        end
    end

end
%----------------------------------------------------------------------
methods(Static, Access = public)
    %------------------------------------------------------------------
    function [imageIn, classOutput, regressOutput] = generateTargets(imageIn, bboxesIn, labelsIn, params)

        trainingSamples = iCreateTrainingSamples(bboxesIn, labelsIn, params);
        classOutput = {trainingSamples.Labels(1, :)};
        regressOutput = {trainingSamples.BboxOffsets(1, :)};
    end
    %------------------------------------------------------------------
    function [imageOut, bboxOut, labelOut] = resizeImageWithBbox(imageIn, bboxIn, labelIn, newSize)

        % Resize image
        imageOut = iResizeImage(imageIn, newSize);
        % Resize boxes
        oldSize = size(imageIn);
        bboxOut = iResizeBboxes(bboxIn, oldSize, newSize);
        % Retain labels
        labelOut = labelIn;
    end
    %------------------------------------------------------------------
    function bboxes = decode(P, reg, minSize, maxSize)

        % Decode regression estimates
        boxVariance = iGetBoxVariance();
        x = reg(:,1) * boxVariance(1);
        y = reg(:,2) * boxVariance(2);
        w = reg(:,3) * boxVariance(3);
        h = reg(:,4) * boxVariance(4);
        % Center of proposals
        px = P(:,1) + P(:,3)/2;
        py = P(:,2) + P(:,4)/2;
        % Compute regression value of ground truth box
        gx = P(:,3).*x + px; % center position
        gy = P(:,4).*y + py;
        gw = P(:,3) .* exp(w);
        gh = P(:,4) .* exp(h);

        if nargin > 2
            % Regression can push boxes outside user defined range. clip the boxes
            % to the min/max range. This is only done after the initial min/max size
            % filtering
            gw = min(gw, maxSize(2));
            gh = min(gh, maxSize(1));
            % Expand to min size
            gw = max(gw, minSize(2));
            gh = max(gh, minSize(1));
        end
        % Convert to [x y w h] format
        bboxes = [ gx - gw/2, gy - gh/2, gw, gh];
        bboxes = double(bboxes);
    end
    %------------------------------------------------------------------
    function target = encode(G, P)

        % Create regression targets
        boxVariance = iGetBoxVariance();
        % Center of proposal
        px = P(:,1) + floor(P(:,3)/2);
        py = P(:,2) + floor(P(:,4)/2);
        % Center of ground truth
        gx = G(:,1) + floor(G(:,3)/2);
        gy = G(:,2) + floor(G(:,4)/2);

        tx = (gx - px) ./(P(:,3)   .* boxVariance(1));
        ty = (gy - py) ./(P(:,4)   .* boxVariance(2));
        tw = log(G(:,3)./(P(:,3))) ./ boxVariance(3);
        th = log(G(:,4)./(P(:,4))) ./ boxVariance(4);

        target = [tx ty tw th]; % observations in columns
    end
end
end

%------------------------------------------------------------------
function s = iUpgradeToLatestVersion(s)
if s.Version == 1.0
    imIdx = iFindLayer(s.Network.Layers, 'nnet.cnn.layer.ImageInputLayer');
    s.AnchorBoxes = ssdObjectDetector.baseAnchorBoxes(s.Network);
    s.InputSize = s.Network.Layers(imIdx).InputSize;
    s.ClassNames(strncmpi(s.ClassNames,s.BackgroundLabel,4)) = [];
end
% Set networks after other properties are set because of
% the need to sync detector and layer properties
if ~isempty(s.Network)
    if s.Version == 1.0
        lgraphTrained = ssdObjectDetector.removeAnchorBoxLayer(layerGraph(s.Network));
        s.Network = assembleNetwork(lgraphTrained);
    end
end
% Set tiled anchor boxes
if isempty(s.TiledAnchorBoxes)
    if s.Version == 1
        s.TiledAnchorBoxes = ssdObjectDetector.calculateTiledAnchorBoxesLgraphForInference(s.Network);
    end
end
end
%----------------------------------------------------------------------
function anchorBoxes = iGetTiledAnchorBoxes(featureMapSize, baseAnchorBoxes, inputImageSize)
numBoxesPerGrid   = size(baseAnchorBoxes, 1);
layerWidth = featureMapSize(2);
layerHeight = featureMapSize(1);

imageWidth = inputImageSize(2);
imageHeight = inputImageSize(1);

% Define prior boxes shapes
boxWidth  = 0.5 * baseAnchorBoxes(:, 2);
boxHeight = 0.5 * baseAnchorBoxes(:, 1);

% Define centers of prior boxes
stepX = imageWidth / layerWidth;
stepY = imageHeight / layerHeight;
linx = (0.5 * stepX):stepX:(imageWidth - 0.5 * stepX);
liny = (0.5 * stepY):stepY:(imageHeight - 0.5 * stepY);
[centersY, centersX] = meshgrid(linx, liny);
centersX = centersX(:);
centersY = centersY(:);

% Define xmin, ymin, xmax, ymax of prior boxes
numGridCells = numel(centersX);
numAnchorBoxes = numGridCells * numBoxesPerGrid;
anchorBoxes = zeros(numAnchorBoxes, 4);

pbIdx = 1;
for bIdx = 1:numBoxesPerGrid
    thisBoxWidth = boxWidth(bIdx);
    thisBoxHeight = boxHeight(bIdx);
    for cIdx = 1:numGridCells
        x1 = (centersX(cIdx) - thisBoxWidth)/imageWidth;
        y1 = (centersY(cIdx) - thisBoxHeight)/imageHeight;
        x2 = (centersX(cIdx) + thisBoxWidth)/imageWidth;
        y2 = (centersY(cIdx) + thisBoxHeight)/imageHeight;
        anchorBoxes(pbIdx, :) = [x1 y1 x2 y2];
        pbIdx = pbIdx + 1;
    end
end

% Axis align coordinates of normalized boxes having negative
% and zero coordinate values
anchorBoxes(:,1) = min(max(anchorBoxes(:,1), 1/imageWidth), 1.0);
anchorBoxes(:,2) = min(max(anchorBoxes(:,2), 1/imageHeight), 1.0);

% Convert to xywh format
anchorBoxes(:, 3) = anchorBoxes(:, 3) - anchorBoxes(:, 1);
anchorBoxes(:, 4) = anchorBoxes(:, 4) - anchorBoxes(:, 2);
end
%-------------------------------------------------------------------------
function sortedAnchorBoxes = iSortAnchorBoxes(anchorBoxes)
    % AnchorBoxes is M-by-1 cell array, each containing an K-by-2 matrix
    % defining the [height width] of K anchor boxes
    % Sort the anchor boxes. Logic: anchorBoxes values corresponding to 
    % larger feature map will have small values
    numAnchorBoxes = numel(anchorBoxes);
    % Find smallest K for all K-by-2 matrix of M-by-1 anchorBoxes
    smallestK = min(cellfun(@numel,anchorBoxes))/2;
    anchorBoxesSum = zeros(1,numAnchorBoxes);
    for idx = 1:numAnchorBoxes
        firstKAnchorBoxes = anchorBoxes{idx}(1:smallestK,:);
        anchorBoxesSum(idx) = sum(firstKAnchorBoxes(:));
    end
    [~,order] = sort(anchorBoxesSum);
    sortedAnchorBoxes = cell(numAnchorBoxes,1);
    for j = 1:numAnchorBoxes
        sortedAnchorBoxes{j} = anchorBoxes{order(j)};
    end
end
%--------------------------------------------------------------------------
function s = iDefaultDetectionParams()

s.roi                  = zeros(0,4);
s.SelectStrongest      = true;
s.Threshold            = 0.5;
s.MinSize              = [1,1];
s.MaxSize              = [];
s.MiniBatchSize        = 128;
s.ExecutionEnvironment = 'auto';
s.Acceleration         = 'auto';
end

%-----------------------------------------------------------------------
function [bboxes, scores, labels] = iPostProcessBatchPredictions(bboxesAll, scoresAll, info, anchorBoxes, trainedImageSize, params)
numMiniBatch = size(bboxesAll,4);
bboxes = cell(numMiniBatch, 1);
scores = cell(numMiniBatch, 1);
labels = cell(numMiniBatch, 1);

for ii = 1:numMiniBatch
    [bboxes{ii}, scores{ii}, labels{ii}] = iPostProcessPredictions(...
        bboxesAll(:, :, :, ii),scoresAll(:, :, :, ii),info,anchorBoxes,trainedImageSize,params);
end
end

%--------------------------------------------------------------------------
function [bboxes, scores, labels] = iPostProcessPredictions(bboxes, scores, info, anchorBoxes, trainedImageSize, params)

if ~isempty(bboxes)

    if isa(bboxes,'dlarray')
        bboxes = extractdata(bboxes);
        scores = extractdata(scores);
    end

    % Make [x y w h] come in second dimension
    bboxes = permute(bboxes, [1 3 2 4]);

    trainedImageScale = [trainedImageSize trainedImageSize];
    outImageScale = [info.originalSize(2) info.originalSize(1) ...
        info.originalSize(2) info.originalSize(1)];

    anchorBoxes = anchorBoxes .* trainedImageScale;
    bboxes = ssdObjectDetector.decode(anchorBoxes, bboxes);
    bboxes = bboxes .* (outImageScale ./ trainedImageScale);


    % Remove scores associated with background. It is always last column
    scores = squeeze(scores(:,:,1:end-1));
    [scores, maxindices] = max(scores, [], 2);

    labels = categorical(maxindices, 1:numel(params.ClassNames), string(params.ClassNames));
    [bboxPred, scorePred, classPred] = ssdObjectDetector.filterBoxesAfterRegression(bboxes, scores, labels, info.originalSize);

    bboxPred  = bboxPred(scorePred >= params.Threshold, :);
    classPred = classPred(scorePred >= params.Threshold, 1);
    scorePred = scorePred(scorePred >= params.Threshold, 1);

    % Filter boxes based on MinSize, MaxSize
    [bboxPred, scorePred, classPred] = filterBBoxes(params.FilterBboxesFunctor,...
        params.MinSize, params.MaxSize, bboxPred, scorePred, classPred);

    % Apply NMS. OverlapThreshold is supposed to be hardcoded to 0.5 and is not
    % a function of the Threshold parameter
    if params.SelectStrongest
        [bboxes, scores, labels] = selectStrongestBboxMulticlass(bboxPred, scorePred, classPred ,...
            'RatioType', 'Union', 'OverlapThreshold',0.5);
    else
        bboxes = bboxPred;
        scores = scorePred;
        labels = classPred;
    end

    % Apply ROI offset
    bboxes(:,1:2) = vision.internal.detector.addOffsetForROI(bboxes(:,1:2), params.ROI, params.UseROI);
    bboxes = floor(bboxes);

    % Make sure floor doesn't end up with zero as one of the
    % coordinates
    bboxes(:,[1,2]) =  max(bboxes(:,[1 2]),1);

    if params.CastToGpuArray
        bboxes = gather(bboxes);
        scores = gather(scores);
        labels = gather(labels);
    end

else
    bboxes = zeros(0,4,'single');
    scores = zeros(0,1,'single');
    labels = categorical(cell(0,1),params.ClassNames);
end
end
%-----------------------------------------------------------------------
function out = iPreProcessForDatastoreRead(in, fcn, numArgOut, varargin)

if isnumeric(in) % Numeric input
    in = {in};
end
if istable(in) % Table input
    in = in{:,1};
else % Cell input
    in = in(:,1);
end
numItems = numel(in);
out = cell(numItems, numArgOut);
for ii = 1:numel(in)
    [out{ii, 1:numArgOut}] = fcn(in{ii},varargin{:});
end
end
%--------------------------------------------------------------------------
function [Ipreprocessed, info] = iPreprocessForDetect(I, roi, useROI, trainedImageSize, params)
if params.CastToGpuArray
     if (strcmp(params.ExecutionEnvironment,'auto') && canUseGPU) || strcmp(params.ExecutionEnvironment,'gpu')
         I = gpuArray(I);
     end
end
Iroi = vision.internal.detector.cropImageIfRequested(I, roi, useROI);
Ipreprocessed     = imresize(Iroi, trainedImageSize);
Ipreprocessed     = dlarray(single(Ipreprocessed),'SSCB');
info.originalSize = size(I);
end
%------------------------------------------------------------------------
function loader = iCreateDataLoader(ds, miniBatchSize, inputLayerSize)
loader = nnet.internal.cnn.DataLoader(ds, ...
    'MiniBatchSize', miniBatchSize, ...
    'CollateFcn', @(x)iTryToBatchData(x, inputLayerSize));
end
%------------------------------------------------------------------------
function data = iTryToBatchData(X, inputLayerSize)

try
    observationDim = numel(inputLayerSize) + 1;
    data{1} = cat(observationDim, X{:,1});
catch e
    if strcmp(e.identifier, 'MATLAB:catenate:dimensionMismatch')
        error(message('vision:ObjectDetector:unableToBatchImagesForDetect'));
    else
        throwAsCaller(e);
    end
end
data{2} = X(:,2:end);
end
%--------------------------------------------------------------------------
function trainingSamples = iCreateTrainingSamples(bboxesIn, labelsIn, params)

tbl = params.AnchorBoxes;
tbl.BBoxes = {bboxesIn};
tbl.Labels = {labelsIn};
trainingSamples = rowfun(...
    @(varargin)iSelectTrainingSamples(params, varargin{:}), tbl,...
    'InputVariables', 1:width(tbl),...
    'OutputFormat', 'table',...
    'NumOutputs', 6, ...
    'OutputVariableNames', {'Assignments','Labels','AnchorBoxes','BboxOffsets','GroundTruthBoxes', 'GroundTruthClasses'});

% Filter out the training sample if it does not have any class object as it
% would not help in training (regression)
if cellfun(@isempty,trainingSamples.GroundTruthClasses)
    error(message('vision:ssd:noTrainingSamplesAvailable'));
end
end
%--------------------------------------------------------------------------
function [assignments, labels, outAnchorBoxes, bboxOffsets, groundTruthBoxes, groundTruthClasses] ...
    = iSelectTrainingSamples(params, varargin)

anchorBoxes = varargin{1};
groundTruthBoxes = [];
groundTruthClasses = {};
thisGroundTruthBBoxes = varargin{2}{1};
thisGroundTruthLabels = varargin{3}{1};
for bidx = 1:numel(thisGroundTruthLabels)
    if all(thisGroundTruthBBoxes(bidx, :))
        groundTruthBoxes = [groundTruthBoxes; thisGroundTruthBBoxes(bidx, :)]; %#ok<AGROW>
        groundTruthClasses = [groundTruthClasses; thisGroundTruthLabels(bidx)]; %#ok<AGROW>
    end
end

assignments    = [];
labels         = [];
outAnchorBoxes = [];
bboxOffsets    = [];
positives      = [];


numPredictionBranches = size(anchorBoxes, 2);
for predIdx = 1:numPredictionBranches
    % Assign region proposals to ground truth boxes
    matchAllGroundTruth = false;
    [thisAssignment, positiveIndex, negativeIndex] = vision.internal.cnn.boxAssignmentUtils.assignBoxesToGroundTruthBoxes(...
        anchorBoxes{predIdx}, groundTruthBoxes, ...
        params.PositiveOverlapRange, ...
        params.NegativeOverlapRange, matchAllGroundTruth);
    assignments = [assignments; thisAssignment];  %#ok<AGROW>
    positives   = [positives; positiveIndex];  %#ok<AGROW>

    % Assign label to each region proposal
    thisLabels = vision.internal.cnn.boxAssignmentUtils.boxLabelsFromAssignment(...
        thisAssignment, groundTruthClasses, positiveIndex, negativeIndex, ...
        params.ClassNames, params.BackgroundLabel);
    labels = [labels; thisLabels]; %#ok<AGROW>

    outAnchorBoxes = [outAnchorBoxes; anchorBoxes{predIdx}]; %#ok<AGROW>
    % Create an array that maps ground truth box to positive
    % proposal box. i.e. this is the closest ground truth box to
    % each positive region proposal
    numBoxes = size(anchorBoxes{predIdx}, 1);
    targets = zeros(numBoxes , 4, 'like', groundTruthBoxes);
    for idx = 1:numBoxes
        if thisAssignment(idx) > 0
            G = groundTruthBoxes(thisAssignment(idx), :);
            P = anchorBoxes{predIdx}(idx, :);

            % Positive sample regression targets
            targets(idx, :) = ssdObjectDetector.encode(G, P);
        end
    end
    bboxOffsets = [bboxOffsets; targets]; %#ok<AGROW> % arrange as numBoxes by 4
end

assignments        = {assignments};
% Passing positive assignment logical flag array along with labels as it
% will be used for filtering positive and negative priors
labels             = {labels,positives};  
outAnchorBoxes     = {outAnchorBoxes};
bboxOffsets        = {bboxOffsets};
groundTruthBoxes   = {groundTruthBoxes};
groundTruthClasses = {groundTruthClasses};
end
%--------------------------------------------------------------------------
function tblOut = augmentationFcn(tblIn, fcnHandle, varargin)
% The purpose of this function is simply to marshal data into the format
% that the datastore needs
tblOut = tblIn;
for idx = 1:size(tblIn, 1)
    imageIn  = tblIn{idx, 1};
    bboxesIn = tblIn{idx, 2};
    labelsIn = tblIn{idx, 3};

    [imageOut, bboxOut, labelsOut] = fcnHandle(imageIn, bboxesIn, labelsIn, varargin{:});

    tblOut{idx, 1} = imageOut;
    tblOut{idx, 2} = bboxOut;
    tblOut{idx, 3} = labelsOut;
end
end
%--------------------------------------------------------------------------
function tblOut = iTargetFcn(tblIn, fcnHandle, varargin)
% The purpose of this function is simply to marshal data into the format
% that the datastore needs. In addition, this function also sets up the
% table variable names for the targets
tblOut = cell2table(cell(size(tblIn)));
for idx = 1:size(tblIn, 1)
    imageIn  = tblIn{idx, 1};
    bboxesIn = tblIn{idx, 2};
    labelsIn = tblIn{idx, 3};

    [imageOut, bboxOut, labelsOut] = fcnHandle(imageIn, bboxesIn, labelsIn, varargin{:});

    tblOut{idx, 1} = {imageOut};
    tblOut{idx, 2} = bboxOut;
    tblOut{idx, 3} = labelsOut;

end
tblOut.Properties.VariableNames = iGetOutputVariableNames();
end
%--------------------------------------------------------------------------
function outVarNames = iGetOutputVariableNames()
outVarNames = {'Image', 'ClassOutput', 'RegressOutput'};
end
%--------------------------------------------------------------------------
function boxVariance = iGetBoxVariance()
% These numbers work for PASCAL VoC dataset, and apply generally as
% well
boxVariance = [0.1 0.1 0.2 0.2];
end
%--------------------------------------------------------------------------
function inputSize = iCheckInputSize(inputSize)
if size(inputSize,2) == 2
    inputSize = [inputSize,1];
end
validateattributes(inputSize, {'numeric'}, ...
    {'2d','ncols',3,'ndims',2,'nonempty','nonsparse',...
    'real','finite','integer','positive','nrows',1,}); 
end
%--------------------------------------------------------------------------
function iValidateClassNames(value)
if ~isvector(value) || ~iIsValidDataType(value)
    error(message('vision:ssd:invalidClasses'));
end
if iHasDuplicates(value)
    error(message('vision:ssd:duplicateClasses'));
end
if isempty(value)
    error(message('vision:ssd:invalidClasses'));
end
end
%--------------------------------------------------------------------------
function iValidateDetectionNetworkSource(value)
    % Validate predictorBranchNames
    if ~isempty(value)
        validateattributes(value, {'cell','string'}, {'row', 'size', [1 NaN]}, ...
            mfilename, 'predictorLayerNames'); 
        
        if ~(iscellstr(value) || isstring(value))
            error(message('vision:ssd:InvalidPredictorLayerNames'));
        end
    end

end

%--------------------------------------------------------------------------
function tf = iIsValidDataType(value)
tf = iscategorical(value) || iscellstr(value) || isstring(value);
end

%--------------------------------------------------------------------------
function tf = iHasDuplicates(value)
tf = ~isequal(value, unique(value, 'stable'));
end
%--------------------------------------------------------------------------
function iAssertValidLayerName(name)
nnet.internal.cnn.layer.paramvalidation.validateLayerName(name);
end
%--------------------------------------------------------------------------
function iValidateAnchorBoxes(anchorBoxes,numDetectionNetworkSource)
validateattributes(anchorBoxes, {'cell'},{'column','size',[NaN NaN]}, ...
    mfilename, 'AnchorBoxes');
if size(anchorBoxes,1) ~= numDetectionNetworkSource
    error(message('vision:ssd:InvalidAnchorBoxesCount'));
end

for i = 1:size(anchorBoxes,1)
    validateattributes(anchorBoxes{i,1}, {'numeric'}, {'size', [NaN 2], 'real',...
        'nonnan', 'finite','nonsparse','nonzero','positive'}, mfilename, 'AnchorBoxes');
end
end
%--------------------------------------------------------------------------
function imageOut = iResizeImage(imageIn, newSize)
imageOut = imresize(imageIn, newSize(1:2));
end
%--------------------------------------------------------------------------
function bboxOut = iResizeBboxes(bboxIn, originalSize, newSize)   
bboxOut   = bboxresize(bboxIn, newSize(1:2)./originalSize(1:2));
end
%--------------------------------------------------------------------------
function idx = iFindLayer(layers, type)
results = arrayfun(@(x)isa(x,type),layers,'UniformOutput', true);
idx = find(results);
end
%---------------------------------------------------------------------------
function iValidateSSDNetwork(lgraph)
% Verify that complete ssd layerGraph does not have softmaxLayer, SSMerge
% layers
softmaxLayerLayerIdx = iFindLayer(lgraph.Layers, 'nnet.cnn.layer.SoftmaxLayer');
ssdMergeLayerLayerIdx= iFindLayer(lgraph.Layers, 'nnet.cnn.layer.SSDMergeLayer');
if ~(any(softmaxLayerLayerIdx, ssdMergeLayerLayerIdx))
    error(message('vision:ssd:invalidLayersInSSDNetwork'))
end
end

%----------------------------------------------------------------------------
function detectionNetworkSource = iExtractDetectionNetworkSourceDAGNetwork(network)
% Extract DetectionNetworkSource from DAGNetwork
ssdMergeLayerClassName = 'nnet.cnn.layer.SSDMergeLayer';
ssdMergeLayerIdx = find(arrayfun(@(x)isa(x,ssdMergeLayerClassName),network.Layers));
analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(network);
ssdMergeLayerAnalyzerIdx = strcmp({analysis.ExternalLayers.Name},network.Layers(ssdMergeLayerIdx(1)).Name);
softMaxLayerIndex = find(arrayfun(@(x)isa(x,'nnet.cnn.layer.SoftmaxLayer'),network.Layers));
softMaxLayerName = network.Layers(softMaxLayerIndex).Name;

if strcmp(analysis.LayerAnalyzers(ssdMergeLayerAnalyzerIdx).Outputs.Destination{1},softMaxLayerName)
    classMergeLayer = network.Layers(ssdMergeLayerIdx(1));
else
    classMergeLayer = network.Layers(ssdMergeLayerIdx(2));
end

% Check classification input sizes to the ssdMergeLayer
classMergeLayerIdx = strcmp({analysis.ExternalLayers.Name},classMergeLayer.Name);
classMergeLayerSources = analysis.LayerAnalyzers(classMergeLayerIdx).Inputs.Source;
% Extracting the classification and regression prediction layer connected
% to each anchorBoxLayer and storing the prediction layer name in desired
% output order
detectionNetworkSource = cell(1,numel(classMergeLayerSources));
for ii = 1:numel(classMergeLayerSources)
    currentClassificationPredictionLayerName = classMergeLayerSources{ii};
    currentClassificationIdx = strcmp({analysis.ExternalLayers.Name},currentClassificationPredictionLayerName);
    currentDetectionNetworkSource = analysis.LayerAnalyzers(currentClassificationIdx).Inputs.Source{1};
    detectionNetworkSource{ii} = currentDetectionNetworkSource;
end
end
