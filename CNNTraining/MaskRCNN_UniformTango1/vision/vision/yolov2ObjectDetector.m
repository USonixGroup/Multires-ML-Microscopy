classdef yolov2ObjectDetector < matlab.mixin.CustomDisplay & vision.internal.detector.ObjectDetector

    % Copyright 2018-2024 The MathWorks, Inc.
    
    properties(SetAccess = protected)
        % Network is a dlnetwork object representing the YOLO v2 network.
        Network
        % size of the input image specified as a vector
        % [H W] or [H W C], where H and W are the image height and
        % width, and C is the number of image channels.
        InputSize
        % An M-by-2 matrix defining the [height width] of image sizes used
        % to train the detector. During detection, an input image is
        % resized to nearest TrainingImageSize before it is processed by
        % the detection network.
        TrainingImageSize
        % AnchorBoxes is an M-by-2 matrix defining the [width height] of M
        % anchor boxes.
        AnchorBoxes
        % ClassNames is a cell array of object class names. These are the
        % object classes that the YOLO v2 detector was trained to find.
        ClassNames
        % ReorganizeLayer Source is the name of the layer whose features
        % will be processed by SpaceToDepth Layer to combine low-level and
        % high-level features
        ReorganizeLayerSource = ''
        % LossFactors is a 1-by-4 vector representing the loss factors for
        % each componenet of the YOLOv2 loss function
        LossFactors = [5 1 1 1]

    end

    properties(Access = private, Transient)
        % LayerIndices A struct that caches indices to certain layers used
        % frequently during detection.
        LayerIndices
    end

    properties (Access = protected, Transient)
        FilterBboxesFunctor
    end

    properties (Access = public, Hidden)
        % FractionDownsampling can either be true or false. If false, it
        % applies floor operation to the downsampling factor. It is set to
        % true by default.
        FractionDownsampling = true;

        % WH2HW can either be true or false. If true,
        % nnet.internal.cnn.layer.util.yoloPredictBBox function computes
        % bounding box dimensions using anchor boxes specified in [width,
        % height] format. It is set to false by default.
        WH2HW = false;
    end


    methods(Static, Access = public, Hidden)
        %------------------------------------------------------------------
        % Filter Boxes based on size.
        %------------------------------------------------------------------
        function [bboxes, scores, labels] = filterSmallBBoxes(bboxes, scores,labels, minSize)
            [bboxes, scores, labels] = vision.internal.cnn.utils.FilterBboxesFunctor.filterSmallBBoxes(minSize, bboxes, scores, labels);
        end

        function [bboxes, scores, labels] = filterLargeBBoxes(bboxes, scores, labels, maxSize)
            [bboxes, scores, labels] = vision.internal.cnn.utils.FilterBboxesFunctor.filterLargeBBoxes(maxSize, bboxes, scores, labels);
        end

        %------------------------------------------------------------------
        % Detector checkpoint function for yolov2.
        %------------------------------------------------------------------
        function detector = detectorCheckpoint(net, detector)
            detector.Network = net;
        end

        %------------------------------------------------------------------
        function A = preprocessforDetect(A,outputSize)
            % Rescale and cast data to single.
            if iscell(A)
                [A{1:min(2,numel(A))}] = iPreprocess(outputSize,A{:});
            else
                A = iPreprocess(outputSize,A);
            end
        end

        %------------------------------------------------------------------
        function A = trainingTransformForDatastore(A,outputSize)
            A = yolov2ObjectDetector.preprocessforDetect(A,outputSize);
            A = iAppendResponsesForTraining(A,outputSize);
        end

        %------------------------------------------------------------------
        function A = trainingTransform(A,outputSize)
            A = iAugmentData(A);
            A = yolov2ObjectDetector.preprocessforDetect(A,outputSize);
            A = iAppendResponsesForTraining(A,outputSize);
        end

        %------------------------------------------------------------------
        function A = validationTransform(A,outputSize)
            A = yolov2ObjectDetector.preprocessforDetect(A,outputSize);
            A = iAppendResponsesForTraining(A,outputSize);
        end

        %------------------------------------------------------------------
        function printHeader(printer, classNames)
            printer.print('*************************************************************************\n');
            printer.printMessage('vision:yolo:trainingBanner');
            printer.linebreak;

            for i = 1:numel(classNames)
                printer.print('* %s\n', classNames{i});
            end

            printer.linebreak;
        end

        %------------------------------------------------------------------
        function printFooter(printer)
            printer.printMessage('vision:yolo:trainingFooter');
            printer.print('*************************************************************************\n');
            printer.linebreak;
        end

    end
    %----------------------------------------------------------------------
    methods
        function varargout = detect(this, I, varargin)
            % bboxes = detect(yolo,I) detects objects within the image I.
            % The location of objects within I are returned in bboxes, an
            % M-by-4 matrix defining M bounding boxes. Each row of bboxes
            % contains a four-element vector, [x, y, width, height]. This
            % vector specifies the upper-left corner and size of a bounding
            % box in pixels. yolo is a yolov2ObjectDetector object
            % and I is a truecolor or grayscale image.
            %
            % bboxes = detect(yolo,IBatch) detects objects within each
            % image contained in the batch of images IBatch. IBatch is a
            % numeric array containing images in the format
            % H-by-W-by-C-by-B, where B is the number of images in the
            % batch, and C is the channel size. For grayscale images, C must be
            % 1. The network input channel size of the detector, yolo, must
            % match the channel size of each image in the batch, H-by-W-by-C-by-B.
            % bboxes is a B-by-1 cell array, containing M-by-4 matrices for
            % each image in the batch.
            %
            % [..., scores] = detect(yolo,I) optionally return the class
            % specific confidence scores for each bounding box. The scores
            % for each detection is product of objectness prediction and
            % classification scores. The range of the scores is [0 1].
            % Larger score values indicate higher confidence in the
            % detection. scores is a B-by-1 cell array, if the input I is
            % a batch of images in the format H-by-W-by-C-by-B.
            %
            % [..., labels] = detect(yolo,I) optionally return the labels
            % assigned to the bounding boxes in an M-by-1 categorical
            % array. The labels used for object classes is defined during
            % training using the trainYOLOv2ObjectDetector function.
            % labels is a B-by-1 cell array, if the input I is a batch of
            % images in the format H-by-W-by-C-by-B.
            %
            % [..., info] = detect(detector,I) optionally returns a
            % structure with two fields that contains information about the
            % class probabilities and objectness score of the detections:
            %
            % - "ClassProbabilities" contains the class
            %    probabilities for each of the detections, stored in a B-by-1
            %    cell array. B is the number of images in the input batch of
            %    images, I. Each cell in the array contains the class
            %    probabilities as an M-by-N numeric array. M is the number of
            %    detections and N is the number of classes.
            % - "ObjectnessScores" contains the objectness score for each of
            %    the detections, stored in a B-by-1 cell array. B is the
            %    number of images in the input batch of images, I. Each cell
            %    in the array contains the objectness score as an M-by-1
            %    numeric array. M is the number of detections.
            %
            % detectionResults = detect(yolo,DS) detects objects within the
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
            % pixels. yolo is a yolov2ObjectDetector object.
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
            %                          YOLO v2 detector. Valid values are:
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
            %   selectStrongestBboxMulticlass(bbox,scores,labels,...
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
            % vehicleDetector = load('yolov2VehicleDetector.mat', 'detector');
            % detector = vehicleDetector.detector;
            %
            % % Read test image.
            % I = imread('highway.png');
            %
            % % Run detector.
            % [bboxes, scores, labels] = detect(detector, I)
            %
            % % Display results.
            % detectedImg = insertObjectAnnotation(I, 'Rectangle', bboxes, cellstr(labels));
            % figure
            % imshow(detectedImg)
            %
            % See also trainYOLOv2ObjectDetector, selectStrongestBboxMulticlass.

            params = this.parseDetectInputs(I,varargin{:});
            [varargout{1:nargout}] = performDetect(this, I, params);
        end
    end
    %----------------------------------------------------------------------
    methods
        function this = yolov2ObjectDetector(varargin)
            narginchk(0,15);
            if nargin < 1
                % Loads the default pretrained model.
                this = iTripwireDefaultYOLOv2Model;

            elseif (isa(varargin{1,1},'yolov2ObjectDetector'))
                clsname = 'yolov2ObjectDetector';
                validateattributes(varargin{1,1},{clsname}, ...
                    {'scalar'}, mfilename);
                this.ModelName            = varargin{1,1}.ModelName;
                this.Network              = varargin{1,1}.Network;
                this.TrainingImageSize    = varargin{1,1}.TrainingImageSize;
                this.FractionDownsampling = varargin{1,1}.FractionDownsampling;
                this.WH2HW                = varargin{1,1}.WH2HW;
                this.AnchorBoxes          = varargin{1,1}.AnchorBoxes;
                this.ClassNames           = varargin{1,1}.ClassNames;
                detectorProperties        = properties(varargin{1,1});

                if(any(ismember('InputSize',detectorProperties)))
                    this.InputSize = varargin{1,1}.InputSize;
                else
                    this.InputSize = this.Network.Layers(this.LayerIndices.ImageLayerIdx).InputSize;
                end
                if(any(ismember('ReorganizeLayerSource',detectorProperties)))
                    this.ReorganizeLayerSource = varargin{1,1}.ReorganizeLayerSource;
                    this.LossFactors           = varargin{1,1}.LossFactors;
                else             
                    net = this.Network;
                    iValidateNetworkLayers(size(this.ClassNames,1),layerGraph(net));
                    yolov2OutIdx = find(arrayfun(@(x)isa(x,'nnet.cnn.layer.YOLOv2OutputLayer'),...
                        net.Layers),1,'first');
                    lossFactors = net.Layers(yolov2OutIdx).LossFactors;
                    spaceToDepthIdx = find(arrayfun(@(x)isa(x,'nnet.cnn.layer.SpaceToDepthLayer'),...
                        net.Layers),1,'first');
                    if(~isempty(spaceToDepthIdx))
                        dagNetAnalysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(net);
                        spaceToDepthIdx = find(arrayfun(@(x)isa(x,'nnet.cnn.layer.SpaceToDepthLayer'),...
                            dagNetAnalysis.ExternalLayers),1,'first');
                        yolov2ReorgLayerSources = dagNetAnalysis.LayerAnalyzers(spaceToDepthIdx);
                        reorganizeLayerSource = yolov2ReorgLayerSources.Inputs.Source{1};
                    end
                    this.Network = iConvertDAGNetworkTodlNetwork(net);
                    this.ReorganizeLayerSource = reorganizeLayerSource;
                    this.LossFactors = lossFactors;

                end
            elseif(isa(varargin{1,1},'string')||isa(varargin{1,1},'char'))
                % Loads and configure the pretrained model as specified in detectorName
                params = yolov2ObjectDetector.parsePretrainedDetectorInputs(varargin{:});
                this = iTripwireYOLOv2Model(params);
                this.ModelName = params.ModelName;
            else
                %Creates a custom YOLO v2 Object Detector
                narginchk(1,15);
                params = yolov2ObjectDetector.parseDetectorInputs(varargin{:});
                this.Network = params.Network;
                % Create categories of this.ClassNames such that the order of the params.ClassNames is retained.
                this.ClassNames = categorical(params.ClassNames,cellstr(params.ClassNames));
                this.AnchorBoxes = params.AnchorBoxes;
                this.InputSize = params.InputSize;
                this.ModelName = params.ModelName;
                iCheckTrainingImageSize(params.TrainingImageSize,this.Network.Layers(this.LayerIndices.ImageLayerIdx).InputSize(1:2));
                this.TrainingImageSize = params.TrainingImageSize;
                this.LossFactors = params.LossFactors;
                this.ReorganizeLayerSource = params.ReorganizeLayerSource;
            end
            params.ClassNames = this.ClassNames;
            params.AnchorBoxes = this.AnchorBoxes;
            iValidateYOLOv2Network(params,this.Network);
            this.FilterBboxesFunctor = vision.internal.cnn.utils.FilterBboxesFunctor;
        end
    end
    %----------------------------------------------------------------------
    methods
        function this = set.Network(this, params)
            if(isa(params,'dlnetwork'))
                validateattributes(params,{'dlnetwork'},{'scalar'});
                this.Network = params;
                this = this.setLayerIndices(params);
            end

        end

    end
    %----------------------------------------------------------------------
    methods (Static)
        % Create yolov2Datastore for training.
        function ds = createYoloTrainingDatastore(trainingData,dsOpts)
            ds = vision.internal.cnn.yolo.yolov2Datastore(trainingData,dsOpts);
        end

        %------------------------------------------------------------------
        function mapping = createMIMODatastoreMapping(ds, lgraph, params)
            externalLayers = lgraph.Layers;

            % Get layer name based on location in Layers array.
            lossName = externalLayers(params.yoloOutputLayerIdx).Name;
            imgName = externalLayers(params.inputImageIdx).Name;

            dst = {
                imgName
                lossName
                };

            mapping = table(ds.OutputTableVariableNames',dst,...
                'VariableNames',{'Data','Destination'});
        end

        %------------------------------------------------------------------
        function mapping = createMIMODatastoreCellMapping(inputLayerSize)
            % There is one input layer: ImageInputLayer.
            %  - the first column from the read output goes to the input layer.
            inputMapping = {1};
            % There is one output layer: YOLOv2OutputLayer.
            %    - the second column contains responses for this layer.
            outputMapping = {2};

            % Output layer is not a classification layer, but a regression layer.
            classificationOutputs = false;

            inputSizes = {inputLayerSize};
            % We make the outputLayerSize empty, so the dispatcher considers the output observation dimension as 1.
            outputSizes = {[]};

            inputFormats = { deep.internal.PlaceholderArray([inputLayerSize NaN],'SSCB') };
            outputFormats = { deep.internal.PlaceholderArray([NaN NaN NaN NaN],'SSCB') };

            mapping = {inputMapping, outputMapping, classificationOutputs, inputSizes, outputSizes, inputFormats, outputFormats};
        end
    end
    %----------------------------------------------------------------------
    methods (Access = protected)
        %------------------------------------------------------------------
        % Parse and validate detection parameters.
        %------------------------------------------------------------------
        function params = parseDetectInputs(this, I, varargin)

            params.DetectionInputWasDatastore = ~isnumeric(I);

            if params.DetectionInputWasDatastore
                sampleImage = vision.internal.cnn.validation.checkDetectionInputDatastore(I, mfilename);
            else
                sampleImage = I;
            end

            networkInputSize = this.InputSize;

            validateChannelSize = true;  % check if the channel size is equal to that of the network
            validateImageSize   = false; % yolov2 can support images smaller than input size
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

            % Validate minsize and maxsize.
            validateMinSize = ~ismember('MinSize', p.UsingDefaults);
            validateMaxSize = ~ismember('MaxSize', p.UsingDefaults);

            if validateMinSize
                vision.internal.detector.ValidationUtils.checkMinSize(userInput.MinSize, [1,1], mfilename);
            end
            useROI = ~ismember('roi', p.UsingDefaults);
            if validateMaxSize
                vision.internal.detector.ValidationUtils.checkSize(userInput.MaxSize, 'MaxSize', mfilename);
                if useROI
                    coder.internal.errorIf(any(userInput.MaxSize > userInput.roi([4 3])) , ...
                        'vision:yolo:modelMaxSizeGTROISize',...
                        userInput.roi(1,4),userInput.roi(1,3));
                else
                    coder.internal.errorIf(any(userInput.MaxSize > sz(1:2)) , ...
                        'vision:yolo:modelMaxSizeGTImgSize',...
                        sz(1,1),sz(1,2));
                end
            end

            if validateMaxSize && validateMinSize
                coder.internal.errorIf(any(userInput.MinSize >= userInput.MaxSize) , ...
                    'vision:ObjectDetector:minSizeGTMaxSize');
            end

            % Validate ROI.

            if useROI
                vision.internal.detector.checkROI(userInput.roi, sz);
                if ~isempty(userInput.roi)
                    sz = userInput.roi([4 3]);
                    vision.internal.detector.ValidationUtils.checkImageSizes(sz(1:2), userInput, validateMinSize, ...
                        userInput.MinSize, ...
                        'vision:ObjectDetector:ROILessThanMinSize', ...
                        'vision:ObjectDetector:ROILessThanMinSize');
                end
            else
                vision.internal.detector.ValidationUtils.checkImageSizes(sz(1:2), userInput, validateMaxSize, ...
                    userInput.MinSize , ...
                    'vision:ObjectDetector:ImageLessThanMinSize', ...
                    'vision:ObjectDetector:ImageLessThanMinSize');
            end

            % Validate threshold.
            yolov2ObjectDetector.checkThreshold(userInput.Threshold);

            % Validate execution environment.
            exeEnv = vision.internal.cnn.validation.checkExecutionEnvironment(...
                userInput.ExecutionEnvironment, mfilename);

            accel = vision.internal.cnn.validation.checkAcceleration(...
                userInput.Acceleration, mfilename);

            params.ROI                      = single(userInput.roi);
            params.UseROI                   = useROI;
            params.SelectStrongest          = logical(userInput.SelectStrongest);
            params.MinSize                  = single(userInput.MinSize);
            params.MaxSize                  = single(userInput.MaxSize);
            params.MiniBatchSize            = double(userInput.MiniBatchSize);
            params.Threshold                = single(userInput.Threshold);
            params.ExecutionEnvironment     = exeEnv;
            params.Acceleration             = accel;
            params.NetworkInputSize         = networkInputSize;
            params.FilterBboxesFunctor      = this.FilterBboxesFunctor;
            params.FractionDownsampling     = this.FractionDownsampling;
            params.WH2HW                    = this.WH2HW;
            params.ClassNames               = this.ClassNames;
        end

        %------------------------------------------------------------------
        function this = setLayerIndices(this, network)
            this.LayerIndices.ImageLayerIdx = yolov2ObjectDetector.findYOLOv2ImageInputLayer(network.Layers);
        end

        %------------------------------------------------------------------
        % Parse and validate detector parameters.
        %------------------------------------------------------------------

    end

    methods(Static, Hidden,Access = protected)
        function params = parsePretrainedDetectorInputs(varargin)
            % Parse inputs for this syntax:
            % detector = yolov2ObjectDetector(detectorName).
            p = inputParser;
            if size(varargin,2) == 1
                p.addRequired('detectorName');
                supportedNetworks = ["darknet19-coco", "tiny-yolov2-coco"];
                detectorName = validatestring(varargin{1,1}, supportedNetworks, mfilename, 'detectorName', 1);
                if strcmp(detectorName,'darknet19-coco')
                    inpSz = [608,608,3];
                else
                    inpSz = [416,416,3];
                end
                p.addParameter('DetectionNetworkSource','');
                p.addParameter('ReorganizeLayerSource','');
                p.addParameter('InputSize', inpSz);
                p.addParameter('TrainingImageSize','');
                p.addParameter('ModelName', '',@iAssertValidLayerName);
                p.addParameter('LossFactors',[5 1 1 1]);
                parse(p, varargin{:});

            elseif (size(varargin,2) == 3) &&...
                    ((isa(varargin{1,2},'string') && isscalar(varargin{1,2})) || (isa(varargin{1,2},'char') && isrow(varargin{1,2}))) ...
                    && strcmp(varargin{1,2},'ModelName')
                % Parse inputs for this syntax:
                % detector = yolov2ObjectDetector(detectorName,'ModelName',modelName).
                p.addRequired('detectorName');

                supportedNetworks = ["darknet19-coco", "tiny-yolov2-coco"];
                detectorName = validatestring(varargin{1,1}, supportedNetworks, mfilename, 'detectorName', 1);

                if strcmp(detectorName,'darknet19-coco')
                    inpSz = [608,608,3];
                else
                    inpSz = [416,416,3];
                end
                p.addParameter('DetectionNetworkSource','');
                p.addParameter('ReorganizeLayerSource','');
                p.addParameter('InputSize', inpSz);
                p.addParameter('TrainingImageSize','');
                p.addParameter('ModelName', '',@iAssertValidLayerName);
                p.addParameter('LossFactors',[5 1 1 1]);
                parse(p, varargin{:});

            else
                %Parse inputs for this syntax
                % detector = yolov2ObjectDetector(detectorName,classNames,anchorBoxes).
                if (size(varargin,2) >= 3) &&...
                        ((isa(varargin{1,2},'string') && isscalar(varargin{1,2})) || (isa(varargin{1,2},'char') && isrow(varargin{1,2})))
                    if( strcmp(varargin{1,2},'InputSize') || strcmp(varargin{1,2},'DetectionNetworkSource') || strcmp(varargin{1,2},'ModelName')||...
                            strcmp(varargin{1,2},'TrainingImageSize')|| strcmp(varargin{1,2},'LossFactors'))
                        error(message('vision:yolo:requireClassesAndAnchors'));
                    elseif strcmp(varargin{1,2},'ReorganizeLayerSource')
                        error(message('vision:yolo:reorganizeLayerSourceRequiredDNS'));
                    end
                end
                p.addRequired('detectorName');
                p.addRequired('classNames');
                p.addRequired('anchorBoxes');

                supportedNetworks = ["darknet19-coco", "tiny-yolov2-coco"];
                detectorName = validatestring(varargin{1,1}, supportedNetworks, mfilename, 'detectorName', 1);

                if strcmp(detectorName,'darknet19-coco')
                    inpSz = [608,608,3];
                else
                    inpSz = [416,416,3];
                end
                p.addParameter('DetectionNetworkSource','');
                p.addParameter('ReorganizeLayerSource','')
                p.addParameter('InputSize', inpSz);
                p.addParameter('TrainingImageSize','');
                p.addParameter('ModelName', '');
                p.addParameter('LossFactors',[5 1 1 1]);

                parse(p, varargin{:});

                params.ClassNames = p.Results.classNames;

                if ~iscolumn(params.ClassNames)
                    params.ClassNames = params.ClassNames';
                end

                if isstring(params.ClassNames) || iscategorical(params.ClassNames)
                    params.ClassNames = cellstr(params.ClassNames);
                end

                params.AnchorBoxes = p.Results.anchorBoxes;
                iValidateClassNames(params.ClassNames);
                iValidateAnchorBoxes(params.AnchorBoxes);
            end

            params.DetectionNetworkSource = p.Results.DetectionNetworkSource;
            params.ReorganizeLayerSource = p.Results.ReorganizeLayerSource;
            if isempty(params.DetectionNetworkSource) && ~isempty(params.ReorganizeLayerSource)
                error(message('vision:yolo:reorganizeLayerSourceRequiredDNS'));
            end
            params.InputSize = p.Results.InputSize;
            params.TrainingImageSize = p.Results.TrainingImageSize;
            params.DetectorName = char(p.Results.detectorName);
            params.InputSize = iAddChannelIfRequired(params.InputSize);
            params.ModelName = char(p.Results.ModelName);
            params.LossFactors = p.Results.LossFactors;
            if strcmp(params.ModelName,'')
                params.ModelName = p.Results.detectorName;
            end

            if(isempty(params.TrainingImageSize))
                params.TrainingImageSize = params.InputSize(1:2);
            end
        end
        %----------------------------------------------------------------------
        function params = parseDetectorInputs(varargin)
            network = iValidateNetwork(varargin{1,1},mfilename);
            imgIdx = arrayfun(@(x)isa(x,'nnet.cnn.layer.ImageInputLayer'),...
                network.Layers);
            imageInputIdx = find(imgIdx,1,'first');
            p = inputParser;
            p.addRequired('network');

            if ~(any(cellfun(@(varargin) ((isstring(varargin) && strcmp(varargin, "DetectionNetworkSource"))||(ischar(varargin) && strcmp(varargin,'DetectionNetworkSource'))), varargin),2))
                if(isa(network,'DAGNetwork'))
                    warning(message("vision:yolo:networkOutputAsdlnetwork"));
                    yolov2LayerGraph = layerGraph(network);

                    %verify if output layer is present and extract class names and
                    %anchor boxes
                    yolov2OutIdx = find(arrayfun(@(x)isa(x,'nnet.cnn.layer.YOLOv2OutputLayer'),...
                        yolov2LayerGraph.Layers),1,'first');
                    if(~isempty(yolov2OutIdx))
                        ClassNames = yolov2LayerGraph.Layers(yolov2OutIdx).Classes;
                        if ischar(ClassNames)
                            %Check if this needs a better error message like
                            %provide a valid yolov2Network as input
                            error(message("vision:yolo:mustHaveClassNames"));
                        end
                        iValidateNetworkLayers(size(ClassNames,1),yolov2LayerGraph);
                        AnchorBoxes = yolov2LayerGraph.Layers(yolov2OutIdx).AnchorBoxes;
                        p.addOptional('ClassNames',ClassNames);
                        p.addOptional('AnchorBoxes',AnchorBoxes);
                    else
                        error(message("vision:yolo:mustHaveOutputLayer"));
                    end

                else
                    p.addRequired('ClassNames');
                    p.addRequired('AnchorBoxes')
                end
            else
                p.addRequired('ClassNames');
                p.addRequired('AnchorBoxes');


            end
            p.addParameter('DetectionNetworkSource','');
            p.addParameter('ReorganizeLayerSource','');
            p.addParameter('InputSize',network.Layers(imageInputIdx).InputSize);
            p.addParameter('TrainingImageSize','');
            p.addParameter('ModelName','');
            p.addParameter('LossFactors',[5 1 1 1]);

            parse(p, varargin{:});
            params.ClassNames = p.Results.ClassNames;
            params.AnchorBoxes = p.Results.AnchorBoxes;
            params.DetectionNetworkSource = p.Results.DetectionNetworkSource;
            params.ReorganizeLayerSource = p.Results.ReorganizeLayerSource;
            params.InputSize = p.Results.InputSize;
            params.TrainingImageSize = p.Results.TrainingImageSize;
            params.ModelName = char(p.Results.ModelName);
            params.LossFactors = p.Results.LossFactors;



            if(isempty(params.TrainingImageSize))
                params.TrainingImageSize = params.InputSize(1:2);
            end
            net = p.Results.network;

            if isempty(params.InputSize)
                params.InputSize = net.Layers(imageInputIdx).InputSize;
            end
            
            iValidateClassNames(params.ClassNames);
            iValidateAnchorBoxes(params.AnchorBoxes);
            validateattributes(params.DetectionNetworkSource, {'string','char'},{'scalartext'},mfilename,'DetectionNetworkSource');
            iValidateDetectionNetworkSourceExistence(net,params.DetectionNetworkSource);
            if(~isempty(params.ReorganizeLayerSource))
                validateattributes(params.ReorganizeLayerSource,{'string','char'},{'scalartext'},mfilename,'ReorganizeLayerSource');
            end
            %Validate LossFactors
            validateattributes(params.LossFactors,{'numeric'},...
                {'2d','nrows',1,'ncols',4,'ndims',2,'nonempty','nonsparse',...
                'real','finite','nonnan','positive'},mfilename,'LossFactors');

            params.InputSize = iAddChannelIfRequired(params.InputSize);

            if ~iscolumn(params.ClassNames)
                params.ClassNames = params.ClassNames';
            end

            if isstring(params.ClassNames) || iscategorical(params.ClassNames)
                params.ClassNames = cellstr(params.ClassNames);
            end

            %Configuring Network for transfer learning.
            if ~isempty(params.DetectionNetworkSource)
                % validate DetectionNetworkSource
                validateattributes(params.DetectionNetworkSource, {'string','char'},{'scalartext'},mfilename,'DetectionNetworkSource');
                % Update image input layer and first convolution layer in the
                % network.
                lgraph = layerGraph(net);
                lgraph = iUpdateFirstConvChannelsAndInputLayer(lgraph,params.InputSize);
                %Adding Object Detection Network
                params.Network = iConfigureDetector(lgraph,params.ClassNames,params.AnchorBoxes,params.DetectionNetworkSource,params.ReorganizeLayerSource);
            else
                %If the input is a yolov2 network, and the inputSize is same as
                %the network inputSize, then no configuration is needed
                if(isa(net,'DAGNetwork'))
                    iValidateNetworkLayers(size(params.ClassNames,1),layerGraph(net))
                    if(~isempty(params.InputSize) && ~isequal(params.InputSize, net.Layers(imageInputIdx).InputSize))
                        % Update image input layer and first convolution layer in the network
                        lgraph = layerGraph(net);
                        lgraph = iUpdateFirstConvChannelsAndInputLayer(lgraph,params.InputSize);
                        net = assembleNetwork(lgraph);
                    end

                    yolov2OutIdx = find(arrayfun(@(x)isa(x,'nnet.cnn.layer.YOLOv2OutputLayer'),...
                        net.Layers),1,'first');
                    params.LossFactors = net.Layers(yolov2OutIdx).LossFactors;
                    spaceToDepthIdx = find(arrayfun(@(x)isa(x,'nnet.cnn.layer.SpaceToDepthLayer'),...
                        net.Layers),1,'first');
                    if(~isempty(spaceToDepthIdx))
                        dagNetAnalysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(net);
                        spaceToDepthIdx = find(arrayfun(@(x)isa(x,'nnet.cnn.layer.SpaceToDepthLayer'),...
                            dagNetAnalysis.ExternalLayers),1,'first');
                        yolov2ReorgLayerSources = dagNetAnalysis.LayerAnalyzers(spaceToDepthIdx);
                        params.ReorganizeLayerSource = yolov2ReorgLayerSources.Inputs.Source{1};
                    end
                    net = iConvertDAGNetworkTodlNetwork(net);
                    params.Network = net;
                elseif(isa(net,'dlnetwork'))
                    iValidateYOLOv2Network(params,net);
                    if(~isempty(params.InputSize) && ~isequal(params.InputSize, net.Layers(imageInputIdx).InputSize))
                        % Update image input layer and first convolution layer in the
                        % network
                        lgraph = layerGraph(net);
                        lgraph = iUpdateFirstConvChannelsAndInputLayer(lgraph,params.InputSize);
                        net = dlnetwork(lgraph);
                    end
                    params.Network = net;
                    %Initialize ReorganizeLayerSource
                    spaceToDepthIdx = find(arrayfun(@(x)isa(x,'nnet.cnn.layer.SpaceToDepthLayer'),...
                        net.Layers),1,'first');
                    if(~isempty(spaceToDepthIdx))
                        dlNetAnalysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(net);
                        spaceToDepthIdx = find(arrayfun(@(x)isa(x,'nnet.cnn.layer.SpaceToDepthLayer'),...
                            dlNetAnalysis.ExternalLayers),1,'first');
                        yolov2ReorgLayerSources = dlNetAnalysis.LayerAnalyzers(spaceToDepthIdx);
                        params.ReorganizeLayerSource = yolov2ReorgLayerSources.Inputs.Source{1};
                    end
                end
            end

        end
    end

    %======================================================================
    % Save/Load
    %======================================================================
    methods(Hidden)
        function s = saveobj(this)
            s.Version                  = 5.0;
            s.ModelName                = this.ModelName;
            s.Network                  = this.Network;
            s.ClassNames               = this.ClassNames;
            s.AnchorBoxes              = this.AnchorBoxes;
            s.InputSize                = this.InputSize;
            s.TrainingImageSize        = this.TrainingImageSize;
            s.FractionDownsampling     = this.FractionDownsampling;
            s.WH2HW                    = this.WH2HW;
            s.ReorganizeLayerSource    = this.ReorganizeLayerSource;
            s.LossFactors              = this.LossFactors;
        end

        function net = matlabCodegenPrivateNetwork(this)
            net = this.Network;
        end

        function varargout = preprocess(this,I,varargin)
            params = varargin{1,1};
            roi    = params.ROI;
            useROI = params.UseROI;
            trainingImageSize = this.TrainingImageSize;
            if params.DetectionInputWasDatastore
                % Copy and reset the given datastore, so external state events are
                % not reflected.
                ds = copy(I);
                reset(ds);
                fcn = @iPreprocessForDetect;
                % We need just the preprocessed image -> num arg out is 1.
                fcnArgOut = 2;
                varargout{1} = transform(ds, @(x)iPreProcessForDatastoreRead(x,fcn,fcnArgOut,params.ExecutionEnvironment,...
                    roi,useROI,trainingImageSize));
                varargout{2} = {};
            else
                [varargout{1:nargout}] = iPreprocessForDetect(I,roi,useROI,trainingImageSize,params.ExecutionEnvironment);
            end

        end

        function features = predict(this,dlX,varargin)
            params = varargin{1,1};
            anchors = this.AnchorBoxes;
            if params.DetectionInputWasDatastore
                features = iPredictUsingDatastore(dlX,this.Network,params,anchors);
            else
                features = iGetFeaturesUsingActivations(this.Network,dlX,params);
            end

        end

        function varargout = postprocess(this,features,info,params)
            if params.DetectionInputWasDatastore
                [varargout{1:nargout}] = features;
            else
                if params.DetectionInputWasBatchOfImages
                    [varargout{1:nargout}] = iPostProcessBatchActivations(features, info, this.AnchorBoxes, params);
                else
                    [varargout{1:nargout}] = iPostProcessActivations(features, info, this.AnchorBoxes, params);
                end
            end
        end

    end

    methods(Static, Hidden)
        function this = loadobj(s)
            try
                vision.internal.requiresNeuralToolbox(mfilename);
                switch s.Version
                    case 1 % <= 19b
                        s = iUpgradeToVersionFour(s);
                        s.FractionDownsampling = false;
                        s.WH2HW = true;
                    case 2 % == 19b update 1
                        s = iUpgradeToVersionFour(s);
                        s.WH2HW = true;
                    case 3
                        s = iUpgradeToVersionFour(s);
                        s.WH2HW = true;
                    otherwise
                        % no-op.
                end
                if(s.Version<5)
                    net = s.Network;
                    yolov2OutIdx = find(arrayfun(@(x)isa(x,'nnet.cnn.layer.YOLOv2OutputLayer'),...
                        net.Layers),1,'first');
                    lossFactors = net.Layers(yolov2OutIdx).LossFactors;
                    spaceToDepthIdx = find(arrayfun(@(x)isa(x,'nnet.cnn.layer.SpaceToDepthLayer'),...
                        net.Layers),1,'first');
                    if(~isempty(spaceToDepthIdx))
                        dagNetAnalysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(net);
                        spaceToDepthIdx = find(arrayfun(@(x)isa(x,'nnet.cnn.layer.SpaceToDepthLayer'),...
                            dagNetAnalysis.ExternalLayers),1,'first');
                        yolov2ReorgLayerSources = dagNetAnalysis.LayerAnalyzers(spaceToDepthIdx);
                        reorganizeLayerSource =  yolov2ReorgLayerSources.Inputs.Source{1};
                    else
                        % Replacing YOLOv2ReorgLayer with SpaceToDepthLayer in old networks if present 
                        yolov2ReorgIdx = find(arrayfun(@(x)isa(x,'nnet.cnn.layer.YOLOv2ReorgLayer'),...
                            net.Layers),1,'first');
                        if(~isempty(yolov2ReorgIdx))
                            dagNetAnalysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(net);
                            yolov2ReorgIdx = find(arrayfun(@(x)isa(x,'nnet.cnn.layer.YOLOv2ReorgLayer'),...
                                dagNetAnalysis.ExternalLayers),1,'first');
                            yolov2ReorgLayerSources = dagNetAnalysis.LayerAnalyzers(yolov2ReorgIdx);
                            reorganizeLayerSource =  yolov2ReorgLayerSources.Inputs.Source{1};
                            blockSize = yolov2ReorgLayerSources.InternalLayer.BlockSize;
                            name = yolov2ReorgLayerSources.InternalLayer.Name;
                            spaceToDepthLyr = spaceToDepthLayer(blockSize,'Name',name);
                            lg = layerGraph(net);
                            lg_updated = replaceLayer(lg,name,spaceToDepthLyr);
                            s.Network = assembleNetwork(lg_updated);
                        else
                            reorganizeLayerSource = '';
                        end

                    end
                    dlnet = iConvertDAGNetworkTodlNetwork(s.Network);
                    this = yolov2ObjectDetector(dlnet,s.ClassNames,s.AnchorBoxes);
                    this.ModelName = s.ModelName;
                    this.FractionDownsampling = s.FractionDownsampling;
                    this.WH2HW = s.WH2HW;
                    this.TrainingImageSize = s.TrainingImageSize;
                    this.ReorganizeLayerSource = reorganizeLayerSource;
                    this.LossFactors = lossFactors;
                else
                    this = yolov2ObjectDetector(s.Network,s.ClassNames,s.AnchorBoxes);
                    this.ModelName = s.ModelName;
                    this.FractionDownsampling = s.FractionDownsampling;
                    this.WH2HW = s.WH2HW;
                    this.TrainingImageSize = s.TrainingImageSize;
                    if(isfield(s,'InputSize'))
                        this.InputSize = s.InputSize;
                    end
                    if(isfield(s,'ReorganizeLayerSource'))
                        this.ReorganizeLayerSource = s.ReorganizeLayerSource;
                    end
                    if(isfield(s,'LossFactors'))
                        this.LossFactors = s.LossFactors;
                    end

                end
            catch ME
                rethrow(ME)
            end
        end

        function n = matlabCodegenDlRedirect(~)
            n = 'vision.internal.codegen.YOLOv2Network';
        end
    end
    %----------------------------------------------------------------------
    % Assemble detector object for training.
    %----------------------------------------------------------------------
    methods(Hidden, Static)
        function detector = assembleDetector(params,net)
            dlnet    = iConvertDAGNetworkTodlNetwork(net);
            detector = yolov2ObjectDetector(dlnet,params.ClassNames,params.AnchorBoxes,'InputSize',params.InputSize);
            detector.ModelName   = params.ModelName;
            detector.TrainingImageSize = params.TrainingImageSize;
            detector.LossFactors = params.LossFactors;
            detector.ReorganizeLayerSource = params.ReorganizeLayerSource;
        end

        %------------------------------------------------------------------
        % Construct detector object for quantization.
        %------------------------------------------------------------------
        function detector = constructDetector(params,net)
            ImgInputSize = params.InputSize;
            detector = yolov2ObjectDetector(net,params.ClassNames,params.AnchorBoxes,'InputSize',ImgInputSize);
            detector.ModelName   = params.ModelName;
            % Retain the values of FractionDownsampling and WH2HW for
            % quantized networks.
            detector.FractionDownsampling     = params.FractionDownsampling;
            detector.WH2HW                    = params.WH2HW;
        end

        %------------------------------------------------------------------
        % Update class names of yolov2OutputLayer in network.
        %------------------------------------------------------------------
        function updatedLgraph = updateNetworkClasses(lgraph, classNames)
            % yolov2OutputLayer created without class names. In such cases
            % the classes are obtained from input.
            outputLayerIdx = ...
                arrayfun(@(x)isa(x,'nnet.cnn.layer.YOLOv2OutputLayer'), ...
                lgraph.Layers);
            outputLayer = lgraph.Layers(outputLayerIdx,1);
            outputLayer.Classes = classNames;
            updatedLgraph = replaceLayer(lgraph,outputLayer.Name,outputLayer);
        end
        %------------------------------------------------------------------
        % Validate Threshold value.
        %------------------------------------------------------------------
        function checkThreshold(threshold)
            validateattributes(threshold, {'single', 'double'}, {'nonempty', 'nonnan', ...
                'finite', 'nonsparse', 'real', 'scalar', '>=', 0, '<=', 1}, ...
                mfilename, 'Threshold');
        end

        %------------------------------------------------------------------
        % Find indexes of ImageInputLayer in network.
        %------------------------------------------------------------------
        

        function imageInputIdx = findYOLOv2ImageInputLayer(externalLayers)
            imageInputIdx = find(...
                arrayfun( @(x)isa(x,'nnet.cnn.layer.ImageInputLayer'), ...
                externalLayers));
        end

    end
end

%--------------------------------------------------------------------------
function s = iDefaultDetectionParams()
s.roi                     = zeros(0,4);
s.SelectStrongest         = true;
s.Threshold               = 0.5;
s.MinSize                 = [1,1];
s.MaxSize                 = [];
s.MiniBatchSize           = 128;
s.ExecutionEnvironment    = 'auto';
end

%--------------------------------------------------------------------------
function network = iValidateNetwork(network,mfilename)
validateattributes(network,{'DAGNetwork','dlnetwork'},...
    {'scalar'}, mfilename);
end
%--------------------------------------------------------------------------
function S = iUpgradeToVersionFour(S)
% iUpgradeToVersionThree   Upgrade a v1 or v2 or v3 saved struct to a v4
% saved struct.
% This means adding class names to yolov2OutputLayer.
S.Version = 4;
lgraph = layerGraph(S.Network);
updatedLgraph = yolov2ObjectDetector.updateNetworkClasses(lgraph, S.ClassNames);
updatedNetwork = assembleNetwork(updatedLgraph);
S.Network = updatedNetwork;
end

%--------------------------------------------------------------------------
function iValidateNetworkLayers(numClasses,yolov2LayerGraph)
iVerifyFullyConnectedExistence(yolov2LayerGraph);
iVerifyGlobalAvgPoolExistence(yolov2LayerGraph);
analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(yolov2LayerGraph);
constraint = vision.internal.cnn.analyzer.constraints.YOLOv2Architecture(numClasses);
out = nnet.internal.cnn.analyzer.constraints.Constraint.getBuiltInConstraints();
archConstraint = arrayfun(@(x)isa(x,'nnet.internal.cnn.analyzer.constraints.Architecture'),out);
out(archConstraint) = constraint;
analysis.applyConstraints(out);
try
    analysis.throwIssuesIfAny();
catch ME
    throwAsCaller(ME);
end
end

%--------------------------------------------------------------------------
function iCheckTrainingImageSize(TrainingImageSize,networkImageSize)
validateattributes(TrainingImageSize, {'numeric'}, ...
    {'2d','ncols',2,'ndims',2,'nonempty','nonsparse',...
    'real','finite','integer','positive'});
if any(TrainingImageSize < networkImageSize,'all')
    error(message('vision:yolo:multiScaleInputError'))
end
end

%--------------------------------------------------------------------------
function clippedBBox = iClipBBox(bbox, imgSize)

clippedBBox  = double(bbox);

x1 = clippedBBox(:,1);
y1 = clippedBBox(:,2);

x2 = clippedBBox(:,3);
y2 = clippedBBox(:,4);

x1(x1 < 1) = 1;
y1(y1 < 1) = 1;

x2(x2 > imgSize(2)) = imgSize(2);
y2(y2 > imgSize(1)) = imgSize(1);

clippedBBox = [x1 y1 x2 y2];
end

%--------------------------------------------------------------------------
function [targetSize, sx, sy] = iFindNearestTrainingImageSize(sz,trainingImageSize)
if(size(trainingImageSize,2)==3)
    trainingImageSize = trainingImageSize(:,1:2);
end
idx = iComputeBestMatch(sz(1:2),trainingImageSize);
targetSize = trainingImageSize(idx,:);

% Compute scale factors to scale boxes from targetSize back to the input
% size.
scale   = sz(1:2)./targetSize;
[sx,sy] = deal(scale(2),scale(1));
end

%--------------------------------------------------------------------------
% Get the index of nearest size in TrainingImageSize training sizes that
% matches given image.
%--------------------------------------------------------------------------
function ind = iComputeBestMatch(preprocessedImageSize,trainingImageSize)
preprocessedImageSize = repmat(preprocessedImageSize,size(trainingImageSize,1),1);
Xdist = (preprocessedImageSize(:,1) - trainingImageSize(:,1));
Ydist = (preprocessedImageSize(:,2) - trainingImageSize(:,2));
dist = sqrt(Xdist.^2 + Ydist.^2);
[~,ind] = min(dist);
end

%--------------------------------------------------------------------------
% Process image with network and outputPredictions in the following format:
% outputPredictions(:, 1:4)   - output boxes in [x1 y1 x2 y2] format.
% outputPredictions(:, 5)     - scores in M-by-1 vector format.
% outputPredictions(:, 6:end) - labels in M-by-1 vector format.
%--------------------------------------------------------------------------
function outputPrediction = iPredictUsingFeatureMap(featureMap, threshold, preprocessedImageSize, anchorBoxes, fractionDownsampling, wh2hw)

gridSize = size(featureMap);

featureMap = permute(featureMap,[2 1 3 4]);
featureMap = reshape(featureMap,gridSize(1)*gridSize(2),gridSize(3),1,[]);
featureMap = reshape(featureMap,gridSize(1)*gridSize(2),size(anchorBoxes,1),gridSize(3)/size(anchorBoxes,1),[]);
featureMap = permute(featureMap,[2 3 1 4]);

% This is to maintain backward compatibility with version 1 detectors.
if fractionDownsampling
    downsampleFactor = preprocessedImageSize(1:2)./gridSize(1:2);
else
    downsampleFactor = floor(preprocessedImageSize(1:2)./gridSize(1:2));
end

if wh2hw
    anchorBoxes = [anchorBoxes(:,2),anchorBoxes(:,1)];
end

% Scale anchor boxes with respect to feature map size
anchorBoxes = anchorBoxes./downsampleFactor;

% Extract IoU, class probabilities from feature map.
iouPred = featureMap(:,1,:,:);
sigmaXY = featureMap(:,2:3,:,:);
expWH = featureMap(:,4:5,:,:);
probPred = featureMap(:,6:end,:,:);

% Compute bounding box coordinates [x,y,w,h] with respect to input image
% dimension.
boxOut = nnet.internal.cnn.layer.util.yoloPredictBBox(sigmaXY, expWH, anchorBoxes, gridSize(1:2), downsampleFactor);

boxOut = permute([boxOut,iouPred,probPred],[2 1 3 4]);
boxOut = reshape(boxOut,size(boxOut,1),[]);
boxOut = permute(boxOut,[2 1 3 4]);

% Extract box coordinates, iou, class probabilities.
bboxesX1Y1X2Y2 = boxOut(:,1:4);
iouPred = boxOut(:,5);
probPred = boxOut(:,6:end);
[imax,idx] = max(probPred,[],2);
confScore = iouPred.*imax;
boxOut = [bboxesX1Y1X2Y2,confScore,idx,probPred];
outputPrediction = boxOut(confScore>=threshold,:);
end

%--------------------------------------------------------------------------
function detectionResults = iPredictUsingDatastore(ds, network, params, anchorBoxes)

loader = iCreateDataLoader(ds,params.MiniBatchSize,params.NetworkInputSize);

% Iterate through data and write results to disk.
k = 1;

bboxes = cell(params.MiniBatchSize, 1);
scores = cell(params.MiniBatchSize, 1);
labels = cell(params.MiniBatchSize, 1);

while hasdata(loader)
    X = nextBatch(loader);
    imgBatch = X{1};
    batchInfo = X{2};
    numMiniBatch = size(batchInfo,1);
    features = iGetFeaturesUsingActivations(network,imgBatch,params);
    for ii = 1:numMiniBatch
        fmap = features(:,:,:,ii);
        [bboxes{k},scores{k},labels{k}] = ...
            iPostProcessActivations(fmap, batchInfo{ii}, anchorBoxes, params);
        k = k + 1;
    end
end

varNames = {'Boxes', 'Scores', 'Labels'};
detectionResults = table(bboxes(1:k-1), scores(1:k-1), labels(1:k-1), 'VariableNames', varNames);
end

%-----------------------------------------------------------------------
function features = iGetFeaturesUsingActivations(network,imgBatch,params)
try
    features = predict(network,imgBatch,...
        'Acceleration',params.Acceleration);

catch ME
    if strcmp(ME.identifier,'nnet_cnn:layer:BatchNormalizationLayer:NotFinalized')
        error(message('vision:yolo:unableToDetect'));
    else
        throwAsCaller(ME);
    end
end
end

%-----------------------------------------------------------------------
function [bboxes, scores, labels, intermediates] = iPostProcessBatchActivations(features, info, anchorBoxes, params)
numMiniBatch = size(features,4);
bboxes = cell(numMiniBatch, 1);
scores = cell(numMiniBatch, 1);
labels = cell(numMiniBatch, 1);
intermediates = cell(numMiniBatch, 1);

for ii = 1:numMiniBatch
    fmap = features(:,:,:,ii);
    [bboxes{ii},scores{ii},labels{ii},intermediates{ii}] = ...
        iPostProcessActivations(fmap, info, anchorBoxes, params);
end
end

%-----------------------------------------------------------------------
function [bboxes, scores, labels, intermediates] = iPostProcessActivations(featureMap, info, anchorBoxes, params)

featureMap = extractdata(gather(featureMap));
outputPrediction = iPredictUsingFeatureMap(featureMap, params.Threshold, info.PreprocessedImageSize, anchorBoxes, params.FractionDownsampling, params.WH2HW);
if ~isempty(outputPrediction)

    bboxesX1Y1X2Y2 = outputPrediction(:,1:4);
    scorePred = outputPrediction(:,5);
    classAndProbPred = outputPrediction(:,6:end);

    % ClipBoxes to boundaries.
    bboxesX1Y1X2Y2 = iClipBBox(bboxesX1Y1X2Y2, info.PreprocessedImageSize);

    % Scale boxes back to size(Iroi).
    bboxesX1Y1X2Y2 = vision.internal.cnn.boxUtils.scaleX1X2Y1Y2(bboxesX1Y1X2Y2, info.ScaleX, info.ScaleY);

    % Convert [x1 y1 x2 y2] to [x y w h].
    bboxPred = vision.internal.cnn.boxUtils.x1y1x2y2ToXYWH(bboxesX1Y1X2Y2);

    % Filter boxes based on MinSize, MaxSize. Append classPred with
    % probPred for filtering
    [bboxPred, scorePred, classAndProbPred] = filterBBoxes(params.FilterBboxesFunctor,...
        params.MinSize,params.MaxSize,bboxPred,scorePred,classAndProbPred);

    % Apply NMS.
    if params.SelectStrongest
        [bboxes, scores, classNames, index] = selectStrongestBboxMulticlass(bboxPred, scorePred, classAndProbPred(:,1), ...
            'RatioType', 'Union', 'OverlapThreshold', 0.5);
        classProbability = classAndProbPred(index,2:end);
        objectnessScore = scores./max(classProbability,[],2);
    else
        bboxes = bboxPred;
        scores = scorePred;
        classNames = classAndProbPred(:,1);
        classProbability = classAndProbPred(:,2:end);
        objectnessScore = scorePred./max(classProbability,[],2);
    end

    % Apply ROI offset
    bboxes(:,1:2) = vision.internal.detector.addOffsetForROI(bboxes(:,1:2), params.ROI, params.UseROI);

    % Convert classId to classNames.
    labels = categorical(params.ClassNames);
    labels = labels(classNames,1);

    % Return the class probabilities and objectness score as a struct
    intermediates.ClassProbabilities = classProbability;
    intermediates.ObjectnessScores = objectnessScore;
else
    bboxes = zeros(0,4,'double');
    scores = zeros(0,1,'single');
    labels = categorical(cell(0,1),cellstr(params.ClassNames));
    intermediates = struct.empty;
end

end

%--------------------------------------------------------------------------
function B = iAugmentData(A)
% Apply random horizontal flipping, and random X/Y scaling. Boxes that get
% scaled outside the bounds are clipped if the overlap is above 0.25. Also,
% jitter image color.
B = cell(size(A));

I = A{1};
sz = size(I);
if numel(sz)==3 && sz(3) == 3
    I = jitterColorHSV(I,...
        'Contrast',0.2,...
        'Hue',0,...
        'Saturation',0.1,...
        'Brightness',0.2);
end

% Randomly flip and scale image.
tform = randomAffine2d('XReflection',true,'Scale',[1 1.1]);
rout = affineOutputView(sz,tform,'BoundsStyle','CenterOutput');
B{1} = imwarp(I,tform,'OutputView',rout);

% Apply same transform to boxes.
[B{2},indices] = bboxwarp(A{2},tform,rout,'OverlapThreshold',0.25);
B{3} = A{3}(indices);

% Return original data only when all boxes are removed by warping.
if isempty(indices)
    B = A;
end
end

%--------------------------------------------------------------------------
function A = iAppendResponsesForTraining(A,outputSize)

% Append label IDs to each box.
A{2} = [A{2} double(A{3})];

% Remove undefined boxes.
A{2}(ismissing(A{3}),:) = [];

% Append output size to each box.
N = size(A{2},1);
A{2} = [A{2} repelem(outputSize(1:2),N,1)];

% Delete labels
A(3) = [];
end

%--------------------------------------------------------------------------
function [I,varargout] = iPreprocess(outputSize,I,varargin)
% Resize image and boxes, then normalize image data between 0 and 1.
sz = size(I);
I = imresize(I,outputSize(1:2));
I = vision.internal.cnn.yolo.yolov2Datastore.normalizeImageAndCastToSingle(I);
if numel(varargin) > 1
    % Resize boxes using the same scale factor as the image resize.
    scale = outputSize(1:2)./sz(1:2);
    varargout{1} = bboxresize(varargin{1},scale);
end
end

%--------------------------------------------------------------------------
function out = iPreProcessForDatastoreRead(in, fcn, numArgOut,executionEnvironment,varargin)
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
    [out{ii, 1:numArgOut}] = fcn(in{ii},varargin{1,1},varargin{1,2},varargin{1,3},executionEnvironment);
end
end

%--------------------------------------------------------------------------
function [Ipreprocessed,info] = iPreprocessForDetect(I, roi, useROI, trainingImageSize,executionEnvironment)
% Crop image if requested.
Iroi = vision.internal.detector.cropImageIfRequested(I, roi, useROI);

% Find the nearest training image size.
[info.PreprocessedImageSize,info.ScaleX,info.ScaleY] = iFindNearestTrainingImageSize(...
    size(Iroi),trainingImageSize);


Ipreprocessed = yolov2ObjectDetector.preprocessforDetect(Iroi,info.PreprocessedImageSize);
if (strcmp(executionEnvironment,'auto') && canUseGPU) || strcmp(executionEnvironment,'gpu')
    Ipreprocessed = gpuArray(Ipreprocessed);
end
if (isa(Ipreprocessed,'double')||isa(Ipreprocessed,'single')||isa(Ipreprocessed,'gpuArray'))
    Ipreprocessed = dlarray(Ipreprocessed,'SSCB');
end
end

%--------------------------------------------------------------------------
function loader = iCreateDataLoader(ds,miniBatchSize,inputLayerSize)
loader = nnet.internal.cnn.DataLoader(ds,...
    'MiniBatchSize',miniBatchSize,...
    'CollateFcn',@(x)iTryToBatchData(x,inputLayerSize));
end

%--------------------------------------------------------------------------
function data = iTryToBatchData(X, inputLayerSize)
try
    observationDim = numel(inputLayerSize) + 1;
    data{1} = cat(observationDim,X{:,1});
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
function detector = iTripwireDefaultYOLOv2Model()
%Check if support package is installed
breadcrumbFile = 'vision.internal.cnn.supportpackages.IsYOLOv2Installed';
fullPath = which(breadcrumbFile);
if isempty(fullPath)
    name     = 'Computer Vision Toolbox Model for YOLO v2 Object Detection';
    basecode = 'YOLOV2';
    throwAsCaller(MException(message('nnet_cnn:supportpackages:InstallRequired', mfilename, name, basecode)));
else
    % Load pretrained network.
    pattern = fullfile(filesep, '+vision','+internal','+cnn','+supportpackages','IsYOLOv2Installed.m');
    idx     = strfind(fullPath, pattern);
    matfile = fullfile(fullPath(1:idx), 'data', 'darknet19-coco.mat');
    data = load(matfile);
    detector = data.detector;
end
end
%--------------------------------------------------------------------------
function detector = iTripwireYOLOv2Model(params)
%Check if support package is installed
breadcrumbFile = 'vision.internal.cnn.supportpackages.IsYOLOv2Installed';
fullPath = which(breadcrumbFile);
if isempty(fullPath)
    name     = 'Computer Vision Toolbox Model for YOLO v2 Object Detection';
    basecode = 'YOLOV2';
    throwAsCaller(MException(message('nnet_cnn:supportpackages:InstallRequired', mfilename, name, basecode)));
else
    if(isfield(params,'ClassNames'))
        data = iLoadPretrainedNetwork(fullPath, params.DetectorName);
        network = data.detector.Network;
        anchorBoxes = params.AnchorBoxes;
        classes = params.ClassNames;
        inputSize = params.InputSize;
        if isempty(params.DetectionNetworkSource)
            if strcmp(params.DetectorName, 'darknet19-coco')
                layersToReplace = {'conv2d_23','yolov2Transform'};
                reorganizeLayerSource = 'leaky_relu_21';
            else
                layersToReplace = {'conv2d_9','yolov2Transform'};
                reorganizeLayerSource = '';
            end

            network = iConfigureNetwork(network,anchorBoxes,classes,layersToReplace,inputSize);
        else
           reorganizeLayerSource = params.ReorganizeLayerSource;
        end

        detector = yolov2ObjectDetector(network,classes,anchorBoxes,'ModelName',params.ModelName,...
            'InputSize',params.InputSize,'DetectionNetworkSource',params.DetectionNetworkSource,...
            'ReorganizeLayerSource',reorganizeLayerSource,'LossFactors',params.LossFactors,'TrainingImageSize',params.TrainingImageSize);

        if(isempty(params.DetectionNetworkSource))
            detector.ReorganizeLayerSource = reorganizeLayerSource;
        end

    %Load pretrained network.
    else
        data = iLoadPretrainedNetwork(fullPath,params.DetectorName);
        detector = data.detector;
        detector.ModelName = params.ModelName;
    end
end
end
%-----------------------------------------------------------------------
function lgraph = iAddDetectionSubNetwork(lgraph,AnchorBoxes,ClassNames,DetectionNetworkSource,names)
% Create yolo v2 detection subnetwork and add after featureLayer
% Select featureLayer output size filters as YOLO v2 detection
% sub-network convolution layer filters.
analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(lgraph);
featureLayerIdx = arrayfun(@(x) x.Name == ...
    DetectionNetworkSource,analysis.LayerAnalyzers);

% Verify that YOLOv2 featureLayer output size is greater than [1,1].
activationSize = analysis.LayerAnalyzers(featureLayerIdx).Inputs.Size{1,1};
if (any(activationSize(1:2) < 2))
    error(message("vision:yolo:mustHaveValidFinalActivationsSize"));
end

outFilters = analysis.LayerAnalyzers(featureLayerIdx).Outputs.Size{1}(3);

% Create new layers specific to YOLO v2.
yolov2Conv1 = convolution2dLayer(3,outFilters,'Name',names.yolov2Conv1,...
    'Padding', 'same',...
    'WeightsInitializer',iNormalWeightInit());

yolov2Batch1 = batchNormalizationLayer('Name',names.yolov2Batch1);
yolov2Relu1 = reluLayer('Name',names.yolov2Relu1);

yolov2Conv2 = convolution2dLayer(3,outFilters,'Name',names.yolov2Conv2,...
    'Padding', 'same',...
    'WeightsInitializer',iNormalWeightInit());

yolov2Batch2 = batchNormalizationLayer('Name',names.yolov2Batch2);
yolov2Relu2 = reluLayer('Name',names.yolov2Relu2);

% YOLO v2 predicts Box co-ordinate (x,y,w,h), objectness score for
% every anchor box.
yolov2Predictions = 5;
NumClasses = size(ClassNames,1);
numFilters = size(AnchorBoxes,1)*(NumClasses + yolov2Predictions);
yolov2ClassConv = convolution2dLayer(1,numFilters,'Name',...
    names.yolov2ClassConv,...
    'WeightsInitializer', iNormalWeightInit());
numAnchors = size(AnchorBoxes,1);

yolov2Transform = yolov2TransformLayer(numAnchors,'Name',names.yolov2Transform);

yolov2Layers = [yolov2Conv1;yolov2Batch1;yolov2Relu1;...
    yolov2Conv2;yolov2Batch2;yolov2Relu2;...
    yolov2ClassConv;yolov2Transform];
lgraph = addLayers(lgraph,yolov2Layers);
lgraph = connectLayers(lgraph,DetectionNetworkSource,names.yolov2Conv1);
end
%--------------------------------------------------------------------------
function names = iDefaultLayerNames()
% Default names of yolo detection subnetwork.
names.yolov2Conv1 = "yolov2Conv1";
names.yolov2Batch1 = "yolov2Batch1";
names.yolov2Relu1 = "yolov2Relu1";
names.yolov2Conv2 = "yolov2Conv2";
names.yolov2Batch2 = "yolov2Batch2";
names.yolov2Relu2 = "yolov2Relu2";
names.yolov2ClassConv = "yolov2ClassConv";
names.yolov2OutputLayer = "yolov2OutputLayer";
names.spaceToDepth = "spaceToDepth";
names.reorgConcat = "reorgConcat";
names.yolov2Transform = "yolov2Transform";
end

%--------------------------------------------------------------------------
function names = iChooseUniqueLayerNames(lgraph)
% Loop over names in lgraph and choose new layer names that do not
% conflict.
names = iDefaultLayerNames();
alreadyUsedNames = string({lgraph.Layers.Name});
fnames = fieldnames(names);
for i = 1:numel(fnames)
    inuse = alreadyUsedNames == names.(fnames{i});
    if any(inuse)
        % append _1 to make unique name.
        names.(fnames{i}) = names.(fnames{i}) + "_1";
    end
end
end
%--------------------------------------------------------------------------
function network = iConfigureDetector(lgraph,ClassNames,AnchorBoxes,detectionNetworkSource,reorganizeLayerSource)
if ~isempty(reorganizeLayerSource)
    lgraph = iRemoveLayers(lgraph,detectionNetworkSource);
    iVerifyFullyConnectedExistence(lgraph);
    % Verfiy Global average pooling layer does not exist in the network.
    iVerifyGlobalAvgPoolExistence(lgraph);
    % Choose unique names for yolo v2 specific layers.
    names = iChooseUniqueLayerNames(lgraph);
    lgraph = iAddDetectionSubNetwork(lgraph,AnchorBoxes,ClassNames,detectionNetworkSource,names);
    % Calculate spaceToDepth layer blocksize and validate depth concatenation layer
    % inputs.
    iValidateReorgLayerSourceExistence(lgraph,reorganizeLayerSource);
    reorgStride = iCalculateReorgStrideAndValidate(lgraph,reorganizeLayerSource,detectionNetworkSource);
    % Add SpaceToDepth layer and depth concatenation layer to lgraph.
    lgraph = iAddReorgAndDepthConcat(lgraph,reorganizeLayerSource,reorgStride,names);
else
    lgraph = iRemoveLayers(lgraph,detectionNetworkSource);
    iVerifyFullyConnectedExistence(lgraph);
    % Verfiy Global average pooling layer does not exist in the network.
    iVerifyGlobalAvgPoolExistence(lgraph);
    % Choose unique names for yolo v2 specific layers.
    names = iChooseUniqueLayerNames(lgraph);
    lgraph = iAddDetectionSubNetwork(lgraph,AnchorBoxes,ClassNames,detectionNetworkSource,names);
end
network = dlnetwork(lgraph);
end
%-------------------------------------------------------------------------


function iVerifyGlobalAvgPoolExistence(lgraph)
% YOLOv2 network should not contain any global average pooling layer as it
% downsamples input feature map to size of [1,1].
idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.GlobalAveragePooling2DLayer')||isa(x,'nnet.cnn.layer.GlobalMaxPooling2DLayer'),...
    lgraph.Layers);
if sum(idx) ~= 0
    error(message("vision:yolo:mustNotHaveAnyGlobalPoolingLayer"));
end
end

%--------------------------------------------------------------------------
function iVerifyFullyConnectedExistence(lgraph)
% YOLOv2 network is based on Convolution Layers and should not
% contain any fullyConnected Layers.
idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.FullyConnectedLayer'),...
    lgraph.Layers);
if sum(idx) ~= 0
    error(message("vision:yolo:mustNotHaveAnyFCLayer"));
end
end
%--------------------------------------------------------------------------
function lgraph= iRemoveLayers(lgraph,detectionNetSource)
% Remove all the layers after detectionNetworkSource.
dg = vision.internal.cnn.RCNNLayers.digraph(lgraph);

% Find the feature extraction nodes to remove nodes after feature
% extraction layer.

for i = 1:size(detectionNetSource,2)
    % Verify that all detectionNetworkSource layers exist in lgraph.
    iVerifyLayerExist(lgraph, detectionNetSource);
end

% Remove layers after last feature extraction layer
% Remove any lingering branches to removed layers in lgraph
allLayers = {lgraph.Layers.Name};
dg = flipedge(dg);
layersBefore = dfsearch(dg,detectionNetSource);
% Remove lingering branches along with layer after layerNames
lgraph = removeLayers(lgraph, allLayers(~(ismember(allLayers, layersBefore))));

end
%--------------------------------------------------------------------------
function iVerifyLayerExist(lgraph, layerName)

numLayers = numel(lgraph.Layers);
foundLayer = false;
for lIdx = 1:numLayers
    if strcmp(layerName, lgraph.Layers(lIdx).Name)
        foundLayer = true;
        break;
    end
end
if ~foundLayer
    error(message('vision:yolo:InvalidLayerName', layerName));
end
end
%--------------------------------------------------------------------------
function w = iNormalWeightInit()
w = @(sz)randn(sz)*0.01;
end
%--------------------------------------------------------------------------
function network = iConfigureNetwork(network,anchorBoxes,classes, layersToReplace,inputSize)
lgraph = layerGraph(network);
lgraph = iUpdateFirstConvChannelsAndInputLayer(lgraph,inputSize);
numAnchors = size(anchorBoxes,1);
numClasses = numel(classes);
numFilters = (numClasses + 5)*numAnchors;
%Calculating no.of Channels in Conv layer to be replaced,to Initalize
%Weights accordingly
ClassConv = lgraph.Layers({lgraph.Layers.Name}==string(layersToReplace{1,1}));
numChannels = ClassConv.NumChannels;
convOut = convolution2dLayer([1,1],numFilters,'Padding','same','WeightsInitializer',iNormalWeightInit(),'Name',['class','convOut']);
convOut.Weights = zeros(1,1,numChannels,numFilters);
convOut.Bias = zeros(1,1,numFilters);
lgraph = replaceLayer(lgraph,layersToReplace{1,1},convOut);
yoloV2Transform = yolov2TransformLayer(numAnchors,'Name','yolov2Transform');
lgraph = replaceLayer(lgraph,layersToReplace{1,2},yoloV2Transform);
network = dlnetwork(lgraph);
end
%--------------------------------------------------------------------------
function lgraph = iUpdateFirstConvChannelsAndInputLayer(yolov2Lgraph,imageSize)
% This function update the channels of first conv layer if InputSize channel
% does not match with channels of first conv layer. It also update the
% imageInputLayer or initialize the dlnetwork if image input layer not present.

if size(imageSize,2)==2
    imageSize = [imageSize 1];
end
lgraph = yolov2Lgraph;
imgIdx = arrayfun(@(x)isa(x,'nnet.cnn.layer.ImageInputLayer'),...
    lgraph.Layers);
imageInputIdx = find(imgIdx,1,'first');

imageInput = imageInputLayer(imageSize,...
    'Name',lgraph.Layers(imageInputIdx).Name,'Normalization','none');

lgraph = replaceLayer(lgraph,lgraph.Layers(imageInputIdx).Name,...
    imageInput);

numChannel = imageSize(3);

idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.Convolution2DLayer'),...
    lgraph.Layers);
convIdx = find(idx,1,'first');
if ~isempty(convIdx)
    numFirstConvLayerChannels = lgraph.Layers(convIdx,1).NumChannels;
else
    error(message('vision:yolo:mustHaveConvLayers'));
end

% If number of channels in image is greater than the channel count of first
% convolutional layer. Update the channel count of first conv layer.Pyramid
% pooling concept has been used for concatenating extra channel. Each extra
% channel is mean of original (initial) channels of conv layer
%
% Zhao, Hengshuang, et al. "Pyramid Scene Parsing Network." 2017 IEEE
% Conference on Computer Vision and Pattern Recognition (CVPR). IEEE, 2017.
if (~strcmp(numFirstConvLayerChannels,'auto'))
    if numFirstConvLayerChannels~=numChannel
        firstConvLayer = lgraph.Layers(convIdx,1);
        firstConvLayerWeights = firstConvLayer.Weights;

        % Compute mean of first conv layer weights.
        meanChannelWeights = reshape(mean(firstConvLayerWeights,3),size(firstConvLayerWeights(:,:,1,:)));
        if (numChannel > numFirstConvLayerChannels)
            extraChanels = abs(numChannel-numFirstConvLayerChannels);
            extraChannelWeights = repmat(meanChannelWeights,1,1,extraChanels);

            % Concatenate mean of first conv layer weights as the additional channels.
            updatedConvLayerWeights = cat(3,firstConvLayerWeights,extraChannelWeights);
        else

            % Replicate mean of first conv layer weights as the additional channels.
            updatedConvLayerWeights = repmat(meanChannelWeights,1,1,numChannel);
        end

        % Configure conv layer with new parameters
        updatedConvLayer = convolution2dLayer(firstConvLayer.FilterSize, firstConvLayer.NumFilters, 'NumChannels', numChannel, ...
            'Stride',firstConvLayer.Stride,...
            'Padding',firstConvLayer.PaddingSize , ...
            'PaddingValue',firstConvLayer.PaddingValue,...
            'DilationFactor', firstConvLayer.DilationFactor, ...
            'Weights',updatedConvLayerWeights,...
            'Bias',firstConvLayer.Bias,...
            'WeightL2Factor',firstConvLayer.WeightL2Factor,...
            'BiasL2Factor',firstConvLayer.BiasL2Factor,...
            'WeightLearnRateFactor',firstConvLayer.WeightLearnRateFactor,...
            'BiasLearnRateFactor',firstConvLayer.BiasLearnRateFactor,...
            'Name', firstConvLayer.Name, ...
            'WeightsInitializer', firstConvLayer.WeightsInitializer, ...
            'BiasInitializer', firstConvLayer.BiasInitializer);
        lgraph = replaceLayer(lgraph,lgraph.Layers(convIdx).Name,...
            updatedConvLayer);
    end
end

% Validate that replaced layer does not cause any issue.
analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(lgraph);
analysis.applyConstraints();
if ~analysis.LayerAnalyzers(2,1).IsLayerInputValid
    error(message('vision:yolo:InvalidLayerSize', ...
        mat2str(analysis.LayerAnalyzers(2,1).Inputs.Size{1}),...
        mat2str([analysis.LayerAnalyzers(2,1).Inputs.Size{1}(1:2),...
        analysis.LayerAnalyzers(2,1).Learnables.Size{1}(3)])));
end
end
%-----------------------------------------------------------------------
function data =  iLoadPretrainedNetwork(fullPath, detectorName)
pattern = fullfile(filesep, '+vision','+internal','+cnn','+supportpackages','IsYOLOv2Installed.m');
idx     = strfind(fullPath, pattern);
if strcmp(detectorName, 'darknet19-coco')
    matfile = fullfile(fullPath(1:idx), 'data', 'darknet19-coco.mat');
else
    matfile = fullfile(fullPath(1:idx), 'data', 'tinyYOLOv2-coco.mat');
end
data = load(matfile);
end
%-----------------------------------------------------------------------
function iValidateAnchorBoxes(value)
%Validate that Anchor Boxes is a Mx2 matrix
validateattributes(value, {'numeric'},{'2d','size',[NaN 2],'real','nonnan','finite','positive'}, ...
    mfilename, 'AnchorBoxes');
end
%-----------------------------------------------------------------------
function iValidateClassNames(value)
if ~isvector(value) || ~iIsValidDataType(value)
    error(message('vision:yolo:invalidClasses'));
end
if iHasDuplicates(value)
    error(message('vision:yolo:duplicateClasses'));
end
if isempty(value)
    error(message('vision:yolo:invalidClasses'));
end
end
%-----------------------------------------------------------------------
function newSize = iAddChannelIfRequired(inputSize)
validateattributes(inputSize, {'numeric'}, ...
    {'2d','nonempty','nonsparse',...
    'real','finite','integer','positive','nrows',1});

if (size(inputSize,2)==2)
    newSize = [inputSize,1];
else
    newSize = inputSize;
end

if ~(size(newSize,2)>=3)
    error(message('vision:yolo:invalidInputSize'));
end

end

%-----------------------------------------------------------------------
function tf = iIsValidDataType(value)
tf = iscategorical(value) || iscellstr(value) || isstring(value);
end

%-----------------------------------------------------------------------
function tf = iHasDuplicates(value)
tf = ~isequal(value, unique(value, 'stable'));
end
%-----------------------------------------------------------------------
function reorgStride = iCalculateReorgStrideAndValidate(lgraph,ReorgLayerSource,DetectionNetworkSource)
% Calculate reorg layer stride and validate depth-concatenation layer
% inputs using network analyzer.
analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(lgraph);
reorgInputSizeIdx = arrayfun(@(x) x.Name == ReorgLayerSource,analysis.LayerAnalyzers);
reorgOutSizeIdx = arrayfun(@(x) x.Name == DetectionNetworkSource,analysis.LayerAnalyzers);
reorgInputSize = analysis.LayerAnalyzers...
    (reorgInputSizeIdx).Outputs.Size{1};
reorgOutSize   = analysis.LayerAnalyzers...
    (reorgOutSizeIdx).Outputs.Size{1};
reorgStride =  floor(reorgInputSize(1:2)./reorgOutSize(1:2));
actual = floor(reorgInputSize(1:2)./reorgStride);
expected = reorgOutSize(1:2);
if ~(actual == expected)
    error(message('vision:yolo:DepthConcatInputMisMatch', ...
        mat2str(expected), mat2str(ImageSize)));
end
end
%-----------------------------------------------------------------------
function lgraph = iAddReorgAndDepthConcat(lgraph,ReorgLayerSource,reorgStride,names)
% Add reorg and depth concatenation layer to the lgraph.
spaceToDepth = spaceToDepthLayer(reorgStride,'Name',names.spaceToDepth);
depthConcat = depthConcatenationLayer(2,'Name',names.reorgConcat);
lgraph = addLayers(lgraph,depthConcat);
lgraph = addLayers(lgraph,spaceToDepth);
lgraph = connectLayers(lgraph,names.yolov2Relu1,strcat(names.reorgConcat,'/in1'));
lgraph = connectLayers(lgraph,ReorgLayerSource,names.spaceToDepth);
lgraph = connectLayers(lgraph,names.spaceToDepth,strcat(names.reorgConcat,'/in2'));
lgraph = disconnectLayers(lgraph,names.yolov2Relu1,names.yolov2Conv2);
lgraph = connectLayers(lgraph,names.reorgConcat,names.yolov2Conv2);
end
%--------------------------------------------------------------------------
function iAssertValidLayerName(name)
nnet.internal.cnn.layer.paramvalidation.validateLayerName(name);
end
%--------------------------------------------------------------------------
function iValidateYOLOv2Network(params,network)

layerNames = string({network.Layers.Name});
% Verfiy Fully Connected layer does not exist in the network.
iVerifyFullyConnectedExistence(network);
iVerifyGlobalAvgPoolExistence(network);
% Verify only one transformLayer exists
layers = network.Layers;
idx = find(arrayfun(@(x)isa(x,'nnet.cnn.layer.YOLOv2TransformLayer'),layers));
if numel(idx) == 0
    error(message("vision:yolo:mustHaveTransformLayer"));
elseif numel(idx) > 1
    error(message("vision:yolo:mustHaveOnlyOneTransformLayer"));
else
    % No error.
end

%Verify that YOLOv2 has only one image input layer.
inputLayersIdx = find(arrayfun(@(x)isa(x,'nnet.cnn.layer.ImageInputLayer'),layers));
if numel(inputLayersIdx) == 0
    error(message("vision:yolo:mustHaveInputLayer"));
elseif numel(inputLayersIdx) > 1
    error(message("vision:yolo:mustHaveOnlyOneInputLayer"));
else
    if ~isa(layers(inputLayersIdx),'nnet.cnn.layer.ImageInputLayer')
        error(message("vision:yolo:mustBeImageInputLayer"));
    end
end
%Compute the number of filters of last convolution layer
numAnchors = size(params.AnchorBoxes,1);
numClasses = size(params.ClassNames,1);
numPredictionsPerAnchor = 5 + numClasses;
expectedFilters = numAnchors*numPredictionsPerAnchor;
layerIdx = find(strcmp(network.OutputNames{1,1},layerNames));
idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.Convolution2DLayer'),network.Layers(1:layerIdx));
filterIdx = find(idx,1,'last');
actualFilters = network.Layers(filterIdx,1).NumFilters;

if ~(expectedFilters == actualFilters)
    error(message('vision:yolo:invalidNumFilters',mat2str(expectedFilters),...
        mat2str(numAnchors),mat2str(numClasses)));
end
% Verify that YOLOv2 final activations size is greater than [1,1]
dlnetAnalysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(network);
yolov2TransformIdx = find(arrayfun(@(x)isa(x,'nnet.cnn.layer.YOLOv2TransformLayer'),...
    dlnetAnalysis.ExternalLayers),1,'first');
yolov2Transform = dlnetAnalysis.LayerAnalyzers(yolov2TransformIdx);
outputSize = yolov2Transform.Inputs.Size{1,1};
if(isempty(outputSize)||any(outputSize(1:2)<2))
    error(message("vision:yolo:mustHaveValidFinalActivationsSize"))
end

end
%--------------------------------------------------------------------------
function dlnet = iConvertDAGNetworkTodlNetwork(dagNet)
lgraph = layerGraph(dagNet);
yolov2OutIdx = find(arrayfun(@(x)isa(x,'nnet.cnn.layer.YOLOv2OutputLayer'),...
    lgraph.Layers),1,'first');
lgraph_Updated = removeLayers(lgraph,lgraph.Layers(yolov2OutIdx).Name);
dlnet = dlnetwork(lgraph_Updated);
end
%--------------------------------------------------------------------------
function iValidateDetectionNetworkSourceExistence(net,featureLayer)

analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(net);
% Validate featureLayer exists in the network or not.
if ~isempty(featureLayer)
    featureLayerSizeIdx = arrayfun(@(x) x.Name == ...
        featureLayer,analysis.LayerAnalyzers);
    if sum(featureLayerSizeIdx)==0
        error(message...
            ('vision:yolo:LayerDoesNotExist',...
            featureLayer));
    end
end
end
%--------------------------------------------------------------------------
function iValidateReorgLayerSourceExistence(lgraph,reorgLayerSource)
% Validate reorgLayerSource exists in the network or not.
if ~isempty(reorgLayerSource)
    analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(lgraph);
    reorgInputSizeIdx = arrayfun(@(x) x.Name == ...
        reorgLayerSource,analysis.LayerAnalyzers);
    if sum(reorgInputSizeIdx)==0
        error(message...
            ('vision:yolo:LayerDoesNotExist',...
            reorgLayerSource));
    end
end
end