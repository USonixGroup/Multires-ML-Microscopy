%yolov4ObjectDetector Detect objects using YOLO v4 deep learning detector.
%
% detector = yolov4ObjectDetector() loads a YOLO v4 object detector trained
% to detect 80 object classes from the COCO dataset using a csp-darknet53
% network.
%
% detector = yolov4ObjectDetector(detectorName) loads a pretrained YOLO v4
% object detector specified by detectorName. detectorName must be
% 'csp-darknet53-coco' or 'tiny-yolov4-coco'.
%
% detector = yolov4ObjectDetector(detectorName, classNames, anchorBoxes)
% configures a pretrained YOLO v4 object detector for transfer learning
% with a new set of object classes and anchor boxes. You must train the
% detector for optimal performance.
%
% detector = yolov4ObjectDetector(network, classNames, anchorBoxes)
% creates a YOLO v4 object detector by using the specified YOLO v4 deep
% learning network and configures for training with a new set of object
% classes and anchor boxes. The network must be a dlnetwork. Use this syntax
% to specify a custom YOLO v4 network for object detection. You must train
% the detector for optimal performance.
%
% detector = yolov4ObjectDetector(network, classNames, anchorBoxes, 'DetectionNetworkSource', layerName)
% creates a YOLO v4 object detector by using the specified deep learning
% network as feature extraction network. This syntax will add detection
% subnetworks to the layers of the feature extraction network specified
% in DetectionNetworkSoruce. You must train the detector for optimal performance.
%
% Inputs:
% -------
%    detectorName   Specify the name of the pretrained YOLO v4 deep learning
%                   network as a string or character vector. The value must
%                   be one of the following:
%
%                       'csp-darknet53-coco'    To use a YOLO v4 deep learning
%                                               network that has CSP-Darknet-53
%                                               as base network and is trained
%                                               on COCO dataset.
%
%                       'tiny-yolov4-coco'      To use a YOLO v4 deep learning
%                                               network that has a smaller base
%                                               network and is trained on COCO
%                                               dataset.
%
%    network        Specify the input network as a dlnetwork object that has
%                   an image input layer. 'Normalization' in the image input
%                   layer must be 'none'. The outputs of the network is used
%                   for predicting object detections. The network must have
%                   the same number of outputs as the number of anchorBoxes.
%                   To select which network outputs to use for prediction,
%                   use the DetectionNetworkSource parameter.
%
%    classNames     Specify the names of object classes that the YOLO v4
%                   object detector is configured to detect. classNames can
%                   be a string vector, a categorical vector, or a cell
%                   array of character vectors.
%
%    anchorBoxes    Specify the anchor boxes as a cell array of size M-by-1.
%                   For more information about anchorBoxes, see
%                   <a href="matlab:helpview('vision','yolov4anchorbox')">Anchor Boxes for YOLO v4 Object Detector</a>.
%
% % Additional input arguments
% ----------------------------
% [...] = yolov4ObjectDetector(..., Name=Value) specifies additional
% name-value pair arguments described below:
%
%   'ModelName'                      Specify the name for the object detector
%                                    as a string or character vector.
%
%                                    Default: '' or specified detectorName
%
%   'InputSize'                      Specify the image size used for training
%                                    the detector. The input size must be
%                                    H-by-W or H-by-W-by-C.
%
%                                    Default: network input size
%
%   'PredictedBoxType'               Specify the type of the bounding boxes
%                                    to be returned by the object detector.
%                                    The value must be either "axis-aligned"
%                                    for axis-aligned bounding boxes or
%                                    "rotated" for rotated rectangle bounding
%                                    boxes.
%
%                                    Default: "axis-aligned"
%
%   'DetectionNetworkSource'         Specify the names of layers in the input
%                                    network to which you want to add the
%                                    detection subnetworks. The value must
%                                    be a string vector or cell array of
%                                    character vectors of size 1-by-M. If it
%                                    is not specified or empty, then
%                                    sub-networks will not be added to the
%                                    input network and it is expected to be
%                                    a complete yolo v4 network with detection
%                                    subnetwork.
%
%                                    Default: {}
%
% yolov4ObjectDetector properties:
%   ModelName                    - Name of the trained object detector.
%   Network                      - YOLO v4 object detection network. (read-only)
%   ClassNames                   - A cell array of object class names. (read-only)
%   AnchorBoxes                  - A cell array of anchor boxes. (read-only)
%   InputSize                    - Image size used during training. (read-only)
%   PredictedBoxType             - Bounding box format of the detector's prediction. (read-only)
%
% yolov4ObjectDetector methods:
%   detect                       - Detect objects in an image.
%
% Example 1: Detect objects using pretrained YOLO v4 detector.
% ------------------------------------------------------------
% % Load the pretrained detector.
% detector = yolov4ObjectDetector();
%
% % Read test image.
% I = imread('highway.png');
%
% % Run detector.
% [bboxes, scores, labels] = detect(detector, I);
%
% % Display results.
% detectedImg = insertObjectAnnotation(I, 'Rectangle', bboxes, labels);
% figure
% imshow(detectedImg)
%
% Example 2: Detect objects using 'tiny-yolov4-coco' pretrained model.
% --------------------------------------------------------------------
% % Load the pretrained detector.
% detector = yolov4ObjectDetector('tiny-yolov4-coco');
%
% % Read test image.
% I = imread('highway.png');
%
% % Run detector.
% [bboxes, scores, labels] = detect(detector, I);
%
% % Display results.
% detectedImg = insertObjectAnnotation(I, 'Rectangle', bboxes, labels);
% figure
% imshow(detectedImg)
%
% Example 3: Configure a pretrained YOLO v4 detector for transfer learning.
% -------------------------------------------------------------------------
% % Specify the input image size.
% imageSize = [224 224 3];
%
% % Specify the class names.
% classes = {'car','person'};
%
% % Specify the anchor boxes.
% anchorBoxes = {[122,177;223,84;80,94];...
%               [111,38;33,47;37,18]};
%
% % Configure the detector.
% detector = yolov4ObjectDetector('tiny-yolov4-coco',classes,anchorBoxes,...
%      'InputSize',imageSize);
%
% See also trainYOLOv4ObjectDetector,ssdObjectDetector
%          yolov2ObjectDetector, yolov3ObjectDetector maskrcnn,
%          rcnnObjectDetector, fastRCNNObjectDetector,
%          fasterRCNNObjectDetector, imageLabeler.

% Copyright 2021-2024 The MathWorks, Inc.

classdef yolov4ObjectDetector < vision.internal.detector.ObjectDetector  &...
        deep.internal.sdk.LearnableParameterContainer &...
        images.dltrain.internal.ValidationTimeInferable

    properties(SetAccess = protected, Hidden)
        % Backbone is a dlnetwork object with image input layer and
        % backbone layers.
        Backbone;

        % Neck is a dlnetwork object with SPP and PAN network as Neck
        % layers.
        Neck;

        % Head is a cell array of dlnetwork objects for each detection heads.
        Head;
    end

    properties(SetAccess = protected)
        % Network is a dlnetwork object with image input layer.
        Network

        % AnchorBoxes is a N-by-1 cell array where each cell element contains
        % an M-by-2 array, where M is number of anchor boxes, N is either
        % network.Outputs or the number of elements in DetectionNetworkSource
        % and M corresponds to array of anchor boxes in [height width] format.
        AnchorBoxes

        % ClassNames specifies the names of the classes that YOLO v4 object
        % detector can detect.
        ClassNames

        % InputSize is a vector of the form [height width] or [height width channels]
        % defining image size used to train the detector. During detection,
        % an input image is resized to this size before it is processed by
        % the detection network.
        InputSize

        % PredictedBoxType is a string that specifies the type of the bounding
        % boxes returned by the object detector. The type of bounding box a
        % detector returns depends on the training data used to train the
        % object detector. PredictedBoxType is specified as "axis-aligned"
        % for axis-aligned bounding boxes or "rotated" for rotated rectangle
        % bounding boxes.
        PredictedBoxType
    end

    properties(SetAccess = protected, Hidden)
        % DetectionNetworkSource is a cell array
        % specifying the names of layers in the input network to
        % which detection subnetworks are connected.
        DetectionNetworkSource
    end

    properties(Dependent = true, Hidden = true)
        % These properties are accesed in dltrain during training the
        % detector.

        % Learnables is the learnable parameters for the YOLO v4 dlnetwork.
        Learnables

        % State is the state of the non-learnable parameters of the
        % YOLO v4 dlnetwork.
        State

        % Layers is the array of layers in the YOLO v4 dlnetwork.
        Layers

        % InputNames is the cell array of input names for the YOLO v4 dlnetwork.
        InputNames

        % OutputNames is the cell array of output names for the YOLO v4 dlnetwork.
        OutputNames
    end

    properties(Access = private, Transient)
        numHeads
    end

    properties (Access = protected, Transient)
        FilterBboxesFunctor
    end

    properties (Access = private)
        % FreezeBackboneAndNeckInternal is a logical array
        % containing freezing properties for detector.
        % FreezeBackboneAndNeckInternal(1) is for freezing backbone.
        % FreezeBackboneAndNeckInternal(2) is for frezing backboneAndNeck.
        FreezeBackboneAndNeckInternal = [false; false];
    end

    properties (Dependent,Hidden)
        FreezeBackbone
        FreezeBackboneAndNeck
    end

    % This is deep.internal.sdk.LearnableParameterContainer interface
    methods(Access=protected)
        function self = updateLearnableParameters(self, updater)
            data = self.Learnables;
            data = updater.apply(data);
            self.Learnables = data;
        end
    end

    % This is images.dltrain.internal.ValidationTimeInferable inferface
    methods (Hidden)
        function results = predictForValidation(self,X)
            if isdlarray(X)
                X = extractdata(X);
            end

            X = gather(X);

            [boxes,scores,labels] = detect(self,X,Threshold=0.001);

            % Detect interface switches conventions from cell containment
            % for batchSize>1 and numeric for batchSize==1. We want a
            % consistent convention for validation so always cell contain.
            if isnumeric(boxes)
                boxes = {boxes};
                scores = {scores};
                labels = {labels};
            end
            
            results = table(boxes,labels,scores);
        end

        function N = numOutputsPredictForValidation(self)
            N = 1;
        end
    end

    methods
        function this = yolov4ObjectDetector(varargin)
            vision.internal.requiresNeuralToolbox(mfilename);
            narginchk(0,11);

            if nargin < 1
                % Loads the default pretrained model.
                this = iTripwireDefaultYOLOv4Model;

            elseif (isa(varargin{1,1},'string')||isa(varargin{1,1},'char'))
                % Loads and configure the pretrained model as specified in detectorName.
                params = yolov4ObjectDetector.parsePretrainedDetectorInputs(varargin{:});
                this = iTripwireYOLOv4Model(params);

            else
                % Creates custom YOLO v4 object detector.
                narginchk(3,11);
                params = yolov4ObjectDetector.parseDetectorInputs(varargin{:});
                this.Network = params.Network;
                this.ClassNames = params.ClassNames;
                this.AnchorBoxes = params.AnchorBoxes;
                this.InputSize = params.InputSize;
                this.ModelName = params.ModelName;
                this.PredictedBoxType = params.PredictedBoxType;
                this.DetectionNetworkSource = params.DetectionNetworkSource;

            end

            this.FilterBboxesFunctor = vision.internal.cnn.utils.FilterBboxesFunctor;
        end
    end

    methods
        function varargout = detect(detector, I, varargin)
            % bboxes = detect(detector,I) detects objects within the image I.
            % The location of objects within I are returned in bboxes,
            % either in axis-aligned rectangle or rotated rectangle format.
            % Axis-aligned bboxes are an M-by-4 matrix defining M bounding
            % boxes. Each row of axis-aligned bboxes contain a four-element
            % vector, [x, y, width, height]. This vector specifies the
            % upper-left corner and size of a bounding box in pixels. When
            % the PredictedBoxType property is set to "rotated", detector
            % returns rotated rectangle bboxes as an M-by-5 matrix defining
            % M bounding boxes. Each row of rotated rectangle bboxes contain
            % a five-element vector, [xctr, yctr, width, height, yaw]. This
            % vector specifies the center coordinate, size, and angle of
            % rotation, in degrees, of a bounding box in spatial coordinates.
            % yaw is an angle between +/-180 degrees and increases positively
            % in the clockwise direction with 0 degrees being located on the
            % right side of an unrotated rectangle. Once 180 degrees of
            % rotation is reached, the angle of rotation will wrap to -180
            % degrees. detector is a yolov4ObjectDetector object and I is a
            % truecolor or grayscale or multi-channel image.
            %
            % [..., scores] = detect(detector,I) optionally return the class
            % specific confidence scores for each bounding box. The scores
            % for each detection is product of objectness prediction and
            % classification scores. The range of the scores is [0 1].
            % Larger score values indicate higher confidence in the
            % detection.
            %
            % [..., labels] = detect(detector,I) optionally return the labels
            % assigned to the bounding boxes in an M-by-1 categorical
            % array. The labels used for object classes is defined during
            % training.
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
            % detectionResults = detect(yolo,ds) detects objects within the
            % series of images returned by the read method of datastore,
            % ds. ds, must be a datastore that returns a table or a cell
            % array with the first column containing images.
            % detectionResults is a 3-column table with variable names
            % 'Boxes', 'Scores', and 'Labels' containing bounding boxes,
            % scores, and the labels. The location of objects within an
            % image, I are returned in bounding boxes, either an M-by-4
            % matrix for axis-aligned bounding boxes or an M-by-5 matrix
            % for rotated rectangle bounding boxes. For axis-aligned bboxes,
            % each row of boxes contains a four-element vector,
            % [x, y, width, height]. This vector specifies the upper-left
            % corner and size of a bounding box in pixels. For rotated
            % rectangle bboxes, each row of boxes contains a five-element
            % vector, [xctr, yctr, width, height, yaw]. This vector
            % specifies the center coordinate, size, and angle of rotation,
            % in degrees, of a bounding box in spatial coordinates. yaw is
            % an angle between +/-180 degrees and increases positively in
            % the clockwise direction. Once 180 degrees of rotation is
            % reached, the angle of rotation will wrap to -180 degrees.
            % yolo is a yolov4ObjectDetector object.
            %
            % [...] = detect(..., roi) optionally detects objects within
            % the rectangular search region specified by roi. roi must be a
            % 4-element vector, [x, y, width, height], that defines a
            % rectangular region of interest fully contained in I. When the
            % input is a datastore, the same roi is applied to every image.
            %
            % [...] = detect(..., Name=Value) specifies additional
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
            %                          YOLO v4 detector. Valid values are:
            %
            %                           'auto'      Use a GPU if it is available,
            %                                       otherwise use the CPU.
            %
            %                           'cpu'       Use the CPU.
            %
            %                           'gpu'       Use the GPU. To use a GPU,
            %                                       you must have Parallel
            %                                       Computing Toolbox(TM), and
            %                                       a CUDA-enabled NVIDIA GPU.
            %                                       If a suitable GPU is not
            %                                       available, an error message
            %                                       is issued.
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
            %    detector resizes the input image to the detector.InputSize.
            %
            % Class Support
            % -------------
            % The input image I can be uint8, uint16, int16, double,
            % single, gpuArray, and it must be real and non-sparse.
            %
            % Example
            % -------
            % % Load pre-trained detector.
            % detector = yolov4ObjectDetector('tiny-yolov4-coco');
            %
            % % Read test image.
            % I = imread('highway.png');
            %
            % % Run detector.
            % [bboxes, scores, labels] = detect(detector, I);
            %
            % % Display results.
            % detectedImg = insertObjectAnnotation(I, 'Rectangle', bboxes, labels);
            % figure
            % imshow(detectedImg)
            %
            % See also yolov2ObjectDetector, yolov4ObjectDetector.

            params = parseDetectInputs(detector,I,varargin{:});
            [varargout{1:nargout}] = performDetect(detector, I, params);
        end
    end

    %----------------------------------------------------------------------
    methods
        function val = get.Layers(this)
            val = this.Network.Layers;
        end
        %------------------------------------------------------------------
        function val = get.Learnables(this)
            if isempty(this.Backbone) && isempty(this.Neck) && isempty(this.Head)
                val = this.Network.Learnables;
            else
                val = {};
                if this.FreezeBackbone
                    val = vertcat(this.Neck.Learnables);
                end

                headLearnables = cellfun(@(head){head.Learnables},this.Head,'UniformOutput',true);
                val = vertcat(val,headLearnables{:});
            end
        end

        function this = set.Learnables(this,val)
            if isempty(this.Backbone) && isempty(this.Neck) && isempty(this.Head)
                this.Network.Learnables = val;
            else
                learnableIdx=0;
                if this.FreezeBackbone
                    numNeckLearnables = size(this.Neck.Learnables,1);
                    this.Neck.Learnables = val(1:numNeckLearnables,:);
                    learnableIdx = numNeckLearnables;
                end

                numHeadLearnables = cellfun(@(head){size(head.Learnables,1)},this.Head,'UniformOutput',true);
                for i=1:this.numHeads
                    this.Head{i}.Learnables = val(learnableIdx+1:learnableIdx+numHeadLearnables{i},:);
                    learnableIdx = learnableIdx + numHeadLearnables{i};
                end
            end
        end

        %------------------------------------------------------------------
        function val = get.State(this)
            if isempty(this.Backbone) && isempty(this.Neck) && isempty(this.Head)
                val = this.Network.State;
            else
                val = {};
                if this.FreezeBackbone
                    val = vertcat(this.Neck.State);
                end
                headState = cellfun(@(head){head.State},this.Head,'UniformOutput',true);
                val = vertcat(val,headState{:});
            end
        end

        function this = set.State(this,val)
            if isempty(this.Backbone) && isempty(this.Neck) && isempty(this.Head)
                this.Network.State = val;
            else
                stateIdx=0;
                if this.FreezeBackbone
                    numNeckState = size(this.Neck.State,1);
                    this.Neck.State = val(1:numNeckState,:);
                    stateIdx = numNeckState;
                end

                numHeadState = cellfun(@(head){size(head.State,1)},this.Head,'UniformOutput',true);
                for i=1:this.numHeads
                    this.Head{i}.State = val(stateIdx+1:stateIdx+numHeadState{i},:);
                    stateIdx = stateIdx + numHeadState{i};
                end
            end
        end

        %------------------------------------------------------------------
        function val = get.InputNames(this)
            val = this.Network.InputNames;
        end

        %------------------------------------------------------------------
        function val = get.OutputNames(this)
            val = this.Network.OutputNames;
        end

        %------------------------------------------------------------------
        function this = set.FreezeBackbone(this,TF)
            this.FreezeBackboneAndNeckInternal(1,1) = TF;
        end

        function TF = get.FreezeBackbone(this)
            TF = this.FreezeBackboneAndNeckInternal(1,1);
        end

        function this = set.FreezeBackboneAndNeck(this,TF)
            this.FreezeBackboneAndNeckInternal(2,1) = TF;
        end

        function TF = get.FreezeBackboneAndNeck(this)
            TF = this.FreezeBackboneAndNeckInternal(2,1);
        end

    end

    %----------------------------------------------------------------------
    methods(Hidden)
        % -----------------------------------------------------------------
        % Initialize sub-networks in detector
        % -----------------------------------------------------------------
        function this = initialize(this)
            dlX = dlarray(rand(this.InputSize, 'single'), 'SSCB');
            backboneFeatures = cell(size(this.Backbone.OutputNames'));
            [backboneFeatures{:}] = predict(this.Backbone, dlX);
            if(~this.Neck.Initialized)
                this.Neck = initialize(this.Neck, backboneFeatures{:});
            end
            neckFeatures = cell(size(this.Neck.OutputNames'));
            [neckFeatures{:}] = predict(this.Neck,backboneFeatures{:});
            for i=1:this.numHeads
                if(~this.Head{i}.Initialized)
                    this.Head{i} = initialize(this.Head{i},...
                        neckFeatures{i});
                end
            end
        end

        % -----------------------------------------------------------------
        % Update Network from sub-networks
        % -------------------------------------------------------------------
        function this = updateDetector(this)
            if ~isempty(this.Backbone) && ~isempty(this.Neck) && ~isempty(this.Head)
                this.Network = iDlNetwork(this);
            end
        end
        %-----------------------------------------------------------------
        % Creates sub-networks while training
        % ----------------------------------------------------------------
        function this = setSubNetworks(this)
            if ~isempty(this.DetectionNetworkSource)
                [this.Backbone, this.Head, this.Neck] = iSubNetworks(this,this.DetectionNetworkSource);
                this.numHeads = size(this.Head,2);
                % Initialize sub networks
                this = initialize(this);
            end
        end
        %-----------------------------------------------------------------
        % Empty sub-networks after training
        % ----------------------------------------------------------------
        function this = unsetSubNetworks(this)
            this.Backbone = [];
            this.Neck = [];
            this.Head = [];
            this.numHeads = [];
        end
        %-----------------------------------------------------------------
        % set DetectionNetworkSource for a detector 
        %-----------------------------------------------------------------
        function this = setDetectionNetworkSource(this,detectionNetworkSource)
            this.DetectionNetworkSource = detectionNetworkSource;
        end
    end

    methods(Hidden)

        %------------------------------------------------------------------
        % Preprocess input data.
        %------------------------------------------------------------------
        function varargout = preprocess(detector, I, varargin)
            % This method preprocesses the input data prior to calling
            % the predict method. It resizes the input data to the
            % detector.InputSize and scales the pixels in between 0 and 1.

            params = parsePreprocessInputs(detector, I, varargin);
            if params.DetectionInputIsDatastore
                % Copy and reset the given datastore, so external state events are
                % not reflected.
                ds = copy(I);
                reset(ds);

                fcn = @iPreprocessForDetect;
                % We need just the preprocessed image -> num arg out is 1.
                fcnArgOut = 2;
                varargout{1} = transform(ds, @(x)iPreProcessForDatastoreRead(x,fcn,fcnArgOut,...
                    params.ROI,params.UseROI,params.ExecutionEnvironment, ...
                    detector.InputSize,params.CastToGpuArray));
                varargout{2} = {};
            else
                [varargout{1:nargout}] = iPreprocessForDetect(I, ...
                    params.ROI, params.UseROI, params.ExecutionEnvironment, ...
                    detector.InputSize,params.CastToGpuArray);
            end
        end

        %------------------------------------------------------------------
        % Predict output feature maps.
        %------------------------------------------------------------------
        function outputFeatures = predict(detector,dlX,varargin)
            % This method predicts features of the preprocessed image dlX.
            % The outputFeatures is a N-by-8 or N-by-10 cell array, where
            % N are the number of outputs in detector.Network. Each cell of
            % outputFeature contains predictions from an output layer.

            predictParams = parsePredictInputs(detector,varargin);
            network = detector.Network;
            anchorBoxes = detector.AnchorBoxes;
            isRotatedBox = strcmp(detector.PredictedBoxType, "rotated");

            if (~isnumeric(dlX) && ~iscell(dlX))

                % Process datastore with network and output the predictions.
                loader = iCreateDataLoader(dlX,predictParams.MiniBatchSize,predictParams.NetworkInputSize);

                % Iterate through data and write results to disk.
                k = 1;

                bboxes = cell(predictParams.MiniBatchSize, 1);
                scores = cell(predictParams.MiniBatchSize, 1);
                labels = cell(predictParams.MiniBatchSize, 1);

                while hasdata(loader)
                    X = nextBatch(loader);
                    imgBatch = X{1};
                    batchInfo = X{2};
                    numMiniBatch = size(batchInfo,1);

                    % Compute predictions.
                    features = iPredictActivations(network, imgBatch, anchorBoxes, predictParams.Acceleration, isRotatedBox);

                    rotationOffset = 2*double(isRotatedBox);

                    for ii = 1:numMiniBatch
                        fmap = cell(size(network.OutputNames'));
                        for i = 1:(8 + rotationOffset)
                            for j = 1:size(fmap,1)
                                fmap{j,i} = features{j,i}(:,:,:,ii);
                            end
                        end
                        [bboxes{k,1},scores{k,1},labels{k,1}] = ...
                            postprocess(detector,fmap, batchInfo{ii}, varargin{1,1});
                        k = k + 1;
                    end
                end
                outputFeatures = cell(1,3);
                outputFeatures{1,1} = bboxes(1:k-1);
                outputFeatures{1,2} = scores(1:k-1);
                outputFeatures{1,3} = labels(1:k-1);

            else

                if iscell(dlX)
                    outputFeatures = iPredictBatchActivations(network, dlX, anchorBoxes, predictParams.Acceleration, isRotatedBox);
                else
                    outputFeatures = iPredictActivations(network, dlX, anchorBoxes, predictParams.Acceleration, isRotatedBox);
                end
            end
        end

        %------------------------------------------------------------------
        % Compute forward activations.
        %------------------------------------------------------------------

        function varargout = forward(detector,dlX)
            % This method computes the activations and state information from
            % the network for preprocessed input data dlX. These outputs
            % are used during training.


            arguments
                detector
            end

            arguments (Repeating)
                dlX
            end


            if isempty(detector.Backbone) && isempty(detector.Neck) && isempty(detector.Head)
                % Compute the activations and state information from the network.
                network = detector.Network;
                [varargout{1:nargout}] = forward(network, dlX{:});
            else
                % Compute the activations and state information from the sub-networks.
                backboneFeatures = cell(size(detector.Backbone.OutputNames'));
                neckFeatures = cell(size(detector.Neck.OutputNames'));

                if ~detector.FreezeBackbone && ~detector.FreezeBackboneAndNeck
                    [backboneFeatures{:}, backboneState] = forward(detector.Backbone, dlX{:},Acceleration="none");
                else
                    % Compute the activations from frozen Backbone
                    dlX = dlarray(extractdata(dlX{:}),dims(dlX{:}));
                    [backboneFeatures{:}] = predict(detector.Backbone, dlX ,Acceleration="none");
                    backboneState = [];
                end

                if detector.FreezeBackboneAndNeck
                    % Compute the activations from frozen Neck
                    [neckFeatures{:}] = predict(detector.Neck, backboneFeatures{:} ,Acceleration="none");
                    neckState = [];
                else
                    [neckFeatures{:}, neckState] = forward(detector.Neck, backboneFeatures{:},Acceleration="none");
                end

                headFeatures= cell(size(cellfun(@(h){h.OutputNames{:}},detector.Head,'UniformOutput',true)'));
                headState = cell(size(headFeatures));
                for i=1:detector.numHeads
                    [headFeatures{i},headState{i}] = forward(detector.Head{i}, neckFeatures{i},Acceleration="none");
                end

                state = vertcat(backboneState,neckState,headState{:});

                if nargout==4
                    [varargout{1:nargout-1}]=headFeatures{:};
                    varargout{nargout}=state;
                else
                    varargout=headFeatures;
                    varargout=[varargout(:)',{state}];
                end

            end
        end


        %------------------------------------------------------------------
        % Postprocess output feature maps.
        %------------------------------------------------------------------
        function varargout = postprocess(detector,YPredData, info, params)
            % This method applies post-processing on the predicted output
            % feature maps and computes the detected bounding boxes, scores
            % and labels.

            if isequal(size(YPredData), [1,3])
                varNames = {'Boxes', 'Scores', 'Labels'};
                result = table(YPredData{1,1}, YPredData{1,2}, YPredData{1,3}, 'VariableNames', varNames);
                [varargout{1:nargout}] = result;
            else
                if params.DetectionInputWasBatchOfImages
                    [varargout{1:nargout}] = iPostprocessMultiDetection(detector,YPredData,info,params);
                else
                    [varargout{1:nargout}] = iPostprocessSingleDetection(detector,YPredData,info,params);
                end
            end
        end

        %------------------------------------------------------------------
        % Parse and validate detection parameters.
        %------------------------------------------------------------------
        function params = parseDetectInputs(detector, I, varargin)

            params.DetectionInputIsDatastore = ~isnumeric(I);

            if params.DetectionInputIsDatastore
                sampleImage = vision.internal.cnn.validation.checkDetectionInputDatastore(I, mfilename);
            else
                sampleImage = I;
            end

            params.CastToGpuArray = ~isgpuarray(sampleImage);

            if size(detector.InputSize,2) == 2
                networkInputSize = [detector.InputSize 1];
            else
                networkInputSize = detector.InputSize;
            end

            validateChannelSize = true;  % check if the channel size is equal to that of the network input channel size
            validateImageSize   = false; % yolov4 can support images smaller than input size
            [sz,params.DetectionInputWasBatchOfImages] = vision.internal.cnn.validation.checkDetectionInputImage(...
                networkInputSize,sampleImage,validateChannelSize,validateImageSize);

            defaults = iGetDefaultYOLOv4DetectionParams();

            p = inputParser;
            p.addOptional('roi', defaults.roi);
            p.addParameter('SelectStrongest', defaults.SelectStrongest);
            p.addParameter('MinSize', defaults.MinSize);
            p.addParameter('MaxSize', sz(1:2));
            p.addParameter('MiniBatchSize', defaults.MiniBatchSize);
            p.addParameter('Threshold', defaults.Threshold);
            p.addParameter('ExecutionEnvironment', defaults.ExecutionEnvironment);
            p.addParameter('Acceleration', defaults.Acceleration);
            parse(p, varargin{:});

            userInput = p.Results;

            % Validate minibatchsize.
            vision.internal.cnn.validation.checkMiniBatchSize(userInput.MiniBatchSize, mfilename);

            % Validate selectstrongest.
            vision.internal.inputValidation.validateLogical(...
                userInput.SelectStrongest, 'SelectStrongest');

            % Validate execution environment.
            supportedInputs = ["auto", "cpu", "gpu"];
            exeEnv = validatestring(userInput.ExecutionEnvironment, supportedInputs, mfilename, 'ExecutionEnvironment');

            % Validate acceleration.
            supportedInputs = ["auto", "mex", "none"];
            accel = validatestring(userInput.Acceleration, supportedInputs, mfilename, 'Acceleration');

            % Validate minsize and maxsize.
            validateMinSize = ~ismember('MinSize', p.UsingDefaults);
            validateMaxSize = ~ismember('MaxSize', p.UsingDefaults);

            if validateMinSize
                vision.internal.detector.ValidationUtils.checkMinSize(userInput.MinSize, [1,1], mfilename);
            end

            if validateMaxSize
                vision.internal.detector.ValidationUtils.checkSize(userInput.MaxSize, 'MaxSize', mfilename);
            end

            if validateMaxSize && validateMinSize
                coder.internal.errorIf(any(userInput.MinSize >= userInput.MaxSize) , ...
                    'vision:ObjectDetector:minSizeGTMaxSize');
            end

            % Validate ROI.
            useROI = ~ismember('roi', p.UsingDefaults);
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
            yolov4ObjectDetector.checkThreshold(userInput.Threshold, mfilename);

            params.ROI                      = single(userInput.roi);
            params.UseROI                   = useROI;
            params.SelectStrongest          = logical(userInput.SelectStrongest);
            params.MinSize                  = single(userInput.MinSize);
            params.MaxSize                  = single(userInput.MaxSize);
            params.MiniBatchSize            = double(userInput.MiniBatchSize);
            params.Threshold                = single(userInput.Threshold);
            params.NetworkInputSize         = double(networkInputSize);
            params.FilterBboxesFunctor      = detector.FilterBboxesFunctor;
            params.ExecutionEnvironment     = string(exeEnv);
            params.Acceleration             = string(accel);
        end

        %------------------------------------------------------------------
        % Parse preprocess input parameters.
        %------------------------------------------------------------------
        function params = parsePreprocessInputs(~, I, varargin)
            params.UseROI = varargin{1,1}{1,1}.UseROI;
            params.ROI = varargin{1,1}{1,1}.ROI;
            params.ExecutionEnvironment = varargin{1,1}{1,1}.ExecutionEnvironment;
            params.DetectionInputIsDatastore = ~isnumeric(I) && ~iscell(I);
            params.CastToGpuArray = varargin{1,1}{1,1}.CastToGpuArray;
        end

        %------------------------------------------------------------------
        % Parse predict input parameters.
        %------------------------------------------------------------------
        function params = parsePredictInputs(~,varargin)
            params = varargin{1,1}{1,1};
        end
    end

    methods(Static, Hidden, Access = protected)
        %------------------------------------------------------------------
        % Parse and validate pretrained detector parameters.
        %------------------------------------------------------------------
        function params = parsePretrainedDetectorInputs(varargin)
            % Parse inputs for this syntax:
            % detector = yolov4ObjectDetector(detectorName).

            p = inputParser;
            if size(varargin,2) == 1
                p.addRequired('detectorName');

                supportedNetworks = ["csp-darknet53-coco", "tiny-yolov4-coco"];
                detectorName = validatestring(varargin{1,1}, supportedNetworks, mfilename, 'detectorName', 1);

                % Default input size.
                if strcmp(detectorName,'csp-darknet53-coco')
                    inpSz = [608,608,3];
                else
                    inpSz = [416,416,3];
                end

                p.addParameter('InputSize', inpSz);
                p.addParameter('DetectionNetworkSource',{});
                p.addParameter('ModelName', '', @iAssertValidLayerName);
                p.addParameter('PredictedBoxType', "axis-aligned", @iValidatePredictedBoxType)

                parse(p, varargin{:});

            elseif (size(varargin,2) == 3) && ...
                    ((isa(varargin{1,2},'string') && isscalar(varargin{1,2})) || (isa(varargin{1,2},'char') && isrow(varargin{1,2}))) ...
                    && strcmp(varargin{1,2},'ModelName')
                % Parse inputs for this syntax:
                % detector = yolov4ObjectDetector(detectorName,'ModelName',modelName).

                p.addRequired('detectorName');

                supportedNetworks = ["csp-darknet53-coco", "tiny-yolov4-coco"];
                detectorName = validatestring(varargin{1,1}, supportedNetworks, mfilename, 'detectorName', 1);

                % Default input size.
                if strcmp(detectorName,'csp-darknet53-coco')
                    inpSz = [608,608,3];
                else
                    inpSz = [416,416,3];
                end

                p.addParameter('InputSize', inpSz);
                p.addParameter('DetectionNetworkSource',{});
                p.addParameter('ModelName', '', @iAssertValidLayerName);
                p.addParameter('PredictedBoxType', "axis-aligned", @iValidatePredictedBoxType)

                parse(p, varargin{:});

                params.isCustomModelName = true;

            else
                % Parse inputs for this syntax:
                % detector = yolov4ObjectDetector(detectorName,classNames,anchorBoxes).

                if (size(varargin,2) >= 3) && ((isa(varargin{1,2},'string') && isscalar(varargin{1,2})) || (isa(varargin{1,2},'char') && isrow(varargin{1,2})))
                    if (strcmp(varargin{1,2},'InputSize') || strcmp(varargin{1,2},'DetectionNetworkSource') || strcmp(varargin{1,2},'ModelName'))
                        error(message('vision:yolo:requireClassesAndAnchors'));
                    end
                end

                p.addRequired('detectorName');
                p.addRequired('classNames');
                p.addRequired('anchorBoxes');

                supportedNetworks = ["csp-darknet53-coco", "tiny-yolov4-coco"];
                detectorName = validatestring(varargin{1,1}, supportedNetworks, mfilename, 'detectorName', 1);

                % Default input size.
                if strcmp(detectorName,'csp-darknet53-coco')
                    inpSz = [608,608,3];
                else
                    inpSz = [416,416,3];
                end

                p.addParameter('InputSize', inpSz);
                p.addParameter('DetectionNetworkSource',{});
                p.addParameter('ModelName', '', @iAssertValidLayerName);
                p.addParameter('PredictedBoxType', "axis-aligned", @iValidatePredictedBoxType)

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
                params.AnchorBoxes = iValidateAnchorBoxes(params.AnchorBoxes);
            end
            params.InputSize = p.Results.InputSize;
            params.DetectionNetworkSource = p.Results.DetectionNetworkSource;
            params.ModelName = char(p.Results.ModelName);
            params.DetectorName = char(p.Results.detectorName);
            params.PredictedBoxType = char(p.Results.PredictedBoxType);

            iCheckInputSize(params.InputSize);

            if ~((mod(params.InputSize(1),32)==0) && (mod(params.InputSize(2),32)==0))
                error(message('vision:yolo:mustBeMultipleOfThirtyTwo',params.DetectorName));
            end

            if strcmp(params.ModelName,'') && isempty(params.DetectionNetworkSource)
                params.ModelName = params.DetectorName;
            end
        end

        %------------------------------------------------------------------
        % Parse and validate detector parameters.
        %------------------------------------------------------------------
        function params = parseDetectorInputs(varargin)
            % Parse inputs for this syntax:
            % detector = yolov4ObjectDetector(network,classNames,anchorBoxes).

            % Validate the input network.
            imageInputIdx = iValidateNetwork(varargin{1,1},mfilename);

            p = inputParser;
            p.addRequired('network');
            p.addRequired('classNames');
            p.addRequired('anchorBoxes');

            p.addParameter('InputSize', []);
            p.addParameter('DetectionNetworkSource',{});
            p.addParameter('ModelName', '', @iAssertValidLayerName);
            p.addParameter('PredictedBoxType', "axis-aligned", @iValidatePredictedBoxType)
            parse(p, varargin{:});

            params.ClassNames = p.Results.classNames;
            params.AnchorBoxes = p.Results.anchorBoxes;
            params.InputSize = p.Results.InputSize;
            params.DetectionNetworkSource = p.Results.DetectionNetworkSource;
            params.ModelName = char(p.Results.ModelName);
            params.PredictedBoxType = char(p.Results.PredictedBoxType);

            net = p.Results.network;

            if isempty(params.InputSize)
                params.InputSize = net.Layers(imageInputIdx).InputSize;
            end

            iValidateClassNames(params.ClassNames);

            iValidateAnchorBoxes(params.AnchorBoxes);

            iCheckInputSize(params.InputSize);

            if ~iscolumn(params.ClassNames)
                params.ClassNames = params.ClassNames';
            end
            if isstring(params.ClassNames) || iscategorical(params.ClassNames)
                params.ClassNames = cellstr(params.ClassNames);
            end
            numClasses = size(params.ClassNames, 1);

            networkInputSize = net.Layers(imageInputIdx).InputSize;
            if numel(networkInputSize)==2
                numNetworkInputChannel = 1;
            else
                numNetworkInputChannel = networkInputSize(3);
            end

            if numel(params.InputSize)==2
                userInputChannel = 1;
            else
                userInputChannel = params.InputSize(3);
            end

            % Configure network for transfer Learning.
            if ~isempty(params.DetectionNetworkSource)
                lgraph = iUpdateFirstConvChannelsAndInputLayer(net,params.InputSize);
                % validate DetectionNetworkSources
                validateattributes(params.DetectionNetworkSource, {'cell','string'}, {'row', 'size', [1 NaN]}, ...
                    mfilename, 'DetectionNetworkSource');
                % Sorted DetectionNetworkSource returned
                [lgraph, params.DetectionNetworkSource] = iConfigureDetector(lgraph,numClasses,params.AnchorBoxes,params.DetectionNetworkSource,params.PredictedBoxType);

                % Verify Fully Connected layer and Global average pooling layer does not exist in the network.
                net = dlnetwork(lgraph);
                iVerifyFullyConnectedExistence(net);
                iVerifyGlobalAvgPoolExistence(net);
                params.Network = net;
            else
                if(isequal(params.InputSize,networkInputSize))
                    % No network updates needed
                    params.Network = net;
                else
                    if numNetworkInputChannel == userInputChannel
                        lgraph = iUpdateFirstConvChannelsAndInputLayer(net,params.InputSize);
                        params.Network = dlnetwork(lgraph);
                    else
                        error(message('vision:yolo:numChannelDifferent'));
                    end
                end
                iValidateYOLOv4Network(params.Network, numClasses, params.AnchorBoxes, params.PredictedBoxType);
            end
        end
    end

    % ------------------------------------------------------------
    % Set Input Layer Noralization
    %-------------------------------------------------------------
    methods(Hidden)
        function self = setInputNormalization(self,stats)
            imageInputIdx = find(arrayfun( @(x)isa(x,'nnet.cnn.layer.ImageInputLayer'),self.Network.Layers));
            currentInputLayer = self.Network.Layers(imageInputIdx);
            map = normalizationStatsDictionary(stats);
            statsSet = map{currentInputLayer.Normalization};
            newInputLayer = imageInputLayer(self.InputSize,"Name",currentInputLayer.Name,...
                "Normalization",currentInputLayer.Normalization,...
                statsSet{:});
            net  = replaceLayer(self.Network ,self.Network.Layers(1).Name,newInputLayer);
            self.Network = initialize(net);
        end
    end

    %======================================================================
    % Save/Load
    %======================================================================
    methods(Hidden)
        function s = saveobj(this)

            % Update Network if required
            if ~isempty(this.Backbone) && ~isempty(this.Neck) && ~isempty(this.Head)
                this = updateDetector(this);
            end
            % Save detector
            s.Version                      = 2;
            s.ModelName                    = this.ModelName;
            s.Network                      = this.Network;
            s.DetectionNetworkSource       = this.DetectionNetworkSource;
            s.ClassNames                   = this.ClassNames;
            s.AnchorBoxes                  = this.AnchorBoxes;
            s.InputSize                    = this.InputSize;
            s.PredictedBoxType             = this.PredictedBoxType;
            s.FreezeBackboneAndNeckInternal = this.FreezeBackboneAndNeckInternal;
        end

        function net = matlabCodegenPrivateNetwork(this)
            net = this.Network;
        end
    end

    methods(Static, Hidden)
        function this = loadobj(s)

            try
                vision.internal.requiresNeuralToolbox(mfilename);

                network                      = s.Network;
                classes                      = s.ClassNames;
                anchorBoxes                  = s.AnchorBoxes;
                inputSize                    = s.InputSize;

                % Add PredictedBoxType property to any yolov4Objectdetector
                % object that does not already have it. This will ensure
                % backward compatability for releases before R23b.
                if ~isfield(s, 'PredictedBoxType')
                    predictedBoxType = "axis-aligned";
                else
                    predictedBoxType = s.PredictedBoxType;
                end

                this = yolov4ObjectDetector(network,classes,anchorBoxes,'InputSize',inputSize,'PredictedBoxType',predictedBoxType);
                this.ModelName = s.ModelName;

                if ~(s.Version == 1.0 || s.Version == 1.1)
                    this.FreezeBackboneAndNeckInternal = s.FreezeBackboneAndNeckInternal;
                    this.DetectionNetworkSource = s.DetectionNetworkSource;
                end

            catch ME
                rethrow(ME)
            end
        end

        function detector = assembleDetector(params, net)

            detector = yolov4ObjectDetector(net, params.ClassNames, params.AnchorBoxes,...
                'InputSize', params.InputSize,...
                'ModelName', params.ModelName,...
                'PredictedBoxType', params.PredictedBoxType);

            % setting DetectionNetworkSource and FreezeBackboneAndNeckInternal property
            % for assembled detector to support freezing with assembled detector
            detector.DetectionNetworkSource = params.DetectionNetworkSource;
            detector.FreezeBackboneAndNeckInternal = params.FreezeBackboneAndNeckInternal;
        end

        function n = matlabCodegenDlRedirect(~)
            n = 'vision.internal.codegen.YOLOv4ObjectDetector';
        end
    end


    %----------------------------------------------------------------------
    methods(Static, Hidden)
        function data = preprocessInput(data, targetSize)
            if istable(data)
                data = table2cell(data);
            end

            if iscell(data)
                for idx = 1:size(data,1)
                    I = data{idx,1};
                    [I, scale] = iPreprocessSingleImg(I,targetSize);

                    bboxes = bboxresize(data{idx,2}, scale);
                    data(idx,1:2) = {I, bboxes};
                end
            else
                batchSize = size(data,4);
                if batchSize>1
                    dataTmp = [];
                    for i = 1:batchSize
                        Itmp = iPreprocessSingleImg(data(:,:,:,i),targetSize);
                        if isempty(dataTmp)
                            dataTmp = Itmp;
                        else
                            dataTmp = cat(4,dataTmp,Itmp);
                        end
                    end
                    data = dataTmp;
                else
                    data = iPreprocessSingleImg(data,targetSize(1:2));
                end
            end
        end

        %------------------------------------------------------------------
        function predictions = yolov4Transform(YPredictions, isRotatedBox, anchorBoxes)
            predictions = cell(size(YPredictions, 1),6);
            rotationIdxOffset = 2*double(isRotatedBox);
            for idx = 1:size(YPredictions, 1)
                % Get the required info on feature size.
                numChannelsPred = size(YPredictions{idx},3);
                numAnchors = size(anchorBoxes{idx},1);
                numPredElemsPerAnchors = numChannelsPred/numAnchors;
                channelsPredIdx = 1:numChannelsPred;

                stride = numPredElemsPerAnchors;
                endIdx = numChannelsPred;

                % X positions.
                startIdx = 1;
                xIds = startIdx:stride:endIdx;
                predictions{idx,2} = sigmoid(YPredictions{idx}(:,:,xIds,:));

                % Y positions.
                startIdx = 2;
                yIds = startIdx:stride:endIdx;
                predictions{idx,3} = sigmoid(YPredictions{idx}(:,:,yIds,:));

                % Width.
                startIdx = 3;
                wIds = startIdx:stride:endIdx;
                predictions{idx,7+rotationIdxOffset} = YPredictions{idx}(:,:,wIds,:);
                predictions{idx,4} = exp(YPredictions{idx}(:,:,wIds,:));

                % Height.
                startIdx = 4;
                hIds = startIdx:stride:endIdx;
                predictions{idx,8+rotationIdxOffset} = YPredictions{idx}(:,:,hIds,:);
                predictions{idx,5} = exp(YPredictions{idx}(:,:,hIds,:));

                % Confidence scores.
                startIdx = 5 + rotationIdxOffset; % startIdx = 7 when using rotated rectangles
                cIds = startIdx:stride:endIdx;
                predictions{idx,1} = sigmoid(YPredictions{idx}(:,:,cIds,:));

                if ~isRotatedBox
                    % Accumulate all non-class indexes
                    nonClassIds = [xIds yIds wIds hIds cIds];
                else
                    % Sin angle.
                    startIdx = 5;
                    sinIds = startIdx:stride:endIdx;
                    predictions{idx,6} = tanh(YPredictions{idx}(:,:,sinIds,:));

                    % Cos angle.
                    startIdx = 6;
                    cosIds = startIdx:stride:endIdx;
                    predictions{idx,7} = tanh(YPredictions{idx}(:,:,cosIds,:));

                    % Accumulate all non-class indexes
                    nonClassIds = [xIds yIds wIds hIds sinIds cosIds cIds];
                end

                % Class probabilities.
                classIdx = setdiff(channelsPredIdx, nonClassIds, 'stable');
                predictions{idx,6+rotationIdxOffset} = sigmoid(YPredictions{idx}(:,:,classIdx,:));
            end
        end

        %------------------------------------------------------------------
        function tiledAnchors = anchorBoxGenerator(anchorBoxes,YPredCell,inputImageSize)
            % Generate tiled anchor offset.
            tiledAnchors = cell(size(YPredCell));
            for i=1:size(YPredCell,1)
                anchors = anchorBoxes{i, :};
                [h,w,~,n] = size(YPredCell{i,1});
                [tiledAnchors{i,2}, tiledAnchors{i,1}] = ndgrid(0:h-1,0:w-1,1:size(anchors,1),1:n);
                [~,~,tiledAnchors{i,3}] = ndgrid(0:h-1,0:w-1,anchors(:,2),1:n);
                [~,~,tiledAnchors{i,4}] = ndgrid(0:h-1,0:w-1,anchors(:,1),1:n);
            end

            % Convert grid cell coordinates to box coordinates.
            for i=1:size(YPredCell,1)
                [h,w,~,~] = size(YPredCell{i,1});
                tiledAnchors{i,1} = (tiledAnchors{i,1}+YPredCell{i,1})./w;
                tiledAnchors{i,2} = (tiledAnchors{i,2}+YPredCell{i,2})./h;
                tiledAnchors{i,3} = (tiledAnchors{i,3}.*YPredCell{i,3})./inputImageSize(2);
                tiledAnchors{i,4} = (tiledAnchors{i,4}.*YPredCell{i,4})./inputImageSize(1);
            end
        end
    end
end
% -------------------------------------------------------------------------
function map = normalizationStatsDictionary(stats)
% This maps knowledge of how different styles of normalization in the input
% layer (Keys) map to different Name/Value inputs to the statistics field
% of the input layer.
map = dictionary(["zerocenter","zscore","rescale-symmetric","rescale-zero-one","none"],...
    {{'Mean',stats.Mean}, {'StandardDeviation',stats.Std,"Mean",stats.Mean},...
    { 'Min', stats.Min, 'Max', stats.Max },...
    { 'Min', stats.Min, 'Max', stats.Max },...
    {} });
end
%--------------------------------------------------------------------------
function iVerifyLayersExist(lgraph, layerNames)
numLayers = numel(lgraph.Layers);
for idx = 1:numel(layerNames)
    foundLayer = false;
    for lIdx = 1:numLayers
        if strcmp(layerNames{idx}, lgraph.Layers(lIdx).Name)
            foundLayer = true;
            break;
        end
    end
    if ~foundLayer
        error(message('vision:yolo:invalidLayerName', layerNames{idx}));
    end
end
end

%--------------------------------------------------------------------------
function iCheckInputSize(inputSize)
validateattributes(inputSize, {'numeric'}, ...
    {'2d','nonempty','nonsparse',...
    'real','finite','integer','positive','nrows',1,});
if ~(size(inputSize,2)==2 || size(inputSize,2)==3)
    error(message('vision:yolo:invalidInputSize'));
end
end

%--------------------------------------------------------------------------
function iValidateClassNames(value)
if ~isvector(value) || ~iIsValidDataType(value)
    error(message('vision:yolo:invalidClasses'));
end
if iHasDuplicates(value)
    error(message('vision:yolo:duplicateClasses'));
end
end

%--------------------------------------------------------------------------
function iAssertValidLayerName(name)
nnet.internal.cnn.layer.paramvalidation.validateLayerName(name);
end

%--------------------------------------------------------------------------
function iValidatePredictedBoxType(predictedBoxType)
validatestring(predictedBoxType,{'axis-aligned', 'rotated'},...
    'yolov4ObjectDetector', 'PredictedBoxType');
end

%--------------------------------------------------------------------------
function value = iValidateAnchorBoxes(value)
%Checking if the value is vector or not.
validateattributes(value, {'cell'},{'vector','size',[NaN NaN]}, ...
    mfilename, 'AnchorBoxes');
%Converting any type vector to column vector.
value = value(:);
for i = 1:size(value,1)
    validateattributes(value{i,1}, {'numeric'}, {'size', [NaN 2], 'real',...
        'nonnan', 'finite','positive'}, mfilename, 'AnchorBoxes');
end
return;
end

%--------------------------------------------------------------------------
function outputFeatures = iPredictBatchActivations(network,dlX,anchorBoxes,acceleration,isRotatedBox)
numMiniBatch = size(dlX,2);
outputFeatures = cell(numMiniBatch,1);
for ii = 1:numMiniBatch
    inp = dlX{ii};
    outputFeatures{ii,1} = iPredictActivations(network,inp,anchorBoxes,acceleration,isRotatedBox);
end
end

%--------------------------------------------------------------------------
function outputFeatures = iPredictActivations(network,dlX,anchorBoxes,acceleration,isRotatedBox)
% Compute predictions.
features = cell(size(network.OutputNames'));
[features{:}] = predict(network, dlX, 'Acceleration',acceleration);

outputFeatures = yolov4ObjectDetector.yolov4Transform(features, isRotatedBox, anchorBoxes);
end



%--------------------------------------------------------------------------
function [bboxes,scores,labels,intermediates] = iPostprocessMultiDetection(detector,YPredData,info,params)
numMiniBatch = size(YPredData{1,1}, 4);
numFeatures = size(YPredData, 1);

isRotatedBox = strcmp(detector.PredictedBoxType, "rotated");
rotationOffset = 2*double(isRotatedBox);

bboxes = cell(numMiniBatch, 1);
scores = cell(numMiniBatch, 1);
labels = cell(numMiniBatch, 1);
intermediates = cell(numMiniBatch, 1);

for ii = 1:numMiniBatch
    fmap = cell(numFeatures,8 + rotationOffset);
    for i = 1:8 + rotationOffset
        for j = 1:numFeatures
            feature = YPredData{j,i};
            fmap{j,i} = feature(:,:,:,ii);
        end
    end
    [bboxes{ii},scores{ii},labels{ii},intermediates{ii}] = ...
        iPostprocessSingleDetection (detector,fmap,info,params);
end
end

%--------------------------------------------------------------------------
function [bboxes,scores,labels,intermediates] = iPostprocessSingleDetection (detector,YPredData,info,params)
% Obtain the classnames detector is trained on.
classes = detector.ClassNames;

isRotatedBox = strcmp(detector.PredictedBoxType, "rotated");
rotationOffset = 2*double(isRotatedBox);

extractDetections = cellfun(@ extractdata, YPredData, 'UniformOutput', false);

anchorBoxes = detector.AnchorBoxes;
extractDetections(:,2:5) = yolov4ObjectDetector.anchorBoxGenerator(anchorBoxes, extractDetections(:,2:5), params.NetworkInputSize);

% Apply following post processing steps to filter the detections:
% * Filter detections based on threshold.
% * Convert bboxes from spatial to pixel dimension.

%Combine the prediction from different heads.
indexes = size(YPredData);
for i = 1:indexes(1)
    for j = 1:5+rotationOffset
        detections{i,j} = reshapePredictions(extractDetections{i,j});
    end
    detections{i,6+rotationOffset} = reshapeClasses(extractDetections{i,6+rotationOffset}, numel(classes));
end
detections = iCombinePredictions(detections);

% Get the raw class probabilities and objectiveness scores
classProbabilities = detections(:,6+rotationOffset:end);

% Filter the classes based on (confidence score * class probability).
[classProbs, classIdx] = max(detections(:,6+rotationOffset:end),[],2);
detections(:,1) = detections(:,1).*classProbs;
detections(:,6+rotationOffset) = classIdx;

% Empty all lingering detection information and append with class
% probabilities
detections(:,7+rotationOffset:end) = [];
detections = [detections,classProbabilities];

% Keep detections whose objectness score is greater than thresh.
detections = detections(detections(:,1)>=params.Threshold,:);

[bboxes,scores,labels,intermediates] = iPostProcessDetections(detections,classes,info,params,isRotatedBox);
end
%------------------------------------------------------------------------------------------

function combinedPredictions = iCombinePredictions(detections)
% If cell array is 2-D, execute 2-D code for speed efficiency
if ismatrix(detections)
    rows = size(detections,1);
    cols = size(detections,2);
    if (rows < cols)
        combinedPredictions = cell(rows,1);
        % Concatenate one dim first
        for n=1:rows
            combinedPredictions{n} = cat(2,detections{n,:});
        end
        % Now concatenate the single column of cells into a matrix
        combinedPredictions = cat(1,combinedPredictions{:});
    else
        combinedPredictions = cell(1, cols);
        % Concatenate one dim first
        for n=1:cols
            combinedPredictions{n} = cat(1,detections{:,n});
        end
        % Now concatenate the single column of cells into a matrix
        combinedPredictions = cat(2,combinedPredictions{:});
    end
    return
end
end

%--------------------------------------------------------------------------

function [bboxes,scores,labels,intermediates] = iPostProcessDetections(detections,classes,info,params,isRotatedBox)

rotationOffset = 2*double(isRotatedBox);

if ~isempty(detections)

    scorePred = detections(:,1);
    bboxesTmp = detections(:,2:5+rotationOffset);
    classAndProbPred = detections(:,6+rotationOffset:end);

    inputImageSize(2) = info.ScaleX.*info.PreprocessedImageSize(2);
    inputImageSize(1) = info.ScaleY.*info.PreprocessedImageSize(1);

    % Obtain boxes for preprocessed image.
    scale = [inputImageSize(2) inputImageSize(1) inputImageSize(2) inputImageSize(1)];
    bboxTmp(:,1:4) = bboxesTmp(:,1:4).*scale;

    if ~isRotatedBox
        % Convert x and y position of detections from centre to top-left.
        % Resize boxes to image size.
        bboxPred = iConvertCenterToTopLeft(bboxTmp);
    else
        phases = tanh(bboxesTmp(:,5:6));
        yaw = rad2deg(atan2(phases(:,1),phases(:,2)));
        bboxPred = [bboxTmp(:,1:4) yaw];
    end

    % Filter boxes based on MinSize, MaxSize.
    [bboxPred, scorePred, classAndProbPred] = filterBBoxes(params.FilterBboxesFunctor,...
        params.MinSize,params.MaxSize,bboxPred,scorePred,classAndProbPred);

    if (size(bboxPred,2)==5 && params.CastToGpuArray)
        % Convert gpuArray data to single format for rotated boxes.
        bboxPred = gather(bboxPred);
        scorePred = gather(scorePred);
        classAndProbPred = gather(classAndProbPred);
    end

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


    if ~isRotatedBox
        % Limit width detections
        detectionsWd = min((bboxes(:,1) + bboxes(:,3)),inputImageSize(1,2));
        bboxes(:,3) = detectionsWd(:,1) - bboxes(:,1);

        % Limit Height detections
        detectionsHt = min((bboxes(:,2) + bboxes(:,4)),inputImageSize(1,1));
        bboxes(:,4) = detectionsHt(:,1) - bboxes(:,2);

        % Apply ROI offset
        bboxes(:,1:2) = vision.internal.detector.addOffsetForROI(bboxes(:,1:2), params.ROI, params.UseROI);

        % Adjust any x, y, w, h to 1 if less than 1.
        bboxes(bboxes(:,1:4)<1) = 1;
    else
        imWidth = inputImageSize(1,2);
        imHeight = inputImageSize(1,1);

        % Clamp center coordinates to image boundaries.
        bboxes(:,1) = max( min( bboxes(:,1), imWidth  ), 1 );
        bboxes(:,2) = max( min( bboxes(:,2), imHeight ), 1 );

        % Adjust any width and height to 1 if less than 1.
        bboxes(bboxes(:,3:4)<1) = 1;
    end

    % Convert classId to classNames.
    % Create categories of labels such that the order of the classes is retained.
    labels = categorical(classes,cellstr(classes));
    labels = labels(classNames);

    if params.CastToGpuArray
        scores = gather(scores);
        bboxes = gather(bboxes);
        labels = gather(labels);
        classProbability = gather(classProbability);
        objectnessScore = gather(objectnessScore);
    end

    % Return the class probabilities and objectness score as a struct
    intermediates.ClassProbabilities = classProbability;
    intermediates.ObjectnessScores = objectnessScore;
else
    if ~isRotatedBox
        bboxes = zeros(0,4,'single');
    else
        bboxes = zeros(0,5,'single');
    end
    scores = zeros(0,1,'single');
    labels = categorical(cell(0,1),cellstr(classes));
    intermediates = struct.empty;
end
end

%--------------------------------------------------------------------------
function iValidateYOLOv4Network(network, numClasses, anchorBoxes, predictedBoxType)

numOutputLayers = size(network.OutputNames,2);
layerNames = string({network.Layers.Name});

% Verify Fully Connected layer does not exist in the network.
iVerifyFullyConnectedExistence(network);

% Verify Global average pooling layer does not exist in the network.
iVerifyGlobalAvgPoolExistence(network);

% Set rotationOffset for rotated rectangle bounding boxes
if strcmp(predictedBoxType,"rotated")
    rotationOffset = 2;
else
    rotationOffset = 0;
end

for i = 1:numOutputLayers
    numAnchorsScale = size(anchorBoxes{i,1}, 1);
    numPredictorsPerAnchor = 5 + rotationOffset + numClasses;

    expectedFilters = numAnchorsScale*numPredictorsPerAnchor;

    % Compute the number of filters for last convolution layer.
    layerIdx = find(strcmp(network.OutputNames{1,i}, layerNames));

    idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.Convolution2DLayer'),network.Layers(1:layerIdx));
    filterIdx = find(idx,1,'last');

    actualFilters = network.Layers(filterIdx,1).NumFilters;

    if ~(expectedFilters == actualFilters)
        error(message('vision:yolo:invalidNumFilters',mat2str(expectedFilters),mat2str(numAnchorsScale),mat2str(numClasses)));
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
% Convert x and y position of detections from centre to top-left.
function bboxes = iConvertCenterToTopLeft(bboxes)
bboxes(:,1) = bboxes(:,1)- bboxes(:,3)/2 + 0.5;
bboxes(:,2) = bboxes(:,2)- bboxes(:,4)/2 + 0.5;
bboxes(bboxes<1) = 1;
end

%--------------------------------------------------------------------------
function x = reshapePredictions(pred)
[h,w,c,n] = size(pred);
x = reshape(pred,h*w*c,1,n);
end

%--------------------------------------------------------------------------
function x = reshapeClasses(pred,numclasses)
[h,w,c,n] = size(pred);
numanchors = c/numclasses;
x = reshape(pred,h*w,numclasses,numanchors,n);
x = permute(x,[1,3,2,4]);
[h,w,c,n] = size(x);
x = reshape(x,h*w,c,n);
end

%--------------------------------------------------------------------------
function lgraph = iUpdateFirstConvChannelsAndInputLayer(dlnet,imageSize)
% This function update the channels of first conv layer if InputSize channel
% does not match with channels of first conv layer. It also update the
% imageInputLayer or initialize the dlnetwork if image input layer not present.

if size(imageSize,2)==2
    imageSize = [imageSize 1];
end

imgIdx = arrayfun(@(x)isa(x,'nnet.cnn.layer.ImageInputLayer'),...
    dlnet.Layers);
imageInputIdx = find(imgIdx,1,'first');

numChannel = imageSize(3);

idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.Convolution2DLayer'),...
    dlnet.Layers);
convIdx = find(idx,1,'first');
if ~isempty(convIdx)
    numFirstConvLayerChannels = dlnet.Layers(convIdx,1).NumChannels;
else
    error(message('vision:yolo:mustHaveConvLayers'));
end
lgraph = layerGraph(dlnet);

% If number of channels in imageSize is not equal to the channel count
% of first convolutional layer, update the channel count of first conv
% layer and use values of properties as it is. Pyramid pooling concept
% has been used for concatenating extra channel. Each extra channel is
% mean of original (initial) channels of conv layer.
%
% Zhao, Hengshuang, et al. "Pyramid Scene Parsing Network." 2017 IEEE
% Conference on Computer Vision and Pattern Recognition (CVPR). IEEE, 2017.
if (~strcmp(numFirstConvLayerChannels,'auto'))
    if numFirstConvLayerChannels~=numChannel
        firstConvLayer = lgraph.Layers(convIdx,1);
        firstConvLayerWeights = firstConvLayer.Weights;
        meanChannelWeights = reshape(mean(firstConvLayerWeights,3),size(firstConvLayerWeights(:,:,1,:)));
        if numChannel>numFirstConvLayerChannels
            extraChanels = abs(numChannel-numFirstConvLayerChannels);
            extraChannelWeights = repmat(meanChannelWeights,1,1,extraChanels);
            updatedConvLayerWeights = cat(3,firstConvLayerWeights,extraChannelWeights);
        else
            updatedConvLayerWeights = repmat(meanChannelWeights,1,1,numChannel);
        end
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

% If imageSize is not equal to the InputSize, replace the imageInputLayer.
if ~isempty(imageInputIdx)
    inputLayer = lgraph.Layers(imageInputIdx,1);
    if ~isequal(inputLayer.InputSize,imageSize)
        imageInput = imageInputLayer(imageSize, ...
            'Normalization','none', ...
            'Name',lgraph.Layers(imageInputIdx).Name);
        lgraph = replaceLayer(lgraph,lgraph.Layers(imageInputIdx).Name,...
            imageInput);
    end
end
end

%--------------------------------------------------------------------------
function [lgraph, orderedDetectionNetworkSource] = iConfigureDetector(lgraph,numClasses,anchorBoxes,detectionNetworkSource,predictedBoxType)
% Verify that all detectionNetworkSource layers exist in lgraph.
iVerifyLayersExist(lgraph, detectionNetworkSource);

% Arrange the featuremap layers in ascending order of the feature map resolution.
[networkActivationSizes, ~] = deep.internal.sdk.forwardDataAttributes(lgraph,'Outputs',detectionNetworkSource);

numOutputLayers = size(detectionNetworkSource,2);
featureMapResolutions = zeros(numOutputLayers,2);

for idx = 1:numOutputLayers
    activationSize = networkActivationSizes{idx,1};
    featureMapResolutions(idx,:) = activationSize(1:2);
end

[orderedFeatureMaps ,order] = sort(featureMapResolutions,'ascend');

orderedDetectionNetworkSource = detectionNetworkSource;

for idx = 1:numOutputLayers
    orderedDetectionNetworkSource{1,idx} = detectionNetworkSource{1,order(idx,1)};
    if idx~=1
        upsampleFactor = orderedFeatureMaps(idx,:) ./ orderedFeatureMaps(idx-1,:);
        % In 24b, custom YOLo v4 creation supports network creation
        % when sorted detection network source feature maps have a upsample
        % factor of 2 between every pair.  
        if ~all(upsampleFactor(:) == 2)
            error(message('vision:yolo:invalidDetectionNetworkSourceForCustomYOLOv4', orderedDetectionNetworkSource{1,idx},orderedDetectionNetworkSource{1,idx-1}));
        end
    end
end

% Get the layer with lowest outputsize in orderedDetectionNetworkSource.
layerWithLowestOutputSize = orderedDetectionNetworkSource{1,1};

% Create a layerGraph for transfer learning.
lgraph = iRemoveLayersAfterSpecifiedLayerAndUnconnectedLayers(lgraph,layerWithLowestOutputSize);

numAnchorBoxGroups = size(anchorBoxes,1);
if numOutputLayers ~= numAnchorBoxGroups
    error(message('vision:yolo:numOutputLayerMismatch'));
end

[orderedNetworkActivationSizes, ~] = deep.internal.sdk.forwardDataAttributes(lgraph,'Outputs',orderedDetectionNetworkSource);

% Set rotationOffset for rotated rectangle bounding boxes
if strcmp(predictedBoxType,"rotated")
    rotationOffset = 2;
else
    rotationOffset = 0;
end

% Add detection heads to feature extraction layers.
numPredictorsPerAnchor = 5 + rotationOffset + numClasses;
for idx = 1:numOutputLayers
    % Verify that YOLO v4 DetectionNetworkSource output size is greater than [1,1].
    activationSize = orderedNetworkActivationSizes{idx,1};
    if (any(activationSize(1:2) < 2))
        error(message("vision:yolo:mustHaveValidFinalActivationsSize"));
    end

    outFilters = activationSize(3);

    numAnchorsScale = size(anchorBoxes{idx,1}, 1);

    % Compute the number of filters for last convolution layer.
    numFilters = numAnchorsScale*numPredictorsPerAnchor;

    % Define detection heads.
    firstConv = convolution2dLayer(3,outFilters,'Padding','same','Name',['customConv',num2str(idx)],'WeightsInitializer','he');
    detectionSubNetwork = [firstConv;
        batchNormalizationLayer('Name',['customBatchNorm',num2str(idx)]);
        leakyReluLayer(0.1,'Name',['customRelu',num2str(idx)])
        convolution2dLayer(1,numFilters,'Padding','same','Name',['customOutputConv',num2str(idx)],'WeightsInitializer','he')
        ];

    % Add initial convolutional block just after the feature outputs from
    % backbone network.
    convBlockInitial = [
        convolution2dLayer(1,ceil(outFilters/2),'Padding','same','Name',['featureConvInitial',num2str(idx)],'WeightsInitializer','he');
        batchNormalizationLayer('Name',['featureBatchNormInitial',num2str(idx)]);
        leakyReluLayer(0.1,'Name',['featureReluInitial',num2str(idx)]);
        ];
    lgraph = addLayers(lgraph,convBlockInitial);
    lgraph = connectLayers(lgraph,orderedDetectionNetworkSource{1,idx},['featureConvInitial',num2str(idx)]);

    % Add Spatial Pyramid Pooling (SPP) module to the first feature output.
    if (idx==1)
        mpool1 = maxPooling2dLayer(5,'Stride',1,'Padding','same','Name','sppMaxPool1');
        mpool2 = maxPooling2dLayer(9,'Stride',1,'Padding','same','Name','sppMaxPool2');
        mpool3 = maxPooling2dLayer(13,'Stride',1,'Padding','same','Name','sppMaxPool3');

        lgraph = addLayers(lgraph,mpool1);
        lgraph = addLayers(lgraph,mpool2);
        lgraph = addLayers(lgraph,mpool3);

        lgraph = connectLayers(lgraph,['featureReluInitial',num2str(idx)],'sppMaxPool1');
        lgraph = connectLayers(lgraph,['featureReluInitial',num2str(idx)],'sppMaxPool2');
        lgraph = connectLayers(lgraph,['featureReluInitial',num2str(idx)],'sppMaxPool3');

        layersSPP = [
            depthConcatenationLayer(4,'Name',['depthConcat_spp_',num2str(idx)]);
            convolution2dLayer(1,ceil(outFilters/2),'Padding','same','Name',['featureConvSPP',num2str(idx)],'WeightsInitializer','he');
            batchNormalizationLayer('Name',['featureBatchNormSPP',num2str(idx)]);
            leakyReluLayer(0.1,'Name',['featureReluSPP',num2str(idx)]);
            ];
        lgraph = addLayers(lgraph,layersSPP);
        lgraph = connectLayers(lgraph,['featureReluInitial',num2str(idx)],['depthConcat_spp_',num2str(idx),'/in1']);
        lgraph = connectLayers(lgraph,'sppMaxPool1',['depthConcat_spp_',num2str(idx),'/in2']);
        lgraph = connectLayers(lgraph,'sppMaxPool2',['depthConcat_spp_',num2str(idx),'/in3']);
        lgraph = connectLayers(lgraph,'sppMaxPool3',['depthConcat_spp_',num2str(idx),'/in4']);
    end

    % Add path aggregation module that concatenates the features in
    % bottom-up approach.
    if numOutputLayers>1
        if (idx>1 && idx<numOutputLayers)
            featureGatherLayers = [
                convolution2dLayer(1,ceil(outFilters/2),'Padding','same','Name',['featureConv_1_',num2str(idx)],'WeightsInitializer','he');
                batchNormalizationLayer('Name',['featureBatchNorm_1_',num2str(idx)]);
                leakyReluLayer(0.1,'Name',['featureRelu_1_',num2str(idx)]);
                resize2dLayer('EnableReferenceInput',true, 'Name',['featureResize',num2str(idx)]);
                depthConcatenationLayer(2,'Name',['depthConcat_top_',num2str(idx)]);
                convolution2dLayer(1,ceil(outFilters/2),'Padding','same','Name',['featureConv_2_',num2str(idx)],'WeightsInitializer','he');
                batchNormalizationLayer('Name',['featureBatchNorm_2_',num2str(idx)]);
                leakyReluLayer(0.1,'Name',['featureRelu_2_',num2str(idx)]);
                depthConcatenationLayer(2,'Name',['depthConcat_bottom_',num2str(idx)]);
                convolution2dLayer(1,ceil(outFilters/2),'Padding','same','Name',['featureConv_3_',num2str(idx)],'WeightsInitializer','he');
                batchNormalizationLayer('Name',['featureBatchNorm_3_',num2str(idx)]);
                leakyReluLayer(0.1,'Name',['featureRelu_3_',num2str(idx)]);
                ];
            detectionSubNetwork = [featureGatherLayers;detectionSubNetwork];
        elseif (idx==numOutputLayers)
            featureGatherLayers = [
                convolution2dLayer(1,ceil(outFilters/2),'Padding','same','Name',['featureConv_1_',num2str(idx)],'WeightsInitializer','he')
                batchNormalizationLayer('Name',['featureBatchNorm_1_',num2str(idx)]);
                leakyReluLayer(0.1,'Name',['featureRelu_1_',num2str(idx)]);
                resize2dLayer('EnableReferenceInput',true, 'Name',['featureResize',num2str(idx)]);
                depthConcatenationLayer(2,'Name',['depthConcat',num2str(idx)]);
                convolution2dLayer(1,ceil(outFilters/2),'Padding','same','Name',['featureConv_2_',num2str(idx)],'WeightsInitializer','he');
                batchNormalizationLayer('Name',['featureBatchNorm_2_',num2str(idx)]);
                leakyReluLayer(0.1,'Name',['featureRelu_2_',num2str(idx)]);
                ];
            detectionSubNetwork = [featureGatherLayers;detectionSubNetwork];
        else
            featureGatherLayers = [
                depthConcatenationLayer(2,'Name',['depthConcat',num2str(idx)]);
                convolution2dLayer(1,ceil(outFilters/2),'Padding','same','Name',['featureConv_1_',num2str(idx)],'WeightsInitializer','he');
                batchNormalizationLayer('Name',['featureBatchNorm_1_',num2str(idx)]);
                leakyReluLayer(0.1,'Name',['featureRelu_1_',num2str(idx)]);
                ];
            detectionSubNetwork = [featureGatherLayers;detectionSubNetwork];
        end
    end

    lgraph = addLayers(lgraph,detectionSubNetwork);

    % Connect appropriate layers.
    if (idx==2 && idx<numOutputLayers)
        lgraph = connectLayers(lgraph,['featureReluSPP',num2str(idx-1)],['featureConv_1_',num2str(idx)]);
        lgraph = connectLayers(lgraph,['featureReluInitial',num2str(idx)],['featureResize',num2str(idx),'/ref']);
        layerToConnect = ['depthConcat_top_',num2str(idx),'/in2'];
    elseif (idx>2 && idx<numOutputLayers)
        lgraph = connectLayers(lgraph,['featureRelu_2_',num2str(idx-1)],['featureConv_1_',num2str(idx)]);
        lgraph = connectLayers(lgraph,['featureReluInitial',num2str(idx)],['featureResize',num2str(idx),'/ref']);
        layerToConnect = ['depthConcat_top_',num2str(idx),'/in2'];
    elseif (idx==numOutputLayers) && (numOutputLayers~=2) && (numOutputLayers>1)
        lgraph = connectLayers(lgraph,['featureRelu_2_',num2str(idx-1)],['featureConv_1_',num2str(idx)]);
        lgraph = connectLayers(lgraph,['featureReluInitial',num2str(idx)],['featureResize',num2str(idx),'/ref']);
        layerToConnect = ['depthConcat',num2str(idx),'/in2'];
    elseif (idx==numOutputLayers) && (numOutputLayers==2)
        lgraph = connectLayers(lgraph,['featureReluSPP',num2str(idx-1)],['featureConv_1_',num2str(idx)]);
        lgraph = connectLayers(lgraph,['featureReluInitial',num2str(idx)],['featureResize',num2str(idx),'/ref']);
        layerToConnect = ['depthConcat',num2str(idx),'/in2'];
    elseif (idx==numOutputLayers) && (numOutputLayers==1)
        lgraph = connectLayers(lgraph,['featureReluSPP',num2str(idx)],['customConv',num2str(idx)]);
    else
        layerToConnect = ['depthConcat',num2str(idx),'/in2'];
    end

    if numOutputLayers>1
        if (idx==1)
            lgraph = connectLayers(lgraph, ['featureReluSPP',num2str(idx)], layerToConnect);
        else
            lgraph = connectLayers(lgraph, ['featureReluInitial',num2str(idx)], layerToConnect);
        end
    end
end

% Add path aggregation module that concatenates the features in top-down approach.
if numOutputLayers>1
    for idx = numOutputLayers:-1:2
        activationSize = orderedNetworkActivationSizes{idx,1};
        outFilters = activationSize(3);

        featureGatherLayers = [
            convolution2dLayer(1,ceil(outFilters/2),'Stride',2,'Padding','same','Name',['featureConv_bottom_',num2str(idx)],'WeightsInitializer','he')
            batchNormalizationLayer('Name',['featureBatchNorm_bottom_',num2str(idx)]);
            leakyReluLayer(0.1,'Name',['featureRelu_bottom_',num2str(idx)]);
            ];

        lgraph = addLayers(lgraph,featureGatherLayers);

        % Connect appropriate layers.
        if (idx==numOutputLayers) && (numOutputLayers~=2)
            lgraph = connectLayers(lgraph, ['featureRelu_2_',num2str(idx)], ['featureConv_bottom_',num2str(idx)]);
            lgraph = connectLayers(lgraph, ['featureRelu_bottom_',num2str(idx)], ['depthConcat_bottom_',num2str(idx-1),'/in2']);
        elseif (idx==numOutputLayers) && (numOutputLayers==2)
            lgraph = connectLayers(lgraph, ['featureRelu_2_',num2str(idx)], ['featureConv_bottom_',num2str(idx)]);
            lgraph = connectLayers(lgraph, ['featureRelu_bottom_',num2str(idx)], ['depthConcat',num2str(idx-1),'/in1']);
        elseif (idx==2 && idx<numOutputLayers)
            lgraph = connectLayers(lgraph, ['featureRelu_3_',num2str(idx)], ['featureConv_bottom_',num2str(idx)]);
            lgraph = connectLayers(lgraph, ['featureRelu_bottom_',num2str(idx)], ['depthConcat',num2str(idx-1),'/in1']);
        else
            lgraph = connectLayers(lgraph, ['featureRelu_3_',num2str(idx)], ['featureConv_bottom_',num2str(idx)]);
            lgraph = connectLayers(lgraph, ['featureRelu_bottom_',num2str(idx)], ['depthConcat_bottom_',num2str(idx-1),'/in2']);
        end
    end
end
end


%--------------------------------------------------------------------------
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

%--------------------------------------------------------------------------
function [output, scale] = iPreprocessSingleImg(img, targetSize)
% Compute the scale to resize the groundtruth bounding boxes.
imgSize = size(img);
scale = targetSize(1:2)./imgSize(1:2);

% Resize the img to targetSize.
img = imresize(img, targetSize(1:2));
img = single(img);

% Rescale the img in the range [0,1].
output = rescale(img,0,1);
end

%--------------------------------------------------------------------------
function [Ipreprocessed,info] = iPreprocessForDetect(I, roi, useROI, executionEnvironment, networkInputSize, castToGpuArray)
% Check if the input datatype is valid or not.
if ~(isa(I,'uint8') || isa(I,'uint16') || isa(I,'int16') ||...
        isa(I,'double') || isa(I,'single') || isa(I,'gpuArray'))
    error(message('vision:ObjectDetector:unSupportedInputClass'));
end

% Convert to gpuArray based on executionEnvironment.
if castToGpuArray
if (strcmp(executionEnvironment,'auto') && canUseGPU) || strcmp(executionEnvironment,'gpu')
    I = gpuArray(I);
end
end

% Crop image if requested.
Iroi = vision.internal.detector.cropImageIfRequested(I, roi, useROI);

% Save preprocessed image size.
info.PreprocessedImageSize = networkInputSize;

% Compute scale factors to scale boxes from targetSize back to the input size.
sz = size(Iroi);
scale   = sz(1:2)./networkInputSize(1:2);
[info.ScaleX,info.ScaleY] = deal(scale(2),scale(1));

Ipreprocessed = yolov4ObjectDetector.preprocessInput(Iroi, info.PreprocessedImageSize);

Ipreprocessed = dlarray(Ipreprocessed,'SSCB');
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
function s = iGetDefaultYOLOv4DetectionParams()
s.roi                     = zeros(0,4);
s.SelectStrongest         = true;
s.Threshold               = 0.5;
s.MinSize                 = [1,1];
s.MaxSize                 = [];
s.MiniBatchSize           = 128;
s.ExecutionEnvironment    = 'auto';
s.Acceleration            = 'auto';
end

%--------------------------------------------------------------------------
function output = iValidateNetwork(network,mfilename)
% YOLOv4 network should be a dlnetwork object and must contain image input
% layer. 'Normalization' in image input layer must be 'none'.
validateattributes(network,{'dlnetwork'},...
    {'scalar'}, mfilename);

imgIdx = arrayfun(@(x)isa(x,'nnet.cnn.layer.ImageInputLayer'),...
    network.Layers);
imageInputIdx = find(imgIdx,1,'first');

if isempty(imageInputIdx)
    error(message('vision:yolo:mustBeImageInputLayer'));
else
    output = imageInputIdx;
end
end

%--------------------------------------------------------------------------
function iVerifyFullyConnectedExistence(network)
% YOLOv4 network is based on Convolution Layers and should not
% contain any fullyConnected Layers.
idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.FullyConnectedLayer'),...
    network.Layers);
if sum(idx) ~= 0
    error(message("vision:yolo:mustNotHaveAnyFCLayer"));
end
end

%--------------------------------------------------------------------------
function iVerifyGlobalAvgPoolExistence(network)
% YOLOv4 network should not contain any global average pooling layer as it
% downsamples input feature map to size of [1,1].
idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.GlobalAveragePooling2DLayer')||isa(x,'nnet.cnn.layer.GlobalMaxPooling2DLayer'),...
    network.Layers);
if sum(idx) ~= 0
    error(message("vision:yolo:mustNotHaveAnyGlobalPoolingLayer"));
end
end

%--------------------------------------------------------------------------
function detector = iTripwireDefaultYOLOv4Model()
% Check if support package is installed
breadcrumbFile = 'vision.internal.cnn.supportpackages.IsYOLOv4Installed';
fullPath = which(breadcrumbFile);
if isempty(fullPath)
    name     = 'Computer Vision Toolbox Model for YOLO v4 Object Detection';
    basecode = 'YOLOV4';

    throwAsCaller(MException(message('nnet_cnn:supportpackages:InstallRequired', mfilename, name, basecode)));
else
    % Load pretrained network.
    pattern = fullfile(filesep, '+vision','+internal','+cnn','+supportpackages','IsYOLOv4Installed.m');
    idx     = strfind(fullPath, pattern);
    matfile = fullfile(fullPath(1:idx), 'data', 'yolov4COCO.mat');
    data = load(matfile);
    detector = data.detector;
end
end

%--------------------------------------------------------------------------
function detector = iTripwireYOLOv4Model(params)
% Check if support package is installed
breadcrumbFile = 'vision.internal.cnn.supportpackages.IsYOLOv4Installed';
fullPath = which(breadcrumbFile);
if isempty(fullPath)
    name     = 'Computer Vision Toolbox Model for YOLO v4 Object Detection';
    basecode = 'YOLOV4';

    throwAsCaller(MException(message('nnet_cnn:supportpackages:InstallRequired', mfilename, name, basecode)));
else
    if(isfield(params,'ClassNames'))
        pattern = fullfile(filesep, '+vision','+internal','+cnn','+supportpackages','IsYOLOv4Installed.m');
        idx     = strfind(fullPath, pattern);

        if strcmp(params.DetectorName, 'csp-darknet53-coco')
            matfile = fullfile(fullPath(1:idx), 'data', 'yolov4COCO.mat');
        else
            matfile = fullfile(fullPath(1:idx), 'data', 'tinyYOLOv4COCO.mat');
        end
        data = load(matfile);
        network = data.detector.Network;
        pretrainedDetector = data.detector;
        pretrainedDetectionNetworkSource = data.detector.DetectionNetworkSource;
        classes = params.ClassNames;
        inputSize = params.InputSize;
        anchorBoxes = params.AnchorBoxes;
        predictedBoxType = params.PredictedBoxType;


        if isempty(params.DetectionNetworkSource)

            % Validate anchorBoxes
        if strcmp(params.DetectorName, 'csp-darknet53-coco')
            anchorBoxes = params.AnchorBoxes;
            if (size(anchorBoxes,1) ~= 3)
                error(message('vision:yolo:numPretrainedAnchorsMismatch',params.ModelName,'3-by-1'));
            end

            anchorsSet1 = anchorBoxes{1,1};
            anchorsSet2 = anchorBoxes{2,1};
            anchorsSet3 = anchorBoxes{3,1};

            anchorsSet1 = iSortAscendAnchors(anchorsSet1);
            anchorsSet2 = iSortAscendAnchors(anchorsSet2);
            anchorsSet3 = iSortAscendAnchors(anchorsSet3);

            anchorBoxes = {anchorsSet3
                anchorsSet2
                anchorsSet1};
        else
            anchorBoxes = params.AnchorBoxes;
            if (size(anchorBoxes,1) ~= 2)
                error(message('vision:yolo:numPretrainedAnchorsMismatch',params.ModelName,'2-by-1'));
            end
        end

        if strcmp(params.DetectorName, 'csp-darknet53-coco')
            layersToReplace = {'conv_140','conv_151','conv_162'};
        else
            layersToReplace = {'conv_31','conv_38'};
        end

            % Create pre-trained detector with custom classes and
            % anchorBoxes
        network = iConfigureNetwork(network,anchorBoxes,classes,layersToReplace,inputSize,predictedBoxType);
            detector = yolov4ObjectDetector(network,classes,anchorBoxes,'ModelName',params.ModelName,...
                'InputSize',params.InputSize,'DetectionNetworkSource',params.DetectionNetworkSource,...
                'PredictedBoxType',params.PredictedBoxType);
            detector.DetectionNetworkSource = pretrainedDetectionNetworkSource;

        else
            % validate anchor boxes
            if (size(anchorBoxes,1) ~= size(params.DetectionNetworkSource,2))
                error(message('vision:yolo:pretrainedBackboneCustomYOLOv4AnchorsMismatch', ...
                    num2str(size(params.DetectionNetworkSource,2))));
            end

            % Validate detection network source
            validateattributes(params.DetectionNetworkSource, {'cell','string'}, {'row', 'size', [1 NaN]}, ...
                mfilename, 'DetectionNetworkSource');

            % Verify detection network source layers are present in pretrained network
            networkLg = layerGraph(network);
            iVerifyLayersExist(networkLg, params.DetectionNetworkSource);

            %  Only backbone layers can be specified as detection network source layers
            backboneLayers = iGetLayersBefore(networkLg,pretrainedDetectionNetworkSource{1});
            if all(ismember(params.DetectionNetworkSource, backboneLayers))

                % Create custom YOLO v4 detector from preained backbone with custom classes and
                % anchorBoxes
                [pretrainedBackbone, ~,~] = iSubNetworks(pretrainedDetector,pretrainedDetectionNetworkSource); % Pretrained Backbone
                detector = yolov4ObjectDetector(pretrainedBackbone,classes,anchorBoxes,'ModelName',params.ModelName,...
                    'InputSize',params.InputSize,'DetectionNetworkSource',params.DetectionNetworkSource,...
                    'PredictedBoxType',params.PredictedBoxType);
            else
                error(message('vision:yolo:invalidDetectionNetworkSourceForCustomYOLOV4withPretrainedBackbone'));
            end

        end

    elseif (isfield(params,'isCustomModelName'))
        % Load pretrained network and update model name.
        pattern = fullfile(filesep, '+vision','+internal','+cnn','+supportpackages','IsYOLOv4Installed.m');
        idx     = strfind(fullPath, pattern);
        if strcmp(params.DetectorName, 'csp-darknet53-coco')
            matfile = fullfile(fullPath(1:idx), 'data', 'yolov4COCO.mat'); %apply if-else
        else
            matfile = fullfile(fullPath(1:idx), 'data', 'tinyYOLOv4COCO.mat');
        end
        data = load(matfile);
        detector = data.detector;
        detector.ModelName = params.ModelName;

    else
        % Load pretrained network.
        pattern = fullfile(filesep, '+vision','+internal','+cnn','+supportpackages','IsYOLOv4Installed.m');
        idx     = strfind(fullPath, pattern);
        if strcmp(params.DetectorName, 'csp-darknet53-coco')
            matfile = fullfile(fullPath(1:idx), 'data', 'yolov4COCO.mat');
        else
            matfile = fullfile(fullPath(1:idx), 'data', 'tinyYOLOv4COCO.mat');
        end
        data = load(matfile);
        detector = data.detector;
    end
end
end

%--------------------------------------------------------------------------
function network = iConfigureNetwork(network,anchorBoxes,classes, layersToReplace,inputSize,predictedBoxType)
lgraph = iUpdateFirstConvChannelsAndInputLayer(network,inputSize);

% Set rotationOffset for rotated rectangle bounding boxes
if strcmp(predictedBoxType,"rotated")
    rotationOffset = 2;
else
    rotationOffset = 0;
end

for i = 1:size(layersToReplace,2)
    numAnchors = size(anchorBoxes{i,1},1);
    numClasses = size(classes,1);
    numFilters = (numClasses + 5 + rotationOffset)*numAnchors;
    convOut = convolution2dLayer([1,1],numFilters,'Padding','same','Name',['convOut',num2str(i)]);
    lgraph = replaceLayer(lgraph,layersToReplace{1,i},convOut);
end
network = dlnetwork(lgraph);
end

%--------------------------------------------------------------------------
function anchors = iSortAscendAnchors(anchors)
% Sort the anchor boxes in ascending order of their areas.
area = anchors(:, 1).*anchors(:, 2);
[~, idx] = sort(area, 'ascend');
anchors = anchors(idx, :);
end

% -------------------------------------------------------------------------
% iDlNetwork : Merges sub-networks to form Network
% -------------------------------------------------------------------------
function dlnetOut = iDlNetwork(detector)
% Reconstruct a monolithic dlnetwork from the backbone, neck
% and head
dlnetBackbone = detector.Backbone;
dlnetNeck = detector.Neck;
dlnetHead = detector.Head;
dlnetOut = dlnetBackbone;
dlnetOut = addLayers(dlnetOut,dlnetNeck.Layers);
numHeads = size(detector.Head,2);

% Disconnect all neck layers
for idx = 1:numel(dlnetNeck.Layers)-1
    srcName = dlnetNeck.Layers(idx).Name;
    destName = dlnetNeck.Layers(idx+1).Name;
    if ~isempty(strfind(dlnetNeck.Layers(idx+1).Name,'cat'))
        destName = strcat(destName,'/in1');
    end
    if ~isempty(strfind(dlnetNeck.Layers(idx+1).Name,'featureResize'))
        destName = strcat(destName,'/in');
    end
    dlnetOut = disconnectLayers(dlnetOut,srcName,destName);
end

% Reconnect all neck layers
lgraphNeckConnections = dlnetNeck.Connections;
numBackboneNeckConnections = numel(dlnetNeck.InputNames);

for idx = 1:height(lgraphNeckConnections)
    dlnetOut = connectLayers(dlnetOut,lgraphNeckConnections.Source{idx},lgraphNeckConnections.Destination{idx});
end

% Connect backbone to neck
if isequal(detector.DetectionNetworkSource,{'mish_106','mish_87','mish_56'}) %csp-darknet53-coco
    dlnetOut = connectLayers(dlnetOut,'mish_106','conv_107');
    dlnetOut = connectLayers(dlnetOut,'mish_87','conv_122');
    dlnetOut = connectLayers(dlnetOut,'mish_56','conv_132');
elseif isequal(detector.DetectionNetworkSource,{'leaky_28','leaky_25'}) %tiny-yolov4-coco
    dlnetOut = connectLayers(dlnetOut,'leaky_28','conv_29');
    dlnetOut = connectLayers(dlnetOut,'leaky_25','concat_36/in2');
else
    for idx=1:numBackboneNeckConnections
        % DetectionNetworkSource will already be sorted from lowest feature map
        % layer to highest
        dlnetOut = connectLayers(dlnetOut,detector.DetectionNetworkSource{idx}, ...
            ['featureConvInitial',num2str(idx)]);
    end
end


for n=1:numHeads
    dlnetOut = addLayers(dlnetOut,dlnetHead{n}.Layers);
    % Disconnect all head layers
    for idx = 1:numel(dlnetHead{n}.Layers)-1
        srcName = dlnetHead{n}.Layers(idx).Name;
        destName = dlnetHead{n}.Layers(idx+1).Name;
        dlnetOut = disconnectLayers(dlnetOut,srcName,destName);
    end

    % Reconnect all head layers
    lgraphHeadConnections = dlnetHead{n}.Connections;
    for idx = 1:height(lgraphHeadConnections)
        dlnetOut = connectLayers(dlnetOut,lgraphHeadConnections.Source{idx},lgraphHeadConnections.Destination{idx});
    end
end

% Connect neck to head
if isequal(detector.DetectionNetworkSource,{'mish_106','mish_87','mish_56'})
    dlnetOut = connectLayers(dlnetOut,'leaky_138','conv_139');
    dlnetOut = connectLayers(dlnetOut,'leaky_149','conv_150');
    dlnetOut = connectLayers(dlnetOut,'leaky_160','conv_161');
elseif isequal(detector.DetectionNetworkSource,{'leaky_28','leaky_25'})
    dlnetOut = connectLayers(dlnetOut,'leaky_29','conv_30');
    dlnetOut = connectLayers(dlnetOut,'concat_36','conv_37');
else
    if numHeads==1
        dlnetOut = connectLayers(dlnetOut,['featureReluSPP',num2str(1)],['customConv',num2str(1)]);
    else
        for idx=1:numHeads
            if idx==1
                dlnetOut = connectLayers(dlnetOut,['featureRelu_1_',num2str(idx)],['customConv',num2str(idx)]);
            elseif idx==numHeads
                dlnetOut = connectLayers(dlnetOut,['featureRelu_2_',num2str(idx)],['customConv',num2str(idx)]);
            else
                dlnetOut = connectLayers(dlnetOut,['featureRelu_3_',num2str(idx)],['customConv',num2str(idx)]);
            end
        end
    end
end

dlX = dlarray(rand(detector.InputSize, 'single'), 'SSCB');
dlnetOut.OutputNames = [];
dlnetOut = initialize(dlnetOut,dlX);

end

% ------------------------------------------------------------------------
% isubNetwoks : Break Network into subNetworks
% ------------------------------------------------------------------------
function [backbone, head, neck] = iSubNetworks(detector,detectionNetworkSource)
% detectionNetworkSource must be sorted in ascending order of feature map
% values of respective layers
lg = layerGraph(detector.Network);
outputNames = detector.Network.OutputNames;

% last layer of backbone - backboneLast
allLayers = {lg.Layers.Name};
backboneLast = detectionNetworkSource{1};

% Backbone
backboneLayers = iGetLayersBefore(lg,backboneLast);
backboneLg = removeLayers(lg,allLayers(~ismember(allLayers, backboneLayers)));

% Head
numHeads = size(outputNames,2);
headLg = cell(numHeads,1);
headLayers = {};
neckLastLayers = {};
for i=1: numHeads
    headILayers = cell(4,1); % yolo v4 type head
    headILayers{1} = outputNames{i};
    headILayers{2} = lg.Connections.Source{strcmp(lg.Connections.Destination,headILayers{1})};
    headILayers{3} = lg.Connections.Source{strcmp(lg.Connections.Destination,headILayers{2})};
    headILayers{4} = lg.Connections.Source{strcmp(lg.Connections.Destination,headILayers{3})};

    neckLastLayers = [neckLastLayers(:)' lg.Connections.Source{strcmp(lg.Connections.Destination,headILayers{4})}];

    headLg{i,1} = removeLayers(lg,allLayers(~ismember(allLayers, headILayers)));
    headLayers = [headLayers(:)' headILayers(:)'];
end

% Neck
neckLg = removeLayers(lg,allLayers(ismember(allLayers,  [headLayers(:)', backboneLayers(:)'])));

% create dlnetworks from layer graphs
inputSize = backboneLg.Layers(1).InputSize;
input = dlarray(randn(inputSize),"SSCB");

% Backbone
backboneFeatures = cell(size(detectionNetworkSource));
backbone = dlnetwork(backboneLg,OutputNames=detectionNetworkSource);
[backboneFeatures{:}] = predict(backbone,input);

% Neck
neckFeatures = cell(size(neckLastLayers));
neck = dlnetwork(neckLg,backboneFeatures{:}, OutputNames=neckLastLayers); % output names important OutputNames={s.neckOutput}
[neckFeatures{:}] = predict(neck,backboneFeatures{:});

% Head
head = cell(1,numHeads);
for i=1: numHeads
    head{i} = dlnetwork(headLg{i}, neckFeatures{i});
end
end

% -----------------------------------------------------------------------
% Helpers for iSubNetworks function
%------------------------------------------------------------------------
% Get all layers before given layer name in lgraph
function layersBefore = iGetLayersBefore(lgraph,layer)
dg = vision.internal.cnn.RCNNLayers.digraph(lgraph);
id=findnode(dg,layer);

% Get layers before layer
if ~(sum(id)==0)
    dg = flipedge(dg);
    layersBefore = dfsearch(dg,layer);
end
end

%--------------------------------------------------------------------------
% Updated Exisitng Function
%--------------------------------------------------------------------------
function lgraphOut = iRemoveLayersAfterSpecifiedLayerAndUnconnectedLayers(lgraph,layerName)
% Remove all the layers from the layergraph after the specified layername.

allLayers = {lgraph.Layers.Name};
% Remove layers after layerName
% Remove any lingering branches to removed layers in lgraph

dg = vision.internal.cnn.RCNNLayers.digraph(lgraph);
id=findnode(dg,layerName);
if ~(sum(id)==0)
    dg = flipedge(dg);
    layersBefore = dfsearch(dg,layerName);
    % Remove lingering branches along with layer after layerNames
    lgraphOut = removeLayers(lgraph, allLayers(~(ismember(allLayers, layersBefore))));
end

end
