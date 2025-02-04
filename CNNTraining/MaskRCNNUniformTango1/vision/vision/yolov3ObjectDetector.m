%yolov3ObjectDetector  Detect objects using YOLO v3 deep learning detector.
%
% detector = yolov3ObjectDetector() loads a YOLO v3 object detector trained
% to detect 80 object classes from the COCO dataset using a darknet53
% network.
%
% detector = yolov3ObjectDetector(detectorName) loads a pretrained YOLO v3
% object detector specified by detectorName. detectorName must be
% 'darknet53-coco' or 'tiny-yolov3-coco'.
%
% detector = yolov3ObjectDetector(detectorName, classNames, anchorBoxes)
% configures a pretrained YOLO v3 object detector for transfer learning
% with a new set of object classes and anchor boxes. You must train the
% detector for optimal performance.
%
% detector = yolov3ObjectDetector(network, classNames, anchorBoxes)
% creates a YOLO v3 object detector by using the specified YOLO v3 deep
% learning network and configures for training with a new set of object
% classes and anchor boxes. The network must be a dlnetwork. Use this syntax
% to specify a custom YOLO v3 network for object detection. You must train
% the detector for optimal performance.
%
% detector = yolov3ObjectDetector(network, classNames, anchorBoxes, 'DetectionNetworkSource', layerName)
% creates a YOLO v3 object detector by using the specified deep learning
% network as feature extraction network. This syntax will add detection
% subnetworks to the layers of the feature extraction network specified
% in DetectionNetworkSoruce. You must train the detector for optimal performance.
%
% Inputs:
% -------
%    detectorName   Specify the name of the pretrained YOLO v3 deep learning
%                   network as a string or character vector. The value must
%                   be one of the following:
%
%                       'darknet53-coco'    To use a YOLO v3 deep learning
%                                           network that has Darknet-53 as
%                                           base network and is trained on
%                                           COCO dataset.
%
%                       'tiny-yolov3-coco'  To use a YOLO v3 deep learning
%                                           network that has a smaller base
%                                           network and is trained on COCO
%                                           dataset.
%
%    network        Specify the input network as a dlnetwork object that has
%                   an image input layer. 'Normalization' in the image input
%                   layer must be 'none'. The outputs of the network is used
%                   for predicting object detections. The network must have
%                   the same number of outputs as the number of anchorBoxes.
%                   To select which network outputs to use for prediction,
%                   use the DetectionNetworkSource parameter.
%
%    classNames     Specify the names of object classes that the YOLO v3
%                   object detector is configured to detect. classNames can
%                   be a string vector, a categorical vector, or a cell
%                   array of character vectors.
%
%    anchorBoxes    Specify the size of anchor bases as an N-by-1 cell array
%                   of M-by-2 matrices, where each row defines the
%                   [height width] of an anchor box. N is the number of
%                   outputs in the network or the number of names specified
%                   using the DetectionNetworkSource argument.
%
% % Additional input arguments
% ----------------------------
% [...] = yolov3ObjectDetector(..., Name=Value) specifies additional
% name-value pair arguments described below:
%
%   'ModelName'                Specify the name for the object detector
%                              as a string or character vector.
%
%                              Default: '' or specified detectorName
%
%   'InputSize'                Specify the image size used for training
%                              the detector. The input size must be H-by-W
%                              or H-by-W-by-C.
%
%                              Default: network input size
%
%   'PredictedBoxType'         Specify the type of the bounding boxes
%                              to be returned by the object detector.
%                              The value must be either "axis-aligned"
%                              for axis-aligned bounding boxes or
%                              "rotated" for rotated rectangle bounding
%                              boxes.
%
%                              Default: "axis-aligned"
%
%   'DetectionNetworkSource'   Specify the names of layers in the input
%                              network to which you want to add the
%                              detection subnetworks. The value must be a
%                              string vector or cell array of character
%                              vectors of size 1-by-M. If it is not
%                              specified or empty, then sub-networks will
%                              not be added to the input network and it is
%                              expected to be a complete yolo v3 network
%                              with detection subnetworks.
%
%                              Default: {}
%
% yolov3ObjectDetector methods:
%   detect           - Detect objects in an image.
%   preprocess       - Preprocess input data.
%   predict          - Compute detector network output for inference.
%   forward          - Compute detector network output for training.
%
% yolov3ObjectDetector properties:
%   ModelName          - Name of the trained object detector.
%   Network            - YOLO v3 object detection network. (read-only)
%   ClassNames         - A cell array of object class names. (read-only)
%   AnchorBoxes        - A cell array of anchor boxes. (read-only)
%   InputSize          - Image size used during training.(read-only)
%   PredictedBoxType   - Bounding box format of the detector's prediction. (read-only)
%
% Example 1: Detect objects using pretrained YOLO v3.
% ---------------------------------------------------
% % Load pre-trained detector.
% detector = yolov3ObjectDetector();
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
% Example 2: Detect objects using custom preprocessing on pretrained YOLO v3.
% ---------------------------------------------------------------------------
% % Load pre-trained detector.
% detector = yolov3ObjectDetector('tiny-yolov3-coco');
%
% % Read test image.
% I = imread('highway.png');
%
% % Resize the test image to network input size.
% I = imresize(I, detector.InputSize(1:2));
% I = im2single(I);
%
% % Run detector.
% [bboxes, scores, labels] = detect(detector, I, 'DetectionPreprocessing', 'none');
%
% % Display results.
% detectedImg = insertObjectAnnotation(I, 'Rectangle', bboxes, labels);
% figure
% imshow(detectedImg)
%
% Example 3: Configure a pretrained YOLO v3 detector for transfer learning.
% -------------------------------------------------------------------------
% % Specify the input image size.
% imageSize = [224 224 3];
%
% % Specify the class names.
% classes = {'vehicle'};
%
% % Specify the anchor boxes.
% anchorBoxes = {[122,177;223,84;80,94];...
%               [111,38;33,47;37,18]};
%
% % Configure the detector.
% detector = yolov3ObjectDetector('tiny-yolov3-coco',classes,anchorBoxes,...
%      'InputSize',imageSize);
%
% See also ssdObjectDetector, yolov2ObjectDetector,
%          yolov4ObjectDetector, maskrcnn,rcnnObjectDetector,
%          fastRCNNObjectDetector, fasterRCNNObjectDetector,
%          imageLabeler, trainNetwork.

% Copyright 2022-2024 The MathWorks, Inc.

classdef yolov3ObjectDetector < vision.internal.detector.ObjectDetector   &...
        deep.internal.sdk.LearnableParameterContainer &...
        images.dltrain.internal.ValidationTimeInferable

    properties(SetAccess = protected)
        % Network is a dlnetwork object with image input layer.
        Network

        % AnchorBoxes is a N-by-1 cell array where each cell element
        % contains an M-by-2 array, where M is number of anchor boxes, N is
        % either network.Outputs or the number of elements in
        % DetectionNetworkSource and M corresponds to array of anchor boxes
        % in [height width] format.
        AnchorBoxes

        % ClassNames specifies the names of the classes that YOLO v3 object
        % detector can detect.
        ClassNames

        % InputSize is a vector of the form [height width] or [height width
        % channels] defining image size used to train the detector. During
        % detection, an input image is resized to InputSize before it is
        % processed by the detection network when the
        % DetectionPreprocessing NVP is 'auto'.
        InputSize


        % PredictedBoxType is a string that specifies the type of the bounding
        % boxes returned by the object detector. The type of bounding box a
        % detector returns depends on the training data used to train the
        % object detector. PredictedBoxType is specified as "axis-aligned"
        % for axis-aligned bounding boxes or "rotated" for rotated rectangle
        % bounding boxes.
        PredictedBoxType
    end

    properties (Access = protected, Transient)
        FilterBboxesFunctor
    end

    properties(Access = private, Transient)
        % LayerIndices  A struct that caches indices to certain layers used
        %               frequently during detection.
        LayerIndices
    end

    properties(Dependent = true, Hidden = true)
        % These properties are accesed in dltrain during training the
        % detector.

        % Learnables is the learnable parameters for the YOLO v3 dlnetwork.
        Learnables

        % State is the state of the non-learnable parameters of the
        % YOLO v3 dlnetwork.
        State

        % Layers is the array of layers in the YOLO v3 dlnetwork.
        Layers

        % InputNames is the cell array of input names for the YOLO v3 dlnetwork.
        InputNames

        % OutputNames is the cell array of output names for the YOLO v3 dlnetwork.
        OutputNames

    end

    properties (SetAccess = protected, Hidden)
        % UseHighLevelTrainer is a logical variable.
        % UseHighLevelTrainer is set to true when YOLO v3 is trained using
        % trainYOLOv3ObjectDetector else set to false to ensure backward
        % compatibility while training YOLO v3 using custom training loop
        UseHighLevelTrainer=false;
    end

    properties(SetAccess = protected, Hidden)
        % Backbone is a dlnetwork object with image input layer and
        % backbone layers.
        Backbone;

        % Neck is a cell array of dlnetwork object or a dlnetwork object
        % with FPN network as Neck layers.
        Neck;

        % Head is a cell array of dlnetwork objects for each detection heads.
        Head;
    end

    properties(SetAccess = protected, Hidden)
        % DetectionNetworkSource is a cell array
        % specifying the names of layers in the input network to
        % which detection subnetworks are connected.
        DetectionNetworkSource
    end

    properties(Access = private, Transient)
        numHeads
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
        function this = yolov3ObjectDetector(varargin)
            vision.internal.requiresNeuralToolbox(mfilename);
            narginchk(0,11);

            if nargin < 1
                % Loads the default pretrained model.
                this = iTripwireDefaultYOLOv3Model;

            elseif (isa(varargin{1,1},'string')||isa(varargin{1,1},'char'))
                % Loads and configure the pretrained model as specified in detectorName.
                params = yolov3ObjectDetector.parsePretrainedDetectorInputs(varargin{:});
                this = iTripwireYOLOv3Model(params);

            else
                % Creates custom YOLO v3 object detector.
                narginchk(3,11);
                params = yolov3ObjectDetector.parseDetectorInputs(varargin{:});
                this.Network = params.Network;
                % Create categories of this.ClassNames such that the order of the params.ClassNames is retained.
                this.ClassNames = categorical(params.ClassNames,cellstr(params.ClassNames));
                this.AnchorBoxes = params.AnchorBoxes;
                this.InputSize = params.InputSize;
                this.ModelName = params.ModelName;
                this.DetectionNetworkSource = params.DetectionNetworkSource;
                this.PredictedBoxType = params.PredictedBoxType;
            end
            this.FilterBboxesFunctor = vision.internal.cnn.utils.FilterBboxesFunctor;
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
                if this.FreezeBackbone && ~isempty(this.Neck)
                    if isa(this.Neck, 'dlnetwork')
                        val = vertcat(this.Neck.Learnables);
                    else
                        neckLearnables = cellfun(@(neck){neck.Learnables},this.Neck,'UniformOutput',true);
                        val = vertcat(neckLearnables{:});
                    end
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
                if this.FreezeBackbone && ~isempty(this.Neck)
                    if isa(this.Neck, 'dlnetwork')
                        numNeckLearnables = size(this.Neck.Learnables,1);
                        this.Neck.Learnables = val(1:numNeckLearnables,:);
                        learnableIdx = numNeckLearnables;
                    else
                        numNeckLearnables = cellfun(@(neck){size(neck.Learnables,1)},this.Neck,'UniformOutput',true);
                        for i=1:size(this.Neck,2)
                            this.Neck{i}.Learnables = val(learnableIdx+1:learnableIdx+numNeckLearnables{i},:);
                            learnableIdx = learnableIdx + numNeckLearnables{i};
                        end
                    end
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
                if this.FreezeBackbone && ~isempty(this.Neck)
                    if isa(this.Neck, 'dlnetwork')
                        val = vertcat(this.Neck.State);
                    else
                        neckState = cellfun(@(neck){neck.State},this.Neck,'UniformOutput',true);
                        val = vertcat(neckState{:});
                    end
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
                if this.FreezeBackbone && ~isempty(this.Neck)
                    if isa(this.Neck, 'dlnetwork')
                        numNeckState = size(this.Neck.State,1);
                        this.Neck.State = val(1:numNeckState,:);
                        stateIdx = numNeckState;
                    else
                        numNeckState = cellfun(@(neck){size(neck.State,1)},this.Neck,'UniformOutput',true);
                        for i=1:size(this.Neck,2)
                            this.Neck{i}.State = val(stateIdx+1:stateIdx+numNeckState{i},:);
                            stateIdx = stateIdx + numNeckState{i};
                        end
                    end
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


        %------------------------------------------------------------------
        function this = set.FreezeBackboneAndNeck(this,TF)
            this.FreezeBackboneAndNeckInternal(2,1) = TF;
        end

        function TF = get.FreezeBackboneAndNeck(this)
            TF = this.FreezeBackboneAndNeckInternal(2,1);
        end
        %------------------------------------------------------------------
    end

    methods(Hidden)
        %------------------------------------------------------------------
        % set/unset UseHighLevelTrainer property for yolov3ObjectDetector
        %------------------------------------------------------------------
        function this = updateUseHighLevelTrainerFlag(this,val)
            this.UseHighLevelTrainer = val;
        end
    end

    methods(Hidden)
        % -----------------------------------------------------------------
        % Initialize sub-networks in detector
        % -----------------------------------------------------------------
        function this = initialize(this)
            pretrainedYOLOv3 = isequal(this.DetectionNetworkSource,{'add_23','add_19','add_11'}) ||  ...
                isequal(this.DetectionNetworkSource,{'leaky_relu_7','leaky_relu_5'});
            dlX = dlarray(rand(this.InputSize, 'single'), 'SSCB');
            backboneFeatures = cell(size(this.Backbone.OutputNames'));
            [backboneFeatures{:}] = predict(this.Backbone, dlX);

            if ~isempty(this.Neck)
                if isa(this.Neck, 'dlnetwork')
                    neckFeatures = cell(size(this.Neck.OutputNames'));
                    if pretrainedYOLOv3 && size(this.DetectionNetworkSource,2)==3
                        neckInputFeatures = backboneFeatures([1,2,2,3,3]);
                    elseif pretrainedYOLOv3 && size(this.DetectionNetworkSource,2)==2
                        neckInputFeatures = backboneFeatures([1,2,2]);
                    else
                        neckInputFeatures = backboneFeatures([1,2,2]);
                    end

                    if(~this.Neck.Initialized)
                        this.Neck = initialize(this.Neck, neckInputFeatures{:});
                    end
                    [neckFeatures{:}] = predict(this.Neck,neckInputFeatures{:});
                elseif isa(this.Neck, 'cell')
                    neckFeatures= cell(size(cellfun(@(n){n.OutputNames{:}},this.Neck,'UniformOutput',true)'));
                    for idx=1:size(this.Neck,2)
                        currNeck = this.Neck{idx};
                        neckInputFeatures = backboneFeatures([idx,idx+1,idx+1]);

                        if(~currNeck.Initialized)
                            this.Neck{idx} = initialize(currNeck, neckInputFeatures{:});
                        end
                        neckFeatures{idx} = predict(this.Neck{idx},neckInputFeatures{:});
                    end
                end
            end

            for i=1:this.numHeads
                if pretrainedYOLOv3 && ~this.Head{i}.Initialized
                    this.Head{i} = initialize(this.Head{i}, neckFeatures{i});
                elseif ~this.Head{i}.Initialized
                    if i==1
                        this.Head{i} = initialize(this.Head{i}, backboneFeatures{1}); % no neck layer for first head
                    else
                        this.Head{i} = initialize(this.Head{i}, neckFeatures{i-1});
                    end
                end
            end
        end


        % -----------------------------------------------------------------
        % Update Network from sub-networks
        % -------------------------------------------------------------------
        function this = updateDetector(this)
            if ~isempty(this.Backbone) && ~isempty(this.Neck) && ~isempty(this.Head) || ...
                    ~isempty(this.Backbone) && isempty(this.Neck) && ~isempty(this.Head) % custom yolo v3 with 1 output layer has no neck
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


    methods
        %------------------------------------------------------------------
        % Preprocess input data.
        %------------------------------------------------------------------
        function varargout = preprocess(detector, I, varargin)
            %preprocess Preprocess input data.
            %
            % Use this method to preprocess the input data prior to calling
            % the predict or forward methods. This method can be used with
            % detector to create a datastore transform. The detect method
            % automatically preprocesses the input data when the
            % DetectionPreprocessing NVP is set to 'auto'.
            %
            % Ipreprocessed = preprocess(detector, I) resizes the input data
            % to the nearest detector.InputSize and scales the pixels to
            % between 0 and 1 when DetectionPreprocessing NVP of detect is
            % 'auto'. When DetectionPreprocessing is 'none', the output and
            % input are the same. The input detector is a
            % yolov3ObjectDetector object and I is an M-by-N-by-P image.
            %
            % data = preprocess(detector, data) resizes the input
            % training data to the nearest detector.InputSize and scales
            % the pixels to between 0 and 1, also updates the corresponding
            % bounding boxes to the resized image dimension. data
            % is a cell array containing image, bounding boxes, labels
            % within each row. bounding boxes is an M-by-4 array in
            % [x, y, width, height] format and are axis aligned. Image and
            % bounding boxes are resized to detector.InputSize by
            % preserving the width and height aspect ratio when
            % DetectionPreprocessing is 'auto'. When DetectionPreprocessing
            % is 'none', the output is the same as the input. detector is a
            % yolov3ObjectDetector object and data is a cell array.
            %
            % [...,info] = preprocess(detector, trainingData) optionally
            % returns the information related to resized image.
            % info is structure containing the following fields:
            %   PreprocessedImage   - Size of preprocessed image.
            %   ScaleX              - Scale factor used to resize the input
            %                         image in x direction.
            %   ScaleY              - Scale factor used to resize the input
            %                         image in y direction.
            %
            % Notes
            % -----
            % - When DetectionPreprocessing is 'auto', input I is
            %   resized to nearest detector.InputSize by preserving the
            %   width and height aspect ratio.
            %
            % Class Support
            % -------------
            % The input image I can be uint8, uint16, int16, double,
            % single, or logical, and it must be real and non-sparse.
            %
            % The output image Ipreprocessed is always single.
            %
            % Example: Preprocess input image.
            % --------------------------------
            % % Load pre-trained YOLO v3 detector.
            % detector = yolov3ObjectDetector('tiny-yolov3-coco');
            %
            % % Read test image.
            % I = imread('highway.png');
            %
            % % Preprocess input image.
            % Ipreprocess = preprocess(detector, I);
            %
            % % Display results.
            % montage({I, Ipreprocess});
            % title("Original Image (Left) vs. Preprocessed Image (Right)")
            %
            % See also yolov3ObjectDetector, imresize, rescale.

            if isempty(varargin)
                trainingImageSize = detector.InputSize;
                networkInputSize = detector.Network.Layers(1,1).InputSize;
                [varargout{1:nargout}] = iPreprocess(I, trainingImageSize, networkInputSize);
            else
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
                        params.DetectionPreprocessing,params.ROI,params.UseROI,...
                        params.ExecutionEnvironment, detector.InputSize, params.CastToGpuArray));
                    varargout{2} = {};
                else
                    [varargout{1:nargout}] = iPreprocessForDetect(I, ...
                        params.DetectionPreprocessing, params.ROI, params.UseROI,...
                        params.ExecutionEnvironment, detector.InputSize, params.CastToGpuArray);
                end
            end
        end

        %------------------------------------------------------------------
        % Predict output feature maps.
        %------------------------------------------------------------------
        function outputFeatures = predict(detector,dlX,varargin)
            % outputFeatures = predict(detector, dlX) predicts features of
            % the preprocessed image dlX. The outputFeatures is a N-by-8
            % or N-by-10 cell array, where N are the number of outputs in
            %detector.Network. Each cell of outputFeature contains
            % predictions from an output layer. detector is a
            % yolov3ObjectDetector object and dlX is a SSCB formatted dlarray.
            %
            % Class Support
            % -------------
            % The input image dlX should be SSCB formatted dlarray,
            % real and non-sparse.
            %
            % Example
            % -------
            % % Load pre-trained YOLO v3 detector.
            % detector = yolov3ObjectDetector('tiny-yolov3-coco');
            %
            % % Read test image and convert to dlarray.
            % I = imread('highway.png');
            % I = single(rescale(I));
            % dlX = dlarray(I, 'SSCB');
            %
            % outputFeatures = predict(detector, dlX);
            %
            % See also yolov3ObjectDetector, dlarray, dlnetwork.

            % Compute network input size required for post processing.
            predictParams = parsePredictInputs(detector,dlX,varargin);
            network = detector.Network;
            anchorBoxes = detector.AnchorBoxes;
            isRotatedBox = strcmp(detector.PredictedBoxType, "rotated");

            if (~isnumeric(dlX) && ~ iscell(dlX))

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
                    if size(dlX,4)>1
                        outputFeatures = iPredictMultiActivations(network, dlX, anchorBoxes, predictParams.Acceleration, isRotatedBox);
                    else
                        outputFeatures = iPredictActivations(network, dlX, anchorBoxes, predictParams.Acceleration, isRotatedBox);
                    end
                end
            end
        end

        %------------------------------------------------------------------
        % Compute forward activations.
        %------------------------------------------------------------------
        function varargout = forward(detector,dlX)
            % [features, act] = forward(detector, dlX) computes
            % features of the preprocessed image dlX. The features is
            % a N-by-8 or N-by-10 cell array, where N are the number
            % of outputs in detector.Network. Each row of the features
            % are the network activations converted from grid cell coordinates 
            % to box coordinates. The act is a N-by-8 or N-by-10 cell array, 
            % where N are the number of outputs in detector.Network.
            % Each row of act contains activations from an output layer. 
            % detector is a yolov3ObjectDetector object and 
            % dlX is a formatted dlarray.
            %
            % [..., state] = forward(detector, dlX) optionally return the
            % state information of the network. The state is used
            % to update the parameters of network in yolo object
            % detector.
            %
            % Class Support
            % -------------
            % The input image dlX should be formatted dlarray,
            % real and non-sparse.
            %
            % Example
            % -------
            % % Load pre-trained YOLO v3 detector.
            % detector = yolov3ObjectDetector('tiny-yolov3-coco');
            %
            % % Read test image and convert to dlarray.
            % I = imread('highway.png');
            % I = single(rescale(I));
            % dlX = dlarray(I, 'SSCB');
            %
            % outputFeatures = forward(detector, dlX);
            %
            % See also yolov3ObjectDetector, dlarray, dlnetwork.

            arguments
                detector
            end

            arguments (Repeating)
                dlX
            end


            if detector.UseHighLevelTrainer
                if isempty(detector.Backbone) && isempty(detector.Neck) && isempty(detector.Head)
                    % Compute the activations and state information from the network.
                    network = detector.Network;
                    [varargout{1:nargout}] = forward(network, dlX{:});

                else
                    % Compute the activations and state information from the sub-networks.
                    backboneFeatures = cell(size(detector.Backbone.OutputNames'));
                    pretrainedYOLOv3 = isequal(detector.DetectionNetworkSource,{'add_23','add_19','add_11'}) ||  ...
                        isequal(detector.DetectionNetworkSource,{'leaky_relu_7','leaky_relu_5'});

                    % Forward for Backbone
                    if ~detector.FreezeBackbone && ~detector.FreezeBackboneAndNeck
                        [backboneFeatures{:}, backboneState] = forward(detector.Backbone, dlX{:},Acceleration="none");
                    else
                        % Compute the activations from frozen Backbone
                        dlX = dlarray(extractdata(dlX{:}),dims(dlX{:}));
                        [backboneFeatures{:}] = predict(detector.Backbone, dlX ,Acceleration="none");
                        backboneState = [];
                    end
                    state = backboneState;

                    % Forward for Neck
                    if ~isempty(detector.Neck)
                        if isa(detector.Neck, 'dlnetwork')
                            neckFeatures = cell(size(detector.Neck.OutputNames'));
                            if pretrainedYOLOv3 && size(detector.DetectionNetworkSource,2)==3
                                neckInputFeatures = backboneFeatures([1,2,2,3,3]);
                            elseif pretrainedYOLOv3 && size(detector.DetectionNetworkSource,2)==2
                                neckInputFeatures = backboneFeatures([1,2,2]);
                            else
                                neckInputFeatures = backboneFeatures([1,2,2]);
                            end

                            if detector.FreezeBackboneAndNeck
                                % Compute the activations from frozen Neck
                                [neckFeatures{:}] = predict(detector.Neck, neckInputFeatures{:} ,Acceleration="none");
                                neckState = [];
                            else
                                [neckFeatures{:}, neckState] = forward(detector.Neck, neckInputFeatures{:},Acceleration="none");
                            end
                            state = vertcat(state,neckState);
                        elseif isa(detector.Neck, 'cell')
                            neckFeatures= cell(size(cellfun(@(h){h.OutputNames{:}},detector.Neck,'UniformOutput',true)'));
                            for idx=1:size(detector.Neck,2)
                                neckInputFeatures = backboneFeatures([idx,idx+1,idx+1]);

                                if detector.FreezeBackboneAndNeck
                                    % Compute the activations from frozen Neck
                                    [neckFeatures{idx}] = predict(detector.Neck{idx}, neckInputFeatures{:} ,Acceleration="none");
                                    neckState = [];
                                else
                                    [neckFeatures{idx}, neckState] = forward(detector.Neck{idx}, neckInputFeatures{:},Acceleration="none");
                                end
                                state = vertcat(state,neckState);
                            end
                        end

                    end

                    % Forward for head
                    headFeatures= cell(size(cellfun(@(h){h.OutputNames{:}},detector.Head,'UniformOutput',true)'));
                    headState = cell(size(headFeatures));
                    for i=1:detector.numHeads
                        if pretrainedYOLOv3
                            [headFeatures{i},headState{i}] = forward(detector.Head{i}, neckFeatures{i},Acceleration="none");
                        else
                            if i==1
                                % no neck layer for first head
                                [headFeatures{i},headState{i}] = forward(detector.Head{i}, backboneFeatures{1},Acceleration="none");
                            else
                                [headFeatures{i},headState{i}] = forward(detector.Head{i}, neckFeatures{i-1},Acceleration="none");
                            end
                        end
                    end
                    state = vertcat(state,headState{:});

                    if nargout==4
                        [varargout{1:nargout-1}]=headFeatures{:};
                        varargout{nargout}=state;
                    else
                        varargout=headFeatures;
                        varargout=[varargout(:)',{state}];
                    end
                end
            else

                % Compute the activations and state information from the network.
                network = detector.Network;
                act = cell(size(network.OutputNames'));
                [act{:},state] = forward(network, dlX{:});

                anchorBoxes = detector.AnchorBoxes;
                isRotatedBox = strcmp(detector.PredictedBoxType, "rotated");
                act = yolov3ObjectDetector.yolov3Transform(act,isRotatedBox,anchorBoxes);

                % Gather the activations in the CPU for post processing and extract dlarray data.
                features = cellfun(@ gather, act,'UniformOutput',false);
                features = cellfun(@ extractdata, features, 'UniformOutput', false);

                % Convert predictions from grid cell coordinates to box coordinates.
                inputImageSize = size(dlX,1:2);
                features(:,2:5) = yolov3ObjectDetector.anchorBoxGenerator(anchorBoxes,features(:,2:5),inputImageSize);

                for ind=1:nargout
                    if ind==1
                        varargout{ind}=features;
                    elseif ind==2
                        varargout{ind}=act;
                    elseif ind==3
                        varargout{ind}=state;
                    end
                end

            end
        end


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
            % degrees. detector is a yolov3ObjectDetector object
            % and I is a truecolor or grayscale image.
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
            % image, I are returned in bounding boxes,either an M-by-4
            % matrix for axis-aligned bounding boxes or an M-by-5 matrix
            % for rotated rectangle bounding boxes. For axis-aligned bboxes,
            % each row of boxes contains a four-element vector,
            % [x, y, width, height]. This vector specifies the upper-left
            % corner and size of a bounding box in pixels. For rotated rectangle
            % bboxes, each row of boxes contains a five-element
            % vector, [xctr, yctr, width, height, yaw]. This vector
            % specifies the center coordinate, size, and angle of rotation,
            % in degrees, of a bounding box in spatial coordinates. yaw is
            % an angle between +/-180 degrees and increases positively in
            % the clockwise direction. Once 180 degrees of rotation is
            % reached, the angle of rotation will wrap to -180 degrees.
            % yolo is a yolov3ObjectDetector object.
            %
            % [...] = detect(..., roi) optionally detects objects within
            % the rectangular search region specified by roi. roi must be a
            % 4-element vector, [x, y, width, height], that defines a
            % rectangular region of interest fully contained in I. When
            % InputPreprocessing is set to 'none', roi cannot be
            % specified. When the input is a datastore, the same roi is
            % applied to every image.
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
            %                          YOLO v3 detector. Valid values are:
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
            % 'DetectionPreprocessing' Specify whether or not the detect method
            %                          automatically preprocesses input images.
            %                          Valid values are 'auto' and 'none':
            %
            %                           'auto'      Resize the input data to the
            %                                       nearest InputSize by preserving the
            %                                       aspect ratio and rescales image
            %                                       pixels between 0 and 1.
            %
            %                           'none'      Input data is not
            %                                       preprocessed. If you choose
            %                                       this option, the datatype
            %                                       of the test image must be
            %                                       either single or double.
            %
            %                         Default: 'auto'
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
            %  - Input image is preprocessed by default when DetectionPreprocessing is 'auto'.
            %
            % Class Support
            % -------------
            % The input image I can be uint8, uint16, int16, double,
            % single, and it must be real and non-sparse.
            %
            % When DetectionPreprocessing is 'none', the input image I
            % can be single or double, and it must be real and non-sparse.
            %
            % Example
            % -------
            % % Load pre-trained detector.
            % detector = yolov3ObjectDetector('tiny-yolov3-coco');
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
            % See also yolov3ObjectDetector,
            %          yolov3ObjectDetector/preprocess, yolov4ObjectDetector.

            params = parseDetectInputs(detector,I,varargin{:});
            [varargout{1:nargout}] = performDetect(detector, I, params);
        end
    end

    methods(Hidden)
        %------------------------------------------------------------------
        % Parse preprocess input parameters.
        %------------------------------------------------------------------
        function params = parsePreprocessInputs(~, I, varargin)
            % Preprocessing method during detect.
            params.DetectionPreprocessing = varargin{1,1}{1,1}.DetectionPreprocessing;
            if (~(strcmp(params.DetectionPreprocessing, 'auto')) && varargin{1,1}{1,1}.UseROI)
                error(message('vision:yolo:roiNotSupported'))
            end
            params.UseROI = varargin{1,1}{1,1}.UseROI;
            params.ROI = varargin{1,1}{1,1}.ROI;
            params.ExecutionEnvironment = varargin{1,1}{1,1}.ExecutionEnvironment;
            params.DetectionInputIsDatastore = ~isnumeric(I) && ~iscell(I);
            params.CastToGpuArray = varargin{1,1}{1,1}.CastToGpuArray; 
        end

        %------------------------------------------------------------------
        % Parse predict input parameters.
        %------------------------------------------------------------------
        function params = parsePredictInputs(~,dlX,varargin)
            if (isempty(varargin) || isempty(varargin{1,1}))
                params.MiniBatchSize = 1;
                params.NetworkInputSize = [];
                params.DetectionInputWasBatchOfImages = iscell(dlX);
                params.Acceleration = 'auto';
            else
                params = varargin{1,1}{1,1};
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

            validateChannelSize = true;  % check if the channel size is equal to that of the network
            validateImageSize   = false; % yolov3 can support images smaller than input size
            [sz,params.DetectionInputWasBatchOfImages] = vision.internal.cnn.validation.checkDetectionInputImage(...
                networkInputSize,sampleImage,validateChannelSize,validateImageSize);

            defaults = iGetDefaultYOLOv3DetectionParams();

            p = inputParser;
            p.addOptional('roi', defaults.roi);
            p.addParameter('SelectStrongest', defaults.SelectStrongest);
            p.addParameter('MinSize', defaults.MinSize);
            p.addParameter('MaxSize', sz(1:2));
            p.addParameter('MiniBatchSize', defaults.MiniBatchSize);
            p.addParameter('Threshold', defaults.Threshold);
            p.addParameter('DetectionPreprocessing', defaults.DetectionPreprocessing);
            p.addParameter('ExecutionEnvironment', defaults.ExecutionEnvironment);
            p.addParameter('Acceleration', defaults.Acceleration);
            parse(p, varargin{:});

            userInput = p.Results;

            vision.internal.cnn.validation.checkMiniBatchSize(userInput.MiniBatchSize, mfilename);

            vision.internal.inputValidation.validateLogical(...
                userInput.SelectStrongest, 'SelectStrongest');

            supportedInputs = ["auto", "none"];
            validatestring(userInput.DetectionPreprocessing, supportedInputs, mfilename, 'DetectionPreprocessing');

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
            yolov3ObjectDetector.checkThreshold(userInput.Threshold, mfilename);

            params.ROI                      = single(userInput.roi);
            params.UseROI                   = useROI;
            params.SelectStrongest          = logical(userInput.SelectStrongest);
            params.MinSize                  = single(userInput.MinSize);
            params.MaxSize                  = single(userInput.MaxSize);
            params.MiniBatchSize            = double(userInput.MiniBatchSize);
            params.Threshold                = single(userInput.Threshold);
            params.NetworkInputSize         = double(networkInputSize);
            params.FilterBboxesFunctor      = detector.FilterBboxesFunctor;
            params.DetectionPreprocessing   = userInput.DetectionPreprocessing;
            params.ExecutionEnvironment     = string(exeEnv);
            params.Acceleration             = string(accel);
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
        function this = setLayerIndices(this, network)
            this.LayerIndices.InputLayerIdx = yolov3ObjectDetector.findYOLOv3ImageInputLayer(network.Layers);
        end
    end

    methods(Static, Hidden, Access = protected)
        %------------------------------------------------------------------
        % Parse and validate pretrained detector parameters.
        %------------------------------------------------------------------
        function params = parsePretrainedDetectorInputs(varargin)
            % Parse inputs for this syntax:
            % detector = yolov3ObjectDetector(detectorName).

            p = inputParser;
            if size(varargin,2) == 1
                p.addRequired('detectorName');

                supportedNetworks = ["darknet53-coco", "tiny-yolov3-coco"];
                detectorName = validatestring(varargin{1,1}, supportedNetworks, mfilename, 'detectorName', 1);

                if strcmp(detectorName,'darknet53-coco')
                    inpSz = [608,608,3];
                else
                    inpSz = [416,416,3];
                end
                p.addParameter('InputSize', inpSz);
                p.addParameter('DetectionNetworkSource',{});
                p.addParameter('ModelName', '', @iAssertValidLayerName);
                p.addParameter('PredictedBoxType', "axis-aligned", @iValidatePredictedBoxType);

                parse(p, varargin{:});

            elseif (size(varargin,2) == 3) &&...
                    ((isa(varargin{1,2},'string') && isscalar(varargin{1,2})) || (isa(varargin{1,2},'char') && isrow(varargin{1,2}))) ...
                    && strcmp(varargin{1,2},'ModelName')
                % Parse inputs for this syntax:
                % detector = yolov3ObjectDetector(detectorName,'ModelName',modelName).

                p.addRequired('detectorName');
                supportedNetworks = ["darknet53-coco", "tiny-yolov3-coco"];
                detectorName = validatestring(varargin{1,1}, supportedNetworks, mfilename, 'detectorName', 1);

                if strcmp(detectorName,'darknet53-coco')
                    inpSz = [608,608,3];
                else
                    inpSz = [416,416,3];
                end
                p.addParameter('InputSize', inpSz);
                p.addParameter('DetectionNetworkSource',{});
                p.addParameter('ModelName', '', @iAssertValidLayerName);
                p.addParameter('PredictedBoxType', "axis-aligned", @iValidatePredictedBoxType);

                parse(p, varargin{:});

                params.isCustomModelName = true;

            else
                % Parse inputs for this syntax:
                % detector = yolov3ObjectDetector(detectorName,classNames,anchorBoxes).
                if (size(varargin,2) >= 3) &&...
                        ((isa(varargin{1,2},'string') && isscalar(varargin{1,2})) || (isa(varargin{1,2},'char') && isrow(varargin{1,2})))
                    if (strcmp(varargin{1,2},'InputSize') || strcmp(varargin{1,2},'DetectionNetworkSource') || strcmp(varargin{1,2},'ModelName'))
                        error(message('vision:yolo:requireClassesAndAnchors'));
                    end
                end

                p.addRequired('detectorName');
                p.addRequired('classNames');
                p.addRequired('anchorBoxes');

                supportedNetworks = ["darknet53-coco", "tiny-yolov3-coco"];
                detectorName = validatestring(varargin{1,1}, supportedNetworks, mfilename, 'detectorName', 1);

                % Default input size.
                if strcmp(detectorName,'darknet53-coco')
                    inpSz = [608,608,3];
                else
                    inpSz = [416,416,3];
                end

                p.addParameter('InputSize', inpSz);
                p.addParameter('DetectionNetworkSource',{});
                p.addParameter('ModelName', '', @iAssertValidLayerName);
                p.addParameter('PredictedBoxType', "axis-aligned", @iValidatePredictedBoxType);

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
            params.InputSize = p.Results.InputSize;
            params.DetectionNetworkSource = p.Results.DetectionNetworkSource;
            params.ModelName = char(p.Results.ModelName);
            params.DetectorName = char(p.Results.detectorName);
            params.PredictedBoxType = char(p.Results.PredictedBoxType);

            params.InputSize = iAddChannelIfRequired(params.InputSize);

            if strcmp(params.ModelName,'') && isempty(params.DetectionNetworkSource)
                params.ModelName = p.Results.detectorName;
            end
        end

        %------------------------------------------------------------------
        % Parse and validate detector parameters.
        %------------------------------------------------------------------
        function params = parseDetectorInputs(varargin)
            % Parse inputs for this syntax:
            % detector = yolov3ObjectDetector(network,classNames,anchorBoxes)

            % Validate the input network.
            imageInputIdx = iValidateNetwork(varargin{1,1},mfilename);

            p = inputParser;
            p.addRequired('network');
            p.addRequired('classNames');
            p.addRequired('anchorBoxes');

            p.addParameter('InputSize', []);
            p.addParameter('DetectionNetworkSource',{});
            p.addParameter('ModelName', '', @iAssertValidLayerName);
            p.addParameter('PredictedBoxType', "axis-aligned", @iValidatePredictedBoxType);
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

            params.InputSize = iAddChannelIfRequired(params.InputSize);

            if ~iscolumn(params.ClassNames)
                params.ClassNames = params.ClassNames';
            end

            if isstring(params.ClassNames) || iscategorical(params.ClassNames)
                params.ClassNames = cellstr(params.ClassNames);
            end

            numClasses = size(params.ClassNames, 1);

            % Configure network for transfer Learning.
            if ~isempty(params.DetectionNetworkSource)

                % validate DetectionNetworkSources
                validateattributes(params.DetectionNetworkSource, {'cell','string'}, {'row', 'size', [1 NaN]}, ...
                    mfilename, 'DetectionNetworkSource');
                % Update image input layer and first convolution layer in the
                % network.
                lgraph = iUpdateFirstConvChannelsAndInputLayer(net,params.InputSize);
                % Sorted DetectionNetworkSource returned
                [lgraph, params.DetectionNetworkSource] = iConfigureDetector(lgraph,numClasses,params.AnchorBoxes,params.DetectionNetworkSource,params.PredictedBoxType);

                % Verify Fully Connected layer and Global average pooling layer does not exist in the network.
                net = dlnetwork(lgraph);
                iVerifyFullyConnectedExistence(net);
                iVerifyGlobalAvgPoolExistence(net);
                params.Network = net;
            else
                iValidateYOLOv3Network(net, numClasses, params.AnchorBoxes, params.PredictedBoxType);
                % If the input is a yolov3 dlnet and the inputSize is not same
                % as network's inputSize, then no configuring is needed.
                if(~isequal(params.InputSize, net.Layers(imageInputIdx).InputSize))
                    % Update image input layer and first convolution layer in the
                    % network.
                    lgraph = iUpdateFirstConvChannelsAndInputLayer(net,params.InputSize);
                    net = dlnetwork(lgraph);
                end
                params.Network = net;
            end
        end

        %------------------------------------------------------------------
        function imageInputIdx = findYOLOv3ImageInputLayer(externalLayers)
            imageInputIdx = find(...
                arrayfun( @(x)isa(x,'nnet.cnn.layer.ImageInputLayer'), ...
                externalLayers));
        end

    end

    %======================================================================
    % Save/Load
    %======================================================================
    methods(Hidden)
        function s = saveobj(this)

            % Update Network if required
            if ~isempty(this.Backbone) && ~isempty(this.Neck) && ~isempty(this.Head) || ...
                    ~isempty(this.Backbone) && isempty(this.Neck) && ~isempty(this.Head)
                this = updateDetector(this);
            end

            s.Version                       = 3.1;
            s.ModelName                     = this.ModelName;
            s.Network                       = this.Network;
            s.DetectionNetworkSource        = this.DetectionNetworkSource;
            s.ClassNames                    = this.ClassNames;
            s.AnchorBoxes                   = this.AnchorBoxes;
            s.InputSize                     = this.InputSize;
            s.FreezeBackboneAndNeckInternal = this.FreezeBackboneAndNeckInternal;
            s.PredictedBoxType              = this.PredictedBoxType;
            s.UseHighLevelTrainer           = this.UseHighLevelTrainer;
        end

        function net = matlabCodegenPrivateNetwork(this)
            net = this.Network;
        end
    end


    methods(Static, Hidden)
        function this = loadobj(s)
            try
                vision.internal.requiresNeuralToolbox(mfilename);
                network            = s.Network;
                classes            = s.ClassNames;
                anchorBoxes        = s.AnchorBoxes;

                if s.Version == 1.0 
                    inputSize = iUpdateInputSize(network);
                    predictedBoxType = "axis-aligned";
                elseif s.Version >= 3.0 % >=24a
                    inputSize          = s.InputSize;
                    predictedBoxType = s.PredictedBoxType;
                else
                    inputSize          = s.InputSize;
                    predictedBoxType = "axis-aligned";
                end

                this = yolov3ObjectDetector(network,classes,anchorBoxes,...
                    'InputSize',inputSize, ...
                    'PredictedBoxType', predictedBoxType);
                this.ModelName     = s.ModelName;

                if s.Version >= 3.0 % >=24a
                    this.FreezeBackboneAndNeckInternal = s.FreezeBackboneAndNeckInternal;
                    this.DetectionNetworkSource = s.DetectionNetworkSource;
                end

                if s.Version >= 3.1 % >=24a
                    this.UseHighLevelTrainer = s.UseHighLevelTrainer;
                end

            catch ME
                rethrow(ME)
            end
        end

        function detector = assembleDetector(params, net)
            detector = yolov3ObjectDetector(net, params.ClassNames, params.AnchorBoxes,...
                'InputSize', params.InputSize,...
                'ModelName', params.ModelName, ...
                'PredictedBoxType', params.PredictedBoxType);

            % setting DetectionNetworkSource and FreezeBackboneAndNeckInternal property
            % for assembled detector to support freezing with assembled detector
            detector.DetectionNetworkSource = params.DetectionNetworkSource;
            detector.FreezeBackboneAndNeckInternal = params.FreezeBackboneAndNeckInternal;

            % preserve UseHighLevelTrainer property in assembled detector 
            detector.UseHighLevelTrainer = params.UseHighLevelTrainer;
        end

        function n = matlabCodegenDlRedirect(~)
            n = 'vision.internal.codegen.YOLOv3ObjectDetector';
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


    methods(Static, Hidden, Access = public)
        %------------------------------------------------------------------
        function data = preprocessInput(data, targetSize)
            % Resize image and boxes, then normalize image data between 0 and 1.
            if ~iscell(data)
                imgData = {data};
            else
                imgData = data;
            end

            for idx = 1:size(imgData,1)
                I = imgData{idx,1};
                I = single(rescale(I));
                if iscell(data)
                    [I,bboxes] = vision.internal.cnn.yolo.LetterBoxImage(I,targetSize(1:2),imgData{idx,2});
                    data(idx,1:2) = {I, bboxes};
                else
                    dataTmp = [];
                    for i = 1:size(I,4)
                        Itmp = I(:,:,:,i);
                        [Itmp,~] = vision.internal.cnn.yolo.LetterBoxImage(Itmp,targetSize(1:2));
                        if isempty(dataTmp)
                            dataTmp = Itmp;
                        else
                            dataTmp = cat(4,dataTmp,Itmp);
                        end
                    end
                    data = dataTmp;
                end
            end
        end
        %-------------------------------------------------------------------

        function predictions = yolov3Transform(YPredictions, isRotatedBox, anchorBoxes)
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


        %--------------------------------------------------------------------------
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

%--------------------------------------------------------------------------
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
function [lgraph, orderedDetectionNetSource]= iRemoveLayers(lgraph,detectionNetSource)
% Remove all the layers after detectionNetworkSource.
dg = vision.internal.cnn.RCNNLayers.digraph(lgraph);

% Find the feature extraction nodes to remove nodes after feature
% extraction layer.

detectionNetSourceIds = [];
for i = 1:size(detectionNetSource,2)
    % Verify that all detectionNetworkSource layers exist in lgraph.
    iVerifyLayersExist(lgraph, detectionNetSource);
    nodeId = findnode(dg,char(detectionNetSource{1,i}));
    detectionNetSourceIds(i) = nodeId;
end

% Name of last feature extraction layer
[id,ind]= max(detectionNetSourceIds);
layerName = detectionNetSource{ind};

% Sort detectionNetSource
[~, sortingIndices] = sort(detectionNetSourceIds, 'descend');
orderedDetectionNetSource = detectionNetSource(sortingIndices);

% Remove layers after last feature extraction layer
% Remove any lingering branches to removed layers in lgraph
allLayers = {lgraph.Layers.Name};
if ~(sum(id)==0)
    dg = flipedge(dg);
    layersBefore = dfsearch(dg,layerName);
    % Remove lingering branches along with layer after layerNames
    lgraph = removeLayers(lgraph, allLayers(~(ismember(allLayers, layersBefore))));
end

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
        error(message('vision:ssd:InvalidLayerName', layerNames{idx}));
    end
end
end

%--------------------------------------------------------------------------
function newSize = iAddChannelIfRequired(inputSize)
validateattributes(inputSize, {'numeric'}, ...
    {'2d','nonempty','nonsparse',...
    'real','finite','integer','positive','nrows',1});

if (size(inputSize,2)==2)
    newSize = [inputSize,1];
else
    newSize = inputSize;
end

if ~(size(newSize,2)==3)
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
if isempty(value)
    error(message('vision:yolo:invalidClasses'));
end
end

%--------------------------------------------------------------------------
function iAssertValidLayerName(name)
nnet.internal.cnn.layer.paramvalidation.validateLayerName(name);
end

%--------------------------------------------------------------------------
function iValidatePredictedBoxType(predictedBoxType)
validatestring(predictedBoxType,{'axis-aligned', 'rotated'},...
    'yolov3ObjectDetector', 'PredictedBoxType');
end

%--------------------------------------------------------------------------
function iValidateAnchorBoxes(value)
validateattributes(value, {'cell'},{'column','size',[NaN NaN]}, ...
    mfilename, 'AnchorBoxes');

for i = 1:size(value,1)
    validateattributes(value{i,1}, {'numeric'}, {'size', [NaN 2], 'real',...
        'nonnan', 'finite','positive'}, mfilename, 'AnchorBoxes');
end

end

%--------------------------------------------------------------------------
function outputFeatures = iPredictMultiActivations(network,dlX, anchorBoxes, acceleration,isRotatedBox)
numMiniBatch = size(dlX,4);
outputFeatures = cell(numMiniBatch,1);
for ii = 1:numMiniBatch
    inp = dlX(:,:,:,ii);
    outputFeatures{ii,1} = iPredictActivations(network, inp, anchorBoxes, acceleration,isRotatedBox);
end
end

%--------------------------------------------------------------------------
function outputFeatures = iPredictBatchActivations(network,dlX, anchorBoxes, acceleration,isRotatedBox)
numMiniBatch = size(dlX,2);
outputFeatures = cell(numMiniBatch,1);
for ii = 1:numMiniBatch
    inp = dlX{ii};
    outputFeatures{ii,1} = iPredictActivations(network, inp, anchorBoxes, acceleration,isRotatedBox);
end
end

%--------------------------------------------------------------------------
function outputFeatures = iPredictActivations(network, dlX, anchorBoxes, acceleration,isRotatedBox)
% Compute predictions.
features = cell(size(network.OutputNames'));
[features{:}] = predict(network, dlX, 'Acceleration', acceleration);

outputFeatures = yolov3ObjectDetector.yolov3Transform(features,isRotatedBox, anchorBoxes);
end

%--------------------------------------------------------------------------
function [bboxes,scores,labels,intermediates] = iPostprocessMultiDetection(detector,YPredData,info,params)
numMiniBatch = size(YPredData,1);
bboxes = cell(numMiniBatch, 1);
scores = cell(numMiniBatch, 1);
labels = cell(numMiniBatch, 1);
intermediates = cell(numMiniBatch, 1);
for ii = 1:numMiniBatch
    [bboxes{ii},scores{ii},labels{ii},intermediates{ii}] = ...
        iPostprocessSingleDetection(detector,YPredData{ii,1},info,params);
end
end

%--------------------------------------------------------------------------
function [bboxes,scores,labels,intermediates] = iPostprocessSingleDetection(detector,YPredData,info,params)
% Obtain the classnames detector is trained on.
classes = detector.ClassNames;

isRotatedBox = strcmp(detector.PredictedBoxType, "rotated");
rotationOffset = 2*double(isRotatedBox);

extractDetections = cellfun(@ extractdata, YPredData, 'UniformOutput', false);
extractDetections(:,2:5) = yolov3ObjectDetector.anchorBoxGenerator(detector.AnchorBoxes, extractDetections(:,2:5), params.NetworkInputSize);

% Apply following post processing steps to filter the detections:
% * Filter detections based on threshold.
% * Convert bboxes from spatial to pixel dimension.

% Combine the prediction from different heads.
detections(:,1:5+rotationOffset) = cellfun(@ reshapePredictions,extractDetections(:,1:5+rotationOffset), 'UniformOutput', false);
detections(:,6+rotationOffset) = cellfun(@(a,b) reshapeClasses(a,b),extractDetections(:,6+rotationOffset),repmat({numel(classes)}, ...
    size(extractDetections(:,6+rotationOffset))),'UniformOutput', false);
detections = iCombinePredictions(detections);

% Get the raw class probabilities
classProbabilities = detections(:,6+rotationOffset:end);

% Filter the classes based on (confidence score * class probability).
[maxClassProb, maxClassIdx] = max(detections(:,6+rotationOffset:end),[],2);
detections(:,1) = detections(:,1).*maxClassProb;
detections(:,6+rotationOffset) = maxClassIdx;

% Empty all lingering detection information and append with class
% probabilities
detections(:,7+rotationOffset:end) = [];
detections = [detections,classProbabilities];

% Keep detections whose objectness score is greater than thresh.
detections = detections(detections(:,1)>=params.Threshold,:);

[bboxes,scores,labels,intermediates] = iPostProcessDetections(detections,classes,info,params,isRotatedBox);
end

%--------------------------------------------------------------------------
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

    if (strcmp(params.DetectionPreprocessing, 'auto'))
        % Resolve g3131823

        % Obtain boxes for preprocesssed image.
        processedImageSize(2) = info.PreprocessedImageSize(2);
        processedImageSize(1) = info.PreprocessedImageSize(1);

        scale = [processedImageSize(2) processedImageSize(1) processedImageSize(2) processedImageSize(1)];
        bboxesTmp(:,1:4) = bboxesTmp(:,1:4).*scale;

        if ~isRotatedBox
            % Convert x and y position of detections from centre to top-left.
            % Resize boxes to image size.
            bboxesTmp = iConvertCenterToTopLeft(bboxesTmp);
        else
            phases = tanh(bboxesTmp(:,5:6));
            yaw = rad2deg(atan2(phases(:,1),phases(:,2)));
            bboxesTmp = [bboxesTmp(:,1:4) yaw];
        end

        % Resize boxes to original image size.
        inputImageSize(2) = info.ScaleX.*info.PreprocessedImageSize(2);
        inputImageSize(1) = info.ScaleY.*info.PreprocessedImageSize(1);

        [shiftedBboxes,shiftedImSz] = vision.internal.cnn.yolo.DeLetterBoxImage(bboxesTmp,info.PreprocessedImageSize,inputImageSize);
         
        bboxPred = iScaleBboxes(shiftedBboxes,inputImageSize,shiftedImSz);

    else
        inputImageSize(2) = info.ScaleX.*info.PreprocessedImageSize(2);
        inputImageSize(1) = info.ScaleY.*info.PreprocessedImageSize(1);

        scale = [inputImageSize(2) inputImageSize(1) inputImageSize(2) inputImageSize(1)];
        bboxesTmp(:,1:4) = bboxesTmp(:,1:4).*scale;

        if ~isRotatedBox
            % Convert x and y position of detections from centre to top-left.
            % Resize boxes to image size.
            bboxPred = iConvertCenterToTopLeft(bboxesTmp);
        else
            phases = tanh(bboxesTmp(:,5:6));
            yaw = rad2deg(atan2(phases(:,1),phases(:,2)));
            bboxPred = [bboxesTmp(:,1:4) yaw];
        end

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
    else
        bboxes = iClipRotatedBBox(bboxes,inputImageSize(1,2),inputImageSize(1,1));
    end

    % Adjust any x, y, w, h to 1 if less than 1. This ignores the angle
    % element of rotated rectangles as the allowed range is (-180,180].
    bboxes(bboxes(:,1:4)<1) = 1;

    % Convert classId to classNames.
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
function iValidateYOLOv3Network(network, numClasses, anchorBoxes, predictedBoxType)
numOutputLayers = size(network.OutputNames,2);
layerNames = string({network.Layers.Name});

% Verfiy Fully Connected layer does not exist in the network.
iVerifyFullyConnectedExistence(network);

% Verfiy Global average pooling layer does not exist in the network.
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
bboxes = floor(bboxes);
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

lgraph = layerGraph(dlnet);

imgIdx = arrayfun(@(x)isa(x,'nnet.cnn.layer.ImageInputLayer'),...
    dlnet.Layers);
imageInputIdx = find(imgIdx,1,'first');

imageInput = imageInputLayer(imageSize,...
    'Name',lgraph.Layers(imageInputIdx).Name,'Normalization','none');

lgraph = replaceLayer(lgraph,lgraph.Layers(imageInputIdx).Name,...
    imageInput);

numChannel = imageSize(3);

idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.Convolution2DLayer'),...
    dlnet.Layers);
convIdx = find(idx,1,'first');
if ~isempty(convIdx)
    numFirstConvLayerChannels = dlnet.Layers(convIdx,1).NumChannels;
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

%--------------------------------------------------------------------------
function [lgraph,orderedDetectionNetSource] = iConfigureDetector(lgraph,numClasses,anchorBoxes,detectionNetworkSource,predictedBoxType)
% Create a layerGraph for transfer learning.
[lgraph,orderedDetectionNetSource] = iRemoveLayers(lgraph,detectionNetworkSource);

% Set rotationOffset for rotated rectangle bounding boxes
if strcmp(predictedBoxType,"rotated")
    rotationOffset = 2;
else
    rotationOffset = 0;
end

% Feature extraction layers in detectionNetworkSource must be specified in the reverse of the order
% in which layers appear in the network architecture
detectionNetworkSource = orderedDetectionNetSource;

numOutputLayers = size(detectionNetworkSource,2);

numAnchorBoxGroups = size(anchorBoxes,1);
if numOutputLayers ~= numAnchorBoxGroups
    error(message('vision:yolo:numOutputLayerMismatch'));
end

analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(lgraph);

% Add detection heads to feature extraction layers.
numPredictorsPerAnchor = 5 + rotationOffset +  numClasses;
for idx = 1:numOutputLayers

    featureLayerIdx = arrayfun(@(x) x.Name == ...
        detectionNetworkSource{1,idx},analysis.LayerAnalyzers);

    % Verify that YOLO v3 DetectionNetworkSource output size is greater than [1,1].
    activationSize = analysis.LayerAnalyzers(featureLayerIdx).Outputs.Size{1,1};
    if (any(activationSize(1:2) < 2))
        error(message("vision:yolo:mustHaveValidFinalActivationsSize"));
    end

    outFilters = analysis.LayerAnalyzers(featureLayerIdx).Outputs.Size{1}(3);

    numAnchorsScale = size(anchorBoxes{idx,1}, 1);

    % Compute the number of filters for last convolution layer.
    numFilters = numAnchorsScale*numPredictorsPerAnchor;

    if (idx>1)
        firstConv = convolution2dLayer(3,outFilters,'Padding','same','Name',['customConv',num2str(idx)],'WeightsInitializer','he');
    else
        firstConv = convolution2dLayer(3,(outFilters*2),'Padding','same','Name',['customConv',num2str(idx)],'WeightsInitializer','he');
    end

    detectionSubNetwork = [firstConv;
        batchNormalizationLayer('Name',['customBatchNorm',num2str(idx)]);
        reluLayer('Name',['customRelu',num2str(idx)])
        convolution2dLayer(1,numFilters,'Padding','same','Name',['customOutputConv',num2str(idx)],'WeightsInitializer','he')
        ];

    % Add layers that concatenate the features.
    if (idx>1)
        featureGatherLayers = [
            convolution2dLayer(1,ceil(outFilters./2),'Padding','same','Name',['featureConv',num2str(idx)],'WeightsInitializer','he')
            batchNormalizationLayer('Name',['featureBatchNorm',num2str(idx)]);
            reluLayer('Name',['featureRelu',num2str(idx)])
            resize2dLayer('EnableReferenceInput',true, 'Name',['featureResize',num2str(idx)]);
            depthConcatenationLayer(2,'Name',['depthConcat',num2str(idx)]);
            ];
        detectionSubNetwork = [featureGatherLayers;detectionSubNetwork];
    end

    lgraph = addLayers(lgraph,detectionSubNetwork);

    % Connect appropriate layers.
    if (idx>1)
        lgraph = connectLayers(lgraph,detectionNetworkSource{1,idx-1},['featureConv',num2str(idx)]);
        lgraph = connectLayers(lgraph,detectionNetworkSource{1,idx},['featureResize',num2str(idx),'/ref']);
        layerToConnect = ['depthConcat',num2str(idx),'/in2'];
    else
        layerToConnect = ['customConv',num2str(idx)];
    end

    lgraph = connectLayers(lgraph, detectionNetworkSource{1,idx}, layerToConnect);
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
function [Ipreprocessed,info] = iPreprocessForDetect(I, detectionPreprocessing, roi, useROI, executionEnvironment, trainingImageSize, castToGpuArray)
% Crop image if requested.
Iroi = vision.internal.detector.cropImageIfRequested(I, roi, useROI);

% Find the scale factor to obtain the original image size.
[info.ScaleX,info.ScaleY] = iComputeScale(size(Iroi),trainingImageSize);
info.PreprocessedImageSize = trainingImageSize;

if (strcmp(detectionPreprocessing, 'auto'))
    Ipreprocessed = yolov3ObjectDetector.preprocessInput(Iroi, info.PreprocessedImageSize);

    % Convert to gpuArray based on executionEnvironment.
    if castToGpuArray
        if (strcmp(executionEnvironment,'auto') && canUseGPU) || strcmp(executionEnvironment,'gpu')
            Ipreprocessed = gpuArray(Ipreprocessed);
        end
    end
else
    if ~(isa(Iroi,'double') || isa(Iroi,'single') || isa(Iroi,'gpuArray'))
        error(message('vision:yolo:unSupportedInputClass'));
    end
    Ipreprocessed = Iroi;
end

if (isa(Ipreprocessed,'double')||isa(Ipreprocessed,'single')||isa(Ipreprocessed,'gpuArray'))
    Ipreprocessed = dlarray(Ipreprocessed,'SSCB');
end
end

%--------------------------------------------------------------------------
function [dataPreprocessed,infoPreprocessed] = iPreprocess(data, trainingImageSize, networkInputSize)
% Resize image and boxes, then normalize image data between 0 and 1.
if iscell(data)
    dataPreprocessed = data;
    for idx = 1:size(data,1)
        I = data{idx,1};
        iValidateInput(networkInputSize, I);
        % Find the scale factor to obtain the original image size.
        [info.ScaleX,info.ScaleY] = iComputeScale(size(data),trainingImageSize);
        info.PreprocessedImageSize = trainingImageSize;
        I = single(rescale(I));

        [I,bboxes] = vision.internal.cnn.yolo.LetterBoxImage(I,info.PreprocessedImageSize(1:2),data{idx,2});
        dataPreprocessed(idx,1:2) = {I, bboxes};
        infoPreprocessed(idx,1) = {info};
    end
else
    % Find the scale factor to obtain the original image size.
    [info.ScaleX,info.ScaleY] = iComputeScale(size(data),trainingImageSize);
    info.PreprocessedImageSize = trainingImageSize;
    dataTmp = [];
    for i = 1:size(data,4)
        Itmp = data(:,:,:,i);
        iValidateInput(networkInputSize, Itmp);
        [Itmp,~] = vision.internal.cnn.yolo.LetterBoxImage(Itmp,info.PreprocessedImageSize(1:2));
        if isempty(dataTmp)
            dataTmp = Itmp;
        else
            dataTmp = cat(4,dataTmp,Itmp);
        end
    end
    dataPreprocessed = dataTmp;
    infoPreprocessed = info;
end

end

%--------------------------------------------------------------------------
function iValidateInput(networkInputSize, sampleImage)
validateChannelSize = true;  % check if the channel size is equal to that of the network
validateImageSize   = false; % yolov3 can support images smaller than input size
[~, params.DetectionInputWasBatchOfImages] = vision.internal.cnn.validation.checkDetectionInputImage(...
    networkInputSize,sampleImage,validateChannelSize,validateImageSize);
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
function s = iGetDefaultYOLOv3DetectionParams()
s.roi                     = zeros(0,4);
s.SelectStrongest         = true;
s.Threshold               = 0.5;
s.MinSize                 = [1,1];
s.MaxSize                 = [];
s.MiniBatchSize           = 128;
s.DetectionPreprocessing  = 'auto';
s.ExecutionEnvironment    = 'auto';
s.Acceleration            = 'auto';
end

%--------------------------------------------------------------------------
function output = iValidateNetwork(network,mfilename)
% YOLOv3 network should be a dlnetwork object and must contain image input
% layer.

validateattributes(network,{'DAGNetwork','dlnetwork'},...
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
function bboxPred = iScaleBboxes(bboxes,imSz,newImSz)
scale   = imSz(1:2)./newImSz(1:2);
[info.ScaleX,info.ScaleY] = deal(scale(2),scale(1));
    if size(bboxes,2) == 4
        % scale axis aligned boxes 
        bboxesX1Y1X2Y2 = vision.internal.cnn.boxUtils.xywhToX1Y1X2Y2(bboxes);
        % Saturate X2,Y2 to the original image dimension, to remove the gray scale
        % scale detections if any.
        bboxesX1Y1X2Y2(:,3) = min(bboxesX1Y1X2Y2(:,3),newImSz(1,2));
        bboxesX1Y1X2Y2(:,4) = min(bboxesX1Y1X2Y2(:,4),newImSz(1,1));
        
        % Scale the boxes to the image dimension.
        bboxesX1Y1X2Y2 = vision.internal.cnn.boxUtils.scaleX1X2Y1Y2(bboxesX1Y1X2Y2, info.ScaleX, info.ScaleY);
        bboxPred = vision.internal.cnn.boxUtils.x1y1x2y2ToXYWH(bboxesX1Y1X2Y2);

    elseif size(bboxes,2) == 5
        % Scale rotated boxes 
        % Convert rotated bbox to vertices for scaling.
        bboxPointsTemp = bbox2points(bboxes);

        % Remap bboxPointsTemp to be an M-by-8 matrix, where M is the number of
        % bounding boxes and each row is specified as [x1 x2 x3 x4 y1 y2 y3 y4].
        bboxPoints = reshape(bboxPointsTemp,[8,size(bboxes,1)])';

        % Scale the boxes to the image dimension
        % Scale [x1 x2 x3 x4 y1 y2 y3 y4] box in the x-direction by info.ScaleX and 
        % in the y-direction by info.ScaleY.

        % Scale the vertices based on scaling factor input.
        bboxPoints(:,1:4) = bboxPoints(:,1:4).* info.ScaleX;
        bboxPoints(:,5:8) = bboxPoints(:,5:8).* info.ScaleY;
        
        % Convert scaled vertices back to bbox format.
        w = max(hypot(bboxPoints(:,2:3) - bboxPoints(:,[1 4]), ...
                   bboxPoints(:,6:7) - bboxPoints(:,[5 8])),[],2);
        h = max(hypot(bboxPoints(:,3:4) - bboxPoints(:,[2 1]), ...
                   bboxPoints(:,7:8) - bboxPoints(:,[6 5])),[],2);

        % Rescale the center point in pixel coordinate system
        xCtr = info.ScaleX * bboxes(:,1);
        yCtr = info.ScaleY * bboxes(:,2);

        bboxPred = [xCtr yCtr w h bboxes(:,5)];
    end
end

%--------------------------------------------------------------------------
function [sx, sy] = iComputeScale(sz,targetSize)
% Compute scale factors to scale boxes from targetSize back to the input
% size.
scale   = sz(1:2)./targetSize(1:2);
[sx,sy] = deal(scale(2),scale(1));
end

%--------------------------------------------------------------------------
function iVerifyFullyConnectedExistence(network)
% YOLOv3 network is based on Convolution Layers and should not
% contain any fullyConnected Layers.
idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.FullyConnectedLayer'),...
    network.Layers);
if sum(idx) ~= 0
    error(message("vision:yolo:mustNotHaveAnyFCLayer"));
end
end

%--------------------------------------------------------------------------
function inputSize = iUpdateInputSize(network)

imgIdx = arrayfun(@(x)isa(x,'nnet.cnn.layer.ImageInputLayer'),...
    network.Layers);
imageInputIdx = find(imgIdx,1,'first');

inputSize = network.Layers(imageInputIdx,1).InputSize;
end

%--------------------------------------------------------------------------
function iVerifyGlobalAvgPoolExistence(network)
% YOLOv3 network should not contain any global average pooling layer as it
% downsamples input feature map to size of [1,1].
idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.GlobalAveragePooling2DLayer')||isa(x,'nnet.cnn.layer.GlobalMaxPooling2DLayer'),...
    network.Layers);
if sum(idx) ~= 0
    error(message("vision:yolo:mustNotHaveAnyGlobalPoolingLayer"));
end
end

%--------------------------------------------------------------------------
function detector = iTripwireDefaultYOLOv3Model()
% Check if support package is installed
breadcrumbFile = 'vision.internal.cnn.supportpackages.IsYOLOv3Installed';
fullPath = which(breadcrumbFile);
if isempty(fullPath)
    iErrorSPKGInstallation;
else
    % Load pretrained network.
    data = iLoadDefaultPretrainedNetwork(fullPath);
    detector = data.detector;
end
end

%--------------------------------------------------------------------------

function detector = iTripwireYOLOv3Model(params)
% Check if support package is installed
breadcrumbFile = 'vision.internal.cnn.supportpackages.IsYOLOv3Installed';
fullPath = which(breadcrumbFile);
if isempty(fullPath)
    iErrorSPKGInstallation;
else
    if(isfield(params,'ClassNames'))
        data = iLoadPretrainedNetwork(fullPath, params.DetectorName);
        network = data.detector.Network;
        pretrainedDetector = data.detector;
        pretrainedDetectionNetworkSource = data.detector.DetectionNetworkSource;
        classes = params.ClassNames;
        inputSize = params.InputSize;
        anchorBoxes = params.AnchorBoxes;
        predictedBoxType = params.PredictedBoxType;


        if isempty(params.DetectionNetworkSource)
            % Validate anchorBoxes
            if strcmp(params.DetectorName, 'darknet53-coco')
                if (size(anchorBoxes,1) ~= 3)
                    error(message('vision:yolo:pretrainedAnchorsMismatch',params.ModelName,'3'));
                end
            else
                if (size(anchorBoxes,1) ~= 2)
                    error(message('vision:yolo:pretrainedAnchorsMismatch',params.ModelName,'2'));
                end
            end

            if strcmp(params.DetectorName, 'darknet53-coco')
                layersToReplace = {'conv2d_59','conv2d_67','conv2d_75'};
            else
                layersToReplace = {'conv2d_10','conv2d_13'};
            end

            % Create pre-trained detector with custom classes and
            % anchorBoxes
            network = iConfigureNetwork(network,anchorBoxes,classes,layersToReplace,inputSize,predictedBoxType);
            detector = yolov3ObjectDetector(network,classes,anchorBoxes,'ModelName',params.ModelName,...
                'InputSize',params.InputSize,'DetectionNetworkSource',params.DetectionNetworkSource, ...
                'PredictedBoxType',params.PredictedBoxType);
            detector.DetectionNetworkSource = pretrainedDetectionNetworkSource;

        else
            % validate anchor boxes 
            if (size(anchorBoxes,1) ~= size(params.DetectionNetworkSource,2))
                    error(message('vision:yolo:pretrainedBackboneCustomYOLOv3AnchorsMismatch', ...
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
                % Create custom YOLO v3 detector from pretrained backbone with custom classes and
                % anchorBoxes
                [pretrainedBackbone, ~,~] = iSubNetworks(pretrainedDetector,pretrainedDetectionNetworkSource); % Pretrained Backbone
                detector = yolov3ObjectDetector(pretrainedBackbone,classes,anchorBoxes,'ModelName',params.ModelName,...
                    'InputSize',params.InputSize,'DetectionNetworkSource',params.DetectionNetworkSource, ...
                    'PredictedBoxType',params.PredictedBoxType);
            else
                error(message('vision:yolo:invalidDetectionNetworkSourceForCustomYOLOV3withPretrainedBackbone'));
            end
        end
    elseif (isfield(params,'isCustomModelName'))
        % Load pretrained network and update model name.
        data = iLoadPretrainedNetwork(fullPath, params.DetectorName);
        detector = data.detector;
        detector.ModelName = params.ModelName;
    else
        % Load pretrained network.
        data = iLoadPretrainedNetwork(fullPath, params.DetectorName);
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
    numClasses = numel(classes);
    numFilters = (numClasses + 5 + rotationOffset)*numAnchors;
    convOut = convolution2dLayer([1,1],numFilters,'Padding','same','Name',['convOut',num2str(i)]);
    lgraph = replaceLayer(lgraph,layersToReplace{1,i},convOut);
end
network = dlnetwork(lgraph);
end

%--------------------------------------------------------------------------
function iErrorSPKGInstallation()
name     = 'Computer Vision Toolbox Model for YOLO v3 Object Detection';
basecode = 'YOLOV3';

throwAsCaller(MException(message('nnet_cnn:supportpackages:InstallRequired', mfilename, name, basecode)));
end

%--------------------------------------------------------------------------
function data = iLoadDefaultPretrainedNetwork(fullPath)
pattern = fullfile(filesep, '+vision','+internal','+cnn','+supportpackages','IsYOLOv3Installed.m');
idx     = strfind(fullPath, pattern);
matfile = fullfile(fullPath(1:idx), 'data','LargeData','yolov3COCO.mat');
data = load(matfile);
end

%--------------------------------------------------------------------------
function data = iLoadPretrainedNetwork(fullPath, detectorName)
pattern = fullfile(filesep, '+vision','+internal','+cnn','+supportpackages','IsYOLOv3Installed.m');
idx     = strfind(fullPath, pattern);

if strcmp(detectorName, 'darknet53-coco')
    matfile = fullfile(fullPath(1:idx), 'data', 'LargeData', 'yolov3COCO.mat');
else
    matfile = fullfile(fullPath(1:idx), 'data', 'LargeData', 'tinyYOLOv3COCO.mat');
end
data = load(matfile);
end

% -------------------------------------------------------------------------
% iDlNetwork : Merges sub-networks to form Network
% -------------------------------------------------------------------------
function dlnetOut = iDlNetwork(detector)
% Reconstruct a monolithic dlnetwork from the backbone, neck
% and head
dlnetBackbone = detector.Backbone;
dlnetHead = detector.Head;
dlnetOut = dlnetBackbone;
numHeads = size(detector.Head,2);

if ~isempty(detector.Neck)
    for i=1:size(detector.Neck,2) % repeat for each neck dlnetwork

        if isa(detector.Neck, 'dlnetwork')
            dlnetNeck = detector.Neck;
        else
            dlnetNeck = detector.Neck{i};
        end

        dlnetOut = addLayers(dlnetOut,dlnetNeck.Layers);

        % Disconnect all neck layers
        for idx = 1:numel(dlnetNeck.Layers)-1
            srcName = dlnetNeck.Layers(idx).Name;
            destName = dlnetNeck.Layers(idx+1).Name;

            if ~isempty(strfind(dlnetNeck.Layers(idx+1).Name,'resizeLayer')) || ...
                    ~isempty(strfind(dlnetNeck.Layers(idx+1).Name,'featureResize'))
                if ismember(strcat(destName,'/in'),dlnetNeck.Connections.Destination)
                    destName = strcat(destName,'/in');
                else
                    destName = strcat(destName,'/ref');
                end
            end

            if ~isempty(strfind(dlnetNeck.Layers(idx+1).Name,'routing_layer')) || ...
                    ~isempty(strfind(dlnetNeck.Layers(idx+1).Name,'depthConcat'))
                if ismember(strcat(destName,'/in1'),dlnetNeck.Connections.Destination)
                    destName = strcat(destName,'/in1');
                else
                    destName = strcat(destName,'/in2');
                end

            end
            dlnetOut = disconnectLayers(dlnetOut,srcName,destName);
        end

        % Reconnect all neck layers
        lgraphNeckConnections = dlnetNeck.Connections;

        for idx = 1:height(lgraphNeckConnections)
            dlnetOut = connectLayers(dlnetOut,lgraphNeckConnections.Source{idx},lgraphNeckConnections.Destination{idx});
        end

        % Connect backbone to neck
        if isequal(detector.DetectionNetworkSource,{'add_23','add_19','add_11'}) %darknet53-coco
            dlnetOut = connectLayers(dlnetOut,'add_23','conv2d_53');
            dlnetOut = connectLayers(dlnetOut,'add_19','resizeLayer1/ref');
            dlnetOut = connectLayers(dlnetOut,'add_19','routing_layer_2/in2');
            dlnetOut = connectLayers(dlnetOut,'add_11','resizeLayer2/ref');
            dlnetOut = connectLayers(dlnetOut,'add_11','routing_layer_4/in2');
        elseif isequal(detector.DetectionNetworkSource,{'leaky_relu_7','leaky_relu_5'}) %tiny
            dlnetOut = connectLayers(dlnetOut,'leaky_relu_7','conv2d_8');
            dlnetOut = connectLayers(dlnetOut,'leaky_relu_5','resizeLayer1/ref');
            dlnetOut = connectLayers(dlnetOut,'leaky_relu_5','routing_layer_2/in2');
        else
            % DetectionNetworkSource will already be sorted from lowest feature map
            % layer to highest
            dlnetOut = connectLayers(dlnetOut,detector.DetectionNetworkSource{i}, ...
                ['featureConv',num2str(i+1)]);
            dlnetOut = connectLayers(dlnetOut,detector.DetectionNetworkSource{i+1}, ...
                ['featureResize',num2str(i+1),'/ref']);
            dlnetOut = connectLayers(dlnetOut,detector.DetectionNetworkSource{i+1}, ...
                ['depthConcat',num2str(i+1),'/in2']);
        end
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
if isequal(detector.DetectionNetworkSource,{'add_23','add_19','add_11'})
    dlnetOut = connectLayers(dlnetOut,'leaky_relu_57','conv2d_58');
    dlnetOut = connectLayers(dlnetOut,'leaky_relu_65','conv2d_66');
    dlnetOut = connectLayers(dlnetOut,'leaky_relu_73','conv2d_74');
elseif isequal(detector.DetectionNetworkSource,{'leaky_relu_7','leaky_relu_5'})
    dlnetOut = connectLayers(dlnetOut,'routing_layer_2','conv2d_12');
    dlnetOut = connectLayers(dlnetOut,'leaky_relu_8','conv2d_9');
else
    for idx=1:numHeads
        if idx==1
            dlnetOut = connectLayers(dlnetOut,detector.DetectionNetworkSource{1},['customConv',num2str(1)]);
        else
            dlnetOut = connectLayers(dlnetOut,['depthConcat',num2str(idx)],['customConv',num2str(idx)]);
        end
    end

end

dlX = dlarray(rand(detector.InputSize, 'single'), 'SSCB');
dlnetOut.OutputNames = [];
dlnetOut = initialize(dlnetOut,dlX);

end

% ------------------------------------------------------------------------
% iSubNetworks : Break Network into sub Networks
% ------------------------------------------------------------------------
function [backbone, head, neck] = iSubNetworks(detector,detectionNetworkSource)
% detectionNetworkSource must be sorted in ascending order of feature map
% values of respective layers

lg = layerGraph(detector.Network);
outputNames = detector.Network.OutputNames;
pretrainedYOLOv3 = isequal(detectionNetworkSource,{'add_23','add_19','add_11'}) ||  ...
    isequal(detectionNetworkSource,{'leaky_relu_7','leaky_relu_5'});
allLayers = {lg.Layers.Name};
backboneLast = detectionNetworkSource{1}; % last layer of backbone

% Find layer graph of sub networks
% Backbone
backboneLayers = iGetLayersBefore(lg,backboneLast);
backboneLg = removeLayers(lg,allLayers(~ismember(allLayers, backboneLayers)));

% Head
numHeads = size(outputNames,2);
headLg = cell(numHeads,1);
headLayers = {};
neckLastLayers = {};
for i=1: numHeads
    headILayers = cell(4,1); % yolo v3 type head
    headILayers{1} = outputNames{i};
    headILayers{2} = lg.Connections.Source{strcmp(lg.Connections.Destination,headILayers{1})};
    headILayers{3} = lg.Connections.Source{strcmp(lg.Connections.Destination,headILayers{2})};
    headILayers{4} = lg.Connections.Source{strcmp(lg.Connections.Destination,headILayers{3})};

    if pretrainedYOLOv3
        neckLastLayers = [neckLastLayers(:)' lg.Connections.Source{strcmp(lg.Connections.Destination,headILayers{4})}];
    else
        if i~=1 % there is no neck for first head in custom yolo v3
            neckLastLayers = [neckLastLayers(:)' lg.Connections.Source{strcmp(lg.Connections.Destination,headILayers{4})}];
        end
    end
    headLg{i,1} = removeLayers(lg,allLayers(~ismember(allLayers, headILayers)));
    headLayers = [headLayers(:)' headILayers(:)'];
end

% Neck
neckLg = removeLayers(lg,allLayers(ismember(allLayers,  [headLayers(:)', backboneLayers(:)'])));

% Construct dlnetworks from layer graphs
inputSize = backboneLg.Layers(1).InputSize;
input = dlarray(randn(inputSize),"SSCB");

% Backbone
backboneFeatures = cell(size(detectionNetworkSource));
backbone = dlnetwork(backboneLg,OutputNames=detectionNetworkSource);
[backboneFeatures{:}] = predict(backbone,input);

% Neck
if ~isempty(neckLastLayers)
    neckFeatures = cell(size(neckLastLayers));
    if pretrainedYOLOv3
        if isequal(detectionNetworkSource,{'add_23','add_19','add_11'})
            neckInputFeatures = backboneFeatures([1,2,2,3,3]);
        else
            neckInputFeatures = backboneFeatures([1,2,2]);
        end
        neck = dlnetwork(neckLg, neckInputFeatures{:},OutputNames=neckLastLayers);
        [neckFeatures{:}] = predict(neck,neckInputFeatures{:});
    else
        % Custom YOLO v3
        % if one output layer - no neck
        % if two output layers - 1 connected component in neckLg
        % if n output layers - n-1 connected components in neckLg
        if size(detectionNetworkSource,2)==2
            neckInputFeatures = backboneFeatures([1,2,2]);
            neck = dlnetwork(neckLg, neckInputFeatures{:},OutputNames=neckLastLayers);
            [neckFeatures{:}] = predict(neck, neckInputFeatures{:});
        elseif size(detectionNetworkSource,2)>=3
            % create separate dlnetworks for each connected component in neck
            neckLayers = {neckLg.Layers.Name};
            neck = cell(1,size(neckLastLayers,2));
            for idx=1:size(neckLastLayers,2)
                neckiLayers = iGetLayersBefore(neckLg,neckLastLayers{idx});
                neckiLg = removeLayers(neckLg,neckLayers(~ismember(neckLayers,neckiLayers)));
                neckInputFeatures = backboneFeatures([idx,idx+1,idx+1]);
                neck{idx} = dlnetwork(neckiLg, neckInputFeatures{:},OutputNames=neckLastLayers(idx));
                neckFeatures{idx} = predict(neck{idx},neckInputFeatures{:});
            end
        end
    end
else
    neck = {}; % no neck if custom yolo v3 has only 1 detection network source
end

% Head
head = cell(1,numHeads);
for i=1: numHeads
    if pretrainedYOLOv3
        head{i} = dlnetwork(headLg{i}, neckFeatures{i});
    else
        if i==1
            head{i} = dlnetwork(headLg{i}, backboneFeatures{1}); % no neck layer for first head
        else
            head{i} = dlnetwork(headLg{i}, neckFeatures{i-1});
        end
    end
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
function clippedBBox = iClipRotatedBBox(bbox, imWidth, imHeight)

if ~isempty(bbox)
    angle = double(bbox(:,5));
    bboxPointsTemp = iObtainBBoxPoints(bbox);

    % Remap bboxPoints to be an M-by-8 matrix, where M is the number of
    % bounding boxes and each row is specified as [x1 x2 x3 x4 y1 y2 y3 y4].
    bboxPoints = reshape(bboxPointsTemp,[8,size(bbox,1)])';

    % Clamp vertices to image boundaries.
    bboxPoints(:,1:4) = max( min( bboxPoints(:,1:4), imWidth  ), 1 );
    bboxPoints(:,5:8) = max( min( bboxPoints(:,5:8), imHeight ), 1 );

    % Since any of the sides of the bounding box may
    % have been clipped, calculate and use the minimum
    % widths and heights to reflect the clipping.
    dLength = hypot(bboxPoints(:,3:4) - bboxPoints(:,1:2), ...
        bboxPoints(:,7:8) - bboxPoints(:,5:6))/2;
    w = round(max(hypot(bboxPoints(:,2:3) - bboxPoints(:,[1 4]), ...
        bboxPoints(:,6:7) - bboxPoints(:,[5 8])),[],2),5);
    h = round(max(hypot(bboxPoints(:,3:4) - bboxPoints(:,[2 1]), ...
        bboxPoints(:,7:8) - bboxPoints(:,[6 5])),[],2),5);

    % Use the potentially clipped diagonal lengths to
    % calculate where the new center coordinate should
    % be.
    if dLength(:,1) <= dLength(:,2)
        xCtr = (bboxPoints(:,1) + bboxPoints(:,3))/2;
        yCtr = (bboxPoints(:,5) + bboxPoints(:,7))/2;
    else
        xCtr = (bboxPoints(:,2) + bboxPoints(:,4))/2;
        yCtr = (bboxPoints(:,6) + bboxPoints(:,8))/2;
    end

    clippedBBoxTemp = [xCtr yCtr w h angle];

    % Since more than one rotated bounding box's vertices can be clipped,
    % maintaining the same input angle can result in a vertice going
    % outside the image boundary. Angle is to be maintained, so reclip if
    % the new rotated rectangle bounding box is not within the image
    % boundary.
    isValidIdx = isValidRotatedBBox(clippedBBoxTemp, imWidth, imHeight);
    if any(~isValidIdx)
        clippedBBoxTemp(~isValidIdx,:) = iClipRotatedBBox(clippedBBoxTemp(~isValidIdx,:), imWidth, imHeight);
    end

    clippedBBox = clippedBBoxTemp;
else
    clippedBBox = bbox;
end

end

%--------------------------------------------------------------------------
function bboxPoints = iObtainBBoxPoints(bbox)
% Bounding boxes are rotated rectangle bounding boxes.

% cosd and sind require single or double inputs, so the bounding box is
% casted to double and recast back to the original input class after
% calculations.
numBboxes = size(bbox, 1);
bboxDouble = cast(bbox, "double");
pointsDouble = zeros(4, 2, numBboxes, "double");

r = bboxDouble(:, 5);
u = [bboxDouble(:,3)/2 bboxDouble(:,3)/2].* [ cosd(r) sind(r)];
v = [bboxDouble(:,4)/2 bboxDouble(:,4)/2].* [-sind(r) cosd(r)];

% upper-left (as seen at 0 degrees of rotation).
pointsDouble(1, 1, :) = bboxDouble(:, 1) - u(:, 1) - v(:, 1);
pointsDouble(1, 2, :) = bboxDouble(:, 2) - u(:, 2) - v(:, 2);

% upper-right (as seen at 0 degrees of rotation).
pointsDouble(2, 1, :) = bboxDouble(:, 1) + u(:, 1) - v(:, 1);
pointsDouble(2, 2, :) = bboxDouble(:, 2) + u(:, 2) - v(:, 2);

% lower-right (as seen at 0 degrees of rotation).
pointsDouble(3, 1, :) = bboxDouble(:, 1) + u(:, 1) + v(:, 1);
pointsDouble(3, 2, :) = bboxDouble(:, 2) + u(:, 2) + v(:, 2);

% lower-left (as seen at 0 degrees of rotation).
pointsDouble(4, 1, :) = bboxDouble(:, 1) - u(:, 1) + v(:, 1);
pointsDouble(4, 2, :) = bboxDouble(:, 2) - u(:, 2) + v(:, 2);

bboxPoints = cast(pointsDouble, "like", bbox);
end

%--------------------------------------------------------------------------
function validROI = isValidRotatedBBox(bbox ,imWidth, imHeight)
% Ensure bbox is an M-by-5 matrix
if ~(~isempty(bbox) && ismatrix(bbox) && size(bbox,2) == 5)
    validROI = false;
else
    % Convert rotated rectangle bounding box to vertices.
    bboxPointsTemp = iObtainBBoxPoints(bbox);

    % Remap bboxPoints to be an M-by-8 matrix, where M is the number of
    % bounding boxes and each row is specified as [x1 x2 x3 x4 y1 y2 y3 y4].
    bboxPoints = round(reshape(bboxPointsTemp,[8,size(bbox,1)])');

    validROI = ...
        all(bboxPoints >= 1 & ...
        [bboxPoints(:,1:4) <= imWidth bboxPoints(:,5:8) <= imHeight],2);

    validROI = validROI & bbox(:,3) > 0 & bbox(:,4) > 0;
end
end

