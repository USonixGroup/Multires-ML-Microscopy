classdef rtmdetObjectDetector < vision.internal.detector.ObjectDetector
    %

    % Copyright 2024 The MathWorks, Inc.

    properties(SetAccess = protected, Hidden)
        % RTMdet dlnetwork
        Network 
    end

    properties(SetAccess = protected)
        % ClassNames specifies the names of the classes that RTMDet object
        % detector can detect.
        ClassNames

        % % InputSize is a vector of the form [height width] or [height width channels]
        % with channels as 3 always 
        % defining image size used to infer from detector. During detection,
        % an input image is resized to this size before it is processed by
        % the detection network if AutoResize is set to true (default).
        InputSize
    end

    properties(Dependent = true)
        % NormalizationStatistics specifies z-score normalization statitics
        % as a structure with fields, Mean and StandardDeviation specified
        % as 1-by-C array of means and standard deviation per channel. The
        % number of channels, C must match the InputSize
        NormalizationStatistics
    end

    properties(Dependent = true, Hidden = true)
        % InputNames is the cell array of input names for the RTMDet dlnetwork.
        InputNames

        % OutputNames is the cell array of output names for the RTMDet dlnetwork.
        OutputNames

        % Anchor points used to compute bounding box locations
        Grids

        % Stride values expanded to match grid points
        ExpandedStrides
    end

    properties (Access = protected, Transient)
        FilterBboxesFunctor
    end

    properties(Dependent, Hidden)
        % Layers is the array of layers in the RTMDet dlnetwork.
        Layers
    end

    properties (Access = private)
        GridsInternal = [];
        ExpandedStridesInternal = [];
        NormalizationStatisticsInternal = [];
    end

    methods
        function this = rtmdetObjectDetector(detectorName,options)
            arguments
                detectorName {mustBeTextScalar, iMustBeValidDetectorName} = iGetSmallNetworkDetectorName();
                options.InputSize {mustBeNumeric, mustBeFinite, mustBePositive, mustBeReal,iMustBeValidInputSize} = []
                options.ModelName {mustBeTextScalar} = ""
                options.NormalizationStatistics = []
            end
            
            vision.internal.requiresNeuralToolbox(mfilename);
            if detectorName == "uninitialized"
                return
            end

            % Loads and configure the pretrained model as specified in detectorName.
            params = rtmdetObjectDetector.parsePretrainedDetectorInputs(detectorName,options);
            this.Network = iTripwireRTMDetModels(params);
            this.ClassNames = iGetCOCOClasses();
            this.InputSize = params.InputSize;
            this.ModelName = params.ModelName;
            if isempty(params.NormalizationStatistics)
                if size(this.InputSize,2)==2
                    numChannel = 1;
                else
                    numChannel = this.InputSize(3);
                end
                this.NormalizationStatistics = iDefaultNormalizationStats(numChannel);
            else
                this.NormalizationStatistics = params.NormalizationStatistics;
            end

            this = initializeGridsAndStrides(this, this.InputSize(1:2));
            this.FilterBboxesFunctor = vision.internal.cnn.utils.FilterBboxesFunctor;
        end
    end

    methods
        function varargout = detect(detector, I, roi, options)
            %

            % Copyright 2024 The MathWorks, Inc.
            arguments
                detector rtmdetObjectDetector
                I {mustBeA(I,["numeric","matlab.io.Datastore","matlab.io.datastore.Datastore","gpuArray"]),mustBeNonempty}
                roi = zeros(0,4)
                options.Threshold (1,1) {rtmdetObjectDetector.checkThreshold(options.Threshold, 'rtmdetObjectDetector')}= 0.25
                options.SelectStrongest {vision.internal.inputValidation.validateLogical(options.SelectStrongest, 'SelectStrongest')} = true
                options.MinSize (1,2) {vision.internal.detector.ValidationUtils.checkMinSize(options.MinSize, [1,1], 'rtmdetObjectDetector')} = [1,1]
                options.MaxSize (1,2) {mustBeInteger, mustBePositive, mustBeFinite, mustBeReal, mustBeNonempty, mustBeNonsparse} = detector.InputSize(1:2)
                options.MiniBatchSize (1,1) {vision.internal.cnn.validation.checkMiniBatchSize(options.MiniBatchSize, 'rtmdetObjectDetector')} = 16
                options.ExecutionEnvironment {mustBeTextScalar,mustBeMember(options.ExecutionEnvironment,{'gpu','cpu','auto'})} = "auto"
                options.Acceleration {mustBeTextScalar,mustBeMember(options.Acceleration,{'auto','mex','none'})} = "auto"
                options.AutoResize {vision.internal.inputValidation.validateLogical(options.AutoResize, 'AutoResize')} = true

            end
            
            
            [params, imageSize, networkInputSize] = iValidateImageInput(detector, I);

            useROI = ~isempty(roi);

            if useROI
                vision.internal.detector.checkROI(roi, imageSize);
            end

            % Validate MinSize and MaxSize only if they are set to
            % non-default values
            validateMinSize = ~isequal(options.MinSize,[1 1]);                   
            validateMaxSize = ~isequal(options.MaxSize,detector.InputSize(1:2));

            % Set MaxSize to default value
            if isequal(options.MaxSize,detector.InputSize(1:2))
                options.MaxSize = imageSize(1:2);
            end
            
            if validateMaxSize
                 if useROI
                    if any(options.MaxSize > roi([4 3]))
                        coder.internal.warning(...
                            'vision:rtmdetObjectDetector:modelMaxSizeGTROISize',...
                            roi(1,4),roi(1,3));
                    end
                else
                    if any(options.MaxSize > imageSize(1:2))
                        coder.internal.warning(...
                            'vision:rtmdetObjectDetector:modelMaxSizeGTImgSize',...
                            imageSize(1,1),imageSize(1,2));
                    end
                end
            end

            if validateMinSize && validateMaxSize
                coder.internal.errorIf(any(options.MinSize >= options.MaxSize) , ...
                    'vision:rtmdetObjectDetector:maxSizeGTMinSize');
            end


            if useROI
                if ~isempty(roi)
                    sz = roi([4 3]);
                    vision.internal.detector.ValidationUtils.checkImageSizes(sz(1:2), options, validateMinSize, ...
                        options.MinSize, ...
                        'vision:rtmdetObjectDetector:roiLessThanMinSize', ...
                        'vision:rtmdetObjectDetector:roiLessThanMinSize');
                end
            else
                vision.internal.detector.ValidationUtils.checkImageSizes(imageSize(1:2), options, validateMinSize, ...
                    options.MinSize , ...
                    'vision:rtmdetObjectDetector:imageLessThanMinSize', ...
                    'vision:rtmdetObjectDetector:imageLessThanMinSize');
            end

            params.ROI                      = single(roi);
            params.UseROI                   = useROI;
            params.SelectStrongest          = logical(options.SelectStrongest);
            params.MinSize                  = single(options.MinSize);
            params.MaxSize                  = single(options.MaxSize);
            params.MiniBatchSize            = double(options.MiniBatchSize);
            params.Threshold                = single(options.Threshold);
            params.NMSThreshold             = single(0.5); 
            params.FilterBboxesFunctor      = detector.FilterBboxesFunctor;
            params.NetworkInputSize         = double(networkInputSize);
            params.ExecutionEnvironment     = string(options.ExecutionEnvironment);
            params.Acceleration             = string(options.Acceleration);
            params.AutoResize               = logical(options.AutoResize);

            [varargout{1:nargout}] = performDetect(detector, I, params);
        end
    end

    %----------------------------------------------------------------------
    methods
        function layers = get.Layers(this)
            layers = this.Network.Layers;
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
        function TF = get.Grids(this)
            TF = this.GridsInternal;
        end

        %------------------------------------------------------------------
        function TF = get.ExpandedStrides(this)
            TF = this.ExpandedStridesInternal;
        end
        %------------------------------------------------------------------
        function this = set.NormalizationStatistics(this,statsStruct)

            if size(this.InputSize,2)==2
                numChannel = 1;
            else
                numChannel = this.InputSize(3);
            end
            iValidateNormalizationStats(statsStruct,numChannel)
            
            this.NormalizationStatisticsInternal = struct("Mean",gather(reshape(statsStruct.Mean,[1 numChannel])),...
                "StandardDeviation",gather(reshape(statsStruct.StandardDeviation,[1 numChannel])));
            
            statsStructForInputNorm = statsStruct;
            if ~all(isfield(statsStructForInputNorm, {'Mean','Std','Max','Min'}))
                statsStructForInputNorm.Mean = reshape(statsStructForInputNorm.Mean,[1 1 numChannel]);
                statsStructForInputNorm.Std = reshape(statsStructForInputNorm.StandardDeviation,[1 1 numChannel]);
                statsStructForInputNorm.Min = [];
                statsStructForInputNorm.Max = [];
            end

            this = setInputNormalization(this,statsStructForInputNorm);
        end

        function statsStruct = get.NormalizationStatistics(this)
            statsStruct = this.NormalizationStatisticsInternal;
        end

    end

    %----------------------------------------------------------------------
    methods(Hidden)

        function this = initializeGridsAndStrides(this, imgSize)
            [grids, expandedStrides] = rtmdetObjectDetector.computeGridsAndStrides(imgSize);
            this.GridsInternal = grids;
            this.ExpandedStridesInternal = expandedStrides;
        end

        %------------------------------------------------------------------
        % Preprocess input data.
        %------------------------------------------------------------------
        function varargout = preprocess(detector, I, varargin)
            % This method preprocesses the input data prior to calling
            % the predict method. It resizes the input data to the
            % detector.InputSize when params.AutoResize is true.
            % Otherwise, input data is passed as-is.

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
                    params.ROI,params.UseROI,params.ExecutionEnvironment,detector.InputSize,...
                    params.AutoResize,params.CastToGpuArray));
                varargout{2} = {};
            else
                [varargout{1:nargout}] = iPreprocessForDetect(I, ...
                    params.ROI, params.UseROI, params.ExecutionEnvironment,detector.InputSize,...
                    params.AutoResize,params.CastToGpuArray);
            end
        end

        %------------------------------------------------------------------
        % Predict output feature maps.
        %------------------------------------------------------------------
        function outputFeatures = predict(detector,dlX,varargin)
            % This method predicts features of the preprocessed image dlX.
            % The outputFeatures is a N-by-1 cell array, where N are the
            % number of outputs in network. Each cell of outputFeature
            % contains predictions from an output layer.

            predictParams = parsePredictInputs(detector,varargin);
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
                    features = iPredictActivations(detector, imgBatch, predictParams.Acceleration,predictParams.AutoResize);

                    for ii = 1:numMiniBatch
                        fmap = features(:,:,ii);
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
                    outputFeatures = iPredictBatchActivations(detector, dlX, predictParams.Acceleration,predictParams.AutoResize);
                else
                    if size(dlX,4)>1
                        outputFeatures = iPredictMultiActivations(detector, dlX, predictParams.Acceleration,predictParams.AutoResize);
                    else
                        outputFeatures = iPredictActivations(detector, dlX, predictParams.Acceleration,predictParams.AutoResize);
                    end
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
        % Parse preprocess input parameters.
        %------------------------------------------------------------------
        function params = parsePreprocessInputs(~, I, varargin)
            params.AutoResize = varargin{1,1}{1,1}.AutoResize;
            if ~params.AutoResize && varargin{1,1}{1,1}.UseROI
                error(message('vision:rtmdetObjectDetector:noROIWithoutAutoResize'))
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
        function params = parsePredictInputs(~,varargin)
            params = varargin{1,1}{1,1};
        end
    end

    methods(Static, Hidden, Access = protected)
        %------------------------------------------------------------------
        % Parse and validate pretrained detector parameters.
        %------------------------------------------------------------------
        function params = parsePretrainedDetectorInputs(detectorName,options)
            params = options;

            % Parse inputs for this syntax:
            % detector = rtmdetObjectDetector(detectorName).
            params.DetectorName = detectorName;
            % Default input size.
            inputSize = [640 640 3];
           
            if isempty(options.InputSize)
                params.InputSize = inputSize;
            end

            iCheckInputSize(params.InputSize);

            if params.InputSize(1) == 1 || params.InputSize(2) == 1
                error(message('vision:rtmdetObjectDetector:inputSizeMustBeAtleastTwo',params.DetectorName));
            end

            if strcmp(params.ModelName,"")
                params.ModelName = params.DetectorName;
            end

            if ~isempty(params.NormalizationStatistics)
                iValidateNormalizationStats(params.NormalizationStatistics,params.InputSize(3))
            end
            
        end
    end

    methods(Static, Hidden)
        function predictions = rtmdetTransform(YPredictions, grids, expandedStrides)
            % Transforms predictions from dlnetwork to
            % box [x1 y1 x2 y2] and cls scores
            
            % Reshape predictions from model 
            for i = 1:size(YPredictions,1)
                [h,w,c,n] = size(YPredictions{i,1});
                YPredictions{i,1} = reshape(permute(YPredictions{i,1},[2 1 3 4]), [h*w c n]);
                YPredictions{i,1} = permute(YPredictions{i,1},[2 1 3 4]);
            end

            % Concat all Predictions from each head 
            predictions = cat(2,YPredictions{:});
 
            % Compute box coordinates x1,y1,x2,y2
            grids = grids';
            predictions(1,:,:) = (grids(1,:) - predictions(1,:,:)) .* expandedStrides';
            predictions(2,:,:) = (grids(2,:) - predictions(2,:,:)) .* expandedStrides';
            predictions(3,:,:) = (grids(1,:) + predictions(3,:,:)) .* expandedStrides';
            predictions(4,:,:) = (grids(2,:) + predictions(4,:,:)) .* expandedStrides';
        end
    end

    methods (Hidden)
        function this = setInputNormalization(this,stats)
            outputNames = this.Network.OutputNames;
            dlnetNetwork = this.Network;
            currentInputLayer = this.Network.Layers(1);
            map = normalizationStatsDictionary(stats);
            statsSet = map(currentInputLayer.Normalization);
            inputSize = size(getExampleInputsFromNetwork(this.Network));
            newInputLayer = imageInputLayer(inputSize,"Name",currentInputLayer.Name,...
                "Normalization",currentInputLayer.Normalization,...
                statsSet{:});
            dlnetNetwork = replaceLayer(dlnetNetwork,this.Network.Layers(1).Name,newInputLayer);
            this.Network = initialize(dlnetNetwork);
            this.Network.OutputNames = outputNames;
        end
    end

    %======================================================================
    % Save/Load
    %======================================================================
    methods(Hidden)
        function s = saveobj(this)
            s.Version                      = 1.0;
            s.ModelName                    = this.ModelName;
            s.Network                      = this.Network;
            s.ClassNames                   = this.ClassNames;
            s.InputSize                    = this.InputSize;
            s.GridsInternal                = this.GridsInternal;
            s.ExpandedStridesInternal      = this.ExpandedStridesInternal;
            s.FilterBboxesFunctor          = this.FilterBboxesFunctor;
            s.NormalizationStatistics      = this.NormalizationStatistics;
        end
        
        function dlnet = matlabCodegenPrivateNetwork(this)
            dlnet = this.Network;
        end
    end

    methods(Static, Hidden)
        function this = loadobj(s)
            try
                vision.internal.requiresNeuralToolbox(mfilename);
                this = rtmdetObjectDetector("uninitialized");
                this.Network                 = s.Network;
                this.ClassNames              = s.ClassNames;
                this.InputSize               = s.InputSize;
                this.ModelName               = s.ModelName;
                this.GridsInternal           = s.GridsInternal;
                this.ExpandedStridesInternal = s.ExpandedStridesInternal;
                this.FilterBboxesFunctor     = s.FilterBboxesFunctor;
                this.NormalizationStatistics = s.NormalizationStatistics;

            catch ME
                rethrow(ME)
            end
        end

        function n = matlabCodegenDlRedirect(~)
            n = 'vision.internal.codegen.RTMDetObjectDetector';
        end
    end

    %----------------------------------------------------------------------
    methods(Static, Hidden)
        function [data, scale, padding] = preprocessInput(data, targetSize)
            if istable(data)
                data = table2cell(data);
            end

            if iscell(data)
                % Preprocess inputs while training
                for idx = 1:size(data,1)
                    I = data{idx,1};
                    [I, scale, padding] = iPreprocessSingleImg(I,targetSize(1:2));
                    
                    bboxes = bboxresize(data{idx,2}, scale);
                    data(idx,1:2) = {I, bboxes};
                end
            else
                batchSize = size(data,4);
                if batchSize>1
                    dataTmp = [];
                    for i = 1:batchSize
                        [Itmp, scale, padding] = iPreprocessSingleImg(data(:,:,:,i),targetSize(1:2));
                        if isempty(dataTmp)
                            dataTmp = Itmp;
                        else
                            dataTmp = cat(4,dataTmp,Itmp);
                        end
                    end
                    data = dataTmp;
                else
                    [data, scale, padding] = iPreprocessSingleImg(data,targetSize(1:2));
                end
            end
        end

        function [grids, expandedStrides] = computeGridsAndStrides(imgSize)
            % Compute horizontal and vertical grid sizes based on image
            % size and strides.

            grids = [];
            expandedStrides = [];

            % Define strides.
            strides = [8, 16, 32];

            for k = 1:numel(strides)
                hsize = ceil(imgSize(1) ./ strides(k));
                wsize = ceil(imgSize(2) ./ strides(k));
                stride = strides(k);
                [xv, yv] = meshgrid(0:(wsize-1), 0:(hsize-1));
                grid = reshape(cat(3, xv', yv'),[],2);
                grids = [grids; grid]; %#ok<AGROW>
                shape = size(grid,1);
                expandedStrides = [expandedStrides; ones(shape,1)*stride]; %#ok<AGROW>
            end
        end
    end
end

%--------------------------------------------------------------------------
function iCheckInputSize(inputSize)
    
    validateattributes(inputSize, {'numeric'}, ...
        {'2d','nonempty','nonsparse',...
        'real','finite','integer','positive','nrows',1}); 
end

%--------------------------------------------------------------------------
function outputFeatures = iPredictMultiActivations(detector,dlX, acceleration,autoResize)
    numMiniBatch = size(dlX,4);
    outputFeatures = cell(numMiniBatch,1);
    for ii = 1:numMiniBatch
        inp = dlX(:,:,:,ii);
        outputFeatures{ii,1} = iPredictActivations(detector, inp, acceleration,autoResize);
    end
end
%--------------------------------------------------------------------------
function outputFeatures = iPredictBatchActivations(detector,dlX, acceleration,autoResize)
    numMiniBatch = size(dlX,2);
    outputFeatures = cell(numMiniBatch,1);
    for ii = 1:numMiniBatch
        inp = dlX{ii};
        outputFeatures{ii,1} = iPredictActivations(detector, inp, acceleration,autoResize);
    end
end

function outputFeatures = iPredictActivations(detector, dlX, acceleration,autoResize)
features = cell(size(detector.Network.OutputNames'));
[features{:}] = predict(detector.Network, dlX, 'Acceleration',acceleration);

if autoResize
    grids = detector.Grids;
    expandedStrides = detector.ExpandedStrides;
else
    [grids, expandedStrides] = rtmdetObjectDetector.computeGridsAndStrides(size(dlX,1:2));
end
outputFeatures = rtmdetObjectDetector.rtmdetTransform(features, grids, expandedStrides);
end

%--------------------------------------------------------------------------
function [bboxes,scores,labels] = iPostprocessMultiDetection(detector,YPredData,info,params)
    numMiniBatch = size(YPredData,1);
    bboxes = cell(numMiniBatch, 1);
    scores = cell(numMiniBatch, 1);
    labels = cell(numMiniBatch, 1);
    for ii = 1:numMiniBatch
        [bboxes{ii},scores{ii},labels{ii}] = ...
            iPostprocessSingleDetection(detector,YPredData{ii,1},info,params);
    end
end

%--------------------------------------------------------------------------
function [bboxes,scores,labels] = iPostprocessSingleDetection(detector,YPredData,info,params)
    % Obtain the classnames detector is trained on.
    classes = detector.ClassNames;

    % The head does not include the final sigmoid needed to map the
    % class prediction logits into probabilities.
    YPredData(5:end,:) = sigmoid(YPredData(5:end,:));

    detections = extractdata(YPredData)';

    % Apply following post processing steps to filter the detections:
    % * Filter detections based on threshold.
    
    % Filter the classes based on class probability.
    [classProbs, classIdx] = max(detections(:,5:end),[],2);
    detections(:,5) = classProbs;
    detections(:,6) = classIdx;

    % Keep detections whose objectness score is greater than thresh.
    detectionsKeep = detections(detections(:,5)>=params.Threshold,:);

    [bboxes,scores,labels] = iPostProcessDetections(detectionsKeep,classes,info,params);
end

%--------------------------------------------------------------------------
function [bboxes,scores,labels] = iPostProcessDetections(detections,classes,info,params)

    if ~isempty(detections)

        scorePred = detections(:,5);
        bboxesTmp = detections(:,1:4);
        classPred = detections(:,6);

        inputImageSize(2) = info.InputImageSize(2);
        inputImageSize(1) = info.InputImageSize(1);

        if params.AutoResize
            % Shift box coordinates by remove padding 
            pad = [info.padding(3) info.padding(1) info.padding(3) info.padding(1)];
            bboxesTmp = bboxesTmp - pad;

            % Resize boxes to image size.
            scale = [info.ScaleX info.ScaleY info.ScaleX info.ScaleY];
            bboxPred = bboxesTmp.*scale;
        else
            bboxPred = bboxesTmp;
        end

        % Convert boxes from [x1 y1 x2 y2] to [x y w h].
        bboxPred = iConvertX1Y1X2Y2ToXYWH(bboxPred);

        % Filter boxes based on MinSize, MaxSize.
        [bboxPred, scorePred, classPred] = filterBBoxes(params.FilterBboxesFunctor,...
            params.MinSize,params.MaxSize,bboxPred,scorePred,classPred);

        % Apply NMS.
        if params.SelectStrongest
            [bboxes, scores, classNames] = selectStrongestBboxMulticlass(bboxPred, scorePred, classPred ,...
                'RatioType', 'Union', 'OverlapThreshold', params.NMSThreshold);
        else
            bboxes = bboxPred;
            scores = scorePred;
            classNames = classPred;
        end
        
        % Limit width detections
        detectionsWd = min((bboxes(:,1) + bboxes(:,3)),inputImageSize(1,2));
        bboxes(:,3) = detectionsWd(:,1) - bboxes(:,1);

        % Limit Height detections
        detectionsHt = min((bboxes(:,2) + bboxes(:,4)),inputImageSize(1,1));
        bboxes(:,4) = detectionsHt(:,1) - bboxes(:,2);

        bboxes(bboxes<1) = 1;

        % Apply ROI offset
        bboxes(:,1:2) = vision.internal.detector.addOffsetForROI(bboxes(:,1:2), params.ROI, params.UseROI);

        % Convert classId to classNames.
        % Create categories of labels such that the order of the classes is retained.
        labels = categorical(classes(classNames),classes);

        if params.CastToGpuArray
            scores = gather(scores);
            bboxes = gather(bboxes);
            labels = gather(labels);
        end
    else
        bboxes = zeros(0,4,'single');
        scores = zeros(0,1,'single');
        labels = categorical(cell(0,1),cellstr(classes));
    end
end

%--------------------------------------------------------------------------
function boxes = iConvertX1Y1X2Y2ToXYWH(boxes)
% Convert [x1 y1 x2 y2] boxes into [x y w h] format. Input and
% output boxes are in pixel coordinates. boxes is an M-by-4
% matrix.
boxes(:,3) = boxes(:,3) - boxes(:,1) + 1;
boxes(:,4) = boxes(:,4) - boxes(:,2) + 1;
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
function [output, scale, padding] = iPreprocessSingleImg(img, targetSize)

    % Compute scaling ratio
    imgSize = size(img,1:2);
    scaleRatio = min(max(targetSize) / max(imgSize), min(targetSize) / min(imgSize));
    scale = [scaleRatio, scaleRatio];
    
    % Determine the interpolation method
    interpolation = 'bilinear';
    if scaleRatio < 1
        interpolation = 'box'; 
    end
    
    % Resize the image if necessary
    if scaleRatio ~= 1
        img = imresize(img, scaleRatio, interpolation, 'Antialiasing', false);
    end
    
    % Compute new size and scale ratio
    imgSize = size(img,1:2);
    ratio = min(targetSize ./ imgSize);
    ratio = min(ratio, 1.0); % Ensure only scaling down
    noPadShape = round(imgSize .* ratio);
    
    % Resize to no padding shape if different
    if ~isequal(imgSize, noPadShape)
        img = imresize(img, noPadShape, 'bilinear', 'Antialiasing', false);
    end
    
    % padding height & width
    paddingH = targetSize(1) - noPadShape(1);
    paddingW  = targetSize(2) - noPadShape(2);
    % Compute padding
    top = round(floor(paddingH / 2) - 0.1);
    left = round(floor(paddingW / 2) - 0.1);
    bottom = paddingH - top;
    right = paddingW - left;
    padding = [top, bottom, left, right];
    
    % Convert to single
    img = single(img);

    % Apply padding if necessary
    output = iPerformPadding(targetSize, img, top,left, 114);

end
%--------------------------------------------------------------------------
function Inew = iPerformPadding(targetSize, I, top,left, padVal)
[Irow,Icol,Ichannels, IBatchSize] = size(I,1:4);

% Apply padding 
if (top~=0 || left~=0)
    % Initialize Inew with padVal
    Inew = ones([targetSize,Ichannels, IBatchSize],'like',I).*padVal;
    Inew(top+1:top+Irow,left+1:left+Icol,:,:) = I;
else
    Inew = I;  
end
end

%--------------------------------------------------------------------------
function [Ipreprocessed,info] = iPreprocessForDetect(I, roi, useROI, executionEnvironment, networkInputSize, autoResize, castToGpuArray)

    % Check if the input datatype is valid or not.
    if ~(isa(I,'uint8') || isa(I,'uint16') || isa(I,'int16') || ...
            isa(I,'double') || isa(I,'single') || isa(I,'gpuArray'))
        error(message('vision:rtmdetObjectDetector:unsupportedInputClassDetect'));
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
    if autoResize
        [Ipreprocessed, scale, padding] = rtmdetObjectDetector.preprocessInput(Iroi, info.PreprocessedImageSize);
        sz = size(Iroi);
        % The scale output from preprocessInput is meant to be used for the
        % training workflow where the computed bounding boxes corresponding
        % to input image are resized to the target size (network input
        % size). In the inference workflow, bounding boxes corresponding to
        % the target size (network input size) are resized to the input
        % train image. Therefore, the scale factor must be inversed.
        [info.ScaleX,info.ScaleY] = deal(1/scale(1),1/scale(2));
        info.InputImageSize = sz;
        info.padding = padding; 
    else
        sz = size(Iroi);
        [info.ScaleX,info.ScaleY] = deal(1,1);
        info.InputImageSize = sz;

        % convert to single 
        Ipreprocessed = single(Iroi);
        info.padding = [0 0 0 0];
    end

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
            error(message('vision:rtmdetObjectDetector:unableToBatchImagesForDetect'));
        else
            throwAsCaller(e);
        end
    end
    data{2} = X(:,2:end);
end

function network = iMakeRTMDetTinyModels()
    % Check if support package is installed
    breadcrumbFile = 'vision.internal.cnn.supportpackages.IsRTMDetInstalled';
    fullPath = which(breadcrumbFile);
    if isempty(fullPath)
        name     = 'Computer Vision Toolbox Model for RTMDet Object Detection';
        basecode = 'RTMDET_DETECTION';
        throwAsCaller(MException(message('nnet_cnn:supportpackages:InstallRequired', mfilename, name, basecode)));
    else
        % Load pretrained network.
        pattern = fullfile(filesep, '+vision','+internal','+cnn','+supportpackages','IsRTMDetInstalled.m');
        idx     = strfind(fullPath, pattern);
        matfile = fullfile(fullPath(1:idx), 'data', 'rtmTinyCOCO.mat');
        data = load(matfile);
        network = data.detector;
    end
end

function network = iMakeRTMDetSmallModels()
    % Check if support package is installed
    breadcrumbFile = 'vision.internal.cnn.supportpackages.IsRTMDetInstalled';
    fullPath = which(breadcrumbFile);
    if isempty(fullPath)
        name     = 'Computer Vision Toolbox Model for RTMDet Object Detection';
        basecode = 'RTMDET_DETECTION';
        throwAsCaller(MException(message('nnet_cnn:supportpackages:InstallRequired', mfilename, name, basecode)));
    else
        % Load pretrained network.
        pattern = fullfile(filesep, '+vision','+internal','+cnn','+supportpackages','IsRTMDetInstalled.m');
        idx     = strfind(fullPath, pattern);
        matfile = fullfile(fullPath(1:idx), 'data', 'rtmSmallCOCO.mat');
        data = load(matfile);
        network = data.detector;
    end
end

function network = iMakeRTMDetMediumModels()
    % Check if support package is installed
    breadcrumbFile = 'vision.internal.cnn.supportpackages.IsRTMDetInstalled';
    fullPath = which(breadcrumbFile);
    if isempty(fullPath)
        name     = 'Computer Vision Toolbox Model for RTMDet Object Detection';
        basecode = 'RTMDET_DETECTION';
        throwAsCaller(MException(message('nnet_cnn:supportpackages:InstallRequired', mfilename, name, basecode)));
    else
        % Load pretrained network.
        pattern = fullfile(filesep, '+vision','+internal','+cnn','+supportpackages','IsRTMDetInstalled.m');
        idx     = strfind(fullPath, pattern);
        matfile = fullfile(fullPath(1:idx), 'data', 'rtmMediumCOCO.mat');
        data = load(matfile);
        network = data.detector;
    end   
end

function network = iMakeRTMDetLargeModels()
    % Check if support package is installed
    breadcrumbFile = 'vision.internal.cnn.supportpackages.IsRTMDetInstalled';
    fullPath = which(breadcrumbFile);
    if isempty(fullPath)
        name     = 'Computer Vision Toolbox Model for RTMDet Object Detection';
        basecode = 'RTMDET_DETECTION';
        throwAsCaller(MException(message('nnet_cnn:supportpackages:InstallRequired', mfilename, name, basecode)));
    else
        % Load pretrained network.
        pattern = fullfile(filesep, '+vision','+internal','+cnn','+supportpackages','IsRTMDetInstalled.m');
        idx     = strfind(fullPath, pattern);
        matfile = fullfile(fullPath(1:idx), 'data', 'rtmLargeCOCO.mat');
        data = load(matfile);
        network = data.detector;
    end
end


%--------------------------------------------------------------------------
function network = iTripwireRTMDetModels(params)
    if strcmp(params.DetectorName,  iGetLargeNetworkDetectorName())
        network = iMakeRTMDetLargeModels;
    elseif strcmp(params.DetectorName,  iGetMediumNetworkDetectorName())
        network = iMakeRTMDetMediumModels;
    elseif strcmp(params.DetectorName,  iGetSmallNetworkDetectorName())
        network = iMakeRTMDetSmallModels;
    elseif strcmp(params.DetectorName,  iGetTinyNetworkDetectorName())
        network = iMakeRTMDetTinyModels;
    end

    if ~isequal(params.InputSize, [640 640 3])
    network = iUpdateFirstConvChannelsAndInputLayer(network,params.InputSize);
    end
end

%--------------------------------------------------------------------------
function dlnetOut = iUpdateFirstConvChannelsAndInputLayer(dlnet,imageSize)
    % This function update the channels of first conv layer if InputSize channel
    % does not match with channels of first conv layer. It also update the
    % imageInputLayer or initialize the dlnetwork if image input layer not present.
    
    if size(imageSize,2)==2
        imageSize = [imageSize 1];
    end
    
    outputNames = dlnet.OutputNames;
    
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
        error(message('vision:rtmdetObjectDetector:mustHaveConvLayers'));
    end
    
    needToReplaceFirstConvLayer = ~strcmp(numFirstConvLayerChannels,'auto') && ...
        numFirstConvLayerChannels~=numChannel;
    needToReplaceInputLayer = ~isempty(imageInputIdx);
    
    if needToReplaceFirstConvLayer || needToReplaceInputLayer
        % Capture the layers once if we're going to edit the graph
        layers = dlnet.Layers;
    end

    % If number of channels in imageSize is not equal to the channel count
    % of first convolutional layer, update the channel count of first conv
    % layer and use values of properties as it is. Pyramid pooling concept
    % has been used for concatenating extra channel. Each extra channel is
    % mean of original (initial) channels of conv layer.
    %
    % Zhao, Hengshuang, et al. "Pyramid Scene Parsing Network." 2017 IEEE
    % Conference on Computer Vision and Pattern Recognition (CVPR). IEEE, 2017.
    if needToReplaceFirstConvLayer
        firstConvLayer = layers(convIdx,1);
        firstConvLayerWeights = firstConvLayer.Weights;
        % RTMDet only supports gray scale and RGB images 
        updatedConvLayerWeights = reshape(mean(firstConvLayerWeights,3),size(firstConvLayerWeights(:,:,1,:)));

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
        dlnet = replaceLayer(dlnet,layers(convIdx).Name,...
            updatedConvLayer);
    end
    
    % If imageSize is not equal to the InputSize, replace the imageInputLayer.
    if needToReplaceInputLayer
        inputLayer = layers(imageInputIdx,1);
        if ~isequal(inputLayer.InputSize,imageSize)
            % Mean and Standard deviation will be set in constructor
            % according to COCO values 
            imageInput = imageInputLayer(imageSize, ...
                'Normalization','zscore', ...
                'Name',layers(imageInputIdx).Name);
            dlnet = replaceLayer(dlnet,layers(imageInputIdx).Name,...
                imageInput);
        end
    end
    dlX = dlarray(rand(imageSize, 'single'), 'SSCB');
    dlnetOut = initialize(dlnet,dlX);
    dlnetOut.OutputNames = outputNames;
end

function detectorName = iGetLargeNetworkDetectorName()
detectorName = "large-network-coco";
end

function detectorName = iGetMediumNetworkDetectorName()
detectorName = "medium-network-coco";
end

function detectorName = iGetSmallNetworkDetectorName()
detectorName = "small-network-coco";
end

function detectorName = iGetTinyNetworkDetectorName()
detectorName = "tiny-network-coco";
end

function map = normalizationStatsDictionary(stats)
    % This maps knowledge of how different styles of normalization in the input
    % layer (Keys) map to different Name/Value inputs to the statistics field
    % of the input layer.
    map = containers.Map({'zerocenter','zscore','rescale-symmetric','rescale-zero-one','none'},...
        { {'Mean',gather(stats.Mean)}, {'StandardDeviation',gather(stats.Std),'Mean',gather(stats.Mean)},...
        { 'Min', gather(stats.Min), 'Max', gather(stats.Max) },...
        { 'Min', gather(stats.Min), 'Max', gather(stats.Max) },...
        {} });

end

function iValidateNormalizationStats(stats,inputChannelSize)
mustBeA(stats,"struct");
mustBeScalarOrEmpty(stats)
tf = isfield(stats, {'Mean','StandardDeviation'});

if ~all(tf)|| ~all(ismember(fieldnames(stats),{'Mean','StandardDeviation'}))
    error(message('vision:rtmdetObjectDetector:invalidNormalizationStatisticsStruct'));
end

meanSize = size(stats.Mean);
stdSize = size(stats.StandardDeviation);
iValidateNormalizationStatsSize(meanSize,inputChannelSize);
iValidateNormalizationStatsSize(stdSize,inputChannelSize);

end

function iValidateNormalizationStatsSize(statsSize,inputChannelSize)
if (numel(statsSize) == 2 && any(statsSize ~= [1 inputChannelSize])) || ...
        (numel(statsSize) == 3 && any(statsSize ~= [1 1 inputChannelSize])) || ...
        numel(statsSize) > 3
    error(message('vision:rtmdetObjectDetector:invalidNormalizationStatisticsSize',inputChannelSize));
end
end

function x = iCreateDummyInput(inputSize)
    dims = repmat('S',1,numel(inputSize)-1);
    dims = [dims,'C'];
    x = dlarray(zeros(inputSize),dims);
end

function x = getExampleInputsFromNetwork(net)
    x = getExampleInputs(net); % Populated when a user calls initialize on network without input layer.
    if isempty(x)
        inputSize = net.Layers(1).InputSize;
        x = iCreateDummyInput(inputSize);
    else
        x = iCreateDummyInput(size(x{1},[1 2 3]));
    end
end

function [params, imageSize, networkInputSize] = iValidateImageInput(detector, I)

params.DetectionInputIsDatastore = ~isnumeric(I);

if params.DetectionInputIsDatastore
    sampleImage = vision.internal.cnn.validation.checkDetectionInputDatastore(I, mfilename);
else
    if ndims(I) > 4
        error(message('vision:rtmdetObjectDetector:invalidInputDimensionality'));
    end
    sampleImage = I;    
end
params.CastToGpuArray = ~isgpuarray(sampleImage);

if size(detector.InputSize,2) == 2
    networkInputSize = [detector.InputSize 1];
else
    networkInputSize = detector.InputSize;
end

validateChannelSize = true;  % check if the channel size is equal to that of the network input channel size
validateImageSize   = false; % rtmdet can support images smaller than input size
[imageSize,params.DetectionInputWasBatchOfImages] = vision.internal.cnn.validation.checkDetectionInputImage(...
    networkInputSize,sampleImage,validateChannelSize,validateImageSize);
end

function statsStruct = iDefaultNormalizationStats(inputChannelSize)
if inputChannelSize == 3
    statsStruct = struct("Mean",[123.6750 116.2800 103.5300],"StandardDeviation",[58.3950 57.1200 57.3750]);
else
    avgMean = mean([123.6750 116.2800 103.5300]);
    avgStd = mean([58.3950 57.1200 57.3750]);
    avgMean = repmat(avgMean,1,inputChannelSize);
    avgStd = repmat(avgStd,1,inputChannelSize);
    statsStruct = struct("Mean",avgMean,"StandardDeviation",avgStd);
end
end

function iMustBeValidInputSize(input)         
     if~isempty(input)  
        % Only gray scale and RGB supported 
        isValidChannelDim = (size(input,2)==2 || (size(input,2)==3 && input(3)==3) || (size(input,2)==3 && input(3)==1));
     else
         isValidChannelDim = false;
     end
     
     if ~(isempty(input) || (isValidChannelDim))
        throwAsCaller(MException('vision:rtmdetObjectDetector:incorrectInputSize',...
           vision.getMessage('vision:rtmdetObjectDetector:incorrectInputSize')));
     end
end

function iMustBeValidDetectorName(input) 
    % if detectorName is "uninitialized" return 
    if strcmp(input,"uninitialized")
        return
    end

    % validate detectorName to be a valid user visible detector type
    supportedNetworks = ["large-network-coco","medium-network-coco","small-network-coco","tiny-network-coco"];
    mustBeMember(input, supportedNetworks);    
end

function classes = iGetCOCOClasses()
classes = {'person','bicycle','car','motorbike','aeroplane','bus',...
    'train','truck','boat','traffic light','fire hydrant',...
    'stop sign','parking meter','bench','bird','cat','dog',...
    'horse','sheep','cow','elephant','bear','zebra','giraffe',...
    'backpack','umbrella','handbag','tie','suitcase','frisbee',...
    'skis','snowboard','sports ball','kite','baseball bat',...
    'baseball glove','skateboard','surfboard','tennis racket',...
    'bottle','wine glass','cup','fork','knife','spoon','bowl',...
    'banana','apple','sandwich','orange','broccoli','carrot',...
    'hot dog','pizza','donut','cake','chair','sofa',...
    'pottedplant','bed','diningtable','toilet','tvmonitor',...
    'laptop','mouse','remote','keyboard','cell phone',...
    'microwave','oven','toaster','sink','refrigerator',...
    'book','clock','vase','scissors','teddy bear',...
    'hair drier','toothbrush'};
classes = classes'; 
end