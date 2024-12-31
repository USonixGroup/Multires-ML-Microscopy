%#codegen

% Copyright 2022-2024 The MathWorks, Inc.

% Codegen class for YOLOv4 Object Detector.
classdef YOLOv4ObjectDetector < coder.internal.NetworkWrapper

    properties

        % Backbone dlnetwork represented as
        % coder.internal.dlnetwork
        Network

        % Anchor boxes is an N-by-1 cell array of M-by-2 matrices, where
        % each row defines the [height width] of an anchor box. N is the
        % number of outputs in the network.
        AnchorBoxes

        % ClassNames is a cell array of object class names. These are the
        % object classes that the YOLO v4 detector was trained to find.
        ClassNames

        % InputSize is a vector of the form [height width] or [height width channels]
        % defining image size used to train the detector.
        InputSize

        % PredictedBoxType is a string that specifies the type of the bounding
        % boxes returned by the object detector. The type of bounding box a
        % detector returns depends on the training data used to train the
        % object detector. PredictedBoxType is specified as "axis-aligned"
        % for axis-aligned bounding boxes or "rotated" for rotated rectangle
        % bounding boxes.
        PredictedBoxType
    end

    methods(Static, Access = public, Hidden)

        %------------------------------------------------------------------
        % Function to make the properties constant at compile time.
        %------------------------------------------------------------------
        function n = matlabCodegenNontunableProperties(~)
            n = {'AnchorBoxes', 'ClassNames', 'InputSize', 'PredictedBoxType'};
        end

        function n = matlabCodegenNetworkProperties()
            n = {'Network'};
        end

        function name = matlabCodegenUserReadableName()
            name = 'yolov4ObjectDetector';
        end
        
    end

    methods

        %------------------------------------------------------------------
        % YOLOv4ObjectDetector class constructor which loads the network.
        %------------------------------------------------------------------
        function obj = YOLOv4ObjectDetector(matfile, networkName)
            coder.allowpcode('plain');

            coder.internal.prefer_const(matfile, networkName);

            % Initialize the detector
            obj = obj@coder.internal.NetworkWrapper(matfile, networkName);

            % Get all the network properties.
            coder.extrinsic('vision.internal.codegen.YOLOv4ObjectDetector.getNetworkProperties');
            [obj.AnchorBoxes, obj.ClassNames, obj.InputSize, obj.PredictedBoxType]...
                = coder.const(@vision.internal.codegen.YOLOv4ObjectDetector.getNetworkProperties,matfile);

        end

        %------------------------------------------------------------------
        % Codegen counterpart of the detect simulation function
        %------------------------------------------------------------------
        % varargout - categorical class labels if nargout = 3
        %             else it is empty
        function [bboxes,scores,varargout] = detect(this, I, varargin)

            coder.gpu.internal.kernelfunImpl(false);
            nargoutchk(1,3);

            returnLabels = coder.const(nargout > 2);    

            % Make the compile time function calls as extrinsic
            coder.extrinsic('vision.internal.detector.checkROI');

            useROI = false; % Default ROI not to be used
            inputImageSize = this.InputSize; % Training image size.
            roiImageSize = coder.nullcopy(ones(1, numel(inputImageSize)));

            if( ~isempty(varargin) && (isa(varargin{1}, 'numeric')) )

                roi = varargin{1};

                % Error out if roi is not a constant.
                coder.internal.assert(coder.internal.isConst(roi), ...
                    'dlcoder_spkg:ObjectDetector:roiConstant')
                coder.internal.errorIf(~(isvector(roi) && (size(roi,2) == 4)), 'dlcoder_spkg:ObjectDetector:roiIncorrectNumel')

                % Check whether ROI is fully contained in image.
                coder.const(feval('vision.internal.detector.checkROI', roi, size(I)));

                % params - Threshold, selectStrongest, minsize
                % maxsize
                useROI = true;
                [params, detectionInputWasBatchOfImages, miniBatchSize]  = this.parseDetectInputs(I, roi, useROI, varargin{2:end});

                roiImageSize(1) = roi(4);
                roiImageSize(2) = roi(3);
                if(numel(inputImageSize) == 3)
                    roiImageSize(3) = inputImageSize(3);
                end
            else

                % If roi is not provided initialize as zeros.
                roi = coder.nullcopy(zeros(1,4));

                % params - Threshold, selectStrongest, minsize
                % maxsize
                [params, detectionInputWasBatchOfImages, miniBatchSize] = this.parseDetectInputs(I, roi, useROI, varargin{:});
                roiImageSize(1) = size(I,1);
                roiImageSize(2) = size(I,2);
                if(numel(inputImageSize) == 3)
                    roiImageSize(3) = size(I,3);
                end
            end

            %% Pre process
            % Extract the roi from image.
            iRoi = vision.internal.detector.cropImageIfRequested(I, roi, useROI);

            iPreprocessed = dlarray(iPreprocessData(iRoi, inputImageSize), "SSCB");

            %% Activations
            % Get the activations from network.
            predictions = this.computeNetworkActivations(iPreprocessed);
            
            %% Post process
            if detectionInputWasBatchOfImages
                [bboxes, scores, varargout{1}] = this.postProcessBatchPredictions(predictions,...
                    roiImageSize,inputImageSize,params,this.ClassNames,returnLabels);
            else
                [bboxes,scores,varargout{1}]  = this.postprocessSingleDetection(predictions,...
                    roiImageSize,inputImageSize,params,this.ClassNames,returnLabels);
            end
        end

        function className = class(~)
            coder.inline('always');
            className = 'yolov4ObjectDetector';
        end
    end

    methods(Hidden = true, Static)

        %------------------------------------------------------------------
        % Get function to fetch the YOLOv4 Network properties.
        %------------------------------------------------------------------
        function [anchors, classNames, inputSize, predictedBoxType] = getNetworkProperties(matfile)
            detectorObj = coder.internal.loadDeepLearningNetwork(matfile);

            % get anchor boxes.
            anchors = detectorObj.AnchorBoxes;

            % cast categorical to cellstrs.
            labelArray = cellstr(detectorObj.ClassNames);
            % compute length of all classnames.
            lengthArray = cellfun(@strlength,labelArray);
            % maxClasses.
            numClasses = numel(labelArray);

            % preallocate 2-D Char array of nClasses x maxLength.
            classNames = char(zeros(numClasses, max(lengthArray)));

            % populate char array.
            for labelIdx = 1: numClasses
                classNames(labelIdx,1:lengthArray(labelIdx)) = labelArray{labelIdx};
            end

            inputSize = detectorObj.InputSize;

            % predictedBoxType can be either 'rotated' or 'axis-aligned'.
            predictedBoxType = detectorObj.PredictedBoxType;
        end

    end

    methods(Access = private)
        %------------------------------------------------------------------
        % Function to parse the inputs.
        %------------------------------------------------------------------
        function [params, detectionInputWasBatchOfImages, miniBatchSize] = parseDetectInputs(this, I, roi, useROI, varargin)

            %--------------------------------------------------------------
            % Validate Input Image.
            %--------------------------------------------------------------
            detectionInputWasBatchOfImages = coder.const(this.iCheckDetectionInputImage(I));

            possibleNameValues = {'Threshold', ...
                'MiniBatchSize', ...
                'SelectStrongest',...
                'MinSize', ...
                'MaxSize', ...
                'ExecutionEnvironment', ...
                'Acceleration'};
            poptions = struct( ...
                'CaseSensitivity',false, ...
                'PartialMatching','unique', ...
                'StructExpand',false, ...
                'IgnoreNulls',true);

            % get input size
            inputSize = coder.nullcopy(zeros(1,4));
            [inputSize(1), inputSize(2) ,inputSize(3), inputSize(4)] = size(I);

            defaults = struct('MiniBatchSize', 128, ...
                'Threshold', 0.5,...
                'SelectStrongest', true, ...
                'MinSize', [1 1],...
                'MaxSize', inputSize(1:2), ...
                'ExecutionEnvironment', 'auto', ...
                'Acceleration', 'auto'); % default

            if (nargin == 1) % only imageSize
                params = defaults;
            else
                pstruct = coder.internal.parseParameterInputs(possibleNameValues, poptions, varargin{:});
                params.Threshold = coder.internal.getParameterValue(pstruct.Threshold, defaults.Threshold, varargin{:});
                params.SelectStrongest = coder.internal.getParameterValue(pstruct.SelectStrongest, defaults.SelectStrongest, varargin{:});
                params.MinSize = coder.internal.getParameterValue(pstruct.MinSize, defaults.MinSize, varargin{:});
                params.MaxSize = coder.internal.getParameterValue(pstruct.MaxSize, defaults.MaxSize, varargin{:});
                miniBatchSize = coder.internal.getParameterValue(pstruct.MiniBatchSize, defaults.MiniBatchSize, varargin{:});
                params.ExecutionEnvironment = coder.internal.getParameterValue(pstruct.ExecutionEnvironment, defaults.ExecutionEnvironment, varargin{:});
                params.Acceleration = coder.internal.getParameterValue(pstruct.Acceleration, defaults.Acceleration, varargin{:});
            end
            params.roi = roi;
            params.useROI = useROI;

            % explicitly constant fold
            coder.internal.assert(coder.internal.isConst(miniBatchSize), ...
                'dlcoder_spkg:ObjectDetector:VariableSizeMiniBatch');

            %--------------------------------------------------------------
            % Validate MiniBatchSize.
            %--------------------------------------------------------------
            vision.internal.cnn.validation.checkMiniBatchSize(coder.const(miniBatchSize), mfilename);

            %--------------------------------------------------------------
            % Ignore ExecutionEnvironment.
            %--------------------------------------------------------------
            if logical(pstruct.ExecutionEnvironment)
                 coder.internal.compileWarning(...
                    'dlcoder_spkg:cnncodegen:IgnoreInputArg', 'detect', 'ExecutionEnvironment');
            end

            %--------------------------------------------------------------
            % Ignore Acceleration.
            %--------------------------------------------------------------
            if logical(pstruct.Acceleration)
                coder.internal.compileWarning(...
                    'dlcoder_spkg:cnncodegen:IgnoreInputArg', 'detect', 'Acceleration');
            end

            %--------------------------------------------------------------
            % Validate Select Strongest.
            %--------------------------------------------------------------
            vision.internal.inputValidation.validateLogical(...
                params.SelectStrongest, 'SelectStrongest');

            validateMinSize = logical(pstruct.MinSize);
            validateMaxSize = logical(pstruct.MaxSize);

            %--------------------------------------------------------------
            % Validate MinSize.
            %--------------------------------------------------------------
            if validateMinSize
                vision.internal.detector.ValidationUtils.checkMinSize(...
                    params.MinSize, [1,1], mfilename);
            end

            %--------------------------------------------------------------
            % Validate Max Size.
            %--------------------------------------------------------------
            if validateMaxSize
                vision.internal.detector.ValidationUtils.checkSize(params.MaxSize, 'MaxSize', mfilename);
            end

            if validateMaxSize && validateMinSize
                coder.internal.errorIf(any(params.MinSize >= params.MaxSize) , ...
                    'vision:ObjectDetector:minSizeGTMaxSize');
            end

            %--------------------------------------------------------------
            % Validate ROI.
            %--------------------------------------------------------------
            if params.useROI
                inputSize = params.roi([4 3]);
                vision.internal.detector.ValidationUtils.checkImageSizes(inputSize(1:2), params, validateMinSize, ...
                    params.MinSize, ...
                    'vision:ObjectDetector:ROILessThanMinSize', ...
                    'vision:ObjectDetector:ROILessThanMinSize');
            else
                vision.internal.detector.ValidationUtils.checkImageSizes(inputSize(1:2), params, validateMaxSize, ...
                    params.MinSize , ...
                    'vision:ObjectDetector:ImageLessThanMinSize', ...
                    'vision:ObjectDetector:ImageLessThanMinSize');
            end

            %--------------------------------------------------------------
            % Validate threshold.
            %--------------------------------------------------------------
            validateattributes(params.Threshold, {'single', 'double'}, {'nonempty', 'nonnan', ...
                'finite', 'nonsparse', 'real', 'scalar', '>=', 0, '<=', 1}, ...
                mfilename, 'Threshold');
        end

        %------------------------------------------------------------------
        function isBatchOfImages = iCheckDetectionInputImage(this, I)

            imSz = coder.nullcopy(zeros(1,4));
            [imSz(1),imSz(2),imSz(3),imSz(4)] = size(I);

            % Assert for variable sized channel or batch dimensions
            coder.internal.assert(coder.internal.isConst([imSz(3) imSz(4)]), ...
                'dlcoder_spkg:ObjectDetector:VariableSizeChannelBatch',mfilename);            

            networkInputSize = this.InputSize;
            if numel(networkInputSize) == 3
                networkChannelSize = coder.const(networkInputSize(3));
            else
                networkChannelSize = coder.const(1);
            end
            imageChannelSize = coder.const(imSz(3));

            isBatchOfImages =  coder.const(imSz(4) > 1);

            if isBatchOfImages
                % Pass the first image in the batch for internal validation.
                Itmp = I(:,:,:,1);
            else
                Itmp = I;
            end

            % multi-channel or grayscale or RGB images allowed
            if coder.const(networkChannelSize > 3 || networkChannelSize == 2)
                vision.internal.inputValidation.validateImage(Itmp, 'I', 'multi-channel');
            else
                vision.internal.inputValidation.validateImage(Itmp, 'I');
            end

            % Validate number of channels for input image and network input
            coder.internal.errorIf(imageChannelSize ~= networkChannelSize, ...
                'vision:ObjectDetector:invalidInputImageChannelSize', ...
                imageChannelSize, ...
                networkChannelSize);
        end

        %------------------------------------------------------------------
        function predictions = yolov4Transform(this, YPredictions)

            % Determine if the detector is a rotated rectangle or
            % axis-aligned detector, and set the offset to be used to
            % account for two additional predictions per anchor in the
            % rotated rectangle case.
            [rotationOffset, isRotatedRectangle] = checkPredictedBoxType(this.PredictedBoxType);

            numPredictionHeads = coder.const(size(YPredictions, 1));
            predictions = cell(numPredictionHeads,6+rotationOffset);
   
            % Unrolling the following loop to improve performance.
            coder.unroll();
            for ii = 1:numPredictionHeads
                % Get the required info on feature size.
                numChannelsPred = coder.const(size(YPredictions{ii},3));
                numAnchors = coder.const(size(this.AnchorBoxes{ii},1));
                numPredElemsPerAnchors = coder.const(numChannelsPred/numAnchors);
                allIds = coder.const(1:numChannelsPred);

                stride = coder.const(numPredElemsPerAnchors);
                endIdx = coder.const(numChannelsPred);

                YPredictionsData = extractdata(YPredictions{ii});

                % Confidence scores.
                startIdx = 5 + rotationOffset;
                confIds = coder.const(startIdx:stride:endIdx); 
                predictions{ii,1} = coder.internal.layer.sigmoid(YPredictionsData(:,:,confIds,:));

                % X positions.
                startIdx = 1;
                xIds = coder.const(startIdx:stride:endIdx);
                predictions{ii,2} = coder.internal.layer.sigmoid(YPredictionsData(:,:,xIds,:));

                % Y positions.
                startIdx = 2;
                yIds = coder.const(startIdx:stride:endIdx);
                predictions{ii,3} = coder.internal.layer.sigmoid(YPredictionsData(:,:,yIds,:));

                % Width.
                startIdx = 3;
                wIds = coder.const(startIdx:stride:endIdx);
                predictions{ii,4} = coder.internal.layer.elementwiseOperationInPlace(@exp, YPredictionsData(:,:,wIds,:));

                % Height.
                startIdx = 4;
                hIds = coder.const(startIdx:stride:endIdx);
                predictions{ii,5} = coder.internal.layer.elementwiseOperationInPlace(@exp, YPredictionsData(:,:,hIds,:));

                if coder.const(~isRotatedRectangle)
                    % Accumulate all the non-class indexes
                    nonClassIds = coder.const([xIds yIds wIds hIds confIds]);
                else
                    % Sin angle.
                    startIdx = 5;
                    sinIds = coder.const(startIdx:stride:endIdx);
                    predictions{ii,6} = coder.internal.layer.tanh(YPredictionsData(:,:,sinIds,:));
                    
                    % Cos angle.
                    startIdx = 6;
                    cosIds = coder.const(startIdx:stride:endIdx);
                    predictions{ii,7} = coder.internal.layer.tanh(YPredictionsData(:,:,cosIds,:));

                    % Accumulate all the non-class indexes
                    nonClassIds = coder.const([xIds yIds wIds hIds sinIds cosIds confIds]);
                end

                
                % Class probabilities.
                % Get the indexes which do not belong to the nonClassIds
                classIdx = setdiff(allIds, nonClassIds, 'stable');
                predictions{ii,6+rotationOffset} = coder.internal.layer.sigmoid(YPredictionsData(:,:,classIdx,:));
            end
        end

        %------------------------------------------------------------------
        function [bboxes, scores, labels] = postProcessBatchPredictions(this,predictions,...
                imageSize,networkInputSize,params,classes,returnLabels)

            coder.internal.prefer_const(returnLabels, imageSize, networkInputSize, classes)

            % Determine if the detector is a rotated rectangle or
            % axis-aligned detector, and set the offset to be used to
            % account for two additional predictions per anchor in the
            % rotated rectangle case.
            rotationOffset = checkPredictedBoxType(this.PredictedBoxType);

            numImages = size(predictions{1,1},4);
            numNetworkOutputs = size(predictions,1); 
            bboxes = cell(numImages, 1);
            scores = cell(numImages, 1);
            labels = cell(numImages, 1);

            for ii = 1:numImages
                fmap = cell(numNetworkOutputs,6+rotationOffset);
                for i = 1:6+rotationOffset
                    for j = 1:numNetworkOutputs
                        feature = predictions{j,i};
                        fmap{j,i} = feature(:,:,:,ii);
                    end
                end
                [bboxes{ii}, scores{ii}, labels{ii}] = this.postprocessSingleDetection(fmap,...
                    imageSize,networkInputSize,params,classes,returnLabels);
            end
        end

        %------------------------------------------------------------------
        function [bboxes,scores,labelNames]  = postprocessSingleDetection(this,extractDetections,...
                imageSize,networkInputSize,params,classes,returnLabels)

            coder.internal.prefer_const(returnLabels, imageSize, networkInputSize, classes);

            % Determine if the detector is a rotated rectangle or
            % axis-aligned detector, and set the offset to be used to
            % account for two additional predictions per anchor in the
            % rotated rectangle case.
            [rotationOffset, isRotatedRectangle] = checkPredictedBoxType(this.PredictedBoxType);
            
            detectionsCell = this.anchorBoxGenerator(extractDetections, networkInputSize);

            % Apply following post processing steps to filter the detections:
            % * Filter detections based on threshold.
            % * Convert bboxes from spatial to pixel dimension.

            % Combine the prediction from different heads.
            numCells = size(detectionsCell, 1);
            detectionSize = coder.nullcopy(zeros(1, numCells));
            for iCell = 1:numCells
                detectionSize(iCell) = numel(detectionsCell{iCell,1});
            end
            detectionSizeIndx = [0 cumsum(detectionSize)];
            predSize = 5 + rotationOffset + size(classes,1);
            detections = coder.nullcopy(zeros(detectionSizeIndx(end), predSize,'single'));
            for iCell = 1:numCells
                for iCol = 1:5+rotationOffset
                    detections(detectionSizeIndx(iCell)+1:detectionSizeIndx(iCell+1),iCol) = reshapePredictions(detectionsCell{iCell, iCol});
                end
                detections(detectionSizeIndx(iCell)+1:detectionSizeIndx(iCell+1),6+rotationOffset:end) = reshapeClasses(detectionsCell{iCell, 6+rotationOffset}, size(classes,1));
            end

            % Filter the classes based on (confidence score * class probability).
            [classProbs, classIdx] = max(detections(:,6+rotationOffset:end),[],2);
            detections(:,1) = detections(:,1).*classProbs;
            detections(:,6+rotationOffset) = classIdx;

            % Keep detections whose objectness score is greater than thresh.
            detections = detections(detections(:,1)>=params.Threshold,:);

            [bboxes,scores,labelNames] = iPostProcessDetections(detections,classes,params,imageSize,returnLabels,isRotatedRectangle);
        end

        %--------------------------------------------------------------------------
        function YPredCell = anchorBoxGenerator(this,YPredCell,inputImageSize)
            % Generate tiled anchor offset.
            anchorIndex = 2:5; % indices corresponding to x,y,w,h predictions for bounding boxes
            numPredictionHeads = size(YPredCell,1);
            tiledAnchors = cell(numPredictionHeads, size(anchorIndex, 2));
            for i=1:numPredictionHeads
                anchors = this.AnchorBoxes{i};
                [h,w,~,n] = size(YPredCell{i,1});
                [tiledAnchors{i,2}, tiledAnchors{i,1}] = ndgrid(0:h-1,0:w-1,1:size(anchors,1),1:n);
                [~,~,tiledAnchors{i,3}] = ndgrid(0:h-1,0:w-1,anchors(:,2),1:n);
                [~,~,tiledAnchors{i,4}] = ndgrid(0:h-1,0:w-1,anchors(:,1),1:n);
            end

            % Convert grid cell coordinates to box coordinates.
            for i=1:size(YPredCell,1)
                [h,w,~,~] = size(YPredCell{i,1});
                YPredCell{i,anchorIndex(1)} = (tiledAnchors{i,1}+YPredCell{i,anchorIndex(1)})./w;
                YPredCell{i,anchorIndex(2)} = (tiledAnchors{i,2}+YPredCell{i,anchorIndex(2)})./h;
                YPredCell{i,anchorIndex(3)} = (tiledAnchors{i,3}.*YPredCell{i,anchorIndex(3)})./inputImageSize(2);
                YPredCell{i,anchorIndex(4)} = (tiledAnchors{i,4}.*YPredCell{i,anchorIndex(4)})./inputImageSize(1);
            end
        end

        %--------------------------------------------------------------------------
        function predictions = computeNetworkActivations(this, I)
            % Compute network activations for a given input.
            NumOutputs = coder.const(numel(this.Network.OutputNames));
            YPredictions = cell(NumOutputs, 1);
            
            [YPredictions{:}] = this.Network.predict(I);
            predictions = this.yolov4Transform(YPredictions);
        end
    end
end

%--------------------------------------------------------------------------
function image = iPreprocessData(image, targetSize)
% Resize the images and scale the pixels to between 0 and 1.
image = imresize(image, targetSize(1:2)); 
image = single(rescaleData(image));
end

%--------------------------------------------------------------------------
% Normalize the image to [0 1]
%--------------------------------------------------------------------------
function resImg = rescaleData(I)
coder.gpu.internal.kernelfunImpl(false);

% Call gpucoder.reduce to compute min & max at a time, g1956916
if coder.gpu.internal.isGpuEnabled % GPU Targets
    outVal = gpucoder.reduce(I(:),{@minFunc, @maxFunc});
    minVal = single(outVal(1));
    maxVal = single(outVal(2));
else % CPU Targets
    minVal = single(min(I(:)));
    maxVal = single(max(I(:)));
end

I = single(I);

resImg = (I  - minVal)./(maxVal - minVal);
end

%--------------------------------------------------------------------------
function [bboxes, scores, labelNames] = iPostProcessDetections(detections,classes,params,inputImageSize,returnLabels,isRotatedRectangle)

if coder.const(isRotatedRectangle)
    rotationOffset = 2;
else
    rotationOffset = 0;
end

if ~isempty(detections)

    scorePred = detections(:,1);
    bboxTemp = detections(:,2:5+rotationOffset);
    classPred = detections(:,6+rotationOffset);

    % Obtain boxes for preprocessed image.
    scale = [inputImageSize(2) inputImageSize(1) inputImageSize(2) inputImageSize(1)];
    bboxTemp(:,1:4) = bsxfun(@times, scale, bboxTemp(:,1:4));

    if coder.const(~isRotatedRectangle)
        % Convert x and y position of detections from center to top-left.
        % Resize boxes to image size.
        bboxPred = iConvertCenterToTopLeft(bboxTemp);
    else
        phases = coder.internal.layer.tanh(bboxTemp(:,5:6));
        yaw = rad2deg(atan2(phases(:,1),phases(:,2)));
        bboxPred = [bboxTemp(:,1:4) yaw];
    end

    % Filter boxes based on MinSize, MaxSize.
    [bboxPred, scorePred, classPred] = iFilterBBoxes(bboxPred,scorePred,...
        classPred, params.MinSize,params.MaxSize,isRotatedRectangle);

    % Apply NMS.
    if params.SelectStrongest
        [bboxes, scores, labels] = selectStrongestBboxMulticlass(bboxPred, scorePred, classPred ,...
            'RatioType', 'Union', 'OverlapThreshold', 0.5);
    else
        bboxes = bboxPred;
        scores = scorePred;
        labels = classPred;
    end

    % Limit height and width detections
    if coder.const(~isRotatedRectangle)
        coder.gpu.kernel();
        for i = 1:size(bboxes,1)
            detectionsWd = min(bboxes(i,1) + bboxes(i,3),inputImageSize(1,2));
            bboxes(i,3) = detectionsWd - bboxes(i,1);
            detectionsHt = min(bboxes(i,2) + bboxes(i,4),inputImageSize(1,1));
            bboxes(i,4) = detectionsHt - bboxes(i,2);

            % Adjust any x, y, w, h to 1 if less than 1.
            bboxes(bboxes(i,1:4)<1) = 1;
        end
    else
        % Clamp center coordinates to image boundaries.
        bboxes(:,1) = max( min( bboxes(:,1), inputImageSize(1,2)  ), 1 );
        bboxes(:,2) = max( min( bboxes(:,2), inputImageSize(1,1) ), 1 );

        % Adjust any width and height to 1 if less than 1.
        bboxes(bboxes(:,3:4)<1) = 1;
    end

    % Apply ROI offset
    bboxes(:,1:2) = vision.internal.detector.addOffsetForROI(bboxes(:,1:2), params.roi, params.useROI);

    if returnLabels
        numBBoxes = size(bboxes, 1);
        labelNames = returnCategoricalLabels(classes, numBBoxes, labels);
    else
        labelNames = [];
    end

else
    if coder.const(~isRotatedRectangle)
        bboxes = zeros(0,4,'single');
    else
        bboxes = zeros(0,5,'single');
    end
    
    scores = zeros(0,1,'single');
    if returnLabels
        numBBoxes = 0;
        labels = [];
        labelNames = returnCategoricalLabels(classes, numBBoxes, labels);
    else
        labelNames = [];
    end
end
end

%--------------------------------------------------------------------------
function x = reshapePredictions(pred)
[h,w,c,n] = size(pred);
x = reshape(pred,h*w*c,1,n);
end

%--------------------------------------------------------------------------
function x = reshapeClasses(pred,numclasses)
[h,w,c,n] = size(pred);
numanchors = (c/numclasses);
x = reshape(pred,h*w,numclasses,numanchors,n);
x = permute(x,[1,3,2,4]);
[h,w,c,n] = size(x);
x = reshape(x,h*w,c,n);
end

%--------------------------------------------------------------------------
% Convert x and y position of detections from center to top-left.
function bboxes = iConvertCenterToTopLeft(bboxes)
coder.gpu.kernel();
for i = 1:size(bboxes,1)
    bboxes(i,1) = bboxes(i,1)- bboxes(i,3)/2 + 0.5;
    bboxes(i,2) = bboxes(i,2)- bboxes(i,4)/2 + 0.5;
end
for i = 1:numel(bboxes)
    if bboxes(i) < 1
        bboxes(i) = 1;
    end
end
end

%--------------------------------------------------------------------------
% Get the detections in the range minSize to maxSize parameters.
%--------------------------------------------------------------------------
function [bboxes1, scores1, labels1] = iFilterBBoxes(bboxes, scores, labels, minSize, maxSize,isRotatedRectangle)
% Remove boxes smaller than minsize and bigger than maxSize

count = 0;

if coder.const(~isRotatedRectangle)
    bboxes1 = coder.nullcopy(zeros(size(bboxes,1), 4, 'like',bboxes));
else
    bboxes1 = coder.nullcopy(zeros(size(bboxes,1), 5, 'like',bboxes));
end
scores1 = coder.nullcopy(zeros(size(bboxes,1), 1, 'like',scores));
labels1 = coder.nullcopy(zeros(size(bboxes,1), 1, 'like',labels));
for i=1:size(bboxes,1)
    if( bboxes(i,4) >= minSize(1) && bboxes(i,3) >= minSize(2) && bboxes(i,4) <= maxSize(1) && bboxes(i,3) <= maxSize(2) )
        count = count+1;
        bboxes1(count,:) = bboxes(i,:);
        scores1(count) = scores(i);
        labels1(count) = labels(i);

    end
end

bboxes1(count+1:end,:) = [];
scores1(count+1:end) = [];
labels1(count+1:end) = [];

end

%--------------------------------------------------------------------------
function labelNames = returnCategoricalLabels(classNames, numBBoxes, labels)
coder.inline('never');
% do not generate CUDA code for this function

% get the class name of each bounding box detected.
labelCells = coder.nullcopy(cell(numBBoxes, 1));
for i=1:numBBoxes
    % transpose to write row vector of char array
    labelCells{i, 1} = nonzeros(classNames(labels(i),:))';
end

% Grow cell array of valueset dynamically.
% Maintaining a varsized labelCells avoids the issue of
% bloating of IR causing slow code-generation for
% large const fixed-sized structs.
% Refer: g2286665
valueset = {};
upperBound = size(classNames,1);
coder.varsize('valueset',[1 upperBound],[0 1]);
for i = 1:upperBound
    valueset{end + 1} = nonzeros(classNames(i,:))';
end

labelNames = categorical(labelCells, valueset);
end

%--------------------------------------------------------------------------
function c = maxFunc(a,b)
c = max(a,b);
end

%--------------------------------------------------------------------------
function c = minFunc(a,b)
c = min(a,b);
end

%--------------------------------------------------------------------------
function [offset, rotated] = checkPredictedBoxType(predictedBoxType)
if coder.const(strcmp(predictedBoxType,"rotated"))
    offset = 2;
    rotated = true;
else
    offset = 0;
    rotated = false;
end
end
