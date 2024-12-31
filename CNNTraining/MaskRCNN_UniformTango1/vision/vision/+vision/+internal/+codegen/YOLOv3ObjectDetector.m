%#codegen

% Copyright 2021-2024 The MathWorks, Inc.

% Codegen class for YOLOv3 Object Detector
classdef YOLOv3ObjectDetector < coder.internal.NetworkWrapper

    properties
        
        % Backbone dlnetwork represented as
        % coder.internal.dlnetwork
        Network

        % AnchorBoxes is a N-by-1 cell array of M-by-2 matrices, where
        % each row defines the [height width] of an anchor box. N is the
        % number of outputs in the network.
        AnchorBoxes

        % ClassNames is a cell array of object class names. These are the
        % object classes that the YOLO v3 detector was trained to find.
        ClassNames

        % InputSize is a M-by-2 matrix defining the [height width] of image
        % sizes used for detection.
        InputSize

        % PredictedBoxType is a string that specifies the type of the bounding
        % boxes returned by the object detector. The type of bounding box a
        % detector returns depends on the training data used to train the
        % object detector. PredictedBoxType is specified as "axis-aligned"
        % for axis-aligned bounding boxes or "rotated" for rotated rectangle
        % bounding boxes.
        PredictedBoxType
    end

    properties (Access = private)
        % LayerSize has the input image size to ImageInputLayer.
        LayerSize

        % LayerIndices A struct that caches indices to certain layers used
        % frequently during detection.
        LayerIndices
    end

    methods(Static, Access = public, Hidden)

        %------------------------------------------------------------------
        % Function to make the properties constant at compile time.
        %------------------------------------------------------------------
        function n = matlabCodegenNontunableProperties(~)
            n = {'LayerSize', 'LayerIndices', 'AnchorBoxes', 'ClassNames', 'InputSize', 'PredictedBoxType'};
        end

        function name = matlabCodegenUserReadableName()
            name = 'yolov3ObjectDetector';
        end

        function n = matlabCodegenNetworkProperties()
            n = {'Network'};
        end
    end

    methods

        %------------------------------------------------------------------
        % YOLOv3ObjectDetector class constructor which loads the network.
        %------------------------------------------------------------------
        function obj = YOLOv3ObjectDetector(matfile, networkName)
            coder.allowpcode('plain');
 
            coder.internal.prefer_const(matfile, networkName);

            % Initialize the detector
            obj = obj@coder.internal.NetworkWrapper(matfile, networkName);
         
            % Get all the network properties.
            coder.extrinsic('vision.internal.codegen.YOLOv3ObjectDetector.getNetworkProperties');
            [obj.LayerSize, obj.AnchorBoxes, obj.ClassNames, obj.InputSize, obj.LayerIndices, obj.PredictedBoxType]...
                = coder.const(@vision.internal.codegen.YOLOv3ObjectDetector.getNetworkProperties,matfile);

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
                [params, detectionInputWasBatchOfImages]  = this.parseDetectInputs(I, roi, useROI, varargin{2:end});

                roiImageSize(1) = roi(4);
                roiImageSize(2) = roi(3);
                if(numel(inputImageSize) == 3)
                    roiImageSize(3) = inputImageSize(3);
                end
            else

                % If roi is not provide initialize as zeros.
                roi = coder.nullcopy(zeros(1,4));

                % params - Threshold, selectStrongest, minsize
                % maxsize
                [params, detectionInputWasBatchOfImages] = this.parseDetectInputs(I, roi, useROI, varargin{:});
                roiImageSize(1) = size(I,1);
                roiImageSize(2) = size(I,2);
                if(numel(inputImageSize) == 3)
                    roiImageSize(3) = size(I,3);
                end
            end

            %% Pre process
            % Extract the roi from image.
            Iroi = vision.internal.detector.cropImageIfRequested(I, roi, useROI);

            % Reshape input to nearest training image size.
            if coder.const(strcmp(params.DetectionPreprocessing, 'auto'))
                Ipreprocessed = dlarray(iPreprocessData(Iroi, inputImageSize), "SSCB");
            else
                Iroi = im2single(Iroi);
                Ipreprocessed = dlarray(Iroi, "SSCB");
            end

            %% Activations
            % Get the activations from network.
            numOutputs = coder.const(numel(this.Network.OutputNames));
            YPredictions = cell(numOutputs, 1);

            [YPredictions{:}] = this.Network.predict(Ipreprocessed);
            predictions = this.yolov3Transform(YPredictions);

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
            className = 'yolov3ObjectDetector';
        end

    end

    methods(Hidden = true, Static)

        %------------------------------------------------------------------
        % Get function to fetch the YOLOv3 Network properties.
        %------------------------------------------------------------------
        function [layerSize, anchors, classNames, inputSize, layerIndices, predictedBoxType] = getNetworkProperties(matfile)
            detectorObj = coder.internal.loadDeepLearningNetwork(matfile);

            externalLayers = detectorObj.Network.Layers;
            layerIndices.ImageLayerIdx = find(...
                arrayfun( @(x)isa(x,'nnet.cnn.layer.ImageInputLayer'), ...
                externalLayers));
            layerSize = detectorObj.Network.Layers(layerIndices.ImageLayerIdx).InputSize;

            anchors = detectorObj.AnchorBoxes;

            % Cast categorical to cellstrs
            labelArray = cellstr(detectorObj.ClassNames);
            % Compute length of all classnames
            lengthArray = cellfun(@strlength,labelArray);
            % maxClasses
            numClasses = numel(labelArray);

            % preallocate 2-D Char array of nClasses x maxLength
            classNames = char(zeros(numClasses, max(lengthArray)));

            % populate char array
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
        % Function to parse the inputs
        %------------------------------------------------------------------
        function [params, detectionInputWasBatchOfImages] = parseDetectInputs(this, I, roi, useROI, varargin)

            %--------------------------------------------------------------
            % Validate Input Image.
            %--------------------------------------------------------------
            detectionInputWasBatchOfImages = coder.const(this.iCheckDetectionInputImage(I));

            possibleNameValues = {'Threshold', ...
                'DetectionPreprocessing', ...
                'MiniBatchSize', ...
                'SelectStrongest',...
                'MinSize', ...
                'MaxSize'};
            poptions = struct( ...
                'CaseSensitivity',false, ...
                'PartialMatching','unique', ...
                'StructExpand',false, ...
                'IgnoreNulls',true);

            % get input size
            inputSize = coder.nullcopy(zeros(1,4));
            [inputSize(1), inputSize(2) ,inputSize(3), inputSize(4)] = size(I);

            defaults =struct('roi', zeros(1,4),...
                'DetectionPreprocessing', 'auto', ...
                'MiniBatchSize', 128, ...
                'useROI', false,...
                'Threshold', 0.5,...
                'SelectStrongest', true, ...
                'MinSize', [1 1],...
                'MaxSize', inputSize(1:2)); % default            

            miniBatchSize = 128;   % default

            if (nargin == 1) % only imageSize
                params =  coder.internal.constantPreservingStruct(...
                    'Threshold', defaults.Threshold,...
                    'SelectStrongest', defaults.SelectStrongest,...
                    'MinSize', defaults.MinSize,...
                    'MaxSize', defaults.MaxSize,...
                    'DetectionPreprocessing', defaults.DetectionPreprocessing,...
                    'MiniBatchSize', defaults.MiniBatchSize,...
                    'roi', roi,...
                    'useROI', useROI);
            else
                pstruct = coder.internal.parseParameterInputs(possibleNameValues, poptions, varargin{:});
                threshold = coder.internal.getParameterValue(pstruct.Threshold, defaults.Threshold, varargin{:});
                selectStrongest = coder.internal.getParameterValue(pstruct.SelectStrongest, defaults.SelectStrongest, varargin{:});
                minSize = coder.internal.getParameterValue(pstruct.MinSize, defaults.MinSize, varargin{:});
                maxSize = coder.internal.getParameterValue(pstruct.MaxSize, defaults.MaxSize, varargin{:});
                detectionPreprocessing = coder.internal.getParameterValue(pstruct.DetectionPreprocessing, defaults.DetectionPreprocessing, varargin{:});
                miniBatchSize = coder.internal.getParameterValue(pstruct.MiniBatchSize, defaults.MiniBatchSize, varargin{:});

                params = coder.internal.constantPreservingStruct(...
                    'Threshold', threshold,...
                    'SelectStrongest', selectStrongest,...
                    'MinSize', minSize,...
                    'MaxSize', maxSize,...
                    'DetectionPreprocessing', detectionPreprocessing,...
                    'MiniBatchSize', miniBatchSize,...
                    'roi', roi,...
                    'useROI', useROI);
            end

            % explicitly constant fold
            coder.internal.assert(coder.internal.isConst(miniBatchSize), ...
                'dlcoder_spkg:ObjectDetector:VariableSizeMiniBatch');

            %--------------------------------------------------------------
            % Validate MiniBatchSize.
            %--------------------------------------------------------------
            vision.internal.cnn.validation.checkMiniBatchSize(coder.const(miniBatchSize), mfilename);

            %--------------------------------------------------------------
            % Validate DetectionPreprocessing
            %--------------------------------------------------------------
            if ~coder.internal.isConst(params.DetectionPreprocessing) || ~coder.const(@(x) strcmpi(x,'none') || strcmpi(x,'auto'), params.DetectionPreprocessing)
                coder.internal.assert(false,'dlcoder_spkg:ObjectDetector:InvalidNVP');
            end

            %--------------------------------------------------------------
            % Validate Select Strongest
            %--------------------------------------------------------------
            vision.internal.inputValidation.validateLogical(...
                params.SelectStrongest, 'SelectStrongest');

            validateMinSize = logical(pstruct.MinSize);
            validateMaxSize = logical(pstruct.MaxSize);

            %--------------------------------------------------------------
            % Validate MinSize
            %--------------------------------------------------------------
            if validateMinSize
                vision.internal.detector.ValidationUtils.checkMinSize(...
                    params.MinSize, [1,1], mfilename);
            end

            %--------------------------------------------------------------
            % Validate Max Size
            %--------------------------------------------------------------
            if validateMaxSize
                vision.internal.detector.ValidationUtils.checkSize(params.MaxSize, 'MaxSize', mfilename);
            end

            if validateMaxSize && validateMinSize
                coder.internal.errorIf(any(params.MinSize >= params.MaxSize) , ...
                    'vision:ObjectDetector:minSizeGTMaxSize');
            end

            %--------------------------------------------------------------
            % Validate ROI
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
            % Validate threshold
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

            networkInputSize = this.LayerSize;
            networkChannelSize = coder.const(networkInputSize(3));
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
        function predictions = yolov3Transform(this, YPredictions)
            
            % Determine if the detector is a rotated rectangle or
            % axis-aligned detector, and set the offset to be used to
            % account for two additional predictions per anchor in the
            % rotated rectangle case.
            [rotationOffset, isRotatedRectangle] = coder.const(@checkPredictedBoxType,this.PredictedBoxType);
            
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
                predictions{ii,4} = coder.internal.layer.elementwiseOperation(@exp, YPredictionsData(:,:,wIds,:),single(1));                

                % Height.
                startIdx = 4;
                hIds = coder.const(startIdx:stride:endIdx);
                predictions{ii,5} = coder.internal.layer.elementwiseOperation(@exp, YPredictionsData(:,:,hIds,:),single(1));                

                if coder.const(~isRotatedRectangle)
                    % Accumulate all the non-class indexes.
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

            numImages = size(predictions{1,1},4);
            bboxes = cell(numImages, 1);
            scores = cell(numImages, 1);
            labels = cell(numImages, 1);

            % Explicitly turn unroll false due to g2187783
            coder.unroll(false);
            for ii = 1:numImages
                extractedPredictionsForImage = iExtractPredictionsForBatchIndex(predictions, ii);
                [bboxes{ii}, scores{ii}, labels{ii}] = this.postprocessSingleDetection(extractedPredictionsForImage,...
                    imageSize,networkInputSize,params,classes,returnLabels);
            end
        end

        %------------------------------------------------------------------
        function [bboxes,scores,labelNames]  = postprocessSingleDetection(this,extractDetections,...
                imageSize,networkInputSize,params,classes,returnLabels)

            coder.internal.prefer_const(returnLabels);
            
            % Determine if the detector is a rotated rectangle or
            % axis-aligned detector, and set the offset to be used to
            % account for two additional predictions per anchor in the
            % rotated rectangle case.
            [rotationOffset, isRotatedRectangle] = coder.const(@checkPredictedBoxType,this.PredictedBoxType);

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

            [bboxes,scores,labelNames] = iPostProcessDetections(detections,classes,params,imageSize,networkInputSize,returnLabels,isRotatedRectangle);
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
    end
end

%--------------------------------------------------------------------------
function image = iPreprocessData(image, targetSize)
% Resize the images and scale the pixels to between 0 and 1.
imgSize = size(image);

% Convert an input image with single channel to 3 channels.
if numel(imgSize) < 3
    image = repmat(image,1,1,3);
end

image = im2single(rescaleData(image));

image = iLetterBoxImage(image,coder.const(targetSize(1:2)));

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
function Inew = iLetterBoxImage(I,targetSize)
% LetterBoxImage returns a resized image by preserving the width and height
% aspect ratio of input Image I. targetSize is a 1-by-2 vector consisting
% the target dimension.
%
% Input I can be uint8, uint16, int16, double, single, or logical, and must
% be real and non-sparse.
coder.internal.prefer_const(targetSize);

[Irow,Icol,Ichannels, IBatchSize] = coder.const(@size,I, 1:4);

% Compute aspect Ratio.
arI = coder.const(Irow./Icol);

% Preserve the maximum dimension based on the aspect ratio.
if (coder.const(arI<1))
    IcolFin = targetSize(1,2);
    IrowFin = coder.const(floor(IcolFin.*arI));
else
    IrowFin = targetSize(1,1);
    IcolFin = coder.const(floor(IrowFin./arI));
end

% Resize the input image.
Itmp = imresize(I,[IrowFin,IcolFin]);


% Compute the offset.
if (coder.const(arI<1))
    buff = coder.const(targetSize(1,1)-IrowFin);
else
    buff = coder.const(targetSize(1,2)-IcolFin);
end

% Place the resized image on the canvas image.
if (coder.const(buff==0))
    Inew = Itmp;
else
    % Initialize Inew with gray values.
    Inew = coder.const(ones([targetSize,Ichannels, IBatchSize],'like',I).*0.5);

    buffVal = coder.const(floor(buff/2));
    if (coder.const(arI<1))
        Inew(buffVal:buffVal+IrowFin-1,:,:,:) = Itmp;
    else
        Inew(:,buffVal:buffVal+IcolFin-1,:,:) = Itmp;
    end
end

end

%--------------------------------------------------------------------------
function [bboxes, scores, labelNames] = iPostProcessDetections(detections,classes,params,inputImageSize,networkInputSize,returnLabels,isRotatedRectangle)

if coder.const(isRotatedRectangle)
    rotationOffset = 2;
else
    rotationOffset = 0;
end

if ~isempty(detections)

    scorePred = detections(:,1);
    bboxTemp = detections(:,2:5+rotationOffset);
    classPred = detections(:,6+rotationOffset);

    if coder.const((strcmp(params.DetectionPreprocessing, 'auto')))
        scale = [networkInputSize(2) networkInputSize(1) networkInputSize(2) networkInputSize(1)];
    else
        scale = [inputImageSize(2) inputImageSize(1) inputImageSize(2) inputImageSize(1)];
    end

    bboxTemp(:,1:4) = bboxTemp(:,1:4).*scale;

    if coder.const(~isRotatedRectangle)
        % Convert x and y position of detections from center to top-left.
        % Resize boxes to image size.
        bboxPred = iConvertCenterToTopLeft(bboxTemp);
    else
        phases = coder.internal.layer.tanh(bboxTemp(:,5:6));
        yaw = rad2deg(atan2(phases(:,1),phases(:,2)));
        bboxPred = [bboxTemp(:,1:4) yaw];
    end

    if coder.const((strcmp(params.DetectionPreprocessing, 'auto')))
        [shiftedBboxes,shiftedImSz] = iDeLetterBoxImage(bboxPred,networkInputSize,inputImageSize);
        bboxPred = iScaleBboxes(shiftedBboxes,inputImageSize,shiftedImSz,isRotatedRectangle);
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

    % Limit detections with respect to the input boundaries.
    if coder.const(~isRotatedRectangle)
        coder.gpu.kernel();
        for i = 1:size(bboxes,1)
            detectionsWd = minFunc(bboxes(i,1) + bboxes(i,3),inputImageSize(1,2));
            bboxes(i,3) = detectionsWd - bboxes(i,1);

            detectionsHt = minFunc(bboxes(i,2) + bboxes(i,4),inputImageSize(1,1));
            bboxes(i,4) = detectionsHt - bboxes(i,2);
        end
    else
        bboxes = clipRotatedBBox(bboxes,inputImageSize(1,2),inputImageSize(1,1));
    end
    
    % Adjust any x, y, w, h to 1 if less than 1. This ignores the angle
    % element of rotated rectangles as the allowed range is (-180,180].
    bboxes(bboxes(:,1:4)<1) = 1;
    
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
        % if PredictedBoxType "axis-aligned", bboxes are M-by-4 numeric matrix as [x y w h]
        % x,y specify upper-left corner and w,h specify width and height of the rectangle 
        bboxes = zeros(0,4,'single');
    else
        % if PredictedBoxType "rotated", bboxes are M-by-5 numeric matrix as [xctr yctr w h yaw]
        % xctr,uctr specify center of the rectangle with width and height
        % as w,h. yaw specified the rotation agnle in degrees 
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
bboxes(:,1) = bboxes(:,1)- bboxes(:,3)/2 + 0.5;
bboxes(:,2) = bboxes(:,2)- bboxes(:,4)/2 + 0.5;
coder.gpu.kernel();
for i = 1:numel(bboxes)
    bboxes(i) = floor(bboxes(i));
    if bboxes(i) < 1
        bboxes(i) = 1;
    end
end
end

%--------------------------------------------------------------------------
function [bboxes,newImSz] = iDeLetterBoxImage(bboxes,inpSz,imgSz)
% DeLetterBoxImage returns the shifted box coordinates by removing the
% gray canvas that is added in LetterBoxImage. The new box coordinates
% are with respect to the resized image obtained before applying the gray
% canvas.
%
% Input bboxes are the bounding boxes in [x,y,w,h] format, computed using
% the letter boxed image.
%
% Input inpSz is the input image size of the network.
%
% Input imgSz is the original image size.

arI = coder.const(imgSz(1,1)./imgSz(1,2));
if coder.const(arI<1)
    IcolFin = inpSz(1,2);
    IrowFin = IcolFin.*arI;

    % Compute the canvas shift.
    buff    = inpSz(1,1)-IrowFin;
    dcShift = buff/2;

    % Update the boxes based on the shift.
    bboxes(:,2) = max(1,(bboxes(:,2) - dcShift));
    newImSz = [IrowFin,IcolFin];
else
    IrowFin = inpSz(1,1);
    IcolFin = IrowFin./arI;

    % Compute the canvas shift.
    buff    = inpSz(1,2)-IcolFin;
    dcShift = buff/2;

    % Update the boxes based on the shift.
    bboxes(:,1) = max(1,(bboxes(:,1) - dcShift));
    newImSz = [IrowFin,IcolFin];
end
end

%--------------------------------------------------------------------------
function bboxPred = iScaleBboxes(bboxes,imSz,newImSz,isRotatedRectangle)

scale = imSz(1:2)./newImSz;

if coder.const(~isRotatedRectangle)
    bboxesX1Y1X2Y2 = xywhToX1Y1X2Y2(bboxes);
    % Saturate X2,Y2 to the original image dimension, to remove the gray scale
    % scale detections if any.
    bboxesX1Y1X2Y2(:,3) = minFunc(bboxesX1Y1X2Y2(:,3),newImSz(1,2));
    bboxesX1Y1X2Y2(:,4) = minFunc(bboxesX1Y1X2Y2(:,4),newImSz(1,1));

    % Scale the boxes to the image dimension.
    bboxesX1Y1X2Y2 = scaleX1X2Y1Y2(bboxesX1Y1X2Y2, scale(2), scale(1));
    bboxPred = x1y1x2y2ToXYWH(bboxesX1Y1X2Y2);
else
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
    bboxPoints(:,1:4) = bboxPoints(:,1:4).* scale(2);
    bboxPoints(:,5:8) = bboxPoints(:,5:8).* scale(1);

    % Convert scaled vertices back to bbox format.
    w = max(hypot(bboxPoints(:,2:3) - bboxPoints(:,[1 4]), ...
        bboxPoints(:,6:7) - bboxPoints(:,[5 8])),[],2);
    h = max(hypot(bboxPoints(:,3:4) - bboxPoints(:,[2 1]), ...
        bboxPoints(:,7:8) - bboxPoints(:,[6 5])),[],2);

    % Rescale the center point in pixel coordinate system
    xCtr = scale(2) * bboxes(:,1);
    yCtr = scale(1) * bboxes(:,2);

    bboxPred = [xCtr yCtr w h bboxes(:,5)];
end
end

%--------------------------------------------------------------------------
% Get the detections in the range minSize to maxSize parameters.
%--------------------------------------------------------------------------
function [bboxes1, scores1, labels1] = iFilterBBoxes(bboxes, scores, labels, minSize, maxSize, isRotatedRectangle)
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
function boxes = xywhToX1Y1X2Y2(boxes)
% Convert [x y w h] box to [x1 y1 x2 y2]. Input and output
% boxes are in pixel coordinates. boxes is an M-by-4
% matrix.
boxes(:,3) = boxes(:,1) + boxes(:,3) - 1;
boxes(:,4) = boxes(:,2) + boxes(:,4) - 1;
end

%--------------------------------------------------------------------------
function scaledBoxes = scaleX1X2Y1Y2(boxes, sx, sy)
% Scale [x1 y1 x2 y2] box in the x-direction by sx and in the
% y-direction by sy.
%
% Scale roi by sx and sy. Input roi is an M-by-4 matrix of M
% ROIs. Each row defines an ROI using the [xmin ymin xmax ymax]
% format in pixel coordinates. The scaled ROIs output in the
% same format. The scaled ROIs are floored and output in pixel
% coordinates.
%
% input  space is u,v output space is x,y sx,sy scale from
% input to output

% convert to spatial coordinates
u1 = boxes(:,1) - 0.5;
u2 = boxes(:,3) + 0.5;
v1 = boxes(:,2) - 0.5;
v2 = boxes(:,4) + 0.5;

% scale
x1 = u1 * sx + (1-sx)/2;
x2 = u2 * sx + (1-sx)/2;
y1 = v1 * sy + (1-sy)/2;
y2 = v2 * sy + (1-sy)/2;

% convert to pixel coordinates
x1 = x1 + 0.5;
x2 = x2 - 0.5;
y1 = y1 + 0.5;
y2 = y2 - 0.5;

scaledBoxes = floor([x1 y1 x2 y2]);
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
function boxes = x1y1x2y2ToXYWH(boxes)
% Convert [x1 y1 x2 y2] boxes into [x y w h] format. Input and
% output boxes are in pixel coordinates. boxes is an M-by-4
% matrix.
boxes(:,3) = boxes(:,3) - boxes(:,1) + 1;
boxes(:,4) = boxes(:,4) - boxes(:,2) + 1;
end

%--------------------------------------------------------------------------
function c = maxFunc(a,b)
c = max(a,b);
end

%--------------------------------------------------------------------------
function c = minFunc(a,b)
c = min(a,b);
end
%-------------------------------------------------------------------------
function extractedPredictionsForImage = iExtractPredictionsForBatchIndex(predictions, batchIndex)
coder.inline('always');

% predict is a cell array of numDetectionHeads x 6, where each element is
% h * w * c * n sized matrix.
extractedPredictionsForImage = cell(size(predictions));
for i = 1:numel(predictions)
    elementVal = predictions{i};
    extractedPredictionsForImage{i} = elementVal(:, :, :, batchIndex);
end
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

%--------------------------------------------------------------------------
function clippedBBox = clipRotatedBBox(bbox, imWidth, imHeight)

clippedBBox = coder.nullcopy(bbox);

if ~isempty(bbox)
    angle = double(bbox(:,5));
    bboxPointsTemp = obtainBBoxPoints(bbox);
    
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
    w = max(hypot(bboxPoints(:,2:3) - bboxPoints(:,[1 4]), ...
        bboxPoints(:,6:7) - bboxPoints(:,[5 8])),[],2);
    h = max(hypot(bboxPoints(:,3:4) - bboxPoints(:,[2 1]), ...
               bboxPoints(:,7:8) - bboxPoints(:,[6 5])),[],2);
    
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
        clippedBBox(~isValidIdx,:) = clipRotatedBBox(clippedBBoxTemp(~isValidIdx,:), imWidth, imHeight);
    else
        clippedBBox = clippedBBoxTemp;
    end
else
    clippedBBox = bbox;
end
end

%--------------------------------------------------------------------------
function bboxPoints = obtainBBoxPoints(bbox)
% Bounding boxes are rotated rectangle bounding boxes.

% cosd and sind require single or double inputs, so the bounding box is
% casted to double and recast back to the original input class after
% calculations.
numBboxes = size(bbox, 1);
bboxDouble = cast(bbox, "double");
pointsDouble = coder.nullcopy(zeros(4, 2, numBboxes, "double"));

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
    bboxPointsTemp = obtainBBoxPoints(bbox);

    % Remap bboxPoints to be an M-by-8 matrix, where M is the number of
    % bounding boxes and each row is specified as [x1 x2 x3 x4 y1 y2 y3 y4].
    bboxPoints = round(reshape(bboxPointsTemp,[8,size(bbox,1)])');
    
    validROI = ...
        all(bboxPoints >= 1 & ...
        [bboxPoints(:,1:4) <= imWidth bboxPoints(:,5:8) <= imHeight],2);
    
    validROI = validROI & bbox(:,3) > 0 & bbox(:,4) > 0;
end
end
%--------------------------------------------------------------------------
function points = bbox2points(bbox)

arguments
        bbox {mustBeNumeric, mustBeReal, mustBeNonempty, mustBeFinite, mustBeNonsparse}
end

validateattributes(bbox(:, [3,4]), {'numeric'}, ...
    {'>=', 0}, 'bbox2points', 'bbox(:,[3,4])');

numBboxes = size(bbox, 1);
sizeBboxes = size(bbox, 2);

if sizeBboxes == 5
    coder.internal.errorIf(~isfloat(bbox), 'vision:bbox:InvalidRotatedDataType');
end

if sizeBboxes == 4
    % Bounding boxes are axis-aligned rectangle bounding boxes.
    points = coder.nullcopy(zeros(4, 2, numBboxes, "like", bbox));

    % upper-left
    points(1, 1, :) = bbox(:, 1);
    points(1, 2, :) = bbox(:, 2);
    
    % upper-right
    points(2, 1, :) = bbox(:, 1) + bbox(:, 3);
    points(2, 2, :) = bbox(:, 2);
    
    % lower-right
    points(3, 1, :) = bbox(:, 1) + bbox(:, 3);
    points(3, 2, :) = bbox(:, 2) + bbox(:, 4);
    
    % lower-left
    points(4, 1, :) = bbox(:, 1);
    points(4, 2, :) = bbox(:, 2) + bbox(:, 4);
else
    % Bounding boxes are rotated rectangle bounding boxes.
    points = coder.nullcopy(zeros(4, 2, numBboxes, 'like', bbox));

    r = bbox(:, 5);
    u = [bbox(:,3)/2 bbox(:,3)/2].* [ cosd(r) sind(r)];
    v = [bbox(:,4)/2 bbox(:,4)/2].* [-sind(r) cosd(r)];

    % upper-left (as seen at 0 degrees of rotation).
    points(1, 1, :) = bbox(:, 1) - u(:, 1) - v(:, 1);
    points(1, 2, :) = bbox(:, 2) - u(:, 2) - v(:, 2);

    % upper-right (as seen at 0 degrees of rotation).
    points(2, 1, :) = bbox(:, 1) + u(:, 1) - v(:, 1);
    points(2, 2, :) = bbox(:, 2) + u(:, 2) - v(:, 2);

    % lower-right (as seen at 0 degrees of rotation).
    points(3, 1, :) = bbox(:, 1) + u(:, 1) + v(:, 1);
    points(3, 2, :) = bbox(:, 2) + u(:, 2) + v(:, 2);

    % lower-left (as seen at 0 degrees of rotation).
    points(4, 1, :) = bbox(:, 1) - u(:, 1) + v(:, 1);
    points(4, 2, :) = bbox(:, 2) - u(:, 2) + v(:, 2);
end
end



