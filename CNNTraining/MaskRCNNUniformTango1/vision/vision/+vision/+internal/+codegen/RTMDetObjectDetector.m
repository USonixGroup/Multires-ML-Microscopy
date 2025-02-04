classdef RTMDetObjectDetector < coder.internal.NetworkWrapper  %#codegen
    % RTMDetObjectDetector Codegen class for rtmdetObjectDetector
    %   Copyright 2024 The MathWorks, Inc.

    properties
        % RTMDet dlnetwork 
        Network

        ModelName

        % ClassNames specifies the names of the classes that RTMDet object
        % detector can detect.
        ClassNames

        % InputSize is a vector of the form [height width] or [height width channels]
        % defining image size used for inference by the detector. During detection,
        % an input image is resized to this size before it is processed by
        % the detection network.
        InputSize
    end

    methods (Hidden, Access = public, Static)
        function n = matlabCodegenNontunableProperties(~)
            n = {'ModelName', 'ClassNames', 'InputSize'};
        end

        function n = matlabCodegenNetworkProperties()
            n = {'Network'};
        end

        function name = matlabCodegenUserReadableName()
            name = 'rtmdetObjectDetector';
        end
    end

    methods
        function obj = RTMDetObjectDetector(matfile, networkName)
            coder.allowpcode('plain');

            coder.internal.prefer_const(matfile, networkName);

            % Initialize the detector
            obj = obj@coder.internal.NetworkWrapper(matfile, networkName);

            % Get all the network properties.
            coder.extrinsic('vision.internal.codegen.RTMDetObjectDetector.getNetworkProperties');
            [obj.ModelName, obj.ClassNames, obj.InputSize] = ...
                coder.const(@feval, 'vision.internal.codegen.RTMDetObjectDetector.getNetworkProperties',matfile);
        end

        function [bboxes,scores,labels] = detect(this, I, roi, options)

             arguments
                this 
                I {mustBeA(I,"numeric"),mustBeNonempty, iMustBeSupportedType, iMustHaveValidDimensionality}
                roi {coder.mustBeConst,mustBeA(roi,"numeric")} =  zeros(1,4);
                options.Threshold  {iCheckThreshold} = 0.25
                options.SelectStrongest {vision.internal.inputValidation.validateLogical(options.SelectStrongest, 'SelectStrongest')} = true
                options.MinSize (1,2) {vision.internal.detector.ValidationUtils.checkMinSize(options.MinSize, [1,1], 'RTMDetObjectDetector')} = [1,1]
                options.MaxSize (1,2) {vision.internal.detector.ValidationUtils.checkSize(options.MaxSize, 'MaxSize', 'RTMDetObjectDetector')} = this.InputSize(1:2)
                options.MiniBatchSize {coder.mustBeConst, vision.internal.cnn.validation.checkMiniBatchSize(options.MiniBatchSize, 'RTMDetObjectDetector')} = 16
                options.ExecutionEnvironment {mustBeTextScalar}
                options.Acceleration {mustBeTextScalar}
                options.AutoResize {coder.mustBeConst, vision.internal.inputValidation.validateLogical(options.AutoResize, 'AutoResize')} = true

             end

            if isfield(options, 'ExecutionEnvironment')
                % If user specifies ExecutionEnvironment throw warning 
                coder.internal.compileWarning(...
                'vision:rtmdetObjectDetector:ignoreInputArg', 'detect', 'ExecutionEnvironment');
            end
                
            if isfield(options, 'Acceleration')
                % If user specifies Acceleration throw warning 
                coder.internal.compileWarning(...
                'vision:rtmdetObjectDetector:ignoreInputArg', 'detect', 'Acceleration');
            end

            nargoutchk(1,3);  

            coder.gpu.internal.kernelfunImpl(false);

            returnLabels = coder.const(nargout > 2);

            detectionInputIsBatchOfImages = coder.const(this.iCheckDetectionInputImage(I));

            % get input size
            imageSize = size(I, 1:4);

            % Make the compile time function calls as extrinsic
            coder.extrinsic('vision.internal.detector.checkROI');

            useROI = false; % Default ROI not to be used
            inputImageSize = this.InputSize; % image size.
            roiImageSize = coder.nullcopy(ones(1, numel(inputImageSize))); 

            if any(roi)
                coder.internal.errorIf(~(isvector(roi) && (size(roi,2) == 4)), ...
                    'vision:rtmdetObjectDetector:roiIncorrectNumel');

                % Check whether ROI is fully contained in image.
                coder.const(feval('vision.internal.detector.checkROI', roi, size(I)));

                % params - Threshold, selectStrongest, minsize
                % maxsize
                useROI = true;

                [params,autoResize] = this.iParseDetectInputs(imageSize,roi,useROI,options);

                roiImageSize(1) = roi(4);
                roiImageSize(2) = roi(3);
                if coder.const((numel(inputImageSize) == 3))
                    roiImageSize(3) = inputImageSize(3);
                end
            else
                % Executed if roi is empty 

                % params - Threshold, selectStrongest, minsize
                % maxsize
                [params,autoResize] = this.iParseDetectInputs(imageSize,roi,useROI,options);

                roiImageSize(1) = size(I,1);
                roiImageSize(2) = size(I,2);

                if coder.const((numel(inputImageSize) == 3))
                    roiImageSize(3) = size(I,3);
                end
            end

            %% Pre process
            
            % Extract the roi from image.
            Iroi = vision.internal.detector.cropImageIfRequested(I, roi, useROI);

            % Compute scale factors to scale boxes from targetSize back to the input size.
            if coder.const(autoResize)
                [resizedImage, scale, padding] = iPreprocessData(Iroi,inputImageSize(1:2));
                sz = size(Iroi);
                [scaleX,scaleY] = deal(1/scale(1),1/scale(2));
                inputImageSize = sz;
            else
                sz = size(Iroi);
                [scaleX,scaleY] = deal(1,1);
                inputImageSize = sz;

                % convert single 
                padding = [0 0 0 0];
                resizedImage = single(Iroi);
            end

            Ipreprocessed = dlarray(resizedImage, "SSCB"); 

            %% Activations and Post Processing
            % Get the activations from network and Post process
            if coder.const(detectionInputIsBatchOfImages)
                [predictions] = this.computeNetworkBatchActivations(Ipreprocessed,autoResize);

                [bboxes, scores, labels] = this.postProcessBatchPredictions(predictions,...
                    roiImageSize,inputImageSize,params,this.ClassNames,returnLabels,scaleX,scaleY,padding,autoResize);
            else
                [predictions] = this.computeNetworkActivations(Ipreprocessed,autoResize);

                [bboxes,scores,labels]  = this.postprocessSingleDetection(predictions,...
                    roiImageSize,inputImageSize,params,this.ClassNames,returnLabels,scaleX,scaleY,padding,autoResize);
            end

        end

        function className = class(~)
            coder.inline('always');
            className = 'rtmdetObjectDetector';
        end
    end


    methods(Hidden = true, Static)
        %------------------------------------------------------------------
        % Get function to fetch the RTMDet Network properties.
        %------------------------------------------------------------------
        function [modelName, classNames, inputSize] = getNetworkProperties(matfile)
            detectorObj = coder.internal.loadDeepLearningNetwork(matfile);

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

            modelName = detectorObj.ModelName;
        end
    end

    methods(Hidden,Access = private)
        function [params,autoResize] = iParseDetectInputs(this,imageSize,roi,useROI,options)
            coder.inline('always');
            coder.internal.prefer_const(imageSize,roi,useROI,options);

            minSize = options.MinSize;
            maxSize = options.MaxSize;

            % Validate MinSize and MaxSize only if they are set to non-default values
            validateMinSize = coder.const(~isequal(minSize,[1 1]));
            validateMaxSize = coder.const(~isequal(maxSize,this.InputSize(1:2)));

            if coder.const(~validateMaxSize)
                maxSize = imageSize(1:2);
            end

            if coder.const(validateMaxSize)
                if useROI
                    if (maxSize(1) > roi(1,4)) && (maxSize(2) > roi(1,3))
                        coder.internal.warning(...
                            'vision:rtmdetObjectDetector:modelMaxSizeGTROISize',...
                            roi(1,4),roi(1,3));
                    end
                else
                    if any(maxSize > imageSize(1:2))
                        coder.internal.warning(...
                            'vision:rtmdetObjectDetector:modelMaxSizeGTImgSize',...
                            imageSize(1,1),imageSize(1,2));
                    end
                end
            end

            if coder.const(validateMaxSize && validateMinSize)
                coder.internal.errorIf(any(minSize >= maxSize) , ...
                    'vision:rtmdetObjectDetector:maxSizeGTMinSize');
            end

            % set output value autoResize
            autoResize = options.AutoResize;

            params.ROI                      = single(roi);
            params.UseROI                   = useROI;
            params.SelectStrongest          = logical(options.SelectStrongest);
            params.MinSize                  = single(minSize);
            params.MaxSize                  = single(maxSize);
            params.MiniBatchSize            = double(options.MiniBatchSize);
            params.Threshold                = single(options.Threshold);
            params.NMSThreshold             = single(0.5); 
    
            % Validate ROI
            if useROI
                if ~isempty(roi)
                    roiSize = roi([4 3]);
                    vision.internal.detector.ValidationUtils.checkImageSizes(roiSize, params, validateMinSize, ...
                        minSize, ...
                        'vision:rtmdetObjectDetector:roiLessThanMinSize', ...
                        'vision:rtmdetObjectDetector:roiLessThanMinSize');
                end
            else
                vision.internal.detector.ValidationUtils.checkImageSizes(imageSize(1:2), params, validateMaxSize, ...
                    minSize , ...
                    'vision:rtmdetObjectDetector:imageLessThanMinSize', ...
                    'vision:rtmdetObjectDetector:imageLessThanMinSize');
            end
        end

        %------------------------------------------------------------------
        function isBatchOfImages = iCheckDetectionInputImage(this, I)
            imSz = size(I, 1:4);

            % Assert for variable sized channel or batch dimensions
            coder.internal.assert(coder.internal.isConst([imSz(3) imSz(4)]), ...
                'vision:rtmdetObjectDetector:variableSizeChannelBatch',mfilename);

            networkInputSize = this.InputSize;
            if numel(networkInputSize) == 3
                networkChannelSize = coder.const(networkInputSize(3));
            else
                networkChannelSize = 1;
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

        %--------------------------------------------------------------------------
        function [predictions] = computeNetworkBatchActivations(this,dlX,autoResize)
            coder.inline('always');
            coder.internal.prefer_const(dlX,autoResize);

            numMiniBatch = size(dlX,4);
            predictions = cell(numMiniBatch,1);
            for ii = 1:numMiniBatch
                inp = dlX(:,:,:,ii);
                predictions{ii,1} = this.computeNetworkActivations(inp,autoResize);
            end
        end

        %--------------------------------------------------------------------------
        function [predictions] = computeNetworkActivations(this,dlX,autoResize)
            coder.inline('always');
            coder.internal.prefer_const(dlX,autoResize);

            numOutputs = coder.const(numel(this.Network.OutputNames));
            YPredictions = cell(numOutputs, 1);

            % Compute network activations for a given input.
            [YPredictions{:}] = this.Network.predict(dlX);

            if autoResize
                [grids, expandedStrides] = computeGridsAndStrides(this.InputSize(1:2));
            else
                [grids, expandedStrides] = computeGridsAndStrides(size(dlX,1:2));
            end

            predictions = this.rtmdetTransform(YPredictions,grids,expandedStrides);
        end

        %------------------------------------------------------------------
        function predictions = rtmdetTransform(~, YPredictions,grids,expandedStrides)
            coder.inline('always');
            coder.internal.prefer_const(YPredictions,grids,expandedStrides);
            % Transforms predictions from dlnetwork to
            % box [x1 y1 x2 y2] and cls scores
            numOutputs = coder.const(numel(YPredictions));
            pred = cell(numOutputs, 1);
            
            % Reshape predictions from model 
            for i = 1:size(YPredictions,1)
                [h,w,c,n] = size(YPredictions{i,1});
                pred{i,1} = permute(reshape(permute(YPredictions{i,1}, [2 1 3 4]), [h*w c n]), [2 1 3]);
            end

            % Concat all Predictions from each head 
            predictions = cat(2,pred{:});
 
            % Compute box coordinates x1,y1,x2,y2
            grids = grids';
            predictions(1,:,:) = (grids(1,:) - predictions(1,:,:)) .* expandedStrides';
            predictions(2,:,:) = (grids(2,:) - predictions(2,:,:)) .* expandedStrides';
            predictions(3,:,:) = (grids(1,:) + predictions(3,:,:)) .* expandedStrides';
            predictions(4,:,:) = (grids(2,:) + predictions(4,:,:)) .* expandedStrides';
        end

        %------------------------------------------------------------------
        function [bboxes, scores, labels] = postProcessBatchPredictions(this,predictions,...
                imageSize,networkInputSize,params,classes,returnLabels,scaleX,scaleY,padding,autoResize)

            coder.internal.prefer_const(returnLabels, imageSize, networkInputSize, classes)

            numImages = size(predictions,1);
            bboxes = cell(numImages, 1);
            scores = cell(numImages, 1);
            labels = cell(numImages, 1);

            for ii = 1:numImages
                [bboxes{ii}, scores{ii}, labels{ii}] = this.postprocessSingleDetection(predictions{ii,1},...
                    imageSize,networkInputSize,params,classes,returnLabels,scaleX,scaleY,padding,autoResize);
            end
        end

        %------------------------------------------------------------------
        function [bboxes,scores,labelNames]  = postprocessSingleDetection(this,predictions,...
                imageSize,networkInputSize,params,classes,returnLabels,scaleX,scaleY,padding,autoResize)

            coder.internal.prefer_const(returnLabels, imageSize, networkInputSize, classes);
            classes = this.ClassNames;

            % The head does not include the final sigmoid needed to map the
            % objectness and class prediction logits into probabilities.
            predictions(5:end,:) = sigmoid(predictions(5:end,:));

            detections = extractdata(predictions)';

            % Apply following post processing steps to filter the detections:
            % * Filter detections based on threshold.

            % Filter the classes based on (confidence score * class probability).
            [classProbs, classIdx] = max(detections(:,5:end),[],2);
            detections(:,5) = classProbs;
            detections(:,6) = classIdx;

            % Keep detections whose objectness score is greater than thresh.
            detectionsKeep = detections(detections(:,5)>=params.Threshold,:);

            [bboxes,scores,labelNames] = iPostProcessDetections(...
                detectionsKeep,classes,params,imageSize,returnLabels,scaleX,scaleY,padding,autoResize);
        end

    end
end

%--------------------------------------------------------------------------
function [output,scale, padding] = iPreprocessData(img, targetSize)
coder.internal.prefer_const(targetSize)

% Compute scaling ratio
imgSize = size(img,1:2);
scaleRatio = min(max(targetSize) / max(imgSize), min(targetSize) / min(imgSize));
scale = [scaleRatio, scaleRatio];

% Resize the image if necessary
if scaleRatio ~= 1
    if scaleRatio < 1
        imgTmp = imresize(img, scaleRatio, 'box', 'Antialiasing', false);
    else
        imgTmp = imresize(img, scaleRatio,'bilinear' , 'Antialiasing', false);
    end
else
    imgTmp = img;
end

% Compute image size and scale ratio
imgSize = size(imgTmp,1:2);
ratio = min(targetSize ./ imgSize);
ratio = min(ratio, 1.0); % Ensure only scaling down
noPadShape = round(imgSize .* ratio);

% Resize to no padding shape if different
if ~isequal(imgSize, noPadShape)
    resizedImg = imresize(imgTmp, noPadShape, 'bilinear', 'Antialiasing', false);
else
    resizedImg = imgTmp;
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
resizedImg = single(resizedImg);

% Apply padding if necessary
output = iPerformPadding(targetSize, resizedImg, top,left, 114);

end


%--------------------------------------------------------------------------
function [Inew] = iPerformPadding(targetSize, I, top,left, padVal)
coder.internal.prefer_const(targetSize, top,left, padVal);
[Irow,Icol,Ichannels, IBatchSize] = size(I,1:4);

% Apply padding 
if (top~=0 || left~=0)
    % Initialize Inew with padVal
    Inew = coder.const(ones([targetSize,Ichannels, IBatchSize],'like',I).*padVal);
    Inew(top+1:top+Irow,left+1:left+Icol,:,:) = I;
else
    Inew = I;  
end
end

%--------------------------------------------------------------------------
function [bboxes, scores, labelNames] = iPostProcessDetections(detections,classes,params,...
    inputImageSize,returnLabels,scaleX,scaleY,padding,autoResize)

if ~isempty(detections)

    scorePred = detections(:,5);
    bboxTemp = detections(:,1:4);
    classPred = detections(:,6);

    if autoResize
        % Shift box coordinates by remove padding 
        pad = [padding(3) padding(1) padding(3) padding(1)];
        bboxTemp = bboxTemp - pad;

        scale = [scaleX scaleY scaleX scaleY];
        bboxPred_ = bboxTemp.*scale;
    else
        bboxPred_ = bboxTemp;
    end

    % Convert boxes from [x1 y1 x2 y2] to [x y w h].
    bboxPred = iConvertX1Y1X2Y2ToXYWH(bboxPred_);

    % Filter boxes based on MinSize, MaxSize.
    [bboxPred, scorePred, classPred] = iFilterBBoxes(bboxPred,scorePred,...
        classPred, params.MinSize,params.MaxSize);

    % Apply NMS.
    if params.SelectStrongest
        [bboxes, scores, labels] = selectStrongestBboxMulticlass(bboxPred, scorePred, classPred ,...
            'RatioType', 'Union', 'OverlapThreshold', params.NMSThreshold);
    else
        bboxes = bboxPred;
        scores = scorePred;
        labels = classPred;
    end

    % Limit height and width detections
    coder.gpu.kernel();
    for i = 1:size(bboxes,1)
        detectionsWd = min(bboxes(i,1) + bboxes(i,3),inputImageSize(1,2));
        bboxes(i,3) = detectionsWd - bboxes(i,1);
        detectionsHt = min(bboxes(i,2) + bboxes(i,4),inputImageSize(1,1));
        bboxes(i,4) = detectionsHt - bboxes(i,2);
    end
    for i = 1:numel(bboxes)
        if bboxes(i) < 1
            bboxes(i) = 1;
        end
    end

    % Apply ROI offset
    bboxes(:,1:2) = vision.internal.detector.addOffsetForROI(bboxes(:,1:2), params.ROI, params.UseROI);

    if returnLabels
        numBBoxes = size(bboxes, 1);
        labelNames = vision.internal.codegen.codegenUtil.returnCategoricalLabels(classes, numBBoxes, labels);
    else
        labelNames = [];
    end

else
    bboxes = zeros(0,4,'single');
    scores = zeros(0,1,'single');
    if returnLabels
        numBBoxes = 0;
        labels = [];
        labelNames = vision.internal.codegen.codegenUtil.returnCategoricalLabels(classes, numBBoxes, labels);
    else
        labelNames = [];
    end
end
end

%--------------------------------------------------------------------------
% Convert [x1 y1 x2 y2] boxes into [x y w h] format.
function bboxes = iConvertX1Y1X2Y2ToXYWH(bboxes)
% Input and output boxes are in pixel coordinates. boxes is an M-by-4
% matrix.
coder.gpu.kernel();
for i = 1:size(bboxes,1)
    bboxes(i,3) = bboxes(i,3) - bboxes(i,1) + 1;
    bboxes(i,4) = bboxes(i,4) - bboxes(i,2) + 1;
end
end

%--------------------------------------------------------------------------
% Get the detections in the range minSize to maxSize parameters.
%--------------------------------------------------------------------------
function [bboxes1, scores1, labels1] = iFilterBBoxes(bboxes, scores, labels, minSize, maxSize)
% Remove boxes smaller than minsize and bigger than maxSize

count = 0;
bboxes1 = coder.nullcopy(zeros(size(bboxes,1), 4, 'like',bboxes));
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
function [grids, expandedStrides] = computeGridsAndStrides(imgSize)
coder.inline('always');
coder.internal.prefer_const(imgSize);
grids = zeros(coder.ignoreConst(0),2);
expandedStrides = zeros(coder.ignoreConst(0),1);

% Define strides.
strides = [8, 16, 32];

% If at least one spatial dimension of the input image exceeds the
% lowest stride by 1, then use floor instead of ceil
for k = coder.unroll(1: numel(strides))
    if mod(imgSize(1),strides(k)) == 1
        hsize = floor(imgSize(1) ./ strides(k));
    else
        hsize = ceil(imgSize(1) ./ strides(k));
    end

    if mod(imgSize(2),strides(k)) == 1
        wsize = floor(imgSize(2) ./ strides(k));
    else
        wsize = ceil(imgSize(2) ./ strides(k));
    end
    stride = strides(k);
    [xv, yv] = meshgrid(0:(wsize-1), 0:(hsize-1));
    grid = reshape(cat(3, xv', yv'),[],2);
    grids = [grids; grid]; %#ok<AGROW>
    shape = size(grid,1);
    expandedStrides = [expandedStrides; ones(shape,1)*stride]; %#ok<AGROW>
end
end

function iMustBeSupportedType(I)
% Check if the input datatype is valid or not.
coder.internal.errorIf(~(isa(I,'uint8') || isa(I,'uint16') || ...
            isa(I,'int16') || isa(I,'double') || isa(I,'single')),...
            'vision:rtmdetObjectDetector:unsupportedInputClassDetect');
end 

function iMustHaveValidDimensionality(I)
% Check if the input dimensions is valid or not.
coder.internal.errorIf(ndims(I) > 4, 'vision:rtmdetObjectDetector:invalidInputDimensionality');
end 

function iCheckThreshold(threshold)
validateattributes(threshold, {'single', 'double'}, {'nonempty', 'nonnan', ...
    'finite', 'nonsparse', 'real', 'scalar', '>=', 0, '<=', 1}, ...
    'RTMDetObjectDetector', 'Threshold');
end
