%#codegen

% Copyright 2018-2024 The MathWorks, Inc.

% Codegen class for YOLOv2 Object Detector
classdef YOLOv2Network < coder.internal.NetworkWrapper

    properties

        % Backbone dlnetwork represented as
        % coder.internal.DeepLearningNetwork
        Network

        % LayerSize has the inpu image size to ImageInputLayer.
        LayerSize

        % AnchorBoxes is an M-by-2 matrix defining the [height width] of M
        % anchor boxes.
        AnchorBoxes

        % ClassNames is a cell array of object class names. These are the
        % object classes that the YOLO v2 detector was trained to find.
        ClassNames

        % An M-by-2 matrix defining the [height width] of image sizes used
        % to train the detector. During detection, an input image is
        % resized to nearest TrainingImageSize before it is processed by
        % the detection network.
        TrainingImageSize

        % LayerIndices A struct that caches indices to certain layers used
        % frequently during detection.
        LayerIndices

        % OutputLayerName has the name of YOLOV2OutputLayer of network.
        OutputLayerName

        % FractionDownsampling can either be true or false. If false, it
        % applies floor operation to the downsampling factor. It is set to
        % true by default.
        FractionDownsampling
        
        % WH2HW can either be true or false. If true,
        % iPostProcessActivations function computes
        % bounding box dimensions using anchor boxes specified in [width,
        % height] format.
        WH2HW
    end

    methods(Static, Access = public, Hidden)

        %------------------------------------------------------------------
        % Function to make the properties constant at compile time.
        %------------------------------------------------------------------
        function n = matlabCodegenNontunableProperties(~)
            n = {'LayerSize', 'AnchorBoxes', ...
                 'TrainingImageSize', 'ClassNames', 'FractionDownsampling', 'WH2HW'};
        end
        
        % this method returning true indicates that the class is opting out
        % of the classGeneration feature
        function optOut = matlabCodegenLowerToStruct(~)
            optOut = true;
        end

        function name = matlabCodegenUserReadableName()
            name = 'yolov2ObjectDetector';
        end

        function n = matlabCodegenNetworkProperties()
            n = {'Network'};
        end
    end

    methods

        %------------------------------------------------------------------
        % YOLOv2Network class constructor which loads the network.
        %------------------------------------------------------------------
        function obj = YOLOv2Network(matfile, networkName)
            coder.allowpcode('plain');
            coder.internal.prefer_const(matfile, networkName);

            % Initialize the detector
            obj = obj@coder.internal.NetworkWrapper(matfile, networkName);
                        
            % Get all the network properties.
            coder.extrinsic('vision.internal.codegen.YOLOv2Network.getNetworkProperties');

            resultStruct...
                = coder.const(@vision.internal.codegen.YOLOv2Network.getNetworkProperties,matfile);

            obj.LayerSize = resultStruct.LayerSize;
            obj.AnchorBoxes = resultStruct.AnchorBoxes;
            obj.ClassNames = resultStruct.ClassNames;
            obj.TrainingImageSize = resultStruct.TrainingImageSize;
            obj.LayerIndices = resultStruct.LayerIndices;
            obj.FractionDownsampling = resultStruct.FractionDownsampling;
            obj.WH2HW = resultStruct.WH2HW;


        end

        %------------------------------------------------------------------
        % Codegen counterpart of the detect simulation function
        %------------------------------------------------------------------
        % varargout - categorical class labels if nargout = 3
        %             else it is empty
        function [bboxes, scores, varargout] = detect(this, I, varargin)

            coder.gpu.internal.kernelfunImpl(false);
            nargoutchk(1,3);

            returnLabels = coder.const(nargout > 2);
            
            % Make the compile time function calls as extrinsic
            coder.extrinsic('vision.internal.detector.checkROI');

            useROI = false; % Default ROI not to be used
            if( ~isempty(varargin) && (isa(varargin{1}, 'numeric')) )

                if( isvector(varargin{1}) && (size(varargin{1},2) == 4) ) % check roi has 4 elements.

                    % roi must be in the format of [x y w h].
                    roi = varargin{1};

                    % Error out if roi is not a constant.
                    coder.internal.assert(coder.internal.isConst(roi), ...
                                          'dlcoder_spkg:ObjectDetector:roiConstant')
                    useROI = true;

                    % params - Threshold, selectStrongest, minsize
                    % maxsize
                    [params, DetectionInputWasBatchOfImages, MiniBatchSize]  = this.parseDetectInputs(I, roi, useROI, varargin{2:end});

                    % Check whether ROI is fully contained in image.
                    coder.const(feval('vision.internal.detector.checkROI', roi, size(I)));
               
                    roiImageSize = coder.const(roi([4 3])); % 4th index corresponds to height, 3rd dimension corresponds to width
                else

                    % Error out if roi doesn't contain 4 elements.
                    coder.internal.errorIf(true, 'dlcoder_spkg:ObjectDetector:roiIncorrectNumel')
                end
            else

                % If roi is not provied intialize as zeros.
                roi = zeros(1,4);
           
                % params - Threshold, selectStrongest, minsize
                % maxsize
                [params, DetectionInputWasBatchOfImages, MiniBatchSize] = this.parseDetectInputs(I, roi, useROI, varargin{:});
                roiImageSize = coder.const(size(I,1:2));
            end

            %% Pre-process
            
            % Extract the roi from image.
            Iroi = vision.internal.detector.cropImageIfRequested(I, roi, useROI);
            % Training image size. 
            trainImageSize = this.TrainingImageSize; 
            % Find the nearest training image size to input image size.
            preprocessedImageSize = coder.const(iComputeBestMatch(roiImageSize,trainImageSize));
            
            % Reshape input to nearest training image size.
            if coder.const ( ( (roiImageSize(1) == preprocessedImageSize(1)) && (roiImageSize(2) == preprocessedImageSize(2))) )
                Ipreprocessed = Iroi;
            else
                Ipreprocessed = imresize(Iroi, preprocessedImageSize);
            end
            
            % Normalize the image in the range [0,1].
            im = dlarray(this.rescaleData(Ipreprocessed),"SSCB");

            %% Activations

            % Get the activations from network. Size of the tmpFeatureMap :
            % numgridsH X numGridsW X numAnchors*boxProperties
            tmpFeatureMap = this.Network.predict(im);
            tmpFeatureMap = extractdata(tmpFeatureMap);


            % Compute scale factors.
            sy = coder.const(roiImageSize(1)./preprocessedImageSize(1));
            sx = coder.const(roiImageSize(2)./preprocessedImageSize(2));
            
            %% Post process
            if DetectionInputWasBatchOfImages
                [bboxes, scores, varargout{1}] = this.iPostProcessBatchPredictions(tmpFeatureMap, ...
                    preprocessedImageSize, params, roi, useROI, sx, sy, returnLabels);
            else
                [bboxes, scores, varargout{1}] = this.iPostProcessActivations(tmpFeatureMap, ...
                    preprocessedImageSize, params, roi, useROI, sx, sy, returnLabels);
            end


        end

        function className = class(~)
            coder.inline('always');
            className = 'yolov2ObjectDetector';
        end

    end

    methods(Hidden = true, Static)

        %------------------------------------------------------------------
        % Get function to fetch the YOLOv2 Network properties.
        %------------------------------------------------------------------
        function resultStruct = getNetworkProperties(matfile)
            detectorObj = coder.internal.loadDeepLearningNetwork(matfile);

            externalLayers = detectorObj.Network.Layers;
            LayerIndices.ImageLayerIdx = find(...
                arrayfun( @(x)isa(x,'nnet.cnn.layer.ImageInputLayer'), ...
                          externalLayers));

            resultStruct = struct();
            resultStruct.LayerSize = detectorObj.Network.Layers(LayerIndices.ImageLayerIdx).InputSize;
            resultStruct.AnchorBoxes = detectorObj.AnchorBoxes;     
            
            % cast categorical to cellstrs
            labelArray = cellstr(detectorObj.ClassNames);
            % compute length of all classnames
            lengthArray = cellfun(@strlength,labelArray);
            % maxClasses
            numClasses = numel(labelArray);
            
            % preallocate 2-D Char array of nClasses x maxLength
            ClassNames = char(zeros(numClasses, max(lengthArray)));
            
            % populate char array
            for labelIdx = 1: numClasses
                ClassNames(labelIdx,1:lengthArray(labelIdx)) = labelArray{labelIdx};
            end

            resultStruct.ClassNames = ClassNames;
            
            resultStruct.TrainingImageSize = detectorObj.TrainingImageSize;

            resultStruct.LayerIndices = LayerIndices;

            resultStruct.FractionDownsampling = detectorObj.FractionDownsampling;
            
            resultStruct.WH2HW = detectorObj.WH2HW;

        end

    end

    methods(Access = private)

        %------------------------------------------------------------------
        % Function to parse the inputs
        %------------------------------------------------------------------
        function [params, detectionInputWasBatchOfImages, miniBatchSize] = parseDetectInputs(this, I, roi, useROI, varargin)

            %------------------------------------------------------------------
            % Validate Input Image.
            %------------------------------------------------------------------
            detectionInputWasBatchOfImages = coder.const(this.iCheckDetectionInputImage(I));
     
            possibleNameValues = {'Threshold', ...
                                  'ExecutionEnvironment',...
                                  'Acceleration', ...
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
            
            defaults = struct('roi', zeros(1,4),...
                              'ExecutionEnvironment', 'notValid',...
                              'Acceleration', 'none', ...
                              'MiniBatchSize', 128, ...
                              'useROI', false,...
                              'Threshold', 0.5,...
                              'SelectStrongest', true, ...
                              'MinSize', [1 1],...
                              'MaxSize', inputSize(1:2)); % default
                          
            miniBatchSize = 128;   % default

            if (nargin == 1) % only imageSize
                params = defaults;
            else
                pstruct = coder.internal.parseParameterInputs(possibleNameValues, poptions, varargin{:});
                Threshold = coder.internal.getParameterValue(pstruct.Threshold, defaults.Threshold, varargin{:});
                SelectStrongest = coder.internal.getParameterValue(pstruct.SelectStrongest, defaults.SelectStrongest, varargin{:});
                MinSize = coder.internal.getParameterValue(pstruct.MinSize, defaults.MinSize, varargin{:});
                MaxSize = coder.internal.getParameterValue(pstruct.MaxSize, defaults.MaxSize, varargin{:});
                Acceleration = coder.internal.getParameterValue(pstruct.Acceleration, defaults.Acceleration, varargin{:});
                miniBatchSize = coder.internal.getParameterValue(pstruct.MiniBatchSize, defaults.MiniBatchSize, varargin{:});

                params.Threshold = Threshold;
                params.SelectStrongest = SelectStrongest;
                params.MinSize = MinSize;
                params.MaxSize = MaxSize;
                params.Acceleration = Acceleration;
            end
            params.roi = roi;
            params.useROI = useROI;
            
            % explicitly constant fold
            coder.internal.assert(coder.internal.isConst(miniBatchSize), ...
                 'dlcoder_spkg:ObjectDetector:VariableSizeMiniBatch');
            miniBatchSize = coder.const(miniBatchSize);
            
            %------------------------------------------------------------------
            % Validate MiniBatchSize.
            %------------------------------------------------------------------
            vision.internal.cnn.validation.checkMiniBatchSize(coder.const(miniBatchSize), mfilename);
            
            if ~coder.internal.isConst(params.Acceleration) || coder.const(@(x) strcmpi(x,'mex') || strcmpi(x,'auto'), params.Acceleration)
                coder.internal.compileWarning(eml_message(...
                    'dlcoder_spkg:cnncodegen:IgnoreInputArg', 'detect', 'Acceleration'));
            end

            %------------------------------------------------------------------
            % Validate Select Strongest
            %------------------------------------------------------------------
            vision.internal.inputValidation.validateLogical(...
                params.SelectStrongest, 'SelectStrongest');

            validateMinSize = logical(pstruct.MinSize);
            validateMaxSize = logical(pstruct.MaxSize);

            %------------------------------------------------------------------
            % Validate MinSize
            %------------------------------------------------------------------
            if validateMinSize
                vision.internal.detector.ValidationUtils.checkMinSize(...
                    params.MinSize, [1,1], mfilename);
            end
            
            %------------------------------------------------------------------
            % Validate Max Size
            %------------------------------------------------------------------
            if validateMaxSize
                vision.internal.detector.ValidationUtils.checkSize(params.MaxSize, 'MaxSize', mfilename);
                coder.internal.errorIf(params.useROI && (params.MaxSize(1) > params.roi(1,4)) && (params.MaxSize(2) > params.roi(1,3)) , ...
                                       'vision:yolo:modelMaxSizeGTROISize',...
                                       params.roi(1,4),params.roi(1,3));
                coder.internal.errorIf(~params.useROI && any(params.MaxSize > inputSize(1:2)) , ...
                                       'vision:yolo:modelMaxSizeGTImgSize',...
                                       inputSize(1), inputSize(2));
            end

            if validateMaxSize && validateMinSize
                coder.internal.errorIf(any(params.MinSize >= params.MaxSize) , ...
                                       'vision:ObjectDetector:minSizeGTMaxSize');
            end

            %------------------------------------------------------------------
            % Validate ROI
            %------------------------------------------------------------------
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

            %------------------------------------------------------------------
            % Validate threshold
            %------------------------------------------------------------------
            validateattributes(params.Threshold, {'single', 'double'}, {'nonempty', 'nonnan', ...
                                'finite', 'nonsparse', 'real', 'scalar', '>=', 0, '<=', 1}, ...
                               mfilename, 'Threshold');

            if logical(pstruct.ExecutionEnvironment)
                 coder.internal.compileWarning(eml_message(...
                    'dlcoder_spkg:cnncodegen:IgnoreInputArg', 'detect', 'ExecutionEnvironment'));
            end

        end
        
       %------------------------------------------------------------------------------------------------------------------------------------
       function isBatchOfImages = iCheckDetectionInputImage(this, I)
              
            imSz = coder.nullcopy(zeros(1,4));
            [imSz(1),imSz(2),imSz(3),imSz(4)] = size(I);
            
            % Assert for variable sized input (height, width, channel,
            % batch dimensions should be fixed size)
            coder.internal.assert(coder.internal.isConst([imSz(1), imSz(2), imSz(3) imSz(4)]), ...
                'dlcoder_spkg:ObjectDetector:VariableSizedInputYOLOv2');
            
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
            if networkChannelSize > 3 || networkChannelSize == 2
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
       
       %--------------------------------------------------------------------------------------------------------------------------------------
       function [bboxes, scores, labels] = iPostProcessBatchPredictions(this, outputFeatureMap, ...
                    preprocessedImageSize, params, roi, useROI, sx, sy, returnLabels)
                
            coder.internal.prefer_const(returnLabels);    
            numImages = size(outputFeatureMap,4);
            bboxes = cell(numImages, 1);
            scores = cell(numImages, 1);
            labels = cell(numImages, 1);
            
            % Explicitly turn unroll false due to g2187783 
            coder.unroll(false);
            for ii = 1:numImages
                [bboxes{ii}, scores{ii}, labels{ii}] = this.iPostProcessActivations(outputFeatureMap(:,:,:,ii), ...
                    preprocessedImageSize, params, roi, useROI, sx, sy, returnLabels);
            end
                
       end
       
       %-----------------------------------------------------------------------------------------------------------------------------------------       
       function [bboxes, scores, labelNames] = iPostProcessActivations(this, outputFeatureMap, ...
                    preprocessedImageSize, params, roi, useROI, sx, sy, returnLabels)
          
            coder.internal.prefer_const(returnLabels);
            
            gridSize = size(outputFeatureMap);
            anchors = this.AnchorBoxes;

            % Reshape the featureMap to
            % numgridsH x numGridsW x numAnchors x boxProperties.
            featureMapData = reshape(outputFeatureMap, gridSize(1), gridSize(2), size(this.AnchorBoxes,1), gridSize(3)/size(this.AnchorBoxes,1));

            % Estimate downsample factor.
            downsampleFactor = preprocessedImageSize(1:2)./gridSize(1:2);

            if coder.const(~this.FractionDownsampling)
                downsampleFactor = floor(downsampleFactor);
            end
            
            % if anchors are in Width-Height format, convert to
            % Height-Weight format
            if coder.const(this.WH2HW)
                anchors = [anchors(:,2),anchors(:,1)];
            end

            % Scale anchor boxes with respect to feature map size
            anchors(:,1) = anchors(:,1)./downsampleFactor(1);
            anchors(:,2) = anchors(:,2)./downsampleFactor(2);

            % Calculate the bounding boxes with respect to image scale and
            % estimate confidence score for each bounding box.
            % boxOut dimension is 2D :
            % featureMapWidth*featureMapHeight*numAnchors x 6.
            boxOut = this.yoloPredictBbox(featureMapData, anchors, gridSize(1:2), downsampleFactor);

            threshold = params.Threshold;

            % filter out all the boxes whose confidence score less than
            % threshold.
            thresholdedPrediction = boxOut(boxOut(:,5)>=threshold,:);
                        
            % Apply NMS to reduce redundant detections.
           if ~isempty(thresholdedPrediction)
                classPred = thresholdedPrediction(:,6:end);
                scorePred = thresholdedPrediction(:,5);
                bboxesX1Y1X2Y2 = thresholdedPrediction(:,1:4);

                % ClipBoxes to boundaries.
                bboxesX1Y1X2Y2 = iClipBBox(bboxesX1Y1X2Y2, preprocessedImageSize);

                % Scale boxes back to size(Iroi).
                bboxPred = scaleX1X2Y1Y2(bboxesX1Y1X2Y2, sx, sy);

                % Remove boxes that are too small or too big after clipping.
                [bboxPred, scorePred, classPred] = this.filterBBoxes(bboxPred, scorePred, classPred, params.MinSize, params.MaxSize);

                if params.SelectStrongest
                    [bboxes, scores, labels] = selectStrongestBboxMulticlass(bboxPred, scorePred, classPred ,...
                                                                      'RatioType', 'Union', 'OverlapThreshold', 0.5);
                else
                    bboxes = bboxPred;
                    scores = scorePred;
                    labels = classPred;
                end

                % Apply ROI offset.
                bboxes(:,1:2) = vision.internal.detector.addOffsetForROI(bboxes(:,1:2), roi, useROI);
                
                if returnLabels
                    numBBoxes = size(bboxes, 1);
                    labelNames = returnCategoricalLabels(this, numBBoxes, labels);
                else
                    labelNames = [];
                end
                
           else
                % No bounding boxes detected.
                bboxes = zeros(0,4,'double');
                scores = zeros(0,1,'single');  
                                
                if returnLabels
                    numBBoxes = 0;
                    labels = [];                    
                    labelNames = returnCategoricalLabels(this, numBBoxes, labels);
                else
                    labelNames = [];
                end

            end      
                
       end

        %------------------------------------------------------------------
        % Function used to calculate the bounding boxes with respect to
        % image scale
        %------------------------------------------------------------------
        % Input anchors is M-by-2 matrix defined by user. Input anchors are defined
        % in [height, width] format.
        function xyBbox = yoloPredictBbox(~, featureMap, anchors, gridSize, downSamplingFactor)

            xyBbox = coder.nullcopy(zeros(gridSize(1,2)*gridSize(1,1) * size(anchors,1), 6,  'like',featureMap));
            numAnchors = size(anchors,1);
            probPred = coder.nullcopy(zeros(size(featureMap,4) - 5, 1));

            % Parallel for loops
            coder.gpu.kernel;
            for anchorIdx = 1 : size(anchors,1)

                coder.gpu.kernel;
                for colIdx=0:gridSize(1,2)-1

                    coder.gpu.kernel;
                    for rowIdx=0:gridSize(1,1)-1

                        ind = rowIdx * gridSize(1,2) * numAnchors + colIdx * numAnchors + anchorIdx;

                        % Compute center with respect to image
                        cx = (featureMap(rowIdx+1, colIdx+1, anchorIdx, 2 ) + colIdx)* downSamplingFactor(1,2);
                        cy = (featureMap(rowIdx+1, colIdx+1, anchorIdx, 3 ) + rowIdx)* downSamplingFactor(1,1);

                        % Compute width, height with respect to image.
                        bw = featureMap(rowIdx+1, colIdx+1, anchorIdx, 4 )* anchors(anchorIdx,2) * downSamplingFactor(1,2);
                        bh = featureMap(rowIdx+1, colIdx+1, anchorIdx, 5 )* anchors(anchorIdx,1) * downSamplingFactor(1,1);

                        xyBbox(ind, 1) = (cx-bw/2);
                        xyBbox(ind, 2) = (cy-bh/2);
                        xyBbox(ind, 3) = (cx+bw/2);
                        xyBbox(ind, 4) = (cy+bh/2);

                        probPred(1:end) = featureMap(rowIdx+1, colIdx+1, anchorIdx, 6:end);
                        iouPred = featureMap(rowIdx+1, colIdx+1, anchorIdx, 1);

                        % Find maximum probability and confidence score
                        [imax,idx] = max(probPred);
                        confScore = iouPred * imax;

                        xyBbox(ind, 5) = confScore;
                        xyBbox(ind, 6) = idx;

                    end
                end
            end
        end

        %------------------------------------------------------------------
        % Normalize the image to [0 1]
        %------------------------------------------------------------------
        function resImg = rescaleData(~, I)
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

        %------------------------------------------------------------------
        % Get the detections in the range minSize to maxSizem parmeters.
        %------------------------------------------------------------------
        function [bboxes1, scores1, labels1] = filterBBoxes(~, bboxes, scores, labels, minSize, maxSize)
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
        
        function labelNames = returnCategoricalLabels(obj, numBBoxes, labels)
            coder.inline('never');
            % do not generate CUDA code for this function
            
            % get the class name of each bounding box detected.
            labelCells = coder.nullcopy(cell(numBBoxes, 1));
            for i=1:numBBoxes
                % transpose to write row vector of char array
                labelCells{i, 1} = nonzeros(obj.ClassNames(labels(i),:))';
            end
            
            % Grow cell array of valueset dynamically.
            % Maintaining a varsized labelCells avoids the issue of
            % bloating of IR causing slow code-generaion for
            % large const fixed-sized structs.
            % Refer: g2286665
            valueset = {};
            upperBound = size(obj.ClassNames,1);
            coder.varsize('valueset',[1 upperBound],[0 1]);
            for i = 1:upperBound
                valueset{end + 1} = nonzeros(obj.ClassNames(i,:))';
            end
            
            labelNames = categorical(labelCells, valueset);
        end

    end

end

%--------------------------------------------------------------------------
% Get the index of nearest size in TrainingImageSize training sizes that
% matches given image.
%--------------------------------------------------------------------------
function outSize = iComputeBestMatch(preprocessedImageSize,trainingImageSize)
    coder.internal.prefer_const(preprocessedImageSize, trainingImageSize);
    preprocessedImageSize = repmat(preprocessedImageSize,size(trainingImageSize,1),1);
    Xdist = (preprocessedImageSize(:,1) - trainingImageSize(:,1));
    Ydist = (preprocessedImageSize(:,2) - trainingImageSize(:,2));
    dist = sqrt(Xdist.^2 + Ydist.^2);
    [~,ind] = coder.const(@min,dist);
    outSize = coder.const(trainingImageSize(ind,:));
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

    % Convert [x1 y1 x2 y2] boxes into [x y w h] format. Input and
    % output boxes are in pixel coordinates. boxes is an M-by-4
    % matrix.
    scaledBoxes(:,3) = scaledBoxes(:,3) - scaledBoxes(:,1) + 1;
    scaledBoxes(:,4) = scaledBoxes(:,4) - scaledBoxes(:,2) + 1;
end

function c = maxFunc(a,b)
    c = max(a,b);
end

function c = minFunc(a,b)
    c = min(a,b);
end