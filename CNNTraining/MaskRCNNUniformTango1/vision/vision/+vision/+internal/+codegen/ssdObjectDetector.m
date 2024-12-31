%#codegen


% Copyright 2019-2024 The MathWorks, Inc.

% Codegen class for CVT ssdObjectDetector
classdef ssdObjectDetector < coder.internal.NetworkWrapper
    
    properties

        % Backbone DAGNetwork represented as
        % coder.internal.DeepLearningNetwork
        Network

        
        % AnchorBoxes is an M-by-2 matrix defining the [height width] of M
        % anchor boxes.
        AnchorBoxes
        
        % ClassNames is a 2D character array of object class names. These
        % are the object classes that the SSD Object detector was trained
        % to find.
        ClassNames

        % InputSize is the size of input size of Image Input
        % layer in the network.
        InputSize
        
    end
    
    properties(Hidden=true)
        
        % BackgroundIndex is the index of back ground label in the
        % ClassNames property
        BackgroundIndex
    end
    
    methods(Static, Access = public, Hidden)
        
        %------------------------------------------------------------------
        % Function to make the properties constant at compile time.
        %------------------------------------------------------------------
        function n = matlabCodegenNontunableProperties(~)
            n = {'AnchorBoxes', 'ClassNames', 'InputSize', ...
                'BackgroundIndex'};
        end
        
        % this method returning true indicates that the class is opting out
        % of the classGeneration feature
        function optOut = matlabCodegenLowerToStruct(~)
            optOut = true;
        end

        function name = matlabCodegenUserReadableName()
            name = 'ssdObjectDetector';
        end

        function n = matlabCodegenNetworkProperties()
            n = {'Network'};
        end
    end
    
    methods
        
        %------------------------------------------------------------------
        % ssdObjectDetector class constructor which loads the network.
        %------------------------------------------------------------------
        function obj = ssdObjectDetector(matfile, networkName)
            coder.allowpcode('plain');
            coder.internal.prefer_const(matfile, networkName);
            
            % Initialize the detector
            obj = obj@coder.internal.NetworkWrapper(matfile, networkName);
      
            % Get all the network properties.
            coder.extrinsic('vision.internal.codegen.ssdObjectDetector.getDetectorProperties');
            [obj.AnchorBoxes, obj.ClassNames, obj.InputSize, obj.BackgroundIndex]...
                = coder.const(@vision.internal.codegen.ssdObjectDetector.getDetectorProperties,matfile);
            
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

            %% Parse Name-value pairs and arguments
            % Make the compile time function calls as extrinsic
            coder.extrinsic('vision.internal.detector.checkROI');
            
            useROI = false; % Default ROI not to be used
            trainingImageSize = coder.const(this.InputSize); % Network Input size
            preprocessedImageSize = coder.nullcopy(zeros(1, 2));
            
            if( ~isempty(varargin) && (isa(varargin{1}, 'numeric')) )
                
                if( isvector(varargin{1}) && (size(varargin{1},2) == 4) ) % check roi has 4 elements.
                    
                    % roi must be in the format of [x y w h].
                    roi = varargin{1};
                    
                    % Error out if roi is not a constant.
                    coder.internal.assert(coder.internal.isConst(roi), ...
                        'dlcoder_spkg:ObjectDetector:roiConstant')
                    useROI = true;
                    
                    % Check whether ROI is fully contained in image.
                    coder.const(feval('vision.internal.detector.checkROI', roi, size(I)));
                    
                    % params - Threshold, selectStrongest, minsize
                    % maxsize
                    [params, DetectionInputWasBatchOfImages, ~] = this.parseDetectInputs(I, roi, useROI, varargin{2:end});
                    
                    % Height is index 4 & Width is index 3
                    preprocessedImageSize(1) = roi(4);
                    preprocessedImageSize(2) = roi(3);
                    
                else
                    
                    % Error out if roi doesn't contain 4 elements.
                    coder.internal.errorIf(true, 'dlcoder_spkg:ObjectDetector:roiIncorrectNumel')
                end
            else
                
                % If roi is not provided, preallocate memory
                roi = coder.nullcopy(zeros(1,4));
                
                % parse params - Threshold, selectStrongest, minsize
                % maxsize
                [params, DetectionInputWasBatchOfImages, ~] = this.parseDetectInputs(I, roi, useROI, varargin{:});
                preprocessedImageSize(1) = size(I,1);
                preprocessedImageSize(2) = size(I,2);
            end
            
            %% Preprocess the Image
            % Extract the roi from image.
            Iroi = vision.internal.detector.cropImageIfRequested(I, roi, useROI);
            
            % Reshape input to Network Image Input layer Input size.
            
            if coder.internal.isConst(preprocessedImageSize) ...
                    &&  isequal(preprocessedImageSize, trainingImageSize(1:2))
                Ipreprocessed = Iroi;
            else
                Ipreprocessed = coder.nullcopy(zeros(coder.const(trainingImageSize(1)), coder.const(trainingImageSize(2)), ...
                        size(Iroi,3), size(Iroi,4), 'like', Iroi));
                Ipreprocessed(:,:,:,:) = imresize(Iroi, [trainingImageSize(1) trainingImageSize(2)]);
            end
            
            % Convert input data to dlarray format
            Ipreprocessed = dlarray(single(Ipreprocessed),'SSCB');
            
            %% Get the bounding boxes and confidence scores using predict
            % Get the predictions from network.
            numOutputs = coder.const(numel(this.Network.OutputNames));
            out = cell(numOutputs, 1);
            
           [out{:}] = this.Network.predict(Ipreprocessed);
           bboxestmp = extractdata(out{1});
           scorestmp = extractdata(out{2});
            %% Post process
            originalSize = [size(I,1) size(I,2) size(I,3)]; % Size of input image passed to detect excluding batchSize
            if DetectionInputWasBatchOfImages
                [bboxes, scores, varargout{1}] = this.iPostProcessBatchPredictions(bboxestmp, scorestmp, ...
                    originalSize, params, roi, useROI, returnLabels);
            else
                [bboxes, scores, varargout{1}] = this.iPostProcessActivations(bboxestmp, scorestmp, ...
                    originalSize, params, roi, useROI, returnLabels);
            end
            
            
        end

        function className = class(~)
            coder.inline('always');
            className = 'ssdObjectDetector';
        end
        
    end

    methods (Hidden)
        function identifier = getNetworkWrapperIdentifier(~)
            % @todo : replace with actual identifier (g2471979)
            identifier = 'ssdObjectDetector';
        end
    end
    
    methods(Hidden=true, Static)
        
        %------------------------------------------------------------------
        % Get function to fetch the ssdObjectDetector class properties.
        %------------------------------------------------------------------
        function [anchors, classNames, inputSize, backgroundIndex] = getDetectorProperties(matfile)
            detectorObj = coder.internal.loadDeepLearningNetwork(matfile);
            
            % Get the Input size
            inputSize = detectorObj.InputSize;
            anchors = detectorObj.TiledAnchorBoxes;
            
            labelArray = detectorObj.ClassNames;
            lengthArray = cellfun(@strlength,labelArray);
            numClasses = numel(labelArray);
            classNames = char(zeros(numClasses, max(lengthArray)));
            for labelIdx = 1: numClasses
                classNames(labelIdx,1:lengthArray(labelIdx)) = labelArray{labelIdx};
            end
            backgroundIndex = size(classNames,1)+1;
        end
    end
    
    methods(Access = private)
        
        %------------------------------------------------------------------
        % Function to parse the inputs
        %------------------------------------------------------------------
        function [params, detectionInputWasBatchOfImages, miniBatchSize] = parseDetectInputs(this, I, roi, useROI, varargin)
            
            % validate input image
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
            
            defaults = struct('roi', [],...
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
                acceleration = coder.internal.getParameterValue(pstruct.Acceleration, defaults.Acceleration, varargin{:});
                miniBatchSize = coder.internal.getParameterValue(pstruct.MiniBatchSize, defaults.MiniBatchSize, varargin{:});

                params.Threshold = Threshold;
                params.SelectStrongest = SelectStrongest;
                vision.internal.detector.ValidationUtils.checkSize(MinSize, 'MinSize', mfilename);
                params.MinSize = [MinSize(1),MinSize(2)]; 
                vision.internal.detector.ValidationUtils.checkSize(MaxSize, 'MinSize', mfilename);
                params.MaxSize = [MaxSize(1),MaxSize(2)];
                params.Acceleration = acceleration;
            end
            
            params.roi = roi;
            params.useROI = useROI;
            
            % explicitly constant fold
            coder.internal.assert(coder.internal.isConst(miniBatchSize), ...
                'dlcoder_spkg:ObjectDetector:VariableSizeMiniBatch');
            miniBatchSize = coder.const(miniBatchSize);
            
            % validate MiniBatchSize name-value pair
            vision.internal.cnn.validation.checkMiniBatchSize(coder.const(miniBatchSize), mfilename);

            if ~coder.internal.isConst(params.Acceleration) || ~coder.const(strcmpi(params.Acceleration, 'none'))
                coder.internal.compileWarning(eml_message(...
                    'dlcoder_spkg:cnncodegen:IgnoreInputArg', 'detect', 'Acceleration'));
            end
            
            vision.internal.inputValidation.validateLogical(...
                params.SelectStrongest, 'SelectStrongest');
            
            validateMinSize = logical(pstruct.MinSize);
            validateMaxSize = logical(pstruct.MaxSize);
            %------------------------------------------------------------------
            % Validate MinSize
            %------------------------------------------------------------------
            if coder.const(validateMinSize)
                vision.internal.detector.ValidationUtils.checkMinSize(...
                    params.MinSize, [1,1], mfilename);
            end
            
            %------------------------------------------------------------------
            % Validate Max Size
            %------------------------------------------------------------------            
            if coder.const(validateMaxSize)
                % If MaxSize is specified, error if MaxSize < MinSize.
                vision.internal.detector.ValidationUtils.checkMaxSize(params.MaxSize, params.MinSize, mfilename);
            end
            
            if coder.const(validateMaxSize) && coder.const(validateMinSize)
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
            
            % Assert for variable sized channel or batch dimensions
            coder.internal.assert(coder.internal.isConst([imSz(3) imSz(4)]), ...
                'dlcoder_spkg:ObjectDetector:VariableSizeChannelBatchSSD');
            
            networkChannelSize = coder.const(this.InputSize(3));
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
        
        % ---------------------------------------------------------------------------------------------------------------------------------------
        function [bboxes, scores, labels] = iPostProcessBatchPredictions(this, bboxesAll, scoresAll, originalSize, params, roi, useROI, returnLabels)
            
            coder.internal.prefer_const(returnLabels);
            
            numImages = size(bboxesAll,4);
            bboxes = cell(numImages, 1);
            scores = cell(numImages, 1);
            labels = cell(numImages, 1);
            
            % Explicitly turn unroll false due to g2187783 
            coder.unroll(false);
            for ii = 1:numImages
                [bboxes{ii}, scores{ii}, labels{ii}] = this.iPostProcessActivations(bboxesAll(:,:,:,ii), scoresAll(:, :, :, ii), ...
                    originalSize, params,roi, useROI, returnLabels);
            end
            
        end
        
        
        %------------------------------------------------------------------
        % Function to post process activations from network and returns
        % detected object properties.
        %------------------------------------------------------------------
        function [bboxes, scores, labelNames] = iPostProcessActivations(this, bboxes, scores, originalSize, params,roi, useROI, returnLabels)
                       
            coder.internal.prefer_const(returnLabels);
            
            % activations return boxes as [x w y h], make it as [x y w h]
            bboxes = permute(bboxes, [1 3 2 4]);
            % Replicate training Image size to do element wise
            % operations with anchor boxes and bboxes.
            trainedImageScale = [this.InputSize(1:2) this.InputSize(1:2)];
            outImageScale = [originalSize(2) originalSize(1) ...
                originalSize(2) originalSize(1)];
            
            % Scale anchor boxes to Image scale
            anchorBoxes = this.AnchorBoxes;
            for i=1:size(anchorBoxes, 1)
                anchorBoxes(i,:) = anchorBoxes(i,:).*trainedImageScale;
            end
            
            % Scale bounding boxes to image scale.
            bboxes = this.decode(anchorBoxes, bboxes);
            
            coder.gpu.kernel();
            for i = 1:size(bboxes,1)
                bboxes(i,:) = bboxes(i,:).*(outImageScale ./ trainedImageScale);
            end
            
            % Make scores associated with background as negative.
            scores(:,this.BackgroundIndex) = -realmax('single');
            
            % index of maximum confidence score is the label
            % scores is a [numbboxes x 1 x nClasses] matrix
            % compute max along nClasses dimension
            [scores, labels] = max(scores, [], 3);
            
            
            % Filter boxes based on MinSize, MaxSize , Threshold and
            % overlap
            [bboxPred, scorePred, classPred] = this.filterBBoxes(bboxes, scores, labels,...
                params.Threshold, params.MinSize,params.MaxSize, originalSize);
            
            % Apply NMS. OverlapThreshold is supposed to be hardcoded to 0.5 and is not
            % a function of the Threshould parameter.
            if params.SelectStrongest
                [bboxes, scores, labels] = selectStrongestBboxMulticlass(bboxPred, scorePred, classPred ,...
                    'RatioType', 'Union', 'OverlapThreshold',0.5);
            else
                bboxes = bboxPred;
                scores = scorePred;
                labels = classPred;
            end
            
            % Apply ROI offset
            bboxes(:,1:2) = vision.internal.detector.addOffsetForROI(bboxes(:,1:2),roi, useROI);
            
            % Make sure floor doesn't end up giving zero as one of the
            % cooridnates.
            coder.gpu.kernel();
            for i = 1:size(bboxes,1)
                bboxes(i,:) = floor(bboxes(i,:));
                bboxes(i ,[1,2]) = max(bboxes(i,[1 2]),1);
            end
            
            if returnLabels               
                numBBoxes = size(bboxes,1);
                labelNames = returnCategoricalLabels(this, numBBoxes, labels);
            else
                labelNames = [];
            end
        end
        
        %------------------------------------------------------------------
        % Decode bounding box estimates
        %------------------------------------------------------------------
        function bboxes = decode(~, P, reg)
            % Decode regression estimates.
            x = reg(:,1) * 0.1;
            y = reg(:,2) * 0.1;
            w = reg(:,3) * 0.2;
            h = reg(:,4) * 0.2;
            
            % center of proposals
            px = P(:,1) + P(:,3)/2;
            py = P(:,2) + P(:,4)/2;
            
            % compute regression value of ground truth box
            gx = P(:,3).*x + px; % center position
            gy = P(:,4).*y + py;
            
            gw = P(:,3) .* exp(w);
            gh = P(:,4) .* exp(h);
            
            % convert to [x y w h] format
            bboxes = [ gx - gw/2, gy - gh/2, gw, gh];
            bboxes = double(bboxes);
            
        end
                
        %------------------------------------------------------------------
        % Get the detections in the range minSize to maxSizem parameters
        % and above threshold
        %------------------------------------------------------------------
        function [bboxes1, scores1, labels1] = filterBBoxes(~, bboxes, scores, labels, threshold, minSize, maxSize, imageSize)
            % Remove boxes smaller than minsize and bigger than maxSize.
            % Remove boxes that have zero overlap with image.
            % Remove bboxes, scores, labels, that are greater than
            % threshold
            
            predicateArray = coder.nullcopy(zeros(size(bboxes,1), 1, 'int32'));
            
            coder.gpu.kernel;
            for i = 1:size(bboxes,1)
                x1 = bboxes(i,1);
                y1 = bboxes(i,2);
                x2 = bboxes(i,3) + x1 - 1;
                y2 = bboxes(i,4) + y1 - 1;
                if( scores(i) > threshold && ...
                        bboxes(i,4) >= minSize(1) && ...
                        bboxes(i,3) >= minSize(2) && ...
                        bboxes(i,4) <= maxSize(1) && ...
                        bboxes(i,3) <= maxSize(2) && ...
                        (x2 > 1) && (y2 > 1) && ...
                        (x1 < imageSize(2)) && (y1 < imageSize(1)))
                    
                    predicateArray(i) = 1;
                else
                    predicateArray(i) = 0;
                    
                end
            end
            
            predicateArray = cumsum(predicateArray);
            
            newNumElem = predicateArray(end);
            
            bboxes1 = coder.nullcopy(zeros(newNumElem, 4, 'like',bboxes));
            scores1 = coder.nullcopy(zeros(newNumElem, 1, 'like',scores));
            labels1 = coder.nullcopy(zeros(newNumElem, 1, 'like',labels));
            
            % handle i == 1 case
            if predicateArray(1) == 1
                bboxes1(1, :) = bboxes(1, :);
                scores1(1) = scores(1);
                labels1(1) = labels(1);
            end
            
            coder.gpu.kernel;
            for i = 2:numel(predicateArray)
                
                if (predicateArray(i) ~= predicateArray(i-1))
                    bboxes1(predicateArray(i), :) = bboxes(i, :);
                    scores1(predicateArray(i)) = scores(i);
                    labels1(predicateArray(i)) = labels(i);
                    
                end
                
            end
            
        end
        
        
        function  labelNames = returnCategoricalLabels(obj, numBBoxes, labels)
            coder.inline('never');
            % do not generate CUDA code for this function
            
            % get the class name of each bounding box detected.
            labelCells = coder.nullcopy(cell(numBBoxes, 1));
            for i=1:numBBoxes
                % nonzeros returns a column vector. Transpose result to
                % row vector to cast to categorical type.
                labelCells{i} = nonzeros(obj.ClassNames(labels(i),:))';
            end
                        
            % Grow cell array of valueset dynamically.
            % Maintaining a varsized labelCells avoids the issue of
            % bloating of IR causing slow code-generaion for
            % large const fixed-sized structs.
            % Refer: g2286665
            valueset = {};
            upperBound = size(obj.ClassNames,1);
            % subtract 1 to remove Background index. Note that labels
            % and labelCells consequentially will already have removed
            % Background classlabel
            coder.varsize('valueset',[1 upperBound],[0 1]);
            for i = 1:size(obj.ClassNames,1)
                    valueset{end + 1} = nonzeros(obj.ClassNames(i,:))';
            end

            labelNames = categorical(labelCells, valueset);
        end
        
    end
       
end
