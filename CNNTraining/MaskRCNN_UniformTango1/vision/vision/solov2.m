% solov2 Create a SOLO V2 network for instance segmentation.
%
% detector = solov2(detectorName) loads a pretrained SOLO V2 instance
% segmentation detector trained on the COCO dataset. The detectorName
% specifies the architecture of the pre-trained network. detectorName must
% be "resnet50-coco" or "light-resnet18-coco".
%
% detector = solov2(detectorName, classNames)
% configures a pretrained SOLO V2 object detector for transfer learning
% with a new set of object classes. This re-initializes the final convolution
% layers in the classification head with the correct number of filters
% based off number of classes. The configured detector must then be trained
% for optimal performance.
%
%
% Inputs:
% -------
%    detectorName   Specify the name of the pretrained SOLO V2 deep learning
%                   network as a string or character vector. The value must
%                   be one of the following:
%
%                   "resnet50-coco"          Use a SOLO V2 deep learning
%                                            network that has resnet50
%                                            as base network and is trained
%                                            on COCO dataset.
%
%                   "light-resnet18-coco"    Use a SOLO V2 deep learning
%                                            network that has a resnet34
%                                            base network with fewer filters
%                                            and is trained on COCO dataset.
%
%    classNames      Specify the names of object classes that the SOLO V2
%                    network is configured to segment. classNames can be a
%                    be a string vector, a categorical vector, or a cell
%                    array of character vectors.
%
% Additional input arguments
% ----------------------------
% [...] = solov2(..., Name=Value) specifies additional name-value pair
% arguments to configure the pre-trained SOLO V2 network as described below:
%
%    "ModelName"       Detector name specified as string or character
%                      vector.
%
%                      Default: detectorName or specified detectorName
%
%
%    "InputSize"       Specify the image sizes to use for detection. The
%                      segmentObjects method resizes input images
%                      to this size in the detector while maintaining the
%                      aspect ratio.
%
%                      Default: network input size
%
% solov2 object properties
% --------------------------
%   ModelName               - Name of the trained solov2 network.
%   Network                 - SOLO V2 instance segmentation network. (read-only)
%   ClassNames              - A string array of object class names.(read-only)
%   InputSize               - The image size used during training.(read-only)
%   GridSizes               - The size of the grid used at each FPN level
%                             in the detection head.(read-only)
%   NormalizationStatistics - Specifies z-score normalization statistics as
%                             a structure with fields, Mean and StandardDeviation.(read-only)
%
% solov2 object methods
% -----------------------
%   segmentObjects -  Segment object instances in an image.
%
% Example - Segment instances using pre-trained SOLO V2
% -------------------------------------------------------
%
% I = imread('visionteam.jpg');
%
% % Load pre-trained SOLO V2 network
% detector = solov2('resnet50-coco');
%
% % Run inference on the SOLO V2 network
% [masks,labels,scores] = segmentObjects(detector,I);
%
% % Visualize the results
% % Overlay the object masks on the input image
% overlayedImage = insertObjectMask(I, masks);
% figure, imshow(overlayedImage)
%
% See also segmentObjects, maskrcnn, evaluateInstanceSegmentation,
%          insertObjectMask.

% Copyright 2023-2024 The MathWorks, Inc.

classdef solov2 < deep.internal.sdk.LearnableParameterContainer


    % Publicly Visible SoloV2 properties
    properties(SetAccess=private)
        % Custom model name
        ModelName
        % Class names the network is trained on
        ClassNames
        % Image Size which the network is trained on
        InputSize
        % Grid sizes for each stage of the FPN backbone
        GridSizes = [12 16 24 36 40];
        % NormalizationStatistics specifies z-score normalization statitics
        % as a structure with fields, Mean and StandardDeviation specified
        % as 1-by-C array of means and standard deviation per channel. The
        % number of channels, C must match the InputSize
        NormalizationStatistics
    end

    % SoloV2 subnetworks
    properties(SetAccess = private, GetAccess = {?hsolov2, ?htrainSOLOV2})
        % Feature extraction backbone network
        FeatureNet
        % FPN Neck network
        NeckNet
        % Kernel regression head
        KernHeadNet
        % Classification head
        ClsHeadNet
        % Mask feature generation network
        MaskFeatNet
    end

    properties(Hidden)
        % Object Size Ranges which the each FPN level is trained on
        ObjSizeRanges=[[384, 2048];
                        [192, 768];
                        [96, 384];
                        [48, 192];
                        [1, 96]];
        % Dataset Mean used for normalization
        DatasetMean = [123.675, 116.28, 103.53];
        % Dataset Std Dev used for normalization
        DatasetStdDev = [58.395, 57.12, 57.375];
        % Number of classes
        NumClasses
    end

    % Network attributes accumulated across all the sub-networks
    properties(Dependent, Hidden=true)
        Learnables
        Layers
        State
        InputNames
        OutputNames
    end

    % Network inference parameters
    properties(Access=private)
        % Ratio of the original input size and the feature resolution each
        % head works on. This is also the minimum size that can be detected
        % by each FPN level.
        Strides = [32, 32, 16, 8, 8];
        % Initial score threshold to filter out predictions before
        % post-processing
        InitialThresh = 0.1;
        % Final Score threshold
        ScoreThreshold
        % Mask binarization threshold
        MaskThreshold = 0.5;
        % Min object size to be predicted
        MinSize
        % Min object size to be predicted
        MaxSize
        % Toggle Non-max supression
        UseSelectStrongest
    end

    % Network training parameters
    properties (Access=private)
          
        FreezeBackbone = true;
        FreezeNeck = true;
    end


    methods(Access=public)

        function obj = solov2(detector, classNames, options)
            arguments
                detector {mustBeMember(detector, ["resnet50-coco", "light-resnet18-coco", "none"])}
                classNames {validateClassNames, mustBeUniqueNames} = ''
                options.InputSize {mustBeNumeric, mustBePositive, mustBeReal, mustBeFinite, mustBeRGBSize} = []
                options.NormalizationStatistics struct {mustBeScalarOrEmpty} = []
                options.ModelName {mustBeTextScalar} = ""
            end

            vision.internal.requiresNeuralToolbox(mfilename);

            % This is added to support load network workflows all the
            % weights and properties will be populated by the loadobj
            % method.
            if(strcmp(detector, 'none'))
                return;
            end

            iTripwireSOLOV2();

            detector = string(detector);
            % Load the pre-trained detector
            if(detector=="resnet50-coco")
                data = load("SoloV2Res50.mat");
            elseif (detector == "light-resnet18-coco")
                data = load("SoloV2Res18.mat");
            else
                assert(false,"Invalid detector");
            end

            % Extract the solov sub-networks
            obj.FeatureNet  = data.FeatureNet;
            obj.NeckNet     = data.NeckNet;
            obj.MaskFeatNet = data.MaskFeatNet;
            obj.ClsHeadNet  = data.ClsHeadNet;
            obj.KernHeadNet = data.KernHeadNet;

            if(options.ModelName=="")
                obj.ModelName = detector;
            else
                obj.ModelName = options.ModelName;
            end

            if(isempty(options.NormalizationStatistics))
                obj.NormalizationStatistics = iGetCOCONormStats();
            else
                iValidateNormStats(options.NormalizationStatistics);
                obj.NormalizationStatistics = options.NormalizationStatistics;
            end

            % Customize network for new input size
            if(isempty(options.InputSize))
                % Default input size = inputLayer size
                layer = obj.FeatureNet.Layers(1);
                obj.InputSize = layer.InputSize;
            else
                obj.InputSize = options.InputSize;
            end

            % Update the input layer with new size
            inputName = 'imageinput';
            featureOutNames = obj.FeatureNet.OutputNames;
            inputLayer = imageInputLayer(obj.InputSize,...
                                        'Normalization',"zscore",...
                                        'Mean', reshape(obj.NormalizationStatistics.Mean, [1 1 3]),...
                                        'StandardDeviation', reshape(obj.NormalizationStatistics.StandardDeviation, [1 1 3]),...
                                        'Name', inputName);

            obj.FeatureNet = replaceLayer(obj.FeatureNet, inputName, inputLayer);
            obj.FeatureNet = dlnetwork(layerGraph(obj.FeatureNet), 'OutputNames',featureOutNames);

            % Customize network for new class names
            if(isempty(classNames))
                obj.ClassNames = iGetCOCOClasses();
                numClasses = length(obj.ClassNames);
                obj.NumClasses = numClasses;
            else
                % Ensure that classNames are always row vectors
                if(iscolumn(classNames))
                    classNames = classNames';
                end
                if(iscategorical(classNames))
                    classNames = string(classNames);
                end

                if(isstring(classNames))
                    classNames = cellstr(classNames);
                end

                obj.ClassNames = classNames;
                numClasses = length(classNames);
                obj.NumClasses = numClasses;

                % update the classification branch architecture's final conv
                % layer with numClasses

                currConvNode = 'x_mask_head_conv_c_8';
                finalConv = convolution2dLayer([3 3], numClasses,...
                                               'Padding', [1 1 1 1], ...
                                               'Stride', [1 1], 'DilationFactor', [1 1],...
                                               'Name', 'x_mask_head_conv_c_8',...
                                               'WeightsInitializer','narrow-normal',...
                                               'BiasInitializer','narrow-normal');

                obj.ClsHeadNet = replaceLayer(obj.ClsHeadNet, currConvNode, finalConv);
                obj.ClsHeadNet = initialize(obj.ClsHeadNet, dlarray(rand(40,40,256), 'SSCB'));

            end
        end
    end

    methods(Access=public, Hidden)

        function [kernPreds, clsPreds, maskFeats] = predict(obj, X, acceleration)

            % Forward pass on the backbone
            [P2, P3, P4, P5] = predict(obj.FeatureNet, X, Acceleration=acceleration);

            if acceleration == "mex"
                % Set Acceleration to auto for networks without input layers
                accelerationIntermediate = "auto";
            else
                accelerationIntermediate = acceleration;
            end

            % Forward pass on the FPN Neck
            F = {};
            [F{5}, F{4}, F{3}, F{2}, F{1}] = predict(obj.NeckNet, P2,P3,P4,P5, Acceleration=accelerationIntermediate);


            % Forward pass on the shared kernel and classification heads
            % across all FPN levels
            kernPreds = {};
            clsPreds = {};

            for lvl = 1:numel(F)
                if(lvl==5)
                    % Scale down lvl 5 features (highest resolution)
                    lvlFeat = dlresize(F{lvl},Scale=[0.5,0.5],Method="linear",...
                                  GeometricTransformMode="half-pixel",...
                                  NearestRoundingMode="floor");

                elseif(lvl==1)
                    % Scale up level 1 features (lowest resolution)
                    lvlFeat = dlresize(F{lvl},OutputSize=[25,25],Method="linear",...
                                  GeometricTransformMode="half-pixel",...
                                  NearestRoundingMode="floor");
                else
                    lvlFeat = F{lvl};
                end

                % Concatenate normalized xy coordinates to features(coordconv)
                lvlCoords = obj.generateCoordinates(size(lvlFeat));
                lvlFeat = cat(3, lvlFeat, lvlCoords);

                % Resize to gridSize
                lvlFeat = dlresize(lvlFeat, OutputSize=[obj.GridSizes(lvl) obj.GridSizes(lvl)], ...
                                   Method="linear", GeometricTransformMode="half-pixel",...
                                   NearestRoundingMode="floor");

                % Forward pas on the Kernel head (with coordconv features)
                kernPreds{lvl} = predict(obj.KernHeadNet,lvlFeat, Acceleration=accelerationIntermediate);
                % Forward pas on the Classification head (without coordconv features)
                clsPreds{lvl} = predict(obj.ClsHeadNet,lvlFeat(:,:,1:end-2,:), Acceleration=accelerationIntermediate);

            end

            % Forward pass on Mask feature head using unified feature
            % architecture.
            % Concatenate coord conv to the deepest FPN level
            F2 = cat(3, F{2}, obj.generateCoordinates(size(F{2})));

            maskFeats = predict(obj.MaskFeatNet,F{5}, F{4}, F{3}, F2, Acceleration=accelerationIntermediate);
        end


        function [kernPreds, clsPreds, maskFeats, state] = forward(obj, X)

            % Forward pass on the backbone
            if(obj.FreezeBackbone)
                [P2, P3, P4, P5, state] = predict(obj.FeatureNet, X);
            else
                [P2, P3, P4, P5, state] = forward(obj.FeatureNet, X);
            end

            % Forward pass on the FPN Neck
            F = {};
            if(obj.FreezeNeck)
                [F{5}, F{4}, F{3}, F{2}, F{1}] = predict(obj.NeckNet, P2,P3,P4,P5);
            else
                [F{5}, F{4}, F{3}, F{2}, F{1}] = forward(obj.NeckNet, P2,P3,P4,P5);
            end


            % Forward pass on the shared kernel and classification heads
            % across all FPN levels
            kernPreds = {};
            clsPreds = {};

            for lvl = 1:numel(F)
                if(lvl==5)
                    % Scale down lvl 1 features
                    lvlFeat = dlresize(F{lvl},Scale=[0.5,0.5],Method="linear",...
                                  GeometricTransformMode="half-pixel",...
                                  NearestRoundingMode="floor");

                elseif(lvl==1)
                    % Scale up level 5 features
                    lvlFeat = dlresize(F{lvl},OutputSize=[25,25],Method="linear",...
                                  GeometricTransformMode="half-pixel",...
                                  NearestRoundingMode="floor");
                else
                    lvlFeat = F{lvl};
                end

                % Concatenate normalized xy coordinates to features(coordconv)
                lvlCoords = obj.generateCoordinates(size(lvlFeat));
                lvlFeat = cat(3, lvlFeat, lvlCoords);

                % Resize to gridSize
                lvlFeat = dlresize(lvlFeat, OutputSize=[obj.GridSizes(lvl) obj.GridSizes(lvl)], ...
                              Method="linear", GeometricTransformMode="half-pixel",...
                              NearestRoundingMode="floor");

                % Forward pas on the Kernel head (with coordconv features)
                [kernPreds{lvl}, stateKern] = forward(obj.KernHeadNet,lvlFeat);
                % Forward pas on the Classification head (without coordconv features)
                [clsPreds{lvl},stateCls] = forward(obj.ClsHeadNet,lvlFeat(:,:,1:end-2,:));

            end

            % Forward pass on Mask feature head using unified feature
            % architecture.
            % Concatenate coord conv to the deepest FPN level
            F2 = cat(3, F{2}, obj.generateCoordinates(size(F{2})));

            maskFeats = forward(obj.MaskFeatNet,F{5}, F{4}, F{3}, F2);

        end
    end

    methods(Access=public)

        function varargout = segmentObjects(obj, im, options)

            %SEGMENTOBJECTS Segment objects in an image using SOLO V2
            % instance segmentation network.
            %
            % masks = segmentObjects(solov2Obj,I) returns object masks within the image
            % I using the trained solov2 network. The objects masks are detected as a
            % H-by-W-by-M logical array, where each channel contains the mask for a
            % single object. H and W are the height and width of the input image I and
            % M is the number of objects detected in the image. solov2Obj is an object
            % of the solov2 class and I is an RGB or grayscale image.
            %
            % masks = segmentObjects(solov2Obj,IBatch) returns objects within each
            % image contained in the batch of images IBatch. IBatch is a numeric array
            % containing images in the format H-by-W-by-C-by-B, where B is the number
            % of images in the batch, and C is the channel size. For grayscale images,
            % C must be 1. masks is a B-by-1 cell array, with each cell containing an
            % H-by-W-by-M array of object masks for each image in the batch, B.
            %
            % [masks, labels] = segmentObjects(solov2Obj,I) optionally returns the
            % labels assigned to the detected M objects as an M-by-1 categorical array.
            % labels is a B-by-1 cell array, if the input I is a batch of images of the
            % format H-by-W-by-C-by-B.
            %
            % [masks, labels, scores] = segmentObjects(solov2Obj,I) optionally return
            % the detection scores for each of the M objects. The score for each
            % detection is the product of the classification score and maskness score.
            % The range of the score is [0 1]. Larger score values indicate higher
            % confidence in the detection. scores is a B-by-1 cell array, if the input
            % I is a batch of images of the format H-by-W-by-C-by-B.
            %
            % outds = segmentObjects(solov2Obj, imds,___) returns the instance
            % segmentation results for images stored in a Datastore imds. The output
            % of read(ds) must be an image array or a cell array. When the output of
            % read(ds) is a cell array, then only the first column of data is processed
            % by the network. The output outds is a fileDatastore representing the
            % instance segmentation results.  The result for each image - object masks,
            % labels, scores and bounding boxes are stored in a .MAT file at the
            % location specified by 'WriteLocation'. A read on the outds returns these
            % outputs in the following order:
            %
            %       1st cell  : Predicted logical object masks for M objects as a
            %                   H-by-W-by-M logical array.
            %       2nd cell  : Predicted object labels as a Mx1 categorical vector.
            %
            %       3rd cell  : Prediction scores as a Mx1 numeric vector.
            %
            %       4th cell  : Prediction boxes as a Mx4 numeric vector.
            %
            % [...] = segmentObjects(..., Name=Value) specifies additional
            % name-value pairs described below:
            %
            % 'Threshold'              A scalar between 0 and 1. Detections
            %                          with scores less than the threshold
            %                          value are removed. Increase this value
            %                          to reduce false positives.
            %
            %                          Default: 0.5
            %
            % 'MaskThreshold'          A scalar between 0 and 1. Thresholds
            %                          the mask probabilities to obtain a
            %                          logical mask.
            %
            %                          Default: 0.5
            %
            % 'SelectStrongest'        A logical scalar. Set this to true to
            %                          eliminate overlapping object masks
            %                          based on their scores. This process is
            %                          often referred to as non-maximum
            %                          suppression. Set this to false if you
            %                          want to perform a custom selection
            %                          operation. When set to false, all the
            %                          segmented masks are returned.
            %
            %                          Default: true
            %
            % 'ExecutionEnvironment'   Specify what hardware resources will be used to
            %                          run the SOLO v2 detector. Valid values for
            %                          resource are:
            %
            %                          'auto' - Use a GPU if it is available, otherwise
            %                                   use the CPU.
            %
            %                          'gpu' - Use the GPU. To use a GPU, you must have
            %                                  Parallel Computing Toolbox(TM), and a
            %                                  CUDA-enabled NVIDIA GPU. If a suitable
            %                                  GPU is not available, an error message
            %                                  is issued.
            %
            %                          'cpu' - Use the CPU.
            %
            %                          Default: 'auto'
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
            %                           Default : 'auto'
            %
            % The following name-value pair arguments control the writing of image
            % files. These arguments apply only when processing images in a datastore.
            %
            % 'MiniBatchSize'        A scalar to specify the size of the image batch
            %                        used to perform inference. This option can be used
            %                        to leverage batch inference to speed up processing,
            %                        comes at a cost of extra memory used. A higher value
            %                        of MiniBatchSize can result in out of memory errors,
            %                        depending on the hardware capabilities.
            %
            %                        Default: 1
            %
            % 'WriteLocation'        A scalar string or character vector to specify a
            %                        folder location to which extracted image files are
            %                        written. The specified folder must have
            %                        write permissions. If the folder already exists,
            %                        the next available number will be added as a suffix
            %                        to the folder name.
            %
            %                        Default: fullfile(pwd, 'SegmentObjectResults'), where
            %                        pwd is the current working directory.
            %
            % 'NamePrefix'           A scalar string or character vector to specify the
            %                        prefix applied to output image file names. For
            %                        input 2-D image inputs, the result MAT files
            %                        are named <prefix>_<imageName>.mat, where
            %                        imageName is the name of the input image
            %                        without its extension.
            %
            %                        Default: 'segmentObj'
            %
            % 'Verbose'              Set true to display progress information.
            %
            %                        Default: true
            %
            % 'MaxNumKernels'        To limit the number of kernels we use MaxNumKernels.
            %                        We choose the top MaxNumKernel highest scoring kernels.
            %                        To optimize the code generation performance we mandate
            %                        fixing the number of kernels. When MaxNumKernels are less
            %                        than the valid thresholded kernels we sort and take top
            %                        MaxNum kernels. When MaxNumKernels are greater than
            %                        valid thresholded kernels we perform zero padding.
            %
            %                        Range : [1 3096]
            %
            %                        Default : 'auto'
            arguments
                obj
                im {validateImageInput}
                options.Threshold (1,1){mustBeNumeric, mustBePositive, mustBeLessThanOrEqual(options.Threshold, 1), mustBeReal} = 0.5
                options.MaskThreshold (1,1){mustBeNumeric, mustBePositive, mustBeLessThanOrEqual(options.MaskThreshold, 1), mustBeReal} = 0.5
                options.SelectStrongest (1,1) logical = true
                options.ExecutionEnvironment {mustBeMember(options.ExecutionEnvironment,{'gpu','cpu','auto'})} = 'auto'
                options.Acceleration {mustBeMember(options.Acceleration,{'mex','none','auto'})} = 'auto'
                options.WriteLocation {mustBeTextScalar} = fullfile(pwd,'SegmentObjectResults')
                options.MiniBatchSize (1,1) {mustBeNumeric, mustBePositive, mustBeReal, mustBeInteger} = 1
                options.NamePrefix {mustBeTextScalar} = "segmentObj"
                options.Verbose (1,1) {validateLogicalFlag} = true
                options.MaxNumKernels = 'auto'
            end


            % Send the data to device or to host based on ExecutionEnvironment
            % option
            if(isequal(options.ExecutionEnvironment, 'auto'))
                if(canUseGPU)
                    options.ExecutionEnvironment = 'gpu';
                else
                    options.ExecutionEnvironment = 'cpu';
                end
            end

            if (~isequal(options.MaxNumKernels, 'auto'))
                validateattributes(options.MaxNumKernels, {'numeric'}, ...
                    {'scalar', 'finite', 'real', 'integer', 'positive'}, mfilename, ...
                    'MaxNumKernels');
                coder.internal.errorIf((options.MaxNumKernels > 3096 || options.MaxNumKernels < 1),...
                    'vision:solov2:invalidMaxNumKernels');
            end

            % If writeLocation is set with a non-ds input, throw a warning
            if(~matlab.io.datastore.internal.shim.isDatastore(im) &&...
                    ~strcmp(options.WriteLocation, fullfile(pwd,'SegmentObjectResults')))

                warning(message('vision:solov2:WriteLocNotSupported'));
            end

            obj.ScoreThreshold = options.Threshold;
            obj.MaskThreshold = options.MaskThreshold;
            obj.UseSelectStrongest = options.SelectStrongest;
            autoResize = true;

            % Check if the input image is a single image or a batch
            if(matlab.io.datastore.internal.shim.isDatastore(im))
                nargoutchk(0,1);
                [varargout{1:nargout}] = ...
                    segmentObjectsInDatastore(obj, im,...
                                            autoResize,...
                                            options.MiniBatchSize,...
                                            options.NamePrefix,...
                                            options.WriteLocation,...
                                            options.Verbose,...
                                            options.ExecutionEnvironment,...
                                            options.Acceleration,...
                                            options.MaxNumKernels);

            elseif(ndims(im)<=3)
                % Process Single image
                nargoutchk(0,4);
                miniBatchSize=1;
                [varargout{1:nargout}] =...
                    segmentObjectsInImgStack(obj, im,...
                               autoResize,...
                               miniBatchSize,...
                               options.ExecutionEnvironment,...
                               options.Acceleration,...
                               options.MaxNumKernels);
            elseif(ndims(im)==4)
                nargoutchk(0,4);
                [varargout{1:nargout}] = ...
                    segmentObjectsInImgStack(obj, im,...
                                 autoResize,...
                                 options.MiniBatchSize,...
                                 options.ExecutionEnvironment,...
                                 options.Acceleration,...
                                 options.MaxNumKernels);
            else
                % Code flow shouldn't reach here (ensured by validation code).
                assert(false, 'Invalid image input.');
            end
        end
    end


    methods(Access=private)

        function [masks, labels, scores] = segmentObjectsInImgStack(obj, im, autoResize, miniBatchSize, executionEnvironment, acceleration, maxNumKern)
            % This function dispatches batches for batch processing of
            % image Stacks.
            stackSize = size(im,4);

            masks = {};
            labels = {};
            scores = {};

            % Process images from the imageStack, a minibatch at a time
            for startIdx = 1 : miniBatchSize : stackSize

                endIdx = min(startIdx+miniBatchSize-1, stackSize);

                imBatch = im(:,:,:,startIdx:endIdx);

                [masksCell, labelCell, scoreCell] = ...
                                            segmentObjectsInBatch(obj, imBatch,...
                                            autoResize, executionEnvironment,...
                                            acceleration, maxNumKern);

                masks = vertcat(masks, masksCell); %#ok<AGROW>
                labels = vertcat(labels, labelCell); %#ok<AGROW>
                scores = vertcat(scores, scoreCell); %#ok<AGROW>
            end

            % For a stack size = 1 (single image) output raw matrices
            % instead of cell arrays.
            if(numel(masks)==1)
                masks = masks{1};
                labels = labels{1};
                scores = scores{1};
            end
        end

        function outds = segmentObjectsInDatastore(obj, imds, autoResize, miniBatchSize, namePrefix, writeLocation, verboseFlag, executionEnvironment, acceleration, maxNumKern)

            imdsCopy = copy(imds);
            imdsCopy.reset();

            % Get a new write location
            fileLocation = vision.internal.GetUniqueFolderName(writeLocation);

            if(~exist(fileLocation, 'dir'))
                success = mkdir(fileLocation);
                if(~success)
                    throwAsCaller(MException('vision:solov2:folderCreationFailed',...
                        vision.getMessage('vision:solov2:folderCreationFailed')));
                end
            end

            % Handle verbose display
            printer = vision.internal.MessagePrinter.configure(verboseFlag);

            printer.linebreak();
            iPrintHeader(printer);
            msg = iPrintInitProgress(printer,'', 1);

            imIdx = 0;

            outFileList = [];
            % Process images from the datastore
            while (hasdata(imdsCopy))

                imBatch = [];
                fileNames = []; % Needed to build output names for result .matfiles

                % Build a minibatch worth of data
                for i = 1:miniBatchSize
                    if(~hasdata(imdsCopy))
                        break;
                    end
                    imIdx = imIdx + 1;
                    [img, imgInfo] = read(imdsCopy); %#ok<AGROW>

                    % Handle combineDS - use first cell, as the image is
                    % expected to be the first output.
                    if(iscell(imgInfo))
                        imgInfo = imgInfo{1};
                    end

                    %If the datastore doesn't expose filename, use the
                    % read index instead
                    if (isfield(imgInfo, 'Filename'))
                        [~,fileNames{i}] = fileparts(imgInfo.Filename); %#ok<AGROW>
                    else
                        fileNames{i} = num2str(imIdx); %#ok<AGROW>
                    end

                    if(iscell(img))
                        imBatch{i} = img{1}; % image should be the first output
                    else
                        imBatch{i} = img;
                    end
                end

                [masksCell, labelCell, scoreCell] = ...
                    segmentObjectsInBatch(obj, imBatch, autoResize, executionEnvironment, acceleration, maxNumKern);

                % Write results to the disk
                for idx = 1:numel(masksCell)

                    matfilename = string(namePrefix)+"_"+string(fileNames{idx})+".mat";

                    masks = masksCell{idx};
                    boxScore = scoreCell{idx};
                    boxLabel = labelCell{idx};

                    currFileName = fullfile(fileLocation, matfilename);

                    save(currFileName,...
                        "masks","boxScore","boxLabel");
                    outFileList = [outFileList; currFileName];
                end
                % Print number of processed images
                msg = iPrintProgress(printer, msg, imIdx+numel(masksCell)-1);
            end

            outds = fileDatastore(outFileList, 'FileExtensions', '.mat',...
                'ReadFcn', @(x)vision.internal.SegmentObjectsReader(x));

            printer.linebreak(2);

        end

        function [masks, labels, scores] = segmentObjectsInBatch(obj, im, autoResize, executionEnvironment, acceleration, maxNumKern)


            if(iscell(im))
                batchSize = numel(im);
                im = cat(4, im{:});
            else
                batchSize = size(im,4);
            end

            if(isequal(executionEnvironment, 'gpu'))
                im = gpuArray(im);
            end

            % Preprocess input
            [im, info] = solov2.preprocessInput(im, obj.InputSize(1:2),...
                obj.NormalizationStatistics.Mean, obj.NormalizationStatistics.StandardDeviation,...
                autoResize);
            im = dlarray(im, 'SSCB');

            % Predict on the solov2 network
            [kernPreds, clsPreds, maskFeats] = predict(obj, im, acceleration);
            % Post process predictions
            [masks, labels, scores] = postprocessOutputs(obj, kernPreds, clsPreds, maskFeats, info, maxNumKern);
        end

        function [masks, labels, scores] = postprocessOutputs(obj, kernPredsBatch, clsPredsBatch, maskFeatsBatch, scaleInfo, maxNumKern)

            batchSize = size(maskFeatsBatch,4);
            masks = cell(batchSize,1);
            labels = cell(batchSize,1);
            scores = cell(batchSize,1);

            % Post process 1 image at a time
            for idx = 1:batchSize

                % Extract predictions for the idx-th batch size
                mlvl_class_score = cellfun(@(x)x(:,:,:,idx), clsPredsBatch, 'UniformOutput',false);
                mlvl_kern = cellfun(@(x)x(:,:,:,idx), kernPredsBatch, UniformOutput=false);
                mask_feat = maskFeatsBatch(:,:,:,idx);

                numLvl = length(mlvl_class_score);

                % Step 1: Perform non max suppression on the class scores in
                % spatial dimension
                for i=1:numLvl

                    classScore = mlvl_class_score{i};

                    % Get max values in 2x2 windows
                    localMax = maxpool(classScore, [2,2], 'Stride', 1, 'Padding','same');

                    keepValues = (classScore==localMax);
                    classScore = classScore.*keepValues;

                    mlvl_class_score{i} = classScore;

                end

                % Step 2: Reshape Class scores and kernel to SxC & platten all levels to a
                % single list
                mask_cls_preds = cellfun(@(x)reshape(x, [size(x,1)*size(x,2), size(x,3)]), mlvl_class_score, 'UniformOutput',false);
                mask_cls_preds = cat(1,mask_cls_preds{:});

                kern_preds = cellfun(@(x)reshape(x, [size(x,1)*size(x,2), size(x,3)]), mlvl_kern, 'UniformOutput',false);
                kern_preds = cat(1,kern_preds{:});

                % Threshold scores with the initialThresh value
                keep_scores = mask_cls_preds>obj.InitialThresh;

                % Indices to keep - Col 1  = rowIdxs, Col 2 = classIdx
                keep_idxs = [];
                [keep_idxs(:,1), keep_idxs(:,2)] = find(extractdata(keep_scores));
                % Mask classification scores
                mask_cls_preds = mask_cls_preds(keep_scores);
                % class labels
                cls_labels = keep_idxs(:,2);
                % Kernel predictions
                kern_preds = kern_preds(keep_idxs(:,1), :);
                
                % Sort the scores if maxNumKern is given by user
                if (~isequal(maxNumKern, 'auto'))
                    mask_cls_predsArr  = extractdata(mask_cls_preds);
                    [mask_cls_preds,index] = sort(mask_cls_predsArr,1,'descend');
                    mask_cls_preds = dlarray(mask_cls_preds);
                    cls_labels = cls_labels(index);
                    kern_preds = kern_preds(index,:);
                end

                % Get the original image sizes
                if(iscell(scaleInfo))
                    scale = scaleInfo{idx}.Scale;
                    originalSize = scaleInfo{idx}.OriginalSize;
                else
                    scale = scaleInfo.Scale;
                    originalSize = scaleInfo.OriginalSize;
                end

                % If no predictions, return empty masks, scores and labels
                if(isempty(kern_preds))
                    masks{idx} = false(originalSize(1), originalSize(2), 0);
                    scores{idx} = [];
                    labels{idx} = categorical([], 1:obj.NumClasses, obj.ClassNames);
                    continue;
                end

                % Step 3: Compute strides for each of the kept indices?
                intervalsSize = obj.GridSizes.^2;
                fpnStrides =[];
                for i = 1:numel(intervalsSize)
                    lvlStrides = repmat(obj.Strides(i), intervalsSize(i),1);
                    fpnStrides = [fpnStrides;lvlStrides];
                end
                fpnStrides = fpnStrides(keep_idxs(:,1));

                % Step4: Generate conv using dynamic conv
                % Reshape kernels to 1x1xnumFeaturesxnumFilters
                kern_preds = extractdata(kern_preds);
                kern_preds = permute(kern_preds,[3 4 2 1]);

                % MaxNumKernels value is given by user and greater
                % than actual kernels, padding is done
                numFilt = size(kern_preds, 4);
                if (~isequal(maxNumKern, 'auto'))
                    if maxNumKern > numFilt
                        kernpredsPadSize = [size(kern_preds, 1:3), maxNumKern];
                        kern_predsPad = zeros(kernpredsPadSize, class(kern_preds));
                        kern_predsPad(:,:,:,1:numFilt) = kern_preds(:,:,:,:);
                        kern_preds = kern_predsPad;
                        fpnStrides(numFilt+1:maxNumKern) = 0;
                    else
                        kern_preds = kern_preds(:,:,:,1:maxNumKern);
                    end
                end

                mask_preds = dlconv(mask_feat,kern_preds,0);
                mask_preds = sigmoid(mask_preds);

                masksPreNMS = mask_preds>obj.MaskThreshold;

                % Step 5: Filter out masks less than stride
                mask_area = squeeze(sum(masksPreNMS,[1 2]));

                if (isequal(maxNumKern, 'auto'))
                    keep_masks_idx = mask_area>fpnStrides;
                else
                    keep_masks_idx = mask_area>fpnStrides(1:maxNumKern);
                end

                masksPreNMS = masksPreNMS(:,:,keep_masks_idx);
                mask_preds = mask_preds(:,:,keep_masks_idx);
                mask_area = mask_area(keep_masks_idx);

                if maxNumKern > numFilt
                    cls_scores = mask_cls_preds(keep_masks_idx(1:numFilt));
                    cls_labels = cls_labels(keep_masks_idx(1:numFilt));
                else
                    cls_scores = mask_cls_preds(keep_masks_idx);
                    cls_labels = cls_labels(keep_masks_idx);
                end

                % Step 6: get maskScores & combime it with class-scores
                mask_scores = squeeze(sum(masksPreNMS .* mask_preds, [1 2]))./mask_area;

                if maxNumKern > numFilt
                    scoresSize = size(cls_scores, 1);
                    mask_scores = mask_scores(1:scoresSize);
                    cls_scores = cls_scores.*mask_scores;
                else
                    cls_scores = cls_scores.*mask_scores;
                end

                % Step7: matrix NMS
                numPreNMS = 500;
                maxPerImg = 100;
                kernel = 'gaussian';
                sigma = 2;
                filterThresh = obj.ScoreThreshold;


                if(obj.UseSelectStrongest)
                    [currScores, currLabels, ~, keep_idx] = vision.internal.solov2.matrixNMS(extractdata(masksPreNMS), cls_labels,...
                                                                            extractdata(cls_scores),...
                                                                            extractdata(mask_area),...
                                                                            numPreNMS, maxPerImg,...
                                                                            kernel, sigma, filterThresh);
                else
                    currScores = extractdata(cls_scores);
                    currLabels = cls_labels;
                    keep_idx   = 1:numel(cls_scores);
                end

                mask_preds = extractdata(mask_preds(:,:,keep_idx));

                % If no predictions, return empty masks, scores and labels
                if(isempty(mask_preds))
                    masks{idx} = false(originalSize(1), originalSize(2), 0);
                    scores{idx} = [];
                    labels{idx} = categorical([], 1:obj.NumClasses, obj.ClassNames);
                    continue;
                end

                % For cases when mask_preds are gpuarray & selectStrongest
                % is set to false. Due to the large number of mask_preds,
                % gpu version of imresize fails due to size restrictions on
                % size of gpu inputs. To work around this, we look over the
                % channel dimension gpu+selectStrongest=false cases.
                if(obj.UseSelectStrongest||~isgpuarray(mask_preds))
                    mask_preds = imresize(mask_preds, obj.InputSize(1:2),'bilinear');
                    mask_preds = imresize(mask_preds, scale,'bilinear');
                else
                    resized_mask_preds = [];
                    for cidx = 1:size(mask_preds,3)
                        curr_pred = imresize(mask_preds(:,:,cidx), obj.InputSize(1:2),'bilinear');
                        resized_mask_preds(:,:,cidx) = imresize(curr_pred, scale,'bilinear'); %#ok<AGROW>
                    end
                    mask_preds = resized_mask_preds;
                end

                % Drop the Padding & threshold
                currMasks = mask_preds(1:originalSize(1), 1:originalSize(2), :)>obj.MaskThreshold;

                masks{idx} = gather(currMasks);
                labels{idx} = categorical(gather(currLabels), 1:obj.NumClasses, obj.ClassNames);
                scores{idx} = gather(currScores);

            end
        end

        function coordFeats = generateCoordinates(~, featSize)
            % This function generates a normalized map of feature
            % coordinates for the coordconv maps.
            xR = linspace(-1, 1, featSize(2));
            yR = linspace(-1, 1, featSize(1));

            [xCoord, yCoord] = meshgrid(xR,yR);

            xCoord = repmat(xCoord, 1,1,1,featSize(4));
            yCoord = repmat(yCoord, 1,1,1,featSize(4));

            coordFeats = dlarray(cat(3,xCoord, yCoord), 'SSCB');
        end
    end

    methods(Hidden)

        function s = saveobj(this)
            % Serialize and save solov2 object
            s.Version            = 1.0;
            s.ModelName          = this.ModelName;
            s.InputSize          = this.InputSize;
            s.FeatureNet         = this.FeatureNet;
            s.NeckNet            = this.NeckNet;
            s.MaskFeatNet        = this.MaskFeatNet;
            s.KernHeadNet        = this.KernHeadNet;
            s.ClsHeadNet         = this.ClsHeadNet;
            s.ClassNames         = this.ClassNames;
            s.NumClasses         = this.NumClasses;
            s.GridSizes          = this.GridSizes;
            s.Strides            = this.Strides;
            s.ObjSizeRanges      = this.ObjSizeRanges;
            s.FreezeBackbone     = this.FreezeBackbone;
            s.FreezeNeck         = this.FreezeNeck;
            s.NormalizationStatistics = this.NormalizationStatistics;
        end

        function [net1, net2, net3, net4, net5, net6, net7,...
                net8, net9, net10, net11, net12, net13] = matlabCodegenPrivateNetwork(this)
            net1 = this.FeatureNet;
            net2 = this.NeckNet;
            net3 = this.MaskFeatNet;
            net4 = this.KernHeadNet;
            net5 = this.KernHeadNet;
            net6 = this.KernHeadNet;
            net7 = this.KernHeadNet;
            net8 = this.KernHeadNet;
            net9 = this.ClsHeadNet;
            net10 = this.ClsHeadNet;
            net11 = this.ClsHeadNet;
            net12 = this.ClsHeadNet;
            net13 = this.ClsHeadNet;
        end

        function obj = configureForTraining(obj, freezeSubnetStr)
            % Configure subnetwork freezing for training
            if(lower(freezeSubnetStr)=="backbone")
                obj.FreezeBackbone = true;
                obj.FreezeNeck = false;
            elseif(lower(freezeSubnetStr)=="backboneandneck")
                obj.FreezeBackbone = true;
                obj.FreezeNeck = true;
            elseif(lower(freezeSubnetStr)=="none")
                obj.FreezeBackbone = false;
                obj.FreezeNeck = false;
            else
                assert(false,"Invalid freezeSubnetStr");
            end
        end

        function obj = setInputNormalization(obj,stats)
            % Set Input normalization stats from the trainer.
            % Standardize stats struct field names
            if(isfield(stats,"Std"))
                stats.StandardDeviation = stats.Std;
            end
            iValidateNormStats(stats);

            % Update the Norm Stats properties
            obj.NormalizationStatistics.Mean = stats.Mean;
            obj.NormalizationStatistics.StandardDeviation = stats.Std;

            % Update the network input layer with new normalization
            % stats
            currInputLayer = obj.FeatureNet.Layers(1);
            newInputLayer = imageInputLayer(currInputLayer.InputSize,...
                                           "Normalization","zscore",...
                                           "Mean",stats.Mean,...
                                           "StandardDeviation",stats.Std,...
                                           "Name",currInputLayer.Name);

            outNames = obj.FeatureNet.OutputNames;
            obj.FeatureNet = replaceLayer(obj.FeatureNet, currInputLayer.Name,...
                                          newInputLayer);

            % Initialize modified network
            obj.FeatureNet = dlnetwork(layerGraph(obj.FeatureNet), 'OutputNames',outNames);
        end
    end

    methods(Static, Hidden)
        function this = loadobj(s)
            % De-serialize and load the solov2 object
            try
                vision.internal.requiresNeuralToolbox(mfilename);

                this = solov2("none");

                this.ModelName          = s.ModelName;
                this.InputSize          = s.InputSize;
                this.FeatureNet         = s.FeatureNet;
                this.NeckNet            = s.NeckNet;
                this.MaskFeatNet        = s.MaskFeatNet;
                this.KernHeadNet        = s.KernHeadNet;
                this.ClsHeadNet         = s.ClsHeadNet;
                this.ClassNames         = s.ClassNames;
                this.NumClasses         = s.NumClasses;
                this.GridSizes          = s.GridSizes;
                this.Strides            = s.Strides;
                this.ObjSizeRanges      = s.ObjSizeRanges;
                this.FreezeBackbone     = s.FreezeBackbone;
                this.FreezeNeck         = s.FreezeNeck;
                if(isfield(s,'NormalizationStatistics'))
                    this.NormalizationStatistics = s.NormalizationStatistics;
                elseif(isfield(s,'DatasetMean')&&isfield(s,'DatasetStdDev'))
                    this.NormalizationStatistics.Mean = s.DatasetMean;
                    this.NormalizationStatistics.StandardDeviation = s.DatasetStdDev;
                else
                    this.NormalizationStatistics = iGetCOCONormStats();
                end
            catch ME
                rethrow(ME)
            end
        end

        function n = matlabCodegenDlRedirect(~)
            n = 'vision.internal.codegen.SOLOv2';
        end
    end

    % Get solov2 network attributes from all the sub-networks
    methods
        function s = get.Learnables(obj)
            % Package all subnetwork learnables into a structure
            if(~obj.FreezeBackbone)
                s.FeatureNet = obj.FeatureNet.Learnables;
            end

            if(~obj.FreezeNeck)
                s.NeckNet = obj.NeckNet.Learnables;
            end

            s.MaskFeatNet = obj.MaskFeatNet.Learnables;
            s.KernHeadNet = obj.KernHeadNet.Learnables;
            s.ClsHeadNet = obj.ClsHeadNet.Learnables;
        end

        function obj = set.Learnables(obj, in)
            % Update Learnables on each subnetwork
            if(isfield(in, 'FeatureNet'))
                obj.FeatureNet.Learnables = in.FeatureNet;
            end

            if(isfield(in, 'NeckNet'))
                obj.NeckNet.Learnables = in.NeckNet;
            end

            if(isfield(in, 'MaskFeatNet'))
                obj.MaskFeatNet.Learnables = in.MaskFeatNet;
            end

            if(isfield(in, 'KernHeadNet'))
                obj.KernHeadNet.Learnables = in.KernHeadNet;
            end

            if(isfield(in, 'ClsHeadNet'))
                obj.ClsHeadNet.Learnables = in.ClsHeadNet;
            end

        end

        function layers = get.Layers(obj)

            layers = vertcat(obj.FeatureNet.Layers,...
                             obj.NeckNet.Layers,...
                             obj.MaskFeatNet.Layers,...
                             obj.KernHeadNet.Layers,...
                             obj.ClsHeadNet.Layers);

        end

        function obj =  set.State(obj, value)
            if(~obj.FreezeBackbone)
                obj.FeatureNet.State = value;
            end
        end
        function state = get.State(obj)
            state = vertcat(obj.FeatureNet.State,...
                             obj.NeckNet.State,...
                             obj.MaskFeatNet.State,...
                             obj.KernHeadNet.State,...
                             obj.ClsHeadNet.State);
        end

        function inputs = get.InputNames(obj)
            inputs = obj.FeatureNet.InputNames;
        end

        function outnames = get.OutputNames(obj)
            outnames = horzcat(...
                             obj.MaskFeatNet.OutputNames,...
                             obj.KernHeadNet.OutputNames,...
                             obj.ClsHeadNet.OutputNames);
        end
    end

    methods(Access=protected)
        function obj = updateLearnableParameters(obj, updater)
            % Get the learnables, update them and then set them back on
            % the subnetworks.
            learnables = obj.Learnables;
            learnables = updater.apply(learnables);
            obj.Learnables = learnables;
        end
    end

    methods(Static, Hidden)
        function [data, info] = preprocessInput(data, targetSize, mean, std, autoResize)
            % Preprocess input data for inference and training
            if istable(data)
                data = table2cell(data);
            end

            if iscell(data)
                % Preprocess inputs while training - One observation at a
                % time
                % Assumption here is that the training data has the
                % following order - image, boxes, labels, masks

                I = data{1};
                info.OriginalSize = size(I);

                % Handle grayscale inputs
                if(size(I,3)==1)
                    I = repmat(I,[1 1 3 1]);
                end

                % Resize Images
                if(~isempty(data{1}))
                    [data{1}, scale] = resizePreservingAspectRatio(single(I),targetSize(1:2),0);
                    info.Scale = scale;
                else
                    info.Scale = 1;
                end

                % Resize Boxes
                data{2} = bboxresize(data{2}, 1/info.Scale);

                % Resize Masks
                if(~isempty(data{4}))
                    data{4} = logical(resizePreservingAspectRatio(uint8(data{4}), targetSize(1:2),0));
                end

            else
                info.OriginalSize = size(data);

                % Handle grayscale inputs
                if(size(data,3)==1)
                    data = repmat(data,[1 1 3 1]);
                end

                % Resize
                [data, scale] = resizePreservingAspectRatio(single(data),targetSize(1:2),0);
                info.Scale = scale;

            end
        end
    end
end

%--------------------------------------------------------------------------
% Helper functions
%--------------------------------------------------------------------------
function [resizedImage, scale] = resizePreservingAspectRatio(img, targetSize, pad)
    % Compute the scale to resize the groundtruth bounding boxes.

    img = single(img);
    imgSize = size(img);

    % Compute Aspect Ratio.
    imgAspectRatio = imgSize(2)/imgSize(1);

    resizeRowsToTargetOutputSize = ceil(imgAspectRatio*targetSize(1));
    resizeColsToTargetOutputSize = ceil(targetSize(2)/imgAspectRatio);
    padSizeIfResizeRowsToTarget = resizeRowsToTargetOutputSize-targetSize(2);
    padSizeIfResizeColsToTarget = resizeColsToTargetOutputSize-targetSize(1);

    % Resize and pad image to final size
    if padSizeIfResizeRowsToTarget < padSizeIfResizeColsToTarget
        scale = imgSize(1)/targetSize(1);
        resizedImage = imresize(img,[targetSize(1),nan]);
        resizedImage(:,end+1:targetSize(2),:) = pad;
    elseif padSizeIfResizeColsToTarget < padSizeIfResizeRowsToTarget
        scale = imgSize(2)/targetSize(2);
        resizedImage = imresize(img,[nan,targetSize(2)]);
        resizedImage(end+1:targetSize(1),:,:) = pad;
    else
        scale = imgSize(1)/targetSize(1);
        resizedImage = imresize(img,targetSize(1:2));
    end

end

function [output, imScale] = iResizeImgBatch(img, targetSize)
    % Compute the scale for resize
    imgSize = size(img);
    scale = targetSize(1:2)./imgSize(1:2);
    [~,index] = max(scale);
    imScale = min(scale);
    img = single(img);

    % Resize the img to targetSize.
    if index == 1

        img = imresize(img, [nan,targetSize(index)],Method="bilinear",Antialiasing=false);

        padding = ones(targetSize(index)-size(img,1),size(img,2),size(img,3),size(img,4))*0;
        if(~isempty(padding))
            img = cat(1,img,padding);
        end
    else
        img = imresize(img, [targetSize(index),nan],Method="bilinear",Antialiasing=false);

        padding = ones(size(img,1),targetSize(index)-size(img,2),size(img,3),size(img,4))*0;
        if(~isempty(padding))
            img = cat(2,img,padding);
        end
    end

    output = img;
end

%--------------------------------------------------------------------------
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
end

function stats = iGetCOCONormStats()
    % Dataset Mean used for normalization
    stats.Mean = [123.675, 116.28, 103.53];
    % Dataset Std Dev used for normalization
    stats.StandardDeviation = [58.395, 57.12, 57.375];
end

%--------------------------------------------------------------------------
% Input validators
%--------------------------------------------------------------------------
function mustBeUniqueNames(input)
    %Check for unique class names
    % input is a single char string, return
    if(ischar(input))
        return;
    end

    % For cell array of char strings, array of strings and categorical
    % arrays, check of uniqueness
    if(length(input)~=length(unique(input)))
       throwAsCaller(MException('vision:solov2:duplicateClassNames',...
              vision.getMessage('vision:solov2:duplicateClassNames')));
    end
end

%--------------------------------------------------------------------------
function validateClassNames(input)

    % Check of single char string
    if(ischar(input))
        return;
    end

    % Check for cell array of char strings
    if(iscell(input))
        if(all(cellfun(@ischar, input)) && isvector(input))
            return;
        end
    end

    % Check for string & categorical vectors
    if( (isstring(input)||iscategorical(input)) && isvector(input))
        return;
    end


    throwAsCaller(MException('vision:solov2:incorrectClassNameType',...
           vision.getMessage('vision:solov2:incorrectClassNameType')));

end

%--------------------------------------------------------------------------
function mustBeRGBSize(input)
    % the size must either be [] or (1,3) with the channel dim =3 and the
    % spatial dims must be multiples of 32

    if(~isempty(input)&&length(input)>2)
       isMultipleOf32 = (mod(input(1),32)==0) && (mod(input(2),32)==0);
       isValidChannelDim = length(input)==3 && input(3)==3;
    else
       isMultipleOf32 = false;
       isValidChannelDim = false;
    end

    if~(isempty(input) || (isMultipleOf32 && isValidChannelDim))
       throwAsCaller(MException('vision:solov2:incorrectInputSize',...
          vision.getMessage('vision:solov2:incorrectInputSize')));
    end
end

%--------------------------------------------------------------------------
function validateImageInput(in)

    im = [];
    if(isnumeric(in))
        im = in;
    elseif(matlab.io.datastore.internal.shim.isDatastore(in))
        out = preview(in);
        if(iscell(out))
            if(isempty(out))
                im = [];
            else
                im = out{1};
            end
        else
            im = out;
        end
    end

    if(~validateImage(im)||isempty(im))
        throwAsCaller(MException('vision:solov2:invalidImageInput',...
               vision.getMessage('vision:solov2:invalidImageInput')));
    end

end

%--------------------------------------------------------------------------
function tf = iValidateNormStats(stats)
    tf = isfield(stats, {'Mean','StandardDeviation'});
    if ~all(tf)
        error(message('vision:solov2:invalidNormalizationStatisticsStruct'));
    end
    iValidateNormStatsSize(size(stats.Mean), 3);
    iValidateNormStatsSize(size(stats.StandardDeviation), 3);
end

%--------------------------------------------------------------------------
function iValidateNormStatsSize(statsSize,inputChannelSize)
    if (numel(statsSize) == 2 && any(statsSize ~= [1 inputChannelSize])) || ...
       (numel(statsSize) == 3 && any(statsSize ~= [1 1 inputChannelSize])) || ...
        numel(statsSize) > 3

        error(message('vision:solov2:invalidNormalizationStatisticsSize',inputChannelSize));
    end
end

%--------------------------------------------------------------------------
function tf = validateImage(in)
    tf = isnumeric(in)&&...
         ndims(in)<=4 && ... && numdims should be less than 3
         (size(in,3)==3||size(in,3)==1); % gray scale or RGB image
end

%--------------------------------------------------------------------------
function validateLogicalFlag(in)
    validateattributes(in,{'logical'}, {'scalar','finite', 'real'});
end

%--------------------------------------------------------------------------
function iTripwireSOLOV2()
    % Check if support package is installed
    breadcrumbFile = 'vision.internal.cnn.supportpackages.IsSOLOV2Installed';
    fullPath = which(breadcrumbFile);
    if isempty(fullPath)
        name     = 'Computer Vision Toolbox Model for SOLO V2 Instance Segmentation';
        basecode = 'SOLOV2';
        throwAsCaller(MException(message('nnet_cnn:supportpackages:InstallRequired', mfilename, name, basecode)));
    end
end

%--------------------------------------------------------------------------
function iPrintHeader(printer)
    printer.printMessage('vision:solov2:verboseHeader');
    printer.print('--------------------------');
    printer.linebreak();
end

%--------------------------------------------------------------------------
function updateMessage(printer, prevMessage, nextMessage)
    backspace = sprintf(repmat('\b',1,numel(prevMessage))); % figure how much to delete
    printer.print([backspace nextMessage]);
end

%--------------------------------------------------------------------------
function nextMessage = iPrintInitProgress(printer, prevMessage, k)
    nextMessage = getString(message('vision:solov2:verboseProgressTxt',k));
    updateMessage(printer, prevMessage(1:end-1), nextMessage);
end

%--------------------------------------------------------------------------
function nextMessage = iPrintProgress(printer, prevMessage, k)
    nextMessage = getString(message('vision:solov2:verboseProgressTxt',k));
    updateMessage(printer, prevMessage, nextMessage);
end
