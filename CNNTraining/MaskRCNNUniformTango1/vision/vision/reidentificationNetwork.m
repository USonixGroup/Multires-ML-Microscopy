classdef reidentificationNetwork < deep.internal.sdk.LearnableParameterContainer
%

% Copyright 2023 The MathWorks, Inc.

    % Publicly visible ReID properties.
    properties(SetAccess=private)
        % Custom model name.
        ModelName
        % ReID Network including the backbone, neck, and classification head.
        Network
        % Class names the network is trained on.
        ClassNames
        % Image Size which the network is trained on.
        InputSize
        % Size of appearance feature vectors.
        FeatureLength
        % Feature extraction layer.
        FeatureLayer
        % NormalizationStatistics specifies z-score normalization statistics
        % as a structure with fields, Mean and StandardDeviation specified
        % as 1-by-C array of means and standard deviation per channel. The
        % number of channels, C must match the InputSize
        NormalizationStatistics
    end

    % ReID subnetworks.
    properties(SetAccess=private, GetAccess = {?hreidentificationNetwork, ...
            ?hExtractReidentificationFeatures,?htrainReidentificationNetwork})
        % Feature extraction backbone network.
        FeatureNetwork
        % Feature extraction neck network
        FeatureNeckNetwork
        % Classification head network.
        ClassificationNetwork
    end

    properties(Hidden)
        % Number of classes
        NumClasses
        % Resize method for resizing input images
        Resize = "scale";
    end

    % Network attributes accumulated across all the sub-networks.
    properties(Dependent, Hidden=true)
        Learnables
        Layers
        State
        InputNames
        OutputNames
    end

    % Network training parameters.
    properties (Access=private)
        FreezeBackbone = true;
    end


    methods(Access=public)

        function obj = reidentificationNetwork(network, classes, options)
            arguments
                network
                classes
                options.ClassificationNetwork = [];
                options.FeatureLayer {mustBeTextScalar} = "";
                options.InputSize {mustBeNumeric, mustBePositive, mustBeReal, mustBeFinite, iMustBeRGBSize} = [];
                options.ModelName {mustBeTextScalar} = "ReidentificationNetwork";
                options.FeatureLength {mustBeNumeric, mustBeFinite, mustBePositive, mustBeReal, mustBeInteger, mustBeScalarOrEmpty, mustBeNonempty} = 256;
                options.NormalizationStatistics struct {mustBeScalarOrEmpty} = []
            end

            vision.internal.requiresNeuralToolbox(mfilename);

            % This is added to support load network workflows all the
            % weights and properties will be populated by the loadobj
            % method.
            if isempty(network) && isa(network,"dlnetwork")
                return;
            end

            % Validate network.
            iValidateNetwork(network);

            % Validate class names.
            iValidateClassNames(classes);
            iMustBeUniqueNames(classes);

            % Set class names and the number of classes.
            classes = iFormatClasses(classes);

            obj.ClassNames = classes;
            obj.NumClasses = length(obj.ClassNames);

            % Remove final classification network head of input network and
            % construct the feature extraction network.
            [backboneNetwork,neckNetwork, options.FeatureLength] = ...
                iConstructFeatureNetwork(network, options.FeatureLayer, options.FeatureLength);

            % Set the feature and classification networks.
            obj.FeatureNetwork = backboneNetwork;
            obj.FeatureNeckNetwork = neckNetwork;
            obj.FeatureLength = options.FeatureLength;

            if isempty(options.ClassificationNetwork)
                obj.ClassificationNetwork = iDefaultClassificationNetwork(obj.NumClasses, obj.FeatureLength);
            else
                obj.ClassificationNetwork = iReplaceFullyConnectedLayer(...
                    options.ClassificationNetwork, obj.NumClasses, obj.FeatureLength);
            end

            % Initialize the feature neck and classification sub-networks.
            obj = initializeSubNetworks(obj);

            obj.ModelName = options.ModelName;

            % Customize network for new input size
            if isempty(options.InputSize)
                % Default input size = inputLayer size
                layer = obj.FeatureNetwork.Layers(1);
                obj.InputSize = layer.InputSize;
            else
                obj.InputSize = options.InputSize;
            end

            % Update the input layer with new size
            imageInputIdx = arrayfun(@(x)isa(x,'nnet.cnn.layer.ImageInputLayer'),backboneNetwork.Layers);
            inputIdx = find(imageInputIdx,1,'first');

            inputLayer = backboneNetwork.Layers(inputIdx);
            inputName = inputLayer.Name;

            % Find the current input normalization in the input layer.
            useLayerStats = obj.InputSize == inputLayer.InputSize;
            normStats = iFindNormalizationStatistics(inputLayer,options.NormalizationStatistics,useLayerStats);
            obj.NormalizationStatistics.Mean = normStats.Mean;
            obj.NormalizationStatistics.StandardDeviation = normStats.StandardDeviation;

            inputLayer = imageInputLayer(obj.InputSize,...
                                        Normalization=normStats.Normalization,...
                                        Mean=dlarray(obj.NormalizationStatistics.Mean),...
                                        StandardDeviation=dlarray(obj.NormalizationStatistics.StandardDeviation),...
                                        Name=inputName);

            obj.FeatureNetwork = replaceLayer(obj.FeatureNetwork, inputName, inputLayer);

            obj.FeatureNetwork = initialize(obj.FeatureNetwork);

            % Assign the feature layer.
            if options.FeatureLayer==""
                obj.FeatureLayer = obj.FeatureNeckNetwork.Layers(end).Name;
            else
                obj.FeatureLayer = options.FeatureLayer;
            end
            % Construct the complete ReID network.
            obj = constructReidentificationNetwork(obj);

        end

        function varargout = extractReidentificationFeatures(obj, im, options)
        % extractReidentificationFeatures Extract object reidentification features from an image.
        %
        % extractedFeatures = extractReidentificationFeatures(reID, I) returns a feature vector of
        % size M-by-1, where M is the size of reID.FeatureLength. reID is a ReID
        % network object and I is an RGB or grayscale image. Use this syntax to
        % obtain a single feature vector for a single RGB or grayscale image.
        %
        % extractedFeatures = extractReidentificationFeatures(reID,IBatch) returns an array of
        % features of size M-by-B, where M is the size of reID.FeatureLength and B
        % is the batch size of the IBatch. IBatch is a numeric array containing
        % images in the format H-by-W-by-C-by-B, where B is the number of images in
        % the batch, and C is the channel size. For grayscale images, C must be 1.
        % Use this syntax when you want to obtain the feature vectors for multiple
        % images at once.
        %
        % [extractedFeatures, labels] = extractReidentificationFeatures(reID, ds) returns an array
        % of features of size M-by-N, where M is the size of reID.FeatureLength and
        % N is the number of images within the input datastore. ds is either an
        % imageDataStore with the Labels property populated or is a datastore that
        % returns a cell array on the read method with two columns:
        %
        % Column 1: A cell vector of the input image.
        % Column 2: A string or a categorical cell vector where each element is of
        %           size 1-by-1 containing the object class name for the given input
        %           image in column 1. Note that all the categorical data returned
        %           by the datastore must have the same categories.
        %
        % labels is a string array of size 1-by-M that contains each object ID or
        % class name corresponding to each extractedFeature column vector. M is
        % equal to the number images within the input datastore. Use this syntax
        % when you want to obtain all feature vectors in a datastore or wish to
        % evaluate the ReID's network performance using evalauteReidentificationNetwork.
        %
        % Additional input arguments
        % ----------------------------
        % [...] = segmentObjects(..., Name=Value) specifies additional name-value
        % pairs described below:
        %
        % "ExecutionEnvironment"   Specify what hardware resources will be used to
        %                          run the ReID network extraction. Valid values for
        %                          resource are:
        %
        %                          'auto' - Use a GPU if it is available, otherwise
        %                                   use the CPU.
        %
        %                          'gpu'  - Use the GPU. To use a GPU, you must
        %                                   have Parallel Computing Toolbox(TM),
        %                                   and a CUDA-enabled NVIDIA GPU. If a
        %                                   suitable GPU is not available, an error
        %                                   message is issued.
        %
        %                          'cpu' - Use the CPU.
        %
        %                          Default: 'auto'
        %
        % "Acceleration"           Optimizations that can improve
        %                          performance at the expense of some
        %                          overhead on the first call, and possible
        %                          additional memory usage. Valid values
        %                          are:
        %
        %                          'auto'  - Automatically select optimizations
        %                                    suitable for the input network and
        %                                    environment.
        %
        %                          'none'  - Disable all acceleration.
        %
        %                           Default: 'auto'
        %
        % "MiniBatchSize"           A scalar to specify the size of the image batch
        %                           used to perform inference. This option can be used
        %                           to leverage batch inference to speed up processing,
        %                           comes at a cost of extra memory used. A higher value
        %                           of MiniBatchSize can result in out of memory errors,
        %                           depending on the hardware capabilities.
        %
        %                           Default: 1
        %
        % "Verbose"                 Set true to display progress information.
        % 
        %                           Default: true
            arguments
                obj 
                im {iValidateImageInput}
                options.ExecutionEnvironment {mustBeMember(options.ExecutionEnvironment,{'gpu','cpu','auto'})} = 'auto'
                options.Acceleration {mustBeMember(options.Acceleration,{'none','auto'})} = 'auto'
                options.MiniBatchSize (1,1) {mustBeNumeric, mustBePositive, mustBeReal, mustBeInteger} = 1
                options.Verbose (1,1) {iValidateLogicalFlag} = true
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

            resize = obj.Resize;

             % Check if the input image is a datstore, single image, or
             % a batch of images.
            if matlab.io.datastore.internal.shim.isDatastore(im)
                % Process a datastore of images.
                nargoutchk(0,2);
                [varargout{1:nargout}] = extractFeaturesInDS(obj,im,...
                    options.MiniBatchSize,resize,options.Verbose,...
                    options.ExecutionEnvironment,options.Acceleration);
            else
                nargoutchk(0,1);
                if ndims(im)<=3
                    % Process a single image.
                    miniBatchSize=1;
                elseif ndims(im)==4
                    % Process a batch of images.
                    miniBatchSize = options.MiniBatchSize;
                else
                    % Code flow shouldn't reach here (ensured by validation code).
                    assert(false, 'Invalid image input.');
                end
                varargout{1} = extractFeaturesInImgStack(obj,im,resize,...
                    miniBatchSize,options.ExecutionEnvironment,options.Acceleration);
            end
        end
    end

    methods(Access=public, Hidden)

        function predictions = predict(obj, X, varargin)

            narginchk(2,3)
            if ~isempty(varargin)
                acceleration = varargin{1};
            else
                acceleration = "auto";
            end

            X = obj.preprocessInput(X,obj.InputSize(1:2),obj.Resize);

            % Forward pass on the whole network.
            classPreds = predict(obj.Network, X, Acceleration=acceleration);

            if size(classPreds,2) == size(X,4)
                [~,predIdx] = max(classPreds,[],1);
            else
                [~,predIdx] = max(classPreds,[],2);
            end
            predictions = obj.ClassNames(predIdx');

        end

        function [pred, states] = forward(obj, X)

            % Forward pass on the backbone. By default, the backbone will
            % be frozen to have the network only learn the feature vector
            % outputs in the Feature Neck Network, as well as the
            % classification parameters in the Classification Network.
            if obj.FreezeBackbone
                [P2, featureState] = predict(obj.FeatureNetwork, X);
            else
                [P2, featureState] = forward(obj.FeatureNetwork, X);
            end

            [P3, neckState] = forward(obj.FeatureNeckNetwork, P2);
            [pred, classState] = forward(obj.ClassificationNetwork, P3);

            states = {featureState,neckState,classState};
        end

    end


    methods(Hidden)

        function s = saveobj(this)
            % Serialize and save reidentification object.
            s.Version                   = 1.0;
            s.ModelName                 = this.ModelName;
            s.InputSize                 = this.InputSize;
            s.Network                   = this.Network;
            s.FeatureLength             = this.FeatureLength;
            s.FeatureLayer              = this.FeatureLayer;
            s.FeatureNetwork            = this.FeatureNetwork;
            s.FeatureNeckNetwork        = this.FeatureNeckNetwork;
            s.ClassificationNetwork     = this.ClassificationNetwork;
            s.ClassNames                = this.ClassNames;
            s.Resize                    = this.Resize;
            s.NumClasses                = this.NumClasses;
            s.FreezeBackbone            = this.FreezeBackbone;
            s.NormalizationStatistics   = this.NormalizationStatistics;
        end

        function obj = constructReidentificationNetwork(obj)
            % Combine the feature network, feature neck network, and
            % classification head into a single dlnetwork.
            network = iCombineTwoDLNetworks(obj.FeatureNetwork,obj.FeatureNeckNetwork);
            network = iCombineTwoDLNetworks(network,obj.ClassificationNetwork);
            obj.Network = initialize(network);
        end

        function obj = initializeSubNetworks(obj)
            % Obtain a sample output from the first network to intialize the
            % sub-network on.
            X = dlarray(ones(obj.FeatureNetwork.Layers(1).InputSize,"single"),"SSCB");
            sampleOutput = predict(obj.FeatureNetwork,X);
        
            % Initialize the feature neck sub-network.
            obj.FeatureNeckNetwork = initialize(obj.FeatureNeckNetwork,sampleOutput);
            sampleOutput = predict(obj.FeatureNeckNetwork,sampleOutput);
        
            % Find the expected format of the input to the custom reidentification
            % fully connect softmax layer.
            fcSoftmaxName = obj.ClassificationNetwork.Layers(end).Name;
            classificationNetworkTemp = removeLayers(obj.ClassificationNetwork,fcSoftmaxName);
            classificationNetworkTemp = initialize(classificationNetworkTemp,sampleOutput);
            sampleOutputTemp = predict(classificationNetworkTemp,sampleOutput);
            inFormat = dims(sampleOutputTemp);
        
            % Replace the layer with the updated format if it is not 'CB' (default).
            if ~strcmp(inFormat,"CB")
                lossFunction = obj.ClassificationNetwork.Layers(end).LossFunction;
                reIDFCSoftmaxLayer = iCreateFullyConnectedSoftmaxLayer(obj.NumClasses,...
                    obj.FeatureLength, lossFunction, inFormat);
                obj.ClassificationNetwork = replaceLayer(obj.ClassificationNetwork,...
                    fcSoftmaxName,reIDFCSoftmaxLayer);
            end
        
            % Initialize the classification head sub-network.
            obj.ClassificationNetwork = initialize(obj.ClassificationNetwork,sampleOutput);
        end

        function obj = setInputNormalization(obj,stats)
            % Set Input normalization stats from the trainer.

            % Standardize stats struct field names.
            if isfield(stats,"Std")
                stats.StandardDeviation = stats.Std;
            end
            iValidateNormStats(stats);

            % Update the Norm Stats properties.
            obj.NormalizationStatistics.Mean = stats.Mean;
            obj.NormalizationStatistics.StandardDeviation = stats.Std;

            % Update the network input layer with new normalization stats.
            currInputLayer = obj.FeatureNetwork.Layers(1);
            normalizationMethod = currInputLayer.Normalization;

            % Set standard deviation and mean to empty if the normalization
            % method calls for it. These stats come from the trainer and
            % are non-empty. Mean is only valid for zscore or zerocenter.
            % Standard deviation is only valid for zscore.
            if ~(strcmp(normalizationMethod,'zscore') || strcmp(normalizationMethod,'zerocenter'))
                stats.Mean = [];
            end

            if ~strcmp(normalizationMethod,'zscore')
                stats.Std = [];
            end
            newInputLayer = imageInputLayer(currInputLayer.InputSize,...
                                            Normalization=currInputLayer.Normalization,...
                                            Mean=stats.Mean,...
                                            StandardDeviation=stats.Std,...
                                            Name=currInputLayer.Name);

            obj.FeatureNetwork = replaceLayer(obj.FeatureNetwork, ...
                currInputLayer.Name,newInputLayer);

            obj.FeatureNetwork = initialize(obj.FeatureNetwork);

            % Reconstruct the reidentification network.
            obj = constructReidentificationNetwork(obj);
        end

        function obj = configureImageInputResizeMethod(obj, resize)
            % Configure resize method for image inputs.
            obj.Resize = resize;
        end

        function obj = configureForTraining(obj, freezeBackbone,lossFunction)
            % Configure backbone freezing for training.
            obj.FreezeBackbone = freezeBackbone;

            if ~strcmp(obj.ClassificationNetwork.Layers(end).LossFunction, lossFunction)
                % Create a new reidentificationFCSoftmaxLayer with the
                % given loss function and replace the old layer.
                reIDFCSoftmaxLayer = iCreateFullyConnectedSoftmaxLayer(...
                    obj.NumClasses, obj.FeatureLength, lossFunction);
                origLayer = obj.ClassificationNetwork.Layers(end);
                obj.ClassificationNetwork = replaceLayer(obj.ClassificationNetwork,...
                    origLayer.Name,reIDFCSoftmaxLayer);
                obj = initializeSubNetworks(obj);
                obj = constructReidentificationNetwork(obj);
            end
        end

    end

     methods(Access=private)

        function [extractedFeatures, labels] = extractFeaturesInDS(obj, imds, ...
                miniBatchSize, resize, verbose, executionEnvironment, acceleration)

            imdsCopy = copy(imds);
            imdsCopy.reset();

            % Set read size to 1 for where valid.
            if isprop(imdsCopy,"ReadSize")
                imdsCopy.ReadSize = 1;
            end

            % Handle verbose display
            printer = vision.internal.MessagePrinter.configure(verbose);

            iPrintHeader(printer);
            msg = iPrintInitProgress(printer,'', 1);

            extractedFeatures = {};
            labels = [];
            imIdx = 0;
            % Process images from the datastore.
            while hasdata(imdsCopy)

                imBatch = [];

                % Build a minibatch worth of data
                for i = 1:miniBatchSize
                    if ~hasdata(imdsCopy)
                        break;
                    end
                    imIdx = imIdx + 1;
                    if isa(imdsCopy,'matlab.io.datastore.ImageDatastore')
                        [img, imgInfo] = read(imdsCopy);
                        if ~isempty(imgInfo.Label)
                            label = imgInfo.Label;
                        else
                            label = [];
                        end
                    else
                        imgData = read(imdsCopy);
                        img = imgData{1};
                        if length(imgData) > 1
                            label = imgData{2};
                        else
                            label = [];
                        end
                    end

                    % Handle combineDS - use first cell, as the image is
                    % expected to be the first output.
                    if iscategorical(label)
                        label = string(label);
                    end

                    if iscell(img)
                        imBatch{i} = img{1}; %#ok<AGROW>
                    else
                        imBatch{i} = img; %#ok<AGROW>
                    end

                    labels = [labels label]; %#ok<AGROW>
                end

                extractedFeaturesCell = extractFeaturesInBatch(obj,imBatch, ...
                    resize,executionEnvironment,acceleration);

                extractedFeatures = horzcat(extractedFeatures,extractedFeaturesCell'); %#ok<AGROW>

                % Print number of processed images
                msg = iPrintProgress(printer, msg, imIdx);
            end

            % Output raw matrices instead of cell arrays.
            extractedFeatures = cell2mat(extractedFeatures);

            iPrintFooter(printer)

        end

        function extractedFeatures = extractFeaturesInImgStack(obj, im, ...
                resize, miniBatchSize, executionEnvironment, acceleration)
            % This function dispatches batches for batch processing of
            % image Stacks.
            stackSize = size(im,4);

            extractedFeatures = {};

            % Process images from the imageStack, a minibatch at a time.
            for startIdx = 1:miniBatchSize:stackSize

                endIdx = min(startIdx+miniBatchSize-1,stackSize);

                imBatch = im(:,:,:,startIdx:endIdx);

                extractedFeaturesCell = extractFeaturesInBatch(obj,imBatch, ...
                    resize,executionEnvironment,acceleration);

                extractedFeatures = horzcat(extractedFeatures,extractedFeaturesCell'); %#ok<AGROW> 
            end

            % Output raw matrices instead of cell arrays.
            extractedFeatures = cell2mat(extractedFeatures);
        end

        function extractedFeatures = extractFeaturesInBatch(obj, im, ...
                resize, executionEnvironment, acceleration)

            if iscell(im)
                batchSize = numel(im);
                im = cat(4, im{:});
            else
                batchSize = size(im,4);
            end

            extractedFeatures = cell(batchSize,1);

            if isequal(executionEnvironment,'gpu')
                im = gpuArray(im);
            end

            % Preprocess input.
            im = reidentificationNetwork.preprocessInput(im,obj.InputSize(1:2),resize);
            im = dlarray(im,'SSCB');

            extractedFeaturesBatch = predict(obj.Network,im,Acceleration=acceleration,Outputs=obj.FeatureLayer);

            extractedFeaturesBatch = gather(extractdata(extractedFeaturesBatch));

            for idx = 1:batchSize
                % Reduce the apperance feature batch to FeatureLength-by-B.
                extractedFeaturesVector = squeeze(extractedFeaturesBatch);
                extractedFeatures{idx} = extractedFeaturesVector(:,idx);
            end

        end
     end

    % Get reidentification network attributes from all the sub-networks.
    methods
        function s = get.Learnables(obj)
            % Package all sub-network learnables into a structure.
            if ~obj.FreezeBackbone
                s.FeatureNetwork = obj.FeatureNetwork.Learnables;
            end

            s.FeatureNeckNetwork = obj.FeatureNeckNetwork.Learnables;
            s.ClassificationNetwork = obj.ClassificationNetwork.Learnables;
        end

        function obj = set.Learnables(obj, in)
            % Update Learnables on each sub-network.
            if isfield(in, 'FeatureNetwork')
                if ~obj.FreezeBackbone
                    obj.FeatureNetwork.Learnables = in.FeatureNetwork;
                end
            end

            obj.FeatureNeckNetwork.Learnables = in.FeatureNeckNetwork;
            obj.ClassificationNetwork.Learnables = in.ClassificationNetwork;
        end

        function obj = set.State(obj, states)
            featureNetworkState = states{1};
            featureNeckNetworkState = states{2};
            classificationNetworkState = states{3};
            if ~obj.FreezeBackbone
                obj.FeatureNetwork.State = featureNetworkState;
            end
            obj.FeatureNeckNetwork.State = featureNeckNetworkState;
            obj.ClassificationNetwork.State = classificationNetworkState;
        end
        function state = get.State(obj)
            state = vertcat(obj.FeatureNetwork.State,...
                             obj.FeatureNeckNetwork.State,...
                             obj.ClassificationNetwork.State);
        end

        function inputs = get.InputNames(obj)
            inputs = obj.FeatureNetwork.InputNames;
        end

        function outnames = get.OutputNames(obj)
            outnames = obj.ClassificationNetwork.OutputNames;
        end

        function layers = get.Layers(obj)
            layers = vertcat(obj.FeatureNetwork.Layers,...
                             obj.FeatureNeckNetwork.Layers,...
                             obj.ClassificationNetwork.Layers);
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

        function this = loadobj(s)
            % De-serialize and load the reidentification network object.
            try
                vision.internal.requiresNeuralToolbox(mfilename);

                this = reidentificationNetwork(dlnetwork.empty,"");

                this.ModelName               = s.ModelName;
                this.InputSize               = s.InputSize;
                this.Network                 = s.Network;
                this.FeatureLength           = s.FeatureLength;
                this.FeatureLayer            = s.FeatureLayer;
                this.FeatureNetwork          = s.FeatureNetwork;
                this.FeatureNeckNetwork      = s.FeatureNeckNetwork;
                this.ClassificationNetwork   = s.ClassificationNetwork;
                this.ClassNames              = s.ClassNames;
                this.Resize                  = s.Resize;
                this.NumClasses              = s.NumClasses;
                this.FreezeBackbone          = s.FreezeBackbone;
                this.NormalizationStatistics = s.NormalizationStatistics;
            catch ME
                rethrow(ME)
            end
        end

        function [data, info] = preprocessInput(data, targetSize, varargin)
            narginchk(2,3)

            % Resize may be "letterbox", "scale", and "none".
            if nargin == 3
                resize = varargin{1};
            else
                resize = "none";
            end
            % Preprocess input data for inference and training.
            if istable(data)
                data = table2cell(data);
            end

            if iscell(data)
                % Assumption here is that the training data has the
                % following order - image, labels
                I = data{1};
                info.OriginalSize = size(I);

                % Handle grayscale inputs
                if size(I,3)==1
                    I = repmat(I,[1 1 3 1]);
                end

                % Obtain batch size of data.
                batchSize = size(I,4);

                % Resize images based on the resize method specified.
                if strcmp(resize,"letterbox")
                    [data{1}, scale] = iResizePreservingAspectRatio(I,targetSize(1:2),0,batchSize);
                    info.Scale = scale;
                elseif strcmp(resize,"scale")
                    data{1} = imresize(im2single(I),targetSize(1:2));
                    info.Scale = size(I,1:2)/targetSize(1:2);
                else
                    data{1} = im2single(I);
                    info.Scale = 1;
                end

            else
                info.OriginalSize = size(data);

                % Handle grayscale inputs
                if size(data,3)==1
                    data = repmat(data,[1 1 3 1]);
                end

                % Obtain batch size of images.
                batchSize = size(data,4);

                % Resize images based on the resize method specified.
                if strcmp(resize,"letterbox")
                    [data, scale] = iResizePreservingAspectRatio(data,targetSize(1:2),0,batchSize);
                    info.Scale = scale;
                elseif strcmp(resize,"scale")
                    data = imresize(im2single(data),targetSize(1:2));
                    info.Scale = size(data,1:2)/targetSize(1:2);
                else
                    data = im2single(data);
                    info.Scale = 1;
                end

            end
        end
    end
end

%--------------------------------------------------------------------------
% Input validators
%--------------------------------------------------------------------------
function iMustBeUniqueNames(input)
     %Check for unique class names
     % input is a single char string, return
     if(ischar(input))
         return;
     end

     % For cell array of char strings, array of strings and categorical
     % arrays, check of uniqueness
     if(length(input)~=length(unique(input)))
        throwAsCaller(MException('vision:reidentification:duplicateClassNames',...
               vision.getMessage('vision:reidentification:duplicateClassNames')));
     end
end

%--------------------------------------------------------------------------
function iValidateNetwork(network)
    validateattributes(network,{'dlnetwork'}, ...
        {'scalar'},"reidentificationNetwork","network");
end

%--------------------------------------------------------------------------
function iValidateClassNames(input)

    % Check if input is a single char string.
    if(ischar(input))
        return;
    end

    % Check for cell array of char strings.
    if(iscell(input))
        if(all(cellfun(@ischar, input)) && isvector(input))
            return;
        end
    end

    % Check for string & categorical vectors.
    if( (isstring(input)||iscategorical(input)) && isvector(input))
        return;
    end


    throwAsCaller(MException('vision:reidentification:incorrectClassNameType',...
           vision.getMessage('vision:reidentification:incorrectClassNameType')));

end

%--------------------------------------------------------------------------
function iMustBeRGBSize(input)
     % The size must either be [] or (1,3) with the channel dim = 3.

     if ~isempty(input) && length(input) > 2
         isValidChannelDim = length(input)==3 && input(3)==3;
     else
         isValidChannelDim = false;
     end

     if ~isempty(input) && ~isValidChannelDim
        throwAsCaller(MException('vision:reidentification:incorrectInputSize',...
           vision.getMessage('vision:reidentification:incorrectInputSize')));
     end
end

%--------------------------------------------------------------------------
function iValidateImageInput(in)

    im = [];
    if isnumeric(in)
        im = in;
    elseif matlab.io.datastore.internal.shim.isDatastore(in)
        out = preview(in);
        if iscell(out)
            if isempty(out)
                im = [];
            else
                im = out{1};
            end
        else
            im = out;
        end
    end

    if ~iValidateImage(im)||isempty(im)
        throwAsCaller(MException('vision:reidentification:invalidImageInput',...
               vision.getMessage('vision:reidentification:invalidImageInput')));
    end

end

%--------------------------------------------------------------------------
function tf = iValidateImage(in)
    tf = isnumeric(in)&&...
         ndims(in)<=4 && ...
         (size(in,3)==3||size(in,3)==1); % Gray scale or RGB image.
end

%--------------------------------------------------------------------------
function iValidateLogicalFlag(in)
    validateattributes(in,{'logical'}, {'scalar','finite', 'real'});
end

%--------------------------------------------------------------------------
function tf = iValidateNormStats(stats)
    tf = isfield(stats, {'Mean','StandardDeviation'});
    if ~all(tf)
        error(message('vision:reidentification:invalidNormalizationStatisticsStruct'));
    end
    if ~isempty(stats.Mean)
        validateattributes(stats.Mean, {'numeric'}, {'nonempty','nonsparse',...
            'real','finite','nonnan'},"reidentificationNetwork","Mean");
        iValidateNormStatsSize(size(stats.Mean), 3);
    end
    if ~isempty(stats.StandardDeviation)
        validateattributes(stats.StandardDeviation, {'numeric'}, ...
            {'nonempty','nonsparse','real','finite','nonnan'},...
            "reidentificationNetwork","Mean");
        iValidateNormStatsSize(size(stats.StandardDeviation), 3);
    end
end

%--------------------------------------------------------------------------
function iValidateNormStatsSize(statsSize,inputChannelSize)
    if (numel(statsSize) == 2 && any(statsSize ~= [1 inputChannelSize])) || ...
       (numel(statsSize) == 3 && any(statsSize ~= [1 1 inputChannelSize])) || ...
        numel(statsSize) > 3
        error(message('vision:reidentification:invalidNormalizationStatisticsSize',inputChannelSize));
    end
end

%--------------------------------------------------------------------------
% Helper Functions
%--------------------------------------------------------------------------
function [resizedImage, scale] = iResizePreservingAspectRatio(img, targetSize, pad, batchSize)
    % Resize images during preprocessing while preserving aspect ratio.

    img = im2single(img);
    imgSize = size(img);

    % Compute Aspect Ratio.
    imgAspectRatio = imgSize(2)/imgSize(1);

    resizeRowsToTargetOutputSize = ceil(imgAspectRatio*targetSize(1));
    resizeColsToTargetOutputSize = ceil(targetSize(2)/imgAspectRatio);
    padSizeIfResizeRowsToTarget = resizeRowsToTargetOutputSize-targetSize(2);
    padSizeIfResizeColsToTarget = resizeColsToTargetOutputSize-targetSize(1);

    % Resize and pad image to final size.
    if padSizeIfResizeRowsToTarget < padSizeIfResizeColsToTarget
        scale = imgSize(1)/targetSize(1);
        resizedOriginalImage = imresize(img,[targetSize(1),nan]);
        resizedSize = size(resizedOriginalImage);
        resizedImage = ones([targetSize 3 batchSize],"like",img).*pad;
        startIdx = floor((targetSize(2) - resizedSize(2))*0.5);
        endIdx = startIdx + resizedSize(2) - 1;
        resizedImage(:,startIdx:endIdx,:,:) = resizedOriginalImage;
    elseif padSizeIfResizeColsToTarget < padSizeIfResizeRowsToTarget
        scale = imgSize(2)/targetSize(2);
        resizedOriginalImage = imresize(img,[nan,targetSize(2)]);
        resizedSize = size(resizedOriginalImage);
        resizedImage = ones([targetSize 3 batchSize],"like",img).*pad;
        startIdx = floor((targetSize(1) - resizedSize(1))*0.5);
        endIdx = startIdx + resizedSize(1) - 1;
        resizedImage(startIdx:endIdx,:,:,:) = resizedOriginalImage;
    else
        scale = imgSize(1)/targetSize(1);
        resizedImage = imresize(img,targetSize(1:2));
    end

end

%--------------------------------------------------------------------------
function classes = iFormatClasses(classes)
    if iscategorical(classes)
        classes = string(classes);
    end

    if isstring(classes)
        classes = cellstr(classes);
    end

    if ~isrow(classes)
        classes = classes';
    end
end

%--------------------------------------------------------------------------
function [network, neckNetwork, featureLength] = iConstructFeatureNetwork(network, featureLayer, featureLength)
    FeatureLayerOutputsVector = true;
    useGap = false;

    % Sort dlnetwork layers.
    network = deep.internal.sdk.toposort(network);

    if featureLayer == ""
        % Prioritize replacing the last fully connected layer in the network.
        fcIdx = arrayfun(@(x)isa(x,'nnet.cnn.layer.FullyConnectedLayer'),network.Layers);
        layerRemovalIdx = find(fcIdx,1,'last');
        if isempty(layerRemovalIdx)
            % If the network does not have a fully connected layer, attempt
            % to replace a singleton global average pooling layer and add a
            % fully connected layer.
            gpIdx = arrayfun(@(x)isa(x,'nnet.cnn.layer.GlobalAveragePooling2DLayer'),network.Layers);
            layerRemovalIdx = find(gpIdx,1,'last');

            if ~isempty(layerRemovalIdx)
                % Verify that the GAP layer produces a singleton vector. It
                % is unlikely that this will ever return false, but this is
                % in place for potential unseen edge cases.
                FeatureLayerOutputsVector = iCheckFeatureLayerDim(network,network.Layers(layerRemovalIdx).Name);
            else
                FeatureLayerOutputsVector = false;
            end

            if isempty(layerRemovalIdx) || ~FeatureLayerOutputsVector
                % If no fully connected layer or global average pooling
                % layer is found, or the GAP layer does not produce a
                % singleton vector, look for the last convolutional layer
                % and see if it returns a singleton vector.
                convIdx = arrayfun(@(x)isa(x,'nnet.cnn.layer.Convolution2DLayer'),network.Layers);
                layerRemovalIdx = find(convIdx,1,'last');
                if isempty(layerRemovalIdx)
                    error(message('vision:reidentification:invalidNetwork'));
                else
                    FeatureLayerOutputsVector = iCheckFeatureLayerDim(network,network.Layers(layerRemovalIdx).Name);
                    if ~FeatureLayerOutputsVector
                        error(message('vision:reidentification:invalidNetwork'));
                    end
                end
            else
                % Verify that at least one global average pooling layer was
                % found and confirm that it returns a one-dimensional output.
                FeatureLayerOutputsVector = iCheckFeatureLayerDim(network,network.Layers(layerRemovalIdx).Name);
                useGap = true;
            end
        end
    else
        [FeatureLayerOutputsVector, featureLength] = iCheckFeatureLayerDim(network,featureLayer);
        layerNames = string({network.Layers.Name});
        layerRemovalIdx = find(strcmp(featureLayer, layerNames),1,'last') + 1;
    end

    network = removeLayers(network, {network.Layers(layerRemovalIdx:end).Name});

    if featureLayer == "" && FeatureLayerOutputsVector
        % Construct the feature neck network.
        featureNeckLayers = iConstructFeatureNeck(featureLength, FeatureLayerOutputsVector, useGap);
    elseif FeatureLayerOutputsVector
        % Transfer the chosen feature layer to the feature neck for
        % training and feature extraction.
        featureNeckLayers = network.Layers(end);
        network = removeLayers(network,network.Layers(end).Name);
    else
        error(message('vision:reidentification:invalidFeatureLayer'));
    end
    neckNetwork = dlnetwork(featureNeckLayers, Initialize=false);
    network = initialize(network);
end

%--------------------------------------------------------------------------
function featureNeckLayers = iConstructFeatureNeck(featureLength, featureLayerOutputsVector, useGap)
    % Create the default feature extraction neck network.
    featureNeckLayers = [
        globalAveragePooling2dLayer(Name="avg_pool_features")
        fullyConnectedLayer(featureLength, Name="fc_num_features")
    ];

    if featureLayerOutputsVector && ~useGap
        % Use only a fully connected layer as the feature extraction neck.
        featureNeckLayers = featureNeckLayers(2);
    end

end

%--------------------------------------------------------------------------
function classificationNetwork = iDefaultClassificationNetwork(numClasses, featureLength)
    % Set up reidentification fully connected softmax layer.
    reIDFCSoftmaxLayer = iCreateFullyConnectedSoftmaxLayer(numClasses, featureLength);
    % Create the default classification head network.
    clsHeadLayers = [
        batchNormalizationLayer(Name="batchnorm_classHead")
        reluLayer(Name="relu_classHead")
        dropoutLayer(Name="dropout_classHead")
        reIDFCSoftmaxLayer
    ];

    classificationNetwork = dlnetwork(clsHeadLayers,Initialize=false);
end

%--------------------------------------------------------------------------
function classificationNetwork = iReplaceFullyConnectedLayer(classificationNet, numClasses, featureLength)
    % Verify that the classification network is a dlnetwork.
    if ~isa(classificationNet, "dlnetwork")
        error(message('vision:reidentification:invalidClassificationNetwork'));
    end

    % Find the last fully connected layer in the classification head.
    fcIdx = arrayfun(@(x)isa(x,'nnet.cnn.layer.FullyConnectedLayer'),classificationNet.Layers);

    if ~any(fcIdx)
        error(message('vision:reidentification:invalidClassificationNetworkMissingFC'));
    else
        layerRemovalIdx = find(fcIdx,1,'last');
    end

    % Remove all layers after the fully connected layer.
    if layerRemovalIdx ~= size(fcIdx)
        classificationNet = removeLayers(classificationNet,...
            {classificationNet.Layers(layerRemovalIdx+1:end).Name});
    end

    % Set up reidentification fully connected softmax layer.
    reIDFCSoftmaxLayer = iCreateFullyConnectedSoftmaxLayer(numClasses, featureLength);

    % Replace the fully connect layer with the custom reidentification
    % fully connected softmax layer.
    classificationNetwork = replaceLayer(classificationNet,...
        classificationNet.Layers(layerRemovalIdx).Name,reIDFCSoftmaxLayer);
end

%--------------------------------------------------------------------------
function reIDFCSoftmaxLayer = iCreateFullyConnectedSoftmaxLayer(numClasses, featureLength, lossFunction, inFormat)
    if ~exist("lossFunction","var")
        lossFunction = "additive-margin-softmax";
    end
    if ~exist("inFormat","var")
        inFormat = 'CB';
    end
    % Set up reidentification fully connected softmax layer.
    weights = randn([numClasses featureLength])*0.01;
    reIDFCSoftmaxLayer = vision.internal.reidentification.reidentificationFCSoftmaxLayer("fc_softmax",weights,lossFunction,inFormat);
    reIDFCSoftmaxLayer = setLearnRateFactor(reIDFCSoftmaxLayer,"K",3);
end

%--------------------------------------------------------------------------
function [tf, featureLength] = iCheckFeatureLayerDim(network, featureLayer)
    X = dlarray(ones(network.Layers(1).InputSize,"single"));
    sampleOutput = predict(network,X,Outputs=featureLayer);

    % Remove dimensions that are equal to 1.
    sampleOutput = squeeze(sampleOutput);
    if isrow(sampleOutput)
        sampleOutput = sampleOutput';
    end
    tf = ismatrix(sampleOutput) && size(sampleOutput,2) == 1;
    featureLength = size(sampleOutput,1);
end

%--------------------------------------------------------------------------
function network = iCombineTwoDLNetworks(net1,net2)
% Combine net1 and net2 into one dlnetwork. net2 is added to net1 in
% series, with each branch of net2 being connected to net1's single
% output. net1 has exactly one output and net2 can have an arbitrary number
% of inputs.

    % Add net2 to net1 without connected the two networks.
    network = iAddTwoUnconnectedDLNetworks(net1,net2);

    % Connect net1 with each of net2's branches.
    for i = 1:size(net2.InputNames,2)
        network = connectLayers(network,net1.Layers(end).Name,net2.InputNames{1,i});
    end
end

%--------------------------------------------------------------------------
function network = iAddTwoUnconnectedDLNetworks(net1,net2)
% Add two dlnetworks together without connecting the networks. net1 is the
% base dlnetwork and net2 is added alongside net1 with its own connections
% intact.

    % Individually add layers from net2 head to net1 to accurately
    % reconstruct net2 with its respective connections.
    network = net1;
    for i = 1:size(net2.Layers,1)
        network = addLayers(network, net2.Layers(i));
    end

    % Rebuild net2's connections in the combined network.
    for i = 1:size(net2.Connections,1)
        network = connectLayers(network,string(net2.Connections{i,1}), ...
            string(net2.Connections{i,2}));
    end
end

%--------------------------------------------------------------------------
function normStats = iFindNormalizationStatistics(layer,inNormStats,useLayerStats)
    % Gather the current input layer's normalization stats and method.
    if ~isempty(inNormStats)
        iValidateNormStats(inNormStats);
        useInStats = true;
        inputMean = inNormStats.Mean;
        inputStdDev = inNormStats.StandardDeviation;
    else
        useInStats = false;
    end
    inputNormMethod = layer.Normalization;

    % Find which stats must be updated.
    defaultStats = iGetDefaultNormStats;

    % If the two cases below are not met, then the input layer's current
    % stats are used, and should be [].
    switch inputNormMethod
        case 'zscore'
            if ~useInStats
                if useLayerStats
                    inputMean = layer.Mean;
                    inputStdDev = layer.StandardDeviation;
                else
                    inputMean = defaultStats.Mean;
                    inputStdDev = defaultStats.StandardDeviation;
                end
            end
        case 'zerocenter'
            if ~useInStats
                if useLayerStats
                    inputMean = layer.Mean;
                else
                    inputMean = defaultStats.Mean;
                end
                inputStdDev = [];
            end
        otherwise
            inputMean = [];
            inputStdDev = [];
    end

    % Reshape normalization stats if needed.
    if ~isempty(inputMean) && (numel(inputMean) == 2 || numel(inputMean) == 3)
        inputMean = reshape(inputMean, [1 1 3]);
    end
    if ~isempty(inputStdDev) && (numel(inputStdDev) == 2 || numel(inputStdDev) == 3)
        inputStdDev = reshape(inputStdDev, [1 1 3]);
    end
    normStats = struct('Normalization',inputNormMethod,'Mean',inputMean,'StandardDeviation',inputStdDev);
end

%--------------------------------------------------------------------------
function stats = iGetDefaultNormStats()
    % Mean used for normalization.
    stats.Mean = [123.675, 116.28, 103.53];
    % Standard deviation used for normalization.
    stats.StandardDeviation = [58.395, 57.12, 57.375];
end

%--------------------------------------------------------------------------
% Vebose Functions
%--------------------------------------------------------------------------
function iPrintHeader(printer)
    printer.linebreak();
    printer.printMessage('vision:reidentification:verboseHeaderExtraction');
    printer.print('--------------------------');
    printer.linebreak();
end

%--------------------------------------------------------------------------
function iUpdateMessage(printer, prevMessage, nextMessage)
    backspace = sprintf(repmat('\b',1,numel(prevMessage))); % Figure out how much to delete.
    printer.print([backspace nextMessage]);
end

%--------------------------------------------------------------------------
function nextMessage = iPrintInitProgress(printer, prevMessage, k)
    nextMessage = getString(message('vision:reidentification:verboseProgressTxt',k));
    iUpdateMessage(printer, prevMessage(1:end-1), nextMessage);
end

%--------------------------------------------------------------------------
function nextMessage = iPrintProgress(printer, prevMessage, k)
    nextMessage = getString(message('vision:reidentification:verboseProgressTxt',k));
    iUpdateMessage(printer, prevMessage, nextMessage);
end

%--------------------------------------------------------------------------
function iPrintFooter(printer)
    printer.linebreak();
    printer.printMessage('vision:reidentification:verboseFooter');
    printer.linebreak(2);
end
