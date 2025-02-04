classdef hrnetObjectKeypointDetector< deep.internal.sdk.LearnableParameterContainer
    %

    % Copyright 2023-2024 The MathWorks, Inc.

    properties(SetAccess = protected)

        % ModelName Name of the trained object keypoint detector.
        ModelName   char

        % Network is a dlnetwork object representing the HRNet Keypoint detection network.
        Network

        % keyPointClasses specifies the names of the keypoint classes that hrnet object
        % keypoint detector can detect.
        KeyPointClasses

        % An M-by-3 matrix defining the [height width channel] of image sizes used
        % to train the detector. If [height width] specified then it will
        % be assumed to be grayscale image. Default will be size of
        % image input layer of the input dlnetwork.
        InputSize

        % A J-by-2 matrix specifying the keypoint connection to draw the skeleton.
        KeypointConnections {mustBeNumeric,mustBeFinite,mustBeReal,mustBeInteger,mustBePositive,mustBeNonsparse, ivalidateKeypointConnections} = []
    end
    
    properties(GetAccess = public, SetAccess = public)

        % A scalar value to decide valid keypoints.
        Threshold {mustBeReal,mustBeFinite,mustBeFloat,mustBeValidThreshold,checkThreshold} = 0.2

    end

    properties(Dependent = true, Hidden=true)
        % These properties are accesed during training the
        % detector.
        % Layers is the array of layers in the HRNet dlnetwork.
        Layers


        % Learnables is the learnable parameters for the hrnet dlnetwork.
        Learnables

        % State is the state of the non-learnable parameters of the
        % HRNet dlnetwork.
        State

        % InputNames is the cell array of input names for the HRNet dlnetwork.
        InputNames

        % OutputNames is the cell array of output names for the HRNet dlnetwork.
        OutputNames
    end

    % This is deep.internal.sdk.LearnableParameterContainer interface
    methods(Access=protected)
        function self = updateLearnableParameters(self, updater)
            data = self.Learnables;
            data = updater.apply(data);
            self.Learnables = data;
        end
    end

    methods
        function this = hrnetObjectKeypointDetector(varargin)

            narginchk(0,8);
            if nargin < 1
                % Loads the default pretrained model.
                this = iTripwireDefaultHRNETKeypointModel;
            elseif isa(varargin{1,1},'string')||isa(varargin{1,1},'char')
                % Loads the pretrained model as specified in keypointDetectorName.
                params = hrnetObjectKeypointDetector.parsePretrainedDetectorInputs(varargin{:});
                this = iTripwireHRNETKeypointModel(params);
            elseif ~isa(varargin{1,1},'dlnetwork')
                if ~(isa(varargin{1,1},'string')||isa(varargin{1,1},'char'))
                    error(message('vision:hrnetObjectKeypoint:mustBeDetectorName'));
                end
            else
                % Creates hrnet object keypoint detector.
                narginchk(2,6);
                params = hrnetObjectKeypointDetector.parseDetectorInputs(varargin{:});
                this.Network = params.Network;
                this.KeyPointClasses = params.KeyPointClasses;
                this.InputSize = params.InputSize;
                this.ModelName = params.ModelName;
                this.KeypointConnections = params.KeypointConnections;
                this.Threshold = params.Threshold;
            end
        end


        function val = get.Layers(this)
            val = this.Network.Layers;
        end

        function this = set.Layers(this,val)
            this.Network.Layers = val;
        end
        
        function val = get.Learnables(this)
            val = this.Network.Learnables;
        end

        function this = set.Learnables(this,val)
            this.Network.Learnables = val;
        end

        function val = get.State(this)
            val = this.Network.State;
        end

        function this = set.State(this,val)
            this.Network.State = val;
        end

        function val = get.InputNames(this)
            val = this.Network.InputNames;
        end

        function this = set.InputNames(this,val)
            this.Network.InputNames = val;
        end

        function val = get.OutputNames(this)
            val = this.Network.OutputNames;
        end

        function this = set.OutputNames(this,val)
            this.Network.OutputNames = val;
        end

        function varargout = detect(this,I,bboxes,params)
            %

            % Copyright 2023 The MathWorks, Inc.

            arguments
                this   hrnetObjectKeypointDetector
                I      {iValidateInferenceInput,mustBeNonempty}
                bboxes {mustBeNumeric,mustBeReal,mustBeFinite,mustBePositive,mustBeNonsparse} = []
                params.MiniBatchSize = 32
                params.Acceleration string {mustBeTextScalar,mustBeMember(params.Acceleration,["auto","mex","none"])} = "auto"
                params.ExecutionEnvironment string {mustBeTextScalar,mustBeMember(params.ExecutionEnvironment,["auto","gpu","cpu"])} = "auto"
            end
            vision.internal.requiresNeuralToolbox(mfilename);
            vision.internal.cnn.validation.checkMiniBatchSize(params.MiniBatchSize,mfilename);
            if ~isnumeric(I) && ~iscell(I)
                iCheckDetectionInputDatastore(I, mfilename);
                keypoints = {};
                scores = {};
                validFlag = {};
                i = 1;
                while hasdata(I)
                    T = read(I);
                    if ~iscell(T)
                        error(message('vision:hrnetObjectKeypoint:datastore'));
                    end
                    img1 = T{1};
                    castToGpuArray = ~isgpuarray(img1);
                    iValidateInferenceInput(img1);
                    bboxes = T{2};
                    ivalidateBboxes(img1,bboxes);

                    if ~isempty(bboxes)
                        [preprocessedCropped, center, scale,bboxOffset] = preprocessHRNet(this,img1,bboxes,params.ExecutionEnvironment);
                        nbboxes = size(bboxes,1);
                        batchSize = params.MiniBatchSize;

                        if params.MiniBatchSize< nbboxes
                            nbatch = floor(nbboxes/batchSize);
                            lastBatch = mod(nbboxes,batchSize);
                            for j = 1:nbatch
                                networkOutput(:,:,:,batchSize*(j-1)+1:j*batchSize) = ...
                                    predict(this.Network,preprocessedCropped(:,:,:,batchSize*(j-1)+1:j*batchSize),'Acceleration',params.Acceleration);
                            end
                            if lastBatch~=0
                                networkOutput(:,:,:,batchSize*nbatch+1:nbboxes) = ...
                                    predict(this.Network,preprocessedCropped(:,:,:,batchSize*nbatch+1:nbboxes),'Acceleration',params.Acceleration);
                            end
                        else
                            networkOutput = predict(this.Network,preprocessedCropped,'Acceleration',params.Acceleration);
                        end
                        networkOutput = extractdata(networkOutput);
                        [keypoints{i,1},scores{i,1},validFlag{i,1}] = postprocessHRNet(this,networkOutput,center,scale,castToGpuArray,bboxOffset);
                    else
                        error(message('vision:hrnetObjectKeypoint:emptyBboxes'));
                    end
                    i = i+1;
                end
                varargout{1} = keypoints;
                varargout{2} = scores;
                varargout{3} = validFlag;
            else
                % Preprocess input image.
                if ~isempty(bboxes)
                    castToGpuArray = ~isgpuarray(I);
                    ivalidateBboxes(I,bboxes);
                    [preprocessedCropped, center, scale,bboxOffset] = preprocessHRNet(this,I,bboxes,params.ExecutionEnvironment);
                    nbboxes = size(bboxes,1);
                    batchSize = params.MiniBatchSize;

                    if params.MiniBatchSize< nbboxes
                        nbatch = floor(nbboxes/batchSize);
                        lastBatch = mod(nbboxes,batchSize);
                        for i = 1:nbatch
                            networkOutput(:,:,:,batchSize*(i-1)+1:i*batchSize) = ...
                                predict(this.Network,preprocessedCropped(:,:,:,batchSize*(i-1)+1:i*batchSize),'Acceleration',params.Acceleration);
                        end
                        if lastBatch~=0
                            networkOutput(:,:,:,batchSize*nbatch+1:nbboxes) = ...
                                predict(this.Network,preprocessedCropped(:,:,:,batchSize*nbatch+1:nbboxes),'Acceleration',params.Acceleration);
                        end
                    else
                        networkOutput = predict(this.Network,preprocessedCropped,'Acceleration',params.Acceleration);
                    end

                    networkOutput = extractdata(networkOutput);
                    [varargout{1},varargout{2},varargout{3}] = postprocessHRNet(this,networkOutput,center,scale,castToGpuArray,bboxOffset);
                else
                    error(message('vision:hrnetObjectKeypoint:emptyBboxes'));
                end
            end
        end

        function varargout = visibleKeypoints(this,keypoints,scores,validity)
            %

            % Copyright 2023 The MathWorks, Inc.
            
            nClasses = size(this.KeyPointClasses,1);  
            if iscell(keypoints) && iscell(scores) && iscell(validity)
                nPerson = size(keypoints,1);
                validKeypoints = cell(nPerson,1);
                validScores = cell(nPerson,1);
                visibleLabels = cell(nPerson,1);

                for i=1:nPerson
                    vkypt = keypoints{i};
                    vscore = scores{i};
                    nPerson = size(validity{i},2);
                    validateattributes(vkypt, {'numeric'}, ...
                        {'2d','ndims',2,'nonempty','nonsparse',...
                        'real','finite','positive'});
                    validateattributes(vscore, {'numeric'}, ...
                        {'2d','ndims',2,'nonempty','nonsparse',...
                        'real','finite','positive'});
                    validateattributes(validity{i}, {'logical'}, ...
                        {'2d','ndims',2,'nonempty','nonsparse',...
                        'real','finite'});
                    if size(validity{i},1) ~= nClasses || size(vscore,1) ~= nClasses || size(vkypt,1) ~= nClasses
                        error(message('vision:hrnetObjectKeypoint:invalidSizeKeypointsScoresValidity'));
                    end

                    if size(vscore,2) ~= nPerson || size(vkypt,2) ~= nPerson
                        error(message('vision:hrnetObjectKeypoint:invalidObjectCount'));
                    end

                    if size(vkypt,2)~=2
                        error(message('vision:hrnetObjectKeypoint:invalidKeypoints'));
                    end
                    validKeypoints{i} = vkypt(validity{i},:);
                    validScores{i} = vscore(validity{i});
                    visibleLabels{i} = this.KeyPointClasses(validity{i});
                end
                varargout{1} = validKeypoints;
                varargout{2} = validScores;
                varargout{3} = visibleLabels;
            else
                nPerson = size(validity,2);
                validKeypoints = cell(nPerson,1);
                validScores = cell(nPerson,1);
                visibleLabels = cell(nPerson,1);
                validateattributes(keypoints, {'numeric'}, ...
                    {'nonempty','nonsparse',...
                    'real','finite','positive'});
                validateattributes(scores, {'numeric'}, ...
                    {'2d','ndims',2,'nonempty','nonsparse',...
                    'real','finite','positive'});
                validateattributes(validity, {'logical'}, ...
                    {'2d','ndims',2,'nonempty','nonsparse',...
                    'real','finite'});
                if size(validity,1) ~= nClasses || size(scores,1) ~= nClasses || size(keypoints,1) ~= nClasses
                    error(message('vision:hrnetObjectKeypoint:invalidSizeKeypointsScoresValidity'));
                end

                if size(scores,2) ~= nPerson || size(keypoints,3) ~= nPerson || size(validity,2)~= nPerson
                    error(message('vision:hrnetObjectKeypoint:invalidObjectCount'));
                end

                if size(keypoints,2)~=2
                    error(message('vision:hrnetObjectKeypoint:invalidKeypoints'));
                end

                for i=1:nPerson
                    vkypt = keypoints(:,:,i);
                    validKeypoints{i} = vkypt(validity(:,i),:);
                    vscore = scores(:,i);
                    validScores{i} = vscore(validity(:,i));
                    visibleLabels{i} = this.KeyPointClasses(validity(:,i));
                end
                varargout{1} = validKeypoints;
                varargout{2} = validScores;
                varargout{3} = visibleLabels;
            end
        end
    end


    methods(Hidden)

        function dlnet = matlabCodegenPrivateNetwork(this)
            % Public matlabCodegenPrivateNetwork method for coder to
            % fetch the underlying dlnetwork
            dlnet = this.Network;
        end

        % Set Input Layer Normalization
        function self = setInputNormalization(self,stats)
            imageInputIdx = find(arrayfun( @(x)isa(x,'nnet.cnn.layer.ImageInputLayer'), ...
                self.Network.Layers));
            currentInputLayer = self.Network.Layers(imageInputIdx);
            map = iNormalizationStatsDictionary(stats);
            statsSet = map{currentInputLayer.Normalization};
            newInputLayer = imageInputLayer(self.InputSize,"Name",currentInputLayer.Name,...
                "Normalization",currentInputLayer.Normalization,...
                statsSet{:});
            net  = replaceLayer(self.Network ,self.Network.Layers(imageInputIdx).Name,newInputLayer);
            self.Network = initialize(net);
        end

        function [croppedImages, center, scale,bboxOffset] = preprocessHRNet(this,I,bboxes,executionEnvironment)

            % Convert to gpuArray based on executionEnvironment.
            if (strcmp(executionEnvironment,'auto') && canUseGPU) || strcmp(executionEnvironment,'gpu')
                I = gpuArray(I);
            end
            I = rescale(I);
            croppedImages = zeros(this.InputSize(1),this.InputSize(2),this.InputSize(3),size(bboxes,1),...
                class(I));
            center = zeros(size(bboxes,1),2);
            scale = zeros(size(bboxes,1),2);
            bboxOffset = zeros(size(bboxes,1),4);
            for k = 1:size(bboxes,1)
                box = bboxes(k,:);
                if box(1,4)==this.InputSize(1) && box(1,3)==this.InputSize(2)
                    % Crop the image based on bounding boxes if bounding
                    % box is same as network input size or image size.
                    croppedImages(:,:,:,k) = vision.internal.detector.cropImage(I, box, false);
                    bboxOffset(k,:) = box;
                else
                    [center(k,:),scale(k,:)] = iBoxToCenterScale(box,this.InputSize(1),this.InputSize(2));
                    trans = iGetAffineTransform(center(k,:), scale(k,:), [this.InputSize(1),this.InputSize(2)],false);
                    croppedImage = imwarp(I,trans,...
                        'OutputView',imref2d([this.InputSize(1) this.InputSize(2)]),...
                        'interpolationMethod','linear','FillValues',0);
                    croppedImages(:,:,:,k) = croppedImage;
                end
            end
            croppedImages = dlarray(single(croppedImages),'SSCB');
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

            % Compute the activations and state information from the network.
            network = detector.Network;
            [varargout{1:nargout}] = forward(network, dlX{:});
        end


        function [keypoints,score,visibilityFlag] = postprocessHRNet(this,networkOutput,center,scale,castToGpuArray,bboxOffset)
            nObjects = size(center,1);
            nKeypoints = size(this.KeyPointClasses,1);
            keypoints = zeros(nKeypoints,2,nObjects,'like',networkOutput);
            score = zeros(nKeypoints,nObjects,'like',networkOutput);
            visibilityFlag = true(nKeypoints,nObjects);
            [heatmapheight,heatmapWidth, numKeypoints,~] = size(networkOutput);
            for j = 1:nObjects
                [jointPts,detectionScore] = iPostPros(networkOutput(:,:,:,j),center(j,:), scale(j,:),heatmapheight,heatmapWidth, numKeypoints,bboxOffset(j,:));
                keypoints(:,:,j) = jointPts;
                score(:,j) = detectionScore;
                visibilityFlag(:,j) = score(:,j)>this.Threshold;
            end
            if castToGpuArray
                keypoints = gather(keypoints);
                score = gather(score);
            end
        end
    end

    %======================================================================
    % Save/Load
    %======================================================================
    methods(Hidden)
        function s = saveobj(this)
            s.Version               = 1.0;
            s.ModelName             = this.ModelName;
            s.Network               = this.Network;
            s.KeyPointClasses       = this.KeyPointClasses;
            s.InputSize             = this.InputSize;
            s.Threshold             = this.Threshold;
            s.KeypointConnections   = this.KeypointConnections;
        end
    end

    methods(Static, Hidden)
        function this = loadobj(s)
            try
                vision.internal.requiresNeuralToolbox(mfilename);
                network            = s.Network;
                keyPointClasses    = s.KeyPointClasses;
                inputSize          = s.InputSize;
                keypointConnections = s.KeypointConnections;
                this = hrnetObjectKeypointDetector(network,keyPointClasses,InputSize = inputSize,KeypointConnections = keypointConnections);
                this.ModelName     = s.ModelName;
                this.Threshold     = s.Threshold;
            catch ME
                rethrow(ME)
            end
        end

        function [X, Y] = preprocessHRNetForTraining(images,keypoints,boundingBox,inputSize, outputSize,numKeypoints)
            % Returns cropped images combined along the batch dimension in X and
            % generated heatmaps of keypoints along with weights in Y.
            miniBatchSize = size(images,1);
            X = zeros(inputSize(1),inputSize(2),inputSize(3),miniBatchSize,"single");
            Y = zeros(outputSize(1),outputSize(2),numKeypoints,2*miniBatchSize,"single");

            for k = 1:miniBatchSize
                I = rescale(images{k});
                if size(I,3)~=inputSize(3)
                    error(message('vision:hrnetObjectKeypoint:InvalidChannelImage'));
                end
                keypoint = keypoints{k}.keypoints{1};
                bbox = boundingBox{k};
                [center, scale] = iBoxToCenterScale(bbox,inputSize(1),inputSize(2));
                trans = iGetAffineTransform(center,scale,inputSize,false);
                imageCropped = imwarp(I,trans,...
                    "OutputView",imref2d([inputSize(1) inputSize(2)]),...
                    "interpolationMethod","linear","FillValues",0);
                for i=1:numKeypoints
                    keypointCurrent  = keypoint(i,1:2);
                    newJoint = [keypointCurrent(1) keypointCurrent(2)  1];
                    newJoint = trans.A * newJoint';
                    keypoint(i,1:2) = newJoint(1:2);
                end
                X(:,:,:,k) = single(imageCropped);
                [heatmaps,weights] = iGenerateHeatmap(single(keypoint),inputSize,outputSize);
                Y(:,:,:,k) = single(heatmaps);
                Y(:,:,:,miniBatchSize+k) = repmat(permute(weights,[2 3 1]),outputSize(1:2));
            end
        end
    end

    methods(Static, Hidden, Access = protected)
        %------------------------------------------------------------------
        % Parse and validate pretrained detector parameters.
        %------------------------------------------------------------------
        function params = parsePretrainedDetectorInputs(varargin)

            p = inputParser;
            if size(varargin,2) == 1
                % Parse inputs for this syntax:
                % detector = hrnetObjectKeypointDetector(keypointDetectorName).
                p.addRequired('keypointDetectorName');

                supportedNetworks = ["human-full-body-w32", "human-full-body-w48"];
                keypointDetectorName = validatestring(varargin{1,1}, supportedNetworks, mfilename, 'keypointDetectorName', 1);
                parse(p, varargin{:});
                params.KeypointDetectorName = char(p.Results.keypointDetectorName);
            elseif (size(varargin,2) == 3) &&...
                    ((isa(varargin{1,2},'string') && isscalar(varargin{1,2})) || (isa(varargin{1,2},'char') && isrow(varargin{1,2}))) ...
                    && (strcmp(varargin{1,2},'InputSize') || strcmp(varargin{1,2},'ModelName')|| ...
                    strcmp(varargin{1,2},'KeypointConnections'))
                % ModelName, InputSize, KeypointConnections can not be
                % configured without configuring keypointClasses
                error(message('vision:hrnetObjectKeypoint:requiresKeypointclasses'));
            else
                % Parse inputs for this syntax
                % detector = hrnetObjectKeypointDetector(keypointDetectorName,keypointClasses,Name=Value).
                p.addRequired('keypointDetectorName');
                p.addRequired('keypointClasses');
                supportedNetworks = ["human-full-body-w32", "human-full-body-w48"];
                keypointDetectorName = validatestring(varargin{1,1}, supportedNetworks, mfilename, 'keypointDetectorName', 1);

                p.addParameter('InputSize', [384,288,3]);
                p.addParameter('ModelName', '', @iAssertValidLayerName);
                p.addParameter('keypointConnections',[]);
                p.addParameter('Threshold',0.2);

                parse(p, varargin{:});

                params.KeypointClasses = p.Results.keypointClasses(:);
                iValidateClassNames( params.KeypointClasses);
                params.KeypointConnections = p.Results.keypointConnections;
                if ~isempty(params.KeypointConnections) && isnumeric(params.KeypointConnections(:))... 
                    && all(isfinite(params.KeypointConnections(:))) && isequal(size(params.KeypointConnections,2),2)
                    numKeypointClasses = size(params.KeypointClasses,1);
                    if numKeypointClasses<max(params.KeypointConnections(:))
                        error(message('vision:hrnetObjectKeypoint:invalidKeypointConnectionIndices'));
                    end
                end
                params.InputSize = iCheckInputSize(p.Results.InputSize);
                params.Threshold = p.Results.Threshold;
                params.ModelName = char(p.Results.ModelName);
                params.KeypointDetectorName = char(p.Results.keypointDetectorName);

                if strcmp(params.ModelName,'')
                    params.ModelName = p.Results.keypointDetectorName;
                end
            end
        end

        %------------------------------------------------------------------
        % Parse and validate detector parameters.
        %------------------------------------------------------------------
        function params = parseDetectorInputs(varargin)
            % Parse inputs for this syntax:
            % detector = hrnetObjectKeypointDetector(network,keyPointClasses,...
            % InputSize = inputSize,KeypointConnections=keypointConnections,...
            % ModelName=modelName,Threshold = threshold);
            p = inputParser;
            p.addRequired('Network');
            p.addRequired('KeyPointClasses');
            p.addParameter('InputSize', []);
            p.addParameter('ModelName', '', @iAssertValidLayerName);
            p.addParameter('KeypointConnections',[]);
            p.addParameter('Threshold',0.2);
            parse(p, varargin{:});

            params.KeyPointClasses = p.Results.KeyPointClasses(:);
            params.InputSize = p.Results.InputSize;
            params.Threshold = p.Results.Threshold;
            params.ModelName = char(p.Results.ModelName);
            iValidateClassNames(params.KeyPointClasses);
            params.Network = p.Results.Network;
            params.KeypointConnections = p.Results.KeypointConnections;
        end
    end
end

%--------------------------------------------------------------------------
function [center,scale] = iBoxToCenterScale(box,modelImageHeight,modelImageWidth)
% Convert bbox from [x, y, w, h] to center and scale.
% The center is the coordinates of the bbox center, and the scale is the
% bbox width and height normalized by a scale factor.

center = [0 0];
boxWidth = box(:,3);
boxHeight = box(:,4);
center(1) = box(:,1) + floor(boxWidth/2);
center(2) = box(:,2) + floor(boxHeight/2);


aspectRatio = modelImageWidth * 1.0 / modelImageHeight;
% Pixel std is 200.0, which serves as the normalization factor to
% to calculate bbox scales.
% https://github.com/leoxiaobin/deep-high-resolution-net.pytorch/blob/master/demo/demo.py#L180
pixelStd = 200;

if boxWidth > aspectRatio * boxHeight
    boxHeight = boxWidth * 1.0 / aspectRatio;
elseif boxWidth < aspectRatio * boxHeight
    boxWidth = boxHeight * aspectRatio;
end
scale = double([boxWidth * 1.0 / pixelStd, boxHeight * 1.0 / pixelStd]);

% 1.25 value is taken from author's github repo.
% https://github.com/leoxiaobin/deep-high-resolution-net.pytorch/blob/master/demo/demo.py#L189
scale = scale * 1.25;
end
%----------------------------------------------------------------------------------
function [targetCords,detectionConfScore] = iPostPros(networkOutput,center,scale,heatmapheight,heatmapWidth, numKeypoints,bboxOffset)
% Post processing of networkOutput to get axis points.
[confs,idx] = max(networkOutput,[],[1,2],'linear');
[y,x,~,~] = ind2sub(size(networkOutput),idx);
joints = permute(cat(1,x,y-1),[3,1,4,2]);
detectionConfScore = squeeze(confs);
detectionConfScore(~lt(detectionConfScore,1)) = 1;

heatMapOutputSize = [heatmapheight, heatmapWidth];
% Preallocate the targetCords.
targetCords = zeros(numKeypoints,2,'like',networkOutput);

if ~all(bboxOffset==0)
    % Skip the post processing steps when bounding box dimensions match the
    % network input or image dimensions. Calculate the target coordinate using
    % bounding box (offset).
    targetCords(:,1) = joints(:,1).*4 + bboxOffset(1)-1;
    targetCords(:,2) = joints(:,2).*4 + bboxOffset(2)-1;
else
    trans = iGetAffineTransform(center, scale, heatMapOutputSize,true);
    trans = trans.A(1:2,:);
    appendOne = ones(length(joints),1);
    joints1 = [joints appendOne];
    targetCords(:,1) = sum(joints1.*trans(1,:),2);
    targetCords(:,2) = sum(joints1.*trans(2,:),2);
    targetCords(~gt(targetCords,0)) = 1;
end
end


%--------------------------------------------------------------------------
function transformMatrix = iGetAffineTransform(center, scale, outputHeatMapSize,invAffineTransform)
% center: Center of the bounding box (x, y).
% scale: Scale of the bounding box wrt [width, height].
% outputHeatMapSize: Size of the destination heatmaps.
% invAffineTransform (boolean): Option to inverse the affine transform direction.
% (inv=False: src->dst or inv=True: dst->src).

% shift (0-100%): Shift translation ratio wrt the width/height.
shift = zeros(1,2,'like',center);

% pixel_std is 200 as per author and mmpose github
% https://github.com/open-mmlab/mmpose/blob/master/mmpose/core/post_processing/post_transforms.py.
scaleTmp = scale*200.0;
srcWidth = scaleTmp(1);
dstHeight = outputHeatMapSize(1);
dstWidth = outputHeatMapSize(2);

srcPoint = [1 , srcWidth * -0.5];
dstDir = double([1, dstWidth * -0.5]);

src = zeros(3, 2);
dst = zeros(3, 2);
src(1, :) = center + scaleTmp .* shift;
src(2, :) = center + srcPoint + scaleTmp .* shift;
dst(1, :) = [dstWidth * 0.5, dstHeight * 0.5];
dst(2, :) = [dstWidth * 0.5, dstHeight * 0.5] + dstDir;

src(3, :) = iGetThirdPoint(src(1,:), src(2,:));
dst(3, :) = iGetThirdPoint(dst(1,:), dst(2, :));

if invAffineTransform
    transformMatrix = fitgeotform2d(dst,src,"affine");
else
    transformMatrix = fitgeotform2d(src,dst,"affine");
end
end
%--------------------------------------------------------------------------
function thirdPoint =  iGetThirdPoint(a, b)
% To calculate the affine matrix, three pairs of points are required. This
% function is used to get the 3rd point, given 2D points a & b.
% The 3rd point is defined by rotating vector `a - b` by 90 degrees
% anticlockwise, using b as the rotation center.
% Args:
%     a : point(x,y)s
%     b : point(x,y)
% Returns:
%     The 3rd point.
direction = a - b;
thirdPoint = b + [-direction(2)-1, direction(1)+1];
end
%--------------------------------------------------------------------------
function map = iNormalizationStatsDictionary(stats)
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
function iValidateInferenceInput(x)

if isnumeric(x) % Includes gpuArray, dlarray
    % RGB images allowed
    vision.internal.inputValidation.validateImage(x);
    if ndims(x) > 4
        error(message('vision:hrnetObjectKeypoint:invalidInputDimensionality'));
    end
end

if ~isnumeric(x) && ~isdatastore(x)
    error(message('vision:hrnetObjectKeypoint:invalidInferenceInput'));
end
end
%--------------------------------------------------------------------------
function TF = isdatastore(x)
TF = isa(x,'matlab.io.Datastore') || isa(x,'matlab.io.datastore.Datastore');
end
%--------------------------------------------------------------------------
function mustBeValidThreshold(x)
if ~isscalar(x)
    error(message('vision:hrnetObjectKeypoint:thresholdMustBeScalar'));
end
end

%--------------------------------------------------------------------------
function iValidateClassNames(value)
if ~isvector(value) || ~iIsValidDataType(value)
    error(message('vision:hrnetObjectKeypoint:invalidKeypointClasses'));
end
if iHasDuplicates(value)
    error(message('vision:hrnetObjectKeypoint:nonUniqueKeypointClasses'));
end
if isempty(value)
    error(message('vision:hrnetObjectKeypoint:invalidKeypointClasses'));
end
end
%--------------------------------------------------------------------------
function tf = iIsValidDataType(value)
tf = iscategorical(value) || iscellstr(value) || isstring(value);
end

%--------------------------------------------------------------------------
function inputSize = iCheckInputSize(inputSize)

validateattributes(inputSize, {'numeric'}, ...
    {'2d','ncols',3,'ndims',2,'nonempty','nonsparse',...
    'real','finite','integer','positive','nrows',1,});

if rem(inputSize(1),32)~=0 || rem(inputSize(2),32)~=0
    error(message('vision:hrnetObjectKeypoint:invalidInputSize'));
end
end
%--------------------------------------------------------------------------
function tf = iHasDuplicates(value)
tf = ~isequal(value, unique(value, 'stable'));
end
%--------------------------------------------------------------------------
function iAssertValidLayerName(name)
nnet.internal.cnn.layer.paramvalidation.validateLayerName(name);
end
%--------------------------------------------------------------------------
function ivalidateBboxes(I,bboxes)
if size(bboxes,2)~=4
    error(message('vision:hrnetObjectKeypoint:invalidBBoxDimension'));
end
[height,width,~] = size(I);
if any(width<bboxes(:,3)) || any(height<bboxes(:,4))
    error(message('vision:hrnetObjectKeypoint:invalidBBoxSize'));
end
end
%--------------------------------------------------------------------------
function ivalidateKeypointConnections(connections)
if ~isempty(connections)
    if size(connections)~=2
        error(message('vision:hrnetObjectKeypoint:invalidConnections'));
    end
end
end
%--------------------------------------------------------------------------
function checkThreshold(threshold)
validateattributes(threshold, {'single', 'double'}, {'nonempty', 'nonnan', ...
    'finite', 'nonsparse', 'real', 'scalar', '>=', 0, '<=', 1});
end
%--------------------------------------------------------------------------
function iCheckDetectionInputDatastore(datastore, filename)
%CHECKDETECTIONINPUTDATASTORE Verify that the input datastore for
% detection is infact a datastore and its read method returns
% a numeric array or an M-by-N column table or cell, with images
% in the first column.
narginchk(2,2);
varDescription = 'Datastore input for detect';
varIndex       = 2;

try
    validateattributes(datastore, {'matlab.io.Datastore', 'matlab.io.datastore.Datastore'},...
        {'nonempty'}, filename, varDescription, varIndex);
catch me
    msg = message('vision:ObjectDetector:invalidDetectionInput');
    throw(addCause(MException(msg), me));
end

sampleData = iReadNonEmpty(datastore);

if isnumeric(sampleData)
    error(message('vision:hrnetObjectKeypoint:mustBeImageandBBox'));
else
    classes = {'cell', 'table'};
    attrs   = {'ndims', 2};
    varDescription = 'Read output of training datastore input';
    try
        validateattributes(sampleData, classes, attrs, filename, varDescription, varIndex);
    catch me
        msg = message('vision:ObjectDetector:invalidDetectionDatastoreReadOutput');
        throw(addCause(MException(msg), me));
    end
    if istable(sampleData)
        images = sampleData{1,1};
    else
        images = sampleData(1,1);
    end
    iCheckImages(images, filename);
end
end
%--------------------------------------------------------------------------
function iCheckImages(images, filename)
msg = message('vision:ObjectDetector:invalidDetectionDatastoreReadOutput');
if ~iscell(images)
    error(msg);
end
I = images{1};
if ndims(I) == 2
    nDims = 2;
else
    nDims = 3;
end
classes        = {'numeric'};
attrs          = {'nonempty', 'nonsparse', 'ndims', nDims};
varDescription = 'Images in read output of datastore';
varIndex       = 1;
try
    validateattributes(I, classes, attrs, filename, varDescription, varIndex);
catch me
    throw(addCause(MException(msg), me));
end
end
%--------------------------------------------------------------------------
function data = iReadNonEmpty(datastore)
cpy = copy(datastore);
reset(cpy);
data = {};
while hasdata(cpy) && isempty(data)
    data = read(cpy);
end
if isempty(data)
    error(message('vision:ObjectDetector:noDataFromDatastore'));
end
end
%--------------------------------------------------------------------------
function detector = iTripwireDefaultHRNETKeypointModel()
% Check if support package is installed
breadcrumbFile = 'vision.internal.cnn.supportpackages.IsKeypointDetectionInstalled';
fullPath = which(breadcrumbFile);
if isempty(fullPath)
    name     = 'Computer Vision Toolbox Model for Object Keypoint Detection';
    basecode = 'KEYPOINT_DETECTION';
    throwAsCaller(MException(message('nnet_cnn:supportpackages:InstallRequired', mfilename, name, basecode)));
else
    % Load pretrained network.
    pattern = fullfile(filesep, '+vision','+internal','+cnn','+supportpackages','IsKeypointDetectionInstalled.m');
    idx     = strfind(fullPath, pattern);
    matfile = fullfile(fullPath(1:idx), 'data', 'human-full-body-w32.mat');
    data = load(matfile);
    detector = data.detector;
end
end
%--------------------------------------------------------------------------
function detector = iTripwireHRNETKeypointModel(params)
% Check if support package is installed
breadcrumbFile = 'vision.internal.cnn.supportpackages.IsKeypointDetectionInstalled';
fullPath = which(breadcrumbFile);
if isempty(fullPath)
    name     = 'Computer Vision Toolbox Model for Object Keypoint Detection';
    basecode = 'KEYPOINT_DETECTION';
    throwAsCaller(MException(message('nnet_cnn:supportpackages:InstallRequired', mfilename, name, basecode)));
else
    % Load pretrained network.
    pattern = fullfile(filesep, '+vision','+internal','+cnn','+supportpackages','IsKeypointDetectionInstalled.m');
    idx     = strfind(fullPath, pattern);
    if strcmp(params.KeypointDetectorName, 'human-full-body-w32')
        matfile = fullfile(fullPath(1:idx), 'data', 'human-full-body-w32.mat');
    else
        matfile = fullfile(fullPath(1:idx), 'data', 'human-full-body-w48.mat');
    end
    data = load(matfile);
    if(isfield(params,'KeypointClasses'))
        keyPointClasses = params.KeypointClasses;
        network = iConfigureNetworkForTransferLearning(data.detector.Network,size(keyPointClasses,1),params.InputSize);
        detector = hrnetObjectKeypointDetector(network,keyPointClasses,InputSize=params.InputSize,KeypointConnections=params.KeypointConnections);
        detector.ModelName     = params.ModelName;
        detector.Threshold     = params.Threshold;
    else
        detector = data.detector;
    end
end
end
%---------------------------------------------------------------------------
function network = iConfigureNetworkForTransferLearning(hrnet,numKeypointClasses,inputSize)
hrnetLGraph = iUpdateFirstConvChannelsAndInputLayer(hrnet,inputSize);
outputLayerIdx = find(arrayfun(@(x)strcmp(x,hrnet.OutputNames{1}),{hrnet.Layers.Name}));
lastConvLayer = hrnetLGraph.Layers(outputLayerIdx);

updatedConvLayer = convolution2dLayer(lastConvLayer.FilterSize, numKeypointClasses,...
    'Name', 'finallayer', 'WeightsInitializer','narrow-normal');
    
lastLayerName = "TopLevelModule_final_layer";
layersName = {hrnetLGraph.Layers.Name};

% To determine the last convolutional layer of the network, search `lastLayerName`. 
% If `lastLayerName` is not available in the network, then find the actual last convolutional layer.
if isempty(find(arrayfun(@(x)strcmp(x,lastLayerName),layersName), 1))
lgraphAnalysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(hrnetLGraph);
convIdx = find(arrayfun(@(x)isa(x,'nnet.cnn.layer.Convolution2DLayer'),lgraphAnalysis.ExternalLayers));
lastLayerName = lgraphAnalysis.ExternalLayers(convIdx(end)).Name;
end

hrnetLGraph = replaceLayer(hrnetLGraph,lastLayerName,...
    updatedConvLayer);
network = dlnetwork(hrnetLGraph);
end
%--------------------------------------------------------------------------
function lgraph = iUpdateFirstConvChannelsAndInputLayer(dlnet,imageSize)
% This function update the channels of first conv layer if InputSize channel does not match with
% channels of first conv layer. It also update the imageInputLayer.

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
    error(message('vision:ssd:networkMustHaveConvLayers'));
end
lgraph = layerGraph(dlnet);
% If number of channels in imageSize is not equal to the channel count of
%  first convolutional layer. Update the channel count of first conv
%  layer and use values of properties as it is.Pyramid pooling concept
%  has been used for concatenating extra channel. Each extra channel is
%  mean of original (initial) channels of conv layer
%
%  Zhao, Hengshuang, et al. "Pyramid Scene Parsing Network." 2017 IEEE
% Conference on Computer Vision and Pattern Recognition (CVPR). IEEE, 2017.
if (~strcmp(numFirstConvLayerChannels,'auto'))
    if numFirstConvLayerChannels~=numChannel
        lastConvLayer = lgraph.Layers(convIdx,1);
        lastConvLayerWeights = lastConvLayer.Weights;
        meanChannelWeights = reshape(mean(lastConvLayerWeights,3),size(lastConvLayerWeights(:,:,1,:)));
        if numChannel>numFirstConvLayerChannels
            extraChanels = abs(numChannel-numFirstConvLayerChannels);
            extraChannelWeights = repmat(meanChannelWeights,1,1,extraChanels);
            updatedConvLayerWeights = cat(3,lastConvLayerWeights,extraChannelWeights);
        else
            updatedConvLayerWeights = repmat(meanChannelWeights,1,1,numChannel);
        end
        updatedConvLayer = convolution2dLayer(lastConvLayer.FilterSize, lastConvLayer.NumFilters, 'NumChannels', numChannel, ...
            'Stride',lastConvLayer.Stride,...
            'Padding',lastConvLayer.PaddingSize , ...
            'PaddingValue',lastConvLayer.PaddingValue,...
            'DilationFactor', lastConvLayer.DilationFactor, ...
            'Weights',updatedConvLayerWeights,...
            'Bias',lastConvLayer.Bias,...
            'WeightL2Factor',lastConvLayer.WeightL2Factor,...
            'BiasL2Factor',lastConvLayer.BiasL2Factor,...
            'WeightLearnRateFactor',lastConvLayer.WeightLearnRateFactor,...
            'BiasLearnRateFactor',lastConvLayer.BiasLearnRateFactor,...
            'Name', lastConvLayer.Name, ...
            'WeightsInitializer', lastConvLayer.WeightsInitializer, ...
            'BiasInitializer', lastConvLayer.BiasInitializer);
        lgraph = replaceLayer(lgraph,lgraph.Layers(convIdx).Name,...
            updatedConvLayer);
    end
end

inputLayer = lgraph.Layers(imageInputIdx,1);
if ~isequal(inputLayer.InputSize,imageSize)
    inputLayerChannel = inputLayer.InputSize(3);
    extraChannels = abs(numChannel - inputLayerChannel);
    % Update imageInputLayer properties value to incorporate,
    % detector's training imageSize
    updatedMean = iUpdateImageInputLayerStats(inputLayer.Mean,imageSize);
    updatedStandardDeviation = iUpdateImageInputLayerStats( ...
        inputLayer.StandardDeviation,imageSize);
    updatedMin = iUpdateImageInputLayerStats(inputLayer.Min,imageSize);
    updatedMax = iUpdateImageInputLayerStats(inputLayer.Max,imageSize);

    if numChannel>inputLayerChannel
        if ~isempty(updatedMean) && numel(updatedMean)~=1
            updatedMean = iUpdateChannelProperties( ...
                updatedMean,extraChannels);
        end
        if ~isempty(updatedStandardDeviation) && numel(updatedStandardDeviation)~=1
            updatedStandardDeviation = iUpdateChannelProperties( ...
                updatedStandardDeviation,extraChannels);
        end
        if ~isempty(updatedMin) && numel(updatedMin)~=1
            updatedMin = iUpdateChannelProperties( ...
                updatedMin,extraChannels);
        end
        if ~isempty(updatedMax) && numel(updatedMax)~=1
            updatedMax = iUpdateChannelProperties( ...
                updatedMax,extraChannels);
        end
    elseif numChannel< inputLayerChannel
        if ~isempty(updatedMean)  && numel(updatedMean)~=1
            updatedMean = repmat(mean(updatedMean,3),1,1,numChannel);
        end
        if ~isempty(updatedStandardDeviation) && numel(updatedStandardDeviation)~=1
            updatedStandardDeviation = repmat( ...
                mean(updatedStandardDeviation,3),1,1,numChannel);
        end
        if ~isempty(updatedMin)  && numel(updatedMin)~=1
            updatedMin = repmat(mean(updatedMin,3),1,1,numChannel);
        end
        if ~isempty(updatedMax) && numel(updatedMax)~=1
            updatedMax = repmat(mean(updatedMax,3),1,1,numChannel);
        end
    end
    imageInput = imageInputLayer(imageSize, ...
        'Normalization',inputLayer.Normalization,...
        'NormalizationDimension',inputLayer.NormalizationDimension,...
        'Mean',updatedMean ,...
        'StandardDeviation',updatedStandardDeviation,...
        'Min',updatedMin,...
        'Max',updatedMax,...
        'Name',lgraph.Layers(imageInputIdx).Name);

    lgraph = replaceLayer(lgraph,lgraph.Layers(imageInputIdx).Name,...
        imageInput);
end

end
%--------------------------------------------------------------------------
function extraChannelsOfImage = iUpdateChannelProperties(image, extraChannels)
% It add the extra channels to network's imageInputLayer to incorporate
% training Image inputSize channels.
imageStatsMean = repmat(mean(image,3),1,1,extraChannels);
extraChannelsOfImage = cat(3,image,imageStatsMean);
end
%------------------------------------------------------------------------------------------------------------------------------------------------------
function updatedImageStats = iUpdateImageInputLayerStats(imageStats,imageSize)
% It updates the imageInputLayer's  properties like mean, standard
% deviation, min, max based on the user InputSize

% 1-by-1-by-c array per channel or empty []  or a numeric scalar
if isequal([1 1],size(imageStats,1:2)) || isempty(imageStats) || numel(imageStats)==1
    updatedImageStats = imageStats;
else
    %  if value h-by-w-by-c array then resize according to inputSize
    updatedImageStats = imresize(imageStats,imageSize(1:2),"nearest");
end
end
%--------------------------------------------------------------------------
% HRNet-based keypoint detection for training typically follows a top-down approach. 
% Before training the model, ground truth keypoints must first be converted to heatmaps 
% to allow for proper regression. The size of the heatmap corresponds to the output size 
% of the HRNet network.
function [heatmaps,weights] = iGenerateHeatmap(keypoints,inputSize,outputSize)
heatmapSize = [outputSize(2),outputSize(1)];
sigma = 3;
featStride = [inputSize(2),inputSize(1)]./heatmapSize;
numKeypoints = size(keypoints,1);
heatmaps = zeros([heatmapSize(2) heatmapSize(1) numKeypoints]);

if size(keypoints,2) ==2
    weights = ones(numKeypoints,1);
else
    weights = keypoints(:,3);
end
tmpSize = sigma * 3;
for k = 1:numKeypoints
    muX = round(keypoints(k,1) ./ featStride(1)+0.5);
    muY = round(keypoints(k,2) ./ featStride(2)+0.5);
    upperLeft = [floor(muX - tmpSize), floor(muY - tmpSize)];
    bottomRight = [floor(muX + tmpSize + 1), floor(muY + tmpSize + 1)];
    if (upperLeft(1) >= heatmapSize(1) || upperLeft(2) >= heatmapSize(2) || ...
            bottomRight(1) <  0 ||  bottomRight(2) < 0)
        weights(k) = 0;
        continue
    end
    sizeRegion = 2 .* tmpSize + 1;
    [x,y] = meshgrid(1:sizeRegion,1:sizeRegion);
    x0 = floor(sizeRegion/2);
    y0 = x0;
    g = exp(-((x - x0).^2 + (y - y0).^2) ./ (2*(sigma^2)));
    gx = [max(0, -upperLeft(1)), min(bottomRight(1),heatmapSize(1))-upperLeft(1)-1] + 1;
    gy = [max(0, -upperLeft(2)), min(bottomRight(2),heatmapSize(2))-upperLeft(2)-1] + 1;
    imgx = [max(0, upperLeft(1)), min(bottomRight(1),heatmapSize(1))-1] + 1;
    imgy = [max(0, upperLeft(2)), min(bottomRight(2),heatmapSize(2))-1] + 1;
    if weights(k) > 0.5
        heatmaps(imgy(1):imgy(2), imgx(1):imgx(2),k) = g(gy(1):gy(2),gx(1):gx(2));
    end
end
end