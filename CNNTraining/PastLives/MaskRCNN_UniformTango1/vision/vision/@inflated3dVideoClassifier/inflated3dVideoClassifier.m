classdef inflated3dVideoClassifier < vision.internal.EnforceScalarValue & ...
        deep.internal.sdk.LearnableParameterContainer
%

%   Copyright 2021-2024 The MathWorks, Inc.

    properties
        % ModelName - The name of the trained video classifier as a
        %             string.
        ModelName string = "googlenet-video"
    end

    properties (SetAccess = private)
        % Classes - The classes the video classifier is
        %           configured to train or classify (read-only).
        Classes categorical
    end

    properties (Dependent, SetAccess = private)
        % VideoSequence - The video sequence used by updateSequence and
        %                 classifySequence for streaming classification (read-only).
        VideoSequence

        % OpticalFlowSequence - The optical flow sequence used by updateSequence and
        %                       classifySequence for streaming classification (read-only).
        OpticalFlowSequence
    end

    properties (SetAccess = private)
        % InputSize - The input size of the video classifier
        %             (read-only).
        InputSize (1,4) double

        % InputNormalizationStatistics - The normalization statistics values for
        %                                normalizing the input data (read-only).
        InputNormalizationStatistics (1,1) struct

        % OpticalFlowMethod - The algorithm to calculate the optical flow data
        %                     (read-only).
        OpticalFlowMethod
    end

    properties (Dependent)
        % VideoLearnables - The learnable parameters for the video subnetwork of
        %                   the I3D Video Classifier.
        VideoLearnables

        % VideoState - State of the non-learnable parameters for the
        %              video subnetwork of the I3D Video Classifier.
        VideoState

        % OpticalFlowLearnables - The learnable parameters for the optical flow subnetwork of
        %                         the I3D Video Classifier.
        OpticalFlowLearnables

        % OpticalFlowState - State of the non-learnable parameters for the
        %                    video subnetwork of the I3D Video Classifier.
        OpticalFlowState
    end

    properties (Dependent, Access = private)
        % Sequence length of the video classifier.
        SequenceLength
    end

    properties (Access = private)
        % Inflated-3D Video Network as a dlnetwork object.
        VideoNetwork
        % Inflated-3D Optical Flow Network as a dlnetwork object.
        OpticalFlowNetwork
        % Counter to indicate the number of frames collected
        % in VideoSequence
        NumFramesCollected = 0
        % Private copy of the VideoSequence property.
        PrivateVideoSequence = []
        % Private copy of the OpticalFlowSequence property.
        PrivateOpticalFlowSequence = []
        % Classifier name
        ClassifierName
        % Optical Flow Object for estimateFlow.
        OpticalFlowObject 
    end

    methods
        function this = inflated3dVideoClassifier(classifierName,classes,varargin)
            vision.internal.requiresNeuralToolbox(mfilename);

            if nargin == 1 && isequal(classifierName,'none')
                % For loadobj to work, this just creates an empty object.
            elseif nargin < 1
                params.ModelName                    = "googlenet-video";
                params.InputNormalizationStatistics = iDefaultInputNormalizationStatistics();
                params.InputSize                    = iDefaultInputSize();
                params.DefaultConstructor           = true;

                this = initializeClassifier(this,params);
            else
                narginchk(2,inf);
                if nargin < 3
                    nvp = {};
                else
                    nvp = varargin;
                end

                params = iParseInputs(classifierName,classes,nvp{:});
                params.DefaultConstructor = false;

                this = initializeClassifier(this,params);
            end

            

        end

        %======================================================================
        % Setters and Getters.
        %======================================================================
        function val = get.VideoSequence(this)
            val = this.PrivateVideoSequence;
            if isempty(val) || this.NumFramesCollected == 0
                val = [];
            else
                numFrames = this.InputSize(4);

                if this.NumFramesCollected < numFrames
                    val = val(:,:,:,1:this.NumFramesCollected);
                end
            end
        end

        function val = get.OpticalFlowSequence(this)
            val = this.PrivateOpticalFlowSequence;
            if isempty(val) || this.NumFramesCollected == 0
                val = [];
            else
                numFrames = this.InputSize(4);

                if this.NumFramesCollected < numFrames
                    val = val(:,:,:,1:this.NumFramesCollected);
                end
            end
        end

        function val = get.SequenceLength(this)
            val = this.InputSize(4);
        end

        function val = get.VideoLearnables(this)
            val = this.VideoNetwork.Learnables;
        end

        function this = set.VideoLearnables(this,val)
            this.VideoNetwork.Learnables = val;
        end

        function val = get.VideoState(this)
            val = this.VideoNetwork.State;
        end

        function this = set.VideoState(this,val)
            this.VideoNetwork.State = val;
        end

        function val = get.OpticalFlowLearnables(this)
            if ~isempty(this.OpticalFlowNetwork)
                val = this.OpticalFlowNetwork.Learnables;
            else
                val = [];
            end
        end

        function this = set.OpticalFlowLearnables(this,val)
            if ~isempty(this.OpticalFlowNetwork)
                this.OpticalFlowNetwork.Learnables = val;
            end
        end

        function val = get.OpticalFlowState(this)
            if ~isempty(this.OpticalFlowNetwork)
                val = this.OpticalFlowNetwork.State;
            else
                val = [];
            end
            
        end

        function this = set.OpticalFlowState(this,val)
            if ~isempty(this.OpticalFlowNetwork)
                this.OpticalFlowNetwork.State = val;
            end
        end

        function this = set.ModelName(this,val)
            iValidateModelName(val);
            this.ModelName = string(val);
        end

        %======================================================================
        % Definition stubs for classifyVideoFile, classifySequence,
        % updateSequence, resetSequence, predict and forward.
        %======================================================================
        [label,score,labels] = classifyVideoFile(this, videoFilename, nvp);
        [label, score] = classifySequence(this);
        this = updateSequence(this,frame);
        this = resetSequence(this);
        varargout = predict(this,varargin);
        varargout = forward(this,varargin);
    end

    methods(Access = protected)
        function this = updateLearnableParameters(this, updater)
            this.VideoLearnables = updater.apply(this.VideoLearnables);
            this.OpticalFlowLearnables = updater.apply(this.OpticalFlowLearnables);
        end
    end

    methods(Access = private)

        function this = initializeClassifier(this, params)
            data     = iTripwireInflated3DGooglenetVideoFlow();
            videoNet = data.VideoNetwork;
            flowNet  = data.FlowNetwork;

            if params.DefaultConstructor
                this.Classes = data.Classes;
            else
                this.Classes = params.Classes;
            end

            this.ModelName                    = params.ModelName;
            this.ClassifierName               = params.ModelName;
            this.InputNormalizationStatistics = params.InputNormalizationStatistics;
            this.InputSize                    = params.InputSize;
            this.NumFramesCollected           = 0;
            this.PrivateVideoSequence         = [];
            this.PrivateOpticalFlowSequence   = [];
            this.OpticalFlowObject            = [];

            if isequal(this.ClassifierName, "googlenet-video")
                this.OpticalFlowMethod = 'none';
            else
                this.OpticalFlowMethod = 'Farneback';
                this.OpticalFlowObject = opticalFlowFarneback;
            end

            if params.DefaultConstructor
                this.VideoNetwork       = videoNet;
                if isequal(this.ClassifierName, "googlenet-video-flow")
                    this.OpticalFlowNetwork = flowNet;
                end
            else
                numClasses   = numel(this.Classes);
                flowChannels = 2;

                videoInputSize = this.InputSize([1,2,4,3]);
                flowInputSize  = [this.InputSize([1,2,4]), flowChannels];

                this.VideoNetwork       = iCreateNewDlnetwork(videoNet, videoInputSize, numClasses);
                if isequal(this.ClassifierName, "googlenet-video-flow")
                    this.OpticalFlowNetwork = iCreateNewDlnetwork(flowNet, flowInputSize, numClasses);
                end
            end
        end

        function data = preprocess(this, video, useFirstFrameInData)
            %preprocess Preprocess for inflated3d video and optical flow.
            inSize = size(video);
            hw = inSize(1:2);
            netSize = this.InputSize(1:2);
            if ~isequal(netSize, hw)
                video = imresize(video, netSize);
            end

            if isequal(this.OpticalFlowMethod, 'none')
                video = single(video);
                data = iNormalizeData(video,this.InputNormalizationStatistics.Video,this.InputSize(3));
            else
                if useFirstFrameInData
                    opticalFlow = opticalFlowFarneback;
                else
                    opticalFlow = this.OpticalFlowObject;
                end
                flow = iComputeFlow(video, opticalFlow, this.InputSize);

                video = single(video);
                flow = single(flow);

                video = iNormalizeData(video,this.InputNormalizationStatistics.Video,this.InputSize(3));
                flow  = iNormalizeData(flow,this.InputNormalizationStatistics.OpticalFlow,2);
                data = {video,flow};
            end
        end

    end

    %======================================================================
    % Save and Load.
    %======================================================================
    methods(Hidden)
        %------------------------------------------------------------------
        function s = saveobj(this)
            s.Version                      = 1.0;
            s.ModelName                    = this.ModelName;
            s.ClassifierName               = this.ClassifierName;
            s.VideoNetwork                 = this.VideoNetwork;
            s.OpticalFlowNetwork           = this.OpticalFlowNetwork;
            s.OpticalFlowMethod            = this.OpticalFlowMethod;
            s.OpticalFlowObject            = this.OpticalFlowObject;
            s.Classes                      = this.Classes;
            s.InputSize                    = this.InputSize;
            s.InputNormalizationStatistics = this.InputNormalizationStatistics;
            s.PrivateVideoSequence         = this.PrivateVideoSequence;
            s.PrivateOpticalFlowSequence   = this.PrivateOpticalFlowSequence;
            s.NumFramesCollected           = this.NumFramesCollected;
        end

    end

    methods(Static, Hidden)
        %------------------------------------------------------------------
        function this = loadobj(structVal)
            % Create an empty classifier object.
            this = inflated3dVideoClassifier('none');

            this.ModelName                    = structVal.ModelName;
            this.ClassifierName               = structVal.ClassifierName;
            this.VideoNetwork                 = structVal.VideoNetwork;
            this.OpticalFlowNetwork           = structVal.OpticalFlowNetwork;
            this.OpticalFlowMethod            = structVal.OpticalFlowMethod;
            this.OpticalFlowObject            = structVal.OpticalFlowObject;
            this.Classes                      = structVal.Classes;
            this.InputSize                    = structVal.InputSize;
            this.InputNormalizationStatistics = structVal.InputNormalizationStatistics;
            this.PrivateVideoSequence         = structVal.PrivateVideoSequence;
            this.PrivateOpticalFlowSequence   = structVal.PrivateOpticalFlowSequence;
            this.NumFramesCollected           = structVal.NumFramesCollected;
        end
    end
end

function lgraph = iReplaceFinalFCLayer(lgraph, numClasses)
    fcIdxes = find(arrayfun(@(x)isa(x,'nnet.cnn.layer.FullyConnectedLayer'), ...
        lgraph.Layers));
    fcLayer = lgraph.Layers(fcIdxes(end));
    newFc = fullyConnectedLayer(numClasses, ...
        'Name','newFullyConnectedLayer', ...
        'WeightLearnRateFactor',10, ...
        'BiasLearnRateFactor',10);
    lgraph = replaceLayer(lgraph,fcLayer.Name,newFc);
end

%------------------------------------------------------------------
function lgraph = iReplaceInputLayer(lgraph, oldInput, inputSize)
    if ~isequal(inputSize, oldInput.InputSize)
        inputLayer = image3dInputLayer(inputSize, 'Name', oldInput.Name, ...
            'Normalization', 'None');
        lgraph = replaceLayer(lgraph, oldInput.Name, inputLayer);
    end
end

%------------------------------------------------------------------
function params = iParseInputs(classifierName,classes,nvp)
    arguments
        classifierName
        classes
        nvp.InputNormalizationStatistics = iDefaultInputNormalizationStatistics();
        nvp.InputSize = iDefaultInputSize();
    end
    classifierName = validatestring(classifierName,["googlenet-video","googlenet-video-flow"],mfilename,'classifierName',1);
    classes = iValidateClasses(classes);

    iValidateInputSize(nvp.InputSize,classifierName);
    numChannels = nvp.InputSize(3);
    iValidateInputNormalizationStatistics(nvp.InputNormalizationStatistics, ...
        numChannels, classifierName);

    params.Classes = classes;
    params.ModelName = classifierName;

    % Double cast numeric values.
    fieldNames = sort(["Min", "Max", "Mean", "StandardDeviation"]);
    for ii = 1:numel(fieldNames)
        params.InputNormalizationStatistics.Video.(fieldNames(ii)) = ...
            double(nvp.InputNormalizationStatistics.Video.(fieldNames(ii)));

        params.InputNormalizationStatistics.OpticalFlow.(fieldNames(ii)) = ...
            double(nvp.InputNormalizationStatistics.OpticalFlow.(fieldNames(ii)));
    end

    params.InputSize = double(nvp.InputSize);

end

%------------------------------------------------------------------
function iValidateInputNormalizationStatistics(stats, numChannels, classifierName)
    if ~isscalar(stats) || ~isstruct(stats) || ...
            ~isfield(stats, 'Video') || ~isfield(stats, 'OpticalFlow')
        error(message('inflated3d:inflated3dVideoClassifier:invalidNormalizationStatistics'));
    end

    switch classifierName
        case "googlenet-video"
            % All the OpticalFlow fields must be empty, [].
            expFieldNames = sort(["Min", "Max", "Mean", "StandardDeviation"]);
            if ~all(arrayfun(@(x)isequal(stats.OpticalFlow.(x),[]), expFieldNames))
                error(message('inflated3d:inflated3dVideoClassifier:statsMustContainEmptyOpticalFlowFields', classifierName));
            end
            iValidateStatisticsStruct(stats.Video, numChannels);
        case "googlenet-video-flow"
            iValidateStatisticsStruct(stats.Video, numChannels);
            iValidateStatisticsStruct(stats.OpticalFlow, 2);
    end
end

%------------------------------------------------------------------
function iValidateStatisticsStruct(stats, numChannels)
    if ~isscalar(stats) || ~isstruct(stats)
        error(message('inflated3d:inflated3dVideoClassifier:invalidNormalizationStatistics'));
    end
    fieldNames = sort(string(fieldnames(stats)));
    expFieldNames = sort(["Min", "Max", "Mean", "StandardDeviation"]);
    if ~isequal(fieldNames(:), expFieldNames(:))
        error(message('inflated3d:inflated3dVideoClassifier:invalidNormalizationStatistics'));
    end

    if isempty(stats.Min)
        if xor(isempty(stats.Min), isempty(stats.Max))
            error(message('inflated3d:inflated3dVideoClassifier:minMaxMustBothBeEmptyOrNonEmpty'));
        end
    else
        classes = {'numeric'};
        attrs   = {'nonsparse', 'real', 'finite', 'row', 'numel', numChannels};

        validateattributes(stats.Min, classes, attrs, mfilename, 'Min');
        validateattributes(stats.Max, classes, attrs, mfilename, 'Max');
        if any(stats.Min >= stats.Max)
            error(message('inflated3d:inflated3dVideoClassifier:minMustBeLesserThanMax'));
        end
    end

    if isempty(stats.Mean)
        if xor(isempty(stats.Mean), isempty(stats.StandardDeviation))
            error(message('inflated3d:inflated3dVideoClassifier:meanStdMustBothBeEmptyOrNonEmpty'));
        end
    else
        classes = {'numeric'};
        attrs   = {'nonsparse', 'real', 'finite', 'row', 'numel', numChannels};
        validateattributes(stats.Mean, classes, attrs, mfilename, 'Mean');
        validateattributes(stats.StandardDeviation, classes, attrs, mfilename, 'StandardDeviation');
    end
end

%------------------------------------------------------------------
function iValidateInputSize(inputSize, classifierName)
    classes = {'numeric'};
    attrs   = {'positive', 'integer', 'nonsparse', 'row', 'numel', 4};

    validateattributes(inputSize, classes, attrs, mfilename, 'InputSize');

    numChannels = inputSize(3);

    if isequal(classifierName, "googlenet-video-flow")
       isRGBOrGray = numChannels == 1 || numChannels == 3;
       if ~isRGBOrGray
           error(message('inflated3d:inflated3dVideoClassifier:numChannelsMustBeRGBOrGrayForFlow', string(numChannels)));
       end
   end
end

%------------------------------------------------------------------
function value = iValidateClasses(value)

    if ~isvector(value) || ~iIsValidDataType(value)
        error(message('inflated3d:inflated3dVideoClassifier:invalidClasses'));
    end

    names = string(value);
    if iHasDuplicates(value)
        error(message('inflated3d:inflated3dVideoClassifier:duplicateClasses'));
    end

    % There should be 1 or more classes
    if numel(names) < 1
        error(message('inflated3d:inflated3dVideoClassifier:multiClass'));
    end

    value = categorical(names,names);
    value = value(:);

    function tf = iIsValidDataType(value)
        tf = iscellstr(value) || isstring(value);
    end

    function tf = iHasDuplicates(value)
        tf = ~isequal(value, unique(value, 'stable'));
    end
end

%------------------------------------------------------------------
function iValidateModelName(value)
    classes = {'string','char'};
    attrs   = {'scalartext'};
    validateattributes(value, classes, attrs, mfilename, 'ModelName');
end

%------------------------------------------------------------------
function stats = iDefaultInputNormalizationStatistics()
    % The statistics values here are from the Inflated-3D GoogleNet
    % video model pretrained on the Kinetics-400 dataset.
    stats.Video.Min               = [0,0,0];
    stats.Video.Max               = [255,255,255];
    stats.Video.Mean              = [];
    stats.Video.StandardDeviation = [];

    stats.OpticalFlow.Min               = [];
    stats.OpticalFlow.Max               = [];
    stats.OpticalFlow.Mean              = [];
    stats.OpticalFlow.StandardDeviation = [];
end

%------------------------------------------------------------------
function inputSize = iDefaultInputSize()
    % The statistics values here are from the Inflated-3D GoogleNet
    % video model pretrained on the Kinetics-400 dataset.
    inputSize = [224,224,3,64];
end

%------------------------------------------------------------------
function data = iTripwireInflated3DGooglenetVideoFlow()
    % Check if support package is installed
    breadcrumbFile = 'vision.internal.cnn.supportpackages.IsInflated3DInstalled';
    fullPath = which(breadcrumbFile);
    if isempty(fullPath)
        name     = 'Computer Vision Toolbox Model for Inflated-3D Video Classification';
        basecode = 'INFLATED_3D_VIDEO';

        throwAsCaller(MException(message('nnet_cnn:supportpackages:InstallRequired', mfilename, name, basecode)));
    else
        pattern = fullfile(filesep, '+vision','+internal','+cnn','+supportpackages','IsInflated3DInstalled.m');
        idx     = strfind(fullPath, pattern);
        matfile = fullfile(fullPath(1:idx), 'data', 'inflated3dPretrained_googlenet_videoflow.mat');
        data    = load(matfile);
    end
end

%------------------------------------------------------------------
function dlnet = iCreateNewDlnetwork(dlnet, inputSize, numClasses)
    lgraph = layerGraph(dlnet);
    lgraph = iReplaceInputLayer(lgraph, lgraph.Layers(1), inputSize);
    lgraph = iReplaceFinalFCLayer(lgraph, numClasses);
    dlnet  = dlnetwork(lgraph);
end

%------------------------------------------------------------------
function data = iNormalizeData(data,dataStats,numChannels)
    % Rescale the data in the range [-1,1]
    if isempty(dataStats.Min)
        data = rescale(data,-1,1);
    else
        [dataMin,dataMax] = iGetMinMax(dataStats, numChannels);
        data = rescale(data,-1,1,...
            'InputMax',dataMax,'InputMin',dataMin);
    end

    if ~isempty(dataStats.Mean)
        [mn,sd] = iGetMeanStd(stats, numChannels);

        % zscore normalization
        data = data - mn;
        data = data ./ sd;
    end
end


%------------------------------------------------------------------
function [minValue,maxValue ] = iGetMinMax(stats, numChannels)
    minValue = stats.Min;
    maxValue = stats.Max;
    minValue = reshape(minValue,1,1,numChannels);
    maxValue = reshape(maxValue,1,1,numChannels);
end

%------------------------------------------------------------------
function [mn,sd] = iGetMeanStd(stats, numChannels)
    mn = stats.Mean;
    sd = stats.StandardDeviation;
    mn = reshape(mn,1,1,numChannels);
    sd = reshape(sd,1,1,numChannels);
end

%------------------------------------------------------------------
function outData = iComputeFlow(data, opticalFlow, inputSize)
    numFrames = size(data,4);
    sz = [inputSize(1:2), 2, numFrames];
    outData = zeros(sz, 'like', data);

    numChannels = inputSize(3);
    isRGBOrGray = numChannels == 1 || numChannels == 3;
    if ~isRGBOrGray
        error(message('inflated3d:inflated3dVideoClassifier:numChannelsMustBeRGBOrGrayForFlow', string(numChannels)));
    end

    isrgb = numChannels == 3;

    for f = 1:numFrames
        if isrgb
            gray = rgb2gray(data(:,:,:,f));
        else
            gray = data(:,:,:,f);
        end
        flow = estimateFlow(opticalFlow,gray);

        outData(:,:,:,f) = cat(3,flow.Vx,flow.Vy);
    end
end
