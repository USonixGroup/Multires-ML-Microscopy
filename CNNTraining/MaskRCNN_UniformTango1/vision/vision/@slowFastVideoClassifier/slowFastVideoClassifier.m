classdef slowFastVideoClassifier < vision.internal.EnforceScalarValue & ...
        deep.internal.sdk.LearnableParameterContainer
%

%   Copyright 2021-2024 The MathWorks, Inc.


    properties
        % ModelName - The name of the trained video classifier as a
        %             string.
        ModelName string = "resnet50-3d"
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
    end

    properties (SetAccess = private)
        % InputSize - The input size of the video classifier
        %             (read-only).
        InputSize (1,4) double

        % InputNormalizationStatistics - The normalization statistics values for
        %                                normalizing the input data (read-only).
        InputNormalizationStatistics (1,1) struct
    end

    properties (Dependent)
        % Learnables - The learnable parameters for the SlowFast video
        %              classifier.
        Learnables
        % State - State of the non-learnable parameters of the
        %         SlowFast video classifier.
        %
        State
    end

    properties (Dependent, Access = private)
        % Sequence length of the fast pathway.
        SequenceLength
    end

    properties (Access = private)
        % Slow Fast Network as a dlnetwork object.
        Network
        % Alpha between slow and fast pathways.
        Alpha
        % Counter to indicate the number of frames collected
        % in VideoSequence
        NumFramesCollected = 0
        % Input size for the fast pathway.
        FastInputSize
        % Input size for the slow pathway.
        SlowInputSize
        % Private copy of the VideoSequence property.
        PrivateVideoSequence = []
        % Classifier name
        ClassifierName
    end

    methods
        function this = slowFastVideoClassifier(classifierName,classes,varargin)
            vision.internal.requiresNeuralToolbox(mfilename);

            if nargin == 1 && isequal(classifierName,'none')
                % For loadobj to work, this just creates an empty object.
            elseif nargin < 1
                params.ModelName                    = "resnet50-3d";
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

                params                    = iParseInputs(classifierName,classes,nvp{:});
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

        function val = get.SequenceLength(this)
            val = this.InputSize(4);
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
        varargout = predict(this,fast,varargin);
        varargout = forward(this,fast,varargin);
    end

    methods(Access = protected)
        function this = updateLearnableParameters(this, updater)
            this.Learnables = updater.apply(this.Learnables);
        end
    end

    methods(Access = private)

        function this = initializeClassifier(this, params)
            data = iTripwireSlowFastResnet50();
            dlnet = data.Network;

            if params.DefaultConstructor
                this.Classes = data.Classes;
            else
                this.Classes = params.Classes;
            end

            [this.SlowInputSize,this.FastInputSize,this.Alpha,slowInputLayer,fastInputLayer] = ...
                iCalculatedSlowFastPathwaySizes(dlnet.Layers);

            this.ModelName = params.ModelName;
            this.InputNormalizationStatistics = params.InputNormalizationStatistics;
            this.InputSize = params.InputSize;
            this.NumFramesCollected = 0;
            this.PrivateVideoSequence = [];

            if params.DefaultConstructor
                this.Network = dlnet;
            else
                fastInputSize = this.InputSize([1,2,4,3]);
                slowInputSize = fastInputSize;
                slowInputSize(3) = fastInputSize(3)/this.Alpha;

                lgraph = layerGraph(dlnet);
                lgraph = iReplaceInputLayer(lgraph, slowInputLayer, slowInputSize);
                lgraph = iReplaceInputLayer(lgraph, fastInputLayer, fastInputSize);

                this.FastInputSize = fastInputSize;
                this.SlowInputSize = slowInputSize;

                lgraph = iReplaceFinalConvLayer(lgraph, numel(this.Classes));
                this.Network = dlnetwork(lgraph);
            end
        end

        function video = preprocess(this, video)
            %preprocess Preprocess for slowFast video.
            inSize = size(video);
            hw = inSize(1:2);
            netSize = this.InputSize(1:2);
            if ~isequal(netSize, hw)
                video = imresize(video, netSize);
            end

            video = single(video);

            % Rescale the data in the range [0,1]
            if isempty(this.InputNormalizationStatistics.Min)
                video = rescale(video,0,1);
            else
                minVal = this.InputNormalizationStatistics.Min;
                maxVal = this.InputNormalizationStatistics.Max;

                % video is HWCT
                numChannels = this.InputSize(3);
                minVal = reshape(minVal,1,1,numChannels);
                maxVal = reshape(maxVal,1,1,numChannels);

                video = rescale(video,0,1,'InputMax',maxVal,'InputMin',minVal);
            end

            if ~isempty(this.InputNormalizationStatistics.Mean)
                mn = this.InputNormalizationStatistics.Mean;
                sd = this.InputNormalizationStatistics.StandardDeviation;

                % video is HWCT
                numChannels = this.InputSize(3);
                mn = reshape(mn,1,1,numChannels);
                sd = reshape(sd,1,1,numChannels);

                % zscore normalization
                video = video - mn;
                video = video ./ sd;
            end
        end

        function dlSlow = getSlowFromFast(this,dlFast)
            %getSlowFromFast Based on the Alpha value get slow
            % pathway frames from fast pathway frames.

            frameDim = 5;
            frames = size(dlFast,frameDim);
            slowFrames = 1:this.Alpha:frames;
            numDims = ndims(dlFast);
            subs(1:numDims) = {':'};
            subs{frameDim} = slowFrames;
            substr = substruct('()',subs);
            dlSlow = subsref(dlFast, substr);
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
            s.Network                      = this.Network;
            s.Classes                      = this.Classes;
            s.InputSize                    = this.InputSize;
            s.InputNormalizationStatistics = this.InputNormalizationStatistics;
            s.FastInputSize                = this.FastInputSize;
            s.SlowInputSize                = this.SlowInputSize;
            s.Alpha                        = this.Alpha;
            s.PrivateVideoSequence         = this.PrivateVideoSequence;
            s.NumFramesCollected           = this.NumFramesCollected;
        end

    end

    methods(Static, Hidden)
        %------------------------------------------------------------------
        function this = loadobj(structVal)
            % Create an empty classifier object.
            this = slowFastVideoClassifier('none');

            this.ModelName                    = structVal.ModelName;
            this.ClassifierName               = structVal.ClassifierName;
            this.Network                      = structVal.Network;
            this.Classes                      = structVal.Classes;
            this.InputSize                    = structVal.InputSize;
            this.InputNormalizationStatistics = structVal.InputNormalizationStatistics;
            this.FastInputSize                = structVal.FastInputSize;
            this.SlowInputSize                = structVal.SlowInputSize;
            this.Alpha                        = structVal.Alpha;
            this.PrivateVideoSequence         = structVal.PrivateVideoSequence;
            this.NumFramesCollected           = structVal.NumFramesCollected;
        end
    end
end

%------------------------------------------------------------------
function [slowInputSize,fastInputSize,alpha,sInputLayer,fInputLayer] = iCalculatedSlowFastPathwaySizes(layers)
    inputLayerIdx = find(arrayfun(@(x)isa(x,'nnet.cnn.layer.Image3DInputLayer'), layers));
    inputLayers = layers(inputLayerIdx);
    inputSizes = vertcat(inputLayers.InputSize);
    [~,sidx] = min(inputSizes(:,3));
    [~,fidx] = max(inputSizes(:,3));
    slowInputSize = inputSizes(sidx,:);
    fastInputSize = inputSizes(fidx,:);
    alpha = fastInputSize(3)/slowInputSize(3);
    sInputLayer = inputLayers(sidx);
    fInputLayer = inputLayers(fidx);
end

%------------------------------------------------------------------
function lgraph = iReplaceFinalConvLayer(lgraph, numClasses)
    oldConvLayer = lgraph.Layers(end-3);
    finalConvLayer = convolution3dLayer(...
        oldConvLayer.FilterSize,...
        numClasses,... % NumFilters
        'Name', oldConvLayer.Name, ...
        'DilationFactor', oldConvLayer.DilationFactor, ...
        'Stride', oldConvLayer.Stride, ...
        'Padding', oldConvLayer.PaddingMode);

    lgraph = replaceLayer(lgraph, oldConvLayer.Name, finalConvLayer);
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
    validatestring(classifierName,{'resnet50-3d'},mfilename,'classifierName',1);
    classes = iValidateClasses(classes);

    iValidateInputSize(nvp.InputSize);
    numChannels = nvp.InputSize(3);
    iValidateInputNormalizationStatistics(nvp.InputNormalizationStatistics, numChannels);

    params.Classes = classes;
    params.ModelName = classifierName;

    % Double cast numeric values.
    params.InputNormalizationStatistics.Min               = double(nvp.InputNormalizationStatistics.Min);
    params.InputNormalizationStatistics.Max               = double(nvp.InputNormalizationStatistics.Max);
    params.InputNormalizationStatistics.Mean              = double(nvp.InputNormalizationStatistics.Mean);
    params.InputNormalizationStatistics.StandardDeviation = double(nvp.InputNormalizationStatistics.StandardDeviation);
    params.InputSize                                      = double(nvp.InputSize);

end

%------------------------------------------------------------------
function iValidateInputNormalizationStatistics(stats, numChannels)
    if ~isscalar(stats) || ~isstruct(stats)
        error(message('slowfast:slowFastVideoClassifier:invalidNormalizationStatistics'));
    end
    fieldNames = sort(string(fieldnames(stats)));
    expFieldNames = sort(["Min", "Max", "Mean", "StandardDeviation"]);
    if ~isequal(fieldNames(:), expFieldNames(:))
        error(message('slowfast:slowFastVideoClassifier:invalidNormalizationStatistics'));
    end

    if isempty(stats.Min)
        if xor(isempty(stats.Min), isempty(stats.Max))
            error(message('slowfast:slowFastVideoClassifier:minMaxMustBothBeEmptyOrNonEmpty'));
        end
    else
        classes = {'numeric'};
        attrs   = {'nonsparse', 'real', 'finite', 'row', 'numel', numChannels};

        validateattributes(stats.Min, classes, attrs, mfilename, 'Min');
        validateattributes(stats.Max, classes, attrs, mfilename, 'Max');
        if any(stats.Min >= stats.Max)
            error(message('slowfast:slowFastVideoClassifier:minMustBeLesserThanMax'));
        end
    end

    if isempty(stats.Mean)
        if xor(isempty(stats.Mean), isempty(stats.StandardDeviation))
            error(message('slowfast:slowFastVideoClassifier:meanStdMustBothBeEmptyOrNonEmpty'));
        end
    else
        classes = {'numeric'};
        attrs   = {'nonsparse', 'real', 'finite', 'row', 'numel', numChannels};
        validateattributes(stats.Mean, classes, attrs, mfilename, 'Mean');
        validateattributes(stats.StandardDeviation, classes, attrs, mfilename, 'StandardDeviation');
    end
end

%------------------------------------------------------------------
function iValidateInputSize(inputSize)
    classes = {'numeric'};
    attrs   = {'positive', 'integer', 'nonsparse', 'row', 'numel', 4};

    validateattributes(inputSize, classes, attrs, mfilename, 'InputSize');
end

%------------------------------------------------------------------
function value = iValidateClasses(value)

    if ~isvector(value) || ~iIsValidDataType(value)
        error(message('slowfast:slowFastVideoClassifier:invalidClasses'));
    end

    names = string(value);
    if iHasDuplicates(value)
        error(message('slowfast:slowFastVideoClassifier:duplicateClasses'));
    end

    % There should be 1 or more classes
    if numel(names) < 1
        error(message('slowfast:slowFastVideoClassifier:multiClass'));
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
    % The statistics values here are from the SlowFast ResNet-50
    % model pretrained on Kinetics-400 dataset.
    stats.Min               = [0,0,0];
    stats.Max               = [255,255,255];
    stats.Mean              = [0.45,0.45,0.45];
    stats.StandardDeviation = [0.225,0.225,0.225];
end

%------------------------------------------------------------------
function inputSize = iDefaultInputSize()
    % The input size value here is from the SlowFast ResNet-50
    % model pretrained on Kinetics-400 dataset.
    inputSize = [256,256,3,32];
end

%------------------------------------------------------------------
function data = iTripwireSlowFastResnet50()
    % Check if support package is installed
    breadcrumbFile = 'vision.internal.cnn.supportpackages.IsSlowFastInstalled';
    fullPath = which(breadcrumbFile);
    if isempty(fullPath)
        name     = 'Computer Vision Toolbox Model for SlowFast Video Classification';
        basecode = 'SLOWFAST_VIDEO';

        throwAsCaller(MException(message('nnet_cnn:supportpackages:InstallRequired', mfilename, name, basecode)));
    else
        pattern = fullfile(filesep, '+vision','+internal','+cnn','+supportpackages','IsSlowFastInstalled.m');
        idx     = strfind(fullPath, pattern);
        matfile = fullfile(fullPath(1:idx), 'data', 'slowFastPretrained_resnet50.mat');
        data    = load(matfile);
    end
end
