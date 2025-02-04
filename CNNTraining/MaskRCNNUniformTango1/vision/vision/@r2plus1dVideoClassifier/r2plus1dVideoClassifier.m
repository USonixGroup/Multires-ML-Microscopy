classdef r2plus1dVideoClassifier < vision.internal.EnforceScalarValue & ...
        deep.internal.sdk.LearnableParameterContainer
%

%   Copyright 2021-2024 The MathWorks, Inc.

    properties
        % ModelName - The name of the trained video classifier as a
        %             string.
        ModelName string = "resnet-3d-18"
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
        % Learnables - The learnable parameters for the R(2+1)D video
        %              classifier.
        Learnables
        % State - State of the non-learnable parameters of the
        %         R(2+1)D video classifier.
        %
        State
    end

    properties (Dependent, Access = private)
        % Sequence length of the video classifier.
        SequenceLength
    end

    properties (Access = private)
        % R(2+1)D Network as a dlnetwork object.
        Network
        % Counter to indicate the number of frames collected
        % in VideoSequence
        NumFramesCollected = 0
        % Private copy of the VideoSequence property.
        PrivateVideoSequence = []
        % Classifier name
        ClassifierName
    end

    methods
        function this = r2plus1dVideoClassifier(classifierName,classes,varargin)
            vision.internal.requiresNeuralToolbox(mfilename);

            if nargin == 1 && isequal(classifierName,'none')
                % For loadobj to work, this just creates an empty object.
            elseif nargin < 1
                params.ModelName                    = "resnet-3d-18";
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
            data = iTripwireR2Plus1DResnet3D18();
            dlnet = data.Network;

            if params.DefaultConstructor
                this.Classes = data.Classes;
            else
                this.Classes = params.Classes;
            end

            this.ModelName = params.ModelName;
            this.InputNormalizationStatistics = params.InputNormalizationStatistics;
            this.InputSize = params.InputSize;
            this.NumFramesCollected = 0;
            this.PrivateVideoSequence = [];

            if params.DefaultConstructor
                this.Network = dlnet;
            else
                lgraph = layerGraph(dlnet);
                lgraph = iReplaceInputLayer(lgraph, lgraph.Layers(1), this.InputSize([1,2,4,3]));
                lgraph = iReplaceFinalConvLayer(lgraph, numel(this.Classes));
                this.Network = dlnetwork(lgraph);
            end
        end

        function video = preprocess(this, video)
            %preprocess Preprocess for r2plus1d video.
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
            s.PrivateVideoSequence         = this.PrivateVideoSequence;
            s.NumFramesCollected           = this.NumFramesCollected;
        end

    end

    methods(Static, Hidden)
        %------------------------------------------------------------------
        function this = loadobj(structVal)
            % Create an empty classifier object.
            this = r2plus1dVideoClassifier('none');

            this.ModelName                    = structVal.ModelName;
            this.ClassifierName               = structVal.ClassifierName;
            this.Network                      = structVal.Network;
            this.Classes                      = structVal.Classes;
            this.InputSize                    = structVal.InputSize;
            this.InputNormalizationStatistics = structVal.InputNormalizationStatistics;
            this.PrivateVideoSequence         = structVal.PrivateVideoSequence;
            this.NumFramesCollected           = structVal.NumFramesCollected;
        end
    end
end

%------------------------------------------------------------------
function lgraph = iReplaceFinalConvLayer(lgraph, numClasses)
    oldConvLayer = lgraph.Layers(end-1);

    if isequal(oldConvLayer.PaddingMode, 'same')
        padding = 'same';
    else
        padding = oldConvLayer.PaddingSize;
    end
    convLayer = convolution3dLayer(...
        oldConvLayer.FilterSize,...
        numClasses,...
        'Name', oldConvLayer.Name,...
        'Padding', padding, ...
        'Stride', oldConvLayer.Stride,...
        'DilationFactor', oldConvLayer.DilationFactor,...
        'WeightLearnRateFactor', oldConvLayer.WeightLearnRateFactor,...
        'WeightL2Factor', oldConvLayer.WeightL2Factor,...
        'BiasLearnRateFactor', oldConvLayer.BiasLearnRateFactor,...
        'BiasL2Factor', oldConvLayer.BiasL2Factor,...
        'WeightsInitializer', oldConvLayer.WeightsInitializer,...
        'BiasInitializer', oldConvLayer.BiasInitializer...
        );

    lgraph = replaceLayer(lgraph, oldConvLayer.Name, convLayer);
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
    validatestring(classifierName,{'resnet-3d-18'},mfilename,'classifierName',1);
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
        error(message('r2plus1d:r2plus1dVideoClassifier:invalidNormalizationStatistics'));
    end
    fieldNames = sort(string(fieldnames(stats)));
    expFieldNames = sort(["Min", "Max", "Mean", "StandardDeviation"]);
    if ~isequal(fieldNames(:), expFieldNames(:))
        error(message('r2plus1d:r2plus1dVideoClassifier:invalidNormalizationStatistics'));
    end

    if isempty(stats.Min)
        if xor(isempty(stats.Min), isempty(stats.Max))
            error(message('r2plus1d:r2plus1dVideoClassifier:minMaxMustBothBeEmptyOrNonEmpty'));
        end
    else
        classes = {'numeric'};
        attrs   = {'nonsparse', 'real', 'finite', 'row', 'numel', numChannels};

        validateattributes(stats.Min, classes, attrs, mfilename, 'Min');
        validateattributes(stats.Max, classes, attrs, mfilename, 'Max');
        if any(stats.Min >= stats.Max)
            error(message('r2plus1d:r2plus1dVideoClassifier:minMustBeLesserThanMax'));
        end
    end

    if isempty(stats.Mean)
        if xor(isempty(stats.Mean), isempty(stats.StandardDeviation))
            error(message('r2plus1d:r2plus1dVideoClassifier:meanStdMustBothBeEmptyOrNonEmpty'));
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
        error(message('r2plus1d:r2plus1dVideoClassifier:invalidClasses'));
    end

    names = string(value);
    if iHasDuplicates(value)
        error(message('r2plus1d:r2plus1dVideoClassifier:duplicateClasses'));
    end

    % There should be 1 or more classes
    if numel(names) < 1
        error(message('r2plus1d:r2plus1dVideoClassifier:multiClass'));
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
    % The statistics values here are from the R(2+1)D Depth-18
    % model pretrained on Kinetics-400 dataset.
    stats.Min               = [0,0,0];
    stats.Max               = [255,255,255];
    stats.Mean              = [0.43216, 0.394666, 0.37645];
    stats.StandardDeviation = [0.22803, 0.22145, 0.216989];
end

%------------------------------------------------------------------
function inputSize = iDefaultInputSize()
    % The input size value here is from the R(2+1)D Depth-18
    % model pretrained on Kinetics-400 dataset.
    inputSize = [112, 112, 3, 32];
end

%------------------------------------------------------------------
function data = iTripwireR2Plus1DResnet3D18()
    % Check if support package is installed
    breadcrumbFile = 'vision.internal.cnn.supportpackages.IsR2Plus1DInstalled';
    fullPath = which(breadcrumbFile);
    if isempty(fullPath)
        name     = 'Computer Vision Toolbox Model for R(2+1)D Video Classification';
        basecode = 'RD_VIDEO';

        throwAsCaller(MException(message('nnet_cnn:supportpackages:InstallRequired', mfilename, name, basecode)));
    else
        pattern = fullfile(filesep, '+vision','+internal','+cnn','+supportpackages','IsR2Plus1DInstalled.m');
        idx     = strfind(fullPath, pattern);
        matfile = fullfile(fullPath(1:idx), 'data', 'r2plus1dPretrained_3d18.mat');
        data    = load(matfile);
    end
end
