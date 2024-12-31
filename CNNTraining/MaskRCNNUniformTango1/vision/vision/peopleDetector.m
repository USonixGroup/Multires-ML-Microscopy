classdef peopleDetector < vision.internal.EnforceScalarValue 
    %

    % Copyright 2024 The MathWorks, Inc.

    properties(SetAccess = protected, Hidden)
        BaseDetector
    end

    properties(SetAccess = protected)
        % ModelName (char vector)   A name for the detector.
        ModelName = '';
    end

    properties(SetAccess = protected)
        % ClassNames specifies 'person' class 
        ClassNames

        % InputSize is a vector of the form [height width] or [height width channels]
        % with channels as 3 always 
        % defining image size used to train the detector. During detection,
        % an input image is resized to this size before it is processed by
        % the detection network if AutoResize is set to true (default).
        InputSize
    end

    properties(Dependent = true, Hidden)
        % NormalizationStatistics specifies z-score normalization statitics
        % as a structure with fields, Mean and StandardDeviation specified
        % as 1-by-C array of means and standard deviation per channel. The
        % number of channels, C must match the InputSize
        NormalizationStatistics
    end

    properties(Dependent = true, Hidden = true)
        % InputNames is the cell array of input names for the peopleDetector BaseDetector dlnetwork.
        InputNames

        % OutputNames is the cell array of output names for the peopleDetector BaseDetector dlnetwork.
        OutputNames
    end

    properties (Access = protected, Transient)
        FilterBboxesFunctor
    end


    properties(Dependent, Hidden)
        % Layers is the array of layers in the peopleDetector.BaseDetector dlnetwork.
        Layers
    end

    methods
        function this = peopleDetector(detectorName,options)
            arguments
                detectorName {mustBeTextScalar, iMustBeValidDetectorName} = iGetSmallNetworkDetectorName();
                options.InputSize {mustBeNumeric, mustBePositive, mustBeReal, mustBeFinite, iMustBeValidInputSize} = []
                options.NormalizationStatistics = []
            end

            vision.internal.requiresNeuralToolbox(mfilename);
            if detectorName == "uninitialized"
                return
            end

            % Loads and configure the pretrained model as specified in detectorName.
            params = peopleDetector.parsePretrainedDetectorInputs(detectorName,options);

            % Display correct error message with RTMDet SPKG is not
            % installed when using peopleDetector
            try 
                this.BaseDetector = iTripwirePeopleDetectorModels(params);
            catch 
                error(message('vision:peopleDetector:installRTMDetSPKGForPeopleDetector'));
            end 
            this.ClassNames = {'person'};
            this.InputSize = params.InputSize(1:2);
            this.ModelName = params.DetectorName;
            this.NormalizationStatistics = this.BaseDetector.NormalizationStatistics;
            this.FilterBboxesFunctor = vision.internal.cnn.utils.FilterBboxesFunctor;

        end
    end

    methods
        function varargout = detect(detector, I, varargin)
            %

            % Copyright 2024 The MathWorks, Inc.

            % Validate I before performing detect using BaseDetector 
            validateattributes(I, {'numeric','matlab.io.Datastore','matlab.io.datastore.Datastore'},...
                {'nonempty','real', 'nonsparse'},...
                'detect','I');
            
            % Perform detection using the BaseDetector
            if ~isnumeric(I) % datastore input
                I = transform(I,@iGrayToRGBPreProcessForDatastore); % Transform gray scale image to RGB type 
                detections = detect(detector.BaseDetector, I, varargin{:});
                [bboxes, scores, labels] = deal(detections.Boxes, detections.Scores, detections.Labels);
            else
                if size(I,3)==1 % Transform gray scale image to RGB type 
                    I = repmat(I,[1 1 3]);
                end
                [bboxes, scores, labels] = detect(detector.BaseDetector, I, varargin{:});
            end
            
            % Post process detections
            [filteredBboxes,filteredScores,filteredLabels] = iPostProcessDetections(bboxes,scores,labels);
            
            % Assign filtered results to varargout
            if (~isnumeric(I))
                [detections.Boxes, detections.Scores, detections.Labels] = deal(filteredBboxes, filteredScores, filteredLabels);
                [varargout{1:nargout}] = detections;
            else
                detections = {filteredBboxes, filteredScores, filteredLabels};
                [varargout{1:nargout}] = deal(detections{1:nargout}); 
            end
            
        end
    end


    %----------------------------------------------------------------------
    methods
        function layers = get.Layers(this)
            layers = this.BaseDetector.Network.Layers;
        end

        %------------------------------------------------------------------
        function val = get.InputNames(this)
            val = this.BaseDetector.Network.InputNames;
        end

        %------------------------------------------------------------------
        function val = get.OutputNames(this)
            val = this.BaseDetector.Network.OutputNames;
        end

        %------------------------------------------------------------------
        function this = set.NormalizationStatistics(this,statsStruct)
            % set NormalizationStatistics for BaseDetector
            numChannel = 3; % numChannels is always 3 for BaseDetector 
            
            iValidateNormalizationStats(statsStruct, numChannel)
            
            this.BaseDetector.NormalizationStatistics = statsStruct;
        end

        function statsStruct = get.NormalizationStatistics(this)
            if isempty(this.BaseDetector)
                statsStruct = [];
            else
                statsStruct = this.BaseDetector.NormalizationStatistics;
            end
        end

    end


    methods(Static, Hidden, Access = protected)
        %------------------------------------------------------------------
        % Parse and validate pretrained detector parameters.
        %------------------------------------------------------------------
        function params = parsePretrainedDetectorInputs(detectorName,options)
            params = options;

            % Parse inputs for this syntax:
            % detector = peopleDetector(detectorName).
            params.DetectorName = detectorName;
            % Default input size.
            inputSize = [640 640 3];
           
            if isempty(options.InputSize)
                params.InputSize = inputSize;
            else
                params.InputSize = [options.InputSize 3];
            end

            iCheckInputSize(params.InputSize);

            if ~isempty(params.NormalizationStatistics)
                iValidateNormalizationStats(params.NormalizationStatistics,params.InputSize(3))
            end
        end
    end
    %======================================================================
    % Save/Load
    %======================================================================
    methods(Hidden)
        function s = saveobj(this)
            s.Version                      = 1.0;
            s.ModelName                    = this.ModelName;
            s.BaseDetector                 = this.BaseDetector;
            s.ClassNames                   = this.ClassNames;
            s.InputSize                    = this.InputSize;
            s.FilterBboxesFunctor          = this.FilterBboxesFunctor;
        end
    end

    methods(Static, Hidden)
        function this = loadobj(s)
            try
                vision.internal.requiresNeuralToolbox(mfilename);
                this = peopleDetector("uninitialized");
                this.BaseDetector             = s.BaseDetector;
                this.ClassNames              = s.ClassNames;
                this.InputSize               = s.InputSize;
                this.ModelName               = s.ModelName;
                this.FilterBboxesFunctor     = s.FilterBboxesFunctor;
                this.NormalizationStatistics = this.BaseDetector.NormalizationStatistics;

            catch ME
                rethrow(ME)
            end
        end
    end

end

%--------------------------------------------------------------------------
function iCheckInputSize(inputSize)
    validateattributes(inputSize, {'numeric'}, ...
        {'2d','nonempty','nonsparse',...
        'real','finite','integer','positive','nrows',1});
end


%--------------------------------------------------------------------------
function [filteredBboxes,filteredScores,filteredLabels] = iPostProcessDetections(bboxes,scores,labels)
% Filter boxes, scores and labels only for person class
if iscell(bboxes)
    filteredBboxes = cell(size(bboxes));
    filteredScores = cell(size(scores));
    filteredLabels = cell(size(labels));
    for i = 1:length(bboxes)
        % Filter boxes, scores, and labels only for person class for each image
        personClassIdx = strcmp('person', string(labels{i}));
        filteredBboxes{i} = bboxes{i}(personClassIdx,:);
        filteredScores{i} = scores{i}(personClassIdx);
        filteredLabels{i} = categorical(repmat({'person'}, size(filteredScores{i})));
    end
else
    % Single image detection, bboxes is an array
    % Filter boxes, scores, and labels only for person class
    personClassIdx = strcmp('person', string(labels));
    filteredBboxes = bboxes(personClassIdx,:);
    filteredScores = scores(personClassIdx);
    filteredLabels = categorical(repmat({'person'}, size(filteredScores)));
end

end

%--------------------------------------------------------------------------
function det = iTripwirePeopleDetectorModels(params)
% Translate peopleDetector detectorNames to RTMDet detectorNames 
networkNames  = ["large-network","medium-network","small-network","tiny-network"];
if ismember(params.DetectorName,networkNames)
    params.DetectorName = strcat(params.DetectorName,'-coco');
end

det = rtmdetObjectDetector(params.DetectorName, "InputSize",params.InputSize, ...
    "NormalizationStatistics",params.NormalizationStatistics);  
end

function detectorName = iGetSmallNetworkDetectorName()
detectorName = "small-network";
end

function iValidateNormalizationStats(stats,inputChannelSize)
mustBeA(stats,"struct");
mustBeScalarOrEmpty(stats)
tf = isfield(stats, {'Mean','StandardDeviation'});

if ~all(tf)|| ~all(ismember(fieldnames(stats),{'Mean','StandardDeviation'}))
    error(message('vision:peopleDetector:invalidNormalizationStatisticsStruct'));
end

meanSize = size(stats.Mean);
stdSize = size(stats.StandardDeviation);
iValidateNormalizationStatsSize(meanSize,inputChannelSize);
iValidateNormalizationStatsSize(stdSize,inputChannelSize);

end

function iValidateNormalizationStatsSize(statsSize,inputChannelSize)
if (numel(statsSize) == 2 && any(statsSize ~= [1 inputChannelSize])) || ...
        (numel(statsSize) == 3 && any(statsSize ~= [1 1 inputChannelSize])) || ...
        numel(statsSize) > 3
    error(message('vision:peopleDetector:invalidNormalizationStatisticsSize',inputChannelSize));
end
end

function iMustBeValidInputSize(input)         
     if~isempty(input)  
        % Only gray scale and RGB supported 
        isValidChannelDim = (size(input,2)==2);
     else
         isValidChannelDim = false;
     end
     
     if ~(isempty(input) || (isValidChannelDim))
        throwAsCaller(MException('vision:peopleDetector:incorrectInputSize',...
           vision.getMessage('vision:peopleDetector:incorrectInputSize')));
     end
end

function iMustBeValidDetectorName(input) 
    % if detectorName is "uninitialized" return 
    if strcmp(input,"uninitialized")
        return
    end

    % validate detectorName to be a valid user visible detector type
    % For 24b supporting both types of detector name 
    % "large-network-coco","medium-network-coco","small-network-coco","tiny-network-coco"
    % "large-network","medium-network","small-network","tiny-network"
    supportedNetworks = ["large-network-coco","medium-network-coco","small-network-coco",...
        "tiny-network-coco","large-network","medium-network","small-network","tiny-network"];
    mustBeMember(input, supportedNetworks);               
end

function in = iGrayToRGBPreProcessForDatastore(in)
    if isnumeric(in) && size(in,3)==1
        in = repmat(in, [1, 1, 3]);
    end
    if iscell(in) % combine datastore 
        in = in(:,1);
        if size(in,3)==1
         in = repmat(in{:}, [1, 1, 3]);
         in = {in};
        end
    end
end