function [detector,info] = trainRCNNObjectDetector(trainingData, network, options, varargin)

% Copyright 2017-2023 The MathWorks, Inc.

if nargin > 3
    [varargin{:}] = convertStringsToChars(varargin{:});
end

vision.internal.requiresStatisticsToolbox(mfilename);
vision.internal.requiresNeuralToolbox(mfilename);

% Initialize warning logger. Logs warnings issued during training and
% reissues them at end of training when Verbose is true.
vision.internal.cnn.WarningLogger.initialize();

[network, params] = parseInputs(trainingData, network, options, mfilename, varargin{:});

if ischar(network) || isstring(network)
    % Generate R-CNN network requested by user.
    lgraphOrLayers = vision.internal.cnn.RCNNLayers.create(params.NumClasses, network, 'rcnn');
        
    % Fill remaining training parameters.
    params = checkNetworkAndFillRemainingParameters(trainingData, lgraphOrLayers, params);
    
    if string(params.BoxRegressionLayer) == "auto" && string(network) == "squeezenet"
        % SqueezeNet uses a convolution layer as the classification layer
        % instead of a fully connected layer. Therefore, the auto selection
        % approach does not apply. Manually select the layer feeding the
        % last conv layer.
        params.BoxRegressionLayer = 'fire9-concat';
    end
else
    % Check network input by user.
    params = checkNetworkAndFillRemainingParameters(trainingData, network, params);
    
    try
        % Transform network input by user into R-CNN network.
        lgraphOrLayers = vision.internal.cnn.RCNNLayers.create(params.NumClasses, network, 'rcnn');
    catch ME
        throwAsCaller(ME);
        
    end
end
lgraphOrLayers = vision.internal.rcnn.removeAugmentationIfNeeded(lgraphOrLayers,'randcrop');

options = iAddMonitorMetrics(options, params);
[detector, ~, info] = rcnnObjectDetector.train(trainingData, lgraphOrLayers, options, params);

%--------------------------------------------------------------------------
function params = checkNetworkAndFillRemainingParameters(trainingData, lgraphOrLayers, params)
% Run network analyzer to get any network related errors.
analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(lgraphOrLayers);
analysis.applyConstraints();
try
    analysis.throwIssuesIfAny();
catch ME
    throwAsCaller(ME);
end

% Check BBox Feature Layer parameters. Auto selection of this parameter
% happens in rcnnObjectDetector.train() if not provided by user.
bboxFeatureLayerName = iCheckBBoxFeatureLayerName(params.BoxRegressionLayer, analysis, mfilename);
 
vision.internal.cnn.validation.checkNetworkLayers(analysis);

if ~params.IsNetwork
    % check if layers or layerGraph has correct number of output classes
    % for detection task (numClasses + 1 for background).
    vision.internal.cnn.validation.checkNetworkClassificationLayer(analysis, trainingData);
end

inputSize = vision.internal.cnn.imageInputSizeFromNetworkAnalysis(analysis);

params.InputSize          = inputSize;
params.BoxRegressionLayer = char(bboxFeatureLayerName);

%--------------------------------------------------------------------------
function [network, params] = parseInputs(trainingData, network, options, fname, varargin)

vision.internal.cnn.validation.checkGroundTruth(trainingData, fname);

network = vision.internal.cnn.validation.checkNetwork(network, fname, ...
    {'SeriesNetwork', 'nnet.cnn.layer.Layer','nnet.cnn.LayerGraph'}, ...
    vision.internal.cnn.RCNNLayers.SupportedPretrainedNetworks);

vision.internal.cnn.validation.checkTrainingOptions(options, fname);

if options.MiniBatchSize < 4
    error(message('vision:rcnn:miniBatchSizeTooSmall'));
end

p = inputParser;
p.addParameter('RegionProposalFcn', @rcnnObjectDetector.proposeRegions);
p.addParameter('UseParallel', vision.internal.useParallelPreference());
p.addParameter('PositiveOverlapRange', [0.5 1]);
p.addParameter('NegativeOverlapRange', [0.1 0.5]);
p.addParameter('NumStrongestRegions', 2000);
p.addParameter('BoxRegressionLayer', 'auto');
p.addParameter('ExperimentMonitor', 'none');
p.parse(varargin{:});

userInput = p.Results;


rcnnObjectDetector.checkRegionProposalFcn(userInput.RegionProposalFcn);

useParallel = vision.internal.inputValidation.validateUseParallel(userInput.UseParallel);

vision.internal.cnn.validation.checkOverlapRatio(userInput.PositiveOverlapRange, fname, 'PositiveOverlapRange');
vision.internal.cnn.validation.checkOverlapRatio(userInput.NegativeOverlapRange, fname, 'NegativeOverlapRange');

vision.internal.cnn.validation.checkStrongestRegions(p.Results.NumStrongestRegions, fname);

params.IsNetwork = isa(network,'SeriesNetwork') || isa(network,'DAGNetwork');

params.NumClasses = width(trainingData) - 1;
params.PositiveOverlapRange          = double(userInput.PositiveOverlapRange);
params.NegativeOverlapRange          = double(userInput.NegativeOverlapRange);
params.RegionProposalFcn             = userInput.RegionProposalFcn;
params.UsingDefaultRegionProposalFcn = ismember('RegionProposalFcn', p.UsingDefaults);
params.NumStrongestRegions           = double(userInput.NumStrongestRegions);
params.UseParallel                   = useParallel;
params.BackgroundLabel               = vision.internal.cnn.uniqueBackgroundLabel(trainingData);
params.Monitor                       = vision.internal.cnn.validation.checkExperimentMonitor(p, userInput, options, mfilename);

% BoxRegressionLayer will be validated later after network is transformed
% into R-CNN network or after network user input network is validated.
params.BoxRegressionLayer  = userInput.BoxRegressionLayer;

vision.internal.cnn.validation.checkPositiveAndNegativeOverlapRatioDoNotOverlap(params);

%--------------------------------------------------------------------------
function val = iCheckBBoxFeatureLayerName(val, networkAnalysis, fname)
validateattributes(val,{'string','char'},{'scalartext'},fname,'BoxRegressionLayer');
allNames = {networkAnalysis.ExternalLayers.Name};

% partial match not allowed for layer name value.
if string(val) == "auto"
    iErrorIfAnyLayerNameIs('auto',allNames);
else
    % Check that layer name exists in network.
    val = validatestring(val,allNames,fname,'BoxRegressionLayer');
end

%--------------------------------------------------------------------------
function iErrorIfAnyLayerNameIs(name,allNames)
if any(strcmp(name,allNames))
    error(message('vision:rcnn:autoOrNoneNotAllowedAsLayerName'))
end

%--------------------------------------------------------------------------
function options = iAddMonitorMetrics(options, params)
monitor = params.Monitor;
if isempty(monitor)
    return;
end
monitor.XLabel = 'Iteration';
monitor.Metrics = ["Loss", "Accuracy", "RMSE"];
monitor.Info = "LearnRate";
maxEpochs = options.MaxEpochs;

% If OutputFcn is already defined, pass that to the function
% recording metrics, so it can call the function before recording
% metrics.
if isempty(options.OutputFcn)
    userOutputFcn = @iPassThroughOutputFcn;
else
    userOutputFcn = options.OutputFcn;
end

% Map the metrics to the OutputFcn info field values.
% The first column contains the monitor metric names, the
% second column contains the info field names.
metricsToInfoMap = [
        "Loss","TrainingLoss";
        "RMSE","TrainingRMSE";
        "Accuracy","TrainingAccuracy";
        ];
    
options.OutputFcn = @(info)iRecordMonitorMetricsAndInfo(info, maxEpochs, ...
    monitor, userOutputFcn, metricsToInfoMap);
monitor.Status = "Start";

%--------------------------------------------------------------------------
function stop = iRecordMonitorMetricsAndInfo(info, maxEpochs, monitor, userOutputFcn, metricsToInfoMap)
stop = userOutputFcn(info) || monitor.Stop;
if stop
    return;
end

% If the State is "done", do not record metrics
% as it contains the last iteration metrics recorded
% previously.
if ~isequal(string(info.State), "done")
    metricsStruct = [];
    for ii = 1:size(metricsToInfoMap,1)
        metricName = metricsToInfoMap(ii,1);
        infoName = metricsToInfoMap(ii,2);
        val = info.(infoName);
        if ~isempty(val)
            metricsStruct.(metricName) = val;
        end
    end
    if ~isempty(metricsStruct)
        monitor.recordMetrics(info.Iteration, metricsStruct);
    end
    if ~isempty(info.BaseLearnRate)
        monitor.updateInfo("LearnRate", info.BaseLearnRate);
    end
end

progress = 100*(info.Epoch/maxEpochs);
monitor.Progress = progress;
% Use the info.State value as the Status, but with 
% upper case first character.
monitor.Status = firstCharUpper(info.State);

%--------------------------------------------------------------------------
function status = firstCharUpper(status)
if isempty(status)
    return;
end
status = char(status);
status = string([upper(status(1)), status(2:end)]);

%--------------------------------------------------------------------------
function stop = iPassThroughOutputFcn(~)
stop = false;
