function [trainedNet, info] = trainNetwork(ds, lgraph, opts, executionSettings,...
    mapping, checkpointSaver, summaryMonitorInfo, infoContent, columnStrategy, ...
    axesConfig, batchFcnStruct, validationPredictStrategyFcn, infoToSummaryMap)
% trainNetwork   Train a MIMO network.

%   Copyright 2016-2022 The MathWorks, Inc.

switch nargin
    case 10
        % Set to empty struct. Training will use default batching functions for
        % creating training batches from columns of data returned by
        % MiniBatchable datastores.
        batchFcnStruct = struct();
        validationPredictStrategyFcn = @(varargin)[];
        infoToSummaryMap = [];
    case 11
        % Set validationPredictStrategy to empty function handle. Training will
        % not be using any validation data.
        validationPredictStrategyFcn = @(varargin)[];
        infoToSummaryMap = [];
    case 12
        % infoToSummaryMap empty as OutputFcn is not being used.
        infoToSummaryMap = [];
end

if isstruct(summaryMonitorInfo)
    summaryBuilderFcn = summaryMonitorInfo.SummaryBuilderFcn;
    monitor = summaryMonitorInfo.Monitor;
    monitorFieldInformation = summaryMonitorInfo.MonitorFieldInformation;
    % Include training options in monitorFieldInformation. 
    monitorFieldInformation.trainingOptions = opts;
else
    summaryBuilderFcn = summaryMonitorInfo;
    monitor = [];
    monitorFieldInformation = [];
end

analyzedLayers = nnet.internal.cnn.analyzer.NetworkAnalyzer(lgraph);
internalLayers = analyzedLayers.InternalLayers;
isaDAG = true;
networkInfo = nnet.internal.cnn.util.ComputeNetworkInfo(isaDAG,internalLayers);

% Assemble internal network
strategy = nnet.internal.cnn.assembler.NetworkAssemblerStrategyFactory...
	.createStrategy(~networkInfo.IsDAG);
assembler = nnet.internal.cnn.assembler.TrainingNetworkAssembler(strategy);    
trainedNet = assembler.assemble(analyzedLayers, executionSettings);

% Retrieve the network input and output size for validation
networkInfo = setNetworkSize(networkInfo,trainedNet);

% Create training dispatcher.
trainingDispatcher = iCreateTrainingDataDispatcher(ds, mapping, trainedNet,...
    opts, executionSettings, batchFcnStruct);

% Store any classification labels or response names in the appropriate
% output layers.
for i = 1:trainedNet.NumOutputs
    trainedNet.OutputLayers{i} = ...
        trainedNet.OutputLayers{i}.storeResponseMetaData( ...
            trainingDispatcher.ResponseMetaData(i) );
end

validationDispatcher = iCreateValidationDispatcher(mapping, trainedNet,...
    opts, executionSettings, batchFcnStruct);

% Create reporters.
[reporter,trainingPlotReporter] = iOptionalReporters(opts, analyzedLayers, ...
    assembler, checkpointSaver, axesConfig, columnStrategy, ...
    executionSettings, networkInfo, trainingDispatcher ,validationDispatcher, ...
    validationPredictStrategyFcn, infoToSummaryMap);

% Add monitor stop interrupt, if any.
addMonitorReporter(reporter,monitor,monitorFieldInformation);

errorState = nnet.internal.cnn.util.ErrorState();
cleanup = onCleanup(@()iFinalizePlot(trainingPlotReporter, errorState));

% Add info recorder
infoRecorder = nnet.internal.cnn.util.traininginfo.Recorder(infoContent);
reporter.add(infoRecorder)

% Create dispatcher for network pre-processing.
inputStatsDispatcher = iCreateInitializationDispatcher(ds, mapping, trainedNet,...
    opts, executionSettings, batchFcnStruct, trainedNet, networkInfo);

% Create the summary function. Pass in the trainedNet in case the summary
% builder function needs to determine the order of the network outputs (as
% is the case for MIMO networks like Faster R-CNN).
fcn = summaryBuilderFcn(trainedNet, executionSettings);

% Create the trainer
trainer = iCreateTrainer(opts, reporter, fcn, executionSettings);

if opts.MiniBatchSize == 1 && ...
        (executionSettings.useParallel || executionSettings.backgroundPrefetch)
    
    % disable parallel processing for a MiniBatchSize of 1 to work around
    % limitation of background dispatcher. 
    es = executionSettings;
    es.useParallel = false;
    es.backgroundPrefetch = false;
    
    fcnInit = summaryBuilderFcn(trainedNet, es);
    
    % Disable verbose to avoid double printing messages.
    s = saveobj(opts);
    s.Verbose = false;
    nopts = opts.loadobj(s);
    initTrainer =  iCreateTrainer(nopts, reporter, fcnInit, es);
  
    % Do pre-processing work required for input and output layers.
    trainedNet = initTrainer.initializeNetwork(trainedNet, inputStatsDispatcher);
else
    % Do pre-processing work required for input and output layers.
    trainedNet = trainer.initializeNetwork(trainedNet, inputStatsDispatcher);
end

% Train using custom trainer
trainedNet = trainer.train(trainedNet, trainingDispatcher);

% Do post-processing work (if any)
trainedNet = trainer.finalizeNetwork(trainedNet, trainingDispatcher);
trainedNet = iPrepareNetworkForOutput(trainedNet, analyzedLayers, assembler);
info = infoRecorder.Info;

% Update error state ready for the cleanup.
errorState.ErrorOccurred = false;

end

%--------------------------------------------------------------------------
function dispatcher = iCreateInitializationDispatcher(ds, mapping, net, opts, executionSettings, batchFcnStruct, trainedNet, networkInfo)
% Create a dispatcher for training initialization.

if opts.MiniBatchSize == 1
    
    % disable parallel processing for a MiniBatchSize of 1 as it only adds
    % overhead.
    executionSettings.useParallel = false;
    executionSettings.backgroundPrefetch = false;

    % Use a datastore transform to resize input images to the network input
    % size if required. This is required to use the built-in input
    % statistics collection provided by the Trainer, which requires all
    % data be the same size.
    %
    % This code path is required when the mini-batchsize is 1. Otherwise,
    % all the input data is assumed to be the same size, which must be true
    % for mini-batches larger that 1.
    
    if istable(mapping) 
        
        % Modify data collation functions to format data correctly for
        % statistic aggregation.
        inputFunctions = iModifyDataCollationForNonImageInputs(mapping,trainedNet,batchFcnStruct);
       
        % Convert from table translator to cell translator mapping format.
        % Tranformed datastores require the use of a cell translator
        % instead of a table translator.       
        [mapping,inputMapping] = iTableToCellTranslator(mapping,trainedNet,networkInfo);
        
    else
        inputMapping = mapping{1};
        inputFunctions = batchFcnStruct.InputFunctions;
    end
    
    % Find data columns that contain images (only image data is resized).
    isImageInput = cellfun(@(x)isa(x,'nnet.internal.cnn.layer.ImageInput'),trainedNet.InputLayers);
    whichColumn = cellfun(@(x)x(1),inputMapping(isImageInput));
    
    % Add transform to resize image data to network input size if required.
    ds = transform(ds,@(data)iResizeToNetworkInputSize(data,whichColumn,networkInfo.InputSizes));
    
    % Setup batching functions for inputs and outputs. Use the default
    % batching for image data and set all others to pass through. All other
    % data is not used for stat collection so it need not be batched.
    batchFcnStruct.InputFunctions = inputFunctions;
    batchFcnStruct.OutputFunctions = repelem({@(x,~)x},1,trainedNet.NumOutputs);
end

% Make any non-image input batching function return [] to use DLT
% accumulators.
if istable(mapping)
    dest = string(mapping.Destination);
    
    for k = 1:trainedNet.NumInputLayers
        layerIdx = trainedNet.InputLayerIndices(k);
        dataColumn = find(string(trainedNet.Layers{layerIdx}.Name) == dest);
        if isfield(batchFcnStruct,mapping.Data{dataColumn})
            % Return empty data for non-image input data.
            batchFcnStruct.(mapping.Data{dataColumn})= @(x)[];
        end
    end
else
    
    for k = 1:numel(batchFcnStruct.InputFunctions)
        if isempty(batchFcnStruct.InputFunctions{k})
            continue
        else
            batchFcnStruct.InputFunctions{k} = @(x,~)[];
        end
    end
end

shuffle = opts.Shuffle;

dispatcher = nnet.internal.cnn.dispatcher.DispatcherFactory.createDispatcherMIMO( ...
    ds, mapping, net, opts.MiniBatchSize, 'truncateLast', ...
    executionSettings.precision, executionSettings, shuffle, batchFcnStruct);
end

%--------------------------------------------------------------------------
function dispatcher = iCreateTrainingDataDispatcher(ds, mapping, net, opts, executionSettings, batchFcn)
% Create a dispatcher.
dispatcher = nnet.internal.cnn.dispatcher.DispatcherFactory.createDispatcherMIMO( ...
    ds, mapping, net, opts.MiniBatchSize, 'discardLast', ...
    executionSettings.precision, executionSettings, opts.Shuffle, batchFcn);
end

%--------------------------------------------------------------------------
function validationDispatcher = iCreateValidationDispatcher(mapping, net, opts, executionSettings, batchFcn)
if ~iIsValidationSpecified(opts)
    validationDispatcher = [];
else
    validationDispatcher = iCreateValidationDataDispatcher(opts.ValidationData, mapping, net, opts, executionSettings, batchFcn);
end
end

%--------------------------------------------------------------------------
function dispatcher = iCreateValidationDataDispatcher(ds, mapping, net, opts, executionSettings, batchFcn)
% iCreateValidationDataDispatcher   Create a dispatcher for validation data

% Validation execution settings
executionSettings = iSetValidationExecutionSettings(executionSettings);

dispatcher = nnet.internal.cnn.dispatcher.DispatcherFactory.createDispatcherMIMO( ...
    ds, mapping, net, opts.MiniBatchSize, 'truncateLast', ...
    executionSettings.precision, executionSettings, opts.Shuffle, batchFcn);
end

%--------------------------------------------------------------------------
function executionSettings = iSetValidationExecutionSettings(trainingExecutionSettings)
% Copy training settings for use with validation
executionSettings = trainingExecutionSettings;
% If the training execution environment is parallel, prefetching cannot be
% used by the validation dispatcher
if trainingExecutionSettings.useParallel
    executionSettings.backgroundPrefetch = false;
end
% Validation dispatcher cannot be parallel
executionSettings.useParallel = false;
end

%--------------------------------------------------------------------------
function trainer = iCreateTrainer(opts, reporters, summaryFcn, executionSettings)
if executionSettings.useParallel
    trainer = nnet.internal.cnn.ParallelTrainer( ...
        opts, executionSettings.precision, reporters, executionSettings, summaryFcn);
else
    trainer = nnet.internal.cnn.Trainer( ...
        opts, executionSettings.precision, reporters, executionSettings, summaryFcn);
end
end

%% Post Training
function externalNet = iPrepareNetworkForOutput(internalNet, ...
    analyzedLayers, assembler)
% If output network is on pool, retrieve it
if isa(internalNet, 'Composite')
    spmd
        [internalNet, workerWithOutput] = iPrepareNetworkForOutputOnPool(internalNet);
    end
    internalNet = internalNet{workerWithOutput.Value};
else
    internalNet = iPrepareNetworkForHostPrediction(internalNet);
end

% Convert to external network for user
externalNet = assembler.createExternalNetwork(internalNet, analyzedLayers);
end

function [internalNet, workerWithResult] = iPrepareNetworkForOutputOnPool(internalNet)
if isempty(internalNet)
    workerWithResult = spmdReduce(@min, inf);
else
    workerWithResult = spmdReduce(@min, spmdIndex);
end
if spmdIndex == workerWithResult
    % Convert to host network on pool, in case client has no GPU
    internalNet = iPrepareNetworkForHostPrediction(internalNet);
end
% Only workerWithResult can be returned using AutoTransfer - network is too
% big
workerWithResult = distributedutil.AutoTransfer( workerWithResult, workerWithResult );
end

function internalNet = iPrepareNetworkForHostPrediction(internalNet)
% Make sure any Acceleration settings do not apply to subsequent prediction
% by setting the optimization scheme back to noop
internalNet = internalNet.prepareNetworkForPrediction();
internalNet = internalNet.optimizeNetworkForPrediction( ...
    nnet.internal.cnn.optimizer.NoOpNetworkOptimizer() );
internalNet = internalNet.setupNetworkForHostPrediction();
end

%% Pretraining: Reporter Setup
%--------------------------------------------------------------------------
function [reporter, trainingPlotReporter] = iOptionalReporters(opts, analyzedLayers, assembler, checkpointSaver,...
         axesConfig,columnStrategy, executionSettings, networkInfo, trainingDispatcher, ...
         validationDispatcher, validationPredictStrategyFcn, infoToSummaryMap)

reporter = nnet.internal.cnn.util.VectorReporter();

isValidationSpecified = iIsValidationSpecified(opts);

if opts.Verbose

    progressDisplayerFrequency = opts.VerboseFrequency;
    if isValidationSpecified
        progressDisplayerFrequency = [progressDisplayerFrequency opts.ValidationFrequency];
    end

    pd = nnet.internal.cnn.util.ProgressDisplayer(columnStrategy);
    pd.Frequency = progressDisplayerFrequency;
    reporter.add( pd );
end

if isValidationSpecified
    % Create a validation reporter
    validationPredictStrategy = validationPredictStrategyFcn(executionSettings.precision,...
        executionSettings.executionEnvironment, opts.Shuffle);
    validationReporter = iValidationReporter( validationDispatcher, executionSettings, ...
        opts.ValidationFrequency, opts.ValidationPatience, opts.Shuffle, validationPredictStrategy );
    reporter.add( validationReporter );
end

% Add checkpoint saver
if ~isempty(opts.CheckpointPath) 
    checkpointSaver.Saver.ConvertorFcn = @(net)iPrepareAndCreateExternalNetwork(net, analyzedLayers, assembler);
    reporter.add( checkpointSaver );
end

% Add OutputFcn reporter
if ~isempty(opts.OutputFcn)
    outputFcnReporter = vision.internal.cnn.OutputFcnCallbackReporter( opts.OutputFcn, infoToSummaryMap );
    reporter.add( outputFcnReporter );
end

% Training plot
config = nnet.internal.cnn.ui.TrainingPlotConfig(opts, networkInfo, trainingDispatcher, executionSettings, axesConfig);
if strcmp(opts.Plots, 'training-progress')
    iThrowTrainingPlotErrorInDeployedApplication();
    parentComponent = [];
    plotFactory = nnet.internal.cnn.ui.UIFigureTrainingPlotFactory(parentComponent);
    trainingPlotter = nnet.internal.cnn.ui.CLITrainingPlotter(plotFactory);
    trainingPlotter.configure(config);
    
    trainingPlotReporter = nnet.internal.cnn.util.TrainingPlotReporter(trainingPlotter);
    reporter.add( trainingPlotReporter );
    
else
    trainingPlotReporter = nnet.internal.cnn.util.EmptyPlotReporter();
end


end

%--------------------------------------------------------------------------
function externalNetwork = iPrepareAndCreateExternalNetwork(...
    internalNetwork, analyzedLayers, assembler)
% Prepare an internal network for prediction, then create an external
% network
internalNetwork = internalNetwork.prepareNetworkForPrediction();
internalNetwork = internalNetwork.setupNetworkForHostPrediction();
externalNetwork = assembler.createExternalNetwork(internalNetwork, ...
    analyzedLayers);
end

%--------------------------------------------------------------------------
function data = iResizeToNetworkInputSize(data,whichColumn,netInputSizes)
if istable(data)
    data = table2cell(data);
end

sizeData = size(data);

numObservations = sizeData(1);

for k = 1:numObservations
   
    for i = whichColumn
        I = data{k,i};
        I = imresize(I,netInputSizes{i}(1:2));
        data{k,i} = I;
    end
    
end
end

%--------------------------------------------------------------------------
function inputFunctions = iModifyDataCollationForNonImageInputs(...
    mapping,trainedNet,batchFcnStruct)

dest = string(mapping.Destination);
inputFunctions = {};
for k = 1:trainedNet.NumInputLayers
    layerIdx = trainedNet.InputLayerIndices(k);
    dataColumn = find(string(trainedNet.Layers{layerIdx}.Name) == dest);
    
    % Set up input functions for batching data. Any input that
    % requires batching is a non-image input layer. We do not
    % collect stats from these layers and make them return [] to
    % allow use of DLT accumulators.
    if isfield(batchFcnStruct,mapping.Data{dataColumn}) %#ok<FNDSB>
        % Return empty data for non-image input data.
        inputFunctions{end+1} = @(x,~)[]; %#ok<AGROW>
    else
        inputFunctions{end+1} = []; %#ok<AGROW>
    end
end
end

%--------------------------------------------------------------------------
function [mapping, inputMapping] = iTableToCellTranslator(mapping,trainedNet,networkInfo)
% Get destination layer names.
dest = string(mapping.Destination);

% Map input data columns to input layers, by index.
inputMapping = {};
for k = 1:trainedNet.NumInputLayers
    layerIdx = trainedNet.InputLayerIndices(k);
    dataColumn = find(string(trainedNet.Layers{layerIdx}.Name) == dest);
    
    inputMapping{end+1} = dataColumn; %#ok<AGROW>
end
assert(~isempty(inputMapping));

% Map output data columns to output layers, by index.
outputMapping = {};
for k = 1:trainedNet.NumOutputLayers
    layerIdx = trainedNet.OutputLayerIndices(k);
    dataColumn = find(string(trainedNet.Layers{layerIdx}.Name) == dest);
    outputMapping{end+1} = dataColumn; %#ok<AGROW>
end
assert(~isempty(outputMapping));

% none of the outputs are classification layers.
areClassificationOutputs = cellfun(@(x)isa(x,'nnet.internal.cnn.layer.ClassificationLayer'),trainedNet.OutputLayers);

mapping = {inputMapping, outputMapping, areClassificationOutputs,...
    networkInfo.InputSizes, networkInfo.OutputSizes, networkInfo.InputFormats, networkInfo.OutputFormats};
end

%--------------------------------------------------------------------------
function tf = iIsValidationSpecified(opts)
tf = ~isempty(opts.ValidationData);
end

%--------------------------------------------------------------------------
function validator = iValidationReporter(validationDispatcher, executionEnvironment, frequency, patience, shuffle, validationPredictStrategy)
validator = nnet.internal.cnn.util.ValidationReporter(validationDispatcher, executionEnvironment, frequency, patience, shuffle, validationPredictStrategy);
end
%--------------------------------------------------------------------------
function iThrowTrainingPlotErrorInDeployedApplication()
    if isdeployed
        try
            trainingPlotter;
        catch e
            if strcmpi(e.identifier, 'MATLAB:UndefinedFunction')
                disp('trainingPlotter cannot run in deployed mode.');
            else
                disp('Unexpected error.');
                disp(e.identifier);
                disp(e.getReport);
            end
        end
    end
end
%--------------------------------------------------------------------------
function iFinalizePlot(trainingPlotReporter, errorState)
trainingPlotReporter.finalizePlot(errorState.ErrorOccurred);
end

%--------------------------------------------------------------------------
function addMonitorReporter(reporter,monitor,monitorFieldInformation)
    if isempty(monitor)
        return;
    end
    monitorReporter = vision.internal.cnn.ExperimentMonitorReporter(monitor, monitorFieldInformation);
    reporter.add(monitorReporter);
end
