function [detector, info] = trainFasterRCNNObjectDetector(trainingData, network, options, varargin)

% Copyright 2016-2023 The MathWorks, Inc. 

if nargin > 3
    [varargin{:}] = convertStringsToChars(varargin{:});
end

vision.internal.requiresNeuralToolbox(mfilename);

% Initialize warning logger. Logs warnings issued during training and
% reissues them at end of training when Verbose is true.
vision.internal.cnn.WarningLogger.initialize();

[trainingData, options, params] = iParseInputs(...
    trainingData, network, options, varargin{:});

% Configure printer for verbose printing
printer = vision.internal.MessagePrinter.configure(options(1).Verbose);

fasterRCNNObjectDetector.printHeader(printer, params.ClassNames);

% Setup execution environment. This includes opening a pool for
% multi-gpu or parallel training. NB: This pool is reused for
% computing region proposals in parallel.
executionSettings = fastRCNNObjectDetector.setupExecutionEnvironment(options(1),params.UseParallel);

if iIsFourStepTraining(params)
    [fastRCNN, rpn, params] = iParseForFourStep(trainingData, network, params);
    [imageInfo,trainingData,options] = iCollectImageInfo(trainingData, rpn, iStageOneParams(params), params, options);
    [detector, info] = iTrainFourStep(trainingData, fastRCNN, rpn, options, params, executionSettings, imageInfo);
else
    [fastRCNN, params] = iParseEndToEnd(trainingData, network, params);
    [imageInfo,trainingData,options] = iCollectImageInfo(trainingData, fastRCNN, iRPNParamsEndToEnd(params), params, options);
    [detector, info] = iTrainEndToEnd(trainingData, fastRCNN, options, params, executionSettings, imageInfo);
end

rcnnObjectDetector.printFooter(printer);
rcnnObjectDetector.printRowOfStars(printer);
printer.linebreak;

%--------------------------------------------------------------------------
function [detector, info] = iTrainFourStep(trainingData, fastRCNN, rpn, options, params, executionSettings, imageInfo)

% Configure detector checkpoint saver
saver = nnet.internal.cnn.trainNetwork.reporter.checkpoint.Saver(options(1).CheckpointPath, options(1).CheckpointFrequency);
if strcmp(options(1).CheckpointFrequencyUnit, "epoch")
    checkpointSaver = vision.internal.cnn.DetectorCheckpointEpochSaver(saver);
else
    checkpointSaver = vision.internal.cnn.DetectorCheckpointIterationSaver(saver);
end
checkpointSaver.DetectorFcn = @(net,dd)fasterRCNNObjectDetector.detectorCheckpoint(net, dd, [], params.FreezeBatchNormalization);

% Configure printer for verbose printing
printer = vision.internal.MessagePrinter.configure(options(1).Verbose);

if any(params.TrainingStage == [1 2 3 4])
    printer.printMessage('vision:rcnn:resumeTrainingAtStage',find(params.DoTrainingStage,1,'first'));
    printer.linebreak;
end

% Add image information to the params struct used to set box mean and std.
params.ImageInfo = imageInfo;

% Update DispatchInBackground setting based on execution environment.
params.DispatchInBackground(1) = executionSettings.backgroundPrefetch;

if params.DoTrainingStage(1)
    
    printer.printMessage('vision:rcnn:fasterStep1');
    stageOneParams               = iStageOneParams(params);
    stageOneParams.MiniBatchSize = options(1).MiniBatchSize;
    if params.TrainingDataWasTable
        % The parameters struct contains the datastore created using the training data needed for training RPN.
        [ds,stageOneParams] = iCreateRPNTrainingDatastoreFromTable(trainingData, rpn, stageOneParams);
    else
        % The parameters struct contains the transformed datastore created using the training datastore input
        % with tranforms needed for training RPN. The RPN network is also modified (only in stage one) with
        % a new output layer to compute targets in the layer instead of the datastore.
        [rpn,stageOneParams.RCNNInfo] = vision.internal.cnn.RCNNLayers.modifyRPNForProposalCalculations(...
                                      rpn, stageOneParams.RCNNInfo, stageOneParams);

        [ds,stageOneParams,options] = iCreateRPNTrainingDatastore(trainingData, stageOneParams,options);
        params.RCNNInfo     = stageOneParams.RCNNInfo;
    end
    
    % Step 1: Train RPN network
    params.TrainingStage = 1;
    checkpointSaver.CheckpointPrefix = 'faster_rcnn_stage_1';
    checkpointSaver.Saver.CheckpointPath = options(1).CheckpointPath;
    checkpointSaver.Detector = iAssembleDetectorFromCheckpoint(fastRCNN, rpn, params);   
    checkpointSaver.DetectorFcn = @(net,dd)fasterRCNNObjectDetector.detectorCheckpoint(net, dd, 1, params.FreezeBatchNormalization);
    
    [stage1Detector, rpn, info] = fasterRCNNObjectDetector.trainRPN(...
        ds, rpn, options(1), executionSettings, stageOneParams, checkpointSaver);
    
    iWarnIfFourStepTrainingLossHasNaNs(info, 1);
    
    % copy box mean/std into params struct for use during checkpointing.
    params.RPNBoxMean = stage1Detector.RPNBoxMean;
    params.RPNBoxStd  = stage1Detector.RPNBoxStd;    
    
    % copy input stats from rpn to fast r-cnn network.
    fastRCNN = iCopyInputStatisticsToFastRCNN(fastRCNN,rpn);
    
    printer.linebreak;
else
    stage1Detector = iAssembleDetectorFromCheckpoint(fastRCNN, rpn, params);
end

minBoxSize = params.ModelSize;
% MinObjectSize is used within Fast R-CNN trainer. Set it to the model size.
params.MinObjectSize = params.ModelSize;

if params.DoTrainingStage(2)
    
    % Setup execution environment. This may be required if the previous
    % training step takes too long and the pool is shutdown. If the pool is
    % still open, it is reused.
    executionSettings = fastRCNNObjectDetector.setupExecutionEnvironment(options(2),params.UseParallel);
    
    params.DispatchInBackground(2) = executionSettings.backgroundPrefetch;
    
    % Step 2: Use RPN as region proposal function for training Fast
    % R-CNN. Use the same layers used to train RPN.
    printer.printMessage('vision:rcnn:fasterStep2');
    
    % Add mini-batch size and exe env to rpn propose method.
    rpnStage1Params = params.InternalOptions{1}.ProposalParams;
    rpnStage1Params.MiniBatchSize        = options(2).MiniBatchSize;
    rpnStage1Params.ExecutionEnvironment = iInferenceExecutionEnvironment(options(2).ExecutionEnvironment);
    
    % scale factor from feature space to image space. Inverse of image to
    % feature space scaling used by ROI pooling layer.
    rpnStage1Params.ScaleFactor          = 1./params.ScaleFactor;    

    params.RegionProposalFcn = @(x)stage1Detector.propose(x,minBoxSize,rpnStage1Params);
    params.UsingDefaultRegionProposalFcn = false;
    
    % disable parallel b/c region proposals will use RPN on GPU.
    prev = params.UseParallel;
    params.UseParallel = false;
    
    % Update checkpoint for stage 2
    params.TrainingStage = 2;
    checkpointSaver.CheckpointPrefix = 'faster_rcnn_stage_2';
    checkpointSaver.Saver.CheckpointPath = options(2).CheckpointPath;
    checkpointSaver.Detector = iAssembleDetectorFromCheckpoint(fastRCNN, rpn, params);
    checkpointSaver.DetectorFcn = @(net,dd)fasterRCNNObjectDetector.detectorCheckpoint(net, dd, 2, params.FreezeBatchNormalization);

    % The train method of fastRCNNObjectDetector takes care of creating either an imageCentricRegionDatastore or a transformed
    % datastore based on whether the training data is a table or a datastore.
    [stage2Detector, fastRCNN, info(2)] = fastRCNNObjectDetector.train(trainingData, fastRCNN, options(2), executionSettings, iStageTwoParams(params), checkpointSaver);
    
    iWarnIfFourStepTrainingLossHasNaNs(info(2), 2);
    
    % copy box mean/std for checkpointing
    params.BoxMean = stage2Detector.BoxMean;
    params.BoxStd  = stage2Detector.BoxStd;    
    
    params.UseParallel = prev;
    
    printer.linebreak;
end

% Freeze conv layers from Fast R-CNN network;
prevConvLayers = iExtractConvLayers(fastRCNN, params.RCNNInfo.FeatureExtractionLayers);
frozenConvLayers = iFreezeConvLayers(prevConvLayers);

if params.DoTrainingStage(3)
    % Step 3: Fine-tune RPN using frozen conv layers form Fast R-CNN.
    
    printer.printMessage('vision:rcnn:fasterStep3');
    
    % Setup execution environment. This may be required if the previous
    % training step takes too long and the pool is shutdown. If the pool is
    % still open, it is reused.
    executionSettings = fastRCNNObjectDetector.setupExecutionEnvironment(options(3),params.UseParallel);
    
    params.DispatchInBackground(3) = executionSettings.backgroundPrefetch;         
    
    for i = 1:numel(frozenConvLayers)
        % replace rpn conv layers with frozen fast r-cnn conv layers.
        rpn = rpn.replaceLayer(frozenConvLayers(i).Name, frozenConvLayers(i));
    end
   
    % Update checkpoint for stage 3
    params.TrainingStage = 3;
    checkpointSaver.CheckpointPrefix = 'faster_rcnn_stage_3';
    checkpointSaver.Saver.CheckpointPath = options(3).CheckpointPath;
    checkpointSaver.Detector = iAssembleDetectorFromCheckpoint(fastRCNN, rpn, params);    
    checkpointSaver.DetectorFcn = @(net,dd)fasterRCNNObjectDetector.detectorCheckpoint(net, dd, 3, params.FreezeBatchNormalization);
     
    stageThreeParams               = iStageThreeParams(params);
    stageThreeParams.MiniBatchSize = options(3).MiniBatchSize;

    if params.TrainingDataWasTable
        [ds,stageThreeParams] = iCreateRPNTrainingDatastoreFromTable(trainingData, rpn, stageThreeParams);
    else
        % For stage three, we do not modify the RPN network, since it's been already modified at stage one.
        [ds,stageThreeParams] = iCreateRPNTrainingDatastore(trainingData, stageThreeParams);
        params.RCNNInfo       = stageThreeParams.RCNNInfo;
    end

    [stage3Detector, rpn, info(3)] = fasterRCNNObjectDetector.trainRPN(...
                                         ds, rpn, options(3), executionSettings, stageThreeParams, checkpointSaver);
    
    iWarnIfFourStepTrainingLossHasNaNs(info(3), 3);
    
    % copy box mean/std into params struct for use during checkpointing.
    params.RPNBoxMean = stage3Detector.RPNBoxMean;
    params.RPNBoxStd  = stage3Detector.RPNBoxStd;  
    
    printer.linebreak;
else
    stage3Detector = iAssembleDetectorFromCheckpoint(fastRCNN, rpn, params);
end

if params.DoTrainingStage(4)
    % Step 4: Train Fast-RCNN using frozen layers of RPN
    printer.printMessage('vision:rcnn:fasterStep4');
    
    % Setup execution environment. This may be required if the previous
    % training step takes too long and the pool is shutdown. If the pool is
    % still open, it is reused.
    executionSettings = fastRCNNObjectDetector.setupExecutionEnvironment(options(4),params.UseParallel);
    
    params.DispatchInBackground(4) = executionSettings.backgroundPrefetch;
    
    % Add mini-batch size and exe env to rpn propose method.
    rpnStage2Params = params.InternalOptions{3}.ProposalParams;
    rpnStage2Params.MiniBatchSize = options(4).MiniBatchSize;
    rpnStage2Params.ExecutionEnvironment = iInferenceExecutionEnvironment(options(4).ExecutionEnvironment);
    
    % scale factor from feature space to image space. Inverse of image to
    % feature space scaling used by ROI pooling layer.
    rpnStage2Params.ScaleFactor          = 1./params.ScaleFactor;  

    params.RegionProposalFcn = @(x)stage3Detector.propose(x,minBoxSize,rpnStage2Params);
    params.UsingDefaultRegionProposalFcn = false;
    
    % disable parallel b/c region proposals will use RPN on GPU.
    prev = params.UseParallel;
    params.UseParallel = false;
    
    % Freeze fastRCNN layers. These are the same as the RPN layers.
    for i = 1:numel(frozenConvLayers)
        % replace rpn conv layers with frozen fast r-cnn conv layers.
        fastRCNN = fastRCNN.replaceLayer(frozenConvLayers(i).Name, frozenConvLayers(i));
    end
        
    % Update checkpoint for stage 4
    params.TrainingStage = 4;
    checkpointSaver.CheckpointPrefix = 'faster_rcnn_stage_4';
    checkpointSaver.Saver.CheckpointPath = options(4).CheckpointPath;
    checkpointSaver.Detector = iAssembleDetectorFromCheckpoint(fastRCNN, rpn, params);
    checkpointSaver.DetectorFcn = @(net,dd)fasterRCNNObjectDetector.detectorCheckpoint(net, dd, 4, params.FreezeBatchNormalization);
    
    % Fine-tune Fast R-CNN
    [stage4Detector, frcnn, info(4)] = fastRCNNObjectDetector.train(trainingData, fastRCNN, options(4), executionSettings, iStageFourParams(params), checkpointSaver);
    
    iWarnIfFourStepTrainingLossHasNaNs(info(4), 4);
   
    % copy box mean/std for checkpointing
    params.BoxMean = stage4Detector.BoxMean;
    params.BoxStd  = stage4Detector.BoxStd;   
    
    % Unfreeze weights back to their original settings.
    frcnn = iUnfreezeConvLayers(frcnn, prevConvLayers);
  
    params.UseParallel = prev;
    
    printer.linebreak;
end

% Mark training complete.
params.TrainingStage = 5;

% If we modified RPN network to accomodate proposal calculations at the output layer
% when datastore is an input, revert it back.
rpn = vision.internal.cnn.RCNNLayers.revertRPNModificationForEndToEndTraining(rpn, params.RCNNInfo);

detector = fasterRCNNObjectDetector.assembleDetectorForPredictionFourStep(frcnn, rpn, params);

%--------------------------------------------------------------------------
function detector = iAssembleDetectorFromCheckpoint(frcnn, rpn, params)
detector = fasterRCNNObjectDetector.assembleDetectorFromCheckpoint(frcnn, rpn, params);

%--------------------------------------------------------------------------
function convLayers = iFreezeConvLayers(convLayers)
for i = 1:numel(convLayers)    
    convLayers(i).WeightLearnRateFactor = 0;
    convLayers(i).BiasLearnRateFactor = 0;
    convLayers(i).WeightL2Factor = 0;
    convLayers(i).BiasL2Factor = 0;    
end 

%--------------------------------------------------------------------------
function lgraph = iUnfreezeConvLayers(lgraph, prevLayers)

for i = 1:numel(prevLayers)
    names = {lgraph.Layers.Name};
    idx = strcmp(prevLayers(i).Name,names);
    
    % Get layer corresponding to previously frozen layer.
    layer = lgraph.Layers(idx);
    
    % Unfreeze
    layer.WeightLearnRateFactor = prevLayers(i).WeightLearnRateFactor;
    layer.BiasLearnRateFactor = prevLayers(i).BiasLearnRateFactor;
    layer.WeightL2Factor = prevLayers(i).WeightL2Factor;
    layer.BiasL2Factor = prevLayers(i).BiasL2Factor;
   
    % update lgraph
    lgraph = lgraph.replaceLayer(layer.Name, layer);
end        

%--------------------------------------------------------------------------
function layers = iExtractConvLayers(lgraph,namesOfLayersToExtract)
names = {lgraph.Layers.Name};
idx = ismember(names, namesOfLayersToExtract);
layers = lgraph.Layers(idx);
idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.Convolution2DLayer'),layers);
layers = layers(idx);

%--------------------------------------------------------------------------
function params = iStageOneParams(params)
params.InternalOptions = params.InternalOptions{1}.RPNMiniBatchParams;
params.PositiveOverlapRange = params.PositiveOverlapRange(1,:);
params.NegativeOverlapRange = params.NegativeOverlapRange(1,:);
params.NumRegionsToSample   = params.NumRegionsToSample(1);
params.DispatchInBackground = params.DispatchInBackground(1);

%--------------------------------------------------------------------------
function params = iStageTwoParams(params)
params.InternalOptions = params.InternalOptions{2};
params.PositiveOverlapRange = params.PositiveOverlapRange(2,:);
params.NegativeOverlapRange = params.NegativeOverlapRange(2,:);
params.NumRegionsToSample   = params.NumRegionsToSample(2);
params.DispatchInBackground = params.DispatchInBackground(2);

%--------------------------------------------------------------------------
function params = iStageThreeParams(params)
params.InternalOptions = params.InternalOptions{3}.RPNMiniBatchParams;
params.PositiveOverlapRange = params.PositiveOverlapRange(3,:);
params.NegativeOverlapRange = params.NegativeOverlapRange(3,:);
params.NumRegionsToSample   = params.NumRegionsToSample(3);
params.DispatchInBackground = params.DispatchInBackground(3);

%--------------------------------------------------------------------------
function params = iStageFourParams(params)
params.InternalOptions = params.InternalOptions{4};
params.PositiveOverlapRange = params.PositiveOverlapRange(4,:);
params.NegativeOverlapRange = params.NegativeOverlapRange(4,:);
params.NumRegionsToSample   = params.NumRegionsToSample(4);
params.DispatchInBackground = params.DispatchInBackground(4);

%--------------------------------------------------------------------------
function params = iRPNParamsEndToEnd(params)
params.PositiveOverlapRange = params.PositiveOverlapRange(1,:);
params.NegativeOverlapRange = params.NegativeOverlapRange(1,:);

%--------------------------------------------------------------------------
function exeenv = iInferenceExecutionEnvironment(exeenv)
% convert multi-gpu and parallel training modes into 'auto'. Otherwise, use
% user specified value.
if any(strcmp(exeenv,{'multi-gpu','parallel'}))
    exeenv = 'auto';
end

%------------------------------------------------------------------
function [ds, params] = iCreateRPNTrainingDatastoreFromTable(trainingData, rpn, params)
[trainingData, trainingSamples, validTrainingData] = vision.internal.cnn.rpn.generateTrainingSamples(trainingData, rpn, params);

rcnnObjectDetector.issueWarningIfRequired(validTrainingData);

params.RandomSelector                 = vision.internal.rcnn.RandomSelector();
params.BackgroundLabel                = 'Background';
trainingData                          = [trainingData trainingSamples];
ds                                    = vision.internal.cnn.rpn.imageCentricRegionDatastore(trainingData,params);
params.Mapping                        = fasterRCNNObjectDetector.createMIMODatastoreMapping(ds.OutputTableVariableNames, rpn);
params.BatchingFunctions              = ds.BatchingFcn;

% Empty since we don't change any layers for non-datastore input.
params.RCNNInfo.RPNProposalParameters = [];
params.ImageInfo.BoxRegressionMean    = ds.BoxRegressionMean;
params.ImageInfo.BoxRegressionStd     = ds.BoxRegressionStd;

%------------------------------------------------------------------
function [ds, params, options] = iCreateRPNTrainingDatastore(datastore, params, options)
params.BatchingFunctions = iProvideRPNBatchingFunctions();

ds = transform(datastore,@iTransformForRPN);
if nargin == 3
    options = vision.internal.cnn.validationReportUtils.updateValidationDataTransform(options, ...
        @(ds)transform(ds,@iTransformForRPN));
end

params.Mapping = fasterRCNNObjectDetector.createRPNMIMODatastoreCellMapping(params.InputSize);

%------------------------------------------------------------------
function [ds, mapping, batchingFcn, params, summaryMonitorInfo] = iCreateEndToEndTrainingDatastoreFromTable(trainingData, lgraph, params, rpnParams, boxMatcher)
[trainingData, trainingSamples, validTrainingData] = ...
    vision.internal.cnn.rpn.generateTrainingSamples(trainingData, lgraph, rpnParams);

rcnnObjectDetector.issueWarningIfRequired(validTrainingData);
% Create training datastore.
ds                = iTrainingDatastoreEndToEnd(trainingData, trainingSamples, params);
% Update region proposal layer parameters.
params.RPNBoxMean = ds.BoxRegressionMean;
params.RPNBoxStd  = ds.BoxRegressionStd;
mapping           = vision.internal.cnn.FasterRCNNEndToEndDatastore.createMIMODatastoreMapping(...
                        ds.OutputTableVariableNames, lgraph);
batchingFcn       = ds.BatchingFcn;

summaryBuilderFcn = @(x,y)vision.internal.cnn.FasterRCNNEndToEndTrainingSummary.makeSummary(x,y,params.ClassNames,boxMatcher);
summaryMonitorInfo.Monitor = params.Monitor;
summaryMonitorInfo.SummaryBuilderFcn = summaryBuilderFcn;
summaryMonitorInfo.MonitorFieldInformation = vision.internal.cnn.FasterRCNNEndToEndValidationContent.monitorSummaryFields(params.IsValidationDataSpecified);

%------------------------------------------------------------------
function [ds, mapping, batchingFcn, params, summaryMonitorInfo, options] = iCreateEndToEndTrainingDatastore(ds, lgraph, params, boxMatcher, options)

% Update region proposal layer parameters.
batchingFcn  = iProvideEndToEndBatchingFunctions();

params.RPNBoxMean           = params.ImageInfo.BoxRegressionMean;
params.RPNBoxStd            = params.ImageInfo.BoxRegressionStd;
mapping                     = vision.internal.cnn.FasterRCNNEndToEndDatastore.createMIMODatastoreCellMappingForDatastoreInput(...
                                         lgraph,batchingFcn,params.InputSize);

options    = vision.internal.cnn.validationReportUtils.updateValidationDataTransform(options, ...
    @(ds)transform(ds,@iTransformForEndToEnd));

% Create training datastore.
ds         = transform(ds, @iTransformForEndToEnd);
summaryBuilderFcn = @(x,y)vision.internal.cnn.FasterRCNNEndToEndTrainingSummary.makeSummary(...
    x,y,params.ClassNames,boxMatcher,params.RCNNInfo.RPNProposalParameters);

summaryMonitorInfo.Monitor = params.Monitor;
summaryMonitorInfo.SummaryBuilderFcn = summaryBuilderFcn;
summaryMonitorInfo.MonitorFieldInformation = vision.internal.cnn.FasterRCNNEndToEndValidationContent.monitorSummaryFields(params.IsValidationDataSpecified);


%--------------------------------------------------------------------------
function ds = iTrainingDatastoreEndToEnd(trainingData, trainingSamples, params)
params.RandomSelector  = vision.internal.rcnn.RandomSelector();
params.BackgroundLabel = 'Background';

% Use first row of overlap ranges for datastore. This first row applies to
% the RPN stage.
params.PositiveOverlapRange = params.PositiveOverlapRange(1,:);
params.NegativeOverlapRange = params.NegativeOverlapRange(1,:);

ds = vision.internal.cnn.FasterRCNNEndToEndDatastore(trainingData,trainingSamples, params);

%--------------------------------------------------------------------------
function checkpointSaver = iCheckPointEndToEnd(options, params)
% Configure detector checkpoint saver.
saver = nnet.internal.cnn.trainNetwork.reporter.checkpoint.Saver(options(1).CheckpointPath, options(1).CheckpointFrequency);
if strcmp(options(1).CheckpointFrequencyUnit, "epoch")
    checkpointSaver = vision.internal.cnn.DetectorCheckpointEpochSaver(saver);
else
    checkpointSaver = vision.internal.cnn.DetectorCheckpointIterationSaver(saver);
end
checkpointSaver.CheckpointPrefix = 'faster_rcnn';
% Create partial detector. During checkpoint creation the DetectorFcn will
% be called with this partial detector as input.
checkpointSaver.Detector = fasterRCNNObjectDetector.assembleDetectorForPredictionEndToEnd([], params);
checkpointSaver.DetectorFcn = @(net,dd)fasterRCNNObjectDetector.detectorCheckpointEndToEnd(...
    net, dd, params.RCNNInfo, params.FreezeBatchNormalization, iClassNamesIncludingBackground(params));

%--------------------------------------------------------------------------
function [detector, info] = iTrainEndToEnd(trainingData, lgraph, options, params, executionSettings, imageInfo)

% Modify network for end-to-end training
boxMatcher.PositiveOverlapRange = params.PositiveOverlapRange(2,:);
boxMatcher.NegativeOverlapRange = params.NegativeOverlapRange(2,:);
[lgraph, params.RCNNInfo]       = vision.internal.cnn.RCNNLayers.modifyForEndToEndTraining(...
                                                  lgraph, params.ClassNames, boxMatcher, ...
                                                  params.NumRegionsToSample(2), ...
                                                  params.InternalOptions.FastForegroundFraction,...
                                                  ~params.TrainingDataWasTable);

params.MiniBatchSize             = options.MiniBatchSize;
params.IsValidationDataSpecified = vision.internal.cnn.validationReportUtils.isValidationSpecified(options);

if params.TrainingDataWasTable
    [ds, mapping, batchingFcn, params, summaryFcn] = iCreateEndToEndTrainingDatastoreFromTable(...
                                                        trainingData, lgraph, params, iRPNParamsEndToEnd(params), boxMatcher);
else
    params.ImageInfo          = imageInfo;
    [lgraph, params.RCNNInfo] = vision.internal.cnn.RCNNLayers.modifyRPNForProposalCalculations(...
                                                        lgraph, params.RCNNInfo, params);

    [ds, mapping, batchingFcn, params, summaryFcn, options] = iCreateEndToEndTrainingDatastore(trainingData, lgraph, params, boxMatcher, options);
end


% Set the mean and std of the targets used for learning fast r-cnn box
% regression to 0 and 1, respectively. 
params.BoxMean = [0 0 0 0];
params.BoxStd =  [1 1 1 1];

proposalParams = iProposalLayerParametersForEndToEndTraining(params);
lgraph = nnet.internal.cnn.layer.RegionProposalLayer.updateParameters(lgraph, proposalParams);

% Configure detector checkpoint saver
checkpointSaver = iCheckPointEndToEnd(options, params);

if params.FreezeBatchNormalization
    lgraph = vision.internal.cnn.RCNNLayers.freezeBatchNorm(lgraph);
end

if params.IsValidationDataSpecified 
    infoContent = vision.internal.cnn.FasterRCNNEndToEndValidationContent();
    validationPredictStrategyFcn = @vision.internal.cnn.ValidationPredictDatastoreStrategy;
else
    infoContent = vision.internal.cnn.FasterRCNNEndToEndTrainingContent();
    validationPredictStrategyFcn = @(varargin)[];
end
columnStrategy = vision.internal.cnn.FasterRCNNEndToEndColumns(params.IsValidationDataSpecified);

axesConfigFactory = vision.internal.cnn.FasterRCNNAxesConfig();
axesConfig = axesConfigFactory.AxesConfiguration;

try
    [net, info] = vision.internal.cnn.trainNetwork(...
        ds, lgraph, options, executionSettings, mapping, checkpointSaver, ...
        summaryFcn,...
        infoContent, ...
        columnStrategy,...
        axesConfig, ...
        batchingFcn,...
        validationPredictStrategyFcn,...
        iInfoToSummaryMapAndStartValues());
        
catch ME
    if strcmp(ME.identifier,'nnet_cnn:internal:cnn:GeneralDatastoreDispatcher:VariableInputSizes')
        error(message('vision:rcnn:unableToBatchImages'));
    else
        rethrow(ME);
    end
end

if params.FreezeBatchNormalization
    lgraph = layerGraph(net);
    lgraph = vision.internal.cnn.RCNNLayers.unfreezeBatchNorm(lgraph);  
    net = vision.internal.cnn.createDAGNetwork(lgraph); 
end

% Revert the modifications made tor end-to-end training.
net = vision.internal.cnn.RCNNLayers.revertModificationForEndToEndTraining(net, params.RCNNInfo, iClassNamesIncludingBackground(params));

% Create detector
detector = fasterRCNNObjectDetector.assembleDetectorForPredictionEndToEnd(net, params);

%--------------------------------------------------------------------------
function [trainingDs, options, params] = iParseInputs(trainingDs, network, options, varargin)

vision.internal.cnn.validation.checkNetwork(network, mfilename, ...
   {'SeriesNetwork', 'nnet.cnn.layer.Layer', 'nnet.cnn.LayerGraph', 'fasterRCNNObjectDetector'},...
   vision.internal.cnn.RCNNLayers.SupportedPretrainedNetworks);

p = inputParser;
p.addParameter('UseParallel', vision.internal.useParallelPreference());
p.addParameter('PositiveOverlapRange', [0.5 1]);
p.addParameter('NegativeOverlapRange', [0.1 0.5]);
p.addParameter('NumStrongestRegions', 2000);
p.addParameter('BoxPyramidScale', 2);
p.addParameter('NumBoxPyramidLevels', 'auto');
p.addParameter('MinBoxSizes', 'auto');
p.addParameter('SmallestImageDimension', []);
p.addParameter('NumRegionsToSample',128);
p.addParameter('TrainingMethod', 'end-to-end');
p.addParameter('FreezeBatchNormalization', true);
p.addParameter('InternalOptions','');
p.addParameter('ExperimentMonitor', 'none');

p.parse(varargin{:});

userInput = p.Results;

params.Monitor = vision.internal.cnn.validation.checkExperimentMonitor(p,userInput,options(1),mfilename);

vision.internal.inputValidation.validateLogical(userInput.UseParallel,'UseParallel');

trainingMethod = iCheckTrainingMethod(userInput.TrainingMethod);
if strcmp(trainingMethod, 'four-step')
    numStages = 4;
else
    numStages = 2;
    
    % Compatibility checks. Throw error when parameters values supported by four-step
    % training are used with the default end-to-end method.
    if ~iWasUserSpecified(p,'TrainingMethod')
        iAssertValidOptionsForEndToEnd(options);
        iAssertValidOverlapRangeSizeForEndToEnd('PositiveOverlapRange', userInput.PositiveOverlapRange);
        iAssertValidOverlapRangeSizeForEndToEnd('NegativeOverlapRange', userInput.NegativeOverlapRange);
        iAssertValidNumRegionsToSampleForEndToEnd(userInput.NumRegionsToSample);
    end
end

vision.internal.cnn.validation.checkOverlapRatio(userInput.PositiveOverlapRange, mfilename, 'PositiveOverlapRange', numStages);
vision.internal.cnn.validation.checkOverlapRatio(userInput.NegativeOverlapRange, mfilename, 'NegativeOverlapRange', numStages);

vision.internal.inputValidation.validateLogical(userInput.FreezeBatchNormalization,'FreezeBatchNormalization');

vision.internal.cnn.validation.checkStrongestRegions(userInput.NumStrongestRegions, mfilename);

userInput.MinBoxSizes = iCheckBoxSizes(userInput.MinBoxSizes, mfilename);

iCheckPyramidScale(userInput.BoxPyramidScale, mfilename);

userInput.NumBoxPyramidLevels = iCheckNumPyramidLevels(userInput.NumBoxPyramidLevels, mfilename);

% Validate NumRegionsToSample. Expand scalar input as needed.
iCheckNumRegionsToSample(userInput.NumRegionsToSample,numStages,mfilename);
userInput.NumRegionsToSample = iExpandScalarNumRegions(userInput.NumRegionsToSample,numStages);

% Check that user does not specify any anchor box parameters when input is
% a layerGraph. In this case the anchor boxes are specified by the region
% proposal layer in the layerGraph.
anchorBoxParamsSpecified = iWasUserSpecified(p,'MinBoxSizes') || ...
    iWasUserSpecified(p,'BoxPyramidScale') || ...
    iWasUserSpecified(p,'NumBoxPyramidLevels');
if isa(network,'nnet.cnn.LayerGraph') && anchorBoxParamsSpecified
    error(message('vision:rcnn:notSupportedForLayerGraph','MinBoxSizes'));
end

% Fill params structure with user input.
params.PositiveOverlapRange     = iReplicateIfRowVectorForFourStepTraining(double(userInput.PositiveOverlapRange), trainingMethod);
params.NegativeOverlapRange     = iReplicateIfRowVectorForFourStepTraining(double(userInput.NegativeOverlapRange), trainingMethod);
params.BoxPyramidScale          = userInput.BoxPyramidScale;
params.NumBoxPyramidLevels      = userInput.NumBoxPyramidLevels;
params.MinBoxSizes              = userInput.MinBoxSizes;
params.NumStrongestRegions      = double(userInput.NumStrongestRegions);
params.UseParallel              = logical(userInput.UseParallel);
params.ImageScale               = double(userInput.SmallestImageDimension);
params.ScaleImage               = ~isempty(params.ImageScale);
params.NumRegionsToSample       = double(userInput.NumRegionsToSample);
params.TrainingMethod           = char(trainingMethod);
params.FreezeBatchNormalization = logical(userInput.FreezeBatchNormalization);
params.TrainingDataWasTable     = istable(trainingDs);

if ~params.TrainingDataWasTable && anchorBoxParamsSpecified
    error(message('vision:rcnn:anchorBoxParamsNotSupportedForDatastore'));
end


if ~params.TrainingDataWasTable && options.DispatchInBackground
    error(message('vision:ObjectDetector:backgroundUnsupportedWithDatastore'));
end

if params.TrainingDataWasTable
    vision.internal.cnn.validation.checkGroundTruth(trainingDs, mfilename);
    params.ClassNames = trainingDs.Properties.VariableNames(2:end);
else
    if params.ScaleImage
        error(message('vision:rcnn:scalingNotAllowedForDatastore'));
    end
    % Copy and reset the given datastore, so external state events are
    % not reflected.
    trainingDs = copy(trainingDs);
    reset(trainingDs);

    params.ClassNames = vision.internal.inputValidation.checkGroundTruthDatastore(trainingDs);
end

params.BackgroundLabel = vision.internal.cnn.uniqueBackgroundLabel(params.ClassNames);

% Fill InternalOptions.
if iWasUserSpecified(p, 'InternalOptions')
    params.InternalOptions = userInput.InternalOptions;
else
    params.InternalOptions = iInternalDefaultOptions(params.TrainingMethod); 
end

% Validate training options
if iIsFourStepTraining(params)
    
    if ~isscalar(options)
        validateattributes(options, {'nnet.cnn.TrainingOptions'}, {'numel',4}, mfilename);
    end
    if isscalar(options)
        options = repelem(options, 4, 1);
    end
    
    % Assert that the user has specified consistent reset normalization
    % values.
    iAssertAllResetNormalizationAreTheSame(options);
    
    % If 'TrainingMethod' is 'four-step', and 'Plots' is 'training-progress', 
    % or OutputFcn is specified, display error message.
    iAssertTrainingProgressIsNone(options);
    iAssertOutputFcnIsEmpty(options);
    iAssertExperimentMonitorIsEmpty(p,params);

    % Disable input normalization computation for steps 2 to 4. The stats
    % from step one can be reused.
    for i = 2:4
        s = saveobj(options(i));
        s.ResetInputNormalization = false;
        options(i) = options(i).loadobj(s);
    end
    
    
else
    validateattributes(options, {'nnet.cnn.TrainingOptions'}, {'numel',1}, mfilename);
end
allowValidationDatastore = ~iIsFourStepTraining(params);
for i = 1:numel(options)    
    vision.internal.cnn.validation.checkTrainingOptions(options(i), mfilename, allowValidationDatastore, params.TrainingDataWasTable);
end

% Validate all options have the same execution environment.
iCheckExecutionEnvironment(options);

iAssertAllMiniBatchSizesAreTheSame(options);

params.DispatchInBackground = [options(:).DispatchInBackground];

% Choose default value for FreezeBatchNormalization if not provided by
% user.
if ~iWasUserSpecified(p, 'FreezeBatchNormalization')
    params.FreezeBatchNormalization = vision.internal.cnn.RCNNLayers.freezeBatchNormHeuristic(options(1).MiniBatchSize);
end

vision.internal.cnn.validation.checkPositiveAndNegativeOverlapRatioDoNotOverlap(params);

% UseParallel must be true for multi-gpu or parallel ExecutionEnvironment.
isParallelTraining = any(strcmp(options(1).ExecutionEnvironment,{'multi-gpu','parallel'}));

if isParallelTraining && ~params.UseParallel
    error(message('vision:rcnn:UseParallelMustBeTrueForParallelTraining'))
end

% Validate that batch norm layers are trained if a user asks for them to be
% frozen. This needs to be checked for layer arrays and layer graphs. Other
% input types always have trained BN layers. 
if params.FreezeBatchNormalization &&  ...
        (isa(network,'nnet.cnn.LayerGraph') || isa(network,'nnet.cnn.layer.Layer'))
    
    vision.internal.cnn.validation.checkBatchNormLayersAreTrained(network);
end

if iIsDetector(network)
    if iIsFourStepTraining(params)
        % In four step training detector may be partially trained, in which
        % case we must get the networks for training in order to fill input
        % size and zero center params.
        lgraph = network.getNetworksForAlternateTraining();
        
    else
        iAssertNotFourStageCheckpoint(network);
        lgraph = network.Network;
    end
    
else
    lgraph = network;
end
params.InputSize = ...
    vision.internal.cnn.RCNNLayers.imageLayerInfo(lgraph);

if params.TrainingDataWasTable
    % Check whether image is multi-channel.  
    isInput3D = numel(params.InputSize)==3;
    if( isInput3D && ~(params.InputSize(3)==1||params.InputSize(3)==3) )
        error(message('vision:rcnn:notMonoRGB'));
    end

    [trainingDs, params.ImageInfo] = iCollectImageInfoAndScaleForTable(trainingDs, params);
end

params.NumClasses               = numel(params.ClassNames);
params.ModelName                = params.ClassNames{1};
params.MiniBatchSize            = options(1).MiniBatchSize;
params.RandomSelector           = vision.internal.rcnn.RandomSelector();

% Normalization is done in the Trainer. By pass local normalization code.
params.NeedsZeroCenterNormalization = false; 

%--------------------------------------------------------------------------
function x = iCheckTrainingMethod(x)
x = validatestring(x,{'four-step','end-to-end'},'TrainingMethod',mfilename);

%--------------------------------------------------------------------------
function tf = iIsFourStepTraining(params)
tf = strcmp(params.TrainingMethod,'four-step');

%--------------------------------------------------------------------------
function params = iSetParametersFromDetector(detector, params)
% Set parameters cached in the detector.
params.ModelSize     = detector.MinObjectSize;
params.TrainingStage = detector.TrainingStage;
params.AnchorBoxes   = detector.AnchorBoxes;

if ~isempty(detector.ClassNames)
    % checkpointed detector has not gotten to stage 2. skip class name check.
    vision.internal.cnn.validation.checkClassNamesMatchGroundTruth(...
        detector.ClassNames, params.ClassNames, params.BackgroundLabel);
end

% Load values for box regression standardization
params.BoxMean    = detector.BoxMean;
params.BoxStd     = detector.BoxStd;
params.RPNBoxMean = detector.RPNBoxMean;
params.RPNBoxStd  = detector.RPNBoxStd;

%--------------------------------------------------------------------------
function params = iEstimateAnchorBoxesFromTrainingData(trainingData, params)

% MinBoxSizes
if iAutoSelectParameter(params.MinBoxSizes)
    
    allBoxesPerClass = iGetBoxSizesPerClass(trainingData, params.ModelSize);
    
    params.MinBoxSizes = iRemoveSimilarBoxesBasedOnIoU(allBoxesPerClass);
    
else
    
    params.MinBoxSizes = double(params.MinBoxSizes);
    
    % check box sizes against model size. box sizes already set above.
    if ~all(all( params.MinBoxSizes >= params.ModelSize ))
        error(message('vision:rcnn:minBoxSizeTooSmall'));
    end
    
end

% Number of levels in anchor box pyramid
if iAutoSelectParameter(params.NumBoxPyramidLevels)
    
    allBoxes = iAllGroundTruthBoxes(trainingData);
    maxSize = max( max( allBoxes(:, [4 3]) ) );
    minSize = min( min( params.MinBoxSizes ) );
    scaleRequired = maxSize / minSize;
    
    if params.BoxPyramidScale > 1 && scaleRequired > 1
        params.NumBoxPyramidLevels = ceil( log(scaleRequired) / log(params.BoxPyramidScale) + 1);
    else
        params.NumBoxPyramidLevels = 1;
        
    end
else
    params.NumBoxPyramidLevels = double(params.NumBoxPyramidLevels);
end

% Initialize anchor boxes from box pyramid parameters.
params.AnchorBoxes = fasterRCNNObjectDetector.initializeAnchorBoxes(...
    params.MinBoxSizes, params.NumBoxPyramidLevels, params.BoxPyramidScale);

%--------------------------------------------------------------------------
function [fastRCNNLayerGraph, rpnLayerGraph, params] = iParseForFourStep(trainingData, network, params)
if iIsDetector(network)
         
    params = iSetParametersFromDetector(network, params);
    
    % Get fast r-cnn and rpn networks stored in detector.
    [fastRCNNLayerGraph, rpnLayerGraph,  params.RCNNInfo] = network.getNetworksForAlternateTraining();
    
    if network.TrainingStage > 5 || network.TrainingStage < 0
        % reset training stage to 0. The network does not need to
        % resume training from a previous stage and should start
        % from the beginning.
        params.TrainingStage = 0;
    end
    
    % Partially trained detectors may not have the image stats set for the
    % Fast R-CNN network. Set the same input layer in both networks. The
    % RPN network is trained first. Use its input layer to initialize the
    % Fast R-CNN network.
    fastRCNNLayerGraph = iCopyInputStatisticsToFastRCNN(fastRCNNLayerGraph,rpnLayerGraph);
    
else % LayerGraph, SeriesNetwork, Layer array, network by name.
    
    % Start training from beginning.
    params.TrainingStage = 0;
    
    % Initialize Fast and RPN box mean/std
    params.BoxMean = [];
    params.BoxStd  = [];
    params.RPNBoxMean = [];
    params.RPNBoxStd  = [];       
    
    if isa(network,'nnet.cnn.LayerGraph')
        % Check user created LayerGraph. This must be a valid Fast R-CNN.
        if params.TrainingDataWasTable
            allowMultiChannel = false;
        else
            allowMultiChannel = true;
        end
        constraints =  vision.internal.cnn.RCNNLayers.constraints('faster-rcnn',params.NumClasses,allowMultiChannel);
        analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(network);
        analysis.applyConstraints(constraints);
        try
            analysis.throwIssuesIfAny();
        catch ME
            throwAsCaller(ME);
        end
        fasterRCNN = network;
     
        params.ModelSize   = vision.internal.cnn.RCNNLayers.minimumObjectSize(network);
        params.RCNNInfo    = vision.internal.cnn.RCNNLayers.fasterRCNNInfo(fasterRCNN);        
        params.AnchorBoxes = params.RCNNInfo.AnchorBoxes;
        
    else % SeriesNetwork, Layer array, or network by name.

        if ~params.TrainingDataWasTable
            error(message('vision:rcnn:networkMustBeLayerGraphForDatastore'));
        end

        % Estimate anchor boxes from training data. 
        params = iEstimateAnchorBoxesForLayers(trainingData, network, params);

        % Create Faster R-CNN network.
        fasterRCNN = vision.internal.cnn.RCNNLayers.create(params.NumClasses, network, 'faster-rcnn', params.AnchorBoxes);
    end

    % Update ROI scale factor prior to training. This updates user created
    % networks as well as auto-created networks.
    [fasterRCNN, params.RCNNInfo.ScaleFactor] = vision.internal.cnn.RCNNLayers.updateROIPoolingLayerScaleFactor(fasterRCNN);
    
    % Remove unsupported data augmentation values.   
    fasterRCNN = vision.internal.rcnn.removeAugmentationIfNeeded(fasterRCNN,{'randcrop','randfliplr'});    
    
    % Split faster r-cn network into fast r-cnn and rpn.
    [fastRCNNLayerGraph, rpnLayerGraph, params.RCNNInfo] = vision.internal.cnn.RCNNLayers.splitFasterIntoFastAndRPN(fasterRCNN);
             
end

% Configure the training stages that must be executed. If resuming training
% this will effect from which stage the training resumes.
if params.TrainingStage == 0 || params.TrainingStage > 4
    params.DoTrainingStage = true(1,4);
else
    params.DoTrainingStage = false(1,4);
    params.DoTrainingStage(params.TrainingStage:end) = true;
end

% Determine if average image needs to be computed.
params.NumAnchors  = size(params.AnchorBoxes,1);
params.ScaleFactor = params.RCNNInfo.ScaleFactor;

params.ColorPreprocessing = vision.internal.cnn.utils.colorPreprocessingForImageInputSize(params.InputSize);

vision.internal.cnn.validation.checkImageScale(params.ImageScale, params.InputSize, mfilename);

%--------------------------------------------------------------------------
function tf = iIsDetector(x)
tf = isa(x, 'fasterRCNNObjectDetector');

%--------------------------------------------------------------------------
function [lgraph, params] = iParseEndToEnd(trainingData, network, params)
if iIsDetector(network)    
    
    params = iSetParametersFromDetector(network, params);
    
    % Get network stored in detector.
    lgraph = layerGraph(network.Network);     
    
    % Extract information 
    params.RCNNInfo = vision.internal.cnn.RCNNLayers.fasterRCNNInfo(lgraph);
    params.AnchorBoxes = params.RCNNInfo.AnchorBoxes;
else
       
    % Initialize Fast and RPN box mean/std
    params.BoxMean = [];
    params.BoxStd  = [];
    params.RPNBoxMean = [];
    params.RPNBoxStd  = [];
              
    if isa(network,'nnet.cnn.LayerGraph')
        % Check user created LayerGraph. This must be a valid Fast R-CNN.
        if params.TrainingDataWasTable
            allowMultiChannel = false;
        else
            allowMultiChannel = true;
        end
        constraints =  vision.internal.cnn.RCNNLayers.constraints('faster-rcnn',params.NumClasses,allowMultiChannel);
        analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(network);
        analysis.applyConstraints(constraints);
        try
            analysis.throwIssuesIfAny();
        catch ME
            throwAsCaller(ME);
        end
        lgraph = network;
        
        params.ModelSize = vision.internal.cnn.RCNNLayers.minimumObjectSize(lgraph);
        params.RCNNInfo = vision.internal.cnn.RCNNLayers.fasterRCNNInfo(lgraph);        
        params.AnchorBoxes = params.RCNNInfo.AnchorBoxes;
        
    else % SeriesNetwork, Layer array, or network by name.

        if ~params.TrainingDataWasTable
            error(message('vision:rcnn:networkMustBeLayerGraphForDatastore'));
        end

        % Estimate anchor boxes from training data. 
        params = iEstimateAnchorBoxesForLayers(trainingData, network, params);

        % Create Faster R-CNN network.
        lgraph = vision.internal.cnn.RCNNLayers.create(params.NumClasses, network, 'faster-rcnn', params.AnchorBoxes);
        
    end

    % Update ROI scale factor prior to training. This updates user created
    % networks as well as auto-created networks.
    [lgraph, params.RCNNInfo.ScaleFactor] = vision.internal.cnn.RCNNLayers.updateROIPoolingLayerScaleFactor(lgraph);
    
    % Remove unsupported data augmentation values.   
    lgraph = vision.internal.rcnn.removeAugmentationIfNeeded(lgraph,{'randcrop','randfliplr'});       
    
end

% Set to -1 for end-to-end training.
params.TrainingStage = -1;

params.NumAnchors  = size(params.AnchorBoxes,1);
params.ScaleFactor = params.RCNNInfo.ScaleFactor;

params.ColorPreprocessing = vision.internal.cnn.utils.colorPreprocessingForImageInputSize(params.InputSize);

vision.internal.cnn.validation.checkImageScale(params.ImageScale, params.InputSize, mfilename);

%--------------------------------------------------------------------------
function s = iProposalLayerParametersForEndToEndTraining(params)
s = nnet.internal.cnn.layer.RegionProposalLayer.emptyProposalParamsStruct();
s.ImageSize = []; % compute dynamically during training.  
s.RPNBoxStd = params.RPNBoxStd;
s.RPNBoxMean = params.RPNBoxMean;
s.MinSize = params.ModelSize; % same as min object size.
s.MaxSize = [inf inf]; % same as image size.
s.NumStrongestRegions = params.NumStrongestRegions;
s.NumStrongestRegionsBeforeProposalNMS = params.InternalOptions.NumStrongestRegionsBeforeProposalNMS;
s.ProposalsOutsideImage = params.InternalOptions.ProposalsOutsideImage;
s.MinScore =  params.InternalOptions.MinScore;
s.OverlapThreshold = params.InternalOptions.OverlapThreshold;
s.ScaleFactor = 1./params.ScaleFactor;
s.BoxFilterFcn = @(a,b,c,d)fasterRCNNObjectDetector.filterBBoxesBySize(a,b,c,d);

%--------------------------------------------------------------------------
function sz = iCheckBoxSizes(sz, fname)
if iIsString(sz)
    sz = validatestring(sz, {'auto'}, fname, 'MinBoxSizes');
else
    validateattributes(sz, {'numeric'}, ...
        {'size',[NaN 2], 'real', 'positive', 'nonsparse'}, ...
        fname, 'MinBoxSizes');     
end

%--------------------------------------------------------------------------
function iCheckPyramidScale(scales, fname)
validateattributes(scales, {'numeric'}, ...
    {'scalar', '>=', 1, 'nonempty', 'real', 'nonsparse'}, ...
    fname, 'BoxPyramidScale');

%--------------------------------------------------------------------------
function num = iCheckNumPyramidLevels(num, fname)
if iIsString(num)
    num = validatestring(num, {'auto'}, fname, 'NumBoxPyramidLevels');
else
    validateattributes(num, {'numeric'},...
        {'scalar', '>=', 1, 'real', 'nonsparse', 'nonempty'},...
        fname, 'NumBoxPyramidLevels');
end

%--------------------------------------------------------------------------
function tf = iAutoSelectParameter(val)
tf = iIsString(val) && strcmpi(val, 'auto');

%--------------------------------------------------------------------------
function tf = iIsString(s)
tf = ischar(s) || isstring(s);

%--------------------------------------------------------------------------
function allBoxes = iAllGroundTruthBoxes(trainingData)
gt = trainingData{:,2:end};
if iscell(gt)
    allBoxes = vertcat( trainingData{:,2:end}{:} );
else
    allBoxes = vertcat( trainingData{:,2:end});
end
allBoxes = iRemoveInvalidBoxes(allBoxes);

%--------------------------------------------------------------------------
function boxes = iRemoveInvalidBoxes(boxes)
% remove boxes non-positive coordinates and non-finite values.
remove = any(boxes <= 0, 2) | any(~isfinite(boxes), 2);
boxes(remove,:) = [];

if isempty(boxes)
    error(message('vision:rcnn:noValidTrainingData'))
end

%--------------------------------------------------------------------------
function boxesPerClass = iGetAllBoxesPerClass(trainingData)
boxesPerClass = cell(1,width(trainingData)-1);
for i = 2:width(trainingData)
    gt = trainingData{:,i};
    if iscell(gt)
        boxes = cat(1,gt{:});
    else
        boxes = vertcat( gt );
    end
    
    boxes = iRemoveInvalidBoxes(boxes);
    
    boxesPerClass{i-1} = boxes;
end

%--------------------------------------------------------------------------
function boxSizesPerClass = iGetBoxSizesPerClass(trainingData, minObjectSizeSupportedByNetwork)

boxesPerClass = iGetAllBoxesPerClass(trainingData);

boxSizesPerClass = zeros(width(trainingData)-1,2);
n = width(trainingData)-1;
for i = 1:numel(boxesPerClass)
    allBoxes = boxesPerClass{i};
    
    % min size based
    minLength = min( min(allBoxes(:,[3 4])) );
    
    % aspect ratio
    ar = allBoxes(:, 3) ./ allBoxes(:, 4);
    medAspectRatio = median(ar);
    
    % box size must also be >= min object size supported by network.
    minModelLength = min(minObjectSizeSupportedByNetwork);
    
    if medAspectRatio < 1
        % height > weight
        w = max(minLength, minModelLength);
        h = w / medAspectRatio;
    else
        % width >= height
        h = max(minLength, minModelLength);
        w = h * medAspectRatio;
    end
    
    boxSizesPerClass(i,:) = round([h w]);
end

% Make all the boxes have the same min length. 
ar = boxSizesPerClass(:,2) ./ boxSizesPerClass(:,1);

% scale class specific boxes so they have the same size
minLength = min( min(boxSizesPerClass) );

% height > width
w = zeros(n,1);
h = zeros(n,1);
idx = ar < 1;

w(idx) = max(minLength, minModelLength);
h(idx) = w(idx) ./ ar(idx);

% width > height
idx = ar >= 1;
h(idx) = max(minLength, minModelLength);
w(idx) = h(idx) .* ar(idx);

boxSizesPerClass = round([h w]);
boxSizesPerClass(any(boxSizesPerClass < 1 , 2)) = [];

%--------------------------------------------------------------------------
function out = iRemoveSimilarBoxesBasedOnIoU(boxSizes)
% Only keep box sizes that have IoU <= 0.5. This ensures that the number of
% anchors is as small as possible.

n = size(boxSizes,1);

bboxes = [ones(n,2) fliplr(boxSizes)];

iou = bboxOverlapRatio(bboxes,bboxes);
keep = iou <= 0.5;

keep(eye(n,'like',true)) = true;

% greedily remove boxes
for i = 1:n
    if keep(i,i) 
        % remove box by setting row and column to 0
        keep(~keep(i,:), :) = false;
        keep(:, ~keep(i,:)) = false;
    end
end

out = boxSizes(diag(keep), :);

%--------------------------------------------------------------------------
function range = iReplicateIfRowVectorForFourStepTraining(range,trainingMethod)
replicate = isrow(range);
if strcmp(trainingMethod,'four-step')
    n = 4;
else
    n = 2;
end
if replicate
    
    range = repelem(range,n,1);
end

%--------------------------------------------------------------------------
function s = iInternalDefaultOptions(trainingMethod)
if strcmp(trainingMethod, 'four-step')
    s{1} = vision.internal.cnn.defaultInternalRPNOptions();
    s{2} = vision.internal.cnn.defaultInternalFastOptions();
    s{3} = vision.internal.cnn.defaultInternalRPNOptions();
    s{4} = vision.internal.cnn.defaultInternalFastOptions();
else
    s = vision.internal.cnn.defaultInternalEndToEndOptions();
end

%--------------------------------------------------------------------------
function iCheckExecutionEnvironment(options)

env = options(1).ExecutionEnvironment;

for i = 2:numel(options)
    if ~strcmp(options(i).ExecutionEnvironment, env)
        error(message('vision:rcnn:inconsistentExeEnv'));
    end
end

%--------------------------------------------------------------------------
function iAssertAllMiniBatchSizesAreTheSame(options)
sz = [options(:).MiniBatchSize];
if ~all(sz(1)==sz)
    error(message('vision:rcnn:inconsistentMiniBatchSize'));
end

%--------------------------------------------------------------------------
function iAssertAllResetNormalizationAreTheSame(options)
v = [options(:).ResetInputNormalization];
if numel(unique(v)) ~= 1
    error(message('vision:rcnn:inconsistentResetNorm'));
end

%--------------------------------------------------------------------------
function iAssertTrainingProgressIsNone(options)
if ~strcmpi(options(1).Plots,'none') || ~strcmpi(options(2).Plots,'none') ||...
        ~strcmpi(options(3).Plots,'none') || ~strcmpi(options(4).Plots,'none')
    error(message('vision:rcnn:fourStepValidationPlotsNotSupported'));
end

%--------------------------------------------------------------------------
function iAssertOutputFcnIsEmpty(options)
empty = arrayfun(@(x)isempty(x.OutputFcn),options);
if ~all(empty)
    error(message('vision:rcnn:fourStepOutputFcnNotSupported'));
end

%--------------------------------------------------------------------------
function iAssertExperimentMonitorIsEmpty(parser,params)
if iWasUserSpecified(parser,'ExperimentMonitor') && ~isempty(params.Monitor)
    error(message('vision:rcnn:fourStepExperimentMonitorNotSupported'));
end

%--------------------------------------------------------------------------
function iCheckNumRegionsToSample(x,expectedNumel,fname)
if ~isscalar(x)
    validateattributes(x,{'numeric'},{'numel',expectedNumel},fname,'NumRegionsToSample')
end
vision.internal.cnn.validation.checkNumRegionsToSample(x, fname);


%--------------------------------------------------------------------------
function x = iExpandScalarNumRegions(x,n)
if isscalar(x)
    x = repelem(x,n,1);
end

%--------------------------------------------------------------------------
function tf = iWasUserSpecified(parser,param)
tf = ~ismember(param,parser.UsingDefaults);

%--------------------------------------------------------------------------
function classNames = iClassNamesIncludingBackground(params)
classNames = [params.ClassNames params.BackgroundLabel];

%--------------------------------------------------------------------------
function iAssertNotFourStageCheckpoint(detector)
if detector.TrainingStage >= 1 && detector.TrainingStage < 5
    % detector is a checkpoint saved using four-stage training. 
    error(message('vision:rcnn:endToEndFromFourStageCheckpoint'));
end

%--------------------------------------------------------------------------
function iWarnIfFourStepTrainingLossHasNaNs(info, step)
msg = message('vision:rcnn:fourStepTrainingLossHasNaNs',step);
vision.internal.cnn.warnIfLossHasNaNs(info, msg);

%--------------------------------------------------------------------------
function iAssertValidOverlapRangeSizeForEndToEnd(pname, value)
if isequal(size(value),[4 2])
    error(message('vision:rcnn:fourStepOverlapRangeUsedForEndToEnd',pname));
end

%--------------------------------------------------------------------------
function iAssertValidOptionsForEndToEnd(value)
if numel(value) == 4
    error(message('vision:rcnn:fourStepOptionsUsedForEndToEnd'));
end

%--------------------------------------------------------------------------
function iAssertValidNumRegionsToSampleForEndToEnd(value)
if numel(value) == 4
    error(message('vision:rcnn:fourStepNumRegionsUsedForEndToEnd'));
end

%--------------------------------------------------------------------------
function params = iEstimateAnchorBoxesForLayers(trainingData, network, params)
if ~iIsDetector(network) && ~isa(network,'nnet.cnn.LayerGraph')
    % SeriesNetwork, Layer array, or network by name.
    params.ModelSize = vision.internal.cnn.RCNNLayers.minimumObjectSize(network);

    % Estimate anchor boxes from training data.
    params = iEstimateAnchorBoxesFromTrainingData(trainingData, params);
end

%--------------------------------------------------------------------------
function [imageInfo, trainingData, options] = iCollectImageInfo(trainingData, rpnLayerGraph, imageInfoParams, params, options)
if params.TrainingDataWasTable
    imageInfo = params.ImageInfo;
else
    transformFcn = @(data)vision.internal.cnn.fastrcnn.validateImagesAndBoxesTransform(data,params.ColorPreprocessing);
    options = vision.internal.cnn.validationReportUtils.updateValidationDataTransform(options, ...
        @(ds)transform(ds,transformFcn));

    % Apply a transform to validate images and boxes.
    trainingData = transform(trainingData, transformFcn);

    imageInfo = vision.internal.cnn.rcnnDatasetStatistics(trainingData, rpnLayerGraph, imageInfoParams);

    % Copy and reset the given datastore, so external state events are
    % not reflected.
    trainingData = copy(trainingData);
    reset(trainingData);
end

%--------------------------------------------------------------------------
function [trainingData, imageInfo] = iCollectImageInfoAndScaleForTable(trainingData, params)
% Collect image size information and average image information. NB: This
% uses a parallel pool opened by setupExecutionEnvironment.
imageInfo = fastRCNNObjectDetector.collectImageInfo(trainingData, params);

% Scale training data if requested.
trainingData = fastRCNNObjectDetector.scaleImageData(trainingData, imageInfo, params);

%--------------------------------------------------------------------------
function data = iTransformForRPN(dataFromDatastore)
images = dataFromDatastore(:, 1);
boxes  = dataFromDatastore(:, 2);
% Image sizes are needed for RPN output layer.
imageSizes = cellfun(@size, images, 'UniformOutput', false);

data = horzcat(images, imageSizes, boxes);

%--------------------------------------------------------------------------
function data = iTransformForEndToEnd(dataFromDatastore)
images = dataFromDatastore(:, 1);
boxes  = dataFromDatastore(:, 2);
labels = dataFromDatastore(:, 3);

% Image sizes are needed for RPN output layer.
imageSizes = cellfun(@size, images, 'UniformOutput', false);
% The second column with boxes, goes to RPN output layer as well as FastRCNN output layer.
data = horzcat(images, imageSizes, boxes, labels);

%--------------------------------------------------------------------------
function batchingFunctions = iProvideEndToEndBatchingFunctions()
% Default input collate function.
batchingFunctions.InputFunctions = [];
% Pass through output collate function.
batchingFunctions.OutputFunctions = {@iPassThroughFcn, @iPassThroughFcn};
batchingFunctions.CatAndSliceStrategy = vision.internal.cnn.catAndSliceStrategy.EndToEndFasterRCNNCatAndSliceStrategy();

%--------------------------------------------------------------------------
function batchingFunctions = iProvideRPNBatchingFunctions()
batchingFunctions.InputFunctions = [];
batchingFunctions.OutputFunctions = {@iPassThroughFcn};
batchingFunctions.CatAndSliceStrategy = vision.internal.cnn.catAndSliceStrategy.RPNFasterRCNNCatAndSliceStrategy();

%--------------------------------------------------------------------------
function batch = iPassThroughFcn(batch, ~)
% function to batch a column of ground truth data to fast rcnn output layer
% and rpn output layer, when datastore is the training data input.

% Pass through
% Categorical labels will be cast per network precision.

%--------------------------------------------------------------------------
function fastRCNN = iCopyInputStatisticsToFastRCNN(fastRCNN,rpn)
% Copy input stats from RPN network to Fast R-CNN.
rpnIdx = arrayfun(@(x)isa(x,'nnet.cnn.layer.ImageInputLayer'),rpn.Layers);
fastIdx = arrayfun(@(x)isa(x,'nnet.cnn.layer.ImageInputLayer'),fastRCNN.Layers);
fastRCNN = replaceLayer(fastRCNN,...
    fastRCNN.Layers(fastIdx).Name, rpn.Layers(rpnIdx));

%--------------------------------------------------------------------------
function m = iInfoToSummaryMapAndStartValues()
% Full info content resides in the validation content. Use that to build
% the map from info to summary fields. All fields are initialized to [].
% This is used for OutputFcn support.
infoContent = vision.internal.cnn.FasterRCNNEndToEndValidationContent();
initialValue = repelem({[]},numel(infoContent.FieldNames),1);
m = [infoContent.FieldNames' infoContent.SummaryNames' initialValue];

