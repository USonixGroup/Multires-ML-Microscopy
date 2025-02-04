function [detector, info] = trainFastRCNNObjectDetector(trainingData, network, options, varargin)

% Copyright 2016-2023 The MathWorks, Inc.

if nargin > 3
    [varargin{:}] = convertStringsToChars(varargin{:});
end

vision.internal.requiresNeuralToolbox(mfilename);

% Initialize warning logger. Logs warnings issued during training and
% reissues them at end of training when Verbose is true.
vision.internal.cnn.WarningLogger.initialize();

[lgraph, params] = iParseInputs(...
    trainingData, network, options, varargin{:});

checkpointSaver = iConfigureCheckpointSaver(options, params);

printer = iConfigureVerbosePrinter(options.Verbose);

fastRCNNObjectDetector.printHeader(printer, params.ClassNames);

% Setup execution environment. This includes opening a pool for
% multi-gpu or parallel training. NB: This pool is reused for
% computing region proposals in parallel.
executionSettings = fastRCNNObjectDetector.setupExecutionEnvironment(options,params.UseParallel);

[trainingData, imageInfo, options] = iCollectImageInfoAndScale(trainingData, params, options);

params.DispatchInBackground = executionSettings.backgroundPrefetch;
params.ImageInfo = imageInfo;

% Train detector.
[detector, ~, info] = fastRCNNObjectDetector.train(trainingData, lgraph, options, executionSettings, params, checkpointSaver);

iWarnIfTrainingLossHasNaNs(info);

detector.ModelName = params.ModelName;

printer = vision.internal.MessagePrinter.configure(options.Verbose);
printer.linebreak;
rcnnObjectDetector.printFooter(printer);
rcnnObjectDetector.printRowOfStars(printer);

%--------------------------------------------------------------------------
function printer = iConfigureVerbosePrinter(verbose)
printer = vision.internal.MessagePrinter.configure(verbose);

%--------------------------------------------------------------------------
function [lgraph, params] = iParseInputs(trainingDs, network, options, varargin)

network = vision.internal.cnn.validation.checkNetwork(network, mfilename, ...
   {'SeriesNetwork', 'nnet.cnn.layer.Layer', 'nnet.cnn.LayerGraph', 'fastRCNNObjectDetector'},...
   vision.internal.cnn.RCNNLayers.SupportedPretrainedNetworks);

allowValidationDatastore = true;
vision.internal.cnn.validation.checkTrainingOptions(options, mfilename,...
    allowValidationDatastore,istable(trainingDs));

p = inputParser;
p.addParameter('RegionProposalFcn', @rcnnObjectDetector.proposeRegions);
p.addParameter('UseParallel', vision.internal.useParallelPreference());
p.addParameter('PositiveOverlapRange', [0.5 1]);
p.addParameter('NegativeOverlapRange', [0.1 0.5]); 
p.addParameter('NumStrongestRegions', 2000);
p.addParameter('SmallestImageDimension', []);
p.addParameter('NumRegionsToSample',128);
p.addParameter('FreezeBatchNormalization', ...
    vision.internal.cnn.RCNNLayers.freezeBatchNormHeuristic(options(1).MiniBatchSize));
p.addParameter('InternalOptions', iInternalDefaultOptions()); % Hidden parameter.
p.addParameter('ExperimentMonitor', 'none');
p.parse(varargin{:});

userInput = p.Results;

params.Monitor = vision.internal.cnn.validation.checkExperimentMonitor(p, userInput, options, mfilename);

rcnnObjectDetector.checkRegionProposalFcn(userInput.RegionProposalFcn);

vision.internal.inputValidation.validateLogical(userInput.UseParallel,'UseParallel');

vision.internal.cnn.validation.checkOverlapRatio(userInput.PositiveOverlapRange, mfilename, 'PositiveOverlapRange');
vision.internal.cnn.validation.checkOverlapRatio(userInput.NegativeOverlapRange, mfilename, 'NegativeOverlapRange');
iCheckNumRegionsToSample(userInput.NumRegionsToSample)
vision.internal.cnn.validation.checkStrongestRegions(userInput.NumStrongestRegions, mfilename);
vision.internal.inputValidation.validateLogical(userInput.FreezeBatchNormalization,'FreezeBatchNormalization');

params.PositiveOverlapRange          = double(userInput.PositiveOverlapRange);
params.NegativeOverlapRange          = double(userInput.NegativeOverlapRange);
params.RegionProposalFcn             = userInput.RegionProposalFcn;
params.UsingDefaultRegionProposalFcn = ismember('RegionProposalFcn', p.UsingDefaults);
params.NumStrongestRegions           = double(userInput.NumStrongestRegions);
params.UseParallel                   = logical(userInput.UseParallel);
params.ImageScale                    = double(userInput.SmallestImageDimension);
params.ScaleImage                    = ~isempty(params.ImageScale);
params.InternalOptions               = userInput.InternalOptions;
params.NumRegionsToSample            = double(userInput.NumRegionsToSample);
params.FreezeBatchNormalization      = logical(userInput.FreezeBatchNormalization);
params.DispatchInBackground          = options.DispatchInBackground;
params.TrainingDataWasTable          = istable(trainingDs);
params.MiniBatchSize                 = options.MiniBatchSize;
params.Verbose                       = options.Verbose;

if ~params.TrainingDataWasTable && params.ScaleImage
    error(message('vision:rcnn:scalingNotAllowedForDatastore'));
end

if ~params.TrainingDataWasTable && options.DispatchInBackground
    error(message('vision:ObjectDetector:backgroundUnsupportedWithDatastore'));
end

if params.TrainingDataWasTable
    vision.internal.cnn.validation.checkGroundTruth(trainingDs, mfilename);
    params.ClassNames = trainingDs.Properties.VariableNames(2:end);
else
    params.ClassNames = ...
        vision.internal.inputValidation.checkGroundTruthDatastore(trainingDs);
end

params.BackgroundLabel               = vision.internal.cnn.uniqueBackgroundLabel(params.ClassNames);
params.ModelName                     = params.ClassNames{1};
params.NumClasses                    = numel(params.ClassNames);

vision.internal.cnn.validation.checkPositiveAndNegativeOverlapRatioDoNotOverlap(params);

% UseParallel must be true for multi-gpu or parallel ExecutionEnvironment.
if any(strcmp(options.ExecutionEnvironment,{'multi-gpu','parallel'})) && ~params.UseParallel
    error(message('vision:rcnn:UseParallelMustBeTrueForParallelTraining'))
end

if isa(network,'fastRCNNObjectDetector')
    lgraph = layerGraph(network.Network);
    
    vision.internal.cnn.validation.checkClassNamesMatchGroundTruth(network.ClassNames, params.ClassNames, params.BackgroundLabel);
    
else
    if isa(network,'nnet.cnn.LayerGraph')
        % Check user created LayerGraph. This must be a valid Fast R-CNN.
        constraints =  vision.internal.cnn.RCNNLayers.constraints('fast-rcnn',params.NumClasses);
        analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(network);
        analysis.applyConstraints(constraints);
        try
            analysis.throwIssuesIfAny();
        catch ME
            throwAsCaller(ME);
        end
        lgraph = network;
                
    else
        % SeriesNetwork, Layer array, or network by name.
              
        allowMultiChannel = ~params.TrainingDataWasTable;
        try
            lgraph = vision.internal.cnn.RCNNLayers.create(params.NumClasses, network, 'fast-rcnn', [], allowMultiChannel);
        catch ME
            throwAsCaller(ME);
        end
        
    end
      
end

% Update ROI scale factor.
lgraph = vision.internal.cnn.RCNNLayers.updateROIPoolingLayerScaleFactor(lgraph);

% Remove unsupported data augmentation values.
lgraph = vision.internal.rcnn.removeAugmentationIfNeeded(lgraph,{'randcrop','randfliplr'});

% Determine if average image needs to be computed.
params.NeedsZeroCenterNormalization = false;

% set default MinObjectSize if required. This defaults to network min size            
params.MinObjectSize = fastRCNNObjectDetector.determineMinBoxSize(lgraph);

% Extract network's image input size.
imageLayerIdx = find(arrayfun(@(x)isa(x,'nnet.cnn.layer.ImageInputLayer'),lgraph.Layers),...
    1,'first');
params.InputSize = lgraph.Layers(imageLayerIdx).InputSize;

if ~params.TrainingDataWasTable && params.UsingDefaultRegionProposalFcn &&...
        (params.InputSize(3) > 3 || params.InputSize(3) == 2)
    error(message('vision:rcnn:regionProposalFcnRequiredForMultiChannel'))
end

params.ColorPreprocessing = vision.internal.cnn.utils.colorPreprocessingForImageInputSize(params.InputSize);

vision.internal.cnn.validation.checkImageScale(userInput.SmallestImageDimension, params.InputSize, mfilename);

params.ImageLayerIdx = imageLayerIdx;

% Validate that batch norm layers are trained if a user asks for them to be
% frozen. This needs to be checked for layer arrays and layer graphs. Other
% input types always have trained BN layers.
if params.FreezeBatchNormalization &&  ...
        (isa(network,'nnet.cnn.LayerGraph') || isa(network,'nnet.cnn.layer.Layer'))
    
    vision.internal.cnn.validation.checkBatchNormLayersAreTrained(network);
end

%--------------------------------------------------------------------------
function s = iInternalDefaultOptions()
s = vision.internal.cnn.defaultInternalFastOptions();

%--------------------------------------------------------------------------
function checkpointSaver = iConfigureCheckpointSaver(options, params)
saver = nnet.internal.cnn.trainNetwork.reporter.checkpoint.Saver(options.CheckpointPath, options.CheckpointFrequency);
if strcmp(options.CheckpointFrequencyUnit, "epoch")
    checkpointSaver = vision.internal.cnn.DetectorCheckpointEpochSaver(saver);
else
    checkpointSaver = vision.internal.cnn.DetectorCheckpointIterationSaver(saver);
end
checkpointSaver.CheckpointPrefix = 'fast_rcnn';
checkpointSaver.DetectorFcn = @(x,y)fastRCNNObjectDetector.detectorCheckpoint(x,y,params.FreezeBatchNormalization);
checkpointSaver.Detector = fastRCNNObjectDetector.partiallyMakeDetector(params);

%--------------------------------------------------------------------------
function iWarnIfTrainingLossHasNaNs(info)
msg = message('vision:rcnn:networkTrainingLossHasNaNs');
vision.internal.cnn.warnIfLossHasNaNs(info,msg);

%--------------------------------------------------------------------------
function iCheckNumRegionsToSample(x)
validateattributes(x,{'numeric'},{'scalar'},mfilename,'NumRegionsToSample')
vision.internal.cnn.validation.checkNumRegionsToSample(x, mfilename);

%--------------------------------------------------------------------------
function [trainingData, imageInfo, options] = iCollectImageInfoAndScale(trainingData, params, options)
% Collect image size information and average image information. NB: This
% uses a parallel pool opened by setupExecutionEnvironment.
if params.TrainingDataWasTable

    % Collect image size information and average image information. NB: This
    % uses a parallel pool opened above by setupExecutionEnvironment.
    imageInfo    = fastRCNNObjectDetector.collectImageInfo(trainingData,params);

    % Scale training data if requested.
    trainingData = fastRCNNObjectDetector.scaleImageData(trainingData, imageInfo, params);
else
    transformFcn = @(data)vision.internal.cnn.fastrcnn.validateImagesAndBoxesTransform(data,params.ColorPreprocessing);
    % Apply a transform to validate images and boxes.
    applyTransform = @(ds)transform(ds,transformFcn);
    trainingData = applyTransform(trainingData);

    options = vision.internal.cnn.validationReportUtils.updateValidationDataTransform(options, applyTransform);

    imageInfo    = rcnnObjectDetector.imageRegressionInfoFromGroundTruthDatastore(trainingData, params);

    % Copy and reset the given datastore, so external state events are
    % not reflected.
    trainingData = copy(trainingData);
    reset(trainingData);
end
