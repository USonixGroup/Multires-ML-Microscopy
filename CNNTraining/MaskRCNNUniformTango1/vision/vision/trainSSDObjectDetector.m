function [trainedDetector, info, ds] = trainSSDObjectDetector(trainingData, inputDetector, options, varargin)
%

% Copyright 2019-2024 The MathWorks, Inc.

    if nargin > 3
        [varargin{:}] = convertStringsToChars(varargin{:});
    end

    vision.internal.requiresNeuralToolbox(mfilename);
    [ssdLayerGraph, params] = iParseInputs(trainingData, inputDetector, options, mfilename, varargin{:});
    assert(isa(ssdLayerGraph,'nnet.cnn.LayerGraph'));

    % Support backward compatibility
    regressLayerIdx = find(arrayfun(@(x)isa(x,'nnet.cnn.layer.RCNNBoxRegressionLayer'),ssdLayerGraph.Layers), 1);

    % Add boxRegressionLayer
    if isempty(regressLayerIdx)
        regressLayerName = "anchorBoxRegression";
        regressionLayer = rcnnBoxRegressionLayer('Name',regressLayerName);
        locMergeIdx = find(arrayfun(@(x)isa(x,'nnet.cnn.layer.SSDMergeLayer'),...
            ssdLayerGraph.Layers),1,'last');
        ssdLayerGraph = addLayers(ssdLayerGraph,regressionLayer);
        ssdLayerGraph = connectLayers(ssdLayerGraph,ssdLayerGraph.Layers(locMergeIdx,1).Name,regressLayerName);
    end

    params.Verbose = options.Verbose;
    % Get anchor box information. Run network analysis to populate boxes.
    % Validate that replaced layer does not cause any issue.
    analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(ssdLayerGraph);
    analysis.applyConstraints();
    ssdLayerGraph = analysis.LayerGraph;

    % Get anchor box data from the network
    if ~isa(inputDetector,'ssdObjectDetector')
        % If input is layerGraph or DAGNetwork (backward compatibility)
        imageInputIdx = find(arrayfun(@(x)isa(x,'nnet.cnn.layer.ImageInputLayer'),...
            ssdLayerGraph.Layers),1,'first');
        params.InputSize = ssdLayerGraph.Layers(imageInputIdx).InputSize;
        params.TiledAnchorBoxes = ssdObjectDetector.calculateTiledAnchorBoxesLgraphForInference(ssdLayerGraph);
        params.AnchorBoxes = ssdObjectDetector.baseAnchorBoxes(ssdLayerGraph);
        anchorBoxData = ssdObjectDetector.extractTiledAnchorBoxesLayerGraphForTrainer(ssdLayerGraph);
        ssdLayerGraph = ssdObjectDetector.removeAnchorBoxLayer(ssdLayerGraph,analysis);
        ssdLayerGraphInitial = ssdObjectDetector.removeLossAndMergeLayersFromLgraph(ssdLayerGraph);
        checkPointNetwork = ssdObjectDetector.updateImageInputLayer(ssdLayerGraphInitial);
        params.ModelName = char(params.ClassNames(1));
    else
        params.InputSize = inputDetector.InputSize;
        params.TiledAnchorBoxes = inputDetector.TiledAnchorBoxes ;
        params.AnchorBoxes = inputDetector.AnchorBoxes;
        anchorBoxData = ssdObjectDetector.extractTiledAnchorBoxesForTrainer(inputDetector);
        checkPointNetwork = inputDetector;
        params.ModelName = inputDetector.ModelName;
    end
    

    % Configure printer for verbose printing
    printer = vision.internal.MessagePrinter.configure(options.Verbose);
    ssdObjectDetector.printHeader(printer, params.ClassNames);

    % Setup checkpoint.
    % Assemble the detector with the initialized dlnetwork.
    checkPointSaver = iConfigureCheckpointSaver(options, params,checkPointNetwork);

    % create datastore. Need to scale roi depending on input
    % image size for this it uses the network layers.
    applyTransformFcn = @(datastore)ssdObjectDetector.createTrainingDatastore(...
        datastore, anchorBoxData, params);
    ds = applyTransformFcn(trainingData);

    % Update ValidationData with necessary transforms, if needed.
    options = vision.internal.cnn.validationReportUtils.updateValidationDataTransform(...
        options, applyTransformFcn);

    mapping = ssdObjectDetector.createMIMODatastoreCellMapping(params.InputSize);

    % Set execution settings required by MIMOtrainer.
    precision = nnet.internal.cnn.util.Precision('single');
    executionSettings = nnet.internal.cnn.assembler.setupExecutionEnvironment( options, ds, precision );
    
    isValidationDataSpecified = vision.internal.cnn.validationReportUtils.isValidationSpecified(options);
    if isValidationDataSpecified 
        infoContent = vision.internal.cnn.FastRCNNAndRPNValidationContent();
        validationPredictStrategyFcn = @vision.internal.cnn.ValidationPredictDatastoreStrategy;
    else
        infoContent = vision.internal.cnn.FastRCNNAndRPNContent();
        validationPredictStrategyFcn = @(varargin)[];
    end
    columnStrategy = vision.internal.cnn.ClassificationRegressionColumns(isValidationDataSpecified);
    axesConfigFactory = vision.internal.cnn.SSDAxesConfig();
    axesConfig = axesConfigFactory.AxesConfiguration;
    
    summaryBuilderFcn = @vision.internal.cnn.SSDSummary.makeSummary;
    summaryMonitorInfo.Monitor = params.Monitor;
    summaryMonitorInfo.SummaryBuilderFcn = summaryBuilderFcn;
    summaryMonitorInfo.MonitorFieldInformation = vision.internal.cnn.FastRCNNAndRPNValidationContent.monitorSummaryFields(isValidationDataSpecified);

    try
        [ssdNetwork, info] = vision.internal.cnn.trainNetwork(...
            ds, ssdLayerGraph, options, executionSettings, mapping, checkPointSaver, ...
            summaryMonitorInfo, ...
            infoContent, ...
            columnStrategy, ...
            axesConfig, ...
            iProvideBatchingFunctions(analysis), ...
            validationPredictStrategyFcn, ...
            iInfoToSummaryMapAndStartValues());
    catch ME
        if strcmp(ME.identifier,'nnet_cnn:internal:cnn:GeneralDatastoreDispatcher:VariableInputSizes')
            error(message('vision:rcnn:unableToBatchImages'));
        else
            rethrow(ME);
        end
    end

    ssdLayerGraphTrained = layerGraph(ssdNetwork);
    % Remove boxRegressionLayer and classification layers
    if isempty(regressLayerIdx)
        ssdLayerGraphTrained = removeLayers(ssdLayerGraphTrained,[regressLayerName, params.ClassificationLayer.Name]);
    else
        rcnnBoxRegressLayer = arrayfun(@(x)isa(x,'nnet.cnn.layer.RCNNBoxRegressionLayer'),ssdLayerGraphTrained.Layers,'UniformOutput', true);
        rcnnBoxRegressionLayerIdx = find(rcnnBoxRegressLayer);
        % It will be called after training and before assembling the detector
        rcnnClassLayer = arrayfun(@(x)isa(x,'vision.internal.cnn.SSDHardNegativeMiningLossLayer'),ssdLayerGraphTrained.Layers,'UniformOutput', true);
        classificationLayerIdx = find(rcnnClassLayer);
        ssdLayerGraphTrained = removeLayers(ssdLayerGraphTrained,{ssdLayerGraphTrained.Layers(rcnnBoxRegressionLayerIdx,1).Name; ssdLayerGraphTrained.Layers(classificationLayerIdx,1).Name});
    end
    ssdNetwork = dlnetwork(ssdLayerGraphTrained);
    trainedDetector = ssdObjectDetector.assembleDetector(params, ssdNetwork);
    ssdObjectDetector.printFooter(printer);
end

%--------------------------------------------------------------------------
function [network, params] = iParseInputs(trainingData, network, options, fname, varargin)

    validateattributes(trainingData, {'matlab.io.Datastore'}, {'scalar'}, fname, 'trainingData', 1);
    trainingData = copy(trainingData);

    params.ClassNames = vision.internal.inputValidation.checkGroundTruthDatastore(trainingData);
    params.numClasses = numel(params.ClassNames);
    params.BackgroundLabel  = vision.internal.cnn.uniqueBackgroundLabel(params.ClassNames);

    if options.DispatchInBackground
        error(message('vision:ObjectDetector:backgroundUnsupportedWithDatastore'));
    end
    % Check network.
    network = vision.internal.cnn.validation.checkNetwork(network, fname, ...
        {'nnet.cnn.LayerGraph', 'ssdObjectDetector'});    
    
    if isa(network,'ssdObjectDetector')
        vision.internal.cnn.validation.checkClassNamesMatchGroundTruth([network.ClassNames' params.BackgroundLabel]', params.ClassNames, params.BackgroundLabel);
        inputSize = network.InputSize;
        isWarningFlag = false;
        network = layerGraph(network.Network);
        [network, params] = iUpdatedClassificationLayer(network,params,isWarningFlag);
        params.OutputLayers = [];        
    else
        outputLayers = iCheckSSDMergeAndAnchorBoxLayers(network, params.numClasses);
        iCheckSSDOutputLayers(network);
        isWarningFlag = true;
        [network, params] = iUpdatedClassificationLayer(network,params,isWarningFlag);
        params.OutputLayers = outputLayers;
        imageInputIdx = find(arrayfun(@(x)isa(x,'nnet.cnn.layer.ImageInputLayer'),...
            network.Layers),1,'first');
        inputSize = network.Layers(imageInputIdx).InputSize;
    end

    sampleData = read(trainingData);
    image = sampleData{1,1};
    if size(image,3)~= inputSize(3)
        error(message('vision:ssd:invalidInputImageChannelSize'));
    end
    reset(trainingData);
    % Check training options.
    allowValidationDatastore = true; % Allow validation data as datastore
    isTrainingDataTable = false; % Always a datastore
    vision.internal.cnn.validation.checkTrainingOptions(options,fname,allowValidationDatastore,isTrainingDataTable);

    % Check optional inputs.
    p = inputParser;
    p.addParameter('PositiveOverlapRange', [0.5 1]);
    p.addParameter('NegativeOverlapRange', [0.0 0.5]);
    p.addParameter('ExperimentMonitor', 'none');
    p.parse(varargin{:});
    userInput = p.Results;

    params.Monitor = vision.internal.cnn.validation.checkExperimentMonitor(p, userInput, options, mfilename);

    vision.internal.cnn.validation.checkOverlapRatio(userInput.PositiveOverlapRange, fname, 'PositiveOverlapRange');
    vision.internal.cnn.validation.checkOverlapRatio(userInput.NegativeOverlapRange, fname, 'NegativeOverlapRange');

    params.PositiveOverlapRange = double(userInput.PositiveOverlapRange);
    params.NegativeOverlapRange = double(userInput.NegativeOverlapRange);
    vision.internal.cnn.validation.checkPositiveAndNegativeOverlapRatioDoNotOverlap(params);
end

%--------------------------------------------------------------------------
function iCheckSSDOutputLayers(network)
    validOutputLayers = {...
        'nnet.cnn.layer.FocalLossLayer',...
        'nnet.cnn.layer.RCNNBoxRegressionLayer',...
        };
    outLayers = network.Layers(ismember({network.Layers.Name},network.OutputNames));
    outLayerClassNames = arrayfun(@class,outLayers,'UniformOutput',false);
    if ~all(ismember(outLayerClassNames,validOutputLayers))
        error(message('vision:ssd:invalidOutputLayers'));
    end
end

%--------------------------------------------------------------------------
function outputLayers = iCheckSSDMergeAndAnchorBoxLayers(network, numClasses)
    ssdMergeLayerClassName = 'nnet.cnn.layer.SSDMergeLayer';
    anchorBoxLayerClassName = 'nnet.cnn.layer.AnchorBoxLayer';
    ssdMergeLayerIdx = find(arrayfun(@(x)isa(x,ssdMergeLayerClassName),network.Layers));
    anchorBoxLayerIdx = find(arrayfun(@(x)isa(x,anchorBoxLayerClassName),network.Layers));

    % The number of anchorBoxLayers cannot be empty.
    if isempty(anchorBoxLayerIdx)
        error(message('vision:ssd:invalidNumAnchorBoxLayers'));
    end

    % There must be two ssdMergeLayers.
    if numel(ssdMergeLayerIdx) ~= 2
        error(message('vision:ssd:invalidNumSSDMergeLayers'));
    end

    % Each ssdMergeLayer must have same number of inputs.
    if network.Layers(ssdMergeLayerIdx(1)).NumInputs ~= network.Layers(ssdMergeLayerIdx(2)).NumInputs
        error(message('vision:ssd:invalidNumInputsForSSDMergeLayers'));
    end

    % Number of anchorBoxLayers must be equal to the number of inputs to
    % the ssd merge layers.
    if numel(anchorBoxLayerIdx) ~= network.Layers(ssdMergeLayerIdx(1)).NumInputs
        error(message('vision:ssd:invalidNumAnchorBoxLayers'));
    end

    analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(network);
    ssdMergeLayerAnalyzerIdx = strcmp({analysis.ExternalLayers.Name},network.Layers(ssdMergeLayerIdx(1)).Name);
    softMaxLayerIndex = find(arrayfun(@(x)isa(x,'nnet.cnn.layer.SoftmaxLayer'),network.Layers));
    softMaxLayerName = network.Layers(softMaxLayerIndex).Name;

    if strcmp(analysis.LayerAnalyzers(ssdMergeLayerAnalyzerIdx).Outputs.Destination{1},softMaxLayerName)
        classMergeLayer = network.Layers(ssdMergeLayerIdx(1));
        regMergeLayer = network.Layers(ssdMergeLayerIdx(2));
    else
        regMergeLayer = network.Layers(ssdMergeLayerIdx(1));
        classMergeLayer = network.Layers(ssdMergeLayerIdx(2));
    end

    % One of the ssdMergeLayers must have 4 channels for regression.
    if regMergeLayer.NumChannels ~= 4
        error(message('vision:ssd:invalidNumChannelsInRegressionMergeLayer'));
    end

    % One of the ssdMergeLayers must have numClasses + 1 (for background)
    % number of channels for classification.
    if classMergeLayer.NumChannels ~= numClasses + 1
        error(message('vision:ssd:invalidNumClassesForTraining','classMergeLayer.NumChannels','numClasses'));
    end

    dg = vision.internal.cnn.RCNNLayers.digraph(network);
    fdg = flipedge(dg);
    if ~analysis.LayerAnalyzers(2,1).IsLayerInputValid
        error(message('vision:ssd:InvalidLayerSize'));
    end

    % Check classification input sizes to the ssdMergeLayer.
    classMergeLayerIdx = strcmp({analysis.ExternalLayers.Name},classMergeLayer.Name);
    classMergeLayerSources = analysis.LayerAnalyzers(classMergeLayerIdx).Inputs.Source;
    classificationPredictionLayers = string(classMergeLayerSources)';
    anchorBoxLayerNames = {network.Layers(anchorBoxLayerIdx).Name};

    % Extracting the classification and regression prediction layer connected to each 
    % anchorBoxLayer and storing the prediction layer name in desired output order.
    outputLayers = cell(1,2*numel(classMergeLayerSources));
    for ii = 1:numel(classMergeLayerSources)
        % Extract the anchorBoxLayers and save the information of 
        % child layers (prediction layers)
        currentClassificationPredictionLayerName = classMergeLayerSources{ii};
        inputSize = analysis.LayerAnalyzers(classMergeLayerIdx).Inputs.Size{ii};
        names = dfsearch(fdg,currentClassificationPredictionLayerName);
        firstAnchorBoxLayerIdx = find(matches(names, anchorBoxLayerNames),1,'first');
        currentAnchorBoxLayerName = names(firstAnchorBoxLayerIdx);
        currentAnchorBoxLayerNameIdx = strcmp({analysis.ExternalLayers.Name}, currentAnchorBoxLayerName);
        currentAnchorBoxLayerSources = analysis.LayerAnalyzers(currentAnchorBoxLayerNameIdx);
        firstChildLayerOfCurrentAnchorBox = currentAnchorBoxLayerSources.Outputs.Destination{1,1}{1};
        firstChildLayerOfCurrentAnchorBoxIdx = strcmp({analysis.ExternalLayers.Name}, firstChildLayerOfCurrentAnchorBox);
        firstChildLayerOfCurrentAnchorBoxName = analysis.LayerAnalyzers(firstChildLayerOfCurrentAnchorBoxIdx).Name;
        firstChildLayerOfCurrentAnchorBoxSpatialDim = analysis.LayerAnalyzers(firstChildLayerOfCurrentAnchorBoxIdx).Outputs.Size{1};
        secondChildLayerOfCurrentAnchorBox = currentAnchorBoxLayerSources.Outputs.Destination{1,1}{2};
        secondChildLayerOfCurrentAnchorBoxIdx = strcmp({analysis.ExternalLayers.Name}, secondChildLayerOfCurrentAnchorBox);
        secondChildLayerOfCurrentAnchorBoxName = analysis.LayerAnalyzers(secondChildLayerOfCurrentAnchorBoxIdx).Name;
        secondChildLayerOfCurrentAnchorBoxSpatialDim = analysis.LayerAnalyzers(secondChildLayerOfCurrentAnchorBoxIdx).Outputs.Size{1};

        currentAnchorBoxLayerSpatialDim = currentAnchorBoxLayerSources.Outputs.Size{1};

        % Verifying if classification prediction layer out of two connected
        % child layer of parent anchorBoxLayer
        isClassPredictionLayerSecondChild = find(ismember(classificationPredictionLayers,secondChildLayerOfCurrentAnchorBox), 1);
        if ~isempty(isClassPredictionLayerSecondChild)
            % If second prediction layer is classification layer
            outputLayers{2*ii-1} = secondChildLayerOfCurrentAnchorBox;
            outputLayers{2*ii} = firstChildLayerOfCurrentAnchorBox;
            if ~isequal(currentAnchorBoxLayerSpatialDim(1,1:2),secondChildLayerOfCurrentAnchorBoxSpatialDim(1,1:2))
                error(message('vision:ssd:invalidClassificationSpatialDimension',secondChildLayerOfCurrentAnchorBoxName, currentAnchorBoxLayerName))
            end
            if ~isequal(currentAnchorBoxLayerSpatialDim(1,1:2),firstChildLayerOfCurrentAnchorBoxSpatialDim(1,1:2))
                error(message('vision:ssd:invalidRegressionSpatialDimension',firstChildLayerOfCurrentAnchorBoxName, currentAnchorBoxLayerName))
            end
        else
            % First child is classification prediction layer.
            outputLayers{2*ii-1} = firstChildLayerOfCurrentAnchorBox;
            outputLayers{2*ii} = secondChildLayerOfCurrentAnchorBox;
            if ~isequal(currentAnchorBoxLayerSpatialDim(1,1:2),secondChildLayerOfCurrentAnchorBoxSpatialDim(1,1:2))
                error(message('vision:ssd:invalidRegressionSpatialDimension',firstChildLayerOfCurrentAnchorBoxName, currentAnchorBoxLayerName))
            end
            if ~isequal(currentAnchorBoxLayerSpatialDim(1,1:2),firstChildLayerOfCurrentAnchorBoxSpatialDim(1,1:2))
                error(message('vision:ssd:invalidClassificationSpatialDimension',secondChildLayerOfCurrentAnchorBoxName, currentAnchorBoxLayerName))
            end
        end


        anchorBoxLayer = analysis.ExternalLayers(strcmp({analysis.ExternalLayers.Name}, currentAnchorBoxLayerName));
        numAnchors = size(anchorBoxLayer.AnchorBoxes,1);
        numFilters = numAnchors * (numClasses + 1);
        if inputSize(3) ~= numFilters
            correctSize = mat2str([inputSize([1 2]) numFilters]);
            incorrectSize = mat2str(inputSize);
            error(message('vision:ssd:invalidInputSizeForSSDMergeLayer',classMergeLayer.Name,correctSize,incorrectSize));
        end
    end
end
%--------------------------------------------------------------------------
function checkpointSaver = iConfigureCheckpointSaver(options, params,checkPointNetwork)
   if ~isa(checkPointNetwork,'ssdObjectDetector')
       dlnet = dlnetwork(checkPointNetwork);
       dagNet = ssdObjectDetector.convertToDAGNetwork(params,dlnet);
   else
       dagNet = checkPointNetwork.Network;
   end
   params.InitializedDAGNet = dagNet;
    saver = nnet.internal.cnn.trainNetwork.reporter.checkpoint.Saver(options.CheckpointPath, options.CheckpointFrequency);
    if strcmp(options.CheckpointFrequencyUnit, "epoch")
        checkpointSaver = vision.internal.cnn.DetectorCheckpointEpochSaver(saver);
    else
        checkpointSaver = vision.internal.cnn.DetectorCheckpointIterationSaver(saver);
    end
    checkpointSaver.CheckpointPrefix = 'ssd';
    checkpointSaver.DetectorFcn = @(x,y)ssdObjectDetector.detectorCheckpoint(x,y);
    checkpointSaver.Detector = ssdObjectDetector.assembleDetector(params);
end
%--------------------------------------------------------------------------
function batchingFunctions = iProvideBatchingFunctions(analysis)

    % Find locations of each output layer in the layers array.
    % SSDBCEHNM output layer
    idx = arrayfun(@(x)isa(x,'vision.internal.cnn.SSDHardNegativeMiningLossLayer'),...
        analysis.ExternalLayers);
    ssdClassifiationLossLayer = find(idx,1,'last');

    % rcnnBoxRegression output layer
    idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.RCNNBoxRegressionLayer'),...
        analysis.ExternalLayers);
    rcnnBoxLayerIdx = find(idx,1,'last');

    [~,outputOrdering] = sort([ssdClassifiationLossLayer, rcnnBoxLayerIdx]);

    batchingFunctions.InputFunctions = [];
    % Setup classification and regression output batching functions.
    batchingFunctions.OutputFunctions = {@iClassificationResponseBatchFcn, ...
                                         @iRegressionResponseBatchFcn};

    batchingFunctions.OutputFunctions = batchingFunctions.OutputFunctions(outputOrdering);

    ordering.InputOrdering = 1;
    ordering.OutputOrdering = outputOrdering;
    batchingFunctions.CatAndSliceStrategy = vision.internal.cnn.catAndSliceStrategy.SSDCatAndSliceStrategy();
    batchingFunctions.CatAndSliceStrategy.Ordering = ordering;
end
%--------------------------------------------------------------------------
function batch = iRegressionResponseBatchFcn(TColumn, ~)
% function to batch a column of regression response data.

    batchSize = size(TColumn, 1);

    numBoxes = size(TColumn{1}{1}, 1);
    batch = zeros(numBoxes, 4, 1, batchSize);

    for batchidx = 1:batchSize
        batch(:, :, :, batchidx) = TColumn{batchidx}{1};
    end

    % Put x, y, w, h in the 3rd dimension.
    batch = permute(batch, [1 3 2 4]);

    % All boxes get equal weight, scaled by number of positive matches.
    weights = ones(size(batch));
    weights(batch == 0) = 0;
    weights = weights./(sum(weights(:)));
    weights = weights.*4;
    batch = {batch, weights};
end
%------------------------------------------------------------------------------------------------
function batch = iClassificationResponseBatchFcn(TColumn, ~)
% function to batch a column of classification response data. TColumn 
% has labels and positive assignment flag

    batchSize = size(TColumn, 1);    
    numBoxes = size(TColumn{1}{1}, 1);
    catsIncludingBackground = categories(TColumn{1}{1});
    numClassesIncludingBackground = numel(catsIncludingBackground);
    % Adding 1 to incorporate positive assignment flag array for each batch 
    batch = zeros(numBoxes, numClassesIncludingBackground+1, 1, batchSize);

    for batchidx = 1:batchSize
        % TColumn{batchidx}{1} is categorical array of labels which has been converted using 
        % vision.internal.cnn.boxAssignmentUtils.assignBoxesToGroundTruthBoxes
        % and this function always fix the Background label as last category.
        % function. One-hot encoded vectors of this categorical array of labels 
        % will have background as last column as the elements of the one-hot 
        % encoded vectors match the same order in categories(TColumn{idx}{1} 
        onehot = onehotencode(TColumn{batchidx}{1},2); 
        batch(:, :, :, batchidx) = [onehot TColumn{batchidx}{2}];
    end

    % Put categories in the 3rd dimension.
    batch = permute(batch, [1 3 2 4]);
end
%--------------------------------------------------------------------------
function m = iInfoToSummaryMapAndStartValues()
% Full info content resides in the validation content. Use that to build
% the map from info to summary fields. All fields are initialized to [].
% This is used for OutputFcn support.
infoContent = vision.internal.cnn.FastRCNNAndRPNValidationContent();
initialValue = repelem({[]},numel(infoContent.FieldNames),1);
m = [infoContent.FieldNames' infoContent.SummaryNames' initialValue];
end
%----------------------------------------------------------------------------------------------------
function [lgraph, params] = iUpdatedClassificationLayer(lgraph, params,isWarning)
% Maintain the same classification Layer name but replacing it with
% SSDHardNegativeMiningLossLayer to incorporating Hard Negative Mining
if isempty(find(arrayfun(@(x)isa(x,'vision.internal.cnn.SSDHardNegativeMiningLossLayer'),lgraph.Layers), 1))
    focalLossLayerClassName = 'nnet.cnn.layer.FocalLossLayer';
    focalLayerIdx = find(arrayfun(@(x)isa(x,focalLossLayerClassName),lgraph.Layers));
    classificationLayerName = 'nnet.cnn.layer.ClassificationOutputLayer';
    classificationLayerIdx = find(arrayfun(@(x)isa(x,classificationLayerName),lgraph.Layers));
    if ~isempty(focalLayerIdx)
        classificationOutputLayer = vision.internal.cnn.SSDHardNegativeMiningLossLayer(lgraph.Layers(focalLayerIdx).Name);
        if isWarning
            warning((message('vision:ssd:replaceFocalLossLayerToSSDHardNegativeMiningLayer')));
        end
        params.ClassificationLayer = lgraph.Layers(focalLayerIdx);
        lgraph = replaceLayer(lgraph,lgraph.Layers(focalLayerIdx).Name,...
            classificationOutputLayer);
    else
        if isempty(classificationLayerIdx)
            anchorBoxLayerName = 'nnet.cnn.layer.SoftmaxLayer';
            anchorBoxLayerIdx = find(arrayfun(@(x)isa(x,anchorBoxLayerName),lgraph.Layers));
            ssdNegMiningLossLayer = vision.internal.cnn.SSDHardNegativeMiningLossLayer('ssdNegMiningLossLayer');
            lgraph = addLayers (lgraph,ssdNegMiningLossLayer);
            lgraph = connectLayers(lgraph,lgraph.Layers(anchorBoxLayerIdx).Name,'ssdNegMiningLossLayer');
            ssdNegMiningLayerIdx = find(arrayfun(@(x)isa(x,'vision.internal.cnn.SSDHardNegativeMiningLossLayer'),lgraph.Layers(anchorBoxLayerIdx+1:end,1)));
            params.ClassificationLayer = lgraph.Layers(anchorBoxLayerIdx+ssdNegMiningLayerIdx);
        else
            classificationOutputLayer = vision.internal.cnn.SSDHardNegativeMiningLossLayer(lgraph.Layers(focalLayerIdx).Name);
            lgraph = replaceLayer(lgraph,lgraph.Layers(classificationLayerIdx).Name,...
                classificationOutputLayer);
        end
    end
end
end
