function [trainedDetector, info] = trainYOLOv2ObjectDetector(trainingData,detector, options, varargin)

% Copyright 2018-2024 The MathWorks, Inc.

% Requires deep learning toolbox.
vision.internal.requiresNeuralToolbox(mfilename);
vision.internal.cnn.WarningLogger.initialize();

% Validate network, trainingData. If detector is passed as input get the
% lgraph from the detector.
[trainingData, lgraph, params, options] = iParseInputsYolov2(...
    options,trainingData,detector, mfilename, varargin{:});
% Get anchor box data from the network
if ~isa(detector,'yolov2ObjectDetector')
    % If input is layerGraph(backward compatibility)
    yolov2OutIdx = find(arrayfun(@(x)isa(x,'nnet.cnn.layer.YOLOv2OutputLayer'),...
        lgraph.Layers),1,'first');
    params.AnchorBoxes = lgraph.Layers(yolov2OutIdx).AnchorBoxes;
    params.LossFactors = lgraph.Layers(yolov2OutIdx).LossFactors;

    yolov2ReorgIdx = find(arrayfun(@(x)isa(x,'nnet.cnn.layer.YOLOv2ReorgLayer'),...
         lgraph.Layers),1,'first');

    if isempty(yolov2ReorgIdx)
        yolov2ReorgIdx = find(arrayfun(@(x)isa(x,'nnet.cnn.layer.SpaceToDepthLayer'),...
            lgraph.Layers),1,'first');
    end

    if ~isempty(yolov2ReorgIdx)
        yolov2ReorgLayer = lgraph.Layers(yolov2ReorgIdx);
        dagNetAnalysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(lgraph);
        yolov2ReorgIdx = strcmp({dagNetAnalysis.ExternalLayers.Name},yolov2ReorgLayer.Name);
        yolov2ReorgLayerSources = dagNetAnalysis.LayerAnalyzers(yolov2ReorgIdx);
        params.ReorganizeLayerSource = yolov2ReorgLayerSources.Inputs.Source{1};
    else
        params.ReorganizeLayerSource = '';
    end
    params.ModelName = char(params.ClassNames(1));
else
    params.AnchorBoxes = detector.AnchorBoxes;
    params.LossFactors = detector.LossFactors;
    params.ReorganizeLayerSource = detector.ReorganizeLayerSource;
    params.ModelName = detector.ModelName;
end

% Configure printer for verbose printing
printer = vision.internal.MessagePrinter.configure(options.Verbose);

% Create the YOLO v2 datastore.
[ds, options] = iSetupDatastore(trainingData, lgraph, params, options);

% Mapping Datastore to MIMO network.
if params.TrainingDataWasTable
    mapping = yolov2ObjectDetector.createMIMODatastoreMapping(ds, lgraph, params);
else
    mapping = yolov2ObjectDetector.createMIMODatastoreCellMapping(params.InputSize);
end

% Setup the checkpoint saver.
checkpointSaver = iSetupCheckpointSaver(params, options);

% Train the network.
[net, info] = iTrainYOLOv2(ds, lgraph, params, mapping, options, checkpointSaver);
iWarnIfTrainingLossHasNaNs(info)
trainedDetector = yolov2ObjectDetector.assembleDetector(params, net);

yolov2ObjectDetector.printFooter(printer);
end

%--------------------------------------------------------------------------
% Parse and validate the input network and trainingData.
%--------------------------------------------------------------------------
function [trainingData, lgraph, params, options] = iParseInputsYolov2(options, trainingData,detector, fname, varargin)
% Validate Network.
vision.internal.cnn.validation.checkNetwork(detector, fname, ...
    {'nnet.cnn.LayerGraph', 'yolov2ObjectDetector'});
%If input is a layerGraph(backward compatibility)
if isa(detector,'yolov2ObjectDetector')
    params.AnchorBoxes = detector.AnchorBoxes;
    params.ClassNames  = detector.ClassNames;
    params.TrainingImageSize = detector.TrainingImageSize;
    iValidateYOLOv2Network(params,detector.Network);
    lgraph = layerGraph(detector.Network);
    yolov2Output = yolov2OutputLayer(detector.AnchorBoxes,'Name',...
                                     'yolov2Out','LossFactors',detector.LossFactors,'Classes',detector.ClassNames);
    lgraph = addLayers(lgraph,yolov2Output);
    yolov2TransformIdx = find(arrayfun(@(x)isa(x,'nnet.cnn.layer.YOLOv2TransformLayer'),...
        lgraph.Layers),1,'first');
    lgraph = connectLayers(lgraph,lgraph.Layers(yolov2TransformIdx).Name,'yolov2Out');

    if any(strcmp(varargin,'TrainingImageSize'))||any(strcmp(varargin,"TrainingImageSize"))
        warning(message('vision:yolo:discourageTrainingImageSize'));
    end
else
    lgraph = detector;
end

params.TrainingDataWasTable = istable(trainingData);

if params.TrainingDataWasTable
    params.ClassNames = trainingData.Properties.VariableNames(2:end);
    params.numClasses = size(trainingData,2)-1;
else
    if options.DispatchInBackground
        error(message('vision:ObjectDetector:backgroundUnsupportedWithDatastore'));
    end
    % Copy and reset the given datastore, so external state events are
    % not reflected.
    trainingData = copy(trainingData);
    reset(trainingData);
    params.ClassNames = vision.internal.inputValidation.checkGroundTruthDatastore(trainingData);
    params.numClasses = numel(params.ClassNames);
end

% Check user created LayerGraph to be valid network.
iValidateNetwork(params.numClasses,lgraph);
params.inputImageIdx = yolov2ObjectDetector.findYOLOv2ImageInputLayer(lgraph.Layers);
params.InputSize = lgraph.Layers(params.inputImageIdx,1).InputSize;
params.yoloOutputLayerIdx = find(...
                    arrayfun( @(x)isa(x,'nnet.cnn.layer.YOLOv2OutputLayer'), ...
                    lgraph.Layers));
if(~isfield(params,'AnchorBoxes'))
    params.AnchorBoxes = lgraph.Layers(params.yoloOutputLayerIdx).AnchorBoxes;
end
networkClassNames = lgraph.Layers(params.yoloOutputLayerIdx,1).Classes;
%Preprocessing to update classNames of yolov2OutputLayer when classes NVP is set to default.
if ischar(networkClassNames) 
    lgraph = yolov2ObjectDetector.updateNetworkClasses(lgraph, params.ClassNames); 
    networkClassNames = lgraph.Layers(params.yoloOutputLayerIdx,1).Classes;
end

networkClassNames = cellstr(networkClassNames);


if ~all(strcmp(params.ClassNames',networkClassNames))
    if isa(detector,'yolov2ObjectDetector')
        error(message('vision:yolo:trainingDataDetectorClassesMismatch'));
    else
        error(message('vision:yolo:outputLayerClassesMismatch'));
    end
end

% Parse optional parameters.
p = inputParser;
%TrainingImageSize are of dimensions M x 2.
p.addParameter('TrainingImageSize', params.InputSize(1:2));
p.addParameter('ExperimentMonitor', 'none');
p.addParameter('UseParallel', vision.internal.useParallelPreference());
parse(p, varargin{:});
userInput = p.Results;
useParallel = vision.internal.inputValidation.validateUseParallel(userInput.UseParallel);
if ~isa(detector,'yolov2ObjectDetector')
params.TrainingImageSize = userInput.TrainingImageSize;
end
params.UseParallel = useParallel;

params.Monitor = vision.internal.cnn.validation.checkExperimentMonitor(p, userInput, options, mfilename);

% Validate training options.
iCheckTrainingOptions(options,params.TrainingDataWasTable);

% Configure printer for verbose printing
printer = vision.internal.MessagePrinter.configure(options.Verbose);
yolov2ObjectDetector.printHeader(printer, params.ClassNames);

% Validate groundTruth.
if params.TrainingDataWasTable
    printer.printMessageNoReturn('vision:yolo:checkTrainingDataBegin');
    [trainingData, validTrainingData] = iCheckGroundTruth(trainingData, useParallel);

    rcnnObjectDetector.issueWarningIfRequired(validTrainingData)
    printer.printMessage('vision:yolo:checkTrainingDataEnd');
else
    % Apply a transform to validate images and boxes.
    transformFcn = @(data)vision.internal.cnn.yolo.validateImagesAndBoxesTransform(data,params.InputSize);
    applyTransformFcn = @(ds)transform(ds, transformFcn);
    trainingData = applyTransformFcn(trainingData);

    % Apply the same transform to ValidationData, if specified.
    options = vision.internal.cnn.validationReportUtils.updateValidationDataTransform(options,applyTransformFcn);
end

% Validate TrainingImageSize sizes.
iCheckTrainingImageSize(params.TrainingImageSize,lgraph,params);
end

%--------------------------------------------------------------------------
% Validate training options (validation data and plot not supported).
%--------------------------------------------------------------------------
function iCheckTrainingOptions(option,isTrainingDataTable)
allowValidationDatastore = true;
vision.internal.cnn.validation.checkTrainingOptions(option, mfilename,allowValidationDatastore,isTrainingDataTable);
end

%--------------------------------------------------------------------------
function iCheckTrainingImageSize(trainingImageSize,lgraph,params)
%imageInputSize = lgraph.Layers(params.inputImageIdx).InputSize;
imageInputSize = params.InputSize;
validateattributes(trainingImageSize, {'numeric'}, ...
    {'2d','ncols',2,'ndims',2,'nonempty','nonsparse',...
    'real','finite','integer','positive'});
if any(trainingImageSize < imageInputSize(1:2),'all')
    error(message('vision:yolo:multiScaleInputError'))
end

% Check compatibility of each TrainingImageSize with reorg layer.
for i=1:size(trainingImageSize,1)
    
    % Set the size of the input layer to TrainingImageSize size in lgraph.
    imageInput =  imageInputLayer([trainingImageSize(i,:),imageInputSize(3)],'Normalization','none',...
        'Name',lgraph.Layers(params.inputImageIdx).Name);
    lgraph = replaceLayer(lgraph,lgraph.Layers(params.inputImageIdx).Name,...
        imageInput);
    analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(lgraph);
    reorgIdx = find(arrayfun(@(x) (isequal(class(x.ExternalLayer),'nnet.cnn.layer.YOLOv2ReorgLayer')||...
        isequal(class(x.ExternalLayer),'nnet.cnn.layer.SpaceToDepthLayer')),...
        analysis.LayerAnalyzers));
    
    % In case of multiple reorg layer iterate over every reorg layer.
    for r = 1:size(reorgIdx,1)
        reorgOutSize   = analysis.LayerAnalyzers(reorgIdx(r,1)).Outputs.Size{1};
        reorgOutLayerName = strsplit(analysis.LayerAnalyzers(reorgIdx(r,1)).Outputs.Destination{1},'/');
        getReorgOutSizes = analysis.LayerAnalyzers(arrayfun(@(x)strcmp(x.Name,reorgOutLayerName(1)),analysis.LayerAnalyzers)).Inputs.Size;
        for n = 1:size(getReorgOutSizes,1)
            if getReorgOutSizes{n,1}(1) ~= reorgOutSize(1) && getReorgOutSizes{n,1}(2) ~= reorgOutSize(2)
                expected = getReorgOutSizes{n,1:2};
                error(message('vision:yolo:multiScaleUnsupported', ...
                    mat2str(expected(1:2)), mat2str(trainingImageSize(i,:))))
            end
        end
    end
end
end
%--------------------------------------------------------------------------
function [trainingData, validTrainingData] = iCheckGroundTruth(trainingData, useParallel)
vision.internal.cnn.validation.checkGroundTruth(trainingData, mfilename);

if useParallel
    numfiles = size(trainingData,1);
    isvalid(numfiles) = struct('HasValidData',[],'HasNoBoxes',[]);
    parfor i = 1:numfiles
        row = trainingData(i,:);
        I = imread(string(row{1,1}));
        imageSize = size(I);
        [trainingData(i,:), isvalid(i).HasValidData, isvalid(i).HasNoBoxes] = vision.internal.cnn.utils.hasValidTrainingData(imageSize, trainingData(i,:));
    end
    hasNoBoxes = vertcat(isvalid(:).HasNoBoxes);
    validTrainingData = ~hasNoBoxes & vertcat(isvalid(:).HasValidData);
else    
    imgNames = rowfun(@string,trainingData(:,1));
    imgArray = rowfun(@(i) {imread(i)},imgNames(:,1));
    imgSizes = rowfun(@(img) size(img{:}),imgArray(:,1));
    [trainingData, hasValidData, hasNoBoxes] = iHasValidTrainingData(imgSizes, trainingData);  
    validTrainingData = ~hasNoBoxes & hasValidData;
end


% Remove rows with no data.
trainingData(hasNoBoxes,:) = [];

if isempty(trainingData)
    error(message('vision:rcnn:noValidTrainingData'));
end
end

%--------------------------------------------------------------------------
function iValidateNetwork(numClasses,yolov2LayerGraph)
analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(yolov2LayerGraph);
constraint = vision.internal.cnn.analyzer.constraints.YOLOv2Architecture(numClasses);
out = nnet.internal.cnn.analyzer.constraints.Constraint.getBuiltInConstraints();
archConstraint = arrayfun(@(x)isa(x,'nnet.internal.cnn.analyzer.constraints.Architecture'),out);
out(archConstraint) = constraint;
analysis.applyConstraints(out);
try
    analysis.throwIssuesIfAny();
catch ME
    throwAsCaller(ME);
end
end
%--------------------------------------------------------------------------
function iValidateYOLOv2Network(params,network)
    layerNames = string({network.Layers.Name});
    % Verify only one transformLayer exists
    layers = network.Layers;
    idx = find(arrayfun(@(x)isa(x,'nnet.cnn.layer.YOLOv2TransformLayer'),layers));
    if numel(idx) == 0
        error(message("vision:yolo:mustHaveTransformLayer"));
    elseif numel(idx) > 1
        error(message("vision:yolo:mustHaveOnlyOneTransformLayer"));
    else
        % No error.
    end 

    %Verify that YOLOv2 has only one image input layer.
    inputLayersIdx = find(arrayfun(@(x)isa(x,'nnet.cnn.layer.ImageInputLayer'),layers));
    if numel(inputLayersIdx) == 0
           error(message("vision:yolo:mustHaveInputLayer"));
    elseif numel(inputLayersIdx) > 1
           error(message("vision:yolo:mustHaveOnlyOneInputLayer"));
    else
        if ~isa(layers(inputLayersIdx),'nnet.cnn.layer.ImageInputLayer')
            error(message("vision:yolo:mustBeImageInputLayer"));
        end
    end
    %Compute the number of filters of last convolution layer
    numAnchors = size(params.AnchorBoxes,1);
    numClasses = size(params.ClassNames,1);
    numPredictionsPerAnchor = 5 + numClasses;
    expectedFilters = numAnchors*numPredictionsPerAnchor;
    layerIdx = find(strcmp(network.OutputNames{1,1},layerNames));
    idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.Convolution2DLayer'),network.Layers(1:layerIdx));
    filterIdx = find(idx,1,'last');
    actualFilters = network.Layers(filterIdx,1).NumFilters;
    
    if ~(expectedFilters == actualFilters)
        error(message('vision:yolo:invalidNumFilters',mat2str(expectedFilters),...
            mat2str(numAnchors),mat2str(numClasses)));
    end

end
%--------------------------------------------------------------------------
function checkpointSaver = iSetupCheckpointSaver(params, options)
    saver = nnet.internal.cnn.trainNetwork.reporter.checkpoint.Saver(options.CheckpointPath, options.CheckpointFrequency);
    if strcmp(options.CheckpointFrequencyUnit, "epoch")
        checkpointSaver = vision.internal.cnn.DetectorCheckpointEpochSaver(saver);
    else
        checkpointSaver = vision.internal.cnn.DetectorCheckpointIterationSaver(saver);
    end
    checkpointSaver.Detector =  params;
    checkpointSaver.DetectorFcn = @(net,dd)yolov2ObjectDetector.assembleDetector(dd,net);
    checkpointSaver.CheckpointPrefix = 'yolov2';
end

%--------------------------------------------------------------------------
% Training function for YOLO v2 object detector.
%--------------------------------------------------------------------------
function [yolov2Net, info] = iTrainYOLOv2(ds, lgraph, params, mapping, opts, checkpointSaver)

% Set execution settings required by MIMOtrainer.
precision = nnet.internal.cnn.util.Precision('single');
executionSettings = nnet.internal.cnn.assembler.setupExecutionEnvironment( opts, ds, precision );

if params.TrainingDataWasTable
    batchFcnStruct.Response = @iWrapResponseInCellFcn;
else
    batchFcnStruct = iProvideCollateFunctions();
end


axesConfigFactory = vision.internal.cnn.YOLOAxesConfig();
axesConfig = axesConfigFactory.AxesConfiguration;

validationSpecified = vision.internal.cnn.validationReportUtils.isValidationSpecified(opts);
if validationSpecified 
    columnStrategy = nnet.internal.cnn.util.RegressionValidationColumns;
    infoContent = nnet.internal.cnn.util.traininginfo.RegressionWithValidationContent();
    validationPredictStrategyFcn = @vision.internal.cnn.ValidationPredictDatastoreStrategy;
else
    infoContent = nnet.internal.cnn.util.traininginfo.RegressionContent();
    columnStrategy = nnet.internal.cnn.util.RegressionColumns();
    validationPredictStrategyFcn = @(varargin)[];
end

summaryBuilderFcn = @vision.internal.cnn.YOLOv2Summary.makeSummary;
summaryMonitorInfo.Monitor = params.Monitor;
summaryMonitorInfo.SummaryBuilderFcn = summaryBuilderFcn;
summaryMonitorInfo.MonitorFieldInformation = vision.internal.cnn.YOLOv2Summary.monitorSummaryFields(validationSpecified);
 
[yolov2Net, info] = vision.internal.cnn.trainNetwork( ...
    ds, lgraph, opts, executionSettings, mapping, checkpointSaver, ...
    summaryMonitorInfo, ...
    infoContent, ...
    columnStrategy, ...
    axesConfig, ...
    batchFcnStruct, ...
    validationPredictStrategyFcn,...
    iInfoToSummaryMapAndStartValues());
end

%--------------------------------------------------------------------------
function m = iInfoToSummaryMapAndStartValues()
% Full info content resides in the validation content. Use that to build
% the map from info to summary fields. All fields are initialized to [].
% This is used for OutputFcn support.
infoContent = nnet.internal.cnn.util.traininginfo.RegressionWithValidationContent();
initialValue = repelem({[]},numel(infoContent.FieldNames),1);
m = [infoContent.FieldNames' infoContent.SummaryNames' initialValue];
end

%--------------------------------------------------------------------------
function [ds,options] = iSetupDatastore(trainingData, lgraph, params, options)
dsOpts.MiniBatchSize = options.MiniBatchSize;
dsOpts.DispatchInBackground = options.DispatchInBackground;
dsOpts.DatastoreOutSize = lgraph.Layers(params.inputImageIdx).InputSize;
dsOpts.TrainingImageSize = params.TrainingImageSize;
if params.TrainingDataWasTable
    ds = yolov2ObjectDetector.createYoloTrainingDatastore(trainingData,dsOpts);
else
    applyTransformFcn = @(ds)vision.internal.cnn.yolo.createYOLOV2TransformedDatastore(ds,dsOpts);
    ds = applyTransformFcn(trainingData);
    options = vision.internal.cnn.validationReportUtils.updateValidationDataTransform(options,...
        applyTransformFcn);
end
end

%--------------------------------------------------------------------------
function iWarnIfTrainingLossHasNaNs(info)
msg = message('vision:yolo:networkTrainingLossHasNaNs');
vision.internal.cnn.warnIfLossHasNaNs(info, msg);
end

%--------------------------------------------------------------------------
function T = iWrapResponseInCellFcn(T)
T = {T};
end

%--------------------------------------------------------------------------
function collateFcns = iProvideCollateFunctions()
% Define CollateFunctions for GeneralDatastoreDispatcher.
collateFcns.InputFunctions  = [];
collateFcns.OutputFunctions = {@(x,~)iWrapResponseInCellFcn(x)};
collateFcns.CatAndSliceStrategy = vision.internal.cnn.catAndSliceStrategy.YOLOv2CatAndSliceStrategy();
end
% -------------------------------------------------------------------------
function [cleanBoxes, noBoxesRemovedFlag, hasNoBoxesFlag] = iHasValidTrainingData(sizes, trainingData)
% Scans training data and removes invalid boxes,
% returning a clean row of boxes.
trainingData = [trainingData sizes];
cleanBoxes = rowfun(@(varargin)nSanitizeTrainingDataRow(varargin{:}),...
    trainingData,...
    'OutputFormat','table', ...
    'ExtractCellContents', true, ...
    'NumOutputs', width(trainingData), ...
    'OutputVariableNames', trainingData.Properties.VariableNames);

% remove noBoxesRemoved flag from cleanBoxes table
noBoxesRemovedFlag = table2array(cleanBoxes(:,end));
cleanBoxes(:,end) = [];

% combine boxes from every class for each training data row 
allBoxes = rowfun(@(varargin){vertcat(varargin{:})},...
    cleanBoxes(:,2:end),'OutputFormat','table',...
    'ExtractCellContents',true);

hasNoBoxesFlag = cellfun(@isempty,table2cell(allBoxes));

function varargout = nSanitizeTrainingDataRow(varargin)
   % Iterate through boxes and remove invalid ones. Ground
   % truth boxes with fractional values are rounded to the
   % nearest pixel center.
   noBoxesRemoved = true;
   varargout    = cell(1,size(varargin,2)); 
   varargout(1) = {varargin(1)};

   for k = 2:numel(varargin)-1
       boxes = varargin{k};
       s = varargin{end};
       boxes = vision.internal.cnn.utils.hasValidTrainingBoxes(s, boxes);
       varargout{k}  = {boxes};
       noBoxesRemoved = noBoxesRemoved && size(varargout{k}{1},1) == size(varargin{k},1);
   end
   varargout(end) = {noBoxesRemoved};   
end
end
