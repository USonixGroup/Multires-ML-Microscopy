function [trainedKeypointDetector, info] = trainHRNetObjectKeypointDetector(trainingData, keypointDetector, options, params)
%

% Copyright 2023 The MathWorks, Inc.

arguments
    trainingData {mustBeScalarOrEmpty}
    keypointDetector hrnetObjectKeypointDetector
    options nnet.cnn.TrainingOptions
    params.ExperimentMonitor {mustBeScalarOrEmpty, iValidateMonitor} = []
end
% Requires deep learning toolbox.
vision.internal.requiresNeuralToolbox(mfilename);
vision.internal.cnn.WarningLogger.initialize();

% Check that the solver is supported.
assertMiniBatchSolver(options, mfilename)

validateDataFormatsAndMetricsAreEmpty(options, mfilename);

% Validate training data.
iValidateData(keypointDetector, trainingData);

% Validate if gpu execution on trainingOptions is supported
if(strcmp(options.ExecutionEnvironment, 'gpu') && ~canUseGPU)
    error(message('vision:hrnetObjectKeypoint:noGPUFound'));
end

% Set output environment.
if options.ExecutionEnvironment == "multi-gpu" 
    outputEnv = "gpu";
elseif options.ExecutionEnvironment == "parallel"
    outputEnv = "auto";
else
    outputEnv = options.ExecutionEnvironment;
end
% Number of outputs from the MiniBatchFcn.
numOutputs = 2;

% Define outputcast for each output from minibatch. 
outputCast(1) = "single";
outputCast(2)="";

% Define minibatch format for each output from minibatch.
miniBatchFormat(1) = "SSCB";
miniBatchFormat(2) = "SSCB";

inputSize = keypointDetector.InputSize;
outputSize = [inputSize(1)/4 inputSize(2)/4];
numKeypoints = size(keypointDetector.KeyPointClasses,1);

% Manage input normalization from training options
if options.ResetInputNormalization
    printer1 = vision.internal.MessagePrinter.configure(true);   
    printer1.printMessage("vision:hrnetObjectKeypoint:computingInputNormalization");
    printer1.linebreak;
    normQueue = minibatchqueue(trainingData,1,...
        OutputAsDlarray=true,...
        PartialMiniBatch="return",...
        MiniBatchFormat='SSCB',...
        MiniBatchFcn = @(images, keypoints,boundingBox)hrnetObjectKeypointDetector.preprocessHRNetForTraining(images,keypoints,boundingBox,inputSize, outputSize,numKeypoints ),...
        MiniBatchSize=options.MiniBatchSize);

    options.ResetInputNormalization = false;
    stats = images.dltrain.internal.computeNormalizationStats(normQueue,1,options.ExecutionEnvironment,NormalizationDimension="channel");
    keypointDetector = setInputNormalization(keypointDetector,stats);
end

% Update the validation datastore.
if ~isempty(options.ValidationData)
    iValidateData(keypointDetector,options.ValidationData);
end

% Define the minibatchqueue for training.
mbq = minibatchqueue(...
    trainingData,...
    numOutputs,...
    "MiniBatchSize",options.MiniBatchSize,...
    "OutputEnvironment",outputEnv,...
    "MiniBatchFcn",@(images, keypoints,boundingBox)hrnetObjectKeypointDetector.preprocessHRNetForTraining(images,keypoints,boundingBox,inputSize, outputSize,numKeypoints ),...
    "MiniBatchFormat",miniBatchFormat,...
    "OutputCast", outputCast,...
    "DispatchInBackground",options.DispatchInBackground);

lossFcn = @(YPred1,YTarg1)iCalculateLoss(YPred1,YTarg1);


% A scalar where each field is a metric to be tracked and each value
% is a function of the same calling syntax as the loss:
metrics = {images.dltrain.internal.FunctionMetric(lossFcn,"Loss")};


% For purposes of ValidationPatience early stopping, define which 
% of the metrics from the metrics set will be used for purposes 
% of early stopping.
validationPatienceMetric = 'Loss';

% Configure printer for verbose printing.
printer = vision.internal.MessagePrinter.configure(options.Verbose);
printHeader(printer, keypointDetector.KeyPointClasses);

% The training function to train the input detector. Returns the trained detector.
[trainedKeypointDetector,infoTrain] = images.dltrain.internal.dltrain(mbq,keypointDetector,options,lossFcn, ...
    metrics,validationPatienceMetric,'ExperimentMonitor',params.ExperimentMonitor);

% Update info output.
info = iUpdateInfo(infoTrain,options);

printFooter(printer);
end
%--------------------------------------------------------------------------
function loss = iCalculateLoss(dlYPred,dlY)
miniBatchSize = size(dlYPred,4);
dlW = logical(dlY(:,:,:,miniBatchSize+1:end));
dlY = dlY(:,:,:,1:miniBatchSize);
outputSize = size(dlYPred,[1,2]);
dlY = reshape(dlY.*dlW,size(dlY,1),size(dlY,2),[]);
dlY = dlarray(dlY,"SSB");
dlYPred = reshape(dlYPred.*dlW,size(dlYPred,1),size(dlYPred,2),[]);
dlYPred = dlarray(dlYPred,"SSB");
loss = mse(dlYPred,dlY);
loss = (loss*1./(outputSize(1)*outputSize(2)));
end
%--------------------------------------------------------------------------
function printHeader(printer, classNames)
    printer.print('*************************************************************************\n');
    printer.printMessage("vision:hrnetObjectKeypoint:trainingHeader");
    printer.linebreak;
    
    for i = 1:numel(classNames)
        printer.print('* %s\n', classNames{i});
    end
    
    printer.linebreak;
end
%------------------------------------------------------------------
function printFooter(printer)
printer.linebreak;    
printer.print('*************************************************************************\n');
printer.printMessage("vision:hrnetObjectKeypoint:trainingFooter");
printer.print('*************************************************************************\n');
printer.linebreak;
end
%--------------------------------------------------------------------------
function info = iUpdateInfo(infoTrain,options)
% Update training losses for each iteration.
trainingLoss = [infoTrain.TrainingLoss];

% Update validation losses for each iteration.
if ~isempty(options.ValidationData)
    validationLoss = [infoTrain.ValidationLoss];
end

% Update learning rate for each iteration.
baseLearnRate = [infoTrain.LearnRate];

indx = [infoTrain.OutputNetworkIteration] == 1;
OutputNetworkIteration = infoTrain([infoTrain.OutputNetworkIteration] == 1).Iteration;

% Update info output.
info.TrainingLoss = trainingLoss;
if ~isempty(options.ValidationData)
    info.ValidationLoss = validationLoss;
end
info.BaseLearnRate = baseLearnRate;
if ~isempty(options.ValidationData)
    info.FinalValidationLoss = validationLoss(indx);
end
info.OutputNetworkIteration = OutputNetworkIteration;
end
%--------------------------------------------------------------------------
function iValidateMonitor(monitor)
    if(isempty(monitor))
        return;
    end

    if ~isa(monitor,'experiments.Monitor')
        error(message('vision:ObjectDetector:unexpectedTypeForExperimentMonitor'))
    end

    if ~isempty(monitor.Metrics)
        error(message('vision:ObjectDetector:nonEmptyMetricsInfoInExperimentMonitor'));
    end
end

% ------ Validation functions ---------------------

function iValidateData(detector,trainData)

    if(~matlab.io.datastore.internal.shim.isDatastore(trainData))
        error(message('vision:hrnetObjectKeypoint:invalidDatastore'));
    end

    data = preview(trainData);
    % data is a cell array of size 1x4
    validateattributes(data,{'cell'}, {'size', [1 4]}, mfilename, 'trainingData',1);

    % validate training images
    validateattributes(data{1},{'numeric'}, {'finite','real','size', [NaN NaN detector.InputSize(3)]});
        
    % validate keypoints
    validateattributes(data{2},{'table'},{'size',[1 1]})
    validateattributes(data{2}{:,:}{:},{'numeric'},{'finite','real','nonzero','positive'})
    
    if ~(size(data{2}{:,:}{:},2)==2 || size(data{2}{:,:}{:},2)==3)
        error(message('vision:hrnetObjectKeypoint:invalidKeypointSize'));
    end

    if size(data{2}{:,:}{:},2)==3 && (max(data{2}{:,:}{:}(:,3))>1 || min(data{2}{:,:}{:}(:,3))<0)
        error(message('vision:hrnetObjectKeypoint:invalidVisibilityFlag'));
    end
    
    if size(data{2}{:,:}{:},1)~= numel(detector.KeyPointClasses)
        error(message('vision:hrnetObjectKeypoint:invalidKeypointCount'));
    end

    %validate boxes
    validateattributes(data{3},{'numeric'},{'finite','real','nonzero','positive','integer','size', [NaN 4]});

    if any(data{3}<1)
        error(message('vision:hrnetObjectKeypoint:bboxesCanNotBeZero'));
    end

    if any(data{3}(:,3:4)==1)
        error(message('vision:hrnetObjectKeypoint:InvalidbboxesHeightWidth'));
    end

    if size(data{2}{:,:},1)~=size(data{3},1)
        error(message('vision:hrnetObjectKeypoint:NumOfbboxesAndKeypointMustBeSame'));
    end

    [height,width] = size(data{1});
    bboxes = data{3};
    if width<bboxes(3) || height<bboxes(4)
        error(message('vision:hrnetObjectKeypoint:invalidBBoxSize'));
    end
end