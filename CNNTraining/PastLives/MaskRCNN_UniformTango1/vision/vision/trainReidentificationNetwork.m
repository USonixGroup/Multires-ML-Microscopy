function [network, info] = trainReidentificationNetwork(trainingData, network, options, params)
%

%  Copyright 2023 The MathWorks, Inc.

    arguments
        trainingData {mustBeScalarOrEmpty}
        network (1,1) {iValidateNetwork}
        options (1,1) nnet.cnn.TrainingOptions
        params.LossFunction {mustBeTextScalar, mustBeMember(params.LossFunction, ["cross-entropy", "cosine-softmax", "additive-margin-softmax"])} = "additive-margin-softmax";
        params.FreezeBackbone {mustBeScalarOrEmpty, mustBeNonempty, mustBeNumericOrLogical} = true;
        params.ExperimentMonitor {mustBeScalarOrEmpty, iValidateMonitor} = [];
        params.Margin {mustBeNumeric, mustBeFinite, mustBeNonnegative, mustBeReal, mustBeScalarOrEmpty, mustBeNonempty, mustBeLessThanOrEqual(params.Margin,1)} = 0.35;
        params.Scale {mustBeNumeric, mustBeFinite, mustBePositive, mustBeReal, mustBeScalarOrEmpty, mustBeNonempty} = 30;
    end

    vision.internal.requiresNeuralToolbox(mfilename);

    % Transform training datastore if the input DS is a valid imageDatastore.
    if isa(trainingData,"matlab.io.datastore.ImageDatastore")
        iValidateImageDatastore(trainingData);
        trainingDS = transform(trainingData, ...
            @(x,info) iTransformImageDatastore(x,info), IncludeInfo=true);
    else
        trainingDS = trainingData;
    end

    % Verify input training datastore.
    iValidateTrainingdata(trainingDS);

    % Manage validation data.
    if ~isempty(options.ValidationData)
        if isa(options.ValidationData,"matlab.io.datastore.ImageDatastore")
            iValidateImageDatastore(options.ValidationData);
            validationDS = transform(options.ValidationData, ...
                @(x,info) iTransformImageDatastore(x,info), IncludeInfo=true);
        else
            validationDS = options.ValidationData;
        end

        iValidateTrainingdata(validationDS);

        % Add preprocessing to validation data.
        options.ValidationData = transform(validationDS,...
            @(x)reidentificationNetwork.preprocessInput(x,network.InputSize(1:2)));
    end

    % Validate classes in the trainingData.
    labelCategories = iGatherLabelCategories(trainingDS, network.ClassNames);

    % Validate if gpu execution on trainingOptions is supported.
    if strcmp(options.ExecutionEnvironment, 'gpu') && ~canUseGPU
        error(message('vision:reidentification:noGPUFound'));
    end

    % Configure network freezing before training.
    network = configureForTraining(network,params.FreezeBackbone,params.LossFunction);

    % Create a minibatchqueue.
    resize = "scale";
    network = configureImageInputResizeMethod(network,resize);
    preprocessDS = transform(trainingDS, ...
        @(x)reidentificationNetwork.preprocessInput(x,network.InputSize(1:2),resize));

    % Manage input normalization from training options.
    if options.ResetInputNormalization
        disp('Computing Input Normalization Statistics.')

        normQueue = minibatchqueue(preprocessDS,1,...
            OutputAsDlarray=true,...
            PartialMiniBatch="return",...
            MiniBatchFormat='SSCB',...
            MiniBatchFcn = @batchForNormalization,...
            MiniBatchSize=options.MiniBatchSize);

        options.ResetInputNormalization = false;
        stats = images.dltrain.internal.computeNormalizationStats(normQueue,1,options.ExecutionEnvironment,NormalizationDimension="channel");
        network = setInputNormalization(network,stats);
    end

    % Set output environment.
    if options.ExecutionEnvironment == "multi-gpu" 
        execEnvironment = "gpu";
    elseif options.ExecutionEnvironment == "parallel"
        execEnvironment = "auto";
    else
        execEnvironment = options.ExecutionEnvironment;
    end

    % Create the minibatch dispatcher.
    miniBatchSize = options.MiniBatchSize;
    miniBatchFcn = @(img,labels) deal(cat(4,img{:}),labels);

    mbqTrain = minibatchqueue(preprocessDS,2, ...
        "MiniBatchFormat",["SSCB",""], ...
        "MiniBatchSize",miniBatchSize, ...
        "OutputCast",["single",""], ...
        "OutputAsDlArray",[true,false], ...
        "MiniBatchFcn",miniBatchFcn, ...
        "OutputEnvironment",[execEnvironment,"cpu"]);

    lossFcn = vision.internal.reidentification.ReidentificationLoss(...
        network.NumClasses,labelCategories,params.LossFunction,params.Margin,params.Scale);
    metrics = {images.dltrain.internal.LossMetricAdapter(lossFcn)};

    options.BatchNormalizationStatistics = 'moving';

    [network,info] = iTrainNetwork(mbqTrain,network,options,lossFcn,metrics,params.ExperimentMonitor);

    % Remove fields from info struct.
    fieldsToRemove = {'OutputNetworkIteration'};
    info = rmfield(info, fieldsToRemove);
end

function out = batchForNormalization(x)
    out = cat(4,x{:});
end

%--------------------------------------------------------------------------
% Input validators
%--------------------------------------------------------------------------
function iValidateImageDatastore(trainingData)
    if isempty(trainingData.Labels)
        error(message("vision:reidentification:invalidImageDatastore"));
    end

    if length(trainingData.Labels) ~= length(trainingData.Files)
        error(message("vision:reidentification:invalidNumberOfLabels"));
    end
end

%--------------------------------------------------------------------------
function tf = iValidateTrainingdata(trainData)

    if ~matlab.io.datastore.internal.shim.isDatastore(trainData)
        error(message('vision:reidentification:invalidDatastore'));
    end

    data = preview(trainData);
    % Data is a cell array of size 1x2.
    validateattributes(data,{'cell'}, {'size', [1 2]}, mfilename, 'trainingData',1);

    % Validate training images.
    validateattributes(data{1},{'numeric'}, {'size', [NaN NaN 3]});

    % Validate labels.
    numObjects = size(data,1);
    validateattributes(data{2},{'categorical'}, {'numel', numObjects});

    tf = true;
end

%--------------------------------------------------------------------------
function tf = iValidateNetwork(net)
    % Verify that the input network is a reidentification network.
    if ~isa(net,'reidentificationNetwork')
        error(message('vision:reidentification:invalidReIDNetwork'));
    end
    tf = true;
end

%--------------------------------------------------------------------------
function iValidateMonitor(monitor)
    if isempty(monitor)
        return;
    end
    validateattributes(monitor,{'experiments.Monitor'}, ...
        {'scalar'},"trainReidentificationNetwork","ExperimentMonitor");
    if ~isempty(monitor.Metrics)
        error(message('vision:ObjectDetector:nonEmptyMetricsInfoInExperimentMonitor'));
    end
end

%--------------------------------------------------------------------------
% Helper Functions
%--------------------------------------------------------------------------
function [data, info] = iTransformImageDatastore(img,info)
    data = {img,info.Label};
end

%--------------------------------------------------------------------------
function [network, info] = iTrainNetwork(mbqTrain, network, options, lossFcn, metrics, experimentMonitor)
    % Train the reidentification network.
    [network,info] = images.dltrain.internal.dltrain(mbqTrain,network,options,lossFcn,metrics,'Loss',ExperimentMonitor=experimentMonitor);

    % Update the reidentificiation network
    network = constructReidentificationNetwork(network);
end

%--------------------------------------------------------------------------
function labelCategories = iGatherLabelCategories(trainingData, classNames)

    % Obtain all label data in the training data.
    trainingDataCopy = copy(trainingData);
    data = readall(trainingDataCopy);
    labels = data(:,2);

    % Verify that all images have a label.
    emptyLabels = cellfun(@isempty,labels);
    if any(emptyLabels)
        error(message("vision:reidentification:invalidNumberOfLabels"));
    end

    % Verify that the label categories match the classes in the
    % reidentification network.
    labels = vertcat(labels{:});
    labelCategories = categories(labels);
    if ~isequal(sort(categories(labels))', sort(classNames))
        error(message('vision:reidentification:invalidLabelCategories'));
    end

end
