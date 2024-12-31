%trainSOLOV2 Train a SOLOV2 network to perform instance segmentation.
%
% Use of this function requires that you have the Deep Learning Toolbox(TM).
%
% trainedSolov2 = trainSOLOV2(trainingData, solov2Obj, options) trains a
% SOLO V2 instance segmentation network. A SOLO V2 network object can be
% trained to detect and segment multiple object classes.
%
% [..., info] = trainSOLOV2(...) additionally returns information on
% training progress such as training loss for each iteration. info is a struct array with each struct containing the
% following fields.
% 
%   LearnRate           - Learning rate at each iteration.
%   TrainingLoss        - Total Training loss at each iteration.
%
% The structure contains ValidationLoss, ValidationClsLoss and
% ValidationMaskLoss only when validation data is provided in trainingoptions.
%
% trainingData - This is a datastore with the following format.
%
%                Datastore format:
%                -----------------
%                A datastore that returns a cell array on the read
%                methods with four cells.
%                1st Cell  : A cell vector of images that can be grayscale,
%                            or a RGB image of size H-by-W-by-1 or H-by-W-by-3.
%                2nd Cell  : A M-by-4 matrix of [x, y, width, height]
%                            bounding boxes specifying object locations
%                            of M objects within each image.
%                3rd Cell  : A categorical vector of size M-by-1 containing the
%                            object class names. Note that all the categorical
%                            data returned by the datastore must have the same
%                            categories.
%                4th Cell  : A H-by-W-by-M logical array of M object masks.
%
%                An option is to use combine method to create a datastore to
%                return the above 4 columns of data on read:
%                     1. ds1, imageDatastore that can return the 1st column of data
%                     2. ds2, boxLabelDatastore that can return the 2nd and 3rd column of data.
%                     3. ds3, imageDatastore with a custom read function to return masks as 4th column.
%                     4. ds = combine(ds1, ds2, ds3);
%
% network        Specify the solov2 network object to train.
%
% options        Training options defined by the trainingOptions function
%                from Deep Learning Toolbox. The training options define
%                the training parameters of the neural network.
%
%
% Additional input arguments
% --------------------------
% [...] = trainSOLOV2(..., Name=Value) specifies additional name-value
% pair arguments described below:
%
%
%   "FreezeSubNetwork"      A string specifying the name of the
%                           sub-network(s), the weights of which will be
%                           frozen during training. The possible set of
%                           values are "backbone", "backboneAndNeck", and
%                           "none". "backbone" freezes the resnet50 feature
%                           extraction layers. "backboneAndNeck" freezes
%                           the backbone and additionally the path
%                           aggregation network used to mix backbone
%                           features at different scales.
%
%                           Default: "backbone"
%
%   "ExperimentMonitor"     solov2 training experiment monitoring,
%                           specified as an experiments.Monitor object for
%                           use with the Experiment Manager app. Use this
%                           object to track the progress of the training,
%                           update information fields in the results
%                           table, record values of the metrics used by
%                           the training, and produce training plots.
%              
%                           Default: experiments.Monitor.empty()
%

% Copyright 2023 The MathWorks, Inc.

function [network, info] = trainSOLOV2(trainingData, network, options, params)

    arguments
        trainingData {iValidateTrainingdata, mustBeScalarOrEmpty}
        network (1,1) {iValidateNetwork}
        options (1,1) nnet.cnn.TrainingOptions
        params.FreezeSubNetwork string {mustBeTextScalar, mustBeNonempty,mustBeMember(params.FreezeSubNetwork, ["backbone", "backboneAndNeck", "none"])} = "backboneAndNeck"
        params.ExperimentMonitor experiments.Monitor {mustBeScalarOrEmpty, iValidateMonitor} = experiments.Monitor.empty()

    end

    vision.internal.requiresNeuralToolbox(mfilename);

    % Validate classes in the trainingData
    iValidateLabelData(trainingData, network.ClassNames);

    % Validate if gpu execution on trainingOptions is supported
    if(strcmp(options.ExecutionEnvironment, 'gpu') && ~canUseGPU)
        error(message('vision:trainSOLOV2:noGPUFound'));
    end

    % Create minibatchqueue
    preprocessDS = transform(trainingData, @(x)solov2.preprocessInput(x,...
                                             network.InputSize(1:2),...
                                             network.NormalizationStatistics.Mean,...
                                             network.NormalizationStatistics.StandardDeviation, true));
    
    % Manage input normalization from training options
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
    
    % Manage Validation
    if(~isempty(options.ValidationData))
        iValidateTrainingdata(options.ValidationData);
        % Add preprocessing to validation data
        options.ValidationData = transform(options.ValidationData,...
                                           @(x)solov2.preprocessInput(x,...
                                             network.InputSize(1:2),...
                                             network.NormalizationStatistics.Mean,...
                                             network.NormalizationStatistics.StandardDeviation, true));
    end
    
    % Configure network freezing before training
    network = configureForTraining(network,params.FreezeSubNetwork);

    % Create minibatch dispatcher
    miniBatchSize = options.MiniBatchSize;
    
    % Set output environment.
    if options.ExecutionEnvironment == "multi-gpu" 
        execEnvironment = "gpu";
    elseif options.ExecutionEnvironment == "parallel"
        execEnvironment = "auto";
    else
        execEnvironment = options.ExecutionEnvironment;
    end

    miniBatchFcn = @(img,boxes,labels,masks) deal(cat(4,img{:}),boxes,labels,masks);

    mbqTrain = minibatchqueue(preprocessDS,4, ...
        "MiniBatchFormat",["SSCB","","",""], ...
        "MiniBatchSize",miniBatchSize, ...
        "OutputCast",["single","","",""], ...
        "OutputAsDlArray",[true,false,false,false], ...
        "MiniBatchFcn",miniBatchFcn, ...
        "OutputEnvironment",[execEnvironment,"cpu","cpu","cpu"]);
    
    lossFcn = vision.internal.solov2.SoloV2Loss(network.GridSizes, network.ObjSizeRanges, network.NumClasses);
    metrics = {images.dltrain.internal.LossMetricAdapter(lossFcn)};

    [network,info] = images.dltrain.internal.dltrain(mbqTrain,network,options,lossFcn,metrics,'Loss', 'ExperimentMonitor',params.ExperimentMonitor);

     % Remove fields from info struct
    fieldsToRemove = {'Epoch', 'Iteration', 'TimeElapsed', 'OutputNetworkIteration'};
    info = rmfield(info, fieldsToRemove);
end

function out = batchForNormalization(x)
    out = cat(4,x{:});
end

% ------ Validation functions -----------

function flag = iValidateTrainingdata(trainData)

    if(~matlab.io.datastore.internal.shim.isDatastore(trainData))
        error(message('vision:trainSOLOV2:invalidDatastore'));
    end

    data = preview(trainData);
    % data is a cell array of size 1x4
    validateattributes(data,{'cell'}, {'size', [1 4]}, mfilename, 'trainingData',1);

    % validate training images
    validateattributes(data{1},{'numeric'}, {'size', [NaN NaN 3]});

    %validate boxes
    validateattributes(data{2},{'numeric'}, {'size', [NaN 4]});

    numObjects = size(data{2},1);

    % validate labels
    validateattributes(data{3},{'categorical'}, {'column', 'numel', numObjects});

    % validate masks
    validateattributes(data{4},{'logical'}, {'size', [NaN NaN numObjects]});
    
    flag = true;
end

function flag = iValidateNetwork(net)
    if(~isa(net,'solov2'))
        error(message('vision:trainSOLOV2:invalidNetwork'));
    end
    flag = true;
end

function iValidateLabelData(trainingData, classNames)
    
    data = preview(trainingData);
    labels = data{3};
    if(~isequal(sort(categories(labels))', sort(classNames)))
        error(message('vision:trainSOLOV2:invalidLabelData'));
    end

end

function iValidateMonitor(monitor)

    if(isempty(monitor))
        return;
    end
    if ~isempty(monitor.Metrics)
        error(message('vision:ObjectDetector:nonEmptyMetricsInfoInExperimentMonitor'));
    end
end

