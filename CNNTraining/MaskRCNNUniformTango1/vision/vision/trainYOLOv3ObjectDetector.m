%trainYOLOv3ObjectDetector Train a YOLO v3 object detector.
%
%  This function requires the Deep Learning Toolbox.
%
%  detector = trainYOLOv3ObjectDetector(trainingData,detector,options)
%  trains a YOLO v3 deep learning object detector. A YOLO v3 object detector
%  can be trained to detect multiple object classes.
%
% [..., info] = trainYOLOv3ObjectDetector(...) additionally returns
% information on training progress such as training loss for each
% iteration. info is a struct with the following fields:
%
%   TrainingLoss        - Training loss at each iteration. This is the
%                         combination of the weighted losses based on the
%                         YOLO v3 loss factors.
%   ValidationLoss      - Validation loss at each iteration.
%   BaseLearnRate       - Learning rate at each iteration.
%   FinalValidationLoss - Final validation loss at the end of the training.
%
% The structure contains the fields ValidationLoss and FinalValidationLoss
% only when options specifies validation data.
%
%  Inputs
%  ------
%  trainingData         A datastore that returns a cell array or a
%                       table on the read methods with three columns:
%                       Column 1: A cell vector of the input image.
%                       Column 2: A cell vector that contains only M-by-4
%                                 matrices of [x y w h] axis-aligned 2D
%                                 bounding boxes or only M-by-5 matrices of
%                                 [xctr, yctr, w, h, yaw] rotated 2D bounding
%                                 boxes specifying object locations within
%                                 each image. For axis-aligned bounding boxes,
%                                 x and y represent the upper-left corner,
%                                 specified in pixels. For rotated rectangle
%                                 bounding boxes, xctr and yctr represent
%                                 the center coordinate, specified in spatial
%                                 coordinates, and yaw is the angle of
%                                 rotation in degrees. yaw is defined as
%                                 increasing positively in the clockwise
%                                 direction. A mixture of M-by-4 and M-by-5
%                                 bounding boxes is not valid.
%                       Column 3: A categorical cell vector where each
%                                 element is of size M-by-1 containing
%                                 the object class names. Note that all
%                                 the categorical data returned by the
%                                 datastore must have the same categories.
%
%                       Use combine method on two datastores to create a
%                       datastore to return the above 3 columns of data on
%                       read:
%                       1. ds1, imageDatastore that can return the 1st
%                       column of data.
%                       2. ds2, boxLabelDatastore that can return the
%                       2nd and 3rd column of data.
%                       3. ds = combine(ds1, ds2);
%
%  detector             A YOLO v3 object detector returned by the
%                       yolov3ObjectDetector function containing the network
%                       and other required parameters. See
%                       <a href="matlab:doc yolov3ObjectDetector">yolov3ObjectDetector documentation</a> for more details.
%
%  options              training options returned by the trainingOptions
%                       function in Deep Learning Toolbox. The training
%                       parameters of the neural network. See
%                       <a href="matlab:doc trainingOptions">trainingOptions documentation</a> for more details.
%
% [...] = trainYOLOv3ObjectDetector(..., Name=Value) specifies additional
% name-value pair arguments described below:
%
% 'ExperimentMonitor'     Detector training experiment monitoring,
%                         specified as an experiments. Monitor object for
%                         use with the Experiment Manager app. Use this
%                         object to track the progress of the training,
%                         update information fields in the results
%                         table, record values of the metrics used by
%                         the training, and produce training plots.
%
%                         The following information is monitored during
%                         training:
%                            - Training loss at each iteration.
%                            - Learning rate at each iteration.
%
%                         When validation data is provided in the training
%                         options input, additional information is
%                         monitored:
%                            - Validation loss at each iteration.
%
%                         The following metrics are computed after training
%                         ends:
%                            - Final validation loss at the end of the
%                              training.
%
%                         Default: 'none'
%
%
% 'FreezeSubNetwork'        A string specifying the name of the
%                           subnetworks, the weights of which will be
%                           frozen during training. The possible set of
%                           values are "backbone", "backboneAndNeck",
%                           "none".
%
%                           - "backbone"         Freezes the complete backbone.
%
%                           - "backboneAndNeck"  Freezes the backbone and the
%                                                path aggregation network used
%                                                to mix backbone features
%                                                at different scales.
%
%                           - "none"             Makes the entire network trainable.
%
%                           Default: "none"
%
%  Notes:
% ---------------
% - Custom YOLO v3 object detector created using
%   yolov3ObjectDetector(net,classes,aboxes) object does not support
%  'FreezeSubNetwork' with value "backbone" or "backboneAndNeck".
% - Any YOLO v3 object detector created before MATLAB® ''R2024a''
%   does not support 'FreezeSubNetwork' with value
%  "backbone" or "backboneAndNeck".
%
%
%  Resume training
%  ---------------
%  [...] = trainYOLOv3ObjectDetector(trainingData,checkpoint,options)
%  resumes training the YOLO v3 object detector. Use this syntax to
%  continue training a detector with additional training data or to perform
%  more training iterations to improve detector accuracy.
%
%  Specify the 'CheckpointPath' parameter in trainingOptions function
%  in order to save detector checkpoints at regular intervals. You may resume
%  training from any one of these saved checkpoints.
%
%  See also yolov3ObjectDetector,  trainYOLOv4ObjectDetector,
%          trainSSDObjectDetector,trainMaskRCNN,trainFasterRCNNObjectDetector,
%          trainFastRCNNObjectDetector, trainRCNNObjectDetector
%          trainingOptions, boxLabelDatastore.
%
% References:
% -----------
% Redmon, Joseph, and Ali Farhadi. "YOLOv3: An Incremental Improvement."
%  arXiv, April 8, 2018. https://doi.org/10.48550/arXiv.1804.02767.
%
%  Copyright 2023-2024 The MathWorks, Inc.

function [trainedDetector, info] = trainYOLOv3ObjectDetector(trainingData, detector, options, params)

arguments
    trainingData 
    detector yolov3ObjectDetector 
    options nnet.cnn.TrainingOptions
    params.FreezeSubNetwork {mustBeText} = "none";
    params.ExperimentMonitor {mustBeScalarOrEmpty, iValidateExperimentMonitor} = [];
end

% Requires deep learning toolbox.
vision.internal.requiresNeuralToolbox(mfilename);
vision.internal.cnn.WarningLogger.initialize();

% Check that the solver and training options are supported.
assertMiniBatchSolver(options, mfilename);
checkTrainingOptions(options,mfilename);

% Validate trainingData.
[trainingData,isRotatedBox] = iValidateInputData(detector,trainingData);

% update detector in any of the two cases to support rotated bounding boxes
% workflow 
% case 1: If ground truth has rotated boxes and detector is "axis-aligned" 
% case 2: If ground truth has no rotated boxes and detector is "rotated" 
if (isRotatedBox && strcmp(detector.PredictedBoxType,"axis-aligned")) || ...
    (~isRotatedBox && strcmp(detector.PredictedBoxType,"rotated"))
    detector = iUpdateOutputLayers(detector,isRotatedBox);
end

% Validate params.FreezeSubNetwork
params.FreezeSubNetwork = iValidateFreezeSubNetwork(params.FreezeSubNetwork);

% Required preprocessing to be performed, which rescales input training data
% and resizes them to the detector input size.
inpSize = detector.InputSize;
cdsTransformed = transform(trainingData, @(x) yolov3ObjectDetector.preprocessInput(x,inpSize));

% Manage input normalization from training options
if options.ResetInputNormalization

    printer1 = vision.internal.MessagePrinter.configure(true);   
    printer1.printMessage("vision:yolo:computingInputNormalization");
    printer1.linebreak;

    normQueue = minibatchqueue(cdsTransformed,1,...
        OutputAsDlarray=true,...
        PartialMiniBatch="return",...
        MiniBatchFormat='SSCB',...
        MiniBatchFcn = @batchForNormalization,...
        MiniBatchSize=options.MiniBatchSize);

    options.ResetInputNormalization = false;
    stats = images.dltrain.internal.computeNormalizationStats(normQueue,1,options.ExecutionEnvironment,NormalizationDimension="channel");
    detector = setInputNormalization(detector,stats);
end

% Update the validation datastore.
if ~isempty(options.ValidationData)
    [valDS,~] = iValidateInputData(detector,options.ValidationData);
    options.ValidationData = transform(valDS, @(x) yolov3ObjectDetector.preprocessInput(x,inpSize));
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
miniBatchFormat(2)="";

% Define the minibatchqueue for training the dlnetwork.
mbq = minibatchqueue(...
    cdsTransformed,...
    numOutputs,...
    "MiniBatchSize",options.MiniBatchSize,...
    "OutputEnvironment",[outputEnv,"cpu"],...
    "MiniBatchFcn",@iCreateBatchData,...
    "MiniBatchFormat",miniBatchFormat,...
    "OutputCast", outputCast, ...
    "OutputAsDlarray",[true,false],...
    "DispatchInBackground",options.DispatchInBackground);

% The backward mapping from classes in order in ClassNames to 1-based
% indices. Used for encoding.
classes = detector.ClassNames;
classToIndexMapping = dictionary(string(classes)',1:length(classes));

% The dltrain expects a single lossFcn that takes as input all of the
% outputs of the network and all of the targets and forms a scalar valued
% loss.
lossParams.NumOutputs = numel(detector.Network.OutputNames);
lossParams.AnchorBoxes = detector.AnchorBoxes;
lossParams.InputSize = detector.InputSize;
lossParams.classToIndexMapping = classToIndexMapping; 
lossFcn = @(varargin) iCalulateYOLOv3Loss(lossParams,isRotatedBox,varargin);

% For purposes of ValidationPatience early stopping, define which
% of the metrics from the metrics set will be used for purposes
% of early stopping.
validationPatienceMetric = 'Loss';

% A cell-array list of metric objects
metrics = {images.dltrain.internal.FunctionMetric(lossFcn,"Loss")};

specifiedMetrics = {};
if ~isempty(options.Metrics)
    
    specifiedMetrics = deep.internal.sdk.getObjectsFromMetricsOption(options); 

    validationOnlyMetrics = cellfun(@(metric) isprop(metric,'ValidationOnly') && metric.ValidationOnly,specifiedMetrics);
    if any(validationOnlyMetrics) && isempty(options.ValidationData)
        metricIdx = find(validationOnlyMetrics,1,'first');
        metric = specifiedMetrics{metricIdx};
        error(message("images:dltrain:validationOnlyMetricsRequireValidationData",metric.Name));
    end

    if (length(specifiedMetrics) > 1)
        error(message('vision:yolo:unsupportedMetricForYOLOv3'));
    elseif (isscalar(specifiedMetrics)) && ~isa(specifiedMetrics{1},"mAPObjectDetectionMetric")
        error(message('vision:yolo:unsupportedMetricForYOLOv3'));
    end
end

metrics = horzcat(metrics,specifiedMetrics);

% Configure printer for verbose printing.
printer = vision.internal.MessagePrinter.configure(options.Verbose);
printHeader(printer, detector.ClassNames);

% set useHighLevelTrainer flag to use YOLO v3 trainer
detector = updateUseHighLevelTrainerFlag(detector,true);

% Configure detector before freezing
detector = iConfigureDetectorBeforeFreezing(params,detector);

% The training function to train the input detector. Returns the trained detector.
[trainedDetector,infoTrain] = images.dltrain.internal.dltrain(mbq,detector,options,lossFcn,metrics,validationPatienceMetric,ExperimentMonitor=params.ExperimentMonitor);

% Configure detector after freezing
if ~strcmp(params.FreezeSubNetwork,"none") && ~isempty(detector.DetectionNetworkSource)
    trainedDetector = updateDetector(trainedDetector);
    trainedDetector = unsetSubNetworks(trainedDetector);
end

% Update info output.
info = iUpdateInfo(infoTrain,options);

printFooter(printer);
end

%--------------------------------------------------------------------------
% Batch data creation function.
%--------------------------------------------------------------------------
function [XTrain,labelsBatched] = iCreateBatchData(groundTruthImages,groundTruthBoxes,groundTruthClasses)
% Concatenate images along the batch dimension.
XTrain = cat(4, groundTruthImages{:,1});

% Pass ragged data straight through to manage within loss function
labelsBatched = horzcat(groundTruthBoxes,groundTruthClasses);
end

%--------------------------------------------------------------------------
% Loss function.
%--------------------------------------------------------------------------
function lossVal = iCalulateYOLOv3Loss(params,isRotatedBox,varargin) 
% Calculates the loss for predictions from the network and target responses.
YPredictions = cell(params.NumOutputs,1);

networkOutputs = varargin{1};
for i = 1:params.NumOutputs
    YPredictions{i} = networkOutputs{i};
end

YPredCell = yolov3ObjectDetector.yolov3Transform(YPredictions, isRotatedBox, params.AnchorBoxes);

% Gather the activations in the CPU for post processing and extract dlarray data.
gatheredPredictions = cellfun(@ gather, YPredCell,'UniformOutput',false);
gatheredPredictions = cellfun(@ extractdata, gatheredPredictions, 'UniformOutput', false);

% Convert predictions from grid cell coordinates to box coordinates.
gatheredPredictions(:,2:5) = yolov3ObjectDetector.anchorBoxGenerator(params.AnchorBoxes,gatheredPredictions(:,2:5),params.InputSize);

rawTargets = networkOutputs{params.NumOutputs+1};
YTrain = iComputeYTrain(rawTargets, isRotatedBox, params);

% Generate target for predictions from the ground truth data.
penaltyThreshold = 0.5;
[boxTarget, objectnessTarget, classTarget, objectMaskTarget, boxErrorScale] = iGenerateTargets(gatheredPredictions, YTrain, params.InputSize, ...
    params.AnchorBoxes, penaltyThreshold, isRotatedBox);

% Compute losses.
if ~isRotatedBox
    boxLoss = iBboxOffsetLoss(YPredCell(:,[2 3 7 8]),boxTarget,objectMaskTarget,boxErrorScale);
    clsLoss = iClassConfidenceLoss(YPredCell(:,6),classTarget,objectMaskTarget);
else
    boxLoss = iRotatedBboxOffsetLoss(YPredCell(:,[2 3 9 10 6 7]),boxTarget,objectMaskTarget,boxErrorScale);
    clsLoss = iClassConfidenceLoss(YPredCell(:,8),classTarget,objectMaskTarget);
end
objLoss = iObjectnessLoss(YPredCell(:,1),objectnessTarget,objectMaskTarget);

lossVal = boxLoss + objLoss + clsLoss;
end
%--------------------------------------------------------------------------
function boxLoss = iBboxOffsetLoss(boxPredCell, boxDeltaTarget, boxMaskTarget, boxErrorScaleTarget)
% Mean squared error for bounding box position.
lossX = sum(cellfun(@(a,b,c,d) mse(a.*c.*d,b.*c.*d),boxPredCell(:,1),boxDeltaTarget(:,1),boxMaskTarget(:,1),boxErrorScaleTarget));
lossY = sum(cellfun(@(a,b,c,d) mse(a.*c.*d,b.*c.*d),boxPredCell(:,2),boxDeltaTarget(:,2),boxMaskTarget(:,1),boxErrorScaleTarget));
lossW = sum(cellfun(@(a,b,c,d) mse(a.*c.*d,b.*c.*d),boxPredCell(:,3),boxDeltaTarget(:,3),boxMaskTarget(:,1),boxErrorScaleTarget));
lossH = sum(cellfun(@(a,b,c,d) mse(a.*c.*d,b.*c.*d),boxPredCell(:,4),boxDeltaTarget(:,4),boxMaskTarget(:,1),boxErrorScaleTarget));
boxLoss = lossX+lossY+lossW+lossH;
end

%--------------------------------------------------------------------------
function boxLoss = iRotatedBboxOffsetLoss(boxPredCell, boxDeltaTarget, boxMaskTarget, boxErrorScaleTarget)
% Mean squared error for bounding box position.
lossX = sum(cellfun(@(a,b,c,d) mse(a.*c.*d,b.*c.*d),boxPredCell(:,1),boxDeltaTarget(:,1),boxMaskTarget(:,1),boxErrorScaleTarget));
lossY = sum(cellfun(@(a,b,c,d) mse(a.*c.*d,b.*c.*d),boxPredCell(:,2),boxDeltaTarget(:,2),boxMaskTarget(:,1),boxErrorScaleTarget));
lossW = sum(cellfun(@(a,b,c,d) mse(a.*c.*d,b.*c.*d),boxPredCell(:,3),boxDeltaTarget(:,3),boxMaskTarget(:,1),boxErrorScaleTarget));
lossH = sum(cellfun(@(a,b,c,d) mse(a.*c.*d,b.*c.*d),boxPredCell(:,4),boxDeltaTarget(:,4),boxMaskTarget(:,1),boxErrorScaleTarget));
lossYaw1 = sum(cellfun(@(a,b,c,d) mse(a.*c.*d,b.*c.*d),boxPredCell(:,5),boxDeltaTarget(:,5),boxMaskTarget(:,1),boxErrorScaleTarget));
lossYaw2 = sum(cellfun(@(a,b,c,d) mse(a.*c.*d,b.*c.*d),boxPredCell(:,6),boxDeltaTarget(:,6),boxMaskTarget(:,1),boxErrorScaleTarget));
boxLoss = lossX+lossY+lossW+lossH+lossYaw1+lossYaw2;
end

%--------------------------------------------------------------------------
function clsLoss = iClassConfidenceLoss(classPredCell, classTarget, boxMaskTarget)
% Binary cross-entropy loss for class confidence score.
clsLoss = sum(cellfun(@(a,b,c) crossentropy(a.*c,b.*c,'TargetCategories','independent'),classPredCell,classTarget,boxMaskTarget(:,3)));
end

%--------------------------------------------------------------------------
function objLoss = iObjectnessLoss(objectnessPredCell, objectnessDeltaTarget, boxMaskTarget)
% Binary cross-entropy loss for objectness score.
objLoss = sum(cellfun(@(a,b,c) crossentropy(a.*c,b.*c,'TargetCategories','independent'),objectnessPredCell,objectnessDeltaTarget,boxMaskTarget(:,2)));
end

%--------------------------------------------------------------------------
function [boxDeltaTarget, objectnessTarget, classTarget, maskTarget, boxErrorScaleTarget] = iGenerateTargets(YPredCellGathered, groundTruth,inputImageSize, anchorBoxes, penaltyThreshold, isRotatedBox)
% generateTargets creates target array for every prediction element
% x, y, width, height, confidence scores and class probabilities. When the
% input data type consists of rotated rectangle bounding boxes, two
% additional parameters corresponding with the real and imaginary phases of
% the angle are also generated.

% Offset for generating two additional parameters with rotated rectangles
rotationOffset = 2*double(isRotatedBox);

boxDeltaTarget = cell(size(YPredCellGathered,1),4+rotationOffset);
objectnessTarget = cell(size(YPredCellGathered,1),1);
classTarget = cell(size(YPredCellGathered,1),1);
maskTarget = cell(size(YPredCellGathered,1),3);
boxErrorScaleTarget = cell(size(YPredCellGathered,1),1);

% Normalize the ground truth boxes w.r.t image input size.
gtScale = [inputImageSize(2) inputImageSize(1) inputImageSize(2) inputImageSize(1)];
groundTruth(:,1:4,:,:) = groundTruth(:,1:4,:,:)./gtScale;

anchorBoxesSet = cell2mat(anchorBoxes);

maskIdx = 1:size(anchorBoxesSet,1);
cellsz = cellfun(@size,anchorBoxes,'uni',false);
convMask = cellfun(@(v)v(1),cellsz);
anchorBoxMask = mat2cell(maskIdx,1,convMask)';

for numPred = 1:size(YPredCellGathered,1)

    % Select anchor boxes based on anchor box mask indices.
    anchors = anchorBoxes{numPred, :};

    bx = YPredCellGathered{numPred,2};
    by = YPredCellGathered{numPred,3};
    bw = YPredCellGathered{numPred,4};
    bh = YPredCellGathered{numPred,5};

    if isRotatedBox
        % Initialize angle variables when using rotated rectangles
        tsin = single(zeros(size(bh)));
        tcos = single(zeros(size(bh)));
    end

    predClasses = YPredCellGathered{numPred,6+rotationOffset};
   
    gridSize = size(bx);
    if numel(gridSize)== 3
        gridSize(4) = 1;
    end
    numClasses = size(predClasses,3)./size(anchors,1);

    % Initialize the required variables.
    mask = zeros(size(bx),'like',bx);
    confMask = ones(size(bx),'like',bx);
    classMask = zeros(size(predClasses),'like',predClasses);
    tx = zeros(size(bx),'like',bx);
    ty = zeros(size(by),'like',by);
    tw = zeros(size(bw),'like',bw);
    th = zeros(size(bh),'like',bh);
    tconf = zeros(size(bx),'like',bx);
    tclass = zeros(size(predClasses),'like',predClasses);
    boxErrorScale = ones(size(bx),'like',bx);

    % Get the IOU of predictions with groundtruth.
    iou = iGetMaxIOUPredictedWithGroundTruth(bx,by,bw,bh,groundTruth,isRotatedBox);

    % Donot penalize the predictions which has iou greater than penalty
    % threshold.
    confMask(iou > penaltyThreshold) = 0;

    for batch = 1:gridSize(4)

        if ~isRotatedBox
            truthBatch = groundTruth(:,1:5,:,batch);
            truthBatch = truthBatch(all(truthBatch,2),:);
        else
            truthBatch = groundTruth(:,1:6,:,batch);
            truthBatch = truthBatch(all([truthBatch(:,1:4) truthBatch(:,6)],2),:);
        end

        % Get boxes with center as 0.
        gtPred = [0-truthBatch(:,3)/2,0-truthBatch(:,4)/2,truthBatch(:,3),truthBatch(:,4)];
        anchorPrior = [0-anchorBoxesSet(:,2)/(2*inputImageSize(2)),0-anchorBoxesSet(:,1)/(2*inputImageSize(1)),anchorBoxesSet(:,2)/inputImageSize(2),anchorBoxesSet(:,1)/inputImageSize(1)];

        % Get the iou of best matching anchor box.
        overLap = bboxOverlapRatio(gtPred,anchorPrior);
        [~,bestAnchorIdx] = max(overLap,[],2);

        % Select gt that are within the mask.
        index = ismember(bestAnchorIdx,anchorBoxMask{numPred});
        truthBatch = truthBatch(index,:);
        bestAnchorIdx = bestAnchorIdx(index,:);
        bestAnchorIdx = bestAnchorIdx - anchorBoxMask{numPred}(1,1) + 1;

        if ~isempty(truthBatch)
            if ~isRotatedBox
                % Convert top left position of ground-truth to centre coordinates.
                truthBatch = [truthBatch(:,1)+truthBatch(:,3)./2,truthBatch(:,2)+truthBatch(:,4)./2,truthBatch(:,3),truthBatch(:,4),truthBatch(:,5)];
                errorScale = 2 - truthBatch(:,3).*truthBatch(:,4);
                truthBatch = [truthBatch(:,1)*gridSize(2),truthBatch(:,2)*gridSize(1),truthBatch(:,3)*inputImageSize(2),truthBatch(:,4)*inputImageSize(1),truthBatch(:,5)];
            else
                errorScale = 2 - truthBatch(:,3).*truthBatch(:,4);
                truthBatch = [truthBatch(:,1)*gridSize(2),truthBatch(:,2)*gridSize(1),truthBatch(:,3)*inputImageSize(2),truthBatch(:,4)*inputImageSize(1),truthBatch(:,5),truthBatch(:,6)];
            end

            for t = 1:size(truthBatch,1)

                % Get the position of ground-truth box in the grid.
                colIdx = ceil(truthBatch(t,1));
                colIdx(colIdx<1) = 1;
                colIdx(colIdx>gridSize(2)) = gridSize(2);
                rowIdx = ceil(truthBatch(t,2));
                rowIdx(rowIdx<1) = 1;
                rowIdx(rowIdx>gridSize(1)) = gridSize(1);
                pos = [rowIdx,colIdx];
                anchorIdx = bestAnchorIdx(t,1);

                mask(pos(1,1),pos(1,2),anchorIdx,batch) = 1;
                confMask(pos(1,1),pos(1,2),anchorIdx,batch) = 1;

                % Calculate the shift in ground-truth boxes.
                tShiftX = truthBatch(t,1)-pos(1,2)+1;
                tShiftY = truthBatch(t,2)-pos(1,1)+1;
                tShiftW = log(truthBatch(t,3)/anchors(anchorIdx,2));
                tShiftH = log(truthBatch(t,4)/anchors(anchorIdx,1));

                % Update the target box.
                tx(pos(1,1),pos(1,2),anchorIdx,batch) = tShiftX;
                ty(pos(1,1),pos(1,2),anchorIdx,batch) = tShiftY;
                tw(pos(1,1),pos(1,2),anchorIdx,batch) = tShiftW;
                th(pos(1,1),pos(1,2),anchorIdx,batch) = tShiftH;

                if isRotatedBox
                    % Calculate the shift in ground-truth boxes and update
                    % the target box for rotated rectangle bounding boxes.
                    tShiftSin = sind(truthBatch(t,5));
                    tShiftCos = cosd(truthBatch(t,5));
                    tsin(pos(1,1),pos(1,2),anchorIdx,batch) = tShiftSin;
                    tcos(pos(1,1),pos(1,2),anchorIdx,batch) = tShiftCos;
                    classIdx = (numClasses*(anchorIdx-1))+truthBatch(t,6);
                else
                    classIdx = (numClasses*(anchorIdx-1))+truthBatch(t,5);    
                end
                boxErrorScale(pos(1,1),pos(1,2),anchorIdx,batch) = errorScale(t);
                tconf(rowIdx,colIdx,anchorIdx,batch) = 1;
                
                tclass(rowIdx,colIdx,classIdx,batch) = 1;
                classMask(rowIdx,colIdx,(numClasses*(anchorIdx-1))+(1:numClasses),batch) = 1;
            end
        end
    end

    if ~isRotatedBox
        boxDeltaTarget(numPred,:) = [{tx} {ty} {tw} {th}];
    else
        boxDeltaTarget(numPred,:) = [{tx} {ty} {tw} {th} {tsin} {tcos}];
    end
       
    objectnessTarget{numPred,1} = tconf;
    classTarget{numPred,1} = tclass;
    maskTarget(numPred,:) = [{mask} {confMask} {classMask}];
    boxErrorScaleTarget{numPred,:} = boxErrorScale;
end
end


%--------------------------------------------------------------------------
function iou = iGetMaxIOUPredictedWithGroundTruth(predx,predy,predw,predh,truth,isRotatedBox)
% getMaxIOUPredictedWithGroundTruth computes the maximum intersection over
%  union scores for every pair of predictions and ground-truth boxes.

[h,w,c,n] = size(predx);
iou = zeros([h w c n],'like',predx);

rotationOffset = double(isRotatedBox);

% For each batch prepare the predictions and ground-truth.
for batchIdx = 1:n
    truthBatch = truth(:,1:4+rotationOffset,1,batchIdx);
    truthBatch = truthBatch(all(truthBatch(:,1:4),2),:);

    % Calculate the IoU exclusively for images that possess ground truth
    % annotations. If an image lacks ground truth, the IoU remains at zero,
    % as all IoU values are initially set to zero.
    if ~isempty(truthBatch)
        predxb = predx(:,:,:,batchIdx);
        predyb = predy(:,:,:,batchIdx);
        predwb = predw(:,:,:,batchIdx);
        predhb = predh(:,:,:,batchIdx);
        predb = [predxb(:),predyb(:),predwb(:),predhb(:)];

        if ~isRotatedBox
            % Convert from center xy coordinate to topleft xy coordinate.
            predb = [predb(:,1)-predb(:,3)./2, predb(:,2)-predb(:,4)./2, predb(:,3), predb(:,4)];
        else
            % Add yaw
            predb = [predb zeros(size(predb,1),1)];
        end
        % Compute and extract the maximum IOU of predictions with ground-truth.
        try
            if ~isRotatedBox
                overlap = bboxOverlapRatio(predb, truthBatch);
            else
                rots = truthBatch(:,5);
                rots = rots - floor((rots+0.5)/pi)*pi;
                idx = (rots > pi/4);
                truthBatch(idx,:) = truthBatch(idx,:);
                overlap = bboxOverlapRatio(predb(:,1:4), truthBatch(:,1:4));
            end
        catch me
            if(any(isnan(predb(:))|isinf(predb(:))))
                error(message('vision:yolo:detectedNaNInf'));
            elseif(any(predb(:,3)<=0 | predb(:,4)<=0))
                error(message('vision:yolo:invalidPredictions'));
            else
                error(message('vision:yolo:invalidGroundTruth'));
            end
        end

        maxOverlap = max(overlap,[],2);
        iou(:,:,:,batchIdx) = reshape(maxOverlap,h,w,c);
    end
end
end

%--------------------------------------------------------------------------
function freezeStr = iValidateFreezeSubNetwork(freezeStr)
if isStringScalar(freezeStr) || ischar(freezeStr)
    freezeStr = validatestring(freezeStr, ["backbone", "backboneAndNeck", "none"], mfilename, "FreezeSubNetwork");
else
    error(message('vision:yolo:invalidFreezeSubNetworkValue'));
end
end

%--------------------------------------------------------------------------
function [inputData,isRotatedBox] = iValidateInputData(detector,inputData)
% Check if the inputData data is valid.
if isa(inputData,'matlab.io.Datastore') || isa(inputData,'matlab.io.datastore.Datastore')
    opt.AllowRotatedBoxes = true;
    opt.AllowEmptyBoxes = true;
    opt.CheckLabels = 'auto';
    opt.IsYOLO = 'true';
    gTruthClassNames = vision.internal.inputValidation.checkGroundTruthDatastore(inputData, opt);
else
    error(message('vision:yolo:invalidTrainInput'));
end

% Copy and reset the given datastore, so external state events are
% not reflected.
inputData = copy(inputData);
reset(inputData);
data = read(inputData);

if istable(data)
    sampleData = data{1,1};
    sampleBox = data{1,2};
else
    sampleData = data(1,1);
    sampleBox = data(1,2);
end

sampleImg = sampleData{1};
% Check if inputData has rotated boxes
isRotatedBox = size(sampleBox{1},2) == 5;

% Check if the ground truth classes matched with the detector classes.
detectorClassNames = cellstr(detector.ClassNames);
if ~isequal(sort(gTruthClassNames)',sort(detectorClassNames))
    error(message('vision:yolo:trainingClassesMismatch'));
end

% Check if network is correctly defined for the image format in the
% inputData data.
iValidateNetworkInputSizes(detector.InputSize,sampleImg);
end


%--------------------------------------------------------------------------
function iValidateNetworkInputSizes(networkInputSize,img)
% Number of channels should be same in the input image and network input
% layer.
imageSize = size(img);

if numel(imageSize)==2
    numImageChannel = 1;
else
    numImageChannel = imageSize(3);
end

if numel(networkInputSize)==2
    numNetworkInputChannel = 1;
else
    numNetworkInputChannel = networkInputSize(3);
end

if numImageChannel~=numNetworkInputChannel
    error(message('vision:yolo:numChannelMismatch'));
end
end

%--------------------------------------------------------------------------
function iValidateExperimentMonitor(monitor)
% Validate experiment monitor input.
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

%--------------------------------------------------------------------------
function info = iUpdateInfo(infoTrain,options)
% Update training losses for each iteration.
trainingLoss = [infoTrain.TrainingLoss];

% Update validation losses for each iteration.
if ~isempty(options.ValidationData)
    validationLoss = [infoTrain.ValidationLoss];
end

% Update validation map50 for each iteration.
if ~isempty(options.ValidationData) && ~isempty(options.Metrics)
    validationmap50 = [infoTrain.Validationmap50];
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

if ~isempty(options.ValidationData) && ~isempty(options.Metrics)
    info.Validationmap50 = validationmap50;
end

info.BaseLearnRate = baseLearnRate;
if ~isempty(options.ValidationData)
    info.FinalValidationLoss = validationLoss(indx);
end
info.OutputNetworkIteration = OutputNetworkIteration;
end

function detector = iConfigureDetectorBeforeFreezing(params,detector)
if strcmp(params.FreezeSubNetwork,"none")
    % No freezing
    detector.FreezeBackbone = 0;
    detector.FreezeBackboneAndNeck = 0;
elseif ~strcmp(params.FreezeSubNetwork,"none") && ~isempty(detector.DetectionNetworkSource)
    if strcmp(params.FreezeSubNetwork,"backbone")
        % Freeze backbone
        detector.FreezeBackbone = 1;
        detector.FreezeBackboneAndNeck = 0;
    elseif strcmp(params.FreezeSubNetwork,"backboneAndNeck")
        % Freeze backbone and neck
        detector.FreezeBackbone = 0;
        detector.FreezeBackboneAndNeck = 1;
    end
    detector = setSubNetworks(detector);
else
    if strcmp(params.FreezeSubNetwork,"backbone")
        error(message('vision:yolo:freezingBackboneNotSupportedYOLOv3'));
    else
        error(message('vision:yolo:freezingBackboneAndNeckNotSupportedYOLOv3'));
    end
end
end
%--------------------------------------------------------------------------
function printHeader(printer, classNames)
printer.print('*************************************************************************\n');
printer.printMessage("vision:yolo:trainingHeaderYOLOv3");
printer.linebreak;
for i = 1:numel(classNames)
    printer.print('* %s\n', string(classNames(i)));
end
printer.linebreak;
end

%------------------------------------------------------------------
function printFooter(printer)
printer.linebreak;
printer.print('*************************************************************************\n');
printer.printMessage("vision:yolo:trainingFooter");
printer.print('*************************************************************************\n');
printer.linebreak;
end

%-----------------------------------------------------------------
function out = batchForNormalization(x)
    out = cat(4,x{:});
end
%--------------------------------------------------------------------------
function detector = iUpdateOutputLayers(detector,isRotatedBox)
network = detector.Network;
anchorBoxes = detector.AnchorBoxes;
classes = detector.ClassNames;
numOutputLayers = size(network.OutputNames,2);
layerNames = string({network.Layers.Name});
detectionNetworkSource = detector.DetectionNetworkSource;
modelName = detector.ModelName;

% Set rotationOffset for rotated rectangle bounding boxes
if isRotatedBox
    rotationOffset = 2;
    predictedBoxType = "rotated";
else
    rotationOffset = 0;
    predictedBoxType = "axis-aligned";
end

for i = 1:numOutputLayers
    layerIdx = find(strcmp(network.OutputNames{1,i}, layerNames));
    
    idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.Convolution2DLayer'),network.Layers(1:layerIdx));  
    filterIdx = find(idx,1,'last');

    numAnchors = size(anchorBoxes{i,1},1);
    numClasses = size(classes,1);
    numFilters = (numClasses + 5 + rotationOffset)*numAnchors;
    convOut = convolution2dLayer([1,1],numFilters,'Padding','same','Name',['convOut',num2str(i)]);
    network = replaceLayer(network,layerNames{1,filterIdx},convOut);
end
network = initialize(network);
detector = yolov3ObjectDetector(network,classes,anchorBoxes,PredictedBoxType=predictedBoxType,ModelName=modelName);
% Set Detection Network Source for updated detector to support freezing 
detector = setDetectionNetworkSource(detector,detectionNetworkSource);
end
%-----------------------------------------------------------------
function checkTrainingOptions(options,trainerName)

if ~isequal(options.InputDataFormats,"auto")
    error(message('nnet_cnn:TrainingOptionsSGDM:TrainerUnsupportedOption', trainerName, 'InputDataFormats'));
end
if ~isequal(options.TargetDataFormats,"auto")
    error(message('nnet_cnn:TrainingOptionsSGDM:TrainerUnsupportedOption', trainerName, 'TargetDataFormats'));
end

end
%----------------------------------------------------------------
function YTrain =  iComputeYTrain(T,isRotatedBox, params)
% Put targets into encodeded form

miniBatchSize = size(T,1);
maxBoxesPerImage = max(cellfun(@length,T(:,2)));
boxesBatched = zeros([maxBoxesPerImage,5+double(isRotatedBox),1,miniBatchSize],'single');

% For each observation assign the boxes with the class index included
for idx = 1:miniBatchSize
    boxes = T{idx,1};
    classes = T{idx,2};
    boxesWithClassIndex = cat(2,boxes,params.classToIndexMapping(classes));
    boxesBatched(1:size(boxes,1),:,1,idx) = boxesWithClassIndex;
end
YTrain = boxesBatched;
end 