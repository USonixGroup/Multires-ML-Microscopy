% Train a Pose Mask R-CNN network to perform 6-DoF object pose estimation.

%   Copyright 2023 The MathWorks, Inc.

function [network, info] = trainPoseMaskRCNN(trainingData, network, trainingMode, options, params)

    arguments
        trainingData {iValidateTrainingdata, mustBeScalarOrEmpty}
        network (1,1) posemaskrcnn
        trainingMode {mustBeMember(trainingMode, {'mask','pose-and-mask'})} 
        options (1,1) nnet.cnn.TrainingOptions
        params.PositiveOverlapRange  {iValidateOverlapRange,...
                                                            mustBeGreaterThanOrEqual(params.PositiveOverlapRange, 0),...
                                                            mustBeLessThanOrEqual(params.PositiveOverlapRange, 1)} = [0.5 1]
        params.NegativeOverlapRange  {iValidateOverlapRange,...
                                                            mustBeGreaterThanOrEqual(params.NegativeOverlapRange, 0),...
                                                            mustBeLessThanOrEqual(params.NegativeOverlapRange, 1),...
                                                            iValidateOverlapRange} = [0.1 0.5]
        params.NumStrongestRegions (1,1)  {mustBeNumeric, mustBePositive, mustBeNonsparse,...
                                           mustBeScalarOrEmpty, mustBeNonempty, ...
                                           mustBeNonNan, mustBeInteger, mustBeFinite, mustBeReal} = 1000
        params.NumRegionsToSample (1,1)        {mustBeNumeric, mustBePositive, mustBeNonsparse,...
                                                mustBeScalarOrEmpty, mustBeNonempty, ...
                                                mustBeNonNan, mustBeInteger, mustBeFinite, mustBeReal} = 128
        params.FreezeSubNetwork string {iValidateFreezeSubNetwork} = "none"
        params.ExperimentMonitor {iValidateMonitor} = []
        params.TranslationLossWeight (1,1)  {mustBeNumeric, mustBeNonsparse,...
                                           mustBeScalarOrEmpty, mustBeNonempty, ...
                                           mustBeNonNan, mustBeFinite, mustBeReal} = 10
        params.RotationLossWeight (1,1)  {mustBeNumeric, mustBeNonsparse,...
                                           mustBeScalarOrEmpty, mustBeNonempty, ...
                                           mustBeNonNan, mustBeFinite, mustBeReal} = 10
        params.BoxLossWeight (1,1)  {mustBeNumeric, mustBeNonsparse,...
                                           mustBeScalarOrEmpty, mustBeNonempty, ...
                                           mustBeNonNan, mustBeFinite, mustBeReal} = 1
        params.ClassLossWeight (1,1)  {mustBeNumeric, mustBeNonsparse,...
                                           mustBeScalarOrEmpty, mustBeNonempty, ...
                                           mustBeNonNan, mustBeFinite, mustBeReal} = 1
        params.MaskLossWeight (1,1)  {mustBeNumeric, mustBeNonsparse,...
                                           mustBeScalarOrEmpty, mustBeNonempty, ...
                                           mustBeNonNan, mustBeFinite, mustBeReal} = 1
        params.RPNLossWeight (1,1)  {mustBeNumeric, mustBeNonsparse,...
                                           mustBeScalarOrEmpty, mustBeNonempty, ...
                                           mustBeNonNan, mustBeFinite, mustBeReal} = 1
        params.InternalOptions struct = struct.empty()
    end

     % Check that the solver is supported.
    assertMiniBatchSolver(options, mfilename)
    validateDataFormatsAndMetricsAreEmpty(options, mfilename);

    % Validate no overlap between pos & neg overlap ranges
    iValidateNoRangeIntersection(params.PositiveOverlapRange, params.NegativeOverlapRange);

    % Validate if gpu execution on trainingOptions is supported
    if(strcmp(options.ExecutionEnvironment, 'gpu') && ~canUseGPU)
        error(message('vision:maskrcnn:noGPUFound'));
    end

    % Fill InternalOptions
    if isempty(params.InternalOptions)
        params.InternalOptions = vision.internal.cnn.pose.defaultPoseMaskRCNNInternalOptions();
        pointCloudClassMap = [];
    else
        % Specify point clouds for the Chamfer-distance based rotation loss
        % as an internal option
        InternalOptions = params.InternalOptions;
        iValidateInternalOptions(network,InternalOptions.ClassPointClouds);
        pointCloudClassMap = processClassPointClouds(...
            params.InternalOptions.ClassPointClouds, network.ClassNames);
    end

    % Apply training mode settings
    if isequal(trainingMode, 'mask')
        params.RotationLossWeight = 0;
        params.TranslationLossWeight = 0;
    end

    % Set output environment.
    if options.ExecutionEnvironment == "multi-gpu" 
        execEnvironment = "gpu";
    elseif options.ExecutionEnvironment == "parallel"
        execEnvironment = "auto";
    else
        execEnvironment = options.ExecutionEnvironment;
    end

    % Create minibatchqueue
    netInputSize = network.InputSize;
    preprocessDS = transform(trainingData,...
        @(x)preprocessData(x, netInputSize, pointCloudClassMap));

    if(~isempty(options.ValidationData))
        iValidateTrainingdata(options.ValidationData);
        % Add preprocessing to validation data
        options.ValidationData = transform(options.ValidationData,...
            @(x)preprocessData(x, netInputSize, pointCloudClassMap));
    end

    miniBatchSize = options.MiniBatchSize;
    miniBatchFcn = @(img,xyzMap,boxes,labels,masks,rotations,translations,intrinsics,xyzMap2,pointClouds)...
        deal(cat(4,img{:}), cat(4,xyzMap{:}), boxes,labels,masks,rotations,translations,intrinsics,xyzMap2,pointClouds);

    % Variables - [image, XYZmap, boxes, labels, masks, rotations, translations, intrinsics, XYZMap, pointclouds]
    numDataItems = 10;
    mbqTrain = minibatchqueue(preprocessDS,numDataItems, ...
        "MiniBatchFormat",["SSCB","SSCB","","","","","","","",""], ...
        "MiniBatchSize",miniBatchSize, ...
        "OutputCast",["single","single","","","","","","","",""], ...
        "OutputAsDlArray",[true,true,false,false,false,false,false,false,false,false], ...
        "MiniBatchFcn",miniBatchFcn, ...
        "OutputEnvironment",[execEnvironment,execEnvironment,"cpu","cpu","cpu","cpu","cpu","cpu","cpu","cpu"]);

    % Create training configuration
    trainConfig = getPoseMaskRCNNTrainConfig(network);
    trainConfig.PositiveOverlapRange = params.PositiveOverlapRange;
    trainConfig.NegativeOverlapRange = params.NegativeOverlapRange;
    trainConfig.NumRegionsToSample = double(params.NumRegionsToSample);
    trainConfig.RotationLossWeight = double(params.RotationLossWeight);
    trainConfig.TranslationLossWeight = double(params.TranslationLossWeight);
    trainConfig.BoxLossWeight = double(params.BoxLossWeight);
    trainConfig.ClassLossWeight = double(params.ClassLossWeight);
    trainConfig.MaskLossWeight = double(params.MaskLossWeight);
    trainConfig.RPNLossWeight = double(params.RPNLossWeight);

    % Configure posemaskrcnn object for training with network freeze options
    % and NumStrongestRegions
    network = configureForTraining(network, double(params.NumStrongestRegions), params.FreezeSubNetwork);

    lossFcn = vision.internal.cnn.pose.PoseMaskRCNNLoss(trainConfig);
    
    % Define metric names. These are same as the lossObj field names as
    % returned by the lossfunction below. This avoids recompute of the metrics.
    metrics = {images.dltrain.internal.LossMetricAdapter(lossFcn)};
    [network,info] = images.dltrain.internal.dltrain(...
        mbqTrain,network,options,lossFcn,metrics, ...
        'Loss', 'ExperimentMonitor',params.ExperimentMonitor);

    % Remove fields from info struct
    fieldsToRemove = {'Epoch', 'Iteration', 'TimeElapsed', 'OutputNetworkIteration'};
    info = rmfield(info, fieldsToRemove);
end


function pointCloudClassMap = processClassPointClouds(classPointClouds, objectClassNames)
    % Returns a dictionary mapping classname to its object point cloud in 
    % canonical pose. The point clouds are returned as N-by-3 numerical
    % arrays.
    pointCloudClassMap = dictionary();
    numSubsampledPoints = 1000;
    for ii = 1:length(classPointClouds)
        className = objectClassNames{ii};
        ptCloud = classPointClouds{ii};
        if numSubsampledPoints < ptCloud.Count
            subSampleFactor = numSubsampledPoints / ptCloud.Count;
            ptCloud = pcdownsample(ptCloud,'random',subSampleFactor,PreserveStructure=true);
        end
        pointCloudClassMap(className) = {ptCloud.Location};
    end
end


function data = preprocessData(data,targetSize, pointCloudClassMap)
    % Converts the returns of the training or validation datastore object 
    % to the format used in training Pose Mask R-CNN.
    %
    % data - cell array containing the returns from a single call to a
    %        training or validation datastore.
    %        It is expected to contain, in order: {RGB image, depth image,
    %        bounding boxes (M-by-4 single), labels (M-by-1 categorical), 
    %        masks (H-by-W-by-M logical), poses (M-by-1 rigidtform3d
    %        vector), intrinsics (M-by-1 cameraIntrinsics).
    % 
    % Returns:
    %   - RGB image (H-by-W-by-3)
    %   - XYZ image (3D world-coordinates for each 2D image pixel position)
    %   - bboxes (M-by-4 single)
    %   - labels (M-by-1 categorical)
    %   - masks (H-by-W-by-M logical)
    %   - rotations (M-by-4, quaternions)
    %   - translations (M-by-3, 3D points)
    %   - point clouds for each instance (numSampledPoints-by-3-M)

    % handle case of Sequential Datastores returning a 1-element cell
    if isscalar(data) && isa(data, 'cell') && isa(data{1}, 'cell')
        data = data{1};
    end
    
    [im,imDepth,bboxes,labels,masks,poses,intrinsics] = data{:};
    imgSize = size(im);

    % Create point clouds for each instance in the data sample
    if ~isempty(pointCloudClassMap)
        instPointClouds = cell(length(labels),1);
        for i = 1:length(labels)
            className = string(labels(i));
            ptCloud = pointCloudClassMap(className);
            instPointClouds{i} = ptCloud{1};
        end
        instPointClouds = single(cat(3, instPointClouds{:}));
    else
        instPointClouds = [];
    end
    
    % Resize the min dimension to targetSize and resize the other dim to
    % maintain aspect ratio.
    
    % Resize images, masks, and bboxes
    [~,minDim] = min(imgSize(1:2));
    resizeSize = [NaN NaN];
    resizeSize(minDim) = targetSize(minDim);
    
    im = imresize(im,resizeSize);
    masks = imresize(masks,resizeSize);
    imDepth = imresize(imDepth,resizeSize);
    
    resizeScale = targetSize(minDim)/imgSize(minDim);
    bboxes = bboxresize(round(bboxes),resizeScale);

    % Padding im, masks and imDepth if resizing does not achieve targetSize
    if (size(im,1) < targetSize(1)) || (size(im,2) < targetSize(2))
        im = iPadInputImage(im, targetSize);
        masks = iPadInputImage(masks, targetSize);
        imDepth = iPadInputImage(imDepth, targetSize);
    end
    
    % Crop to target size
    cropWindow = randomWindow2d(size(im),targetSize(1:2));
    [bboxes,indices] = bboxcrop(bboxes,cropWindow,'OverlapThreshold',0.7);
    im = imcrop(im,cropWindow);
    
    [r,c] = deal(cropWindow.YLimits(1):cropWindow.YLimits(2),cropWindow.XLimits(1):cropWindow.XLimits(2));
    masks = masks(r,c,indices);

    imDepth = imcrop(imDepth,cropWindow);
    
    labels = labels(indices);
    
    if(isempty(bboxes))
        data = [];
        return;
    end
    
    bboxes = max(bboxes,1);
    
    % Normalize color image using COCO imageset means
    imageMean = single([123.675, 116.28, 103.53]);
    im = single(im);
    im(:,:,1) = im(:,:,1) - imageMean(1);
    im(:,:,2) = im(:,:,2) - imageMean(2);
    im(:,:,3) = im(:,:,3) - imageMean(3);

    % Create XYZ map of world-coordinates at every image pixel position,
    % taking into account image resizing operations
    xyzIm = iComputeXYZImage(imDepth, intrinsics.K, resizeScale);
    xyzIm = single(xyzIm);
    scaledIntrinsics = iScaleIntrinsics(intrinsics, resizeScale);

    % Get rotations and translations from rigidtform3d pose
    translations = arrayfun(@(x)(x.Translation), poses, 'UniformOutput', false);
    translations = single(vertcat(translations{:}));
    rotationCell = arrayfun(@(x)(x.R), poses, 'UniformOutput', false);
    rotmArray = cat(3, rotationCell{:});
    
    % Convert the rotation matrices into Mx4 array of quaternions for training
    rotations = single(vision.internal.quaternion.rotationToQuaternion(rotmArray))';
    for ii = 1:size(rotations,1)
       rotations(ii,:) = vision.internal.cnn.pose.uniqueQuaternion(rotations(ii,:));
    end

    % inputs
    data{1} = im;
    data{2} = xyzIm;

    % targets
    data{3} = bboxes;
    data{4} = labels;
    data{5} = masks;
    data{6} = rotations;
    data{7} = translations;
    data{8} = scaledIntrinsics;
    data{9} = xyzIm; % needed to repeat this to make it available in PoseMaskRCNNLoss inputs
    data{10} = instPointClouds;
    
end

function trainConfig = getPoseMaskRCNNTrainConfig(network)
    % Network parameters
    trainConfig.ImageSize = network.InputSize;
    trainConfig.NumClasses = numel(network.ClassNames);
    trainConfig.ClassNames = {network.ClassNames, 'background'};
    trainConfig.BackgroundClass = 'background';
    trainConfig.ClassAgnosticMasks = false;  
    trainConfig.ScaleFactor = [0.0625 0.0625];
    trainConfig.MaskOutputSize = [14 14];
       
    % Region Proposal network params
    trainConfig.AnchorBoxes = network.AnchorBoxes;
    trainConfig.NumAnchors = size(network.AnchorBoxes,1);
    trainConfig.NumRegionsToSample = 200;

    % NMS threshold
    trainConfig.OverlapThreshold = 0.7;
    trainConfig.MinScore = 0;
    trainConfig.RPNClassNames = {'Foreground', 'Background'};
    trainConfig.RPNBoxStd   = [1 1 1 1];
    trainConfig.RPNBoxMean  = [0 0 0 0];

    % RPN training target generation params
    trainConfig.RandomSelector = vision.internal.rcnn.RandomSelector();
    trainConfig.StandardizeRegressionTargets = false;
    trainConfig.MiniBatchPadWithNegatives = true;
    trainConfig.ProposalsOutsideImage = 'clip';
    trainConfig.BBoxRegressionNormalization = 'valid';
    trainConfig.RPNROIPerImage = trainConfig.NumRegionsToSample;
    trainConfig.CategoricalLookup = reshape(categorical([1 2 3],[1 2],trainConfig.RPNClassNames),[],1);

    % Loss scalers
    trainConfig.RotationLossWeight = 10;
    trainConfig.TranslationLossWeight = 0.01;
    trainConfig.RPNLossWeight = 1;
    trainConfig.BoxLossWeight = 1;
    trainConfig.ClassLossWeight = 1;
    trainConfig.MaskLossWeight = 1;

end

function scaledIntrinsics = iScaleIntrinsics(intrinsics, scaleRatio)
    Kscaled = [scaleRatio 0 0; 0 scaleRatio 0; 0 0 1] * intrinsics.K;
    scaledIntrinsics = cameraIntrinsics(...
        [Kscaled(1,1) Kscaled(2,2)], [Kscaled(1,3)  Kscaled(2,3)],...
        round(intrinsics.ImageSize * scaleRatio));
end

function xyz = iComputeXYZImage(imD, K, scaleRatio)
    % Create X,Y,Z "image" in camera-centered world coordinate system
    % (refer to posemaskrcnn.computeXYZImage() )
    Kscaled = [scaleRatio 0 0; 0 scaleRatio 0; 0 0 1] * K;
    [U,V] = meshgrid(1:size(imD,2), 1:size(imD,1)); % image coordinates of each pixel
    U = U .* imD;
    V = V .* imD;
    uvz = cat(3,U,V,imD);
    uvz = reshape(uvz, size(U,1)*size(U,2), 3);
    xyz = pinv(Kscaled)* uvz';
    xyz = xyz';
    xyz = reshape(xyz, size(U,1), size(U,2), 3);
end

function imageOut = iPadInputImage(imageIn, targetSize)
    sz = size(imageIn);
    imageOut = zeros(targetSize(1),targetSize(2),size(imageIn,3), 'like', imageIn);
    if size(imageIn,3) > 1
        imageOut(1:sz(1),1:sz(2),:) = imageIn;
    else
        imageOut(1:sz(1),1:sz(2)) = imageIn;
    end
end

function flag = iValidateInternalOptions(network, ClassPointClouds)
    arguments
        network (1,1) posemaskrcnn %#ok<INUSA>
        ClassPointClouds (:,1) cell {iValidatePointCloud(network,ClassPointClouds)} %#ok<INUSA>
    end
    flag = true;
end

function flag = iValidatePointCloud(network,categoryPointClouds)
    if ~isequal(length(network.ClassNames), size(categoryPointClouds,1))
        eid = 'vision:trainPoseMaskRCNN:incorrectPointCloudClasses';
        error(message(eid))
    end
    for i = 1:length(categoryPointClouds)
        validateattributes(categoryPointClouds{i},{'pointCloud'},{'size', [1 1]});
    end
    flag = true;
end

function flag = iValidateTrainingdata(trainData)

    if(~matlab.io.datastore.internal.shim.isDatastore(trainData))
        error(message('vision:maskrcnn:invalidDatastore'));
    end

    data = preview(trainData);
    
    % handle case of Sequential Datastores returning a 1-element cell
    if isscalar(data) && isa(data, 'cell') && isa(data{1}, 'cell')
        data = data{1};
    end
    
    % data is a cell array of size 1x7
    validateattributes(data,{'cell'}, {'nonempty','size', [1 7]}, mfilename, 'trainingData',1);

    % validate training color images
    validateattributes(data{1},{'numeric'}, {'nonempty','size', [NaN NaN 3]});
    
    % validate training depth images
    validateattributes(data{2},{'numeric'}, {'nonempty','size', [NaN NaN 1]});

    % validate bounding boxes
    validateattributes(data{3},{'numeric'}, {'size', [NaN 4]});

    numObjects = size(data{3},1);

    % validate labels
    validateattributes(data{4},{'categorical'}, {'column', 'numel', numObjects});

    % validate masks
    validateattributes(data{5},{'logical'}, {'size', [NaN NaN numObjects]});

    % validate poses
    validateattributes(data{6},{'rigidtform3d'}, {'column', 'numel', numObjects});

    % validate camera intrinsics
    validateattributes(data{7},{'cameraIntrinsics'}, {'size', [1 1]});

    flag = true;
end

function iValidateFreezeSubNetwork(freezeStr)
    if isscalar(freezeStr)
        validatestring(lower(freezeStr), ["backbone", "rpn", "none"], mfilename,"FreezeSubNetwork");
    else
        if(~all(sort(lower(freezeStr)) == ["backbone", "rpn"]))
            error(message('vision:maskrcnn:invalidFreezeNetStr'));
        end
    end
end

function iValidateOverlapRange(x)

validateattributes(x,{'numeric'},{'size',[1, 2], 'real', 'finite', 'increasing', 'nonnan', 'nonsparse'});

end

function iValidateNoRangeIntersection(posRange, negRange)
    if(any(negRange > posRange(1)))
        error(message('vision:maskrcnn:overlapRangeIntersection'));
    end
end

function iValidateMonitor(monitor)
    
    if(isempty(monitor))
        return;
    end
    validateattributes(monitor, {'experiments.Monitor'}, {'nonempty','scalar'});
    if ~isempty(monitor.Metrics)
        error(message('vision:ObjectDetector:nonEmptyMetricsInfoInExperimentMonitor'));
    end
end

