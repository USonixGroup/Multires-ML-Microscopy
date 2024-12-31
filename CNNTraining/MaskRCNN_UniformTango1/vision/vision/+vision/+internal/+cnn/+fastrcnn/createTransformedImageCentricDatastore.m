function [ds, batchFcn] = createTransformedImageCentricDatastore(trainingDatastore, params)
%createTransformedImageCentricRegionDatastore Fast RCNN image centric training datastore.
%

% Copyright 2019-2020 The MathWorks, Inc.

% Generates training data for Fast RCNN.

    options       = iSetupOptions(params);
    batchFcn      = options.BatchingFcn;
    ds            = transform(trainingDatastore,...
        @(data, info)iCreateImageCentricData(data, info, options),...
        'IncludeInfo', true);
end

function [tbl, info] = iCreateImageCentricData(data, info, params)

    [posRegressionTargets,~,regionProposals,positive,negative,allLabels,images] = ...
        rcnnObjectDetector.extractTargetsUsingDataFromDatastore(data, info, params.RegionProposalFcn, params);

    numImages     = numel(images);
    emptyTargets  = false(numImages, 1);
    roiBatch      = cell(numImages,1);
    responseBatch = cell(numImages,2);


    for idx = 1:numImages

        if isempty(posRegressionTargets{idx})
            emptyTargets(idx) = true;
            continue;
        end

        samples     = regionProposals{idx};
        posSamples  = samples(positive{idx},:);

        % 1:1 ratio between positive and negatives.
        numPos = params.NumPositiveSamplesPerBatch;
        N      = size(posSamples,1);
        numPos = min(numPos, N);

        pid        = params.RandomSelector.randperm(N, numPos);
        posSamples = posSamples(pid, :);

        bb = samples(negative{idx},:);
        N  = size(bb,1);

        if params.MiniBatchPadWithNegatives
            numNeg = min(N, params.ROIPerImage - numPos);
        else
            % honor foreground fraction
            numNeg = min(N, floor(numPos/params.PercentageOfPositiveSamples - numPos));
        end

        nid        = params.RandomSelector.randperm(N, numNeg);
        negSamples = bb(nid,:);

        % response
        labels      = allLabels{idx};
        posResponse = labels(positive{idx});
        negResponse = labels(negative{idx});

        % training rois
        roi = [posSamples; negSamples];

        % CLS response
        % data in mini-batch need not be shuffled. training responses
        % are averaged over all mini-batch samples so order does not
        % matter.


        clsResponse = [posResponse(pid,:);negResponse(nid,:)];
        clsResponse = reshape(clsResponse,1,1,1,[]);

        % REG Response
        numNeg = size(negSamples,1);
        numPos = size(posSamples,1);

        posTargets = posRegressionTargets{idx};
        posTargets = iStandardizeRegressionTargets(posTargets, params);
        posTargets = posTargets(:,pid);
        negTargets = zeros([4 numNeg],'like',posTargets);

        targets = [posTargets negTargets];

        % expand targets array for K-1 classes, excluding the
        % background.
        numClasses = numel(categories(labels));
        targets    = repmat(targets, numClasses-1, 1);
        targets    = reshape(targets, [1 1 (numClasses-1)*4 numNeg+numPos]);

        % Define instance weight
        if strcmp(params.BBoxRegressionNormalization,'batch')
            w = 1/(numPos+numNeg);
        else
            % only "valid" or positive boxes.
            % define instance weight that effectively computes the average
            % when summing weighted per sample loss. The average is
            % computed over number of observations. The number of
            % observations only include the number of positives because
            % only positive samples are regressed.
            w = 1/numPos;
        end

        % Create instance weights for each proposal.
        dummifiedPosResponse = nnet.internal.cnn.util.dummify(posResponse(pid));
        dummifiedNegResponse = nnet.internal.cnn.util.dummify(negResponse(nid));

        % create a "class selection" array. This facilitates selecting
        % class specific targets when computing the regression loss.
        selection = cat(4, w * dummifiedPosResponse, zeros(size(dummifiedNegResponse),'like',dummifiedPosResponse));
        selection = squeeze(selection);

        % get the location of "background" label in selection and
        % remove it.
        bgIdx              = strcmp(params.BackgroundLabel, categories(posResponse));
        selection(bgIdx,:) = [];

        % duplicate selection entries 4 times for tx,ty,tw,th, and
        % reshape to 4D array
        selection = repelem(selection, 4, 1);
        selection = reshape(selection, [1 1 (numClasses-1)*4 numNeg+numPos]);

        % pack output for training cls loss and reg loss
        responseBatch{idx,1} = clsResponse;
        responseBatch{idx,2} = {targets, selection};

        % ROI pooling layer expects roi format to be [x1 y1 x2 y2].
        roiBatch{idx} = vision.internal.cnn.boxUtils.xywhToX1Y1X2Y2(roi);
    end

    % Remove images that have no positive samples. There are
    % usually many more negatives than positives, which is why do
    % not remove images that do not have any negatives.
    images(emptyTargets,:)        = [];
    roiBatch(emptyTargets,:)      = [];
    responseBatch(emptyTargets,:) = [];

    tbl = horzcat(images, roiBatch, responseBatch(:,1), responseBatch(:,2));
end

%------------------------------------------------------------------
function options = iSetupOptions(params)

    fcnCopy = params.RegionProposalFcn;
    fcn     = @(x,filename)rcnnObjectDetector.invokeRegionProposalFcn(fcnCopy, x, filename);

    options.RegionProposalFcn = fcn;
    % Set the default background label.
    options.BackgroundLabel   = params.BackgroundLabel;

    % Create class names from ground truth data. For object
    % detection add a "Background" class.
    options.ClassNames = params.ClassNames;

    options.PositiveOverlapRange = params.PositiveOverlapRange;
    options.NegativeOverlapRange = params.NegativeOverlapRange;

    options.ScaleImage = params.ScaleImage;
    options.ImageScale = params.ImageScale;

    options.StandardizeRegressionTargets = isempty(params.InternalOptions.BoxRegressionMeanStd);
    if options.StandardizeRegressionTargets
        options.BoxRegressionMean = params.ImageInfo.BoxRegressionMean';
        options.BoxRegressionStd  = params.ImageInfo.BoxRegressionStd';
    else
        options.BoxRegressionMean = params.InternalOptions.BoxRegressionMeanStd(1,:);
        options.BoxRegressionStd  = params.InternalOptions.BoxRegressionMeanStd(2,:);
    end

    options.PercentageOfPositiveSamples = params.InternalOptions.FastForegroundFraction;
    options.ROIPerImage                 = params.NumRegionsToSample;
    options.MiniBatchSize               = params.MiniBatchSize;
    options.RandomSelector              = params.RandomSelector;
    options.MiniBatchPadWithNegatives   = params.InternalOptions.MiniBatchPadWithNegatives;
    options.BBoxRegressionNormalization = params.InternalOptions.SmoothL1Normalization;
    options.ColorPreprocessing          = params.ColorPreprocessing;
    options.DispatchInBackground        = params.DispatchInBackground;

    options.OutputTableVariableNames    = {'Image', 'ROI', 'ClassificationResponse','RegressionResponse'};

    % BatchingFcn Define custom batching function for a cell of M-by-4
    % ROIs. This overrides the default batching function which attempts
    % to cat ROI along the 4-th dimension. Each struct field should
    % correspond to one of the table variable names.
    options.BatchingFcn.InputFunctions = {[], ...
        @(x,~)vision.internal.cnn.catAndSliceStrategy.FastRCNNCatAndSliceStrategy.roiBatch(x)};
    options.BatchingFcn.OutputFunctions = {[], ...
        @(x,~)vision.internal.cnn.catAndSliceStrategy.FastRCNNCatAndSliceStrategy.regressionResponseBatch(x)};

    options.BatchingFcn.CatAndSliceStrategy = vision.internal.cnn.catAndSliceStrategy.FastRCNNCatAndSliceStrategy();

    options.NumPositiveSamplesPerBatch = iGetNumPositiveSamplesPerBatch(options);

    % Needed for region proposal extraction.
    options.MinObjectSize = params.MinObjectSize;

    % No need to calculate average image during read,
    % since this is calculated at the begining once.
    options.NeedsZeroCenterNormalization = false;
    options.NumStrongestRegions          = params.NumStrongestRegions;
    options.InputSize                    = params.InputSize;
end

function val = iGetNumPositiveSamplesPerBatch(params)
    val = floor( params.PercentageOfPositiveSamples * params.ROIPerImage );
end

function targets = iStandardizeRegressionTargets(targets, params)
    if params.StandardizeRegressionTargets
        % Standardize regression targets
        targets = (targets - params.BoxRegressionMean)./params.BoxRegressionStd;
    end
end
