function detector = trainACFObjectDetector(trainingData, varargin)

% Copyright 2016-2023 The MathWorks, Inc.

if nargin > 1
     [varargin{:}] = convertStringsToChars(varargin{:});
end

[params, trainingData] = parseInputs(trainingData, varargin{:});
printer = vision.internal.MessagePrinter.configure(params.Verbose);

printer.printMessage('vision:acfObjectDetector:trainBegin');
printer.printMessage('vision:acfObjectDetector:trainStageSummary', ...
    params.NumStages, params.ModelSize(1), params.ModelSize(2));

tStart = tic;
classifier = [];
prevStream = RandStream.setGlobalStream(RandStream('mrg32k3a','Seed',0));

% Create datastore for training data
ds = prepareTrainingDatastore(trainingData);

% Collect positive examples
positiveImageSet = vision.internal.acf.sampleWindows(...
    ds, params, true, classifier, printer);

if isempty(positiveImageSet)
    error(message('vision:acfObjectDetector:TooFewPositiveExamples')); 
end

% Compute local decorrelation filters if required
if (length(params.FilterSize) == 2)
    positiveInstances = vision.internal.acf.computeSingleScaleChannels(...
        positiveImageSet, params);
    params.Filters = vision.internal.acf.channelCorrelation(...
        positiveInstances, params.FilterSize(1), params.FilterSize(2));
end

% Compute lambdas
if (isempty(params.Lambdas))
    printer.printMessageNoReturn('vision:acfObjectDetector:trainComputeLambda');
    printer.print('...');

    numPositive = size(positiveImageSet); 
    numPositive(1:end-1) = 1;
    siz = size(positiveImageSet);
    nd = ndims(positiveImageSet);
    bounds = cell(1, nd);
    for d = 1 : nd
        bounds{d} = repmat( siz(d)/numPositive(d), [1 numPositive(d)] ); 
    end
    ls = vision.internal.acf.scaleChannels(mat2cell(positiveImageSet, bounds{:}), params);
    params.Lambdas = round(ls*10^5)/10^5;

    printer.printMessage('vision:acfObjectDetector:stepCompletion');
end

% Compute features for positives
printer.printMessageNoReturn('vision:acfObjectDetector:trainComputeFeature');
printer.print('...');

positiveInstances = vision.internal.acf.computeSingleScaleChannels(positiveImageSet, params);
positiveInstances = reshape(positiveInstances, [], size(positiveInstances,4))';
clear positiveImageSet;

printer.printMessage('vision:acfObjectDetector:stepCompletion');

accumNegativeInstances = [];

% Iterate bootstrapping and training
for stage = 1 : params.NumStages
    printer.print('--------------------------------------------\n');
    printer.printMessage('vision:acfObjectDetector:trainStageStart', stage);

    % Sample negatives
    negativeImageSet = vision.internal.acf.sampleWindows(ds, params, false, classifier, printer);
    
    if (stage == 1 && isempty(negativeImageSet))
        error(message('vision:acfObjectDetector:TooFewNegativeExamples')); 
    end
    
    if (stage > 1)
        printer.printMessage('vision:acfObjectDetector:ReportNewNegatives', size(negativeImageSet, 4));
    end
    
    printer.printMessageNoReturn('vision:acfObjectDetector:trainComputeFeature');
    printer.print('...');
    
    % Compute features of negative examples
    negativeInstances = vision.internal.acf.computeSingleScaleChannels(negativeImageSet, params);
    clear negativeImageSet;
    negativeInstances = reshape(negativeInstances, [], size(negativeInstances, 4))';
  
    printer.printMessage('vision:acfObjectDetector:stepCompletion');

    % Accumulate negatives from previous stages
    accumNegativeInstances = [accumNegativeInstances; negativeInstances];  %#ok<AGROW>

    total = size(accumNegativeInstances, 1);
    if (params.NumNegativeSamples < total)
        ind = vision.internal.samplingWithoutReplacement(total, params.NumNegativeSamples);
        accumNegativeInstances = accumNegativeInstances(ind, :); 
    end

    printer.printMessageNoReturn('vision:acfObjectDetector:trainAdaBoost', ...
        size(positiveInstances, 1), size(accumNegativeInstances, 1));
    printer.print('...');
    
    % Train boosted classifier. The number of weak learns can be less than
    % the specified number.
    classifier = vision.internal.acf.trainBoostTreeClassifier(...
        accumNegativeInstances, positiveInstances, ...
        params.MaxWeakLearners(stage), params.MaxTreeDepth, params.FracFeatures);
    classifier.hs = classifier.hs + params.CalibrationValue;          

    printer.printMessage('vision:acfObjectDetector:stepCompletion');
    printer.printMessage('vision:acfObjectDetector:reportNumLearners', size(classifier.child, 2));    
    reset(ds);
end

detector = constructDetector(classifier, params);
RandStream.setGlobalStream(prevStream);

tElapsed = toc(tStart);
printer.print('--------------------------------------------\n');
printer.printMessage('vision:acfObjectDetector:trainEnd', num2str(tElapsed));

function detector = constructDetector(classifier, params)
c = rmfield(classifier, {'errs', 'losses'});
% Parameters used by detector
p.ModelName         = params.ModelName;
p.ModelSize         = params.ModelSize;
p.ModelSizePadded   = params.ModelSizePadded;
p.ChannelPadding    = params.ChannelPadding;
p.NumApprox         = params.NumApprox;
p.Shrink            = params.Shrink;
p.SmoothChannels    = params.SmoothChannels;
p.PreSmoothColor    = params.PreSmoothColor;
p.NumUpscaledOctaves= params.NumUpscaledOctaves;
p.gradient          = params.gradient;
p.hog               = params.hog;
p.Lambdas           = params.Lambdas;
% Training parameters specified by user
p.NumStages         = params.NumStages;
p.NegativeSamplesFactor = params.NegativeSamplesFactor;
p.MaxWeakLearners   = params.MaxWeakLearners;

detector = acfObjectDetector(c, p);

%==========================================================================
function [params, trainingData] = parseInputs(trainingData, varargin)

checkGroundTruth(trainingData);

p = inputParser;
p.addParameter('ObjectTrainingSize', 'Auto', @checkObjectTrainingSize);
p.addParameter('NumStages', 4, ...
    @(x)validateattributes(x,{'double'},{'scalar','positive','integer','nonsparse'},mfilename,'NumStages'));
p.addParameter('NegativeSamplesFactor', 5, ...
    @(x)validateattributes(x,{'double'},{'scalar','positive','nonsparse','finite'},mfilename,'NegativeSamplesFactor'));
p.addParameter('MaxWeakLearners', 2048);
p.addParameter('Verbose', true, ...
    @(x)validateattributes(x,{'logical'}, {'scalar','nonempty'},mfilename,'Verbose'));
% Undocumented option:
% Flip ground truth patch from left to right and increase the number of
% positive examples. It may introduce performance degeneration if the
% shape of the object is asymmetric. turn on this option only when
% available positive examples are not enough.
p.addParameter('FlipGroundTruth', false, ...
    @(x)validateattributes(x,{'logical'}, {'scalar','nonempty'},mfilename,'FlipGroundTruth'));
% Turn on/off parallel computing for training.
p.addParameter('UseParallel', vision.internal.useParallelPreference());

p.parse(varargin{:});

checkMaxWeakLearners(p.Results.MaxWeakLearners, p.Results.NumStages);

params.UseParallel = vision.internal.inputValidation.validateUseParallel(p.Results.UseParallel);

params.ModelName             = modelNameFromTrainingData(trainingData);
params.ModelSize             = p.Results.ObjectTrainingSize;   
params.NumStages             = p.Results.NumStages;
params.NegativeSamplesFactor = p.Results.NegativeSamplesFactor;
params.MaxWeakLearners       = p.Results.MaxWeakLearners;
params.Verbose               = p.Results.Verbose;

% Set internal parameters for detection
params.Threshold               = -1;
params.WindowStride            = 4;
params.NumScaleLevels          = 8;

% Set internal parameters for Adaboost
params.CalibrationValue = 0.005;
params.MaxTreeDepth     = 2;
params.FracFeatures     = 1;
params.MinWeight        = 0.01;
    
% Set internal parameters for jittering boxes
params.Flip       = p.Results.FlipGroundTruth; % flip along left-right
params.MaxJitters = 1000;
params.NumSteps   = 0;
params.Bound      = 0;
    
% Set fixed internal parameters for pyramid
params.NumApprox          = params.NumScaleLevels - 1;
params.Shrink             = 4;
params.SmoothChannels     = 1;
params.PreSmoothColor     = 1;
params.NumUpscaledOctaves = 0;

% Parameters for gradient computation
params.gradient.FullOrientation       = 0; % if true compute angles in [0,2*pi) else in [0,pi)
params.gradient.NormalizationRadius   = 5; % normalization radius for gradient
params.gradient.NormalizationConstant = 0.005; % normalization constant for gradient

% Parameters for HOG computation
params.hog.NumBins         = 6; % number of orientation channels
params.hog.Normalize       = 0; % if true perform 4-way hog normalization/clipping
params.hog.CellSize        = params.Shrink; % spatial bin size 
params.hog.Interpolation   = 'Orientation'; % 'Both' for spatial and orientation interpolation
params.hog.FullOrientation = params.gradient.FullOrientation;

% If filter size is provided as [w, n], filters are automatically computed.
params.Filters    = [];
params.FilterSize = [];
params.Lambdas    = [];

% Check that initial table is valid
if istable(trainingData)
    checkTableIsValid(trainingData);
end

% Parameters for sampling windows
[bboxes, params.NumImages, trainingData] = boxesAndNumOfImagesFromTrainingData(trainingData);
params.NumPositiveSamples  = size(bboxes, 1); % use all ground truth

% Set the size if it is given from the user
if (ischar(params.ModelSize) || isstring(params.ModelSize))
    params.ModelSize = computeModelSize(bboxes);
end

if params.Flip
    params.NumPositiveSamples = params.NumPositiveSamples * 2;
end
params.NumNegativeSamples  = ceil(params.NegativeSamplesFactor * params.NumPositiveSamples);
params.NumNegativePerImage = 25;
% At most 10000 negative examples are used for one stage
params.NumNegativeAccumulation = max(10000, params.NumNegativeSamples*2);

shrink = params.Shrink;
params.ModelSizePadded = ceil(params.ModelSize/shrink)*shrink;
params.ChannelPadding = ceil((params.ModelSizePadded-params.ModelSize)/shrink/2)*shrink;

% Set the bootstrap parameters
if isscalar(params.MaxWeakLearners)
    if params.MaxWeakLearners <= 1024
        params.MaxWeakLearners = repelem(params.MaxWeakLearners, params.NumStages);
    else
        numWeakLearners = zeros(1, params.NumStages);
        base = floor(log2(params.MaxWeakLearners));
        for i = params.NumStages-1:-1:1
            numWeakLearners(i) = max(256, 2^(base-(params.NumStages-i)));
        end
        numWeakLearners(end) = params.MaxWeakLearners;
        params.MaxWeakLearners = numWeakLearners;
    end
end

%==========================================================================
function modelSize = computeModelSize(bboxes)

aspectRatio = bboxes(:, 4)./bboxes(:, 3); % height / width
meanAspectRatio = median(aspectRatio);
minWidth = max(min(bboxes(:, 3)), 8);
minHeight = minWidth * meanAspectRatio;
if minHeight < 8
    minHeight = 8;
    minWidth = minHeight / meanAspectRatio;
end

modelSize = round([minHeight minWidth]);

%==========================================================================
function checkGroundTruth(gt)
validateattributes(gt, {'table', 'matlab.io.Datastore', 'matlab.io.datastore.Datastore'},...
    {'nonempty'}, mfilename, 'trainingData', 1);

if istable(gt)
    if width(gt) < 2
        error(message('vision:ObjectDetector:trainingDataTableWidthLessThanTwo'));
    end

else 
   % Datastore training data may have optional label data. Check them if
   % they are present. Also permit empty boxes to be present in the
   % training data. Images w/ no boxes are only used for negative samples. 
    opts.CheckLabels = 'optional';
    opts.AllowEmptyBoxes = true;
    vision.internal.inputValidation.checkGroundTruthDatastore(gt,opts);
end

%==========================================================================
function [data, info] = checkTrainingDataDuringTraining(data, info)
% This function is applied as a transform to the training datastore and
% runs every time data is read from the datastore during training.

% Standardize data to be a cell array if needed.
data = iConvertToCellIfNeeded(data);

for i = 1:size(data,1)
    checkTrainingDataImages(data, info, i);
    checkTrainingDataBoxes(data, info, i);

    % Clamp bounding boxes where needed
    data{i,2} = clampDSBoundingBoxesIfNeeded(data, i);

    % N.B. Checking label data is not required during training as the
    % labels are not used. We only check labels once during datastore
    % validation. 
end

%==========================================================================
function checkTrainingDataImages(data, info, idx)
try
    % Image data must be stored in a scalar cell.
    validateattributes(data(idx,1),{'cell'},{'nonempty','scalar'});

    % Cell contents must be numeric.
    validateattributes(data{idx,1},{'double','single','uint8','uint16','logical'}, ...
        {'nonempty','real','nonsparse'});
catch
    throwDetailedError(...
        'vision:ObjectDetector:readOutputInvalidImageClass', info, idx)
end

% Image should be RGB or grayscale.
if ~any(ndims(data{idx,1}) == [2 3]) || ~any(size(data{idx,1},3) == [1 3])
    throwDetailedError(...
        'vision:ObjectDetector:readOutputInvalidRGBOrGrayscaleImage', info, idx)
end

%==========================================================================
function checkTrainingDataBoxes(data, info, idx)
try
    % Must be scalar cell.
    validateattributes(data(idx,2),{'cell'},{'nonempty','scalar'});

    % Cell contents must be real and non-sparse.
    validateattributes(data{idx,2},{'numeric'}, ...
        {'real', 'nonsparse', 'finite'});
catch
    throwDetailedError(...
        'vision:ObjectDetector:readOutputInvalidBboxFileInfo',info, idx);
end

if ~isempty(data{idx,2})
    bboxes = data{idx,2};

    % Boxes must be stored in a cell and cell contents must be M-by-4 matrices
    if size(bboxes,2) ~= 4
        throwDetailedError(...
            'vision:ObjectDetector:readOutputInvalidBboxFileInfo',info, idx);
    end

    try
        % Width and height of the boxes must be positive.
        mustBePositive(bboxes(:,3:4))
    catch
        throwDetailedError(...
            'vision:ObjectDetector:readOutputInvalidBboxFileInfo',info, idx);
    end

    % Check if Boxes are located within the image frame.
    if any(bboxes(:,1) >= width(data{idx,1})) || any(bboxes(:,2) >= height(data{idx,1}))
        filename = info{1,1}.Filename;
        error(message('vision:ObjectDetector:trainingDataBboxOutsideBounds', filename));
    end
end


%==========================================================================
function data = clampTableBoundingBoxesIfNeeded(data)
% Check if Boxes are contained within the image frame for table input.
% If the width or height extends outside frame, clamp it to the image size
% and display a warning.

if ~isempty(data)
    fileNames = data{:,1};
    for i = 1:size(data,1)
        imgInfo = imfinfo(fileNames{i});
        w = imgInfo.Width;
        h = imgInfo.Height;
        filename = imgInfo.Filename;

        validIndices = logical([0 cellfun(@(x) ~isempty(x), data{i,2:end})]);

        if ~all(validIndices == 0)
            % Check that no bounding box starting coordinate is located outside the
            % image frame
            bboxLoc = cellfun(@(x) [x(:,1) x(:,2)], data{i,validIndices}, UniformOutput=false);
            if any(cellfun(@(x) any(x(:,1) >= w), bboxLoc)) || ...
                    any(cellfun(@(x) any(x(:,2) >= h), bboxLoc))
                error(message('vision:ObjectDetector:trainingDataBboxOutsideBounds', filename));
            end

            data{i,validIndices} = cellfun(@(x) vision.internal.detector.clipBBox(x, [h w]),...
                data{i,validIndices}, UniformOutput=false);
        end

    end
end

%==========================================================================
function data = checkTableIsValid(data)
% Check if bounding boxes can be vertically concatenated
vertcat(data{:, 2}{:});

% Check if bounding boxes are all empty
if all(cellfun(@(x) isempty(x(:)), data{:,2}))
    error(message('vision:ObjectDetector:trainingDataHasNoBoxes'));
end

% Check if table can be converted into datastore
imageDatastore(data{:,1});
boxLabelDatastore(data(:,2));

%==========================================================================
function bboxes = clampDSBoundingBoxesIfNeeded(data, i)
% Check if Boxes are contained within the image frame for datastore input.
% If the width or height extends outside frame, clamp it to the image size
% and display an warning.

bboxes = data{i,2};
w = size(data{i,1},2);
h = size(data{i,1},1);

% Find non-empty bbox indices.
nonEmptyIndices = find(~arrayfun(@isempty,all(bboxes,2)));
if ~isempty(nonEmptyIndices)
    bboxes(nonEmptyIndices,:) = vision.internal.detector.clipBBox(bboxes(nonEmptyIndices,:), [h w]);
end

%==========================================================================
function throwDetailedError(msgID, info, dsRecordIdx)
exp = vision.internal.detector.ObjectDetectorDatastoreException(...
    msgID, info, dsRecordIdx);
throw(exp);

%==========================================================================
function trainingData = prepareTrainingDatastore(trainingData)

if istable(trainingData)
    % Create image and box datastores from the table data. Only the second
    % column from the table is used for training as ACF is a single class
    % detector. The remainder is ignored.
    imds = imageDatastore(trainingData{:,1});
    blds = boxLabelDatastore(trainingData(:,2));
    trainingData = combine(imds,blds);

else % datastore
    % Copy and reset input datastore to prevent modifying input datastore,
    % which is a handle object.
    trainingData = copy(trainingData);
    reset(trainingData);

    % Attach validation transform on the output of the datastore.
    trainingData = transform(trainingData, ...
        @checkTrainingDataDuringTraining, 'IncludeInfo', true);
end

%==========================================================================
function name = modelNameFromTrainingData(trainingData)
if istable(trainingData)
    % Ignore any other columns.
    name = trainingData.Properties.VariableNames{2};

else % datastore
    out = preview(trainingData);
    out = iConvertToCellIfNeeded(out);
    if size(out,2) > 2
        % Use label data as category name.
        cats = categories(out{1,3});
        if numel(cats) > 1
            error(message('vision:ObjectDetector:tooManyClasses'));
        end
        name = char(cats(1));
    else
        % No label provided. Set to '';
        name = '';
    end
end

%==========================================================================
function [bboxes, numImages, trainingData] = boxesAndNumOfImagesFromTrainingData(trainingData)
if istable(trainingData)
    % Check bounding boxes and clamp to image frame for table trainingData inputs
    trainingData = clampTableBoundingBoxesIfNeeded(trainingData);
    bboxes = trainingData{:, 2};

else % datastore
    % Copy and reset datastore to prevent modifying input datastore, which
    % is a handle.
    ds = copy(trainingData);
    reset(ds);

    % Apply a validation transform to ensure box data is valid.
    tds = transform(ds, @checkTrainingDataDuringTraining, 'IncludeInfo', true);

    % Apply a selection transform to read out just the box data.
    tds = transform(tds, @(data,info)deal(data(:,2),info), 'IncludeInfo', true);

    % Read all the boxes out of the datastore.
    try
        bboxes = readall(tds);
    catch ME
        throw(ME.cause{1});
    end
end

% Count the number of observations and vertcat all boxes together.
numImages = size(bboxes,1);
bboxes = vertcat(bboxes{:});

if isempty(bboxes)
    error(message('vision:ObjectDetector:trainingDataHasNoBoxes'));
end

%==========================================================================
function tf = checkObjectTrainingSize(objectTrainingSize)
if (ischar(objectTrainingSize) || isstring(objectTrainingSize))
    validatestring(objectTrainingSize, {'Auto'}, mfilename, ...
        'ObjectTrainingSize');
else
    validateattributes(objectTrainingSize,...
        {'numeric'},...
        {'nonempty', 'real', 'nonsparse', 'vector', 'integer', ...
        'positive', 'numel', 2, '>=', 8},...
        mfilename, 'ObjectTrainingSize');
end
tf = true;

%==========================================================================
function checkMaxWeakLearners(maxWeakLearners, numStages)
validateattributes(maxWeakLearners,{'double'},...
    {'real','positive','integer','nonsparse'},mfilename,'MaxWeakLearners');

if ~isscalar(maxWeakLearners)
    validateattributes(maxWeakLearners,{'double'},...
        {'real','positive','integer','vector','nonsparse','numel',numStages},...
        mfilename,'MaxWeakLearners');
end

%==========================================================================
function data = iConvertToCellIfNeeded(data)
if istable(data)
    data = table2cell(data);
elseif isnumeric(data)
    data = {data};
end

% LocalWords:  ACF grayscale nd acf visiondata datastores imds blds bboxes RCNN Labeler mrg
% LocalWords:  decorrelation nonsparse Adaboost jittering Bbox
