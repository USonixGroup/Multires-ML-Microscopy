function locationSet = balanceBoxLabels(varargin)

% Copyright 2020-2023 The MathWorks, Inc.

    % Algorithm preamble: The balancing of the box labels
    % works by oversampling the infrequent classes. The box labels are
    % counted across the dataset and sorted based on each class count. Each
    % image size is split into several quadrants based on the
    % NumQuadTreeLevels option. The algorithm randomly picks several blocks
    % within each quadrant that has infrequent classes relatively. The blocks
    % without any objects are discarded. The balancing stops once the
    % specified number of blocks are selected.

    try
        % Wrap in a private function to cut stack trace.
        params = iParseInputs(varargin{:});
    catch me
        throw(me)
    end

    stats = params.Stats;
    % Remove the labels with zero count.
    stats(stats.Count == 0,:) = [];
    stats.Label = removecats(stats.Label);

    if isempty(stats)
        error(message('vision:balanceBoxLabels:invalidBoxLabels'));
    end

    % Balance classes using inverse frequency weights.
    totalBoxes = sum(stats.Count);
    frequency = stats.Count / totalBoxes;
    stats.Weights = 1./frequency;
    params.Stats = stats;

    params.Printer = vision.internal.MessagePrinter.configure(params.Verbose);
    if params.bigImagesSpecified
        params.ImageSizes = iGetBigImageSizes(params.images,params.levels);
    else
        params.ImageSizes = iGetBlockedImageSizes(params.images,params.levels);
    end
    params.NumImages = size(params.ImageSizes,1);

    params.Printer.printMessage('vision:balanceBoxLabels:balancingBoxesHeader',params.NumImages,totalBoxes);

    % Step 1
    allLabelsTbl = iCollectBlocksFromImages(params);

    % Step 2
    locationSet = iCollectObservationsFromBlocks(params,allLabelsTbl);

    params.Printer.printMessage('vision:balanceBoxLabels:balancingBoxesFooter');
end

%--------------------------------------------------------------------------
function allLabelsTbl = iCollectBlocksFromImages(params)

    numLevels = params.InternalOptions.NumQuadTreeLevels;

    allBoxes = params.AllBoxes;
    allLabels = params.AllLabels;

    params.Printer.printMessage('vision:balanceBoxLabels:step1ImageProgress');
    % Start the console wait bar for Step 1.
    waitBar = iConsoleWaitBar(params.NumImages,params.Verbose);
    cleanupObj = onCleanup(@() stop(waitBar));

    % Group windows from each image for each class.
    grouper = vision.internal.bbox.quadTreeSampling.PerClassGrouper(params.Stats.Label);
    for ii = 1:params.NumImages

        imSize = params.ImageSizes(ii,:);

        bboxes = allBoxes{ii};
        labels = allLabels{ii};

        % QuadTree objects each image.
        bboxQT = vision.internal.bbox.quadTreeSampling.PerClassBboxQuadTree(imSize,...
            params.blockSize,bboxes,labels,numLevels,params.Stats,params.OverlapThreshold);
        % Create quad tree nodes as per InternalOptions.NumQuadTreeLevels.
        createQuadTreeNodes(bboxQT);

        imageNumber = ii;
        levelNumber = params.levels(ii);

        % Create Windows and NumBoxesRatio metric for each class
        % from all the quad-tree nodes for an image.
        createWindowsWithMetric(bboxQT,imageNumber,levelNumber,grouper);
        iUpdateConsoleWaitBar(waitBar,params.Verbose);
    end

    allLabelsTbl = table(grouper.UniqueLabels,grouper.Windows,grouper.NumBoxesRatio);
    allLabelsTbl.Properties.VariableNames = {'Label','Windows','NumBoxesRatio'};
end

%--------------------------------------------------------------------------
function locationSet = iCollectObservationsFromBlocks(params,allLabelsTbl)
    params.Printer.printMessage('vision:balanceBoxLabels:step2observationProgress');
    % Start the console wait bar for Step 2.
    waitBar = iConsoleWaitBar(params.numObservations,params.Verbose);
    cleanupObj = onCleanup(@() stop(waitBar));

    bboxes = params.AllBoxes;
    labels = params.AllLabels;

    [~,sorted] = sortrows(params.Stats,"Count");

    allLabelsTbl = allLabelsTbl(sorted,:);
    allLabelsTbl.Weights = params.Stats.Weights(sorted);

    locationSet = allLabelsTbl;

    originalNumObservations = params.numObservations;
    % The first time we sample blocks, increase the numObservations to the multiple factor,
    % so we get enough samples based on rarer classes. Selecting blockSets algorithm retrieves
    % lesser blocks than specified, because blocks are chosen randomly from windows where sometimes
    % blocks do not contain objects (which needs to be removed).
    params.numObservations = round(params.InternalOptions.NumObservationsMultiple*params.numObservations);
    overlapThreshold = params.OverlapThreshold;

    variableBlocksPerClass = false;
    blocksPerWindow = params.InternalOptions.BlocksPerWindow;
    locationSet = vision.internal.bbox.quadTreeSampling.utils.selectBlockSetsFromWindows(locationSet,...
        params.blockSize,bboxes,labels,params.numObservations,...
        variableBlocksPerClass,overlapThreshold,blocksPerWindow);
    params.numObservations = originalNumObservations;

    currNumObservations = sum(cellfun(@(x)size(x,1),locationSet.BlockSets));
    iter = 1;
    prevBlocksPerClass = [];

    iUpdateConsoleWaitBar(waitBar,params.Verbose,currNumObservations);

    while currNumObservations < params.numObservations
        % Sample until we reach a point where we need to duplicate blocks.
        iter = iter + 1;

        % Sort by label count again, so we pick from infrequent labels.
        locationSet = sortrows(locationSet,"LabelWindowCount");
        blocksPerClass = iFindBlocksPerClass(params.numObservations - currNumObservations,locationSet);
        if isequal(prevBlocksPerClass,blocksPerClass)
            % Case when there's no convergence of finding boxes in a set of windows.
            if params.InternalOptions.DuplicateBlocks
                locationSet = iDuplicateBlockSetsUsingBlocksPerClass(locationSet,blocksPerClass);
                currNumObservations = sum(cellfun(@(x)size(x,1),locationSet.BlockSets));
                iUpdateConsoleWaitBar(waitBar,params.Verbose,currNumObservations);
            end
            break;
        end
        prevBlocksPerClass = blocksPerClass;

        variableBlocksPerClass = true;
        locationSet = vision.internal.bbox.quadTreeSampling.utils.selectBlockSetsFromWindows(locationSet,...
            params.blockSize,bboxes,labels,blocksPerClass,...
            variableBlocksPerClass,overlapThreshold,blocksPerWindow);
        currNumObservations = sum(cellfun(@(x)size(x,1),locationSet.BlockSets));
        iUpdateConsoleWaitBar(waitBar,params.Verbose,currNumObservations);
    end
    params.CurrentNumObservations = currNumObservations;

    locationSet = iConstructBlockLocationSet(locationSet,params);
end

%--------------------------------------------------------------------------
function locationSet = iConstructBlockLocationSet(locationSet,params)
    numToReduce =  params.CurrentNumObservations - params.numObservations;

    while numToReduce > 0
        % Sort descending, so we pick more common classes to remove.
        newLocationSet = sortrows(locationSet,"LabelWindowCount","descend");
        blocksPerClass = iFindBlocksPerClass(numToReduce,newLocationSet);

        nonZeroBlocks = find(blocksPerClass ~= 0);
        blockSets = newLocationSet.BlockSets;
        isEmptyBlockSets = cellfun(@isempty,blockSets);
        emptyBlockSets = find(isEmptyBlockSets);
        if all(ismember(nonZeroBlocks,emptyBlockSets))
            % If there are no blocks where we need to remove blocks,
            % then break.
            break;
        end
        nonEmptyBlockSets = find(~isEmptyBlockSets);
        for ii = 1:numel(nonEmptyBlockSets)
            cBlockSets = blockSets{nonEmptyBlockSets(ii)};
            if blocksPerClass(ii) <= 0
                continue;
            end
            sz = size(cBlockSets,1);
            remPerClass = blocksPerClass(ii);
            idx = 1:min(sz,remPerClass);
            cBlockSets(idx,:) = [];
            blockSets{ii} = cBlockSets;
        end
        newLocationSet.BlockSets = blockSets;
        locationSet = newLocationSet;
        params.CurrentNumObservations = sum(cellfun(@(x)size(x,1),locationSet.BlockSets));
        numToReduce = params.CurrentNumObservations - params.numObservations;
    end
    blockLocations = vertcat(locationSet.BlockSets{:});
    if numToReduce < 0
        numToReduce = abs(numToReduce);
        if params.InternalOptions.DuplicateBlocks && size(blockLocations,1) >= numToReduce
            blockLocations = vertcat(blockLocations(1:numToReduce,:), blockLocations);
        end
    end

    % We need image numbers, levels, and [x,y] locations.
    if isempty(blockLocations)
        imageNumber = [];
        blockOrigin = [];
    else
        % blockLocations is a N-by-6 matrix. 
        % - col 1 contains the image number
        % - col 2 contains the level number
        % - col 3:4 contains the block origin
        % - col 5:6 contains the block size
        imageNumber = blockLocations(:,1);
        blockOrigin = blockLocations(:,3:4);
       
    end

    if params.bigImagesSpecified
        blockSize = params.blockSize;
    else
        % blockedImage stores the complete dimension information. We may
        % need to fill in the missing values.
        blockSize = iAddTrailingBlockSizeDimsIfNeeded(...
            params.blockSize, params.images, params.levels);

        blockOrigin = iAddTrailingBlockOriginsIfNeeded(...
            blockOrigin, blockSize);
    end

    locationSet = blockLocationSet(imageNumber,blockOrigin,blockSize,params.levels);
end

%--------------------------------------------------------------------------
function locationSet = iDuplicateBlockSetsUsingBlocksPerClass(locationSet,blocksPerClass)
    nonZeroBlocks = find(blocksPerClass ~= 0);
    blockSets = locationSet.BlockSets;
    isEmptyBlockSets = cellfun(@isempty,blockSets);
    emptyBlockSets = find(isEmptyBlockSets);

    if all(ismember(nonZeroBlocks,emptyBlockSets))
        return;
    end
    for ii = 1:numel(nonZeroBlocks)
        idx = nonZeroBlocks(ii);
        blockSets = locationSet.BlockSets{idx};
        numBlocks = size(blockSets,1);
        if numBlocks == 0
            continue;
        end
        rNumBlocks = blocksPerClass(idx);
        if rNumBlocks <= numBlocks
            dupes = blockSets(1:rNumBlocks,:);
            dupeBlockSets = vertcat(blockSets,dupes);
        else
            dupes = repmat(blockSets,floor(rNumBlocks/numBlocks),1);
            m = mod(rNumBlocks,numBlocks);
            dupes1 = blockSets(1:m,:);
            dupeBlockSets = vertcat(dupes,dupes1);
        end
        locationSet.BlockSets{idx} = dupeBlockSets;
    end
end

%--------------------------------------------------------------------------
function blocksPerClass = iFindBlocksPerClass(numObservations,locationSet)
    blocksPerClass = ceil(numObservations*(locationSet.Weights ./ sum(locationSet.Weights)));
    remainder = sum(blocksPerClass) - numObservations;

    % Ignore the case where remainder < 0, since we need more block locations
    % anyway, and we would cut the extraneous block locations at the end.
    if remainder > 0 && ~isempty(blocksPerClass)
        h = numel(blocksPerClass);
        revIdx = h:-1:1;
        revBlocksPerClass = blocksPerClass(revIdx);
        cSums = cumsum(revBlocksPerClass);
        setToZero = cSums < remainder;
        revBlocksPerClass(setToZero) = 0;
        setToZeroLast = find(setToZero,1,'last');
        if ~isempty(setToZeroLast) && setToZeroLast < h && remainder - cSums(setToZeroLast) > 0
            remainder = remainder - cSums(setToZeroLast);
            lastNonZeroValue = revBlocksPerClass(setToZeroLast+1);
            revBlocksPerClass(setToZeroLast + 1) = lastNonZeroValue - remainder;
        end

        blocksPerClass = revBlocksPerClass(revIdx);
    end
end

%--------------------------------------------------------------------------
function blockSize = iAddTrailingBlockSizeDimsIfNeeded(blockSize, bims, levels)
% Add trailing block size dimensions, if needed, so that the block size in
% the location set has the same number of dimensions as the blockedImage.
% This is required to created a blockedImageDatastore from the
% blockLocationSet returned by balanceBoxLabels.
blockSize(end+1:bims(1).NumDimensions) = ...
    bims(1).Size(levels(1), numel(blockSize)+1:end);
end

%--------------------------------------------------------------------------
function blockOrigins = iAddTrailingBlockOriginsIfNeeded(blockOrigins, blockSize)
if ~isempty(blockOrigins)
    numDims = numel(blockSize);
    blockOrigins(:,end+1:numDims) = 1;
end
end

%--------------------------------------------------------------------------
function params = iParseInputs(varargin)
    % Parse and validate the input parameters.
    %

    [varargin{:}] = convertStringsToChars(varargin{:});
    numericOptions = {'blockSize', 'numObservations', 'OverlapThreshold'};

    inputIsBlockedImage = false;
    if nargin > 1 && isa(varargin{2}, 'blockedImage')
        inputIsBlockedImage = true;
        % blockedImages input must contain 3 other inputs: boxLabels,
        % blockSize, and numObservations (and any additional NV pairs).
        if nargin > 4 && ~isequal(mod(nargin,2), 0)
            error(message('vision:balanceBoxLabels:invalidBlockedImageNumInputs'))
        end
    end

    p = inputParser;
    p.addRequired('boxLabels',@iCheckBoxLabels);
    if inputIsBlockedImage
        p.addRequired('blockedImages',@iCheckBlockedImages);
    else
        numericOptions(end+1) = {'bigImageLevels'};
        p.addRequired('bigImages',@iCheckBigImages);
        p.addRequired(numericOptions{4},@(x)iCheckLevels(x,numericOptions{4}));
    end
    p.addRequired(numericOptions{1},@iCheckBlockSize);
    p.addRequired(numericOptions{2},@(x)iCheckScalarNumeric(x,numericOptions{2}));

    p.addParameter('Verbose',true,@(x)iCheckLogicalScalar(x,'Verbose'));
    p.addParameter(numericOptions{3},1,@iCheckOverlapThreshold);
    p.addParameter('Levels',1,@(x)iCheckLevels(x,'Levels'));

    p.addParameter('InternalOptions',iDefaultInternalOptions,@iCheckInternalOptionsFieldnames);

    parse(p, varargin{:});
    params = p.Results;

    if inputIsBlockedImage
        params.images = params.blockedImages;
        params = rmfield(params, "blockedImages");
        params.bigImagesSpecified = false;
        params.levels = params.Levels;
    else
        params.images = params.bigImages;
        params = rmfield(params, "bigImages");
        params.bigImagesSpecified = true;
        if iWasUserSpecified(p,'Levels')
            error(message('vision:balanceBoxLabels:invalidLevelsNVWithBigImages'));
        end
        params.levels = params.bigImageLevels;
    end

    for ii = 1:numel(numericOptions)
        % Convert to double
        params.(numericOptions{ii}) = double(params.(numericOptions{ii}));
    end

    blds = iGetBoxLabelDatastore(params.boxLabels);

    if numel(params.images) ~= size(blds.LabelData,1)
        if inputIsBlockedImage
            error(message('vision:balanceBoxLabels:invalidSizeBlockedImageBoxLabels'));
        else
            error(message('vision:balanceBoxLabels:invalidSizeBigImageBoxLabels'));
        end
    end

    if isscalar(params.levels)
        params.levels = repmat(params.levels,size(params.images));
    elseif numel(params.levels) ~= numel(params.images)
        if inputIsBlockedImage
            error(message('vision:balanceBoxLabels:invalidSizeBlockedImageLevels'));
        else
            error(message('vision:balanceBoxLabels:invalidSizeBigImageLevels'));
        end
    end

    if iWasUserSpecified(p,'InternalOptions')
        params.InternalOptions = iValidateInternalOptions(params.InternalOptions);
    end

    params.Stats = countEachLabel(blds);
    params.AllBoxes = blds.LabelData(:,1);
    params.AllLabels = blds.LabelData(:,2);
end

%--------------------------------------------------------------------------
function opts = iDefaultInternalOptions()
    opts = vision.internal.bbox.balanceBoxLabelsDefaultInternalOptions;
end

%--------------------------------------------------------------------------
function iCheckInternalOptionsFieldnames(x)
    classes = {'struct'};
    attrs = {'nonempty', 'scalar'};
    name = 'InternalOptions';
    validateattributes(x,classes,attrs,mfilename,name);

    opts = iDefaultInternalOptions;
    cellfun(@(str)validatestring(str,fieldnames(opts),mfilename,name),fieldnames(x), ...
        'UniformOutput', false);
end

%--------------------------------------------------------------------------
function opts = iValidateInternalOptions(opts)
    defaultOpts = iDefaultInternalOptions;
    if isfield(opts,'NumQuadTreeLevels')
        opts.NumQuadTreeLevels = iCheckNumQuadTreeLevels(opts.NumQuadTreeLevels);
    else
        opts.NumQuadTreeLevels = defaultOpts.NumQuadTreeLevels;
    end
    if isfield(opts,'NumObservationsMultiple')
        iCheckScalarReal(opts.NumObservationsMultiple, 'NumObservationsMultiple');
        opts.NumObservationsMultiple = double(opts.NumObservationsMultiple);
    else
        opts.NumObservationsMultiple = defaultOpts.NumObservationsMultiple;
    end
    if isfield(opts,'DuplicateBlocks')
        iCheckLogicalScalar(opts.DuplicateBlocks,'DuplicateBlocks');
    else
        opts.DuplicateBlocks = defaultOpts.DuplicateBlocks;
    end

    if isfield(opts,'BlocksPerWindow')
        iCheckScalarNumeric(opts.BlocksPerWindow,'BlocksPerWindow');
        opts.BlocksPerWindow = double(opts.BlocksPerWindow);
    else
        opts.DuplicateBlocks = defaultOpts.DuplicateBlocks;
    end
end

%--------------------------------------------------------------------------
function iCheckLogicalScalar(tf,name)
    classes = {'logical'};
    attrs = {'nonempty', 'scalar'};
    validateattributes(tf,classes,attrs,mfilename,name);
end

%--------------------------------------------------------------------------
function numLevels = iCheckNumQuadTreeLevels(numLevels)
    name = 'NumQuadTreeLevels';
    if ischar(numLevels)
        numLevels = validatestring(numLevels,{'auto'},mfilename,name);
    else
        iCheckScalarNumeric(numLevels,name);
        numLevels = double(numLevels);
    end
end

%--------------------------------------------------------------------------
function imageSizes = iGetBigImageSizes(bigImages,levels)
    imageSizes = arrayfun(@(m,x){m.LevelSizes(x,:)},bigImages,levels);
    imageSizes = vertcat(imageSizes{:});
end

%--------------------------------------------------------------------------
function imageSizes = iGetBlockedImageSizes(blockedImages,levels)
    imageSizes = arrayfun(@(m,x){m.Size(x,:)},blockedImages,levels);
    imageSizes = vertcat(imageSizes{:});
end

%--------------------------------------------------------------------------
function iCheckLevels(x, name)
    levelAttrs = {'vector','integer'};
    iCheckNumeric(x,name,levelAttrs,3);
end

%--------------------------------------------------------------------------
function iCheckBlockSize(x)
    blockSizeAttrs = {'size', [1,2],'integer'};
    iCheckNumeric(x,'blockSize',blockSizeAttrs,2);

end

%--------------------------------------------------------------------------
function iCheckOverlapThreshold(x)
    attrs = {'scalar','real','<=',1};
    iCheckNumeric(x,'OverlapThreshold',attrs);
end

%--------------------------------------------------------------------------
function iCheckScalarReal(x,name)
    attrs = {'scalar','real'};
    iCheckNumeric(x,name,attrs);
end

%--------------------------------------------------------------------------
function iCheckScalarNumeric(x,name)
    attrs = {'scalar','integer'};
    iCheckNumeric(x,name,attrs);
end

%--------------------------------------------------------------------------
function iCheckNumeric(x,name,attrs,varargin)
    classes = {'numeric'};
    attrs = [{'nonempty', 'nonzero',...
        'finite', 'nonnan', 'nonsparse', 'positive'}, attrs{:}];
    validateattributes(x,classes,attrs,mfilename,name,varargin{:});
end


%--------------------------------------------------------------------------
function iCheckBigImages(x)
    classes = {'bigimage'};
    attrs = {'nonempty', 'vector'};
    name = 'bigImages';
    validateattributes(x,classes,attrs,mfilename,name);
end

%--------------------------------------------------------------------------
function iCheckBlockedImages(x)
    classes = {'blockedImage'};
    attrs = {'nonempty', 'vector'};
    name = 'blockedImages';
    validateattributes(x,classes,attrs,mfilename,name);
end


%--------------------------------------------------------------------------
function iCheckBoxLabels(x)
    classes = {'table'};
    attrs = {'nonempty', 'ncols', 2};
    name = 'boxLabels';
    validateattributes(x,classes,attrs,mfilename,name);
end

%--------------------------------------------------------------------------
function blds = iGetBoxLabelDatastore(boxLabels)
    try
        blds = boxLabelDatastore(boxLabels);
    catch cause
        msg = message('vision:balanceBoxLabels:invalidBoxLabels');
        throw(addCause(MException(msg),cause));
    end
end

%--------------------------------------------------------------------------
function waitBar = iConsoleWaitBar(n,verbose)
    enableWaitBar = iNeedConsoleWaitBar(verbose);
    waitBar = vision.internal.ConsoleWaitBar( ...
        n, ...                       % total number of iterations
        "Verbose",enableWaitBar, ... % whether to print or not
        "PrintElapsedTime",1, ...    % print elapsed time
        "PrintRemainingTime",1);     % print estimated time remaining

    waitBar.start();
end

%--------------------------------------------------------------------------
function iUpdateConsoleWaitBar(waitBar,verbose,numIterations)
    if iNeedConsoleWaitBar(verbose) && ~isempty(waitBar)
        if nargin == 2
            numIterations = 1;
        end
        if numIterations >= waitBar.CompletedIterations
            numIterations = numIterations - waitBar.CompletedIterations;
        end
        waitBar.update(numIterations);
    end
end

%--------------------------------------------------------------------------
function tf = iNeedConsoleWaitBar(verbose)
    tf = verbose && isempty(dbstatus);
end

%--------------------------------------------------------------------------
function tf = iWasUserSpecified(parser,param)
    tf = ~ismember(param,parser.UsingDefaults);
end

%   Copyright 2019-2023 The MathWorks, Inc.
