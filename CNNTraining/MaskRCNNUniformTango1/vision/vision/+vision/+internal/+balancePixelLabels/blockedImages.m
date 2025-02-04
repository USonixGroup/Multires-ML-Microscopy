function locationSet = blockedImages(blockedLabeledImages, blockSize, numObservations, varargin)
% blockedImages  Internal function to compute blockLocationSet object
% representing the block locations in the corresponding
% blockedLabeledImages.

%   Copyright 2020 The MathWorks, Inc.

params = parseNameValuePairs(varargin{:});

useParallelFlag = params.UseParallel;

% The levels specified in 'Levels' Name-Value pair must be further validated
levels = params.Levels;

[blockedLabeledImages, levels, blockSize, numObservations] = validateInputs(blockedLabeledImages, levels, blockSize, numObservations);

[classNames, pixelLabelIDs] = getClassesAndPixelLabelIDs(blockedLabeledImages, levels, params);

% Divide each image into macro blocks and compute the macro block pixel counts
[macroBlockPixelCounts, macroBlockSize] = computeMacroBlockPixelCounts(blockedLabeledImages, classNames, pixelLabelIDs, levels, blockSize, useParallelFlag);

% Group the macro block pixel counts for each class and sort them in
% descending order of macro blocks with the most pixels for a particular
% class.
[classBags, classBagsIndices] = vision.internal.balancePixelLabels.groupByClassAndSortMacroBlockPixelCounts(macroBlockPixelCounts, classNames);

% Calculate aggregate pixel counts for all images in the dataset using the
% counts for all macro blocks.
pixelLabelCounts = vision.internal.balancePixelLabels.calculatePopulationCounts(classNames, macroBlockPixelCounts);

imageNumberSet = zeros(numObservations,1);
blockOriginSet = zeros(numObservations,size(macroBlockSize,2));
for blockIdx = 1:numObservations
    desiredClassWeights = vision.internal.balancePixelLabels.calculateClassWeights(pixelLabelCounts);
    desiredClassWeights = vision.internal.balancePixelLabels.downvoteClassesWithZeroPixelLabelCounts(desiredClassWeights);
    
    % Randomly select a class most likely to require balancing and for that
    % class randomly find the macro block with the likelihood of containing
    % most pixels of the chosen class.
    bagIndex = vision.internal.balancePixelLabels.weightedRandomSelectionOfClassBagIndex(desiredClassWeights);
    
    %  macroBlockBag is the randomly chosen classBag with numMacroblocks-by-3 entries where
    %       Column 1 - Image Number corresponding to the macroblock
    %       Column 2 - Pixel counts of the class corresponding to bagIndex for a macroblock
    %       Column 3 - Macroblock origin (1-by-numDimsOfBlockedImage)
    macroBlockBag = classBags{bagIndex};
    macroBlockWeights = [macroBlockBag{:,2}];
    macroBlockIndex = vision.internal.balancePixelLabels.weightedRandomSelectionOfMacroBlockIndex(macroBlockWeights);
    
    % Update the class weights for all classes based on the selected macro block pixel counts.
    statsIdx = classBagsIndices{bagIndex}(macroBlockIndex);
    pixelLabelCounts = vision.internal.balancePixelLabels.updatePixelLabelCountsBasedOnSelectedMacroBlock(pixelLabelCounts, macroBlockPixelCounts, statsIdx);
    
    % Select a random block location within the selected macro block
    imageNumber = macroBlockBag{macroBlockIndex,1};
    macroBlockOrigin = macroBlockBag{macroBlockIndex,3};
    
    blockOrigin = selectBlockOriginFromUniformRange(macroBlockOrigin, macroBlockSize(imageNumber,:), blockSize, blockedLabeledImages(imageNumber));
    
    % Convert all block origins from world to intrinsic co-ordinates
    blockOrigin = world2sub(blockedLabeledImages(imageNumber), blockOrigin, 'level', levels(imageNumber));
    
    % Add image number and block origin of the selected block to the blockSet
    imageNumberSet(blockIdx) = imageNumber;
    blockOriginSet(blockIdx,:) = blockOrigin;
    
end

% Flip the spatial co-ordinates which are in [row, column, ...] format as
% blockLocationSet expects location to be specified in [X, Y, ...] format
blockOriginSet(:,1:2) = fliplr(blockOriginSet(:,1:2));

% Output a blockLocationSet object with information on the selected blocks.
% For images with dimensions beyond the number of dimensions specified in
% blockSize, extend effective blockSize to the image size, for channels
% beyond the number of dimensions specified in blockSize.
effectiveBlockSize = [blockSize blockedLabeledImages(1).Size(numel(blockSize)+1:end)];
locationSet = blockLocationSet(imageNumberSet, blockOriginSet, effectiveBlockSize, levels);

end

function blockOriginOut = selectBlockOriginFromUniformRange(macroBlockOrigin,macroBlockSize,blockSize,blockedLabeledImage)
% selectBlockOriginFromUniformRange  The function selects a block of size,
% blockSize at a random location within a macro block of size,
% macroBlockSize, located at macroBlockOrigin. The spatial reference in
% blockedLabeledImage is used to determine the extents of the image
% containing the macroBlock.

blockOriginOut = macroBlockOrigin;

for dimIdx = 1:numel(macroBlockOrigin)
    blockStart = macroBlockOrigin(dimIdx);
    if dimIdx <= numel(blockSize)
        blockSizeEffective = blockSize(dimIdx);
    else
        blockSizeEffective = macroBlockSize(dimIdx);
    end
    blockEnd = round(min(blockStart + macroBlockSize(dimIdx),blockedLabeledImage.WorldEnd(dimIdx)) - blockSizeEffective);
    
    span = blockEnd - blockStart;
    blockOriginOut(dimIdx) = floor(blockStart + span*rand);
end

end

function stats = computeMacroBlockHistograms(block, classNames, pixelLabelIDs)

if iscategorical(block.Data)
    classCounts = countcats(block.Data(:));
else
    classCounts = arrayfun(...
        @(x) nnz(block.Data == x),...
        pixelLabelIDs);
end

% Combine duplicate classes if any
[uniqueClasses, ~, indc] = unique(classNames, 'stable');
if numel(uniqueClasses) ~= numel(classNames)
    uniqueCounts = zeros(numel(uniqueClasses),1);
    for cInd = 1:numel(uniqueClasses)
        uniqueCounts(cInd) = sum(classCounts(indc==cInd));
    end
    classCounts = uniqueCounts;
end

stats.classCounts = classCounts';

end

function imageMacroBlockPixelCounts = moveMacroBlocksWithinImageExtents(imageMacroBlockPixelCountsBI, blockedImages, level, macroBlockSize, imgIdx)
% moveMacroBlocksWithinImageExtents The function moves any macro blocks
% which are outside image extents to within. The origin of macro blocks
% which are partially within the image such as those at the image
% boundaries are moved such that those macro blocks are completely within
% the image.

% Create 2 datastores - one for obtaining macroblock stats and other for
% obtaining macroblock location.
imageMacroBlockPixelCountsStatsDS = blockedImageDatastore(imageMacroBlockPixelCountsBI);
imageMacroBlockLocationDS = blockedImageDatastore(blockedImages, ...
    'BlockSize', macroBlockSize);
imageMacroBlockPixelCounts = cell(imageMacroBlockPixelCountsStatsDS.TotalNumBlocks,3);
if (imageMacroBlockPixelCountsStatsDS.TotalNumBlocks ~= imageMacroBlockLocationDS.TotalNumBlocks)
    error(message('vision:balancePixelLabels:mismatchedTotalNumBlocks'));
end

idx = 1;
while hasdata(imageMacroBlockPixelCountsStatsDS)
    % Get macroblock pixel counts
    [data, ~] = read(imageMacroBlockPixelCountsStatsDS);
    % Get macroblock location
    [~, info] = read(imageMacroBlockLocationDS);
    imageMacroBlockPixelCounts{idx,1} = imgIdx;
    imageMacroBlockPixelCounts{idx,2} = data{1}.classCounts;
    imageMacroBlockPixelCounts{idx,3} = info.Start;
    
    tileEnd = imageMacroBlockPixelCounts{idx,3} + macroBlockSize - 1;
    for dimIdx = 1:numel(tileEnd)
        if tileEnd(dimIdx) > blockedImages.WorldEnd(level,dimIdx)
            newMacroBlockOrigin = imageMacroBlockPixelCounts{idx,3}(dimIdx) - macroBlockSize(dimIdx) + 1;
            if newMacroBlockOrigin >= blockedImages.WorldStart(level,dimIdx)
                imageMacroBlockPixelCounts{idx,3}(dimIdx) = newMacroBlockOrigin;
            end
        end
    end
    
    idx = idx + 1;
end

end

function macroBlockSize = getMacroBlockSize(bigLabeledImage, level, blockSize)
% getMacroBlockSize Set the macro block size based on an adhoc factor

imageSize = bigLabeledImage.Size(level,:);

numElemsBlockSize = numel(blockSize);

if all(arrayfun(@(x,y)(floor(x/y) >= 2),imageSize(1:numElemsBlockSize), blockSize))
    factor = 2;
else
    factor = 1;
end

% Apply the factor to only the spatial dimensions
macroBlockSize = imageSize;
macroBlockSize(1:numElemsBlockSize) = factor*blockSize;

end

function params = parseNameValuePairs(varargin)

% Convert string inputs to character vectors.
args = matlab.images.internal.stringToChar(varargin);

parser = inputParser();
parser.FunctionName = 'balancePixelLabels';
parser.CaseSensitive = false;
parser.PartialMatching = true;
parser.KeepUnmatched = false;
parser.addParameter('UseParallel', false, @validateUseParallel)
parser.addParameter('Classes', {}, @checkClassNames)
parser.addParameter('PixelLabelIDs', [], @checkPixelLabelID)
parser.addParameter('Levels', 1, @checkLevels)
parser.parse(args{:});

params = parser.Results;

end

function tf = validateUseParallel(useParallel)

isLogicalScalar(useParallel);
if useParallel && ~matlab.internal.parallel.isPCTInstalled()
    error(message('vision:balancePixelLabels:couldNotOpenPool'))
else
    tf = true;
end
end

function tf = isLogicalScalar(input)

validateattributes(input, {'numeric', 'logical'}, {'scalar','finite'})
tf = true;

end

function tf = checkLevels(levels)

% Levels need further validation which is done later.
validateattributes(levels, images.internal.iptnumerictypes, ["integer","positive"])
tf = true;

end

function [macroBlockPixelCounts, macroBlockSize] = computeMacroBlockPixelCounts(blockedLabeledImages, classNames, pixelLabelIDs, levels, blockSize, useParallelFlag)
% computeMacroBlockPixelCounts Computes pixel counts per class for all the
% macroblocks in all the images.
%
% macroBlockPixelCounts - (numImages x numMacroBlocksPerImage)-by-3
%                         Column 1 - Image Number corresponding to the macroblock
%                         Column 2 - Pixel counts per class of the macroblock (1-by-numClasses)
%                         Column 3 - Macroblock origin (1-by-numDimsOfBlockedImage)
% macroBlockSize        - numImages-by-numDimsOfBlockedImage

numImages = numel(blockedLabeledImages);
macroBlockPixelCounts = {};
macroBlockSize = [];
% Calculate macro block pixel counts for all images
for imgIdx = 1:numImages
    % Calculate macro block size for each image.
    macroBlockSize(imgIdx,:) = getMacroBlockSize(blockedLabeledImages(imgIdx), levels(imgIdx), blockSize); %#ok<AGROW>
    % Compute the pixel counts for each class for all macro blocks
    imageMacroBlockPixelCountsBI = apply(blockedLabeledImages(imgIdx),@(x)computeMacroBlockHistograms(x, classNames, pixelLabelIDs),...
        'Level', levels(imgIdx), 'BlockSize', macroBlockSize(imgIdx,:),...
        'UseParallel', useParallelFlag, 'DisplayWaitbar', false);
    % Update the origin of macro blocks in an image such that their extents are completely within the image extents
    imageMacroBlockPixelCounts = moveMacroBlocksWithinImageExtents(imageMacroBlockPixelCountsBI,blockedLabeledImages(imgIdx),levels(imgIdx),macroBlockSize(imgIdx,:), imgIdx);
    % Append pixel counts for macro blocks for all images
    macroBlockPixelCounts = [macroBlockPixelCounts;imageMacroBlockPixelCounts]; %#ok<AGROW>
    
end

end

function validateCategoricalInputLabeledImages(blockedLabeledImages, levels)

if strcmpi(blockedLabeledImages(1).ClassUnderlying(levels(1)), "categorical")
    if any(arrayfun(@(x,y)~strcmpi(x.ClassUnderlying(y),"categorical"),blockedLabeledImages, levels))
        error(message('vision:balancePixelLabels:allImagesMustBeCategorical'));
    end
    
    classes = categories(blockedLabeledImages(1).InitialValue);
    
    if any(arrayfun(@(x)~isequal(sort(categories(x.InitialValue)),sort(classes)),blockedLabeledImages))
        error(message('vision:balancePixelLabels:allImagesMustHaveSameCategories'));
    end
end
end

function validateNumericalLogicalInputLabeledImages(blockedLabeledImages, levels)

if any(strcmpi(blockedLabeledImages(1).ClassUnderlying(levels(1)), images.internal.iptnumerictypes))
    if any(arrayfun(@(x,y)~any(strcmpi(x.ClassUnderlying(y),images.internal.iptnumerictypes)),blockedLabeledImages, levels))
        error(message('vision:balancePixelLabels:allImagesMustBeNumericalOrLogical'));
    end
end
end

function [classNames, pixelLabelIDs] = getClassesAndPixelLabelIDs(blockedLabeledImages, levels, params)

if blockedLabeledImages(1).ClassUnderlying == "categorical"
    if ~(isempty(params.Classes) && isempty(params.PixelLabelIDs))
        error(message('vision:balancePixelLabels:classesPixelLabelIDsNotSupported'));
    end
    
    % Validate that all the remaining images are categorical and all of
    % them have the same categories
    validateCategoricalInputLabeledImages(blockedLabeledImages, levels);
    
    classNames = categories(blockedLabeledImages(1).InitialValue);
    classNames = cellstr(classNames);
    checkClassNames(classNames);
    pixelLabelIDs = []; % not used so setting to empty
else
    if isempty(params.Classes) || isempty(params.PixelLabelIDs)
        error(message('vision:balancePixelLabels:classesPixelLabelIDsMustBeSpecified'));
    end
    
    % Validate that all the remaining images are numerical or logical
    validateNumericalLogicalInputLabeledImages(blockedLabeledImages, levels);
    
    classNames = params.Classes;
    classNames = cellstr(classNames);
    
    pixelLabelIDs = params.PixelLabelIDs;
    
    % convert vector into cell of column vectors.
    if isvector(pixelLabelIDs)
        if numel(classNames) == 1 && isrow(pixelLabelIDs)
            % special case for single class name with scalar
            % leave it as a row.
        else
            pixelLabelIDs = reshape(pixelLabelIDs,[],1);
        end
    else
        pixelLabelIDs = double(pixelLabelIDs);
    end    
    
    if numel(classNames) ~= numel(pixelLabelIDs)
        error(message('vision:balancePixelLabels:invalidNumelClassesLabels'));
    end
    
end

end

%--------------------------------------------------------------------------
function checkClassNames(x)

if iscell(x)
    validateattributes(x, {'cell'}, {'vector', 'nonempty'},...
        '', 'Classes');
    
    if ~iscellstr(x)
        error(message('vision:balancePixelLabels:invalidClassNamesType'));
    end
else
    % char or string array
    if ischar(x)
        validateattributes(x, {'string','char'}, ...
            {'row', 'nonempty'}, '', 'classNames');
    else
        validateattributes(x, {'string'}, ...
            {'vector', 'nonempty'}, '', 'classNames');
    end
end

end
%--------------------------------------------------------------------------
function checkPixelLabelID(pxid)

checkVectorPixelLabelID(pxid);

c = unique(pxid, 'rows');
if size(pxid,1) ~= size(c,1)
    error(message('vision:balancePixelLabels:pxidDuplicateIDs'));
end

end

%--------------------------------------------------------------------------
function checkVectorPixelLabelID(pxid)
validateattributes(pxid, {'numeric'}, ...
    {'integer', 'vector', 'nonempty', 'finite', 'real', 'nonsparse'}, ...
    '', 'PixelLabelIDs');
end

function [blockedLabeledImages, levels, blockSize, numObservations] = validateInputs(blockedLabeledImages, levels, blockSize, numObservations)

if isscalar(levels)
    % make it equal to number of images
    levels = repmat(levels, [1 numel(blockedLabeledImages)]);
else
    % Vector of levels
    if isscalar(blockedLabeledImages)
        % replicate single image to match number of levels
        blockedLabeledImages = repmat(blockedLabeledImages, [1 numel(levels)]);
    end
end

% At this point, numel(images)==numel(levels)
numImages = numel(blockedLabeledImages);
validateattributes(levels, images.internal.iptnumerictypes, {"integer", "positive", "vector", "numel", numImages}, '', "Levels")

% Each level should be valid for its corresponding bigimage
for idx = 1:numel(levels)
    numLevels = blockedLabeledImages(idx).NumLevels;
    validateattributes(levels(idx), images.internal.iptnumerictypes, {'<=', numLevels}, '', "Levels");
end

levels = double(levels);

validateattributes(blockSize, images.internal.iptnumerictypes, ["integer", "positive", "vector"], '', "blockSize", 2)

% Scalar block sizes are expanded to 2-D only.
if isscalar(blockSize)
    blockSize = [blockSize blockSize];
end

% Image size as the specified level for all images must be larger than
% block size
numelBlockSize = numel(blockSize);
firstImageSize = blockedLabeledImages(1).Size(levels(1),:);
numelFirstImageSize = numel(firstImageSize);

for idx = 1:numel(levels)
    imageSize = blockedLabeledImages(idx).Size(levels(idx),:);
    numelImageSize = numel(imageSize);
    
    if  numelFirstImageSize ~= numelImageSize
        error(message('vision:balancePixelLabels:allImagesMustHaveSameNumDims',numelFirstImageSize, numelImageSize, idx));
    end
    
    if numelBlockSize > numelImageSize
        error(message('vision:balancePixelLabels:blockSizeNumDimsLargerThanImageSizeNumDims',numelBlockSize, numelImageSize, idx));
    end
    
    
    imageSizeForNumelBlockSize = imageSize(1:numelBlockSize);
    if any(blockSize > imageSizeForNumelBlockSize)
        error(message('vision:balancePixelLabels:blockSizeLargerThanImageSize',regexprep(num2str(blockSize),'\s+','x'), regexprep(num2str(imageSizeForNumelBlockSize),'\s+','x'), idx));
    end
    
    % Images must have the same size for dimensions beyond
    % numel(blockSize). This restriction is enforced because we implicitly
    % expand blockSize to the extra dimensions in the input images. Image
    % sizes can be different for the dimensions for which blockSize is
    % fully specified.
    if numelFirstImageSize > numelBlockSize
        extraDims = numelBlockSize+1:numelFirstImageSize;
        imageSizeForExtraDims = imageSize(extraDims);
        firstImageSizeForExtraDims = firstImageSize(extraDims);
        if any(imageSizeForExtraDims ~= firstImageSizeForExtraDims)
            firstMismatchedDimStr = num2ordinal(extraDims(~(imageSizeForExtraDims == firstImageSizeForExtraDims)));
            error(message('vision:balancePixelLabels:expectSameSizeImagesForDimsBeyondNumelBlockSize', numelBlockSize, firstMismatchedDimStr, regexprep(num2str(imageSize),'\s+','x'), idx));
        end
    end
    
end

validateattributes(numObservations, images.internal.iptnumerictypes, ["integer", "positive", "scalar"], '', "numObservations", 3)

end
