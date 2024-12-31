function locationSet = bigImages(bigLabeledImages, levels, blockSize, numObservations, varargin)
% bigImages  Internal function to compute blockLocationSet object
% representing the block locations in the corresponding bigimage label objects.

%   Copyright 2020 The MathWorks, Inc.

[bigLabeledImages, levels, blockSize, numObservations] = validateRequiredInputs(bigLabeledImages, levels, blockSize, numObservations);

useParallelFlag = parseNameValuePairs(varargin{:});

classNames = getClassesAndPixelLabelIDs(bigLabeledImages);

% Divide each image into macro blocks and compute the macro block pixel counts
[macroBlockPixelCounts, macroBlockSize] = computeMacroBlockPixelCounts(bigLabeledImages, levels, blockSize, useParallelFlag);

% Group the macro block pixel counts for each class and sort them in
% descending order of macro blocks with the most pixels for a particular
% class.
[classBags, classBagsIndices] = vision.internal.balancePixelLabels.groupByClassAndSortMacroBlockPixelCounts(macroBlockPixelCounts, classNames);

% Calculate aggregate pixel counts for all images in the dataset using the
% counts for all macro blocks.
pixelLabelCounts = vision.internal.balancePixelLabels.calculatePopulationCounts(classNames, macroBlockPixelCounts);

imageNumberSet = zeros(numObservations,1);
blockOriginSet = zeros(numObservations,2);
for blockIdx = 1:numObservations
    desiredClassWeights = vision.internal.balancePixelLabels.calculateClassWeights(pixelLabelCounts);
    desiredClassWeights = vision.internal.balancePixelLabels.downvoteClassesWithZeroPixelLabelCounts(desiredClassWeights);
    
    % Randomly select a class most likely to require balancing and for that
    % class randomly find the macro block with the likelihood of containing
    % most pixels of the chosen class.
    bagIndex = vision.internal.balancePixelLabels.weightedRandomSelectionOfClassBagIndex(desiredClassWeights);
    macroBlockBag = classBags{bagIndex};
    macroBlockWeights = [macroBlockBag{:,2}];
    macroBlockIndex = vision.internal.balancePixelLabels.weightedRandomSelectionOfMacroBlockIndex(macroBlockWeights);
    
    % Update the class weights for all classes based on the selected macro block pixel counts.
    statsIdx = classBagsIndices{bagIndex}(macroBlockIndex);
    pixelLabelCounts = vision.internal.balancePixelLabels.updatePixelLabelCountsBasedOnSelectedMacroBlock(pixelLabelCounts, macroBlockPixelCounts, statsIdx);
    
    % Select a random block location within the selected macro block
    imageNumber = macroBlockBag{macroBlockIndex,1};
    macroBlockOrigin = macroBlockBag{macroBlockIndex,3};
    
    blockOrigin = selectBlockOriginFromUniformRange(macroBlockOrigin,macroBlockSize(imageNumber,:),blockSize,bigLabeledImages(imageNumber).SpatialReferencing(levels(imageNumber)));
    
    % Convert all block origins from world to intrinsic co-ordinates
    [blockOrigin(1), blockOrigin(2)] = worldToIntrinsic(bigLabeledImages(imageNumber).SpatialReferencing(levels(imageNumber)), blockOrigin(1), blockOrigin(2));
    
    % Add image number and block origin of the selected block to the blockSet
    imageNumberSet(blockIdx) = imageNumber;
    blockOriginSet(blockIdx,:) = blockOrigin;
    
end

% Output a blockLocationSet object with information on the selected blocks
locationSet = blockLocationSet(imageNumberSet, blockOriginSet, blockSize, levels);

end

function stats = computeMacroBlockHistograms(block)

[stats, ~] = histcounts(block);

end

function blockOriginOut = selectBlockOriginFromUniformRange(macroBlockOrigin,macroBlockSize,blockSize,Rlevel)
% selectBlockOriginFromUniformRange  The function selects a block of size,
% blockSize at a random location within a macro block of size,
% macroBlockSize, located at macroBlockOrigin. The spatial reference
% object, Rlevel is used to determine the extents of the image containing
% the macroBlock.

blockOriginOut = [];
blockStartX = macroBlockOrigin(1);
blockEndX = round(min(blockStartX + macroBlockSize(2),Rlevel.XWorldLimits(2)) - blockSize(2));

blockStartY = macroBlockOrigin(2);
blockEndY = round(min(blockStartY + macroBlockSize(1),Rlevel.YWorldLimits(2)) - blockSize(1));

spanX = blockEndX - blockStartX;
spanY = blockEndY - blockStartY;

blockOriginOut(1) = floor(blockStartX + spanX*rand);
blockOriginOut(2) = floor(blockStartY + spanY*rand);

end

function imageMacroBlockPixelCounts = moveMacroBlocksWithinImageExtents(imageMacroBlockPixelCounts, spatialRef, macroBlockSize)
% moveMacroBlocksWithinImageExtents The function moves any macro blocks
% which are outside image extents to within. The origin of macro blocks
% which are partially within the image such as those at the image
% boundaries are moved such that those macro blocks are completely within
% the image.

for idx = 1:size(imageMacroBlockPixelCounts,1)
    tileEnd = imageMacroBlockPixelCounts{idx,2} + fliplr(macroBlockSize)-1;
    if tileEnd(1) > spatialRef.XWorldLimits(2)
        newMacroBlockOriginX = imageMacroBlockPixelCounts{idx,2}(1) - macroBlockSize(2) + 1;
        if newMacroBlockOriginX >= spatialRef.XWorldLimits(1)
            imageMacroBlockPixelCounts{idx,2}(1) = newMacroBlockOriginX;
        end
    end
    
    if tileEnd(2) > spatialRef.YWorldLimits(2)
        newMacroBlockOriginY = imageMacroBlockPixelCounts{idx,2}(2) - macroBlockSize(1) + 1;
        if newMacroBlockOriginY >= spatialRef.YWorldLimits(1)
            imageMacroBlockPixelCounts{idx,2}(2) = newMacroBlockOriginY;
        end
    end
    
end

end

function macroBlockSize = getMacroBlockSize(bigLabeledImages,levels,blockSize)
% getMacroBlockSize Set the macro block size based on an adhoc factor

imageSize = bigLabeledImages.LevelSizes(levels,:);

if ((floor(imageSize(1)/blockSize(1)) >= 2) && ...
        (floor(imageSize(2)/blockSize(2)) >= 2))
    factor = 2;
else
    factor = 1;
end

macroBlockSize = factor*blockSize;

end

function macroBlockPixelCounts = appendImagePixelCountsToOverallPixelCounts(macroBlockPixelCounts, imageMacroBlockPixelCounts, imageNumber)

numMacroBlocks = size(imageMacroBlockPixelCounts,1);

macroBlocksIdx = (1:numMacroBlocks) + numMacroBlocks*(imageNumber-1);
macroBlockPixelCounts(macroBlocksIdx,1) = {imageNumber};
macroBlockPixelCounts(macroBlocksIdx,2:3) = imageMacroBlockPixelCounts;

end

function useParallelFlag = parseNameValuePairs(varargin)

% Convert string inputs to character vectors.
args = matlab.images.internal.stringToChar(varargin);

% Parse remainder of input arguments.
parser = inputParser();
parser.FunctionName = 'balancePixelLabels';
parser.CaseSensitive = false;
parser.PartialMatching = true;
parser.KeepUnmatched = false;
parser.addParameter('UseParallel', false, @validateUseParallel)
parser.parse(args{:});

useParallelFlag = parser.Results.UseParallel;
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

function [bigLabeledImages, levels, blockSize, numObservations] = validateRequiredInputs(bigLabeledImages, levels, blockSize, numObservations)

validateInputLabeledImages(bigLabeledImages);

if isscalar(levels)
    % make it equal to number of images
    levels = repmat(levels, [1 numel(bigLabeledImages)]);
else
    % Vector of levels
    if isscalar(bigLabeledImages)
        % replicate single image to match number of levels
        bigLabeledImages = repmat(bigLabeledImages, [1 numel(levels)]);
    end
end

% At this point, numel(images)==numel(levels)
numImages = numel(bigLabeledImages);
validateattributes(levels, images.internal.iptnumerictypes, {"integer","positive", "vector", "numel", numImages}, '', "levels", 2)

% Each level should be valid for its corresponding bigimage
for idx = 1:numel(levels)
    numLevels = numel(bigLabeledImages(idx).SpatialReferencing);
    validateattributes(levels(idx), images.internal.iptnumerictypes, {'<=', numLevels}, '', "levels", 2);
end

levels = double(levels);

validateattributes(blockSize, images.internal.iptnumerictypes, {"integer","positive", "vector", "size", [1 2]}, '', "blockSize", 3)

% Image size as the specified level for all images must be larger than
% block size
for idx = 1:numel(levels)
    imageSize = bigLabeledImages(idx).SpatialReferencing(levels(idx)).ImageSize;
    if any(blockSize > imageSize)
        error(message('vision:balancePixelLabels:blockSizeLargerThanImageSize',regexprep(num2str(blockSize),'\s+','x'), regexprep(num2str(imageSize),'\s+','x'), idx));
    end
end

validateattributes(numObservations, images.internal.iptnumerictypes, ["integer","positive", "scalar"], '', "numObservations", 4)

end

function validateInputLabeledImages(labeledImages)

if (arrayfun(@(x)~strcmpi(x.ClassUnderlying,'categorical'),labeledImages))
    error(message('vision:balancePixelLabels:allImagesMustBeCategorical'));
end

classes = labeledImages(1).Classes;

if (arrayfun(@(x)~isequal(sort(x.Classes),sort(classes)),labeledImages))
    error(message('vision:balancePixelLabels:allImagesMustHaveSameCategories'));
end
end

function [macroBlockPixelCounts, macroBlockSize] = computeMacroBlockPixelCounts(bigLabeledImages, levels, blockSize, useParallelFlag)

numImages = numel(bigLabeledImages);
macroBlockPixelCounts = {};
macroBlockSize = zeros(numImages,numel(blockSize));

% Calculate macro block pixel counts for all images
for imgIdx = 1:numImages
    % Calculate macro block size for each image.
    macroBlockSize(imgIdx,:) = getMacroBlockSize(bigLabeledImages(imgIdx), levels(imgIdx), blockSize);
    
    % Compute the pixel counts for each class for all macro blocks
    imageMacroBlockPixelCounts = apply(bigLabeledImages(imgIdx),levels(imgIdx),@computeMacroBlockHistograms,'BlockSize',macroBlockSize(imgIdx,:),'UseParallel',useParallelFlag,'DisplayWaitbar',false);
    % Update the origin of macro blocks in an image such that their extents are completely within the image extents
    imageMacroBlockPixelCounts = moveMacroBlocksWithinImageExtents(imageMacroBlockPixelCounts,bigLabeledImages(imgIdx).SpatialReferencing(levels(imgIdx)),macroBlockSize(imgIdx,:));
    % Append pixel counts for macro blocks for all images
    macroBlockPixelCounts = appendImagePixelCountsToOverallPixelCounts(macroBlockPixelCounts, imageMacroBlockPixelCounts, imgIdx);
end

end

function classNames = getClassesAndPixelLabelIDs(bigLabeledImages)

classNames = bigLabeledImages(1).Classes;

end
