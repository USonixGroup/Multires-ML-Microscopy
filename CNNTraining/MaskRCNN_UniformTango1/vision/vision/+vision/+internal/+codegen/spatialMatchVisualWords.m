% spatialMatchVisualWords is codegen implementation for visionSpatialMatchVisualWords
function outputIndexPairs = spatialMatchVisualWords(wordsIdx1, wordsIdx2, location1, location2, mWords, ...
    nNeighbours, mThreshold)
%#codegen

%   Copyright 2022-2023 The MathWorks, Inc.
numLoc1 = size(location1, 1);
numLoc2 = size(location2, 1);
numMatchingWords = coder.internal.indexInt(numel(mWords));

ndim = 2;

locTree1 = buildTree(location1, numLoc1, ndim);

locTree2 = buildTree(location2, numLoc2, ndim);

outputIndexPairs = zeros(coder.ignoreConst(0), 2, 'uint32');

wordIndexPairs1 = fillWordIndexPair(numLoc1, wordsIdx1);
wordIndexPairs2 = fillWordIndexPair(numLoc2, wordsIdx2);

for i=1:numMatchingWords
    wordId = mWords(i);
    coder.varsize('indices1');
    coder.varsize('indices2');
    indices1 = zeros(1, 0, 'uint32');
    indices2 = zeros(1, 0, 'uint32');
    for j=1:numLoc1
        if wordIndexPairs1(j,1) == wordId
            indices1 = [indices1, wordIndexPairs1(j, 2)];
        end
    end
    for j=1:numLoc2
        if wordIndexPairs2(j,1) == wordId
            indices2 = [indices2, wordIndexPairs2(j, 2)];
        end
    end
    neighborIndices1 = findWordNeighbors(locTree1, indices1, nNeighbours, location1);
    neighborIndices2 = findWordNeighbors(locTree2, indices2, nNeighbours, location2);

    hists1 = createNeighborWordHistograms(wordsIdx1,neighborIndices1);
    hists2 = createNeighborWordHistograms(wordsIdx2,neighborIndices2);

    outputIndexPairs = findBestMatch(hists1, hists2, indices1, indices2, mThreshold+1, outputIndexPairs);
end
end

% -------------------------------------------------------------------------
function locTree = buildTree(location, numLoc, ndim)
% Build KDtree
if coder.internal.isTargetMATLABHost()
    locTree = vision.internal.buildable.kdtreeBuildable.kdtreeConstruct(class(location));
    pLocationHandle = vision.internal.buildable.kdtreeBuildable.kdtreeGetLocationPointer(location, class(location));
    vision.internal.buildable.kdtreeBuildable.kdtreeIndex(locTree, class(location), ...
        pLocationHandle, numLoc, ndim);
else
    locTree = vision.internal.buildable.kdtreeBuildablePortable.kdtreeConstruct(class(location));
    pLocationHandle = vision.internal.buildable.kdtreeBuildablePortable.kdtreeGetLocationPointer(location, class(location));
    vision.internal.buildable.kdtreeBuildablePortable.kdtreeIndex(locTree, class(location), ...
        pLocationHandle, numLoc, ndim);
end
end

% -------------------------------------------------------------------------
function wIndexPairs = fillWordIndexPair(numLoc, wIdx)
% Fill word-index pairs
wIndexPairs=[wIdx(1:numLoc)'; (1:numLoc)]';
end

% -------------------------------------------------------------------------
function idxVector = findWordNeighbors(tree, indices, nNeighbours, location)
% Find neighbors of respective words
searchOpts.eps = 0;
idxVector = coder.nullcopy(zeros(numel(indices), nNeighbours+1, 'uint32'));
numIndices = coder.internal.indexInt(numel(indices));
for i=1:numIndices
    query = [location(indices(i), 1), location(indices(i), 2)];
    if coder.internal.isTargetMATLABHost()
        idx = vision.internal.buildable.kdtreeBuildable.kdtreeKNNSearch(tree, ...
            class(location), query, nNeighbours+1, searchOpts);
    else
        idx = vision.internal.buildable.kdtreeBuildablePortable.kdtreeKNNSearch(tree, ...
            class(location), query, nNeighbours+1, searchOpts);
    end
    idxVector(i, :) = idx(:);
end
end

% -------------------------------------------------------------------------
function hists = createNeighborWordHistograms(words, neighborIndices)
% Create neighbour word histograms
nSize = coder.internal.indexInt(size(neighborIndices, 1));
hists = coder.nullcopy(cell(nSize, 1));
parfor i=1:nSize
    hists{i} = createWordHistogram(words, neighborIndices(i, :));
end
end

% -------------------------------------------------------------------------
function hist = createWordHistogram(words, indices)
% Create word histograms
nIndices = coder.internal.indexInt(numel(indices));
hist = zeros(numel(indices), 2, 'uint32');
for i=1:nIndices
    if ((indices(i) < numel(words)) && (indices(i) ~= 0))
        bin = words(indices(i));
        hist(bin, 1) = bin;
        hist(bin, 2) = hist(bin, 2)+1;
    end
end
end

% -------------------------------------------------------------------------
function outputIndexPairs = findBestMatch(hists1, hists2, indices1, indices2, mThreshold, outputIndexPairs)
% Find best match between indices1 and indices2
nNeighbours1 = numel(hists1);
nNeighbours2 = numel(hists2);

for i=1:nNeighbours1
    h1 = hists1{i};
    count = coder.nullcopy(zeros(nNeighbours2, 1, 'uint32'));

    parfor j=1:nNeighbours2
        count(j) = histIntersection(h1,hists2{j});
    end
    [result, resultIndex] = max(count);

    if result >= mThreshold
        outputIndexPairs = [outputIndexPairs; [indices1(i), indices2(resultIndex)]];
    end
end
end

% -------------------------------------------------------------------------
function dist = histIntersection(h1, h2)
% Find intersection of histograms
dist = uint32(0);
hSize = coder.internal.indexInt(size(h1, 1));
for i=1:hSize
    bin = h1(i, 1);
    [hasBin, idx] = ismember(bin, h2(:, 1));
    if hasBin
        xval = h1(i, 2);
        yval = h2(idx, 2);
        dist = dist+min(xval, yval);
    end
end
end
