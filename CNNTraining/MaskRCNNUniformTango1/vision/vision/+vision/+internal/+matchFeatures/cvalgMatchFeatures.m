function [indexPairs, matchMetric] = cvalgMatchFeatures(features1in, ...
    features2in, metric, ...
    threshold, method, ...
    maxRatioThreshold, isPrenormalized, ...
    uniqueMatches, isLegacyMethod, outputClass)

% Main algorithm used by the matchFeatures function. See matchFeatures
% for more details.

% Copyright 2011-2022 The MathWorks, Inc.
%#codegen

features1in = features1in';
features2in = features2in';

if isempty(features1in) || isempty(features2in)
    indexPairs  = zeros(0, 2, 'uint32');
    matchMetric = zeros(0, 1, outputClass);
    return;
end

matchThreshold = cast(threshold, outputClass);

% cast feature data to expected output class
[features1, features2] = castFeatures(features1in, features2in, metric,...
    method, outputClass);

% normalize features using L2-norm
if ~isPrenormalized && ~strcmpi(metric, 'hamming')
    [features1, features2] = normalizeFeatures(features1, features2, method, metric);
end

% Find matches based on selected method
if isLegacyMethod
    
    [indexPairs, matchMetric] = findMatchesLegacy(features1, features2, ...
        metric, method, maxRatioThreshold, matchThreshold, outputClass);
    
elseif strcmpi(method, 'approximate')
    
    [indexPairs, matchMetric] = findMatchesApproximate(features1,features2,...
        metric, maxRatioThreshold, matchThreshold, uniqueMatches, outputClass);
    
else % exhaustive
    
    [indexPairs, matchMetric] = findMatchesExhaustive(features1,features2, ...
        metric, maxRatioThreshold, matchThreshold, uniqueMatches, outputClass);
    
end

indexPairs  = indexPairs';
matchMetric = matchMetric';


%==========================================================================
% Use Approximate nearest neighbor search to find matches.
%==========================================================================
function [indexPairs, matchMetric] = findMatchesApproximate(features1,features2,...
    metric, maxRatioThreshold, matchThreshold, uniqueMatches, outputClass)

N2 = cast(size(features2, 2), 'uint32');

[indexPairs, matchMetric] = findApproximateNearestNeighbors(features1,...
    features2, metric, outputClass);

[indexPairs, matchMetric] = removeWeakMatches(indexPairs, ...
    matchMetric, matchThreshold, metric);

% Perform Ratio Test
if maxRatioThreshold ~= 1
    [indexPairs, matchMetric] = removeAmbiguousMatches(indexPairs, ...
        matchMetric, maxRatioThreshold, N2, metric);
else
    matchMetric = matchMetric(1,:);
end

if isempty(indexPairs)
    indexPairs  = zeros(2, 0, 'uint32');
    matchMetric = zeros(1, 0, outputClass);
    return;
end

if uniqueMatches
    % perform backward match to remove non-unique matches.
    
    % avoid search for the same feature more than once.
    idx = unique(indexPairs(2,:));
    
    reverse_index_pairs = findApproximateNearestNeighbors(...
        features2(:,idx), features1, metric, outputClass);
    
    f2ToF1Matches = zeros(1,N2,'uint32');
    f2ToF1Matches(idx) = reverse_index_pairs(2,:);
    
    symmetric_pairs = f2ToF1Matches(indexPairs(2,:)) == indexPairs(1,:);
    
else % branch required for codegen
    symmetric_pairs = true(1,size(indexPairs,2));
end

indexPairs  = indexPairs(:,symmetric_pairs);
matchMetric = matchMetric(1,symmetric_pairs);

%==========================================================================
% Find approximate nearest neighbors. Return indices to the matching
% features and the distance metrics for the 2 nearest neighbors (for the
% ratio test).
%==========================================================================
function [indexPairs, metrics] = findApproximateNearestNeighbors(...
    features1, features2, metric, outputClass)

if isempty(coder.target)
    [indexPairs, metrics] = ocvFlannBasedMatching(...
        features1, features2, metric);
    
else
    coder.internal.errorIf(~coder.internal.isTargetMATLABHost,...
        'vision:matchFeatures:codegenApproxHostOnly');
    
    features1 = features1';
    features2 = features2';
    
    [indexPairs, metrics] = ...
        vision.internal.buildable.matchFeaturesApproxNN.findApproximateNearestNeighbors(...
        features1, features2, metric);
end

if coder.isColumnMajor
    % type of metric output should match feature input type
    metrics = cast(metrics, outputClass);
    
    % convert to 1-based indexing & cast to uint32
    indexPairs  = bsxfun(@plus, uint32(indexPairs(1,:)), uint32(1));
    
    indexPairs  = vertcat(uint32(1:size(indexPairs,2)), indexPairs(1,:));
else
    % type of metric output should match feature input type
    metrics = cast(metrics', outputClass);
    
    % convert to 1-based indexing & cast to uint32
    idxPairs = indexPairs';
    indexPairs  = bsxfun(@plus, uint32(idxPairs(1,:)), uint32(1));
    
    indexPairs  = vertcat(uint32(1:size(indexPairs,2)), indexPairs(1,:));
end

%==========================================================================
% Find matches using an exhaustive search.
%==========================================================================
function [indexPairs, matchMetric] = findMatchesExhaustive(features1, features2,...
    metric, maxRatioThreshold, matchThreshold, uniqueMatches, outputClass)

N1 = uint32(size(features1,2));
N2 = uint32(size(features2,2));

% Check if the matches need to be computed in batches to avoid memory
% issues.
scoreMatrixSize = N1*N2;
% Set the memory threshold to ~8 GB for double and ~4 GB for single.
sizeThreshold = 1e9;

if scoreMatrixSize < sizeThreshold
    [indexPairs, matchMetric] = exhaustiveSearch(features1, features2,...
        metric, maxRatioThreshold, matchThreshold, uniqueMatches, N1, N2, outputClass);
else % Do several iterations.
    % Set the number of iterations and the number of features2 to use per
    % iteration. All features1 is used at every iteration. 
    numIterations = ceil(double(scoreMatrixSize)/sizeThreshold);
    numFeatures2 = floor(double(N2)/numIterations);

    % Initialize variables for code generation.
    indexPairsCell = cell(1, numIterations);
    matchMetricCell = cell(1, numIterations);
    idxFeatures2 = 0;
    numMatches = zeros(1, numIterations);
    for i = 1:numIterations
        if i < numIterations
            idxFeatures2 = (i-1)*numFeatures2+1:i*numFeatures2;
        else % Last iteration.
            idxFeatures2 = idxFeatures2(end)+1:double(N2);
            numFeatures2 = numel(idxFeatures2);
        end
        [relativeIndexPairs, pairMetric] = exhaustiveSearch(features1, ...
            features2(:, idxFeatures2), metric, maxRatioThreshold, ...
            matchThreshold, uniqueMatches, N1, uint32(numFeatures2), outputClass);
        numMatches(i) = numel(pairMetric);
        matchMetricCell{i} = pairMetric;
        % The indices returned in the second row are relative to the set of
        % features2 used in the iteration. Use idxFeatures2 to convert to
        % an absolute index.
        indexPairsCell{i} = [relativeIndexPairs(1,:); ...
            relativeIndexPairs(2,:) + idxFeatures2(1) - 1];
    end

    % Combine the results of all the iterations in a matrix of index pairs
    % and a vector of pair metrics.
    totalMatches = sum(numMatches);
    indexPairs = zeros(2, totalMatches, 'uint32');
    matchMetric = zeros(1, totalMatches, outputClass);
    currentIdx = 0;
    for n = 1:numIterations
        endIdx = currentIdx + numMatches(n);
        indexPairs(:, currentIdx+1:endIdx) = indexPairsCell{n};
        matchMetric(currentIdx+1:endIdx) = matchMetricCell{n};
        currentIdx = endIdx;
    end
end

%==========================================================================
% Get scores and select matches using an exhaustive search.
%==========================================================================
function [indexPairs, matchMetric] = exhaustiveSearch(features1, features2,...
    metric, maxRatioThreshold, matchThreshold, uniqueMatches, N1, N2, outputClass)
% SCORES is an N1-by-N2 correspondence metric matrix where the rows
% correspond to the feature vectors in FEATURES1, and the columns
% correspond to the feature vectors in FEATURES2.

scores = vision.internal.matchFeatures.exhaustiveDistanceMetrics( ...
    features1, features2, N1, N2, outputClass, metric);

performRatioTest = maxRatioThreshold ~= 1;

if performRatioTest
    numNearestNeighbors = 2;
else
    numNearestNeighbors = 1;
end

[indexPairs, matchMetric] = findNearestNeighbors(scores, metric, numNearestNeighbors);

[indexPairs, matchMetric] = removeWeakMatches(indexPairs, ...
    matchMetric, matchThreshold, metric);

% Perform Ratio Test
if performRatioTest
    [indexPairs, matchMetric] = removeAmbiguousMatches(indexPairs, ...
        matchMetric, maxRatioThreshold, N2, metric);
else
    matchMetric = matchMetric(1,:);
end

if isempty(indexPairs)
    indexPairs  = zeros(2, 0, 'uint32');
    matchMetric = zeros(1, 0, outputClass);
    return;
end

if uniqueMatches
    uniqueIndices = findUniqueIndices(scores, metric, indexPairs);
else
    % branch required for codegen
    uniqueIndices = true(1,size(indexPairs,2));
end
indexPairs  = indexPairs(:, uniqueIndices);
matchMetric = matchMetric(1, uniqueIndices);

%==========================================================================
% Find nearest neighbors using an exhaustive search. Return indices to the
% matching features and the distance metrics for the 2 nearest neighbors
% (for the ratio test).
%==========================================================================
function  [indexPairs, topTwoMetrics] = findNearestNeighbors(scores, metric, n)

if strcmp(metric, 'normxcorr')
    [topTwoMetrics, topTwoIndices] = vision.internal.partialSort(scores, n, 'descend');
else
    [topTwoMetrics, topTwoIndices] = vision.internal.partialSort(scores, n, 'ascend');
end

indexPairs = vertcat(uint32(1:size(scores,1)), topTwoIndices(1,:));

%==========================================================================
% Find matches using legacy method modes.
%==========================================================================
function [indexPairs, matchMetric] = findMatchesLegacy(features1, features2, ...
    metric, method, maxRatioThreshold, matchThreshold, outputClass)

N1 = uint32(size(features1,2));
N2 = uint32(size(features2,2));

switch method
    case 'nearestneighborsymmetric'
        scores = vision.internal.matchFeatures.exhaustiveDistanceMetrics( ...
            features1, features2, N1, N2, outputClass, metric);
        
        % keep this for backward compatibility
        [indexPairs, matchMetric] = findMatchesNN(scores, metric, ...
            matchThreshold);
        
    case 'threshold'
        % keep this for backward compatibility
        
        scores = vision.internal.matchFeatures.exhaustiveDistanceMetrics( ...
            features1, features2, N1, N2, outputClass, metric);
        
        [indexPairs, matchMetric] = findMatchesThreshold(scores, ...
            metric, matchThreshold);
        
    case 'nearestneighbor_old'
        % keep this for backward compatibility
        
        scores = vision.internal.matchFeatures.exhaustiveDistanceMetrics( ...
            features1, features2, N1, N2, outputClass, metric);
        
        [indexPairs, matchMetric] = findMatchesNN_old(scores, N1, N2, ...
            metric);
        
        [indexPairs, matchMetric] = removeWeakMatches(indexPairs, ...
            matchMetric, matchThreshold, metric);
        
    case 'nearestneighborratio'
        if N2 > 1
            [indexPairs, matchMetric] = findMatchesExhaustive(...
                features1, features2, metric, maxRatioThreshold, ...
                matchThreshold, false, outputClass);
        else
            % If FEATURES2 contains only 1 feature, we cannot use ratio.
            % Use NearestNeighborSymmetric instead, resulting in a single
            % match
            scores = vision.internal.matchFeatures.exhaustiveDistanceMetrics( ...
                features1, features2, N1, N2, outputClass, metric);
            [indexPairs, matchMetric] = findMatchesNN(scores, ...
                metric, matchThreshold);
        end
        
end

%==========================================================================
% Remove ambiguous matches using the nearest neighbor ratio test.
%==========================================================================
function [indexPairs, matchMetric] = removeAmbiguousMatches(indexPairs, ...
    matchMetric, maxRatio, N2, metric)

if N2 > 1
    % apply ratio test only if there are more than 1 feature vectors
    unambiguousIndices = findUnambiguousMatches(matchMetric, maxRatio, metric);
else
    unambiguousIndices = true(1,size(matchMetric,2));
end

indexPairs  = indexPairs(:, unambiguousIndices);
matchMetric = matchMetric(1,unambiguousIndices);

%==========================================================================
% Enforce uniqueness by applying a bi-directional match constraint
%==========================================================================
function uniqueIndices = findUniqueIndices(scores, metric, ...
    indexPairs)

if strcmpi(metric,'normxcorr')
    [~, idx] = max(scores(:,indexPairs(2,:)));
else
    [~, idx] = min(scores(:,indexPairs(2,:)));
end

idx = cast(idx, 'like', indexPairs);
uniqueIndices = idx == indexPairs(1,:);

%==========================================================================
% Find matches using Nearest-Neighbor strategy
%==========================================================================
function [indexPairs, matchMetric] = findMatchesNN(scores, ...
    metric, matchThreshold)
% Find the maximum(minimum) entry in scores.
% Make it a match.
% Eliminate the corresponding row and the column.
% Repeat.

nRows = size(scores, 1);
nCols = size(scores, 2);
nMatches = min(nRows, nCols);
indexPairs = zeros([2, nMatches], 'uint32');
matchMetric = zeros(1, nMatches, 'like', scores);

useMax = strcmp(metric, 'normxcorr');

for i = 1:nMatches
    if useMax
        [matchMetric(i), ind] = max(scores(:));
        [r, c] = ind2sub(size(scores), ind);
    else
        [matchMetric(i), ind] = min(scores(:));
        [r, c] = ind2sub(size(scores), ind);
    end
    
    indexPairs(:, i) = [r, c];
    if useMax
        scores(r, :) = -inf('like',scores);
        scores(:, c) = -inf('like',scores);
    else
        scores(r, :) = inf('like',scores);
        scores(:, c) = inf('like',scores);
    end
end

[indexPairs, matchMetric] = removeWeakMatches(indexPairs, ...
    matchMetric, matchThreshold, metric);

%==========================================================================
% Normalize features to be unit vectors
%==========================================================================
function [features1, features2] = normalizeFeatures(features1, features2, ...
    method, metric)

% move this to parsing where we map old method values.
% normalize the features
if strcmp(method, 'nearestneighbor_old') && ...
        strcmp(metric, 'normxcorr')
    % for backward compatibility, subtract the mean from features
    f1Mean = mean(features1);
    features1 = bsxfun(@minus, features1, f1Mean);
    f2Mean = mean(features2);
    features2 = bsxfun(@minus, features2, f2Mean);
end

% Convert feature vectors to unit vectors
features1 = vision.internal.matchFeatures.normalizeFeature(features1);
features2 = vision.internal.matchFeatures.normalizeFeature(features2);

%==========================================================================
% Cast features to expected output class.
%==========================================================================
function [features1, features2] = castFeatures(features1in, features2in,...
    metric, method, outputClass)

if ~strcmp(metric, 'hamming')
    if strcmpi(method, 'approximate')
        % approximate search requires single
        features1 = cast(features1in, 'single');
        features2 = cast(features2in, 'single');
    else
        features1 = cast(features1in, outputClass);
        features2 = cast(features2in, outputClass);
    end
else
    % do not cast binary feature data
    features1 = features1in;
    features2 = features2in;
end

%==========================================================================
% Find unambiguous matches using David Lowe's disambiguation strategy
%==========================================================================
function unambiguousIndices = findUnambiguousMatches(topTwoScores, maxRatioThreshold, metric)

if strcmpi(metric, 'normxcorr')
    % If the metric is 'normxcorr', then the scores are cosines
    % of the angles between the feature vectors.
    % The ratio of the angles is an approximation of the ratio of
    % euclidean distances.  See David Lowe's demo code.
    
    topTwoScores(topTwoScores > 1) = 1; % prevent complex angles
    topTwoScores = acos(topTwoScores);
end

% handle division by effective zero
zeroInds = topTwoScores(2, :) < cast(1e-6, 'like', topTwoScores);
topTwoScores(:, zeroInds) = 1;
ratios = topTwoScores(1, :) ./ topTwoScores(2, :);

unambiguousIndices = ratios <= maxRatioThreshold;

%==========================================================================
% Find matches using a bidirectional greedy strategy
%==========================================================================
function [indexPairs, matchMetric] = findMatchesThreshold(scores, ...
    metric, matchThreshold)

if strcmp(metric, 'normxcorr')
    inds = find(scores >= matchThreshold);
else
    inds = find(scores <= matchThreshold);
end

matchMetric = scores(inds)';
[rowInds, colInds] = ind2sub(size(scores), inds);
indexPairs = cast([rowInds, colInds]', 'uint32');

%==========================================================================
% Find matches using the old nearest neighbor strategy for compatibility
%==========================================================================
function [indexPairs, matchMetric] = findMatchesNN_old(scores, N1, N2,...
    metric)
% SCORES is an N1-by-N2 correspondence metric matrix where the rows
% correspond to the feature vectors in FEATURES1, and the columns
% correspond to the feature vectors in FEATURES2.

% For each feature vector in FEATURES1 find the best match in FEATURES2.
% This is simply the entry along each row with the minimum or maximum
% metric value, depending on the metric.
row_index_pairs = [(1:N1); zeros(1, N1)];
if (strcmp(metric, 'normxcorr'))
    [row_scores, row_index_pairs(2, :)] = max(scores, [], 2);
else
    [row_scores, row_index_pairs(2, :)] = min(scores, [], 2);
end

% For each feature vector in FEATURES2 find the best match in FEATURES1.
% This is simply the entry down each col with the minimum or maximum metric
% value, depending on the metric.
col_index_pairs = [zeros(1, N2); (1:N2)];
if (strcmp(metric, 'normxcorr'))
    [col_scores, col_index_pairs(1, :)] = max(scores, [], 1);
else
    [col_scores, col_index_pairs(1, :)] = min(scores, [], 1);
end

% Concatenate both row and column lists
M = [row_scores', col_scores];
indexPairs = [row_index_pairs, col_index_pairs];

% Remove duplicate entries in the matches list
[trimmed_index_pairs, I, ~] = unique(indexPairs', 'rows');
matchMetric = M(I);
indexPairs = trimmed_index_pairs';

%==========================================================================
% Remove weak matches
%==========================================================================
function [indices, matchMetric] = removeWeakMatches(indices, ...
    matchMetric, matchThreshold, metric)

if (strcmp(metric, 'normxcorr'))
    inds = matchMetric(1,:) >= matchThreshold;
else
    inds = matchMetric(1,:) <= matchThreshold;
end

indices = indices(:, inds);
matchMetric = matchMetric(:, inds);