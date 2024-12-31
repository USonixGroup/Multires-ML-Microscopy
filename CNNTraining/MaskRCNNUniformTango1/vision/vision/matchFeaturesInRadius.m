function [indexPairs, matchMetric] = matchFeaturesInRadius(features1, ...
    features2, points2, centerPoints, radius, varargin)

% Copyright 2020-2023 The MathWorks, Inc.

%#codegen

% Parse and check inputs
[features1, features2, centerPoints, points2, radius, metric, matchThreshold, ...
    maxRatio, uniqueMatch, outputClass, numPoints1, numPoints2] = parseInputs(...
    features1, features2, centerPoints, points2, radius, varargin{:});

vision.internal.matchFeatures.checkFeatureConsistency(features1, features2);

% Convert match threshold percent to a numeric threshold
matchThreshold = vision.internal.matchFeatures.percentToLevel( ...
    matchThreshold, size(features1, 2), metric, outputClass);

% Use transposed feature vectors to compute distance
features1 = features1';
features2 = features2';

% Cast and normalize non-binary features
if ~strcmp(metric, 'hamming')
    features1 = cast(features1, outputClass);
    features2 = cast(features2, outputClass);

    % Convert feature vectors to unit vectors
    features1 = vision.internal.matchFeatures.normalizeFeature(features1);
    features2 = vision.internal.matchFeatures.normalizeFeature(features2);
end

% Find feature points within the radius
allSpatialDist = vision.internal.matchFeatures.metricSSD( ...
    points2', centerPoints', numPoints2, numPoints1, class(centerPoints));
isInRadius     = allSpatialDist <= repmat( radius.^2, numPoints2, 1);

% Find strong matches
matchScores    = vision.internal.matchFeatures.exhaustiveDistanceMetrics(...
    features2, features1, numPoints2, numPoints1, outputClass, metric);
isStrongMatch  = matchScores <= matchThreshold;
isValid        = isStrongMatch & isInRadius;
doRatioCheck   = maxRatio ~= 1;

if isempty(coder.target)
    indices  = vision.internal.ratioTestInRadius(matchScores, isValid, maxRatio, doRatioCheck);
    isPointMatched = find(indices);
    indexPairs = [uint32(isPointMatched), indices(isPointMatched)];
else
    indexPairs     = zeros(numPoints1, 2, 'uint32');
    for centerIdx  = 1:numPoints1
        neighborIdxLogical  = isValid(:, centerIdx);
        candidateScores     = matchScores(neighborIdxLogical, centerIdx);
        neighborIdxLinear   = find(neighborIdxLogical);

        if numel(neighborIdxLinear) == 1
            matchIndex = neighborIdxLinear(1);

        elseif numel(neighborIdxLinear) > 1

            % Pick the best two candidates within the radius
            [topTwoMetrics, topTwoIndices] = vision.internal.partialSort(candidateScores', 2, 'ascend');
            matchIndex = neighborIdxLinear(topTwoIndices(1));

            % Perform ratio test
            if doRatioCheck
                if  topTwoMetrics(2) < cast(1e-6, outputClass)
                    ratio = cast(1, outputClass);
                else
                    ratio = topTwoMetrics(1) /topTwoMetrics(2);
                end

                if ratio > maxRatio
                    continue % Ambiguous match
                end
            end
        else
            matchIndex = 0; % For codegen
            continue % No match found
        end

        indexPairs(centerIdx, :) = [centerIdx, matchIndex];
    end

    isPointMatched = indexPairs(:, 1) ~= uint32(0);
    indexPairs     = indexPairs(isPointMatched, :);
end

% Check if the match is unique
numMatch = nnz(isPointMatched);
if uniqueMatch && numMatch > 0
    isUnique = true(numMatch, 1);
    for i = 1: numMatch
        feature2Idx = indexPairs(i, 2);
        % Find all the matched features in image 1 within the radius
        matchedFeature1Idx = find(isValid(feature2Idx,:));
        if numel(matchedFeature1Idx) > 1
            [~, ia]      = min(matchScores(feature2Idx, matchedFeature1Idx));
            % The matched feature 1 should be the best within the radius
            isUnique(i)  = matchedFeature1Idx(ia) == indexPairs(i, 1);
        end
    end
    indexPairs   = indexPairs(isUnique, :);
end

if nargout > 1

    % Get the corresponding match score
    if any(isPointMatched)
        matchMetric = matchScores(sub2ind([numPoints2, numPoints1], indexPairs(:, 2), indexPairs(:,1)));
    else
        matchMetric = zeros(0, 1, outputClass);
    end
end
end

%--------------------------------------------------------------------------
% Parse and check inputs
%--------------------------------------------------------------------------
function [features1, features2, centerPoints, points2, radius, metric, ...
    matchThreshold, maxRatio, uniqueMatch, outputClass, numPoints1, numPoints2] = ...
    parseInputs(featuresIn1, featuresIn2, centerPoints, points2, radius, varargin)

fileName = 'matchFeaturesInRadius';

vision.internal.inputValidation.checkFeatures(featuresIn1, fileName, 'features1');
vision.internal.inputValidation.checkFeatures(featuresIn2, fileName, 'features2');

coder.internal.errorIf(~isequal(class(featuresIn1), class(featuresIn2)),...
    'vision:matchFeatures:featuresNotSameClass');

isBinaryFeature = isa(featuresIn1, 'binaryFeatures');

if isBinaryFeature
    features1   = featuresIn1.Features;
    features2   = featuresIn2.Features;
else
    features1   = featuresIn1;
    features2   = featuresIn2;
end

% Determine output class
if (isa(features1, 'double'))
    outputClass = 'double';
else
    outputClass = 'single';
end

numPoints1 = size(features1, 1);
numPoints2 = size(features2, 1);

points2      = checkPoints2(points2, numPoints2);
checkCenterPoints(centerPoints, numPoints1);
centerPoints = cast(centerPoints, 'like', points2);
radius       = checkRadius(radius, numPoints1);

isRadiusSearch  = true;
defaultParams   = vision.internal.matchFeatures.getDefaultParameters(isBinaryFeature, isRadiusSearch);

% Parse inputs
if isempty(coder.target)  % Simulation
    [metricTemp, matchThreshold, maxRatio, uniqueMatch] = ...
        parseOptionalInputsSimulation(defaultParams, varargin{:});
else % Code generation
    [metricTemp, matchThreshold, maxRatio, uniqueMatch] = ...
        parseOptionalInputsCodegen(defaultParams, varargin{:});
end

vision.internal.matchFeatures.checkMatchThreshold(matchThreshold, fileName);
vision.internal.matchFeatures.checkMaxRatioThreshold(maxRatio, fileName);
vision.internal.matchFeatures.checkMetric(metricTemp, fileName);
vision.internal.matchFeatures.checkUniqueMatches(uniqueMatch, fileName);

if isBinaryFeature
    metric = 'hamming';
else
    metric = lower(metricTemp);
end
maxRatio = cast(maxRatio, outputClass);

end
%--------------------------------------------------------------------------
function [metric, matchThreshold, maxRatio, uniqueMatch] = ...
    parseOptionalInputsSimulation(defaultParams, varargin)
%parseOptionalInputsSimulation Parse parameters for simulation workflow

% Setup parser
parser = inputParser;
parser.addParameter('MatchThreshold', defaultParams.MatchThreshold);
parser.addParameter('MaxRatio', defaultParams.MaxRatio);
parser.addParameter('Metric', defaultParams.Metric);
parser.addParameter('Unique', defaultParams.Unique);

% Parse input
parser.parse(varargin{:});
r = parser.Results;

matchThreshold  = r.MatchThreshold;
maxRatio        = r.MaxRatio;
metric          = r.Metric;
uniqueMatch     = r.Unique;
end

%--------------------------------------------------------------------------
function [metric, matchThreshold, maxRatio, uniqueMatch] = ...
    parseOptionalInputsCodegen(defaultParams, varargin)
%parseOptionalInputsCodegen Parse parameters for codegen workflow

% Set parser inputs
params = struct( ...
    'Metric',                uint32(0), ...
    'MatchThreshold',        uint32(0), ...
    'MaxRatio',              0, ...
    'Unique',                false);

popt = struct( ...
    'CaseSensitivity', false, ...
    'StructExpand',    true, ...
    'PartialMatching', true);

optarg = eml_parse_parameter_inputs(params, popt, varargin{:});

metric = eml_get_parameter_value(optarg.Metric, ...
    defaultParams.Metric, varargin{:});
matchThreshold = eml_get_parameter_value(optarg.MatchThreshold, ...
    defaultParams.MatchThreshold, varargin{:});
maxRatio = eml_get_parameter_value(optarg.MaxRatio, ...
    defaultParams.MaxRatio, varargin{:});
uniqueMatch = eml_get_parameter_value(optarg.Unique, ...
    defaultParams.Unique, varargin{:});
end

%--------------------------------------------------------------------------
function checkCenterPoints(points, numPoints)
checkImagePointsWithKnownSize(points, numPoints, 'centerPoints')
end

%--------------------------------------------------------------------------
function pointsOut = checkPoints2(pointsIn, numPoints)
coder.internal.errorIf( ~isnumeric(pointsIn)...
    && ~vision.internal.inputValidation.isValidPointObj(pointsIn), ...
    'vision:points:ptsClassInvalid', 'points2');

if ~isnumeric(pointsIn)
    pointsOut = pointsIn.Location;
else
    pointsOut = pointsIn; % For codegen
end

checkImagePointsWithKnownSize(pointsOut, numPoints, 'points2')
end

function checkImagePointsWithKnownSize(imagePoints, numPoints, varName)
validateattributes(imagePoints, {'numeric'}, ...
    {'nonsparse', 'real', 'size', [numPoints, 2]}, ...
    'matchFeaturesInRadius', varName);
end

%--------------------------------------------------------------------------
function radius = checkRadius(r, numPoints)
if isscalar(r)
    validateattributes(r, {'single', 'double'}, ...
        {'nonnan', 'nonsparse', 'real', 'positive'}, ...
        'matchFeaturesInRadius', 'radius');
    radius = repmat(r, 1, numPoints);
else
    validateattributes(r, {'single', 'double'}, ...
        {'vector', 'nonnan', 'nonsparse', 'real', 'positive', 'numel', numPoints}, ...
        'matchFeaturesInRadius', 'radius');
    radius = r(:)'; % Row vector
end
end