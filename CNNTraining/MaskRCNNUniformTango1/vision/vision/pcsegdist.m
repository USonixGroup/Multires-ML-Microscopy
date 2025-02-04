function [labels, numClusters] = pcsegdist(ptCloud, minDistance, varargin)

% Copyright 2017-2024 The MathWorks, Inc.

%#codegen

narginchk(2, 8);

[minPoints, maxPoints, parallelSearch, approximateMethod] = parseAndValidateInputs(ptCloud, minDistance, varargin{:});

minDistance = cast(minDistance, 'like', ptCloud.Location);

isGPU = coder.gpu.internal.isGpuEnabled;
if isGPU
    if ~isempty(varargin)
        error(message('vision:pointcloud:unsupportedNameValueArgumentsGPU'))
    end
    % GPU implementation of point-cloud clustering
    [labels, numClusters] = vision.internal.codegen.gpu.pcsegdistImpl(ptCloud.Location, minDistance);
else
    [labels, numClusters] = pcsegdistImpl(ptCloud, minDistance, minPoints, ...
        maxPoints, parallelSearch, approximateMethod);
end

end

%--------------------------------------------------------------------------
function [minPoints, maxPoints, parallelSearch, approximateMethod] = parseAndValidateInputs(ptCloud, minDistance, varargin)

% Validate the first argument
validateattributes(ptCloud, {'pointCloud'}, {'scalar'}, 'pcsegdist', 'ptCloud');

% Validate the second argument
validateattributes(minDistance, {'single','double'}, ...
    {'nonnan', 'nonsparse', 'scalar', 'real', 'positive', 'finite'}, ...
    'pcsegdist', 'minDistance');

if coder.target()
    [numClusterPoints, parallelSearch, selectedMethod] = parseInputsCodegen(varargin{:});    
else
    [numClusterPoints, parallelSearch, selectedMethod] = parseInputsSim(ptCloud, minDistance, varargin{:});
end

vision.internal.inputValidation.validateLogical(parallelSearch,...
    'ParallelNeighborSearch');
[minPoints, maxPoints] = validateNumClusterPoints(numClusterPoints);

method = validatestring(selectedMethod, {'approximate','exhaustive'}, ...
    mfilename, 'Method');

approximateMethod = strcmp(method, "approximate");

if coder.target()
    % The exhaustive method does not support code generation
    coder.internal.assert(approximateMethod, 'vision:pointcloud:methodNotSupportedCodegen')
elseif parallelSearch & ~approximateMethod
    % The exhaustive method does not support parallel neighbor search
    warning(message('vision:pointcloud:unsupportedParallelWithExhaustive'))
end

end

function [numClusterPoints, parallelSearch, selectedMethod] = parseInputsSim(ptCloud, minDistance, args)

arguments
    ptCloud %#ok<*INUSA>
    minDistance
    args.NumClusterPoints = [1, Inf];
    args.ParallelNeighborSearch = false;
    args.Method char = "approximate";
end

parallelSearch = args.ParallelNeighborSearch;
numClusterPoints = args.NumClusterPoints;
selectedMethod = args.Method;
end

%--------------------------------------------------------------------------
function [numClusterPoints, parallelSearch, selectedMethod] = parseInputsCodegen(varargin)

% Set input parser
defaults = struct(...
    'NumClusterPoints', [1, Inf], ...
    'ParallelNeighborSearch', false, ...
    'Method', "approximate");
% Define parser mapping struct
pvPairs = struct( ...
    'NumClusterPoints', [uint32(0), uint32(0)], ...
    'ParallelNeighborSearch', uint32(0), ...
    'Method', uint32(0));
% Specify parser options
poptions = struct( ...
    'CaseSensitivity', false, ...
    'StructExpand',    true, ...
    'PartialMatching', true);

% Parse name-value arguments
pstruct = coder.internal.parseParameterInputs(pvPairs, ...
    poptions, varargin{:});

% Extract inputs
numClusterPoints = coder.internal.getParameterValue(pstruct.NumClusterPoints,...
    defaults.NumClusterPoints, varargin{:});
parallelSearch = coder.internal.getParameterValue(pstruct.ParallelNeighborSearch,...
    defaults.ParallelNeighborSearch, varargin{:});
selectedMethod = coder.internal.getParameterValue(pstruct.Method,...
    defaults.Method, varargin{:});
end

%--------------------------------------------------------------------------
function [minPoints, maxPoints] = validateNumClusterPoints(numClusterPoints)
if isscalar(numClusterPoints)
    validateattributes(numClusterPoints, {'numeric'}, ...
        {'nonnan', 'nonsparse', 'real', 'positive', 'integer'}, ...
        mfilename, 'NumClusterPoints');
    minPoints = numClusterPoints;
    maxPoints = Inf;
else
    validateattributes(numClusterPoints, {'numeric'}, ...
        {'nonnan', 'nonsparse', 'real', 'positive', 'increasing', ...
        'numel', 2}, mfilename, 'NumClusterPoints');
    
    minPoints = numClusterPoints(1);
    maxPoints = numClusterPoints(2);
    
    % Verify that minPoints and maxPoints are integers or Inf for maxPoints
    isValid = (minPoints == floor(minPoints)) && ...
        (isinf(maxPoints) || maxPoints == floor(maxPoints));
    if ~isValid
        coder.internal.error('vision:pointcloud:mustBeIntegerNumClusterPoints')
    end
    minPoints = numClusterPoints(1);
    maxPoints = numClusterPoints(2);
end

end

%--------------------------------------------------------------------------
function [labels, numClusters] = pcsegdistImpl(ptCloud, minDistance, ...
    minPoints, maxPoints, parallelSearch, approximateMethod)

isOrganized = ndims(ptCloud.Location) == 3;
if isOrganized
    labelsSize = size(ptCloud.Location, 1:2);
else
    labelsSize = [size(ptCloud.Location, 1) 1];
end

labels = zeros(labelsSize, 'uint32');

% Remove invalid points. pc is an unorganized point cloud having valid
% points
[pc, validIndices] = removeInvalidPoints(ptCloud);

if pc.Count == 0
    % There are no valid points in the point cloud.
    numClusters = zeros(class(ptCloud.Location));
    return
end

if approximateMethod
    L = pcsegdistApproximateImpl(pc, minDistance, parallelSearch);
else
    L = lidarClusterDBSCAN(pc.Location, minDistance, uint64(1));
end

% Get unique labels sorted in increasing order
uniqueLabels = unique(L);

% Remove clusters with invalid sizes
if (minPoints > 1) || isfinite(maxPoints)
    numClusters = cast(0, 'like', ptCloud.Location);
    [L, numClusters] = filterClustersByNumPoints(L, uniqueLabels, minPoints, maxPoints, numClusters);
else
    numClusters = cast(numel(uniqueLabels), 'like', ptCloud.Location);
    for k = 1:numClusters
        L(L == uniqueLabels(k)) = k;
    end
end

labels(validIndices) = L;
end

%--------------------------------------------------------------------------
function L = pcsegdistApproximateImpl(pc, minDistance, parallelSearch)

L = zeros(pc.Count, 1, 'uint32');
newLabel = 0;
% Find the neighbors for each point and label them accordingly
if parallelSearch
    indices = multiQueryRadiusSearchImpl(pc, pc.Location, minDistance);
    for i = 1:pc.Count
        % Move to the next point if previously labeled
        if L(i) ~= 0
            continue;
        end
        
        ind = indices{i};
        [L, newLabel] = labelPoints(L, ind, i, newLabel);
    end
else
    for i = 1:pc.Count
        % Move to the next point if previously labeled
        if L(i) ~= 0
            continue;
        end
        
        % Find neighbors including itself
        ind = findNeighborsInRadius(pc, pc.Location(i,:), minDistance);
        [L, newLabel] = labelPoints(L, ind, i, newLabel);
        
    end
end
end

%--------------------------------------------------------------------------
function [L, numClusters] = filterClustersByNumPoints(L, uniqueLabels, minPoints, maxPoints, numClusters)

% Check the number of points for each label
for n = 1:numel(uniqueLabels)
    numPoints = nnz(L == uniqueLabels(n));
    % Use a 0 label for points in invalid clusters
    if (numPoints < minPoints) || (numPoints > maxPoints)
        L(L == uniqueLabels(n)) = 0;
    else
        % Label valid clusters
        numClusters = numClusters + 1;
        L(L == uniqueLabels(n)) = numClusters;
    end
end

end

%--------------------------------------------------------------------------
function [L, newLabel] = labelPoints(L, ind, i, newLabel)
for k = 1:numel(ind)
    j = ind(k);
    % For each neighbor, if the current point and neighbor are
    % labeled, keep the smallest label.
    if L(j) > 0 && L(i) > 0
        if L(j) > L(i)
            L(L==L(j)) = L(i);
        elseif L(j) < L(i)
            L(L==L(i)) = L(j);
        end
        % If the neighbor has a label and the point does not, label the
        % current point. If the current point has a label and the
        % neighbor does not, label the neighbor.
    else
        if L(j) > 0
            L(i) = L(j);
        elseif L(i) > 0
            L(j) = L(i);
        end
    end
end
% If the current point is not labeled, create a new label and label
% all its neighbors.
if L(i) == 0
    newLabel = newLabel+1;
    L(ind) = newLabel;
end
end

