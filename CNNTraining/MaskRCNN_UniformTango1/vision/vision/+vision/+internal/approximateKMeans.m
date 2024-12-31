% approximateKMeans Performs approximate K-Means clustering.
%   [centers, assignments] = approximateKMeans(features, K) clusters
%   features into K groups and returns the cluster centers and the feature
%   vector assignments to each cluster. features must be a M-by-N matrix,
%   where M is the number of features to cluster, and N is the dimension of
%   each feature vector. The output centers is a K-by-N matrix of cluster
%   centers. assignments is a 1-by-M array of cluster assignments.
%
%   [...] = approximateKMeans(...,Name,Value) specifies additional
%   name-value pair arguments described below:
%
%   'MaxIterations'   Maximum number of iterations before the K-Means
%                     algorithm is terminated.
%
%                     Default: 100
%
%   'Threshold'       When the change in the total sum of intra-cluster
%                     distances is below the Threshold, the K-Means
%                     algorithm is terminated.
%
%                     Default: 0.0001
%
%   'NumTrials'       The number of times the K-Means algorithm is run with
%                     different initial cluster centers. The solution with
%                     the lowest total sum of intra-cluster distances is
%                     returned.
%
%                     Default: 1
%
%   'Initialization'  Specify the method used to initialize the cluster
%                     centers as 'Random' or 'KMeans++'.
%
%                     Default: 'KMeans++'
%
%
%   'DistanceMethod'  Specify the distance method used to cluster
%                     the features with centers as 'L2' or 'BboxIoU'.
%                     'L2'       - The distance metric is the euclidean L2
%                                  distance (L2 norm).
%                     'BboxIOU'  - The distance metric is the bounding box
%                                  IoU (intersection-over-union). The distance
%                                  is calculated using the equation,
%                                  1 - bboxOverlapRatio(box1,box2);
%                                  The features input must be N-by-2.
%
%                     Default: 'L2'
%

% References
% ----------
% J. Philbin, O. Chum, M. Isard, J. Sivic, and A. Zisserman. Object
% retrieval with large vocabularies and fast spatial matching. InProc.
% Computer Vision and Pattern Recognition (CVPR), 2007
%
% Arthur, D. and Vassilvitskii, S. (2007). "k-means++: the advantages of
% careful seeding". Proceedings of the eighteenth annual ACM-SIAM symposium
% on Discrete algorithms. Society for Industrial and Applied Mathematics
% Philadelphia, PA, USA. pp. 1027-1035.

%   Copyright 2014-2022 MathWorks, Inc.
%#codegen

function [bestCenters, bestAssignments, bestDists, bestCompactness] = approximateKMeans(features, K, varargin)

if isSim
    params = parseInputs(features, K, varargin{:});
    printer = vision.internal.MessagePrinter.configure(params.Verbose);
else
    params = parseInputsCodegen(features, K, varargin{:});
end

% Create K-Means cluster assigner for numeric features.
assigner = vision.internal.kmeans.ClusterAssignmentApproximateKDTree(params.DistanceMethod);

% Create K-Means cluster updater for numeric features.
if isSim && params.UseParallel
    updater = vision.internal.kmeans.ClusterUpdateMeanParallel(K);
else
    updater = vision.internal.kmeans.ClusterUpdateMean(K);
end

% Create metric used for clustering.
metric = vision.internal.kmeans.DistanceMetricSSD();

% Create cluster initializer.
if strcmpi(params.Initialization, 'random')
    initializer = vision.internal.kmeans.ClusterInitializationRandom();
else
    if isSim
        initializer = vision.internal.kmeans.ClusterInitializationKMeansPP(metric,printer);
    else
        initializer = vision.internal.kmeans.ClusterInitializationKMeansPP(metric);
    end
end

[bestCenters, bestAssignments, bestDists, bestCompactness] = ...
    vision.internal.kmeans.clusterdata(...
    features, K, initializer, assigner, updater, metric, ...
    'NumTrials', params.NumTrials,...
    'MaxIterations', params.MaxIterations, ...
    'Threshold', params.Threshold, ...
    'UseParallel', params.UseParallel, ...
    'Verbose', params.Verbose);

%--------------------------------------------------------------------------
function params = parseInputs(features, K, varargin)

if size(features, 1) < K
    error(message('vision:kmeans:numDataGTEqK'))
end

parser = inputParser();
parser.addOptional('MaxIterations', 100, @checkMaxIterations);
parser.addOptional('Threshold', single(.0001), @checkThreshold);
parser.addOptional('Initialization', 'KMeans++');
parser.addOptional('DistanceMethod', 'L2');
parser.addOptional('Verbose', false);
parser.addOptional('NumTrials', 1, @(x)isscalar(x) && isnumeric(x));
parser.addOptional('UseParallel', vision.internal.useParallelPreference());

parser.parse(varargin{:});

initMethod = validatestring(parser.Results.Initialization, ...
                            {'Random', 'KMeans++'}, mfilename);

distanceMethod = validatestring(parser.Results.DistanceMethod, ...
                                {'L2', 'BboxIoU'}, mfilename);

if iWasUserSpecified(parser, 'DistanceMethod') ...
        && isequal(distanceMethod, 'BboxIoU') ...
        && (~ismatrix(features) || ~isequal(size(features, 2), 4))
    featureSize = join(split(num2str(size(features))),'-by-');
    error(message('vision:kmeans:invalidSizeForBboxIoU', featureSize{1}));
end

vision.internal.inputValidation.validateLogical(parser.Results.Verbose,'Verbose');
useParallel = vision.internal.inputValidation.validateUseParallel(parser.Results.UseParallel);

params.MaxIterations  = double (parser.Results.MaxIterations);
params.Threshold      = single (parser.Results.Threshold);
params.Initialization = initMethod;
params.DistanceMethod = distanceMethod;
params.Verbose        = logical(parser.Results.Verbose);
params.NumTrials      = double (parser.Results.NumTrials);
params.UseParallel    = useParallel;
params.K              = K;

%--------------------------------------------------------------------------
function params = parseInputsCodegen(features, K, varargin)

coder.internal.errorIf(size(features, 1) < K, 'vision:kmeans:numDataGTEqK');
% Set input parser
defaults = struct(...
    'MaxIterations',   100, ...
    'Threshold',       single(.0001), ...
    'Initialization',  'KMeans++', ...
    'DistanceMethod',  'L2', ...
    'NumTrials',        1, ...
    'UseParallel',      false, ...
    'Verbose',          false);

% Define parser mapping struct
pvPairs = struct( ...
    'MaxIterations',  uint32(0), ...
    'Threshold',      uint32(0), ...
    'Initialization', uint32(0), ...
    'DistanceMethod', uint32(0), ...
    'NumTrials',      uint32(0), ...
    'UseParallel',    uint32(0), ...
    'Verbose',        uint32(0));

% Specify parser options
poptions = struct( ...
    'CaseSensitivity', false, ...
    'StructExpand',    true, ...
    'PartialMatching', true);

% Parse PV pairs
pstruct = coder.internal.parseParameterInputs(pvPairs, ...
                                              poptions, varargin{:});
% Extract inputs
maxIterations   = coder.internal.getParameterValue(pstruct.MaxIterations, defaults.MaxIterations, varargin{:});
threshold       = coder.internal.getParameterValue(pstruct.Threshold, defaults.Threshold, varargin{:});
initialization  = coder.internal.getParameterValue(pstruct.Initialization, defaults.Initialization, varargin{:});
distanceMethod  = coder.internal.getParameterValue(pstruct.DistanceMethod, defaults.DistanceMethod, varargin{:});
useParallel     = coder.internal.getParameterValue(pstruct.UseParallel, defaults.UseParallel, varargin{:});
numTrials       = coder.internal.getParameterValue(pstruct.NumTrials, defaults.NumTrials, varargin{:});

checkMaxIterations(maxIterations);
checkThreshold(threshold);
validateattributes(numTrials, {'numeric'}, {'scalar'}, 'NumTrials');

initMethod = validatestring(initialization, ...
                            {'Random', 'KMeans++'}, mfilename);

distanceMethod = validatestring(distanceMethod, ...
                                {'L2', 'BboxIoU'}, mfilename);

if iWasUserSpecifiedCodegen(varargin, 'DistanceMethod') ...
        && isequal(distanceMethod, 'BboxIoU') ...
        && (~ismatrix(features) || ~isequal(size(features, 2), 4))
    featureSize = join(split(num2str(size(features))),'-by-');
    error(message('vision:kmeans:invalidSizeForBboxIoU', featureSize{1}));
end

params.MaxIterations  = double (maxIterations);
params.Threshold      = single (threshold);
params.Initialization = initMethod;
params.DistanceMethod = distanceMethod;
params.NumTrials      = double (numTrials);
params.K              = K;
params.UseParallel    = useParallel;
params.Verbose        = false;

%--------------------------------------------------------------------------
function checkMaxIterations(val)

validateattributes(val,{'numeric'}, ...
                   {'scalar','integer','positive','real','finite'}, mfilename);

%--------------------------------------------------------------------------
function checkThreshold(val)

validateattributes(val,{'numeric'}, ...
                   {'scalar','positive','real','finite'}, mfilename);

%--------------------------------------------------------------------------
function tf = iWasUserSpecified(parser,param)
tf = ~ismember(param,parser.UsingDefaults);

%--------------------------------------------------------------------------
function tf = isSim()
tf = coder.target('MATLAB');

%--------------------------------------------------------------------------
function tf = iWasUserSpecifiedCodegen(userdata, name )
tf = any(strcmp(userdata,name));
