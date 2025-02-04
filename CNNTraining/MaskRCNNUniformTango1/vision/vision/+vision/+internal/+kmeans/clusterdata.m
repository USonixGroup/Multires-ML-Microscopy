%clusterdata Cluster data using K-Means style iteration.
%
%   [bestCenters, bestAssignments] = clusterdata(features, K, initializer, assigner, clusterUpdater)
%   clusters features into K groups and returns the cluster centers and the
%   feature vector assignments to each cluster.
%
%   [..., bestDist] = clusterdata(...) additionally returns the distance from
%   each feature to the cluster center.
%
%   [..., bestCompactness] = clusterdata(...) additionally returns the
%   compactness score of each cluster. The compactness measures how well
%   the data has been clustered.
%
%   Inputs
%   ------
%      features          The feature data to cluster.
%
%      K                 The number of clusters.
%
%      initializer       An object that implements the
%                        ClusterInitializationStrategy interface.
%
%      assigner          An object that implements the
%                        ClusterAssignmentStrategy interface.
%
%      clusterUpdater    An object that implements the ClusterUpdateStrategy
%                        interface.
%
%      compactnessMetric An object that implements the
%                        ClusterCompactnessStrategy interface.
%
%   [...] = cluster(...,Name,Value) specifies additional
%   name-value pair arguments described below:
%
%   MaxIterations   Maximum number of iterations before the K-Means
%                   algorithm is terminated.
%
%                   Default: 100
%
%   Threshold       When the change in the total sum of intra-cluster
%                   distances is below the Threshold, the K-Means
%                   algorithm is terminated.
%
%                   Default: 0.0001
%
%   NumTrials       The number of times the K-Means algorithm is run with
%                   different initial cluster centers. The solution with
%                   the lowest total sum of intra-cluster distances is
%                   returned.
%
%                   Default: 1
%
%   UseParallel     Use MATLAB workers to cluster data.
%
%                   Default: false
%
%   Verbose         Display clustering progress.
%
%                   Default: false


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

% Copyright 2020-2022 The MathWorks, Inc.
%#codegen

function [bestCenters, bestAssignments, bestDists, bestCompactness] = ...
    clusterdata(features, K, initializer, assigner, clusterUpdater, metric, varargin)

[params] = parseInputs(features, K, varargin{:});

% Initialize cluster compactness metric.
compactnessMetric = vision.internal.kmeans.ClusterCompactness(metric);
if isa(compactnessMetric.Metric, 'vision.internal.kmeans.DistanceMetricSSD')
    compactness = zeros(1, 1, class(features));
    bestCompactness = inf(1, 1, class(features));
else
    compactness = single(0);
    bestCompactness = single(inf);
end
if ~isempty(coder.target)
    bestCenters = zeros(coder.ignoreConst(0), coder.ignoreConst(0), class(features));
    bestAssignments = zeros(coder.ignoreConst(0), 1, 'uint32');
    bestDists = zeros(coder.ignoreConst(0), 1, 'single');
end

N = size(features,1);
if isSim
    printer = vision.internal.MessagePrinter.configure(params.Verbose);
    printer.printMessage('vision:kmeans:numFeatures', N);
    printer.printMessage('vision:kmeans:numClusters', K);
end

trial = 1;

while trial <= params.NumTrials
    centers = initializeClusterCenters(initializer, features, K);

    if isSim
        printTrialStartMessage(printer, trial, params);
        msg = printProgress(printer, '', 0, params.MaxIterations,'');
    end

    [assignments, dists, isValid] = assignDataToClusters(assigner, features, centers, params);

    % Remove invalid features that contain Inf or NaN
    if any(~isValid)
        features = features(logical(isValid),:);

        coder.internal.errorIf(isempty(features), 'vision:kmeans:allHadInfNaN');
        coder.internal.errorIf(size(features, 1) < K, 'vision:kmeans:tooManyInfNaN', K);
        if ~isempty(features) && ~(size(features, 1) < K)
            coder.internal.warning('vision:kmeans:droppingInfNaN', ...
                N-size(features,1), N);
        end

        N = size(features,1);

        % Do not count this as a valid trial.
        continue
    end

    [centers, assignments] = clusterUpdater.updateClusters(features, assignments, dists);

    prevCompactness = clusterCompactness(compactnessMetric, features, centers, assignments);
    prevDist        = dists;
    prevAssignments = assignments;

    for i = 1:params.MaxIterations
        start = tic;
        [assignments, dists] = assignDataToClusters(assigner, features, centers, params);

        % The approximate assignment strategy can produce a worse cluster
        % assignment than the previous iteration. Keep the old assignment
        % when this happens.
        idx = (prevAssignments~=assignments) & (prevDist < dists);
        assignments(idx) = prevAssignments(idx);

        % Update cluster centers.
        [centers, assignments] = clusterUpdater.updateClusters(features, assignments, dists);

        % Evaluate termination criteria
        compactness = clusterCompactness(compactnessMetric, features, centers, assignments);

        delta = abs(prevCompactness - compactness)/(prevCompactness + eps(single(1)));

        elapsedTimeMessage = sprintf('(~%.2f seconds/iteration)',toc(start));
        if isSim
            msg = printProgress(printer, msg, i, params.MaxIterations, elapsedTimeMessage);
        end

        if delta <= params.Threshold
            if isSim
                printer.printMessage('vision:kmeans:trialEnd', i);
            end
            break;
        end

        prevCompactness = compactness;
        prevDist        = dists;
        prevAssignments = assignments;
    end

    if compactness < bestCompactness
        bestCenters     = centers;
        bestAssignments = assignments;
        bestDists       = dists;
        bestCompactness = compactness;
    end

    % completed trial
    trial = trial + 1;
end
if isSim
    printer.linebreak;
end

%--------------------------------------------------------------------------
function [assignments, dists, varargout] = assignDataToClusters(assigner, features, centers, params)

% Capture the rand state for each assignment. This is used in parallel code
% paths to ensure that assignment strategies that rely on the random stream
% have deterministic results (e.g. KD-Tree).
randState = rng;

if isSim
    if params.UseParallel
        [assignments, dists, varargout{1:nargout-2}] = assignDataToClustersParallel(assigner, features, centers, randState);
    else
        [assignments, dists, varargout{1:nargout-2}] = assignDataToClustersSerial(assigner, features, centers, 0);
    end
else
    [assignments, dists, varargout{1:nargout-2}] = assignDataToClustersSerial(assigner, features, centers, 0);
end

%--------------------------------------------------------------------------
function [assignments, dists, varargout] = assignDataToClustersSerial(assigner, features, centers, randState)
[assignments, dists, varargout{1:nargout-2}] = assigner.assign(...
    features, centers, randState);

%--------------------------------------------------------------------------
function [assignments, dists, varargout] = assignDataToClustersParallel(assigner, features, centers, randState)

% outputs
assignments = [];
dists       = [];
isValid     = [];

[numFeatures, featureDim] = size(features);

% get the current parallel pool
pool = gcp();

if isempty(pool)
    [assignments, dists] = assignDataToClustersSerial(assigner, features, centers, randState);
else

    % Divide the work evenly amongst the workers. This helps minimize the
    % number of indexing operations.
    chunkSize = floor(numFeatures/pool.NumWorkers);

    % The remainder is processed in serial
    if chunkSize == 0
        remainder = numFeatures;
    else
        remainder = rem(numFeatures,chunkSize);
    end

    % features are reshaped into 3-D array to avoid data copies between
    % workers.
    featuresCube = reshape(features(1:end-remainder,:)', featureDim, chunkSize, []);

    parfor n = 1:size(featuresCube,3)

        f = reshape(featuresCube(:,:,n),featureDim,[])';

        [tassignments, tdists, tisValid] = assignDataToClustersSerial(assigner, f, centers, randState);

        assignments = [assignments; tassignments];
        dists       = [dists; tdists];
        isValid     = [isValid; tisValid];
    end

    % finish the remainder
    [tassignments, tdists, tisValid] = assignDataToClustersSerial(...
        assigner, features(end-remainder+1:end,:), centers, randState);

    assignments = [assignments; tassignments];
    dists       = [dists; tdists];
    isValid     = [isValid; tisValid];

    if nargout == 3
        varargout{1} = isValid;
    end

end

%--------------------------------------------------------------------------
function printTrialStartMessage(printer, trial, params)

if params.NumTrials > 1
    printer.printMessageNoReturn('vision:kmeans:trialStart', trial, params.NumTrials);
else
    printer.printMessageNoReturn('vision:kmeans:clustering');
end

%--------------------------------------------------------------------------
function updateMessage(printer, prevMessage, nextMessage)
backspace = sprintf(repmat('\b',1,numel(prevMessage))); % figure how much to delete
printer.printDoNotEscapePercent([backspace nextMessage]);

%--------------------------------------------------------------------------
function nextMessage = printProgress(printer, prevMessage, i, N, elapsed)
nextMessage = getString(message('vision:kmeans:clusteringProgress',i,N,elapsed));
updateMessage(printer, prevMessage, nextMessage);

%--------------------------------------------------------------------------
function iMustBeFloatOrUint8(x)
validateattributes(x,{'double','single','uint8'},{'nonempty'});

%--------------------------------------------------------------------------
function [params] = parseInputs(features, K, varargin)

if isSim
    params = parseInputsSim(varargin{:});
else
    params = parseInputsCodegen(varargin{:});
end
iMustBeFloatOrUint8(features);
validateattributes(K,{'numeric'},{'scalar','integer','positive','real','finite'});

%--------------------------------------------------------------------------
function params = parseInputsSim(args)
arguments %#ok
    args.NumTrials (1,1) double {mustBeReal, mustBeInteger, mustBePositive, mustBeFinite} = 1
    args.MaxIterations (1,1) double {mustBeReal, mustBeInteger, mustBePositive, mustBeFinite} = 100
    args.Threshold (1,1) double {mustBeReal, mustBePositive, mustBeFinite} = 0.0001
    args.UseParallel (1,1) logical {mustBeNumericOrLogical(args.UseParallel)} = false
    args.Verbose (1,1) logical {mustBeNumericOrLogical(args.Verbose)} = false
end
params.MaxIterations  = args.MaxIterations;
params.Threshold      = args.Threshold;
params.NumTrials      = args.NumTrials;
params.UseParallel    = args.UseParallel;
params.Verbose        = args.Verbose;
%--------------------------------------------------------------------------
function params = parseInputsCodegen(varargin)
% Set input parser
defaults = struct(...
    'MaxIterations',   100, ...
    'Threshold',       single(.0001), ...
    'NumTrials',        1, ...
    'UseParallel',      true, ...
    'Verbose',          false);

% Define parser mapping struct
pvPairs = struct( ...
    'MaxIterations',  uint32(0), ...
    'Threshold',      uint32(0), ...
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
useParallel     = coder.internal.getParameterValue(pstruct.UseParallel, defaults.UseParallel, varargin{:});
numTrials       = coder.internal.getParameterValue(pstruct.NumTrials, defaults.NumTrials, varargin{:});
verbose = coder.internal.getParameterValue(pstruct.Verbose, defaults.Verbose, varargin{:});

validateattributes(maxIterations,{'numeric'}, ...
                   {'scalar','integer','positive','real','finite'});
validateattributes(threshold,{'numeric'}, ...
                   {'scalar','positive','real','finite'});
validateattributes(numTrials, {'numeric'}, {'scalar'});

params.MaxIterations  = double (maxIterations);
params.Threshold      = single (threshold);
params.NumTrials      = double (numTrials);
params.UseParallel    = useParallel;
% params.Verbose = verbose;
params.Verbose = false;

function tf = isSim()
tf = coder.target('MATLAB');
