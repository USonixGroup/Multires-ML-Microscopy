classdef ClusterInitializationKMeansPP < vision.internal.kmeans.ClusterInitializationStrategy
% ClusterInitializationKMeansPP KMeans++ cluster initialization.

% Copyright 2020-2023 The MathWorks, Inc.
%#codegen

    properties(Access = private)
        % Metric Distance metric used to compute distances between
        %        features. Metric must be an object that implements the
        %        vision.internal.kmeans.DistanceMetricStrategy.
        Metric

        % VerbosePrinter Verbose printing utility object.
        VerbosePrinter
    end

    methods
        %------------------------------------------------------------------
        function this = ClusterInitializationKMeansPP(metric, varargin)
            this.Metric = metric;
            if isSim
                this.VerbosePrinter = varargin{1};
            end
        end

        %------------------------------------------------------------------
        function centers = initializeClusterCenters(this, features, numClusters)
        % Initialize cluster centers using KMeans++.
            [M,N] = size(features);

            centers       = zeros(numClusters, N, 'like', features);
            centerIndices = zeros(1,numClusters);
            
            coder.varsize('minDistances', [inf inf], [1, 1]);
            minDistances = inf(M, 1);

            one = ones(1,'like', features);

            % Select first center randomly
            rng(0);
            centerIndices(1) = randi(M,1);
            centers(1, :)    = features(centerIndices(1), :);
            if isSim
                this.VerbosePrinter.printMessageNoReturn('vision:kmeans:initialization');
            end

            % Transpose feature vectors outside loop. Set metric
            % FeatureDimension to 1 to process features as column vectors.
            featuresTransposed = features';
            this.Metric.FeatureDimension = 1;

            msg = '';
            for k = 2:numClusters
                % Randomly select next cluster center based on weighted distances to
                % current set of cluster centers. This biases the center selection
                % towards those centers that are furthest away from existing centers.
                if isSim
                    msg = iPrintInitProgress(this.VerbosePrinter, msg, k, numClusters);
                end

                % Compute squared distances from features to the newest cluster center
                dists = this.Metric.sqdist(featuresTransposed, centers(k-1,:)');

                % Update the minimum distances to the cluster centers
                minDistances = double(min(dists, minDistances));

                samplingWeights = bsxfun(@rdivide, minDistances, ...
                                         sum(minDistances) + eps(class(minDistances)));

                % Weighted sampling using the minimum distances as weights.
                edges = [0; cumsum(samplingWeights)];

                edges(end)         = one; % CDF must end at 1
                edges(edges > one) = one; % and must have all values <= 1
                if all(isfinite(edges))
                    centerIndices(k) = discretize(rand(1), edges);
                else
                    % pick a random center when edges have Infs or NaNs
                    centerIndices(k) = randi(M,1);
                end

                centers(k, :) = features(centerIndices(k), :);

            end
            if isSim
                this.VerbosePrinter.print('.\n');
            end
        end
    end
end

%--------------------------------------------------------------------------
function nextMessage = iPrintInitProgress(printer, prevMessage, k, K)
    nextMessage = sprintf('%.2f%%%%',100*k/K);
    iUpdateMessage(printer, prevMessage(1:end-1), nextMessage);
end

%--------------------------------------------------------------------------
function iUpdateMessage(printer, prevMessage, nextMessage)
    backspace = sprintf(repmat('\b',1,numel(prevMessage))); % figure how much to delete
    printer.printDoNotEscapePercent([backspace nextMessage]);
end

%--------------------------------------------------------------------------
function tf = isSim()
tf = coder.target('MATLAB');
end
