classdef ClusterUpdateMeanParallel < vision.internal.kmeans.ClusterUpdateMean
% ClusterUpdateMeanParallel Update clusters in parallel by computing
% the mean of the current cluster assignments. Only supports floating
% point features.

% Copyright 2020-2022 The MathWorks, Inc.
%#codegen

    methods

        %------------------------------------------------------------------
        function this = ClusterUpdateMeanParallel(numClusters)
            this@vision.internal.kmeans.ClusterUpdateMean(numClusters);
        end

        %------------------------------------------------------------------
        function [centers, assignments] = updateClusters(this, features, assignments, dists)
        % Returns updated cluster centers and cluster assignments. The
        % cluster update is done in parallel by computing partial
        % cluster summations in parallel and then averaging at the end.
        %
        %    1) Split up all the features into distinct sub-sets.
        %
        %    2) For each feature sub-set, sum the contribution of each
        %       feature to its assigned cluster. And keep track of the
        %       number of features belonging to each cluster. The
        %       overall cluster sum is tabulated as a parallel
        %       reduction within the parfor loop.
        %
        %    3) After all sub-sets are processed in parallel, serially
        %       compute the cluster centers using the cluster sums and
        %       counts.

            [numFeatures, featureDim] = size(features);

            % Get the current parallel pool.
            pool = gcp('nocreate');

            if isempty(pool)
                % Update clusters in serial.
                [centers, assignments] = updateClusters@vision.internal.kmeans.ClusterUpdateMean(this, features, assignments, dists);
            else
                % Divide the work evenly amongst the workers. This helps
                % minimize the number of indexing operations.
                chunkSize = floor(numFeatures/pool.NumWorkers);

                % The remainder is processed in serial.
                if chunkSize == 0
                    remainder = numFeatures;
                else
                    remainder = rem(numFeatures,chunkSize);
                end

                % Data is reshaped into 3-D array to avoid data copies between workers.
                assignmentsCube = reshape(assignments(1:end-remainder), 1, chunkSize, []);
                featuresCube    = reshape(features(1:end-remainder,:)', featureDim, chunkSize, []);

                % Process chucks of the data in parallel and compute partial cluster
                % sums. These partial sums are then averaged serially for the final
                % cluster center.
                centerSums = zeros(this.NumClusters, featureDim);
                counts     = zeros(this.NumClusters,1);
                parfor n = 1:size(featuresCube,3)
                    f = reshape(featuresCube(:,:,n),featureDim,[])';

                    a = assignmentsCube(:,:,n);

                    [partialSums, partialCounts] = sumClusterFeatures(this, f, a);

                    centerSums = centerSums + partialSums;
                    counts     = counts     + partialCounts;
                end

                % finish the remainder
                f = features(end-remainder+1:end,:);
                a = assignments(end-remainder+1:end);

                [partialSums, partialCounts] = sumClusterFeatures(this, f, a);

                centerSums = centerSums + partialSums;
                counts     = counts     + partialCounts;

                [centerSums, assignments, counts] = reinitializeEmptyClusters(this, features, assignments, centerSums, counts, dists);

                centers = computeClusterCenters(this, centerSums, counts, features);
            end

        end

        %------------------------------------------------------------------
        function s = saveobj(this)
            s.NumClusters = this.NumClusters;
        end
    end

    methods (Static)
        function this = loadobj(s)
            this = vision.internal.kmeans.ClusterUpdateMeanParallel(s.NumClusters);
        end
    end
end
