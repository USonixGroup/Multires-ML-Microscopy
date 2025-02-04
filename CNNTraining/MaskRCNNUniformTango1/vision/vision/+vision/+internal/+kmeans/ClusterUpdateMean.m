classdef ClusterUpdateMean < vision.internal.kmeans.ClusterUpdateStrategy
% ClusterUpdateMean Update clusters by computing the mean of the
% current cluster assignments. Only supports floating point features.

% Copyright 2020-2022 The MathWorks, Inc.
%#codegen

    methods

        %------------------------------------------------------------------
        function this = ClusterUpdateMean(numClusters)
            this.NumClusters = numClusters;
        end

        %------------------------------------------------------------------
        function [centers, assignments] = updateClusters(this, features, assignments, dists)

            [centerSums, counts] = sumClusterFeatures(this, features, assignments);

            [centerSums, assignments, counts] = reinitializeEmptyClusters(this, features, assignments, centerSums, counts, dists);

            centers = computeClusterCenters(this, centerSums, counts, features);

        end
    end

    methods (Access = protected)
        %--------------------------------------------------------------------------
        function [accum, counts] = sumClusterFeatures(this, features, assignments)
        % sum up features assigned to each cluster. To be used during cluster
        % update.

            [M, N] = size(features);

            K = this.NumClusters;
            accum  = zeros(K, N);
            counts = zeros(K, 1);

            % Load assignments into sparse matrix to avoid checks within the for-loop
            assignmentMatrix = sparse(1:M, double(assignments), logical(assignments), M, K);
            for k = 1:K
                indices = (assignments == k);
                accum(k,:) = sum(features(indices,:), 1, 'double');
                counts(k)  = nnz(assignmentMatrix(:,k));
            end
        end

        %--------------------------------------------------------------------------
        % Returns updated cluster sums, assignments and counts. For each empty
        % cluster, reinitialize it using a feature that is the furthest from any
        % other cluster center, taking care not to create more empty clusters in
        % the process.
        %--------------------------------------------------------------------------
        function [centerSums, assignments, counts] = reinitializeEmptyClusters(~,...
                                                                               features, assignments, centerSums, counts, dists)

            emptyClusterIdx = find(counts == 0);

            for i = 1:numel(emptyClusterIdx)

                empty = emptyClusterIdx(i);

                clusterIsEmpty = true;
                while clusterIsEmpty

                    [maxValue, idx] = max(dists);

                    if maxValue == -inf
                        % No alternate choices left.
                        break;
                    end

                    % Prevent feature from being selected again
                    dists(idx) = -inf(1,'like',dists);

                    % Remove feature from assigned cluster only if another empty
                    % cluster is not created in the process.
                    previous = assignments(idx);
                    if counts(previous) > 1

                        assignments(idx) = empty;

                        % remove feature from it's previous cluster
                        centerSums(previous, :) = centerSums(previous, :) - features(idx, :);
                        counts(previous)        = counts(previous) - 1;

                        % and move feature to empty cluster
                        centerSums(empty, :) = features(idx, :);
                        counts(empty)       = 1;

                        clusterIsEmpty = false;
                    end
                end
            end

        end

        %--------------------------------------------------------------------------
        function centers = computeClusterCenters(~, centerSums, counts, features)
            K = numel(counts);
            countInv = spdiags(1./(counts+eps), 0, K, K);  % reduce storage costs of K-by-K diagonal matrix
            centers  = cast(full(countInv * centerSums),'like',features);
        end

    end
end
