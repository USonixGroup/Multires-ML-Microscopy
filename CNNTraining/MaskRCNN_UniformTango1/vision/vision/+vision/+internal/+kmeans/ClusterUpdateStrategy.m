classdef ClusterUpdateStrategy
% ClusterUpdateStrategy Abstract interface for updating cluster
% centers.

% Copyright 2020-2022 The MathWorks, Inc.
%#codegen

    properties (SetAccess = protected)
        NumClusters
    end

    methods (Abstract)
        %updateClusters Update cluster centers.
        % [centers, assignments] = updateClusters(this, features, assignments, dists)
        % return cluster centers computed from features and cluster
        % assignments.
        [centers, assignments] = updateClusters(this, features, assignments, dists)
    end

    methods (Access = protected)
        %------------------------------------------------------------------
        function counts = numFeaturesPerCluster(this, assignments)
        % Return number of features per cluster.
            counts = accumarray(assignments,1,[this.NumClusters,1]);
        end

        %------------------------------------------------------------------
        function [assignments, counts] = reinitializeEmptyClusters(this,...
                                                                   assignments, dists)
        % Returns updated cluster sums, assignments and counts. For
        % each empty cluster, reinitialize it using a feature that is
        % the furthest from any other cluster center, taking care not
        % to create more empty clusters in the process.
            counts = numFeaturesPerCluster(this, assignments);

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
                    previous = assignments(idx); % previous cluster Ki
                    if counts(previous) > 1

                        % assign feature to empty cluster.
                        assignments(idx) = empty;

                        % remove feature from it's previous cluster
                        counts(previous) = counts(previous) - 1;

                        % and move feature to empty cluster
                        counts(empty) = 1;

                        clusterIsEmpty = false;
                    end
                end
            end

        end
    end
end
