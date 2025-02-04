classdef ClusterCompactness
% ClusterCompactness Compute cluster compactness using a distance
% metric.

% Copyright 2020-2023 The MathWorks, Inc.
%#codegen

    properties
        Metric
    end

    methods (Sealed)
        function this = ClusterCompactness(metric)
            this.Metric = metric;
        end

        %clusterCompactness Cluster compactness
        %  compactness = clusterCompactness(features, centers, assignments)
        %  return a compactness metric each cluster. The inputs are the
        %  clustered features, cluster centers, and cluster assignments for
        %  each feature.
        function compactness = clusterCompactness(this, features, centers, assignments)
            d = this.Metric.dist(features, centers(assignments,:));
            compactness = sum(d);
        end
    end
end
