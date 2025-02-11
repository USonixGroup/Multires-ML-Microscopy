classdef ClusterAssignmentStrategy
% ClusterAssignmentStrategy Abstract interface for assigning features
% to clusters.

% Copyright 2020-2022 The Mathworks, Inc.
%#codegen

    methods(Abstract)
        % assign Assign features to clusters.
        %
        % [assignments, dists] = assign(features, clusterCenters) assign
        % features to cluster centers using DistanceMetric. Return the
        % cluster assignments and the distance from each feature to the
        % cluster center.
        %
        % [..., isValid] = assign(features, clusterCenters, randState)
        % optionally return which assignments are valid.
        [assignments, dists, varargout] = assign(features, clusterCenters, randState)

        % saveobj Serialize object to support serial and parallel
        % assignment.
        s = saveobj(this)
    end

    methods (Abstract, Static)
        % loadobj Deserialize object to support serial and parallel
        % assignment.
        this = loadobj(s)
    end
end
