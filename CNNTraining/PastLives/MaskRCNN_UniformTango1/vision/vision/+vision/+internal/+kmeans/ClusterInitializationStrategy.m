classdef ClusterInitializationStrategy
% ClusterInitializationStrategy Abstract interface for defining
% cluster initialization strategies.

% Copyright 2020-2022 The MathWorks, Inc.
%#codegen

    methods (Abstract)
        %initializeClusterCenters Select initial cluster centers.
        %
        % centers = initializeClusterCenters(this, features, numClusters)
        % return selected cluster centers. The number of centers is equal
        % to numClusters. The input features are the data to be clustered.
        centers = initializeClusterCenters(this, features, numClusters)
    end
end
