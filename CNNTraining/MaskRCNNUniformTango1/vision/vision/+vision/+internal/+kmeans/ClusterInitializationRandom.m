classdef ClusterInitializationRandom < vision.internal.kmeans.ClusterInitializationStrategy
    % ClusterInitializationRandom Random cluster initialization.
    
    % Copyright 2020 The MathWorks, Inc.
    
    methods
        function centers = initializeClusterCenters(~, features, K)
            % Randomly sample cluster centers from feature vectors.
            N       = size(features, 1);
            idx     = randperm(N,K);
            centers = features(idx,:);
        end
    end
end