classdef ClusterAssignmentBinaryFeatures < vision.internal.kmeans.ClusterAssignmentStrategy
% ClusterAssignmentBinaryFeature Assign binary features to a cluster
% using hamming distance.

% Copyright 2020-2022 The MathWorks, Inc.
%#codegen

    methods
        function this = ClusterAssignmentBinaryFeatures()
        end

        %------------------------------------------------------------------
        function [assignments, dists, isValid] = assign(~, features, clusterCenters, ~)
        % Use matchFeatures to assign features to cluster centers. Set
        % a high match threshold to ensure all features are assigned to
        % a cluster.

            metric = 'hamming';
            method = 'exhaustive';
            isPrenormalized = false;
            uniqueMatches = false;
            isLegacyMethod = false;
            outputClass = 'single';
            maxRatioThreshold = 1;

            % Set match threshold to maximum number of bits to ensure every
            % feature is matched to a cluster center.
            matchThreshold = size(features,2) * 8;

            [matchIndices, dists] = ...
                vision.internal.matchFeatures.cvalgMatchFeatures(...
                features, clusterCenters, metric, matchThreshold, method, ...
                maxRatioThreshold, isPrenormalized, uniqueMatches, isLegacyMethod, ...
                outputClass);


            assignments = double(matchIndices(:,2));

            % All assignments are valid for binary features.
            isValid = true(size(dists));
        end

        function s = saveobj(~)
            s.Version = 1;
        end
    end

    methods(Static)
        function this = loadobj(~)
            this = vision.internal.kmeans.ClusterAssignmentBinaryFeatures();
        end
    end
end
