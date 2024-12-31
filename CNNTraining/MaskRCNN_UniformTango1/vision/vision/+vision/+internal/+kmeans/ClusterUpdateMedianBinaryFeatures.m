classdef ClusterUpdateMedianBinaryFeatures < vision.internal.kmeans.ClusterUpdateStrategy
% ClusterUpdateMedianBinaryFeatures Cluster update strategy for binary
% features. Uses the cluster median to compute cluster "center".

% Copyright 2020-2022 The MathWorks, Inc.
%#codegen
    methods
        function this = ClusterUpdateMedianBinaryFeatures(numClusters)
            this.NumClusters = numClusters;
        end

        %------------------------------------------------------------------
        function [centers, assignments] = updateClusters(this, features, assignments, dists)

        % Reinitialize empty clusters.
            assignments = reinitializeEmptyClusters(this,...
                                                    assignments, dists);

            % Load assignments into sparse matrix to avoid checks within the for-loop
            [M, N] = size(features);
            assignmentMatrix = sparse(1:M, double(assignments), logical(assignments), M, this.NumClusters);
            centers = zeros(this.NumClusters,N,'uint8');
            for i = 1:this.NumClusters
                % Compute median feature vector for each cluster.
                if coder.target('MATLAB')
                    featuresInCluster = features(assignmentMatrix(:,i),:);
                    centers(i,:) = visionBinaryFeaturesMedian(featuresInCluster);
                else
                    indices = (assignments == i);
                    featuresInCluster = features(indices, :);
                    centers(i,:) = vision.internal.codegen.binaryFeaturesMedian(featuresInCluster);
                end
            end
        end
    end
    
end

