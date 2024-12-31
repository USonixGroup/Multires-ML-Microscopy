classdef VocabularyBuilderApproximateKMeans < vision.internal.bof.VocabularyBuilderStrategy
%VocabularyBuilderApproximateKMeans Concrete implementation of
% the VocabularyBuilder class for numeric feature types that uses
% Approximate Kmeans and L2 distance metric to cluster and find the
% nearest neighbors respectively.

%   Copyright 2020-2022 MathWorks, Inc.
%#codegen

    properties
        NumTrials

        MaxIterations

        Threshold

        UseParallel
    end

    methods

        function clusterCenters = create(this, features, K)
        %create - function to create the vocabulary by calling approximate
        % K-Means to cluster the feature descriptors and assign the
        % appropriate cluster centers.

            clusterCenters = vision.internal.approximateKMeans(features, K, ...
                                                               'MaxIterations', this.MaxIterations, ...
                                                               'NumTrials', this.NumTrials, ...
                                                               'Threshold', this.Threshold, ...
                                                               'Verbose', this.Verbose, ...
                                                               'UseParallel', this.UseParallel);
        end

    end

end
