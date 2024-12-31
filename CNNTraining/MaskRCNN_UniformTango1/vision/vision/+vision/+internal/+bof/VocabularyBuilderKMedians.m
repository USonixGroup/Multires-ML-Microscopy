classdef VocabularyBuilderKMedians < vision.internal.bof.VocabularyBuilderStrategy
%VocabularyBuilderKMedians Concrete implementation of
% the VocabularyBuilder class for binary feature types that uses
% K-Medians clustering with Hamming distance metric to build
% vocabulary.

%   Copyright 2020-2022 MathWorks, Inc.
%#codegen

    properties
        NumTrials

        MaxIterations

        Threshold

        UseParallel
    end

    methods

        function clusterCenters = create(this, features, numClusters)

            % Create Hamming distance metric.
            metric = vision.internal.kmeans.DistanceMetricHamming();

            % Create cluster initialization strategy.
            if coder.target('MATLAB')
                % Create a printer for verbose printing.
                printer = vision.internal.MessagePrinter.configure(this.Verbose);
                initializer = vision.internal.kmeans.ClusterInitializationKMeansPP(metric, printer);
            else
                initializer = vision.internal.kmeans.ClusterInitializationKMeansPP(metric);
            end

            % Create binary feature cluster assigner.
            assigner = vision.internal.kmeans.ClusterAssignmentBinaryFeatures();

            % Create binary feature cluster updater.
            updater = vision.internal.kmeans.ClusterUpdateMedianBinaryFeatures(numClusters);

            clusterCenters = vision.internal.kmeans.clusterdata(...
                features, numClusters, initializer, assigner, updater, metric, ...
                'NumTrials',     this.NumTrials,...
                'MaxIterations', this.MaxIterations, ...
                'Threshold',     this.Threshold, ...
                'UseParallel',   this.UseParallel, ...
                'Verbose',       this.Verbose);
        end

    end

end
