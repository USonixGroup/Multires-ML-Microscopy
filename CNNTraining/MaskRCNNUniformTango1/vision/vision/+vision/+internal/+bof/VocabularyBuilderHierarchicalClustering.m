classdef VocabularyBuilderHierarchicalClustering < vision.internal.bof.VocabularyBuilderStrategy
% VocabularyBuilderHierarchicalClustering Concrete implementation of
% the VocabularyBuilder class for building a vocabulary tree. This
% vocabulary builder applies other VocabularyBuilders to build each
% level of the vocabulary tree.

% Copyright 2021-2022 The MathWorks, Inc.
%#codegen
    properties
        % VocabularyBuilder The VocabularyBuilder used to cluster the
        % vocabulary at each level of the vocabulary tree.
        VocabularyBuilder

        % CreateEncoderFcn A function handle that creates an encoder to
        % assign features to the clusters generated at each level of the
        % vocabulary tree. The function handle must create an object that
        % implements the vision.internal.bof.EncoderStrategy interface.
        CreateEncoderFcn

        % NumLevels The number of levels in the vocabulary tree.
        NumLevels

        % BranchingFactor The number of branches per tree node.
        BranchingFactor

        % NumClusteringSteps The total number of clustering steps required.
        % This is used for verbose message display.
        NumClusteringSteps
    end

    methods
        function this = VocabularyBuilderHierarchicalClustering(...
            builder, encoderFcn, numLevels, branchingFactor, verbose)

            this.VocabularyBuilder = builder;
            this.CreateEncoderFcn = encoderFcn;
            this.NumLevels = numLevels;
            this.BranchingFactor = branchingFactor;
            this.NumClusteringSteps = sum(branchingFactor.^(0:this.NumLevels-1));
            this.Verbose = verbose;
        end

        %------------------------------------------------------------------
        % visualWords = create(this, features, numVisualWords)
        % create the vocabulary given the features and desired number
        % of visual words.
        function vt = create(this, features, varargin)

            printer = vision.internal.MessagePrinter.configure(this.Verbose);
            printStartingMessage(this, printer);

            % Initialize the clustering steps to 1. The steps are updated
            % during the tree contruction and used in the verbose log.
            step = 1;
            this.VocabularyBuilder.Verbose = this.Verbose;
            vt = buildVocabularyTree(this, features, this.NumLevels,[], step, printer);
        end
    end

    methods(Access = private)

        %------------------------------------------------------------------
        % Recursively builds the vocabulary tree.
        function [vt,step] = buildVocabularyTree(this, features, level, branch, step, printer)

            if level > 1

                printProgress(this, printer, level, branch, step);

                vt = vision.internal.bof.VocabularyTree;
                vt.Words = invokeVocabularyBuilder(this, features, printer);

                if size(features,1) <= this.BranchingFactor
                    % Not enough features to build the tree. Stop early. To
                    % maintain a consistent verbose log, recurse through
                    % the remainder of the tree and print informative
                    % message.
                    for i = 1:this.BranchingFactor
                        step = step + 1;
                        printDeadBranchProgress(this, printer, level-1, i, step);
                    end

                else

                    % Create an encoder and use it to assign features to a
                    % branch of the tree.
                    encoder = this.CreateEncoderFcn(vt.Words,size(vt.Words,1));
                    assignments = assignVisualWords(encoder, features);
                    % For-each branch, partition features by branch and
                    % recursively build the next level of the tree.
                    for i = 1:this.BranchingFactor

                        % Partition the features by branch assignment.
                        fgroup = features(assignments == i,:);
                        step = step + 1;

                        % Recurse down to build the next level.
                        [vt.Nodes(i), step] = buildVocabularyTree(this, fgroup, level-1, i, step, printer);
                    end
                end
            else
                % Reached the final level. Build the final vocabulary.
                printProgress(this, printer, level, branch, step);
                vt = vision.internal.bof.VocabularyTree();
                vt.Words = invokeVocabularyBuilder(this, features, printer);
            end
        end

        %------------------------------------------------------------------
        function words = invokeVocabularyBuilder(this, features, printer)
            if isempty(features)
                % Skip building a branch when features are empty.
                printer.printMessage('vision:kmeans:numFeatures',0);
                printer.printMessage('vision:bagOfFeatures:skipBranch');
                printer.linebreak;

                words = [];
            else
                % Limit branching factor to the number of features we have.
                branchingFactor = min(size(features,1), this.BranchingFactor);
                words = create(this.VocabularyBuilder, features, branchingFactor);

                % Assert our assumption that a VocabularyBuilder always
                % returns the expected number of words. This is an internal
                % contract we enforce upon VocabularyBuilder. Users will
                % not encounter this error.
                assert(size(words,1), branchingFactor,...
                       'The number of words must equal the branching factor: %d',...
                       branchingFactor);
            end
        end

        %------------------------------------------------------------------
        function printStartingMessage(this, printer)
            printer.printMessage('vision:bagOfFeatures:vocabCreation', ...
                                 this.BranchingFactor^this.NumLevels);

            printer.printMessage('vision:bagOfFeatures:vacabNumLevels', ...
                                 this.NumLevels);

            printer.printMessage('vision:bagOfFeatures:vacabBranchingFactor', ...
                                 this.BranchingFactor);

            printer.printMessage('vision:bagOfFeatures:vacabNumClusterSteps', ...
                                 this.NumClusteringSteps);

            printer.linebreak();
        end

        %------------------------------------------------------------------
        function printProgress(this, printer, level, branch, step)
            if isempty(branch)
                printer.printMessage('vision:bagOfFeatures:vocabClusterStepLevelOne', ...
                                     step,...
                                     this.NumClusteringSteps, ...
                                     this.NumLevels-level+1);

            else
                printer.printMessage('vision:bagOfFeatures:vocabClusterStep', ...
                                     step,...
                                     this.NumClusteringSteps, ...
                                     this.NumLevels-level+1,...
                                     branch);
            end
        end

        %------------------------------------------------------------------
        function step = printDeadBranchProgress(this, printer, level, branch, step)
        % Handling printing for cases when there is nothing to actually
        % do because we cannot cluster features.
            printer.printMessage('vision:bagOfFeatures:deadBranch',...
                                 step,...
                                 this.NumClusteringSteps,...
                                 this.NumLevels-level+1,...
                                 branch);

            if level > 1
                for i = 1:this.BranchingFactor
                    step = step + 1;
                    printDeadBranchProgress(this, printer, level-1, i, step);
                end
            end
        end

    end
    methods  (Access = public, Static, Hidden)
        %------------------------------------------------------------------
        function codegenClassName = matlabCodegenRedirect(~)
            codegenClassName = 'vision.internal.codegen.bof.VocabularyBuilderHierarchicalClustering';
        end
    end
end
