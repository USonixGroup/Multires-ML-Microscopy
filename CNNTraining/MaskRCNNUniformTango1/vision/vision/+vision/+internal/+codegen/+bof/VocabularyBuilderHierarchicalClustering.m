classdef VocabularyBuilderHierarchicalClustering < vision.internal.bof.VocabularyBuilderStrategy
% VocabularyBuilderHierarchicalClustering is the codegen implementation for
% vision.internal.bof.VocabularyBuilderHierarchicalClustering

% Copyright 2022 The MathWorks, Inc.
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
        % Initialize the clustering steps to 1. The steps are updated
        % during the tree contruction and used in the verbose log.
            this.VocabularyBuilder.Verbose = this.Verbose;
            vt = buildVocabularyTree(this, features, this.NumLevels);
        end
    end

    methods(Access = private)

        %------------------------------------------------------------------
        % Builds the vocabulary tree level-wise
        % Returns struct vt with properties
        % Words : cell array which stores the words of all Nodes across all levels linearly
        % numLevels : total number of levels in the tree
        % maxWords : vocabulary size of the data
        % maxBranchingFactor : maximum branchingFactor
        % BranchingFactor: BranchingFactor used in creating the tree
        function vt = buildVocabularyTree(this, features, level)
            if level > 1
                words = invokeVocabularyBuilder(this, features);
                % Set the intial values of maxWords, maxBranchingFactor
                maxWords = 0;
                maxBranchingFactor = 0;
                % Current level of tree being processed
                currentLevel = 1;
                % Cell array that stores the features for the children of current level
                coder.varsize('featureCellArray');
                % Cell array that stores the words of all the nodes from root to current level
                coder.varsize('tree');
                % Cell array that stores the words of all the children of current level
                coder.varsize('children');
                tree = coder.nullcopy(cell(1, 1));
                tree{currentLevel, 1} = words;
                % Set the intial value of total number of levels in the result tree
                numLevels = 1;
                featureCellArray = coder.nullcopy(cell(this.BranchingFactor, 1));
                children = coder.nullcopy(cell(this.BranchingFactor, 1));
                wordsClass = class(words);
                featureClass = class(features);
                % Not enough features to build the tree
                % empty values indicate that there will be no further division of the node
                if size(features,1) <= this.BranchingFactor
                    for i = 1:this.BranchingFactor
                        children{i} = zeros(0, 0, wordsClass);
                        featureCellArray{i} = zeros(0, 0, featureClass);
                    end
                else
                    % Create an encoder and use it to assign features to a
                    % branch of the tree.
                    encoder = this.CreateEncoderFcn(words, size(words,1));
                    if isa(encoder, 'vision.internal.codegen.bof.EncoderNumericFeaturesKDTree')
                        numQueryCols = size(features, 2);
                        numVocabCols = size(words,2);
                        coder.internal.assert((isequal(numQueryCols, numVocabCols)),'vision:ocvShared:invalidInputClass');
                        coder.internal.assert((isequal(class(features), class(words))),'vision:ocvShared:invalidInputClass');
                        assignments = vision.internal.bof.EncoderNumericFeaturesKDTree.assignVisualWords(encoder.VocabularySearchTree, ...
                                                                                                         encoder.KDTreeSearchOptions, features);
                    else
                        assignments = vision.internal.codegen.bof.EncoderBinaryFeatures.assignVisualWords(encoder.Vocabulary.Features, features);
                    end
                    for i = 1:this.BranchingFactor
                        % Partition the features by branch assignment.
                        % features for children of current level
                        featureCellArray{i} = features(assignments == i,:);

                        % children of current level
                        children{i} = invokeVocabularyBuilder(this, featureCellArray{i});
                    end
                end
                currentLevel = currentLevel+1;
                % nChildren denotes the total number of nodes in next level
                nChildren = this.BranchingFactor;
                for i=currentLevel:level
                    % Check if there is atleast one node that can be branched
                    if ~isemptyFeatures(featureCellArray)
                        % Append the nodes of current level to the existing nodes in the tree
                        tree = appendTree(this, tree, children);
                        % Increment the total number of levels in the tree
                        numLevels = numLevels+1;
                        % fNext: cell array which stores the features for nodes of next level
                        fNext = coder.nullcopy(cell(nChildren*this.BranchingFactor, 1));
                        % childrenNext: cell array which stores the nodes of next level
                        childrenNext = coder.nullcopy(cell(nChildren*this.BranchingFactor, 1));
                        % Find the children of all the nodes in the current level
                        for j=1:nChildren
                            features = featureCellArray{j};
                            if size(features,1) <= this.BranchingFactor
                                % Not enough features to build the tree
                                % indicates children{j} is a leafNode
                                % update maxWords
                                if i <= level-1
                                    maxWords = maxWords+size(children{j}, 1);
                                end
                                % As tree is stored linearly features and children of each node has to be stored
                                for z = 1:this.BranchingFactor
                                    fNext{(j-1)*this.BranchingFactor+z} = zeros(0, 0, featureClass);
                                    childrenNext{(j-1)*this.BranchingFactor+z} = zeros(0, 0, wordsClass);
                                end
                            else
                                % Partition the features by branch assignment.
                                root = children{j};
                                rng(0)
                                encoder = this.CreateEncoderFcn(root,size(root,1));
                                if isa(encoder, 'vision.internal.codegen.bof.EncoderNumericFeaturesKDTree')
                                    numQueryCols = size(features, 2);
                                    numVocabCols = size(root,2);
                                    coder.internal.assert((isequal(numQueryCols, numVocabCols)),'vision:ocvShared:invalidInputClass');
                                    coder.internal.assert((isequal(class(features), class(root))),'vision:ocvShared:invalidInputClass');
                                    assignments = vision.internal.bof.EncoderNumericFeaturesKDTree.assignVisualWords(encoder.VocabularySearchTree, ...
                                                                                                                     encoder.KDTreeSearchOptions, features);
                                else
                                    assignments = vision.internal.codegen.bof.EncoderBinaryFeatures.assignVisualWords(encoder.Vocabulary.Features, features);
                                end
                                for z=1:this.BranchingFactor
                                    fgroup = features(assignments == z,:);
                                    % Store the features of next level linearly
                                    fNext{(j-1)*this.BranchingFactor+z} = fgroup;
                                    temp = invokeVocabularyBuilder(this, fgroup);
                                    % Store the nodes of next level linearly
                                    childrenNext{(j-1)*this.BranchingFactor+z} = temp;
                                end
                            end
                        end
                        % Update f cell array
                        featureCellArray = fNext;
                        % Update children cell array
                        children = childrenNext;
                        % Increment number of children
                        nChildren = nChildren*this.BranchingFactor;
                    end
                end
                leafNode = 0;
                % leafNode indicates the linear index of first node in the last level of the tree
                for i=0:(numLevels-2)
                    leafNode = leafNode+this.BranchingFactor^i;
                end
                leafNode = leafNode+1;
                for i=leafNode:numel(tree)
                    % Update maxWords with vocabulary sizes of all nodes in the last level
                    maxWords = maxWords+size(tree{i}, 1);
                end
				for i=1:numel(tree)
				    % Update maxBranchingFactor of the tree
                    maxBranchingFactor = max(maxBranchingFactor, size(tree{i}, 1));
				end
                vt.Words = tree;
                vt.numLevels = numLevels;
                vt.maxWords = maxWords;
                vt.maxBranchingFactor = maxBranchingFactor;
                vt.BranchingFactor = this.BranchingFactor;
            else
                % If number of levels in the tree is 1
                words = invokeVocabularyBuilder(this, features);
                vt.Words = {words};
                vt.numLevels = 1;
                vt.maxWords = size(words, 1);
                vt.maxBranchingFactor = size(words, 1);
                vt.BranchingFactor = this.BranchingFactor;
            end
        end

        %-------------------------------------------------------------------
        function newTree = appendTree(~, tree, children)
        % Appends children nodes to existing tree
            nTree = numel(tree);
            nChildren = numel(children);
            newTree = coder.nullcopy(cell(nTree+nChildren, 1));
            for i=1:nTree
                newTree{i} = tree{i};
            end
            for i=1:nChildren
                newTree{i+nTree} = children{i};
            end
        end

        %------------------------------------------------------------------
        function words = invokeVocabularyBuilder(this, features, varargin)
            if isempty(features)
                % Skip building a branch when features are empty.
                words = zeros(coder.ignoreConst(0), coder.ignoreConst(0), class(features));
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
    end
end
function isEmpty = isemptyFeatures(features)
    isEmpty = true;
    for i=1:numel(features)
        % Tree can be branched further
        if size(features{i}, 1) > 0
            isEmpty = false;
        end
    end
end
