classdef EncoderVocabularyTree < vision.internal.bof.EncoderStrategy
    % EncoderVocabularyTree is the codegen implementation of vision.internal.EncoderVocabularyTree

    % Copyright 2022-2023 The MathWorks, Inc.
    %#codegen

    properties(Access = private)
        % Encoder The EncoderStrategy object to use at each level of the
        % vocabulary tree.
        Encoder

        % Numlevels: Total number of levels in the Encoder tree
        NumLevels

        % BranchingFactor: BranchingFactor used to build the Encoder tree
        BranchingFactor

        % VocabularySizes: Cell array which stores the size of vocabulary of each node
        VocabularySizes

        % Vocabulary: Cell array which stores the vocabulary of each node
        Vocabulary

        EncoderFunction
    end

    properties (Hidden,GetAccess = public, SetAccess = protected)
        EncoderClass
    end

    methods(Static)
        function props = matlabCodegenNontunableProperties(~)
            % Used for code generation
            props = {'EncoderClass'};
        end
    end

    methods
        function this = EncoderVocabularyTree(vocabTree, createEncoderFcn)
            if nargin ~= 0
                words = vocabTree.Words;
                vocabSize = coder.nullcopy(cell(numel(words), 1));
                this.Encoder = createEncoderFcn(words{1}, size(words{1},1));
                vocabs = coder.nullcopy(cell(numel(words), 1));
                if isa(this.Encoder, 'vision.internal.codegen.bof.EncoderNumericFeaturesKDTree')
                    this.EncoderClass = 'EncoderNumericFeaturesKDTree';
                else
                    this.EncoderClass = 'EncoderBinaryFeatures';
                end
                for i=1:numel(words)
                    if strcmp(this.EncoderClass, 'EncoderNumericFeaturesKDTree')
                        vocabs{i} = words{i};
                    else
                        if ~isempty(words{i})
                            encoder = createEncoderFcn(words{i}, size(words{i},1));
                            vocabs{i} = encoder.Vocabulary.Features;
                        else
                            vocabs{i} = words{i};
                        end
                    end
                end

                % Evaluate the vocabulary size of each node in the tree
                for j=vocabTree.numLevels:-1:1
                    % leafNode indicates the linear index of first node in the jth level
                    leafNode = 0;
                    for i=0:(j-2)
                        leafNode = leafNode+vocabTree.BranchingFactor^i;
                    end
                    leafNode = leafNode+1;
                    % Vocabulary sizes of nodes in the last level of the tree
                    if j == vocabTree.numLevels
                        for i=leafNode:numel(words)
                            vocabSize{i, 1} = size(words{i}, 1);
                        end
                    else
                        % numNodesInLevel: number of nodes in the current level
                        numNodesInLevel = vocabTree.BranchingFactor^(j-1);
                        % childStartIndex: linear index of the first child of current leafNode
                        childStartIndex = leafNode+numNodesInLevel;
                        % Iterate through all nodes of current level
                        for i=leafNode:(leafNode+numNodesInLevel-1)
                            sum = 0;
                            emptyNodes = 0;
                            % Check if all children of ith node are empty
                            for z=childStartIndex:(childStartIndex+vocabTree.BranchingFactor-1)
                                if ~isempty(words{z, 1})
                                    sum = sum+vocabSize{z, 1};
                                else
                                    emptyNodes = emptyNodes+1;
                                end
                            end
                            % If all children of ith node are empty
                            if isequal(emptyNodes, vocabTree.BranchingFactor)
                                % VocabularySize of ith node is size(words{i}, 1)
                                vocabSize{i, 1} = size(words{i}, 1);
                            else
                                % VocabularySize of ith node is sum of vocabulary sizes
                                % of all the children of ith node
                                vocabSize{i, 1} = sum;
                            end
                            % Update the startIndex of child of i+1 node
                            childStartIndex = childStartIndex+vocabTree.BranchingFactor;
                        end
                    end
                end
                this.Encoder = createEncoderFcn(words{1}, size(words{1},1));
                this.VocabularySize = vocabSize{1};
                this.BranchingFactor = vocabTree.BranchingFactor;
                this.NumLevels = vocabTree.numLevels;
                this.VocabularySizes = vocabSize;
                this.Vocabulary = vocabs;
                this.EncoderFunction = createEncoderFcn;
            end
        end

        %------------------------------------------------------------------
        %assignVisualWords Assign features to visual words.
        %   assignments = assignVisualWords(this, features) assigns
        %   features to visual words. The output assignments is an M-by-1
        %   vector of linear indices that map each feature vector to a
        %   visual word.
        %------------------------------------------------------------------
        function assignments = assignVisualWords(this, features)
            % Kick-off the assignments from the top-level of the tree.
            assignments = zeros(size(features,1),1);
            if numel(this.Vocabulary) > 1
                % leaf node is an array which stores the linear index of
                % first node of levels 2 to last level
                leafNodes = zeros(this.NumLevels-1, 1);
                leafNodes(1) = 2;
                for j=2:this.NumLevels-1
                    leafNodes(j) = leafNodes(j-1)+(this.BranchingFactor ^ (j-1));
                end
            else
                leafNodes = 1;
            end
            curLevel = coder.ignoreConst(2);
            offset = 0;
            if strcmp(this.EncoderClass, 'EncoderNumericFeaturesKDTree')
                numQueryCols = size(features, 2);
                numVocabCols = size(this.Vocabulary{1},2);
                coder.internal.assert((isequal(numQueryCols, numVocabCols)),'vision:ocvShared:invalidInputClass');
                coder.internal.assert((isequal(class(features), class(this.Vocabulary{1}))),'vision:ocvShared:invalidInputClass');
                [searchTree, searchOptions] = getTree(this.Vocabulary, 1, this.EncoderFunction);
                whichBranch = vision.internal.bof.EncoderNumericFeaturesKDTree.assignVisualWords(searchTree, searchOptions, features);
            else
                whichBranch = vision.internal.bof.EncoderBinaryFeatures.assignVisualWords(this.Vocabulary{1}, features);
            end

            if numel(this.Vocabulary)==1
                assignments = double(whichBranch);
            else
                for i=1:this.BranchingFactor
                    % Select which features should be assigned down this
                    % Node.
                    whichFeatures = whichBranch == i;

                    % Recursively assign the features down this branch of
                    % the tree.
                    [assignments, offset] = iAssignRecursively(this.Vocabulary, this.EncoderFunction, ...
                        this.VocabularySizes, assignments, ...
                        features, whichFeatures, offset, i+1, curLevel, ...
                        this.NumLevels, this.BranchingFactor, leafNodes, this.EncoderClass);
                end
            end
        end

        %------------------------------------------------------------------
        function s = saveobj(this)
            s = saveobj@vision.internal.bof.EncoderStrategy(this);
            if strcmp(this.EncoderClass, 'EncoderNumericFeaturesKDTree')
                s.EncoderVocab = this.Encoder.Vocabulary;
            else
                s.EncoderVocab = this.Encoder.Vocabulary.Features;
            end
            s.EncodervSize = this.Encoder.VocabularySize;
            s.VocabularySizes = this.VocabularySizes;
            s.BranchingFactor = this.BranchingFactor;
            s.NumLevels = this.NumLevels;
            s.Vocabulary = this.Vocabulary;
            s.EncoderFunction = this.EncoderFunction;
            s.VocabularySize = this.VocabularySize;
        end

    end

    methods (Static)
        function this = loadobj(s)
            this = vision.internal.bof.EncoderVocabularyTree();
            this.Normalization  = s.Normalization;
            this.SparseOutput   = s.SparseOutput;
            this.VocabularySize = s.VocabularySize;

            this.BranchingFactor = s.BranchingFactor;
            this.NumLevels = s.NumLevels;
            this.VocabularySizes = s.VocabularySizes;
            vocabs = s.Vocabulary;
            coder.varsize('vocabs')
            this.Vocabulary = vocabs;
            this.EncoderFunction = s.EncoderFunction;
            this.Encoder        = this.EncoderFunction(s.EncoderVocab, s.EncodervSize);
        end
    end
end
%--------------------------------------------------------------------------
function [assignments, offset] = iAssignRecursively(...
    vocabulary, encoderFunction, vocabSizes, ...
    assignments, features, whichFeatures, offset, ...
    curNodeIdx, curLevel, numLevels, bFactor, leafNodes, encoderClass)

if isempty(vocabulary{curNodeIdx})
    % No-op. Return input assignments and offset. The encoder is empty when
    % there are no visual words in this branch of the tree. This can happen
    % if the data used to build the tree is such that no parts of it are
    % partitioned down this specific branch of the tree.

elseif curLevel == numLevels
    % Current level is the last level of the tree
    if strcmp(encoderClass, 'EncoderNumericFeaturesKDTree')
        numQueryCols = size(features, 2);
        numVocabCols = size(vocabulary{curNodeIdx},2);
        coder.internal.assert((isequal(numQueryCols, numVocabCols)),'vision:ocvShared:invalidInputClass');
        coder.internal.assert((isequal(class(features), class(vocabulary{curNodeIdx}))),'vision:ocvShared:invalidInputClass');
        [searchTree, searchOptions] = getTree(vocabulary, curNodeIdx, encoderFunction);
        aidx = vision.internal.bof.EncoderNumericFeaturesKDTree.assignVisualWords(searchTree, ...
            searchOptions, features(whichFeatures,:));
    else
        aidx = vision.internal.bof.EncoderBinaryFeatures.assignVisualWords( ...
            vocabulary{curNodeIdx}, features(whichFeatures,:));
    end

    assert(iscolumn(aidx));
    assignments(whichFeatures, :) = aidx + offset;
    numWords = vocabSizes{curNodeIdx};
    offset = offset + numWords;

else

    empty = 0;
    % Linear index of the first child of current node
    childNodeIdx = (curNodeIdx-leafNodes(curLevel-1))*(bFactor) + leafNodes(curLevel);
    % Check if all the children are empty
    for i = 1:bFactor
        if isequal(vocabSizes{childNodeIdx+i-1}, 0)
            empty = empty+1;
        end
    end
    if empty == bFactor
        if strcmp(encoderClass, 'EncoderNumericFeaturesKDTree')
            % Update assignments as current node is a leaf node
            numQueryCols = size(features, 2);
            numVocabCols = size(vocabulary{curNodeIdx},2);
            coder.internal.assert((isequal(numQueryCols, numVocabCols)),'vision:ocvShared:invalidInputClass');
            coder.internal.assert((isequal(class(features), class(vocabulary{curNodeIdx}))),'vision:ocvShared:invalidInputClass');
            [searchTree, searchOptions] = getTree(vocabulary, curNodeIdx, encoderFunction);
            aidx = vision.internal.bof.EncoderNumericFeaturesKDTree.assignVisualWords( ...
                searchTree, ...
                searchOptions, features(whichFeatures,:));
        else
            aidx = vision.internal.bof.EncoderBinaryFeatures.assignVisualWords( ...
                vocabulary{curNodeIdx}, features(whichFeatures,:));
        end
        assert(iscolumn(aidx));
        assignments(whichFeatures, :) = aidx + offset;

        % Update offset for next leaf node.
        numWords = vocabSizes{curNodeIdx};
        offset = offset + numWords;
    else
        % Determine which branch to descend.
        if strcmp(encoderClass, 'EncoderNumericFeaturesKDTree')
            numQueryCols = size(features, 2);
            numVocabCols = size(vocabulary{curNodeIdx},2);
            coder.internal.assert((isequal(numQueryCols, numVocabCols)),'vision:ocvShared:invalidInputClass');
            coder.internal.assert((isequal(class(features), class(vocabulary{curNodeIdx}))),'vision:ocvShared:invalidInputClass');
            [searchTree, searchOptions] = getTree(vocabulary, curNodeIdx, encoderFunction);
            aidx = vision.internal.bof.EncoderNumericFeaturesKDTree.assignVisualWords(searchTree, ...
                searchOptions, features(whichFeatures,:));
        else
            aidx = vision.internal.bof.EncoderBinaryFeatures.assignVisualWords( ...
                vocabulary{curNodeIdx}, features(whichFeatures,:));
        end
        assert(iscolumn(aidx));
        whichBranch = zeros(size(whichFeatures));
        whichBranch(whichFeatures,1) = aidx;

        for i = 1:bFactor
            whichFeatures = whichBranch == i;
            nextLevel = coder.ignoreConst(curLevel+1);
            [assignments, offset] = iAssignRecursively(...
                vocabulary, encoderFunction, vocabSizes, ...
                assignments, features, whichFeatures, offset, ...
                childNodeIdx+i-1, nextLevel, numLevels, bFactor, leafNodes, encoderClass);
        end
    end
end
end

function [searchTree, searchOptions] = getTree(vocabulary, nodeIdx, encoderFunction)
encoder = encoderFunction(vocabulary{nodeIdx}, size(vocabulary{nodeIdx},1));
searchTree = encoder.VocabularySearchTree;
searchOptions = encoder.KDTreeSearchOptions;
end
