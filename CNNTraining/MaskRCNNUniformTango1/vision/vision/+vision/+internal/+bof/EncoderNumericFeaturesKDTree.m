classdef EncoderNumericFeaturesKDTree < vision.internal.bof.EncoderStrategy
% EncoderNumericFeaturesKDTree Encoder strategy for numeric features
% using KD-Tree to map features to visual words.

%   Copyright 2020-2022 MathWorks, Inc.
%#codegen
    properties(Access = private)
        % Vocabulary Contains the feature vectors that constitute the
        %            visual vocabulary of the bag-of-features.
        Vocabulary

        % KDTreeIndexState Random state at the time KD-Tree is created.
        %                  This is used to recreate the KD-Tree in loadobj.
        KDTreeIndexState


    end

    properties (Access = private, Transient)
        % VocabularySearchTree The KD-Tree used to map features to visual
        %                      words.
        VocabularySearchTree

        %
        KDTreeSearchOptions
    end

    methods
        %------------------------------------------------------------------
        function this = EncoderNumericFeaturesKDTree(vocabulary, vocabSize, randState)
            this.Vocabulary = vocabulary;
            this.VocabularySize = vocabSize;
            this.KDTreeIndexState = randState;
            this.KDTreeSearchOptions = getSearchOptions(this);
            this.initializeVocabularySearchTree();
        end

        %------------------------------------------------------------------
        function s = saveobj(this)
            s = saveobj@vision.internal.bof.EncoderStrategy(this);
            s.Vocabulary          = this.Vocabulary;
            s.KDTreeIndexState    = this.KDTreeIndexState;
        end
    end

    methods
        %------------------------------------------------------------------
        %assignVisualWords Assign features to visual words.
        %   assignments = assignVisualWords(this, features) assigns
        %   features to visual words. The output assignments is an M-by-1
        %   vector of linear indices that map each feature vector to a
        %   visual word.
        %------------------------------------------------------------------
        function assignments = assignVisualWords(this, features)
        % Encode features into visual words using a KD-Tree to map
        % features to visual words.
            assignments = this.VocabularySearchTree.knnSearch(...
                features, 1, this.KDTreeSearchOptions); % K = 1

            % Make assignments an M-by-1 vector.
            assignments = reshape(assignments,[],1);
        end
    end

    methods (Static)
        %------------------------------------------------------------------
        function this = loadobj(s)
            this = vision.internal.bof.EncoderNumericFeaturesKDTree(...
                s.Vocabulary, s.VocabularySize, s.KDTreeIndexState);
            this.Normalization = s.Normalization;
            this.SparseOutput  = s.SparseOutput;
        end
    end

    methods (Access = private)
        %------------------------------------------------------------------
        function initializeVocabularySearchTree(this)
        % Set the random state to use for the KD-Tree.
            sprev = rng(this.KDTreeIndexState);
            restoreRandState = onCleanup(@()rng(sprev));

            % Create and index a KD-Tree.
            this.VocabularySearchTree = vision.internal.Kdtree();
            index(this.VocabularySearchTree, this.Vocabulary);
        end

        %------------------------------------------------------------------
        function opts = getSearchOptions(this)
            opts = struct(...
                'checks', int32(32),...
                'eps', single(0),...
                'grainSize', int32(10000),...
                'tbbQueryThreshold', uint32(this.VocabularySize));
        end
    end
    methods  (Access = public, Static, Hidden)
        %------------------------------------------------------------------
        function codegenClassName = matlabCodegenRedirect(~)
            codegenClassName = 'vision.internal.codegen.bof.EncoderNumericFeaturesKDTree';
        end
    end
end

% LocalWords:  KD
