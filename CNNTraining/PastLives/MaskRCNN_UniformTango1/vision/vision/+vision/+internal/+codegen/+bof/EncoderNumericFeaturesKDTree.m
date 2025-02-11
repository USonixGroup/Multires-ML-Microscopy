classdef EncoderNumericFeaturesKDTree < vision.internal.bof.EncoderStrategy
% EncoderNumericFeaturesKDTree is the codegen implementation of
% vision.internal.EncoderNumericFeaturesKDTree

%   Copyright 2022-2023 MathWorks, Inc.
%#codegen
    properties(Access = public)
        % Vocabulary Contains the feature vectors that constitute the
        %            visual vocabulary of the bag-of-features.
        Vocabulary

        % KDTreeIndexState Random state at the time KD-Tree is created.
        %                  This is used to recreate the KD-Tree in loadobj.
        KDTreeIndexState

    end

    properties (Access = public, Transient)
        % VocabularySearchTree The KD-Tree used to map features to visual
        %                      words.
        VocabularySearchTree

        %
        KDTreeSearchOptions
    end

    properties (Access = protected) % For shared library codegen
        LocationHandle

        %
        HasLocationHandleAllocated = false;
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

    methods (Static)
        %------------------------------------------------------------------
        %assignVisualWords Assign features to visual words.
        %   assignments = assignVisualWords(this, features) assigns
        %   features to visual words. The output assignments is an M-by-1
        %   vector of linear indices that map each feature vector to a
        %   visual word.
        %------------------------------------------------------------------
        function assignments = assignVisualWords(vocabTree, searchOpts, features)
        % Encode features into visual words using a KD-Tree to map
        % features to visual words.
            if coder.internal.isTargetMATLABHost()
                assignments = vision.internal.buildable.kdtreeBuildable.kdtreeKNNSearch(vocabTree, ...
                                                                                        class(features), features, 1, searchOpts);
            else
                assignments = vision.internal.buildable.kdtreeBuildablePortable.kdtreeKNNSearch(vocabTree, ...
                    class(features), features, 1, searchOpts);
            end

            % Make assignments an M-by-1 vector.
            assignments = reshape(assignments,[],1);
        end
    end

    methods (Access = private)
        %------------------------------------------------------------------
        function initializeVocabularySearchTree(this)
            if coder.internal.isTargetMATLABHost()
                % Kd-tree of descriptors in codegen mode.
                % Construct Kdtree.
                this.VocabularySearchTree = vision.internal.buildable.kdtreeBuildable.kdtreeConstruct(class(this.Vocabulary));
                this.LocationHandle = vision.internal.buildable.kdtreeBuildable.kdtreeGetLocationPointer(this.Vocabulary, class(this.Vocabulary));
                this.HasLocationHandleAllocated = true;
                vision.internal.buildable.kdtreeBuildable.kdtreeIndex(this.VocabularySearchTree, ...
                                                                      class(this.Vocabulary), this.LocationHandle, size(this.Vocabulary, 1), size(this.Vocabulary, 2));
            else
                this.VocabularySearchTree = vision.internal.buildable.kdtreeBuildablePortable.kdtreeConstruct(class(this.Vocabulary));
                this.LocationHandle = vision.internal.buildable.kdtreeBuildablePortable.kdtreeGetLocationPointer(this.Vocabulary, class(this.Vocabulary));
                this.HasLocationHandleAllocated = true;
                vision.internal.buildable.kdtreeBuildablePortable.kdtreeIndex(this.VocabularySearchTree, ...
                                                                      class(this.Vocabulary), this.LocationHandle, size(this.Vocabulary, 1), size(this.Vocabulary, 2));

            end
        end

        %------------------------------------------------------------------
        function opts = getSearchOptions(this)
            opts = struct(...
                'eps', single(0),...
                'grainSize', int32(10000),...
                'tbbQueryThreshold', uint32(this.VocabularySize));
        end
    end
end

% LocalWords:  KD
%------------------------------------------------------------------
% delete function
%------------------------------------------------------------------
function delete(this)
            
    if coder.internal.isTargetMATLABHost()
        if this.HasLocationHandleAllocated
            vision.internal.buildable.kdtreeBuildable.kdtreeDeleteLocationPointer(this.LocationHandle, class(this.Vocabulary));
            vision.internal.buildable.kdtreeBuildable.kdtreeDelete(this.VocabularySearchTree, class(this.Vocabulary));
            this.HasLocationHandleAllocated = false;
        end
    else
        % Portable code
        if this.HasLocationHandleAllocated
            vision.internal.buildable.kdtreeBuildablePortable.kdtreeDeleteLocationPointer(this.LocationHandle, class(this.Vocabulary));
            vision.internal.buildable.kdtreeBuildablePortable.kdtreeDelete(this.VocabularySearchTree, class(this.Vocabulary));
            this.HasLocationHandleAllocated = false;
        end
    end
end
