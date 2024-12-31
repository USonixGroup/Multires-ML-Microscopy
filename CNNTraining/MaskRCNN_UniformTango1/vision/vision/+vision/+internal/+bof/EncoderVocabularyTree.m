classdef EncoderVocabularyTree < vision.internal.bof.EncoderStrategy
% EncoderVocabularyTree Encoder strategy to build use a VocabularyTree
% for encoding features.

% Copyright 2021-2022 The MathWorks, Inc.
%#codegen

    properties(Access = private)
        % Encoder The EncoderStrategy object to use at each level of the
        % vocabulary tree.
        Encoder

        % Nodes An array of EncoderVocabularyTree objects representing the
        % nodes (children) of this EncoderVocabularyTree object. When
        % assigning visual words, we recurse down this list of objects.
        Nodes = vision.internal.bof.EncoderVocabularyTree.empty()
    end

    methods
        function this = EncoderVocabularyTree(vocabTree, createEncoderFcn)
            if nargin ~= 0

                words = vocabTree.Words;

                % Words can be empty if the vocabulary tree was not able to
                % generate clusters. Do not create and encoder and recurse
                % unless the words are not empty.
                if ~isempty(words)
                    this.Encoder = createEncoderFcn(words,size(words,1));

                    if ~isempty(vocabTree.Nodes)
                        for i = 1:numel(vocabTree.Nodes)
                            this.Nodes(i) = vision.internal.bof.EncoderVocabularyTree(...
                                vocabTree.Nodes(i), createEncoderFcn);
                        end
                    end
                end

                % Cache the size of the vocabulary.
                this.VocabularySize = numVisualWords(vocabTree);
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
            whichBranch = assignVisualWords(this.Encoder, features);
            offset = 0;

            if isempty(this.Nodes)
                assignments = double(whichBranch);
            else

                for i = 1:numel(this.Nodes)
                    % Select which features should be assigned down this
                    % Node.
                    whichFeatures = whichBranch == i;

                    % Recursively assign the features down this branch of
                    % the tree.
                    [assignments, offset] = iAssignRecursively(this.Nodes(i), assignments, features, whichFeatures, offset);
                end
            end

        end

        %------------------------------------------------------------------
        function s = saveobj(this)
            s = saveobj@vision.internal.bof.EncoderStrategy(this);
            s.Encoder = this.Encoder;
            s.Nodes = this.Nodes;
        end

    end

    methods (Static)
        function this = loadobj(s)
            this = vision.internal.bof.EncoderVocabularyTree();
            this.Normalization  = s.Normalization;
            this.SparseOutput   = s.SparseOutput;
            this.VocabularySize = s.VocabularySize;
            this.Encoder        = s.Encoder;
            this.Nodes          = s.Nodes;
        end
    end

    methods  (Access = public, Static, Hidden)
        %------------------------------------------------------------------
        function codegenClassName = matlabCodegenRedirect(~)
            codegenClassName = 'vision.internal.codegen.bof.EncoderVocabularyTree';
        end
    end
end

%--------------------------------------------------------------------------
function [assignments, offset] = iAssignRecursively(...
    encoderTree, assignments, features, whichFeatures, offset)

    if isempty(encoderTree.Encoder)
        % No-op. Return input assignments and offset. The encoder is empty when
        % there are no visual words in this branch of the tree. This can happen
        % if the data used to build the tree is such that no parts of it are
        % partitioned down this specific branch of the tree.

    elseif isempty(encoderTree.Nodes)
        % Reached leaf. Assign features to visual words.
        aidx = assignVisualWords(encoderTree.Encoder, features(whichFeatures,:));

        assert(iscolumn(aidx));

        % Increment assignments using offset into final feature vector and
        % update feature assignments. The assignments are indices relative to
        % the global assignment vector. The offset is updated below based on
        % the total number of words we just processed.
        assignments(whichFeatures, :) = aidx + offset;

        % Update offset for next leaf node.
        numWords = encoderTree.VocabularySize;
        offset = offset + numWords;

    else

        % Determine which branch to descend.
        aidx = assignVisualWords(encoderTree.Encoder, features(whichFeatures,:));
        assert(iscolumn(aidx));
        whichBranch = zeros(size(whichFeatures));
        whichBranch(whichFeatures,1) = aidx;

        for i = 1:numel(encoderTree.Nodes)
            whichFeatures = whichBranch == i;
            [assignments, offset] = iAssignRecursively(...
                encoderTree.Nodes(i), assignments, features, whichFeatures, offset);
        end

    end

end
