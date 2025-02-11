classdef EncoderBinaryFeatures < vision.internal.bof.EncoderStrategy
% EncoderBinaryFeatures Binary feature encoding strategy.

% Copyright 2020-2022 The MathWorks, Inc.
%#codegen

    properties
        % Vocabulary Contains the feature vectors that constitute the
        %            visual vocabulary of the bag-of-features.
        Vocabulary
    end

    methods
        %------------------------------------------------------------------
        function this = EncoderBinaryFeatures(vocabulary, vocabSize)
            this.Vocabulary = binaryFeatures(vocabulary);
            this.VocabularySize = vocabSize;
        end

        %------------------------------------------------------------------
        function s = saveobj(this)
            s = saveobj@vision.internal.bof.EncoderStrategy(this);
            s.Vocabulary = this.Vocabulary;

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
        % Match features to nearest vocabulary word and generated
        % histogram of visual words.

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

            vocabularyFeatures = this.Vocabulary.Features;

            assignments = vision.internal.matchFeatures.cvalgMatchFeatures(...
                features, vocabularyFeatures, metric, matchThreshold, method, ...
                maxRatioThreshold, isPrenormalized, uniqueMatches, isLegacyMethod, ...
                outputClass);

            assignments = assignments(:,2);
        end
    end

    methods (Static)
        function this = loadobj(s)
            this = vision.internal.bof.EncoderBinaryFeatures(...
                s.Vocabulary.Features, s.VocabularySize);
        end
    end

    methods  (Access = public, Static, Hidden)
        %------------------------------------------------------------------
        function codegenClassName = matlabCodegenRedirect(~)
            codegenClassName = 'vision.internal.codegen.bof.EncoderBinaryFeatures';
        end
    end
end
