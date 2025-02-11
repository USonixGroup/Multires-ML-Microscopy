classdef (Abstract) EncoderStrategy < handle
%EncoderStrategy Abstract class interface for encoder methods used in
% bagOfFeatures. The Encoder is chosen based on feature type after the
% extractor output is validated.

%   Copyright 2020-2022 MathWorks, Inc.
%#codegen

    properties
        % VocabularySize Integer scalar, VocabularySize >=2. Specifies
        %                number of visual words used to encode a set of
        %                features.
        VocabularySize (1,1)

        % Normalization Specify the type of normalization applied to the
        %               visual word histogram vector. Set to either 'L2'
        %               or 'none'.
        %
        %               Default: "L2"
        Normalization string = "L2"

        % SparseOutput True or false. Set to true to return visual word
        %              histograms as sparse matrices. This reduces
        %              memory consumption for large visual vocabularies
        %              where the visual word histograms contain many
        %              zero elements.
        %
        %              Default: false
        SparseOutput logical = false

    end

    methods (Abstract)
        %------------------------------------------------------------------
        %assignVisualWords Assign features to visual words.
        %   assignments = assignVisualWords(this, features) assigns
        %   features to visual words. The output assignments is an M-by-1
        %   vector of linear indices that map each feature vector to a
        %   visual word.
        %------------------------------------------------------------------
        assignments = assignVisualWords(this, features)
    end

    methods
        function s = saveobj(this)
            s.Normalization  = this.Normalization;
            s.SparseOutput   = this.SparseOutput;
            s.VocabularySize = this.VocabularySize;
            s.Version        = 1;
        end
    end

    methods (Sealed)
        %encode Encode features into visual words histogram.
        %   h = encode(this, features) return the visual word histogram.
        %   The length of h is VocabularySize.
        %
        %   [..., words] = encode(..., locations) optionally return the
        %   visual words as a vision.internal.visualWords object.
        function [featureVector, varargout] = encode(this, features, locations)

            assignments = assignVisualWords(this, features);

            h = histcounts(assignments, 1:this.VocabularySize+1);
            featureVector = single(h);

            if strcmpi(this.Normalization,'L2')
                featureVector = featureVector ./ (norm(featureVector,2) + eps('single'));
            end

            if coder.target('MATLAB')
                if this.SparseOutput
                    % Use sparse storage to reduce memory consumption when
                    % featureVector has many zero elements.
                    featureVector = sparse(double(featureVector));
                end
            end

            if nargout == 2
                % Optionally return visual words.
                varargout{1} = vision.internal.visualWords(assignments, locations, this.VocabularySize);
            end
        end
    end
end