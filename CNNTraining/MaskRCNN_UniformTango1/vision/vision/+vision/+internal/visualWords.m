%visualWords Object for storing visual words
%
%  visualWords Object stores the visual words representing an image.
%
%  A visualWords object is used to store visual word data encoded from an
%  image using the bag of features algorithm. The words and their locations
%  within an image are stored.
%
%  visualWords are returned by the bagOfFeatures encode method, and the
%  retrieveImages function. Rely on these functions to create visualWord
%  objects.
%
%  visualWords read-only properties:
%    Word           - A vector of visual word identifiers.
%    Location       - Visual word locations within an image.
%    VocabularySize - Number of visual words in the vocabulary.
%    Count          - Number of visual words held by the object.
%
%  visualWords methods:
%    spatialMatch - Find matching visual words using spatial constraints.
%
% Example
% -------
%  setDir  = fullfile(toolboxdir('vision'),'visiondata','imageSets');
%  imgSets = imageSet(setDir, 'recursive');
%
%  trainingSets = partition(imgSets, 2); % pick the first 2 images from each set
%  bag = bagOfFeatures(trainingSets);    % bag creation can take a few minutes
%
%  I = read(imgSets(1), 1);
%  [wordHistogram, words] = encode(bag, I);
%
% See also bagOfFeatures, retrieveImages, indexImages, invertedImageIndex
%
%#codegen
%
%   Copyright 2014-2022 The MathWorks, Inc.
classdef visualWords < vision.internal.visualWordsImpl

    %======================================================================
    methods
        function this = visualWords(varargin)
            this = this@vision.internal.visualWordsImpl(varargin{:});
        end
    end

    %======================================================================
    methods(Hidden, Static, Access=private)
        function params = parseSpatialMatchInput(words1, words2, args)

            arguments
                words1 (:,:) {vision.internal.visualWords.checkWords(words1,'words1')}
                words2 (:,:) {vision.internal.visualWords.checkWords(words2,'words2')}
                args.NumNeighbors {vision.internal.visualWords.checkScalar(args.NumNeighbors, 'NumNeighbors')} = 10;
                args.MatchThreshold {vision.internal.visualWords.checkScalar(args.MatchThreshold, 'MatchThreshold')} = 1;
            end

            if words1.VocabularySize ~= words2.VocabularySize
                error(message('vision:visualWords:vocabSizeNotEqual'));
            end

            params.NumNeighbors   = double(args.NumNeighbors);
            params.MatchThreshold = double(args.MatchThreshold);

            if params.MatchThreshold > params.NumNeighbors
                error(message('vision:visualWords:thresholdGTNumNeighbors'));
            end

        end

        % -----------------------------------------------------------------
        function checkWords(words,name)
            validateattributes(words, {'vision.internal.visualWords'},...
                               {'scalar'}, mfilename, name);
        end
    end

    %======================================================================
    methods(Hidden)
        function s = saveobj(this)
            s.WordIndex      = this.WordIndex;
            s.Location       = this.Location;
            s.VocabularySize = this.VocabularySize;
        end

        % -----------------------------------------------------------------
        function this = addROIOffset(this,roi)
        % Add offset to location to compensate for ROI.
            this.Location = vision.internal.detector.addOffsetForROI(this.Location, roi, true);
        end

        %------------------------------------------------------------------
        function [indexPairs, score] = spatialMatch(words1, words2, varargin)
        %spatialMatch Find matching visual words using spatial constraints
        %
        % indexPairs = spatialMatch(words1, words2) returns a M-by-2
        % matrix containing indices to the visual words most likely to
        % correspond between words1 and words2. words1 and words2 must
        % be visualWords objects. Visual words in words1 are matched to
        % those in words2 only if they share at least 1 common
        % neighbor.
        %
        % [..., score] = spatialMatch(...) optionally returns a score
        % metric based on the number of matching visual words. The
        % score ranges between 0 and 1. The score is 1 when all the
        % visual words in words1 are spatially matched to visual words
        % in words2.
        %
        % [...] = spatialMatch(..., Name,Value) specifies additional
        % name-value pair arguments described below:
        %
        %   'NumNeighbors'    Specify the number of nearest neighbors
        %                     to consider when looking for shared
        %                     neighbors between a pair of words.
        %
        %                     Default: 10
        %
        %   'MatchThreshold'  Specify the number of common neighbors
        %                     required for a match.
        %
        %                     Default: 1
        %

            params = vision.internal.visualWords.parseSpatialMatchInput(words1,words2,varargin{:});

            hist1 = vision.internal.visualWordsImpl.createSparseHistogram(words1);
            hist2 = vision.internal.visualWordsImpl.createSparseHistogram(words2);

            matchingWords = uint32(find(hist1 & hist2));

            indexPairs = visionSpatialMatchVisualWords(words1.WordIndex,...
                                                       words2.WordIndex, words1.Location, words2.Location,...
                                                       matchingWords, params.NumNeighbors, params.MatchThreshold);

            % The score is the number of spatially matched words over total
            % number of words. A perfect score is 1.
            numMatched = size(indexPairs,1);

            score = numMatched ./ (words1.Count + eps);

        end

    end

    %======================================================================
    methods(Static,Hidden)
        function this = loadobj(s)
            this = vision.internal.visualWords(s.WordIndex, s.Location, s.VocabularySize);
        end
    end

    methods(Access = public, Static, Hidden)
        %------------------------------------------------------------------
        function codegenClassName = matlabCodegenRedirect(~)
            codegenClassName = 'vision.internal.codegen.visualWords';
        end
    end

end
