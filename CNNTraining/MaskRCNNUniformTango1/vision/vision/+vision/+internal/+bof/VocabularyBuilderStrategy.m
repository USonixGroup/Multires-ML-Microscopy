classdef (Abstract) VocabularyBuilderStrategy
% VocabularyBuilderStrategy Abstract class interface for defining
% strategies to use for clustering the features and building the
% vocabulary. VocabularyBuilderStrategy is chosen based on feature
% type.

%   Copyright 2020-2022 MathWorks, Inc.
%#codegen

    properties
        % Verbose A logical indicating whether or not verbose messages
        % should be displayed.
        Verbose logical
    end

    methods(Abstract)
        %
        % visualWords = create(this, features, numVisualWords)
        % create the vocabulary given the features and desired number
        % of visual words.
        clusterCenters = create(this, features, numVisualWords)

    end

end
