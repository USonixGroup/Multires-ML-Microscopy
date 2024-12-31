function [metrics, queries, gallerySets] = evaluateReidentificationNetwork(features, labels, options)
%

% Copyright 2023 The MathWorks, Inc.

arguments
    features {mustBeNumeric, mustBeNonsparse, mustBeReal, mustBeFinite, mustBeNonempty}
    labels
    options.DistanceMetric {mustBeMember(options.DistanceMetric,{'cosine','euclidean-squared'})} = 'cosine'
    options.RandomizeQueries {mustBeNumericOrLogical, mustBeScalarOrEmpty, mustBeFinite} = true
    options.Rank = 1
    options.Verbose {mustBeNumericOrLogical, mustBeScalarOrEmpty, mustBeFinite} = false
end

numQueries = size(features,2);
labels = iValidateLabels(labels,numQueries);

maxRank =length(unique(labels));
ranks = iValidateRank(options.Rank,maxRank);

metrics = reidentificationMetrics.compute(features,labels,options.DistanceMetric,...
    ranks,numQueries,options.RandomizeQueries,options.Verbose);
queries = gatherQueries(metrics);
gallerySets = gatherGallerySets(metrics);

end

%--------------------------------------------------------------------------
% Validation functions
%--------------------------------------------------------------------------
function ranks = iValidateRank(rank,maxRank)
    if isnumeric(rank)
        validateattributes(rank, {'numeric'}, {'nonempty','nonsparse','real',...
            'finite','integer','positive','nrows',1,'<=',maxRank,'nonnan',...
            'increasing'},mfilename,"Rank");
        if length(rank)==length(unique(rank))
            ranks = rank;
        else
            % Error if a rank is repeated. This should not be reached due
            % to increasing validation attibute.
            error(message('vision:reidentification:invalidRanks'));
        end
    else
        if strcmpi(rank,"All")
            ranks = 1:maxRank;
        else
            error(message('vision:reidentification:invalidRankType'));
        end
    end
end

%--------------------------------------------------------------------------
function labels = iValidateLabels(labels,numQueries)
    % labels = convertStringsToChars(labels);
    if isequal(class(labels),'char')
        labels = cellstr(labels);
    end
    if iscell(labels)
        if isempty(labels)
            error(message('vision:reidentification:invalidMetricsLabelEmpty'));
        end
        if ~iscellstr(labels)
            error(message('vision:reidentification:invalidMetricsLabelType'));
        end

        labels = string(labels);
    end

    if iscategorical(labels)
        labels = string(labels);
    end

    % Valid labels will be string arrays at this point, but if an invalid
    % label type is entered, the additional accepted class types are to be
    % displayed.
    validateattributes(labels, {'string', 'char', 'categorical'}, ...
        {'numel', numQueries}, mfilename, "Labels");

    if ~isrow(labels)
        labels = labels';
    end
end
