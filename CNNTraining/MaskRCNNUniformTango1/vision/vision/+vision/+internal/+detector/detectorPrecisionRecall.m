function [ap, precision, recall] = detectorPrecisionRecall(labels, numExpected, scores, varargin)
% Compute average precision metric for detector results. Follows
% PASCAL VOC 2011 average precision metric. labels greater than
% zero are for a positive samples and smaller than zero for
% negative samples.

%   Copyright 2016-2020 The MathWorks, Inc.

% Note, if scores is empty, only a global value of precision and recall can
% be computed. ap is set to be precision value in this case.


returnNaNforMissingClass = false;
if ~isempty(varargin)
    % Boolean Flag for outputting NaN values for missing classes
    returnNaNforMissingClass = varargin{1};
end

if (returnNaNforMissingClass && isempty(labels) && numExpected == 0)
    ap = NaN;
    precision = NaN;
    recall = NaN;
    return;
end

if (isempty(labels) || numExpected == 0)
    ap = 0;
    precision = 1;
    recall = 0;
    return;
end

hasScore = (nargin >= 3);
if hasScore
    [~, idx] = sort(scores, 'descend');
    labels = labels(idx);

    tp = labels > 0;
    fp = labels <= 0;

    tp = cumsum(tp);
    fp = cumsum(fp);

    precision = tp ./ (tp + fp);
    recall = tp ./ numExpected;

    % Change in recall for every true positive.
    deltaRecall = 1/numExpected; 

    ap = sum( precision .* (labels>0) ) * deltaRecall;

    % By convention, start precision at 1 and recall at 0
    precision = [1; precision];
    recall    = [0; recall];
else
    tp = sum(labels > 0);
    fp = sum(labels <= 0);

    precision = tp / (tp + fp);
    recall = tp / numExpected;
    ap = precision;
end
