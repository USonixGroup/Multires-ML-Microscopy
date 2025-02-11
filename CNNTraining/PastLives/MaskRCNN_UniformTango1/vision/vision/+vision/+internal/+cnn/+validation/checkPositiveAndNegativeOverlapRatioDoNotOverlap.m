function checkPositiveAndNegativeOverlapRatioDoNotOverlap(params)
% positive and negative ranges should not overlap

%   Copyright 2016-2020 The MathWorks, Inc.

if params.PositiveOverlapRange(1) < params.NegativeOverlapRange(2)
    error(message('vision:rcnn:rangesOverlap'));
end
