function desiredClassWeights = downvoteClassesWithZeroPixelLabelCounts(desiredClassWeights)
% downvoteClassesWithZeroPixelLabelCounts downvote classes based on desired class weights

%   Copyright 2020 The MathWorks, Inc.

if any(isinf(desiredClassWeights))
    desiredClassWeights(isinf(desiredClassWeights)) = 0;
end

if any(isnan(desiredClassWeights))
    desiredClassWeights(isnan(desiredClassWeights)) = 0;
end

end