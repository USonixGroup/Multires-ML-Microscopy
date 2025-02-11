function idx = weightedRandomSelectionOfClassBagIndex(weights)
% weightedRandomSelectionOfClassBagIndex weighted random selection of an
% entry in the bag. Returns the index of an entry in the classBag cell
% array containing the sorted list of counts corresponding to the occurence
% of a class in each macroblock.

%   Copyright 2020 The MathWorks, Inc.

replace = true;
weightsIndices = 1:length(weights);
if any(weights)
    idx = vision.internal.balancePixelLabels.weightedRandomSample(weightsIndices,1,replace,weights);
else
    % If all weights are zeros, return a random index    
    idx = vision.internal.balancePixelLabels.weightedRandomSample(weightsIndices,1,replace);
end

end