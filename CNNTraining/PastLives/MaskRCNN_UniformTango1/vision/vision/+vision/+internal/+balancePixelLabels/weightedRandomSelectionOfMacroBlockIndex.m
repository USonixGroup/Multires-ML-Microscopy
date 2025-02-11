function idx = weightedRandomSelectionOfMacroBlockIndex(weights)
% weightedRandomSelectionOfMacroBlockIndex Weighted random selection of an
% index corresponding to the cell array containing the list of macroblocks.

%   Copyright 2020 The MathWorks, Inc.

replace = true;
% Don't consider weights that have zero values. We assume that weights are
% sorted in descending order such that all zero values are grouped at the
% end of the weights vector
[~,zerosIdx] = find(weights == 0);
if isempty(zerosIdx)
    weightsIndices = 1:length(weights);
else
    weightsIndices = 1:min(zerosIdx)-1;
end

if ~isempty(weightsIndices)
    idx = vision.internal.balancePixelLabels.weightedRandomSample(weightsIndices,1,replace,weights(weightsIndices));
else
    % If all weights are zeros, return a random index
    idx = vision.internal.balancePixelLabels.weightedRandomSample(1:length(weights),1,replace);
end

end