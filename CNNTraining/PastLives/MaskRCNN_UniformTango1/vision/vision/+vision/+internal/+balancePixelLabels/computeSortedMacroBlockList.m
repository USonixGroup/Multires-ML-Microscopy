function [classBags, idxList] = computeSortedMacroBlockList(macroBlockList,classIdx)
% computeSortedMacroBlockList The function outputs sorted pixel counts for
% the class specified by classIdx. The pixel counts in macro blocks are
% sorted in descending order for a given class. It also returns a sort
% index list which specifies how the pixel counts for a class label were
% rearranged to get the sorted output

%   Copyright 2020 The MathWorks, Inc.

% Sort the pixel count for the specified class by descending value
histgrams = cat(1,macroBlockList{:,2});
[sortedClassPixelCounts, idxList] = sort(histgrams(:,classIdx),'descend');

% Output sorted pixel counts for the class specified by classIdx  
classBags = macroBlockList(idxList,:);
for idx = 1:length(sortedClassPixelCounts)
    classBags{idx,2} = sortedClassPixelCounts(idx);
end
end