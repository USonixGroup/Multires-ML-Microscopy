function [classBags, classBagsIndices] = groupByClassAndSortMacroBlockPixelCounts(macroBlockPixelCounts, classNames)
% groupByClassAndSortMacroBlockPixelCounts Create bags for each class
% containing sorted pixel counts corresponding to each macroblock. Also,
% store the corresponding indices for each sorted entry in the bag.
% 
%  classBags        - numClasses-by-1 cell array where each element has numMacroblocks-by-3 entries
%                         Column 1 - Image Number corresponding to the macroblock
%                         Column 2 - Pixel counts of the kth class for a given macroblock (scalar)
%                         Column 3 - Macroblock origin (1-by-numDimsOfBlockedImage)
%  classBagsIndices - numClasses-by-1 cell array where each element has numMacroblocks-by-1 entries
%                         Sorted indices for the entries in classBags

%   Copyright 2020 The MathWorks, Inc.

numCategories = numel(unique(classNames));

classBags = cell(numCategories,1);
classBagsIndices = cell(numCategories,1);
for classIdx = 1:numCategories
    [classBags{classIdx}, classBagsIndices{classIdx}] = vision.internal.balancePixelLabels.computeSortedMacroBlockList(macroBlockPixelCounts, classIdx);
end

end