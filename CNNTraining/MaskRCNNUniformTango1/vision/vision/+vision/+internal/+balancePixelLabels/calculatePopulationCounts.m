function populationCounts = calculatePopulationCounts(classNames, macroBlockPixelCounts)
% calculatePopulationCounts Calculate population counts for each class
% for each macroblock

%   Copyright 2020 The MathWorks, Inc.

numCategories = numel(unique(classNames));

populationCounts = zeros(1,numCategories);
for idx = 1:size(macroBlockPixelCounts,1)
    populationCounts = populationCounts + macroBlockPixelCounts{idx,2};
end

end