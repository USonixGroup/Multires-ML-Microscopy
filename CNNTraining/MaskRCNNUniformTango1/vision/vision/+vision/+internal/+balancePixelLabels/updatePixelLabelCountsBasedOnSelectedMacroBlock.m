function updatedPixelLabelCounts = updatePixelLabelCountsBasedOnSelectedMacroBlock(updatedPixelLabelCounts, macroBlockStats, statsIdx)
% updatePixelLabelCountsBasedOnSelectedMacroBlock Update the pixel label
% counts of each class with the stats of the selected macroblock

%   Copyright 2020 The MathWorks, Inc.

updatedPixelLabelCounts = updatedPixelLabelCounts + macroBlockStats{statsIdx,2};

end