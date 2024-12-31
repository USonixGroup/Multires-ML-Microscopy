% -------------------------------------------------------------------------
% Compute the number of overlapping blocks within a 2D window.
% -------------------------------------------------------------------------
function nBlocks = getNumBlocksPerWindow(params)
%#codegen

%   Copyright 2013-2020 The MathWorks, Inc.

windowSize   = single(params.WindowSize);
cellSize     = single(params.CellSize);
blockSize    = single(params.BlockSize);
blockOverlap = single(params.BlockOverlap);

numCellsPerWindow = floor(windowSize./cellSize);
nBlocks = int32(floor((numCellsPerWindow - blockSize)./(blockSize - blockOverlap)) + 1);
end
