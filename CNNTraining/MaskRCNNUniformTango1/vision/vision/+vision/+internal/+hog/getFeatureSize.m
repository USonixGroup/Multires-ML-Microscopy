% -------------------------------------------------------------------------
% Compute HOG feature size
% -------------------------------------------------------------------------
function sz = getFeatureSize(params)
%#codegen

%   Copyright 2013-2020 The MathWorks, Inc.

nBlocks = vision.internal.hog.getNumBlocksPerWindow(params);
sz = prod([params.NumBins params.BlockSize nBlocks]);
end
