function blockSets = getBlockSets(blockSize, windows, numBlocks)
% getBlockSets Randomly sample block locations given a blockSize,
% windows and number of blocks to sample.
%
% This function is for internal use only and is likely to change in future
% releases.

% Copyright 2019 The MathWorks, Inc.
    numWindow = size(windows,1);
    blockSets = cell(numWindow,1);
    for jj = 1:numWindow
        window = windows(jj,:);
        x = window(3);
        y = window(4);

        w = window(5);
        h = window(6);

        xC = x:1:x+w-blockSize(2);
        yC = y:1:y+h-blockSize(1);

        numX = numel(xC);
        blockSetsPerWindow = cell(numX,1);
        for ii = 1:numX
            nY = numel(yC);
            blockSetsPerWindow{ii} = [xC(ii)*ones(nY,1), yC', repmat(blockSize,nY,1)];
        end
        blockSetsPerWindow = vertcat(blockSetsPerWindow{:});

        nB = size(blockSetsPerWindow,1);

        numBlocks = min(nB,numBlocks);
        indices = randperm(nB,numBlocks);
        blockSetsPerWindow = blockSetsPerWindow(indices,:);
        imageAndLevel = repmat(window(1:2),size(blockSetsPerWindow,1),1);

        blockSets{jj} = horzcat(imageAndLevel, blockSetsPerWindow);
    end
    blockSets = vertcat(blockSets{:});
end

