function labelsTbl = selectBlockSetsFromWindows(labelsTbl, blockSize, bboxes, labels, numBlocksToSample, variableBlocksPerClass, overlapThreshold, blocksPerWindow)
% selectBlockSetsFromWindows Selects block locations from the windows
% that were already created. labelsTbl must contain the windows from which
% block locations can be selected.
%
% This function is for internal use only and is likely to change in future
% releases.

% Copyright 2019 The MathWorks, Inc.
    h = height(labelsTbl);
    if ~any(labelsTbl.Properties.VariableNames == "BlockSets")
        labelsTbl.BlockSets = cell(h, 1);
        labelsTbl.Bboxes = cell(h, 1);
        labelsTbl.BboxLabels = cell(h, 1);
        labelsTbl.LabelWindowCount = zeros(h, 1);
        labelsTbl.OtherWindowCount = zeros(h, 1);
    end

    if variableBlocksPerClass
        % BlocksPerClass is already chosen based on the remaining
        % number of blocks to be sampled.
        blocksPerClass = numBlocksToSample;
    else
        totalBlocks = numBlocksToSample;
        blocksPerClass = floor(totalBlocks/h);
        windowsPerClass = ceil(blocksPerClass/blocksPerWindow);
    end

    for ii = 1:h
        if variableBlocksPerClass
            if blocksPerClass(ii) <= 0
                continue;
            end
            windowsPerClass = ceil(blocksPerClass(ii)/blocksPerWindow);
        end
        label = labelsTbl.Label(ii);
        windows = labelsTbl.Windows{ii};
        if isempty(windows)
            % If there are no windows
            labelsTbl.Windows{ii} = double.empty(0,6);
            continue;
        end
        [~,idxes] = sort(labelsTbl.NumBoxesRatio{ii},'descend');
        n = numel(idxes);
        if n > windowsPerClass
            n = windowsPerClass;
        end

        if variableBlocksPerClass
            thisblocksPerWindow = ceil(blocksPerClass(ii)/n);
        else
            % Check to see if Windows from other classes contributed
            % to this class's Window count.
            otherWindowCount = labelsTbl.OtherWindowCount(ii);
            thisClassNumBlocks = blocksPerClass - otherWindowCount;
            if thisClassNumBlocks <= 0
                % Do not choose from this class of Windows anymore.
                % We have chosen enough from this class, since it filled
                % the required number of blocks for this class from Windows
                % of other classes.
                labelsTbl.Windows{ii} = double.empty(0,6);
                continue;
            end
            thisblocksPerWindow = ceil(thisClassNumBlocks/n);
        end
        if thisblocksPerWindow <= 0
            if ~variableBlocksPerClass
                labelsTbl.Windows{ii} = double.empty(0,6);
            end
            continue;
        end
        if n > thisblocksPerWindow
            % How to reduce the n - number of windows that gets picked?
            % Just swap blocksPerWindow and number of windows.
            temp = n;
            n = thisblocksPerWindow;
            thisblocksPerWindow = temp;
        end

        windows = windows(idxes(1:n),:);

        % Randomly sample block locations from the selected windows.
        blockSets = vision.internal.bbox.quadTreeSampling.utils.getBlockSets(blockSize,...
            windows,thisblocksPerWindow);

        % Select blocks that contain this class. This may result in lesser than
        % the required numBlocksToSample.
        [blockSets,outBboxes,outLabels,labelsTbl] = iSelectBlockSetBboxesPerClass(bboxes,labels,blockSets,...
            labelsTbl,label,overlapThreshold);

        labelsTbl.BlockSets{ii} = vertcat(labelsTbl.BlockSets{ii},blockSets);
        labelsTbl.Bboxes{ii} = vertcat(labelsTbl.Bboxes{ii},outBboxes);
        labelsTbl.BboxLabels{ii} = vertcat(labelsTbl.BboxLabels{ii},outLabels);
    end
end

function [blockSets,outBboxes,outLabels,labelsTbl] = iSelectBlockSetBboxesPerClass(bboxes,labels,blockSets,labelsTbl,label,overlapThreshold)
% Select blocks based on all of the boxes and labels available. Remove blocks
% that do not contain this class label. This might result in lesser number of
% blocks than the required, numBlocksToSample.
    numImages = size(blockSets,1);
    outBboxes = cell(numImages,1);
    outLabels = cell(numImages,1);
    removeBlockSets = false(numImages,1);
    for ii = 1:numImages
        blockSet = blockSets(ii,:);
        imNum = blockSet(1);
        win = blockSet(3:6);

        [outBboxes{ii}, indices] = bboxcrop(bboxes{imNum},win,...
            'OverlapThreshold',overlapThreshold);
        blockSetLabels = labels{imNum}(indices);
        outLabels{ii} = blockSetLabels;
        removeBlockSets(ii) = isempty(blockSetLabels) || ~any(blockSetLabels == label);
        if ~removeBlockSets(ii)
            [~,mi] = ismember(blockSetLabels,labelsTbl.Label);
            [umi,~,ic]=unique(mi);
            c = accumarray(ic,1);
            labelsTbl.LabelWindowCount(umi) = labelsTbl.LabelWindowCount(umi) + c;
            labelsTbl.OtherWindowCount(umi) = labelsTbl.OtherWindowCount(umi) + 1;
        end
    end
    % Remove block sets that do not have the current label.
    blockSets(removeBlockSets,:) = [];
    outBboxes(removeBlockSets,:) = [];
    outLabels(removeBlockSets,:) = [];
end
