classdef PerClassBboxQuadTree < handle
% PerClassQuadTree A quad tree for each class of bounding boxes.
%
% This classdef is for internal use only and is likely to change in future
% releases.

% Copyright 2019-2020 The MathWorks, Inc.
    properties (SetAccess=private)
        Width
        Height
        BlockSize
        Bboxes
        Labels
        Stats
        HeadNodes

        NumLevels

        LabelsMetricTbl
    end

    methods
        function this = PerClassBboxQuadTree(imageSize,blockSize,bboxes,labels,numLevels,stats,overlapThreshold)
            this.Width     = imageSize(2);
            this.Height    = imageSize(1);
            this.BlockSize = blockSize;
            this.Bboxes    = bboxes;
            this.Labels    = labels;
            this.Stats     = stats;

            window = [1,1,this.Width,this.Height];

            numLabels = numel(stats.Label);
            this.HeadNodes = vision.internal.bbox.quadTreeSampling.PerClassQuadTreeNode.empty(numLabels,0);
            for ii = 1:numLabels
                this.HeadNodes(ii) = vision.internal.bbox.quadTreeSampling.PerClassQuadTreeNode(...
                    window, size(bboxes,1), bboxes, labels, stats, stats.Label(ii),overlapThreshold);
            end

            % If numLevels is 'auto', quad tree level goes until the
            % blockSize. Make this the maximum value.
            if strcmpi(numLevels,'auto')
                numLevels = realmax;
            end
            this.NumLevels = numLevels;
        end

        function createQuadTreeNodes(this)
            % Create quad tree nodes as per NumQuadTreeLevels, aka., this.NumLevels.
            % If NumLevels is 'auto', the number of levels reached is as much as the
            % BlockSize allows the split. For example, if BlockSize is [30,30], then
            % once the split half of a window for a level reaches less than [30,30],
            % creation of the nodes after that level stops.
            nextQueue = {this.HeadNodes};
            level = 0;

            while level < this.NumLevels && ~isempty(nextQueue)
                currQueue = nextQueue;
                nextQueue = {};

                for jj = 1:numel(currQueue)
                    currentNodes = currQueue{jj};
                    numNodes = numel(currentNodes);

                    for ii=1:numNodes
                        currNode = currentNodes(ii);
                        if isempty(currNode.Nodes) && ...
                                currNode.Probability ~= 0 && ...
                                ~cannotSplitWindow(currNode,this.BlockSize)

                            currNode.fillEmptyNodes(this.Bboxes,this.Labels,this.Stats,this.BlockSize);
                            nextQueue{end+1} = currNode.Nodes;
                        end
                    end
                end
                level = level + 1;
            end
        end

        function leafNodes = getLeafNodes(this)
            % Get all the leaf nodes from this quad tree.
            currNodes = this.HeadNodes;
            prevNodes = currNodes;
            while ~isempty(currNodes)
                prevNodes = currNodes;
                currNodes = [currNodes.Nodes];
            end
            leafNodes = prevNodes;
        end

        function createWindowsWithMetric(this,imageNumber,imageLevel,grouper)
            % This function creates Windows and NumBoxesRatio metric for each class
            % from all the nodes for an image.
            %   Window        = [imageNumber,levelNumber,x,y,w,h]
            %   NumBoxesRatio = Ratio of number of boxes of a class in a window and
            %                   total number of boxes in that window.

            % Get all the leaf nodes.
            nodes = getLeafNodes(this);

            % Find nodes with non-zero weights.
            nnzNodes = nodes([nodes.ClassWeightsSum] > 0);

            windows = vertcat(nnzNodes.Window);
            if isempty(windows)
                return;
            end

            labels = vertcat(nnzNodes.Label);
            numBoxesInWindow = vertcat(nnzNodes.NumBoxesInWindow);

            % Find the indices of the unique windows to group them
            % based on window sizes.
            [~, ~, G] = unique(windows, 'rows');

            % Group windows based on the unique window size values.
            %
            % We need an object to hold on to Window values and
            % NumBoxesRatio values. A handle object is needed because,
            % splitapply cannot update value objects like table
            % between calls to each row.
            splitapply(@(w,l,n)iSplitApply(w,l,n,grouper,imageNumber,imageLevel),...
                windows,labels,numBoxesInWindow,... % data to be grouped
                G); % grouping variable
        end
    end
end

function iSplitApply(window,labels,numBoxes,grouper,imageNumber,imageLevel)
    totalBoxes = sum(numBoxes);
    numBoxesRatio = numBoxes / totalBoxes;
    blockSetWindow = [imageNumber, imageLevel, window(1,:)];

    % For each of the labels, place windows that contain a specific
    % label. Each label can contain the same window.
    grouper.group(blockSetWindow,labels,numBoxesRatio);
end
