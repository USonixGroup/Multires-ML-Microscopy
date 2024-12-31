classdef PerClassQuadTreeNode < handle
% PerClassQuadTreeNode A quad tree node for each class of bounding boxes.
%
% This classdef is for internal use only and is likely to change in future
% releases.

% Copyright 2019 The MathWorks, Inc.

    properties (SetAccess=private)
        Window % [x y width height]
        NumBoxesInWindow
        ClassWeights
        ClassWeightsSum
        Probability
        Nodes
        Label
        OverlapThreshold
    end

    methods
        function this = PerClassQuadTreeNode(window, numBoxesInParent, allBboxes, allLabels, stats, nodeLabel,overlapThreshold)
            this.Window = window;
            this.Label = nodeLabel;
            nodeLabelIndices = allLabels == nodeLabel;
            nodeBboxes = allBboxes(nodeLabelIndices,:);
            nodeLabels = allLabels(nodeLabelIndices,:);

            % Get boxes at least 10% in the window
            [valid, indices] = bboxcrop(nodeBboxes,window,...
                'OverlapThreshold',overlapThreshold);
            this.NumBoxesInWindow = size(valid,1);

            validLabels = nodeLabels(indices,:);
            validLabelsTbl = table(validLabels,'VariableNames',{'Label'});
            invStats = join(validLabelsTbl,stats,'Keys','Label');
            this.ClassWeights = invStats.Weights;
            this.ClassWeightsSum = sum(invStats.Weights);

            this.Probability = this.NumBoxesInWindow/numBoxesInParent;
            this.Nodes = [];
            this.OverlapThreshold = overlapThreshold;
        end

        function setNodes(this, window, allBboxes,allLabels,stats)
            numNodes = size(window,1);

            this.Nodes = vision.internal.bbox.quadTreeSampling.PerClassQuadTreeNode.empty(numNodes,0);

            for ii = 1:numNodes
                this.Nodes(ii) = vision.internal.bbox.quadTreeSampling.PerClassQuadTreeNode(window(ii,:), ...
                    this.NumBoxesInWindow, allBboxes, allLabels, stats, this.Label,this.OverlapThreshold);
            end
        end

        function [x,y,w,h,wHalf,hHalf,noRight,noBottom] = getWindowSplitValues(this,blockSize)
            x = this.Window(1);
            y = this.Window(2);
            w = this.Window(3);
            h = this.Window(4);
            wHalf = floor(w/2);
            hHalf = floor(h/2);
            noRight = wHalf < blockSize(2);
            noBottom = hHalf < blockSize(1);
        end

        function tf = cannotSplitWindow(this,blockSize)
            [~,~,~,~,~,~,noRight,noBottom] = getWindowSplitValues(this,blockSize);
            tf = noRight && noBottom;
        end

        function fillEmptyNodes(this, allBboxes, allLabels, stats, blockSize)
            if ~isempty(this.Nodes)
                return;
            end

            [x,y,w,h,wHalf,hHalf,noRight,noBottom] = getWindowSplitValues(this,blockSize);

            if noRight && noBottom
                % Cannot split
                return;
            end

            if noRight && ~noBottom
                % Just the left vertical strip.
                % Take the whole width.

                wHalf = w;
                windows = vertcat(...
                    iTopLeft(x,y,w,h,wHalf,hHalf),...
                    iBottomLeft(x,y,w,h,wHalf,hHalf));

                setNodes(this,windows,allBboxes,allLabels,stats);
                return;
            end

            if ~noRight && noBottom
                % Just the top horizontal strip.
                % Take the whole height.

                hHalf = h;
                windows = vertcat(...
                    iTopLeft(x,y,w,h,wHalf,hHalf),...
                    iTopRight(x,y,w,h,wHalf,hHalf));

                setNodes(this,windows,allBboxes,allLabels,stats);
                return;
            end

            windows = vertcat(...
                iTopLeft(x,y,w,h,wHalf,hHalf),...
                iTopRight(x,y,w,h,wHalf,hHalf),...
                iBottomLeft(x,y,w,h,wHalf,hHalf),...
                iBottomRight(x,y,w,h,wHalf,hHalf));

            setNodes(this,windows,allBboxes,allLabels,stats);
        end
    end
end

function win = iTopLeft(x,y,~,~,wHalf,hHalf)
    win = [x,y,wHalf,hHalf];
end

function win = iTopRight(x,y,w,~,wHalf,hHalf)
    win = [x + wHalf,y,w - wHalf,hHalf];
end

function win = iBottomLeft(x,y,~,h,wHalf,hHalf)
    win = [x,y + hHalf,wHalf,h - hHalf];
end

function win = iBottomRight(x,y,w,h,wHalf,hHalf)
    win = [x + wHalf,y + hHalf,w - wHalf,h - hHalf];
end
