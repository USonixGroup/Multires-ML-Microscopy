%ViewSetFeatureGraph Object for storing matched feature points across views
%
% See also imageviewset.

% Copyright 2020-2021 The MathWorks, Inc.
%#codegen
classdef ViewSetFeatureGraph

    properties (GetAccess = public, SetAccess = protected)
        FeatureGraph
    end

    methods (Access = public)
        function this = ViewSetFeatureGraph(varargin)
            if isempty(coder.target)
                this.FeatureGraph = graph;
            else
                pointsType = varargin{1};
                nodeTable = table(zeros(coder.ignoreConst(0), 1, 'uint32'), ...
                    zeros(coder.ignoreConst(0), 1), zeros(coder.ignoreConst(0), 2, pointsType), ...
                    'VariableNames', {'viewId', 'pointIdx', 'points'});
                this.FeatureGraph = graph(zeros(coder.ignoreConst(0)), zeros(coder.ignoreConst(0)), ...
                    zeros(0, 0), nodeTable);
            end
        end

        %------------------------------------------------------------------
        function this = addNodes(this, viewTable)

            if this.isSimMode()
                nViews = height(viewTable);
            else
                nViews = numel(viewTable.ViewId);
            end
            for i = 1:nViews
                viewId = viewTable.ViewId(i);
                points = viewTable.Points{i};

                % Add new nodes
                if ~isempty(points)
                    if ~isnumeric(points)
                        numPoints = size(points.Location, 1);
                        p = points.Location; % [x, y] location
                    else
                        numPoints = size(points, 1);
                        p = points;
                    end
                    viewId    = repmat(viewId, [numPoints, 1]);
                    pointIdx  = (1:numPoints)';

                    this.FeatureGraph = addnode(this.FeatureGraph, ...
                                                table(viewId, pointIdx, p, 'VariableNames', {'viewId', ...
                                                                                             'pointIdx', 'points'}));
                end
            end
        end

        %------------------------------------------------------------------
        function this = updateNodes(this, viewTable, wipedConnTable, pointsChanged)

            if pointsChanged
                if this.isSimMode()
                    viewsWithPoints = viewTable.ViewId( ...
                        cellfun(@(x) ~isempty(x), viewTable.Points));
                else
                    coder.varsize('viewsWithPoints', [Inf, 1],[1 0]);
                    viewsWithPoints = zeros(0, 1, 'uint32');
                    for i=1:numel(viewTable.ViewId)
                        if ~isempty(viewTable.Points{i})
                            viewsWithPoints = [viewsWithPoints; viewTable.ViewId(i)];
                        end
                    end
                end

                % Remove nodes
                if ~isempty(viewsWithPoints)
                    this = removeNodes(this, viewsWithPoints);
                end

                % Add new nodes for the views
                this = addNodes(this, viewTable);

            else % Only features changed
                 % Remove old edges only
                this = removeEdges(this, wipedConnTable);
            end
        end

        %------------------------------------------------------------------
        function this = removeNodes(this, viewId)
            nodeId = getNodeId(this, viewId);
            this.FeatureGraph = rmnode(this.FeatureGraph, nodeId);
        end

        %------------------------------------------------------------------
        function this = addEdges(this, connTable)

            if this.isSimMode()
                nConns = height(connTable);
            else
                nConns = numel(connTable.ViewId1);
            end
            for i = 1:nConns
                matches = connTable.Matches{i};
                if ~isempty(matches)
                    [s, t] = getEdgeId(this, connTable.ViewId1(i), ...
                                       connTable.ViewId2(i), matches);
                    this.FeatureGraph = addedge(this.FeatureGraph, s, t);
                end
            end
        end

        %------------------------------------------------------------------
        function this = updateEdges(this, connTable, oldMatches)

            if this.isSimMode()
                nConns = height(connTable);
            else
                nConns = numel(connTable.ViewId1);
            end
            for i = 1:nConns
                viewId1 = connTable.ViewId1(i);
                viewId2 = connTable.ViewId2(i);

                % Remove old edges
                if ~isempty(oldMatches{i})
                    [sOld, tOld] = getEdgeId(this, viewId1, viewId2, oldMatches{i});
                    this.FeatureGraph = rmedge(this.FeatureGraph, sOld, tOld);
                end

                % Add new edges
                newMatches = connTable.Matches{i};
                if ~isempty(newMatches)
                    [sNew, tNew] = getEdgeId(this, viewId1, viewId2, newMatches);
                    this.FeatureGraph = addedge(this.FeatureGraph, sNew, tNew);
                end
            end
        end

        %------------------------------------------------------------------
        function this = removeEdges(this, connTable)

            if this.isSimMode()
                nConns = height(connTable);
            else
                nConns = numel(connTable.ViewId1);
            end
            for i = 1:nConns
                matches = connTable.Matches{i};
                if ~isempty(matches)
                    [s, t] = getEdgeId(this, connTable.ViewId1(i), ...
                                       connTable.ViewId2(i), matches);
                    this.FeatureGraph = rmedge(this.FeatureGraph, s, t);
                end
            end
        end

        %------------------------------------------------------------------
        function tracks = createTracks(this, viewIds, numViews, minNumViews)

            if numel(viewIds) < numViews
                subgraphIdx = false(numnodes(this.FeatureGraph), 1);
                if this.isSimMode()
                    for i = viewIds(:)'
                        subgraphIdx = subgraphIdx | (this.FeatureGraph.Nodes.viewId == i);
                    end
                else
                    for i = 1:numel(viewIds)
                        subgraphIdx = subgraphIdx | (this.FeatureGraph.Nodes.viewId == (viewIds(i)));
                    end
                end
                % Because imageviewset is a value object, the change to
                % FeatureGraph will not be visible to the caller.
                fGraph = subgraph(this.FeatureGraph, find(subgraphIdx));
            else
                fGraph = this.FeatureGraph;
            end

            % Check if the feature graph has edges
            if isempty(fGraph.Edges)
                tracks = this.makeEmptyPointTrack(class(fGraph.Nodes.points));
                return;
            end

            % Find connected components in the feature graph
            bins = conncomp(fGraph);
            allViewIds = fGraph.Nodes.viewId;
            allFeatureIndices = fGraph.Nodes.pointIdx;
            allPoints = fGraph.Nodes.points;

            % Sort bins, so all connected nodes are grouped.
            [binsSorted,sortInx] = sort(bins'); %#ok<TRSRT>

            % Find bin transitions and remove tracks of length 1
            diffBins=[1;binsSorted(2:end) - binsSorted(1:end-1)];
            nonSingleBins= ~[diffBins(1:end-1)>0 & diffBins(2:end)>0; ...
                             diffBins(end)>0];

            binsSorted=binsSorted(nonSingleBins);

            % If there are no tracks of length greater than 1, return empty
            % pointTrack array.
            if isempty(binsSorted)
                tracks = this.makeEmptyPointTrack(class(allPoints(1)));
                return;
            end

            sortInx=sortInx(nonSingleBins);
            diffBins=diffBins(nonSingleBins);

            % Specify bin transitions as inclusive index pairs
            binInx = find(diffBins);
            binInx = [binInx(1:end-1) (binInx(2:end)-1);...
                      binInx(end) length(binsSorted)];

            % Each track should contain at least minNumViews points
            binInx = binInx(abs(binInx(:,1) - binInx(:,2))+1 >= minNumViews, :);

            if isempty(binInx)
                tracks = this.makeEmptyPointTrack(class(allPoints(1)));
                return;
            end

            % Create all tracks first to avoid reallocating memory on each
            % iteration
            numTracks = size(binInx, 1);
            if this.isSimMode()
                tracks = repmat(pointTrack(0,[0 0]),1, numTracks);
                for i = 1:numTracks
                    idx = sortInx(binInx(i,1):binInx(i,2));
                    viewIds = allViewIds(idx);
                    [~, uidx] = unique(viewIds, 'first');
                    tracks(i) = pointTrack(viewIds(uidx), ...
                                           allPoints(idx(uidx), :), allFeatureIndices(idx(uidx)));
                end
            else
                idx = sortInx(binInx(1,1):binInx(1,2));
                viewIds = allViewIds(idx);
                [~, uidx] = unique(viewIds, 'first');
                tracks = pointTrack(viewIds(uidx), ...
                                    allPoints(idx(uidx), :), allFeatureIndices(idx(uidx)));
                for i = 2:numTracks
                    idx = sortInx(binInx(i,1):binInx(i,2));
                    viewIds = allViewIds(idx);
                    [~, uidx] = unique(viewIds, 'first');
                    tracks(i) = pointTrack(viewIds(uidx), ...
                                           allPoints(idx(uidx), :), allFeatureIndices(idx(uidx)));
                end
            end
        end
    end

    methods (Access = protected)
        %------------------------------------------------------------------
        function [s, t] = getEdgeId(this, viewId1, viewId2, matches)
            allNodes           = this.FeatureGraph.Nodes;
            viewIdsAndPointIdx = vertcat(allNodes{:, 1:2});
            numMatches         = size(matches, 1);
            viewId1 = repmat(viewId1, numMatches, 1);
            viewId2 = repmat(viewId2, numMatches, 1);
            [~,s]   = ismember([viewId1, matches(:,1)], viewIdsAndPointIdx, 'row');
            [~,t]   = ismember([viewId2, matches(:,2)], viewIdsAndPointIdx, 'row');
        end

        %------------------------------------------------------------------
        function nodeId = getNodeId(this, viewIds)
            if this.isSimMode()
                nNodes = this.FeatureGraph.numnodes;
            else
                nNodes = height(this.FeatureGraph.Nodes);
            end
            if (nNodes > 0)
                nodeId = find(ismember(this.FeatureGraph.Nodes.viewId, viewIds));
            else
                nodeId = [];
            end
        end
        function obj = makeEmptyPointTrack(this, classType)
            if this.isSimMode()
                obj = pointTrack.empty();
            else
                vIds = zeros(coder.ignoreConst(1), coder.ignoreConst(0), 'uint32');
                points = zeros(coder.ignoreConst(0), coder.ignoreConst(2), classType);
                fIndices = zeros(coder.ignoreConst(1), coder.ignoreConst(0), 'uint32');
                pObject = pointTrack(vIds, points, fIndices);
                obj = repmat(pObject, 0, 0);
            end
        end
    end
    methods(Static, Access = private)
        %--------------------------------------------------------------------------
        function out = isSimMode()
        % Check if simulation mode
            out = isempty(coder.target);
        end
    end
end
