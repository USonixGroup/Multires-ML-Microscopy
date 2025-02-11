%ViewSetBase Base view set interface class

% Copyright 2019-2023 The MathWorks, Inc.
classdef (Abstract, Hidden) ViewSetBase < vision.internal.ViewSetBaseImpl & ...
        matlab.mixin.Scalar

    properties (Abstract, SetAccess = protected)
        Views

        Connections
    end

    properties (SetAccess = protected)
        %NumViews Number of views in a view set
        NumViews

        %NumConnections Number of connections in a view set
        NumConnections
    end

    methods (Abstract)
        this = addView(this, varargin)
        this = updateView(this, varargin)
        this = deleteView(this, viewId)

        this = addConnection(this, varargin)
        this = updateConnection(this, varargin)
        this = deleteConnection(this, varargin)

        this = optimizePoses(this, varargin)
    end

    methods (Abstract, Access = protected)
        viewTable = checkViewTable(view, fillMissingVars)
        viewTable = parseViewInputs(viewId, varargin)
    end

    methods (Abstract, Hidden)
        that = saveobj(this)
    end

    methods (Abstract, Static, Hidden)
        this = loadobj(that)
    end

    methods
        %------------------------------------------------------------------
        function numViews = get.NumViews(this)
            numViews = height(this.Views);
        end

        %------------------------------------------------------------------
        function numConns = get.NumConnections(this)
            numConns = height(this.Connections);
        end

        %------------------------------------------------------------------
        function [viewTable, connIdx] = connectedViews(this, viewId)

            viewId = checkViewId(this, viewId);

            fromViewId = this.Connections.ViewId1==viewId;
            toViewId   = this.Connections.ViewId2==viewId;

            connectedViewIds = [...
                this.Connections.ViewId2(fromViewId); ...
                this.Connections.ViewId1(toViewId)];

            % Find linear index into view table
            [~, viewIdx] = intersect(this.Views.ViewId, connectedViewIds, 'stable');

            viewTable = this.Views(viewIdx, :);

            connIdx = fromViewId | toViewId;
        end

        %------------------------------------------------------------------
        function sensorPoses = poses(this, viewIds)

            if nargin>1
                viewIds = checkViewIds(this, viewIds);
                [~, idx] = ismember(viewIds(:), this.Views.ViewId);
                sensorPoses = this.Views(idx, {'ViewId', 'AbsolutePose'});
            else
                sensorPoses = this.Views(:, {'ViewId', 'AbsolutePose'});
            end
        end

        %------------------------------------------------------------------
        function G = createPoseGraph(this, weight)

            % Remove unnecessary variables from the table
            edgeTable = removevars(this.Connections, this.ExtraConnectionVariables);

            % Convert ViewId1, ViewId2 from view indices to linear indices
            viewIds = this.Views.ViewId;
            [~, edgeTable.ViewId1] = ismember(edgeTable.ViewId1, viewIds);
            [~, edgeTable.ViewId2] = ismember(edgeTable.ViewId2, viewIds);

            % Merge linear indices to a single variable EndNodes
            edgeTable = mergevars(edgeTable, {'ViewId1', 'ViewId2'}, ...
                                  'NewVariableName', 'EndNodes');

            % Optionally specify the weight of the edges
            if nargin > 1
                edgeTable.Weight = weight;
            end

            nodeTable = removevars(this.Views, this.ExtraViewVariables);

            G = digraph(edgeTable, nodeTable);
        end

        %------------------------------------------------------------------
        function views = findView(this, viewIds)
            validateattributes(viewIds, {'numeric'}, {'nonsparse', 'vector', ...
                                                      'integer', 'positive', 'real'}, 'findView', 'viewIds');
            numViews = numel(viewIds);

            checkIfViewIsMissing(this, viewIds);

            views = table();
            for i = 1:numViews
                views(i,:) = getView(this, viewIds(i));
            end
        end

        %------------------------------------------------------------------
        function conn = findConnection(this, viewIds1, viewIds2)
            validateattributes(viewIds1, {'numeric'}, {'nonsparse', 'vector', ...
                                                       'integer', 'positive', 'real'}, 'findConnection', 'viewIds1');
            validateattributes(viewIds2, {'numeric'}, {'nonsparse', 'vector', ...
                                                       'integer', 'positive', 'real', 'size', size(viewIds1)}, ...
                               'findConnection', 'viewIds2');
            checkIfViewIsMissing(this, [viewIds1(:); viewIds2(:)]);
            numConnections = numel(viewIds1);

            conn = table();
            for i = 1:numConnections
                checkIfConnectionIsMissing(this, viewIds1(i), viewIds2(i))
                idx = getConnectionIndex(this, viewIds1(i), viewIds2(i));
                conn(i,:) = this.Connections(idx,:);
            end
        end

        %------------------------------------------------------------------
        function varargout = plot(this, varargin)

            persistent plotParser
            if isempty(plotParser)
                plotParser = inputParser;
                plotParser.FunctionName = mfilename;

                addParameter(plotParser, 'Parent', [], @this.checkParent);
                addParameter(plotParser, 'ShowViewIds', 'off', @this.checkOnOff);
            end

            parse(plotParser, varargin{:});

            parent      = plotParser.Results.Parent;
            showViewIds = matlab.lang.OnOffSwitchState(plotParser.Results.ShowViewIds);

            % Create a new axes
            axesHandle = newplot(parent);

            % Create a pose graph
            G = createPoseGraph(this);

            % Extract translations from absolute poses
            trans = nan(height(G.Nodes), 3);
            for e = 1 : height(G.Nodes)
                trans(e, :) = G.Nodes.AbsolutePose(e).Translation;
            end

            if showViewIds
                nodeLabels = G.Nodes.ViewId;
            else
                nodeLabels = {};
            end

            % Plot graph
            graphPlotHandle = plot(axesHandle, G, ...
                                   'XData', trans(:, 1), ...
                                   'YData', trans(:, 2), ...
                                   'ZData', trans(:, 3), ...
                                   'NodeLabel', nodeLabels, ...
                                   'LineWidth', 2);

            xlabel(axesHandle, 'X')
            ylabel(axesHandle, 'Y')
            zlabel(axesHandle, 'Z')

            axis(axesHandle, 'equal')

            if height(G.Nodes) > 0
                customizeDatatip(this, graphPlotHandle);
            end

            if nargout>0
                nargoutchk(0,1);
                varargout{1} = graphPlotHandle;
            end
        end
    end

    methods (Access = protected)
        %------------------------------------------------------------------
        function view = getView(this, viewId)

            view = this.Views(getViewIndex(this, viewId), :);
        end

        %------------------------------------------------------------------
        function conn = getConnections(this, viewId)

            viewIds = this.Connections.ViewId1;
            conn = this.Connections(viewIds == viewId, :);
        end

        %------------------------------------------------------------------
        function customizeDatatip(~, graphPlotHandle)
        % Customize data tip to show view id

        % View ID
            graphPlotHandle.DataTipTemplate.DataTipRows(1).Label = getString(message('vision:viewSet:viewId'));

            % Clear out the rest
            graphPlotHandle.DataTipTemplate.DataTipRows(2:end) = [];
        end
    end

    %----------------------------------------------------------------------
    % Validation
    %----------------------------------------------------------------------
    methods (Access = protected)
        %------------------------------------------------------------------
        function checkOnOff(this, onoff)

            if isstring(onoff) || ischar(onoff)
                validatestring(onoff, {'on','off'}, this.ClassName, 'ShowViewIds');
            elseif isa(onoff, 'matlab.lang.OnOffSwitchState')
                % always valid
            else
                validateattributes(onoff, {'numeric','logical'}, ...
                                   {'scalar','binary'}, this.ClassName, 'ShowViewIds');
            end
        end

        %------------------------------------------------------------------
        function checkParent(~, parent)

        % An empty parent is allowed. This is passed on as an input to
        % newplot.
            if ~isempty(parent)
                vision.internal.inputValidation.validateAxesHandle(parent);
            end
        end
    end
end
