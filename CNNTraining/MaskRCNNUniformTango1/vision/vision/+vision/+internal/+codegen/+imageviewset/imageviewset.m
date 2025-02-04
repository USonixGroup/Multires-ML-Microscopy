classdef imageviewset < vision.internal.codegen.imageviewset.imageviewsetBase
% Class imageviewset is used for code generation support
%#codegen

% Copyright 2021-2023 The MathWorks, Inc.

    properties (Access = ?matlab.unittest.TestCase)
        IncrementalBuilding (1, 1) {mustBeNumericOrLogical} = false
    end

    properties (GetAccess = public, Constant, Hidden)
        ClassName = 'imageviewset';
    end

    properties (Constant, Access = protected)
        Version   = 1.3;
        ExtraViewVariables          = {'Features', 'Points'};
        ExtraConnectionVariables    = {'Matches'};

        % Parser options
        ParserOptions = struct( ...
            'CaseSensitivity', false, ...
            'StructExpand',    true, ...
            'PartialMatching', true);
    end

    methods
        %------------------------------------------------------------------
        % Constructor
        %------------------------------------------------------------------
        function this = imageviewset()

            this = this@vision.internal.codegen.imageviewset.imageviewsetBase();
        end
        %------------------------------------------------------------------
        % addView: Add a new view to the imageviewset object
        %------------------------------------------------------------------
        function this = addView(this, varargin)
        % Parse the inputs to addView
            if istable(varargin{1})
                fillMissingVars = true;
                [this, viewTable] = checkViewTable(this, varargin{1}, fillMissingVars);
            else
                [this, viewTable] = parseViewInputs(this, varargin{:});
            end

            if isempty(this.ViewId)
                viewExists = false;
            else
                viewExists = hasView(this, viewTable.ViewId);
            end

            if viewExists
                existingViews = viewTable.ViewId(viewExists);
                coder.internal.error('vision:viewSet:viewIdAlreadyExists', existingViews(1));
            end

            if isempty(this.ViewId)
                % Assigning AbsolutePose
                if strcmp(this.AbsPoseClass, 'double')
                    this.AbsPosesDouble = viewTable.AbsolutePose;
                else
                    this.AbsPosesSingle = viewTable.AbsolutePose;
                end
                % Assigning Features
                if strcmp(this.FeaturesClass, 'double')
                    this.FeaturesDouble = viewTable.Features;
                elseif strcmp(this.FeaturesClass, 'single')
                    this.FeaturesSingle = viewTable.Features;
                elseif strcmp(this.FeaturesClass, 'uint8')
                    this.FeaturesBinary = viewTable.Features;
                else
                    bin_features = cell(numel(viewTable.ViewId), 1);
                    for i = 1:numel(viewTable.ViewId)
                        bin_features{i} = viewTable.Features{i}.Features;
                    end
                    this.FeaturesBinary = bin_features;
                end
                % Assigning Points
                if strcmp(this.PointsClass, 'double')
                    this.PointsDouble = viewTable.Points;
                else
                    if strcmp(this.PointsClass, 'single')
                        this.PointsSingle = viewTable.Points;
                    else
                        pointsLocation = cell(numel(viewTable.ViewId), 1);
                        for i = 1:numel(viewTable.ViewId)
                            pointsLocation{i} = viewTable.Points{i}.Location;
                        end
                        this.PointsSingle = pointsLocation;
                        % Storing the  properties of Points object like
                        % Metrics, Scale when points being input is not numeric
                        this = this.addProps(viewTable.Points);
                    end
                end
                % Assigning ViewId
                this.ViewId = viewTable.ViewId;
            else
                % Assigning the AbsolutePose
                if  strcmp(this.AbsPoseClass, 'double')
                    this.AbsPosesDouble = [this.AbsPosesDouble, viewTable.AbsolutePose];
                else
                    this.AbsPosesSingle = [this.AbsPosesSingle, viewTable.AbsolutePose];
                end
                % Assigning Features
                if strcmp(this.FeaturesClass, 'double')
                    this.FeaturesDouble = getUpdatedFeatures(this, viewTable.Features);
                elseif strcmp(this.FeaturesClass, 'single')
                    this.FeaturesSingle = getUpdatedFeatures(this, viewTable.Features);
                else
                    this.FeaturesBinary = getUpdatedFeatures(this, viewTable.Features);
                end
                % Assigning Points
                if strcmp(this.PointsClass, 'double')
                    this.PointsDouble = getUpdatedPoints(this, viewTable.Points);
                else
                    this.PointsSingle = getUpdatedPoints(this, viewTable.Points);
                    % Storing the  properties of Points object like
                    % Metrics, Scale when points being input is not numeric
                    this = this.addProps(viewTable.Points);
                end
                % Assigning ViewId
                this.ViewId = [this.ViewId; viewTable.ViewId];
            end

            % Add nodes to the feature graph
            if this.IncrementalBuilding
                this.FeatureGraph = addNodes(this.FeatureGraph, viewTable);
            end
        end

        %------------------------------------------------------------------
        % updateView: Modify an existing view
        %------------------------------------------------------------------
        function this = updateView(this, varargin)

            if istable(varargin{1})
                fillMissingVars = false;
                % Parsing the viewTable
                [this, viewTable, unsetColumns] = checkViewTable(this, varargin{1}, fillMissingVars);
                checkIfViewIsMissing(this, viewTable.ViewId);

                allViewIds = this.Views.ViewId;
                [viewIdsToUpdate, ~, ib] = intersect(viewTable.ViewId, allViewIds, 'stable');

                % Get the property names that are changes
                pointsChanged   = ~any(strcmp('Points', unsetColumns));
                featuresChanged = ~any(strcmp('Features', unsetColumns));
                absPosesChanged = ~any(strcmp('absPose', unsetColumns));
                % Update the views using the view table
                for i=1:numel(viewTable.ViewId)
                    idx = ib(i);
                    % Update the AbsolutePose
                    if absPosesChanged
                        if  strcmp(this.AbsPoseClass, 'double')
                            this.AbsPosesDouble(idx) = viewTable.AbsolutePose(i);
                        else
                            this.AbsPosesSingle(idx) = viewTable.AbsolutePose(i);
                        end
                    end
                    % Update the Features
                    if featuresChanged
                        if strcmp(this.FeaturesClass, 'double')
                            this.FeaturesDouble{idx} = viewTable.Features{i};
                        elseif strcmp(this.FeaturesClass, 'single')
                            this.FeaturesSingle{idx} = viewTable.Features{i};
                        else
                            this.FeaturesBinary{idx} = viewTable.Features{i}.Features;
                        end
                    end
                    % Update the Points
                    if pointsChanged
                        if strcmp(this.PointsClass, 'double')
                            this.PointsDouble{idx} = viewTable.Points{i};
                        else
                            if strcmp(this.PointsClass, 'single')
                                this.PointsSingle{idx} = viewTable.Points{i};
                            else
                                pointsLocation = viewTable.Points{i}.Location;
                                this.PointsSingle{idx} = pointsLocation;
                                this = this.updateProps(viewTable.Points{i}, idx);
                            end
                        end
                    end
                end
            else
                % Parsing the inputs
                [this, viewTable, unsetColumns] = parseViewInputs(this, varargin{:});
                viewIdsToUpdate = viewTable.ViewId;
                checkIfViewIsMissing(this, viewIdsToUpdate);

                viewIdx = getViewIndex(this, viewIdsToUpdate);
                % Get the property names that are changes
                pointsChanged   = ~any(strcmp('Points', unsetColumns));
                featuresChanged = ~any(strcmp('Features', unsetColumns));
                absPosesChanged = ~any(strcmp('absPose', unsetColumns));
                % Update the Features
                if featuresChanged
                    if strcmp(this.FeaturesClass, 'double')
                        this.FeaturesDouble{viewIdx} = viewTable.Features{1};
                    elseif strcmp(this.FeaturesClass, 'single')
                        this.FeaturesSingle{viewIdx} = viewTable.Features{1};
                    elseif strcmp(this.FeaturesClass, 'uint8')
                        this.FeaturesBinary{viewIdx} = viewTable.Features{1};
                    else
                        this.FeaturesBinary{viewIdx} = viewTable.Features{1}.Features;
                    end
                end
                % Update the Points
                if pointsChanged
                    if strcmp(this.PointsClass, 'double')
                        this.PointsDouble{viewIdx} = viewTable.Points{1};
                    else
                        if strcmp(this.PointsClass, 'single')
                            this.PointsSingle{viewIdx} = viewTable.Points{1};
                        else
                            pointsLocation = viewTable.Points{1}.Location;
                            this.PointsSingle{viewIdx} = pointsLocation;
                            this = this.updateProps(viewTable.Points{1}, viewIdx);
                        end
                    end
                end
                % Update the AbsolutePose
                if absPosesChanged
                    if  strcmp(this.AbsPoseClass, 'double')
                        this.AbsPosesDouble(viewIdx) = viewTable.AbsolutePose;
                    else
                        this.AbsPosesSingle(viewIdx) = viewTable.AbsolutePose;
                    end
                end
            end

            % If the features or the points of the views are modified,
            % 1) delete matches in all the view's connections and
            % 2) update all the corresponding edges in the feature graph
            shouldRemoveMatches = pointsChanged || featuresChanged;
            if shouldRemoveMatches
                [this, wipedConnTable] = wipeMatches(this, viewIdsToUpdate);

                % Update existing edges in the feature graph
                if this.IncrementalBuilding
                    this.FeatureGraph = updateNodes(this.FeatureGraph, ...
                                                    viewTable, wipedConnTable, pointsChanged);
                end
            end
        end

        %------------------------------------------------------------------
        % deleteView: Delete an existing view
        %------------------------------------------------------------------
        function this = deleteView(this, viewIds)

            viewIds = checkViewIds(this, viewIds);
            checkIfViewIsMissing(this, viewIds);
            % Getting the indices of views to be deleted
            removeViewsIdx = find(ismember(this.ViewId, viewIds));
            % Get viewsLeft which are the difference between existing views
            % and views removed
            [viewsLeft, indicesViewsLeft] = setdiff(this.ViewId, ...
                                                    this.ViewId(removeViewsIdx));

            coder.varsize('removeConnsIdx', [1, inf],[0 1]);
            removeConnsIdx = zeros(1,0);
            for i=1:numel(viewIds)
                if ~isempty(this.ViewId1)
                    % Get the indices of connections to be deleted
                    connIdx = getConnectionIndexToAndFrom(this, viewIds(i));
                    if ~isempty(connIdx)
                        if isempty(removeConnsIdx)
                            removeConnsIdx = connIdx;
                        else
                            removeConnsIdx = [removeConnsIdx connIdx]; 
                        end
                    end
                end
            end
            % If all the views are deleted
            if isempty(viewsLeft)
                % Get the default values of all properties
                [~, poseSingle, poseDouble, fDouble, fSingle, ...
                 fBinary, pSingle, pDouble] = ...
                 vision.internal.codegen.imageviewset.imageviewsetBase.initializeViewsetData();
                % Assigning AbsolutePose
                if strcmp(this.AbsPoseClass, 'double')
                    this.AbsPosesDouble = poseDouble;
                else
                    this.AbsPosesSingle = poseSingle;
                end
                % Assigning Features
                if strcmp(this.FeaturesClass, 'double')
                    this.FeaturesDouble = fDouble;
                elseif strcmp(this.FeaturesClass, 'single')
                    this.FeaturesSingle = fSingle;
                else
                    this.FeaturesBinary = fBinary;
                end
                % Assigning Points
                if strcmp(this.PointsClass, 'double')
                    this.PointsDouble = pDouble;
                else
                    this.PointsSingle = pSingle;
                    % Delete other properties of Points objects
                    this = this.delProps(indicesViewsLeft);
                end
            else
                nViewsLeft = numel(this.ViewId(indicesViewsLeft));
                % Assigning the remaining AbsolutePose
                if strcmp(this.AbsPoseClass, 'double')
                    this.AbsPosesDouble = this.AbsPosesDouble(indicesViewsLeft);
                else
                    this.AbsPosesSingle = this.AbsPosesSingle(indicesViewsLeft);
                end
                % Assigning the remaining Features
                this = this.updateFeatures(indicesViewsLeft, nViewsLeft);
                % Assigning the remaining Points
                this = this.updatePoints(indicesViewsLeft, nViewsLeft);
            end

            % Delete the connections associated with the views being
            % deleted if there are any connections present in the view set
            if coder.internal.prop_has_class(this, 'RelClass')
                viewId1 = this.ViewId1;
                viewId2 = this.ViewId2;
                for i=1:numel(removeConnsIdx)
                    view1 = viewId1(removeConnsIdx(i));
                    view2 = viewId2(removeConnsIdx(i));
                    this = deleteConnection(this, view1, view2);
                end
            end

            % Assigning the remaining ViewIds
            if isempty(viewsLeft)
                this.ViewId = zeros(0, 1,'uint32');
            else
                this.ViewId = this.ViewId(indicesViewsLeft);
            end

            % Remove nodes in the feature graph. Any edges incident upon
            % the nodes are also removed.
            if this.IncrementalBuilding
                this.FeatureGraph = removeNodes(this.FeatureGraph, viewIds);
            end
        end

        %------------------------------------------------------------------
        % findView: Finds views associated with the given view ids
        %------------------------------------------------------------------
        function views = findView(this, viewIds)
            views = findView@vision.internal.codegen.imageviewset.imageviewsetBase(this, viewIds);
        end

        %------------------------------------------------------------------
        % addConnection: Add a connection between existing views
        %------------------------------------------------------------------
        function this = addConnection(this, viewId1, viewId2, varargin)

        % Parse connection inputs
            [this, connTable] = parseConnectionInputs(this, viewId1, viewId2, varargin{:});

            connectionExists = hasConnection(this, connTable.ViewId1, connTable.ViewId2);

            coder.internal.errorIf(connectionExists, 'vision:viewSet:connectionAlreadyExists', ...
                                   connTable.ViewId1(1), connTable.ViewId2(1));

            if isempty(this.ViewId1)
                % Assigning the ViewIds
                this.ViewId1 = connTable.ViewId1;
                this.ViewId2 = connTable.ViewId2;
                % Assigning RelativePose
                if strcmp(this.RelClass, 'simtform3d') ...
                        || strcmp(this.RelClass, 'affine3d')
                    if strcmp(this.RelPoseClass, 'single')
                        this.RaffinePoseSingle = connTable.RelativePose;
                    else
                        this.RaffinePoseDouble = connTable.RelativePose;
                    end
                else
                    if strcmp(this.RelPoseClass, 'single')
                        this.RrigidPoseSingle = connTable.RelativePose;
                    else
                        this.RrigidPoseDouble = connTable.RelativePose;
                    end
                end
                % Assigning the Information matrix
                if strcmp(this.InfoMatClass, 'single')
                    this.InfoMatSingle = connTable.InformationMatrix;
                else
                    this.InfoMatDouble = connTable.InformationMatrix;
                end
                % Assigning the Matches
                if strcmp(this.MatchesClass, 'single')
                    this.MatchesSingle = connTable.Matches;
                elseif strcmp(this.MatchesClass, 'double')
                    this.MatchesDouble = connTable.Matches;
                else
                    this.MatchesInt = connTable.Matches;
                end
            else
                % Assigning the ViewIds
                this.ViewId1 = [this.ViewId1; connTable.ViewId1];
                this.ViewId2 = [this.ViewId2; connTable.ViewId2];
                % Assigning the RelativePose
                if strcmp(this.RelClass, 'simtform3d') ...
                        || strcmp(this.RelClass, 'affine3d')
                    if strcmp(this.RelPoseClass, 'single')
                        this.RaffinePoseSingle = [this.RaffinePoseSingle connTable.RelativePose];
                    else
                        this.RaffinePoseDouble = [this.RaffinePoseDouble connTable.RelativePose];
                    end
                else
                    if strcmp(this.RelPoseClass, 'single')
                        this.RrigidPoseSingle = [this.RrigidPoseSingle connTable.RelativePose];
                    else
                        this.RrigidPoseDouble = [this.RrigidPoseDouble connTable.RelativePose];
                    end
                end
                % Assigning the Information Matrix
                if strcmp(this.InfoMatClass, 'single')
                    this.InfoMatSingle = getUpdatedInfoMat(this, connTable);
                else
                    this.InfoMatDouble = getUpdatedInfoMat(this, connTable);
                end
                % Assigning the Matches
                if strcmp(this.MatchesClass, 'single')
                    this.MatchesSingle = getUpdatedMatches(this, connTable);
                elseif strcmp(this.MatchesClass, 'double')
                    this.MatchesDouble = getUpdatedMatches(this, connTable);
                else
                    this.MatchesInt = getUpdatedMatches(this, connTable);
                end
            end

            % Add edges in the feature graph
            if this.IncrementalBuilding
                this.FeatureGraph = addEdges(this.FeatureGraph, connTable);
            end
        end

        %------------------------------------------------------------------
        % findConnection: Find the connection associated between the given
        % viewIds
        %------------------------------------------------------------------
        function conn = findConnection(this, viewIds1, viewIds2)

            conn = findConnection@vision.internal.codegen.imageviewset.imageviewsetBase(this, ...
                                                                                   viewIds1, viewIds2);
        end

        %------------------------------------------------------------------
        % hasView: Check if a view with the given viewIds exists
        %------------------------------------------------------------------
        function tf = hasView(this, viewIds)

            tf = hasView@vision.internal.codegen.imageviewset.imageviewsetBase(this, viewIds);
        end

        %------------------------------------------------------------------
        % updateConnection: Modify the existing connection associated
        % between given viewIds
        %------------------------------------------------------------------
        function this = updateConnection(this, varargin)
        % Parse the connection inputs
            [this, connTable, unsetColumns] = parseConnectionInputs(this, varargin{:});

            checkIfConnectionIsMissing(this, connTable.ViewId1, connTable.ViewId2);

            conIdx = getConnectionIndex(this, connTable.ViewId1, connTable.ViewId2);
            idx = conIdx(1);

            % Get the properties being updated
            rPoseUpdated = ~any(strcmp('relPose', unsetColumns));
            iMatUpdated = ~any(strcmp('infoMat', unsetColumns));
            mUpdated = ~any(strcmp('Matches', unsetColumns));
            % Updating the RelativePose
            if rPoseUpdated
                if strcmp(this.RelClass, 'simtform3d') ...
                        || strcmp(this.RelClass, 'affine3d')
                    if strcmp(this.RelPoseClass, 'single')
                        this.RaffinePoseSingle(idx) = connTable.RelativePose;
                    else
                        this.RaffinePoseDouble(idx) = connTable.RelativePose;
                    end
                else
                    if strcmp(this.RelPoseClass, 'single')
                        this.RrigidPoseSingle(idx) = connTable.RelativePose;
                    else
                        this.RrigidPoseDouble(idx) = connTable.RelativePose;
                    end
                end
            end
            % Updating the Information Matrix
            if iMatUpdated
                if strcmp(this.InfoMatClass, 'single')
                    this.InfoMatSingle{idx} = connTable.InformationMatrix{1};
                else
                    this.InfoMatDouble{idx} = connTable.InformationMatrix{1};
                end
            end
            % Updating the Matches
            if mUpdated
                if strcmp(this.MatchesClass, 'single')
                    oldMatches =  {this.MatchesSingle{idx}};
                    this.MatchesSingle{idx} = connTable.Matches{1};
                elseif strcmp(this.MatchesClass, 'double')
                    oldMatches =  {this.MatchesDouble{idx}};
                    this.MatchesDouble{idx} = connTable.Matches{1};
                else
                    oldMatches =  {this.MatchesInt{idx}};
                    this.MatchesInt{idx} = connTable.Matches{1};
                end

                % Update old edges in feature graph
                if this.IncrementalBuilding
                    this.FeatureGraph = updateEdges(this.FeatureGraph, connTable, oldMatches);
                end
            end
        end

        %------------------------------------------------------------------
        % deleteConnection: Delete an existing connection between the given
        % viewIds
        %------------------------------------------------------------------
        function this = deleteConnection(this, viewId1, viewId2)

            checkIfConnectionIsMissing(this, viewId1, viewId2);
            % Get the index of the connection to be deleted
            idx = getConnectionIndex(this, viewId1, viewId2);
            % Get the indices of the connections left
            % 1:numel(this.ViewId1) denoted the indices of all the existing
            % connections
            [connsLeft, indicesConnsLeft] = setdiff((1:numel(this.ViewId1)), idx(1));
            % Get the connection being deleted which is to be passed to
            % removeEdges function
            [vId1, vId2, rPose, iMat, matches] = this.getConnection(idx(1));
            connTable = struct('ViewId1', vId1, 'ViewId2', vId2);
            connTable.RelativePose = rPose;
            connTable.InformationMatrix = iMat;
            connTable.Matches = matches;
            % If no connections are left
            if isempty(connsLeft)
                % Assigning the ViewIds
                this.ViewId1 = zeros(0, 1,'uint32');
                this.ViewId2 = zeros(0, 1,'uint32');
                % Assigning the RelativePose
                if strcmp(this.RelClass, 'simtform3d') ...
                        || strcmp(this.RelClass, 'affine3d')
                    if strcmp(this.RelPoseClass, 'double')
                        this.RaffinePoseDouble = vision.internal.codegen.imageviewset.imageviewsetBase.makeEmptyAffine3d('double');
                    else
                        this.RaffinePoseSingle = vision.internal.codegen.imageviewset.imageviewsetBase.makeEmptyAffine3d('single');
                    end
                else
                    if strcmp(this.RelPoseClass, 'double')
                        this.RrigidPoseDouble = vision.internal.codegen.imageviewset.imageviewsetBase.makeEmptyRigid3d('double');
                    else
                        this.RrigidPoseSingle = vision.internal.codegen.imageviewset.imageviewsetBase.makeEmptyRigid3d('single');
                    end
                end
                % Assigning the Information Matrix
                if strcmp(this.InfoMatClass, 'single')
                    this.InfoMatSingle = {zeros(coder.ignoreConst(0), coder.ignoreConst(0), 'single')};
                else
                    this.InfoMatDouble = {zeros(coder.ignoreConst(0), coder.ignoreConst(0))};
                end
                % Assigning the Matches
                if strcmp(this.MatchesClass, 'single')
                    this.MatchesSingle = {zeros(coder.ignoreConst(0), 2, 'single')};
                elseif strcmp(this.MatchesClass, 'double')
                    this.MatchesDouble = {zeros(coder.ignoreConst(0), 2)};
                else
                    this.MatchesInt = {zeros(coder.ignoreConst(0), 2, 'uint32')};
                end
            else
                % Storing the remaining connection properties
                this.ViewId1 = this.ViewId1(indicesConnsLeft);
                this.ViewId2 = this.ViewId2(indicesConnsLeft);
                if strcmp(this.RelClass, 'simtform3d') ...
                        || strcmp(this.RelClass, 'affine3d')
                    if strcmp(this.RelPoseClass, 'double')
                        this.RaffinePoseDouble = this.RaffinePoseDouble(indicesConnsLeft);
                    else
                        this.RaffinePoseSingle = this.RaffinePoseSingle(indicesConnsLeft);
                    end
                else
                    if strcmp(this.RelPoseClass, 'double')
                        this.RrigidPoseDouble = this.RrigidPoseDouble(indicesConnsLeft);
                    else
                        this.RrigidPoseSingle = this.RrigidPoseSingle(indicesConnsLeft);
                    end
                end
                this = this.updateInfoMat(indicesConnsLeft);
                this = this.updateMatches(indicesConnsLeft);
            end

            % Remove edges in feature graph
            if this.IncrementalBuilding
                this.FeatureGraph = removeEdges(this.FeatureGraph, connTable);
            end
        end

        %------------------------------------------------------------------
        % hasConnection: Checks if there is an existing connection between
        % the given viewIds
        %------------------------------------------------------------------
        function tf = hasConnection(this, viewId1, viewId2)

            tf = hasConnection@vision.internal.codegen.imageviewset.imageviewsetBase(this, viewId1, viewId2);
        end

        %------------------------------------------------------------------
        % connectedViews: Gets the views connected to the given viewIds
        %------------------------------------------------------------------
        function [viewTable, dist] = connectedViews(this, varargin)

            narginchk(2,6);
            [viewId, maxDistance, minNumMatches]  = parseConnectedViewsInputs(this, varargin{:});
            isMinMatchesSyntax = minNumMatches>0;

            isStrongConnection = zeros(coder.ignoreConst(0), 1, 'logical');

            if maxDistance == 1 % Directly connected views
                [viewTable, connIdx]  = connectedViews@vision.internal.codegen.imageviewset.imageviewsetBase(this, viewId);

                if isMinMatchesSyntax
                    % Get the indices of Strong Connections
                    matches = this.Connections.Matches;
                    for i=1:numel(connIdx)
                        if connIdx(i) == 1
                            if (size(matches{i}, 1) >= minNumMatches)
                                isStrongConnection = [isStrongConnection; logical(1)]; %#ok<*AGROW> 
                            else
                                isStrongConnection = [isStrongConnection; logical(0)]; %#ok<*AGROW> 
                            end
                        end
                    end
                    n = numel(find(isStrongConnection));
                    vIds =  viewTable.ViewId;
                    absPose = viewTable.AbsolutePose;
                    if n > 0
                        % Return only the strong connections
                        viewTable.ViewId = vIds(isStrongConnection);
                        id = 0;
                        for i=1:numel(isStrongConnection)
                            if isStrongConnection(i) == 1
                                id = i;
                                break;
                            end
                        end
                        abPose = absPose(id);
                        for i = id+1:numel(isStrongConnection)
                            if isStrongConnection(i) == 1
                                abPose = [abPose, absPose(i)];
                            end
                        end
                        viewTable.AbsolutePose = abPose;
                        features = cell(n, 1);
                        points = cell(n, 1);
                        featuresTab = viewTable.Features;
                        pointsTab = viewTable.Points;
                        k = 1;
                        for i = 1:numel(isStrongConnection)
                            if isStrongConnection(i)==1
                                features{k} = featuresTab{i};
                                points{k} = pointsTab{i};
                                k = k+1;
                            end
                        end
                        viewTable.Features = features;
                        viewTable.Points = points;
                    else
                        % Empty struct when there are no strong connections
                        % left
                        viewTable = struct('ViewId', zeros(0, 1,'uint32'), ...
                            'AbsolutePose', rigidtform3d(eye(4, 4, this.AbsPoseClass)));
                        viewTable.Features = {};
                        viewTable.Points = {};
                    end
                end

                dist = ones(height(viewTable), 1);

            else % Indirectly connected views

                connections = this.Connections;
                strongConnectionIdx = zeros(coder.ignoreConst(0), 1);
                if isMinMatchesSyntax
                    matches = connections.Matches;
                    for i = 1:numel(matches)
                        if size(matches{i}, 1) >= minNumMatches
                            strongConnectionIdx = [strongConnectionIdx; i];
                        end
                    end

                    s = connections.ViewId1(strongConnectionIdx);
                    t = connections.ViewId2(strongConnectionIdx);
                else
                    s = connections.ViewId1;
                    t = connections.ViewId2;
                end

                % Check if the query view is not connected to any view
                if ~ismember(viewId, [s; t])
                    viewTable = struct('ViewId', zeros(coder.ignoreConst(0), 1,'uint32'), ...
                        'AbsolutePose', rigidtform3d(eye(4, 4, this.AbsPoseClass)));
                    viewTable.Features = {};
                    viewTable.Points = {};
                    dist = [];
                else

                    % Create a graph using view IDs
                    G = graph(s, t);

                    [nearestViewIds, dist] = nearest(G,double(viewId),maxDistance,'Method','unweighted');
                    viewTable = findView(this, nearestViewIds);
                end
            end
        end

        %------------------------------------------------------------------
        % findTracks: Find matched points across multiple views
        %------------------------------------------------------------------
        function tracks = findTracks(this, varargin)
        % Parsing the inputs
            [viewIds, minTrackLength] = parseFindTracksInputs(this, varargin{:});

            % Return empty pointTrack when there are no views
            if isempty(this.ViewId1)
                if coder.internal.prop_has_class(this, 'PointsClass')
                    if strcmp(this.PointsClass, 'double')
                        tracks = this.makeEmptyPointTrack('double');
                    else
                        tracks = this.makeEmptyPointTrack('single');
                    end
                else
                    tracks = this.makeEmptyPointTrack('double');
                end
            else
                if ~this.IncrementalBuilding
                    this.FeatureGraph = addNodes(this.FeatureGraph, this.Views);
                    this.FeatureGraph = addEdges(this.FeatureGraph, this.Connections);
                end
                tracks = createTracks(this.FeatureGraph, viewIds, ...
                                      this.NumViews, minTrackLength);
            end
        end

        %------------------------------------------------------------------
        % poses: Returns the AbsolutePose associated with the given ViewIds
        %------------------------------------------------------------------
        function sensorPoses = poses(this, varargin)

            if nargin>1
                viewIds = checkViewIds(this, varargin{:});
                checkIfViewIsMissing(this, viewIds);
                [~, idx] = ismember(viewIds(:), this.ViewId);
                viewsId = this.ViewId(idx);
                absPoses = this.Views.AbsolutePose(idx);
            else
                viewsId = this.ViewId;
                absPoses = this.Views.AbsolutePose;
            end
            sensorPoses = struct('ViewId', viewsId, 'AbsolutePose', ...
                                 absPoses);
        end

        %------------------------------------------------------------------
        % createPoseGraph: Return a pose graph
        %------------------------------------------------------------------
        function G = createPoseGraph(this)

        % Get the Matches of the viewset object
            if coder.internal.prop_has_class(this, 'MatchesClass')
                if strcmp(this.MatchesClass, 'single')
                    matches = this.MatchesSingle;
                elseif strcmp(this.MatchesClass, 'double')
                    matches = this.MatchesDouble;
                else
                    matches = this.MatchesInt;
                end
            else
                matches = this.MatchesDouble;
            end
            % Compute the weights from the Matches
            nConns = numel(matches);
            coder.varsize('weight', [Inf, 1],[1 0]);
            weight = zeros(nConns, 1);
            for i = 1:nConns
                weight(i) = size(matches{i}, 1);
            end

            G = createPoseGraph@vision.internal.codegen.imageviewset.imageviewsetBase(this, weight);
        end

        %------------------------------------------------------------------
        % optimizePoses: Returns the optimized AbsolutePoses of the viewset
        %------------------------------------------------------------------
        function varargout = optimizePoses(this, varargin)

            isMinMatchesSyntax = nargin>1 && isnumeric(varargin{1});
            if isMinMatchesSyntax
                minNumMatches = varargin{1};
                checkMinNumMatches(this, minNumMatches);
            end

            G = createPoseGraph(this);

            % Remove edges with weight below minNumMatches
            if isMinMatchesSyntax
                weakEdges = G.Edges.Weight < minNumMatches;
                G = rmedge(G, find(weakEdges));
                % Ensure that the modified digraph is connected.
                bins = conncomp(G, 'Type', 'weak');
                if ~isempty(bins)
                    coder.internal.errorIf(max(bins)>1, 'vision:viewSet:disconnectedGraph');
                end
            end

            poseTable = vision.internal.optimizePoses(G, varargin{1+isMinMatchesSyntax:end});
            for i = 1:height(poseTable)
                this = updateView(this, poseTable.ViewId(i), rigidtform3d(poseTable.AbsolutePose{i}'));
            end
            varargout{1} = this;

            if ~any(strcmp('Scale', poseTable.Properties.VariableNames))
                nargoutchk(0,1);
            else
                nargoutchk(0,2);
                varargout{2} =  poseTable.Scale;
            end
        end
    end

    methods (Access = protected)
        %------------------------------------------------------------------
        % Utility Functions
        %------------------------------------------------------------------
        function [this, wipedConnTable] = wipeMatches(this, viewIds)

        % Initialize the wipedConnTable
            wipedConnTable = struct('ViewId1', uint32([]), 'ViewId2', uint32([]));
            wipedConnTable.RelativePose = {};
            wipedConnTable.InformationMatrix = {};
            wipedConnTable.Matches = {};
            % Return if no connections are added
            if ~coder.internal.prop_has_class(this, 'RelClass')
                return;
            end
            % Return if there are no views
            if isempty(this.ViewId1)
                return;
            end

            for id = 1:numel(viewIds)
                idx = getConnectionIndexToAndFrom(this, viewIds(id));
                conIdx = find(idx);
                nConns = numel(conIdx);
                if ~isempty(conIdx)
                    vId1 = zeros(nConns, 1, 'uint32');
                    vId2 = zeros(nConns, 1, 'uint32');
                    rPose = cell(nConns, 1);
                    match = cell(nConns, 1);
                    iMats = cell(nConns, 1);
                    for i=1:nConns
                        % Get the connection at required index
                        [v1, v2, r, iMat, m] = this.getConnection(conIdx(i));
                        vId1(i) = v1;
                        vId2(i) = v2;
                        rPose{i} = r{1};
                        iMats{i} = iMat{1};
                        match{i} = m{1};
                        % Update the Matches
                        if strcmp(this.MatchesClass, 'single')
                            this.MatchesSingle{conIdx(i)} = zeros(0, 2, 'single');
                        elseif strcmp(this.MatchesClass, 'double')
                            this.MatchesDouble{conIdx(i)} = zeros(0, 2);
                        else
                            this.MatchesInt{conIdx(i)} = zeros(0, 2, 'uint32');
                        end
                    end
                    % Update the wipedConnTable table
                    wipedConnTable = struct('ViewId1', vId1, 'ViewId2', vId2);
                    wipedConnTable.RelativePose = rPose;
                    wipedConnTable.InformationMatrix = iMats;
                    wipedConnTable.Matches = match;
                end
            end
        end

        %------------------------------------------------------------------
        % Parse the View inputs
        %------------------------------------------------------------------
        function [this, viewTable, unsetColumns] = parseViewInputs(this, varargin)

            coder.internal.prefer_const(varargin{:});

            narginchk(2, 7);
            viewId = uint32(this.checkViewId(varargin{1}));
            unsetColumns = cell(1, coder.ignoreConst(0));
            if length(varargin) == 1
                % If only viewId is given as input
                absPose = this.getDeafultPose();
                points = this.getDeafultPoints();
                features = this.getDeafultFeatures();
                unsetColumns = {'absPose', 'Features', 'Points'};
            elseif length(varargin) == 2
                % ViewId and AbsolutePose are inputs
                tform = varargin{2};
                this.checkPose(tform, 'absPose');
                absPose = rigidtform3d(tform.T');
                points = this.getDeafultPoints();
                features = this.getDeafultFeatures();
                unsetColumns = {'Features', 'Points'};
            elseif length(varargin) == 3
                % ViewId and (Points or Features) are inputs
                absPose = this.getDeafultPose();
                [features, points] = parseParams(this, varargin{2:end});
                if contains('Features', varargin{2},...
                            'IgnoreCase',true)
                    unsetColumns = {'absPose', 'Points'};
                else
                    if contains('Points', varargin{2},...
                                'IgnoreCase',true)
                        unsetColumns = {'absPose', 'Features'};
                    end
                end
            elseif length(varargin) == 4
                % ViewId, AbsolutePose and (Points or Features) are inputs
                tform = varargin{2};
                this.checkPose(tform, 'absPose');
                absPose = rigidtform3d(tform.T');
                [features, points] = parseParams(this, varargin{3:end});
                if contains('Features', varargin{3},...
                            'IgnoreCase',true)
                    unsetColumns = {'Points'};
                end
                if contains('Points', varargin{3},...
                            'IgnoreCase',true)
                    unsetColumns = {'Features'};
                end
            elseif length(varargin) == 5
                % ViewId, Points and Features are inputs
                absPose = this.getDeafultPose();
                [features, points] = parseParams(this, varargin{2:end});
                unsetColumns = {'absPose'};
            else
                if length(varargin) == 6
                    % ViewId, AbsolutePose, Points and Features are inputs
                    tform = varargin{2};
                    this.checkPose(tform, 'absPose');
                    absPose = rigidtform3d(tform.T');
                    [features, points] = parseParams(this, varargin{3:end});
                    unsetColumns = {'none'};
                end
            end
            this.checkPose(absPose, 'absPose', {'rigidtform3d'});
            % Error if the class of given AbsolutePose is not equal to the
            % class of existing AbsolutePoses if there are any exiting
            % Views
            if coder.internal.prop_has_class(this, 'AbsPoseClass')
                coder.internal.assert(isequal(this.AbsPoseClass, class(absPose.T)),...
                                      'images:geotrans:differentTypes');
            end
            % Error if the class of given Features is not equal to the
            % class of existing Features if there are any exiting
            % Views
            if coder.internal.prop_has_class(this, 'FeaturesClass')
                coder.internal.assert(isequal(this.FeaturesClass, ...
                                              class(features)), 'vision:viewSet:differentFeatures', ...
                                      this.FeaturesClass);
            end
            % Error if the class of given Points is not equal to the
            % class of existing Points if there are any exiting
            % Views
            if coder.internal.prop_has_class(this, 'PointsClass')
                coder.internal.assert(isequal(this.PointsClass, class(points)),...
                                      'vision:viewSet:differentPoints', this.PointsClass);
            end

            % Store the class of 'T' property of AbsolutePose
            this.AbsPoseClass = class(absPose.T);
            % Stroe the class of Points
            this.PointsClass = class(points);
            % Stroe the class of Features
            this.FeaturesClass = class(features);

            % Return the struct with ViewId, AbsolutePose, Features and
            % Points
            viewTable = struct('ViewId', viewId, 'AbsolutePose', absPose);
            viewTable.Features = {features};
            viewTable.Points = {points};
        end

        %------------------------------------------------------------------
        % Parse the input Points and Features
        %------------------------------------------------------------------
        function [features, points] = parseParams(this, varargin)
            defaults = struct(...
                'Points',    this.getDeafultPoints(), ...
                'Features',  this.getDeafultFeatures());

            % Define parser mapping struct
            params = struct('Points', uint32(0), ...
                            'Features', uint32(0));

            % Parse PV pairs
            pstruct = coder.internal.parseParameterInputs(params, this.ParserOptions, varargin{:});
            features = coder.internal.getParameterValue(pstruct.Features, defaults.Features, varargin{:});
            points = coder.internal.getParameterValue(pstruct.Points, defaults.Points, varargin{:});
            this.checkFeatures(features);
            this.checkPoints(points);
        end

        %------------------------------------------------------------------
        % Parse the input Matches
        %------------------------------------------------------------------
        function matches = parseMatches(this, varargin)
            defaults = struct(...
                'Matches',    this.getDefaultMatches());

            % Define parser mapping struct
            params = struct('Matches', uint32(0));

            % Parse PV pairs
            pstruct = coder.internal.parseParameterInputs(params, this.ParserOptions, varargin{:});
            matches = coder.internal.getParameterValue(pstruct.Matches, defaults.Matches, varargin{:});
            this.checkMatches(matches);
        end

        %------------------------------------------------------------------
        function checkMatchesOutOfBounds(this, matches, viewId1, viewId2)
        % Check that match indices are valid.
            view1 = getViewIndex(this, viewId1);
            view2 = getViewIndex(this, viewId2);

            if strcmp(this.PointsClass, 'double')
                points1 = this.PointsDouble{view1};
                points2 = this.PointsDouble{view2};
            else
                points1 = this.PointsSingle{view1};
                points2 = this.PointsSingle{view2};
            end

            areMatchesOutOfBounds = ...
                max(matches(:, 1)) > size(points1, 1) || ...
                max(matches(:, 2)) > size(points2, 1);
            coder.internal.errorIf(areMatchesOutOfBounds, 'vision:viewSet:matchIdxOutOfBounds');
        end

        %------------------------------------------------------------------
        % Validate and Return the viewTable
        %------------------------------------------------------------------
        function [this, viewTable, unsetColumns] = checkViewTable(this, viewTableIn, fillMissingVars)

            coder.internal.prefer_const(fillMissingVars, viewTableIn);

            variableNames = viewTableIn.Properties.VariableNames;

            hasRequiredVariables = isequal(variableNames{1}, 'ViewId');
            coder.internal.errorIf(~hasRequiredVariables, 'vision:viewSet:requiredColumnsMissing', ...
                                   'viewTable', 'ViewId');

            hasAbsolutePose = any(strcmp('AbsolutePose', variableNames));
            hasPoints       = any(strcmp('Points', variableNames));
            hasFeatures     = any(strcmp('Features', variableNames));

            hasOneOptionalVariables   = width(viewTableIn)==2 && (hasAbsolutePose || hasPoints || hasFeatures);
            hasTwoOptionalVariables   = width(viewTableIn)==3 && hasAbsolutePose && (hasPoints || hasFeatures);
            hasThreeOptionalVariables = width(viewTableIn)==4 && hasAbsolutePose && hasPoints && hasFeatures;

            optionalColumnsInvalid = width(viewTableIn)>1 && ~(hasOneOptionalVariables || hasTwoOptionalVariables || hasThreeOptionalVariables);
            coder.internal.errorIf(optionalColumnsInvalid,'vision:viewSet:optionalColumnsInvalid', ...
                                   'viewTable', 'ViewId', 'Features, Points');

            viewIds = uint32(this.checkViewIds(viewTableIn.ViewId));
            % Get the height of input table
            theight = height(viewTableIn);
            points =  cell(theight, 1);
            features =  cell(theight, 1);
            % Initialize aPose with the AbsolutePose of viewId in first row
            if hasAbsolutePose
                if isa(viewTableIn.AbsolutePose, 'cell')
                    tform = viewTableIn.AbsolutePose{1};
                else
                    tform = viewTableIn.AbsolutePose(1);
                end
                checkPoses(this, tform, 'AbsolutePose');
                % Error if the class of given AbsolutePose is not equal to the
                % class of existing AbsolutePose if there are any exiting
                % Views
                if coder.internal.prop_has_class(this, 'AbsPoseClass')
                    coder.internal.assert(isequal(this.AbsPoseClass, class(tform.T)),...
                                          'images:geotrans:differentTypes');
                end
                absPose = rigidtform3d(tform.T');
            else
                absPose = this.getDeafultPose();
            end
            this.AbsPoseClass = class(absPose.T);
            for i=1:theight
                if hasAbsolutePose
                    if isa(viewTableIn.AbsolutePose, 'cell')
                        tform = viewTableIn.AbsolutePose{i};
                    else
                        tform = viewTableIn.AbsolutePose(i);
                    end
                    checkPoses(this, tform, 'AbsolutePose');
                    absPose(i) = rigidtform3d(tform.T');
                    % Error if the class of given AbsolutePose is not equal to the
                    % class of existing AbsolutePose if there are any exiting
                    % Views
                    if coder.internal.prop_has_class(this, 'AbsPoseClass')
                        coder.internal.assert(isequal(this.AbsPoseClass, class(absPose(i).T)),...
                                              'images:geotrans:differentTypes');
                    end
                    % Store the class of given AbsolutePose
                    this.AbsPoseClass = class(absPose(i).T);
                else
                    if fillMissingVars
                        absPose(i) = this.getDeafultPose();
                        this.AbsPoseClass = class(absPose(i).T);
                    end
                end

                if hasPoints
                    points{i} = viewTableIn.Points{i};
                    a = checkPointsArray(this, {points{i}});
                    points{i} = a{1};
                    % Error if the class of given Points is not equal to the
                    % class of existing Points if there are any exiting
                    % Views
                    if coder.internal.prop_has_class(this, 'PointsClass')
                        coder.internal.assert(isequal(this.PointsClass, class(points{i})),...
                                              'vision:viewSet:differentPoints', this.PointsClass);
                    end
                    % Store the class of given Points
                    this.PointsClass = class(points{i});
                else
                    if fillMissingVars
                        points{i} = this.getDeafultPoints();
                        this.PointsClass = class(points{i});
                    end
                end

                if hasFeatures
                    features{i} = viewTableIn.Features{i};
                    f = checkFeaturesArray(this, {features{i}});
                    features{i} = f{1};
                    % Error if the class of given Points is not equal to the
                    % class of existing Points if there are any exiting
                    % Views
                    if coder.internal.prop_has_class(this, 'FeaturesClass')
                        coder.internal.assert(isequal(this.FeaturesClass, class(features{i})),...
                                              'vision:viewSet:differentFeatures', ...
                                              this.FeaturesClass);
                    end
                    % Store the class of given Features
                    this.FeaturesClass = class(features{i});
                else
                    if fillMissingVars
                        features{i} = this.getDeafultFeatures();
                        this.FeaturesClass = class(features{i});
                    end
                end
            end

            if fillMissingVars
                viewTable = struct('ViewId', viewIds, 'AbsolutePose', absPose);
                viewTable.Features = features;
                viewTable.Points = points;
                unsetColumns = {'none'};
            else
                if hasAbsolutePose
                    viewTable = struct('ViewId', viewIds, 'AbsolutePose', absPose);
                    if hasFeatures
                        if hasPoints
                            % If viewId, AbsolutePose, Points, Features are given
                            viewTable.Features = features;
                            viewTable.Points = points;
                            unsetColumns = {'none'};
                        else
                            % If viewId, AbsolutePose, Features are given
                            viewTable.Features = features;
                            unsetColumns = {'Points'};
                        end
                    else
                        if hasPoints
                            % If viewId, AbsolutePose, Points are given
                            viewTable.Points = points;
                            unsetColumns = {'Features'};
                        else
                            % If viewId, AbsolutePose is given
                            unsetColumns = {'Features', 'Points'};
                        end
                    end
                else
                    viewTable = struct('ViewId', viewIds);
                    if hasFeatures
                        if hasPoints
                            % If viewId, Points, Features are given
                            viewTable.Features = features;
                            viewTable.Points = points;
                            unsetColumns = {'absPose'};
                        else
                            % If viewId ,Features are given
                            viewTable.Features = features;
                            unsetColumns = {'absPose', 'Points'};
                        end
                    else
                        if hasPoints
                            % If viewId, Points are given
                            viewTable.Points = points;
                            unsetColumns = {'absPose', 'Features'};
                        else
                            % If only viewId is given
                            unsetColumns = {'absPose', 'Features', 'Points'};
                        end
                    end
                end
            end
        end

        %------------------------------------------------------------------
        % Parse the connection inputs
        %------------------------------------------------------------------
        function [this, connTable, unsetColumns] = parseConnectionInputs(this, varargin)
            coder.internal.prefer_const(varargin{:});

            narginchk(3, 7);
            viewId1 = this.checkViewId(varargin{1});
            viewId2 = this.checkViewId(varargin{2});
            if length(varargin) == 2
                % Only ViewId1, ViewId2 are given
                relPose = this.getDefaultRelPose();
                unsetColumns = {'relPose', 'infoMat', 'Matches'};
            elseif length(varargin) == 3
                % ViewId1, ViewId2, RelativePose are given
                tform = varargin{3};
                this.checkPose(tform, 'relPose', {'rigidtform3d','simtform3d','rigid3d','affine3d'});
                % Similarity transform input
                if isa(tform, 'simtform3d') || isa(tform, 'affine3d')
                    relPose = simtform3d(tform.T');
                else % Rigid transform
                    relPose = rigidtform3d(tform.T');
                end
                unsetColumns = {'infoMat', 'Matches'};
            elseif length(varargin) == 4
                if isstring(varargin{3}) || ischar(varargin{3})
                    % ViewId1, ViewId2, Matches are given
                    m = this.parseMatches(varargin{3:end});
                    relPose = this.getDefaultRelPose();
                    unsetColumns = {'relPose', 'infoMat'};
                else
                    % ViewId1, ViewId2, RelativePose, InfoMat are given
                    tform = varargin{3};
                    this.checkPose(tform, 'relPose', {'rigidtform3d','simtform3d','rigid3d','affine3d'});
                    % Similarity transform input
                    if isa(tform, 'simtform3d') || isa(tform, 'affine3d')
                        relPose = simtform3d(tform.T');
                    else % Rigid transform
                        relPose = rigidtform3d(tform.T');
                    end
                    infoMat = varargin{4};
                    validateattributes(infoMat, {'single','double'}, ...
                                       {'real', '2d'}, this.ClassName, 'infoMat')
                    unsetColumns = {'Matches'};
                end
            elseif length(varargin) == 5
                % ViewId1, ViewId2, RelativePose, Matches are given
                tform = varargin{3};
                this.checkPose(tform, 'relPose', {'rigidtform3d','simtform3d','rigid3d','affine3d'});
                % Similarity transform input
                if isa(tform, 'simtform3d') || isa(tform, 'affine3d')
                    relPose = simtform3d(tform.T');
                else % Rigid transform
                    relPose = rigidtform3d(tform.T');
                end
                m = this.parseMatches(varargin{4:end});
                unsetColumns = {'infoMat'};
            else
                % ViewId1, ViewId2, RelativePose, InfoMat, Matches are given
                tform = varargin{3};
                this.checkPose(tform, 'relPose', {'rigidtform3d','simtform3d','rigid3d','affine3d'});
                % Similarity transform input
                if isa(tform, 'simtform3d') || isa(tform, 'affine3d')
                    relPose = simtform3d(tform.T');
                else % Rigid transform
                    relPose = rigidtform3d(tform.T');
                end
                infoMat = varargin{4};
                validateattributes(infoMat, {'single','double'}, ...
                                   {'real', '2d'}, this.ClassName, 'infoMat')
                m = this.parseMatches(varargin{5:end});
                unsetColumns = {'none'};
            end
            
            % Old classes should be converted at this point
            this.checkPose(relPose, 'relPose', {'rigidtform3d','simtform3d'});
            % Error if the class of given RelativePose is not equal to the
            % class of existing RelativePose if there are any exiting
            % Connections
            if coder.internal.prop_has_class(this, 'RelClass')
                coder.internal.assert(isequal(this.RelClass, class(relPose)),...
                                      'vision:viewSet:differentRelPoses', this.RelClass);
            end
            % Error if the class of given RelativePose(T property) is not equal to the
            % class of existing RelativePose(T property) if there are any exiting
            % Connections
            if coder.internal.prop_has_class(this, 'RelPoseClass')
                coder.internal.assert(isequal(this.RelPoseClass, class(relPose.T)),...
                                      'vision:viewSet:differentTypes', this.RelPoseClass);
            end
            % Store the class of RelativePose (simtform3d or rigidtform3d)
            this.RelClass = class(relPose);
            % Store the class of RelativePose 'T' property(single or double)
            this.RelPoseClass = class(relPose.T);

            isSimilarityConnection = strcmp(this.RelClass, 'simtform3d') || ...
                strcmp(this.RelClass, 'affine3d');
            infoMatDoF = 6 + isSimilarityConnection;

            if isSimilarityConnection && ~isSimilarity(relPose)
                coder.internal.error('vision:viewSet:notSimilarityTransform', 'relPose');
            end

            infoMatUnset = any(strcmp('infoMat', unsetColumns));
            if infoMatUnset
                infoMatrix = this.getDefaultInfoMat();
            else
                % Ideally only size should be checked
                validateattributes(infoMat, {'single','double'}, ...
                                   {'size', [infoMatDoF, infoMatDoF]}, this.ClassName, 'infoMat');
                infoMatrix = {infoMat};
            end

            mUnset = any(strcmp('Matches', unsetColumns));
            if mUnset
                matches = this.getDefaultMatches();
            else
                checkMatchesOutOfBounds(this, m, viewId1, viewId2);
                matches = {m};
            end

            % Error if the class of given Information Matrix is not equal to the
            % class of existing Information Matrix if there are any exiting
            % Connections
            if coder.internal.prop_has_class(this, 'InfoMatClass')
                coder.internal.assert(isequal(this.InfoMatClass, class(infoMatrix{1})),...
                                      'vision:viewSet:differentIMat', this.InfoMatClass);
            end
            % Error if the class of given Matches is not equal to the
            % class of existing Matches if there are any exiting
            % Connections
            if coder.internal.prop_has_class(this, 'MatchesClass')
                coder.internal.assert(isequal(this.MatchesClass, class(matches{1})),...
                                      'vision:viewSet:differentMatches', this.MatchesClass);
            end

            % Store the class of Information Matrix
            this.InfoMatClass = class(infoMatrix{1});
            % Store the class of Matches
            this.MatchesClass = class(matches{1});

            coder.internal.errorIf(~hasView(this, viewId1), ...
                                   'vision:viewSet:missingViewId', viewId1(1));
            coder.internal.errorIf(~hasView(this, viewId2), ...
                                   'vision:viewSet:missingViewId', viewId2(1));

            % Create the structure
            connTable = struct('ViewId1', viewId1, 'ViewId2', viewId2);
            connTable.RelativePose = relPose;
            connTable.InformationMatrix = infoMatrix;
            connTable.Matches = matches;
        end

        %------------------------------------------------------------------
        % Parse the inputs to findTrack function
        %------------------------------------------------------------------
        function [viewIds, minTrackLength]  = parseFindTracksInputs(this, varargin)

            narginchk(1, 4);
            if isempty(varargin)
                viewIds = this.ViewId;
                vars = {};
            elseif length(varargin) == 1
                viewIds = uint32(this.checkViewIds(varargin{1}));
                vars = {};
            elseif length(varargin) == 2
                viewIds = this.ViewId;
                vars = varargin;
            else
                viewIds = uint32(this.checkViewIds(varargin{1}));
                vars = {varargin{2}; varargin{3}};
            end
            defaults = struct(...
                'MinTrackLength',    2);

            % Define parser mapping struct
            params = struct('MinTrackLength', uint32(0));

            % Parse PV pairs
            pstruct = coder.internal.parseParameterInputs(params, this.ParserOptions, vars{:});
            minTrackLength = coder.internal.getParameterValue(pstruct.MinTrackLength, defaults.MinTrackLength, vars{:});
            this.checkMinTrackLength(minTrackLength);
        end
        
        %------------------------------------------------------------------
        % Parse the inputs to connectedViews function
        %------------------------------------------------------------------
        function [viewId, maxDistance, minNumMatches]  = parseConnectedViewsInputs(this, varargin)
            viewId = varargin{1};

            defaults = struct('MaxDistance', 1, 'MinNumMatches', 0);

            % Define parser struct
            params = struct('MaxDistance', uint32(0), 'MinNumMatches', uint32(0));

            % Parse PV pairs
            pstruct = coder.internal.parseParameterInputs(params, this.ParserOptions, varargin{2:end});
            maxDistance = coder.internal.getParameterValue(pstruct.MaxDistance, defaults.MaxDistance, varargin{2:end});
            minNumMatches = coder.internal.getParameterValue(pstruct.MinNumMatches, defaults.MinNumMatches, varargin{2:end});
            this.checkMaxDistance(maxDistance);
            allowZero = true;
            this.checkMinNumMatches(minNumMatches, allowZero);
        end

        %------------------------------------------------------------------
        function checkPoints(this, points)
            vision.internal.inputValidation.checkPoints(...
                points, this.ClassName, 'Points');
        end

        %------------------------------------------------------------------
        function pointsArray = checkPointsArray(this, pointsArray)
            vision.internal.inputValidation.checkPoints(...
                vertcat(pointsArray{:}), this.ClassName, 'Points');
        end

        %------------------------------------------------------------------
        function checkFeatures(this, features)
            vision.internal.inputValidation.checkFeatures(features, this.ClassName, 'Features');
        end

        %------------------------------------------------------------------
        function featuresArray = checkFeaturesArray(this, featuresArray)
            if isnumeric(featuresArray{1})
                vision.internal.inputValidation.checkFeatures(...
                    vertcat(featuresArray{:}), this.ClassName, 'Features');
            else % binaryFeatures
                for i = 1:numel(featuresArray)
                    vision.internal.inputValidation.checkFeatures(...
                        featuresArray{i}, this.ClassName, 'Features');
                end
            end
        end

        %-----------------------------------------------------------------
        function checkMinTrackLength(this, minTrackLength)
            validateattributes(minTrackLength, {'numeric'}, {'nonsparse', 'scalar', ...
                                                             'integer', 'positive', '>=', 2}, ...
                               this.ClassName, 'minTrackLength');

            coder.internal.errorIf(this.NumViews >= 2 && minTrackLength > this.NumViews, ...
                                   'vision:viewSet:exceedNumViews', 'MinTrackLength');
        end

        %-----------------------------------------------------------------
        function checkMinNumMatches(this, minNumMatches, varargin)
            if nargin > 2
                allowZero = varargin{1};
            else
                allowZero = false;
            end

            if ~allowZero
                validateattributes(minNumMatches, {'numeric'}, ...
                    {'scalar', 'positive', 'real', 'integer', 'nonsparse'}, ...
                    this.ClassName, 'MinNumMatches');
            else
                validateattributes(minNumMatches, {'numeric'}, ...
                    {'scalar', 'nonnegative', 'real', 'integer', 'nonsparse'}, ...
                    this.ClassName, 'MinNumMatches');
            end
        end

        %-----------------------------------------------------------------
        function checkMaxDistance(this, maxDist)
            validateattributes(maxDist, {'numeric'}, ...
                {'scalar', 'positive', 'real', 'integer', 'nonsparse'}, ...
                this.ClassName, 'MaxDistance');
        end

        %------------------------------------------------------------------
        function checkMatches(this, matches)
            if ~isempty(matches)
                validateattributes(matches, {'numeric'}, ...
                                   {'nonsparse', '2d', 'ncols', 2, 'integer', 'positive'}, ...
                                   this.ClassName, 'Matches');
            end
        end

        %------------------------------------------------------------------
        % Return the default value of Points attribute of Views property
        %------------------------------------------------------------------
        function points = getDeafultPoints(this)
        % If at least one view exists return the Points object based on
        % class of existing Points
            if coder.internal.prop_has_class(this, 'PointsClass')
                if strcmp(this.PointsClass, 'double')
                    points = zeros(0,2);
                else
                    if strcmp(this.PointsClass, 'cornerPoints')
                        points = cornerPoints();
                    elseif strcmp(this.PointsClass, 'BRISKPoints')
                        points = BRISKPoints();
                    elseif strcmp(this.PointsClass, 'SURFPoints')
                        points = SURFPoints();
                    elseif strcmp(this.PointsClass, 'ORBPoints')
                        points = ORBPoints();
                    elseif strcmp(this.PointsClass, 'KAZEPoints')
                        points = KAZEPoints();
                    elseif strcmp(this.PointsClass, 'MSERRegions')
                        points = MSERRegions();
                    else
                        points = zeros(0, 2, 'single');
                    end
                end
            else
                % Return this if the view being added is the first view
                points = zeros(0,2);
            end
        end

        %------------------------------------------------------------------
        % Return the default value of Features attribute of Views property
        %------------------------------------------------------------------
        function features = getDeafultFeatures(this)
        % If at least one view exists return the Features object based on
        % class of existing Features
            if coder.internal.prop_has_class(this, 'FeaturesClass')
                if strcmp(this.FeaturesClass, 'double')
                    features = zeros(0,0);
                elseif strcmp(this.FeaturesClass, 'single')
                    features = zeros(0, 0, 'single');
                elseif strcmp(this.FeaturesClass, 'uint8')
                    features = zeros(0, 0, 'uint8');
                else
                    features = binaryFeatures(zeros(0, 0, 'uint8'));
                end
            else
                % Return this if the view being added is the first view
                features = zeros(0,0);
            end
        end

        %------------------------------------------------------------------
        % Return the default value of AbsolutePose attribute of Views
        % property
        %------------------------------------------------------------------
        function absPose = getDeafultPose(this)
        % If at least one view exists return the AbsolutePose based on
        % class of existing AbsolutePose
            if coder.internal.prop_has_class(this, 'AbsPoseClass')
                if strcmp(this.AbsPoseClass, 'single')
                    absPose = rigidtform3d(eye(3, 3, 'single'), zeros(1, 3, 'single'));
                else
                    absPose = rigidtform3d(eye(3), [0 0 0]);
                end
            else
                % Return this if the view being added is the first view
                absPose = rigidtform3d(eye(3), [0 0 0]);
            end

        end

        %------------------------------------------------------------------
        % Return the default value of RelativePose attribute of Connections
        % property
        %------------------------------------------------------------------
        function relPose = getDefaultRelPose(this)
        % If at least one connection exists return the RelativePose based on
        % class of existing RelativePose
            if coder.internal.prop_has_class(this, 'RelClass')
                if strcmp(this.RelClass, 'simtform3d') ...
                        || strcmp(this.RelClass, 'affine3d')
                    if strcmp(this.RelPoseClass, 'single')
                        relPose = simtform3d(eye(4, 4, 'single'));
                    else
                        relPose = simtform3d(eye(4));
                    end
                else
                    if strcmp(this.RelPoseClass, 'single')
                        relPose = rigidtform3d(eye(3, 3, 'single'), zeros(1, 3, 'single'));
                    else
                        relPose = rigidtform3d(eye(3), [0 0 0]);
                    end
                end
            else
                % Return this if the connection being added is the first
                % connection
                relPose = rigidtform3d(eye(3), [0 0 0]);
            end
        end

        %------------------------------------------------------------------
        % Return the default value of Information Matrix attribute of
        % Connections property
        %------------------------------------------------------------------
        function infoMat = getDefaultInfoMat(this)
            if strcmp(this.RelClass, 'simtform3d') ...
                    || strcmp(this.RelClass, 'affine3d')
                if coder.internal.prop_has_class(this, 'InfoMatClass')
                    if strcmp(this.InfoMatClass, 'single')
                        infoMat = {eye(7, 7, 'single')};
                    else
                        infoMat = {eye(7, 7)};
                    end
                else
                    infoMat = {eye(7, 7)};
                end
            else
                if coder.internal.prop_has_class(this, 'InfoMatClass')
                    if strcmp(this.InfoMatClass, 'single')
                        infoMat = {eye(6, 6, 'single')};
                    else
                        infoMat = {eye(6, 6)};
                    end
                else
                    infoMat = {eye(6, 6)};
                end
            end
        end

        %------------------------------------------------------------------
        % Return the default value of Matches attribute of
        % Connections property
        %------------------------------------------------------------------
        function matches = getDefaultMatches(this)
        % If at least one connection exists return the Matches based on
        % class of existing Matches
            if coder.internal.prop_has_class(this, 'MatchesClass')
                if strcmp(this.MatchesClass, 'single')
                    matches = {zeros(0, 2, 'single')};
                elseif strcmp(this.MatchesClass, 'double')
                    matches = {zeros(0, 2)};
                else
                    matches = {zeros(0, 2, 'uint32')};
                end
            else
                matches = {zeros(0, 2, 'uint32')};
            end
        end
    end
end
