classdef (Hidden)imageviewsetBase < vision.internal.ViewSetBaseImpl
    % imageviewsetBase class is the interface class for codegen
    % version of imageviewset

    % Copyright 2021-2023 The MathWorks, Inc.
    % Notes
    %     1. imageviewset code generation implementation is different from
    %     simulation. As imageviewset constructor is empty, need to accommodate
    %     all rigidtform3d, simtform3d, Points types in the constructor for code generation.
    %     Properties AbsPosesDouble, AbsPosesSingle, PointsSingle,
    %     PointsDouble, FeaturesSingle, FeaturesDouble are used for Views
    %     related functions and similar properties are defined for Connections
    %     functions.
    %
    %     2. Non Tunable properties AbsPoseClass, PointsClass, FeaturesClass
    %     are used to maintain the homogeneous nature for code generation.

    %#codegen

    properties (Dependent, GetAccess = public, SetAccess = protected)
        % Views  A structure containing view attributes,
        % 'ViewId', 'AbsolutePose', 'Features' and 'Points'.
        Views

        % Connections A structure containing connection attributes,
        % 'ViewId1', 'ViewId2', 'RelativePose', 'InformationMatrix' and
        % 'Matches'
        Connections

        FeatureGraph
    end

    properties (SetAccess = protected)
        % FeatureGraphSingle is a graph which store Points of type single
        % in Nodes
        FeatureGraphSingle
        % FeatureGraphDouble is a graph which store Points of type double
        % in Nodes
        FeatureGraphDouble
    end

    properties (SetAccess = protected)
        %NumViews Number of views in a view set
        NumViews

        %NumConnections Number of connections in a view set
        NumConnections
    end

    properties (Hidden, SetAccess = protected)
        % Properties used for Views structure
        % AbsPosesDouble holds rigidtform3d objects.This is initialized in
        % codegen mode if class of Rotation or T matrix is 'double'.
        AbsPosesDouble
        % AbsPosesSingle holds rigidtform3d objects.This is initialized in
        % codegen mode if class of Rotation or T matrix is 'single'.
        AbsPosesSingle
        % FeaturesSingle holds Features elements. This is initialized in
        % codegen mode if class of Features is 'single'.
        FeaturesSingle
        % FeaturesDouble holds Features elements. This is initialized in
        % codegen mode if class of Features is 'double'.
        FeaturesDouble
        % FeaturesBinary holds Features elements. This is initialized in
        % codegen mode if class of Features is 'BinaryFeatures'.
        FeaturesBinary
        % PointsSingle holds Points elements. This is initialized in
        % codegen mode if class of Points is 'single' or any other
        % supported object.
        PointsSingle
        % PointsDouble holds Points elements. This is initialized in
        % codegen mode if class of Points is 'double'
        PointsDouble
        % ViewId A view identifier for the view
        ViewId

        % Properties used for Connections structure
        % RaffinePoseSingle holds simtform3d objects.This is initialized in
        % codegen mode if class of T matrix is 'single'.
        RaffinePoseSingle
        % RaffinePoseDouble holds simtform3d objects.This is initialized in
        % codegen mode if class of matrix is 'double'.
        RaffinePoseDouble
        % RrigidPoseSingle holds rigidtform3d objects.This is initialized in
        % codegen mode if class of Rotation or T matrix is 'single'.
        RrigidPoseSingle
        % RrigidPoseDouble holds rigidtform3d objects.This is initialized in
        % codegen mode if class of Rotation or T matrix is 'double'.
        RrigidPoseDouble
        % ViewId1 A view identifier for the first view of connection.
        ViewId1
        % ViewId2 A view identifier for the second view of connection.
        ViewId2
        % MatchesSingle holds Matches.This is initialized in codegen mode
        % if class of Matches is 'single'.
        MatchesSingle
        % MatchesSingle holds Matches.This is initialized in codegen mode
        % if class of Matches is 'double'.
        MatchesDouble
        % MatchesSingle holds Matches.This is initialized in codegen mode
        % if class of Matches is 'uint32'.
        MatchesInt
        % InfoMatSingle Information matrix for the measurement.This is
        % initialized in codegen mode if class of matrix is 'single'
        InfoMatSingle
        % InfoMatDouble Information matrix for the measurement.This is
        % initialized in codegen mode if class of matrix is 'double'
        InfoMatDouble

        % Properties used for Points properties in case when the Points are
        % not numeric
        % Metrics stores the Metrics property of Points object
        Metrics
        % Scale stores the Scale property of Points object
        Scale
        % Orientation stores the Orientation property of Points object
        Orientation
        % Laplacian stores the SignOfLaplacian property of Points object
        Laplacian
        % PixelList stores the PixelList property of Points object
        PixelList
        % PixelLengths stores the PixelLengths property of Points object
        PixelLengths
    end

    properties (Hidden,GetAccess = public, SetAccess = protected)
        % AbsPoseClass holds the class of Rotation and T of rigidtform3d.
        % This is initialized in codegen mode for Views.
        AbsPoseClass
        % PointsClass holds the class of Points.
        % This is initialized in codegen mode for Views.
        PointsClass
        % FeaturesClass holds the class of Features.
        % This is initialized in codegen mode for Views.
        FeaturesClass
        % RelPoseClass holds the class of RelativePose (simtform3d or rigidtform3d).
        % This is initialized in codegen mode for Connections.
        RelClass
        % RelClass holds the class of Rotation and T of rigidtform3d.
        % This is initialized in codegen mode for Connections.
        RelPoseClass
        % InfoMatClass holds the class of information matrix.
        % This is initialized in codegen mode for Connections.
        InfoMatClass
        % MatchesClass holds the class of Matches.
        % This is initialized in codegen mode for Connections.
        MatchesClass
    end

    properties (Abstract, Access = protected, Constant)
        Version

        ExtraViewVariables

        ExtraConnectionVariables
    end

    methods(Static)
        function props = matlabCodegenNontunableProperties(~)
        % Used for code generation
            props = {'AbsPoseClass', 'PointsClass', 'FeaturesClass', ...
                     'InfoMatClass', 'RelClass', 'RelPoseClass', 'MatchesClass'};
        end
    end

    methods
        %------------------------------------------------------------------
        % Constructor
        %------------------------------------------------------------------
        function this = imageviewsetBase()
        % Initializing the variables
            [viewID, poseSingle, poseDouble, fDouble, fSingle, ...
             fBinary, pSingle, pDouble, rPose, rPoseSingle, ...
             mSingle, mDouble, mInt, infMatSingle, infMatDouble] = ...
             vision.internal.codegen.imageviewset.imageviewsetBase.initializeViewsetData();

            % Assigning the imageviewset variables related to Views.
            this.ViewId = viewID;
            this.AbsPosesSingle = poseSingle;
            this.AbsPosesDouble = poseDouble;
            this.FeaturesSingle = fSingle;
            this.FeaturesDouble = fDouble;
            this.FeaturesBinary = fBinary;
            this.PointsSingle = pSingle;
            this.PointsDouble = pDouble;

            % Assigning the imageviewset variables related to Connections.
            this.ViewId1 = viewID;
            this.ViewId2 = viewID;
            this.RrigidPoseDouble = poseDouble;
            this.RrigidPoseSingle = poseSingle;
            this.RaffinePoseDouble = rPose;
            this.RaffinePoseSingle = rPoseSingle;
            this.MatchesDouble = mDouble;
            this.MatchesSingle = mSingle;
            this.MatchesInt = mInt;
            this.InfoMatDouble = infMatDouble;
            this.InfoMatSingle = infMatSingle;

            [metrics, scale, orientation, lapalacian, pList, pLengths] = ...
                vision.internal.codegen.imageviewset.imageviewsetBase.initializePointAttributes();
            % Assigning the imageviewset variables related to Points.
            this.Metrics = metrics;
            this.Scale = scale;
            this.Orientation = orientation;
            this.Laplacian = lapalacian;
            this.PixelList = pList;
            this.PixelLengths = pLengths;

            % Assigning the imageviewset variables related to FeatureGraph.
            [gSingle, gDouble] = vision.internal.codegen.imageviewset.imageviewsetBase.initializeFeatureGraph();
            this.FeatureGraphSingle = gSingle;
            this.FeatureGraphDouble = gDouble;

        end

        %------------------------------------------------------------------
        % Get the data from dependent property-Views
        %------------------------------------------------------------------
        function views = get.Views(this)

        % Get the AbsolutePoses based on value of AbsPoseClass
            if coder.internal.prop_has_class(this, 'AbsPoseClass')
                if strcmp(this.AbsPoseClass, 'single')
                    abPose = this.AbsPosesSingle;
                else
                    abPose = this.AbsPosesDouble;
                end
            else
                abPose = this.AbsPosesDouble;
            end

            % Get the Points based on value of PointsClass
            if coder.internal.prop_has_class(this, 'PointsClass')
                n = numel(this.ViewId);
                if n==0
                    % If the viewIds is empty, return empty cell for Points
                    points = cell(0, 1);
                else
                    if strcmp(this.PointsClass, 'double')
                        points = this.PointsDouble;
                    else
                        c = cell(n, 1);
                        % If the PointsClass is neither single nor double
                        % Create the Points objects based on PointsClass and
                        % return this cell for Points
                        pts = this.PointsSingle;
                        metric = this.Metrics;
                        scale = this.Scale;
                        orientation = this.Orientation;
                        if strcmp(this.PointsClass, 'cornerPoints')
                            parfor i=1:n
                                c{i} = cornerPoints(pts{i}, 'Metric', metric{i});
                            end
                        elseif strcmp(this.PointsClass, 'BRISKPoints')
                            parfor i=1:n
                                c{i} = BRISKPoints(pts{i}, 'Metric', metric{i}, ...
                                                   'Scale', scale{i}, 'Orientation', orientation{i});
                            end
                        elseif strcmp(this.PointsClass, 'SURFPoints')
                            laplace = this.Laplacian;
                            parfor i=1:n
                                c{i} = SURFPoints(pts{i}, 'Metric', metric{i}, ...
                                                  'Scale', scale{i}, 'Orientation', orientation{i}, ...
                                                  'SignOfLaplacian', laplace{i});
                            end
                        elseif strcmp(this.PointsClass, 'ORBPoints')
                            parfor i=1:n
                                c{i} = ORBPoints(pts{i}, 'Metric', metric{i}, ...
                                                 'Scale', scale{i}, 'Orientation', orientation{i});
                            end
                        elseif strcmp(this.PointsClass, 'KAZEPoints')
                            parfor i=1:n
                                c{i} = KAZEPoints(pts{i}, 'Metric', metric{i}, ...
                                                  'Scale', scale{i}, 'Orientation', orientation{i});
                            end
                        elseif strcmp(this.PointsClass, 'MSERRegions')
                            pList = this.PixelList;
                            pLength = this.PixelLengths;
                            parfor i=1:n
                                c{i} = MSERRegions(pList{i}, pLength{i});
                            end
                        else
                            c = this.PointsSingle;
                        end
                        points = c;
                    end
                end
            else
                % Return empty cell for points when Views are not
                % initialized
                points = cell(0, 1);
            end

            % Get the Features based on value of FeaturesClass
            if coder.internal.prop_has_class(this, 'FeaturesClass')
                n = numel(this.ViewId);
                if n==0
                    % Return empty cell as features when viewIds is empty
                    features = cell(0, 1);
                else
                    if strcmp(this.FeaturesClass, 'single')
                        features = this.FeaturesSingle;
                    elseif strcmp(this.FeaturesClass, 'double')
                        features = this.FeaturesDouble;
                    elseif strcmp(this.FeaturesClass, 'uint8')
                        features = this.FeaturesBinary;
                    else
                        % Create the binaryFeatures
                        c = cell(n, 1);
                        binFeatures = this.FeaturesBinary;
                        parfor i=1:n
                            c{i} = binaryFeatures(binFeatures{i});
                        end
                        features = c;
                    end
                end
            else
                % Return empty cell as features when Views are not
                % initialized
                features = cell(0, 1);
            end

            % Create the Views structure
            views = struct('ViewId', this.ViewId, 'AbsolutePose', abPose);
            views.Features = features;
            views.Points = points;
        end

        %------------------------------------------------------------------
        % Get the data from dependent property-Connections
        %------------------------------------------------------------------
        function Connections = get.Connections(this)
        % Get the RelativePose based on the value of RelClass and
        % RelPoseClass
            if coder.internal.prop_has_class(this, 'RelClass')
                if strcmp(this.RelClass, 'simtform3d') ...
                        || strcmp(this.RelClass, 'affine3d')
                    if strcmp(this.RelPoseClass, 'single')
                        relPose = this.RaffinePoseSingle;
                    else
                        relPose = this.RaffinePoseDouble;
                    end
                    nPoses = numel(relPose);
                    rPose = cell(nPoses, 1);
                    for i=1:nPoses
                        rPose{i} = simtform3d(relPose(i).T');
                    end
                else
                    if strcmp(this.RelPoseClass, 'single')
                        relPose = this.RrigidPoseSingle;
                    else
                        relPose = this.RrigidPoseDouble;
                    end
                    nPoses = numel(relPose);
                    rPose = cell(nPoses, 1);
                    for i=1:nPoses
                        rPose{i} = rigidtform3d(relPose(i).T');
                    end
                end
            else
                rPose = cell(0, 1);
            end

            % Get the Information Matrix based on the value of InfoMatClass
            if coder.internal.prop_has_class(this, 'InfoMatClass')
                if strcmp(this.InfoMatClass, 'single')
                    infoMat = this.InfoMatSingle;
                else
                    infoMat = this.InfoMatDouble;
                end
                % Return empty cell as Information Matrix when ViewId1 is
                % empty
                if (numel(this.ViewId1)==0) && isempty(infoMat{1})
                    infoMat = cell(0, 1);
                end
            else
                % Return empty cell as Information Matrix when Connections
                % are not initialized
                infoMat = cell(0, 1);
            end

            % Get the Matches based on the value of MatchesClass
            if coder.internal.prop_has_class(this, 'MatchesClass')
                if strcmp(this.MatchesClass, 'single')
                    matches = this.MatchesSingle;
                elseif strcmp(this.MatchesClass, 'double')
                    matches = this.MatchesDouble;
                else
                    matches = this.MatchesInt;
                end
                % Return empty cell as Matches when ViewId1 is empty
                if (numel(this.ViewId1)==0) && isempty(matches{1})
                    matches = cell(0, 1);
                end
            else
                % Return empty cell as Matches when Connections are not
                % initialized
                matches = cell(0, 1);
            end

            % Create the Connections struct
            Connections = struct('ViewId1', this.ViewId1, 'ViewId2', this.ViewId2);
            Connections.RelativePose = rPose;
            Connections.InformationMatrix = infoMat;
            Connections.Matches = matches;
        end

        function featureGraph = get.FeatureGraph(this)
        % Get the FeatureGraph based on the value of PointsClass
            if coder.internal.prop_has_class(this, 'PointsClass')
                if strcmp(this.PointsClass, 'double')
                    featureGraph = this.FeatureGraphDouble;
                else
                    featureGraph = this.FeatureGraphSingle;
                end
            else
                featureGraph = this.FeatureGraphDouble;
            end
        end

        function this = set.FeatureGraph(this, value)
        % Set the FeatureGraph based on the value of PointsClass
            if coder.internal.prop_has_class(this, 'PointsClass')
                if strcmp(this.PointsClass, 'double')
                    this.FeatureGraphDouble = value;
                else
                    this.FeatureGraphSingle = value;
                end
            else
                this.FeatureGraphDouble = value;
            end
        end

        %------------------------------------------------------------------
        % Get the NumViews
        %------------------------------------------------------------------
        function numViews = get.NumViews(this)
            if(isempty(this.ViewId))
                numViews = 0;
            else
                numViews = numel(this.ViewId);
            end
        end

        %------------------------------------------------------------------
        % Get the NumConnections
        %------------------------------------------------------------------
        function numConns = get.NumConnections(this)
            if(isempty(this.ViewId1))
                numConns = 0;
            else
                numConns = numel(this.ViewId1);
            end
        end

        %------------------------------------------------------------------
        % Get the views connected to the view viewId.
        %------------------------------------------------------------------
        function [viewTable, connIdx] = connectedViews(this, viewId)

        % Initialize the viewTable
            viewTable = struct('ViewId', zeros(coder.ignoreConst(0), 1,'uint32'), ...
                'AbsolutePose', rigidtform3d(eye(4, 4, this.AbsPoseClass)));
            viewTable.Features = {};
            viewTable.Points = {};
            viewId = checkViewId(this, viewId);

            viewId1 = this.ViewId1;
            viewId2 = this.ViewId2;
            fromViewId = viewId1==viewId;
            toViewId   = viewId2==viewId;

            connectedViewIds = [...
                viewId2(fromViewId); ...
                viewId1(toViewId)];

            % Find linear index into view table
            viewIdx = coder.nullcopy(zeros(0, 1));
            vIds = this.ViewId;
            parfor i=1:numel(this.ViewId)
                for j = 1:numel(connectedViewIds)
                    if vIds(i) == connectedViewIds(j)
                        viewIdx = [viewIdx;i];
                    end
                end
            end
            % [~, viewIdx] = intersect(this.ViewId, connectedViewIds, 'stable');
            if ~isempty(viewIdx)
                views = this.Views;
                viewId = views.ViewId(viewIdx);
                viewTable = struct('ViewId', viewId, ...
                                   'AbsolutePose',views.AbsolutePose(viewIdx));
                n = numel(viewId);
                features = cell(n, 1);
                points = cell(n, 1);
                featuresAll = views.Features;
                pointsAll = views.Points;
                for i = 1:n
                    features{i} = featuresAll{viewIdx(i)};
                    points{i} = pointsAll{viewIdx(i)};
                end
                viewTable.Features = features;
                viewTable.Points = points;
            end

            connIdx = fromViewId | toViewId;
        end

        %------------------------------------------------------------------
        % Returns a pose graph
        %------------------------------------------------------------------
        function G = createPoseGraph(this, weight)

            endNodes = [this.ViewId1 this.ViewId2];
            if ~coder.internal.prop_has_class(this, 'RelClass')
                % When the number of connections are zero edgeTable should
                % be empty. Retaining only endNodes as table cannot be
                % created with empty cell arrays in codegen.
                edgeTable = table(endNodes, ...
                    'VariableNames', {'EndNodes'});
            else
                connections = this.Connections;
                relPose = connections.RelativePose;
                infoMat = connections.InformationMatrix;
                if(nargin > 1)
                    % Remove weight if number of connections is zero
                    if this.NumConnections==0
                        weight = [];
                    end
                    edgeTable = table(endNodes, relPose, infoMat, weight, ...
                        'VariableNames', {'EndNodes', ...
                        'RelativePose', 'InformationMatrix', 'Weight'});
                else
                    edgeTable = table(endNodes, relPose, infoMat, ...
                        'VariableNames', {'EndNodes', ...
                        'RelativePose', 'InformationMatrix'});
                end
            end

            numViews = numel(this.ViewId);
            absPoseCell = coder.nullcopy(cell(numViews, 1));

            if coder.internal.prop_has_class(this, 'AbsPoseClass')
                % Get the AbsolutePose based on the value of AbsPoseClass
                if strcmp(this.AbsPoseClass, 'double')
                    absPose = this.AbsPosesDouble;
                    for i=1:numViews
                        absPoseCell{i} = absPose(i);
                    end
                else
                    absPose = this.AbsPosesSingle;
                    for i=1:numViews
                        absPoseCell{i} = absPose(i);
                    end
                end
            else
                absPose = this.AbsPosesDouble;
                for i=1:numViews
                    absPoseCell{i} = absPose(i);
                end
            end
            nodeTable = table(this.ViewId, absPoseCell, ...
                              'VariableNames', {'ViewId', 'AbsolutePose'});
            G = digraph(edgeTable, nodeTable);
        end

        %------------------------------------------------------------------
        % Find views associated with viewIds
        %------------------------------------------------------------------
        function views = findView(this, viewIds)
            validateattributes(viewIds, {'numeric'}, {'nonsparse', 'vector', ...
                                                      'integer', 'positive', 'real'}, 'findView', 'viewIds');

            checkIfViewIsMissing(this, viewIds);
            % Getting the num views
            numViews = numel(viewIds);
            % Getting the index of the 1 view
            viewIdx = getViewIndex(this, viewIds(1));
            vId = coder.nullcopy(zeros(numViews, 1, 'uint32'));
            % Assigning the first viewId
            vIds = this.ViewId;
            vId(1) = vIds(viewIdx);
            features = coder.nullcopy(cell(numViews, 1));
            points = coder.nullcopy(cell(numViews, 1));
            % Assigning the first pose
            views = this.Views;
            absPose = views.AbsolutePose;
            featuresFull = views.Features;
            pointsFull = views.Points;
            aPoses = absPose(viewIdx);
            features{1} = featuresFull{viewIdx};
            points{1} = pointsFull{viewIdx};

            if (numViews > 1)
                % Looping over the remaining views
                for i = 2:numViews
                    viewIdx = getViewIndex(this, viewIds(i));
                    vId(i) = vIds(viewIdx);                    
                    features{i} = featuresFull{viewIdx};
                    points{i} = pointsFull{viewIdx};
                    aPoses(i) = absPose(viewIdx);
                end
            end
            views = struct('ViewId', vId, 'AbsolutePose', aPoses);
            views.Features = features;
            views.Points = points;
        end

        %------------------------------------------------------------------
        % Find connections associated with view IDs
        %------------------------------------------------------------------
        function conn = findConnection(this, viewIds1, viewIds2)
            validateattributes(viewIds1, {'numeric'}, {'nonsparse', 'vector', ...
                                                       'integer', 'positive', 'real'}, 'findConnection', 'viewIds1');
            validateattributes(viewIds2, {'numeric'}, {'nonsparse', 'vector', ...
                                                       'integer', 'positive', 'real', 'size', size(viewIds1)}, ...
                               'findConnection', 'viewIds2');
            checkIfViewIsMissing(this, [viewIds1(:); viewIds2(:)]);
            numConnections = numel(viewIds1);
            vId1 = zeros(numConnections, 1, 'uint32');
            vId2 = zeros(numConnections, 1, 'uint32');
            relPose = coder.nullcopy(cell(numConnections, 1));
            infoMat = coder.nullcopy(cell(numConnections, 1));
            matches = coder.nullcopy(cell(numConnections, 1));
            vIds1 = this.ViewId1;
            vIds2 = this.ViewId2;
            connections = this.Connections;
            rPose = connections.RelativePose;
            iMat = connections.InformationMatrix;
            match = connections.Matches;
            for i = 1:numConnections
                checkIfConnectionIsMissing(this, viewIds1(i), viewIds2(i))
                idx = getConnectionIndex(this, viewIds1(i), viewIds2(i));
                % Assigning the viewIds
                vId1(i) = vIds1(idx);
                vId2(i) = vIds2(idx);
                relPose{i} = rPose{idx};
                infoMat{i} = iMat{idx};
                matches{i} = match{idx};
            end
            conn = struct('ViewId1', vId1, 'ViewId2', vId2);
            conn.RelativePose = relPose;
            conn.InformationMatrix = infoMat;
            conn.Matches = matches;
        end
    end

    methods (Access = protected)

        %------------------------------------------------------------------
        % Get the connections values
        %------------------------------------------------------------------
        function conn = getConnections(this, viewId)
            viewIds = this.ViewId1;
            conn = this.Connections(viewIds == viewId, :)';
        end

        %------------------------------------------------------------------
        % Get the Connection at given Connection Index
        %------------------------------------------------------------------
        function [vId1, vId2, rPose, iMats, match] = getConnection(this, connId)
            vId1 = this.ViewId1(connId);
            vId2 = this.ViewId2(connId);
            if strcmp(this.RelClass, 'simtform3d') ...
                    || strcmp(this.RelClass, 'affine3d')
                if strcmp(this.RelPoseClass, 'single')
                    rPose = {simtform3d(this.RaffinePoseSingle(connId).T')};
                else
                    rPose = {simtform3d(this.RaffinePoseDouble(connId).T')};
                end
            else
                if strcmp(this.RelPoseClass, 'single')
                    rPose = {rigidtform3d(this.RrigidPoseSingle(connId).T')};
                else
                    rPose = {rigidtform3d(this.RrigidPoseDouble(connId).T')};
                end
            end
            if strcmp(this.InfoMatClass, 'single')
                iMats = {this.InfoMatSingle{connId}};
            else
                iMats = {this.InfoMatDouble{connId}};
            end
            if strcmp(this.MatchesClass, 'single')
                match = {this.MatchesSingle{connId}};
            elseif strcmp(this.MatchesClass, 'double')
                match = {this.MatchesDouble{connId}};
            else
                match = {this.MatchesInt{connId}};
            end
        end

        %------------------------------------------------------------------
        % Add new Features to the corresponding features cells
        %------------------------------------------------------------------
        function fUpdated = getUpdatedFeatures(this, features)

        % Number of existing Features
            oldFeatures = numel(this.ViewId);
            % Number of new Features to be added
            newFeatures = numel(features);
            % Total number of updated Features
            fSize = oldFeatures+newFeatures;
            fUpdated = coder.nullcopy(cell(fSize, 1));
            featuresSingle = this.FeaturesSingle;
            featuresDouble = this.FeaturesDouble;
            featuresBinary = this.FeaturesBinary;
            % Store the old Features first and then append the new Features
            for i = 1:(oldFeatures+1)
                if i > oldFeatures
                    for k = 1:newFeatures
                        if strcmp(this.FeaturesClass, 'binaryFeatures')
                            fUpdated{i+k-1} = features{k}.Features;
                        else
                            fUpdated{i+k-1} = features{k};
                        end
                    end
                else
                    if strcmp(this.FeaturesClass, 'single')
                        fUpdated{i} = featuresSingle{i};
                    elseif strcmp(this.FeaturesClass, 'double')
                        fUpdated{i} = featuresDouble{i};
                    else
                        fUpdated{i} = featuresBinary{i};
                    end
                end
            end
        end

        %------------------------------------------------------------------
        % Add the new Points to the corresponding Points cell
        %------------------------------------------------------------------
        function pUpdated = getUpdatedPoints(this, points)
        % Number of existing Points
            oldPoints = numel(this.ViewId);
            % Number of new Points to be added
            newPoints = numel(points);
            % Total number of updated Points
            pSize = oldPoints+newPoints;
            pUpdated = coder.nullcopy(cell(pSize, 1));
            pointsSingle = this.PointsSingle;
            pointsDouble = this.PointsDouble;
            % Store the old Points first and then append the new Points
            for i = 1:(oldPoints+1)
                if i > oldPoints
                    for k = 1:newPoints
                        if strcmp(this.PointsClass, 'double')
                            pUpdated{i+k-1} = points{k};
                        else
                            if strcmp(this.PointsClass, 'single')
                                pUpdated{i+k-1} = points{k};
                            else
                                pUpdated{i+k-1} = points{k}.Location;
                            end
                        end
                    end
                else
                    if strcmp(this.PointsClass, 'double')
                        pUpdated{i} = pointsDouble{i};
                    else
                        pUpdated{i} = pointsSingle{i};
                    end
                end
            end
        end

        %------------------------------------------------------------------
        % Add the new Information Matrix to the corresponding InfoMat cell
        %------------------------------------------------------------------
        function infoMatUpdated = getUpdatedInfoMat(this, connTable)

            infoMatSize = numel(this.ViewId1);
            infoMatUpdated = coder.nullcopy(cell(infoMatSize, 1));
            infoMatSingle = this.InfoMatSingle;
            infoMatDouble = this.InfoMatDouble;
            for i = 1:infoMatSize
                if i == infoMatSize
                    infoMatUpdated{i} = connTable.InformationMatrix{1};
                else
                    if strcmp(this.InfoMatClass, 'single')
                        infoMatUpdated{i} = infoMatSingle{i};
                    else
                        infoMatUpdated{i} = infoMatDouble{i};
                    end
                end
            end
        end

        %------------------------------------------------------------------
        % Add the new Matches to the corresponding Matches cell
        %------------------------------------------------------------------
        function mUpdated = getUpdatedMatches(this, connTable)

            mSize = numel(this.ViewId1);
            mUpdated = coder.nullcopy(cell(mSize, 1));
            matchesSingle = this.MatchesSingle;
            matchesDouble = this.MatchesDouble;
            matchesInt = this.MatchesInt;
            for i = 1:mSize
                if i == mSize
                    mUpdated{i} = connTable.Matches{1};
                else
                    if strcmp(this.MatchesClass, 'single')
                        mUpdated{i} = matchesSingle{i};
                    elseif strcmp(this.MatchesClass, 'double')
                        mUpdated{i} = matchesDouble{i};
                    else
                        mUpdated{i} = matchesInt{i};
                    end
                end
            end
        end

        %------------------------------------------------------------------
        % Update the existing Features with new values
        %------------------------------------------------------------------
        function this = updateFeatures(this, indicesViewsLeft, fSize)
            coder.varsize('featuresUpdated');
            featuresUpdated = coder.nullcopy(cell(1, fSize));
            if strcmp(this.FeaturesClass, 'single')
                features = this.FeaturesSingle;
                for i = 1:fSize
                    featuresUpdated{i} = features{indicesViewsLeft(i)};
                end
                this.FeaturesSingle = featuresUpdated;
            elseif strcmp(this.FeaturesClass, 'double')
                features = this.FeaturesDouble;
                for i = 1:fSize
                    featuresUpdated{i} = features{indicesViewsLeft(i)};
                end
                this.FeaturesDouble = featuresUpdated;
            else
                features = this.FeaturesBinary;
                for i = 1:fSize
                    featuresUpdated{i} = features{indicesViewsLeft(i)};
                end
                this.FeaturesBinary = featuresUpdated;
            end
        end

        %------------------------------------------------------------------
        % Update the existing Points with new values
        %------------------------------------------------------------------
        function this = updatePoints(this, indicesViewsLeft, pSize)
            coder.varsize('pointsUpdated');
            pointsUpdated = coder.nullcopy(cell(1, pSize));
            if strcmp(this.PointsClass, 'double')
                points = this.PointsDouble;
                for i = 1:pSize
                    pointsUpdated{i} = points{indicesViewsLeft(i)};
                end
                this.PointsDouble = pointsUpdated;
            else
                points = this.PointsSingle;
                for i = 1:pSize
                    pointsUpdated{i} = points{indicesViewsLeft(i)};
                end
                this.PointsSingle = pointsUpdated;
                this = this.delProps(indicesViewsLeft);
            end
        end

        %------------------------------------------------------------------
        % Update the existing Information Matrix with new values
        %------------------------------------------------------------------
        function this = updateInfoMat(this, indicesConnsLeft)
            coder.varsize('iUpdated');
            iSize = numel(this.ViewId1);
            iUpdated = coder.nullcopy(cell(iSize, 1));
            if strcmp(this.InfoMatClass, 'single')
                infoMat = this.InfoMatSingle;
                for i = 1:iSize
                    iUpdated{i} = infoMat{indicesConnsLeft(i)};
                end
                this.InfoMatSingle = iUpdated;
            else
                infoMat = this.InfoMatDouble;
                for i = 1:iSize
                    iUpdated{i} = infoMat{indicesConnsLeft(i)};
                end
                this.InfoMatDouble = iUpdated;
            end
        end

        %------------------------------------------------------------------
        % Update the existing Matches with new values
        %------------------------------------------------------------------
        function this = updateMatches(this, indicesConnsLeft)
            coder.varsize('mUpdated');
            mSize = numel(this.ViewId1);
            mUpdated = coder.nullcopy(cell(mSize, 1));
            if strcmp(this.MatchesClass, 'single')
                matches = this.MatchesSingle;
                for i = 1:mSize
                    mUpdated{i} = matches{indicesConnsLeft(i)};
                end
                this.MatchesSingle = mUpdated;
            elseif strcmp(this.MatchesClass, 'double')
                matches = this.MatchesDouble;
                for i = 1:mSize
                    mUpdated{i} = matches{indicesConnsLeft(i)};
                end
                this.MatchesDouble = mUpdated;
            else
                matches = this.MatchesInt;
                for i = 1:mSize
                    mUpdated{i} = matches{indicesConnsLeft(i)};
                end
                this.MatchesInt = mUpdated;
            end
        end

        %------------------------------------------------------------------
        % Store the properties of Points Objects
        %------------------------------------------------------------------
        function this = addProps(this, points)
            if strcmp(this.PointsClass, 'cornerPoints')
                this = this.updateMetrics(points);
            elseif strcmp(this.PointsClass, 'BRISKPoints')
                this = this.updateMetrics(points);
                this = this.updateScale(points);
                this = this.updateOrientation(points);
            elseif strcmp(this.PointsClass, 'SURFPoints')
                this = this.updateMetrics(points);
                this = this.updateScale(points);
                this = this.updateOrientation(points);
                % Store the additional property SignOfLaplacian
                oldLaplacian = numel(this.ViewId);
                newLaplacian = numel(points);
                lSize = oldLaplacian+newLaplacian;
                uLapalcian = coder.nullcopy(cell(lSize, 1));
                laplacian = this.Laplacian;
                for i=1:(oldLaplacian+1)
                    if i > oldLaplacian
                        for k=1:newLaplacian
                            uLapalcian{i+k-1} = points{k}.SignOfLaplacian;
                        end
                    else
                        uLapalcian{i} = laplacian{i};
                    end
                end
                this.Laplacian = uLapalcian;
            elseif strcmp(this.PointsClass, 'ORBPoints')
                this = this.updateMetrics(points);
                this = this.updateScale(points);
                this = this.updateOrientation(points);
            elseif strcmp(this.PointsClass, 'KAZEPoints')
                this = this.updateMetrics(points);
                this = this.updateScale(scale);
                this = this.updateOrientation(points);
            else
                if strcmp(this.PointsClass, 'MSERRegions')
                    % Store the additional properties PixelList, PixelLengths
                    oldPixel = numel(this.ViewId);
                    newPixel = numel(points);
                    pSize = oldPixel+newPixel;
                    uPixelList = coder.nullcopy(cell(pSize, 1));
                    uPixelLengths = coder.nullcopy(cell(pSize, 1));
                    pList = this.PixelList;
                    pLength = this.PixelLengths;
                    for i=1:(oldPixel+1)
                        if i>oldPixel
                            for k=1:newPixel
                                uPixelList{i+k-1} = points{k}.PixelList;
                                uPixelLengths{i+k-1} = points{k}.Lengths;
                            end
                        else
                            uPixelList{i} = pList{i};
                            uPixelLengths{i} = pLength{i};
                        end
                    end
                    this.PixelList = uPixelList;
                    this.PixelLengths = uPixelLengths;
                end
            end
        end

        %------------------------------------------------------------------
        % Update the properties of Points Objects
        %------------------------------------------------------------------
        function this = updateProps(this, pObject, viewIdx)
            if strcmp(this.PointsClass, 'cornerPoints')
                this.Metrics{viewIdx} = pObject.Metric;
            elseif strcmp(this.PointsClass, 'BRISKPoints')
                this.Metrics{viewIdx} = pObject.Metric;
                this.Scale{viewIdx} = pObject.Scale;
                this.Orientation{viewIdx} = pObject.Orientation;
            elseif strcmp(this.PointsClass, 'SURFPoints')
                this.Metrics{viewIdx} = pObject.Metric;
                this.Scale{viewIdx} = pObject.Scale;
                this.Orientation{viewIdx} = pObject.Orientation;
                this.Laplacian{viewIdx} = pObject.SignOfLaplacian;
            elseif strcmp(this.PointsClass, 'ORBPoints')
                this.Metrics{viewIdx} = pObject.Metric;
                this.Scale{viewIdx} = pObject.Scale;
                this.Orientation{viewIdx} = pObject.Orientation;
            elseif strcmp(this.PointsClass, 'KAZEPoints')
                this.Metrics{viewIdx} = pObject.Metric;
                this.Scale{viewIdx} = pObject.Scale;
                this.Orientation{viewIdx} = pObject.Orientation;
            else
                if strcmp(this.PointsClass, 'MSERRegions')
                    this.PixelList{viewIdx} = pObject.PixelList;
                    this.PixelLength{viewIdx} = pObject.Lengths;
                end
            end
        end

        %------------------------------------------------------------------
        % Delete the properties of Points Objects
        %------------------------------------------------------------------
        function this = delProps(this, indicesViewsLeft)
            if strcmp(this.PointsClass, 'cornerPoints')
                this.Metrics = this.getUpdatedMetrics(indicesViewsLeft);
            elseif strcmp(this.PointsClass, 'BRISKPoints')
                this.Metrics = this.getUpdatedMetrics(indicesViewsLeft);
                this.Scale = this.getUpdatedScale(indicesViewsLeft);
                this.Orientation = this.getUpdatedOrientation(indicesViewsLeft);
            elseif strcmp(this.PointsClass, 'SURFPoints')
                this.Metrics = this.getUpdatedMetrics(indicesViewsLeft);
                this.Scale = this.getUpdatedScale(indicesViewsLeft);
                this.Orientation = this.getUpdatedOrientation(indicesViewsLeft);
                this.Laplacian = this.getUpdatedLaplacian(indicesViewsLeft);
            elseif strcmp(this.PointsClass, 'ORBPoints')
                this.Metrics = this.getUpdatedMetrics(indicesViewsLeft);
                this.Scale = this.getUpdatedScale(indicesViewsLeft);
                this.Orientation = this.getUpdatedOrientation(indicesViewsLeft);
            elseif strcmp(this.PointsClass, 'KAZEPoints')
                this.Metrics = this.getUpdatedMetrics(indicesViewsLeft);
                this.Scale = this.getUpdatedScale(indicesViewsLeft);
                this.Orientation = this.getUpdatedOrientation(indicesViewsLeft);
            else
                if strcmp(this.PointsClass, 'MSERRegions')
                    this.PixelList = this.getUpdatedPList(indicesViewsLeft);
                    this.PixelLength = this.getUpdatedPLength(indicesViewsLeft);
                end
            end
        end

        %------------------------------------------------------------------
        % Add the new Metrics to the existing Metrics cell
        %------------------------------------------------------------------
        function this = updateMetrics(this, points)
        % Number of existing Metrics elements
            oldMetrics = numel(this.ViewId);
            % Number of metrics to be appended
            newMetrics = numel(points);
            % New number of Metrics
            mSize = oldMetrics+newMetrics;
            uMetric = coder.nullcopy(cell(mSize, 1));
            metrics = this.Metrics;
            for i=1:(oldMetrics+1)
                if i>oldMetrics
                    for k=1:newMetrics
                        uMetric{i+k-1} = points{k}.Metric;
                    end
                else
                    uMetric{i} = metrics{i};
                end
            end
            this.Metrics = uMetric;
        end

        %------------------------------------------------------------------
        % Add the new Scale to the existing Scales cell
        %------------------------------------------------------------------
        function this = updateScale(this, points)
        % Number of existing Scale elements
            oldScale = numel(this.ViewId);
            % Number of Scale to be appended
            newScale = numel(points);
            % New number of Scale
            uSize = oldScale+newScale;
            uScale = coder.nullcopy(cell(uSize, 1));
            scale = this.Scale;
            for i=1:(oldScale+1)
                if i>oldScale
                    for k=1:newScale
                        uScale{i+k-1} = points{k}.Scale;
                    end
                else
                    uScale{i} = scale{i};
                end
            end
            this.Scale = uScale;
        end

        %------------------------------------------------------------------
        % Add the new Orientation to the existing Orientation cell
        %------------------------------------------------------------------
        function this = updateOrientation(this, points)
        % Number of existing Orientation elements
            oldOrient = numel(this.ViewId);
            % Number of Orientation to be appended
            newOrient = numel(points);
            % New number of Orientation
            oSize = oldOrient+newOrient;
            uOrientation = coder.nullcopy(cell(oSize, 1));
            orientation = this.Orientation;
            for i=1:(oldOrient+1)
                if i>oldOrient
                    for k=1:newOrient
                        uOrientation{i+k-1} = points{k}.Orientation;
                    end
                else
                    uOrientation{i} = orientation{i};
                end
            end
            this.Orientation = uOrientation;
        end

        %------------------------------------------------------------------
        % Update the existing Metrics
        %------------------------------------------------------------------
        function metrics = getUpdatedMetrics(this, indicesViewsLeft)
            coder.varsize('metrics');
            mSize = numel(indicesViewsLeft);
            metrics = coder.nullcopy(cell(1, mSize));
            curMetrics = this.Metrics;
            if isempty(indicesViewsLeft)
                metrics = {zeros(coder.ignoreConst(0), 1, 'single')};
            else
                for i = 1:numel(indicesViewsLeft)
                    metrics{i} = curMetrics{indicesViewsLeft(i)};
                end
            end
        end

        %------------------------------------------------------------------
        % Update the existing Scale
        %------------------------------------------------------------------
        function scale = getUpdatedScale(this, indicesViewsLeft)
            coder.varsize('scale');
            scSize = numel(indicesViewsLeft);
            scale = coder.nullcopy(cell(1, scSize));
            curScales = this.Scale;
            if isempty(indicesViewsLeft)
                scale = {zeros(coder.ignoreConst(0), 1, 'single')};
            else
                for i = 1:scSize
                    scale{i} = curScales{indicesViewsLeft(i)};
                end
            end
        end

        %------------------------------------------------------------------
        % Update the existing Orientation
        %------------------------------------------------------------------
        function orient = getUpdatedOrientation(this, indicesViewsLeft)
            coder.varsize('orient');
            oSize = numel(indicesViewsLeft);
            orient = coder.nullcopy(cell(1, oSize));
            orientation = this.Orientation;
            if isempty(indicesViewsLeft)
                orient = {zeros(coder.ignoreConst(0), 1, 'single')};
            else
                for i = 1:oSize
                    orient{i} = orientation{indicesViewsLeft(i)};
                end
            end
        end

        %------------------------------------------------------------------
        % Update the existing Laplacian
        %------------------------------------------------------------------
        function laplacian = getUpdatedLaplacian(this, indicesViewsLeft)
            coder.varsize('laplacian');
            lSize = numel(indicesViewsLeft);
            laplacian = coder.nullcopy(cell(1, lSize));
            curLaplacian = this.Laplacian;
            if isempty(indicesViewsLeft)
                laplacian = {zeros(coder.ignoreConst(0), 1, 'int8')};
            else
                for i = 1:lSize
                    laplacian{i} = curLaplacian{indicesViewsLeft(i)};
                end
            end
        end

        %------------------------------------------------------------------
        % Update the existing PixelList
        %------------------------------------------------------------------
        function pList = getUpdatedPList(this, indicesViewsLeft)
            coder.varsize('pList');
            pSize = numel(indicesViewsLeft);
            pList = coder.nullcopy(cell(1, pSize));
            pixelList = this.PixelList;
            if isempty(indicesViewsLeft)
                pList = {zeros(coder.ignoreConst(0), 2, 'int32')};
            else
                for i = 1:pSize
                    pList{i} = pixelList{indicesViewsLeft(i)};
                end
            end
        end

        %------------------------------------------------------------------
        % Update the existing PixelLength
        %------------------------------------------------------------------
        function pLength = getUpdatedPLength(this, indicesViewsLeft)
            coder.varsize('pList');
            pSize = numel(indicesViewsLeft);
            pLength = coder.nullcopy(cell(1, pSize));
            pixelLength = this.PixelLength;
            if isempty(indicesViewsLeft)
                pLength = {zeros(coder.ignoreConst(0), 2, 'int32')};
            else
                for i = 1:pSize
                    pLength{i} = pixelLength{indicesViewsLeft(i)};
                end
            end
        end
    end

    methods(Hidden, Static)

        %------------------------------------------------------------------
        % Initialize the viewset data
        %------------------------------------------------------------------
        function [vId, posesSingleArray, posesArray, fDouble, ...
                  fSingle, fbinary, pSingle, pDouble, rposesArray, ...
                  rposesSingleArray, mSingle, mDouble, mInt, infMatSingle, ...
                  infMatDouble] = initializeViewsetData()

            posesSingle = vision.internal.codegen.imageviewset.imageviewsetBase.makeEmptyRigid3d('single');
            coder.varsize('posesSingleArray');
            posesSingleArray = posesSingle;

            poses = vision.internal.codegen.imageviewset.imageviewsetBase.makeEmptyRigid3d('double');
            coder.varsize('posesArray');
            posesArray = poses;

            rposes = vision.internal.codegen.imageviewset.imageviewsetBase.makeEmptyAffine3d('double');
            coder.varsize('rposesArray');
            rposesArray = rposes;

            rposesSingle = vision.internal.codegen.imageviewset.imageviewsetBase.makeEmptyAffine3d('single');
            coder.varsize('rposesArraySingle');
            rposesSingleArray = rposesSingle;

            coder.varsize('fSingle');
            coder.varsize('fDouble');
            coder.varsize('fbinary');

            fSingle = {zeros(coder.ignoreConst(0), coder.ignoreConst(0), 'single')};
            fDouble = {zeros(coder.ignoreConst(0), coder.ignoreConst(0))};
            fbinary = {zeros(coder.ignoreConst(0), coder.ignoreConst(0), 'uint8')};

            coder.varsize('pSingle');
            coder.varsize('pDouble');

            pSingle = {zeros(coder.ignoreConst(0), coder.ignoreConst(2), 'single')};
            pDouble = {zeros(coder.ignoreConst(0), coder.ignoreConst(2))};

            coder.varsize('mSingle');
            coder.varsize('mDouble');
            coder.varsize('mInt');
            mSingle = {zeros(coder.ignoreConst(0), 2, 'single')};
            mDouble = {zeros(coder.ignoreConst(0), 2)};
            mInt = {zeros(coder.ignoreConst(0), 2, 'uint32')};

            coder.varsize('infMatSingle');
            coder.varsize('infMatDouble');
            infMatSingle = {zeros(coder.ignoreConst(0), coder.ignoreConst(0), 'single')};
            infMatDouble = {zeros(coder.ignoreConst(0), coder.ignoreConst(0))};

            vId = zeros(coder.ignoreConst(0), 1, 'uint32');


        end

        function [gSingle, gDouble] = initializeFeatureGraph()
            gSingle = vision.internal.ViewSetFeatureGraph('single');
            gDouble = vision.internal.ViewSetFeatureGraph('double');
        end

        %------------------------------------------------------------------
        % Initialize the Point Attributes
        %------------------------------------------------------------------
        function [metrics, scale, orientation, laplacian, pList, ...
                  pLengths] = initializePointAttributes()
            coder.varsize('m');
            m = {zeros(coder.ignoreConst(0), 1, 'single')};
            metrics = m;
            scale = m;
            orientation = m;
            coder.varsize('laplacian');
            coder.varsize('pList');
            coder.varsize('pLengths');
            laplacian = {zeros(coder.ignoreConst(0), 1, 'int8')};
            pList = {zeros(coder.ignoreConst(0), 2, 'int32')};
            pLengths = {zeros(coder.ignoreConst(0), 1, 'int32')};
        end

        %------------------------------------------------------------------
        % Make empty rigid3d object
        %------------------------------------------------------------------
        function obj = makeEmptyRigid3d(classType)
            rot = eye(3, classType);
            trans = zeros(1,3, classType);
            rObject = rigidtform3d(rot, trans);
            obj = repmat(rObject, 0, 0);
        end

        %------------------------------------------------------------------
        % Make empty simtform3d object
        %------------------------------------------------------------------
        function obj = makeEmptyAffine3d(classType)
            T = eye(4, classType);
            aObject = simtform3d(T);
            obj = repmat(aObject, 0, 0);
        end

        %------------------------------------------------------------------
        % Make empty pointTrack object
        %------------------------------------------------------------------
        function obj = makeEmptyPointTrack(classType)
            vIds = zeros(coder.ignoreConst(1), coder.ignoreConst(0), 'uint32');
            points = zeros(coder.ignoreConst(0), coder.ignoreConst(2), classType);
            fIndices = zeros(coder.ignoreConst(1), coder.ignoreConst(0), 'uint32');
            pObject = pointTrack(vIds, points, fIndices);
            obj = repmat(pObject, 0, 0);
        end

    end
end
