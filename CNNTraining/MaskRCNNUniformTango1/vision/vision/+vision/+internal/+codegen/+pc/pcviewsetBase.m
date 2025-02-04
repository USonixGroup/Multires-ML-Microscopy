classdef (Hidden)pcviewsetBase < vision.internal.ViewSetBaseImpl
    % pcviewsetBase class is the interface class for codegen
    % version of pcviewset

    % Copyright 2021 The MathWorks, Inc.
    % Notes
    %     1. pcviewset code generation implementation is different from 
    %     simulation. As pcviewset constructor is empty, need to accommodate
    %     all pointClouds and rigidtform3d types in the constructor for code generation.
    %     Properties PtCloudSingleOrg, PtCloudSingleUnOrg, PtCloudDoubleOrg, 
    %     PtCloudDoubleUnOrg ,AbsPosesDouble, AbsPosesSingle are used for Views
    %     related functions and similar properties are defined for Connections
    %     functions.
    %     
    %     2. Non Tunable properties AbsPoseClass, PtCloudClass are used to 
    %     maintain the homogeneous nature for code generation.

    %#codegen

    properties (Dependent, GetAccess = public, SetAccess = protected)
        % Views  A structure containing view attributes,
        % 'ViewId', 'AbsolutePose' and 'PointCloud'.
        Views

        % Connections A structure containing connection attributes,
        % 'ViewId1', 'ViewId2', 'RelativePose' and 'InformationMatrix'.
        Connections
    end

    properties (SetAccess = protected)
        %NumViews Number of views in a view set
        NumViews

        %NumConnections Number of connections in a view set
        NumConnections
    end

    properties (Hidden, SetAccess = protected)
        % Properties used for Views structure
        % AbsPosesDouble holds a rigidtform3d objects.This is initialized in
        % codegen mode if class of Rotation or T matrix is 'double'
        AbsPosesDouble
        % AbsPosesSingle holds a rigidtform3d objects.This is initialized in
        % codegen mode if class of Rotation or T matrix is 'single'
        AbsPosesSingle
        % PtCloudSingleOrg holds a pointCloud objects. This is initialized in
        % codegen mode if class of pointCloud Location is 'single' and
        % point cloud is organized
        PtCloudSingleOrg
        % PtCloudSingleUnOrg holds a pointCloud objects. This is initialized in
        % codegen mode if class of pointCloud Location is 'single' and
        % point cloud is un organized
        PtCloudSingleUnOrg
        % PtCloudSingleOrg holds a pointCloud objects. This is initialized in
        % codegen mode if class of pointCloud Location is 'double' and
        % point cloud is organized
        PtCloudDoubleOrg
        % PtCloudSingleUnOrg holds a pointCloud objects. This is initialized in
        % codegen mode if class of pointCloud Location is 'double' and
        % point cloud is un organized
        PtCloudDoubleUnOrg
        % ViewId A view identifier for the view
        ViewId

        % Properties used for Connections structure
        % ViewId1 A view identifier for the first view.
        ViewId1
        % ViewId2 A view identifier for the second view.
        ViewId2
        % RelPoseDouble holds a rigidtform3d objects.This is initialized in
        % codegen mode if class of Rotation or T matrix is 'double'
        RelPoseDouble
        % RelPoseSingle holds a rigidtform3d objects.This is initialized in
        % codegen mode if class of Rotation or T matrix is 'single'
        RelPoseSingle
        % InfoMatSingle Information matrix for the measurement.This is
        % initialized in codegen mode if class of matrix is 'single'
        InfoMatSingle
        % InfoMatDouble Information matrix for the measurement.This is
        % initialized in codegen mode if class of matrix is 'double'
        InfoMatDouble
    end

    properties (Hidden,GetAccess = public, SetAccess = protected)
        % AbsPoseClass holds the class of Rotation and T of rigidtform3d.
        % This is initialized in codegen mode for views
        AbsPoseClass
        % PtCloudClass holds the class of pointCloud Location.
        % This is initialized in codegen mode for views
        PtCloudClass
        % IsPtCloudOrg is 1 for organized pointCloud.
        % This is initialized in codegen mode for views.
        IsPtCloudOrg
        % RelPoseClass holds the class of Rotation and T of rigidtform3d.
        % This is initialized in codegen mode for connections
        RelPoseClass
        % InfoMatClass holds the class of information matrix.
        % This is initialized in codegen mode for connections
        InfoMatClass
    end

    methods(Static)
        function props = matlabCodegenNontunableProperties(~)
            % Used for code generation
            props = {'AbsPoseClass','RelPoseClass','PtCloudClass', ...
                'InfoMatClass', 'IsPtCloudOrg'};
        end
    end

    methods
        %------------------------------------------------------------------
        % Constructor
        %------------------------------------------------------------------
        function this = pcviewsetBase()

            coder.inline('always');
            % Initializing the variables
            [viewID, poseSingle, poseDouble, ptCloudDoubleOrg, ...
                ptCloudDoubleUnOrg, ptCloudSingleOrg, ptCloudSingleUnOrg, ...
                infMatSingle, infMatDouble] = ...
                vision.internal.codegen.pc.pcviewsetBase.initializeViewsetData();
            % Assigning the pcviewset variables
            this.ViewId = viewID;
            this.AbsPosesSingle = poseSingle;
            this.AbsPosesDouble = poseDouble;
            this.PtCloudSingleOrg = ptCloudSingleOrg;
            this.PtCloudSingleUnOrg = ptCloudSingleUnOrg;
            this.PtCloudDoubleOrg = ptCloudDoubleOrg;
            this.PtCloudDoubleUnOrg = ptCloudDoubleUnOrg;

            this.ViewId1 = viewID;
            this.ViewId2 = viewID;
            this.RelPoseSingle = poseSingle;
            this.RelPoseDouble = poseDouble;
            this.InfoMatSingle = infMatSingle;
            this.InfoMatDouble = infMatDouble;
        end

        %------------------------------------------------------------------
        % Get the data from dependent property-Views
        %------------------------------------------------------------------
        function views = get.Views(this)

            if isa(this.AbsPoseClass, 'single')
                abPose = this.AbsPosesSingle;
            else
                abPose = this.AbsPosesDouble;
            end
            if isa(this.PtCloudClass, 'double')
                if this.IsPtCloudOrg
                    ptCloud = this.PtCloudDoubleOrg;
                else
                    ptCloud = this.PtCloudDoubleUnOrg;
                end
            else
                if this.IsPtCloudOrg
                    ptCloud = this.PtCloudSingleOrg;
                else
                    ptCloud = this.PtCloudSingleUnOrg;
                end
            end

            views = struct('ViewId', this.ViewId, ...
                'AbsolutePose', abPose, 'PointCloud', ptCloud);
        end

        %------------------------------------------------------------------
        % Get the data from dependent property-Connections
        %------------------------------------------------------------------
        function Connections = get.Connections(this)
            if isa(this.RelPoseClass, 'single')
                relPose = this.RelPoseSingle;
            else
                relPose = this.RelPoseDouble;
            end

            if isa(this.InfoMatClass, 'single')
                infoMat = this.InfoMatSingle;
            else
                infoMat = this.InfoMatDouble;
            end

            Connections = struct('ViewId1', this.ViewId1, 'ViewId2', this.ViewId2, ...
                'RelativePose', relPose);
            Connections.InformationMatrix = infoMat;
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
            %   connectedViews Returns connected views
            %   viewTable = connectedViews(vSet, viewId) returns a table of
            %   views connected to the view viewId.
            %
            %   [viewTable, connIdx] = connectedViews(vSet, viewId) also
            %   returns the logic index of connections associated with the
            %   connected views.
            %
            %   See also Connections.

            viewId = checkViewId(this, viewId);
            fromViewId = this.ViewId1==viewId;
            toViewId   = this.ViewId2==viewId;

            connectedViewIds = [...
                this.ViewId2(fromViewId); ...
                this.ViewId1(toViewId)];

            % Find linear index into view table
            [~, viewIdx] = intersect(this.ViewId, connectedViewIds, 'stable');
            if ~isempty(viewIdx)
                viewId = this.Views.ViewId(viewIdx);
                viewTable = struct('ViewId', viewId, 'AbsolutePose',this.Views.AbsolutePose(viewIdx),...
                    'PointCloud', this.Views.PointCloud(viewIdx));
            else
                viewTable = struct('ViewId', zeros(0, 1,'uint32'), 'AbsolutePose', rigidtform3d(),...
                    'PointCloud', pointCloud(zeros(coder.ignoreConst(0),3)));
            end

            connIdx = fromViewId | toViewId;
        end

        %------------------------------------------------------------------
        % poses: Returns absolute poses associated with views
        %------------------------------------------------------------------
        function sensorPoses = poses(this, viewIds)

            coder.inline('always');
            if nargin>1
                viewIds = checkViewIds(this, viewIds);
                checkIfViewIsMissing(this, viewIds);
                [~, idx] = ismember(viewIds(:), this.Views.ViewId);
                viewsId = this.Views.ViewId(idx);
                if isa(this.AbsPoseClass, 'double')
                    absPoses = this.AbsPosesDouble(idx);
                else
                    absPoses = this.AbsPosesSingle(idx);
                end
            else

                viewsId = this.ViewId;
                if isa(this.AbsPoseClass, 'double')
                    absPoses = this.AbsPosesDouble;
                else
                    absPoses = this.AbsPosesSingle;
                end
            end
            sensorPoses = struct('ViewId', viewsId, 'AbsolutePose', absPoses);
        end
        %------------------------------------------------------------------
        % Returns a pose graph
        %------------------------------------------------------------------
        function G = createPoseGraph(this, weight)

            numConns = numel(this.ViewId1);
            endNodes = [this.ViewId1 this.ViewId2];
            infoMatCells = coder.nullcopy(cell(numConns, 1));
            relPoseCells = coder.nullcopy(cell(numConns, 1));

            if isa(this.InfoMatClass, 'single')
                for i=1:numConns
                    infoMatCells{i} = this.InfoMatSingle{i};
                end
            else
                for i=1:numConns
                    infoMatCells{i} = this.InfoMatDouble{i};
                end
            end
            if isa(this.RelPoseClass, 'single')
                for i=1:numConns
                    relPoseCells{i} = this.RelPoseSingle(i);
                end
            else
                for i=1:numConns
                    relPoseCells{i} = this.RelPoseDouble(i);
                end
            end

            if(nargin > 1)
                edgeTable = table(endNodes, relPoseCells, infoMatCells, weight, ...
                    'VariableNames', {'EndNodes', ...
                    'RelativePose', 'InformationMatrix', 'Weight'});
            else
                edgeTable = table(endNodes, relPoseCells, infoMatCells, ...
                    'VariableNames', {'EndNodes', ...
                    'RelativePose', 'InformationMatrix'});
            end

            numViews = numel(this.ViewId);
            absPoseCell = coder.nullcopy(cell(numViews, 1));
            if isa(this.AbsPoseClass, 'double')
                for i=1:numViews
                    absPoseCell{i} = this.AbsPosesDouble(i);
                end
            else
                for i=1:numViews
                    absPoseCell{i} = this.AbsPosesSingle(i);
                end
            end
            nodeTable = table(this.ViewId, absPoseCell, ...
                'VariableNames', {'ViewId', 'AbsolutePose'});
            G = digraph(edgeTable, nodeTable);
        end

        %------------------------------------------------------------------
        % Find views associated with view IDs
        %------------------------------------------------------------------
        function views = findView(this, viewIds)
            validateattributes(viewIds, {'numeric'}, {'nonsparse', 'vector', ...
                'integer', 'positive', 'real'}, 'findView', 'viewIds');

            checkIfViewIsMissing(this, viewIds);
            % Getting the num views
            numViews = numel(viewIds);
            vId = zeros(numViews, 1, 'uint32');
            % Getting the index of the 1 views
            viewIdx = getViewIndex(this, viewIds(1));
            % Assigning the first viewId
            vId(1) = this.ViewId(viewIdx);
            % Assigning the pose
            absPoses = this.Views.AbsolutePose(viewIdx);
            % Assigning the pointCloud
            ptCloud = this.Views.PointCloud(viewIdx);

            if (numViews > 1)
                % Looping over the remaining views
                for i = 2:numViews
                    viewIdx = getViewIndex(this, viewIds(i));
                    % Assigning the viewId
                    vId(i) = this.ViewId(viewIdx);
                    % Assigning the pose
                    absPoses(i) = this.Views.AbsolutePose(viewIdx);
                    % Assigning the pointCloud
                    ptCloud(i) = this.Views.PointCloud(viewIdx);
                end
            end
            views = struct('ViewId', vId, ...
                'AbsolutePose', absPoses, 'PointCloud', ptCloud);

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
            % Initializing the outputs
            vId1 = zeros(numConnections, 1, 'uint32');
            vId2 = zeros(numConnections, 1, 'uint32');
            infoMat = coder.nullcopy(cell(1, numConnections));
            % Getting the connection index
            idx = getConnectionIndex(this, viewIds1(1), viewIds2(1));
            % Assigning the viewId1
            vId1(1) = this.ViewId1(idx);
            % Assigning the viewId2
            vId2(1) = this.ViewId2(idx);
            % Assigning the relative pose
            relPose = this.Connections.RelativePose(idx);
            % Assigning the information matrix
            infoMat{1} = this.Connections.InformationMatrix{idx};
            if (numConnections > 1)
                for i = 2:numConnections
                    checkIfConnectionIsMissing(this, viewIds1(i), viewIds2(i))
                    idx = getConnectionIndex(this, viewIds1(i), viewIds2(i));
                    % Assigning the viewId1
                    vId1(i) = this.ViewId1(idx);
                    % Assigning the viewId2
                    vId2(i) = this.ViewId2(idx);
                    % Assigning the relative pose
                    relPose(i) = this.Connections.RelativePose(idx);
                    % Assigning the information matrix
                    infoMat{i} = this.Connections.InformationMatrix{idx};
                end
            end
            conn = struct('ViewId1', vId1, 'ViewId2', vId2, ...
                'RelativePose', relPose);
            conn.InformationMatrix = infoMat;
        end

    end

    methods (Access = protected)

        %------------------------------------------------------------------
        % Update the information matrix
        %------------------------------------------------------------------
        function infoMatUpdated = getUpdatedInfoMat(this, connTable)

            infoMatSize = numel(this.ViewId1);
            infoMatUpdated = coder.nullcopy(cell(1, infoMatSize));
            for i = 1:infoMatSize
                if i == infoMatSize
                    infoMatUpdated{i} = connTable.InformationMatrix{1};
                else
                    if strcmp(this.InfoMatClass, 'single')
                        infoMatUpdated{i} = this.InfoMatSingle{i};
                    else
                        infoMatUpdated{i} = this.InfoMatDouble{i};
                    end
                end
            end
        end

    end

    methods(Hidden, Static)

        %------------------------------------------------------------------
        % Initialize the viewset data
        %------------------------------------------------------------------
        function [vId, posesSingleArray, posesArray, ptOrgArray, ...
                ptUnOrgArray, ptOrgArraySingle, ptUnOrgArraySingle, infMatSingle,...
                infMatDouble] = initializeViewsetData()

            posesSingle = vision.internal.codegen.pc.pcviewsetBase.makeEmptyRigid3d('single');
            coder.varsize('posesSingleArray');
            posesSingleArray = posesSingle;

            poses = vision.internal.codegen.pc.pcviewsetBase.makeEmptyRigid3d('double');
            coder.varsize('posesArray');
            posesArray = poses;

            ptCloudOrg = vision.internal.codegen.pc.pcviewsetBase.makeEmptyPtCloudOrganized('double');
            coder.varsize('ptOrgArray');
            ptOrgArray = ptCloudOrg;

            ptCloudUnOrg = vision.internal.codegen.pc.pcviewsetBase.makeEmptyPtCloudUnOrganized('double');
            coder.varsize('ptUnOrgArray');
            ptUnOrgArray = ptCloudUnOrg;

            ptCloudOrgSingle = vision.internal.codegen.pc.pcviewsetBase.makeEmptyPtCloudOrganized('single');
            coder.varsize('ptOrgArraySingle');
            ptOrgArraySingle = ptCloudOrgSingle;

            ptCloudUnOrgSingle = vision.internal.codegen.pc.pcviewsetBase.makeEmptyPtCloudUnOrganized('single');
            coder.varsize('ptUnOrgArraySingle');
            ptUnOrgArraySingle = ptCloudUnOrgSingle;

            vId = zeros(coder.ignoreConst(0), 1, 'uint32');

            % Default value for Information Matrix is eye(6);
            coder.varsize('infMatSingle');
            coder.varsize('infMatDouble');
            infMatSingle = {zeros(coder.ignoreConst(0), coder.ignoreConst(0), 'single')};
            infMatDouble = {zeros(coder.ignoreConst(0), coder.ignoreConst(0))};
        end

        %------------------------------------------------------------------
        % Make empty unorganized pointCloud object
        %------------------------------------------------------------------
        function obj = makeEmptyPtCloudUnOrganized(classType)
            location  = zeros(coder.ignoreConst(0),3, classType);
            color     = zeros(coder.ignoreConst(0),3, 'uint8');
            normal    = zeros(coder.ignoreConst(0),3, classType);
            intensity = zeros(coder.ignoreConst(0),coder.ignoreConst(0), classType);
            pc = pointCloud(location, 'Color', color, 'Normal', normal...
                ,'Intensity', intensity);
            obj = repmat(pc,0,0);
        end

        %------------------------------------------------------------------
        % Make empty organized pointCloud object
        %------------------------------------------------------------------
        function obj = makeEmptyPtCloudOrganized(classType)
            location  = zeros(coder.ignoreConst(0), coder.ignoreConst(0), 3, classType);
            color     = zeros(coder.ignoreConst(0), coder.ignoreConst(0), 3, 'uint8');
            normal    = zeros(coder.ignoreConst(0), coder.ignoreConst(0), 3, classType);
            intensity = zeros(coder.ignoreConst(0),coder.ignoreConst(0), classType);
            pc = pointCloud(location, 'Color', color, 'Normal', normal...
                ,'Intensity', intensity);
            obj = repmat(pc,0,0);
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

    end

end
