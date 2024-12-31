classdef pcviewset < vision.internal.codegen.pc.pcviewsetBase
    % pcviewset class is used for code generation support

    % Copyright 2021-2023 The MathWorks, Inc.
    %#codegen

    properties (Constant, GetAccess = public, Hidden)

        ClassName = 'pcviewset';
    end

    properties (Constant, Access = protected)

        Version   = 1.0;
        ExtraViewVariables          = {'PointCloud'};
        ExtraConnectionVariables    = {};
    end

    methods

        %------------------------------------------------------------------
        % Constructor
        %------------------------------------------------------------------
        function this = pcviewset()

            coder.inline('always');
            this = this@vision.internal.codegen.pc.pcviewsetBase();
        end

        %------------------------------------------------------------------
        % addView: Add a new view to the view set
        %------------------------------------------------------------------
        function this = addView(this, varargin)

            coder.inline('always');
            fillMissingVars = true;
            % Check the inputs of addView
            if istable(varargin{1})
                [this, viewTable] = checkViewTable(this, varargin{1}, fillMissingVars);
            else
                [this, viewTable] = parseViewInputs(this, fillMissingVars, varargin{:});
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
                this.ViewId = viewTable.ViewId;
                % Assigning the absolutePose
                if isa(this.AbsPoseClass, 'double')
                    this.AbsPosesDouble = viewTable.AbsolutePose;
                else
                    this.AbsPosesSingle = viewTable.AbsolutePose;
                end
                % Assigning the pointCloud
                if isa(this.PtCloudClass, 'double')
                    if this.IsPtCloudOrg
                        this.PtCloudDoubleOrg = assign(this.PtCloudDoubleOrg, viewTable.PointCloud);
                    else
                        this.PtCloudDoubleUnOrg = assign(this.PtCloudDoubleUnOrg, viewTable.PointCloud);
                    end
                else
                    if this.IsPtCloudOrg
                        this.PtCloudSingleOrg = assign(this.PtCloudSingleOrg, viewTable.PointCloud);
                    else
                        this.PtCloudSingleUnOrg = assign(this.PtCloudSingleUnOrg, viewTable.PointCloud);
                    end
                end
            else
                this.ViewId = [this.ViewId; viewTable.ViewId];
                % Assigning the absolutePose
                if  isa(this.AbsPoseClass, 'double')
                    this.AbsPosesDouble = [this.AbsPosesDouble, viewTable.AbsolutePose];
                else
                    this.AbsPosesSingle = [this.AbsPosesSingle, viewTable.AbsolutePose];
                end
                % Assigning the pointCloud
                if isa(this.PtCloudClass, 'double')
                    if this.IsPtCloudOrg
                        this.PtCloudDoubleOrg = [this.PtCloudDoubleOrg, viewTable.PointCloud];
                    else
                        this.PtCloudDoubleUnOrg = [this.PtCloudDoubleUnOrg, viewTable.PointCloud];
                    end
                else
                    if this.IsPtCloudOrg
                        this.PtCloudSingleOrg = [this.PtCloudSingleOrg, viewTable.PointCloud];
                    else
                        this.PtCloudSingleUnOrg = [this.PtCloudSingleUnOrg, viewTable.PointCloud];
                    end
                end
            end
        end

        %------------------------------------------------------------------
        % updateView: Modify an existing view
        %------------------------------------------------------------------
        function this = updateView(this, varargin)

            coder.inline('always');
            fillMissingVars = false;
            if istable(varargin{1})
                [this, viewTable] = checkViewTable(this, varargin{1}, fillMissingVars);
            else
                [this, viewTable] = parseViewInputs(this, fillMissingVars, varargin{:});
            end

            checkIfViewIsMissing(this, viewTable.ViewId);
            idx = find(this.ViewId == viewTable.ViewId, 1, 'first');
            varNames = fieldnames(viewTable);
            % Assigning the absolutePose
            if any(strcmp('AbsolutePose', varNames))
                if  isa(this.AbsPoseClass, 'double')
                    this.AbsPosesDouble(idx) = viewTable.AbsolutePose;
                else
                    this.AbsPosesSingle(idx) = viewTable.AbsolutePose;
                end
            end
            % Assigning the pointCloud
            if any(strcmp('PointCloud', varNames))
                if isa(this.PtCloudClass, 'double')
                    if this.IsPtCloudOrg
                        this.PtCloudDoubleOrg(idx) = viewTable.PointCloud;
                    else
                        this.PtCloudDoubleUnOrg(idx) = viewTable.PointCloud;
                    end
                else
                    if this.IsPtCloudOrg
                        this.PtCloudSingleOrg(idx) = viewTable.PointCloud;
                    else
                        this.PtCloudSingleUnOrg(idx) = viewTable.PointCloud;
                    end
                end
            end
        end

        %------------------------------------------------------------------
        % deleteView: Remove an existing view
        %------------------------------------------------------------------
        function this = deleteView(this, viewIds)

            coder.inline('always');
            viewIds = checkViewIds(this, viewIds);
            checkIfViewIsMissing(this, viewIds);
            % Getting the index of the views
            removeViewsIdx = find(ismember(this.ViewId,viewIds));

            coder.varsize('removeConnsIdx', [1, inf],[0 1]);
            removeConnsIdx = zeros(1,0);
            for i=1:numel(viewIds)
                if ~isempty(this.Connections.ViewId1)
                    % Get the index of connections being deleted
                    connIdx = getConnectionIndexToAndFrom(this, viewIds(i));
                    if ~isempty(connIdx)
                        if isempty(removeConnsIdx)
                            removeConnsIdx = connIdx;
                        else
                            removeConnsIdx = [removeConnsIdx connIdx]; %#ok
                        end
                    end
                end
            end
            % Compute viewsLeft which is difference between all ViewIds
            % and viewsRemoved
            [viewsLeft, indicesViewsLeft] = setdiff(this.ViewId, ...
                this.ViewId(removeViewsIdx));

            if isempty(viewsLeft)
                this.ViewId = zeros(0, 1,'uint32');
                if isa(this.AbsPoseClass, 'double')
                    this.AbsPosesDouble = vision.internal.codegen.pc.pcviewsetBase.makeEmptyRigid3d('double');
                else
                    this.AbsPosesSingle = vision.internal.codegen.pc.pcviewsetBase.makeEmptyRigid3d('single');
                end
                if isa(this.PtCloudClass, 'double')
                    if this.IsPtCloudOrg
                        this.PtCloudDoubleOrg = vision.internal.codegen.pc.pcviewsetBase.makeEmptyPtCloudOrganized('double');
                    else
                        this.PtCloudDoubleUnOrg = vision.internal.codegen.pc.pcviewsetBase.makeEmptyPtCloudUnOrganized('double');
                    end
                else
                    if this.IsPtCloudOrg
                        this.PtCloudSingleOrg = vision.internal.codegen.pc.pcviewsetBase.makeEmptyPtCloudOrganized('single');
                    else
                        this.PtCloudSingleUnOrg = vision.internal.codegen.pc.pcviewsetBase.makeEmptyPtCloudUnOrganized('single');
                    end
                end
            else
                % Assigning the remaining poses
                if isa(this.AbsPoseClass, 'double')
                    this.AbsPosesDouble = this.AbsPosesDouble(indicesViewsLeft);
                else
                    this.AbsPosesSingle = this.AbsPosesSingle(indicesViewsLeft);
                end
                % Assigning the remaining pointclouds
                if isa(this.PtCloudClass, 'double')
                    if this.IsPtCloudOrg
                        this.PtCloudDoubleOrg = this.PtCloudDoubleOrg(indicesViewsLeft);
                    else
                        this.PtCloudDoubleUnOrg = this.PtCloudDoubleUnOrg(indicesViewsLeft);
                    end
                else
                    if this.IsPtCloudOrg
                        this.PtCloudSingleOrg = this.PtCloudSingleOrg(indicesViewsLeft);
                    else
                        this.PtCloudSingleUnOrg = this.PtCloudSingleUnOrg(indicesViewsLeft);
                    end
                end
                % Delete the corresponding connection
                if ~isempty(this.ViewId1)
                    for i=1:numel(removeConnsIdx)
                        view1 = this.ViewId1(removeConnsIdx(i));
                        view2 = this.ViewId2(removeConnsIdx(i));
                        this = deleteConnection(this, view1, view2);
                    end
                end
                % Assigning the remaining viewIds
                this.ViewId = this.ViewId(indicesViewsLeft);
            end
        end

        %------------------------------------------------------------------
        % hasView: Check if a view exists
        %------------------------------------------------------------------
        function tf = hasView(this, viewIds)

            coder.inline('always');
            tf = hasView@vision.internal.codegen.pc.pcviewsetBase(this, viewIds);
        end

        %------------------------------------------------------------------
        % findView: Find views associated with view IDs
        %------------------------------------------------------------------
        function views = findView(this, viewIds)

            coder.inline('always');
            views = findView@vision.internal.codegen.pc.pcviewsetBase(this, viewIds);
        end
        %------------------------------------------------------------------
        % addConnection: Add a connection between existing views
        %------------------------------------------------------------------
        function this = addConnection(this, viewId1, viewId2, varargin)

            coder.inline('always');
            fillMissingVars = true;
            [this, connTable] = parseConnectionInputs(this, fillMissingVars, viewId1, viewId2, varargin{:});
            connectionExists = hasConnection(this, connTable.ViewId1, connTable.ViewId2);

            coder.internal.errorIf(connectionExists, 'vision:viewSet:connectionAlreadyExists', ...
                connTable.ViewId1, connTable.ViewId2);
            % Assigning the connection properties
            if isempty(this.ViewId1)
                this.ViewId1 = connTable.ViewId1;
                this.ViewId2 = connTable.ViewId2;
                if isa(this.RelPoseClass, 'single')
                    this.RelPoseSingle = connTable.RelativePose;
                else
                    this.RelPoseDouble = connTable.RelativePose;
                end
                if isa(this.InfoMatClass, 'single')
                    this.InfoMatSingle = connTable.InformationMatrix;
                else
                    this.InfoMatDouble = connTable.InformationMatrix;
                end
            else
                this.ViewId1 = [this.ViewId1; connTable.ViewId1];
                this.ViewId2 = [this.ViewId2; connTable.ViewId2];
                if isa(this.RelPoseClass, 'single')
                    this.RelPoseSingle = [this.RelPoseSingle connTable.RelativePose];
                else
                    this.RelPoseDouble = [this.RelPoseDouble connTable.RelativePose];
                end
                if isa(this.InfoMatClass, 'single')
                    this.InfoMatSingle = getUpdatedInfoMat(this, connTable);
                else
                    this.InfoMatDouble = getUpdatedInfoMat(this, connTable);
                end
            end
        end

        %------------------------------------------------------------------
        % updateConnection: Modify an existing connection
        %------------------------------------------------------------------
        function this = updateConnection(this, varargin)

            coder.inline('always');
            fillMissingVars = false;
            [this, connTable] = parseConnectionInputs(this, fillMissingVars, varargin{:});

            checkIfConnectionIsMissing(this, connTable.ViewId1, connTable.ViewId2);
            % Get the scalar index of the connection
            idx = getConnectionIndex(this, connTable.ViewId1, connTable.ViewId2);
            varNames = connTable.Properties.VariableNames;
            % Assigning the relativePose
            if any(strcmp(varNames, 'RelativePose'))
                if isa(this.RelPoseClass, 'single')
                    this.RelPoseSingle(idx) = connTable.RelativePose;
                else
                    this.RelPoseDouble(idx) = connTable.RelativePose;
                end
            end
            % Assigning the informationMatrix
            if any(strcmp(varNames, 'InformationMatrix'))
                if isa(connTable.InformationMatrix, 'single')
                    this.InfoMatSingle{idx} = connTable.InformationMatrix{1};
                else
                    this.InfoMatDouble{idx} = connTable.InformationMatrix{1};
                end
            end
        end

        %------------------------------------------------------------------
        % deleteConnection: Delete an existing connection
        %------------------------------------------------------------------
        function this = deleteConnection(this, viewId1, viewId2)

            coder.inline('always');
            checkIfConnectionIsMissing(this, viewId1, viewId2);
            % Get the index of the connection being deleted
            idx = getConnectionIndex(this, viewId1, viewId2);
            % Get the indices of the connections left
            % 1:numel(this.Connections.ViewId1) denotes the indices of
            % all connections
            [connsLeft, indicesConnsLeft] = setdiff((1:numel(this.ViewId1)), idx(1));
            % No connections are left
            if isempty(connsLeft)
                this.ViewId1 = zeros(0, 1,'uint32');
                this.ViewId2 = zeros(0, 1,'uint32');
                if isa(this.RelPoseClass, 'double')
                    this.RelPoseDouble = vision.internal.codegen.pc.pcviewsetBase.makeEmptyRigid3d('double');
                else
                    this.RelPoseSingle = vision.internal.codegen.pc.pcviewsetBase.makeEmptyRigid3d('single');
                end

                if isa(this.InfoMatClass, 'single')
                    this.InfoMatSingle = {zeros(coder.ignoreConst(0), coder.ignoreConst(0), 'single')};
                else
                    this.InfoMatDouble = {zeros(coder.ignoreConst(0), coder.ignoreConst(0))};
                end
            else
                % Store only the details of remaining connections
                this.ViewId1 = this.ViewId1(indicesConnsLeft);
                this.ViewId2 = this.ViewId2(indicesConnsLeft);
                if isa(this.RelPoseClass, 'single')
                    this.RelPoseSingle = this.RelPoseSingle(indicesConnsLeft);
                else
                    this.RelPoseDouble = this.RelPoseDouble(indicesConnsLeft);
                end
                coder.varsize('infoMatUpdated');
                infoMatSize = numel(this.ViewId1);
                infoMatUpdated = coder.nullcopy(cell(1, infoMatSize));
                if isa(this.InfoMatClass, 'single')
                    for i = 1:infoMatSize
                        infoMatUpdated{i} = this.InfoMatSingle{indicesConnsLeft(i)};
                    end
                    this.InfoMatSingle = infoMatUpdated;
                else
                    for i = 1:infoMatSize
                        infoMatUpdated{i} = this.InfoMatDouble{indicesConnsLeft(i)};
                    end
                    this.InfoMatDouble = infoMatUpdated;
                end
            end
        end

        %------------------------------------------------------------------
        % hasConnection: Check if a connection between two views exists
        %-------------------------------------------------------------------
        function tf = hasConnection(this, viewId1, viewId2)

            tf = hasConnection@vision.internal.codegen.pc.pcviewsetBase...
                (this, viewId1, viewId2);
        end

        %------------------------------------------------------------------
        % findConnection: Find connections associated with view IDs
        %------------------------------------------------------------------
        function conn = findConnection(this, viewIds1, viewIds2)

            coder.inline('always');
            conn = findConnection@vision.internal.codegen.pc.pcviewsetBase(this, ...
                viewIds1, viewIds2);
        end

        %------------------------------------------------------------------
        % poses: Returns absolute poses associated with views
        %------------------------------------------------------------------
        function sensorPoses = poses(this, varargin)

            coder.inline('always');
            sensorPoses = poses@vision.internal.codegen.pc.pcviewsetBase(this, varargin{:});
        end

        %------------------------------------------------------------------
        % createPoseGraph: returns a pose graph
        %------------------------------------------------------------------
        function G = createPoseGraph(this)

            coder.inline('always');
            G = createPoseGraph@vision.internal.codegen.pc.pcviewsetBase(this);
        end

        %------------------------------------------------------------------
        % optimizePoses: Optimize point cloud view set poses
        %------------------------------------------------------------------
        function this = optimizePoses(this, varargin)

            coder.inline('always');
            G = createPoseGraph(this);
            poseTable = vision.internal.optimizePoses(G, varargin{:});
            for i=1:numel(this.ViewId)
                this = updateView(this, this.ViewId(i), rigidtform3d(poseTable.AbsolutePose{i}'), ...
                    'PointCloud', this.Views.PointCloud(i));
            end
        end
    end

    methods (Access = protected)
        %------------------------------------------------------------------
        % Validate and initialize the viewTable
        %------------------------------------------------------------------
        function [this, viewTable] = checkViewTable(this, viewTableIn, fillMissingVars)

            coder.inline('always');
            coder.internal.prefer_const(fillMissingVars, viewTableIn);
            variableNames = viewTableIn.Properties.VariableNames;

            hasRequiredVariables = isequal(variableNames{1}, 'ViewId');
            coder.internal.errorIf(~hasRequiredVariables,'vision:viewSet:requiredColumnsMissing', ...
                'viewTable', 'ViewId');

            hasAbsolutePose = any(strcmp('AbsolutePose', variableNames));
            hasPointCloud   = any(strcmp('PointCloud',variableNames));

            hasOneOptionalVariables = width(viewTableIn)==2 && (hasAbsolutePose || hasPointCloud);
            hasTwoOptionalVariables = width(viewTableIn)==3 && hasAbsolutePose && hasPointCloud;

            optionalColumnsInvalid = width(viewTableIn)>1 && ~(hasOneOptionalVariables || hasTwoOptionalVariables);
            coder.internal.errorIf(optionalColumnsInvalid,'vision:viewSet:optionalColumnsInvalid', ...
                'viewTable', 'ViewId', 'AbsolutePose, PointCloud');

            % Initializing the properties
            viewIds = uint32(this.checkViewId(viewTableIn.ViewId));

            if hasAbsolutePose
                tform = viewTableIn.AbsolutePose(1);
                this.checkPose(tform, 'AbsolutePose');
                if coder.internal.prop_has_class(this, 'AbsPoseClass')
                    coder.internal.assert(isequal(class(this.AbsPoseClass), class(tform.T)),...
                        'images:geotrans:differentTypes');
                end
                absPose = rigidtform3d(tform.T');
                this.AbsPoseClass = cast([], 'like', absPose.T);
            else
                this.AbsPoseClass = double([]);
                if fillMissingVars
                    absPose = rigidtform3d(eye(3), [0 0 0]);
                end
            end

            if hasPointCloud
                ptCloud = viewTableIn.PointCloud;
                this.checkPointCloud(ptCloud);
                if coder.internal.prop_has_class(this, 'PtCloudClass')
                    oldClass = class(this.PtCloudClass);
                    newClass = class(ptCloud.Location);
                    coder.internal.assert(isequal(oldClass, newClass),...
                        'vision:pointcloud:differentTypes');
                end
                this.PtCloudClass = cast([], 'like', ptCloud.Location);
                if coder.internal.prop_has_class(this, 'IsPtCloudOrg')
                    oldValue = this.IsPtCloudOrg;
                    newValue = 0;
                    if ~ismatrix(ptCloud.Location)
                        newValue = 1;
                    end
                    coder.internal.assert(isequal(oldValue, newValue),...
                        'vision:pointcloud:differentPointCloudTypes');
                end

                if ismatrix(ptCloud.Location)
                    this.IsPtCloudOrg = 0;
                else
                    this.IsPtCloudOrg = 1;
                end
            else
                this.PtCloudClass = double([]);
                this.IsPtCloudOrg = 0;
                if fillMissingVars
                    ptCloud = pointCloud(zeros(coder.ignoreConst(0),3));

                end
            end

            if fillMissingVars
                viewTable = struct('ViewId', viewIds, 'AbsolutePose', absPose, 'PointCloud', ptCloud);
            else
                % Return a table with the columns specified by user
                if hasPointCloud
                    if hasAbsolutePose
                        viewTable = struct('ViewId', viewIds, 'AbsolutePose', absPose, 'PointCloud', ptCloud);
                    else
                        viewTable = struct('ViewId', viewIds, 'PointCloud', ptCloud);
                    end
                else
                    if hasAbsolutePose
                        viewTable = struct('ViewId', viewIds, 'AbsolutePose', absPose);
                    else
                        viewTable = struct('ViewId', viewIds);
                    end
                end
            end
        end

        %------------------------------------------------------------------
        % Validate and initialize the inputs for pcviewset
        %------------------------------------------------------------------
        function [this, viewTable] = parseViewInputs(this, fillMissingVars, varargin)

            coder.inline('always');
            coder.internal.prefer_const(fillMissingVars, varargin{:});

            narginchk(3, 6);
            % varargin may contain optional param (absolutePose) and PV
            % pair(PointCloud)
            len = length(varargin);
            % Initializing the properties
            viewId = uint32(this.checkViewId(varargin{1}));

            if (len==2) ||(len==4)  % secondArg = absolutePose
                tform  = varargin{2};
                this.checkPose(tform, 'absPose');
                absolutePose = rigidtform3d(tform.T');
                pvPairStartIdx = 3; % varargin{3:end} must be PV pair
            else
                absolutePose = rigidtform3d(eye(3), [0 0 0]);
                pvPairStartIdx = 2;
            end

            % Define parser mapping struct
            pvPairs = struct('PointCloud', uint32(0));

            % Specify parser options
            poptions = struct( ...
                'CaseSensitivity', false, ...
                'StructExpand',    true, ...
                'PartialMatching', true);
            % Parse PV pairs
            pstruct = coder.internal.parseParameterInputs(pvPairs, ...
                poptions, varargin{pvPairStartIdx:end});
            % Extract inputs
            ptCloud = coder.internal.getParameterValue( ...
                pstruct.PointCloud, pointCloud(zeros(coder.ignoreConst(0),3)), ...
                varargin{pvPairStartIdx:end});
            this.checkPointCloud(ptCloud);

            % Initializing the non tunable properties
            if coder.internal.prop_has_class(this, 'AbsPoseClass')
                oldClass = class(this.AbsPoseClass);
                newClass = class(absolutePose.T);
                coder.internal.assert(isequal(oldClass, newClass),...
                    'images:geotrans:differentTypes');
            end

            this.AbsPoseClass = cast([], 'like', absolutePose.T);

            if coder.internal.prop_has_class(this, 'PtCloudClass')
                oldClass = class(this.PtCloudClass);
                newClass = class(ptCloud.Location);
                coder.internal.assert(isequal(oldClass, newClass),...
                    'vision:pointcloud:differentTypes');
            end

            this.PtCloudClass = cast([], 'like', ptCloud.Location);

            if coder.internal.prop_has_class(this, 'IsPtCloudOrg')
                oldValue = this.IsPtCloudOrg;
                newValue = 0;
                if ~ismatrix(ptCloud.Location)
                    newValue = 1;
                end
                coder.internal.assert(isequal(oldValue, newValue),...
                    'vision:pointcloud:differentPointCloudTypes');
            end

            if ismatrix(ptCloud.Location)
                this.IsPtCloudOrg = 0;
            else
                this.IsPtCloudOrg = 1;
            end

            if fillMissingVars
                viewTable = struct('ViewId', viewId, 'AbsolutePose', absolutePose, 'PointCloud', ptCloud);
            else
                % Return a table with the columns specified by user
                if nargin == 3
                    viewTable = struct('ViewId', viewId);
                elseif nargin == 4
                    viewTable = struct('ViewId', viewId, 'AbsolutePose', absolutePose);
                elseif nargin == 5
                    viewTable = struct('ViewId', viewId, 'PointCloud', ptCloud);
                elseif nargin == 6
                    viewTable = struct('ViewId', viewId, 'AbsolutePose', absolutePose,...
                        'PointCloud', ptCloud);
                end
            end
        end

        %------------------------------------------------------------------
        % Validate and initialize the inputs for connections
        %------------------------------------------------------------------
        function [this, connTable] = parseConnectionInputs(this, fillMissingVars, varargin)
            coder.inline('always');
            narginchk(4,6);
            coder.internal.prefer_const(fillMissingVars, varargin{:});
            % Initializing the properties
            viewId1 = this.checkViewId(varargin{1});
            viewId2 = this.checkViewId(varargin{2});

            coder.internal.errorIf(~hasView(this, viewId1), ...
                'vision:viewSet:missingViewId', viewId1);
            coder.internal.errorIf(~hasView(this, viewId2), ...
                'vision:viewSet:missingViewId', viewId2);
            % Initializing the properties
            if nargin == 4
                relativePose = rigidtform3d(eye(3), [0 0 0]);
                infoMat = {eye(6)};
            elseif nargin == 5
                tform = varargin{3};
                this.checkPose(tform, 'relPose')
                relativePose = rigidtform3d(tform.T');
                infoMat = {eye(6)};
            elseif nargin == 6
                tform = varargin{3};
                this.checkPose(tform, 'relPose')
                relativePose = rigidtform3d(tform.T');
                iMat = varargin{4};
                this.checkInfoMat(iMat);
                infoMat = {iMat};
            end
            % Initializing the non tunable properties
            this.RelPoseClass = cast([], 'like', relativePose.T);
            this.InfoMatClass = cast([], 'like', infoMat{1});

            if fillMissingVars
                connTable = table(viewId1, viewId2, relativePose, infoMat, ...
                    'VariableNames', {'ViewId1', 'ViewId2', 'RelativePose', 'InformationMatrix'});
            else
                if nargin == 4
                    connTable = table(viewId1, viewId2, ...
                        'VariableNames', {'ViewId1', 'ViewId2'});
                elseif nargin == 5
                    connTable = table(viewId1, viewId2, relativePose, ...
                        'VariableNames', {'ViewId1', 'ViewId2', 'RelativePose'});
                elseif nargin == 6
                    connTable = table(viewId1, viewId2, relativePose, infoMat, ...
                        'VariableNames', {'ViewId1', 'ViewId2', 'RelativePose', 'InformationMatrix'});
                end
            end
        end
    end

end
