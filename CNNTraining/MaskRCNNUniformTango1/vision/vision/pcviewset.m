classdef pcviewset < vision.internal.ViewSetBase

    % Copyright 2019-2023 The MathWorks, Inc.
    
    %#codegen

    properties (SetAccess = protected)
        Views = table('Size', [0,3], ...
            'VariableTypes', {'uint32', 'rigidtform3d', 'pointCloud'}, ...
            'VariableNames',{'ViewId', 'AbsolutePose', 'PointCloud'});
        Connections = table('Size',[0,4], ...
            'VariableTypes', {'uint32', 'uint32', 'rigidtform3d', 'cell'}, ...
            'VariableNames', {'ViewId1', 'ViewId2', 'RelativePose', 'InformationMatrix'});
    end

    properties (Constant, GetAccess = public, Hidden)
        ClassName = mfilename;
    end

    properties (Constant, Access = protected)

        Version = 1.1;
        
        ExtraViewVariables = {'PointCloud'};
        ExtraConnectionVariables = {};
    end

    methods
        %------------------------------------------------------------------
        function this = addView(this, varargin)

            fillMissingVars = true;
            if istable(varargin{1})
                viewTable = checkViewTable(this, varargin{1}, fillMissingVars);
            else
                viewTable = parseViewInputs(this, fillMissingVars, varargin{:});
            end

            viewExists = hasView(this, viewTable.ViewId);
            if any( viewExists )
                existingViews = viewTable.ViewId(viewExists);
                error(message('vision:viewSet:viewIdAlreadyExists', existingViews(1)))
            end

            this.Views = [this.Views; viewTable];
        end

        %------------------------------------------------------------------
        function this = updateView(this, varargin)

            fillMissingVars = false;
            if istable(varargin{1})
                viewTable = checkViewTable(this, varargin{1}, fillMissingVars);
                checkIfViewIsMissing(this, viewTable.ViewId);

                allViewIds = this.Views.ViewId;
                [~, ia] = ismember(viewTable.ViewId, allViewIds);
                this.Views(ia, viewTable.Properties.VariableNames) = viewTable;
            else
                viewTable = parseViewInputs(this, fillMissingVars, varargin{:});
                checkIfViewIsMissing(this, viewTable.ViewId);

                idx = find(this.Views.ViewId == viewTable.ViewId, 1, 'first');

                varNames = viewTable.Properties.VariableNames;
                if ismember('AbsolutePose', varNames)
                    this.Views.AbsolutePose(idx) = viewTable.AbsolutePose;
                end
                if ismember('PointCloud', varNames)
                    this.Views.PointCloud(idx)   = viewTable.PointCloud;
                end
            end
        end

        %------------------------------------------------------------------
        function this = deleteView(this, viewIds)

            viewIds = checkViewIds(this, viewIds);

            for i=1:numel(viewIds)
                checkIfViewIsMissing(this, viewIds(i));

                viewIdx = getViewIndex(this, viewIds(i));
                this.Views(viewIdx, :) = [];

                if ~isempty(this.Connections)
                    connIdx = getConnectionIndexToAndFrom(this, viewIds(i));
                    this.Connections(connIdx, :) = [];
                end
            end
        end

        %------------------------------------------------------------------
        function tf = hasView(this, viewIds)
            tf = hasView@vision.internal.ViewSetBase(this, viewIds);
        end

        %------------------------------------------------------------------
        function views = findView(this, viewIds)
            views = findView@vision.internal.ViewSetBase(this, viewIds);
        end

        %------------------------------------------------------------------
        function this = addConnection(this, viewId1, viewId2, varargin)

            fillMissingVars = true;
            connTable = parseConnectionInputs(this, fillMissingVars, viewId1, viewId2, varargin{:});

            if hasConnection(this, connTable.ViewId1, connTable.ViewId2)
                error(message('vision:viewSet:connectionAlreadyExists', ...
                    connTable.ViewId1, connTable.ViewId2));
            end

            this.Connections = [this.Connections; connTable];
        end

        %------------------------------------------------------------------
        function conn = findConnection(this, viewIds1, viewIds2)
            conn = findConnection@vision.internal.ViewSetBase(this, ...
                viewIds1, viewIds2);
        end

        %------------------------------------------------------------------
        function this = updateConnection(this, varargin)
            fillMissingVars = false;
            connTable = parseConnectionInputs(this, fillMissingVars, varargin{:});

            checkIfConnectionIsMissing(this, connTable.ViewId1, connTable.ViewId2);

            idx = getConnectionIndex(this, connTable.ViewId1, connTable.ViewId2);

            this.Connections.RelativePose(idx) = connTable.RelativePose;
            this.Connections{idx, 'InformationMatrix'} = connTable.InformationMatrix;
        end

        %------------------------------------------------------------------
        function this = deleteConnection(this, viewId1, viewId2)

            checkIfConnectionIsMissing(this, viewId1, viewId2);
            idx = getConnectionIndex(this, viewId1, viewId2);

            this.Connections(idx, :) = [];
        end

        %------------------------------------------------------------------
        function tf = hasConnection(this, viewId1, viewId2)
            tf = hasConnection@vision.internal.ViewSetBase(this, viewId1, viewId2);
        end

        %------------------------------------------------------------------
        function sensorPoses = poses(this, varargin)
            sensorPoses = poses@vision.internal.ViewSetBase(this, varargin{:});
        end

        %------------------------------------------------------------------
        function this = optimizePoses(this, varargin)
            G = createPoseGraph(this);
            poseTable = vision.internal.optimizePoses(G, varargin{:});
            this = updateView(this, poseTable);
        end

        %------------------------------------------------------------------
        function G = createPoseGraph(this)
            G = createPoseGraph@vision.internal.ViewSetBase(this);
        end

        %------------------------------------------------------------------
        function varargout = plot(this, varargin)
            [varargout{1:nargout}] = plot@vision.internal.ViewSetBase(this, varargin{:});
        end
    end

    methods (Access = protected)
        %------------------------------------------------------------------
        function viewTable = checkViewTable(this, viewTable, fillMissingVars)

            variableNames = viewTable.Properties.VariableNames;

            hasRequiredVariables = isequal(variableNames{1}, 'ViewId');
            if ~hasRequiredVariables
                error(message('vision:viewSet:requiredColumnsMissing', ...
                    'viewTable', 'ViewId'));
            end

            hasAbsolutePose = any(ismember(variableNames(2:end), 'AbsolutePose'));
            hasPointCloud   = any(ismember(variableNames(2:end), 'PointCloud'));

            hasOneOptionalVariables = width(viewTable)==2 && (hasAbsolutePose || hasPointCloud);
            hasTwoOptionalVariables = width(viewTable)==3 && hasAbsolutePose && hasPointCloud;

            if width(viewTable)>1 && ~(hasOneOptionalVariables || hasTwoOptionalVariables)
                error(message('vision:viewSet:optionalColumnsInvalid', ...
                    'viewTable', 'ViewId', 'AbsolutePose, PointCloud'));
            end

            viewTable.ViewId = checkViewIds(this, viewTable.ViewId);

            if hasAbsolutePose
                checkPoses(this, viewTable.AbsolutePose, 'AbsolutePose');
                % Check if existing absolute pose table is filled with
                % rigid3d objects and if so, convert them to rigidtform3d
                if isa(viewTable.AbsolutePose, 'rigid3d')
                    viewTable.AbsolutePose = vision.internal.viewset.rigid3dConvert(viewTable.AbsolutePose);
                end
            elseif fillMissingVars
                viewTable.AbsolutePose = repelem(rigidtform3d, height(viewTable), 1);
            end

            if hasPointCloud
                viewTable.PointCloud = checkPointClouds(this, viewTable.PointCloud);
            elseif fillMissingVars
                viewTable.PointCloud = repelem(pointCloud(zeros(0, 3)), height(viewTable), 1);
            end
        end

        %------------------------------------------------------------------
        function viewTable = parseViewInputs(this, fillMissingVars, varargin)

            persistent viewParser
            if isempty(viewParser)
                viewParser = inputParser;
                viewParser.FunctionName = this.ClassName;

                addRequired(viewParser, 'viewId', @this.checkViewId);
                addOptional(viewParser, 'absPose', rigidtform3d, @(tform)this.checkPose(tform, 'absPose'));
                addParameter(viewParser, 'PointCloud', pointCloud(zeros(0,3)), @this.checkPointCloud);
            end

            parse(viewParser, varargin{:});

            ViewId       = viewParser.Results.viewId;
            originalPose = viewParser.Results.absPose;
            PointCloud   = viewParser.Results.PointCloud;
            
            % Check if absolute pose is a rigid3d object and if so, 
            % convert it to a rigidtform3d object
            if isa(originalPose, 'rigid3d')
                AbsolutePose = vision.internal.viewset.rigid3dConvert(originalPose);
            else
                AbsolutePose = originalPose;
            end

            if fillMissingVars
                viewTable = table(ViewId, AbsolutePose, PointCloud);
            else
                % Return a table with only the columns specified by the
                % user
                unsetColumns = viewParser.UsingDefaults;

                viewTable = table(ViewId);

                if ~ismember('absPose', unsetColumns)
                    viewTable.AbsolutePose = AbsolutePose;
                end

                if ~ismember('PointCloud', unsetColumns)
                    viewTable.PointCloud = PointCloud;
                end
            end
        end

        %------------------------------------------------------------------
        function connTable = parseConnectionInputs(this, fillMissingVars, varargin)

            persistent connParser
            if isempty(connParser)
                connParser = inputParser;
                connParser.FunctionName = this.ClassName;

                addRequired(connParser, 'viewId1', @this.checkViewId);
                addRequired(connParser, 'viewId2', @this.checkViewId);
                addOptional(connParser, 'relPose', rigidtform3d, @(tform)this.checkPose(tform, 'relPose'));
                addOptional(connParser, 'infoMat', eye(6), @this.checkInfoMat);
            end

            parse(connParser, varargin{:});

            ViewId1             = connParser.Results.viewId1;
            ViewId2             = connParser.Results.viewId2;
            originalPose        = connParser.Results.relPose;
            InformationMatrix   = {connParser.Results.infoMat};

            % Check if relative pose is a rigid3d object and if so, 
            % convert it to a rigidtform3d object
            if isa(originalPose, 'rigid3d')
                RelativePose = vision.internal.viewset.rigid3dConvert(originalPose);
            else
                RelativePose = originalPose;
            end

            if ~hasView(this, ViewId1)
                error(message('vision:viewSet:missingViewId', ViewId1));
            end

            if ~hasView(this, ViewId2)
                error(message('vision:viewSet:missingViewId', ViewId2));
            end

            if fillMissingVars
                connTable = table(ViewId1, ViewId2, RelativePose, InformationMatrix);
            else
                % Return a table with only the columns specified by the
                % user
                unsetColumns = connParser.UsingDefaults;

                connTable = table(ViewId1, ViewId2);

                if ~ismember('relPose', unsetColumns)
                    connTable.RelativePose = RelativePose;
                end

                if ~ismember('InformationMatrix', unsetColumns)
                    connTable.InformationMatrix = InformationMatrix;
                end
            end
        end

        %------------------------------------------------------------------
        function checkPointCloud(this, ptCloud)

            validateattributes(ptCloud, {'pointCloud'}, {'scalar'}, this.ClassName, 'PointCloud');
        end

        %------------------------------------------------------------------
        function ptCloud = checkPointClouds(this, ptCloud)

            validateattributes(ptCloud, {'pointCloud'}, {'vector'}, this.ClassName, 'PointCloud');
        end
    end

    methods  (Access = public, Static, Hidden)
        %------------------------------------------------------------------
        function codegenClassName = matlabCodegenRedirect(~)
            codegenClassName = 'vision.internal.codegen.pc.pcviewset';
        end
    end

    methods (Hidden)
        %------------------------------------------------------------------
        % saveobj is implemented to ensure compatibility across releases by
        % converting the class to a struct prior to saving. It also
        % contains a version number, which can be used to customize load in
        % case the interface is updated.
        %------------------------------------------------------------------
        function that = saveobj(this)

            % this - object
            % that - struct

            that.Views       = this.Views;
            that.Connections = this.Connections;
            that.Version     = this.Version;
        end
    end

    methods (Static, Hidden)
        %------------------------------------------------------------------
        function this = loadobj(that)

            % that - struct
            % this - object

            this = pcviewset();

            this.Views          = that.Views;
            this.Connections    = that.Connections;

            if that.Version < 1.1 % Before 22b
                % Check if conversion to rigidtform3d is needed for
                % previous pcviewset objects
                this.Views = vision.internal.viewset.viewTableConvCheck(this.Views);
                this.Connections = vision.internal.viewset.connTableConvCheck(this.Connections);
            end
        end
    end
end