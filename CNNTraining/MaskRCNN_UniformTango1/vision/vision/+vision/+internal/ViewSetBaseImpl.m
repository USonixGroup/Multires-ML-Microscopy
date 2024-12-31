classdef (Hidden)ViewSetBaseImpl
% ViewSetBaseImpl class is the interface class for simulation and
% codegen version of ViewSet

% Copyright 2021-2023 The MathWorks, Inc.
%#codegen

    properties (Abstract, GetAccess = public, Constant, Hidden)
        %ClassName
        %   Name of the concrete class, used for error messages
        ClassName
    end
    properties (Abstract, Access = protected, Constant)
        %Version
        %   Version field used for forward compatibility
        Version

        %ExtraViewVariables
        %   Variables in the Views table not needed for building the pose
        %   graph
        ExtraViewVariables

        %ExtraConnectionVariables
        %   Variables in the Connections table not needed for building the
        %   pose graph
        ExtraConnectionVariables
    end

    methods

        %------------------------------------------------------------------
        % Check the existence of view
        %------------------------------------------------------------------
        function tf = hasView(this, viewIds)
            viewIds = checkViewIds(this, viewIds);
            if isSimMode
                tf = vision.internal.inputValidation.checkIfHasView(...
                    this.Views, viewIds);
            else
                tf = vision.internal.inputValidation.checkIfHasView(...
                    this.ViewId, viewIds);
            end
        end

        %------------------------------------------------------------------
        % Check if a connection between two views exists
        %-------------------------------------------------------------------
        function tf = hasConnection(this, viewId1, viewId2)

            viewId1 = checkViewId(this, viewId1);
            viewId2 = checkViewId(this, viewId2);

            hasView1 = any(hasView(this, viewId1), 'all');
            hasView2 = any(hasView(this, viewId2), 'all');

            tf = hasView1 && hasView2 && any((getConnectionIndex(this, viewId1, viewId2)), 'all');
        end
    end

    methods (Access = protected)
        %------------------------------------------------------------------
        % Get the view index value
        %------------------------------------------------------------------
        function viewIdx = getViewIndex(this, viewId)

            viewIds = this.Views.ViewId;
            if isempty(coder.target)
                viewIdx = find(viewIds == viewId(:));
            else
                [~, viewIdx] = intersect(viewIds, cast(viewId, 'like', viewIds), 'stable');
            end
        end

        %------------------------------------------------------------------
        % Get the connections index values
        %------------------------------------------------------------------
        function connIdx = getConnectionIndex(this, viewId1, viewId2)

            if isSimMode
                if isempty(this.Connections)
                    connIdx = [];
                else
                    connIdx = (viewId1==this.Connections.ViewId1) & ...
                              (viewId2==this.Connections.ViewId2);
                end
            else
                connIdx = zeros(1, 0);
                connectionsViewId1 = this.ViewId1;
                connectionsViewId2 = this.ViewId2;
                for i = 1:numel(connectionsViewId1)
                    if ((viewId1(1) == connectionsViewId1(i)) && (viewId2(1) == connectionsViewId2(i)))
                        connIdx = i;
                    end
                end
            end
        end

        %------------------------------------------------------------------
        % Get the connections index values from both sides
        %------------------------------------------------------------------
        function connIdx = getConnectionIndexToAndFrom(this, viewId)

            if isSimMode
                viewIds1 = this.Connections.ViewId1;
                viewIds2 = this.Connections.ViewId2;
                connIdx = viewIds1==viewId | viewIds2==viewId;
            else

                coder.varsize('connIdx');
                connIdx = [];
                viewIds1 = this.ViewId1;
                viewIds2 = this.ViewId2;
                if(~isempty(viewIds1))
                    for i = 1:numel(viewIds1)
                        if(viewId == viewIds1(i)  || ...
                           viewId == viewIds2(i))
                            connIdx = [connIdx i]; %#ok
                        end
                    end
                end
            end
        end
    end

    %----------------------------------------------------------------------
    % Validation
    %----------------------------------------------------------------------
    methods (Access = protected)
        %------------------------------------------------------------------
        % Validate the viewIds
        %------------------------------------------------------------------
        function viewIds = checkViewIds(this, viewIds)

            viewIds = vision.internal.inputValidation.checkViewIds(...
                viewIds, false, this.ClassName, 'viewIds');
        end

        %------------------------------------------------------------------
        % Validate the viewId
        %------------------------------------------------------------------
        function viewId = checkViewId(this, viewId)

            viewId = vision.internal.inputValidation.checkViewIds(...
                viewId, true, this.ClassName, 'viewId');
        end

        %------------------------------------------------------------------
        % Check for the missing view
        %------------------------------------------------------------------
        function checkIfViewIsMissing(this, viewId)
            viewId = uint32(viewId);
            vision.internal.inputValidation.checkIfViewIsMissing(...
                this.Views.ViewId, viewId);
        end

        %------------------------------------------------------------------
        % Check the connection missing
        %------------------------------------------------------------------
        function checkIfConnectionIsMissing(this, viewId1, viewId2)
            missingConnection = ~hasConnection(this, viewId1, viewId2);
            coder.internal.errorIf(missingConnection, ...
                                   'vision:viewSet:missingConnection', viewId1(1), viewId2(1));
        end

        %------------------------------------------------------------------
        % Validate the poses
        %------------------------------------------------------------------
        function checkPoses(this, tform, poseName)

            validateattributes(tform, {'rigidtform3d','simtform3d','rigid3d'}, {'vector'}, ...
                               this.ClassName, poseName);
        end

        %------------------------------------------------------------------
        % Validate the pose
        %------------------------------------------------------------------
        function checkPose(this, tform, poseName, varargin)

            if nargin > 3
                type = varargin{1};
            else
                type = {'rigidtform3d','simtform3d','rigid3d'};
            end
            validateattributes(tform, type, {'scalar'},this.ClassName, poseName);
        end

        %------------------------------------------------------------------
        % Validate the pointCloud
        %------------------------------------------------------------------
        function checkPointCloud(this, ptCloud)

            validateattributes(ptCloud, {'pointCloud'}, {'scalar'},...
                               this.ClassName, 'PointCloud');
        end

        %------------------------------------------------------------------
        % Validate the information matrix
        %------------------------------------------------------------------
        function checkInfoMat(this, infoMat, varargin)
            if nargin > 2
                sz = varargin{1};
            else
                sz = [6, 6];
            end

            validateattributes(infoMat, {'single','double'}, ...
                               {'real','size', sz}, this.ClassName, 'infoMat');
        end
    end

end

%------------------------------------------------------------------
function tf = isSimMode()
% Check if simulation mode
    tf = isempty(coder.target);
end
