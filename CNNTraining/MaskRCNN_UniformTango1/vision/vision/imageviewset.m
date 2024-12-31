% imageviewset Object for managing data for structure-from-motion, visual odometry and visual SLAM
%   Use this object to store view attributes, such as feature descriptors,
%   feature points and absolute camera poses, and pairwise connection
%   attributes between views, such as point matches, relative camera poses
%   and information matrix. Also use this object to find point tracks used
%   by triangulateMultiview, bundleAdjustment and optimizePoses functions.
%
%   vSet = imageviewset() returns an imageviewset object. Add views and
%   connections to it using addView and addConnection.
%
%   imageviewset properties:
%   Views           - A table containing view attributes (read-only)
%   Connections     - A table containing connection attributes (read-only)
%   NumViews        - Number of views (read-only)
%   NumConnections  - Number of connections (read-only)
%
%   imageviewset methods:
%   addView             - Add a new view
%   updateView          - Update an existing view
%   deleteView          - Delete an existing view
%   hasView             - Check if a view exists
%   findView            - Find views associated with view IDs
%   addConnection       - Add a new connection between two views
%   updateConnection    - Update an existing connection
%   deleteConnection    - Delete an existing connection
%   hasConnection       - Check if a connection between two views exists
%   findConnection      - Find connections associated with view IDs
%   connectedViews      - Returns directly and indirectly connected views
%   poses               - Returns absolute poses associated with views
%   optimizePoses       - Optimize image view set poses
%   createPoseGraph     - Returns a pose graph
%   findTracks          - Find matched points across multiple views
%   plot                - Plot a view set
%
%
%   Example: Find point tracks across a sequence of images
%   ------------------------------------------------------
%   % Load images
%   imageDir = fullfile(toolboxdir('vision'), 'visiondata', ...
%    'structureFromMotion');
%   images = imageDatastore(imageDir);
%
%   % Compute features for the first image
%   I = rgb2gray(readimage(images, 1));
%   pointsPrev = detectSURFFeatures(I);
%   [featuresPrev, pointsPrev] = extractFeatures(I, pointsPrev);
%
%   % Create an imageViewSet object
%   vSet = imageviewset;
%   vSet = addView(vSet, 1, 'Features', featuresPrev, 'Points', pointsPrev);
%
%   % Compute features and matches for the rest of the images
%   for i = 2:numel(images.Files)
%     I = rgb2gray(readimage(images, i));
%     points = detectSURFFeatures(I);
%     [features, points] = extractFeatures(I, points);
%     vSet = addView(vSet, i, 'Features', features, 'Points', points);
%     pairsIdx = matchFeatures(featuresPrev, features);
%     vSet = addConnection(vSet, i-1, i, 'Matches', pairsIdx);
%     featuresPrev = features;
%   end
%
%   % Find point tracks
%   tracks = findTracks(vSet);
%
%   See also worldpointset, pcviewset, digraph.

% Copyright 2019-2023 The MathWorks, Inc.

%#codegen

classdef imageviewset < vision.internal.ViewSetBase

    properties (SetAccess = protected)
        %Views A table containing view attributes.
        %   A four-column table with columns 'ViewId', 'Features', 'Points'
        %   and 'AbsolutePose'.
        %
        %   ViewId          - A view identifier for the view specified as a
        %                     unique integer.
        %
        %   Features        - Feature vectors, specified as an M-by-N matrix
        %                     of M feature vectors or a binaryFeatures object.
        %
        %   Points          - Image points, specified as an M-by-2 matrix
        %                     of [x,y] coordinates or an M-element feature
        %                     point array of any of the feature point types
        %                     (cornerPoints, BRISKPoints, SIFTPoints,
        %                     SURFPoints, ORBPoints, KAZEPoints, MSERRegions).
        %
        %   AbsolutePose    - Absolute pose of the view, specified as a
        %                     rigidtform3d object.
        %
        %   See also worldpointset, pcviewset, table, rigidtform3d,
        %            extractFeatures.
        Views = table('Size', [0,4], ...
                      'VariableTypes', {'uint32', 'rigidtform3d', 'cell', 'cell'}, ...
                      'VariableNames',{'ViewId', 'AbsolutePose', 'Features', 'Points'});

        %Connections A table containing connection attributes.
        %   A five-column table with columns 'ViewId1', 'ViewId2',
        %   'RelativePose', 'InformationMatrix' and 'Matches'
        %
        %   ViewId1             - A view identifier for the first view.
        %
        %   ViewId2             - A view identifier for the second view.
        %
        %   RelativePose        - Relative pose of the second view with
        %                         respect to the first, specified as a
        %                         rigidtform3d object or an simtform3d
        %                         object. When specified as an simtform3d
        %                         object, the relative pose must represent
        %                         a 3-D similarity transformation.
        %
        %   InformationMatrix   - Information matrix for the measurement.
        %                         Information matrix represents uncertainty
        %                         of the measurement error and is inverse
        %                         of the covariance matrix. When RelativePose
        %                         is a rigidtform3d object, InformationMatrix
        %                         is a 6-by-6 matrix. When RelativePose is
        %                         an simtform3d object, InformationMatrix
        %                         is a 7-by-7 matrix.
        %
        %   Matches             - An M-by-2 matrix containing the indices
        %                         of matched feature points between two views.
        %
        %   See also worldpointset, pcviewset, table, rigidtform3d,
        %            simtform3d, matchFeatures.
        Connections = table('Size',[0,5], ...
                            'VariableTypes', {'uint32', 'uint32', 'cell', 'cell', 'cell'}, ...
                            'VariableNames', {'ViewId1', 'ViewId2', 'RelativePose', 'InformationMatrix', 'Matches'});
    end

    properties (GetAccess = ?matlab.unittest.TestCase, SetAccess = protected)
        FeatureGraph  = vision.internal.ViewSetFeatureGraph
    end

    properties (Access = ?matlab.unittest.TestCase)
        IncrementalBuilding (1, 1) {mustBeNumericOrLogical} = false
    end

    properties (GetAccess = public, Constant, Hidden)
        ClassName = mfilename;
    end

    properties (Constant, Access = protected)
        % 1.0   :  R2020a  first shipped version
        % 1.1   :  R2020b  enable incremental building of feature graph
        % 1.2   :  R2021a  enable similarity pose graph optimization
        % 1.3   :  R2022a  allow disabling incremental building of feature graph
        % 1.4   :  R2022b  convert rigid3d and affine3d poses to rigidtform3d and simtform3d
        Version   = 1.4;

        ExtraViewVariables          = {'Features', 'Points'};
        ExtraConnectionVariables    = {'Matches'};
    end

    methods      
        %------------------------------------------------------------------
        function this = addView(this, varargin)
        %addView Add a new view to the view set
        %   vSet = addView(vSet, viewId) adds a new view denoted by
        %   viewId to the view set.
        %
        %   vSet = addView(vSet, viewId, absPose) additionally
        %   specifies a rigidtform3d object specifying the absolute pose of
        %   the view.
        %
        %   vSet = addView(..., Name, Value) specifies additional
        %   name-value pair arguments as described below:
        %
        %   'Features'     Feature vectors, specified as an M-by-N matrix
        %                  of M feature vectors or a binaryFeatures object.
        %
        %   'Points'       Image points, specified as an M-by-2 matrix
        %                  of [x,y] coordinates or an M-element feature point
        %                  array of any of the feature point types (cornerPoints, 
        %                  BRISKPoints, SIFTPoints, SURFPoints, ORBPoints, 
        %                  KAZEPoints, MSERRegions).
        %
        %   vSet = addView(vSet, viewTable) adds a new view or a set of
        %   views specified as a table, viewTable. viewTable must
        %   contain a column named 'ViewId', and one or more columns
        %   named 'AbsolutePose', 'Features' and 'Points'.
        %
        %   Example
        %   -------
        %   % Create an empty imageviewset object
        %   vSet = imageviewset;
        %
        %   % Detect interest points in the image
        %   imageDir = fullfile(toolboxdir('vision'), 'visiondata', ...
        %      'structureFromMotion');
        %   I = imread(fullfile(imageDir, 'image1.jpg'));
        %   points = detectSURFFeatures(rgb2gray(I));
        %
        %   % Add a new view
        %   vSet = addView(vSet, 1, 'Points', points);
        %
        %   See also detectSURFFeatures, detectHarrisFeatures,
        %     detectMinEigenFeatures, detectFASTFeatures,
        %     detectBRISKFeatures, detectMSERFeatures, table, rigidtform3d.

            if istable(varargin{1})
                fillMissingVars = true;
                viewTable = checkViewTable(this, varargin{1}, fillMissingVars);
            else
                viewTable = parseViewInputs(this, varargin{:});
            end

            viewExists = hasView(this, viewTable.ViewId);
            if any( viewExists )
                existingViews = viewTable.ViewId(viewExists);
                error(message('vision:viewSet:viewIdAlreadyExists', existingViews(1)))
            end

            this.Views = [this.Views; viewTable];

            % Add nodes to the feature graph
            if this.IncrementalBuilding
                this.FeatureGraph = addNodes(this.FeatureGraph, viewTable);
            end
        end

        %------------------------------------------------------------------
        function this = updateView(this, varargin)
        %updateView Modify an existing view
        %   vSet = updateView(vSet, viewId, absPose) modifies an
        %   existing view denoted by viewId with the absolute pose
        %   absPose, specified as a rigidtform3d object.
        %
        %   View attributes can be specified using additional name-value
        %   pairs described below:
        %
        %   'Features'     Feature vectors, specified as an M-by-N matrix
        %                  of M feature vectors or a binaryFeatures object.
        %
        %   'Points'       Image points, specified as an M-by-2 matrix
        %                  of [x,y] coordinates or an M-element feature
        %                  point array of any of the feature point types
        %                  (cornerPoints, BRISKPoints, SIFTPoints, SURFPoints, 
        %                  ORBPoints, KAZEPoints, MSERRegions).
        %
        %   vSet = updateView(vSet, viewTable) modifies views specified
        %   in viewTable. viewTable must contain a column named 'ViewId',
        %   and one or more columns named 'AbsolutePose', 'Features'
        %   and 'Points'.
        %
        %   Example
        %   -------
        %   % Create an empty imageviewset object
        %   vSet = imageviewset;
        %
        %   % Detect interest points in the image
        %   imageDir = fullfile(toolboxdir('vision'), 'visiondata', ...
        %      'structureFromMotion');
        %   I = imread(fullfile(imageDir, 'image1.jpg'));
        %   points = detectSURFFeatures(rgb2gray(I));
        %
        %   % Add a new view
        %   vSet = addView(vSet, 1, 'Points', points);
        %
        %   % Update the absolute pose of the view
        %   absPose = rigidtform3d(eye(3),[0 0 1]);
        %   vSet = updateView(vSet, 1, absPose);
        %
        %   See also detectSURFFeatures, detectHarrisFeatures,
        %     detectMinEigenFeatures, detectFASTFeatures,
        %     detectBRISKFeatures, detectMSERFeatures, table, rigidtform3d.

            if istable(varargin{1})
                fillMissingVars = false;
                [viewTable, unsetColumns] = checkViewTable(this, varargin{1}, fillMissingVars);
                checkIfViewIsMissing(this, viewTable.ViewId);

                allViewIds = this.Views.ViewId;
                [viewIdsToUpdate, ~, ib] = intersect(viewTable.ViewId, allViewIds, 'stable');

                variableNames = viewTable.Properties.VariableNames;

                % Update the views using the view table
                this.Views(ib, variableNames) = viewTable;

                pointsChanged   = ~ismember('Points', unsetColumns);
                featuresChanged = ~ismember('Features', unsetColumns);
            else
                [viewTable, unsetColumns] = parseViewInputs(this, varargin{:});
                viewIdsToUpdate = viewTable.ViewId;
                checkIfViewIsMissing(this, viewIdsToUpdate);

                viewIdx = getViewIndex(this, viewIdsToUpdate);

                pointsChanged   = ~ismember('Points', unsetColumns);
                featuresChanged = ~ismember('Features', unsetColumns);
                absPosesChanged = ~ismember('absPose', unsetColumns);

                if featuresChanged
                    this.Views.Features(viewIdx) = viewTable.Features;
                end

                if pointsChanged
                    this.Views.Points(viewIdx) = viewTable.Points;
                end

                if absPosesChanged
                    this.Views.AbsolutePose(viewIdx) = viewTable.AbsolutePose;
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
        function this = deleteView(this, viewIds)
        %deleteView Remove an existing view
        %   vSet = deleteView(vSet, viewId) deletes an existing view or
        %   a set of views. viewId is either an integer or a vector
        %   containing the ids of the views to be deleted.
        %
        %   Example
        %   -------
        %   % Create an empty imageviewset object
        %   vSet = imageviewset;
        %
        %   % Detect interest points in the image
        %   imageDir = fullfile(toolboxdir('vision'), 'visiondata', ...
        %      'structureFromMotion');
        %   I = imread(fullfile(imageDir, 'image1.jpg'));
        %   points = detectSURFFeatures(rgb2gray(I));
        %
        %   % Add a new view
        %   vSet = addView(vSet, 1, 'Points', points);
        %
        %   % Delete the view
        %   vSet = deleteView(vSet, 1);
        %
        %   See also imageviewset/addView, detectSURFFeatures,
        %     detectHarrisFeatures, detectMinEigenFeatures,
        %     detectFASTFeatures, detectBRISKFeatures,
        %     detectMSERFeatures, table, rigidtform3d.

            viewIds = checkViewIds(this, viewIds);

            for viewId = viewIds'
                checkIfViewIsMissing(this, viewId);

                viewIdx = getViewIndex(this, viewId);
                this.Views(viewIdx, :) = [];

                if ~isempty(this.Connections)
                    connIdx = getConnectionIndexToAndFrom(this, viewId);
                    this.Connections(connIdx, :) = [];
                end
            end

            % Remove nodes in the feature graph. Any edges incident upon
            % the nodes are also removed.
            if this.IncrementalBuilding
                this.FeatureGraph = removeNodes(this.FeatureGraph, viewIds);
            end
        end

        %------------------------------------------------------------------
        function views = findView(this, viewIds)
        %findView Find views associated with view IDs
        %   views = findView(vSet, viewIds) returns a table containing
        %   view attributes that correspond to the view IDs in viewIds.
        %
        %   Example
        %   -------
        %   % Create an empty imageviewset object
        %   vSet = imageviewset;
        %
        %   % Read a pair of images
        %   imageDir = fullfile(toolboxdir('vision'), 'visiondata', ...
        %     'structureFromMotion');
        %   I1 = rgb2gray(imread(fullfile(imageDir, 'image1.jpg')));
        %   I2 = rgb2gray(imread(fullfile(imageDir, 'image2.jpg')));
        %
        %   % Detect interest points in the two images
        %   points1 = detectSURFFeatures(I1);
        %   points2 = detectSURFFeatures(I2);
        %
        %   % Extract feature descriptors
        %   [features1, validPoints1] = extractFeatures(I1, points1);
        %   [features2, validPoints2] = extractFeatures(I2, points2);
        %
        %   % Add the points to the imageviewset object
        %   vSet = addView(vSet, 1, 'Features', features1, 'Points', validPoints1);
        %   vSet = addView(vSet, 2, 'Features', features2, 'Points', validPoints2);
        %
        %   % Find the view that corresponds to viewId 1
        %   view = findView(vSet, 1)
        %
        %   % Check the feature points that correspond to viewId 1
        %   view.Points{:}
        %
        %   See also imageviewset/addView, imageviewset/findConnection,
        %       table, extractFeatures.

            views = findView@vision.internal.ViewSetBase(this, viewIds);
        end

        %------------------------------------------------------------------
        function this = addConnection(this, viewId1, viewId2, varargin)
        %addConnection Add a connection between existing views
        %   vSet = addConnection(vSet, viewId1, viewId2) adds a
        %   connection between views viewId1 and viewId2.
        %
        %   vSet = addConnection(vSet, viewId1, viewId2, relPose)
        %   additionally specifies the relative pose of viewId2 with
        %   respect to viewId1 as a rigidtform3d or an simtform3d
        %   object.
        %
        %   vSet = addConnection(vSet, viewId1, viewId2, relPose,
        %   infoMat) additionally specifies the information matrix
        %   associated with the connection. When relPose is a rigidtform3d
        %   object, infoMat is a 6-by-6 matrix. When relPose is an
        %   simtform3d object, infoMat is a 7-by-7 matrix.
        %
        %   vSet = addConnection(...,'Matches', featureMatches)
        %   additionally specifies the indices of matched points between
        %   two views specified as an M-by-2 matrix.
        %
        %   Note
        %   ----
        %   When the connection corresponds to a loop closure, use
        %   estimateGeometricTransform3D to estimate the 3-D similarity
        %   transformation between viewId1 and viewId2 from matched
        %   3-D point pairs.
        %
        %   Example
        %   -------
        %   % Create an empty imageviewset object
        %   vSet = imageviewset;
        %
        %   % Read a pair of images
        %   imageDir = fullfile(toolboxdir('vision'), 'visiondata', ...
        %     'structureFromMotion');
        %   I1 = rgb2gray(imread(fullfile(imageDir, 'image1.jpg')));
        %   I2 = rgb2gray(imread(fullfile(imageDir, 'image2.jpg')));
        %
        %   % Detect interest points in the two images
        %   points1 = detectSURFFeatures(I1);
        %   points2 = detectSURFFeatures(I2);
        %
        %   % Extract feature descriptors
        %   [features1, validPoints1] = extractFeatures(I1, points1);
        %   [features2, validPoints2] = extractFeatures(I2, points2);
        %
        %   % Add the points to the viewSet object
        %   vSet = addView(vSet, 1, 'Features', features1, 'Points', validPoints1);
        %   vSet = addView(vSet, 2, 'Features', features2, 'Points', validPoints2);
        %
        %   % Match features and store the matches
        %   indexPairs = matchFeatures(features1, features2);
        %   vSet = addConnection(vSet, 1, 2, 'Matches', indexPairs);
        %
        %   See also rigidtform3d, simtform3d, detectSURFFeatures,
        %            detectHarrisFeatures, detectMinEigenFeatures,
        %            detectFASTFeatures, detectMSERFeatures,
        %            detectBRISKFeatures, matchFeatures, bundleAdjustment,
        %            triangulateMultiview, estgeotform3d.

            connTable = parseConnectionInputs(this, viewId1, viewId2, varargin{:});

            if hasConnection(this, connTable.ViewId1, connTable.ViewId2)
                error(message('vision:viewSet:connectionAlreadyExists', ...
                              connTable.ViewId1, connTable.ViewId2));
            end

            this.Connections = [this.Connections; connTable];

            % Add edges in the feature graph
            if this.IncrementalBuilding
                this.FeatureGraph = addEdges(this.FeatureGraph, connTable);
            end
        end

        %------------------------------------------------------------------
        function conn = findConnection(this, viewIds1, viewIds2)
        %findConnection Find connections associated with view IDs
        %   conn = findConnection(vSet, viewIds1, viewIds2) returns a
        %   table containing connection attributes for connections
        %   between viewIds1 and viewIds2. viewIds1 and viewIds2 are
        %   M-element vectors of view IDs.
        %
        %   Example
        %   -------
        %   % Create an empty imageviewset object
        %   vSet = imageviewset;
        %
        %   % Add three views to the view set
        %   viewId1 = 10;
        %   viewId2 = 5;
        %   viewId3 = 2;
        %   vSet = addView(vSet, viewId1);
        %   vSet = addView(vSet, viewId2);
        %   vSet = addView(vSet, viewId3);
        %
        %   % Add a connection between viewId1 and viewId2
        %   relPose1 = rigidtform3d(eye(3), [0 0 1]);
        %   vSet = addConnection(vSet, viewId1, viewId2, relPose1);
        %
        %   % Add a connection between viewId2 and viewId3
        %   relPose2 = rigidtform3d(eye(3), [0 0 2]);
        %   vSet = addConnection(vSet, viewId2, viewId3, relPose2);
        %
        %   % Get the connection between viewId1 and viewId2
        %   conn = findConnection(vSet, viewId1, viewId2)
        %
        %   % Check the relative pose between viewId1 and viewId2
        %   conn.RelativePose{:}
        %
        %   See also imageviewset/addConnection, imageviewset/findView,
        %       table.

            conn = findConnection@vision.internal.ViewSetBase(this, ...
                                                              viewIds1, viewIds2);
        end

        %------------------------------------------------------------------
        function tf = hasView(this, viewIds)
        %hasView Check if a view exists
        %   tf = hasView(vSet, viewId) returns true if a view with ID
        %   viewId exists, and false otherwise.
        %
        %   Example
        %   -------
        %   % Create an empty imageviewset object
        %   vSet = imageviewset;
        %
        %   % Detect interest points in the image
        %   imageDir = fullfile(toolboxdir('vision'), 'visiondata', ...
        %      'structureFromMotion');
        %   I = imread(fullfile(imageDir, 'image1.jpg'));
        %   points = detectSURFFeatures(rgb2gray(I));
        %
        %   % Add a new view
        %   vSet = addView(vSet, 1, 'Points', points);
        %
        %   % Check if view with ID 1 exists
        %   hasView(vSet, 1)
        %
        %   % Check if view with ID 2 exists
        %   hasView(vSet, 2)
        %
        %   See also imageviewset/hasConnection.

            tf = hasView@vision.internal.ViewSetBase(this, viewIds);
        end

        %------------------------------------------------------------------
        function this = updateConnection(this, varargin)
        %updateConnection Modify an existing connection
        %   vSet = updateConnection(vSet, viewId1, viewId2, relPose)
        %   modifies an existing connection between views viewId1 and
        %   viewId2 with the relative pose relPose. relPose is a rigidtform3d
        %   or an simtform3d object.
        %
        %   vSet = updateConnection(vSet, viewId1, viewId2, relPose,
        %   infoMat) additionally specifies the information matrix
        %   associated with the connection. When relPose is a rigidtform3d
        %   object, infoMat is a 6-by-6 matrix. When relPose is an
        %   simtform3d object, infoMat is a 7-by-7 matrix.
        %
        %   vSet = updateConnection(..., 'Matches', featureMatches)
        %   additionally modifies the indices of matched points between
        %   two views specified as an M-by-2 matrix.
        %
        %   Example
        %   -------
        %   % Create an empty imageviewset object
        %   vSet = imageviewset;
        %
        %   % Read a pair of images
        %   imageDir = fullfile(toolboxdir('vision'), 'visiondata', ...
        %     'structureFromMotion');
        %   I1 = rgb2gray(imread(fullfile(imageDir, 'image1.jpg')));
        %   I2 = rgb2gray(imread(fullfile(imageDir, 'image2.jpg')));
        %
        %   % Detect interest points in the two images
        %   points1 = detectSURFFeatures(I1);
        %   points2 = detectSURFFeatures(I2);
        %
        %   % Extract feature descriptors
        %   [features1, validPoints1] = extractFeatures(I1, points1);
        %   [features2, validPoints2] = extractFeatures(I2, points2);
        %
        %   % Add the points to the viewSet object
        %   vSet = addView(vSet, 1, 'Features', features1, 'Points', validPoints1);
        %   vSet = addView(vSet, 2, 'Features', features2, 'Points', validPoints2);
        %
        %   % Match features and store the matches
        %   indexPairs = matchFeatures(features1, features2);
        %   vSet = addConnection(vSet, 1, 2, 'Matches', indexPairs);
        %
        %   % Store a relative pose between the views
        %   theta = 30; % degrees
        %   trans = [2, 3, 4];
        %   tform = rigidtform3d([0, 0, theta], trans);
        %   vSet = updateConnection(vSet, 1, 2, tform);
        %
        %   See also imageviewset/addConnection, rigidtform3d, simtform3d,
        %     detectSURFFeatures, detectHarrisFeatures, detectMinEigenFeatures,
        %     detectFASTFeatures, detectMSERFeatures, detectBRISKFeatures,
        %     matchFeatures, bundleAdjustment, triangulateMultiview.

            [connTable, unsetColumns] = parseConnectionInputs(this, varargin{:});

            checkIfConnectionIsMissing(this, connTable.ViewId1, connTable.ViewId2);

            idx = getConnectionIndex(this, connTable.ViewId1, connTable.ViewId2);

            poseTypeChanged = false;
            if ~ismember('relPose', unsetColumns)
                poseTypeChanged = ~strcmp(class(connTable.RelativePose{:}), ...
                                          class(this.Connections.RelativePose{idx}));
                this.Connections.RelativePose(idx) = connTable.RelativePose;
            end

            if ~ismember('infoMat', unsetColumns) || poseTypeChanged
                this.Connections.InformationMatrix(idx) = connTable.InformationMatrix;
            end

            if ~ismember('Matches', unsetColumns)
                oldMatches =  this.Connections.Matches(idx);
                this.Connections.Matches(idx) = connTable.Matches;

                % Update old edges in feature graph
                if this.IncrementalBuilding
                    this.FeatureGraph = updateEdges(this.FeatureGraph, connTable, oldMatches);
                end
            end
        end

        %------------------------------------------------------------------
        function this = deleteConnection(this, viewId1, viewId2)
        %deleteConnection Delete a connection between two views
        %   vSet = deleteConnection(vSet, viewId1, viewId2) deletes the
        %   connection between views with view identifiers viewId1 and
        %   viewId2.
        %
        %   Example
        %   -------
        %   % Create an empty imageviewset object
        %   vSet = imageviewset;
        %
        %   % Read a pair of images
        %   imageDir = fullfile(toolboxdir('vision'), 'visiondata', ...
        %     'structureFromMotion');
        %   I1 = rgb2gray(imread(fullfile(imageDir, 'image1.jpg')));
        %   I2 = rgb2gray(imread(fullfile(imageDir, 'image2.jpg')));
        %
        %   % Detect interest points in the two images
        %   points1 = detectSURFFeatures(I1);
        %   points2 = detectSURFFeatures(I2);
        %
        %   % Extract feature descriptors
        %   [features1, validPoints1] = extractFeatures(I1, points1);
        %   [features2, validPoints2] = extractFeatures(I2, points2);
        %
        %   % Add the points to the viewSet object
        %   vSet = addView(vSet, 1, 'Features', features1, 'Points', validPoints1);
        %   vSet = addView(vSet, 2, 'Features', features2, 'Points', validPoints2);
        %
        %   % Match features and store the matches
        %   indexPairs = matchFeatures(features1, features2);
        %   vSet = addConnection(vSet, 1, 2, 'Matches', indexPairs);
        %
        %   % Delete the connection between the views
        %   vSet = deleteConnection(vSet, 1, 2);
        %
        %   See also imageviewset/addConnection, imageviewset/updateConnection,
        %     detectSURFFeatures, detectHarrisFeatures, detectMinEigenFeatures,
        %     detectFASTFeatures, detectMSERFeatures, detectBRISKFeatures,
        %     matchFeatures, bundleAdjustment, triangulateMultiview.

            checkIfConnectionIsMissing(this, viewId1, viewId2);
            idx = getConnectionIndex(this, viewId1, viewId2);
            connTable = this.Connections(idx, :);
            this.Connections(idx, :) = [];

            % Remove edges in feature graph
            if this.IncrementalBuilding
                this.FeatureGraph = removeEdges(this.FeatureGraph, connTable);
            end
        end

        %------------------------------------------------------------------
        function tf = hasConnection(this, viewId1, viewId2)
        %hasConnection Check if a connection between two views exists
        %   tf = hasConnection(vSet, viewId1, viewId2) returns true if
        %   a connection between viewId1 and viewId2 exists, and false
        %   otherwise.
        %
        %   Example
        %   -------
        %   % Create an empty imageviewset object
        %   vSet = imageviewset;
        %
        %   % Add a pair of views
        %   vSet = addView(vSet, 1);
        %   vSet = addView(vSet, 2);
        %
        %   % Add a connection
        %   vSet = addConnection(vSet, 1, 2);
        %
        %   % Check if the connection exists
        %   tf = hasConnection(vSet, 1, 2)
        %
        %   See also imageviewset/hasView.

            tf = hasConnection@vision.internal.ViewSetBase(this, viewId1, viewId2);
        end

        %------------------------------------------------------------------
        function [viewTable, dist] = connectedViews(this, varargin)
        %connectedViews Returns directly and indirectly connected views
        %   viewTable = connectedViews(vSet, viewId) returns a table of
        %   views connected to the view viewId. The views can be directly
        %   or indirectly connected to the view viewId.
        %
        %   By default, only directly connected views are returned.
        %   The distance between any two directly connected views is
        %   equal to 1. To find indirectly connected views, specify a
        %   value larger than 1 for the 'MaxDistance' name-value argument.
        %
        %   [viewTable, dist] = connectedViews(...) also returns the
        %   distances between view viewId and each view in the view table.
        %
        %   [...] = connectedViews(..., Name, Value) specifies
        %   additional name-value pair arguments as described below:
        %
        %   'MaxDistance'    - A positive integer specifying the maximum
        %                      distance from the view viewId to the
        %                      connected views. The distance between any 
        %                      two directly connected views is equal to 1.
        %
        %                      Default: 1
        %
        %   'MinNumMatches'  - A nonnegative integer specifying the minimum
        %                      number of matched feature points for a
        %                      connection to be valid. When set to 0, all
        %                      connections in the connection table are valid. 
        %
        %                      Default: 0
        %
        %   Example
        %   -------
        %   % Create an empty imageviewset object
        %   vSet = imageviewset;
        %
        %   % Add several views
        %   pose1 = rigid3d;
        %   vSet = addView(vSet, 1, pose1, 'Features', rand(10, 3), ...
        %       'Points', rand(10 ,2));
        %   pose2 = rigid3d(eye(3), [1 0 0]);
        %   vSet = addView(vSet, 2, pose2, ...
        %       'Features', rand(10, 3), 'Points', rand(10 ,2));
        %   pose3 = rigid3d(eye(3), [2 0 0]);
        %   vSet = addView(vSet, 3, pose3, ...
        %       'Features', rand(10, 3), 'Points', rand(10 ,2));
        %   pose4 = rigid3d(eye(3), [0 1 0]);
        %   vSet = addView(vSet, 4, pose4, ...
        %       'Features', rand(10, 3), 'Points', rand(10 ,2));
        %   pose5 = rigid3d(eye(3), [0 -1 0]);
        %   vSet = addView(vSet, 5, pose5, ...
        %       'Features', rand(10, 3), 'Points', rand(10 ,2));
        %
        %   % Add connections
        %   vSet = addConnection(vSet, 1, 2, 'Matches', [1 2; 2 3; 3 4; 4 5]);
        %   vSet = addConnection(vSet, 2, 3, 'Matches', [1 2; 2 3; 3 4]);
        %   vSet = addConnection(vSet, 2, 4, 'Matches', [4 1]);
        %   vSet = addConnection(vSet, 3, 4, 'Matches', [4 1]);
        %   vSet = addConnection(vSet, 1, 5, 'Matches', [1 2; 4 1]);
        %
        %   % Display the view set with view IDs
        %   plot(vSet, 'ShowViewIds', 'on');
        %
        %   % Get the strongly connected views of view 2 with at least
        %   % 3 matched feature points
        %   viewTableStrong = connectedViews(vSet, 2, 'MinNumMatches', 3)
        %
        %   % Get views within distance 2 of view 3
        %   [viewTableNeaby, dist] = connectedViews(vSet, 3, 'MaxDistance', 2)
        %
        %   See also Views, Connections.

            narginchk(2, 7);
            [viewId, maxDistance, minNumMatches] = parseConnectedViewsInput(this, varargin{:}); 
            
            if maxDistance == 1  % Directly connected views
                [viewTable, connIdx]  = connectedViews@vision.internal.ViewSetBase(this, viewId);

                if minNumMatches ~= 0
                    isStrongConnection  = cellfun(@(x) size(x, 1) >= minNumMatches, ...
                        this.Connections.Matches(connIdx));
                    viewTable = viewTable(isStrongConnection, :);
                end

                dist = ones(height(viewTable), 1);
            else % Indirectly connected views
                if minNumMatches ~= 0
                    isStrongConnection  = cellfun(@(x) size(x, 1) >= minNumMatches, ...
                        this.Connections.Matches);
                    s = this.Connections.ViewId1(isStrongConnection);
                    t = this.Connections.ViewId2(isStrongConnection);
                else
                    s = this.Connections.ViewId1;
                    t = this.Connections.ViewId2;
                end

                % Check if the query view is not connected to any view
                if ~ismember(viewId, [s; t])
                    viewTable = table();
                    dist = [];
                    return
                end

                % Create a graph using view IDs
                G = graph(s, t);

                [nearestViewIds, dist] = G.nearest(double(viewId),maxDistance,'Method','unweighted');
                viewTable = findView(this, nearestViewIds);
            end
        end

        %------------------------------------------------------------------
        function tracks = findTracks(this, varargin)
        % findTracks Find matched points across multiple views
        %   tracks = findTracks(vSet) finds point tracks across multiple
        %   views. Each track contains 2-D projections of the same 3-D
        %   world point. tracks is an array of pointTrack objects.
        %
        %   tracks = findTracks(vSet, viewIds) finds point tracks
        %   across a subset of views, viewIds, specified as a vector
        %   of integers.
        %
        %   tracks = findTracks(..., 'MinTrackLength', minTrackLength)
        %   additionally uses a name-value pair to specify the minimum
        %   length of the tracks. minTrackLength is an integer equal to
        %   or greater than 2. The default value is 2.
        %
        %   Example: Find point tracks across a sequence of images
        %   ------------------------------------------------------
        %   % Load images
        %   imageDir = fullfile(toolboxdir('vision'), 'visiondata', ...
        %    'structureFromMotion');
        %   images = imageDatastore(imageDir);
        %
        %   % Compute features for the first image
        %   I = rgb2gray(readimage(images, 1));
        %   pointsPrev = detectSURFFeatures(I);
        %   [featuresPrev, pointsPrev] = extractFeatures(I, pointsPrev);
        %
        %   % Create an imageviewset object
        %   vSet = imageviewset;
        %   vSet = addView(vSet, 1, 'Points', pointsPrev);
        %
        %   % Compute features and matches for the rest of the images
        %   for i = 2:numel(images.Files)
        %    I = rgb2gray(readimage(images, i));
        %    points = detectSURFFeatures(I);
        %    [features, points] = extractFeatures(I, points);
        %    vSet = addView(vSet, i, 'Features', features, 'Points', points);
        %    pairsIdx = matchFeatures(featuresPrev, features);
        %    vSet = addConnection(vSet, i-1, i, 'Matches', pairsIdx);
        %    featuresPrev = features;
        %  end
        %
        %  % Find point tracks
        %  tracks = findTracks(vSet);
        %
        %  See also detectSURFFeatures, detectHarrisFeatures, detectMinEigenFeatures,
        %    detectFASTFeatures, detectBRISKFeatures, detectMSERFeatures,
        %    pointTrack, matchFeatures, bundleAdjustment, triangulateMultiview.

            [viewIds, minTrackLength] = parseFindTracksInputs(this, varargin{:});

            if isempty(this.Connections)
                tracks = pointTrack.empty();
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
        function sensorPoses = poses(this, varargin)
        %poses Returns absolute poses associated with views
        %   sensorPoses = poses(vSet) returns absolute poses associated
        %   with the views contained in the view set. sensorPoses is a
        %   table containing two columns 'ViewId' and 'AbsolutePose'.
        %
        %   sensorPoses = poses(vSet, viewIds) returns the absolute
        %   poses associated with a subset of views specified by
        %   viewIds, a vector of integers.
        %
        %   Example
        %   -------
        %   % Load images
        %   imageDir = fullfile(toolboxdir('vision'), 'visiondata', ...
        %    'structureFromMotion');
        %   images = imageDatastore(imageDir);
        %
        %   % Compute features for the first image
        %   I = rgb2gray(readimage(images, 1));
        %   pointsPrev = detectSURFFeatures(I);
        %   [featuresPrev, pointsPrev] = extractFeatures(I, pointsPrev);
        %
        %   % Create an imageviewset object
        %   vSet = imageviewset;
        %   vSet = addView(vSet, 1, 'Points', pointsPrev);
        %
        %   % Compute features and matches for the rest of the images
        %   for i = 2:numel(images.Files)
        %    I = rgb2gray(readimage(images, i));
        %    points = detectSURFFeatures(I);
        %    [features, points] = extractFeatures(I, points);
        %    vSet = addView(vSet, i, 'Features', features, 'Points', points);
        %    pairsIdx = matchFeatures(featuresPrev, features);
        %    vSet = addConnection(vSet, i-1, i, 'Matches', pairsIdx);
        %    featuresPrev = features;
        %   end
        %
        %   % Get camera poses
        %   sensorPoses = poses(vSet)
        %
        %   See also table.

            sensorPoses = poses@vision.internal.ViewSetBase(this, varargin{:});
        end

        %------------------------------------------------------------------
        function G = createPoseGraph(this)
        %createPoseGraph returns a pose graph
        %   createPoseGraph creates a pose graph from the view set. The
        %   pose graph can be used for inspection, modification,
        %   visualization and pose graph optimization.
        %
        %   G = createPoseGraph(vSet) returns a pose graph derived from
        %   the views and connections in the view set vSet. G is a
        %   digraph object with nodes corresponding to views and edges
        %   corresponding to connections. The weight of an edge in G is
        %   computed as the number of matched feature points in the
        %   Connections table.
        %
        %   Notes
        %   -----
        %   - The EndNodes of the Edges in the digraph G correspond to
        %     indices into the Views table, not ViewIds.
        %   - Use optimizePoseGraph function to optimize the pose
        %     graph. Use of this function requires that you have the
        %     Navigation Toolbox(TM).
        %
        %   Example: Create and visualize pose graph from imageviewset
        %   ----------------------------------------------------------
        %   % Create an image view set
        %   vSet = imageviewset;
        %
        %   % Add 4 views specifying absolute poses
        %   rot = eye(3); % identity rotation
        %   vSet = addView(vSet, 1, rigidtform3d(rot, [ 0 0 0]));
        %   vSet = addView(vSet, 2, rigidtform3d(rot, [ 3 0 0]));
        %   vSet = addView(vSet, 3, rigidtform3d(rot, [ 8 0 0]));
        %   vSet = addView(vSet, 4, rigidtform3d(rot, [10 0 0]));
        %
        %   % Add 3 exact odometry connections specifying relative poses
        %   vSet = addConnection(vSet, 1, 2, rigidtform3d(rot, [3 0 0]));
        %   vSet = addConnection(vSet, 2, 3, rigidtform3d(rot, [5 0 0]));
        %   vSet = addConnection(vSet, 3, 4, rigidtform3d(rot, [2 0 0]));
        %
        %   % Add 1 loop closure connection
        %   vSet = addConnection(vSet, 4, 1, rigidtform3d(rot, [-9 0 0]));
        %
        %   % Create a pose graph
        %   G = createPoseGraph(vSet);
        %
        %   % Plot pose graph and label graph edges
        %   hG = plot(G);
        %   labeledge(hG, 1:3, 'odometry')
        %   labeledge(hG, 4, 'loop')
        %   highlight(hG, 'Edges', 4, 'EdgeColor', 'r')
        %
        %   See also digraph, optimizePoseGraph.

            weight = cellfun(@(x)size(x, 1), this.Connections.Matches);
            G = createPoseGraph@vision.internal.ViewSetBase(this, weight);
        end

        %------------------------------------------------------------------
        function varargout = optimizePoses(this, varargin)
        %optimizePoses Optimize image view set poses
        %   optimizePoses optimizes the absolute poses for Views in the
        %   view set using the relative pose constraints established by
        %   Connections using pose graph optimization. This can be used
        %   to correct drift in odometry after detecting loop closures.
        %   The relative poses of loop closures are usually specified
        %   as simtform3d objects representing 3-D similarity
        %   transformations. The relative poses of regular Connections
        %   are specified as rigidtform3d objects representing 3-D rigid
        %   transformations.
        %
        %   vSetOptim = optimizePoses(vSet) returns an image view set
        %   whose absolute poses are optimized. vSet and vSetOptim are
        %   imageviewset objects.
        %
        %   [vSetOptim, poseScale] = optimizePoses(vSet) also returns
        %   the scales associated with the optimized absolute poses.
        %   This output is available only when the RelativePose of
        %   at least one connection is represented as an simtform3d
        %   object.
        %
        %   vSetOptim = optimizePoses(vSet, minNumMatches) specifies
        %   minNumMatches, the minimum number of matched feature points
        %   in a connection for it to be included in optimization.
        %
        %   vSetOptim = optimizePoses(..., Name, Value) specifies
        %   name-value pair arguments as described below:
        %
        %   'MaxIterations' A positive integer specifying the maximum
        %                   number of iterations before optimization is
        %                   terminated. Increase this value at the
        %                   expense of speed for more accurate results.
        %
        %                   Default: 300
        %
        %   'Tolerance'     A positive scalar specifying the tolerance
        %                   of the optimization cost function. If the
        %                   cost function changes by less than this
        %                   value between two iterations, optimization
        %                   is terminated.
        %
        %                   Default: 1e-8
        %
        %
        %   'Verbose'       Set true to display progress information.
        %
        %                   Default: false
        %
        %   Notes
        %   -----
        %   - The optimizePoses method uses the Levenberg Marquardt
        %     optimization algorithm with sparse Cholesky factorization
        %     from the g2o graph optimization library.
        %   - When the RelativePose of all the connections are represented
        %     as rigidtform3d objects, the optimization is performed over
        %     3-D rigid transformations. Otherwise, the optimization is
        %     performed over 3-D similarity transformations.
        %   - The optimizePoses function holds the first view fixed.
        %
        %   Example
        %   -------
        %   % Create a view set
        %   vSet = imageviewset;
        %
        %   % Add 4 nodes, specifying absolute poses
        %   absPoses = repelem(rigidtform3d, 4, 1);
        %
        %   absPoses(1).Translation = [ 0   0 0];
        %   absPoses(2).Translation = [ 1   0 0];
        %   absPoses(3).Translation = [ 2   0 0];
        %   absPoses(4).Translation = [ 0.1 0 0];
        %
        %   vSet = addView(vSet, 1, absPoses(1));
        %   vSet = addView(vSet, 2, absPoses(2));
        %   vSet = addView(vSet, 3, absPoses(3));
        %   vSet = addView(vSet, 4, absPoses(4));
        %
        %   % Add 4 edges - 3 odometry and 1 loop closure
        %   relPoses = repelem(rigidtform3d, 4, 1);
        %
        %   relPoses(1).Translation = [ 1   0 0];
        %   relPoses(2).Translation = [ 1   0 0];
        %   relPoses(3).Translation = [-1.9 0 0];
        %   relPoses(4).Translation = [ 0.2 0 0];
        %
        %   vSet = addConnection(vSet, 1, 2, relPoses(1)); % odometry
        %   vSet = addConnection(vSet, 2, 3, relPoses(2)); % odometry
        %   vSet = addConnection(vSet, 3, 4, relPoses(3)); % odometry
        %   vSet = addConnection(vSet, 4, 1, relPoses(4)); % loop closure
        %
        %   % Optimize view set
        %   vSetOptim = optimizePoses(vSet);
        %
        %   % Print original and optimized locations
        %   disp('Original absolute translations:')
        %   disp(vertcat(vSet.Views.AbsolutePose.Translation))
        %
        %   disp('Optimized absolute translations:')
        %   disp(vertcat(vSetOptim.Views.AbsolutePose.Translation))
        %
        %   See also createPoseGraph.

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
                if max(bins)>1
                    error(message('vision:viewSet:disconnectedGraph'))
                end
            end

            poseTable    = vision.internal.optimizePoses(G, varargin{1+isMinMatchesSyntax:end});
            varargout{1} = updateView(this, poseTable(:, 1:2)); % ViewId and AbsolutePose

            if ~ismember('Scale', poseTable.Properties.VariableNames)
                nargoutchk(0,1);
            else
                nargoutchk(0,2);
                varargout{2} =  poseTable.Scale;
            end
        end

        %------------------------------------------------------------------
        function varargout = plot(this, varargin)
        %plot Display view set
        %   plot(vSet) plots the view set views and connections.
        %
        %   h = plot(vSet) additionally returns a GraphPlot object. Use
        %   methods and properties of the object to inspect and adjust
        %   the plotted graph.
        %
        %   plot(..., Name, Value) specifies additional name-value pair
        %   arguments described below:
        %
        %   'Parent'        Handle to an axes on which to display the
        %                   view set.
        %
        %                   Default: gca (current axes)
        %
        %   'ShowViewIds'   String 'on' or 'off', specifying whether
        %                   view ids should be displayed.
        %
        %                   Default: 'off'
        %
        %   Example
        %   -------
        %   % Create an empty image view set
        %   vSet = imageviewset;
        %
        %   % Define 3 relative poses
        %   relPoses = repelem(rigidtform3d, 3, 1);
        %   relPoses(1).Translation = [3 0 0];
        %   relPoses(2).Translation = [5 0 0];
        %   relPoses(3).Translation = [2 0 0];
        %
        %   % Accumulate absolute poses
        %   absPoses = repelem(rigidtform3d, 4, 1);
        %   absPoses(2).A = absPoses(1).A * relPoses(1).A;
        %   absPoses(3).A = absPoses(2).A * relPoses(2).A;
        %   absPoses(4).A = absPoses(3).A * relPoses(3).A;
        %
        %   % Add 4 views
        %   vSet = addView(vSet, 1, absPoses(1));
        %   vSet = addView(vSet, 2, absPoses(2));
        %   vSet = addView(vSet, 3, absPoses(3));
        %   vSet = addView(vSet, 4, absPoses(4));
        %
        %   % Add 3 connections
        %   vSet = addConnection(vSet, 1, 2, relPoses(1));
        %   vSet = addConnection(vSet, 2, 3, relPoses(2));
        %   vSet = addConnection(vSet, 3, 4, relPoses(3));
        %
        %   % Display the view set with view IDs
        %   plot(vSet, 'ShowViewIds', 'on')
        %
        %   See also digraph/plot, view.

            [varargout{1:nargout}] = plot@vision.internal.ViewSetBase(this, varargin{:});
        end
    end

    methods (Access = protected)
        %------------------------------------------------------------------
        % Utility Functions
        %------------------------------------------------------------------
        function [this, wipedConnTable] = wipeMatches(this, viewIds)

            wipedConnTable = [];

            if isempty(this.Connections)
                return;
            end

            displayWarning = false;
            for id = viewIds(:)'
                idx = getConnectionIndexToAndFrom(this, id);
                % idx is a logical index. Use find to convert it to numeric
                % index.
                for j = find(idx')
                    if ~isempty(this.Connections.Matches{j})
                        displayWarning = true;
                    end
                    wipedConnTable = [wipedConnTable; this.Connections(j,:)]; %#ok<AGROW>
                    this.Connections.Matches{j} = [];
                end
            end

            if displayWarning
                warning(message('vision:viewSet:deletingMatches'));
            end
        end

        %------------------------------------------------------------------
        % Input Parsing
        %------------------------------------------------------------------
        function [viewTable, unsetColumns] = parseViewInputs(this, varargin)

            persistent viewParser
            if isempty(viewParser)
                viewParser = inputParser;
                viewParser.FunctionName = this.ClassName;

                addRequired(viewParser, 'viewId', @this.checkViewId);
                addOptional(viewParser, 'absPose', rigidtform3d, @(tform)this.checkPose(tform, 'absPose'));
                addParameter(viewParser,'Features', [], @this.checkFeatures);
                addParameter(viewParser,'Points',   [], @this.checkPoints);
            end

            parse(viewParser, varargin{:});

            ViewId          = viewParser.Results.viewId;
            originalPose    = viewParser.Results.absPose;
            Features        = {viewParser.Results.Features};
            Points          = {viewParser.Results.Points};

            % Check if absolute pose is a rigid3d object and if so, 
            % convert it to a rigidtform3d object
            if isa(originalPose, 'rigid3d')
                AbsolutePose = vision.internal.viewset.rigid3dConvert(originalPose);
            else
                AbsolutePose = originalPose;
            end

            viewTable = table(ViewId, AbsolutePose, Features, Points);

            % Returning usingDefaults to be used in updateView to determine
            % which columns should not be updated.
            unsetColumns = viewParser.UsingDefaults;
        end
        
        function [viewId, maxDist, minNumMatches] = parseConnectedViewsInput(this, varargin)
            persistent connParser
            if isempty(connParser)
                connParser = inputParser;
                connParser.FunctionName = this.ClassName;

                addRequired(connParser, 'viewId', @this.checkViewId);
                addOptional(connParser, 'minNumMatchesPositional',0, @this.checkMinNumMatches);
                addParameter(connParser,'MinNumMatches', 0, @this.checkMinNumMatches);
                addParameter(connParser,'MaxDistance', 1, @this.checkMaxDistance);
            end
            parse(connParser, varargin{:});

            viewId  = connParser.Results.viewId;
            maxDist = connParser.Results.MaxDistance;

            unsetValues = connParser.UsingDefaults;
            
            % Errors out if maximum distance is specified using both the
            % positional input and the Name-Value pair
            if isempty(unsetValues) || isequal(unsetValues, {'MaxDistance'})
                error(message('vision:viewSet:minNumMatchesDuplicated'));
            elseif ismember('minNumMatchesPositional', unsetValues)
                minNumMatches = connParser.Results.MinNumMatches;
            else
                minNumMatches = connParser.Results.minNumMatchesPositional;
            end
        end

        %------------------------------------------------------------------
        function checkMatchesOutOfBounds(this, matches, viewId1, viewId2)
        % Check that match indices are valid.
            view1 = getView(this, viewId1);
            view2 = getView(this, viewId2);

            points1 = view1.Points{1};
            points2 = view2.Points{1};

            areMatchesOutOfBounds = ...
                max(matches(:, 1)) > size(points1, 1) || ...
                max(matches(:, 2)) > size(points2, 1);
            if areMatchesOutOfBounds
                error(message('vision:viewSet:matchIdxOutOfBounds'));
            end
        end

        %------------------------------------------------------------------
        function [viewTable, unsetColumns] = checkViewTable(this, viewTable, fillMissingVars)

            variableNames = viewTable.Properties.VariableNames;

            hasRequiredVariables = isequal(variableNames{1}, 'ViewId');
            if ~hasRequiredVariables
                error(message('vision:viewSet:requiredColumnsMissing', ...
                              'viewTable', 'ViewId'));
            end

            hasAbsolutePose = any(ismember(variableNames(2:end), 'AbsolutePose'));
            hasPoints       = any(ismember(variableNames(2:end), 'Points'));
            hasFeatures     = any(ismember(variableNames(2:end), 'Features'));

            hasOneOptionalVariables   = width(viewTable)==2 && (hasAbsolutePose || hasPoints || hasFeatures);
            hasTwoOptionalVariables   = width(viewTable)==3 && hasAbsolutePose && (hasPoints || hasFeatures);
            hasThreeOptionalVariables = width(viewTable)==4 && hasAbsolutePose && hasPoints && hasFeatures;

            if width(viewTable)>1 && ~(hasOneOptionalVariables || hasTwoOptionalVariables || hasThreeOptionalVariables)
                error(message('vision:viewSet:optionalColumnsInvalid', ...
                              'viewTable', 'ViewId', 'AbsolutePose, Features, Points'));
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

            if hasPoints
                viewTable.Points   = checkPointsArray(this, viewTable.Points);
            elseif fillMissingVars
                viewTable.Points   = repmat({zeros(0, 0)}, height(viewTable), 1);
            end

            if hasFeatures
                viewTable.Features = checkFeaturesArray(this, viewTable.Features);
            elseif fillMissingVars
                viewTable.Features = repmat({zeros(0, 0)}, height(viewTable), 1);
            end

            unsetColumns = this.ExtraViewVariables(~[hasFeatures, hasPoints]);
        end

        %------------------------------------------------------------------
        function [connTable, unsetColumns] = parseConnectionInputs(this, varargin)

            persistent connParser
            if isempty(connParser)
                connParser = inputParser;
                connParser.FunctionName = this.ClassName;

                addRequired(connParser, 'viewId1', @this.checkViewId);
                addRequired(connParser, 'viewId2', @this.checkViewId);
                addOptional(connParser, 'relPose', rigidtform3d, @(tform)this.checkPose(...
                    tform, 'relPose', {'rigidtform3d','simtform3d','rigid3d','affine3d'}));
                addOptional(connParser, 'infoMat', eye(6), @(infoMat)validateattributes(...
                    infoMat, {'single','double'}, {'real', '2d'}, this.ClassName, 'infoMat'));
                addParameter(connParser,'Matches', zeros(0, 2, 'uint32'), @this.checkMatches);
            end

            parse(connParser, varargin{:});

            ViewId1             = connParser.Results.viewId1;
            ViewId2             = connParser.Results.viewId2;
            originalPose        = connParser.Results.relPose;
            InformationMatrix   = connParser.Results.infoMat;
            Matches             = connParser.Results.Matches;

            % Check if relative pose is a rigid3d or affine3d object and
            % if so, convert it to a rigidtform3d or simtform3d object
            if isa(originalPose, 'rigid3d')
                RelativePose = rigidtform3d(originalPose.T');
            elseif isa(originalPose, 'affine3d')
                RelativePose = simtform3d(originalPose.T');
            else
                RelativePose = originalPose;
            end

            if ~hasView(this, ViewId1)
                error(message('vision:viewSet:missingViewId', ViewId1));
            end

            if ~hasView(this, ViewId2)
                error(message('vision:viewSet:missingViewId', ViewId2));
            end

            isSimilarityConnection = isa(RelativePose, 'simtform3d');
            infoMatDoF = 6 + isSimilarityConnection;

            if isSimilarityConnection && ~isSimilarity(RelativePose)
                error(message('vision:viewSet:notSimilarityTransform', 'relPose'));
            end
            RelativePose = {RelativePose};

            unsetColumns = connParser.UsingDefaults;

            if ismember('infoMat', unsetColumns)
                if isSimilarityConnection
                    InformationMatrix = eye(infoMatDoF);
                end
            else
                % Ideally only size should be checked
                validateattributes(InformationMatrix, {'single','double'}, ...
                                   {'size', [infoMatDoF, infoMatDoF]}, this.ClassName, 'infoMat');
            end
            InformationMatrix = {InformationMatrix};

            if ~ismember('Matches', unsetColumns)
                if isempty(Matches)
                    Matches = zeros(0, 2, 'uint32');
                else
                    checkMatchesOutOfBounds(this, Matches, ViewId1, ViewId2);
                end
            end
            Matches = {Matches};

            connTable = table(ViewId1, ViewId2, RelativePose, InformationMatrix, Matches);
        end

        %------------------------------------------------------------------
        function [viewIds, minTrackLength]  = parseFindTracksInputs(this, varargin)

            trackParser = inputParser;
            trackParser.FunctionName = this.ClassName;

            addOptional(trackParser, 'viewIds', this.Views.ViewId);
            addParameter(trackParser,'MinTrackLength', 2, @this.checkMinTrackLength);

            parse(trackParser, varargin{:});

            viewIds        = this.checkViewIds(trackParser.Results.viewIds);
            minTrackLength = trackParser.Results.MinTrackLength;
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
                for i = 1:numel(featuresArray)
                    vision.internal.inputValidation.checkFeatures(...
                        vertcat(featuresArray{i}), this.ClassName, 'Features');
                end
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

            if this.NumViews >= 2 && minTrackLength > this.NumViews
                error(message('vision:viewSet:exceedNumViews', 'MinTrackLength'));
            end
        end

        %-----------------------------------------------------------------
        function checkMinNumMatches(this, minNumMatches)
            validateattributes(minNumMatches, {'numeric'}, ...
                               {'scalar', 'nonnegative', 'real', 'integer'}, ...
                               this.ClassName, 'MinNumMatches');
        end
        
        %-----------------------------------------------------------------
        function checkMaxDistance(this, maxDist)
            validateattributes(maxDist, {'numeric'}, ...
                {'scalar', 'positive', 'real', 'integer'}, ...
                this.ClassName, 'MaxDistance');
        end

        %-----------------------------------------------------------------
        function checkMatches(this, matches)
            if ~isempty(matches)
                validateattributes(matches, {'numeric'}, ...
                                   {'nonsparse', '2d', 'ncols', 2, 'integer', 'positive'}, ...
                                   this.ClassName, 'Matches');
            end
        end
    end

    methods  (Access = public, Static, Hidden)
        %------------------------------------------------------------------
        function codegenClassName = matlabCodegenRedirect(~)
            codegenClassName = 'vision.internal.codegen.imageviewset.imageviewset';
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

            that.Views        = this.Views;
            that.Connections  = this.Connections;
            that.Version      = this.Version;
            that.FeatureGraph = this.FeatureGraph;
        end
    end

    methods (Static, Hidden)
        %------------------------------------------------------------------
        function this = loadobj(that)

        % that - struct
        % this - object

            this = imageviewset();

            this.Views       = that.Views;
            this.Connections = that.Connections;

            if that.Version < 1.2 % Before 21a
                this.Connections.RelativePose = num2cell(this.Connections.RelativePose);
            end

            if that.Version > 1.0 % After 20a
                this.FeatureGraph  = that.FeatureGraph;
            else
                % Loading an earlier version requires reconstructing the
                % feature graph using views and connections
                this.FeatureGraph = addNodes(this.FeatureGraph, that.Views);
                this.FeatureGraph = addEdges(this.FeatureGraph, that.Connections);
            end

            if that.Version < 1.4 % Before 22b
                % Check if conversion to rigidtform3d or simtform3d is needed
                % for previous imageviewset objects
                this.Views = vision.internal.viewset.viewTableConvCheck(this.Views);
                this.Connections = vision.internal.viewset.connTableConvCheck(this.Connections);
            end
        end
    end
end
