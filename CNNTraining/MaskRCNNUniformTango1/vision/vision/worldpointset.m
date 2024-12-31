% worldpointset Object for managing 3-D point attributes and 3-D to 2-D correspondences
%   Use this object to store correspondences between 3-D world points and
%   2-D image points across camera views. Use worldpointset with imageviewset
%   to manage image and map data for structure-from-motion, visual odometry,
%   and visual SLAM.
%
%   wpSet = worldpointset() returns a worldpointset object. Add world
%   points and 3-D to 2-D point correspondences using addWorldPoints and
%   addCorrespondences methods. 
%
%   worldpointset properties:
%
%   WorldPoints                 - [X, Y, Z] coordinates of 3-D world points (read-only)
%   ViewIds                     - IDs of views associated with world points (read-only)
%   Correspondences             - 3-D to 2-D point correspondences (read-only)
%   ViewingDirection            - Mean viewing direction of world points (read-only)
%   DistanceLimits              - Distance limits of world points (ready-only)
%   RepresentativeViewId        - Representative view ID of world points (ready-only)
%   RepresentativeFeatureIndex  - Index of feature associated with the representative view (ready-only)
%   Count                       - Number of 3-D world points (read-only)
%
%   worldpointset methods:
%
%   addWorldPoints              - Add world points to the set
%   updateWorldPoints           - Update world points in the set
%   removeWorldPoints           - Remove world points from the set
%   addCorrespondences          - Add 3-D to 2-D point correspondences
%   updateCorrespondences       - Update 3-D to 2-D point correspondences
%   removeCorrespondences       - Remove 3-D to 2-D point correspondences
%   updateLimitsAndDirection    - Update distance limits and mean viewing direction
%   updateRepresentativeView    - Update representative view ID and feature index
%   findWorldPointsInView       - Find the world points observed in a view
%   findViewsOfWorldPoint       - Find the views that observe a world point
%   findWorldPointsInTracks     - Find the world points corresponding to
%                                 point tracks
%
%    Notes
%    -----
%    - To reduce memory use, world point are suggested to be stored as single.
%
%   Example 1: Triangulate stereo images
%   ------------------------------------
%   % Load stereoParams
%   load('webcamsSceneReconstruction.mat');
%
%   % Read in the stereo pair of images.
%   I1 = imread('sceneReconstructionLeft.jpg');
%   I2 = imread('sceneReconstructionRight.jpg');
%
%   % Undistort the images
%   I1 = undistortImage(I1, stereoParams.CameraParameters1);
%   I2 = undistortImage(I2, stereoParams.CameraParameters2);
%
%   % Detect and extract feature points
%   roi = [30, 30, size(I1, 2) - 30, size(I1, 1) - 30];
%   imagePoints1 = detectSURFFeatures(rgb2gray(I1), 'ROI', roi);
%   imagePoints2 = detectSURFFeatures(rgb2gray(I2), 'ROI', roi);
%
%   [feature1, validPoints1] = extractFeatures(rgb2gray(I1), imagePoints1, 'Upright', true);
%   [feature2, validPoints2] = extractFeatures(rgb2gray(I2), imagePoints2, 'Upright', true);
%
%   % Match features
%   indexPairs = matchFeatures(feature1, feature2);
%
%   % Compute the 3-D world points
%   matchedPoints1 = validPoints1(indexPairs(:,1));
%   matchedPoints2 = validPoints2(indexPairs(:,2));
%   worldPoints    = triangulate(matchedPoints1, matchedPoints2, stereoParams);
%
%   % Create a worldpointset object
%   wpSet = worldpointset;
%
%   % Add world points
%   [wpSet, newPointIndices] = addWorldPoints(wpSet, worldPoints);
%
%   % Add 3-D to 2-D correspondences
%   wpSet = addCorrespondences(wpSet, 1, newPointIndices, indexPairs(:, 1));
%   wpSet = addCorrespondences(wpSet, 2, newPointIndices, indexPairs(:, 2));
%
%   % Display the world points
%   pcshow(wpSet.WorldPoints, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
%       'MarkerSize', 45);
%
%   Example 2: Monocular Visual Simultaneous Localization and Mapping
%   -----------------------------------------------------------------
%   % This example shows how to process image data from a monocular camera
%   % to build a map of an indoor environment and estimate the trajectory
%   % of the camera simultaneously.
%   % <a href="matlab:helpview('vision','MonocularVisualSimultaneousLocalizationAndMappingExample')">View example</a>
%
%   See also imageviewset, pointTrack.

%   References
%   ----------
%   Mur-Artal, Raul, Jose Maria Martinez Montiel, and Juan D. Tardos.
%   "ORB-SLAM: a versatile and accurate monocular SLAM system." IEEE
%   Transactions on Robotics 31, no. 5 (2015): 1147-1163.

%   Copyright 2020-2023 The MathWorks, Inc.

%#codegen

classdef worldpointset < vision.internal.EnforceScalarValue

    properties (GetAccess = public, SetAccess = protected)
        %WorldPoints An M-by-3 matrix of [X, Y, Z] world points.
        %
        %   See also triangulate, triangulateMultiview, worldToImage.
        WorldPoints 

        %ViewIds An M-element row vector of view IDs.
        %
        %   See also imageviewset/Views, pointTrack.
        ViewIds 
		
        %PointIds An optional M-element row vector of point IDs.
        PointIds 

        %ViewingDirection An M-by-3 matrix representing the mean viewing 
        %   direction of each world point. The mean viewing direction is 
        %   the average of all the unit vectors pointing from the world  
        %   point to the camera centers of the associated views.
        ViewingDirection 

        %DistanceLimits An M-by-2 vector representing the minimum and maximum
        %   distances from which each world point is observed.
        DistanceLimits 

        %RepresentativeViewId An M-element column vector containing the ID  
        %   of the view that contains the representative feature descriptor 
        %   for each world point.
        RepresentativeViewId 

        %RepresentativeFeatureIndex An M-element column vector containing  
        %   the index of the representative feature in the representative 
        %   view for each world point. A representative feature is the 
        %   medoid of all the feature descriptors associated with the 
        %   world point.  
        RepresentativeFeatureIndex 
    end

    properties (Dependent, GetAccess = public, SetAccess = private)
        %Count A scalar specifying the number of world points.
        %
        Count

        %Correspondences A table containing 3-D to 2-D Correspondences.
        %   A three-column table with columns 'PointIndex', 'ViewId' and
        %   'FeatureIndex'.
        %
        %   PointIndex     - Linear index of world points
        %
        %   ViewId         - An 1-by-N vector specifying the IDs of views
        %                    associated with the world point
        %
        %   FeatureIndex   - An 1-by-N vector specifying the indices of the
        %                    corresponding feature points in the associated
        %                    views
        %
        %   See also imageviewset/Views, pointTrack.
        Correspondences
    end

    properties (GetAccess = ?matlab.unittest.TestCase, SetAccess = private)
        CorrespondencesInternal
    end

    properties (Access = protected, Constant)
        ClassName = mfilename
        % R2020b: 1.0, first version
        % R2022b: 1.1, added additional attributes
        % R2023a: 1.2, added pointIds
        Version   = 1.2;
    end

    methods
        %------------------------------------------------------------------
        % Constructor
        %------------------------------------------------------------------
        function this = worldpointset()
            this.WorldPoints = zeros(coder.ignoreConst(0), coder.ignoreConst(3), 'single');
            this.ViewIds = zeros(1, coder.ignoreConst(0), 'uint32');
            this.PointIds = zeros(1, coder.ignoreConst(0), 'uint32');
            if isempty(coder.target)
                this.CorrespondencesInternal = sparse([]);
            else
                this.CorrespondencesInternal = sparse(coder.ignoreConst(0), coder.ignoreConst(0));
            end

            this.DistanceLimits             = zeros(coder.ignoreConst(0), coder.ignoreConst(2), 'single');
            this.ViewingDirection           = zeros(coder.ignoreConst(0), coder.ignoreConst(3), 'single');
            this.RepresentativeViewId       = zeros(coder.ignoreConst(0), coder.ignoreConst(1));
            this.RepresentativeFeatureIndex = zeros(coder.ignoreConst(0), coder.ignoreConst(1));
        end

        %------------------------------------------------------------------
        function count = get.Count(obj)
            count = size(obj.WorldPoints, 1);
        end

        %--------------------------------------------------------------------
        function Correspondences = get.Correspondences(obj)
		
            PointIndex   = (1:obj.Count).';

            % Create a table from internal correspondence sparse matrix
            outputColumnVector = false;
            if obj.isSimMode()
                [ViewId, FeatureIndex] = arrayfun(...
                    @(x)findViewsOfSingleWorldPoint(obj, x, outputColumnVector), ...
                    PointIndex, 'UniformOutput', false);
            else
                [ViewId, FeatureIndex] = findViewsOfSingleWorldPoint(obj, PointIndex, ...
                    outputColumnVector);
            end
			
			if ~isempty(obj.PointIds)
				PointId = (obj.PointIds)';
			else
				PointId = uint32(PointIndex);
            end

		    Correspondences = table(PointId, ViewId, FeatureIndex, 'VariableNames', ...
					{'PointId', 'ViewId', 'FeatureIndex'});
					
        end
		
		%------------------------------------------------------------------
        function worldPoints = selectWorldPoints(obj, pointIds)
           %selectWorldPoints grabs 3D points from WorldPoints using PointIds
           %   wpSet = selectWorldPoints(wpSet, pointIds)

            if ~isempty(obj.PointIds)
			    pointIds = checkPointIds(obj, pointIds);
			    pointIndices = pointId2PointIndex(obj, pointIds);
			    worldPoints = obj.WorldPoints(pointIndices,:);
            else
                coder.internal.error('vision:worldpointset:unavailableIds');
            end

        end
        %------------------------------------------------------------------
        function [obj, newPointIndices] = addWorldPoints(obj, worldPoints, varargin)
            %addWorldPoints add world points to a worldpointset object
            %   wpSet = addWorldPoints(wpSet, worldPoints) adds world points
            %   to a worldpointset object by specifying the [X, Y, Z]
            %   coordinates, denoted by worldPoints.
			%
            %   wpSet = addWorldPoints(wpSet, worldPoints, pointIds) adds world points
            %   to a worldpointset object by specifying the [X, Y, Z]
            %   coordinates denoted by worldPoints and its corresponding PointIds.
            %
            %   [wpSet, newPointIndices] = addWorldPoints(...) additionally returns the 
            %   indices of the added world points.
            %
            %   Example
            %   -------
            %   % Generate 3-D world points
            %   worldPoints = rand(100, 3);
            %
            %   % Create a worldpointset object
            %   wpSet = worldpointset;
            %
            %   % Add world points
            %   wpSet = addWorldPoints(wpSet, worldPoints);
            %
            %   See also removeWorldPoints, updateWorldPoints.

            checkWorldPoints(obj, worldPoints);
            
            numNewPoints = size(worldPoints, 1);

            % Return indices as a column vector
            newPointIndices = (obj.Count+1:obj.Count+numNewPoints).';

            % Set Point IDs (optional)
            if nargin > 2
				% Check if the set was created without PointIds
                if ~isempty(obj.WorldPoints) && isempty(obj.PointIds)
                   coder.internal.error('vision:worldpointset:nopointid');
                end
                newIds = varargin{1};
                newIds = checkPointIds(obj, newIds, worldPoints);
                if obj.isSimMode()
                    obj.PointIds(1,newPointIndices) = newIds;
                else
                   obj.PointIds = [obj.PointIds, newIds(:)'];
                end
            end
			
			% Check if user forgot to provide PointIds
            if nargin == 2
                if ~isempty(obj.PointIds)
                   coder.internal.error('vision:worldpointset:missingPointId');
                end
            end

            % Set [X, Y, Z] Location
            obj.WorldPoints = [obj.WorldPoints; worldPoints];

            % Initialize correspondences for new world points
            if numel(obj.ViewIds) == 0
                obj.CorrespondencesInternal = sparse(zeros(obj.Count, coder.ignoreConst(0)));
            else
                if obj.isSimMode()
                    obj.CorrespondencesInternal(newPointIndices,:) = 0;
                else
                    newCorrespondences = sparse(zeros(numNewPoints, size(obj.CorrespondencesInternal, 2)));
                    obj.CorrespondencesInternal = [obj.CorrespondencesInternal; newCorrespondences];
                end
            end
        end

        %------------------------------------------------------------------
        function obj = removeWorldPoints(obj, pointIds)
            %removeWorldPoints remove world points from a worldpointset object
            %   wpSet = removeWorldPoints(wpSet, pointIds) removes world
            %   points from a worldpointset object by specifying the IDs
            %   of the world points to be removed, denoted by pointIds.
            %
            %   Example
            %   -------
            %   % Generate 3-D world points
            %   worldPoints = rand(100, 3);
            %
            %   % Create a worldpointset object
            %   wpSet = worldpointset;
            % 
            %   % Create a set of pointIds
            %   pointIds = randperm(100);
            %
            %   % Add world points
            %   [wpSet, newPointIndices] = addWorldPoints(wpSet, worldPoints, pointIds);
            %
            %   % Remove the first 50 world points
            %   wpSet = removeWorldPoints(wpSet, pointIds(1:50))
            %
            %   See also addWorldPoints, updateWorldPoints.

            checkIfNoPointIsAdded(obj);
			pointIndices = pointIdOrPointIndex(obj, pointIds);
            checkPointIndices(obj, pointIndices);

            % Remove world points
            obj.WorldPoints(pointIndices, :) = [];
			
            % Remove pointIds
			if ~isempty(obj.PointIds)
                obj.PointIds(:, pointIndices) = [];
			end

            % Remove Correspondences
            if obj.isSimMode()
                obj.CorrespondencesInternal(pointIndices, :) = [];
            else
                % In codegen [] cannot be assigned to sparse matrices
                correspondences = obj.CorrespondencesInternal;
                correspondencesSize = coder.internal.indexInt(size(correspondences, 1));
                numRows = 0;
                for i = 1:correspondencesSize
                    if ~ismember(i, pointIndices)
                        numRows = numRows+1;
                    end
                end
                k = 1;
                indices = coder.nullcopy(zeros(numRows, 1));
                for i=1:correspondencesSize
                    if ~ismember(i, pointIndices)
                        indices(k) = i;
                        k = k+1;
                    end
                end
                updatedCorres = correspondences(indices, :);
                obj.CorrespondencesInternal = updatedCorres;
            end

            % Update other properties
            if ~isempty(obj.ViewingDirection)
                obj.ViewingDirection(pointIndices, :)        = [];
                obj.DistanceLimits(pointIndices, :)          = [];
            end

            if ~isempty(obj.RepresentativeViewId)
                obj.RepresentativeViewId(pointIndices)       = [];
                obj.RepresentativeFeatureIndex(pointIndices) = [];
            end
        end

        %------------------------------------------------------------------
        function obj = updateWorldPoints(obj, pointIds, worldPoints)
            %updateWorldPoints update world points in a worldpointset object
            %   wpSet = updateWorldPoints(wpSet, pointIds, worldPoints)
            %   updates the locations of the world points, specified by
            %   pointIds, in a worldpointset object with new locations,
            %   specified by worldPoints.
            %
            %   Example
            %   -------
            %   % Generate 3-D world points
            %   worldPoints = rand(100, 3);
            %
            %   % Create a worldpointset object
            %   wpSet = worldpointset;
            %
            %   % Add world points
            %   wpSet = addWorldPoints(wpSet, worldPoints);
            %
            %   % Update the first 50 world points with new locations
            %   pointIds = 1:50;
            %   newWorldPoints = worldPoints(pointIds,:) + [0 0 5];
            %   wpSet = updateWorldPoints(wpSet, pointIds, newWorldPoints);
            %
            %   See also addWorldPoints, removeWorldPoints.

			
            pointIndices = pointIdOrPointIndex(obj, pointIds);
            checkPointIndices(obj, pointIndices);
            checkWorldPoints(obj, worldPoints, numel(pointIndices));
             
            % Update world points
            obj.WorldPoints(pointIndices,:) = worldPoints;
        end

        %------------------------------------------------------------------
        function obj = addCorrespondences(obj, viewId, pointIds, featureIndices)
            %addCorrespondences add 3-D to 2-D projection correspondences
            %   wpSet = addCorrespondences(wpSet, viewId, pointIndices,
            %   featureIndices) adds 3-D to 2-D projection correspondences
            %   between a view, specified by viewId, and the world points,
            %   specified by pointIndices, to a worldpointset object.
            %
            %   viewId is a scalar specifying the view ID. pointIndices is
            %   an M-element array specifying the indices of world points.
            %   featureIndices is an M-element array containing the indices
            %   of the corresponding feature points in the view.
            %
            %   Example
            %   -------
            %   % Generate 3-D world points
            %   worldPoints = rand(100, 3);
            %
            %   % Create a worldpointset object
            %   wpSet = worldpointset;
            %
            %   % Add world points
            %   wpSet = addWorldPoints(wpSet, worldPoints);
            %
            %   % View 1 observe the first 10 world points
            %   viewId = 1;
            %   pointIndices   = 1:10;
            %   featureIndices = 1:10;
            %   wpSet  = addCorrespondences(wpSet, viewId, pointIndices, ...
            %       featureIndices);
            %
            %   See also updateCorrespondences, removeCorrespondences,
            %       imageviewset.

			pointIndices = pointIdOrPointIndex(obj, pointIds);
            checkIfNoPointIsAdded(obj);
            viewId = checkViewId(obj, viewId);
            checkPointIndices(obj, pointIndices);
            checkFeatureIndices(obj, featureIndices, numel(pointIndices));

			viewIndex = viewId2ViewIndex(obj, viewId);

            if obj.isSimMode()
                if ~isempty(viewIndex) % Old view: add correspondences
                    obj.CorrespondencesInternal(pointIndices, viewIndex) = featureIndices;
                else % New view: add a new column
                    obj.ViewIds = [obj.ViewIds, viewId];
                    newNumViews = numel(obj.ViewIds);
                    obj.CorrespondencesInternal(pointIndices, newNumViews) = featureIndices;
                end
            else
                if ~isempty(viewIndex) % Old view: add correspondences
                    obj.CorrespondencesInternal(pointIndices(:), viewIndex) = featureIndices(:);
                else % New view: add a new column
                    obj.ViewIds = [obj.ViewIds, viewId];
                    % New column to be concatenated to correspondences
                    corresUpd = sparse(zeros(obj.Count, 1));
                    corresUpd(pointIndices(:), 1) = featureIndices(:);
                    % Concatenate the new column to existing correspondences
                    obj.CorrespondencesInternal = [obj.CorrespondencesInternal corresUpd];
                end
            end
        end

        %------------------------------------------------------------------
        function obj = updateCorrespondences(obj, viewId, pointIds, featureIndices)
            %updateCorrespondences update 3-D to 2-D projection correspondences
            %   wpSet = updateCorrespondences(wpSet, viewId, pointIds,
            %   featureIndices) updates 3-D to 2-D projection correspondences
            %   between a view, specified by viewId, and the world points,
            %   specified by pointIds, in a worldpointset object.
            %
            %   viewId is a scalar specifying the view ID. pointIds is
            %   an M-element array specifying the IDs of world points.
            %   featureIndices is an M-element array containing the new
            %   indices of the corresponding feature points in the view.
            %
            %   Example
            %   -------
            %   % Generate 3-D world points
            %   worldPoints = rand(100, 3);
            %
            %   % Create a worldpointset object
            %   wpSet = worldpointset;
            %
            %   % Add world points
            %   wpSet = addWorldPoints(wpSet, worldPoints);
            %
            %   % Add correspondences for a view
            %   viewId = 1;
            %   pointIds   = 1:10;
            %   featureIndices = 1:10;
            %   wpSet  = addCorrespondences(wpSet, viewId, pointIds, ...
            %       featureIndices);
            %
            %   % Update the feature indices
            %   newFeatureIndices = 11:20;
            %   wpSet  = updateCorrespondences(wpSet, viewId, pointIds, ...
            %       newFeatureIndices);
            %
            %   See also addCorrespondences, removeCorrespondences,
            %       imageviewset.

			pointIndices = pointIdOrPointIndex(obj, pointIds);
            viewId = checkViewId(obj, viewId);
            checkIfViewIsMissing(obj, viewId);
            checkPointIndices(obj, pointIndices);
            checkFeatureIndices(obj, featureIndices, numel(pointIndices));

            viewIndex = checkIfProjectionIsMissing(obj, viewId, pointIndices);

            obj.CorrespondencesInternal(pointIndices(:), viewIndex) = featureIndices(:);
        end

        %------------------------------------------------------------------
        function obj = removeCorrespondences(obj, viewId, pointIds)
            %removeCorrespondences remove 3-D to 2-D projection correspondences
            %   wpSet = removeProjection (wpSet, viewId, pointIds)
            %   removes 3-D to 2-D projection correspondences between a view,
            %   specified by viewId, and the world points, specified by
            %   pointIds, from a worldpointset object.
            %
            %   viewId is a scalar specifying the view ID. pointIds is an
            %   M-element array specifying the IDs of world points.
            %
            %   Example
            %   -------
            %   % Generate 3-D world points
            %   worldPoints = rand(100, 3);
            %
            %   % Create a worldpointset object
            %   wpSet = worldpointset;
            %
            %   % Add world points
            %   wpSet = addWorldPoints(wpSet, worldPoints);
            %
            %   % Add correspondences for a view
            %   viewId = 1;
            %   pointIds   = 1:10;
            %   featureIndices = 1:10;
            %   wpSet  = addCorrespondences(wpSet, viewId, pointIds, ...
            %       featureIndices);
            %
            %   % Remove the first 5 correspondences
            %   pointIds = 1:5;
            %   wpSet  = removeCorrespondences(wpSet, viewId, pointIds);
            %
            %   See also addCorrespondences, updateCorrespondences,
            %       imageviewset.

			pointIndices = pointIdOrPointIndex(obj, pointIds);
            viewId = checkViewId(obj, viewId);
            checkIfViewIsMissing(obj, viewId);
            checkPointIndices(obj, pointIndices);

			viewIndex = viewId2ViewIndex(obj, viewId);

            % Set feature indices to zero
            obj.CorrespondencesInternal(pointIndices, viewIndex) = 0;

            % If none of the world points is observed in the view, remove
            % the view
            observeZeroPoint = ~any(obj.CorrespondencesInternal(:,viewIndex));
            if observeZeroPoint
                if obj.isSimMode()
                    obj.CorrespondencesInternal(:, viewIndex) = [];
                else
                    if viewIndex == 1
                        % If the given viewId is the first view
                        obj.CorrespondencesInternal = obj.CorrespondencesInternal(:, (viewIndex+1:end));
                    elseif viewIndex == size(obj.CorrespondencesInternal, 2)
                        % If the given viewId is the last view
                        obj.CorrespondencesInternal = obj.CorrespondencesInternal(:, (1:viewIndex-1));
                    else
                        obj.CorrespondencesInternal = [obj.CorrespondencesInternal(:, (1:viewIndex-1)), obj.CorrespondencesInternal(:, (viewIndex+1:end))];
                    end
                end
                obj.ViewIds(obj.ViewIds == viewId) = [];
            end
        end
        
        %------------------------------------------------------------------
        function obj = updateLimitsAndDirection(obj, pointIds, viewTable)
            %updateLimitsAndDirection update distance limits and viewing direction
            %   wpSet = updateLimitsAndDirection(wpSet, pointIds,
            %   viewTable) updates distance limits and viewing direction  
            %   of world points specified by pointIds, in a worldpointset 
            %   object based on a table of views, viewTable. viewTable must
            %   contain four columns named 'ViewId', 'AbsolutePose', 
            %   'Features' and 'Points'. Use findView to retrieve a table 
            %   of views from an imageviewset object.
            %
            %   Example
            %   -------
            %   % Create a worldpointset object
            %   wpSet = worldpointset;
            %   worldPoints = rand(10, 3); % Add world points
            %   pointIds   = randperm(10);
            %   wpSet = addWorldPoints(wpSet, worldPoints, pointIds); 
            %   featureIndices = 1:10;
            %   viewId1 = 1;
            %   wpSet   = addCorrespondences(wpSet, viewId1, pointIds, ...
            %       featureIndices); % Add correspondences for view 1
            %   viewId2 = 2;
            %   wpSet   = addCorrespondences(wpSet, viewId2, pointIds, ...
            %       featureIndices);  % Add correspondences for view 2
            %   
            %   % Create an imageviewset object
            %   vSet  = imageviewset;
            %   pose1 = rigidtform3d();
            %   vSet  = addView(vSet, viewId1, pose1);
            %   pose2 = rigidtform3d(eye(3), [1 0 0]); % Translation in x-axis
            %   vSet  = addView(vSet, viewId2, pose2);
            %
            %   % Update distance limits and viewing direction
            %   viewTable = findView(vSet, [viewId1, viewId2]);
            %   wpSet = updateLimitsAndDirection(wpSet, pointIds, viewTable);
            %
            %   % Check distance limits and viewing direction 
            %   wpSet.ViewingDirection
            %   wpSet.DistanceLimits
            %
            %   See also imageviewset/addView, imageviewset/findView,  
            %       updateRepresentativeView, imageviewset.
            
			pointIndices = pointIdOrPointIndex(obj, pointIds);
            checkPointIndices(obj, pointIndices);
            [allViewIds, allPoses] = checkViewTable(obj, viewTable);
            
            % Initialize attributes in the first call
            if isempty(obj.ViewingDirection)
                obj.ViewingDirection = NaN(obj.Count, 3, 'single');
                obj.DistanceLimits   = NaN(obj.Count, 2, 'single');
            end

            if obj.isSimMode()
                allViewLocations = vertcat(allPoses.Translation);
            else
                allViewLocations = zeros(length(allPoses), 3);
                for i=1:length(allPoses)
                    allViewLocations(i, :) = allPoses(i).Translation;
                end
            end
            
            % Get all the views associated with the 3-D points
            viewIdsToUpdate = findViewsOfWorldPoint(obj, pointIds);
            
            isSinglePoint = isscalar(pointIndices);

            % Check if there are views missing in the view table
            checkMissingViewInViewTable(obj, isSinglePoint, viewIdsToUpdate, allViewIds);

            if ~obj.isSimMode()
                viewDirectionSize = max(pointIndices, [], 'all');
                if viewDirectionSize > coder.internal.indexInt(size(obj.ViewingDirection, 1))
                    updatedViewDirection = coder.nullcopy(zeros(viewDirectionSize, 3, class(obj.ViewingDirection)));
                    updatedDistanceLimits = coder.nullcopy(zeros(viewDirectionSize, 2, class(obj.DistanceLimits)));
                    for i=1:coder.internal.indexInt(size(obj.ViewingDirection, 1))
                        updatedViewDirection(i, :) = obj.ViewingDirection(i, :);
                    end
                    for i=1:coder.internal.indexInt(size(obj.DistanceLimits, 1))
                        updatedDistanceLimits(i, :) = obj.DistanceLimits(i, :);
                    end
                    obj.ViewingDirection = updatedViewDirection;
                    obj.DistanceLimits = updatedDistanceLimits;
                end
            end
            for i = 1:coder.internal.indexInt(numel(pointIndices))
                
                if isSinglePoint && obj.isSimMode()
                    viewIdsOfSinglePoint = viewIdsToUpdate;
                else % viewIdsToUpdate is always a cell array in codegen mode
                    viewIdsOfSinglePoint = viewIdsToUpdate{i};
                end

                if ~isempty(viewIdsOfSinglePoint) % Correspondence exists
                    
                    % Find views in the view table by ID
                    [~, ia] = intersect(allViewIds, viewIdsOfSinglePoint);

                    currPointIdx = pointIndices(i);

                    % Get all vectors from the 3-D point to cameras
                    directionVectors     = obj.WorldPoints(currPointIdx,:) - allViewLocations(ia, :);

                    % Compute distance
                    viewToPointDistances = vecnorm(directionVectors, 2, 2);

                    % Check if a 3-D point coincides with a camera center
                    coder.internal.errorIf(any(viewToPointDistances<10*eps('single')), 'vision:worldpointset:invalidViewLocation', currPointIdx);

                    % Mean of normalized direction vectors
                    meanDirectionVector  = mean(directionVectors ./ viewToPointDistances, 1);

                    meanDirectionVectorNorm = norm(meanDirectionVector);

                    % Handle [0 0 0] viewing direction
                    meanDirectionVectorNorm(meanDirectionVectorNorm==0) = 1;

                    obj.ViewingDirection(currPointIdx, :) = meanDirectionVector/meanDirectionVectorNorm;
                    obj.DistanceLimits(currPointIdx, :)   = [min(viewToPointDistances), max(viewToPointDistances)];
                end
            end
        end

        %------------------------------------------------------------------
        function obj = updateRepresentativeView(obj, pointIds, viewTable)
            %updateRepresentativeView update representative view ID and the associated feature index
            %   wpSet = updateRepresentativeView(wpSet, pointIds, viewTable) 
            %   updates representative view ID and representative feature 
            %   index of world points specified by pointIds, in a 
            %   worldpointset object based on a table of views, viewTable. 
            %   viewTable must contain four columns named 'ViewId', 
            %   'AbsolutePose', 'Features', and 'Points'. Use findView to 
            %   retrieve a table of views from an imageviewset object.
            %
            %   Example
            %   -------
            %   % Create an imageviewset object
            %   vSet    = imageviewset;
            %   viewId1 = 1; % Add view 1
            %   features1 = binaryFeatures(randi(255, [10, 32], 'uint8'));
            %   vSet = addView(vSet, viewId1, 'Features', features1);
            %   viewId2 = 2; % Add view 2
            %   features2 = binaryFeatures(randi(255, [10, 32], 'uint8'));
            %   vSet = addView(vSet, viewId2, 'Features', features2);
            %   viewId3 = 3; % Add view 3
            %   features3 = binaryFeatures(randi(255, [10, 32], 'uint8'));
            %   vSet = addView(vSet, viewId3, 'Features', features3);
            %
            %   % Create a worldpointset object
            %   wpSet = worldpointset;
            %   worldPoints = rand(10, 3); % Add world points
            %   wpSet = addWorldPoints(wpSet, worldPoints);
            %   pointIds= 1:10; 
            %   featureIndices1 = randperm(10);
            %   wpSet  = addCorrespondences(wpSet, viewId1, pointIds, ...
            %       featureIndices1); % Add correspondences for view 1
            %   featureIndices2 = randperm(10);
            %   wpSet  = addCorrespondences(wpSet, viewId2, pointIds, ...
            %       featureIndices2); % Add correspondences for view 2
            %   featureIndices3 = randperm(10);
            %   wpSet  = addCorrespondences(wpSet, viewId3, pointIds, ...
            %       featureIndices3); % Add correspondences for view 3
            %
            %   % Update representative view
            %   viewTable = findView(vSet, [viewId1, viewId2, viewId3]);
            %   wpSet = updateRepresentativeView(wpSet, pointIds, viewTable);
            %
            %   % Check representative view ID and feature index
            %   wpSet.RepresentativeViewId
            %   wpSet.RepresentativeFeatureIndex
            %
            %   See also imageviewset/addView, imageviewset/findView,  
            %       updateLimitsAndDirection, imageviewset.
            
			pointIndices = pointIdOrPointIndex(obj, pointIds);
            checkPointIndices(obj, pointIndices);
            [allViewIds, ~, allViewFeatures] = checkViewTable(obj, viewTable);
            
            % Initialize attributes 
            if isempty(obj.RepresentativeViewId)                            
                obj.RepresentativeViewId         = NaN(obj.Count, 1);
                obj.RepresentativeFeatureIndex   = NaN(obj.Count, 1);
            end

            % Inspect feature type and feature vector width
            firstFeature    = allViewFeatures{1};
            isBinaryFeature = isa(firstFeature, 'binaryFeatures');

            if isBinaryFeature
                featureVectorWidth    = size(firstFeature.Features, 2);
            else
                featureVectorWidth    = size(firstFeature, 2);
            end

            [viewIdsToUpdate, featureIndices]   = findViewsOfWorldPoint(obj, pointIds);

            isSinglePoint = isscalar(pointIndices);
            
            % Check if there are views missing in the view table
            checkMissingViewInViewTable(obj, isSinglePoint, viewIdsToUpdate, allViewIds);

            if ~obj.isSimMode()
                viewRepresentativeSize = max(pointIndices, [], 'all');
                if viewRepresentativeSize > coder.internal.indexInt(size(obj.RepresentativeViewId))
                    updatedRepresentativeViewId = coder.nullcopy(zeros(viewRepresentativeSize, 1, class(obj.RepresentativeViewId)));
                    updatedRepresentativeFeatureIndex = coder.nullcopy(zeros(viewRepresentativeSize, 1, class(obj.RepresentativeFeatureIndex)));
                    for i=1:coder.internal.indexInt(size(obj.RepresentativeViewId, 1))
                        updatedRepresentativeViewId(i, :) = obj.RepresentativeViewId(i, :);
                    end
                    for i=1:coder.internal.indexInt(size(obj.RepresentativeFeatureIndex, 1))
                        updatedRepresentativeFeatureIndex(i, :) = obj.RepresentativeFeatureIndex(i, :);
                    end
                    obj.RepresentativeViewId = updatedRepresentativeViewId;
                    obj.RepresentativeFeatureIndex = updatedRepresentativeFeatureIndex;
                end
            end

            for i = 1:coder.internal.indexInt(numel(pointIndices))

                if isSinglePoint && obj.isSimMode()
                    viewIdsOfSinglePoint        = viewIdsToUpdate;
                    featureIndicesOfSinglePoint = featureIndices;
                else % viewIdsToUpdate is always a cell array in codegen mode
                    viewIdsOfSinglePoint        = viewIdsToUpdate{i};
                    featureIndicesOfSinglePoint = featureIndices{i};
                end

                numViews  = numel(viewIdsOfSinglePoint);
                
                currPointIdx = pointIndices(i);
                if numViews < 3 % If less than 3 views, select the latest view
                    obj.RepresentativeViewId(currPointIdx)       = viewIdsOfSinglePoint(numViews);
                    obj.RepresentativeFeatureIndex(currPointIdx) = featureIndicesOfSinglePoint(numViews);
                else
                    % Find views in the view table by ID
                    [~, ia] = intersect(allViewIds, viewIdsOfSinglePoint);
                    
                    % Initialize feature descriptor matrix
                    allFeaturesOfSinglePoint = zeros(featureVectorWidth, numViews, 'single'); 

                    for k = 1:numViews
                        featureOfSingleView = allViewFeatures{ia(k)}; % numViews >=3 ensures that allViewFeatures is a cell array
                        if isBinaryFeature
                            allFeaturesOfSinglePoint(:, k) =  featureOfSingleView.Features(featureIndicesOfSinglePoint(k), :)';
                        else
                            allFeaturesOfSinglePoint(:, k) =  featureOfSingleView(featureIndicesOfSinglePoint(k))';
                        end
                    end
                    
                    % Find medoid feature descriptor
                    medoidIndex = findMedoidFeature(obj, allFeaturesOfSinglePoint, isBinaryFeature, numViews);

                    obj.RepresentativeViewId(currPointIdx)       = viewIdsOfSinglePoint(medoidIndex);
                    obj.RepresentativeFeatureIndex(currPointIdx) = featureIndicesOfSinglePoint(medoidIndex);
                end
            end
        end

        %------------------------------------------------------------------
        function [pointRef, featureIndices] = findWorldPointsInView(obj, viewId)
            %findWorldPointsInView find world points observed in a view
            %   [pointIds, featureIndices] = findWorldPointsInView(wpSet, viewId)
            %   returns the indices of world points observed in a view and
            %   the indices of the associated features in the view.
            %
            %   viewId is a scalar or an array of view IDs.
            %   - When viewId is a scalar, pointIds is an M-element
            %     array of world point IDs and featureIndices is an
            %     M-element array containing the indices of the corresponding
            %     feature points in the view.
            %   - When viewId is an N-element array, pointIds is an
            %     N-element cell array of world point IDs and
            %     featureIndices is an N-element cell array of feature indices.
            %
            %   Example
            %   -------
            %   % Generate 3-D world points
            %   worldPoints = rand(100, 3);
            %
            %   % Create a worldpointset object
            %   wpSet = worldpointset;
            %
            %   % Add world points
            %   wpSet = addWorldPoints(wpSet, worldPoints);
            %
            %   % Add 3-D to 2-D correspondences for view 1
            %   viewId1 = 1;
            %   pointIndices1   = 1:10;
            %   featureIndices1 = 1:10;
            %   wpSet  = addCorrespondences(wpSet, viewId1, pointIndices1, ...
            %       featureIndices1);
            %
            %   % Add 3-D to 2-D correspondences for view 2
            %   viewId2 = 2;
            %   pointIndices2   = 6:10;
            %   featureIndices2 = 1:5;
            %   wpSet  = addCorrespondences(wpSet, viewId2, pointIndices2, ...
            %       featureIndices2);
            %
            %   % Find world points in view 2
            %   [pointIds, featureIndices] = findWorldPointsInView(...
            %       wpSet, viewId2)
            %
            %   See also addCorrespondences, findViewsOfWorldPoint,
            %       imageviewset.

            [pointIndices, featureIndices] = findVisibilityOfView(obj, viewId);

            if ~isempty(obj.PointIds)
                if iscell(pointIndices)
                    n = length(pointIndices);
                    pointIds = cell(n,1);
                    for i=1:n
                        PIdx=pointIndices{i};
                        PIds=obj.PointIds(1,PIdx)';
                        pointIds{i}=double(PIds);
                    end
                else
			        pointIds = double(obj.PointIds(1,pointIndices))';
                end
                pointRef = pointIds;
            else
                pointRef = pointIndices;
            end
        end

        %------------------------------------------------------------------
        function [viewIds, featureIndices] = findViewsOfWorldPoint(obj, pointIds)
            %findViewsOfWorldPoint find views that observe a world point
            %   [viewIds, featureIndices] = findViewsOfWorldPoint(wpSet, pointId)
            %   returns the Ids of views that observed the world point and
            %   the indices of the associated features in the views.
            %
            %   pointId is a scalar or an array of world point indices.
            %   - When pointId is a scalar, viewIds is an M-element array
            %     of view IDs and featureIndices is an M-element array
            %     containing the indices of the corresponding feature points
            %     in the views.
            %   - When pointId is an N-element array, viewIds is an
            %     N-element cell array of view IDs and featureIndices is an
            %     N-element cell array of feature indices.
            %
            %   Example
            %   -------
            %   % Generate 3-D world points
            %   worldPoints = rand(100, 3);
            %
            %   % Create a worldpointset object
            %   wpSet = worldpointset;
            %
            %   % Add world points
            %   wpSet = addWorldPoints(wpSet, worldPoints);
            %
            %   % Add 3-D to 2-D correspondences for view 1
            %   viewId1 = 1;
            %   pointIds1   = 1:10;
            %   featureIndices1 = 1:10;
            %   wpSet  = addCorrespondences(wpSet, viewId1, pointIds1, ...
            %       featureIndices1);
            %
            %   % Add 3-D to 2-D correspondences for view 2
            %   viewId2 = 2;
            %   pointIds2   = 6:10;
            %   featureIndices2 = 1:5;
            %   wpSet  = addCorrespondences(wpSet, viewId2, pointIds2, ...
            %       featureIndices2);
            %
            %   % Find views of world points
            %   pointIds = 6:10;
            %   viewIds = findViewsOfWorldPoint(wpSet, pointIds);
            %
            %   See also findWorldPointsInView, findPointsInTracks,
            %       imageviewset.

		    pointIndices = pointIdOrPointIndex(obj, pointIds);

            checkPointIndices(obj, pointIndices);

            outputColumnVector = true;

            if obj.isSimMode()
                if isscalar(pointIds) % Single world point
                    [viewIds, featureIndices] = findViewsOfSingleWorldPoint(obj, ...
                        pointIndices, outputColumnVector);
                else % Multiple world points
                    [viewIds, featureIndices] = arrayfun(...
                        @(x)findViewsOfSingleWorldPoint(obj, x, outputColumnVector), ...
                        pointIndices(:), 'UniformOutput', false);
                end
            else
                viewIds = coder.nullcopy(cell(numel(pointIndices), 1));
                featureIndices = coder.nullcopy(cell(numel(pointIndices), 1));
                if isscalar(pointIndices) % Single world point
                    [vIds, fIndices] = findViewsOfSingleWorldPoint(obj, ...
                        pointIndices, outputColumnVector);
                    viewIds{1} = vIds{:};
                    featureIndices{1} = fIndices{:};
                else % Multiple world points
                    for i=1:numel(pointIndices)
                        [vIds, fIndices] = findViewsOfSingleWorldPoint(obj, ...
                            pointIndices(i), outputColumnVector);
                        viewIds{i} = vIds{:};
                        featureIndices{i} =  fIndices{:};
                    end
                end
            end
        end

        %------------------------------------------------------------------
        function [pointIds, validIndex] = findWorldPointsInTracks(obj, tracks)
            %findWorldPointsInTracks find the world points corresponding to point tracks
            %   [pointIds, validIndex] = findWorldPointsInTracks(wpSet, tracks)
            %   returns the indices of world points corresponding to the point
            %   tracks, denoted by tracks. Use findTracks of the imageviewset
            %   class to find point tracks across views.
            %
            %   tracks is an M-element array of pointTrack object. validIndex
            %   found is an M-element logical array indicating if a world
            %   point is for the point track.
            %
            %   Example
            %   -------
            %   % Load precomputed world point set and image view set
            %   data = load(fullfile(toolboxdir('vision'), 'visiondata', ...
            %     'worldpointsetAndTracks.mat'));
            %
            %   % Find point tracks across views
            %   tracks = findTracks(data.vSet);
            %
            %   % Find 3-D world points corresponding to point tracks
            %   pointIds = findWorldPointsInTracks(data.wpSet, tracks);
            %
            %   See also findWorldPointsInView, findViewsOfWorldPoint,
            %       imageviewset/findTracks.

            checkIfNoPointIsAdded(obj);

            checkPointTracks(obj, tracks);

            numTracks = numel(tracks);

            pointIndices = zeros(numTracks, 1);
            validIndex   = false(numTracks, 1);

            for i = 1:numTracks
                track = tracks(i);

                % Use view ID and feature index to find the world point
                viewIdInTrack = track.ViewIds;
                checkIfViewIsMissing(obj, viewIdInTrack);

				viewIndex = viewId2ViewIndex(obj, viewIdInTrack);
                featureIndex = track.FeatureIndices;

                % Convert to full matrix in order to use ismember
                subCorrespondences = full(obj.CorrespondencesInternal(:, viewIndex));
                [worldPointFound, pointIdx] = ismember(featureIndex, ...
                    subCorrespondences, 'rows');

                if worldPointFound
                    pointIndices(i) = pointIdx;
                    validIndex(i)   = true;
                end

            end

            % Only return indices for valid point tracks
            pointIndices = pointIndices(validIndex);
            if ~isempty(obj.PointIds)
			    pointIds = double(obj.PointIds(1,pointIndices)');
            else
                pointIds = pointIndices;
            end
        end
    end

    methods (Access = public, Hidden)
        function [pointIndices, featureIndices, varargout] = findVisibilityOfView(obj, viewId)
            % [pointIndices, featureIndices] = findVisibilityOfView(wpSet, viewId)
            %   returns the indices of world points observed in a view and
            %   the indices of the associated features in the view.
            %
            % [..., uniquePointIndices, visibility] = findVisibilityOfView(wpSet, viewId)
            %   also returns the union of indices of world points observed
            %   in the views, and the visibility binary matrix.
            viewId = checkViewIds(obj, viewId);
            checkIfViewIsMissing(obj, viewId);
            checkIfNoPointIsAdded(obj);

			viewIndex = viewId2ViewIndex(obj, viewId);

            if obj.isSimMode()
                if isscalar(viewId) % Single view
                    [pointIndices, featureIndices] = ...
                        findWorldPointsInSingleView(obj, viewIndex);
                else % Multiple views
                    [pointIndices, featureIndices] = arrayfun(...
                        @(x)findWorldPointsInSingleView(obj, x), ...
                        viewIndex(:), 'UniformOutput', false);
                end
            else
                pointIndices = coder.nullcopy(cell(numel(viewIndex), 1));
                featureIndices = coder.nullcopy(cell(numel(viewIndex), 1));

                if isscalar(viewId) % Single view
                    [pointIndices{1}, featureIndices{1}] = ...
                        findWorldPointsInSingleView(obj, viewIndex);
                else % Multiple views
                    for i=1:numel(viewIndex)
                        [ptIndices, ftIndices] = findWorldPointsInSingleView(obj, viewIndex(i));
                        pointIndices{i,1} = ptIndices;
                        featureIndices{i,1} = ftIndices;
                    end
                end
            end

            if nargout > 2
                if obj.isSimMode()
                    if isscalar(viewId) 
                        uniquePointIndices = pointIndices;
                    else
                        uniquePointIndices = unique(vertcat(pointIndices{:}));
                    end
                else
                    uniquePointIndices = zeros(coder.ignoreConst(0), 1);
                    if isscalar(viewId)
                        uniquePointIndices = pointIndices{:};
                    else
                        for i=1:numel(pointIndices)
                            uniquePointIndices = unique([uniquePointIndices; pointIndices{i, :}]);
                        end
                    end
                end
                visibility = obj.CorrespondencesInternal(uniquePointIndices, viewIndex);
                if obj.isSimMode
                    visibility(visibility~=0) = 1;
                else
                    for i=1:numel(visibility)
                        if visibility(i) ~=0
                            visibility(i) = 1;
                        end
                    end
                end
                varargout{1} = uniquePointIndices;
                varargout{2} = visibility;
            end
        end
    end

    methods (Access = protected)
        %------------------------------------------------------------------
        function [viewIds, featureIndices] = findViewsOfSingleWorldPoint(...
                obj, pointIndex, outputColumnVector)
				
            if obj.isSimMode()
                [~, viewIdx, featureIndices] = ...
                    find(obj.CorrespondencesInternal(pointIndex, :)); % Row vector

                viewIds = viewIndex2ViewId(obj, viewIdx);
                if isempty(featureIndices)
                    featureIndices = zeros(1, 0);
                end

                if outputColumnVector
                    viewIds = viewIds(:);
                    featureIndices = featureIndices(:);
                end
            else
                numPoints = numel(pointIndex);
                viewIds = cell(numPoints, 1);
                featureIndices = cell(numPoints, 1);
                for i=1:numPoints
                    [~, viewIdx, fIndices] = ...
                        find(obj.CorrespondencesInternal(pointIndex(i), :));

                    vIds = viewIndex2ViewId(obj, viewIdx);
                    if outputColumnVector
                        % Convert viewds and feature indices to columns
                        % in case of column output format
                        viewIdsCol = vIds(:);
                        featureIndicesCol = fIndices(:);
                        viewIds{i, 1} = viewIdsCol;
                        featureIndices{i, 1} = featureIndicesCol;
                    else
                        viewIds{i, 1} = vIds;
                        featureIndices{i, 1} = fIndices;
                    end
                end
            end
        end

        %------------------------------------------------------------------
        function [pointIndices, featureIndices] = findWorldPointsInSingleView(...
                obj, viewIndex)
            [pointIndices, ~, featureIndices] = ...
                find(obj.CorrespondencesInternal(:, viewIndex));
            featureIndices = double(featureIndices(:));
        end
        %------------------------------------------------------------------
        function viewIndex = viewId2ViewIndex(obj, viewIds)
            [~, ~, viewIndex] = intersect(viewIds, obj.ViewIds, 'stable');
        end
		
        %------------------------------------------------------------------
        function pointIndex = pointId2PointIndex(obj, pointIds)
            [~, ~, pointIndex] = intersect(pointIds, obj.PointIds, 'stable');
        end
        %------------------------------------------------------------------
        function pointIndices = pointIdOrPointIndex(obj, pointIds)
			pointIds = checkPointIds(obj, pointIds);
            if ~isempty(obj.PointIds)
				pointIndices = pointId2PointIndex(obj, pointIds);
			else
				pointIndices = double(pointIds);
            end
        end
        %------------------------------------------------------------------
        function viewId = viewIndex2ViewId(obj, viewIndex)
            if isempty(viewIndex)
                viewId = zeros(1, 0, 'uint32');
            else
                viewId = obj.ViewIds(viewIndex(:));
            end
        end

        %------------------------------------------------------------------
        function tf = hasView(obj, viewIds)
            if isempty(obj.ViewIds)
                tf = false;
            else
                tf = ismember(viewIds, obj.ViewIds);
            end
        end

        %------------------------------------------------------------------
        function medoidIdx = findMedoidFeature(~, features, isBinaryFeature, N)
            if isBinaryFeature
                scores   = vision.internal.matchFeatures.metricHamming(uint8(features), uint8(features), N, N, 'single');
            else
                scores   = vision.internal.matchFeatures.metricSSD(features, features, N, N, 'single');
            end
            [~, medoidIdx]    = min(sum(scores));
        end
    end
    %----------------------------------------------------------------------
    % Validation
    %----------------------------------------------------------------------
    methods (Access = protected)
        %------------------------------------------------------------------
        function idArray = checkPointIds(obj, idArray, varargin)

           validatePointIds(obj, idArray);
           idArray = reshape(idArray,1,[]);
		   
           if numel(idArray) ~= numel(unique(idArray', 'rows'))
            coder.internal.error('vision:worldpointset:duplicateIds');
           end

           if nargin > 2
			   % This check is only used when adding new points
			   % Check if the input PointIds are already present in the set
			   if any(ismember(idArray, obj.PointIds)) 
                   coder.internal.error('vision:worldpointset:duplicateIds2');
			   end
			   % Check if the input PointIds and WorldPoints are the same size
               worldPoints = varargin{1};
               if length(idArray)~=height(worldPoints)
                   coder.internal.error('vision:worldpointset:wrongIDSize');
               end
		   else
			   % This check is only used when manipulating existing points
			   % Check if the input PointIds that we are looking for exist
			    if ~isempty(obj.PointIds) && ~all(ismember(idArray, obj.PointIds))  
                   coder.internal.error('vision:worldpointset:unavailableIds');
			    end
           end

        end
        %------------------------------------------------------------------
        function viewIds = checkViewIds(obj, viewIds)
            viewIds = vision.internal.inputValidation.checkViewIds(...
                viewIds, false, obj.ClassName, 'viewIds');
        end

        %------------------------------------------------------------------
        function viewId = checkViewId(obj, viewId)
            viewId = vision.internal.inputValidation.checkViewIds(...
                viewId, true, obj.ClassName, 'viewId');
        end

        %------------------------------------------------------------------
        function checkIfViewIsMissing(obj, viewId)
            vision.internal.inputValidation.checkIfViewIsMissing(...
                obj.ViewIds, viewId);
        end

        %------------------------------------------------------------------
        function checkMissingViewInViewTable(obj, isSinglePoint, viewIdsToUpdate, allViewIds)
            if obj.isSimMode()
                if isSinglePoint
                    vision.internal.inputValidation.checkIfViewIsMissing(...
                        allViewIds, viewIdsToUpdate);
                else
                    vision.internal.inputValidation.checkIfViewIsMissing(...
                        allViewIds, vertcat(viewIdsToUpdate{:}));
                end
            else

                for i = 1:numel(viewIdsToUpdate)
                    vision.internal.inputValidation.checkIfViewIsMissing(...
                        allViewIds, viewIdsToUpdate{i});
                end
            end
        end

        %------------------------------------------------------------------
        function viewIndex = checkIfProjectionIsMissing(obj, viewId, pointIndices)
			viewIndex = viewId2ViewIndex(obj, viewId);

            % Find the first missing projection
            isMissing = find(~obj.CorrespondencesInternal(pointIndices, viewIndex), 1);
            if ~isempty(isMissing)
                coder.internal.error('vision:worldpointset:missingProjection', ...
                pointIndices(isMissing(1)), viewId(1));
            end

        end

        %------------------------------------------------------------------
        function checkIfNoPointIsAdded(obj)
            coder.internal.errorIf(obj.Count == 0, 'vision:worldpointset:noWorldPoints');
        end

        %------------------------------------------------------------------
        function checkWorldPoints(obj, worldPoints, varargin)
            types = {'single', 'double'};
            attrs = {'finite', 'nonempty', 'nonsparse', '2d', 'ncols', 3};
            if obj.isSimMode()
                if nargin == 3 % Number of points specified
                    attrs(end+1:end+2) = {'nrows', varargin{1}};
                end
                validateattributes(worldPoints, types, attrs, obj.ClassName, 'worldPoints');
            else
                if nargin == 3 % Number of points specified
                    attrsUpdated = cell(1, 2);
                    attrsUpdated{1} = 'nrows';
                    attrsUpdated{2} = varargin{1};
                    validateattributes(worldPoints, types, {attrs{:}, attrsUpdated{:}}, obj.ClassName, 'worldPoints');
                else
                    validateattributes(worldPoints, types, attrs, obj.ClassName, 'worldPoints');
                end
            end
        end

        %------------------------------------------------------------------
        function validatePointIds(obj, pointIds)
            validateattributes(pointIds, {'numeric'}, ...
                {'vector', 'nonsparse', 'positive', 'integer'}, obj.ClassName, 'pointIds');
        end

        %------------------------------------------------------------------
        function checkPointIndices(obj, pointIndices)
            validateattributes(pointIndices, {'numeric'}, ...
                {'vector', 'nonsparse', 'positive', 'integer', ...
                '<=', obj.Count}, obj.ClassName, 'pointIndices');
        end

        %------------------------------------------------------------------
        function checkFeatureIndices(obj, featureIndices, numFeatures)
            validateattributes(featureIndices, {'numeric'}, ...
                {'vector', 'nonsparse', 'positive', 'integer', ...
                'numel', numFeatures}, obj.ClassName, 'featureIndices');
        end

        %------------------------------------------------------------------
        function checkPointTracks(obj, tracks)
            validateattributes(tracks, {'pointTrack'}, ...
                {'nonempty','vector'}, obj.ClassName, 'pointTracks');
        end

        %------------------------------------------------------------------
        function [viewIds, poses, features] = checkViewTable(obj, viewTable)
            
            if obj.isSimMode()
                variableNames = viewTable.Properties.VariableNames;
                hasCorrectVariables = isequal(variableNames, ...
                    {'ViewId', 'AbsolutePose', 'Features', 'Points'});
            else
                variableNames = fieldnames(viewTable);
                hasCorrectVariables = isequal(variableNames, ...
                    {'ViewId'; 'AbsolutePose'; 'Features'; 'Points'});
            end
            
            coder.internal.errorIf(~hasCorrectVariables, 'vision:viewSet:optionalColumnsInvalid', ...
                              'viewTable', 'ViewId, AbsolutePose, Features, Points', ' ');

            viewIds  = checkViewIds(obj, viewTable.ViewId);
            poses = checkPoses(obj, viewTable.AbsolutePose); 
            % Skip checking feature points as they are not used
            features = checkFeaturesArray(obj, viewTable.Features);
        end
        
        %------------------------------------------------------------------
        function poses = checkPoses(this, poses)

            validateattributes(poses, {'rigidtform3d','rigid3d'}, {'vector'}, ...
                this.ClassName, 'AbsolutePose');
        end

        %------------------------------------------------------------------
        function featuresArray = checkFeaturesArray(this, featuresArray)
            if this.isSimMode() && isnumeric(featuresArray{1})
                vision.internal.inputValidation.checkFeatures(...
                    vertcat(featuresArray{:}), this.ClassName, 'Features');
            else % binaryFeatures
                for i = 1:numel(featuresArray)
                    vision.internal.inputValidation.checkFeatures(...
                        featuresArray{i}, this.ClassName, 'Features');
                end
            end
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

            that.WorldPoints                 = this.WorldPoints;
            that.ViewIds                     = this.ViewIds;
            that.PointIds                    = this.PointIds;

            that.ViewingDirection            = this.ViewingDirection;
            that.DistanceLimits              = this.DistanceLimits;
            that.RepresentativeViewId        = this.RepresentativeViewId;
            that.RepresentativeFeatureIndex  = this.RepresentativeFeatureIndex;

            that.CorrespondencesInternal     = this.CorrespondencesInternal;
            that.Version                     = this.Version;
        end
    end

    methods (Static, Hidden)
        %------------------------------------------------------------------
        function this = loadobj(that)

            % that - struct
            % this - object

            this = worldpointset();

            this.WorldPoints                = that.WorldPoints;
            this.ViewIds                    = that.ViewIds;

            if that.Version > 1.0
                this.ViewingDirection           = that.ViewingDirection;
                this.DistanceLimits             = that.DistanceLimits;
                this.RepresentativeFeatureIndex = that.RepresentativeFeatureIndex;
                this.RepresentativeViewId       = that.RepresentativeViewId;
            end

            if that.Version > 1.1
                this.PointIds               = that.PointIds;
            end

            this.CorrespondencesInternal    = that.CorrespondencesInternal;
        end
    end

    methods(Static, Access = private)
        %------------------------------------------------------------------
        function out = isSimMode()
            % Check if simulation mode
            out = isempty(coder.target);
        end
    end
end
