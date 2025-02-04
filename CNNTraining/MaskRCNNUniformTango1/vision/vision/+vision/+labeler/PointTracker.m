classdef PointTracker < vision.labeler.AutomationAlgorithm & vision.labeler.mixin.Temporal
    %PointTracker Automation algorithm to track ROIs using KLT.
    %   PointTracker is a temporal automation algorithm for tracking one or
    %   more Rectangle ROI's using KLT Feature Point Tracking in the Video
    %   Labeler and the Ground Truth Labeler Apps. Ground Truth Labeler App
    %   requires that you have the Automated Driving Toolbox(TM).
    %
    %   See also videoLabeler, vision.labeler.AutomationAlgorithm,
    %   vision.labeler.mixin.Temporal, vision.PointTracker.

    % Copyright 2017-2024 The MathWorks, Inc.
    
    %----------------------------------------------------------------------
    % Algorithm Description
    %----------------------------------------------------------------------
    properties (Constant)
        %Name Algorithm Name
        %   Character vector specifying name of algorithm.
        Name            = vision.getMessage('vision:labeler:PointTrackerName');
        
        %Description Algorithm Description
        %   Character vector specifying short description of algorithm.
        Description     = vision.getMessage('vision:labeler:PointTrackerDesc');
        
        %UserDirections Algorithm Usage Directions
        %   Cell array of character vectors specifying directions for
        %   algorithm users to follow in order to use algorithm.
        UserDirections  = {...
            vision.getMessage('vision:labeler:PointTrackerROISelection'),...
            vision.getMessage('vision:labeler:PointTrackerRun'),...
            vision.labeler.AutomationAlgorithm.getDefaultUserDirections('review'),...
            [vision.labeler.AutomationAlgorithm.getDefaultUserDirections('rerun') ' '...
            vision.getMessage('vision:labeler:PointTrackerNote')],...
            vision.labeler.AutomationAlgorithm.getDefaultUserDirections('accept')};
    end
    
    %----------------------------------------------------------------------
    % Tracker Properties
    %----------------------------------------------------------------------
    properties
        %InitialLabels Set of labels at algorithm initialization
        %   Table with columns Name, Type and Position for labels marked at
        %   initialization.
        InitialLabels
        
        %ImageSize Image size
        %   Size of image being processed.
        ImageSize
        
        %Trackers Cell array of trackers
        %   Cell array of point tracker objects.
        Trackers
        
        %OldPoints Cell array of points being tracked
        %   Cell array of points (feature point locations) being tracked in
        %   the same order as Trackers.
        OldPoints
        
        %IndexList Index list to InitialLabels
        %   Array mapping trackers to corresponding labels in
        %   InitialLabels.
        IndexList
        
        %BBoxPoints Bounding Box points
        %   Cell array of bounding box corner points for each tracker.
        BBoxPoints

        %Angle Angle of rotation for rotated rectangle
        %   Value of yaw, in degrees, of rotated rectangles.
        Angle

        %Rotatable Logical declaration of if ROI is a rotated rectangle
        %   true if ROIs are rotated rectangles, false otherwise.
        Rotatable
        
        %MinimumRequiredPoints Minimum feature points to continue track
        %   Minimum number of feature points to continue tracking
        MinimumRequiredPoints = 4

        % Flag to indicate presence of attributes.
        HasAttributes = false;

        % Attribute Data
        %   struct array to store label attributes.
        AttribData;

    end
    
    %----------------------------------------------------------------------
    % Settings Properties
    %----------------------------------------------------------------------
    properties
        %FeatureDetectorNames List of feature detectors
        %   Cell array of character vectors containing message catalog
        %   specifier names of possible feature detectors.
        FeatureDetectorNames = {
            'MinEigenFeature'
            'HarrisFeature'
            'FASTFeature'
            'BRISKFeature'
            'KAZEFeature'
            'SURFFeature'
            'MSERFeature'};
        
        %FeatureDetectorHandles Function handles to feature detectors
        %   Function handles to feature detectors used to initialize point
        %   tracker.
        FeatureDetectorHandles = {
            @detectMinEigenFeatures
            @detectHarrisFeatures
            @detectFASTFeatures
            @detectBRISKFeatures
            @detectKAZEFeatures
            @detectSURFFeatures
            @detectMSERFeatures};
        
        %FeatureDetectorSelection Index of selected features detector
        %   Index to FeatureDetectorNames and FeatureDetectorHandles
        %   containing selected feature detector
        FeatureDetectorSelection = 1
        
        % Feature List Box for the settings dialog
        FeatureSelectionList
    end
    
    %----------------------------------------------------------------------
    % Setup
    %----------------------------------------------------------------------
    methods
        function flag = supportsReverseAutomation(~)
            flag = true;
        end  
        
        function isValid = checkLabelDefinition(~, labelDef)
            
            % Only Rectangular ROI label definitions are valid for the
            % Point Tracker.
            isValid = labelDef.Type==labelType.Rectangle || ...
                labelDef.Type==labelType.RotatedRectangle;
        end
        
        function isReady = checkSetup(algObj, videoLabels)
            
            % There must be at least one ROI label before the algorithm can
            % be executed.
            assert(~isempty(videoLabels), 'There are no ROI labels to track. Draw at least one ROI label.');

            % Check if there are any attributes.
            if any(ismember(videoLabels.Properties.VariableNames,'Attributes'))
               algObj.HasAttributes = true;
            end
            
            isReady = true;   
        end
        
        function settingsDialog(this)
            % Create a dialog listing feature detectors to choose from.
            
            % Get translated message strings for feature detector names.
            featureDetectorStrings = cellfun(...
                @(s)vision.getMessage(sprintf('vision:labeler:%s', s)), ...
                this.FeatureDetectorNames, 'UniformOutput', false);
            
            promptString = vision.getMessage('vision:labeler:FeatureDetectorSelect');
            nameString   = sprintf('%s %s', this.Name, ...
                vision.getMessage('vision:labeler:Settings'));

            dlgSize = [400 250];
            monitorPos = get(0, 'MonitorPositions');
            monitorPos = monitorPos(1,:);
            center = monitorPos(1:2)+[monitorPos(3), monitorPos(4)]/2;
            dlgPos = round([center-dlgSize/2 dlgSize]); 

            parentObj = uifigure('Name',nameString, 'Position', dlgPos,...
                'WindowStyle', 'modal','Resize','off');
            matlab.graphics.internal.themes.figureUseDesktopTheme(parentObj);


            grid = uigridlayout(parentObj,[3 2],...
                "RowHeight", {30,'1x',30});

            promptText = uilabel('Parent', grid,...
                'Text',promptString,'WordWrap','on',...
                'HorizontalAlignment','left');

            promptText.Layout.Row = 1;
            promptText.Layout.Column = 1:2;

            this.FeatureSelectionList = uilistbox('Parent', grid,...
                'Items', featureDetectorStrings,...
                'Multiselect', 'off',...
                'Value', featureDetectorStrings{1},...
                'ValueChangedFcn',@doselectFeature);

            this.FeatureSelectionList.Layout.Row = 2;  
            this.FeatureSelectionList.Layout.Column = 1:2;
          
            this.FeatureDetectorSelection = this.FeatureSelectionList.Value;

            okButton = uibutton('Parent', grid,...
                'ButtonPushedFcn',@(e,d)onOK(this,e,d),...
                'Text',getString(message('MATLAB:uistring:popupdialogs:OK')),...
                'FontSize', 12);
            okButton.Layout.Row = 3;
            okButton.Layout.Column = 1;

            cancelButton = uibutton('Parent', grid,...
                'ButtonPushedFcn',@(e,d)onCancel(this,e,d),...
                'Text',getString(message('MATLAB:uistring:popupdialogs:Cancel')),...
                'FontSize', 12);
            cancelButton.Layout.Row = 3;
            cancelButton.Layout.Column = 2;

            function doselectFeature(~,evt)
               evt.Source.Value = evt.Value;
            end  

            function onOK(this,~, ~)
               listBox = this.FeatureSelectionList;
               idx = find(strcmp(listBox.Value,featureDetectorStrings));
               this.FeatureDetectorSelection = idx;
               fig = ancestor(listBox,'figure');
               close(fig);
            end

            function onCancel( this,~,~)
               listBox = this.FeatureSelectionList;
               fig = ancestor(listBox,'figure');
               close(fig);
            end
        end
    end
    
    %----------------------------------------------------------------------
    % Execution
    %----------------------------------------------------------------------
    methods
        function initialize(algObj, I, videoLabels)
            
            % Cache initial labels marked during setup. These will be used
            % as initializations for point trackers.
            algObj.InitialLabels = videoLabels;
            
            algObj.Trackers   = {};
            algObj.OldPoints  = {};
            algObj.IndexList  = [];
            algObj.BBoxPoints = {};
            algObj.AttribData = {};
            algObj.Angle      = {};
            algObj.Rotatable  = {};
            
            algObj.ImageSize  = size(I);

            % Double check to ensure non-empty attributes.
            if (algObj.HasAttributes)
                if ~isempty(videoLabels.Attributes{1}) 
                    algObj.HasAttributes = true;
                else
                    algObj.HasAttributes = false;
                end
            end

        end
        
        function autoLabels = run(algObj, I)
            
            autoLabels = [];
            
            % Check which labels were marked on this frame. These will be
            % used as initializations for the trackers.
            idx = algObj.InitialLabels.Time==algObj.CurrentTime;
            
            % Convert to grayscale
            Igray = im2gray(I);
            
            if any(idx)
                % Initialize new trackers for each of the labels marked on
                % this frame.
                idx = find(idx);
                algObj.IndexList = [algObj.IndexList; idx(:)];
                
                for n = idx'
                    initializeTrack(algObj, Igray, n);
                end
                
            else
                % Update old trackers
                numTrackers = numel(algObj.Trackers);
                for n = 1 : numTrackers
                    newLabel = updateTrack(algObj, Igray, n);
                    autoLabels = [autoLabels newLabel]; %#ok<AGROW>
                end
            end
            
        end
        
        function terminate(algObj)
            
            % Release all trackers
            for n = 1 : numel(algObj.Trackers)
                tracker = algObj.Trackers{n};
                if ~isempty(tracker)
                    release(tracker);
                end
            end
            
            % Empty arrays
            algObj.InitialLabels  = [];
            algObj.OldPoints      = {};
            algObj.IndexList      = [];
            algObj.BBoxPoints     = {};
            algObj.AttribData     = {};
            algObj.Angle          = {};
            algObj.Rotatable      = {};

        end
    end
    
    %----------------------------------------------------------------------
    % Private methods
    %----------------------------------------------------------------------
    methods (Access = private)
        function initializeTrack(algObj, Igray, n)
            
            % Find region of interest for feature computation
            bboxPos = algObj.InitialLabels{n, 'Position'};

            % Extract cell value if tracking both axis-aligned rectangles
            % and rotated rectangles.
            if iscell(bboxPos)
                bbox = bboxPos{1};
            else
                bbox = bboxPos;
            end

            bboxNew = algObj.computeReducedBoundingBox(bbox);

            % Determine if ROI is a rotated rectangle.
            if size(bbox,2) == 4
                algObj.Rotatable{end+1} = false;
                algObj.Angle{end+1}     = 0;
            else
                algObj.Rotatable{end+1} = true;
                algObj.Angle{end+1}     = bbox(5);
            end
            
            % Detect feature locations using the selected feature detector
            detectFeatures = algObj.FeatureDetectorHandles{algObj.FeatureDetectorSelection};
            points = detectFeatures( Igray, 'ROI', bboxNew );
            
            if ~isempty(points)
                % Construct a tracker
                tracker = vision.PointTracker('MaxBidirectionalError', 2);
                
                % Initialize tracker with detected features
                points = points.Location;
                tracker.initialize(points, Igray);
                bboxpoints = bbox2points(bbox);
                
            else
                % No features were found for this ROI. 
                tracker     = [];
                points      = [];
                bboxpoints  = [];
            end
            
            % Cache the tracker and information associated with it, to be
            % used later when updating the tracks.
            algObj.Trackers{end+1}    = tracker;
            algObj.OldPoints{end+1}   = points;
            algObj.BBoxPoints{end+1}  = bboxpoints;

            % Store the initial attributes if the ROI has custom attributes
            if algObj.HasAttributes
                algObj.AttribData{end+1} = algObj.InitialLabels.Attributes{n};
            end
        end
        
        function autoLabels = updateTrack(algObj, Igray, n)
            
            autoLabels = [];
            
            tracker = algObj.Trackers{n};
            
			% No update needed for ROIs with no features.
            if isempty(tracker)
                return;
            end
            
            % Track points into new frame.
            [points, isFound] = step(tracker, Igray);
            
            visiblePoints   = points(isFound,:);
            bboxPoints      = algObj.BBoxPoints{n};
            oldPoints       = algObj.OldPoints{n};
            oldInliers      = oldPoints(isFound,:);

            numPoints = size(visiblePoints, 1); 
            
            % Some points may be lost, continue tracking only if
            % there are enough points.
            if numPoints >= algObj.MinimumRequiredPoints
                
                % Estimate geometric transformation between old and
                % new points, eliminating outliers.
                [tform, inlierIndex, status] = ...
                    estgeotform2d(oldInliers, ...
                    visiblePoints, 'Similarity', 'MaxDistance', 4);
                
                visiblePoints = visiblePoints(inlierIndex,:);
                
                if ~algObj.Rotatable{n}
                    angle = cast(0,'like',bboxPoints);
                    rotatable = false;
                else
                    % Add estimated geometric transformation angle change.
                    algObj.Angle{n} = algObj.Angle{n} + tform.RotationAngle;

                    % Ensure that the angle is within (-180, 180] degrees.
                    if algObj.Angle{n} <= -180
                        algObj.Angle{n} = algObj.Angle{n} + 360;
                    elseif algObj.Angle{n} > 180
                        algObj.Angle{n} = algObj.Angle{n} - 360;
                    end

                    angle = cast(algObj.Angle{n},'like',bboxPoints);
                    rotatable = true;
                end

                if status ~= 0
                    % Break out of the tracking loop, tracking
                    % failed.
                    return;
                end
                
                % Transform the bounding box using estimated
                % geometric transformation
                bboxPoints = transformPointsForward(tform, bboxPoints);

                % Compute bounding box
                newPosition = computeROIPosition(algObj, bboxPoints, rotatable, angle);
                
                % Add a new label at newPosition
                idx = algObj.IndexList(n);
                type = algObj.InitialLabels{idx,'Type'};
                name = algObj.InitialLabels{idx,'Name'};
                autoLabels = struct('Type', type, 'Name', name, 'Position', newPosition);
                
                % Update old points
                algObj.OldPoints{n}   = visiblePoints;
                algObj.BBoxPoints{n}  = bboxPoints;
                tracker.setPoints(visiblePoints);

                if(algObj.HasAttributes) 
                    autoLabels.Attributes = algObj.AttribData{idx};
                end
            end
        end
        
        function newBboxClipped = computeReducedBoundingBox(algObj, bbox)
            
            % Use 80% of bounding box area.
            if size(bbox,2) == 4
                centroid = bbox(1:2) + bbox(3:4)/2;
            else
                centroid = bbox(1:2);
            end
            newExtent = 0.8 * bbox(3:4);
            
            newBbox = [centroid - newExtent/2 newExtent];
            newBboxClipped = vision.internal.detector.clipBBox(newBbox,algObj.ImageSize);
        end
        
        function position = computeROIPosition(algObj, bboxPoints, rotatable, angle)
            
            xs = bboxPoints(:,1);
            ys = bboxPoints(:,2);
            
            imHeight    = algObj.ImageSize(1);
            imWidth     = algObj.ImageSize(2);

            % Compute the new position of the ROI.
            if ~rotatable
                % Clamp position to image boundaries. 
                Xmin = max( min( min(xs), imWidth  ), 1 );
                Ymin = max( min( min(ys), imHeight ), 1 );
                Xmax = max( min( max(xs), imWidth  ), 1 );
                Ymax = max( min( max(ys), imHeight ), 1 );

                position = [Xmin Ymin Xmax-Xmin Ymax-Ymin];
            else
                % Calculate the intended center vertices for the new bbox.
                xCtrTemp = bboxPoints(1,1) + (bboxPoints(3,1) - bboxPoints(1,1))/2;
                yCtrTemp = bboxPoints(1,2) + (bboxPoints(3,2) - bboxPoints(1,2))/2;

                % Clamp center coordinates to image boundaries.
                xCtr = max( min( xCtrTemp, imWidth  ), 1 );
                yCtr = max( min( yCtrTemp, imHeight ), 1 );

                % Calculate the difference of the center coordinate for
                % width and height adjustments.
                xDiff = abs(xCtrTemp - xCtr);
                yDiff = abs(yCtrTemp - yCtr);

                switch angle
                    case {0, 180}
                        w = abs(bboxPoints(2,1) - bboxPoints(1,1) - xDiff);
                        h = abs(bboxPoints(3,2) - bboxPoints(2,2) - yDiff);
                    case {90, -90}
                        w = abs(bboxPoints(2,2) - bboxPoints(1,2) - yDiff);
                        h = abs(bboxPoints(3,1) - bboxPoints(2,1) - xDiff);
                    otherwise
                        % Calculate the midpoints of both the top and
                        % bottom, as well as the left and right sides of
                        % the rotated rectangle.
                        widthMidpoints = [
                            (bboxPoints(2,1) + bboxPoints(1,1))/2,...
                            (bboxPoints(2,2) + bboxPoints(1,2))/2;...
                            (bboxPoints(3,1) + bboxPoints(4,1))/2,...
                            (bboxPoints(3,2) + bboxPoints(4,2))/2];

                        heightMidpoints = [
                            (bboxPoints(4,1) + bboxPoints(1,1))/2,...
                            (bboxPoints(4,2) + bboxPoints(1,2))/2;...
                            (bboxPoints(3,1) + bboxPoints(2,1))/2,...
                            (bboxPoints(3,2) + bboxPoints(2,2))/2];

                        % Use the clamped center coodinate and the distance
                        % from the midpoints of the width and height to
                        % determine if either the width or height needs to
                        % be adjusted. This is needed when a rotated
                        % bounding box has reached the image frame and
                        % continues onward.
                        h = max(round(min(hypot([xCtr; xCtr] - widthMidpoints(:,1), ...
                            [yCtr; yCtr] - widthMidpoints(:,2)))),1)*2;
                        w = max(round(min(hypot([xCtr; xCtr] - heightMidpoints(:,1), ...
                                   [yCtr; yCtr] - heightMidpoints(:,2)))),1)*2;
                end
                
                position = [xCtr yCtr w h angle];
            end
        end
    end
end
