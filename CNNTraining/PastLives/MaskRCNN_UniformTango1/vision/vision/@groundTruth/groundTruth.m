%groundTruth Object for storing ground truth labels
%
%   groundTruth is used to store ground truth labels for a collection of
%   images, a video, a sequence of images, or a custom data source.
%
%   gTruth = groundTruth(dataSource, labelDefs, labelData) returns gTruth,
%   an object containing ground truth labels. dataSource is an object of
%   type groundTruthDataSource describing a collection of images, video,
%   image sequence, or custom data source. labelDefs is a table defining
%   the labels as described in the LabelDefinitions property below.
%   labelData is a table or timetable of label data as described in the
%   LabelData property below.
%
%   groundTruth properties:
%   DataSource          - A <a href="matlab:help('groundTruthDataSource')">groundTruthDataSource</a> object. 
%
%   LabelDefinitions    - A table of definitions for ROI and Scene label
%                         categories. Use <a href="matlab:help('labelDefinitionCreator')">labelDefinitionCreator</a> to create
%                         the label definitions table. The table must
%                         contain two to four columns: Name, Type
%                         PixelLabelID, Group and Description (optional), as
%                         described below:
%                         * Name is a character vector specifying the name
%                           of the label category.
%                         * Type is a labelType enumeration specifying the
%                           type of label category.
%                         * Group is a character vector specifying the
%                           group of label category.
%                         * PixelLabelID specifies the pixel label values
%                           used to represent a label category.
%                           PixelLabelID is required when any Type is
%                           labelType.PixelLabel. PixelLabelID values must
%                           be a scalar, column vector, or M-by-3 matrix of
%                           integer valued label IDs. 
%                         * Description is a character vector describing
%                           the label category.
%
%                         For example, a label definition table with 3
%                         label categories:
%
%                          Name        Type       PixelLabelID    Group 
%                         ______    __________    ____________    ______
% 
%                         'Cars'    Rectangle         []          'Vehicle'
%                         'Lane'    Line              []          'None'
%                         'Road'    PixelLabel        [3]         'None'
%
%                         LabelDefinitions is read-only.
%
%   LabelData           - Label data for each ROI and Scene label organized
%                         in a <a href="matlab:help('table')">table</a> or <a href="matlab:help('timetable')">timetable</a>. Each column holds labels
%                         for Rectangle, Rotated Rectangle, Line, ProjectedCuboid, or Scene 
%                         labels for a single label category. For PixelLabel 
%                         labels, a single column named 'PixelLabelData' is 
%                         used to hold the label data for all label 
%                         categories of labelType.PixelLabel. 
%
%                         The table has as many rows as there are images or
%                         timestamps in the data source. LabelData is a
%                         timetable when Source is a groundTruthDataSource
%                         with timestamps. Otherwise it is a table.
%                         LabelData is read-only.
%
%                         The label type determines the storage format of
%                         label data:
%
%                         labelType.Rectangle
%                         -------------------
%                         Labels in each row are stored as M-by-4
%                         matrices of [x, y, width, height] bounding box
%                         locations.
%
%                         labelType.RotatedRectangle
%                         --------------------------
%                         Labels in each row are stored as M-by-5
%                         matrices of [xctr, yctr, width, height, yaw] bounding box
%                         locations.
%
%                         labelType.Line
%                         --------------
%                         Labels in each row are stored as M-by-1 cell
%                         arrays. Each element of the cell array holds
%                         [x, y] locations for points used to mark the
%                         polyline.
%
%                         labelType.ProjectedCuboid
%                         -------------------------
%                         Labels in each row are stored as M-by-8 matrices of
%                         [x1, y1, width1, height1, x2, y2, width2, height2]. 
%                         It represents the locations of the primary and 
%                         secondary faces.
%
%                         labelType.PixelLabel
%                         --------------------
%                         Pixel label data for all label categories is
%                         represented by a single label matrix. The label
%                         matrix must be stored on disk as an uint8 image,
%                         and the image filename must be specified as a
%                         character vector in the LabelData table. The
%                         label matrix must have 1 or 3 channels. For a
%                         3-channel label matrix, the RGB pixel values
%                         represent label IDs.
%
%                         labelType.Scene
%                         ---------------
%                         Labels are stored as logical values representing
%                         presence or absence of the scene label.
%
%                         labelType.Custom
%                         ----------------
%                         Labels are stored as provided in the table.                       
%
%    Alternatively, for ROI label data grouped by label type, a single column labeled 'ROILabelData'
%    can be used. The data here is stored as a structure containing atleast one of the following fields -
%         RectangleData - A M1x2 cell array containing M1 rectangle
%                         annotations. The first column contains the
%                         the 1x4 rectangle as [x y w h]. The second
%                         column holds the corresponding label as a string.
%  RotatedRectangleData - A M1x2 cell array containing M1 rectangle
%                         annotations. The first column contains the
%                         the 1x5 rectangle as [xctr yctr w h yaw]. The second
%                         column holds the corresponding label as a string.
%         PolygonData   - A M2x2 cell array containing M2 Polygon
%                         annotations.In each row, the first column contains the
%                         the P1x2 - x,y vertices go the polygon. The second
%                         column holds the corresponding label as a string.
%         LineData      - A M2x2 cell array containing M2 Polygon
%                         annotations.In each row, the first column contains the
%                         the P1x2 - x,y vertices go the polygon. The second
%                         column holds the corresponding label as a string.
%         ProjCuboidData- A M1x2 cell array containing M1 rectangle
%                         annotations. The first column contains the
%                         the 1x8 cuboid as [x1 y1 w1 h1 x2 y2 w2 h2]. The second
%                         column holds the corresponding label as a string.
%     This format of label data can be used to preserve the implicit ordering/stacking order
%     of the rois.
%
%   groundTruth methods:
%   selectLabelsByName  - Return groundTruth object for selected label
%                         names.
%   selectLabelsByType  - Return groundTruth object for selected label
%                         types.
%   selectLabelsByGroup - Return groundTruth object for selected label
%                         groups.
%   changeFilePaths     - Change DataSource and PixelLabelData file paths.
%   gatherLabelData     - Gather annotation data from the groundTruth
%                         object.
%   merge               - Merge two or more groundTruth objects.
%
%
%   Notes
%   -----
%   - Use VideoReader, imageDatastore, or custom reader function to access
%     images from the original data source.
%   - groundTruth created using a video data source is guaranteed to remain
%     consistent only for the same computer platform since it relies on 
%     video reading capabilities of the Operating System. If cross-platform  
%     use is required convert the video into a sequence of image files with
%     associated time stamps.
%   - Ground truth data that is not an ROI (Rectangle, Rotated Rectangle, Line, ProjectedCuboid,
%     PixelLabel) or Scene label can be added to groundTruth by providing 
%     a label definition whose labelType is Custom.
%   - Create training data for an object detector from arrays of
%     groundTruth objects using the <a href="matlab:help('objectDetectorTrainingData')">objectDetectorTrainingData</a> function.
%   - Create training data for semantic segmentation from arrays of
%     groundTruth objects using the <a href="matlab:help('pixelLabelTrainingData')">pixelLabelTrainingData</a> function.
%   - The storage format for labels containing a hierarchy of
%     sublabels or attributes is described <a href="matlab:helpview('vision','labelHierarchy')">here</a>.
%   - For Rectangle, Rotated Rectangle, Line and ProjectedCuboid label types, if the ground 
%     truth data is not a floating-point array, then it will be cast to single.
%   
%
%   Example 1: Create ground truth for stop signs and cars
%   ------------------------------------------------------
%   % Create a data source from a collection of images
%   data = load('stopSignsAndCars.mat');
%   imageFilenames = data.stopSignsAndCars.imageFilename(1:2)
%   imageFilenames = fullfile(toolboxdir('vision'), 'visiondata', imageFilenames);
%   dataSource = groundTruthDataSource(imageFilenames);
%     
%   % Define labels used to specify ground truth
%   names = {'stopSign'; 'carRear'};
%   types = [
%       labelType('Rectangle')
%       labelType('Rectangle')
%       ];
% 
%   labelDefs = table(names, types, 'VariableNames', {'Name', 'Type'})
% 
%   % Initialize label data for rectangle ROIs.
%   numRows = numel(imageFilenames);
%   stopSignTruth = {[856   318    39    41]; [445   523    52    54]};
%   carRearTruth = {[398   378   315   210]; [332   633   691   287]};
% 
%   % Construct table of label data
%   labelData = table(stopSignTruth, carRearTruth, 'VariableNames', names)
% 
%   % Create groundTruth object.
%   gTruth = groundTruth(dataSource, labelDefs, labelData)
%
%   Example 2: Create ground truth for lanes
%   ----------------------------------------
%   % Create a data source from image
%   dataSource = groundTruthDataSource({'stopSignTest.jpg'});
%
%   % Define labels used to specify ground truth
%   names = {'Lane'};
%   types = [labelType('Line')];
%   labelDefs = table(names, types, 'VariableNames', {'Name', 'Type'})
%
%   % 2 lane markers to the first frame.
%   laneMarkerTruth{1} = {[257 254;311 180] [327 183;338 205;374 250]};
%
%   % Construct table of label data
%   labelData = table(laneMarkerTruth, 'VariableNames', names)
%
%   % Create groundTruth object
%   gTruth = groundTruth(dataSource, labelDefs, labelData)
%
%   Example 3: Create ground truth for pixel labels
%   -----------------------------------------------
%   % Create data source
%   dataSource = groundTruthDataSource({'visionteam.jpg'});
%   
%   % Define pixel labels for Person and Background.
%   names = {'Person';'Background'};
%   types = [labelType('PixelLabel'); labelType('PixelLabel')];
%
%   % Define pixel label IDs. Label IDs 1 and 2 correspond to Person and
%   % Background, respectively.
%   pixelLabelID = {1; 2};
%
%   labelDefs = table(names, types, pixelLabelID, ...
%                     'VariableNames', {'Name', 'Type', 'PixelLabelID'})
%
%   % Specify location of pixel label data for visionteam.jpg
%   dataFile = {'visionteamPixelLabels.jpg'}
%   
%   % Construct table of label data for pixel label data
%   labelData = table(dataFile, 'VariableNames', {'PixelLabelData'})
%
%   % Create groundTruth object
%   gTruth = groundTruth(dataSource, labelDefs, labelData)
%   
%
%   See also groundTruthDataSource, labelType, objectDetectorTrainingData, 
%            pixelLabelTrainingData, labelDefinitionCreator.

% Copyright 2016-2023 The MathWorks, Inc.

classdef groundTruth < handle & matlab.mixin.CustomDisplay
    
    properties (Dependent)
        %DataSource Data source specifies as a groundTruthDataSource object
        %   A groundTruthDataSource object describing a collection of
        %   images, video file, image sequence or custom data source for
        %   the ground truth data.
        DataSource
    end
    
    properties (SetAccess = private, GetAccess = public)
        %LabelDefinitions Table of label definitions
        %   A table of definitions for ROI and Scene label categories
        %   containing three to five columns: Name, Type, PixelLabelID,
        %   Group and Description (optional).
        %   * Name is a character vector specifying the name of the label
        %     category.
        %   * Type is a labelType enumeration specifying the type of label
        %     category.
        %   * PixelLabelID specifies the pixel label values used to
        %     represent a label category. PixelLabelID is required when any
        %     Type is labelType.PixelLabel. PixelLabelID values must be a
        %     scalar, column vector, or M-by-3 matrix of integer valued
        %     label IDs.
        %   * Group is a character vector specifying the group of the
        %     label. If this column is not specified, all the labels will be
        %     assigned to a default group 'None'
        %   * Description is a character vector describing the label
        %     category.
        %
        %     For example, a label definition table with 3 label
        %     categories:
        %
        %     Name        Type       PixelLabelID    Group 
        %    ______    __________    ____________    ______
        %
        %    'Cars'    Rectangle         []          'None'
        %    'Lane'    Line              []          'None'
        %    'Road'    PixelLabel        [3]         'None'
        %
        %   LabelDefinitions is read-only.
        LabelDefinitions
        
        %LabelData A table or timetable of label data
        %   Label data for each ROI and Scene label organized in a table or
        %   timetable. Each column holds labels for Rectangle, Rotated Rectangle, Line, 
        %   ProjectedCuboid, or Scene labels for a single label category. 
        %   For PixelLabel labels, a single column named 'PixelLabelData' 
        %   is used to hold the label data for all label categories of
        %   labelType.PixelLabel.
        %
        %   The table has as many rows as there are images or timestamps in
        %   the data source. LabelData is a timetable when Source is a
        %   groundTruthDataSource with timestamps. Otherwise it is a table.
        %
        %   LabelData is read-only.
        LabelData
        
    end
    
    properties (Access = protected, Hidden)
        %Version
        %   Version number to allow compatible load and save.
        Version = ver('vision');
                
        PolygonOrder
        
        IsSourceROILabelData
    end
    
    properties (Access = private)
        %DSource Private container for DataSource.
        DSource
    end
    
    properties(Access = private, Dependent)
        %SourceType The type of datasource.
        SourceType
    end
    
    methods (Hidden)
        % Stub method declarations defined in separate files.
        [timeRanges,labels] = getSceneTimeRanges(this);
        filenames = writeSceneVideosForTimeRanges(this, timeRanges, rootFolder, folderNames, nvp);
    end

    methods
        % Stub method declarations defined in separate files.
        gTruth = merge(gTruth1, varargin);
    end

    methods(Static, Access = private)
        % Stub method declarations defined in separate files. 
        def = mergeLabelDefinitions(allDefinitions)
    end

    methods
        %------------------------------------------------------------------
        function this = groundTruth(dataSource, labelDefs, labelData)
            
            this.DataSource            = dataSource;
            this.LabelDefinitions      = vision.internal.labeler.validation.checkLabelDefinitions(labelDefs);

            [labelData, isROILabelData]= vision.internal.labeler.validation.checkLabelData(labelData, this.DSource, this.LabelDefinitions);
            
            % Is input data in ROiLabelData format
            this.IsSourceROILabelData = isROILabelData;
            
            if(isROILabelData)
                [this.LabelData, this.PolygonOrder] = this.roiLabelData2LabelData(labelData);
            else
                 this.LabelData = labelData;               
            end
        end
        
        %------------------------------------------------------------------
        function gTruth = selectLabels(this, labels)
            %selectLabels Select ground truth data for a set of labels.
            %   gtLabel = selectLabels(gTruth, labelNames) returns a new
            %   groundTruth object with only the labels specified by
            %   labelNames. labelNames can be string array or cell array of
            %   character vectors
            %
            %   gtLabel = selectLabels(gTruth, types) returns a new
            %   groundTruth object with all labels of type types.
            %   types is a labelType enumeration.
            %
            %   Example 1: Select ground truth data by labelName and types
            %   ----------------------------------------------------------
            %   % Add the image directory to the MATLAB path
            %   imageDir = fullfile(matlabroot, 'toolbox', 'vision', 'visiondata', 'stopSignImages');
            %   addpath(imageDir);
            %
            %   % Load the groundTruth object
            %   load('stopSignsAndCarsGroundTruth.mat');
            %
            %   % View the label definitions
            %   stopSignsAndCarsGroundTruth.LabelDefinitions
            %
            %   % Obtain the ground truth data for labelName 'stopSign'
            %   stopSignGroundTruth = selectLabels(stopSignsAndCarsGroundTruth, ...
            %                                     'stopSign');
            %
            %   % Obtain the ground truth data for labelType Rectangle
            %   rectGroundTruth = selectLabels(stopSignsAndCarsGroundTruth, ...
            %                                  labelType.Rectangle);
            %
            %   % Obtain ground truth for 'carRear' and 'carFront'
            %   carGroundTruth = selectLabels(stopSignsAndCarsGroundTruth, ...
            %                                 {'carRear', 'carFront'});
            %
            %   % Remove the image directory from the path
            %   rmpath(imageDir); 
            
            % Handle array indexing for this method
            if numel(this)>1
                for n = 1 : numel(this)
                    gTruth(n) = selectLabels(this(n), labels); %#ok<AGROW>
                end
                return;
            end
            
            allLabelNames = this.LabelDefinitions.Name;
            labelTypes = this.LabelDefinitions.Type;
            
            % Validate input and convert to label names.
            labelNames = vision.internal.groundTruthHelper.validateLabelNameOrType(labels, allLabelNames,labelTypes);
            
            % Find column indices into LabelData.
            indexList = vision.internal.groundTruthHelper.labelName2Index(labelNames, allLabelNames);          
                                   
            % Construct groundTruth object with selected labels. Use
            % private DSource to get data source in case an invalid data
            % source is present. Direct DataStore access yields an error
            % for invalid data sources.
            dataSource  = this.DSource;
            labelDefs   = this.LabelDefinitions(indexList,:);
            
            if isa(labels,'labelType') && any(labels == labelType.PixelLabel)
                % pixel labels are all in one column of label data table.
                labelNames(labelDefs.Type == labelType.PixelLabel) = [];
                labelNames{end+1} = 'PixelLabelData';
            end
            
            labelData = this.LabelData(:, labelNames);
            
            gTruth = groundTruth(dataSource, labelDefs, labelData);
        end

        %------------------------------------------------------------------
        function gTruth = selectLabelsByName(this, labelNames)
        %selectLabelsByName Select ground truth data for a set of labels.
        %   gtLabel = selectLabelsByName(gTruth, labelNames) returns a new
        %   groundTruth object with only the labels specified by
        %   labelNames. labelNames can be string array or cell array of
        %   character vectors
        %
        %   Example 1: Select ground truth data by labelName
        %   ----------------------------------------------------------
        %   % Add the image directory to the MATLAB path
        %   imageDir = fullfile(matlabroot, 'toolbox', 'vision', 'visiondata', 'stopSignImages');
        %   addpath(imageDir);
        %
        %   % Load the groundTruth object
        %   load('stopSignsAndCarsGroundTruth.mat');
        %
        %   % View the label definitions
        %   stopSignsAndCarsGroundTruth.LabelDefinitions
        %
        %   % Obtain the ground truth data for labelName 'stopSign'
        %   stopSignGroundTruth = selectLabelsByName(stopSignsAndCarsGroundTruth, ...
        %                                     'stopSign');
        %
        %   % Obtain ground truth for 'carRear' and 'carFront'
        %   carGroundTruth = selectLabels(stopSignsAndCarsGroundTruth, ...
        %                                 {'carRear', 'carFront'});
        %
        %   % Remove the image directory from the path
            %   rmpath(imageDir);         
        
            gTruth = selectLabels(this, labelNames);
        end
        
        %------------------------------------------------------------------
        function gTruth = selectLabelsByType(this, labelTypes)
        %selectLabelsByType Select ground truth data for a set of labels.
        %   gtLabel = selectLabelsByType(gTruth, labelTypes) returns a new
        %   groundTruth object containing all labels specified by
        %   labelTypes. labelTypes is a labelType enumeration.
        %
        %   Example 1: Select ground truth data by types
        %   ----------------------------------------------------------
        %   % Add the image directory to the MATLAB path
        %   imageDir = fullfile(matlabroot, 'toolbox', 'vision', 'visiondata', 'stopSignImages');
        %   addpath(imageDir);
        %
        %   % Load the groundTruth object
        %   load('stopSignsAndCarsGroundTruth.mat');
        %
        %   % View the label definitions
        %   stopSignsAndCarsGroundTruth.LabelDefinitions
        %
        %   % Obtain the ground truth data for labelType Rectangle
        %   rectGroundTruth = selectLabelsByType(stopSignsAndCarsGroundTruth, ...
        %                                  labelType.Rectangle);
        %
        %   % Remove the image directory from the path
        %   rmpath(imageDir);         
        
            gTruth = selectLabels(this, labelTypes);
        end
        
        %------------------------------------------------------------------
        function gTruth = selectLabelsByGroup(this, groups)
        %selectLabelsByGroup Select ground truth data for a set of labels.
        %   gtLabel = selectLabelsByGroup(gTruth, groups) returns a new
        %   groundTruth object with all labels specified by groups.
        %   groups can be string array or cell array of character
        %   vectors
        
            % Handle array indexing for this method
            if numel(this)>1
                for n = 1 : numel(this)
                    gTruth(n) = selectLabelsByGroup(this(n), groups); %#ok<AGROW>
                end
                return;
            end        
        
            validateattributes(groups, ...
                {'char', 'string', 'cell'}, ...
                {'nonempty','vector'}, mfilename, 'groups');
            
            allGroupNames = this.LabelDefinitions.Group;
            labelDefs = this.LabelDefinitions;
            
            [selectedLabelDefs, selectedLabelNames] = vision.internal.groundTruthHelper.selectLabelsByGroup(groups, allGroupNames, labelDefs);
            
            % Construct groundTruth object with selected labels. Use
            % private DSource to get data source in case an invalid data
            % source is present. Direct DataStore access yields an error
            % for invalid data sources.
            dataSource  = this.DSource;
            
            labelData = this.LabelData(:, selectedLabelNames);

            gTruth = groundTruth(dataSource, selectedLabelDefs, labelData);
            
        end
        
        %------------------------------------------------------------------
        function unresolvedFilePaths = changeFilePaths(this, alternateFilePaths)
            %changeFilePaths Change DataSource and PixelLabelData file paths
            %   changeFilePaths is used to resolve groundTruth object file 
            %   paths that have moved or cannot be found.
            %
            %   unresolvedFilePaths = changeFilePaths(gTruth, alternateFilePaths)
            %   replace file paths stored in the gTruth object with those
            %   provided in alternateFilePaths. The replacement can be done
            %   for both:
            %       - DataSource
            %       - PixelLabelData column of property LabelData
            %   The paths are only replaced if the alternate paths exist.
            %   Otherwise, the old paths are retained. unresolvedFilePaths
            %   is a cell array of file paths that were not found. This
            %   includes the files from DataSource and PixelLabelData.
            %
            %   Inputs
            %   ------
            %   alternateFilePaths      Specifies a set of alternative paths used to
            %                           locate the missing files. The paths can
            %                           be specified as in the examples below:
            %
            %                           Single root path needs to be changed
            %                           ------------------------------------
            %                           oldLocation = "Z:\data";
            %                           newLocation = "/mynetwork/myFolder";
            %
            %                           alternateFilePath = [oldLocation, newLocation];
            %                           changeFilePaths(gTruth, alternateFilePath);
            %
            %                           Multiple root paths need to be changed
            %                           --------------------------------------
            %                           oldLocation1 = "Z:\data";
            %                           newLocation1 = "/mynetwork/myFolder";
            %                           oldLocation2 = "Y:\PixelLabelData";
            %                           newLocation2 = ["/mynetwork1/PixelLabelData",...
            %                                           "/mynetwork2/PixelLabelData"];
            %
            %                           % combine multiple paths into a cell array of string vectors
            %                           alternateFilePath = {[oldLocation1,newLocation1];...
            %                                                   [oldLocation2,newLocation2]};
            %                           changeFilePaths(gTruth, alternateFilePath);
            %
            %   Outputs
            %   -------
            %   unresolvedFilePaths     A string array containing the paths that are not
            %                           found. The method checks whether alternate
            %                           paths exist. If not, the old paths are retained
            %                           Out of these, only the paths that could not be
            %                           found are returned as unresolvedFilePaths.
            %
            %   See also groundTruthDataSource
            
            % Handle array indexing for this method
            if numel(this)>1
                unresolvedFilePaths = [];
                for n = 1 : numel(this)
                    unresolvedFilePaths = [unresolvedFilePaths; changeFilePaths(this(n), alternateFilePaths)]; %#ok<AGROW>
                end
                return;
            end
            
            % Validate the inputs
            alternateFilePaths  = convertStringsToChars(alternateFilePaths);
            validAltPaths       = vision.internal.labeler.validation.validateFilePaths(alternateFilePaths);
            
            % Obtain the source files for modification
            dataSourceObjExists = isa(this.DSource, 'groundTruthDataSource');
            
            if dataSourceObjExists
                sourceFiles = this.DataSource.Source;
                isCustomSource = (this.DataSource.SourceType == vision.internal.labeler.DataSourceType.CustomReader);
            else
                % If the data source cannot be found while loading the
                % groundTruth object, the DataSource property contains the 
                % file name
                sourceFiles = this.DSource.Source;
                isCustomSource = isempty(this.DSource.Source);
            end
            
            unresolvedSrcFiles = {};
            
            if ~isCustomSource
                % The flag is used for conditions where you convert the source
                % files to a cell array
                changeBackSourceFiles = false;
                if ischar(sourceFiles)
                    sourceFiles = {sourceFiles};
                    changeBackSourceFiles = true;
                end

                if isa(sourceFiles, 'matlab.io.datastore.ImageDatastore')
                    imds = sourceFiles;
                    sourceFiles = imds.Files;
                    isImageDatastore = true;
                else
                    isImageDatastore = false;
                end

                % Resolve source files
                [sourceFiles, unresolvedSrcFiles] = resolveFiles(sourceFiles, validAltPaths);
                
                allSrcExists  = (numel(unresolvedSrcFiles) == 0);
                
                if allSrcExists
                    % If all ths source files were resolved, enter here
                    if isscalar(sourceFiles) && changeBackSourceFiles
                        sourceFiles = sourceFiles{1};
                    end
                    
                    srcTypeImgSequence = (dataSourceObjExists && this.DataSource.SourceType == vision.internal.labeler.DataSourceType.ImageSequence);
                    isImageSequence = (isscalar(cellstr(sourceFiles)) && isfolder(sourceFiles))...
                        || srcTypeImgSequence;
                    
                    % Need to check if the sourceType is image sequence,
                    % since timestamps need to be passed for creating
                    % source object
                    if isImageSequence
                        % For image sequence
                        timeStamps = this.LabelData.Time;
                        % For sourcetype of image sequence, if the gTruth
                        % object is in workspace, then the SourceType exists
                        % and this means Source has full file names. We
                        % just need the path to create source object.
                        % Whereas for a object loaded from file, if the
                        % files are missing, only the folder name exists in
                        % DataSource field. We need to differentiate that
                        % here.
                        if srcTypeImgSequence
                            sourceFiles = fileparts(sourceFiles{1});
                        end
                        dataSource = groundTruthDataSource(sourceFiles, timeStamps);
                    elseif isImageDatastore
                        imds.Files = sourceFiles;
                        dataSource = groundTruthDataSource(imds);
                    else
                        dataSource = groundTruthDataSource(sourceFiles);
                    end
                else
                    dataSource = vision.internal.InvalidGroundTruthDataSource(sourceFiles);
                end
                
                % Assign directly to private property because dataSource
                % can be invalid. 
                this.DSource = dataSource;
            else
                warning(message('vision:groundTruth:CustomSourceOnlyPixel'));
            end
            
            % Resolve pixel label data
            unresolvedPxDataFiles = {};
            if any(strcmp(this.LabelData.Properties.VariableNames, 'PixelLabelData'))
                pixelLabelDataFiles = this.LabelData.PixelLabelData;
                [pixelLabelDataFiles,unresolvedPxDataFiles] = resolveFiles(pixelLabelDataFiles, validAltPaths);
                this.LabelData.PixelLabelData = pixelLabelDataFiles;
            end
            
            unresolvedFilePaths = [unresolvedSrcFiles'; unresolvedPxDataFiles'];
            unresolvedFilePaths = convertCharsToStrings(unresolvedFilePaths);

            numUnresolvedFiles = numel(unresolvedFilePaths);

            if numUnresolvedFiles > 0
                disp(vision.getMessage('vision:groundTruth:NumFilesNotResolved', numUnresolvedFiles));
            else
                disp(vision.getMessage('vision:groundTruth:AllFilePathsResolved'));
            end
        end
       
        function [labelData, ts] = gatherLabelData(this, typeOfLabel, nvp)
            % gatherLabelData Gather label data from ground truth objectGather label data from ground truth object.
            %  
            %     labelData = gatherLabelData(gTruth, typeOfLabel) returns
            %     synchronized label data for images. typeOfLabel specifies 
            %     the type of label data to gather as labelType.Rectangle, 
            %     labelType.RotatedRectangle, labelType.Line, labelType.ProjectedCuboid, 
            %     labelType.Polygon, labelType.Scene, or labelType.PixelLabel.
            %  
            %     [...,timestamps] = gatherLabelData(...) optionally returns the image
            %     timestamps associated with the gathered label data. 
            %  
            %     Inputs
            %     ------
            %     gTruth       A vector of groundTruth objects.
            %  
            %     typeOfLabel  A vector of labelType objects. When specified as a vector, the
            %                  function gathers typeOfLabel label data for the source.
            %                  The label data is grouped in columns of the table either by
            %                  label name or label type based on the 'GroupLabelData' NV Pair.
            %                    
            %                  Valid values are labelType.Scene,
            %                  labelType.Rectangle, labelType.RotatedRectangle
            %                  labelType.Line, labelType.ProjectedCuboid and labelType.Polygon.
            %                  When the type of label is labelType.Scene, no other types of labels can
            %                  be specified.
            %  
            %     Outputs
            %     -------
            %     labelData     An M-by-1 cell array of tables. M is the number of
            %                   elements in gTruth object. When typeOfLabel contains
            %                   ROI label types, labelData{m}, contains a
            %                   table of ROI label data of types typeOfLabel. The columns
            %                   of the table are either grouped as label names or as label type,
            %                   dependent on the value of 'GroupLabelData' NV Pair.
            %  
            %                   The type of label determines the format of the data in
            %                   the output labelData. See label data format for more
            %                   information.
            %  
            %     timestamps    An M-by-1 cell array of duration vectors. timestamps{m}
            %                   are the timestamps for source in gTruth{m}.
            %  
            %     [...] = gatherLabelData(...,Name,Value) specifies additional name-value
            %     pair arguments described below:
            %  
            %     'SampleFactor'   The sample factor used to subsample label data. A
            %                      sample factor of K includes every Kth frame.
            %                      Increase the sample factor to drop redundant frames
            %                      from signals with high sample rates, such as video.
            %  
            %                      Default: 1
            % 
            %     'GroupLabelData' Specify the grouping of columns of the tables returned
            %                      in label data. The two available grouping options are-
            %                      
            %                      'LabelName' - Groups the label data by label definitions.
            %                      'LabelType' - Groups the label data by label type. This
            %                                    option can used to gather label data with
            %                                    the roi stacking order retained.
            %                    
            %                      Default: 'LabelName'
            %  
            %     Notes
            %     -----
            %     - Attribute and sublabel data are not returned. Only the ROI label
            %       position is returned in labelData.
            %     - Label data is not gathered at timestamps where one of the source is
            %       missing label data.
            %
            %     Example : Gather polygon labels
            %     -------------------------------
            %     % Gather all polygon labels from the groundtruth image
            %     data = load('groundtruthVisionTeam.mat');
            %     gtruth = data.groundtruthVisionTeam;
            %
            %     % Gather all the polygon objects
            %     labelData = gatherLabelData(gtruth, labelType('Polygon'),...
            %                                         'GroupLabelData', 'LabelType');
            %
            %     polygons = labelData{1}.PolygonData{1}(:,1);
            %     polygonLabels = labelData{1}.PolygonData{1}(:,2);
            %
            %     % Visualize polygon labels
            %     im = imread('visionteam.jpg');
            %     imshow(im)
            %
            %    showShape('polygon', polygons, 'Label', polygonLabels)

            arguments
                % Required inputs.
                this        {validateattributes(this,{'groundTruth'},{'vector'})}
                typeOfLabel {iValidateLabelType(typeOfLabel)}

                % Name-value 
                nvp.SampleFactor (1,1) ...
                    {mustBeNumeric,mustBeReal,mustBeFinite,mustBeInteger,mustBePositive} = 1;
                % Name-value 
                nvp.GroupLabelData...
                    {mustBeMember(nvp.GroupLabelData, {'LabelName', 'LabelType'})} = 'LabelName';
            end
           
            % Scene labels are not supported for GroupLabelData='LabelType'
            if(any(typeOfLabel==labelType.Scene) && ...
                            strcmp(nvp.GroupLabelData, 'LabelType'))
                error(message('vision:groundTruth:invalidLabelTypeForGrouping'));
            end
            % Format user input into canonical sizes and types.
            typeOfLabel      = reshape(typeOfLabel,1,[]);   
            nvp.SampleFactor = double(nvp.SampleFactor);
            
            % Gather label data from ground truth object. 
            [labelData, ts] = iGatherROILabelData(this,typeOfLabel, nvp.GroupLabelData);

            % Subsample label data and time stamps.
            [labelData, ts] = iApplySampleFactor(labelData, ts, nvp.SampleFactor);

            % Remove attribute and sublabel data.
            labelData = iRemoveAttribSublabelData(labelData);

            end

        %------------------------------------------------------------------
        function set.DataSource(this, dataSource)
            dataSource = validateDataSource(this, dataSource);
            
            % dataSource can be empty or contain a cellstr of image file
            % names if the groundTruth object was loaded from a source that
            % could not be found. LabelData can be empty if the object is
            % being constructed on load and the LabelData isn't set up yet.
            if ~(isempty(dataSource) || iscellstr(dataSource) || ischar(dataSource))...
                    && ~isempty(this.LabelData) && hasTimeStamps(dataSource) %#ok<ISCLSTR>
                vision.internal.labeler.validation.checkTimes(this.LabelData, dataSource);
                
                % Ensure that source times match exactly with dataSource.
                % Across platforms, videos may provide slightly different
                % time stamps. In that case, modify labelData to use row
                % times from the source.
                this.LabelData.Time = dataSource.TimeStamps;
            end
            
            this.DSource = dataSource;
        end
        
        %------------------------------------------------------------------
        function dataSource = get.DataSource(this)
            dataSource = this.DSource;
            
            if ~this.hasValidDataSource()
                % Return the invalid data. 
                dataSource = this.DSource.Source;
            end
            
        end
        
        %------------------------------------------------------------------
        function type = get.SourceType(this)
            type = this.DSource.SourceType;
        end
       
    end
    
    methods (Hidden)
        %------------------------------------------------------------------
        % saveobj is implemented to ensure compatibility across releases by
        % converting the class to a struct prior to saving it. It also
        % contains a version number, which can be used to customize the
        % loading process.
        %------------------------------------------------------------------
        function that = saveobj(this)
             
            if this.hasValidDataSource()
                that.DataSource = saveobj(this.DataSource);
            else
                % Issue a warning when saving invalid data sources.
                warning(message('vision:groundTruth:badSource'))
                that.DataSource = this.DSource.serialize();
            end
            
            that.LabelDefinitions   = table2struct(this.LabelDefinitions);
            if istimetable(this.LabelData)
                that.LabelData = table2struct(timetable2table(this.LabelData));
            else
                that.LabelData = table2struct(this.LabelData);
            end
            
            if(~isempty(this.PolygonOrder))
                that.PolygonOrder = table2struct(this.PolygonOrder);
            else
                that.PolygonOrder = [];
            end
            
            that.Version   = this.Version;
        end
        
        function [labelData, polyOrderData] = roiLabelData2LabelData(this, roiLabelData)
            % roiLabelData2LabelData converts a ROILabelData style of flattened
            % roi list
            labelDefs = this.LabelDefinitions;
            hasHierarchy   = any( strcmp(labelDefs.Properties.VariableNames, 'Hierarchy') );
            roiLabelNames = this.LabelDefinitions.Name;
            
            columnNames = roiLabelData.Properties.VariableNames;
            
            % Get Pixel index
            [~, pixelIdx] = find(strcmp('PixelLabelData', columnNames));
            
            pixelColumn = roiLabelData(:, pixelIdx);
            
            if istimetable(pixelColumn)
                pixelColumn = timetable2table(pixelColumn,'ConvertRowTimes', false);
            end
            
            % Get scene Label col Idx
            sceneLabelNames = roiLabelNames(labelDefs.Type == labelType.Scene);
            
            sceneIdx = arrayfun(@(x)find(strcmp(x,columnNames)), sceneLabelNames);
            
            sceneLabelColumns = roiLabelData(:, sceneIdx);
            
            if istimetable(sceneLabelColumns)
                sceneLabelColumns = timetable2table(sceneLabelColumns,'ConvertRowTimes', false);
            end
            
            % Roi Columns required (equal to number of non-pixel, non-scene columns)
            removeIdx = (labelDefs.Type == labelType.PixelLabel)|(labelDefs.Type == labelType.Scene);
            
            
           roiLabelNames = roiLabelNames(~removeIdx);
           roiLabelTypes = labelDefs.Type(~removeIdx);
           

           varTypes = repmat({'cell'}, numel(roiLabelNames), 1);
                
           labelData = table('Size', [size(roiLabelData,1) numel(roiLabelNames)],...
                                  'VariableTypes',varTypes,...
                                  'VariableNames', roiLabelNames);
           % Cache polygon stacking order
           polyNames = labelDefs.Name((labelDefs.Type == labelType.Polygon));
           polyOrderData = table('Size', [size(roiLabelData,1) numel(polyNames)],...
                                  'VariableTypes', repmat({'cell'}, numel(polyNames), 1),...
                                  'VariableNames', polyNames);
            
            
            for r = 1:size(roiLabelData,1)
                
                if(iscell(roiLabelData.ROILabelData))
                    if(isempty(roiLabelData.ROILabelData{r}))
                        continue;
                    end
                else
                    if(isempty(roiLabelData.ROILabelData(r)))
                        continue;
                    end
                end
                
                fieldNames = fields(roiLabelData.ROILabelData(r));
                
                for f = 1:numel(fieldNames)
                    
                    currentLabelType = this.getROILabelTypeFromFieldName(fieldNames{f});
                    
                    supportedClasses = roiLabelNames(roiLabelTypes == currentLabelType);
                    
                    data = roiLabelData.ROILabelData(r).(fieldNames{f});
                    
                    for c=1:numel(supportedClasses)
                        
                        classROIIdx = strcmp(data(:,2),supportedClasses{c});
                        classROIData = data(classROIIdx, 1);
                        
                        if((currentLabelType==labelType.Rectangle)||...
                                (currentLabelType == labelType.RotatedRectangle)||...
                                (currentLabelType==labelType.ProjectedCuboid)||...
                                (currentLabelType==labelType.Point) || (hasHierarchy))
                           classROIData = vertcat(classROIData{:});
                        end
                        
                        labelData.(supportedClasses{c}){r} = classROIData;
                        
                        % Stacking order is same as implicit order of
                        % polygons
                        if(currentLabelType == labelType.Polygon)
                            polyOrderData.(supportedClasses{c}){r} = find(classROIIdx);
                        end
                                                                      
                    end
                end                
            end
            labelData = [labelData pixelColumn sceneLabelColumns];
            if(istimetable(roiLabelData))
               labelData = table2timetable(labelData,'RowTimes', roiLabelData.Time); 
            end
        end
        
        function roiLabelTableData = labelData2ROILabelData(this, labelData)
            
            
            labelDefs = this.LabelDefinitions;
            columnNames = labelData.Properties.VariableNames';
            hasHierarchy   = any( strcmp(this.LabelDefinitions.Properties.VariableNames, 'Hierarchy') );
            hasSceneLabels = any(labelDefs.Type==labelType.Scene);
            hasPixelLabels = any(labelDefs.Type==labelType.PixelLabel);
            hasPolygonLabels = any(labelDefs.Type==labelType.Polygon);
            
            outColumns = {'ROILabelData'};
            columnType = {'struct'};
            
            if(hasPixelLabels)
                outColumns = [outColumns; {'PixelLabelData'}];
                columnType = [columnType; {'cell'}];
            end
            
            if(hasSceneLabels)
                sceneLabelNames = labelDefs.Name(labelDefs.Type == labelType.Scene);
                sceneColumnType = repmat({'logical'}, numel(sceneLabelNames), 1);
                outColumns = [outColumns; sceneLabelNames];
                columnType = [columnType; sceneColumnType];
            end
            
            roiLabelTableData = table('Size', [size(labelData, 1) numel(outColumns)],...
                                  'VariableTypes', columnType,...
                                  'VariableNames', outColumns);

            % Loop over all rows of the table
            for r = 1:size(labelData,1)

                for i=1:numel(columnNames)

                    currentName = columnNames{i};
                    currentType = labelDefs.Type(strcmp(currentName, labelDefs.Name));
                    
                    if(strcmp(currentName,'PixelLabelData'))
                        roiLabelTableData.PixelLabelData(r) = labelData.PixelLabelData(r);
                        continue;
                    end
                    
                    if(currentType == labelType.Scene)
                        roiLabelTableData.(currentName)(r) = labelData.(currentName)(r);
                        continue;
                    end
                    
                    if(currentType == labelType.Custom)
                       continue; 
                    end
                        
                    % The column is a ROI Type. Accumulate the data in
                    % ROILabelData structure.
                    
                    currentFieldName = getROIFieldNameFromLabelType(this, currentType);

                    currentData = labelData.(currentName){r};

                    if(~iscell(currentData))
                        currentData = mat2cell(currentData, ones(1, size(currentData,1)), size(currentData,2) );
                    end                

                    currentLabels = repmat({currentName}, size(currentData,1), 1);

                    data = [currentData, currentLabels];
                    
                    % Add ROI order to the Polygon Data
                    if(~isempty(currentData) &&...
                            (currentType == labelType.Polygon) &&...
                            this.IsSourceROILabelData)
                        currentPolygonOrder = this.PolygonOrder.(currentName){r};
                        currentPolygonOrder = mat2cell(currentPolygonOrder, ones(1, size(currentPolygonOrder,1)), size(currentPolygonOrder,2) );
                        data = [data, currentPolygonOrder]; %#ok<AGROW>
                    end

                    if(isfield(roiLabelTableData.ROILabelData(r), currentFieldName))

                        existingData = roiLabelTableData.ROILabelData(r).(currentFieldName);

                        roiLabelTableData.ROILabelData(r).(currentFieldName) = [existingData ; data];

                    else

                        roiLabelTableData.ROILabelData(r).(currentFieldName) = data;

                    end
                end
                
                % If polygon labels exist and order is maintained sort the
                % polygons based on order.
                if(hasPolygonLabels)
                    if(~isempty(roiLabelTableData.ROILabelData(r).PolygonData) &&...
                            this.IsSourceROILabelData)
                        pData = roiLabelTableData.ROILabelData(r).PolygonData;
                        
                        % Get Polygon order 
                        order = cell2mat(pData(:,3));
                        
                        [~, idx] = sort(order);
                        pData = pData (idx, 1:2);
                        roiLabelTableData.ROILabelData(r).PolygonData = pData;
                    end
                end
            end
            
            if(istimetable(labelData))
               roiLabelTableData = table2timetable(roiLabelTableData,...
                                                   'RowTimes', labelData.Time); 
            end
            
        end
            
        %------------------------------------------------------------------
        function hierarchyStruct = hierarchyOf(this, labelName)
            %hierarchyOf Get hierarchy for a label
            %   hierarchyOf returns the Hierarchy struct from the
            %   LabelDefinitions table for a specified label.
            %
            %   hStruct = hierarchyOf(gTruth, labelName) returns hStruct,
            %   the hierarchy struct for the label specified by labelName
            %   in the groundTruth object gTruth.
            %
            %   See also groundTruth/LabelDefinitions.
            
            % Strictly allow exactly two arguments.
            narginchk(2,2);
            % Call static method
            hierarchyStruct = groundTruth.hierarchyOfLabel(...
                this.LabelDefinitions, labelName);
        end
        
        %------------------------------------------------------------------
        function templateStruct = templateFor(this, labelName)
            %templateFor Get template struct for a label
            %   templateFor returns a template struct for a specified label
            %   that can be used to populate the LabelData table.
            %
            %   templateStruct = templateFor(gTruth, labelName) returns
            %   templateStruct, a template struct for the label specified
            %   by labelName in the groundTruth object gTruth.
            %
            %   See also groundTruth/LabelData.
            
            % Strictly allow exactly two arguments.
            narginchk(2,2);
            % Call static method
            templateStruct = groundTruth.templateForLabel(...
                this.LabelDefinitions, labelName);
        end
        
        %------------------------------------------------------------------
        function changeLabelName(this, oldName, newName, newDescription)
            %changeLabelName change label name
            %   changeLabelName changes the name of a specified label.
            %   changeLabelName(gTruth, oldName, newName) changes the label
            %   oldLabel to newLabel in the groundTruth object gTruth.
            %   oldLabel cannot be a sublabel or attribute.
            %
            %   changeLabelName(gTruth, oldName, newName, newDescription)
            %   additionally specifies the new label description.
            %
            %   Notes
            %   -----
            %   - When gTruth is an array of groundTruth objects, oldName
            %     is changed to newName in any of the elements that contain
            %     the label oldName.
            %
            %
            %   Examples
            %   --------
            %   % Load a groundTruth object MAT-file
            %   data = load('stopSignsAndCarsGroundTruth.mat')
            %   gTruth = data.stopSignsAndCarsGroundTruth;
            %
            %   % Change label 'carRear' to 'vehicleRear'
            %   changeLabelName(gTruth, 'carRear', 'vehicleRear')
            %
            %   % Change label 'carFront' to 'vehicleFront'
            %   changeLabelName(gTruth, 'carFront', 'vehicleFront')
            %
            %   % Inspect new label definitions
            %   gTruth.LabelDefinitions
            %
            %   See also groundTruth/selectLabel.
            
            narginchk(3,4);
            
            % Validate old name
            validateattributes(oldName, {'char', 'string'}, ...
                {'scalartext'}, 'changeLabelName', 'oldName');
            
            % Validate new name
            validateattributes(newName, {'char', 'string'}, ...
                {'scalartext'}, 'changeLabelName', 'newName');
            
            if ~isvarname(newName)
                error(message('vision:groundTruth:invalidNewLabelName',newName));
            end
            
            
            updateDescFlag = nargin>3;
            if updateDescFlag
                % Validate new description
                validateattributes(newDescription, {'char', 'string'}, ...
                    {'scalartext'}, 'changeLabelName', 'newDescription');
            end
            
            % Old name should be one of the existing labels
            % For an array of gTruth objects, old name should exist in at
            % least one of the elements.
            allLabelNames = this(1).LabelDefinitions.Name;
            for n = 2 : numel(this)
                allLabelNames = [allLabelNames; this(n).LabelDefinitions.Name]; %#ok<AGROW>
            end
            validatestring(oldName, unique(allLabelNames), 'changeLabelName', ...
                'oldName');
            
            for n = 1 : numel(this)
                thisElementLabelNames = this(n).LabelDefinitions.Name;
                
                % New name cannot be one of the existing labels in this
                % element
                if any(strcmp(thisElementLabelNames,oldName)) && any(strcmp(thisElementLabelNames, newName))
                    error(message('vision:groundTruth:labelNameExists',newName))
                end
                
                % Update label definitions table
                this(n).LabelDefinitions.Name = strrep(...
                    thisElementLabelNames, oldName, newName);
                
                % If a Description was provided, update the Description
                if updateDescFlag
                    index = find(strcmp(this(n).LabelDefinitions.Name, newName));
                    index = index(1);
                    
                    hasDescription = any( strcmp(this(n).LabelDefinitions.Properties.VariableNames, 'Description') );
                    hasHierarchy   = any( strcmp(this(n).LabelDefinitions.Properties.VariableNames, 'Hierarchy') );
                    
                    % If the LabelDefinitions table does not have a
                    % Description column, add one.
                    if ~hasDescription
                        this(n).LabelDefinitions.Description = repmat({''}, ...
                            height(this(n).LabelDefinitions),1);
                    end
                    
                    % Update the Description
                    this(n).LabelDefinitions.Description{index} = char(newDescription);
                    
                    % If the LabelDefinitions table has a Hierarchy, this
                    % needs to be updated as well.
                    if hasHierarchy
                        if isfield(this(n).LabelDefinitions.Hierarchy{index}, 'Description')
                            this(n).LabelDefinitions.Hierarchy{index}.Description = char(newDescription);
                        end
                    end
                end
                
                % Update variable name in LabelData table
                this(n).LabelData.Properties.VariableNames = ...
                    strrep(this(n).LabelData.Properties.VariableNames, oldName, newName);
            end
            
        end
        
        %------------------------------------------------------------------
        function tf = hasValidDataSource(this)
            tf = ~isa(this.DSource,'vision.internal.InvalidGroundTruthDataSource');
        end
        
        %------------------------------------------------------------------
        function labeltype = getROILabelTypeFromFieldName (~, fieldName)
    
            switch fieldName
                case 'RectangleData'
                    labeltype = labelType.Rectangle;
                
                case 'RotatedRectangleData'
                    labeltype = labelType.RotatedRectangle;

                case 'LineData'
                    labeltype = labelType.Line;

                case 'PolygonData'
                    labeltype = labelType.Polygon;

                case 'ProjCuboidData'
                    labeltype = labelType.ProjectedCuboid;
                case 'PointData'
                    labeltype = labelType.Point;
                otherwise
                    %TODO
                    error('Error');
            end
        end
        
        %------------------------------------------------------------------
        function fieldName = getROIFieldNameFromLabelType (~, labeltype)
    
            switch labeltype
                case labelType.Rectangle
                    fieldName = 'RectangleData';
                
                case labelType.RotatedRectangle
                    fieldName = 'RotatedRectangleData';

                case labelType.Line
                    fieldName = 'LineData';

                case labelType.Polygon
                    fieldName = 'PolygonData';

                case labelType.ProjectedCuboid
                    fieldName = 'ProjCuboidData';
                    
                case labelType.PixelLabel
                    fieldName = 'PixelLabelData';
                 case labelType.Point
                    fieldName = 'PointData'; 
                    
                otherwise
                    %TODO
                    error('Error');
            end
        end
    end
    
    methods (Hidden, Static)
        %------------------------------------------------------------------
        function this = loadobj(that)
            
            % If the object was saved with a bad data source, don't rely on
            % DataSource and SourceType.
            badDataSource = ~isstruct(that.DataSource);
            
            if ~badDataSource
                that = updatePreviousVersion(that);
                
                % Use the new enum type as the SourceType.
                switch that.DataSource.SourceType
                    case "VideoReader"
                        that.SourceType = vision.internal.labeler.DataSourceType.VideoReader;
                    case "ImageSequence"
                        that.SourceType = vision.internal.labeler.DataSourceType.ImageSequence;
                    case "CustomReader"
                        that.SourceType = vision.internal.labeler.DataSourceType.CustomReader;
                    case "ImageDatastore"
                        that.SourceType = vision.internal.labeler.DataSourceType.ImageDatastore;
                    otherwise
                        error('Unknown source');
                end
            end
            
            % First load label definitions and label data.
            labelDefs = struct2table(that.LabelDefinitions, 'AsArray', true);
            
            % The labeler apps export hierarchy as cell arrays. However
            % when loading the ground truth object, struct2table function
            % above, converts the hierarchy to array of structs. Converting
            % this array to a cell array
            hasHierarchy   = any(strcmp(labelDefs.Properties.VariableNames, 'Hierarchy'));

            if hasHierarchy && isstruct(labelDefs.Hierarchy)
                labelDefs.Hierarchy = arrayfun(@(x){x}, labelDefs.Hierarchy);
            end

            if isfield(that.LabelDefinitions, 'PixelLabelID')
                % struct2table puts scalars as numeric scalars into table,
                % whereas we expect PixelLabelID to be cell arrays. Convert
                % to cell if required.
                if ~iscell(labelDefs.PixelLabelID)
                    % PixelLabelIDs may be M-by-1 or M-by-3. Split along
                    % the 2nd dimension to ensure both kinds of
                    % PixelLabelIDs are taken care of correctly.
                    labelDefs.PixelLabelID = num2cell(labelDefs.PixelLabelID, 2);
                end
            end
            
            labelData = struct2table(that.LabelData, 'AsArray', true);
            
            if(isfield(that, 'PolygonOrder'))
                if(~isempty(that.PolygonOrder))
                    polygonOrder = struct2table(that.PolygonOrder, 'AsArray', true);
                else
                    polygonOrder = [];
                end
            else
                polygonOrder = [];
            end
            if (~badDataSource && that.SourceType ~= vision.internal.labeler.DataSourceType.ImageDatastore) ...
                    || (badDataSource && any(strcmp(labelData.Properties.VariableNames, 'Time')))
                labelData  = table2timetable(labelData);
            end
            
            if any(labelDefs.Type == labelType.PixelLabel)
                % Try to locate pixel label data files.
                adjustedPixelLabelData = vision.internal.labeler.tryToAdjustFilePaths(...
                    labelData.PixelLabelData, false);
                notFound = cellfun(@(x)isempty(x),adjustedPixelLabelData);
                
                % Copy over updated filepaths, leaving the original paths
                % in place if we were unable to locate them.
                labelData.PixelLabelData(~notFound) = adjustedPixelLabelData(~notFound);

                % Warn if any pixel label data file is not found.
                nonEmptyIdx =~cellfun(@isempty,labelData.PixelLabelData);
                pixelLabelFiles = labelData.PixelLabelData(nonEmptyIdx);
                if ~all(isfile(pixelLabelFiles))
                    warning(message('vision:groundTruth:PixelLabelDataNotFoundOnLoad'))
                end
            end
            
            % Ensure that all Line labels are read in as cell arrays.
            if any(labelDefs.Type == labelType.Line)
                lineNames = labelDefs.Name(labelDefs.Type == labelType.Line);
                allNames = labelData.Properties.VariableNames;
                for lIdx = 1:numel(lineNames)
                    matchIdx = find(strcmp(allNames, lineNames{lIdx}));
                    for idx = 1:height(labelData)
                        if ~isstruct(labelData{idx, matchIdx}) && ...
                           ~iscell(labelData{idx, matchIdx}{1}) && ...
                           ~isempty(labelData{idx, matchIdx}{1}) && ...
                           ~isstruct(labelData{idx, matchIdx}{1})
                            % As sublabel PolyLines are stored in structs
                            labelData{idx, matchIdx}{1} = {labelData{idx, matchIdx}{1}}; %#ok<CCAT1>
                        end
                    end
                end
            end
            
            if ~badDataSource
                try
                    % Try to find the source on the file system.
                    
                    if isequal(that.SourceType, vision.internal.labeler.DataSourceType.ImageSequence)
                        
                        % We don't know the exact location of the MAT-file.
                        % Assume pwd and try searching the file system for the
                        % path.
                        that.DataSource.Source = vision.internal.labeler.tryToAdjustFilePaths(...
                            that.DataSource.Source, true);
                        
                    elseif isequal(that.SourceType, vision.internal.labeler.DataSourceType.VideoReader)
                        % We don't know the exact location of the MAT-file.
                        % Assume pwd and try searching the file system for the
                        % path.
                        pathName = pwd;
                        [~,fileName,ext] = fileparts(that.DataSource.Source);
                        fileName = strcat(fileName,ext);
                        absoluteFileName = fullfile(pathName, fileName);
                        that.DataSource.Source = vision.internal.uitools.tryToAdjustPath(that.DataSource.Source, pathName, absoluteFileName);
                        
                    elseif isequal(that.SourceType, vision.internal.labeler.DataSourceType.ImageDatastore)
                        isDatastoreSource = isa(that.DataSource.Source, 'matlab.io.datastore.ImageDatastore');
                        if isDatastoreSource
                            source = that.DataSource.Source.Files;
                        else
                            source = that.DataSource.Source;
                        end
                        adjustedSource = vision.internal.labeler.tryToAdjustFilePaths(...
                            source, false);
                        notFound = cellfun(@(x)isempty(x),adjustedSource);
                        
                        % Copy over updated filepaths, leaving the original paths
                        % in place if we were unable to locate them.
                        if isDatastoreSource
                            that.DataSource.Source.Files(~notFound) = adjustedSource(~notFound);
                        else
                            that.DataSource.Source(~notFound) = adjustedSource(~notFound);
                        end
                        
                    else
                        % Custom Reader
                        % Don't do anything here. The custom data source name
                        % should not be modified. If a problem occurs, the
                        % groundTruthDataSource call below will issue an error.
                    end
                catch
                    % Don't do anything here. If tryToAdjustPath issues an
                    % error, break out. checkIfSourceExists will issue the
                    % necessary warning.
                end
            end
            
            % Check if sources exist, warn if they don't.
            isOnPath = groundTruth.checkIfSourceExists(that);
          
            if isOnPath
                
                try
                    dataSource = groundTruthDataSource.loadobj(that.DataSource);
                catch ME
                    % Create a "broken" object that needs to be modified by
                    % the user.
                    this = groundTruth(vision.internal.InvalidGroundTruthDataSource([]), labelDefs, labelData);
                    warning(ME.identifier,'%s',ME.message);
                    return;
                end
                
                this = groundTruth(dataSource, labelDefs, labelData);
                
            else
                               
                % Create a "broken" object that needs to be modified by the
                % user. The DataSource property instead of being a
                % groundTruthDataSource object is one of the following:
                %   - Image Collection  : cellstr of image names
                %   - Video             : char array with video name
                %   - Image Sequence    : char array with folder name
                %   - Custom Source     : char array with source name
                if badDataSource
                    invalidDataSource = vision.internal.InvalidGroundTruthDataSource(that.DataSource);
                elseif isequal(that.SourceType, vision.internal.labeler.DataSourceType.ImageSequence)
                    pathname = obtainDirName(that.DataSource.Source{1});
                    invalidDataSource = vision.internal.InvalidGroundTruthDataSource(pathname);
                else
                    invalidDataSource = vision.internal.InvalidGroundTruthDataSource(that.DataSource.Source);
                end
                this = groundTruth(invalidDataSource, labelDefs, labelData);
            end
            %Set the polygon order using the hidden method
            this.setPolygonOrder(polygonOrder);
        end
        
        %------------------------------------------------------------------
        function hierarchyStruct = hierarchyOfLabel(labelDefs, labelName)
            % Given a labelName and a labelDefinitions table, this helper
            % method returns the hierarchy struct for that labelName.
            assert(istable(labelDefs));
            assert(ischar(labelName) || isstring(labelName));
            % Default return value is empty.
            hierarchyStruct = [];
            % If there is no hierarchy field, return empty.
            hasHierarchy = any(contains(labelDefs.Properties.VariableNames, 'Hierarchy'));
            if hasHierarchy
            % Loop through top level labels, and return hierarchy when name
            % matches.
                numTopLevelLabels = height(labelDefs);
                for idx = 1:numTopLevelLabels
                    if strcmpi(labelDefs.Name{idx}, labelName)
                        hierarchyStruct = labelDefs.Hierarchy{idx};
                        break
                    end
                end
            end            
        end
        
        %------------------------------------------------------------------
        function templateStruct = templateForLabel(labelDefs, labelName)
            % given a labelDefinitions table and a labelName (presumably a
            % valid label in the Definitions table), this method returns a
            % template for filling out LabelData for that hierarchy.
            assert(istable(labelDefs));
            assert(ischar(labelName) || isstring(labelName));
            % Default return value is empty.
            templateStruct = [];
            hierarchyStruct = groundTruth.hierarchyOfLabel(labelDefs, labelName);
            if ~isempty(hierarchyStruct)
                switch hierarchyStruct.Type
                    case labelType.Rectangle
                        hierarchyStruct.Position = [0 0 0 0];
                    case labelType.RotatedRectangle
                        hierarchyStruct.Position = [0 0 0 0 0];
                    case labelType.Line
                        hierarchyStruct.Position = [0 0; 10 10; 20 20];
                    case labelType.ProjectedCuboid
                        hierarchyStruct.Position = [0 0 0 0 0 0 0 0];
                    case labelType.Point
                        hierarchyStruct.Position = [0 0]; 
                end
                templateStruct = rmfield(hierarchyStruct, 'Type');
                templateStruct = rmfield(templateStruct, 'Description');
                % Process any sub-labels
                possibleFields = fieldnames(templateStruct);
                for idx = 1:numel(possibleFields)
                    thisField = templateStruct.(possibleFields{idx});
                    if isstruct(thisField)
                       % Either attribute or sublabel
                       issublabel = isfield(thisField, 'Type');
                       if issublabel
                            switch thisField.Type
                                case labelType.Rectangle
                                    thisField.Position = [0 0 0 0];
                                case labelType.RotatedRectangle
                                    thisField.Position = [0 0 0 0 0];
                                case labelType.Line
                                    thisField.Position = [0 0; 10 10; 20 20];
                                case labelType.ProjectedCuboid
                                    thisField.Position = [0 0 0 0 0 0 0 0];
                                case labelType.Point
                                    thisField.Position = [0 0]; 
                            end
                            thisField = rmfield(thisField, 'Type');
                            thisField = rmfield(thisField, 'Description');
                            templateStruct.(possibleFields{idx}) = thisField;
                       else
                          % Process any attributes
                          thisField = rmfield(thisField, 'Description');
                          isList = isfield(thisField, 'ListItems');
                          if isList
                              thisField = thisField.ListItems{1};
                          else
                              thisField = thisField.DefaultValue;
                          end
                          templateStruct.(possibleFields{idx}) = thisField;
                       end
                    end
                end
            end
        end

        %--------------------------------------------------------------------------
        function validateIsStringOrCellstr(x,name)
            if ~(isstring(x) || iscellstr(x))
                error(message('vision:groundTruth:ValidateInputNotStringOrCellstr',name));
            end
        end

        %--------------------------------------------------------------------------
        function validateStringValuesAreNotEmpty(x,name)
            if any(strlength(deblank(string(x))) == 0,'all')
                error(message('vision:groundTruth:ValidateStringsMustBeNonEmpty',name));
            end
        end
    end
    
    methods (Access = private, Static)
        %------------------------------------------------------------------
        function isOnPath = checkIfSourceExists(that)
            
            if isempty(that.DataSource) || ischar(that.DataSource) || iscellstr(that.DataSource)
                warning(message('vision:groundTruth:badSource'))
                isOnPath = false;
                return;
            end
            
            if isequal(that.SourceType, vision.internal.labeler.DataSourceType.ImageSequence)
                % Check that each file in the list of image names exists.
                
                sourceName = that.DataSource.Source;
                isOnPath = checkIfImageSequenceExists(sourceName);
                
            elseif isequal(that.SourceType, vision.internal.labeler.DataSourceType.VideoReader)
                
                sourceName = that.DataSource.Source;
                isOnPath = checkIfVideoExists(sourceName);
                
            elseif isequal(that.SourceType, vision.internal.labeler.DataSourceType.ImageDatastore)
                % Check that each file in the list of image names exists.
                if isa(that.DataSource.Source, 'matlab.io.datastore.ImageDatastore')
                    source = that.DataSource.Source.Files;
                else
                    source = that.DataSource.Source;
                end
                isOnPath = checkImageFilenames(source);
            else
                % Custom Reader 
                % Assume that the custom data source is on path. If not, we
                % will error out later.
                isOnPath = true;
            end
        end
    end
    
    methods (Access = private)
        
        %------------------------------------------------------------------
        function dataSource = validateDataSource(~, dataSource)
            if isa(dataSource,'vision.internal.InvalidGroundTruthDataSource')
                % allow InvalidGroundTruthDataSource type. It is used to
                % deal with cases when data sources cannot be found during
                % object loading.
            else
                validateattributes(dataSource, {'groundTruthDataSource'},...
                    {'scalar'}, 'groundTruth');
            end
        end
        
        %------------------------------------------------------------------
        function labels = validateLabelNameOrType(this, labels)
            
            validateattributes(labels, ...
                {'char', 'string', 'cell', 'labelType'}, ...
                {'nonempty','vector'}, mfilename, 'label names or types');
            
            allLabelNames = this.LabelDefinitions.Name;
            
            if isstring(labels) || ischar(labels)
                labels = cellstr(labels);      
            end
            
            if iscellstr(labels)
                pixelLabels = allLabelNames( this.LabelDefinitions.Type == labelType.PixelLabel );
                
                if ~isempty(intersect(labels,pixelLabels))
                    error(message('vision:groundTruth:pixelLabelSelectByNameNotSupported'))
                end
            end
            
            if ~(iscellstr(labels) || isa(labels, 'labelType'))
                error(message('vision:groundTruth:invalidLabelSpecification'))
            end
             
            if isa(labels, 'labelType')                                
                
                labType = labels;
                tempLabels = {};
                for idx = 1:numel(labType)
                    tempLabels = [tempLabels; allLabelNames( this.LabelDefinitions.Type == labType(idx) )]; %#ok<AGROW>
                end
                labels = tempLabels;
                if isempty(labels)
                    error(message('vision:groundTruth:typeNotPresent', char(labType)))
                end
            end
            
            % Expect the list of labels provided to be unique
            labels = unique(labels,'stable');
        end
        
        %------------------------------------------------------------------
        function indexList = labelName2Index(this, labelNames)
            
            %Note this does not work for arrays of groundTruth objects
            
            allLabelNames = this.LabelDefinitions.Name;
            
            % Find column indices into LabelData.
            indexList = zeros(numel(labelNames),1);
            for n = 1 : numel(labelNames)
                idx = find( strcmp(allLabelNames, labelNames{n}) );
                if ~isscalar(idx)
                    error(message('vision:groundTruth:labelNotFound',labelNames{n}))
                end
                indexList(n) = idx;
            end
        end

        %------------------------------------------------------------------
        function filenames = prepareFilePathsForTimeRanges(this, signalNames, folderNames, ext)
            numSequences = numel(this);

            % Create unique integer values for each of the time ranges aka folder names
            uids = cellfun(@(x)size(x,1), folderNames);
            uids = [0;cumsum(uids)];

            % pad zeros to the beginning of the suffix number.
            maxWidth  = strlength(string(uids(end)));

            filenames = cell(numSequences,1);
            for i = 1:numSequences
                folders = folderNames{i};
                signalName = signalNames{i};
                uid = string(uids(i)+1:uids(i+1));
                uid = pad(uid,maxWidth,"left","0");
                uid = uid(:);
                filenames{i} = folders + filesep + signalName + "_" + uid + ext;
            end
        end

        %------------------------------------------------------------------
        function frame = readFrame(this, index)
            reader = this.DataSource.Reader;
            frame = readFrameAtPosition(reader, index);
        end
    end

    %----------------------------------------------------------------------
    % CustomDisplay methods.
    methods(Access = protected)
        
        function propgrp = getPropertyGroups(this)
            if ~isscalar(this)
                propgrp = getPropertyGroups@matlab.mixin.CustomDisplay(this);
            else
                if this.hasValidDataSource()
                    propgrp = getPropertyGroups@matlab.mixin.CustomDisplay(this);
                else
                    propList = struct('DataSource',[],...
                        'LabelDefinitions',this.LabelDefinitions,...
                        'LabelData',this.LabelData);
                    propList.DataSource = this.DSource.Source;
                    propgrp = matlab.mixin.util.PropertyGroup(propList);
                end
            end
        end
        
        %------------------------------------------------------------------
        function s = getFooter(this)
            if ~isscalar(this)
                s = getFooter@matlab.mixin.CustomDisplay(this);
            else
                % Display invalid datasource message if required.
                if this.hasValidDataSource()
                    s = '';
                else
                    msg = message('vision:groundTruth:invalidDataSourceFooter');
                    s = msg.getString();
                end
            end     
        end
    end
    
    methods (Hidden)
        function [roiData, ts] = getROIDataTableByLabelType(this, typeOfLabels, grouping)
            
            
            labelDefs = this.LabelDefinitions;
            
            conditionToRemove = ~ismember(labelDefs.Type, typeOfLabels) |...
                labelDefs.Type == labelType.PixelLabel;
            
            allLabelNames = string(labelDefs.Name);
            allLabelTypes = labelDefs.Type;
            
            selectedLabelNames = allLabelNames(~conditionToRemove);
            selectedLabelTypes = allLabelTypes(~conditionToRemove);
            
            if any(typeOfLabels == labelType.PixelLabel)
                selectedLabelNames(end+1) = "PixelLabelData";
                selectedLabelTypes(end+1) = labelType.PixelLabel;
            end
            
            switch grouping
                
                case 'LabelName'
                    labelData = this.LabelData;

                    dataLabelNames = string(labelData.Properties.VariableNames)';

                    indices = arrayfun(@(x)(any(x==selectedLabelNames)), dataLabelNames);

                    roiData = labelData(:, indices);
                    
                case 'LabelType'
                    
                    roiLabelData = this.labelData2ROILabelData(this.LabelData);
                    
                    labelTypeFieldNames = arrayfun(...
                        @(x){this.getROIFieldNameFromLabelType(x)},...
                        unique(selectedLabelTypes));
                    
                    roiData = table('Size', [size(roiLabelData,1) numel(labelTypeFieldNames)],...
                                  'VariableTypes',repmat({'cell'}, [1 numel(labelTypeFieldNames)]),...
                                  'VariableNames', labelTypeFieldNames);
                    
                    
                    for r = 1:size(roiLabelData,1) 
                      for i = 1:numel(labelTypeFieldNames)
                          currentFieldName = labelTypeFieldNames{i};
                          
                          if(strcmp( currentFieldName, 'PixelLabelData'))
                              currROITypeData = roiLabelData.(currentFieldName){r};
                          else
                              currROITypeData = roiLabelData.ROILabelData(r).(currentFieldName);
                          end
                        
                          roiData.(currentFieldName){r} = currROITypeData; 
                      end
                    end
                    
                    if(istimetable(roiLabelData))
                       roiData = table2timetable(roiData,'RowTimes', roiLabelData.Time); 
                    end
                otherwise
                    assert(false, 'Incorrect grouping option.');
            end
            
            if(istimetable(roiData))
               ts = roiData.Time;
            else
               ts = []; 
            end
            
            
        end  
        
        function polyOrder = getPolygonOrder(this)
           polyOrder = this.PolygonOrder; 
        end
        
        function setPolygonOrder(this, polyOrder)
           this.PolygonOrder = polyOrder; 
        end
    end
end

%--------------------------------------------------------------------------
function [fileList, unresolvedFiles] = resolveFiles(fileList, validAltPaths)

unresolvedFiles = {};

numFiles = numel(fileList);

% Iterate through each file in the list
for idx = 1:numFiles
    file = createOSDependPath(fileList{idx});
    
    if ~isempty(file)
        % Find the best match in the validAltPaths. Best match is the path
        % with the maximum match
        bestMatchValue = 0;
        bestMatchIdx = 0;
        
        for vAltPathNum = 1:numel(validAltPaths)
            oldPath = createOSDependPath(validAltPaths{vAltPathNum}{1});
            if startsWith(file, oldPath)
                if numel(oldPath) > bestMatchValue
                    bestMatchValue = numel(oldPath);
                    bestMatchIdx   = vAltPathNum;
                end
            end
        end
        
        if bestMatchIdx > 0
            % Best match found to replace
            oldPath = validAltPaths{bestMatchIdx}{1};
            
            for altPathNum = 2:numel(validAltPaths{bestMatchIdx})
                % Create a new temp path with the alternate paths of best
                % match path. 
                altPath = validAltPaths{bestMatchIdx}{altPathNum};
                origFile = fileList{idx};
                numElementsToRep = numel(oldPath);
                origFile(1:numElementsToRep) = [];
                tempPath = strcat(altPath, origFile);
                newTempPath = regexprep(tempPath, '[\\/]',filesep);
                
                % See if the temp path exists. Else see if the current path
                % in the source object exists. Else, add it to unresolved
                % paths
                if exist(newTempPath,'file')
                    fileList{idx} = newTempPath;
                else
                    if ~exist(fileList{idx}, 'file')
                        unresolvedFiles{end+1} = fileList{idx}; %#ok<AGROW>
                    end
                end
            end
        else
            if ~exist(fileList{idx}, 'file')
                unresolvedFiles{end+1} = fileList{idx}; %#ok<AGROW>
            end
        end
    end
end
end

function newPath = createOSDependPath(path)
% On Windows platform, files and folder names are case insensitive. This
% method creates a lower case path for Windows platform for processing.
% This way we can consider same paths provided in different casing such as
% 'E:\folder\filename.png' and 'e:\Folder\filename.png'
if ispc
    newPath = lower(path);
else
    newPath = path;
end
end

%--------------------------------------------------------------------------
function that = updatePreviousVersion(that)
% 17a version in ADST did not have SourceType property. Add it using new
% class type.

if strcmp(that.Version.Version, '1.0')
    if isa(that.DataSource.SourceType, 'driving.internal.videoLabeler.DataSourceType') || ...
       isa(that.DataSource.SourceType, 'vision.internal.videoLabeler.DataSourceType')
        % ADST is installed
        type = string(that.DataSource.SourceType);
    else
        % ADST not installed. In this case ValueNames contains the enum type.
        type = string(that.DataSource.SourceType.ValueNames{1});
    end
    
    that.DataSource.SourceType = type;
end
end

%--------------------------------------------------------------------------
function isOnPath = checkIfImageSequenceExists(sourceName)

doSkipChecks = strcmp(vision.internal.labeler.labelerFeature('skipChecks'),'on');
if doSkipChecks  
    fprintf('vision.internal.labeler.labelerFeature(''skipChecks'') feature is on. For faster loading, bypassing the check for existence of images in the input folder.');
    isOnPath = true;
else
    fileOnPathIndices = cellfun(@(fName)exist(fName,'file')==2,sourceName);
    isOnPath = all(fileOnPathIndices);

    % If not provide a warning...
    if ~isOnPath
        dirName = obtainDirName(sourceName{1});
        isDirOnPath = exist(dirName, 'dir');

        if ~isDirOnPath
            % that either the directory cannot be found,
            warning(message('vision:groundTruth:badImageDirSource', dirName))
        else
            % or that one or more images cannot be found.
            warning(message('vision:groundTruth:badImageNameSource', dirName))
        end
    end
end
end

%--------------------------------------------------------------------------
function dirName = obtainDirName(sourceName)
    dirName = fileparts(sourceName);
    if isempty(dirName)
        tempDirName = fileparts(regexprep(sourceName, '[\\/]',filesep));
        dirName = regexprep(tempDirName, '[\\/]','\');
    end
end

%--------------------------------------------------------------------------
function isOnPath = checkIfVideoExists(sourceName)

isOnPath = exist(sourceName,'file')==2;
if ~isOnPath
    warning(message('vision:groundTruth:badVideoSource', sourceName))
end
end

%--------------------------------------------------------------------------
function isOnPath = checkImageFilenames(sourceName)
% check if files exist.
fileOnPathIndices = cellfun(@(fName)exist(fName,'file')==2,sourceName);
isOnPath = all(fileOnPathIndices);

% If not provide a warning...
if ~isOnPath
    warning(message('vision:groundTruth:badImageFiles'));
end
end

%--------------------------------------------------------------------------
function tf = isCharPath(pth)
    tf = ischar(pth) && isrow(pth) && numel(pth) > 0;
end

%--------------------------------------------------------------------------
function tf = isStringPathVector(pth)
    tf = isstring(pth) && isvector(pth) && numel(pth) > 1 && all(strlength(pth) > 0);
end

%--------------------------------------------------------------------------
function tf = isCellCharPath(pth)
    tf = iscell(pth) && all(cellfun(@isCharPath, pth)) && numel(pth) > 0;
end

%--------------------------------------------------------------------------
function iValidateLabelType(typeOfLabel)
validateattributes(typeOfLabel,{'cell','labelType'},{'nonempty','vector'},mfilename,'typeOfLabel');

if iscell(typeOfLabel)
    cellfun(@nValidateLabelTypesInCell, typeOfLabel)
else
    nValidateLabelTypes(typeOfLabel);
end

     function nValidateLabelTypes(ltype)
        isScene = any(ltype == labelType.Scene,'all');
   
        % Scene labels cannot be specified with any other label type.
        if ~isscalar(ltype) && isScene
            error(message('driving:groundTruthMultiSignal:GatherLabelDataOnlyScene'))
        end

        if ~isScene     
            % Check ROI label types.
            validateattributes(ltype,{'labelType'},{'vector'},...
                mfilename,'typeOfLabel');
            iAssertSupportedLabelTypes(ltype);
        end
     end
end

%--------------------------------------------------------------------------
function iAssertSupportedLabelTypes(typeOfLabel)
% labelType.Custom is not supported.
if any(typeOfLabel == labelType.Custom)
    error(message('driving:groundTruthMultiSignal:GatherLabelDataCustomLabelNotSupported'))
end
end

%--------------------------------------------------------------------------
function [labelData, ts] = iGatherROILabelData(gt, typeOfLabels, grouping)
numSequences = numel(gt);
labelData    = cell(numSequences,1);
ts    = cell(numSequences,1);

for i = 1:numSequences 
    [labelData{i}, ts{i}] = getROIDataTableByLabelType(gt(i), typeOfLabels, grouping);
end

missingLabelData = cellfun(@(x)isempty(x), labelData);
if any(missingLabelData,'all')
    [seq,sig] = find(missingLabelData,1,'first');
    error(message('driving:groundTruthMultiSignal:GatherLabelDataNoLabelData',...
        signalName(sig),seq,string(typeOfLabels(sig))));
end

end

%--------------------------------------------------------------------------
function [tblcell,ts] = iApplySampleFactor(tblcell,ts,factor)
tblcell = cellfun(@(x) x(1:factor:end,:), tblcell, 'Uni', 0);
ts = cellfun(@(x) x(1:factor:end,:), ts, 'Uni', 0);
end
%--------------------------------------------------------------------------
function tblcell = iRemoveAttribSublabelData(tblcell)
numSequences = size(tblcell,1);

for i = 1:numSequences
        
    % For each table variable, extract the position from ground truth
    % data packed into a struct. A struct is used when sublabels or
    % attributes are defined for a particular label.
    invars = tblcell{i}.Properties.VariableNames;
    tblcell{i} = rowfun(@nExtractPositionFromStruct,tblcell{i},...
        'ExtractCellContents',true,'OutputFormat','table',...
        'OutputVariableNames',invars);
end
    %----------------------------------------------------------------------
    % Extract Position value from struct. Exclude other sublabel or
    % attribute information.
    function varargout = nExtractPositionFromStruct(varargin)
        varargout = cell(size(varargin));
        for k = 1:numel(varargin)
            var = varargin{k};
            if isstruct(var)
                varargout{k} = { vertcat(var.Position) };
            else
                varargout(k) = {varargin(k)};
            end
        end
    end
end
