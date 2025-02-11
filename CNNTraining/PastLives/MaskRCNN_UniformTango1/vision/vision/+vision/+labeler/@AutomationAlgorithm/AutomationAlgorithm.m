%AutomationAlgorithm Interface for algorithm automation in labeling.
%   AutomationAlgorithm specifies the interface for defining custom
%   automation algorithms to run in the Image Labeler, Video Labeler,
%   Ground Truth Labeler and Lidar Labeler apps. Use of the Ground Truth 
%   Labeler app requires Automated Driving Toolbox(TM).
%
%   To define a custom automation algorithm, you must construct a class
%   that inherits from the vision.labeler.AutomationAlgorithm class. The
%   AutomationAlgorithm class is an abstract class that defines the
%   signature for methods and properties that the labeling apps use for
%   loading and executing algorithms in the automation mode to generate
%   ground truth labels.
%
%   To define and use a custom automation algorithm with a labeling app,
%   follow these steps:
%
%   1. Create a +vision/+labeler folder within a folder that is already
%      on the MATLAB path. For example, if the folder /local/MyProject is
%      on the MATLAB path, then create a +vision/+labeler folder
%      hierarchy as follows:
%
%           projectFolder = fullfile('local','MyProject');
%           automationFolder = fullfile('+vision','+labeler');
%
%           mkdir(projectFolder, automationFolder)
%
%   2. Define a class that inherits from
%      vision.labeler.AutomationAlgorithm and implements the automation
%      algorithm. For temporal automation algorithms (algorithms that rely
%      on the concept of linear time, like a tracking algorithm),
%      additionally inherit from vision.labeler.mixin.Temporal. Note that
%      temporal automation algorithms can only be defined for use with the
%      Video Labeler, Ground Truth Labeler and Lidar Labeler apps.
%       
%      To define a nontemporal automation algorithm, <a href=
%      "matlab:vision.labeler.AutomationAlgorithm.openTemplateInEditor">open this template class</a>
%      and follow the outlined steps.
%
%      To define a temporal automation algorithm, <a
%      href="matlab:vision.labeler.AutomationAlgorithm.openTemplateInEditor('temporal')">open this template class</a>
%      and follow the outlined steps.
%
%   3. Save the file to the +vision/+labeler folder. Saving the file to
%      the package directory is required to use your custom algorithm from
%      within the app. You can add a folder to the path using the ADDPATH
%      function.
%
%   4. Refresh the algorithm list from within the app to start using your
%      custom algorithm.
%
%   
%   Application Programming Interface Specification
%   -----------------------------------------------
%   The AutomationAlgorithm class defines the following predefined
%   properties:
%
%   <a href="matlab:help('vision.labeler.AutomationAlgorithm.SelectedLabelDefinitions')">SelectedLabelDefinitions</a>    - Selected ROI and Scene label definitions
%   <a href="matlab:help('vision.labeler.AutomationAlgorithm.ValidLabelDefinitions')">ValidLabelDefinitions</a>       - All valid label definitions
%   <a href="matlab:help('vision.labeler.AutomationAlgorithm.GroundTruth')">GroundTruth</a>                 - Ground truth labels marked before automation
%   <a href="matlab:help('vision.labeler.AutomationAlgorithm.CurrentIndex')">CurrentIndex</a>               - Index to the current frame being processed 
%   
%   Clients of AutomationAlgorithm are required to define the following
%   user-defined properties:
%
%   <a href="matlab:help('vision.labeler.AutomationAlgorithm.Name')">Name</a>            - Algorithm name
%   <a href="matlab:help('vision.labeler.AutomationAlgorithm.Description')">Description</a>     - Algorithm description
%   <a href="matlab:help('vision.labeler.AutomationAlgorithm.UserDirections')">UserDirections</a>  - Algorithm usage directions
%   
%   Clients of AutomationAlgorithm implement the following user-defined
%   methods to define execution of the algorithm:
%
%   <a href="matlab:help('vision.labeler.AutomationAlgorithm.checkLabelDefinition')">checkLabelDefinition</a>          - Check if label definition is valid
%   <a href="matlab:help('vision.labeler.AutomationAlgorithm.checkSetup')">checkSetup</a>                    - Check if algorithm is ready for execution (optional)
%   <a href="matlab:help('vision.labeler.AutomationAlgorithm.checkSignalType')">checkSignalType</a>               - Check if signal type is valid
%   <a href="matlab:help('vision.labeler.AutomationAlgorithm.supportsMultisignalAutomation')">supportsMultisignalAutomation</a> - Check if algorithm supports automation of multiple signals
%   <a href="matlab:help('vision.labeler.AutomationAlgorithm.initialize')">initialize</a>                    - Initialize state for algorithm execution (optional)
%   <a href="matlab:help('vision.labeler.AutomationAlgorithm.run')">run</a>                           - Run algorithm on image
%   <a href="matlab:help('vision.labeler.AutomationAlgorithm.terminate')">terminate</a>                     - Terminate algorithm execution and clean up state (optional)
%
%   Clients of AutomationAlgorithm can also implement the following
%   user-defined methods:
%   
%   <a href="matlab:help('vision.labeler.AutomationAlgorithm.settingsDialog')">settingsDialog</a>      - Define settings dialog
%   Constructor         - A constructor can be defined, but must take no
%                         input arguments.
%
%   
%   See also imageLabeler, videoLabeler, groundTruthLabeler, 
%   lidarLabeler, vision.labeler.mixin.Temporal.
   

% Copyright 2017-2024 The MathWorks, Inc.

classdef AutomationAlgorithm < handle
    
    %======================================================================
    % Application Programming Interface
    %======================================================================
    
    properties (Abstract, Constant)
        %Name Algorithm name
        %   Character vector specifying name of Automation Algorithm
        %   defined.
        Name
        
        %Description Algorithm Description
        %   Character vector describing Automation Algorithm.
        Description
    
        %UserDirections Algorithm usage directions
        %   UserDirections is used to specify a set of directions displayed
        %   in the right panel of the App. Specify UserDirections as a cell
        %   array of character vectors.
        UserDirections
               
    end
    
    %----------------------------------------------------------------------
    % Use this property to query labels in the App.
    %----------------------------------------------------------------------
    properties (GetAccess = public, SetAccess = private)
        
        %GroundTruth Ground truth labels
        %   groundTruth object holding all labels marked in the labeler app
        %   prior to automation. See <a href="matlab:help('groundTruth')">groundTruth</a> for a
        %   description of the groundTruth object. The object will be a
        %   groundTruthMultisignal object, in the case of Ground Truth
        %   Labeler App. This app requires Automated Driving Toolbox (TM)
        %
        %   See also groundTruth, groundTruthMultisignal, groundTruthLidar.
        GroundTruth
        
        %SelectedLabelDefinitions Selected label definitions
        %   Struct corresponding to selected label definitions in the
        %   labeler app. The labeling apps support selection of only one
        %   label definition per automation session. The selected label
        %   definition is highlighted in yellow in either the 'ROI Labels'
        %   or 'Scene Labels' pane on the left.
        %
        %   The SelectedLabelDefinitions struct always contains fields Type
        %   and Name. Type is a <a href="matlab:help('labelType')">labelType</a> enumeration with possible values
        %   Rectangle, Line, Polygon, Projected cuboid, PixelLabel, Cuboid and
        %   Scene. Name is a character vector containing the name of the
        %   selected label definitions. If the selected label has
        %   attributes, an additional field, Attributes, contains a struct
        %   with attribute names as the field names. For label definitions
        %   of type PixelLabel, an additional field, PixelLabelID, holds
        %   the ID for each selected pixel label.
        %
        %   Example: SelectedLabelDefinitions for Rectangle
        %   -----------------------------------------------
        %   selectedLabelDefs.Name = 'Car';
        %   selectedLabelDefs.Type = labelType.Rectangle;
        %   selectedLabelDefs.Attributes = struct('distance', struct('DefaultValue', 0, 'Description', ''))
        %
        %   Example: SelectedLabelDefinitions for PixelLabel
        %   ------------------------------------------------
        %   selectedLabelDefs.Name         = 'Road';
        %   selectedLabelDefs.Type         = labelType.PixelLabel;
        %   selectedLabelDefs.PixelLabelID = 2;
        %
        %   See also labelType.
        SelectedLabelDefinitions
        
        %ValidLabelDefinitions Valid label definitions
        %   Struct array containing all valid label definitions in the
        %   labeler app. These are all the label definitions that satisfy
        %   the checkLabelDefinition method.
        %
        %   ValidLabelDefinitions is a struct array with fields Type and
        %   Name. If the label has attributes, it contains additional 
        %   field Attributes. Type is a <a href="matlab:help('labelType')">labelType</a> enumeration with possible values 
        %   Rectangle, Line, Polygon, Projected cuboid, PixelLabel and Scene. 
        %   Name is a character vector containing the name of the label.
        %   Attributes is a struct with attribute names as the field names.
        %   For label definitions of type PixelLabel, an additional field
        %   PixelLabelID holds the ID for each pixel label.
        %
        %   Example: ValidLabelDefinitions for Rectangle, Line and Scene
        %   ------------------------------------------------------------
        %   validLabelDefs(1).Name = 'Car';
        %   validLabelDefs(1).Type = labelType.Rectangle;
        %   validLabelDefs(2).Name = 'LaneMarker';
        %   validLabelDefs(2).Type = labelType.Line;
        %   validLabelDefs(3).Name = 'Sunny';
        %   validLabelDefs(3).Type = labelType.Scene;
        %
        %   Example: ValidLabelDefinitions for PixelLabel
        %   ---------------------------------------------
        %   validLabelDefs(1).Name          = 'Road';
        %   validLabelDefs(1).Type          = labelType.PixelLabel;
        %   validLabelDefs(1).PixelLabelID  = 1;
        %   validLabelDefs(2).Name          = 'Sky';
        %   validLabelDefs(2).Type          = labelType.PixelLabel;
        %   validLabelDefs(2).PixelLabelID  = 2;
        %
        %   See also labelType.
        ValidLabelDefinitions
        
        %CurrentIndex Index of the current frame
        %   Index of the current frame. This value is updated as the
        %   algorithm runs. Use this index value to access the current
        %   frame's ground truth data from GroundTruth property.
        CurrentIndex
    end
    
    properties (SetAccess = public)
        
        %SignalName Name of the signal that is automated
        %   A string holding the name of the signal that is being
        %   automated. This property is used when using Ground Truth
        %   Labeler App and the GroundTruth property is of type
        %   groundTruthMultisignal. The app and groundTruthMultisignal
        %   object requires Automated Driving Toolbox (TM)
        SignalName
    end
    
    properties (SetAccess = public)
        %SignalType Type of the selected signals that are being automated
        SignalType
    end
    
    %----------------------------------------------------------------------
    % Optionally override this method to provide Algorithm Settings for the
    % user in the labeler app.
    %----------------------------------------------------------------------
    methods
        %settingsDialog
        %   settingsDialog is invoked when the user clicks on the Settings
        %   button. Use this method to define a dialog for setting
        %   algorithm parameters.
        %
        %   settingsDialog(algObj) is used to display algorithm settings in
        %   a dialog. Use a modal dialog, created using functions such as
        %   dialog, inputdlg or listdlg.
        %
        %   See also dialog, inputdlg, listdlg.
        settingsDialog(this)
    end
    
    %----------------------------------------------------------------------
    % Override these methods to define how the automation algorithm
    % operates. checkLabelDefinition() and run() must be overridden, while
    % initialize() and terminate() are optional.
    %----------------------------------------------------------------------
    methods (Abstract)
        %checkLabelDefinition
        %   checkLabelDefinition is invoked for each ROI Label definition
        %   and Scene Label definition in the labeler app session. Use this
        %   function to restrict automation algorithms to certain label
        %   types relevant to the particular algorithm. Label definitions
        %   that return false will be disabled during automation.
        %
        %   isValid = checkLabelDefinition(algObj, labelDef) should return
        %   TRUE for valid label definitions and FALSE for invalid label
        %   definitions. labelDef is a struct containing two fields Type
        %   and Name. Type is an enumeration of class labelType with
        %   possible values Rectangle, Line, Polygon, Projected cuboid, Cuboid,
        %   PixelLabel and Scene. Name is a character vector containing the
        %   name of the specified label.
        %
        %   Below is an example of a labelDef structure:
        %
        %           Type: Rectangle
        %           Name: 'Car'
        %
        %
        %   Example: Restrict automation to only Rectangle ROI labels 
        %   ---------------------------------------------------------
        %
        %   function checkLabelDefinition(algObj, labelDef)
        %       
        %       isValid = (labelDef.Type == labelType.Rectangle);
        %   end
        %
        %   
        %   Example: Restrict automation to only PixelLabel labels
        %   ------------------------------------------------------
        %   
        %   function checkLabelDefinition(algObj, labelDef)
        %   
        %       isValid = (labelDef.Type == labelType.PixelLabel);
        %   end
        %
        %
        %   Example: Restrict automation to only Cuboid labels
        %   ------------------------------------------------------
        %   
        %   function checkLabelDefinition(algObj, labelDef)
        %   
        %       isValid = (labelDef.Type == labelType.Cuboid);
        %   end
        %
        %   Notes
        %   -----
        %   In order to access the selected label definitions (highlighted
        %   in yellow on the left panels), use the <a href="matlab:help('vision.labeler.AutomationAlgorithm.SelectedLabelDefinitions')">SelectedLabelDefinitions</a>
        %   property.
        %
        %   See also labelType,
        %   vision.labeler.AutomationAlgorithm.SelectedLabelDefinitions
        isValid = checkLabelDefinition(this, labelDef)
    end
    
    methods
        function isReady = checkSetup(this, varargin) %#ok<INUSD>
        %checkSetup
        %   checkSetup is invoked when the user clicks RUN. If checkSetup
        %   returns TRUE, the app proceeds to execute initialize(), run()
        %   and terminate(). If checkSetup returns FALSE or throws an
        %   exception, a dialog is displayed. If an exception is thrown,
        %   the dialog message echoes the exception message. If the method
        %   returns false, the dialog message asks the user to set up the
        %   algorithm correctly. This method is optional.
        %
        %   isReady = checkSetup(algObj) should return TRUE if the user
        %   completed setup correctly and the automation algorithm algObj
        %   is ready to begin execution, FALSE otherwise.
        %
        %   isReady = checkSetup(algObj, labelsToAutomate) additionally
        %   provides labelsToAutomate, all labels marked before executing
        %   the algorithm. labelsToAutomate is a table containing all
        %   labels marked before executing the algorithm. It is a table
        %   with variables Name, Type, Time and Position. checkSetup is
        %   called with this syntax only for temporal automation algorithms
        %   that do not support pixel labels.
        %
        %   ---------------------------------------------------------------
        %   VARIABLE NAME | DESCRIPTION
        %   --------------|------------------------------------------------
        %   Type          | labelType enumeration with possible values
        %                 | Rectangle, Line, Polygon, Projected cuboid, 
        %                 | Cuboid.
        %   --------------|------------------------------------------------
        %   Name          | Character vector specifying the label name.
        %   --------------|------------------------------------------------
        %   Time          | Scalar double specifying time stamp in seconds
        %                 | at which label was marked.
        %   --------------|------------------------------------------------
        %   Position      | Position specifying location of ROI labels and 
        %                 | empty for Scene labels as described below:
        %                 |
        %                 |------------------------------------------------
        %                 | LABEL TYPE | DESCRIPTION
        %                 |------------|-----------------------------------
        %                 | Rectangle  | 1-by-4 vector specifying position
        %                 |            | of bounding box locations as 
        %                 |            | [x y w h]. Multiple Rectangle ROIs
        %                 |            | can be specified as an M-by-4
        %                 |            | matrix.
        %                 |------------|-----------------------------------
        %                 | Line       | N-by-2 or N-by-3 vector specifying 
        %                 |            | N points along a polyline as:
        %                 |            |
        %                 |            | [x1,y1; x2,y2;...xN,yN] 
        %                 |            | [x1,y1,z1; x2,y2,z2;...;xN,yN,zN] 
        %                 |------------|-----------------------------------
        %                 | Polygon    | N-by-2 vector specifying N vertices
        %                 |            | of a polygon as:
        %                 |            |
        %                 |            | [x1,y1; x2,y2;...xN,yN] 
        %                 |------------|-----------------------------------
        %                 | Projected  | 1-by-8 vector specifying position
        %                 | Cuboid     | of primary and secondary faces as 
        %                 |            | [x1 y1 w1 h1 x2 y2 w2 h2]. Multiple 
        %                 |            | Projected cuboid ROIs can be 
        %                 |            | specified as an M-by-8 matrix.        
        %                 |------------|-----------------------------------
        %                 | Cuboid     | 1-by-9 vector specifying xctr,
        %                 |            | yctr, zctr, xlen, ylen, zlen,
        %                 |            | xrot, yrot, zrot
        %                 |            | 
        %                 |            |
        %                 |            | [xctr yctr zctr xlen ylen zlen
        %                 |            |  xrot yrot zrot]
        %   ---------------------------------------------------------------
        %
        %   Below is an example of a labelsToAutomate table:
        %
        %      Type           Name        Time           Position  
        %    _________    ____________    _________    ____________
        %
        %    Rectangle    'Car'           0.033333     [1x4 double]
        %    Line         'LaneMarker'    0.066667     [5x2 double]
        %    Cuboid       'truck'         0.099999     [1x9 double]
        %
        %   
        %   Example: Check that at least one ROI label is drawn
        %   ---------------------------------------------------
        %
        %   function isReady = checkSetup(algObj, labelsToAutomate)
        %   
        %       notEmpty = ~isempty(labelsToAutomate);
        %       
        %       hasROILabels = any(labelsToAutomate.Type == labelType.Rectangle);
        %
        %       if notEmpty && hasROILabels
        %           isReady = true;
        %       else
        %           isReady = false;
        %       end
        %       
        %   end
        %
        %   See also table, labelType.
        
        isReady = true;
        end
        
        function initialize(this, frame, varargin) %#ok<INUSD>
        %initialize
        %   initialize is invoked before the automation algorithm executes.
        %   Use this method to initialize state of the automation
        %   algorithm. This method is optional.
        %
        %   initialize(algObj, frame) initializes state of the automation
        %   algorithm algObj. For signalType Image, frame is a numeric
        %   matrix containing the image frame corresponding to the first
        %   image. For signalType PointCloud, frame is a pointCloud
        %   corresponding to the first frame.
        %
        %   initialize(algObj, frame, labelsToAutomate) additionally
        %   provides labelsToAutomate, all labels marked before executing
        %   the algorithm. initialize is called with this syntax only for
        %   temporal automation algorithms that do not support pixel
        %   labels.
        %
        %   See also vision.labeler.AutomationAlgorithm.checkSetup.
        end
        
    end
    
    methods (Static)
        % This method is static to allow the apps to call it and check the
        % signal type before instantiation. When users refresh the
        % algorithm list, we can quickly check and discard algorithms for
        % any signal that is not supported in a given app.
               
        function isValid = checkSignalType(signalType)
        %checkSignalType
        %   checkSignalType is invoked for the selected signal type in the
        %   labeler app session. Use this function to restrict automation
        %   algorithms to certain signal types relevant to the particular
        %   algorithm. By default, checkSignalType is set to allow Image
        %   signal types.
        %
        %   Example 1: Support Image signal types
        %
        %   function isValid = checkSignalType(signalType)
        %       isValid = (signalType == vision.labeler.loading.SignalType.Image);
        %   end
        
        %   Example 2: Support PointCloud signal types
        %
        %   function isValid = checkSignalType(signalType)
        %       isValid = (signalType == vision.labeler.loading.SignalType.PointCloud);
        %   end
        %
        %   See also vision.labeler.loading.SignalType
        isValid = (signalType == vision.labeler.loading.SignalType.Image);    
            
        end
        
        % This method is static to allow the apps to call it and check that
        % the algorithm supports automation of multiple signals before
        % instantiation. If the algorithm supports multisignal automation,
        % set success to true.
        %------------------------------------------------------------------
        function success = supportsMultisignalAutomation(~)
            %supportsMultisignalAutomation
            %   supportsMultisignalAutomation is to allow the apps to
            %   call it and check that the algorithm supports automation of
            %   multiple signals. By default, supportsMultisignalAutomation
            %   is set to allow automation of single signal only.
            success = false;
        end
        
    end
    
    methods (Abstract)
        %run
        %   run is invoked on each image or pointCloud frame chosen for
        %   automation in the labeler app. Use this method to execute the
        %   algorithm to compute labels. Assign labels based on the
        %   algorithm in this method.
        %
        %   autoLabels = run(algObj, frame) processes a single Image or
        %   pointCloud and produces automated labels in autoLabels. The
        %   format of autoLabels depends on the type of automation
        %   algorithm being defined.
        %   
        %   Algorithms without pixel labels 
        %   -------------------------------
        %   For automation algorithms without pixel labels, autoLabels can
        %   either be a struct array (or table) with fields Type, Name,
        %   Position and optionally Attributes. The Attributes field exists
        %   only when labels with attributes have been defined.
        %
        %   The fields of the struct array are described below:
        %   
        %   Type        A <a href="matlab:help('labelType')">labelType</a> enumeration that defines the type of label. 
        %               Type can have values Rectangle, Line, Polygon, Projected 
        %               cuboid, Cuboid or Scene.
        %
        %   Name        A character vector specifying a label name that
        %               returns true for checkLabelDefinition. Only
        %               existing label names previously defined in the
        %               labeler app can be used.
        %
        %   Position    Positions of the labels. The type of label determines
        %               the format of the position data.
        %
        %   Attributes  An array of structs representing the attributes
        %               contained by the automated labels. Each attribute
        %               is specified as a field of the struct, with the
        %               name of the field representing the name of the
        %               attribute and the value of the field representing
        %               the value of the attribute.
        %
        %   Algorithms with pixel labels
        %   ----------------------------
        %   For automation algorithms with pixel labels, autoLabels must be
        %   a <a href="matlab:helpview('vision','categoricalLabelMatrix')">Categorical label matrix</a>, where each category represents a
        %   pixel label.
        %
        %
        %   Below is an example of how to specify an autoLabels structue
        %   for an algorithm that detects a Car, a Truck, finds a lane and
        %   classifies the scene as Sunny.
        %   
        %   % Rectangle labeled 'Car' positioned with top-left at (20,20)
        %   % with width and height equal to 50.
        %   autoLabels(1).Name      = 'Car';
        %   autoLabels(1).Type      = labelType('Rectangle');
        %   autoLabels(1).Position  = [20 20 50 50];
        %
        %   % Cuboid labeled 'truck'
        %   autoLabels(2).Name      = 'truck';
        %   autoLabels(2).Type      = labelType('Cuboid');
        %   autoLabels(2).Position  = [10 10 10 50 50 50 0 0 0];
        %
        %   % Line labeled 'LaneMarker' with 3 points.
        %   autoLabels(3).Name      = 'LaneMarker';
        %   autoLabels(3).Type      = labelType('Line');
        %   autoLabels(3).Position  = [100 100; 100 110; 110 120];
        %
        %   % Scene labeled 'Sunny'
        %   autoLabels(4).Name      = 'Sunny';
        %   autoLabels(4).Type      = labelType('Scene');
        %   autoLabels(4).Position  = true; 
        %
        %   See also categorical.
        autoLabels = run(this, frame)
    end
    
    methods
        function terminate(this) %#ok<MANU>
        %terminate
        %   terminate is invoked after run() has been invoked on the last
        %   frame in the specified interval or the user stops algorithm
        %   running. Use this method to clean up state. This method is
        %   optional.
        %
        %   terminate(algObj) cleans up state of the automation algorithm
        %   algObj.    
            
        end
    end
    
    %======================================================================
    % Implementation
    %======================================================================
    
    properties (Access = private, Hidden)
        %Version Version
        %   Tag used to record versioning of object.
        Version = ver('vision');
    end
    
    methods (Hidden)
        %------------------------------------------------------------------
        % Use this method to identify if the concrete automation algorithm
        % class inherits the temporal mixin.
        %------------------------------------------------------------------
        function tf = hasTemporalContext(algObj)
            tf = isa(algObj, 'vision.labeler.mixin.Temporal');
        end
        
        %------------------------------------------------------------------
        % Use this method to identify if the concrete automation algorithm
        % class is the old driving.automation.AutomationAlgorithm instead
        % of the new vision.labeler.AutomationAlgorithm with (or
        % without) the temporal mixin.
        %------------------------------------------------------------------
        function tf = isDrivingAutomationAlgorithm(algObj)
            tf = isa(algObj, 'driving.automation.AutomationAlgorithm');
        end
    end
    
    methods (Static, Hidden)
        %------------------------------------------------------------------
        function openTemplateInEditor(varargin)
            %openTemplateInEditor opens a template AutomationAlgorithm
            %class in the MATLAB editor.
            if nargin > 0
                [varargin{:}] = convertStringsToChars(varargin{:});
            end
            if nargin==1
                templateType = varargin{1};
            else
                templateType = 'nontemporal';
            end
            
            if strcmpi(templateType,'temporal')
                fileName = 'TemporalAutomationAlgorithmExample';
            elseif strcmpi(templateType,'multisignal')
                fileName = 'MultiSignalAutomationAlgorithmExample';
            elseif strcmpi(templateType,'blockedImageAutomation')
                fileName = 'BlockedImageAutomationAlgorithmExample';            
            else
                fileName = 'AutomationAlgorithmExample';
            end
            
            % Read in template code. Use full path to avoid local versions
            % of the file from being read.
            example = fullfile(toolboxdir('vision'),'vision','+vision',...
                '+internal',[fileName, '.m']);
            fid = fopen(example);
            contents = fread(fid,'*char');
            fclose(fid);
            
            % Open template code in an untitled file in the editor
            editorDoc = matlab.desktop.editor.newDocument(contents');
            
            % Change the function name to the name of the untitled file
            contents = regexprep(editorDoc.Text,...
                fileName, 'MyCustomAlgorithm','once');
            
            editorDoc.Text = contents;
            editorDoc.smartIndentContents;
            editorDoc.goToLine(1);
        end
        
        %------------------------------------------------------------------
        function text = getDefaultUserDirections(name,varargin)
            if nargin > 0
                [varargin{:}] = convertStringsToChars(varargin{:});
            end
            switch name
                case 'selectroidef'
                    labelName = varargin{1};
                    text = vision.getMessage('vision:labeler:SelectROIDefinitionInstruction', labelName);
                case 'rundetector'
                    labelName = varargin{1};
                    text = vision.getMessage('vision:labeler:RunDetectorInstruction', labelName);
                case 'review'
                    text = vision.getMessage('vision:labeler:ReviewInstruction');
                case 'rerun'
                    text = vision.getMessage('vision:labeler:RerunInstruction');
                case 'accept'
                    text = vision.getMessage('vision:labeler:AcceptInstruction');
            end
        end
    end
    
    methods (Access = { ?vision.internal.labeler.tool.LabelerTool,...
                        ?vision.internal.labeler.tool.AlgorithmSetupHelper,...
                        ?vision.internal.imageLabeler.multiUser.view.AutomationTab,...
                        ?vision.internal.labeler.multiUser.view.tabs.AutomationTab,...
                        ?vision.internal.labeler.multiUser.controller.LabelerToolMultiUser, ...
                        ?vision.internal.labeler.multiUser.view.View, ...
                        ?vision.internal.labeler.automation.SemiAutomatedAlgorithmAdaptor,...
                        ?matlab.unittest.TestCase})
        %------------------------------------------------------------------
        % Setup
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        function setVideoLabels(this, labels)
            
            assert(isa(labels,'groundTruth') || ...
                isa(labels,'groundTruthMultisignal') ||...
                isa(labels,'groundTruthLidar'), ...
                'Expected a groundTruth object');
            this.GroundTruth = labels;
        end
                
        %------------------------------------------------------------------
        function setValidLabelDefinitions(this, labelDefs)
            
            this.ValidLabelDefinitions = labelDefs;
        end
        
        %------------------------------------------------------------------
        function setSelectedLabelDefinitions(this, selections)
            
            this.SelectedLabelDefinitions = selections;
        end
        
        %------------------------------------------------------------------
        function setCurrentIndex(this, index)
            assert(isscalar(index),'CurrentIndex must be a scalar!');
            this.CurrentIndex = index;
        end

        %------------------------------------------------------------------
        function importLabels(varargin)
            % This is a stub. Automation Algorithms with no temporal
            % component cannot import labels.
        end
        
        %------------------------------------------------------------------
        function isReady = verifyAlgorithmSetup(this)

            isReady = checkSetup(this);
        end

        %------------------------------------------------------------------
        % Execution
        %------------------------------------------------------------------
        
        %------------------------------------------------------------------
        function doInitialize(this, frame)
            for i = 1 : numel(this.ValidLabelDefinitions.Type)
                if this.ValidLabelDefinitions(i).Type == labelType.Cuboid
                    this.ValidLabelDefinitions(i).Type= labelType.Cuboid;
                    this.SelectedLabelDefinitions(i).Type = labelType.Cuboid;
                end
            end
            initialize(this, frame);
        end
        %------------------------------------------------------------------
        function videoLabels = doRun(this, frame, info)
            if this.isBlockedImageAutomation()
                % run() input : bim is a blockedImage
                % run() output : bout is a blockedImage
                if ~isa(frame,'blockedImage')
                    bim = blockedImage(frame);
                else
                    bim = frame;
                end
                bout = run(this,bim);
                % Extract data from the blockedImage "bout" output
                blockedImageLabelData = bout.gather();
                if ~iscategorical(blockedImageLabelData)
                    % Extract ROIs : Rectangle, Line, Polygon 
                    videoLabels = this.extractROIsFromBlockedImageStruct(blockedImageLabelData);
                else
                    % Extract categorical label data for blockedImage
                    % automation. This indicates the images are regular
                    % images (not Blocked Image), as Blocked Images do not
                    % support pixel labels
                    videoLabels = blockedImageLabelData;
                end
                % Delete tempname directory created for blockedImage/apply
                if ischar(bout.Source)
                    rmdir(bout.Source,'s');
                end
            else
                if (nargin == 3)
                    videoLabels = run(this, frame, info);
                else
                    videoLabels = run(this, frame);
                end
            end
            videoLabelsTemp = cell(numel(this.SignalName), 1);
            % Check the entries in videoLabels and convert it to a struct.
            for i = 1:numel(this.SignalName)
                if (numel(this.SignalName) == 1)
                    % to Handle single signal cases
                    videoLabelsTemp{i,1} = checkVideoLabelValidity(this, videoLabels);
                    videoLabels = videoLabelsTemp;
                else
                    videoLabels{i,1} = checkVideoLabelValidity(this, videoLabels{i,1});
                end
            end
            
            if(isempty(this.SignalName))
                % to Handle ImageLabeler
                videoLabels = checkVideoLabelValidity(this, videoLabels);
            end
        end
        
        %------------------------------------------------------------------
        function hasSettings = hasSettingsDefined(this)
            
            % Find the method associated with settingsDialog
            meta = metaclass(this);
            
            methodNames = {meta.MethodList.Name};
            settingsMethodIdx = strcmp('settingsDialog', methodNames);
            
            settingsMethod = meta.MethodList(settingsMethodIdx);
            settingsMethod = settingsMethod(1);
            
            % If the defining class for this method is abstract,
            % settingsDialog was not defined.
            hasSettings = ~settingsMethod.DefiningClass.Abstract;
        end
        
        %------------------------------------------------------------------
        function doSettings(this)
            
            settingsDialog(this);
        end
    end
    
    methods (Access = private)
        %------------------------------------------------------------------
        function videoLabels = checkVideoLabelValidity(algObj, videoLabels)
            
            if ~isempty(videoLabels)
                
                catValidity = iscategorical(videoLabels);
                if catValidity
                    if ~isa(algObj, 'vision.labeler.FunctionalAutomationAlgorithm')
                        % Custom automation functions permit other label
                        % types along with pixel labels for categorical 
                        % automation labels. Custom automation classes do
                        % not.

                        % All labels must be pixel labels
                        if ~all([algObj.ValidLabelDefinitions.Type]==labelType.PixelLabel)
                            error(vision.getMessage('vision:labeler:InvalidCategoricalAutoLabels', algObj.Name));
                        end
                    end
                elseif ~isempty(videoLabels)
                    
                    % Check that struct fields match
                    expFieldNames = {'Type','Name','Position'};
                    isStructLabels = isstruct(videoLabels);
                    if isStructLabels && ~(all(isfield(videoLabels, expFieldNames)))
                        error(vision.getMessage('vision:labeler:InvalidStructAutoLabels', algObj.Name));
                    end

                    isTableLabels  = istable(videoLabels);
                    if isTableLabels
                        % Check that table column names match
                        variableNames = videoLabels.Properties.VariableNames;

                        if ~(all(ismember(lower(expFieldNames), lower(variableNames))))
                            error(vision.getMessage('vision:labeler:InvalidTableAutoLabels', algObj.Name));
                        end

                        videoLabels = table2struct(videoLabels);
                    end
                    
                    if isStructLabels || isTableLabels
                        arrayfun(@validateEachEntry, videoLabels);
                    else
                        error(vision.getMessage('vision:labeler:InvalidDataTypeAutoLabels', algObj.Name));
                    end

                    % g2936018: Convert categorical label names to strings
                    categoricalNameIdx = arrayfun(@isNameCategorical, videoLabels);
                    if any(categoricalNameIdx)
                        videoLabels(categoricalNameIdx) = arrayfun(@convertNameToString, videoLabels(categoricalNameIdx));
                    end
                end
            end
            
            %--------------------------------------------------------------
            function validateEachEntry(s)
                name = s.Name;
                type = s.Type;
                pos  = s.Position;

                % Check that name is a char vector, string, or categorical.
                if ~ischar(name) && ~((isstring(name) || iscategorical(name)) && isscalar(name))
                    error(vision.getMessage('vision:labeler:InvalidNameAutoLabels', algObj.Name));
                end

                % Check that type is a labelType enum
                if ~isa(type,'labelType')
                    error(vision.getMessage('vision:labeler:InvalidTypeAutoLabels', algObj.Name));
                end

                % Check position value based on label type. 
                if ismember(type, [labelType.Rectangle; labelType.RotatedRectangle; ...
                        labelType.Line; labelType.Polygon; labelType.Cuboid; ...
                        labelType.ProjectedCuboid; labelType.Point])
                    % Validate shape data
                    if ~vision.internal.labeler.validation.validateShapeData(pos, type)
                        error(vision.getMessage('vision:labeler:InvalidPositionAutoLabels', algObj.Name, string(type)));
                    end
                    
                elseif isequal(type, labelType.Scene)
                     if ~islogical(pos)
                         error(vision.getMessage('vision:labeler:InvalidSceneAutoLabels', algObj.Name));
                     end

                elseif isequal(type, labelType.PixelLabel)
                    error(message('vision:labeler:PixelLabelAutoLabelsMustBeCategorical'));
                else
                    assert(false,'Invalid label Type')
                end
            end

            %--------------------------------------------------------------
            function tf = isNameCategorical(s)
                tf = iscategorical(s.Name);
            end

            %--------------------------------------------------------------
            function s = convertNameToString(s)
                s.Name = string(s.Name);
            end
        end
        
        function [videoLabels,isValid] = checkVideoLabelStructFieldValidity(~, videoLabels)
            if ~isempty(videoLabels)              
                % Check that struct fields match
                structValidity = isstruct(videoLabels) ...
                    && (all(isfield(videoLabels,{'Type','Name','Position'})));
                
                % Check that table column names match
                tableValidity  = istable(videoLabels);
                if tableValidity
                    variableNames = videoLabels.Properties.VariableNames;
                    
                    tableValidity = tableValidity ...
                        && any(strcmpi('Type',variableNames)) ...
                        && any(strcmpi('Name',variableNames)) ...
                        && any(strcmpi('Position',variableNames));
                    
                    videoLabels = table2struct(videoLabels);
                end
                
                isValid = structValidity || tableValidity;
            end
        end
        
        function tf = isBlockedImageAutomation(this)
            metaClass = metaclass(this);
            metaSuperclass = metaClass.SuperclassList;
            superclasses   = {metaSuperclass.Name};            
            expectedClass = 'vision.labeler.mixin.BlockedImageAutomation';
            tf = ismember(expectedClass, superclasses);           
        end
    end
    
    methods (Hidden)
        
        function [rois] = extractROIsFromBlockedImageStruct(this, blockedImageStructArray)
            % Extracting ROIs from blockedImage Automation. Performing a
            % gather operation extracts the underlying structs stored in
            % the blockedImage : bout. The struct must contain 3 fields,
            % i.e. Type, Name and Position.

            % Check that the struct array contains all required fields
            [~, isValid] = checkVideoLabelStructFieldValidity(this, blockedImageStructArray(1));
            if ~isValid
                error(getString( message('vision:imageLabeler:IncorrectStructFormatBlockedImageAutomation')));
            end
            % Get the indices of blocks that were processed in the apply
            % call from the BlockLocationSet passed to blockedImage/apply().
            blockLocationSet = this.getBlockLocationSet();
            blockSize = this.getBlockSize();
            blockSubs = (blockLocationSet.BlockOrigin(:,[2 1]) - 1)./blockSize + 1;
            structArraySize = size(blockedImageStructArray);
            blockIndices = sub2ind(structArraySize, blockSubs(:,1),blockSubs(:,2));

            names = {blockedImageStructArray(blockIndices).Name};
            [uniqueNames, ~, ic] = unique(names);

            hasAttributes = isfield(blockedImageStructArray,'Attributes');
            if hasAttributes
                roiStruct = struct('Name','','Type',[],'Position',[],'Attributes',[]);
            else
                roiStruct = struct('Name','','Type',[],'Position',[]);
            end
            rois = repmat(roiStruct, numel(uniqueNames), 1);

            for uniqueNameIdx = 1:numel(uniqueNames)
                idx = blockIndices(uniqueNameIdx == ic);

                rois(uniqueNameIdx).Name = uniqueNames{uniqueNameIdx};
                rois(uniqueNameIdx).Position = vertcat(blockedImageStructArray(idx).Position);
                rois(uniqueNameIdx).Type = blockedImageStructArray(idx(1)).Type;

                if hasAttributes
                    rois(uniqueNameIdx).Attributes = vertcat(blockedImageStructArray(idx).Attributes);
                end
            end
        end
    end
end
