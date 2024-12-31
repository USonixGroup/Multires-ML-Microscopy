classdef CharucoBoardDetectorImpl < vision.calibration.PatternDetector
    % CharucoBoardDetectorImpl Implements the interface for ChArUco board detection for camera calibration.
    %
    %   CharucoBoardDetectorImpl is the base class for ChArUco board detectors
    %   in the Single and Stereo Camera Calibrator Apps. It implements the interface
    %   for these detectors to be used in the apps and from the command line.
    %
    %   For more information on defining a custom calibration pattern detector
    %   using this interface, see the PatternDetector class:
    %
    %       edit vision.calibration.PatternDetector

    % Copyright 2024 The MathWorks, Inc.

    %----------------------------------------------------------------------
    properties(Constant)
        
        Name = vision.getMessage("vision:caltool:CharucoBoardPattern");
    end

    %----------------------------------------------------------------------
    properties
        Panel

        WorldUnits = vision.getMessage("vision:caltool:millimeters");

        MarkerFamily = "DICT_4X4_1000";

        PatternDims

        CheckerSize

        MarkerSize

        MinMarkerID

        OriginCheckerColor
    end

    properties (Access = private)
        ParentFigure
    end

    %----------------------------------------------------------------------
    % UI components for the properties panel.
    %----------------------------------------------------------------------
    properties (Access = private)

        WorldUnitsPopup
        
        CheckerSizeEditBox

        MarkerSizeEditBox

        MarkerFamilyPopup

        PatternDimsEditBox

        MinMarkerIDEditBox

        OriginCheckerColorButtons

        OriginCheckerColorBlackButton

        OriginCheckerColorWhiteButton

        TemplateImageBrowseButton
    end

    %----------------------------------------------------------------------
    properties (Access = private, Constant)
        AvailableUnits = vision.internal.calibration.availableWorldUnits;

        AvailableMarkerFamilies = vision.internal.supportedCharucoBoardFamilies;

        PropertiesGuideDark = fullfile(toolboxdir("vision"),"vision", ...
                    "+vision", "+internal", "+calibration", ...
                    "+tool", "CharucoPropertiesVisualGuideDark.png");
        
        PropertiesGuideLight = fullfile(toolboxdir("vision"),"vision", ...
                "+vision", "+internal", "+calibration", ...
                "+tool", "CharucoPropertiesVisualGuideLight.png");
        
        MeasurementsGuideDark = fullfile(toolboxdir("vision"),"vision", ...
                "+vision", "+internal", "+calibration", ...
                "+tool", "CharucoMeasurementsVisualGuideDark.png");

        MeasurementsGuideLight = fullfile(toolboxdir("vision"),"vision", ...
                "+vision", "+internal", "+calibration", ...
                "+tool", "CharucoMeasurementsVisualGuideLight.png");
    end

    %----------------------------------------------------------------------
    methods

        %------------------------------------------------------------------
        % Detection method for pattern keypoint detection in the calibration
        % images.
        %------------------------------------------------------------------
        function [imagePoints, imagesUsed] = detectPatternPoints(this, varargin)

            if isa(this, "vision.calibration.stereo.CharucoBoardDetector")
                imageFileNames = varargin(1:2);
                otherInputs    = varargin(3:end);
            else
                imageFileNames = varargin(1);
                otherInputs    = varargin(2:end);
            end
            
            detectorParams = validateAndParseInputs(this, otherInputs{:});
            
            [imagePoints, imagesUsed, userCanceled] = detectCharucoBoardPoints(...
                imageFileNames{:}, detectorParams{:});

            vision.internal.errorIf(userCanceled, "vision:uitools:LoadingCanceledByUser");
        end

        %--------------------------------------------------------------------
        function inputs = validateAndParseInputs(this, patternDims, markerFamily, checkerSize, markerSize, options)
            arguments
                this %#ok
                patternDims  {validatePatternDims} = this.PatternDims
                markerFamily {validateMarkerFamily} = this.MarkerFamily
                checkerSize  {validateMeasurement(checkerSize,"checkerSize")} = this.CheckerSize
                markerSize   {validateMeasurement(markerSize,"markerSize")} = this.MarkerSize
                options.MinMarkerID {validateMinMarkerID(options.MinMarkerID, markerFamily)} = this.MinMarkerID
                options.OriginCheckerColor {validateOriginCheckerColor} = this.OriginCheckerColor
                options.ShowProgressBar    {validateShowProgressBar} = true
                options.ProgressBarParent  {validateProgressBarParent} = []
            end
            
            inputs = {patternDims, markerFamily, checkerSize, markerSize, ...
                "MinMarkerID", options.MinMarkerID, "OriginCheckerColor", ...
                options.OriginCheckerColor, "ShowProgressBar", options.ShowProgressBar, ...
                "ProgressBarParent", options.ProgressBarParent};
        end

        %------------------------------------------------------------------
        % World point generation.
        %------------------------------------------------------------------
        function worldPoints = generateWorldPoints(this, options)
            arguments
                this %#ok
                options.patternDims  (:,:)  {validatePatternDims} = this.PatternDims
                options.checkerSize  (1,1)  {validateMeasurement(options.checkerSize,"checkerSize")} = this.CheckerSize
            end
            
            worldPoints = patternWorldPoints("charuco-board", options.patternDims,...
                options.checkerSize);
        end
    end

    %----------------------------------------------------------------------
    methods

        %------------------------------------------------------------------
        % Set up properties panel related to ChArUco board detection.
        %------------------------------------------------------------------
        function propertiesPanel(this, panel)
            this.Panel = panel;
            this.ParentFigure = ancestor(panel,"Figure");

            % Add and configure UI components in the properties panel.
            configureUIComponents(this);

            % Initialize property values.
            initializePropertyValues(this);
        end

        %------------------------------------------------------------------
        % Set up method for drawing axes labels.
        %------------------------------------------------------------------
        function [originLabel, xLabel, yLabel] = drawImageAxesLabels(this, imagePoints)
            
            numPatternRows = min(this.PatternDims)-1;
            numPatternCols = max(this.PatternDims)-1;
            [originLabel, xLabel, yLabel] = ...
                vision.internal.calibration.tool.getAxesLabelPositions(...
                imagePoints, numPatternRows, numPatternCols);
        end

        %------------------------------------------------------------------
        % Public API to enable/disable dialog interactions and optionally 
        % update board properties.
        %------------------------------------------------------------------
        function updateDialogState(this, state, varargin)

            % Update dialog only if it exists.
            if ishandle(this.ParentFigure)
                
                [allHandles, accordion] = getAllInteractiveHandles(this);
                
                updateStates(this, state, allHandles, accordion);
    
                if nargin > 2
                    updateBoardProperties(this, varargin{:})
                end
            end
        end
        %------------------------------------------------------------------
        function [allHandles, accordion] = getAllInteractiveHandles(this)

            % Get parent UI figure and all UI elements in the figure.
            children = this.ParentFigure.Children.Children;

            % Helper function handles to identify grid layouts and accordions.
            isaGridLayout = @(y) arrayfun(@(x)isa(x,"matlab.ui.container.GridLayout"), y);
            isaAccordion = @(y) arrayfun(@(x)isa(x,"matlab.ui.container.internal.Accordion"), y);

            % Remove grid layouts from the children.
            uiHandles = children(~isaGridLayout(children));
            
            % Remove accordion handles from the left over children.
            allHandles = uiHandles(~isaAccordion(uiHandles));

            % Add all interactive handles under each grid layout.
            gridLayoutHandles = children(isaGridLayout(children));
            allHandles = vertcat(allHandles,gridLayoutHandles.Children);

            accordion = uiHandles(isaAccordion(uiHandles));
        end

        %------------------------------------------------------------------
        function updateStates(this, state, allHandles, accordion)

            accordion.Visible = state;

            set(allHandles, "Enable", state);

            if state
                this.ParentFigure.Pointer = "arrow";
            else
                this.ParentFigure.Pointer = "watch";
            end
        end
    end

    %----------------------------------------------------------------------
    % Helpers to configure UI components on the properties panel.
    %----------------------------------------------------------------------
    methods (Access = private)

        %------------------------------------------------------------------
        function configureUIComponents(this)

            % Create a grid layout with two rows for the board measurement
            % and charuco properties panels.
            panelLayout = uigridlayout(this.Panel, Padding=0);
            panelLayout.RowHeight = {"fit","fit"};
            panelLayout.ColumnWidth = "1x";

            % Configure measurements panel.
            configureMeasurementsPanel(this, panelLayout);

            % Configure properties panel.
            configureCharucoPropsPanel(this, panelLayout);
        end

        %------------------------------------------------------------------
        %  Configure measurements panel using the following grid layout
        %------------------------------------------------------------------
        %  ---------------------------------
        % | Board Measurements              |
        % |---------------------------------|
        % |        |               |        |  <--- Row 1 (Checker Size)
        % |------------------------|        |
        % |        |               |        |  <--- Row 2 (Marker Size)
        % |------------------------|        |
        % |        |               |        |  <--- Row 3 (Units)
        %  ---------------------------------
        %------------------------------------------------------------------
        function configureMeasurementsPanel(this, panelLayout)

            % Measurement panel with title
            measurementsPanel = uipanel(panelLayout,...
                Title=vision.getMessage("vision:caltool:MeasurementsPanelTitle"));
            measurementsPanel.Layout.Row = 1;

            % Define a grid layout within the panel: (UI elements + image)
            panelInnerLayout = uigridlayout(measurementsPanel);
            panelInnerLayout.RowHeight = {"fit"};
            panelInnerLayout.ColumnWidth = {"fit", "0.3x"};

            % Define a grid layout for the UI elements: (Labels + UI element)
            rowHeight = 25;
            gridLayout = uigridlayout(panelInnerLayout, Padding=2);
            gridLayout.Layout.Column = 1;
            gridLayout.RowHeight = {rowHeight, rowHeight, rowHeight};
            gridLayout.ColumnWidth = {"1.4x", "1x"};

            % Configure the UI components.
            configureCheckerSizeSelector(this, gridLayout, 1);
            configureMarkerSizeSelector(this, gridLayout, 2);
            configureUnitsSelector(this, gridLayout, 3);
            
            % Configure the UI component for measurements thumbnail image.
            configureMeasurementsPanelThumbnail(this, panelInnerLayout);
        end

        %------------------------------------------------------------------
        %  Configure properties panel using the following grid layout
        %------------------------------------------------------------------
        %  ---------------------------------
        % | ChArUco Properties              |
        % |---------------------------------|
        % |                     |           |  <--- Row 1 (Template selector)
        % |---------------------------------|
        % |        |               |        |  <--- Row 2 (Marker family)
        % |------------------------|
        % |        |               |        |  <--- Row 3 (Pattern Dims)
        % |------------------------|        |
        % |        |               |        |  <--- Row 4 (Min Marker ID)
        % |------------------------|        |
        % |        |               |        |  <--- Row 5 (Origin Checker Color)
        %  ---------------------------------
        %------------------------------------------------------------------
        function configureCharucoPropsPanel(this, panelLayout)

            % ChArUco properties with title
            charucoPanel = uipanel(panelLayout,...
                Title=vision.getMessage("vision:caltool:CharucoPropsPanelTitle"));
            charucoPanel.Layout.Row = 2;

            % Define a grid layout within the panel: (UI elements + image)
            rowHeight = 25;
            panelInnerLayout = uigridlayout(charucoPanel);
            panelInnerLayout.RowHeight = {rowHeight, "fit"};
            panelInnerLayout.ColumnWidth = {"fit", "fit"};

            % Configure the template selector for auto-populating properties.
            configureTemplateSelector(this, panelInnerLayout, 1)
            
            % Define a grid layout for the UI elements: (Labels + UI element)
            gridLayout = uigridlayout(panelInnerLayout, Padding=10);
            gridLayout.RowHeight = {rowHeight, rowHeight, rowHeight, rowHeight};
            gridLayout.ColumnWidth = {"fit", 150};
            gridLayout.Layout.Column = 1;
            gridLayout.Layout.Row = 2;

            % Configure the UI components.
            configureMarkerFamilyUIComponents(this, gridLayout, 1);
            configurePatternDimsSelector(this, gridLayout, 2);
            configureMinimumMarkerID(this, gridLayout, 3);
            configureOriginCheckerColor(this, gridLayout, 4);
            
            % Configure the UI component for properties thumbnail image.
            configurePropertiesPanelThumbnail(this, panelInnerLayout);
        end
        
        %------------------------------------------------------------------
        function configureCheckerSizeSelector(this, gridLayout, rowIdx)

            % CheckerSize Label
            checkerSizeLabel = uilabel(Parent=gridLayout,...
                Text=vision.getMessage("vision:caltool:CheckerSize"));
            checkerSizeLabel.Layout.Column = 1;
            checkerSizeLabel.Layout.Row = rowIdx;

            % CheckerSize Editbox
            initCheckerSize = 20;
            this.CheckerSizeEditBox = uieditfield(gridLayout, ...
                Tag="CheckerSizeEditBox", Value=num2str(initCheckerSize), ...
                ValueChangedFcn=@(~, ~) doCheckerSizeChanged(this), ...
                Tooltip=vision.getMessage("vision:caltool:CheckerSizeToolTip"));
            this.CheckerSizeEditBox.Layout.Column = 2;
        end

        %------------------------------------------------------------------
        function configureMarkerSizeSelector(this, gridLayout, rowIdx)

            % Marker Size Label
            markerSizeLabel = uilabel(Parent=gridLayout, ...
                Text=vision.getMessage("vision:caltool:MarkerSize"));
            markerSizeLabel.Layout.Column = 1;
            markerSizeLabel.Layout.Row = rowIdx;

            % Marker Size Editbox
            initMarkerSize = 15;
            this.MarkerSizeEditBox = uieditfield(gridLayout, ...
                Tag="MarkerSizeEditBox", Value=num2str(initMarkerSize), ...
                ValueChangedFcn=@(~, ~) doMarkerSizeChanged(this), ...
                Tooltip=vision.getMessage("vision:caltool:MarkerSizeToolTip"));
            this.MarkerSizeEditBox.Layout.Column = 2;
        end

        %------------------------------------------------------------------
        function configureUnitsSelector(this, gridLayout, rowIdx)

            % Units Label
            unitsLabel = uilabel(Parent=gridLayout, ...
                Text=vision.getMessage("vision:caltool:MeasurementUnits"));
            unitsLabel.Layout.Column = 1;
            unitsLabel.Layout.Row = rowIdx;

            % Units Selection
            initUnits = vision.getMessage("vision:caltool:millimeters");
            this.WorldUnitsPopup = uidropdown(Parent=gridLayout,...
                Items=this.AvailableUnits, Value=initUnits, ...
                ValueChangedFcn=@(~, ~) doUnitsChanged(this), ...
                Tooltip=vision.getMessage("vision:caltool:MeasurementUnitsToolTip"), ...
                Tag="UnitsSelectorPopup");
            this.WorldUnitsPopup.Layout.Row = rowIdx;
            this.WorldUnitsPopup.Layout.Column = 2;
        end

        %------------------------------------------------------------------
        function configureTemplateSelector(this, panelGridLayout, rowIdx)

            % Grid layout for 2 columns: label and button.
            gridLayout = uigridlayout(panelGridLayout, Padding=0);
            gridLayout.RowHeight = {"fit"};
            gridLayout.ColumnWidth = {"fit","fit"};
            gridLayout.Layout.Row = rowIdx;
            gridLayout.Layout.Column = [1 2];
            
            % Auto-populate Prompt Label
            templateImageLabel = uilabel(Parent=gridLayout, ...
                Text=vision.getMessage("vision:caltool:AutoPopulatePrompt"));
            templateImageLabel.Layout.Row = 1;
            templateImageLabel.Layout.Column = 1;
            
            % Browse Button
            this.TemplateImageBrowseButton = uibutton(Parent=gridLayout,...
                Text=vision.getMessage("vision:caltool:BrowseButton"), ...
                ButtonPushedFcn=@(~, ~) doBrowseButtonPushed(this), ...
                Tooltip=vision.getMessage("vision:caltool:TemplateBrowseButtonToolTip"),...
                Tag="BrowseButton");
            this.TemplateImageBrowseButton.Layout.Row = 1;
            this.TemplateImageBrowseButton.Layout.Column = 2;
        end

        %------------------------------------------------------------------
        function configureMarkerFamilyUIComponents(this, gridlayout, rowIdx)

            % Marker Family Label
            label = uilabel(Parent=gridlayout, ...
                Text=vision.getMessage("vision:caltool:MarkerFamily"));
            label.Layout.Row = rowIdx;
            label.Layout.Column = 1;

            % Marker Family Selection
            initMarkerFamily = this.MarkerFamily;
            this.MarkerFamilyPopup = uidropdown(Parent=gridlayout,...
                Items=this.AvailableMarkerFamilies,Value=initMarkerFamily,...
                ValueChangedFcn=@(~, ~) doMarkerFamilyChanged(this),...
                Tooltip=vision.getMessage("vision:caltool:MarkerFamilyToolTip"),...
                Tag="MarkerFamilyPopup");
            this.MarkerFamilyPopup.Layout.Row = rowIdx;
            this.MarkerFamilyPopup.Layout.Column = 2;
        end

        %------------------------------------------------------------------
        function configurePatternDimsSelector(this, gridlayout, rowIdx)

            selectorGridLayout = uigridlayout(gridlayout, Padding=[0 0 2 2]);
            selectorGridLayout.RowHeight = "fit";
            selectorGridLayout.ColumnWidth = {"fit", "fit"};
            selectorGridLayout.Layout.Row = rowIdx;
            selectorGridLayout.Layout.Column = [1 2];

            % Pattern Dimensions Label
            patternDimsLabel = uilabel(Parent=selectorGridLayout, ...
                Text=vision.getMessage("vision:caltool:PatternDims"));
            patternDimsLabel.Layout.Column = 1;
            
            % Pattern Dimensions Editbox
            initPatternDims = "[ 4, 3 ]";
            this.PatternDimsEditBox = uieditfield(selectorGridLayout,...
                Tag="PatternDimsEditbox", Value=initPatternDims, ...
                ValueChangedFcn=@(~, ~) doPatternDimsChanged(this), ...
                Tooltip=vision.getMessage("vision:caltool:PatternDimsToolTip"));
            this.PatternDimsEditBox.Layout.Column = 2;
        end

        %------------------------------------------------------------------
        function configureMinimumMarkerID(this, gridLayout, rowIdx)
            
            % Minimum Marker ID Label
            minMarkerIDlabel = uilabel(Parent=gridLayout, ...
                Text=vision.getMessage("vision:caltool:MinMarkerID"));
            minMarkerIDlabel.Layout.Row = rowIdx;
            minMarkerIDlabel.Layout.Column = 1;

            % Minimum Marker ID Editbox
            initMinMarkerID = 0;
            this.MinMarkerIDEditBox = uieditfield(gridLayout, ...
                Tag="MinMarkerIDEditBox", Value=num2str(initMinMarkerID), ...
                ValueChangedFcn=@(~, ~) doMinMarkerIDChanged(this), ...
                Tooltip=vision.getMessage("vision:caltool:MinMarkerIDToolTip"));
            this.MinMarkerIDEditBox.Layout.Row = rowIdx;
            this.MinMarkerIDEditBox.Layout.Column = 2;
        end

        %------------------------------------------------------------------
        function configureOriginCheckerColor(this, gridLayout, rowIdx)
            
            % Origin Checker Color Label
            originCheckerColorLabel = uilabel(Parent=gridLayout, ...
                Text=vision.getMessage("vision:caltool:OriginCheckerColor"));
            originCheckerColorLabel.Layout.Row = rowIdx;
            originCheckerColorLabel.Layout.Column = 1;

            % Origin Checker Color Radiobutton group
            this.OriginCheckerColorButtons = uibuttongroup(gridLayout, ...
                SelectionChangedFcn= @(~, ~) doOriginCheckerColorChanged(this), ...
                Tooltip=vision.getMessage("vision:caltool:OriginCheckerColorToolTip"), ...
                BorderType = "none");
            this.OriginCheckerColorButtons.Layout.Row = rowIdx;
            this.OriginCheckerColorButtons.Layout.Column = 2;

            % Origin Checker Color Radiobuttons
            this.OriginCheckerColorBlackButton = uiradiobutton(...
                this.OriginCheckerColorButtons, Position=[1 0 50 20], ...
                Tag="OriginCheckerColorBlackButton", Value=true, ...
                Text=vision.getMessage("vision:caltool:Black"));
            this.OriginCheckerColorWhiteButton = uiradiobutton(...
                this.OriginCheckerColorButtons, Position=[60 0 50 20],...
                Tag="OriginCheckerColorWhiteButton", Value=false, ...
                Text=vision.getMessage("vision:caltool:White"));
        end

        %------------------------------------------------------------------
        function configurePropertiesPanelThumbnail(this, gridLayout)

            thumbnailSize = 140;
            thumbnailGridLayout = uigridlayout(gridLayout, Padding=0);
            thumbnailGridLayout.RowHeight = thumbnailSize;
            thumbnailGridLayout.ColumnWidth = thumbnailSize;
            thumbnailGridLayout.Layout.Row = 2;
            thumbnailGridLayout.Layout.Column = 2;

            if isDarkMode
                visualGuideFile = this.PropertiesGuideDark;
            else
                visualGuideFile = this.PropertiesGuideLight;
            end

            vision.internal.calibration.tool.displayPatternThumbnail(...
                thumbnailGridLayout, visualGuideFile);
        end

        %------------------------------------------------------------------
        function configureMeasurementsPanelThumbnail(this, gridLayout)

            thumbnailGridLayout = uigridlayout(gridLayout, Padding=0);
            thumbnailGridLayout.RowHeight = 90;
            thumbnailGridLayout.ColumnWidth = 119;
            thumbnailGridLayout.Layout.Row = 1;
            thumbnailGridLayout.Layout.Column = 3;

            if isDarkMode
                visualGuideFile = this.MeasurementsGuideDark;
            else
                visualGuideFile = this.MeasurementsGuideLight;
            end

            vision.internal.calibration.tool.displayPatternThumbnail(...
                thumbnailGridLayout, visualGuideFile);
        end
    end

    %----------------------------------------------------------------------
    % Callbacks for UI components to update corresponding properties.
    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        function initializePropertyValues(this)

            % Marker family
            this.MarkerFamily = this.MarkerFamilyPopup.Value;

            % Checker size
            this.CheckerSize = str2double(this.CheckerSizeEditBox.Value);

            % Marker size
            this.MarkerSize = str2double(this.MarkerSizeEditBox.Value);

            % Pattern dimensions
            this.PatternDims = str2num(this.PatternDimsEditBox.Value); %#ok
            
            % Units
            this.WorldUnits = this.WorldUnitsPopup.Value;

            % Minimum marker ID
            this.MinMarkerID = str2double(this.MinMarkerIDEditBox.Value);

            % Origin checker color
            if this.OriginCheckerColorBlackButton.Value
                this.OriginCheckerColor = "black";
            else
                this.OriginCheckerColor = "white";
            end
        end

        %------------------------------------------------------------------
        function doUnitsChanged(this)
            this.WorldUnits = this.WorldUnitsPopup.Value;
        end

        %------------------------------------------------------------------
        function doCheckerSizeChanged(this)
            
            try
                checkerSize = str2double(this.CheckerSizeEditBox.Value);
                validateMeasurement(checkerSize, "checkerSize")
                this.CheckerSize = checkerSize;
            catch ME
                this.CheckerSizeEditBox.Value = num2str(this.CheckerSize);
                iAlert(this, ME.message)
            end
        end

        %------------------------------------------------------------------
        function doMarkerSizeChanged(this)
            
            try
                markerSize = str2double(this.MarkerSizeEditBox.Value);
                validateMeasurement(markerSize, "markerSize")
                this.MarkerSize = markerSize;
            catch ME
                this.MarkerSizeEditBox.Value = num2str(this.MarkerSize);
                iAlert(this, ME.message)
            end
        end

        %------------------------------------------------------------------
        function doMarkerFamilyChanged(this)
            this.MarkerFamily = this.MarkerFamilyPopup.Value;
        end

        %------------------------------------------------------------------
        function doPatternDimsChanged(this)
            
            try
                patternDims = str2num(this.PatternDimsEditBox.Value); %#ok
                validatePatternDims(patternDims)
                this.PatternDims = patternDims;
            catch ME
                this.PatternDimsEditBox.Value = ...
                    "[ " + this.PatternDims(1) + ", " + this.PatternDims(2) + " ]";
                iAlert(this, ME.message)
            end
        end

        %------------------------------------------------------------------
        function doMinMarkerIDChanged(this)

            try
                minMarkerID = str2double(this.MinMarkerIDEditBox.Value);
                validateMinMarkerID(minMarkerID, this.MarkerFamily)
                this.MinMarkerID = minMarkerID;
            catch ME
                this.MinMarkerIDEditBox.Value = num2str(this.MinMarkerID);
                iAlert(this, ME.message)
            end
        end
        
        %------------------------------------------------------------------
        function doOriginCheckerColorChanged(this)
            if this.OriginCheckerColorBlackButton.Value
                this.OriginCheckerColor = "black";
            else
                this.OriginCheckerColor = "white";
            end
        end

        %------------------------------------------------------------------
        function doBrowseButtonPushed(this)

            updateDialogState(this, false);

            templateImageFile = imgetfile;

            autoPopulateProperties(this, templateImageFile)

            %--------------------------------------------------------------
            function autoPopulateProperties(this, templateImageFile)
                try
                    if ~isempty(templateImageFile)
                        I = imread(templateImageFile);
                    else
                        % Return to normal state when no image is selected.
                        updateDialogState(this, true);
                        return
                    end
                catch ME
                    % Open error dialog.
                    dlgTitle = vision.getMessage("vision:caltool:AutoPopulateErrorTitle");
                    iAlert(this, ME.message, dlgTitle)
                    updateDialogState(this, true);
                    return
                end
    
                try
                    [patternDims, markerFamily, minMarkerID, originCheckerColor] = ...
                        vision.internal.calibration.detectCharucoBoardProperties(I);
                    
                    launchAutoPopulatePropsDlg(this, templateImageFile, patternDims, ...
                            markerFamily, minMarkerID, originCheckerColor)
                catch ME
                    
                    launchAutoPopulatePropsDlg(this, templateImageFile, ME.message)
    
                    return
                end
            end
        end
    end
    
    %----------------------------------------------------------------------
    % Helper functions.
    %----------------------------------------------------------------------
    methods (Access = private)
        %------------------------------------------------------------------
        function launchAutoPopulatePropsDlg(this, templateImageFile, varargin)

            pos = this.ParentFigure.Position;
            center = [round(pos(1) + (pos(3)/2)), round(pos(2) + pos(4)/2)];
            vision.internal.calibration.webTool.AutoPopulateCharucoPropsDlg(...
                this, templateImageFile, center, varargin{:});
        end

        %------------------------------------------------------------------
        function updateBoardProperties(this, patternDims, markerFamily, ...
                minMarkerID, originCheckerColor)

            this.PatternDimsEditBox.Value = "[ " + patternDims(1) + ", "+ patternDims(2) + " ]";
            this.MarkerFamilyPopup.Value = markerFamily;
            this.MinMarkerIDEditBox.Value = num2str(minMarkerID);
            if originCheckerColor == "black"
                this.OriginCheckerColorBlackButton.Value = true;
            else
                this.OriginCheckerColorWhiteButton.Value = true;
            end

            doPatternDimsChanged(this);
            doMarkerFamilyChanged(this);
            doMinMarkerIDChanged(this);
            doOriginCheckerColorChanged(this);
        end

        %------------------------------------------------------------------
        function iAlert(this, errorMsg, varargin)
            
            if nargin > 2
                title = varargin{1};
            else
                title = vision.getMessage("MATLAB:uistring:popupdialogs:ErrorDialogTitle");
            end
            uialert(this.ParentFigure, errorMsg, title);
        end
    end
end

%--------------------------------------------------------------
% Identify the theme
%--------------------------------------------------------------
function tf = isDarkMode(~)
    s = settings;
    mode = s.matlab.appearance.MATLABTheme.ActiveValue;
    tf = strcmpi(mode, "Dark");
end

%--------------------------------------------------------------
% Validation functions for command line workflow
%--------------------------------------------------------------
function tf = validateShowProgressBar(showProgressBar)
    validateattributes(showProgressBar, ["logical", "numeric"],...
        "scalar", mfilename, "ShowProgressBar");
    tf = true;
end

%--------------------------------------------------------------
function tf = validateProgressBarParent(progressBarParent)
    if ~isempty(progressBarParent)
        validateattributes(progressBarParent, "matlab.ui.container.internal.AppContainer",...
            "nonempty", mfilename, "ProgressBarParent");
    end
    tf = true;
end

%--------------------------------------------------------------------
function color = validateOriginCheckerColor(color)
    validateattributes(color, {'char', 'string'}, {'nonempty', 'vector'}, '', 'OriginCheckerColor');
    
    validPatterns = {'black', 'white'};
    color = validatestring(color, ...
        validPatterns, '', "OriginCheckerColor");
end

%--------------------------------------------------------------------
function validateMarkerFamily(markerFamily)
    validateattributes(markerFamily, {'char', 'string'}, {'nonempty', 'vector'}, '', 'markerFamily');
    
    supportedFamilies = vision.internal.supportedCharucoBoardFamilies();
    validatestring(markerFamily, supportedFamilies, '', 'markerFamily');
end

%--------------------------------------------------------------------
function validatePatternDims(patternDims)
   % Get translated pattern dimensions variable using the catalog.
   varname = iTranslatedVariableName("vision:caltool:PatternDims");

   % Remove the [dim1, dim2] description from the varname.
   varname = strtrim(regexprep(varname, '\[ [^\]]* \]', ''));

   validateattributes(patternDims, "numeric",...
        {"nonempty", "vector", "numel", 2, "real", "finite", "integer",...
        "positive", ">=", 3}, "", varname);
end

%--------------------------------------------------------------------
function validateMeasurement(value, varname)

    if strcmpi(varname, "checkerSize")
        varname = iTranslatedVariableName("vision:caltool:CheckerSize");
    else
        varname = iTranslatedVariableName("vision:caltool:MarkerSize");
    end
   
   validateattributes(value, "numeric",...
        {"nonempty", "scalar", "real", "finite", "positive"}, "", varname);
end

%--------------------------------------------------------------------------
function validateMinMarkerID(minMarkerID, familyName)
    
    if strcmpi(familyName, "DICT_ARUCO_ORIGINAL")
        familySize = 1024;
    else
        familySize = 1000;
    end

    varname = iTranslatedVariableName("vision:caltool:MinMarkerID");
    validateattributes(minMarkerID, "numeric",...
        {"nonempty", "scalar", "real", "finite", "integer", "nonnegative"}, "", ...
        varname);

    vision.internal.errorIf(minMarkerID < 0 || minMarkerID >= familySize,...
        "vision:aruco:invalidMinMarkerID", familyName, familySize-1);
end

%--------------------------------------------------------------------------
function varname = iTranslatedVariableName(catalog)

    msg = vision.getMessage(catalog);
    lowerCaseMsg = lower(msg);
    msgNoTrailingSpaces = strtrim(lower(lowerCaseMsg));
    varname = regexprep(msgNoTrailingSpaces, ':', '');
end