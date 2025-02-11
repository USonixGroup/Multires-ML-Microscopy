classdef AprilGridDetectorImpl < vision.calibration.PatternDetector
    % AprilGridDetectorImpl Implements the interface for AprilGrid detection for camera calibration.
    %
    %   AprilGridDetectorImpl is the base class for AprilGrid detectors
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
        
        Name = vision.getMessage("vision:caltool:AprilGridPattern");
    end

    %----------------------------------------------------------------------
    properties
        Panel

        WorldUnits = vision.getMessage("vision:caltool:millimeters");

        TagFamily = "tag36h11";

        PatternDims

        TagSize

        TagSpacing

        MinTagID

        NumBorderBits
    end

    properties (Access = private)
        ParentFigure
    end

    %----------------------------------------------------------------------
    % UI components for the properties panel.
    %----------------------------------------------------------------------
    properties (Access = private)

        WorldUnitsPopup
        
        TagSizeEditBox

        TagSpacingEditBox

        TagFamilyPopup

        PatternDimsEditBox

        MinTagIDEditBox

        NumBorderBitsButtons

        OneBorderBitButton

        TwoBorderBitButton

        TemplateImageBrowseButton
    end

    %----------------------------------------------------------------------
    properties (Access = private, Constant)
        AvailableUnits = vision.internal.calibration.availableWorldUnits;

        AvailableTagFamilies = vision.internal.supportedAprilGridFamilies;

        PropertiesGuideDark = fullfile(toolboxdir("vision"),"vision", ...
                    "+vision", "+internal", "+calibration", ...
                    "+tool", "AprilGridPropertiesVisualGuideDark.png");
        
        PropertiesGuideLight = fullfile(toolboxdir("vision"),"vision", ...
                "+vision", "+internal", "+calibration", ...
                "+tool", "AprilGridPropertiesVisualGuideLight.png");
        
        MeasurementsGuideDark = fullfile(toolboxdir("vision"),"vision", ...
                "+vision", "+internal", "+calibration", ...
                "+tool", "AprilGridMeasurementsVisualGuideDark.png");

        MeasurementsGuideLight = fullfile(toolboxdir("vision"),"vision", ...
                "+vision", "+internal", "+calibration", ...
                "+tool", "AprilGridMeasurementsVisualGuideLight.png");
    end

    %----------------------------------------------------------------------
    methods

        %------------------------------------------------------------------
        % Detection method for pattern keypoint detection in the calibration
        % images.
        %------------------------------------------------------------------
        function [imagePoints, imagesUsed] = detectPatternPoints(this, varargin)

            if isa(this, "vision.calibration.stereo.AprilGridDetector")
                imageFileNames = varargin(1:2);
                otherInputs    = varargin(3:end);
            else
                imageFileNames = varargin(1);
                otherInputs    = varargin(2:end);
            end
            
            detectorParams = validateAndParseInputs(this, otherInputs{:});
            
            [imagePoints, imagesUsed, userCanceled] = detectAprilGridPoints(...
                imageFileNames{:}, detectorParams{:});

            vision.internal.errorIf(userCanceled, "vision:uitools:LoadingCanceledByUser");
        end

        %--------------------------------------------------------------------
        function inputs = validateAndParseInputs(this, patternDims, tagFamily, options)
            arguments
                this %#ok
                patternDims  {validatePatternDims} = this.PatternDims
                tagFamily {validateTagFamily} = this.TagFamily
                options.MinTagID {validateMinTagID(options.MinTagID, tagFamily)} = this.MinTagID
                options.NumBorderBits {validateNumBorderBits} = this.NumBorderBits
                options.ShowProgressBar    {validateShowProgressBar} = true
                options.ProgressBarParent  {validateProgressBarParent} = []
            end
            
            inputs = {patternDims, tagFamily, "MinTagID", options.MinTagID, "NumBorderBits", ...
                options.NumBorderBits, "ShowProgressBar", options.ShowProgressBar, ...
                "ProgressBarParent", options.ProgressBarParent};
        end

        %------------------------------------------------------------------
        % World point generation.
        %------------------------------------------------------------------
        function worldPoints = generateWorldPoints(this, options)
            arguments
                this %#ok
                options.PatternDims  (:,:)  {validatePatternDims} = this.PatternDims
                options.TagSize  (1,1)  {validateMeasurement(options.TagSize,"TagSize")} = this.TagSize
                options.TagSpacing  (1,1)  {validateMeasurement(options.TagSpacing,"TagSpacing")} = this.TagSpacing
            end
            
            worldPoints = patternWorldPoints("aprilgrid", options.PatternDims,...
                options.TagSize, options.TagSpacing);
        end
    end

    %----------------------------------------------------------------------
    methods

        %------------------------------------------------------------------
        % Set up properties panel related to AprilGrid detection.
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
            
            numPatternRows = min(this.PatternDims)*2;
            numPatternCols = max(this.PatternDims)*2;
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
            % and AprilGrid properties panels.
            panelLayout = uigridlayout(this.Panel, Padding=0);
            panelLayout.RowHeight = {"fit","fit"};
            panelLayout.ColumnWidth = "1x";

            % Configure measurements panel.
            configureMeasurementsPanel(this, panelLayout);

            % Configure properties panel.
            configureAprilGridPropsPanel(this, panelLayout);
        end

        %------------------------------------------------------------------
        %  Configure measurements panel using the following grid layout
        %------------------------------------------------------------------
        %  ---------------------------------
        % | Board Measurements              |
        % |---------------------------------|
        % |        |               |        |  <--- Row 1 (Tag Size)
        % |------------------------|        |
        % |        |               |        |  <--- Row 2 (Tag Spacing)
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
            panelInnerLayout.ColumnWidth = {"fit", "0.15x"};

            % Define a grid layout for the UI elements: (Labels + UI element)
            rowHeight = 25;
            gridLayout = uigridlayout(panelInnerLayout, Padding=2);
            gridLayout.Layout.Column = 1;
            gridLayout.RowHeight = {rowHeight, rowHeight, rowHeight};
            gridLayout.ColumnWidth = {"1.4x", "1x"};

            % Configure the UI components.
            configureTagSizeSelector(this, gridLayout, 1);
            configureTagSpacingSelector(this, gridLayout, 2);
            configureUnitsSelector(this, gridLayout, 3);
            
            % Configure the UI component for measurements thumbnail image.
            configureMeasurementsPanelThumbnail(this, panelInnerLayout);
        end

        %------------------------------------------------------------------
        %  Configure properties panel using the following grid layout
        %------------------------------------------------------------------
        %  ---------------------------------
        % | AprilGrid Properties            |
        % |---------------------------------|
        % |                     |           |  <--- Row 1 (Template selector)
        % |---------------------------------|
        % |        |               |        |  <--- Row 2 (Tag family)
        % |------------------------|
        % |        |               |        |  <--- Row 3 (Pattern Dims)
        % |------------------------|        |
        % |        |               |        |  <--- Row 4 (Min Tag ID)
        % |------------------------|        |
        % |        |               |        |  <--- Row 5 (Num Border Bits)
        %  ---------------------------------
        %------------------------------------------------------------------
        function configureAprilGridPropsPanel(this, panelLayout)

            % AprilGrid properties with title
            aprilGridPanel = uipanel(panelLayout,...
                Title=vision.getMessage("vision:caltool:AprilGridPropsPanelTitle"));
            aprilGridPanel.Layout.Row = 2;

            % Define a grid layout within the panel: (UI elements + image)
            rowHeight = 25;
            panelInnerLayout = uigridlayout(aprilGridPanel);
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
            configureTagFamilyUIComponents(this, gridLayout, 1);
            configurePatternDimsSelector(this, gridLayout, 2);
            configureMinimumTagID(this, gridLayout, 3);
            configureNumBorderBits(this, gridLayout, 4);
            
            % Configure the UI component for properties thumbnail image.
            configurePropertiesPanelThumbnail(this, panelInnerLayout);
        end
        
        %------------------------------------------------------------------
        function configureTagSizeSelector(this, gridLayout, rowIdx)

            % TagSize Label
            tagSizeLabel = uilabel(Parent=gridLayout,...
                Text=vision.getMessage("vision:caltool:TagSize"),...
                Tooltip=vision.getMessage("vision:caltool:TagSizeToolTip"));
            tagSizeLabel.Layout.Column = 1;
            tagSizeLabel.Layout.Row = rowIdx;

            % TagSize Editbox
            initTagSize = 20;
            this.TagSizeEditBox = uieditfield(gridLayout, ...
                Tag="TagSizeEditBox", Value=num2str(initTagSize), ...
                ValueChangedFcn=@(~, ~) doTagSizeChanged(this), ...
                Tooltip=vision.getMessage("vision:caltool:TagSizeToolTip"));
            this.TagSizeEditBox.Layout.Column = 2;
            this.TagSizeEditBox.Layout.Row = rowIdx;
        end

        %------------------------------------------------------------------
        function configureTagSpacingSelector(this, gridLayout, rowIdx)

            % Tag Spacing Label
            tagSpacingLabel = uilabel(Parent=gridLayout, ...
                Text=vision.getMessage("vision:caltool:TagSpacing"),...
                Tooltip=vision.getMessage("vision:caltool:TagSpacingToolTip"));
            tagSpacingLabel.Layout.Column = 1;
            tagSpacingLabel.Layout.Row = rowIdx;

            % Tag Spacing Editbox
            initTagSpacing = 15;
            this.TagSpacingEditBox = uieditfield(gridLayout, ...
                Tag="TagSpacingEditBox", Value=num2str(initTagSpacing), ...
                ValueChangedFcn=@(~, ~) doTagSpacingChanged(this), ...
                Tooltip=vision.getMessage("vision:caltool:TagSpacingToolTip"));
            this.TagSpacingEditBox.Layout.Column = 2;
            this.TagSpacingEditBox.Layout.Row = rowIdx;
        end

        %------------------------------------------------------------------
        function configureUnitsSelector(this, gridLayout, rowIdx)

            % Units Label
            unitsLabel = uilabel(Parent=gridLayout,...
                Text=vision.getMessage("vision:caltool:MeasurementUnits"),...
                Tooltip=vision.getMessage("vision:caltool:MeasurementUnitsToolTip"));
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
                Text=vision.getMessage("vision:caltool:AutoPopulatePromptAprilGrid"),...
                Tooltip=vision.getMessage("vision:caltool:TemplateBrowseButtonToolTip"));
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
        function configureTagFamilyUIComponents(this, gridlayout, rowIdx)

            % Tag Family Label
            label = uilabel(Parent=gridlayout, ...
                Text=vision.getMessage("vision:caltool:TagFamily"),...
                Tooltip=vision.getMessage("vision:caltool:TagFamilyToolTip"));
            label.Layout.Row = rowIdx;
            label.Layout.Column = 1;

            % Tag Family Selection
            initTagFamily = this.TagFamily;
            this.TagFamilyPopup = uidropdown(Parent=gridlayout,...
                Items=this.AvailableTagFamilies,Value=initTagFamily,...
                ValueChangedFcn=@(~, ~) doTagFamilyChanged(this),...
                Tooltip=vision.getMessage("vision:caltool:TagFamilyToolTip"),...
                Tag="TagFamilyPopup");
            this.TagFamilyPopup.Layout.Row = rowIdx;
            this.TagFamilyPopup.Layout.Column = 2;
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
                Text=vision.getMessage("vision:caltool:PatternDims"),...
                Tooltip=vision.getMessage("vision:caltool:PatternDimsToolTip"));
            patternDimsLabel.Layout.Column = 1;
            patternDimsLabel.Layout.Row = 1;
            
            % Pattern Dimensions Editbox
            initPatternDims = "[ 4, 3 ]";
            this.PatternDimsEditBox = uieditfield(selectorGridLayout,...
                Tag="PatternDimsEditbox", Value=initPatternDims, ...
                ValueChangedFcn=@(~, ~) doPatternDimsChanged(this), ...
                Tooltip=vision.getMessage("vision:caltool:PatternDimsToolTip"));
            this.PatternDimsEditBox.Layout.Column = 2;
            this.PatternDimsEditBox.Layout.Row = 1;
        end

        %------------------------------------------------------------------
        function configureMinimumTagID(this, gridLayout, rowIdx)
            
            % Minimum Tag ID Label
            minTagIDlabel = uilabel(Parent=gridLayout, ...
                Text=vision.getMessage("vision:caltool:MinTagID"),...
                Tooltip=vision.getMessage("vision:caltool:MinTagIDToolTip"));
            minTagIDlabel.Layout.Row = rowIdx;
            minTagIDlabel.Layout.Column = 1;

            % Minimum Tag ID Editbox
            initMinTagID = 0;
            this.MinTagIDEditBox = uieditfield(gridLayout, ...
                Tag="MinTagIDEditBox", Value=num2str(initMinTagID), ...
                ValueChangedFcn=@(~, ~) doMinTagIDChanged(this), ...
                Tooltip=vision.getMessage("vision:caltool:MinTagIDToolTip"));
            this.MinTagIDEditBox.Layout.Row = rowIdx;
            this.MinTagIDEditBox.Layout.Column = 2;
        end

        %------------------------------------------------------------------
        function configureNumBorderBits(this, gridLayout, rowIdx)
            
            % Num Border Bits Label
            numBorderBitsLabel = uilabel(Parent=gridLayout, ...
                Text=vision.getMessage("vision:caltool:NumBorderBits"),...
                Tooltip=vision.getMessage("vision:caltool:NumBorderBitsToolTip"));
            numBorderBitsLabel.Layout.Row = rowIdx;
            numBorderBitsLabel.Layout.Column = 1;

            % Num Border Bits Radiobutton group
            this.NumBorderBitsButtons = uibuttongroup(gridLayout, ...
                SelectionChangedFcn= @(~, ~) doNumBorderBitsChanged(this), ...
                Tooltip=vision.getMessage("vision:caltool:NumBorderBitsToolTip"), ...
                BorderType = "none");
            this.NumBorderBitsButtons.Layout.Row = rowIdx;
            this.NumBorderBitsButtons.Layout.Column = 2;

            % Num Border Bits Radiobuttons
            this.OneBorderBitButton = uiradiobutton(...
                this.NumBorderBitsButtons, Position=[1 0 50 20], ...
                Tag="OneBorderBitButton", Value=false, ...
                Text=vision.getMessage("vision:caltool:OneBorderBit"));
            this.TwoBorderBitButton = uiradiobutton(...
                this.NumBorderBitsButtons, Position=[60 0 50 20],...
                Tag="TwoBorderBitButton", Value=true, ...
                Text=vision.getMessage("vision:caltool:TwoBorderBit"));
        end

        %------------------------------------------------------------------
        function configurePropertiesPanelThumbnail(this, gridLayout)

            thumbnailGridLayout = uigridlayout(gridLayout, Padding=0);
            thumbnailGridLayout.RowHeight = 140;
            thumbnailGridLayout.ColumnWidth = 140;
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
            thumbnailGridLayout.RowHeight = 113;
            thumbnailGridLayout.ColumnWidth = 147;
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
    methods (Access=private)
        %------------------------------------------------------------------
        function initializePropertyValues(this)

            % Tag family
            this.TagFamily = this.TagFamilyPopup.Value;

            % Tag size
            this.TagSize = str2double(this.TagSizeEditBox.Value);

            % Tag spacing
            this.TagSpacing = str2double(this.TagSpacingEditBox.Value);

            % Pattern dimensions
            this.PatternDims = str2num(this.PatternDimsEditBox.Value); %#ok
            
            % Units
            this.WorldUnits = this.WorldUnitsPopup.Value;

            % Minimum tag ID
            this.MinTagID = str2double(this.MinTagIDEditBox.Value);

            % Origin checker color
            if this.OneBorderBitButton.Value
                this.NumBorderBits = 1;
            else
                this.NumBorderBits = 2;
            end
        end

        %------------------------------------------------------------------
        function doUnitsChanged(this)
            this.WorldUnits = this.WorldUnitsPopup.Value;
        end

        %------------------------------------------------------------------
        function doTagSizeChanged(this)
            
            try
                tagSize = str2double(this.TagSizeEditBox.Value);
                validateMeasurement(tagSize, "tagSize")
                this.TagSize = tagSize;
            catch ME
                this.TagSizeEditBox.Value = num2str(this.TagSize);
                iAlert(this, ME.message)
            end
        end

        %------------------------------------------------------------------
        function doTagSpacingChanged(this)
            
            try
                tagSpacing = str2double(this.TagSpacingEditBox.Value);
                validateMeasurement(tagSpacing, "tagSpacing")
                this.TagSpacing = tagSpacing;
            catch ME
                this.TagSpacingEditBox.Value = num2str(this.TagSpacing);
                iAlert(this, ME.message)
            end
        end

        %------------------------------------------------------------------
        function doTagFamilyChanged(this)
            this.TagFamily = this.TagFamilyPopup.Value;
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
        function doMinTagIDChanged(this)

            try
                minTagID = str2double(this.MinTagIDEditBox.Value);
                validateMinTagID(minTagID, this.TagFamily)
                this.MinTagID = minTagID;
            catch ME
                this.MinTagIDEditBox.Value = num2str(this.MinTagID);
                iAlert(this, ME.message)
            end
        end
        
        %------------------------------------------------------------------
        function doNumBorderBitsChanged(this)
            if this.OneBorderBitButton.Value
                this.NumBorderBits = 1;
            else
                this.NumBorderBits = 2;
            end
        end

        %------------------------------------------------------------------
        function doBrowseButtonPushed(this)

            updateDialogState(this, false);

            templateImageFile = imgetfile;

            autoPopulateProperties(this, templateImageFile)
        end

        %------------------------------------------------------------------
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
                [patternDims, tagFamily, minTagID, numBorderBits] = ...
                    vision.internal.calibration.detectAprilGridProperties(I);
                
                launchAutoPopulatePropsDlg(this, templateImageFile, patternDims, ...
                        tagFamily, minTagID, numBorderBits)
            catch ME
                
                launchAutoPopulatePropsDlg(this, templateImageFile, ME.message)

                return
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
            vision.internal.calibration.webTool.AutoPopulateAprilGridPropsDlg(...
                this, templateImageFile, center, varargin{:});
        end

        %------------------------------------------------------------------
        function updateBoardProperties(this, patternDims, tagFamily, ...
                minTagID, numBorderBits)

            this.PatternDimsEditBox.Value = "[ " + patternDims(1) + ", "+ patternDims(2) + " ]";
            this.TagFamilyPopup.Value = tagFamily;
            this.MinTagIDEditBox.Value = num2str(minTagID);
            if numBorderBits == 1
                this.OneBorderBitButton.Value = true;
            else
                this.TwoBorderBitButton.Value = true;
            end

            % Update class properties.
            this.PatternDims = patternDims;
            this.MinTagID = minTagID;
            doTagFamilyChanged(this);
            doNumBorderBitsChanged(this);
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
function validateNumBorderBits(bits)
    varname = iTranslatedVariableName("vision:caltool:NumBorderBits");
    validateattributes(bits, {'numeric'}, {'nonempty', 'vector'}, '', varname);
end

%--------------------------------------------------------------------
function validateTagFamily(tagFamily)
    varname = iTranslatedVariableName("vision:caltool:TagFamily");
    validateattributes(tagFamily, {'char', 'string'}, {'nonempty', 'vector'}, '', varname);
    
    supportedFamilies = vision.internal.supportedAprilGridFamilies();
    validatestring(tagFamily, supportedFamilies, '', varname);
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

    if strcmpi(varname, "tagSize")
        varname = iTranslatedVariableName("vision:caltool:TagSize");
    else
        varname = iTranslatedVariableName("vision:caltool:TagSpacing");
    end
   
   validateattributes(value, "numeric",...
        {"nonempty", "scalar", "real", "finite", "positive"}, "", varname);
end

%--------------------------------------------------------------------------
function validateMinTagID(minTagID, familyName)
    
    if strcmpi(familyName, "DICT_ARUCO_ORIGINAL")
        familySize = 1024;
    else
        familySize = 1000;
    end

    varname = iTranslatedVariableName("vision:caltool:MinTagID");
    validateattributes(minTagID, "numeric",...
        {"nonempty", "scalar", "real", "finite", "integer", "nonnegative"}, "", ...
        varname);

    vision.internal.errorIf(minTagID < 0 || minTagID >= familySize,...
        "vision:aruco:invalidMinMarkerID", familyName, familySize-1);
end

%--------------------------------------------------------------------------
function varname = iTranslatedVariableName(catalog)

    msg = vision.getMessage(catalog);
    lowerCaseMsg = lower(msg);
    msgNoTrailingSpaces = strtrim(lower(lowerCaseMsg));
    varname = regexprep(msgNoTrailingSpaces, ':', '');
end